// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#include "isp-iommu.h"
#include "isp-ipc.h"
#include "isp-regs.h"

#define ISP_IPC_FLAG_TERMINAL_ACK	0x3
#define ISP_IPC_BUFEXC_STAT_META_OFFSET 0x10

struct isp_sm_deferred_work {
	struct work_struct work;
	struct apple_isp *isp;
	struct isp_surf *surf;
};

struct isp_bufexc_stat {
	u64 unk_0; // 2
	u64 unk_8; // 2

	u64 meta_iova;
	u64 pad_20[3];
	u64 meta_size; // 0x4640
	u64 unk_38;

	u32 unk_40; // 1
	u32 unk_44;
	u64 unk_48;

	u64 iova0;
	u64 iova1;
	u64 iova2;
	u64 iova3;
	u32 pad_70[4];

	u32 unk_80; // 2
	u32 unk_84; // 1
	u32 unk_88; // 0x10 || 0x13
	u32 unk_8c;
	u32 pad_90[96];

	u32 unk_210; // 0x28
	u32 unk_214;
	u32 index;
	u16 bes_width; // 1296, 0x510
	u16 bes_height; // 736, 0x2e0

	u32 unk_220; // 0x0 || 0x1
	u32 pad_224[3];
	u32 unk_230; // 0xf7ed38
	u32 unk_234; // 3
	u32 pad_238[2];
	u32 pad_240[16];
} __packed;
static_assert(sizeof(struct isp_bufexc_stat) == ISP_IPC_BUFEXC_STAT_SIZE);

static inline dma_addr_t chan_msg_iova(struct isp_channel *chan, u32 index)
{
	return chan->iova + (index * ISP_IPC_MESSAGE_SIZE);
}

static inline void chan_read_msg_index(struct apple_isp *isp,
				       struct isp_channel *chan,
				       struct isp_message *msg, u32 index)
{
	isp_ioread(isp, chan_msg_iova(chan, index), msg, sizeof(*msg));
}

static inline void chan_read_msg(struct apple_isp *isp,
				 struct isp_channel *chan,
				 struct isp_message *msg)
{
	chan_read_msg_index(isp, chan, msg, chan->cursor);
}

static inline void chan_write_msg_index(struct apple_isp *isp,
					struct isp_channel *chan,
					struct isp_message *msg, u32 index)
{
	isp_iowrite(isp, chan_msg_iova(chan, index), msg, sizeof(*msg));
}

static inline void chan_write_msg(struct apple_isp *isp,
				  struct isp_channel *chan,
				  struct isp_message *msg)
{
	chan_write_msg_index(isp, chan, msg, chan->cursor);
}

static inline void chan_update_cursor(struct isp_channel *chan)
{
	if (chan->cursor >= (chan->num - 1)) {
		chan->cursor = 0;
	} else {
		chan->cursor += 1;
	}
}

static int chan_handle_once(struct apple_isp *isp, struct isp_channel *chan)
{
	int err;

	lockdep_assert_held(&chan->lock);

	err = chan->ops->handle(isp, chan);
	if (err < 0) {
		dev_err(isp->dev, "%s: handler failed: %d)\n", chan->name, err);
		return err;
	}

	chan_write_msg(isp, chan, &chan->rsp);

	isp_mbox_write32(isp, ISP_MBOX_IRQ_DOORBELL, chan->doorbell);

	chan_update_cursor(chan);

	return 0;
}

static inline bool chan_rx_done(struct apple_isp *isp, struct isp_channel *chan)
{
	if (((chan->req.arg0 & 0xf) == ISP_IPC_FLAG_ACK) ||
	    ((chan->req.arg0 & 0xf) == ISP_IPC_FLAG_TERMINAL_ACK)) {
		return true;
	}
	return false;
}

int ipc_chan_handle(struct apple_isp *isp, struct isp_channel *chan)
{
	int err = 0;

	spin_lock(&chan->lock);
	while (1) {
		chan_read_msg(isp, chan, &chan->req);
		if (chan_rx_done(isp, chan)) {
			err = 0;
			break;
		}
		err = chan_handle_once(isp, chan);
		if (err < 0) {
			break;
		}
	}
	spin_unlock(&chan->lock);

	return err;
}

static inline bool chan_tx_done(struct apple_isp *isp, struct isp_channel *chan)
{
	chan_read_msg(isp, chan, &chan->rsp);
	if ((chan->rsp.arg0) == (chan->req.arg0 | ISP_IPC_FLAG_ACK)) {
		chan_update_cursor(chan);
		return true;
	}
	return false;
}

int ipc_chan_send(struct apple_isp *isp, struct isp_channel *chan,
		  unsigned long timeout)
{
	long t;

	chan_write_msg(isp, chan, &chan->req);
	wmb();

	isp_mbox_write32(isp, ISP_MBOX_IRQ_DOORBELL, chan->doorbell);

	t = wait_event_interruptible_timeout(isp->wait, chan_tx_done(isp, chan),
					     timeout);
	if (t == 0) {
		dev_err(isp->dev,
			"%s: timed out on request [0x%llx, 0x%llx, 0x%llx]\n",
			chan->name, chan->req.arg0, chan->req.arg1,
			chan->req.arg2);
		return -ETIME;
	}

	isp_dbg(isp, "%s: request success (%ld)\n", chan->name, t);

	return 0;
}

int ipc_tm_handle(struct apple_isp *isp, struct isp_channel *chan)
{
	struct isp_message *rsp = &chan->rsp;

#ifdef APPLE_ISP_DEBUG
	struct isp_message *req = &chan->req;
	char buf[512];
	dma_addr_t iova = req->arg0 & ~ISP_IPC_FLAG_TERMINAL_ACK;
	u32 size = req->arg1;
	if (iova && size && test_bit(ISP_STATE_LOGGING, &isp->state)) {
		size = min_t(u32, size, 512);
		isp_ioread(isp, iova, buf, size);
		isp_dbg(isp, "ISPASC: %.*s", size, buf);
	}
#endif

	rsp->arg0 = ISP_IPC_FLAG_ACK;
	rsp->arg1 = 0x0;
	rsp->arg2 = 0x0;

	return 0;
}

/* The kernel accesses exactly two dynamically allocated shared surfaces:
 * 1) LOG: Surface for terminal logs. Optional, only enabled in debug builds.
 * 2) STAT: Surface for BUFT2H rendered frame stat buffer. We isp_ioread() in
 * the BUFT2H ISR below. Since the BUFT2H IRQ is triggered by the BUF_H2T
 * doorbell, the STAT vmap must complete before the first buffer submission
 * under VIDIOC_STREAMON(). The CISP_CMD_PRINT_ENABLE completion depends on the
 * STAT buffer SHAREDMALLOC ISR, which is part of the firmware initialization
 * sequence. We also call flush_workqueue(), so a fault should not occur.
 */
static void sm_malloc_deferred_worker(struct work_struct *work)
{
	struct isp_sm_deferred_work *dwork =
		container_of(work, struct isp_sm_deferred_work, work);
	struct apple_isp *isp = dwork->isp;
	struct isp_surf *surf = dwork->surf;
	int err;

	err = isp_surf_vmap(isp, surf); /* Can't vmap in interrupt ctx */
	if (err < 0) {
		isp_err(isp, "failed to vmap iova=0x%llx size=0x%llx\n",
			surf->iova, surf->size);
		goto out;
	}

#ifdef APPLE_ISP_DEBUG
	/* Only enabled in debug builds so it shouldn't matter, but 
	 * the LOG surface is always the first surface requested. 
	 */
	if (!test_bit(ISP_STATE_LOGGING, &isp->state))
		set_bit(ISP_STATE_LOGGING, &isp->state);
#endif

out:
	kfree(dwork);
}

int ipc_sm_handle(struct apple_isp *isp, struct isp_channel *chan)
{
	struct isp_message *req = &chan->req, *rsp = &chan->rsp;

	if (req->arg0 == 0x0) {
		struct isp_sm_deferred_work *dwork;
		struct isp_surf *surf;

		dwork = kzalloc(sizeof(*dwork), GFP_KERNEL);
		if (!dwork)
			return -ENOMEM;
		dwork->isp = isp;

		surf = isp_alloc_surface_gc(isp, req->arg1);
		if (!surf) {
			isp_err(isp, "failed to alloc requested size 0x%llx\n",
				req->arg1);
			kfree(dwork);
			return -ENOMEM;
		}
		dwork->surf = surf;

		rsp->arg0 = surf->iova | ISP_IPC_FLAG_ACK;
		rsp->arg1 = 0x0;
		rsp->arg2 = 0x0; /* macOS uses this to index surfaces */

		INIT_WORK(&dwork->work, sm_malloc_deferred_worker);
		if (!queue_work(isp->wq, &dwork->work)) {
			isp_err(isp, "failed to queue deferred work\n");
			isp_free_surface(isp, surf);
			kfree(dwork);
			return -ENOMEM;
		}
		/* To the gc it goes... */

	} else {
		/* This should be the shared surface free request, but
		 * 1) The fw doesn't request to free all of what it requested
		 * 2) The fw continues to access the surface after
		 * So we link it to the gc, which runs after fw shutdown
		 */
#ifdef APPLE_ISP_DEBUG
		if (test_bit(ISP_STATE_LOGGING, &isp->state))
			clear_bit(ISP_STATE_LOGGING, &isp->state);
#endif
		rsp->arg0 = req->arg0 | ISP_IPC_FLAG_ACK;
		rsp->arg1 = 0x0;
		rsp->arg2 = 0x0;
	}

	return 0;
}

int ipc_bt_handle(struct apple_isp *isp, struct isp_channel *chan)
{
	struct isp_message *req = &chan->req, *rsp = &chan->rsp;
	struct isp_buffer *tmp, *buf;
	int err = 0;

	/* No need to read the whole struct */
	u64 meta_iova;
	isp_ioread(isp, req->arg0 + ISP_IPC_BUFEXC_STAT_META_OFFSET, &meta_iova,
		   sizeof(meta_iova));

	spin_lock(&isp->buf_lock);
	list_for_each_entry_safe_reverse(buf, tmp, &isp->buffers, link) {
		if (buf->meta->iova == meta_iova) {
			enum vb2_buffer_state state = VB2_BUF_STATE_ERROR;
			buf->vb.vb2_buf.timestamp = ktime_get_ns();
			buf->vb.sequence = isp->sequence++;
			buf->vb.field = V4L2_FIELD_NONE;
			if (req->arg2 == ISP_IPC_BUFEXC_FLAG_RENDER)
				state = VB2_BUF_STATE_DONE;
			vb2_buffer_done(&buf->vb.vb2_buf, state);
			list_del(&buf->link);
			break;
		}
	}
	spin_unlock(&isp->buf_lock);

	rsp->arg0 = req->arg0 | ISP_IPC_FLAG_ACK;
	rsp->arg1 = 0x0;
	rsp->arg2 = ISP_IPC_BUFEXC_FLAG_ACK;

	return err;
}
