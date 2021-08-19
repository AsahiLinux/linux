// SPDX-License-Identifier: GPL-2.0
/*
 * Apple ANS NVM Express device driver
 * Copyright (C) 2021 The Asahi Linux Contributors
 *
 * This file is essentially a simplified version of the
 * NVM Express device driver (pci.c) with much less features
 * (hmb, shadow doorbell, poll queues etc.) but as a platform
 * device with Apple-specific quirks.
 *
 * The NVM Express device driver (pci.c) is
 * Copyright (c) 2011-2014, Intel Corporation
 */

#include <linux/apple-rtkit.h>
#include <linux/apple-sart.h>
#include <linux/async.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/blk-integrity.h>
#include <linux/dmapool.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/once.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/iopoll.h>

#include "trace.h"
#include "nvme.h"

#define SQ_SIZE(q)	((q)->q_depth << (q)->sqes)
#define CQ_SIZE(q)	((q)->q_depth * sizeof(struct nvme_completion))

/*
 * These can be higher, but we need to ensure that any command doesn't
 * require an sg allocation that needs more than a page of data.
 */
#define NVME_MAX_KB_SZ	4096
#define NVME_MAX_SEGS	127

#define APPLE_ANS_BOOT_TIMEOUT msecs_to_jiffies(1000)

/* Apple ANS2 registers */
#define APPLE_ANS2_QUEUE_DEPTH 64
#define APPLE_ANS2_MAX_PEND_CMDS 64
#define APPLE_NVMMU_NUM_TCBS 64

#define APPLE_ANS2_LINEAR_ASQ_DB 0x2490c
#define APPLE_ANS2_LINEAR_IOSQ_DB 0x24910

#define APPLE_NVMMU_NUM 0x28100
#define APPLE_NVMMU_BASE_ASQ 0x28108
#define APPLE_NVMMU_BASE_IOSQ 0x28110
#define APPLE_NVMMU_TCB_INVAL 0x28118
#define APPLE_NVMMU_TCB_STAT 0x28120
#define APPLE_NVMMU_TCB_SIZE                                                   \
	(sizeof(struct apple_nvmmu_tcb) * APPLE_NVMMU_NUM_TCBS)

#define APPLE_ANS2_MAX_PEND_CMDS_CTRL 0x1210

#define APPLE_ANS2_BOOT_STATUS 0x1300
#define APPLE_ANS2_BOOT_STATUS_OK 0xde71ce55

#define APPLE_ANS2_UNKNOWN_CTRL 0x24008
#define APPLE_ANS2_PRP_NULL_CHECK BIT(11)

#define APPLE_ANS2_LINEAR_SQ_CTRL 0x24908
#define APPLE_ANS2_LINEAR_SQ_EN BIT(0)

#define APPLE_ANS2_TCB_DMA_FROM_DEVICE BIT(0)
#define APPLE_ANS2_TCB_DMA_TO_DEVICE BIT(1)

struct apple_nvme_dev;
struct apple_nvme_queue;

static void apple_nvme_dev_disable(struct apple_nvme_dev *dev, bool shutdown);
static bool __apple_nvme_disable_io_queues(struct apple_nvme_dev *dev, u8 opcode);

/*
 * Represents an NVM Express device.  Each nvme_dev is a PCI function.
 */
struct apple_nvme_dev {
	struct apple_nvme_queue *adminq;
	struct apple_nvme_queue *ioq;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	bool adminq_online;
	bool ioq_online;
	u32 db_stride;
	void __iomem *nvme_mmio;
	int platform_irq;
	struct work_struct remove_work;
	struct mutex shutdown_lock;
	struct nvme_ctrl ctrl;

	mempool_t *iod_mempool;

	/* Apple ANS2 support */
	struct apple_rtkit *rtk;
	struct apple_sart *sart;
};

static inline struct apple_nvme_dev *to_apple_nvme_dev(struct nvme_ctrl *ctrl)
{
	return container_of(ctrl, struct apple_nvme_dev, ctrl);
}

/*
 * An NVM Express queue.  Each device has at least two (one for admin
 * commands and one for I/O commands).
 */
struct apple_nvme_queue {
	struct apple_nvme_dev *dev;
	spinlock_t sq_lock;
	void *sq_cmds;
	struct nvme_completion *cqes;
	dma_addr_t sq_dma_addr;
	dma_addr_t cq_dma_addr;
	u32 __iomem *q_db;
	u32 q_depth;
	u16 cq_head;
	u8 cq_phase;
	u8 sqes;
	unsigned long flags;
#define NVMEQ_ENABLED		0
#define NVMEQ_SQ_CMB		1
#define NVMEQ_DELETE_ERROR	2
	struct completion delete_done;

	u32 __iomem *ans2_q_db;
	void __iomem *nvmmu_base;
	void *ans2_tcb_ptr;
	dma_addr_t ans2_tcb_dma_addr;

	bool is_adminq;
};

/*
 * The nvme_iod describes the data in an I/O.
 *
 * The sg pointer contains the list of PRP/SGL chunk allocations in addition
 * to the actual struct scatterlist.
 */
struct apple_nvme_iod {
	struct nvme_request req;
	struct nvme_command cmd;
	struct apple_nvme_queue *nvmeq;
	int aborted;
	int npages;		/* In the PRP list. 0 means small pool in use */
	int nents;		/* Used in scatterlist */
	dma_addr_t first_dma;
	unsigned int dma_len;	/* length of single DMA segment mapping */
	dma_addr_t meta_dma;
	struct scatterlist *sg;
};

/* Apple ANS2 support */
struct apple_nvmmu_tcb {
	u8 opcode;
	u8 dma_flags;
	u8 command_id;
	u8 _unk0;
	u32 length;
	u64 _unk1[2];
	u64 prp1;
	u64 prp2;
	u64 _unk2[2];
	u8 aes_iv[8];
	u8 _aes_unk[64];
};

static void apple_nvmmu_inval(struct apple_nvme_queue *nvmeq, unsigned tag)
{
	struct apple_nvme_dev *dev = nvmeq->dev;
	struct apple_nvmmu_tcb *tcb;

	tcb = nvmeq->ans2_tcb_ptr + tag * sizeof(struct apple_nvmmu_tcb);
	memset(tcb, 0, sizeof(*tcb));

	writel(tag, dev->nvme_mmio + APPLE_NVMMU_TCB_INVAL);
	if (readl(dev->nvme_mmio + APPLE_NVMMU_TCB_STAT))
		dev_warn(dev->dev, "NVMMU TCB invalidation failed\n");
}

/*
 * Will slightly overestimate the number of pages needed.  This is OK
 * as it only leads to a small amount of wasted memory for the lifetime of
 * the I/O.
 */
static int apple_nvme_npages_prp(void)
{
	unsigned nprps = DIV_ROUND_UP(NVME_MAX_KB_SZ + NVME_CTRL_PAGE_SIZE,
				      NVME_CTRL_PAGE_SIZE);
	return DIV_ROUND_UP(8 * nprps, PAGE_SIZE - 8);
}

static size_t apple_nvme_iod_alloc_size(void)
{
	size_t npages = apple_nvme_npages_prp();

	return sizeof(__le64 *) * npages +
		sizeof(struct scatterlist) * NVME_MAX_SEGS;
}

static int apple_nvme_admin_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
				unsigned int hctx_idx)
{
	struct apple_nvme_dev *dev = data;

	WARN_ON(hctx_idx != 0);
	WARN_ON(dev->admin_tagset.tags[0] != hctx->tags);

	hctx->driver_data = dev->adminq;
	return 0;
}

static int apple_nvme_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
			  unsigned int hctx_idx)
{
	struct apple_nvme_dev *dev = data;

	WARN_ON(hctx_idx != 0);
	WARN_ON(dev->tagset.tags[0] != hctx->tags);

	hctx->driver_data = dev->ioq;
	return 0;
}

static int apple_nvme_init_request(struct blk_mq_tag_set *set, struct request *req,
			     unsigned int hctx_idx, unsigned int numa_node)
{
	struct apple_nvme_dev *dev = set->driver_data;
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct apple_nvme_queue *nvmeq = NULL;

	if (set == &dev->tagset)
		nvmeq = dev->ioq;
	else if (set == &dev->admin_tagset)
		nvmeq = dev->adminq;

	BUG_ON(!nvmeq);
	iod->nvmeq = nvmeq;

	nvme_req(req)->ctrl = &dev->ctrl;
	nvme_req(req)->cmd = &iod->cmd;
	return 0;
}

/**
 * apple_nvme_submit_cmd() - Copy a command into a queue and ring the doorbell
 * @nvmeq: The queue to use
 * @cmd: The command to send
 */
static void apple_nvme_submit_cmd(struct apple_nvme_queue *nvmeq, struct nvme_command *cmd)
{
	u32 tag = nvme_tag_from_cid(cmd->common.command_id);
	struct apple_nvmmu_tcb *tcb;

	tcb = nvmeq->ans2_tcb_ptr + tag * sizeof(struct apple_nvmmu_tcb);
	memset(tcb, 0, sizeof(*tcb));

	tcb->opcode = cmd->common.opcode;
	tcb->prp1 = cmd->common.dptr.prp1;
	tcb->prp2 = cmd->common.dptr.prp2;
	tcb->length = cmd->rw.length;
	tcb->command_id = tag;

	if (nvme_is_write(cmd))
		tcb->dma_flags = APPLE_ANS2_TCB_DMA_TO_DEVICE;
	else
		tcb->dma_flags = APPLE_ANS2_TCB_DMA_FROM_DEVICE;

	memcpy(nvmeq->sq_cmds + (tag << nvmeq->sqes), cmd, sizeof(*cmd));
	writel(tag, nvmeq->ans2_q_db);
}

static void **apple_nvme_iod_list(struct request *req)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	return (void **)(iod->sg + blk_rq_nr_phys_segments(req));
}

static void apple_nvme_free_prps(struct apple_nvme_dev *dev, struct request *req)
{
	const int last_prp = NVME_CTRL_PAGE_SIZE / sizeof(__le64) - 1;
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	dma_addr_t dma_addr = iod->first_dma;
	int i;

	for (i = 0; i < iod->npages; i++) {
		__le64 *prp_list = apple_nvme_iod_list(req)[i];
		dma_addr_t next_dma_addr = le64_to_cpu(prp_list[last_prp]);

		dma_pool_free(dev->prp_page_pool, prp_list, dma_addr);
		dma_addr = next_dma_addr;
	}
}

static void apple_nvme_unmap_sg(struct apple_nvme_dev *dev, struct request *req)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);

	dma_unmap_sg(dev->dev, iod->sg, iod->nents, rq_dma_dir(req));
}

static void apple_nvme_unmap_data(struct apple_nvme_dev *dev, struct request *req)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);

	if (iod->dma_len) {
		dma_unmap_page(dev->dev, iod->first_dma, iod->dma_len,
			       rq_dma_dir(req));
		return;
	}

	WARN_ON_ONCE(!iod->nents);

	apple_nvme_unmap_sg(dev, req);
	if (iod->npages == 0)
		dma_pool_free(dev->prp_small_pool, apple_nvme_iod_list(req)[0],
			      iod->first_dma);
	else
		apple_nvme_free_prps(dev, req);
	mempool_free(iod->sg, dev->iod_mempool);
}

static void apple_nvme_print_sgl(struct scatterlist *sgl, int nents)
{
	int i;
	struct scatterlist *sg;

	for_each_sg(sgl, sg, nents, i) {
		dma_addr_t phys = sg_phys(sg);
		pr_warn("sg[%d] phys_addr:%pad offset:%d length:%d "
			"dma_address:%pad dma_length:%d\n",
			i, &phys, sg->offset, sg->length, &sg_dma_address(sg),
			sg_dma_len(sg));
	}
}

static blk_status_t apple_nvme_setup_prps(struct apple_nvme_dev *dev,
		struct request *req, struct nvme_rw_command *cmnd)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct dma_pool *pool;
	int length = blk_rq_payload_bytes(req);
	struct scatterlist *sg = iod->sg;
	int dma_len = sg_dma_len(sg);
	u64 dma_addr = sg_dma_address(sg);
	int offset = dma_addr & (NVME_CTRL_PAGE_SIZE - 1);
	__le64 *prp_list;
	void **list = apple_nvme_iod_list(req);
	dma_addr_t prp_dma;
	int nprps, i;

	length -= (NVME_CTRL_PAGE_SIZE - offset);
	if (length <= 0) {
		iod->first_dma = 0;
		goto done;
	}

	dma_len -= (NVME_CTRL_PAGE_SIZE - offset);
	if (dma_len) {
		dma_addr += (NVME_CTRL_PAGE_SIZE - offset);
	} else {
		sg = sg_next(sg);
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);
	}

	if (length <= NVME_CTRL_PAGE_SIZE) {
		iod->first_dma = dma_addr;
		goto done;
	}

	nprps = DIV_ROUND_UP(length, NVME_CTRL_PAGE_SIZE);
	if (nprps <= (256 / 8)) {
		pool = dev->prp_small_pool;
		iod->npages = 0;
	} else {
		pool = dev->prp_page_pool;
		iod->npages = 1;
	}

	prp_list = dma_pool_alloc(pool, GFP_ATOMIC, &prp_dma);
	if (!prp_list) {
		iod->first_dma = dma_addr;
		iod->npages = -1;
		return BLK_STS_RESOURCE;
	}
	list[0] = prp_list;
	iod->first_dma = prp_dma;
	i = 0;
	for (;;) {
		if (i == NVME_CTRL_PAGE_SIZE >> 3) {
			__le64 *old_prp_list = prp_list;
			prp_list = dma_pool_alloc(pool, GFP_ATOMIC, &prp_dma);
			if (!prp_list)
				goto free_prps;
			list[iod->npages++] = prp_list;
			prp_list[0] = old_prp_list[i - 1];
			old_prp_list[i - 1] = cpu_to_le64(prp_dma);
			i = 1;
		}
		prp_list[i++] = cpu_to_le64(dma_addr);
		dma_len -= NVME_CTRL_PAGE_SIZE;
		dma_addr += NVME_CTRL_PAGE_SIZE;
		length -= NVME_CTRL_PAGE_SIZE;
		if (length <= 0)
			break;
		if (dma_len > 0)
			continue;
		if (unlikely(dma_len < 0))
			goto bad_sgl;
		sg = sg_next(sg);
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);
	}
done:
	cmnd->dptr.prp1 = cpu_to_le64(sg_dma_address(iod->sg));
	cmnd->dptr.prp2 = cpu_to_le64(iod->first_dma);
	return BLK_STS_OK;
free_prps:
	apple_nvme_free_prps(dev, req);
	return BLK_STS_RESOURCE;
bad_sgl:
	WARN(DO_ONCE(apple_nvme_print_sgl, iod->sg, iod->nents),
			"Invalid SGL for payload:%d nents:%d\n",
			blk_rq_payload_bytes(req), iod->nents);
	return BLK_STS_IOERR;
}

static blk_status_t apple_nvme_setup_prp_simple(struct apple_nvme_dev *dev,
		struct request *req, struct nvme_rw_command *cmnd,
		struct bio_vec *bv)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	unsigned int offset = bv->bv_offset & (NVME_CTRL_PAGE_SIZE - 1);
	unsigned int first_prp_len = NVME_CTRL_PAGE_SIZE - offset;

	iod->first_dma = dma_map_bvec(dev->dev, bv, rq_dma_dir(req), 0);
	if (dma_mapping_error(dev->dev, iod->first_dma))
		return BLK_STS_RESOURCE;
	iod->dma_len = bv->bv_len;

	cmnd->dptr.prp1 = cpu_to_le64(iod->first_dma);
	if (bv->bv_len > first_prp_len)
		cmnd->dptr.prp2 = cpu_to_le64(iod->first_dma + first_prp_len);
	return BLK_STS_OK;
}

static blk_status_t apple_nvme_map_data(struct apple_nvme_dev *dev, struct request *req,
		struct nvme_command *cmnd)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	blk_status_t ret = BLK_STS_RESOURCE;
	int nr_mapped;

	if (blk_rq_nr_phys_segments(req) == 1) {
		struct bio_vec bv = req_bvec(req);

		if (bv.bv_offset + bv.bv_len <= NVME_CTRL_PAGE_SIZE * 2)
			return apple_nvme_setup_prp_simple(dev, req,
						     &cmnd->rw, &bv);
	}

	iod->dma_len = 0;
	iod->sg = mempool_alloc(dev->iod_mempool, GFP_ATOMIC);
	if (!iod->sg)
		return BLK_STS_RESOURCE;
	sg_init_table(iod->sg, blk_rq_nr_phys_segments(req));
	iod->nents = blk_rq_map_sg(req->q, req, iod->sg);
	if (!iod->nents)
		goto out_free_sg;

	nr_mapped = dma_map_sg_attrs(dev->dev, iod->sg, iod->nents,
				     rq_dma_dir(req), DMA_ATTR_NO_WARN);
	if (!nr_mapped)
		goto out_free_sg;

	ret = apple_nvme_setup_prps(dev, req, &cmnd->rw);
	if (ret != BLK_STS_OK)
		goto out_unmap_sg;
	return BLK_STS_OK;

out_unmap_sg:
	apple_nvme_unmap_sg(dev, req);
out_free_sg:
	mempool_free(iod->sg, dev->iod_mempool);
	return ret;
}

static blk_status_t apple_nvme_map_metadata(struct apple_nvme_dev *dev, struct request *req,
		struct nvme_command *cmnd)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);

	iod->meta_dma = dma_map_bvec(dev->dev, rq_integrity_vec(req),
			rq_dma_dir(req), 0);
	if (dma_mapping_error(dev->dev, iod->meta_dma))
		return BLK_STS_IOERR;
	cmnd->rw.metadata = cpu_to_le64(iod->meta_dma);
	return BLK_STS_OK;
}

/*
 * NOTE: ns is NULL when called on the admin queue.
 */
static blk_status_t apple_nvme_queue_rq(struct blk_mq_hw_ctx *hctx,
			 const struct blk_mq_queue_data *bd)
{
	struct nvme_ns *ns = hctx->queue->queuedata;
	struct apple_nvme_queue *nvmeq = hctx->driver_data;
	struct apple_nvme_dev *dev = nvmeq->dev;
	struct request *req = bd->rq;
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_command *cmnd = &iod->cmd;
	blk_status_t ret;

	iod->aborted = 0;
	iod->npages = -1;
	iod->nents = 0;

	/*
	 * We should not need to do this, but we're still using this to
	 * ensure we can drain requests on a dying queue.
	 */
	if (unlikely(!test_bit(NVMEQ_ENABLED, &nvmeq->flags)))
		return BLK_STS_IOERR;

	if (!nvme_check_ready(&dev->ctrl, req, true))
		return nvme_fail_nonready_command(&dev->ctrl, req);

	ret = nvme_setup_cmd(ns, req);
	if (ret)
		return ret;

	if (blk_rq_nr_phys_segments(req)) {
		ret = apple_nvme_map_data(dev, req, cmnd);
		if (ret)
			goto out_free_cmd;
	}

	if (blk_integrity_rq(req)) {
		ret = apple_nvme_map_metadata(dev, req, cmnd);
		if (ret)
			goto out_unmap_data;
	}

	blk_mq_start_request(req);
	apple_nvme_submit_cmd(nvmeq, cmnd);
	return BLK_STS_OK;
out_unmap_data:
	apple_nvme_unmap_data(dev, req);
out_free_cmd:
	nvme_cleanup_cmd(req);
	return ret;
}

static void apple_nvme_common_complete_rq(struct request *req)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct apple_nvme_dev *dev = iod->nvmeq->dev;

	if (blk_integrity_rq(req))
		dma_unmap_page(dev->dev, iod->meta_dma,
			       rq_integrity_vec(req)->bv_len, rq_data_dir(req));
	if (blk_rq_nr_phys_segments(req))
		apple_nvme_unmap_data(dev, req);
	nvme_complete_rq(req);
}

/* We read the CQE phase first to check if the rest of the entry is valid */
static inline bool apple_nvme_cqe_pending(struct apple_nvme_queue *nvmeq)
{
	struct nvme_completion *hcqe = &nvmeq->cqes[nvmeq->cq_head];

	return (le16_to_cpu(READ_ONCE(hcqe->status)) & 1) == nvmeq->cq_phase;
}

static inline void apple_nvme_ring_cq_doorbell(struct apple_nvme_queue *nvmeq)
{
	writel(nvmeq->cq_head, nvmeq->q_db + nvmeq->dev->db_stride);
}

static inline struct blk_mq_tags *apple_nvme_queue_tagset(struct apple_nvme_queue *nvmeq)
{
	if (nvmeq->is_adminq)
		return nvmeq->dev->admin_tagset.tags[0];
	else
		return nvmeq->dev->tagset.tags[0];
}

static inline int apple_nvme_queue_id(struct apple_nvme_queue *nvmeq)
{
	return nvmeq->is_adminq ? 0 : 1;
}

static inline void apple_nvme_handle_cqe(struct apple_nvme_queue *nvmeq, u16 idx)
{
	struct nvme_completion *cqe = &nvmeq->cqes[idx];
	__u16 command_id = READ_ONCE(cqe->command_id);
	struct request *req;

	apple_nvmmu_inval(nvmeq, nvme_tag_from_cid(command_id));

	/*
	 * AEN requests are special as they don't time out and can
	 * survive any kind of queue freeze and often don't respond to
	 * aborts.  We don't even bother to allocate a struct request
	 * for them but rather special case them here.
	 */
	if (unlikely(nvme_is_aen_req(apple_nvme_queue_id(nvmeq), command_id))) {
		nvme_complete_async_event(&nvmeq->dev->ctrl,
				cqe->status, &cqe->result);
		return;
	}

	req = nvme_find_rq(apple_nvme_queue_tagset(nvmeq), command_id);
	if (unlikely(!req)) {
		dev_warn(nvmeq->dev->ctrl.device,
			"invalid id %d completed on queue %d\n",
			command_id, le16_to_cpu(cqe->sq_id));
		return;
	}

	if (!nvme_try_complete_req(req, cqe->status, cqe->result))
		apple_nvme_common_complete_rq(req);
}

static inline void apple_nvme_update_cq_head(struct apple_nvme_queue *nvmeq)
{
	u32 tmp = nvmeq->cq_head + 1;

	if (tmp == nvmeq->q_depth) {
		nvmeq->cq_head = 0;
		nvmeq->cq_phase ^= 1;
	} else {
		nvmeq->cq_head = tmp;
	}
}

static inline int nvme_process_cq(struct apple_nvme_queue *nvmeq)
{
	int found = 0;

	while (apple_nvme_cqe_pending(nvmeq)) {
		found++;
		/*
		 * load-load control dependency between phase and the rest of
		 * the cqe requires a full read memory barrier
		 */
		dma_rmb();
		apple_nvme_handle_cqe(nvmeq, nvmeq->cq_head);
		apple_nvme_update_cq_head(nvmeq);
	}

	if (found)
		apple_nvme_ring_cq_doorbell(nvmeq);
	return found;
}

static irqreturn_t apple_nvme_irq(int irq, void *data)
{
	struct apple_nvme_dev *dev = data;
	bool handled = false;

	if (dev->adminq_online && nvme_process_cq(dev->adminq))
		handled = true;
	if (dev->ioq_online && nvme_process_cq(dev->ioq))
		handled = true;

	if (handled)
		return IRQ_HANDLED;
	return IRQ_NONE;
}

/*
 * Poll for completions for any interrupt driven queue
 * Can be called from any context.
 */
static void apple_nvme_poll_irqdisable(struct apple_nvme_queue *nvmeq)
{
	disable_irq(nvmeq->dev->platform_irq);
	nvme_process_cq(nvmeq);
	enable_irq(nvmeq->dev->platform_irq);
}

static void apple_nvme_submit_async_event(struct nvme_ctrl *ctrl)
{
	struct apple_nvme_dev *dev = to_apple_nvme_dev(ctrl);
	struct nvme_command c = { };

	c.common.opcode = nvme_admin_async_event;
	c.common.command_id = NVME_AQ_BLK_MQ_DEPTH;
	apple_nvme_submit_cmd(dev->adminq, &c);
}

static int apple_adapter_delete_queue(struct apple_nvme_dev *dev, u8 opcode)
{
	struct nvme_command c = { };

	c.delete_queue.opcode = opcode;
	/* we only have a single IO queue */
	c.delete_queue.qid = cpu_to_le16(1);

	return nvme_submit_sync_cmd(dev->ctrl.admin_q, &c, NULL, 0);
}

static int apple_adapter_alloc_cq(struct apple_nvme_dev *dev,
		struct apple_nvme_queue *nvmeq)
{
	struct nvme_command c = { };
	int flags = NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED;

	/*
	 * Note: we (ab)use the fact that the prp fields survive if no data
	 * is attached to the request.
	 */
	c.create_cq.opcode = nvme_admin_create_cq;
	c.create_cq.prp1 = cpu_to_le64(nvmeq->cq_dma_addr);
	c.create_cq.cqid = cpu_to_le16(1);
	c.create_cq.qsize = cpu_to_le16(nvmeq->q_depth - 1);
	c.create_cq.cq_flags = cpu_to_le16(flags);
	c.create_cq.irq_vector = cpu_to_le16(0);

	return nvme_submit_sync_cmd(dev->ctrl.admin_q, &c, NULL, 0);
}

static int apple_adapter_alloc_sq(struct apple_nvme_dev *dev,
						struct apple_nvme_queue *nvmeq)
{
	struct nvme_command c = { };
	int flags = NVME_QUEUE_PHYS_CONTIG;

	/*
	 * Note: we (ab)use the fact that the prp fields survive if no data
	 * is attached to the request.
	 */
	c.create_sq.opcode = nvme_admin_create_sq;
	c.create_sq.prp1 = cpu_to_le64(nvmeq->sq_dma_addr);
	c.create_sq.sqid = cpu_to_le16(1);
	c.create_sq.qsize = cpu_to_le16(nvmeq->q_depth - 1);
	c.create_sq.sq_flags = cpu_to_le16(flags);
	c.create_sq.cqid = cpu_to_le16(1);

	return nvme_submit_sync_cmd(dev->ctrl.admin_q, &c, NULL, 0);
}

static int apple_adapter_delete_cq(struct apple_nvme_dev *dev)
{
	return apple_adapter_delete_queue(dev, nvme_admin_delete_cq);
}

static int apple_adapter_delete_sq(struct apple_nvme_dev *dev)
{
	return apple_adapter_delete_queue(dev, nvme_admin_delete_sq);
}

static void apple_abort_endio(struct request *req, blk_status_t error)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct apple_nvme_queue *nvmeq = iod->nvmeq;

	dev_warn(nvmeq->dev->ctrl.device,
		 "Abort status: 0x%x", nvme_req(req)->status);
	atomic_inc(&nvmeq->dev->ctrl.abort_limit);
	blk_mq_free_request(req);
}

static bool apple_nvme_should_reset(struct apple_nvme_dev *dev, u32 csts)
{
	/* If there is a reset/reinit ongoing, we shouldn't reset again. */
	switch (dev->ctrl.state) {
	case NVME_CTRL_RESETTING:
	case NVME_CTRL_CONNECTING:
		return false;
	default:
		break;
	}

	/* We shouldn't reset unless the controller is on fatal error state
	 */
	if (!(csts & NVME_CSTS_CFS))
		return false;

	return true;
}

static void apple_nvme_warn_reset(struct apple_nvme_dev *dev, u32 csts)
{
	dev_warn(dev->ctrl.device,
		 "controller is down; will reset: CSTS=0x%x\n", csts);
}

static enum blk_eh_timer_return apple_nvme_timeout(struct request *req, bool reserved)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct apple_nvme_queue *nvmeq = iod->nvmeq;
	struct apple_nvme_dev *dev = nvmeq->dev;
	struct request *abort_req;
	struct nvme_command cmd = { };
	u32 csts = readl(dev->nvme_mmio + NVME_REG_CSTS);

	/*
	 * Reset immediately if the controller is failed
	 */
	if (apple_nvme_should_reset(dev, csts)) {
		apple_nvme_warn_reset(dev, csts);
		apple_nvme_dev_disable(dev, false);
		nvme_reset_ctrl(&dev->ctrl);
		return BLK_EH_DONE;
	}

	/*
	 * Did we miss an interrupt?
	 */
	apple_nvme_poll_irqdisable(nvmeq);

	if (blk_mq_request_completed(req)) {
		dev_warn(dev->ctrl.device,
			 "I/O %d QID %d timeout, completion polled\n",
			 req->tag, apple_nvme_queue_id(nvmeq));
		return BLK_EH_DONE;
	}

	/*
	 * Shutdown immediately if controller times out while starting. The
	 * reset work will see the pci device disabled when it gets the forced
	 * cancellation error. All outstanding requests are completed on
	 * shutdown, so we return BLK_EH_DONE.
	 */
	switch (dev->ctrl.state) {
	case NVME_CTRL_CONNECTING:
		nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_DELETING);
		fallthrough;
	case NVME_CTRL_DELETING:
		dev_warn_ratelimited(dev->ctrl.device,
			 "I/O %d QID %d timeout, disable controller\n",
			 req->tag, apple_nvme_queue_id(nvmeq));
		nvme_req(req)->flags |= NVME_REQ_CANCELLED;
		apple_nvme_dev_disable(dev, true);
		return BLK_EH_DONE;
	case NVME_CTRL_RESETTING:
		return BLK_EH_RESET_TIMER;
	default:
		break;
	}

	/*
	 * Shutdown the controller immediately and schedule a reset if the
	 * command was already aborted once before and still hasn't been
	 * returned to the driver, or if this is the admin queue.
	 */
	if (nvmeq->is_adminq || iod->aborted) {
		dev_warn(dev->ctrl.device,
			 "I/O %d QID %d timeout, reset controller\n",
			 req->tag, apple_nvme_queue_id(nvmeq));
		nvme_req(req)->flags |= NVME_REQ_CANCELLED;
		apple_nvme_dev_disable(dev, false);
		nvme_reset_ctrl(&dev->ctrl);

		return BLK_EH_DONE;
	}

	if (atomic_dec_return(&dev->ctrl.abort_limit) < 0) {
		atomic_inc(&dev->ctrl.abort_limit);
		return BLK_EH_RESET_TIMER;
	}
	iod->aborted = 1;

	cmd.abort.opcode = nvme_admin_abort_cmd;
	cmd.abort.cid = req->tag;
	cmd.abort.sqid = cpu_to_le16(1);

	dev_warn(nvmeq->dev->ctrl.device,
		"I/O %d timeout, aborting\n",
		 req->tag);

	abort_req = nvme_alloc_request(dev->ctrl.admin_q, &cmd,
			BLK_MQ_REQ_NOWAIT);
	if (IS_ERR(abort_req)) {
		atomic_inc(&dev->ctrl.abort_limit);
		return BLK_EH_RESET_TIMER;
	}

	abort_req->end_io_data = NULL;
	blk_execute_rq_nowait(abort_req, false, apple_abort_endio);

	/*
	 * The aborted req will be completed on receiving the abort req.
	 * We enable the timer again. If hit twice, it'll cause a device reset,
	 * as the device then is in a faulty state.
	 */
	return BLK_EH_RESET_TIMER;
}

static void apple_nvme_free_queue(struct apple_nvme_queue *nvmeq)
{
	dma_free_coherent(nvmeq->dev->dev, CQ_SIZE(nvmeq), (void *)nvmeq->cqes,
			  nvmeq->cq_dma_addr);
	if (!nvmeq->sq_cmds)
		return;

	dma_free_coherent(nvmeq->dev->dev, APPLE_NVMMU_TCB_SIZE,
			  nvmeq->ans2_tcb_ptr, nvmeq->ans2_tcb_dma_addr);
	dma_free_coherent(nvmeq->dev->dev, SQ_SIZE(nvmeq), nvmeq->sq_cmds,
			  nvmeq->sq_dma_addr);
}

/**
 * apple_nvme_suspend_queue - put queue into suspended state
 * @nvmeq: queue to suspend
 */
static int apple_nvme_suspend_queue(struct apple_nvme_queue *nvmeq)
{
	if (!test_and_clear_bit(NVMEQ_ENABLED, &nvmeq->flags))
		return 1;

	/* ensure that apple_nvme_queue_rq() sees NVMEQ_ENABLED cleared */
	mb();

	if (nvmeq->is_adminq && nvmeq->dev->ctrl.admin_q)
		blk_mq_quiesce_queue(nvmeq->dev->ctrl.admin_q);
	if (nvmeq->is_adminq)
		nvmeq->dev->adminq_online = false;
	else
		nvmeq->dev->ioq_online = false;
	return 0;
}

static void apple_nvme_disable_admin_queue(struct apple_nvme_dev *dev, bool shutdown)
{
	if (shutdown)
		nvme_shutdown_ctrl(&dev->ctrl);
	else
		nvme_disable_ctrl(&dev->ctrl);

	apple_nvme_poll_irqdisable(dev->adminq);
}

static int apple_nvme_alloc_queue(struct apple_nvme_dev *dev, bool is_adminq)
{
	struct apple_nvme_queue *nvmeq;

	if (is_adminq) {
		nvmeq = dev->adminq;
		nvmeq->sqes = NVME_ADM_SQES;
		nvmeq->q_depth = NVME_AQ_DEPTH;
	} else {
		nvmeq = dev->ioq;
		nvmeq->sqes = NVME_NVM_IOSQES;
		nvmeq->q_depth = APPLE_ANS2_QUEUE_DEPTH;
	}

	nvmeq->cqes = dma_alloc_coherent(dev->dev, CQ_SIZE(nvmeq),
					 &nvmeq->cq_dma_addr, GFP_KERNEL);
	if (!nvmeq->cqes)
		goto free_nvmeq;

	nvmeq->ans2_tcb_ptr =
		dma_alloc_coherent(dev->dev, APPLE_NVMMU_TCB_SIZE,
				   &nvmeq->ans2_tcb_dma_addr, GFP_KERNEL);
	if (!nvmeq->ans2_tcb_ptr)
		goto free_cqdma;

	lo_hi_writeq(nvmeq->ans2_tcb_dma_addr, nvmeq->nvmmu_base);

	nvmeq->sq_cmds = dma_alloc_coherent(dev->dev, SQ_SIZE(nvmeq),
				&nvmeq->sq_dma_addr, GFP_KERNEL);
	if (!nvmeq->sq_cmds)
		goto free_ans2;

	nvmeq->dev = dev;
	spin_lock_init(&nvmeq->sq_lock);
	nvmeq->cq_head = 0;
	nvmeq->cq_phase = 1;
	nvmeq->q_db = &dev->dbs[apple_nvme_queue_id(nvmeq) * 2 * dev->db_stride];
	dev->ctrl.queue_count++;

	return 0;

 free_ans2:
	lo_hi_writeq(0, nvmeq->nvmmu_base);
	dma_free_coherent(dev->dev, APPLE_NVMMU_TCB_SIZE,
			  nvmeq->ans2_tcb_ptr,
			  nvmeq->ans2_tcb_dma_addr);
 free_cqdma:
	dma_free_coherent(dev->dev, CQ_SIZE(nvmeq), (void *)nvmeq->cqes,
			  nvmeq->cq_dma_addr);
 free_nvmeq:
	return -ENOMEM;
}

static void apple_nvme_init_queue(struct apple_nvme_queue *nvmeq)
{
	struct apple_nvme_dev *dev = nvmeq->dev;

	nvmeq->cq_head = 0;
	nvmeq->cq_phase = 1;
	nvmeq->q_db = &dev->dbs[apple_nvme_queue_id(nvmeq) * 2 * dev->db_stride];
	memset((void *)nvmeq->cqes, 0, CQ_SIZE(nvmeq));
	wmb(); /* ensure the first interrupt sees the initialization */
}

static int apple_nvme_map_queues(struct blk_mq_tag_set *set)
{
	if (WARN_ON(set->nr_maps != 2))
		return -EINVAL;

	set->map[HCTX_TYPE_DEFAULT].nr_queues = 1;
	set->map[HCTX_TYPE_READ].nr_queues = 0;

	return 0;
}

static const struct blk_mq_ops apple_nvme_mq_admin_ops = {
	.queue_rq	= apple_nvme_queue_rq,
	.complete	= apple_nvme_common_complete_rq,
	.init_hctx	= apple_nvme_admin_init_hctx,
	.init_request	= apple_nvme_init_request,
	.timeout	= apple_nvme_timeout,
};

static const struct blk_mq_ops apple_nvme_mq_ops = {
	.queue_rq	= apple_nvme_queue_rq,
	.complete	= apple_nvme_common_complete_rq,
	.init_hctx	= apple_nvme_init_hctx,
	.init_request	= apple_nvme_init_request,
	.timeout	= apple_nvme_timeout,
        .map_queues     = apple_nvme_map_queues,
};

static void apple_nvme_dev_remove_admin(struct apple_nvme_dev *dev)
{
	if (dev->ctrl.admin_q && !blk_queue_dying(dev->ctrl.admin_q)) {
		/*
		 * If the controller was reset during removal, it's possible
		 * user requests may be waiting on a stopped queue. Start the
		 * queue to flush these to completion.
		 */
		blk_mq_unquiesce_queue(dev->ctrl.admin_q);
		blk_cleanup_queue(dev->ctrl.admin_q);
		blk_mq_free_tag_set(&dev->admin_tagset);
	}
}

static int apple_nvme_alloc_admin_tags(struct apple_nvme_dev *dev)
{
	if (!dev->ctrl.admin_q) {
		dev->admin_tagset.ops = &apple_nvme_mq_admin_ops;
		dev->admin_tagset.nr_hw_queues = 1;

		dev->admin_tagset.queue_depth = NVME_AQ_MQ_TAG_DEPTH;
		dev->admin_tagset.timeout = NVME_ADMIN_TIMEOUT;
		dev->admin_tagset.numa_node = dev->ctrl.numa_node;
		dev->admin_tagset.cmd_size = sizeof(struct apple_nvme_iod);
		dev->admin_tagset.flags = BLK_MQ_F_NO_SCHED;
		dev->admin_tagset.driver_data = dev;

		if (blk_mq_alloc_tag_set(&dev->admin_tagset))
			return -ENOMEM;
		dev->ctrl.admin_tagset = &dev->admin_tagset;

		dev->ctrl.admin_q = blk_mq_init_queue(&dev->admin_tagset);
		if (IS_ERR(dev->ctrl.admin_q)) {
			blk_mq_free_tag_set(&dev->admin_tagset);
			return -ENOMEM;
		}
		if (!blk_get_queue(dev->ctrl.admin_q)) {
			apple_nvme_dev_remove_admin(dev);
			dev->ctrl.admin_q = NULL;
			return -ENODEV;
		}
	} else
		blk_mq_unquiesce_queue(dev->ctrl.admin_q);

	return 0;
}

static int apple_nvme_configure_admin_queue(struct apple_nvme_dev *dev)
{
	int result;
	u32 aqa;

	result = nvme_disable_ctrl(&dev->ctrl);
	if (result < 0)
		return result;

	result = apple_nvme_alloc_queue(dev, true);
	if (result)
		return result;

	dev->ctrl.numa_node = dev_to_node(dev->dev);

	aqa = dev->adminq->q_depth - 1;
	aqa |= aqa << 16;

	writel(aqa, dev->nvme_mmio + NVME_REG_AQA);
	lo_hi_writeq(dev->adminq->sq_dma_addr, dev->nvme_mmio + NVME_REG_ASQ);
	lo_hi_writeq(dev->adminq->cq_dma_addr, dev->nvme_mmio + NVME_REG_ACQ);

	result = nvme_enable_ctrl(&dev->ctrl);
	if (result)
		return result;

	apple_nvme_init_queue(dev->adminq);

	dev->adminq_online = true;
	set_bit(NVMEQ_ENABLED, &dev->adminq->flags);
	return result;
}

static int apple_nvme_setup_io_queues_trylock(struct apple_nvme_dev *dev)
{
	/*
	 * Give up if the lock is being held by nvme_dev_disable.
	 */
	if (!mutex_trylock(&dev->shutdown_lock))
		return -ENODEV;

	/*
	 * Controller is in wrong state, fail early.
	 */
	if (dev->ctrl.state != NVME_CTRL_CONNECTING) {
		mutex_unlock(&dev->shutdown_lock);
		return -ENODEV;
	}

	return 0;
}

static int apple_nvme_create_io_queue(struct apple_nvme_dev *dev)
{
	int ret = 0;

	if (apple_nvme_alloc_queue(dev, false))
		return -ENOMEM;

	clear_bit(NVMEQ_DELETE_ERROR, &dev->ioq->flags);

	ret = apple_adapter_alloc_cq(dev, dev->ioq);
	if (ret)
		return ret;

	ret = apple_adapter_alloc_sq(dev, dev->ioq);
	if (ret)
		goto release_cq;

	ret = apple_nvme_setup_io_queues_trylock(dev);
	if (ret)
		goto release_sq;
	apple_nvme_init_queue(dev->ioq);

	dev->ioq_online = true;
	set_bit(NVMEQ_ENABLED, &dev->ioq->flags);
	mutex_unlock(&dev->shutdown_lock);
	return 0;

release_sq:
	apple_adapter_delete_sq(dev);
release_cq:
	apple_adapter_delete_cq(dev);
	return ret;
}

static void apple_nvme_disable_io_queues(struct apple_nvme_dev *dev)
{
	if (__apple_nvme_disable_io_queues(dev, nvme_admin_delete_sq))
		__apple_nvme_disable_io_queues(dev, nvme_admin_delete_cq);
}

static int apple_nvme_setup_io_queues(struct apple_nvme_dev *dev)
{
	unsigned int nr_io_queues = 1;
	int result;

	result = nvme_set_queue_count(&dev->ctrl, &nr_io_queues);
	if (result < 0)
		return result;

	if (nr_io_queues == 0)
		return 0;

	result = apple_nvme_create_io_queue(dev);
	if (result || !dev->ioq_online)
		return result;

	return 0;
}

static void apple_nvme_del_queue_end(struct request *req, blk_status_t error)
{
	struct apple_nvme_queue *nvmeq = req->end_io_data;

	blk_mq_free_request(req);
	complete(&nvmeq->delete_done);
}

static void apple_nvme_del_cq_end(struct request *req, blk_status_t error)
{
	struct apple_nvme_queue *nvmeq = req->end_io_data;

	if (error)
		set_bit(NVMEQ_DELETE_ERROR, &nvmeq->flags);

	apple_nvme_del_queue_end(req, error);
}

static int apple_nvme_delete_queue(struct apple_nvme_queue *nvmeq, u8 opcode)
{
	struct request_queue *q = nvmeq->dev->ctrl.admin_q;
	struct request *req;
	struct nvme_command cmd = { };

	cmd.delete_queue.opcode = opcode;
	 /* we only have a single IO queue */
	cmd.delete_queue.qid = cpu_to_le16(1);

	req = nvme_alloc_request(q, &cmd, BLK_MQ_REQ_NOWAIT);
	if (IS_ERR(req))
		return PTR_ERR(req);

	req->end_io_data = nvmeq;

	init_completion(&nvmeq->delete_done);
	blk_execute_rq_nowait(req, false,
			opcode == nvme_admin_delete_cq ?
				apple_nvme_del_cq_end : apple_nvme_del_queue_end);
	return 0;
}

static bool __apple_nvme_disable_io_queues(struct apple_nvme_dev *dev, u8 opcode)
{
	unsigned long timeout = NVME_ADMIN_TIMEOUT;

	if (!dev->ioq_online)
		return true;

	if (apple_nvme_delete_queue(dev->ioq, opcode))
		return false;

	timeout =
		wait_for_completion_io_timeout(&dev->ioq->delete_done, timeout);
	if (timeout == 0)
		return false;
	return true;
}

static void apple_nvme_dev_add(struct apple_nvme_dev *dev)
{
	int ret;

	if (!dev->ctrl.tagset) {
		dev->tagset.ops = &apple_nvme_mq_ops;
		dev->tagset.nr_hw_queues = 1;
		dev->tagset.nr_maps = 2; /* admin + io */
		dev->tagset.timeout = NVME_IO_TIMEOUT;
		dev->tagset.numa_node = dev->ctrl.numa_node;
		dev->tagset.queue_depth = APPLE_ANS2_QUEUE_DEPTH - 1;
		dev->tagset.cmd_size = sizeof(struct apple_nvme_iod);
		dev->tagset.flags = BLK_MQ_F_SHOULD_MERGE;
		dev->tagset.driver_data = dev;

		/*
		 * This Apple controller requires tags to be unique
		 * across admin and IO queue, so reserve the first 32
		 * tags of the IO queue.
		 */
		dev->tagset.reserved_tags = NVME_AQ_DEPTH;

		ret = blk_mq_alloc_tag_set(&dev->tagset);
		if (ret) {
			dev_warn(dev->ctrl.device,
				"IO queues tagset allocation failed %d\n", ret);
			return;
		}
		dev->ctrl.tagset = &dev->tagset;
	} else {
		WARN_ON(!dev->adminq_online);
		WARN_ON(!dev->ioq_online);

		blk_mq_update_nr_hw_queues(&dev->tagset, 1);

		/* Free previously allocated IO queue that is no longer usable */
		apple_nvme_free_queue(dev->ioq);
		dev->ctrl.queue_count--;
	}
}

static int apple_nvme_enable(struct apple_nvme_dev *dev)
{
	if (readl(dev->nvme_mmio + NVME_REG_CSTS) == -1)
		return -ENODEV;

	dev->ctrl.cap = lo_hi_readq(dev->nvme_mmio + NVME_REG_CAP);

	dev->ctrl.sqsize = APPLE_ANS2_QUEUE_DEPTH - 1; /* 0's based queue depth */
	dev->db_stride = 1 << NVME_CAP_STRIDE(dev->ctrl.cap);
	dev->dbs = dev->nvme_mmio + 4096;

	return 0;
}

static void apple_nvme_dev_disable(struct apple_nvme_dev *dev, bool shutdown)
{
	bool dead = true, freeze = false;
	u32 csts;

	mutex_lock(&dev->shutdown_lock);
	csts = readl(dev->nvme_mmio + NVME_REG_CSTS);

	if (dev->ctrl.state == NVME_CTRL_LIVE ||
	    dev->ctrl.state == NVME_CTRL_RESETTING) {
		freeze = true;
		nvme_start_freeze(&dev->ctrl);
	}
	dead = !!((csts & NVME_CSTS_CFS) || !(csts & NVME_CSTS_RDY));

	/*
	 * Give the controller a chance to complete all entered requests if
	 * doing a safe shutdown.
	 */
	if (!dead && shutdown && freeze)
		nvme_wait_freeze_timeout(&dev->ctrl, NVME_IO_TIMEOUT);

	nvme_stop_queues(&dev->ctrl);

	if (!dead && dev->ctrl.queue_count > 0) {
		apple_nvme_disable_io_queues(dev);
		apple_nvme_disable_admin_queue(dev, shutdown);
	}
	if (dev->ioq_online)
		apple_nvme_suspend_queue(dev->ioq);
	apple_nvme_suspend_queue(dev->adminq);
	if (dev->ctrl.queue_count > 1)
		nvme_process_cq(dev->ioq);

	blk_mq_tagset_busy_iter(&dev->tagset, nvme_cancel_request, &dev->ctrl);
	blk_mq_tagset_busy_iter(&dev->admin_tagset, nvme_cancel_request, &dev->ctrl);
	blk_mq_tagset_wait_completed_request(&dev->tagset);
	blk_mq_tagset_wait_completed_request(&dev->admin_tagset);

	/*
	 * The driver will not be starting up queues again if shutting down so
	 * must flush all entered requests to their failed completion to avoid
	 * deadlocking blk-mq hot-cpu notifier.
	 */
	if (shutdown) {
		nvme_start_queues(&dev->ctrl);
		if (dev->ctrl.admin_q && !blk_queue_dying(dev->ctrl.admin_q))
			blk_mq_unquiesce_queue(dev->ctrl.admin_q);
	}
	mutex_unlock(&dev->shutdown_lock);
}

static int apple_nvme_disable_prepare_reset(struct apple_nvme_dev *dev, bool shutdown)
{
	if (!nvme_wait_reset(&dev->ctrl))
		return -EBUSY;
	apple_nvme_dev_disable(dev, shutdown);
	return 0;
}

static int apple_nvme_setup_prp_pools(struct apple_nvme_dev *dev)
{
	dev->prp_page_pool = dma_pool_create("prp list page", dev->dev,
						NVME_CTRL_PAGE_SIZE,
						NVME_CTRL_PAGE_SIZE, 0);
	if (!dev->prp_page_pool)
		return -ENOMEM;

	/* Optimisation for I/Os between 4k and 128k */
	dev->prp_small_pool = dma_pool_create("prp list 256", dev->dev,
						256, 256, 0);
	if (!dev->prp_small_pool) {
		dma_pool_destroy(dev->prp_page_pool);
		return -ENOMEM;
	}
	return 0;
}

static void apple_nvme_release_prp_pools(struct apple_nvme_dev *dev)
{
	dma_pool_destroy(dev->prp_page_pool);
	dma_pool_destroy(dev->prp_small_pool);
}

static void apple_nvme_free_tagset(struct apple_nvme_dev *dev)
{
	if (dev->tagset.tags)
		blk_mq_free_tag_set(&dev->tagset);
	dev->ctrl.tagset = NULL;
}

static void apple_nvme_free_ctrl(struct nvme_ctrl *ctrl)
{
	struct apple_nvme_dev *dev = to_apple_nvme_dev(ctrl);

	apple_nvme_free_tagset(dev);
	if (dev->ctrl.admin_q)
		blk_put_queue(dev->ctrl.admin_q);
	mempool_destroy(dev->iod_mempool);
	put_device(dev->dev);
	kfree(dev->adminq);
	kfree(dev->ioq);
	kfree(dev);
}

static void apple_nvme_remove_dead_ctrl(struct apple_nvme_dev *dev)
{
	/*
	 * Set state to deleting now to avoid blocking nvme_wait_reset(), which
	 * may be holding this pci_dev's device lock.
	 */
	nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_DELETING);
	nvme_get_ctrl(&dev->ctrl);
	apple_nvme_dev_disable(dev, false);
	nvme_kill_queues(&dev->ctrl);
	if (!queue_work(nvme_wq, &dev->remove_work))
		nvme_put_ctrl(&dev->ctrl);
}

static void apple_nvme_reset_work(struct work_struct *work)
{
	struct apple_nvme_dev *dev =
		container_of(work, struct apple_nvme_dev, ctrl.reset_work);
	int result;

	if (WARN_ON(dev->ctrl.state != NVME_CTRL_RESETTING)) {
		result = -ENODEV;
		goto out;
	}

	/*
	 * If we're called to reset a live controller first shut it down before
	 * moving on.
	 */
	if (dev->ctrl.ctrl_config & NVME_CC_ENABLE)
		apple_nvme_dev_disable(dev, false);
	nvme_sync_queues(&dev->ctrl);

	mutex_lock(&dev->shutdown_lock);

	result = apple_nvme_enable(dev);
	if (result)
		goto out_unlock;

	result = apple_nvme_configure_admin_queue(dev);
	if (result)
		goto out_unlock;

	result = apple_nvme_alloc_admin_tags(dev);
	if (result)
		goto out_unlock;

	/*
	 * Limit the max command size to prevent iod->sg allocations going
	 * over a single page.
	 */
	dev->ctrl.max_hw_sectors = min_t(u32,
		NVME_MAX_KB_SZ << 1, dma_max_mapping_size(dev->dev) >> 9);
	dev->ctrl.max_segments = NVME_MAX_SEGS;

	/*
	 * Don't limit the IOMMU merged segment size.
	 */
	dma_set_max_seg_size(dev->dev, 0xffffffff);
	dma_set_min_align_mask(dev->dev, NVME_CTRL_PAGE_SIZE - 1);

	mutex_unlock(&dev->shutdown_lock);

	/*
	 * Introduce CONNECTING state from nvme-fc/rdma transports to mark the
	 * initializing procedure here.
	 */
	if (!nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_CONNECTING)) {
		dev_warn(dev->ctrl.device,
			"failed to mark controller CONNECTING\n");
		result = -EBUSY;
		goto out;
	}

	/*
	 * We do not support an SGL for metadata (yet), so we are limited to a
	 * single integrity segment for the separate metadata pointer.
	 */
	dev->ctrl.max_integrity_segments = 1;

	result = nvme_init_ctrl_finish(&dev->ctrl);
	if (result)
		goto out;

	result = apple_nvme_setup_io_queues(dev);
	if (result)
		goto out;

	/*
	 * Keep the controller around but remove all namespaces if we don't have
	 * any working I/O queue.
	 */
	if (!dev->ioq_online) {
		dev_warn(dev->ctrl.device, "IO queues not created\n");
		nvme_kill_queues(&dev->ctrl);
		nvme_remove_namespaces(&dev->ctrl);
		apple_nvme_free_tagset(dev);
	} else {
		nvme_start_queues(&dev->ctrl);
		nvme_wait_freeze(&dev->ctrl);
		apple_nvme_dev_add(dev);
		nvme_unfreeze(&dev->ctrl);
	}

	/*
	 * If only admin queue live, keep it to do further investigation or
	 * recovery.
	 */
	if (!nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_LIVE)) {
		dev_warn(dev->ctrl.device,
			"failed to mark controller live state\n");
		result = -ENODEV;
		goto out;
	}

	nvme_start_ctrl(&dev->ctrl);
	return;

 out_unlock:
	mutex_unlock(&dev->shutdown_lock);
 out:
	if (result)
		dev_warn(dev->ctrl.device,
			 "Removing after probe failure status: %d\n", result);
	apple_nvme_remove_dead_ctrl(dev);
}

static void apple_nvme_remove_dead_ctrl_work(struct work_struct *work)
{
	struct apple_nvme_dev *dev = container_of(work, struct apple_nvme_dev, remove_work);

	if (dev_get_drvdata(dev->dev))
		device_release_driver(dev->dev);
	nvme_put_ctrl(&dev->ctrl);
}

static int apple_nvme_reg_read32(struct nvme_ctrl *ctrl, u32 off, u32 *val)
{
	*val = readl(to_apple_nvme_dev(ctrl)->nvme_mmio + off);
	return 0;
}

static int apple_nvme_reg_write32(struct nvme_ctrl *ctrl, u32 off, u32 val)
{
	writel(val, to_apple_nvme_dev(ctrl)->nvme_mmio + off);
	return 0;
}

static int apple_nvme_reg_read64(struct nvme_ctrl *ctrl, u32 off, u64 *val)
{
	*val = lo_hi_readq(to_apple_nvme_dev(ctrl)->nvme_mmio + off);
	return 0;
}

static int apple_nvme_get_address(struct nvme_ctrl *ctrl, char *buf, int size)
{
	struct device *dev = to_apple_nvme_dev(ctrl)->dev;

	return snprintf(buf, size, "%s\n", dev_name(dev));
}

static struct apple_nvme_dev *apple_nvme_dev_alloc(struct device *parent)
{
	struct apple_nvme_dev *dev;
	size_t alloc_size;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	dev->dev = get_device(parent);
        dev->adminq = kzalloc(sizeof(*dev->adminq), GFP_KERNEL);
        if (!dev->adminq)
                goto free_dev;

	dev->ioq = kzalloc(sizeof(*dev->ioq), GFP_KERNEL);
        if (!dev->ioq)
                goto free_adminq;

	dev->adminq->is_adminq = true;
	dev->ioq->is_adminq = false;

	INIT_WORK(&dev->ctrl.reset_work, apple_nvme_reset_work);
	INIT_WORK(&dev->remove_work, apple_nvme_remove_dead_ctrl_work);
	mutex_init(&dev->shutdown_lock);

	if (apple_nvme_setup_prp_pools(dev))
		goto free_ioq;

	/*
	 * Double check that our mempool alloc size will cover the biggest
	 * command we support.
	 */
	alloc_size = apple_nvme_iod_alloc_size();
	WARN_ON_ONCE(alloc_size > PAGE_SIZE);

	dev->iod_mempool = mempool_create(1, mempool_kmalloc,
						mempool_kfree,
						(void *) alloc_size);
	if (!dev->iod_mempool)
		goto free_pools;

	return dev;

free_pools:
	apple_nvme_release_prp_pools(dev);
free_ioq:
	kfree(dev->ioq);
free_adminq:
        kfree(dev->adminq);
free_dev:
	kfree(dev);
	return NULL;
}

static void apple_nvme_dev_free(struct apple_nvme_dev *dev)
{
	mempool_destroy(dev->iod_mempool);
	apple_nvme_release_prp_pools(dev);
	put_device(dev->dev);
	kfree(dev->adminq);
	kfree(dev->ioq);
	kfree(dev);
}

/*
 * The driver's remove may be called on a device in a partially initialized
 * state. This function must not have any dependencies on the device state in
 * order to proceed.
 */
static int apple_nvme_remove(struct platform_device *pdev)
{
	struct apple_nvme_dev *dev = platform_get_drvdata(pdev);

	nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_DELETING);
	platform_set_drvdata(pdev, NULL);
	flush_work(&dev->ctrl.reset_work);
	nvme_stop_ctrl(&dev->ctrl);
	nvme_remove_namespaces(&dev->ctrl);
	apple_nvme_dev_disable(dev, true);
	apple_nvme_dev_remove_admin(dev);
	if (dev->ctrl.queue_count > 1)
		apple_nvme_free_queue(dev->ioq);
	if (dev->ctrl.queue_count > 0)
		apple_nvme_free_queue(dev->adminq);
        dev->ctrl.queue_count = 0;
	apple_nvme_release_prp_pools(dev);
	nvme_uninit_ctrl(&dev->ctrl);

	return 0;
}

static const struct nvme_ctrl_ops nvme_ctrl_ops = {
	.name			= "platform",
	.module			= THIS_MODULE,
	.flags			= NVME_F_METADATA_SUPPORTED,
	.reg_read32		= apple_nvme_reg_read32,
	.reg_write32		= apple_nvme_reg_write32,
	.reg_read64		= apple_nvme_reg_read64,
	.free_ctrl		= apple_nvme_free_ctrl,
	.submit_async_event	= apple_nvme_submit_async_event,
	.get_address		= apple_nvme_get_address,
};

static void apple_nvme_async_probe(void *data, async_cookie_t cookie)
{
	struct apple_nvme_dev *dev = data;

	flush_work(&dev->ctrl.reset_work);
	flush_work(&dev->ctrl.scan_work);
	nvme_put_ctrl(&dev->ctrl);
}

static void apple_nvme_rx_callback(void *cookie, u8 endpoint, u64 message)
{
        struct apple_nvme_dev *dev = cookie;
	dev_warn(dev->dev, "Unexpected message from ANS2: %016llx\n", (u64)message);
}

static void *apple_nvme_sart_alloc(void *cookie, size_t size, dma_addr_t *dma_handle,
			    gfp_t flag)
{
	struct apple_nvme_dev *dev = cookie;
	void *cpu_addr = dma_alloc_coherent(dev->dev, size, dma_handle, flag);

	apple_sart_add_allowed_region(dev->sart, *dma_handle, size);

	return cpu_addr;
}

static struct apple_rtkit_ops sart_rtkit_ops =
{
	.flags = APPLE_RTKIT_SHMEM_OWNER_LINUX,
	.shmem_alloc = apple_nvme_sart_alloc,
	.recv_message = apple_nvme_rx_callback,
};

static int apple_nvme_probe(struct platform_device *pdev)
{
	int result, ret;
	struct apple_nvme_dev *dev;
	struct resource *res;
	u32 ans2_boot_status;

	dev = apple_nvme_dev_alloc(&pdev->dev);
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);
	dev->nvme_mmio = devm_platform_ioremap_resource(pdev, 0);
	result = PTR_ERR_OR_ZERO(dev->nvme_mmio);
	if (result)
		goto out;

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64))) {
		result = ENXIO;
		goto out_unmap;
	}

	dev->dbs = dev->nvme_mmio + NVME_REG_DBS;
	dev->adminq->ans2_q_db = dev->nvme_mmio + APPLE_ANS2_LINEAR_ASQ_DB;
	dev->adminq->nvmmu_base = dev->nvme_mmio + APPLE_NVMMU_BASE_ASQ;
	dev->ioq->ans2_q_db = dev->nvme_mmio + APPLE_ANS2_LINEAR_IOSQ_DB;
	dev->ioq->nvmmu_base = dev->nvme_mmio + APPLE_NVMMU_BASE_IOSQ;

	dev->platform_irq = platform_get_irq(pdev, 0);
	if (dev->platform_irq < 0) {
		result = dev->platform_irq;
		goto out_unmap;
	}

	result = devm_request_irq(dev->dev, dev->platform_irq, apple_nvme_irq,
				  0, "nvme", dev);
	if (result)
		goto out_unmap;

	dev->sart = apple_sart_get(&pdev->dev);
	if (IS_ERR(dev->sart)) {
		result = PTR_ERR(dev->sart);
		goto out_unmap;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "coproc");
	if (!res) {
		result = -EINVAL;
		goto out_unmap;
	}

	dev->rtk = apple_rtkit_init(dev->dev, dev, res, NULL, 0,
				    &sart_rtkit_ops);
	if (!dev->rtk) {
		result = PTR_ERR(dev->rtk);
		goto out_unmap;
	}

	ret = apple_rtkit_boot_wait(dev->rtk, APPLE_ANS_BOOT_TIMEOUT);
	if (ret) {
		dev_err(dev->dev, "RTKit did not boot");
		goto out_unmap;
	}

	result = readl_poll_timeout(
		dev->nvme_mmio + APPLE_ANS2_BOOT_STATUS, ans2_boot_status,
		ans2_boot_status == APPLE_ANS2_BOOT_STATUS_OK, 100, 10000000);
	if (result) {
		dev_err(dev->dev, "ANS did not boot");
		goto out_unmap;
	}

	writel(APPLE_ANS2_MAX_PEND_CMDS | (APPLE_ANS2_MAX_PEND_CMDS << 16),
	       dev->nvme_mmio + APPLE_ANS2_MAX_PEND_CMDS_CTRL);
	writel(APPLE_ANS2_LINEAR_SQ_EN, dev->nvme_mmio + APPLE_ANS2_LINEAR_SQ_CTRL);
	writel(readl(dev->nvme_mmio + APPLE_ANS2_UNKNOWN_CTRL) &
		       ~APPLE_ANS2_PRP_NULL_CHECK,
	       dev->nvme_mmio + APPLE_ANS2_UNKNOWN_CTRL);
	writel(APPLE_NVMMU_NUM_TCBS - 1, dev->nvme_mmio + APPLE_NVMMU_NUM);

	result = nvme_init_ctrl(&dev->ctrl, &pdev->dev, &nvme_ctrl_ops,
				NVME_QUIRK_NO_SCAN_NS_LIST | NVME_QUIRK_SKIP_CID_GEN);
	if (result)
		goto out_unmap;

	nvme_reset_ctrl(&dev->ctrl);
	async_schedule(apple_nvme_async_probe, dev);

	return 0;

out_unmap:
	devm_iounmap(&pdev->dev, dev->nvme_mmio);
out:
	apple_nvme_dev_free(dev);
	return result;
}

static void apple_nvme_shutdown(struct platform_device *pdev)
{
	struct apple_nvme_dev *dev = platform_get_drvdata(pdev);

	apple_nvme_disable_prepare_reset(dev, true);
}

static const struct of_device_id nvme_of_device_ids[] = {
	{ .compatible = "apple,t8103-ans-nvme", },
	{},
};

static struct platform_driver nvme_driver = {
	.driver = {
		.name = "apple-ans-nvme",
		.owner = THIS_MODULE,
		.of_match_table = nvme_of_device_ids,
	},
	.probe = apple_nvme_probe,
	.remove = apple_nvme_remove,
	.shutdown = apple_nvme_shutdown,
};

static int __init apple_nvme_init(void)
{
	int ret;

	BUILD_BUG_ON(sizeof(struct nvme_create_cq) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_create_sq) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_delete_queue) != 64);
	BUILD_BUG_ON(sizeof(struct apple_nvmmu_tcb) != 128);
	BUILD_BUG_ON(BLK_MQ_MAX_DEPTH < APPLE_ANS2_QUEUE_DEPTH);

	ret = platform_driver_register(&nvme_driver);

	return ret;
}

static void __exit apple_nvme_exit(void)
{
	platform_driver_unregister(&nvme_driver);
	flush_workqueue(nvme_wq);
}

MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
module_init(apple_nvme_init);
module_exit(apple_nvme_exit);
