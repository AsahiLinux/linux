// SPDX-License-Identifier: GPL-2.0-only OR MIT

#include <linux/apple-rtkit.h>
#include <linux/apple-mailbox.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#include <linux/dma-mapping.h>
#include <linux/mailbox_client.h>
#include <linux/io.h>
#include <linux/wait.h>

#define rtk_err(format, arg...) dev_err(rtk->dev, "RTKit: " format, ##arg)
#define rtk_warn(format, arg...) dev_warn(rtk->dev, "RTKit: " format, ##arg)
#define rtk_info(format, arg...) dev_info(rtk->dev, "RTKit: " format, ##arg)
#define rtk_dbg(format, arg...) dev_dbg(rtk->dev, "RTKit: " format, ##arg)

struct apple_rtkit_shmem {
	void *buffer;
	void __iomem *iomem;
	size_t size;
	dma_addr_t iova;
};

enum { APPLE_RTKIT_STATE_INIT,
       APPLE_RTKIT_STATE_BOOTING,
       APPLE_RTKIT_STATE_RUNNING,
       APPLE_RTKIT_STATE_BOOT_FAILED,
};

struct apple_rtkit {
	void *cookie;
	const struct apple_rtkit_ops *ops;
	struct device *dev;
	void __iomem *regs;
	struct mbox_client mbox_cl;
	struct mbox_chan *mbox_chan;
	struct completion boot_completion;
	unsigned state;
	int boot_result;
	int version;

	struct wait_queue_head wq;
	DECLARE_KFIFO(msg_fifo, struct apple_mbox_msg, 64);

	DECLARE_BITMAP(endpoints, 0x100);

	struct apple_rtkit_shmem ioreport_buffer;
	struct apple_rtkit_shmem crashlog_buffer;

	struct apple_rtkit_shmem syslog_buffer;
	char *syslog_msg_buffer;
	size_t syslog_n_entries;
	size_t syslog_msg_size;
};

#define APPLE_RTKIT_CPU_CONTROL 0x44
#define APPLE_RTKIT_CPU_CONTROL_RUN BIT(4)

#define APPLE_RTKIT_EP_MGMT 0
#define APPLE_RTKIT_EP_CRASHLOG 1
#define APPLE_RTKIT_EP_SYSLOG 2
#define APPLE_RTKIT_EP_DEBUG 3
#define APPLE_RTKIT_EP_IOREPORT 4

#define APPLE_RTKIT_MGMT_WAKEUP 0x60000000000220

#define APPLE_RTKIT_MGMT_TYPE GENMASK(59, 52)

#define APPLE_RTKIT_MGMT_HELLO 1
#define APPLE_RTKIT_MGMT_HELLO_REPLY 2
#define APPLE_RTKIT_MGMT_HELLO_MINVER GENMASK(15, 0)
#define APPLE_RTKIT_MGMT_HELLO_MAXVER GENMASK(31, 16)

#define APPLE_RTKIT_MGMT_EPMAP 8
#define APPLE_RTKIT_MGMT_EPMAP_LAST BIT(51)
#define APPLE_RTKIT_MGMT_EPMAP_BASE GENMASK(34, 32)
#define APPLE_RTKIT_MGMT_EPMAP_BITMAP GENMASK(31, 0)

#define APPLE_RTKIT_MGMT_EPMAP_REPLY 8
#define APPLE_RTKIT_MGMT_EPMAP_REPLY_MORE BIT(0)

#define APPLE_RTKIT_MGMT_STARTEP 5
#define APPLE_RTKIT_MGMT_STARTEP_EP GENMASK(39, 32)
#define APPLE_RTKIT_MGMT_STARTEP_FLAG BIT(1)

#define APPLE_RTKIT_MGMT_BOOT_DONE 7
#define APPLE_RTKIT_MGMT_BOOT_DONE_UNK GENMASK(15, 0)

#define APPLE_RTKIT_MGMT_BOOT_DONE2 0xb

#define APPLE_RTKIT_CRASHLOG_CRASH 1

#define APPLE_RTKIT_BUFFER_REQUEST 1
#define APPLE_RTKIT_BUFFER_REQUEST_SIZE GENMASK(51, 44)
#define APPLE_RTKIT_BUFFER_REQUEST_IOVA GENMASK(41, 0)

#define APPLE_RTKIT_SYSLOG_TYPE GENMASK(59, 52)

#define APPLE_RTKIT_SYSLOG_LOG 5

#define APPLE_RTKIT_SYSLOG_INIT 8
#define APPLE_RTKIT_SYSLOG_N_ENTRIES GENMASK(7, 0)
#define APPLE_RTKIT_SYSLOG_MSG_SIZE GENMASK(31, 24)

#define APPLE_RTKIT_MIN_SUPPORTED_VERSION 11
#define APPLE_RTKIT_MAX_SUPPORTED_VERSION 12

static void apple_rtkit_boot_abort(struct apple_rtkit *rtk, int res)
{
	WARN_ON(rtk->state != APPLE_RTKIT_STATE_BOOTING);
	rtk->state = APPLE_RTKIT_STATE_BOOT_FAILED;
	rtk->boot_result = res;
	mbox_free_channel(rtk->mbox_chan);
	complete(&rtk->boot_completion);
}

static void apple_rtkit_management_send(struct apple_rtkit *rtk, u8 type,
					u64 msg)
{
	msg &= ~APPLE_RTKIT_MGMT_TYPE;
	msg |= FIELD_PREP(APPLE_RTKIT_MGMT_TYPE, type);
	apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_MGMT, msg);
}

static void apple_rtkit_management_rx_hello(struct apple_rtkit *rtk, u64 msg)
{
	u64 reply;

	int min_ver = FIELD_GET(APPLE_RTKIT_MGMT_HELLO_MINVER, msg);
	int max_ver = FIELD_GET(APPLE_RTKIT_MGMT_HELLO_MAXVER, msg);
	int want_ver = min(APPLE_RTKIT_MAX_SUPPORTED_VERSION, max_ver);

	rtk_dbg("Min ver %d, max ver %d\n", min_ver, max_ver);

	if (min_ver > APPLE_RTKIT_MAX_SUPPORTED_VERSION) {
		rtk_err("Firmware min version %d is too new\n", min_ver);
		apple_rtkit_boot_abort(rtk, -ENOTSUPP);
		return;
	}

	if (max_ver < APPLE_RTKIT_MIN_SUPPORTED_VERSION) {
		rtk_err("Firmware max version %d is too old\n", max_ver);
		apple_rtkit_boot_abort(rtk, -ENOTSUPP);
		return;
	}

	rtk_info("Initializing (protocol version %d)\n", want_ver);
	rtk->version = want_ver;

	reply = FIELD_PREP(APPLE_RTKIT_MGMT_HELLO_MINVER, want_ver) |
		FIELD_PREP(APPLE_RTKIT_MGMT_HELLO_MAXVER, want_ver);

	apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_HELLO_REPLY, reply);
}

static void apple_rtkit_management_rx_epmap(struct apple_rtkit *rtk, u64 msg)
{
	int i, ep;
	u64 reply;

	for (i = 0; i < 32; ++i) {
		u32 bitmap = FIELD_GET(APPLE_RTKIT_MGMT_EPMAP_BITMAP, msg);
		u32 base = FIELD_GET(APPLE_RTKIT_MGMT_EPMAP_BASE, msg);
		if (bitmap & BIT(i))
			set_bit(32 * base + i, rtk->endpoints);
	}

	reply = FIELD_PREP(APPLE_RTKIT_MGMT_EPMAP_BASE,
			   FIELD_GET(APPLE_RTKIT_MGMT_EPMAP_BASE, msg));
	if (msg & APPLE_RTKIT_MGMT_EPMAP_LAST)
		reply |= APPLE_RTKIT_MGMT_EPMAP_LAST;
	else
		reply |= APPLE_RTKIT_MGMT_EPMAP_REPLY_MORE;

	apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_EPMAP_REPLY, reply);

	if (msg & APPLE_RTKIT_MGMT_EPMAP_LAST) {
		for_each_set_bit (ep, rtk->endpoints, 0x100) {
			switch (ep) {
			/* the management endpoint is started by default */
			case APPLE_RTKIT_EP_MGMT:
				break;

			/*
                         * we need to start at least these system endpoints or
                         * RTKit refuses to boot
                         */
			case APPLE_RTKIT_EP_SYSLOG:
			case APPLE_RTKIT_EP_CRASHLOG:
			case APPLE_RTKIT_EP_DEBUG:
			case APPLE_RTKIT_EP_IOREPORT:
				apple_rtkit_start_ep(rtk, ep);
				break;

			/*
                         * everything above 0x20 is an app-specific endpoint
                         * which can be started later by the driver itself
                         */
			case 0x20 ... 0xff:
				break;

			default:
				rtk_warn("Unknown system ep: %d\n", ep);
			}
		}
	}
}

static void apple_rtkit_management_rx_boot_done(struct apple_rtkit *rtk,
						u64 msg)
{
	u64 reply;
	reply = FIELD_PREP(APPLE_RTKIT_MGMT_BOOT_DONE_UNK, 0x20);
	apple_rtkit_management_send(rtk, 0xb, reply);
}

static void apple_rtkit_management_rx_boot_done2(struct apple_rtkit *rtk,
						 u64 msg)
{
	rtk->state = APPLE_RTKIT_STATE_RUNNING;
	complete_all(&rtk->boot_completion);
	rtk_info("system endpoints successfuly initialized!");
}

static void apple_rtkit_management_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_MGMT_TYPE, msg);

	switch (type) {
	case APPLE_RTKIT_MGMT_HELLO:
		apple_rtkit_management_rx_hello(rtk, msg);
		break;
	case APPLE_RTKIT_MGMT_EPMAP:
		apple_rtkit_management_rx_epmap(rtk, msg);
		break;
	case APPLE_RTKIT_MGMT_BOOT_DONE:
		apple_rtkit_management_rx_boot_done(rtk, msg);
		break;
	case APPLE_RTKIT_MGMT_BOOT_DONE2:
		apple_rtkit_management_rx_boot_done2(rtk, msg);
		break;
	}
}

static void
apple_rtkit_common_rx_get_buffer_linux(struct apple_rtkit *rtk,
				       struct apple_rtkit_shmem *buffer, u8 ep,
				       u64 msg)
{
	size_t size = FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_SIZE, msg) << 12;
	dma_addr_t iova = FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_IOVA, msg);
	u64 reply;

	rtk_dbg("DMA buffer request for 0x%zx bytes\n", size);

	if (rtk->ops->shmem_alloc)
		buffer->buffer = rtk->ops->shmem_alloc(rtk->cookie, size, &iova,
						       GFP_KERNEL);
	else
		buffer->buffer =
			dma_alloc_coherent(rtk->dev, size, &iova, GFP_KERNEL);

	if (!buffer->buffer) {
		rtk_warn("couldn't allocate 0x%zx bytes.\n", size);
		apple_rtkit_boot_abort(rtk, -ENOMEM);
		return;
	}

	buffer->size = size;
	buffer->iova = iova;

	reply = FIELD_PREP(APPLE_RTKIT_SYSLOG_TYPE, APPLE_RTKIT_BUFFER_REQUEST);
	reply |=
		FIELD_PREP(APPLE_RTKIT_BUFFER_REQUEST_SIZE, buffer->size >> 12);
	reply |= FIELD_PREP(APPLE_RTKIT_BUFFER_REQUEST_IOVA, buffer->iova);
	apple_rtkit_send_message(rtk, ep, reply);
}

static void
apple_rtkit_common_rx_get_buffer_rtkit(struct apple_rtkit *rtk,
				       struct apple_rtkit_shmem *buffer, u8 ep,
				       u64 msg)
{
	size_t size = FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_SIZE, msg) << 12;
	dma_addr_t iova = FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_IOVA, msg);

	rtk_dbg("shmem buffer request for 0x%zx bytes at 0x%llx\n", size, iova);

	buffer->size = size;
	buffer->iova = iova;
	buffer->iomem = rtk->ops->shmem_map(rtk->cookie, iova, size);

	if (!buffer->iomem) {
		buffer->size = 0;
		buffer->iova = 0;
		rtk_warn("failed to map 0x%zx bytes at 0x%llx\n", size, iova);
		apple_rtkit_boot_abort(rtk, -ENOMEM);
	}
}

static void apple_rtkit_common_rx_get_buffer(struct apple_rtkit *rtk,
					     struct apple_rtkit_shmem *buffer,
					     u8 ep, u64 msg)
{
	if (rtk->ops->flags & APPLE_RTKIT_SHMEM_OWNER_RTKIT)
		return apple_rtkit_common_rx_get_buffer_rtkit(rtk, buffer, ep,
							      msg);
	else if (rtk->ops->flags & APPLE_RTKIT_SHMEM_OWNER_LINUX)
		return apple_rtkit_common_rx_get_buffer_linux(rtk, buffer, ep,
							      msg);
}

static void apple_rtkit_crashlog_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_SYSLOG_TYPE, msg);

	if (type != APPLE_RTKIT_CRASHLOG_CRASH) {
		rtk_warn("Unknown crashlog message: %llx\n", msg);
		return;
	}

	if (!rtk->crashlog_buffer.size)
		return apple_rtkit_common_rx_get_buffer(rtk,
							&rtk->crashlog_buffer,
							APPLE_RTKIT_EP_CRASHLOG,
							msg);

	rtk_err("co-processor has crashed.\n");
}

static void apple_rtkit_ioreport_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_SYSLOG_TYPE, msg);

	switch (type) {
	case APPLE_RTKIT_BUFFER_REQUEST:
		apple_rtkit_common_rx_get_buffer(rtk, &rtk->ioreport_buffer,
						 APPLE_RTKIT_EP_IOREPORT, msg);
		break;
	/* unknown, must be ACKed */
	case 0x8:
	case 0xc:
		apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_IOREPORT, msg);
		break;
	default:
		rtk_warn("Unknown ioreport message: %llx\n", msg);
	}
}

static void apple_rtkit_syslog_rx_init(struct apple_rtkit *rtk, u64 msg)
{
	rtk->syslog_n_entries = FIELD_GET(APPLE_RTKIT_SYSLOG_N_ENTRIES, msg);
	rtk->syslog_msg_size = FIELD_GET(APPLE_RTKIT_SYSLOG_MSG_SIZE, msg);

	rtk->syslog_msg_buffer =
		devm_kzalloc(rtk->dev, rtk->syslog_msg_size, GFP_KERNEL);

	rtk_dbg("syslog initialized: entries: %zd, msg_size: %zd\n",
		rtk->syslog_n_entries, rtk->syslog_msg_size);
}

static void apple_rtkit_memcpy(struct apple_rtkit *rtk, void *dst,
			       struct apple_rtkit_shmem *bfr, size_t offset,
			       size_t len)
{
	if (rtk->ops->flags & APPLE_RTKIT_SHMEM_OWNER_RTKIT)
		memcpy_fromio(dst, bfr->iomem + offset, len);
	else
		memcpy(dst, bfr->buffer + offset, len);
}

static void apple_rtkit_syslog_rx_log(struct apple_rtkit *rtk, u64 msg)
{
	u32 idx = msg & 0xff;
	char log_context[24];
	size_t entry_size = 0x20 + rtk->syslog_msg_size;

	if (!rtk->syslog_buffer.size) {
		rtk_warn(
			"received syslog message but syslog_buffer.size is zero");
		goto done;
	}
	if (rtk->ops->flags & APPLE_RTKIT_SHMEM_OWNER_LINUX &&
	    !rtk->syslog_buffer.buffer) {
		rtk_warn(
			"received syslog message but have no syslog_buffer.buffer");
		goto done;
	}
	if (rtk->ops->flags & APPLE_RTKIT_SHMEM_OWNER_RTKIT &&
	    !rtk->syslog_buffer.iomem) {
		rtk_warn(
			"received syslog message but have no syslog_buffer.iomem");
		goto done;
	}
	if (idx > rtk->syslog_n_entries) {
		rtk_warn("syslog index %d out of range", idx);
		goto done;
	}

	apple_rtkit_memcpy(rtk, log_context, &rtk->syslog_buffer,
			   idx * entry_size + 8, sizeof(log_context));
	apple_rtkit_memcpy(rtk, rtk->syslog_msg_buffer, &rtk->syslog_buffer,
			   idx * entry_size + 8 + sizeof(log_context),
			   rtk->syslog_msg_size);

	log_context[sizeof(log_context) - 1] = 0;
	rtk->syslog_msg_buffer[rtk->syslog_msg_size - 1] = 0;
	rtk_info("syslog message: %s: %s", log_context, rtk->syslog_msg_buffer);

done:
	apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_SYSLOG, msg);
}

static void apple_rtkit_syslog_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_SYSLOG_TYPE, msg);

	switch (type) {
	case APPLE_RTKIT_BUFFER_REQUEST:
		apple_rtkit_common_rx_get_buffer(rtk, &rtk->syslog_buffer,
						 APPLE_RTKIT_EP_SYSLOG, msg);
		break;
	case APPLE_RTKIT_SYSLOG_INIT:
		apple_rtkit_syslog_rx_init(rtk, msg);
		break;
	case APPLE_RTKIT_SYSLOG_LOG:
		apple_rtkit_syslog_rx_log(rtk, msg);
		break;
	default:
		rtk_warn("Unknown syslog message: %llx\n", msg);
	}
}

static void apple_rtkit_rx(struct apple_rtkit *rtk, struct apple_mbox_msg *msg)
{
	u8 ep = msg->msg1;

	switch (ep) {
	case APPLE_RTKIT_EP_MGMT:
		apple_rtkit_management_rx(rtk, msg->msg0);
		break;
	case APPLE_RTKIT_EP_CRASHLOG:
		apple_rtkit_crashlog_rx(rtk, msg->msg0);
		break;
	case APPLE_RTKIT_EP_SYSLOG:
		apple_rtkit_syslog_rx(rtk, msg->msg0);
		break;
	case APPLE_RTKIT_EP_IOREPORT:
		apple_rtkit_ioreport_rx(rtk, msg->msg0);
		break;
	case 0x20 ... 0xff:
		rtk->ops->recv_message(rtk->cookie, ep, msg->msg0);
		break;
	default:
		rtk_warn("message to unknown endpoint %02x: %llx\n", ep,
			 msg->msg0);
	}
}

static int apple_rtkit_worker(void *data)
{
	struct apple_rtkit *rtk = data;
	struct apple_mbox_msg m;

	while (true) {
		wait_event_interruptible(rtk->wq,
					 kfifo_len(&rtk->msg_fifo) > 0);

		if (kfifo_out(&rtk->msg_fifo, &m, 1) == 1)
			apple_rtkit_rx(rtk, &m);
	}

	do_exit(0);
}

static void apple_rtkit_rx_callback(struct mbox_client *cl, void *mssg)
{
	struct apple_rtkit *rtk = container_of(cl, struct apple_rtkit, mbox_cl);
	struct apple_mbox_msg *msg = mssg;
	u8 ep = msg->msg1;

	dma_rmb();

	if (ep >= 0x20 && (rtk->ops->flags & APPLE_RTKIT_RECV_ATOMIC)) {
		rtk->ops->recv_message(rtk->cookie, ep, msg->msg0);
	} else {
		kfifo_in(&rtk->msg_fifo, msg, 1);
		wake_up(&rtk->wq);
	}
}

int apple_rtkit_send_message(struct apple_rtkit *rtk, u8 ep, u64 message)
{
	struct apple_mbox_msg msg;
	int ret;

	if (WARN_ON(ep >= 0x20 && rtk->state != APPLE_RTKIT_STATE_RUNNING))
		return -EINVAL;

	msg.msg0 = (u64)message;
	msg.msg1 = ep;
	dma_wmb();
	ret = mbox_send_message(rtk->mbox_chan, &msg);
	if (ret < 0)
		return ret;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_send_message);

int apple_rtkit_start_ep(struct apple_rtkit *rtk, u8 endpoint)
{
	u64 msg;

	if (WARN_ON(!test_bit(endpoint, rtk->endpoints)))
		return -EINVAL;
	if (WARN_ON(endpoint >= 0x20 &&
		    rtk->state != APPLE_RTKIT_STATE_RUNNING))
		return -EINVAL;

	msg = FIELD_PREP(APPLE_RTKIT_MGMT_STARTEP_EP, endpoint);
	msg |= APPLE_RTKIT_MGMT_STARTEP_FLAG;
	apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_STARTEP, msg);

	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_start_ep);

struct apple_rtkit *apple_rtkit_init(struct device *dev, void *cookie,
				     struct resource *res,
				     const char *mbox_name,
				     int mbox_idx,
				     const struct apple_rtkit_ops *ops)
{
	struct apple_rtkit *rtk;

	if (!ops)
		return ERR_PTR(-EINVAL);

	if (!(ops->flags & APPLE_RTKIT_SHMEM_OWNER_LINUX) &&
	    !(ops->flags & APPLE_RTKIT_SHMEM_OWNER_RTKIT))
		return ERR_PTR(-EINVAL);

	rtk = devm_kzalloc(dev, sizeof(*rtk), GFP_KERNEL);
	if (!rtk)
		return ERR_PTR(-ENOMEM);

	rtk->dev = dev;
	rtk->state = APPLE_RTKIT_STATE_INIT;
	rtk->cookie = cookie;
	rtk->ops = ops;
	rtk->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(rtk->regs))
		return rtk->regs;

	INIT_KFIFO(rtk->msg_fifo);
	init_waitqueue_head(&rtk->wq);
	init_completion(&rtk->boot_completion);
	// TODO: add a way to stop this thread
	kthread_run(apple_rtkit_worker, rtk, "%s-rtkit-worker", dev_name(dev));

	rtk->mbox_cl.dev = dev;
	rtk->mbox_cl.tx_block = true;
	rtk->mbox_cl.knows_txdone = false;
	rtk->mbox_cl.rx_callback = &apple_rtkit_rx_callback;

	if (mbox_name)
		rtk->mbox_chan = mbox_request_channel_byname(&rtk->mbox_cl, mbox_name);
	else
		rtk->mbox_chan = mbox_request_channel(&rtk->mbox_cl, mbox_idx);

	if (IS_ERR(rtk->mbox_chan))
		return (struct apple_rtkit *)rtk->mbox_chan;


	return rtk;
}
EXPORT_SYMBOL_GPL(apple_rtkit_init);

int apple_rtkit_boot(struct apple_rtkit *rtk)
{
	int ret;
	u32 cpu_ctrl;

	if (rtk->state != APPLE_RTKIT_STATE_INIT)
		return 0;
	rtk->state = APPLE_RTKIT_STATE_BOOTING;

	cpu_ctrl = readl_relaxed(rtk->regs + APPLE_RTKIT_CPU_CONTROL);
	if (cpu_ctrl & APPLE_RTKIT_CPU_CONTROL_RUN) {
		rtk_dbg("sending wakeup message\n");
		ret = apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_MGMT,
					       APPLE_RTKIT_MGMT_WAKEUP);
	} else {
		rtk_dbg("starting CPU\n");
		cpu_ctrl |= APPLE_RTKIT_CPU_CONTROL_RUN;
		writel_relaxed(cpu_ctrl, rtk->regs + APPLE_RTKIT_CPU_CONTROL);
		ret = 0;
	}

	if (ret)
		apple_rtkit_boot_abort(rtk, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_boot);

int apple_rtkit_boot_wait(struct apple_rtkit *rtk, unsigned long timeout)
{
	int ret;
	long t;

	switch (rtk->state)
	{
	case APPLE_RTKIT_STATE_BOOTING:
	case APPLE_RTKIT_STATE_INIT:
		break;
	case APPLE_RTKIT_STATE_RUNNING:
		return 0;
		break;
	default:
		return -EINVAL;
		break;
	}

	ret = apple_rtkit_boot(rtk);
	if (ret)
		return ret;

	rtk_dbg("waiting for boot\n");


	t = wait_for_completion_interruptible_timeout(&rtk->boot_completion, timeout);
	if (t == -ERESTARTSYS)
		return t;
	else if (t == 0)
		return -ETIME;

	if (rtk->state == APPLE_RTKIT_STATE_RUNNING)
		return 0;
	return rtk->boot_result;
}
EXPORT_SYMBOL_GPL(apple_rtkit_boot_wait);
