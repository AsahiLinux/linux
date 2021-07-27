#ifndef APPLE_ASC_MAILBOX
#define APPLE_ASC_MAILBOX 1

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>

/* A2I = Application Processor (us) to I/O Processor (usually RTKit) */
#define APPLE_IOP_A2I_CONTROL 0x110
#define APPLE_IOP_A2I_CONTROL_FULL BIT(16)
#define APPLE_IOP_A2I_CONTROL_EMPTY BIT(17)

#define APPLE_IOP_I2A_CONTROL 0x114
#define APPLE_IOP_I2A_CONTROL_FULL BIT(16)
#define APPLE_IOP_I2A_CONTROL_EMPTY BIT(17)

#define APPLE_IOP_A2I_MBOX_DATA 0x800
#define APPLE_IOP_A2I_MBOX_INFO 0x808
#define APPLE_IOP_I2A_MBOX_DATA 0x830
#define APPLE_IOP_I2A_MBOX_INFO 0x838

/* max channels to save memory; IPC protocol supports up to 0x100 chans */
#define APPLE_IOP_MAX_ACTIVE_CHANS 20
#define APPLE_IOP_MAX_ENDPOINTS 0x100

struct apple_mbox;

struct apple_chan_priv {
	u8 endpoint;
	bool needs_irq;
	bool enabled;
	struct apple_mbox *apple_mbox;
};

struct apple_mbox_msg {
	u64 msg;
	u64 info;
};

struct apple_mbox {
	void __iomem *regs, *sart_regs;
	struct resource *mmio_shmem;
	int irq_can_send, irq_can_recv;

	bool allow_can_send_irq_enable;

	struct clk_bulk_data *clks;
	int num_clks;

	spinlock_t lock;

	s8 epmap[APPLE_IOP_MAX_ENDPOINTS];
	struct mbox_chan chans[APPLE_IOP_MAX_ACTIVE_CHANS];

	DECLARE_BITMAP(rtkit_endpoints, APPLE_IOP_MAX_ENDPOINTS);

	DECLARE_KFIFO(recv_fifo, struct apple_mbox_msg, 16);
	bool recv_full;

	struct device *dev;
	struct mbox_controller controller;
};

#endif