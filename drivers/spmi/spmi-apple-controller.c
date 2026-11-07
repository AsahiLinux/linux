// SPDX-License-Identifier: GPL-2.0
/*
 * Apple SoC SPMI device driver
 *
 * Copyright The Asahi Linux Contributors
 *
 * Inspired by:
 *		OpenBSD support Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
 *		Correllium support Copyright (C) 2021 Corellium LLC
 *		hisi-spmi-controller.c
 *		spmi-pmic-arb.c Copyright (c) 2021, The Linux Foundation.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/spmi.h>

/* SPMI Controller Registers */
#define SPMI_STATUS_REG 0
#define SPMI_CMD_REG 0x4
#define SPMI_RSP_REG 0x8
#define SPMI_ACT_REG 0xa4

#define SPMI_IRQ_MASK_BASE 0x20
#define SPMI_IRQ_ACK_BASE 0x60
#define SPMI_NUM_PERIPHERAL_IRQS 256
#define SPMI_NUM_IRQS (SPMI_NUM_PERIPHERAL_IRQS + 32)

#define SPMI_IRQ_NOTIFY 256

/* SPMI_RSP_REG reply word */
#define SPMI_REPLY_FRAME_PARITY_STATUS GENMASK(31, 16)
#define SPMI_REPLY_ACK BIT(15)
#define SPMI_REPLY_SLAVE_ID GENMASK(14, 8)
#define SPMI_REPLY_CMD GENMASK(7, 0)

#define SPMI_ACT_FIFO_FLUSH BIT(0)
#define SPMI_RX_FIFO_EMPTY BIT(24)

#define REG_POLL_INTERVAL_US 10000
#define REG_POLL_TIMEOUT_US (REG_POLL_INTERVAL_US * 5)

struct apple_spmi {
	void __iomem *regs;
	struct mutex fifo_lock;
	struct completion fifo_rx;
	struct irq_domain *irqd;
	raw_spinlock_t irq_mask_lock;
	DECLARE_BITMAP(irq_mask_cache, SPMI_NUM_PERIPHERAL_IRQS);
	int irq;
	bool notify_irq;
	bool prev_fail;
};

#define poll_reg(spmi, reg, val, cond) \
	readl_poll_timeout((spmi)->regs + (reg), (val), (cond), \
			   REG_POLL_INTERVAL_US, REG_POLL_TIMEOUT_US)

static void apple_spmi_irq_ack_raw(struct apple_spmi *spmi, u32 irq)
{
	u32 __iomem *reg = spmi->regs + SPMI_IRQ_ACK_BASE + (irq / 32) * 4;

	writel(BIT(irq % 32), reg);
}

static void apple_spmi_irq_mask_raw(struct apple_spmi *spmi, u32 irq)
{
	u32 __iomem *reg = spmi->regs + SPMI_IRQ_MASK_BASE + (irq / 32) * 4;

	writel(readl(reg) & ~BIT(irq % 32), reg);
}

static void apple_spmi_irq_unmask_raw(struct apple_spmi *spmi, u32 irq)
{
	u32 __iomem *reg = spmi->regs + SPMI_IRQ_MASK_BASE + (irq / 32) * 4;

	writel(readl(reg) | BIT(irq % 32), reg);
}

static void apple_spmi_irq_ack(struct irq_data *d)
{
	struct apple_spmi *spmi = irq_data_get_irq_chip_data(d);

	apple_spmi_irq_ack_raw(spmi, d->hwirq);
}

static void apple_spmi_irq_mask(struct irq_data *d)
{
	struct apple_spmi *spmi = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&spmi->irq_mask_lock, flags);
	apple_spmi_irq_mask_raw(spmi, d->hwirq);
	clear_bit(d->hwirq, spmi->irq_mask_cache);
	raw_spin_unlock_irqrestore(&spmi->irq_mask_lock, flags);
}

static void apple_spmi_irq_unmask(struct irq_data *d)
{
	struct apple_spmi *spmi = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&spmi->irq_mask_lock, flags);
	set_bit(d->hwirq, spmi->irq_mask_cache);
	apple_spmi_irq_unmask_raw(spmi, d->hwirq);
	raw_spin_unlock_irqrestore(&spmi->irq_mask_lock, flags);
}

static inline u32 apple_spmi_pack_cmd(u8 opc, u8 sid, u16 param)
{
	return opc | sid << 8 | (u32)param << 16 | (1 << 15);
}

/* Wait for Rx FIFO to have something */
static int apple_spmi_wait_rx_not_empty(struct spmi_controller *ctrl)
{
	struct apple_spmi *spmi = spmi_controller_get_drvdata(ctrl);
	int ret;
	u32 status;

	if (spmi->notify_irq) {
		ret = wait_for_completion_timeout(&spmi->fifo_rx,
			usecs_to_jiffies(REG_POLL_TIMEOUT_US));
		if (!ret)
			ret = -ETIMEDOUT;
		else if (readl(spmi->regs + SPMI_STATUS_REG) & SPMI_RX_FIFO_EMPTY)
			ret = -EIO;
		else
			ret = 0;
	} else {
		ret = poll_reg(spmi, SPMI_STATUS_REG, status, !(status & SPMI_RX_FIFO_EMPTY));
	}

	if (ret) {
		spmi->prev_fail = true;
		dev_err(&ctrl->dev,
			"failed to wait for RX FIFO not empty\n");
		return ret;
	}

	return 0;
}

static int spmi_raw_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid, u16 param,
			const u8 *buf_wr, size_t len_wr, u8 *buf_rd, size_t len_rd)
{
	struct apple_spmi *spmi = spmi_controller_get_drvdata(ctrl);
	u32 spmi_cmd = apple_spmi_pack_cmd(opc, sid, param);
	u32 reply, rsp;
	size_t i = 0, j;
	int ret;

	guard(mutex)(&spmi->fifo_lock);

	if (spmi->prev_fail) {
		writel(SPMI_ACT_FIFO_FLUSH, spmi->regs + SPMI_ACT_REG);
		apple_spmi_irq_ack_raw(spmi, SPMI_IRQ_NOTIFY);
		spmi->prev_fail = false;
	}
	reinit_completion(&spmi->fifo_rx);

	writel(spmi_cmd, spmi->regs + SPMI_CMD_REG);

	while (i < len_wr) {
		j = min_t(size_t, sizeof(spmi_cmd), len_wr - i);
		spmi_cmd = 0;
		memcpy(&spmi_cmd, buf_wr + i, j);
		writel(spmi_cmd, spmi->regs + SPMI_CMD_REG);
		i += j;
	}

	ret = apple_spmi_wait_rx_not_empty(ctrl);
	if (ret)
		return ret;

	reply = readl(spmi->regs + SPMI_RSP_REG);

	/* Read SPMI data reply */
	i = 0;
	while (i < len_rd) {
		if (readl(spmi->regs + SPMI_STATUS_REG) & SPMI_RX_FIFO_EMPTY) {
			spmi->prev_fail = true;
			dev_err_ratelimited(&ctrl->dev,
					    "FIFO lacks reply data, controller stuck?\n");
			return -EIO;
		}
		rsp = readl(spmi->regs + SPMI_RSP_REG);
		j = min_t(size_t, sizeof(spmi_cmd), len_rd - i);
		memcpy(buf_rd + i, &rsp, j);
		i += j;
	}

	if (!(readl(spmi->regs + SPMI_STATUS_REG) & SPMI_RX_FIFO_EMPTY)) {
		dev_warn(&ctrl->dev, "FIFO has extra data\n");
		spmi->prev_fail = true;
	}

	if (!len_rd && !FIELD_GET(SPMI_REPLY_ACK, reply)) {
		dev_err(&ctrl->dev, "command not acknowledged\n");
		return -EIO;
	}
	if (~FIELD_GET(SPMI_REPLY_FRAME_PARITY_STATUS, reply) & ((1 << len_rd) - 1)) {
		dev_err(&ctrl->dev, "some frames failed parity check\n");
		return -EIO;
	}
	return 0;
}

/* Send a raw command with 1..16 input data frames */
static int spmi_raw_cmd_input(struct spmi_controller *ctrl, u8 opc, u8 sid,
			 u16 param, u8 *buf, size_t len)
{
	return spmi_raw_cmd(ctrl, opc, sid, param, NULL, 0, buf, len);
}

/* Send a raw command with (optional) body and an input ACK */
static int spmi_raw_cmd_ack(struct spmi_controller *ctrl, u8 opc, u8 sid,
			  u16 param, const u8 *buf, size_t len)
{
	return spmi_raw_cmd(ctrl, opc, sid, param, buf, len, NULL, 0);
}

static int spmi_read_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid,
			 u16 saddr, u8 *buf, size_t len)
{
	switch (opc) {
	case SPMI_CMD_EXT_READ:
	case SPMI_CMD_EXT_READL:
		return spmi_raw_cmd_input(ctrl, opc | (len - 1), sid, saddr, buf, len);
	case SPMI_CMD_READ:
		return spmi_raw_cmd_input(ctrl, opc | saddr, sid, saddr, buf, len);
	}
	return -EINVAL;
}

static int spmi_write_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid,
			  u16 saddr, const u8 *buf, size_t len)
{
	switch (opc) {
	case SPMI_CMD_WRITE:
		return spmi_raw_cmd_ack(ctrl, opc | saddr, sid, buf[0] << 8 | saddr, NULL, 0);
	case SPMI_CMD_ZERO_WRITE:
		return spmi_raw_cmd_ack(ctrl, opc | buf[0], sid, buf[0] << 8 | saddr, NULL, 0);
	case SPMI_CMD_EXT_WRITE:
	case SPMI_CMD_EXT_WRITEL:
		return spmi_raw_cmd_ack(ctrl, opc | (len - 1), sid, saddr, buf, len);
	}
	return -EINVAL;
}

static int spmi_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid)
{
	switch (opc) {
	case SPMI_CMD_RESET:
	case SPMI_CMD_SLEEP:
	case SPMI_CMD_SHUTDOWN:
	case SPMI_CMD_WAKEUP:
		return spmi_raw_cmd_ack(ctrl, opc, sid, 0, NULL, 0);
	}
	return -EINVAL;
}

static int apple_spmi_irq_set_type(struct irq_data *d, unsigned int type)
{
	/* all interrupts have MSI semantics */
	return type == IRQ_TYPE_EDGE_RISING ? 0 : -EINVAL;
}

static struct irq_chip apple_spmi_irq_chip = {
	.name = "apple_spmi",
	.irq_mask = apple_spmi_irq_mask,
	.irq_unmask = apple_spmi_irq_unmask,
	.irq_ack = apple_spmi_irq_ack,
	.irq_set_type = apple_spmi_irq_set_type,
	.flags = IRQCHIP_ONESHOT_SAFE,
};

static int apple_spmi_irq_domain_map(struct irq_domain *irqd,
					unsigned int irq, irq_hw_number_t hw)
{
	irq_domain_set_info(irqd, irq, hw, &apple_spmi_irq_chip, irqd->host_data,
				handle_edge_irq, NULL, NULL);
	return 0;
}

static int apple_spmi_irq_domain_translate(struct irq_domain *irqd,
					struct irq_fwspec *fwspec,
					unsigned long *hwirq,
					unsigned int *type)
{
	u32 *args = fwspec->param;

	if (fwspec->param_count != 2)
		return -EINVAL;

	if (args[0] >= SPMI_NUM_PERIPHERAL_IRQS)
		return -EINVAL;
	*hwirq = args[0];
	*type = args[1] & IRQ_TYPE_SENSE_MASK;
	return 0;
}

static int apple_spmi_irq_domain_alloc(struct irq_domain *irqd, unsigned int virq,
				unsigned int nr_irqs, void *arg)
{
	unsigned int type = IRQ_TYPE_NONE;
	struct irq_fwspec *fwspec = arg;
	irq_hw_number_t hwirq;
	int i, ret;

	ret = apple_spmi_irq_domain_translate(irqd, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	if (hwirq + nr_irqs > SPMI_NUM_PERIPHERAL_IRQS)
		return -EINVAL;

	for (i = 0; i < nr_irqs; i++) {
		ret = apple_spmi_irq_domain_map(irqd, virq + i, hwirq + i);
		if (ret)
			return ret;
	}

	return 0;
}

static void apple_spmi_irq_domain_free(struct irq_domain *irqd, unsigned int virq,
				unsigned int nr_irqs)
{
	int i;

	for (i = 0; i < nr_irqs; i++) {
		struct irq_data *d = irq_domain_get_irq_data(irqd, virq + i);

		irq_set_handler(virq + i, NULL);
		irq_domain_reset_irq_data(d);
	}
}

static const struct irq_domain_ops apple_spmi_irq_domain_ops = {
	.translate	= apple_spmi_irq_domain_translate,
	.alloc		= apple_spmi_irq_domain_alloc,
	.free		= apple_spmi_irq_domain_free,
};

static void apple_spmi_irq_handler(struct irq_desc *desc)
{
	struct apple_spmi *spmi = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	bool handled = false;
	unsigned long val, offset, bit;

	chained_irq_enter(chip, desc);
	val = readl(spmi->regs + SPMI_IRQ_ACK_BASE + (SPMI_IRQ_NOTIFY / 32) * 4);
	if (val & BIT(SPMI_IRQ_NOTIFY % 32)) {
		apple_spmi_irq_ack_raw(spmi, SPMI_IRQ_NOTIFY);
		complete(&spmi->fifo_rx);
		handled = true;
	}

	for (offset = 0; offset < SPMI_NUM_PERIPHERAL_IRQS / 8; offset += sizeof(val)) {
		val = readq(spmi->regs + SPMI_IRQ_ACK_BASE + offset);
		/**
		 * because of other masters in the bus, we're going to get a multitude of
		 * interrupts we're not interested in. irq_resolve_mapping isn't very
		 * optimized for the nonexistent path, so instead we mask with (a locally
		 * cached version of) the IRQ mask
		 */
		val &= spmi->irq_mask_cache[offset / sizeof(val)];
		for_each_set_bit(bit, &val, 64) {
			generic_handle_domain_irq(spmi->irqd, offset * 8 + bit);
			handled = true;
		}
	}
	if (!handled)
		handle_bad_irq(desc);
	chained_irq_exit(chip, desc);
}

static void apple_spmi_teardown_irq(void *data)
{
	struct apple_spmi *spmi = data;

	for (size_t offset = 0; offset < SPMI_NUM_IRQS / 8; offset += 4)
		writel(0, spmi->regs + SPMI_IRQ_MASK_BASE + offset);

	synchronize_irq(spmi->irq);
	irq_set_chained_handler_and_data(spmi->irq, NULL, NULL);
}

static int apple_spmi_init_irq(struct platform_device *pdev,
			       struct apple_spmi *spmi, int irq)
{
	int ret;
	struct irq_domain_info info = {
		.fwnode		= pdev->dev.fwnode,
		.hwirq_max	= ~0U,
		.ops		= &apple_spmi_irq_domain_ops,
		.host_data	= spmi,
	};

	raw_spin_lock_init(&spmi->irq_mask_lock);

	for (size_t offset = 0; offset < SPMI_NUM_IRQS / 8; offset += 4) {
		writel(0, spmi->regs + SPMI_IRQ_MASK_BASE + offset);
		writel(U32_MAX, spmi->regs + SPMI_IRQ_ACK_BASE + offset);
	}

	spmi->irqd = devm_irq_domain_instantiate(&pdev->dev, &info);
	if (IS_ERR(spmi->irqd))
		return PTR_ERR(spmi->irqd);

	ret = devm_add_action(&pdev->dev, apple_spmi_teardown_irq, spmi);
	if (ret)
		return ret;

	irq_set_chained_handler_and_data(spmi->irq, apple_spmi_irq_handler, spmi);
	apple_spmi_irq_unmask_raw(spmi, SPMI_IRQ_NOTIFY);
	spmi->notify_irq = true;

	return 0;
}

static int apple_spmi_probe(struct platform_device *pdev)
{
	struct apple_spmi *spmi;
	struct spmi_controller *ctrl;
	int ret;

	ctrl = devm_spmi_controller_alloc(&pdev->dev, sizeof(*spmi));
	if (IS_ERR(ctrl))
		return -ENOMEM;

	spmi = spmi_controller_get_drvdata(ctrl);
	mutex_init(&spmi->fifo_lock);
	init_completion(&spmi->fifo_rx);
	platform_set_drvdata(pdev, spmi);

	spmi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spmi->regs))
		return PTR_ERR(spmi->regs);

	ctrl->dev.of_node = pdev->dev.of_node;

	ctrl->read_cmd = spmi_read_cmd;
	ctrl->write_cmd = spmi_write_cmd;
	ctrl->cmd = spmi_cmd;

	spmi->irq = platform_get_irq_optional(pdev, 0);
	if (spmi->irq < 0 && spmi->irq != -ENXIO)
		return spmi->irq;
	if (spmi->irq >= 0) {
		ret = apple_spmi_init_irq(pdev, spmi, spmi->irq);
		if (ret)
			return ret;
	}

	ret = devm_spmi_controller_add(&pdev->dev, ctrl);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "spmi_controller_add failed\n");

	return 0;
}

static const struct of_device_id apple_spmi_match_table[] = {
	{ .compatible = "apple,t8103-spmi", },
	{ .compatible = "apple,spmi", },
	{}
};
MODULE_DEVICE_TABLE(of, apple_spmi_match_table);

static struct platform_driver apple_spmi_driver = {
	.probe		= apple_spmi_probe,
	.driver		= {
		.name	= "apple-spmi",
		.of_match_table = apple_spmi_match_table,
	},
};
module_platform_driver(apple_spmi_driver);

MODULE_AUTHOR("Jean-Francois Bortolotti <jeff@borto.fr>");
MODULE_DESCRIPTION("Apple SoC SPMI driver");
MODULE_LICENSE("GPL");
