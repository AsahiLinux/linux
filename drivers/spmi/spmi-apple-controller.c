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
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spmi.h>

/* SPMI Controller Registers */
#define SPMI_STATUS_REG 0
#define SPMI_CMD_REG 0x4
#define SPMI_RSP_REG 0x8
#define SPMI_ACT_REG 0xa4

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
	bool prev_fail;
};

#define poll_reg(spmi, reg, val, cond) \
	readl_poll_timeout((spmi)->regs + (reg), (val), (cond), \
			   REG_POLL_INTERVAL_US, REG_POLL_TIMEOUT_US)

static inline u32 apple_spmi_pack_cmd(u8 opc, u8 sid, u16 saddr, size_t len)
{
	return opc | sid << 8 | saddr << 16 | (len - 1) | (1 << 15);
}

/* Wait for Rx FIFO to have something */
static int apple_spmi_wait_rx_not_empty(struct spmi_controller *ctrl)
{
	struct apple_spmi *spmi = spmi_controller_get_drvdata(ctrl);
	int ret;
	u32 status;

	ret = poll_reg(spmi, SPMI_STATUS_REG, status, !(status & SPMI_RX_FIFO_EMPTY));
	if (ret) {
		spmi->prev_fail = true;
		dev_err(&ctrl->dev,
			"failed to wait for RX FIFO not empty\n");
		return ret;
	}

	return 0;
}

static int spmi_read_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid,
			 u16 saddr, u8 *buf, size_t len)
{
	struct apple_spmi *spmi = spmi_controller_get_drvdata(ctrl);
	u32 spmi_cmd = apple_spmi_pack_cmd(opc, sid, saddr, len);
	u32 reply, rsp;
	size_t len_read = 0;
	u8 i;
	int ret;

	if (spmi->prev_fail) {
		writel(SPMI_ACT_FIFO_FLUSH, spmi->regs + SPMI_ACT_REG);
		spmi->prev_fail = false;
	}

	writel(spmi_cmd, spmi->regs + SPMI_CMD_REG);

	ret = apple_spmi_wait_rx_not_empty(ctrl);
	if (ret)
		return ret;

	reply = readl(spmi->regs + SPMI_RSP_REG);

	/* Read SPMI data reply */
	while (len_read < len) {
		if (readl(spmi->regs + SPMI_STATUS_REG) & SPMI_RX_FIFO_EMPTY) {
			spmi->prev_fail = true;
			dev_err_ratelimited(&ctrl->dev,
					    "FIFO lacks reply data, controller stuck?\n");
			return -EIO;
		}
		rsp = readl(spmi->regs + SPMI_RSP_REG);
		i = 0;
		while ((len_read < len) && (i < 4)) {
			buf[len_read++] = ((0xff << (8 * i)) & rsp) >> (8 * i);
			i += 1;
		}
	}

	if (!(readl(spmi->regs + SPMI_STATUS_REG) & SPMI_RX_FIFO_EMPTY)) {
		dev_warn(&ctrl->dev, "FIFO has extra data\n");
		spmi->prev_fail = true;
	}

	if (~FIELD_GET(SPMI_REPLY_FRAME_PARITY_STATUS, reply) & ((1 << len) - 1)) {
		dev_err(&ctrl->dev, "some frames failed parity check\n");
		return -EIO;
	}
	return 0;
}

static int spmi_write_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid,
			  u16 saddr, const u8 *buf, size_t len)
{
	struct apple_spmi *spmi = spmi_controller_get_drvdata(ctrl);
	u32 spmi_cmd = apple_spmi_pack_cmd(opc, sid, saddr, len);
	u32 reply;
	size_t i = 0, j;
	int ret;

	if (spmi->prev_fail) {
		writel(SPMI_ACT_FIFO_FLUSH, spmi->regs + SPMI_ACT_REG);
		spmi->prev_fail = false;
	}

	writel(spmi_cmd, spmi->regs + SPMI_CMD_REG);

	while (i < len) {
		j = 0;
		spmi_cmd = 0;
		while ((j < 4) & (i < len))
			spmi_cmd |= buf[i++] << (j++ * 8);

		writel(spmi_cmd, spmi->regs + SPMI_CMD_REG);
	}

	ret = apple_spmi_wait_rx_not_empty(ctrl);
	if (ret)
		return ret;

	reply = readl(spmi->regs + SPMI_RSP_REG);

	if (!(readl(spmi->regs + SPMI_STATUS_REG) & SPMI_RX_FIFO_EMPTY)) {
		dev_warn(&ctrl->dev, "FIFO has extra data\n");
		spmi->prev_fail = true;
	}

	if (!FIELD_GET(SPMI_REPLY_ACK, reply)) {
		dev_err(&ctrl->dev, "command not acknowledged\n");
		return -EIO;
	}
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

	spmi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spmi->regs))
		return PTR_ERR(spmi->regs);

	ctrl->dev.of_node = pdev->dev.of_node;

	ctrl->read_cmd = spmi_read_cmd;
	ctrl->write_cmd = spmi_write_cmd;

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
