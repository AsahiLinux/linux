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
 *		spmi-pmic-ard.c Copyright (c) 2021, The Linux Foundation.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spmi.h>

/* SPMI Controller Registers */
#define SPMI_STATUS_REG 0
#define SPMI_CMD_REG 0x4
#define SPMI_RSP_REG 0x8

#define SPMI_RX_FIFO_EMPTY BIT(24)
#define SPMI_TX_FIFO_EMPTY BIT(8)

/* Apple SPMI controler */
struct apple_spmi {
	void __iomem *regs;
	struct spmi_controller *ctrl;
};

static inline u32 read_reg(struct apple_spmi *spmi, int offset)
{
	return (readl_relaxed(spmi->regs + offset));
}

static inline void write_reg(u32 value, struct apple_spmi *spmi, int offset)
{
	writel_relaxed(value, spmi->regs + offset);
}

static int spmi_read_cmd(struct spmi_controller *ctrl, u8 opc, u8 slave_id,
			 u16 slave_addr, u8 *__buf, size_t bc)
{
	struct apple_spmi *spmi;
	u32 spmi_cmd = opc | slave_id << 8 | slave_addr << 16 | (bc - 1) |
		       (1 << 15);
	u32 rsp;
	volatile u32 status;
	size_t len_to_read;
	u8 i;

	spmi = spmi_controller_get_drvdata(ctrl);

	write_reg(spmi_cmd, spmi, SPMI_CMD_REG);

	/* Wait for Rx FIFO to have something */
	/* Quite ugly msleep, need to find a better way to do it */
	i = 0;
	do {
		status = read_reg(spmi, SPMI_STATUS_REG);
		msleep(10);
		i += 1;
	} while ((status & SPMI_RX_FIFO_EMPTY) && i < 5);

	if (i >= 5) {
		dev_err(&ctrl->dev,
			"spmi_read_cmd:took to long to get the status");
		return -1;
	}

	/* Read SPMI reply status */
	rsp = read_reg(spmi, SPMI_RSP_REG);

	len_to_read = 0;
	/* Read SPMI data reply */
	while (!(status & SPMI_RX_FIFO_EMPTY) && (len_to_read < bc)) {
		rsp = read_reg(spmi, SPMI_RSP_REG);
		i = 0;
		while ((len_to_read < bc) && (i < 4)) {
			__buf[len_to_read++] = ((0xff << (8 * i)) & rsp) >>
					       (8 * i);
			i += 1;
		}
	}

	return 0;
}

static int spmi_write_cmd(struct spmi_controller *ctrl, u8 opc, u8 slave_id,
			  u16 slave_addr, const u8 *__buf, size_t bc)
{
	struct apple_spmi *spmi;
	u32 spmi_cmd = opc | slave_id << 8 | slave_addr << 16 | (bc - 1) |
		       (1 << 15);
	volatile u32 rsp;
	volatile u32 status;
	size_t i = 0, j;

	spmi = spmi_controller_get_drvdata(ctrl);

	write_reg(spmi_cmd, spmi, SPMI_CMD_REG);

	while (i < bc) {
		j = 0;
		spmi_cmd = 0;
		while ((j < 4) & (i < bc)) {
			spmi_cmd |= __buf[i++] << (j++ * 8);
		}
		write_reg(spmi_cmd, spmi, SPMI_CMD_REG);
	}

	/* Wait for Rx FIFO to have something */
	/* Quite ugly msleep, need to find a better way to do it */
	i = 0;
	do {
		status = read_reg(spmi, SPMI_STATUS_REG);
		msleep(10);
		i += 1;
	} while ((status & SPMI_RX_FIFO_EMPTY) && i < 5);

	if (i >= 5) {
		dev_err(&ctrl->dev,
			"spmi_write_cmd:took to long to get the status");
		return -1;
	}

	rsp = read_reg(spmi, SPMI_RSP_REG);
	(void)rsp; // TODO: check stuff here

	return 0;
}

static int spmi_controller_probe(struct platform_device *pdev)
{
	struct apple_spmi *spmi;
	struct spmi_controller *ctrl;
	int ret;

	ctrl = spmi_controller_alloc(&pdev->dev, sizeof(struct apple_spmi));
	if (IS_ERR(ctrl)) {
		dev_err_probe(&pdev->dev, PTR_ERR(ctrl),
			      "Can't allocate spmi_controller data\n");
		return -ENOMEM;
	}

	spmi = spmi_controller_get_drvdata(ctrl);
	spmi->ctrl = ctrl;
	platform_set_drvdata(pdev, ctrl);

	spmi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spmi->regs)) {
		dev_err_probe(&pdev->dev, PTR_ERR(spmi->regs),
			      "Can't get ioremap regs.\n");
		return PTR_ERR(spmi->regs);
	}

	ctrl->dev.of_node = of_node_get(pdev->dev.of_node);

	/* Callbacks */
	ctrl->read_cmd = spmi_read_cmd;
	ctrl->write_cmd = spmi_write_cmd;

	ret = spmi_controller_add(ctrl);
	if (ret) {
		dev_err(&pdev->dev,
			"spmi_controller_add failed with error %d!\n", ret);
		goto err_put_controller;
	}

	/* Let's look for other nodes in device tree like the rtc */
	ret = devm_of_platform_populate(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev,
			"spmi_controller_probe: devm_of_platform_populate failed with error %d!\n",
			ret);
		goto err_devm_of_platform_populate;
	}

	return 0;

err_put_controller:
	spmi_controller_put(ctrl);
err_devm_of_platform_populate:
	return ret;
}

static void spmi_del_controller(struct platform_device *pdev)
{
	struct spmi_controller *ctrl = platform_get_drvdata(pdev);

	spmi_controller_remove(ctrl);
	spmi_controller_put(ctrl);
}

static const struct of_device_id spmi_controller_match_table[] = {
	{
		.compatible = "apple,spmi",
	},
	{}
};
MODULE_DEVICE_TABLE(of, spmi_controller_match_table);

static struct platform_driver spmi_controller_driver = {
	.probe		= spmi_controller_probe,
	.remove		= spmi_del_controller,
	.driver		= {
		.name	= "apple-spmi",
		.of_match_table = spmi_controller_match_table,
	},
};
module_platform_driver(spmi_controller_driver);

MODULE_AUTHOR("Jean-Francois Bortolotti <jeff@borto.fr>");
MODULE_DESCRIPTION("Apple SoC SPMI driver");
MODULE_LICENSE("GPL");
