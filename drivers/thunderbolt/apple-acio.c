// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple ACIO (Thunderbolt/USB4) Controller Driver
 *
 * The Apple ACIO is a custom Thunderbolt/USB4 host controller found in
 * Apple Silicon SoCs. It consists of:
 *   - NHI-like register space for ring buffer DMA
 *   - An IOP coprocessor running Thunderbolt firmware via RTKit
 *   - DART (IOMMU) for DMA isolation
 *   - Connection to ATC PHY for physical layer
 *
 * This driver bridges the Apple ACIO hardware to the Linux Thunderbolt
 * subsystem, enabling PCIe tunneling, DisplayPort tunneling, and USB3
 * tunneling over Thunderbolt/USB4.
 *
 * Copyright (C) 2026 Asahi Linux Contributors
 * Derived from clean-room reverse engineering of macOS hardware traces.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/soc/apple/rtkit.h>

#define ACIO_NHI_VERSION		0x0000
#define ACIO_NHI_CAPS			0x0004
#define ACIO_NHI_CAPS_HOP_COUNT		GENMASK(15, 0)

#define ACIO_CPU_CONTROL		0x0044
#define ACIO_CPU_CONTROL_RUN		BIT(4)

#define ACIO_BOOT_TIMEOUT_MS		5000

struct apple_acio {
	struct device *dev;
	void __iomem *nhi_base;     /* NHI register space */
	void __iomem *hbw_base;     /* High bandwidth fabric */
	void __iomem *lbw_base;     /* Low bandwidth fabric */
	void __iomem *coproc_base;  /* IOP coprocessor control */
	void __iomem *mbox_base;    /* IOP mailbox */

	struct apple_rtkit *rtk;
	struct phy *usb4_phy;

	u32 hop_count;
	int index;
};

static int acio_rtk_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_acio *acio = cookie;

	if (bfr->iova) {
		bfr->buffer =
			devm_ioremap(acio->dev, bfr->iova, bfr->size);
		if (!bfr->buffer)
			return -ENOMEM;
	} else {
		bfr->buffer =
			dma_alloc_coherent(acio->dev, bfr->size,
					   &bfr->iova, GFP_KERNEL);
		if (!bfr->buffer)
			return -ENOMEM;
	}

	return 0;
}

static void acio_rtk_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	/* nothing for now */
}

static void acio_rtk_crashed(void *cookie, const void *crashlog,
			     size_t crashlog_size)
{
	struct apple_acio *acio = cookie;

	dev_err(acio->dev, "ACIO IOP firmware crashed! (log size: %zu)\n",
		crashlog_size);
}

static void acio_rtk_recv(void *cookie, u8 ep, u64 msg)
{
	struct apple_acio *acio = cookie;

	dev_dbg(acio->dev, "RTKit message: ep=%u msg=0x%016llx\n", ep, msg);
}

static const struct apple_rtkit_ops acio_rtkit_ops = {
	.crashed = acio_rtk_crashed,
	.recv_message = acio_rtk_recv,
	.shmem_setup = acio_rtk_shmem_setup,
	.shmem_destroy = acio_rtk_shmem_destroy,
};

static int apple_acio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_acio *acio;
	u32 val;
	int ret;

	acio = devm_kzalloc(dev, sizeof(*acio), GFP_KERNEL);
	if (!acio)
		return -ENOMEM;

	acio->dev = dev;
	platform_set_drvdata(pdev, acio);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;

	/* Map NHI registers */
	acio->nhi_base = devm_platform_ioremap_resource_byname(pdev, "nhi");
	if (IS_ERR(acio->nhi_base))
		return dev_err_probe(dev, PTR_ERR(acio->nhi_base),
				     "Failed to map NHI registers\n");

	/* Map high-bandwidth fabric */
	acio->hbw_base = devm_platform_ioremap_resource_byname(pdev, "hbw");
	if (IS_ERR(acio->hbw_base))
		return dev_err_probe(dev, PTR_ERR(acio->hbw_base),
				     "Failed to map HBW registers\n");

	/* Map low-bandwidth fabric */
	acio->lbw_base = devm_platform_ioremap_resource_byname(pdev, "lbw");
	if (IS_ERR(acio->lbw_base))
		return dev_err_probe(dev, PTR_ERR(acio->lbw_base),
				     "Failed to map LBW registers\n");

	/* Read NHI capabilities */
	val = readl(acio->nhi_base + ACIO_NHI_VERSION);
	dev_info(dev, "ACIO NHI version: 0x%08x\n", val);

	val = readl(acio->nhi_base + ACIO_NHI_CAPS);
	acio->hop_count = FIELD_GET(ACIO_NHI_CAPS_HOP_COUNT, val);
	dev_info(dev, "ACIO NHI caps: 0x%08x, hop_count=%u\n",
		 val, acio->hop_count);

	/* Get USB4 PHY */
	acio->usb4_phy = devm_phy_optional_get(dev, "usb4-phy");
	if (IS_ERR(acio->usb4_phy))
		return dev_err_probe(dev, PTR_ERR(acio->usb4_phy),
				     "Failed to get USB4 PHY\n");

	dev_info(dev, "Apple ACIO Thunderbolt controller probed\n");

	/*
	 * TODO Phase 2: Boot IOP coprocessor via RTKit
	 * TODO Phase 3: Initialize ring buffers and register with TB subsystem
	 * TODO Phase 4: Enable PCIe/DP/USB3 tunneling
	 */

	return 0;
}

static void apple_acio_remove(struct platform_device *pdev)
{
	struct apple_acio *acio = platform_get_drvdata(pdev);

	dev_info(acio->dev, "Apple ACIO Thunderbolt controller removed\n");
}

static const struct of_device_id apple_acio_of_match[] = {
	{ .compatible = "apple,t8112-acio" },
	{ .compatible = "apple,acio" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_acio_of_match);

static struct platform_driver apple_acio_driver = {
	.probe = apple_acio_probe,
	.remove = apple_acio_remove,
	.driver = {
		.name = "apple-acio",
		.of_match_table = apple_acio_of_match,
	},
};
module_platform_driver(apple_acio_driver);

MODULE_AUTHOR("Asahi Linux Contributors");
MODULE_DESCRIPTION("Apple ACIO Thunderbolt/USB4 Controller");
MODULE_LICENSE("Dual MIT/GPL");
