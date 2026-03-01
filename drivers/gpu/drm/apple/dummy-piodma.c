// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#include <drm/drm_module.h>

#include <linux/component.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of_device.h>

static int dcp_piodma_comp_bind(struct device *dev, struct device *main,
				void *data)
{
	return 0;
}

static void dcp_piodma_comp_unbind(struct device *dev, struct device *main,
				   void *data)
{
	/* nothing to do */
}

static const struct component_ops dcp_piodma_comp_ops = {
	.bind	= dcp_piodma_comp_bind,
	.unbind	= dcp_piodma_comp_unbind,
};
static int dcp_piodma_probe(struct platform_device *pdev)
{
	int ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;

	return component_add(&pdev->dev, &dcp_piodma_comp_ops);
}

static int dcp_piodma_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcp_piodma_comp_ops);

	return 0;
}

static void dcp_piodma_shutdown(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcp_piodma_comp_ops);
}

static const struct of_device_id of_match[] = {
	{ .compatible = "apple,dcp-piodma" },
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver dcp_piodma_platform_driver = {
	.probe		= dcp_piodma_probe,
	.remove		= dcp_piodma_remove,
	.shutdown	= dcp_piodma_shutdown,
	.driver	= {
		.name = "apple,dcp-piodma",
		.of_match_table	= of_match,
	},
};

drm_module_platform_driver(dcp_piodma_platform_driver);

MODULE_AUTHOR("Asahi Linux contributors");
MODULE_DESCRIPTION("[HACK] Apple DCP PIODMA shim");
MODULE_LICENSE("Dual MIT/GPL");
