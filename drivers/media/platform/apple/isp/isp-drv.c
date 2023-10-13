// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple Image Signal Processor driver
 *
 * Copyright (C) 2023 The Asahi Linux Contributors
 */

#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>

#include "isp-cam.h"
#include "isp-fw.h"
#include "isp-iommu.h"
#include "isp-v4l2.h"

static void apple_isp_detach_genpd(struct apple_isp *isp)
{
	if (isp->pd_count <= 1)
		return;

	for (int i = isp->pd_count - 1; i >= 0; i--) {
		if (isp->pd_link[i])
			device_link_del(isp->pd_link[i]);
		if (!IS_ERR_OR_NULL(isp->pd_dev[i]))
			dev_pm_domain_detach(isp->pd_dev[i], true);
	}

	return;
}

static int apple_isp_attach_genpd(struct apple_isp *isp)
{
	struct device *dev = isp->dev;

	isp->pd_count = of_count_phandle_with_args(
		dev->of_node, "power-domains", "#power-domain-cells");
	if (isp->pd_count <= 1)
		return 0;

	isp->pd_dev = devm_kcalloc(dev, isp->pd_count, sizeof(*isp->pd_dev),
				   GFP_KERNEL);
	if (!isp->pd_dev)
		return -ENOMEM;

	isp->pd_link = devm_kcalloc(dev, isp->pd_count, sizeof(*isp->pd_link),
				    GFP_KERNEL);
	if (!isp->pd_link)
		return -ENOMEM;

	for (int i = 0; i < isp->pd_count; i++) {
		int flags = DL_FLAG_STATELESS;

		/* Primary power domain uses RPM integration */
		if (i == 0)
			flags |= DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE;

		isp->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(isp->pd_dev[i])) {
			apple_isp_detach_genpd(isp);
			return PTR_ERR(isp->pd_dev[i]);
		}

		isp->pd_link[i] =
			device_link_add(dev, isp->pd_dev[i], flags);

		if (!isp->pd_link[i]) {
			apple_isp_detach_genpd(isp);
			return -EINVAL;
		}
	}

	return 0;
}

static int apple_isp_init_iommu(struct apple_isp *isp)
{
	struct device *dev = isp->dev;
	phys_addr_t heap_base;
	size_t heap_size;
	u64 vm_size;
	int err;
	int idx;
	int size;
	struct device_node *mem_node;
	const __be32 *maps, *end;

	isp->domain = iommu_get_domain_for_dev(isp->dev);
	if (!isp->domain)
		return -ENODEV;
	isp->shift = __ffs(isp->domain->pgsize_bitmap);

	idx = of_property_match_string(dev->of_node, "memory-region-names",
				       "heap");
	mem_node = of_parse_phandle(dev->of_node, "memory-region", idx);
	if (!mem_node) {
		dev_err(dev, "No memory-region found for heap\n");
		return -ENODEV;
	}

	maps = of_get_property(mem_node, "iommu-addresses", &size);
	if (!maps || !size) {
		dev_err(dev, "No valid iommu-addresses found for heap\n");
		return -ENODEV;
	}

	end = maps + size / sizeof(__be32);

	while (maps < end) {
		maps++;
		maps = of_translate_dma_region(dev->of_node, maps, &heap_base,
					       &heap_size);
	}

	isp->fw.heap_top = heap_base + heap_size;

	err = of_property_read_u64(dev->of_node, "apple,dart-vm-size",
				   &vm_size);
	if (err) {
		dev_err(dev, "failed to read 'apple,dart-vm-size': %d\n", err);
		return err;
	}

	// FIXME: refactor this, maybe use regular iova stuff?
	drm_mm_init(&isp->iovad, isp->fw.heap_top,
		    vm_size - (heap_base & 0xffffffff));

	return 0;
}

static void apple_isp_free_iommu(struct apple_isp *isp)
{
	drm_mm_takedown(&isp->iovad);
}

static int isp_of_read_coord(struct device *dev, struct device_node *np,
			     const char *prop, struct coord *val)
{
	u32 xy[2];
	int ret;

	ret = of_property_read_u32_array(np, prop, xy, 2);
	if (ret) {
		dev_err(dev, "failed to read '%s' property\n", prop);
		return ret;
	}

	val->x = xy[0];
	val->y = xy[1];
	return 0;
}

static int apple_isp_init_presets(struct apple_isp *isp)
{
	struct device *dev = isp->dev;
	struct isp_preset *preset;
	int err = 0;

	struct device_node *np __free(device_node) =
		of_get_child_by_name(dev->of_node, "sensor-presets");
	if (!np) {
		dev_err(dev, "failed to get DT node 'presets'\n");
		return -EINVAL;
	}

	isp->num_presets = of_get_child_count(np);
	if (!isp->num_presets) {
		dev_err(dev, "no sensor presets found\n");
		return -EINVAL;
	}

	isp->presets = devm_kzalloc(
		dev, sizeof(*isp->presets) * isp->num_presets, GFP_KERNEL);
	if (!isp->presets)
		return -ENOMEM;

	preset = isp->presets;
	for_each_child_of_node_scoped(np, child) {
		u32 xywh[4];

		err = of_property_read_u32(child, "apple,config-index",
					   &preset->index);
		if (err) {
			dev_err(dev, "no apple,config-index property\n");
			return err;
		}

		err = isp_of_read_coord(dev, child, "apple,input-size",
					&preset->input_dim);
		if (err)
			return err;
		err = isp_of_read_coord(dev, child, "apple,output-size",
					&preset->output_dim);
		if (err)
			return err;

		err = of_property_read_u32_array(child, "apple,crop", xywh, 4);
		if (err) {
			dev_err(dev, "failed to read 'apple,crop' property\n");
			return err;
		}
		preset->crop_offset.x = xywh[0];
		preset->crop_offset.y = xywh[1];
		preset->crop_size.x = xywh[2];
		preset->crop_size.y = xywh[3];

		preset++;
	}

	return 0;
}

static const char * isp_fw2str(enum isp_firmware_version version)
{
	switch (version) {
	case ISP_FIRMWARE_V_12_3:
		return "12.3";
	case ISP_FIRMWARE_V_12_4:
		return "12.4";
	case ISP_FIRMWARE_V_13_5:
		return "13.5";
	default:
		return "unknown";
	}
}

#define ISP_FW_VERSION_MIN_LEN	3
#define ISP_FW_VERSION_MAX_LEN	5

static enum isp_firmware_version isp_read_fw_version(struct device *dev,
						     const char *name)
{
	u32 ver[ISP_FW_VERSION_MAX_LEN];
	int len = of_property_read_variable_u32_array(dev->of_node, name, ver,
						      ISP_FW_VERSION_MIN_LEN,
						      ISP_FW_VERSION_MAX_LEN);

	switch (len) {
	case 3:
		if (ver[0] == 12 && ver[1] == 3 && ver[2] <= 1)
			return ISP_FIRMWARE_V_12_3;
		else if (ver[0] == 12 && ver[1] == 4 && ver[2] == 0)
			return ISP_FIRMWARE_V_12_4;
		else if (ver[0] == 13 && ver[1] == 5 && ver[2] == 0)
			return ISP_FIRMWARE_V_13_5;

		dev_warn(dev, "unknown %s: %d.%d.%d\n", name, ver[0], ver[1], ver[2]);
		break;
	case 4:
		dev_warn(dev, "unknown %s: %d.%d.%d.%d\n", name, ver[0], ver[1],
			 ver[2], ver[3]);
		break;
	case 5:
		dev_warn(dev, "unknown %s: %d.%d.%d.%d.%d\n", name, ver[0],
			 ver[1], ver[2], ver[3], ver[4]);
		break;
	default:
		dev_warn(dev, "could not parse %s: %d\n", name, len);
		break;
	}

	return ISP_FIRMWARE_V_UNKNOWN;
}

static enum isp_firmware_version isp_check_firmware_version(struct device *dev)
{
	enum isp_firmware_version version, compat;

	/* firmware version is just informative */
	version = isp_read_fw_version(dev, "apple,firmware-version");
	compat = isp_read_fw_version(dev, "apple,firmware-compat");

	dev_info(dev, "ISP firmware-compat: %s (FW: %s)\n", isp_fw2str(compat),
		 isp_fw2str(version));

	return compat;
}

static int apple_isp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_isp *isp;
	int err;

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (err)
		return err;

	isp = devm_kzalloc(dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	isp->dev = dev;
	isp->hw = of_device_get_match_data(dev);
	platform_set_drvdata(pdev, isp);
	dev_set_drvdata(dev, isp);

	/* Differences between firmware versions are rather minor so try to work
	 * with unknown firmware.
	 */
	isp->fw_compat = isp_check_firmware_version(dev);

	err = of_property_read_u32(dev->of_node, "apple,platform-id",
				   &isp->platform_id);
	if (err) {
		dev_err(dev, "failed to get 'apple,platform-id' property: %d\n",
			err);
		return err;
	}

	err = of_property_read_u32(dev->of_node, "apple,temporal-filter",
				   &isp->temporal_filter);
	if (err)
		isp->temporal_filter = 0;

	err = apple_isp_init_presets(isp);
	if (err) {
		dev_err(dev, "failed to initialize presets\n");
		return err;
	}

	err = apple_isp_attach_genpd(isp);
	if (err) {
		dev_err(dev, "failed to attatch power domains\n");
		return err;
	}

	isp->coproc = devm_platform_ioremap_resource_byname(pdev, "coproc");
	if (IS_ERR(isp->coproc)) {
		err = PTR_ERR(isp->coproc);
		goto detach_genpd;
	}

	isp->mbox = devm_platform_ioremap_resource_byname(pdev, "mbox");
	if (IS_ERR(isp->mbox)) {
		err = PTR_ERR(isp->mbox);
		goto detach_genpd;
	}

	isp->gpio = devm_platform_ioremap_resource_byname(pdev, "gpio");
	if (IS_ERR(isp->gpio)) {
		err = PTR_ERR(isp->gpio);
		goto detach_genpd;
	}

	isp->mbox2 = devm_platform_ioremap_resource_byname(pdev, "mbox2");
	if (IS_ERR(isp->mbox2)) {
		err = PTR_ERR(isp->mbox2);
		goto detach_genpd;
	}

	isp->irq = platform_get_irq(pdev, 0);
	if (isp->irq < 0) {
		err = isp->irq;
		goto detach_genpd;
	}
	if (!isp->irq) {
		err = -ENODEV;
		goto detach_genpd;
	}

	mutex_init(&isp->iovad_lock);
	mutex_init(&isp->video_lock);
	spin_lock_init(&isp->buf_lock);
	init_waitqueue_head(&isp->wait);
	INIT_LIST_HEAD(&isp->gc);
	INIT_LIST_HEAD(&isp->bufs_pending);
	INIT_LIST_HEAD(&isp->bufs_submitted);
	isp->wq = alloc_workqueue("apple-isp-wq", WQ_UNBOUND, 0);
	if (!isp->wq) {
		dev_err(dev, "failed to create workqueue\n");
		err = -ENOMEM;
		goto detach_genpd;
	}

	err = apple_isp_init_iommu(isp);
	if (err) {
		dev_err(dev, "failed to init iommu: %d\n", err);
		goto destroy_wq;
	}

	err = apple_isp_alloc_firmware_surface(isp);
	if (err) {
		dev_err(dev, "failed to alloc firmware surface: %d\n", err);
		goto free_iommu;
	}

	pm_runtime_enable(dev);

	err = apple_isp_detect_camera(isp);
	if (err) {
		dev_err(dev, "failed to detect camera: %d\n", err);
		goto free_surface;
	}

	err = apple_isp_setup_video(isp);
	if (err) {
		dev_err(dev, "failed to register video device: %d\n", err);
		goto free_surface;
	}

	dev_info(dev, "apple-isp probe!\n");

	return 0;

free_surface:
	pm_runtime_disable(dev);
	apple_isp_free_firmware_surface(isp);
free_iommu:
	apple_isp_free_iommu(isp);
destroy_wq:
	destroy_workqueue(isp->wq);
detach_genpd:
	apple_isp_detach_genpd(isp);
	return err;
}

static void apple_isp_remove(struct platform_device *pdev)
{
	struct apple_isp *isp = platform_get_drvdata(pdev);

	apple_isp_remove_video(isp);
	pm_runtime_disable(isp->dev);
	apple_isp_free_firmware_surface(isp);
	apple_isp_free_iommu(isp);
	destroy_workqueue(isp->wq);
	apple_isp_detach_genpd(isp);
}

static const struct apple_isp_hw apple_isp_hw_t8103 = {
	.gen = ISP_GEN_T8103,
	.pmu_base = 0x23b704000,

	.dsid_count = 4,
	.dsid_clr_base0 = 0x200014000,
	.dsid_clr_base1 = 0x200054000,
	.dsid_clr_base2 = 0x200094000,
	.dsid_clr_base3 = 0x2000d4000,
	.dsid_clr_range0 = 0x1000,
	.dsid_clr_range1 = 0x1000,
	.dsid_clr_range2 = 0x1000,
	.dsid_clr_range3 = 0x1000,

	.clock_scratch = 0x23b738010,
	.clock_base = 0x23bc3c000,
	.clock_bit = 0x1,
	.clock_size = 0x4,
	.bandwidth_scratch = 0x23b73800c,
	.bandwidth_base = 0x23bc3c000,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x4,

	.scl1 = false,
	.lpdp = false,
	.meta_size = ISP_META_SIZE_T8103,
};

static const struct apple_isp_hw apple_isp_hw_t6000 = {
	.gen = ISP_GEN_T8103,
	.pmu_base = 0x28e584000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200014000,
	.dsid_clr_base1 = 0x200054000,
	.dsid_clr_base2 = 0x200094000,
	.dsid_clr_base3 = 0x2000d4000,
	.dsid_clr_range0 = 0x1000,
	.dsid_clr_range1 = 0x1000,
	.dsid_clr_range2 = 0x1000,
	.dsid_clr_range3 = 0x1000,

	.clock_scratch = 0x28e3d0868,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x28e3d0980,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,

	.scl1 = false,
	.lpdp = false,
	.meta_size = ISP_META_SIZE_T8103,
};

static const struct apple_isp_hw apple_isp_hw_t8112 = {
	.gen = ISP_GEN_T8112,
	.pmu_base = 0x23b704000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200f14000,
	.dsid_clr_range0 = 0x1000,

	.clock_scratch = 0x23b3d0560,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x23b3d05d0,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,

	.scl1 = false,
	.lpdp = false,
	.meta_size = ISP_META_SIZE_T8112,
};

static const struct apple_isp_hw apple_isp_hw_t6020 = {
	.gen = ISP_GEN_T8112,
	.pmu_base = 0x290284000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200f14000,
	.dsid_clr_range0 = 0x1000,

	.clock_scratch = 0x28e3d10a8,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x28e3d1200,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,

	.scl1 = true,
	.lpdp = true,
	.meta_size = ISP_META_SIZE_T8112,
};

static const struct of_device_id apple_isp_of_match[] = {
	{ .compatible = "apple,t8103-isp", .data = &apple_isp_hw_t8103 },
	{ .compatible = "apple,t8112-isp", .data = &apple_isp_hw_t8112 },
	{ .compatible = "apple,t6000-isp", .data = &apple_isp_hw_t6000 },
	{ .compatible = "apple,t6020-isp", .data = &apple_isp_hw_t6020 },
	{},
};
MODULE_DEVICE_TABLE(of, apple_isp_of_match);

static __maybe_unused int apple_isp_runtime_suspend(struct device *dev)
{
	/* RPM sleep is called when the V4L2 file handle is closed */
	return 0;
}

static __maybe_unused int apple_isp_runtime_resume(struct device *dev)
{
	return 0;
}

static __maybe_unused int apple_isp_suspend(struct device *dev)
{
	struct apple_isp *isp = dev_get_drvdata(dev);

	/* We must restore V4L2 context on system resume. If we were streaming
	 * before, we (essentially) stop streaming and start streaming again.
	 */
	apple_isp_video_suspend(isp);

	return 0;
}

static __maybe_unused int apple_isp_resume(struct device *dev)
{
	struct apple_isp *isp = dev_get_drvdata(dev);

	apple_isp_video_resume(isp);

	return 0;
}

static const struct dev_pm_ops apple_isp_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(apple_isp_suspend, apple_isp_resume)
	RUNTIME_PM_OPS(apple_isp_runtime_suspend, apple_isp_runtime_resume, NULL)
};

static struct platform_driver apple_isp_driver = {
	.driver	= {
		.name		= "apple-isp",
		.of_match_table	= apple_isp_of_match,
		.pm		= pm_ptr(&apple_isp_pm_ops),
	},
	.probe	= apple_isp_probe,
	.remove	= apple_isp_remove,
};
module_platform_driver(apple_isp_driver);

MODULE_AUTHOR("Eileen Yoon <eyn@gmx.com>");
MODULE_DESCRIPTION("Apple ISP driver");
MODULE_LICENSE("GPL v2");
