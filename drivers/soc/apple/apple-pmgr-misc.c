// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMGR device power state driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#define APPLE_CLKGEN_PSTATE 0
#define APPLE_CLKGEN_PSTATE_DESIRED GENMASK(3, 0)

#define SYS_DEV_PSTATE_SUSPEND 1

enum sys_device {
	DEV_FABRIC,
	DEV_DCS,
	DEV_MAX,
};

struct apple_pmgr_sys_device {
	void __iomem *base;
	u32 active_state;
	u32 suspend_state;
};

struct apple_pmgr_misc {
	struct device *dev;
	struct apple_pmgr_sys_device devices[DEV_MAX];
};

static void apple_pmgr_sys_dev_set_pstate(struct apple_pmgr_misc *misc,
					  enum sys_device dev, bool active)
{
	u32 pstate;
	u32 val;

	if (!misc->devices[dev].base)
		return;

	if (active)
		pstate = misc->devices[dev].active_state;
	else
		pstate = misc->devices[dev].suspend_state;

	printk("set %d ps to pstate %d\n", dev, pstate);

	val = readl_relaxed(misc->devices[dev].base + APPLE_CLKGEN_PSTATE);
	val &= ~APPLE_CLKGEN_PSTATE_DESIRED;
	val |= FIELD_PREP(APPLE_CLKGEN_PSTATE_DESIRED, pstate);
	writel_relaxed(val, misc->devices[dev].base);
}

static int __maybe_unused apple_pmgr_misc_suspend_noirq(struct device *dev)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < DEV_MAX; i++)
		apple_pmgr_sys_dev_set_pstate(misc, i, false);

	return 0;
}

static int __maybe_unused apple_pmgr_misc_resume_noirq(struct device *dev)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < DEV_MAX; i++)
		apple_pmgr_sys_dev_set_pstate(misc, i, true);

	return 0;
}

static bool apple_pmgr_init_device(struct apple_pmgr_misc *misc,
				   enum sys_device dev, const char *device_name)
{
	void __iomem *base;
	char name[32];
	u32 val;

	snprintf(name, sizeof(name), "%s-ps", device_name);

	base = devm_platform_ioremap_resource_byname(
		to_platform_device(misc->dev), name);
	if (!base)
		return false;

	val = readl_relaxed(base + APPLE_CLKGEN_PSTATE);

	misc->devices[dev].base = base;
	misc->devices[dev].active_state =
		FIELD_GET(APPLE_CLKGEN_PSTATE_DESIRED, val);
	misc->devices[dev].suspend_state = SYS_DEV_PSTATE_SUSPEND;

	snprintf(name, sizeof(name), "apple,%s-min-ps", device_name);
	of_property_read_u32(misc->dev->of_node, name,
			     &misc->devices[dev].suspend_state);

	return true;
}

static int apple_pmgr_misc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_pmgr_misc *misc;
	int ret = -ENODEV;

	misc = devm_kzalloc(dev, sizeof(*misc), GFP_KERNEL);
	if (!misc)
		return -ENOMEM;

	misc->dev = dev;

	if (apple_pmgr_init_device(misc, DEV_FABRIC, "fabric"))
		ret = 0;

	if (apple_pmgr_init_device(misc, DEV_DCS, "dcs"))
		ret = 0;

	platform_set_drvdata(pdev, misc);

	return ret;
}

static const struct of_device_id apple_pmgr_misc_of_match[] = {
	{ .compatible = "apple,t6000-pmgr-misc" },
	{}
};

MODULE_DEVICE_TABLE(of, apple_pmgr_misc_of_match);

static const struct dev_pm_ops apple_pmgr_misc_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(apple_pmgr_misc_suspend_noirq,
				      apple_pmgr_misc_resume_noirq)
};

static struct platform_driver apple_pmgr_misc_driver = {
	.probe = apple_pmgr_misc_probe,
	.driver = {
		.name = "apple-pmgr-misc",
		.of_match_table = apple_pmgr_misc_of_match,
		.pm = pm_ptr(&apple_pmgr_misc_pm_ops),
	},
};

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("PMGR misc driver for Apple SoCs");
MODULE_LICENSE("GPL v2");

module_platform_driver(apple_pmgr_misc_driver);
