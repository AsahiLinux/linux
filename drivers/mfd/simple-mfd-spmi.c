// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Simple MFD - SPMI
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spmi.h>
#include <linux/of_platform.h>

static const struct regmap_config spmi_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
};

static int simple_spmi_probe(struct spmi_device *sdev)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spmi_ext(sdev, &spmi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return devm_of_platform_populate(&sdev->dev);
}

static const struct of_device_id simple_spmi_id_table[] = {
	{ .compatible = "apple,spmi-pmu" },
	{}
};
MODULE_DEVICE_TABLE(of, simple_spmi_id_table);

static struct spmi_driver pmic_spmi_driver = {
	.probe = simple_spmi_probe,
	.driver = {
		.name = "simple-mfd-spmi",
		.owner = THIS_MODULE,
		.of_match_table = simple_spmi_id_table,
	},
};
module_spmi_driver(pmic_spmi_driver);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Simple MFD - SPMI driver");
MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
