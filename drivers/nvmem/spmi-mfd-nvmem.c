// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Generic SPMI MFD NVMEM driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct spmi_mfd_nvmem {
	struct regmap *regmap;
	unsigned int base;
};

static int spmi_mfd_nvmem_read(void *priv, unsigned int offset,
                               void *val, size_t bytes)
{
	struct spmi_mfd_nvmem *nvmem = priv;

        return regmap_bulk_read(nvmem->regmap, nvmem->base + offset, val, bytes);
}

static int spmi_mfd_nvmem_write(void *priv, unsigned int offset,
                                void *val, size_t bytes)
{
	struct spmi_mfd_nvmem *nvmem = priv;

	return regmap_bulk_write(nvmem->regmap, nvmem->base + offset, val, bytes);
}

static int spmi_mfd_nvmem_probe(struct platform_device *pdev)
{
	struct spmi_mfd_nvmem *nvmem;
	const __be32 *addr;
	int len;
	struct nvmem_config nvmem_cfg = {
		.dev = &pdev->dev,
		.name = "spmi_mfd_nvmem",
		.id = NVMEM_DEVID_AUTO,
		.word_size = 1,
		.stride = 1,
		.reg_read = spmi_mfd_nvmem_read,
		.reg_write = spmi_mfd_nvmem_write,
		.add_legacy_fixed_of_cells = true,
	};

	nvmem = devm_kzalloc(&pdev->dev, sizeof(*nvmem), GFP_KERNEL);
	if (!nvmem)
		return -ENOMEM;

	nvmem_cfg.priv = nvmem;

	nvmem->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!nvmem->regmap) {
		dev_err(&pdev->dev, "Parent regmap unavailable.\n");
		return -ENXIO;
	}

	addr = of_get_property(pdev->dev.of_node, "reg", &len);
	if (!addr) {
		dev_err(&pdev->dev, "no reg property\n");
		return -EINVAL;
	}
	if (len != 2 * sizeof(u32)) {
		dev_err(&pdev->dev, "invalid reg property\n");
		return -EINVAL;
	}

	nvmem->base = be32_to_cpup(&addr[0]);
	nvmem_cfg.size = be32_to_cpup(&addr[1]);

	return PTR_ERR_OR_ZERO(devm_nvmem_register(&pdev->dev, &nvmem_cfg));
}

static const struct of_device_id spmi_mfd_nvmem_id_table[] = {
	{ .compatible = "apple,spmi-pmu-nvmem" },
	{ .compatible = "spmi-mfd-nvmem" },
	{ },
};
MODULE_DEVICE_TABLE(of, spmi_mfd_nvmem_id_table);

static struct platform_driver spmi_mfd_nvmem_driver = {
	.probe = spmi_mfd_nvmem_probe,
	.driver = {
		.name = "spmi-mfd-nvmem",
		.of_match_table	= spmi_mfd_nvmem_id_table,
	},
};

module_platform_driver(spmi_mfd_nvmem_driver);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("SPMI MFD NVMEM driver");
