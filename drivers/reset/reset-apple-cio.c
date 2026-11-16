// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC CIO (USB4/Thunderbolt) block reset driver
 *
 * Copyright The Asahi Linux Contributors
 *
 * The CIO blocks have a reset inside the power manager (PMGR) that
 * has to be deasserted before their co-processor can be booted. On
 * t8103 each port comes with a dedicated register page while t600x
 * uses a single register shared by all ports of a die inside the PMGR
 * MMIO region.
 */

#include <linux/bits.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#define APPLE_CIO_RESET_POLL_US 100
#define APPLE_CIO_RESET_TIMEOUT_US 100000

#define T8103_CIO_CTRL_STRIDE 0x4000
#define T8103_CIO_CTRL_INIT_REQ BIT(0)
#define T8103_CIO_CTRL_INIT_DONE BIT(2)

#define T6000_CIO_CTRL_INIT_REQ(port) BIT(port)
#define T6000_CIO_CTRL_INIT_BUSY(port) BIT(16 + (port))

struct apple_cio_reset;

struct apple_cio_reset_variant {
	unsigned int nr_resets;
	bool pmgr_child;
	int (*deassert)(struct apple_cio_reset *priv, unsigned long id);
};

struct apple_cio_reset {
	struct reset_controller_dev rcdev;
	const struct apple_cio_reset_variant *variant;
	struct regmap *regmap;
	u32 offset;
	struct mutex lock; /* serializes access to the shared t600x register */
};

static int t8103_cio_deassert(struct apple_cio_reset *priv, unsigned long id)
{
	u32 offset = priv->offset + id * T8103_CIO_CTRL_STRIDE;
	u32 val;
	int ret;

	/*
	 * After writing BIT(0) the reset is deasserted and the CIO block comes up.
	 * Once the sequence is done BIT(0) is cleared automatically and then BIT(2)
	 * is set. There's no way to assert the reset again other than cutting the power
	 * to the entire CIO block. Writing the bit again has no effect.
	 */
	ret = regmap_write(priv->regmap, offset, T8103_CIO_CTRL_INIT_REQ);
	if (ret)
		return ret;

	return regmap_read_poll_timeout(priv->regmap, offset, val,
					val == T8103_CIO_CTRL_INIT_DONE,
					APPLE_CIO_RESET_POLL_US,
					APPLE_CIO_RESET_TIMEOUT_US);
}

static int t6000_cio_deassert(struct apple_cio_reset *priv, unsigned long id)
{
	u32 val;
	int ret;

	/*
	 * Note that even the mutex is likely paranoid and not actually required
	 * here: The way this controller works is that setting bit N immediately
	 * results in bit 16+n being set. Once the reset is fully released both
	 * bits are cleared again but macOS specifically waits for the upper one
	 * to clear before continuing.
	 * Manually clearing bits is ignored and has no effect, the only way to
	 * put the block into reset again is shut down its power domains.
	 * Thus, there's no need to either RMW here or wait before polling.
	 */
	guard(mutex)(&priv->lock);

	ret = regmap_write(priv->regmap, priv->offset, T6000_CIO_CTRL_INIT_REQ(id));
	if (ret)
		return ret;

	return regmap_read_poll_timeout(priv->regmap, priv->offset, val,
					!(val & T6000_CIO_CTRL_INIT_BUSY(id)),
					APPLE_CIO_RESET_POLL_US,
					APPLE_CIO_RESET_TIMEOUT_US);
}

static const struct apple_cio_reset_variant apple_t8103_cio_reset = {
	.nr_resets = 2,
	.deassert = t8103_cio_deassert,
};

static const struct apple_cio_reset_variant apple_t6000_cio_reset = {
	.nr_resets = 4,
	.pmgr_child = true,
	.deassert = t6000_cio_deassert,
};

static int apple_cio_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_cio_reset *priv =
		container_of(rcdev, struct apple_cio_reset, rcdev);

	return priv->variant->deassert(priv, id);
}

static const struct reset_control_ops apple_cio_reset_ops = {
	.deassert = apple_cio_reset_deassert,
};

static const struct regmap_config apple_cio_reset_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
};

static int apple_cio_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_cio_reset *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->variant = of_device_get_match_data(dev);
	if (!priv->variant)
		return -ENODEV;

	ret = devm_mutex_init(dev, &priv->lock);
	if (ret)
		return ret;

	if (priv->variant->pmgr_child) {
		priv->regmap = syscon_node_to_regmap(dev->of_node->parent);
		if (IS_ERR(priv->regmap))
			return dev_err_probe(dev, PTR_ERR(priv->regmap),
					     "Failed to get parent regmap\n");

		ret = of_property_read_u32(dev->of_node, "reg", &priv->offset);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to read reg offset\n");
	} else {
		void __iomem *base;

		base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(base))
			return PTR_ERR(base);

		priv->regmap = devm_regmap_init_mmio(dev, base,
						     &apple_cio_reset_regmap_config);
		if (IS_ERR(priv->regmap))
			return dev_err_probe(dev, PTR_ERR(priv->regmap),
					     "Failed to init MMIO regmap\n");
	}

	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.ops = &apple_cio_reset_ops;
	priv->rcdev.of_node = dev->of_node;
	priv->rcdev.nr_resets = priv->variant->nr_resets;

	return devm_reset_controller_register(dev, &priv->rcdev);
}

static const struct of_device_id apple_cio_reset_match[] = {
	{
		.compatible = "apple,t8103-cio-reset",
		.data = &apple_t8103_cio_reset,
	},
	{
		.compatible = "apple,t6000-cio-reset",
		.data = &apple_t6000_cio_reset,
	},
	{},
};
MODULE_DEVICE_TABLE(of, apple_cio_reset_match);

static struct platform_driver apple_cio_reset_driver = {
	.driver = {
		.name = "apple-cio-reset",
		.of_match_table = apple_cio_reset_match,
	},
	.probe = apple_cio_reset_probe,
};
module_platform_driver(apple_cio_reset_driver);

MODULE_AUTHOR("Sven Peter <sven@kernel.org>");
MODULE_DESCRIPTION("Apple SoC CIO block reset driver");
MODULE_LICENSE("Dual MIT/GPL");
