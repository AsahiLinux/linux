// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Driver for clock divider registers on Apple's SoC
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

struct apple_clkdiv {
	struct clk_divider div;
	struct clk_gate gate;
	spinlock_t lock;
};

#define CLKDIV_ENABLE	BIT(19)
#define CLKDIV_DIVISOR	GENMASK(9, 1) // TODO: this may need to be GENMASK(9, 0)

static int apple_clkdiv_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct clk_parent_data pdata = { .index = 0 };
	struct apple_clkdiv *appldiv;
	struct resource *res;
	struct clk_hw *hw;
	void __iomem *base;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	appldiv = devm_kzalloc(&pdev->dev, sizeof(*appldiv), GFP_KERNEL);
	if (!appldiv)
		return -ENOMEM;

	spin_lock_init(&appldiv->lock);

	appldiv->gate.reg = base;
	appldiv->gate.bit_idx = __bf_shf(CLKDIV_ENABLE);
	appldiv->gate.lock = &appldiv->lock;
	appldiv->div.reg = base;
	appldiv->div.shift = __bf_shf(CLKDIV_DIVISOR);
	appldiv->div.width = __const_hweight32(CLKDIV_DIVISOR);
	
	appldiv->div.lock = &appldiv->lock;
	appldiv->div.flags = CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO;

	hw = devm_clk_hw_register_composite_pdata(&pdev->dev, np->name,
						  &pdata, 1, NULL, NULL,
						  &appldiv->div.hw,
						  &clk_divider_ops,
						  &appldiv->gate.hw,
						  &clk_gate_ops,
						  CLK_SET_RATE_GATE);
	if (IS_ERR(hw))
		return PTR_ERR(hw);

	return devm_of_clk_add_hw_provider(&pdev->dev, of_clk_hw_simple_get, hw);
}

static const struct of_device_id apple_clkdiv_of_match[] = {
	{ .compatible = "apple,clkdiv" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_clkdiv_of_match);

static struct platform_driver apple_clkdiv_driver = {
	.driver = {
		.name = "clk-apple-clkdiv",
		.of_match_table = apple_clkdiv_of_match,
	},
	.probe = apple_clkdiv_probe,
};
module_platform_driver(apple_clkdiv_driver);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("Driver for CLKDIV registers on Apple SoCs");
MODULE_LICENSE("Dual MIT/GPL");
