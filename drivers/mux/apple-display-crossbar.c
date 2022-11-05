// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple Silicon Display Crossbar multiplexer driver
 *
 * Copyright (C) Asahi Linux Contributors
 *
 * Author: Sven Peter <sven@svenpeter.dev>
 */

#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#define FIFO_WR_DPTX_CLK_EN 0x000
#define FIFO_WR_N_CLK_EN 0x004
#define FIFO_WR_UNK_EN 0x008
#define FIFO_RD_PCLK1_EN 0x020
#define FIFO_RD_PCLK2_EN 0x024
#define FIFO_RD_N_CLK_EN 0x028
#define FIFO_RD_UNK_EN 0x02c

#define OUT_PCLK1_EN 0x040
#define OUT_PCLK2_EN 0x044
#define OUT_N_CLK_EN 0x048
#define OUT_UNK_EN 0x04c

#define CROSSBAR_DISPEXT_EN 0x050
#define CROSSBAR_MUX_CTRL 0x060
#define CROSSBAR_MUX_CTRL_DPPHY_SELECT0 GENMASK(23, 20)
#define CROSSBAR_MUX_CTRL_DPIN1_SELECT0 GENMASK(19, 16)
#define CROSSBAR_MUX_CTRL_DPIN0_SELECT0 GENMASK(15, 12)
#define CROSSBAR_MUX_CTRL_DPPHY_SELECT1 GENMASK(11, 8)
#define CROSSBAR_MUX_CTRL_DPIN1_SELECT1 GENMASK(7, 4)
#define CROSSBAR_MUX_CTRL_DPIN0_SELECT1 GENMASK(3, 0)
#define CROSSBAR_ATC_EN 0x070

#define FIFO_WR_DPTX_CLK_EN_STAT 0x800
#define FIFO_WR_N_CLK_EN_STAT 0x804
#define FIFO_RD_PCLK1_EN_STAT 0x820
#define FIFO_RD_PCLK2_EN_STAT 0x824
#define FIFO_RD_N_CLK_EN_STAT 0x828

#define OUT_PCLK1_EN_STAT 0x840
#define OUT_PCLK2_EN_STAT 0x844
#define OUT_N_CLK_EN_STAT 0x848

#define UNK_TUNABLE 0xc00

#define ATC_DPIN0 BIT(0)
#define ATC_DPIN1 BIT(4)
#define ATC_DPPHY BIT(8)

enum { MUX_DPPHY = 0, MUX_DPIN0 = 1, MUX_DPIN1 = 2, MUX_MAX = 3 };
static const char *apple_dpxbar_names[MUX_MAX] = { "dpphy", "dpin0", "dpin1" };

struct apple_dpxbar_hw {
	unsigned int n_ufp;
	u32 tunable;
};

struct apple_dpxbar {
	struct device *dev;
	void __iomem *regs;
	int selected_dispext[MUX_MAX];
	spinlock_t lock;
};

static inline void dpxbar_mask32(struct apple_dpxbar *xbar, u32 reg, u32 mask,
				 u32 set)
{
	u32 value = readl(xbar->regs + reg);
	value &= ~mask;
	value |= set;
	writel(value, xbar->regs + reg);
}

static inline void dpxbar_set32(struct apple_dpxbar *xbar, u32 reg, u32 set)
{
	dpxbar_mask32(xbar, reg, 0, set);
}

static inline void dpxbar_clear32(struct apple_dpxbar *xbar, u32 reg, u32 clear)
{
	dpxbar_mask32(xbar, reg, clear, 0);
}

static int apple_dpxbar_set(struct mux_control *mux, int state)
{
	struct apple_dpxbar *dpxbar = mux_chip_priv(mux->chip);
	unsigned int index = mux_control_get_index(mux);
	unsigned long flags;
	unsigned int mux_state;
	unsigned int dispext_bit;
	unsigned int atc_bit;
	bool enable;
	int ret = 0;
	u32 mux_mask, mux_set;

	if (state == MUX_IDLE_DISCONNECT) {
		/*
		 * Technically this will select dispext0,0 in the mux control
		 * register. Practically that doesn't matter since everything
		 * else is disabled.
		 */
		mux_state = 0;
		enable = false;
	} else if (state >= 0 && state < 9) {
		dispext_bit = 1 << state;
		mux_state = state;
		enable = true;
	} else {
		return -EINVAL;
	}

	switch (index) {
	case MUX_DPPHY:
		mux_mask = CROSSBAR_MUX_CTRL_DPPHY_SELECT0 |
			   CROSSBAR_MUX_CTRL_DPPHY_SELECT1;
		mux_set =
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPPHY_SELECT0, mux_state) |
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPPHY_SELECT1, mux_state);
		atc_bit = ATC_DPPHY;
		break;
	case MUX_DPIN0:
		mux_mask = CROSSBAR_MUX_CTRL_DPIN0_SELECT0 |
			   CROSSBAR_MUX_CTRL_DPIN0_SELECT1;
		mux_set =
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN0_SELECT0, mux_state) |
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN0_SELECT1, mux_state);
		atc_bit = ATC_DPIN0;
		break;
	case MUX_DPIN1:
		mux_mask = CROSSBAR_MUX_CTRL_DPIN1_SELECT0 |
			   CROSSBAR_MUX_CTRL_DPIN1_SELECT1;
		mux_set =
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN1_SELECT0, mux_state) |
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN1_SELECT1, mux_state);
		atc_bit = ATC_DPIN1;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&dpxbar->lock, flags);

	/* ensure the selected dispext isn't already used in this crossbar */
	if (enable) {
		for (int i = 0; i < MUX_MAX; ++i) {
			if (i == index)
				continue;
			if (dpxbar->selected_dispext[i] == state) {
				spin_unlock_irqrestore(&dpxbar->lock, flags);
				return -EBUSY;
			}
		}
	}

	dpxbar_set32(dpxbar, OUT_N_CLK_EN, atc_bit);
	dpxbar_clear32(dpxbar, OUT_UNK_EN, atc_bit);
	dpxbar_clear32(dpxbar, OUT_PCLK1_EN, atc_bit);
	dpxbar_clear32(dpxbar, CROSSBAR_ATC_EN, atc_bit);

	if (dpxbar->selected_dispext[index] >= 0) {
		u32 prev_dispext_bit = 1 << dpxbar->selected_dispext[index];

		dpxbar_set32(dpxbar, FIFO_WR_N_CLK_EN, prev_dispext_bit);
		dpxbar_set32(dpxbar, FIFO_RD_N_CLK_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_WR_UNK_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_RD_UNK_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_WR_DPTX_CLK_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_RD_PCLK1_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, CROSSBAR_DISPEXT_EN, prev_dispext_bit);

		dpxbar->selected_dispext[index] = -1;
	}

	dpxbar_mask32(dpxbar, CROSSBAR_MUX_CTRL, mux_mask, mux_set);

	if (enable) {
		dpxbar_clear32(dpxbar, FIFO_WR_N_CLK_EN, dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_RD_N_CLK_EN, dispext_bit);
		dpxbar_clear32(dpxbar, OUT_N_CLK_EN, atc_bit);
		dpxbar_set32(dpxbar, FIFO_WR_UNK_EN, dispext_bit);
		dpxbar_set32(dpxbar, FIFO_RD_UNK_EN, dispext_bit);
		dpxbar_set32(dpxbar, OUT_UNK_EN, atc_bit);
		dpxbar_set32(dpxbar, FIFO_WR_DPTX_CLK_EN, dispext_bit);
		dpxbar_set32(dpxbar, FIFO_RD_PCLK1_EN, dispext_bit);
		dpxbar_set32(dpxbar, OUT_PCLK1_EN, atc_bit);
		dpxbar_set32(dpxbar, CROSSBAR_ATC_EN, atc_bit);
		dpxbar_set32(dpxbar, CROSSBAR_DISPEXT_EN, dispext_bit);

		/*
		 * Work around some HW quirk:
		 * Without toggling the RD_PCLK enable here the connection
		 * doesn't come up. Testing has shown that a delay of about
		 * 5 usec is required which is doubled here to be on the
		 * safe side.
		 */
		dpxbar_clear32(dpxbar, FIFO_RD_PCLK1_EN, dispext_bit);
		udelay(10);
		dpxbar_set32(dpxbar, FIFO_RD_PCLK1_EN, dispext_bit);

		dpxbar->selected_dispext[index] = state;
	}

	spin_unlock_irqrestore(&dpxbar->lock, flags);

	if (enable)
		dev_info(dpxbar->dev, "Switched %s to dispext%u,%u\n",
			 apple_dpxbar_names[index], mux_state >> 1,
			 mux_state & 1);
	else
		dev_info(dpxbar->dev, "Switched %s to disconnected state\n",
			 apple_dpxbar_names[index]);

	return ret;
}

static const struct mux_control_ops apple_dpxbar_ops = {
	.set = apple_dpxbar_set,
};

static int apple_dpxbar_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mux_chip *mux_chip;
	struct apple_dpxbar *dpxbar;
	const struct apple_dpxbar_hw *hw;
	int ret;

	hw = of_device_get_match_data(dev);
	mux_chip = devm_mux_chip_alloc(dev, MUX_MAX, sizeof(*dpxbar));
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	dpxbar = mux_chip_priv(mux_chip);
	mux_chip->ops = &apple_dpxbar_ops;
	spin_lock_init(&dpxbar->lock);

	dpxbar->dev = dev;
	dpxbar->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dpxbar->regs))
		return PTR_ERR(dpxbar->regs);

	writel(hw->tunable, dpxbar->regs + UNK_TUNABLE);

	for (unsigned int i = 0; i < MUX_MAX; ++i) {
		mux_chip->mux[i].states = hw->n_ufp;
		mux_chip->mux[i].idle_state = MUX_IDLE_DISCONNECT;
		dpxbar->selected_dispext[i] = -1;
	}

	ret = devm_mux_chip_register(dev, mux_chip);
	if (ret < 0)
		return ret;

	return 0;
}

const static struct apple_dpxbar_hw apple_dpxbar_hw_t8103 = {
	.n_ufp = 2,
	.tunable = 0,
};

const static struct apple_dpxbar_hw apple_dpxbar_hw_t6000 = {
	.n_ufp = 9,
	.tunable = 5,
};

static const struct of_device_id apple_dpxbar_ids[] = {
	{
		.compatible = "apple,t8103-display-crossbar",
		.data = &apple_dpxbar_hw_t8103,
	},
	{
		.compatible = "apple,t8112-display-crossbar",
		.data = &apple_dpxbar_hw_t8103,
	},
	{
		.compatible = "apple,t6000-display-crossbar",
		.data = &apple_dpxbar_hw_t6000,
	},
	{}
};
MODULE_DEVICE_TABLE(of, apple_dpxbar_ids);

static struct platform_driver apple_dpxbar_driver = {
	.driver = {
		.name = "apple-display-crossbar",
		.of_match_table	= apple_dpxbar_ids,
	},
	.probe = apple_dpxbar_probe,
};
module_platform_driver(apple_dpxbar_driver);

MODULE_DESCRIPTION("Apple Silicon display crossbar multiplexer driver");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_LICENSE("GPL v2");
