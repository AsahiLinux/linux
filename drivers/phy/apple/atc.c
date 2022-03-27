// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Type-C PHY driver
 *
 * Copyright (C) 2021 The Asahi Linux Contributors
 */

#include "atc.h"
#include "trace.h"

#include <dt-bindings/phy/phy.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_tbt.h>

#define rcdev_to_apple_atcphy(_rcdev) \
	container_of(_rcdev, struct apple_atcphy, rcdev)

#define AUSPLL_DCO_EFUSE_SPARE 0x222c
#define AUSPLL_RODCO_ENCAP_EFUSE GENMASK(10, 9)
#define AUSPLL_RODCO_BIAS_ADJUST_EFUSE GENMASK(14, 12)

#define AUSPLL_FRACN_CAN 0x22a4
#define AUSPLL_DLL_START_CAPCODE GENMASK(18, 17)

#define AUSPLL_CLKOUT_DTC_VREG 0x2220
#define AUSPLL_DTC_VREG_ADJUST GENMASK(16, 14)

#define AUS_COMMON_SHIM_BLK_VREG 0x0a04
#define AUS_VREG_TRIM GENMASK(6, 2)

#define CIO3PLL_CLK_CTRL 0x2a00
#define CIO3PLL_CLK_PCLK_EN BIT(1)
#define CIO3PLL_CLK_REFCLK_EN BIT(5)

#define CIO3PLL_DCO_NCTRL 0x2a38
#define CIO3PLL_DCO_COARSEBIN_EFUSE0 GENMASK(6, 0)
#define CIO3PLL_DCO_COARSEBIN_EFUSE1 GENMASK(23, 17)

#define CIO3PLL_FRACN_CAN 0x2aa4
#define CIO3PLL_DLL_CAL_START_CAPCODE GENMASK(18, 17)

#define CIO3PLL_DTC_VREG 0x2a20
#define CIO3PLL_DTC_VREG_ADJUST GENMASK(16, 14)

#define ACIOPHY_CROSSBAR 0x4c
#define ACIOPHY_CROSSBAR_PROTOCOL GENMASK(4, 0)
#define ACIOPHY_CROSSBAR_PROTOCOL_USB4 0x0
#define ACIOPHY_CROSSBAR_PROTOCOL_USB4_SWAPPED 0x1
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3 0xa
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED 0xb
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP 0x10
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED 0x10
#define ACIOPHY_CROSSBAR_PROTOCOL_DP 0x14
#define ACIOPHY_CROSSBAR_DPMODE GENMASK(17, 5)

#define ACIOPHY_LANE_MODE 0x48
#define ACIOPHY_LANE_MODE_RX0 GENMASK(2, 0)
#define ACIOPHY_LANE_MODE_TX0 GENMASK(5, 3)
#define ACIOPHY_LANE_MODE_RX1 GENMASK(8, 6)
#define ACIOPHY_LANE_MODE_TX1 GENMASK(11, 9)
#define ACIOPHY_LANE_MODE_USB4 0
#define ACIOPHY_LANE_MODE_USB3 1
#define ACIOPHY_LANE_MODE_DP 2
#define ACIOPHY_LANE_MODE_OFF 3

#define ATCPHY_POWER_CTRL 0x20000
#define ATCPHY_POWER_STAT 0x20004
#define ATCPHY_POWER_SLEEP_SMALL BIT(0)
#define ATCPHY_POWER_SLEEP_BIG BIT(1)
#define ATCPHY_POWER_CLAMP_EN BIT(2)
#define ATCPHY_POWER_APB_RESET_N BIT(3)
#define ATCPHY_POWER_PHY_RESET_N BIT(4)

#define ATCPHY_MISC 0x20008
#define ATCPHY_MISC_RESET_N BIT(0)
#define ATCPHY_MISC_LANE_SWAP BIT(2)

/* pipehandler registers */
#define PIPEHANDLER_OVERRIDE 0x00
#define PIPEHANDLER_OVERRIDE_RXVALID BIT(0)
#define PIPEHANDLER_OVERRIDE_RXDETECT BIT(2)

#define PIPEHANDLER_OVERRIDE_VALUES 0x04

#define PIPEHANDLER_MUX_CTRL 0x0c
#define PIPEHANDLER_MUX_MODE GENMASK(1, 0)
#define PIPEHANDLER_MUX_MODE_USB3PHY 0
#define PIPEHANDLER_MUX_MODE_DUMMY_PHY 0
#define PIPEHANDLER_CLK_SELECT GENMASK(5, 3)
#define PIPEHANDLER_CLK_USB3PHY 1
#define PIPEHANDLER_CLK_DUMMY_PHY 2
#define PIPEHANDLER_LOCK_REQ 0x10
#define PIPEHANDLER_LOCK_ACK 0x14
#define PIPEHANDLER_LOCK_EN BIT(0)

#define PIPEHANDLER_AON_GEN 0x1C
#define DWC3_FORCE_CLAMP_EN BIT(4)
#define DWC3_RESET_N BIT(0)

/* TODO: non-select probably just means that these bits are always active unlike the OVERRIDE_EN above */
#define PIPEHANDLER_NONSELECTED_OVERRIDE 0x20
#define PIPEHANDLER_NONSELECTED_NATIVE_RESET BIT(12)
#define PIPEHANDLER_DUMMY_PHY_EN BIT(15)
#define PIPEHANDLER_NONSELECTED_NATIVE_POWER_DOWN GENMASK(3, 0)

/* USB2 PHY regs */
#define USB2PHY_USBCTL 0x00
#define USB2PHY_USBCTL_HOST_EN BIT(1)

#define USB2PHY_CTL 0x04
#define USB2PHY_CTL_RESET BIT(0)
#define USB2PHY_CTL_PORT_RESET BIT(1)
#define USB2PHY_CTL_APB_RESET_N BIT(2)
#define USB2PHY_CTL_SIDDQ BIT(3)

#define USB2PHY_SIG 0x08
#define USB2PHY_SIG_VBUSDET_FORCE_VAL BIT(0)
#define USB2PHY_SIG_VBUSDET_FORCE_EN BIT(1)
#define USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL BIT(2)
#define USB2PHY_SIG_VBUSVLDEXT_FORCE_EN BIT(3)
#define USB2PHY_SIG_HOST (7 << 12)

static const struct {
	const struct atcphy_mode_configuration normal;
	const struct atcphy_mode_configuration swapped;
} atcphy_modes[APPLE_ATCPHY_MODE_MAX] = {
	[APPLE_ATCPHY_MODE_USB2] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_OFF},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_OFF},
			.set_swap = false, /* doesn't matter since the SS lanes are off */
		},
	},
	[APPLE_ATCPHY_MODE_USB3] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3,
			.lane_mode = {ACIOPHY_LANE_MODE_USB3, ACIOPHY_LANE_MODE_OFF},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_USB3},
			.set_swap = true,
		},
	},
	[APPLE_ATCPHY_MODE_USB3_DP] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP,
			.lane_mode = {ACIOPHY_LANE_MODE_USB3, ACIOPHY_LANE_MODE_DP},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED,
			.lane_mode = {ACIOPHY_LANE_MODE_DP, ACIOPHY_LANE_MODE_USB3},
			.set_swap = true,
		},
	},
	[APPLE_ATCPHY_MODE_USB4] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB4,
			.lane_mode = {ACIOPHY_LANE_MODE_USB4, ACIOPHY_LANE_MODE_USB4},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB4_SWAPPED,
			.lane_mode = {ACIOPHY_LANE_MODE_USB4, ACIOPHY_LANE_MODE_USB4},
			.set_swap = false, /* intentionally false */
		},
	},
	[APPLE_ATCPHY_MODE_DP] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_DP,
			.lane_mode = {ACIOPHY_LANE_MODE_DP, ACIOPHY_LANE_MODE_DP},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_DP,
			.lane_mode = {ACIOPHY_LANE_MODE_DP, ACIOPHY_LANE_MODE_DP},
			.set_swap = false, /* intentionally false */
		},
	},
};

static inline void mask32(void __iomem *reg, u32 mask, u32 set)
{
	u32 value = readl_relaxed(reg);
	value &= ~mask;
	value |= set;
	writel_relaxed(value, reg);
}

static inline void set32(void __iomem *reg, u32 set)
{
	mask32(reg, 0, set);
}

static inline void clear32(void __iomem *reg, u32 clear)
{
	mask32(reg, clear, 0);
}

static void atcphy_apply_tunable(struct apple_atcphy *atcphy,
				 void __iomem *regs,
				 struct atcphy_tunable *tunable)
{
	size_t i;

	for (i = 0; i < tunable->sz; ++i)
		mask32(regs + tunable->values[i].offset,
		       tunable->values[i].mask, tunable->values[i].value);
}

static void atcphy_apply_tunables(struct apple_atcphy *atcphy,
				  enum atcphy_mode mode)
{
	int lane0 = atcphy->swap_lanes ? 1 : 0;
	int lane1 = atcphy->swap_lanes ? 0 : 1;

	atcphy_apply_tunable(atcphy, atcphy->regs.axi2af,
			     &atcphy->tunables.axi2af);
	atcphy_apply_tunable(atcphy, atcphy->regs.core,
			     &atcphy->tunables.common);

	switch (mode) {
	case APPLE_ATCPHY_MODE_USB3:
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_usb3[lane0]);
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_usb3[lane1]);
		break;
	case APPLE_ATCPHY_MODE_USB3_DP:
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_usb3[lane0]);
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_displayport[lane1]);
		break;
	case APPLE_ATCPHY_MODE_DP:
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_displayport[lane0]);
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_displayport[lane1]);
		break;
	case APPLE_ATCPHY_MODE_USB4:
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_usb4[lane0]);
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_usb4[lane1]);
		break;
	case APPLE_ATCPHY_MODE_MAX:
	case APPLE_ATCPHY_MODE_OFF:
	case APPLE_ATCPHY_MODE_USB2:
		break;
	}
}

static void atcphy_setup_pll_fuses(struct apple_atcphy *atcphy)
{
	void __iomem *regs = atcphy->regs.core;

	/* CIO3PLL fuses */

	/* TODO: the mask is one bit wider than the fuse for some reason */
	mask32(regs + CIO3PLL_DCO_NCTRL, CIO3PLL_DCO_COARSEBIN_EFUSE0,
	       FIELD_PREP(CIO3PLL_DCO_COARSEBIN_EFUSE0,
			  atcphy->fuses.cio3pll_dco_coarsebin[0]));
	mask32(regs + CIO3PLL_DCO_NCTRL, CIO3PLL_DCO_COARSEBIN_EFUSE1,
	       FIELD_PREP(CIO3PLL_DCO_COARSEBIN_EFUSE1,
			  atcphy->fuses.cio3pll_dco_coarsebin[1]));
	/* with the cio3pll workaround this fuse will only be a single bit while the mask has two bits */
	mask32(regs + CIO3PLL_FRACN_CAN, CIO3PLL_DLL_CAL_START_CAPCODE,
	       FIELD_PREP(CIO3PLL_DLL_CAL_START_CAPCODE,
			  atcphy->fuses.cio3pll_dll_start_capcode[0]));

	if (atcphy->quirks.t8103_cio3pll_workaround) {
		mask32(regs + AUS_COMMON_SHIM_BLK_VREG, AUS_VREG_TRIM,
		       FIELD_PREP(AUS_VREG_TRIM,
				  atcphy->fuses.aus_cmn_shm_vreg_trim));
		/* the fuse again only has a single bit while the mask allows two */
		mask32(regs + CIO3PLL_FRACN_CAN, CIO3PLL_DLL_CAL_START_CAPCODE,
		       FIELD_PREP(CIO3PLL_DLL_CAL_START_CAPCODE,
				  atcphy->fuses.cio3pll_dll_start_capcode[1]));
		mask32(regs + CIO3PLL_DTC_VREG, CIO3PLL_DTC_VREG_ADJUST,
		       FIELD_PREP(CIO3PLL_DTC_VREG_ADJUST,
				  atcphy->fuses.cio3pll_dtc_vreg_adjust));
	} else {
		mask32(regs + CIO3PLL_DTC_VREG, CIO3PLL_DTC_VREG_ADJUST,
		       FIELD_PREP(CIO3PLL_DTC_VREG_ADJUST,
				  atcphy->fuses.cio3pll_dtc_vreg_adjust));
		mask32(regs + AUS_COMMON_SHIM_BLK_VREG, AUS_VREG_TRIM,
		       FIELD_PREP(AUS_VREG_TRIM,
				  atcphy->fuses.aus_cmn_shm_vreg_trim));
	}

	/* AUSPLL fuses */
	mask32(regs + AUSPLL_DCO_EFUSE_SPARE, AUSPLL_RODCO_ENCAP_EFUSE,
	       FIELD_PREP(AUSPLL_RODCO_ENCAP_EFUSE,
			  atcphy->fuses.auspll_rodco_encap));
	mask32(regs + AUSPLL_DCO_EFUSE_SPARE, AUSPLL_RODCO_BIAS_ADJUST_EFUSE,
	       FIELD_PREP(AUSPLL_RODCO_BIAS_ADJUST_EFUSE,
			  atcphy->fuses.auspll_rodco_bias_adjust));
	mask32(regs + AUSPLL_FRACN_CAN, AUSPLL_DLL_START_CAPCODE,
	       FIELD_PREP(AUSPLL_DLL_START_CAPCODE,
			  atcphy->fuses.auspll_fracn_dll_start_capcode));
	mask32(regs + AUSPLL_CLKOUT_DTC_VREG, AUSPLL_DTC_VREG_ADJUST,
	       FIELD_PREP(AUSPLL_DTC_VREG_ADJUST,
			  atcphy->fuses.auspll_dtc_vreg_adjust));

	/* TODO: is this actually required again? */
	mask32(regs + AUS_COMMON_SHIM_BLK_VREG, AUS_VREG_TRIM,
	       FIELD_PREP(AUS_VREG_TRIM, atcphy->fuses.aus_cmn_shm_vreg_trim));
}

static int atcphy_cio_power_off(struct apple_atcphy *atcphy)
{
	u32 reg;
	int ret;

	/* enable all reset lines */
	clear32(atcphy->regs.core + ATCPHY_POWER_CTRL,
		ATCPHY_POWER_PHY_RESET_N);
	clear32(atcphy->regs.core + ATCPHY_POWER_CTRL,
		ATCPHY_POWER_APB_RESET_N);
	set32(atcphy->regs.core + ATCPHY_POWER_CTRL, ATCPHY_POWER_CLAMP_EN);
	clear32(atcphy->regs.core + ATCPHY_MISC, ATCPHY_MISC_RESET_N);

	// TODO: why clear? is this SLEEP_N? or do we enable some power management here?
	clear32(atcphy->regs.core + ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_BIG);
	ret = readl_relaxed_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT,
					 reg, !(reg & ATCPHY_POWER_SLEEP_BIG),
					 100, 100000);
	if (ret) {
		dev_err(atcphy->dev, "failed to sleep atcphy \"big\"\n");
		return ret;
	}

	clear32(atcphy->regs.core + ATCPHY_POWER_CTRL,
		ATCPHY_POWER_SLEEP_SMALL);
	ret = readl_relaxed_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT,
					 reg, !(reg & ATCPHY_POWER_SLEEP_SMALL),
					 100, 100000);
	if (ret) {
		dev_err(atcphy->dev, "failed to sleep atcphy \"small\"\n");
		return ret;
	}

	return 0;
}

static int atcphy_cio_power_on(struct apple_atcphy *atcphy)
{
	u32 reg;
	int ret;

	set32(atcphy->regs.core + ATCPHY_MISC, ATCPHY_MISC_RESET_N);

	// TODO: why set?! see above
	set32(atcphy->regs.core + ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_SMALL);
	ret = readl_relaxed_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT,
					 reg, reg & ATCPHY_POWER_SLEEP_SMALL,
					 100, 100000);
	if (ret) {
		dev_err(atcphy->dev, "failed to wakeup atcphy \"small\"\n");
		return ret;
	}

	set32(atcphy->regs.core + ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_BIG);
	ret = readl_relaxed_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT,
					 reg, reg & ATCPHY_POWER_SLEEP_BIG, 100,
					 100000);
	if (ret) {
		dev_err(atcphy->dev, "failed to wakeup atcphy \"big\"\n");
		return ret;
	}

	clear32(atcphy->regs.core + ATCPHY_POWER_CTRL, ATCPHY_POWER_CLAMP_EN);
	set32(atcphy->regs.core + ATCPHY_POWER_CTRL, ATCPHY_POWER_APB_RESET_N);

	return 0;
}

static void atcphy_configure_lanes(struct apple_atcphy *atcphy,
				   enum atcphy_mode mode)
{
	const struct atcphy_mode_configuration *mode_cfg;

	if (atcphy->swap_lanes)
		mode_cfg = &atcphy_modes[mode].swapped;
	else
		mode_cfg = &atcphy_modes[mode].normal;

	trace_atcphy_configure_lanes(mode, mode_cfg);

	if (mode_cfg->set_swap)
		set32(atcphy->regs.core + ATCPHY_MISC, ATCPHY_MISC_LANE_SWAP);
	else
		clear32(atcphy->regs.core + ATCPHY_MISC, ATCPHY_MISC_LANE_SWAP);

	mask32(atcphy->regs.core + ACIOPHY_LANE_MODE, ACIOPHY_LANE_MODE_RX0,
	       FIELD_PREP(ACIOPHY_LANE_MODE_RX0, mode_cfg->lane_mode[0]));
	mask32(atcphy->regs.core + ACIOPHY_LANE_MODE, ACIOPHY_LANE_MODE_TX0,
	       FIELD_PREP(ACIOPHY_LANE_MODE_TX0, mode_cfg->lane_mode[0]));
	mask32(atcphy->regs.core + ACIOPHY_LANE_MODE, ACIOPHY_LANE_MODE_RX1,
	       FIELD_PREP(ACIOPHY_LANE_MODE_RX1, mode_cfg->lane_mode[1]));
	mask32(atcphy->regs.core + ACIOPHY_LANE_MODE, ACIOPHY_LANE_MODE_TX1,
	       FIELD_PREP(ACIOPHY_LANE_MODE_TX1, mode_cfg->lane_mode[1]));
	mask32(atcphy->regs.core + ACIOPHY_CROSSBAR, ACIOPHY_CROSSBAR_PROTOCOL,
	       FIELD_PREP(ACIOPHY_CROSSBAR_PROTOCOL, mode_cfg->crossbar));
}

static int atcphy_pipehandler_lock(struct apple_atcphy *atcphy)
{
	int ret;
	u32 reg;

	if (readl_relaxed(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ) &
	    PIPEHANDLER_LOCK_EN)
		dev_warn(atcphy->dev, "pipehandler already locked\n");

	set32(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ,
	      PIPEHANDLER_LOCK_EN);

	ret = readl_relaxed_poll_timeout(
		atcphy->regs.pipehandler + PIPEHANDLER_LOCK_ACK, reg,
		reg & PIPEHANDLER_LOCK_EN, 1000, 1000000);
	if (ret) {
		clear32(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ, 1);
		dev_warn(atcphy->dev, "pipehandler lock not acked\n");
	}

	return ret;
}

static int atcphy_pipehandler_unlock(struct apple_atcphy *atcphy)
{
	int ret;
	u32 reg;

	clear32(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ,
		PIPEHANDLER_LOCK_EN);
	ret = readl_relaxed_poll_timeout(
		atcphy->regs.pipehandler + PIPEHANDLER_LOCK_ACK, reg,
		!(reg & PIPEHANDLER_LOCK_EN), 1000, 1000000);
	if (ret)
		dev_warn(atcphy->dev, "pipehandler lock release not acked\n");

	return ret;
}

static int atcphy_configure_pipehandler(struct apple_atcphy *atcphy,
					enum atcphy_mode mode)
{
	int ret;

	/* Ignore rx detect and valid signals while changing the PHY routing? */
	clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE_VALUES, 14); // TODO: why 14?
	set32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE,
	      PIPEHANDLER_OVERRIDE_RXVALID | PIPEHANDLER_OVERRIDE_RXDETECT);

	/*
	 * this likely locks the mux selection against dwc3's access. changing
	 * the configuration without this lock can lock up both dwc3 and the USB3
	 * PHY. dwc3 must not be softreset but both PHYs should be suspended here.
	 */
	ret = atcphy_pipehandler_lock(atcphy);
	if (ret)
		return ret;

	switch (mode) {
	case APPLE_ATCPHY_MODE_USB3:
	case APPLE_ATCPHY_MODE_USB3_DP:
		/* switch dwc3's superspeed PHY to the real physical PHY */
		mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
		       PIPEHANDLER_CLK_SELECT,
		       FIELD_PREP(PIPEHANDLER_CLK_SELECT,
				  PIPEHANDLER_CLK_USB3PHY));
		mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
		       PIPEHANDLER_MUX_MODE,
		       FIELD_PREP(PIPEHANDLER_MUX_MODE,
				  PIPEHANDLER_MUX_MODE_USB3PHY));

		/* use real rx detect/valid values again */
		clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE,
			PIPEHANDLER_OVERRIDE_RXVALID |
				PIPEHANDLER_OVERRIDE_RXDETECT);
		break;
	default:
		dev_warn(
			atcphy->dev,
			"unknown mode in pipehandler_configure: %d, switching to safe state\n",
			mode);
		fallthrough;
	case APPLE_ATCPHY_MODE_USB2:
	case APPLE_ATCPHY_MODE_OFF:
		/* switch dwc3's superspeed PHY back to the dummy (and also USB4 PHY?) */
		mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
		       PIPEHANDLER_CLK_SELECT,
		       FIELD_PREP(PIPEHANDLER_CLK_SELECT,
				  PIPEHANDLER_CLK_DUMMY_PHY));
		mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
		       PIPEHANDLER_MUX_MODE,
		       FIELD_PREP(PIPEHANDLER_MUX_MODE,
				  PIPEHANDLER_MUX_MODE_DUMMY_PHY));

		/* keep ignoring rx detect and valid values from the USB3/4 PHY? */
		set32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE,
		      PIPEHANDLER_OVERRIDE_RXVALID |
			      PIPEHANDLER_OVERRIDE_RXDETECT);
		break;
	}

	ret = atcphy_pipehandler_unlock(atcphy);
	if (ret)
		return ret;

	return 0;
}

static int atcphy_cio_configure(struct apple_atcphy *atcphy,
				enum atcphy_mode mode)
{
	int ret;

	ret = atcphy_cio_power_on(atcphy);
	if (ret)
		return ret;

	atcphy_setup_pll_fuses(atcphy);
	atcphy_apply_tunables(atcphy, mode);

	// TODO: without this sometimes device aren't recognized but no idea what it does
	// ACIOPHY_PLL_TOP_BLK_AUSPLL_PCTL_FSM_CTRL1.APB_REQ_OV_SEL = 255
	set32(atcphy->regs.core + 0x1014, 255 << 13);
	// AUSPLL_TOP_AUSPLL_APB_CMD_CMDOVERRIDE.APB_OVERRIDE = 1
	set32(atcphy->regs.core + 0x2000, 1 << 28);

	/* enable clocks and configure lanes */
	set32(atcphy->regs.core + CIO3PLL_CLK_CTRL, CIO3PLL_CLK_PCLK_EN);
	set32(atcphy->regs.core + CIO3PLL_CLK_CTRL, CIO3PLL_CLK_REFCLK_EN);
	atcphy_configure_lanes(atcphy, mode);

	/* take the USB3 PHY out of reset and configure the PIPE handler */
	set32(atcphy->regs.core + ATCPHY_POWER_CTRL, ATCPHY_POWER_PHY_RESET_N);
	atcphy_configure_pipehandler(atcphy, mode);

	return 0;
}

static int atcphy_usb3_power_off(struct phy *phy)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	mutex_lock(&atcphy->lock);
	atcphy_configure_pipehandler(atcphy, APPLE_ATCPHY_MODE_USB2);
	atcphy_cio_power_off(atcphy);
	atcphy->mode = APPLE_ATCPHY_MODE_OFF;
	mutex_unlock(&atcphy->lock);

	return 0;
}

static int atcphy_usb3_set_mode(struct phy *phy, enum phy_mode mode,
				int submode)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	trace_atcphy_usb3_set_mode(atcphy, mode, submode);
	/* usb3_support is invariant after _probe and doesn't need to be guarded */
	if (!atcphy->usb3_support)
		return 0;

	/* TODO:
	 * we kinda rely on the fact that switch_set and mux_set will always
	 * be called before we reach this part here. Right now the tipd code
	 * guarantees that we will always win this race because it calls those
	 * two before usb_role_switch (which will then only put work on a queue
	 * which finally calls set_mode) but it's still ugly.
	 */
	mutex_lock(&atcphy->lock);
	if (atcphy->mode == atcphy->target_mode)
		goto out;

	switch (atcphy->target_mode) {
	case APPLE_ATCPHY_MODE_OFF:
	case APPLE_ATCPHY_MODE_USB2:
		/*
		 * no need to do anything since the super-speed PHY has either
		 * neven been brought up or was shut down already when power_off
		 * was called.
		 */
		atcphy->mode = atcphy->target_mode;
		break;
	case APPLE_ATCPHY_MODE_USB3:
		atcphy_cio_configure(atcphy, atcphy->target_mode);
		atcphy->mode = APPLE_ATCPHY_MODE_USB3;
		break;
	case APPLE_ATCPHY_MODE_DP:
	case APPLE_ATCPHY_MODE_USB4:
	case APPLE_ATCPHY_MODE_USB3_DP:
	default:
		dev_warn(
			atcphy->dev,
			"Unknown or unsupported mode %d; falling back to USB2 only\n",
			atcphy->target_mode);
		atcphy->mode = APPLE_ATCPHY_MODE_USB2;
		break;
	}

out:
	mutex_unlock(&atcphy->lock);
	return 0;
}

static const struct phy_ops apple_atc_usb3_phy_ops = {
	.owner = THIS_MODULE,
	.set_mode = atcphy_usb3_set_mode,
	.power_off = atcphy_usb3_power_off,
};

static int atcphy_usb2_power_on(struct phy *phy)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	mutex_lock(&atcphy->lock);

	/* take the PHY out of its low power state */
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_SIDDQ);
	udelay(10);

	/* reset the PHY for good measure */
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
	set32(atcphy->regs.usb2phy + USB2PHY_CTL,
	      USB2PHY_CTL_RESET | USB2PHY_CTL_PORT_RESET);
	udelay(10);
	set32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL,
		USB2PHY_CTL_RESET | USB2PHY_CTL_PORT_RESET);

	set32(atcphy->regs.usb2phy + USB2PHY_SIG,
	      USB2PHY_SIG_VBUSDET_FORCE_VAL | USB2PHY_SIG_VBUSDET_FORCE_EN |
		      USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL |
		      USB2PHY_SIG_VBUSVLDEXT_FORCE_EN);

	/* enable the dummy PHY for the SS lanes */
	set32(atcphy->regs.pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE,
	      PIPEHANDLER_DUMMY_PHY_EN);
	mutex_unlock(&atcphy->lock);

	return 0;
}

static int atcphy_usb2_power_off(struct phy *phy)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	mutex_lock(&atcphy->lock);
	/* reset the PHY before transitioning to low power mode */
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
	set32(atcphy->regs.usb2phy + USB2PHY_CTL,
	      USB2PHY_CTL_RESET | USB2PHY_CTL_PORT_RESET);

	/* switch the PHY to low power mode */
	set32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_SIDDQ);
	mutex_unlock(&atcphy->lock);

	return 0;
}

static int atcphy_usb2_set_mode(struct phy *phy, enum phy_mode mode,
				int submode)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);
	int ret;

	mutex_lock(&atcphy->lock);

	switch (mode) {
	case PHY_MODE_USB_HOST:
	case PHY_MODE_USB_HOST_LS:
	case PHY_MODE_USB_HOST_FS:
	case PHY_MODE_USB_HOST_HS:
	case PHY_MODE_USB_HOST_SS:
		set32(atcphy->regs.usb2phy + USB2PHY_SIG, USB2PHY_SIG_HOST);
		set32(atcphy->regs.usb2phy + USB2PHY_USBCTL,
		      USB2PHY_USBCTL_HOST_EN);
		ret = 0;
		break;

	case PHY_MODE_USB_DEVICE:
	case PHY_MODE_USB_DEVICE_LS:
	case PHY_MODE_USB_DEVICE_FS:
	case PHY_MODE_USB_DEVICE_HS:
	case PHY_MODE_USB_DEVICE_SS:
		clear32(atcphy->regs.usb2phy + USB2PHY_SIG, USB2PHY_SIG_HOST);
		clear32(atcphy->regs.usb2phy + USB2PHY_USBCTL,
			USB2PHY_USBCTL_HOST_EN);
		ret = 0;
		break;

	default:
		dev_err(atcphy->dev, "Unknown mode for usb2 phy: %d\n", mode);
		ret = -EINVAL;
	}

	mutex_unlock(&atcphy->lock);
	return ret;
}

static const struct phy_ops apple_atc_usb2_phy_ops = {
	.owner = THIS_MODULE,
	.set_mode = atcphy_usb2_set_mode,
	/*
	 * This PHY is always matched with a dwc3 controller. Currently,
	 * first dwc3 initializes the PHY and then soft-resets itself and
	 * then finally powers on the PHY. This should be reasonable.
	 * Annoyingly, the dwc3 soft reset is never completed when the USB2 PHY
	 * is powered off so we have to pretend that these two are actually
	 * init/exit here to ensure the PHY is powered on and out of reset
	 * early enough.
	 */
	.init = atcphy_usb2_power_on,
	.exit = atcphy_usb2_power_off,
};

static struct phy *atcphy_xlate(struct device *dev,
				struct of_phandle_args *args)
{
	struct apple_atcphy *atcphy = dev_get_drvdata(dev);

	switch (args->args[0]) {
	case PHY_TYPE_USB2:
		return atcphy->phy_usb2;
	case PHY_TYPE_USB3:
		return atcphy->phy_usb3;
	}
	return ERR_PTR(-ENODEV);
}

static int atcphy_probe_phy(struct apple_atcphy *atcphy)
{
	atcphy->phy_usb2 =
		devm_phy_create(atcphy->dev, NULL, &apple_atc_usb2_phy_ops);
	if (IS_ERR(atcphy->phy_usb2))
		return PTR_ERR(atcphy->phy_usb2);
	phy_set_drvdata(atcphy->phy_usb2, atcphy);

	atcphy->phy_usb3 =
		devm_phy_create(atcphy->dev, NULL, &apple_atc_usb3_phy_ops);
	if (IS_ERR(atcphy->phy_usb3))
		return PTR_ERR(atcphy->phy_usb3);
	phy_set_drvdata(atcphy->phy_usb3, atcphy);

	atcphy->phy_provider =
		devm_of_phy_provider_register(atcphy->dev, atcphy_xlate);
	if (IS_ERR(atcphy->phy_provider))
		return PTR_ERR(atcphy->phy_provider);

	return 0;
}

static int atcphy_dwc3_reset_assert(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct apple_atcphy *atcphy = rcdev_to_apple_atcphy(rcdev);

	clear32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN, DWC3_RESET_N);
	set32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN,
	      DWC3_FORCE_CLAMP_EN);

	return 0;
}

static int atcphy_dwc3_reset_deassert(struct reset_controller_dev *rcdev,
				      unsigned long id)
{
	struct apple_atcphy *atcphy = rcdev_to_apple_atcphy(rcdev);

	clear32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN,
		DWC3_FORCE_CLAMP_EN);
	set32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN, DWC3_RESET_N);

	return 0;
}

const struct reset_control_ops atcphy_dwc3_reset_ops = {
	.assert = atcphy_dwc3_reset_assert,
	.deassert = atcphy_dwc3_reset_deassert,
};

static int atcphy_reset_xlate(struct reset_controller_dev *rcdev,
			      const struct of_phandle_args *reset_spec)
{
	return 0;
}

static int atcphy_probe_rcdev(struct apple_atcphy *atcphy)
{
	atcphy->rcdev.owner = THIS_MODULE;
	atcphy->rcdev.nr_resets = 1;
	atcphy->rcdev.ops = &atcphy_dwc3_reset_ops;
	atcphy->rcdev.of_node = atcphy->dev->of_node;
	atcphy->rcdev.of_reset_n_cells = 0;
	atcphy->rcdev.of_xlate = atcphy_reset_xlate;

	return devm_reset_controller_register(atcphy->dev, &atcphy->rcdev);
}

static int atcphy_sw_set(struct typec_switch *sw,
			 enum typec_orientation orientation)
{
	struct apple_atcphy *atcphy = typec_switch_get_drvdata(sw);

	trace_atcphy_sw_set(orientation);

	mutex_lock(&atcphy->lock);
	switch (orientation) {
	case TYPEC_ORIENTATION_NONE:
		atcphy->target_mode = APPLE_ATCPHY_MODE_OFF;
		break;
	case TYPEC_ORIENTATION_NORMAL:
		atcphy->swap_lanes = false;
		break;
	case TYPEC_ORIENTATION_REVERSE:
		atcphy->swap_lanes = true;
		break;
	}
	mutex_unlock(&atcphy->lock);

	return 0;
}

static int atcphy_probe_switch(struct apple_atcphy *atcphy)
{
	struct typec_switch_desc sw_desc = {
		.drvdata = atcphy,
		.fwnode = atcphy->dev->fwnode,
		.set = atcphy_sw_set,
	};

	return PTR_ERR_OR_ZERO(typec_switch_register(atcphy->dev, &sw_desc));
}

static int atcphy_mux_set(struct typec_mux *mux, struct typec_mux_state *state)
{
	struct apple_atcphy *atcphy = typec_mux_get_drvdata(mux);
	unsigned long mode = state->mode;

	mutex_lock(&atcphy->lock);
	trace_atcphy_mux_set(state);

	if (state->alt != NULL) {
		dev_warn(
			atcphy->dev,
			"Attempted switch to alt mode not suppported; falling back to safe state\n");
		mode = TYPEC_STATE_SAFE;
	}

	if (mode == TYPEC_MODE_USB4) {
		dev_warn(
			atcphy->dev,
			"USB4/usb4 mode is not supported yet; falling back to safe state\n");
		mode = TYPEC_STATE_SAFE;
	}

	if (!atcphy->usb3_support) {
		switch (mode) {
		case TYPEC_MODE_USB3:
		case TYPEC_STATE_USB:
			dev_warn(
				atcphy->dev,
				"No USB3 support; falling back to USB2 only\n");
			mode = TYPEC_MODE_USB2;
			break;
		case TYPEC_MODE_USB2:
		case TYPEC_STATE_SAFE:
			break;
		default:
			dev_warn(
				atcphy->dev,
				"Unsupported mode with only usb2 support (%ld); falling back to safe state\n",
				mode);
			mode = TYPEC_STATE_SAFE;
		}
	}

	switch (mode) {
	case TYPEC_STATE_USB:
	case TYPEC_MODE_USB3:
		atcphy->target_mode = APPLE_ATCPHY_MODE_USB3;
		break;
	case TYPEC_MODE_USB2:
		atcphy->target_mode = APPLE_ATCPHY_MODE_USB2;
		break;
	default:
		dev_err(atcphy->dev,
			"Unknown mode in mux_set (%ld), falling back to safe state\n",
			state->mode);
		fallthrough;
	case TYPEC_STATE_SAFE:
		atcphy->target_mode = APPLE_ATCPHY_MODE_OFF;
		break;
	}

	mutex_unlock(&atcphy->lock);

	return 0;
}

static int atcphy_probe_mux(struct apple_atcphy *atcphy)
{
	struct typec_mux_desc mux_desc = {
		.drvdata = atcphy,
		.fwnode = atcphy->dev->fwnode,
		.set = atcphy_mux_set,
	};

	return PTR_ERR_OR_ZERO(typec_mux_register(atcphy->dev, &mux_desc));
}

static int atcphy_parse_tunable(struct apple_atcphy *atcphy,
				struct atcphy_tunable *tunable,
				const char *name)
{
	struct property *prop;
	const __le32 *p = NULL;
	int i;

	prop = of_find_property(atcphy->np, name, NULL);
	if (!prop) {
		dev_err(atcphy->dev, "tunable %s not found\n", name);
		return -ENOENT;
	}

	if (prop->length % (3 * sizeof(u32)))
		return -EINVAL;

	tunable->sz = prop->length / (3 * sizeof(u32));
	tunable->values = devm_kcalloc(atcphy->dev, tunable->sz,
				       sizeof(*tunable->values), GFP_KERNEL);
	if (!tunable->values)
		return -ENOMEM;

	for (i = 0; i < tunable->sz; ++i) {
		p = of_prop_next_u32(prop, p, &tunable->values[i].offset);
		p = of_prop_next_u32(prop, p, &tunable->values[i].mask);
		p = of_prop_next_u32(prop, p, &tunable->values[i].value);
	}

	trace_atcphy_parsed_tunable(name, tunable);

	return 0;
}

static int atcphy_load_tunables(struct apple_atcphy *atcphy)
{
	int ret;

	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.axi2af,
				   "apple,tunable-axi2af");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.common,
				   "apple,tunable-common");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.lane_usb3[0],
				   "apple,tunable-lane0-usb");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.lane_usb3[1],
				   "apple,tunable-lane1-usb");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.lane_usb4[0],
				   "apple,tunable-lane0-cio");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.lane_usb4[1],
				   "apple,tunable-lane1-cio");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy,
				   &atcphy->tunables.lane_displayport[0],
				   "apple,tunable-lane0-dp");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy,
				   &atcphy->tunables.lane_displayport[1],
				   "apple,tunable-lane1-dp");
	if (ret)
		return ret;

	return 0;
}

static int atcphy_load_fuses(struct apple_atcphy *atcphy)
{
	int ret;

	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "aus_cmn_shm_vreg_trim",
		&atcphy->fuses.aus_cmn_shm_vreg_trim);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "auspll_rodco_encap",
		&atcphy->fuses.auspll_rodco_encap);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "auspll_rodco_bias_adjust",
		&atcphy->fuses.auspll_rodco_bias_adjust);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "auspll_fracn_dll_start_capcode",
		&atcphy->fuses.auspll_fracn_dll_start_capcode);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "auspll_dtc_vreg_adjust",
		&atcphy->fuses.auspll_dtc_vreg_adjust);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "cio3pll_dco_coarsebin0",
		&atcphy->fuses.cio3pll_dco_coarsebin[0]);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "cio3pll_dco_coarsebin1",
		&atcphy->fuses.cio3pll_dco_coarsebin[1]);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "cio3pll_dll_start_capcode",
		&atcphy->fuses.cio3pll_dll_start_capcode[0]);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "cio3pll_dtc_vreg_adjust",
		&atcphy->fuses.cio3pll_dtc_vreg_adjust);
	if (ret)
		return ret;

	/* 
	 * Only one of the two t8103 PHYs requires the following additional fuse
	 * and a slighly different configuration sequence if it's present.
	 * The other t8103 instance and all t6000 instances don't which means
	 * we must not fail here in case the fuse isn't present.
	 */
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "cio3pll_dll_start_capcode_workaround",
		&atcphy->fuses.cio3pll_dll_start_capcode[1]);
	if (ret == -ENOENT) {
		atcphy->quirks.t8103_cio3pll_workaround = false;
		goto success;
	}
	if (ret)
		return ret;

	atcphy->quirks.t8103_cio3pll_workaround = true;
success:
	trace_atcphy_fuses(atcphy);
	return 0;
}

static int atcphy_probe(struct platform_device *pdev)
{
	struct apple_atcphy *atcphy;
	struct device *dev = &pdev->dev;
	int ret;

	atcphy = devm_kzalloc(&pdev->dev, sizeof(*atcphy), GFP_KERNEL);
	if (!atcphy)
		return -ENOMEM;

	atcphy->dev = dev;
	atcphy->np = dev->of_node;
	platform_set_drvdata(pdev, atcphy);

	mutex_init(&atcphy->lock);

	atcphy->regs.core = devm_platform_ioremap_resource_byname(pdev, "core");
	if (IS_ERR(atcphy->regs.core))
		return PTR_ERR(atcphy->regs.core);
	atcphy->regs.axi2af =
		devm_platform_ioremap_resource_byname(pdev, "axi2af");
	if (IS_ERR(atcphy->regs.axi2af))
		return PTR_ERR(atcphy->regs.axi2af);
	atcphy->regs.usb2phy =
		devm_platform_ioremap_resource_byname(pdev, "usb2phy");
	if (IS_ERR(atcphy->regs.usb2phy))
		return PTR_ERR(atcphy->regs.usb2phy);
	atcphy->regs.pipehandler =
		devm_platform_ioremap_resource_byname(pdev, "pipehandler");
	if (IS_ERR(atcphy->regs.pipehandler))
		return PTR_ERR(atcphy->regs.pipehandler);

	atcphy->usb3_support = true;
	ret = atcphy_load_fuses(atcphy);
	if (ret)
		atcphy->usb3_support = false;
	ret = atcphy_load_tunables(atcphy);
	if (ret)
		atcphy->usb3_support = false;
	if (!atcphy->usb3_support)
		dev_warn(
			atcphy->dev,
			"tunables and/or fuses not available; only USB2 will be supported\n");

	atcphy->mode = APPLE_ATCPHY_MODE_OFF;
	atcphy->target_mode = APPLE_ATCPHY_MODE_OFF;

	ret = atcphy_probe_rcdev(atcphy);
	if (ret)
		return ret;
	ret = atcphy_probe_mux(atcphy);
	if (ret)
		return ret;
	ret = atcphy_probe_switch(atcphy);
	if (ret)
		return ret;
	return atcphy_probe_phy(atcphy);
}

static const struct of_device_id atcphy_match[] = {
	{
		.compatible = "apple,t8103-atcphy",
	},
	{},
};
MODULE_DEVICE_TABLE(of, atcphy_match);

static struct platform_driver atcphy_driver = {
	.driver = {
		.name = "phy-apple-atc",
		.of_match_table = atcphy_match,
	},
	.probe = atcphy_probe,
};

module_platform_driver(atcphy_driver);

MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple Type-C PHY driver");

MODULE_LICENSE("GPL");
