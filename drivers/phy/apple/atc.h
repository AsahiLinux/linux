// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Apple Type-C PHY driver
 *
 * Copyright (C) The Asahi Linux Contributors
 * Author: Sven Peter <sven@svenpeter.dev>
 */

#ifndef APPLE_PHY_ATC_H
#define APPLE_PHY_ATC_H 1

#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/usb/typec_mux.h>
#include <linux/reset-controller.h>
#include <linux/types.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_tbt.h>
#include <linux/workqueue.h>

enum atcphy_dp_link_rate {
	ATCPHY_DP_LINK_RATE_RBR,
	ATCPHY_DP_LINK_RATE_HBR,
	ATCPHY_DP_LINK_RATE_HBR2,
	ATCPHY_DP_LINK_RATE_HBR3,
};

enum atcphy_pipehandler_state {
	ATCPHY_PIPEHANDLER_STATE_INVALID,
	ATCPHY_PIPEHANDLER_STATE_USB2,
	ATCPHY_PIPEHANDLER_STATE_USB3,
};

enum atcphy_mode {
	APPLE_ATCPHY_MODE_OFF,
	APPLE_ATCPHY_MODE_USB2,
	APPLE_ATCPHY_MODE_USB3,
	APPLE_ATCPHY_MODE_USB3_DP,
	APPLE_ATCPHY_MODE_USB4,
	APPLE_ATCPHY_MODE_DP,
};

struct atcphy_dp_link_rate_configuration {
	u16 freqinit_count_target;
	u16 fbdivn_frac_den;
	u16 fbdivn_frac_num;
	u16 pclk_div_sel;
	u8 lfclk_ctrl;
	u8 vclk_op_divn;
	bool plla_clkout_vreg_bypass;
	bool bypass_txa_ldoclk;
	bool txa_div2_en;
};

struct atcphy_mode_configuration {
	u32 crossbar;
	u32 crossbar_dp_single_pma;
	bool crossbar_dp_both_pma;
	u32 lane_mode[2];
	bool dp_lane[2];
	bool set_swap;
};

struct atcphy_tunable {
	size_t sz;
	struct {
		u32 offset;
		u32 mask;
		u32 value;
	} * values;
};

struct apple_atcphy {
	struct device_node *np;
	struct device *dev;

	struct {
		unsigned int t8103_cio3pll_workaround : 1;
	} quirks;

	/* calibration fuse values */
	struct {
		bool present;
		u32 aus_cmn_shm_vreg_trim;
		u32 auspll_rodco_encap;
		u32 auspll_rodco_bias_adjust;
		u32 auspll_fracn_dll_start_capcode;
		u32 auspll_dtc_vreg_adjust;
		u32 cio3pll_dco_coarsebin[2];
		u32 cio3pll_dll_start_capcode[2];
		u32 cio3pll_dtc_vreg_adjust;
	} fuses;

	/* tunables provided by firmware through the device tree */
	struct {
		struct atcphy_tunable axi2af;
		struct atcphy_tunable common;
		struct atcphy_tunable lane_usb3[2];
		struct atcphy_tunable lane_displayport[2];
		struct atcphy_tunable lane_usb4[2];
	} tunables;

	bool usb3_power_on;
	bool swap_lanes;

	enum atcphy_mode mode;
	int dp_link_rate;

	struct {
		void __iomem *core;
		void __iomem *axi2af;
		void __iomem *usb2phy;
		void __iomem *pipehandler;
		void __iomem *lpdptx;
	} regs;

	struct phy *phy_usb2;
	struct phy *phy_usb3;
	struct phy *phy_dp;
	struct phy_provider *phy_provider;
	struct reset_controller_dev rcdev;
	struct typec_switch *sw;
	struct typec_mux *mux;

	bool dwc3_online;
	struct completion dwc3_shutdown_event;
	struct completion atcphy_online_event;

	enum atcphy_pipehandler_state pipehandler_state;

	struct mutex lock;

	struct work_struct mux_set_work;
	enum atcphy_mode target_mode;
	bool dp_only;
};

#endif
