// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Apple Type-C PHY driver
 *
 * Copyright (C) The Asahi Linux Contributors
 * Author: Sven Peter <sven@svenpeter.dev>
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM appletypecphy

#if !defined(_APPLETYPECPHY_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _APPLETYPECPHY_TRACE_H_

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>
#include "atc.h"

#define show_dp_lr(lr)                                  \
	__print_symbolic(lr, { ATCPHY_DP_LINK_RATE_RBR, "RBR" }, \
			 { ATCPHY_DP_LINK_RATE_HBR, "HBR" },          \
			 { ATCPHY_DP_LINK_RATE_HBR2, "HBR2" },          \
			 { ATCPHY_DP_LINK_RATE_HBR3, "HBR3" })

#define show_sw_orientation(orientation)                                  \
	__print_symbolic(orientation, { TYPEC_ORIENTATION_NONE, "none" }, \
			 { TYPEC_ORIENTATION_NORMAL, "normal" },          \
			 { TYPEC_ORIENTATION_REVERSE, "reverse" })

TRACE_EVENT(atcphy_sw_set, TP_PROTO(enum typec_orientation orientation),
	    TP_ARGS(orientation),

	    TP_STRUCT__entry(__field(enum typec_orientation, orientation)),

	    TP_fast_assign(__entry->orientation = orientation;),

	    TP_printk("orientation: %s",
		      show_sw_orientation(__entry->orientation)));

#define show_mux_state(state)                                                 \
	__print_symbolic(state.mode, { TYPEC_STATE_SAFE, "USB Safe State" }, \
			 { TYPEC_STATE_USB, "USB" })

#define show_atcphy_mode(mode)                                      \
	__print_symbolic(mode, { APPLE_ATCPHY_MODE_OFF, "off" },    \
			 { APPLE_ATCPHY_MODE_USB2, "USB2" },        \
			 { APPLE_ATCPHY_MODE_USB3, "USB3" },        \
			 { APPLE_ATCPHY_MODE_USB3_DP, "DP + USB" }, \
			 { APPLE_ATCPHY_MODE_USB4, "USB4" },        \
			 { APPLE_ATCPHY_MODE_DP, "DP-only" })

TRACE_EVENT(atcphy_usb3_set_mode,
	    TP_PROTO(struct apple_atcphy *atcphy, enum phy_mode mode,
		     int submode),
	    TP_ARGS(atcphy, mode, submode),

	    TP_STRUCT__entry(__field(enum atcphy_mode, mode)
					     __field(enum phy_mode, phy_mode)
						     __field(int, submode)),

	    TP_fast_assign(__entry->mode = atcphy->mode;
			   __entry->phy_mode = mode;
			   __entry->submode = submode;),

	    TP_printk("mode: %s, phy_mode: %d, submode: %d",
		      show_atcphy_mode(__entry->mode), __entry->phy_mode,
		      __entry->submode));

TRACE_EVENT(
	atcphy_configure_lanes,
	TP_PROTO(enum atcphy_mode mode,
		 const struct atcphy_mode_configuration *cfg),
	TP_ARGS(mode, cfg),

	TP_STRUCT__entry(__field(enum atcphy_mode, mode) __field_struct(
		struct atcphy_mode_configuration, cfg)),

	TP_fast_assign(__entry->mode = mode; __entry->cfg = *cfg;),

	TP_printk(
		"mode: %s, crossbar: 0x%02x, lanes: {0x%02x, 0x%02x}, swap: %d",
		show_atcphy_mode(__entry->mode), __entry->cfg.crossbar,
		__entry->cfg.lane_mode[0], __entry->cfg.lane_mode[1],
		__entry->cfg.set_swap));

TRACE_EVENT(atcphy_mux_set, TP_PROTO(struct typec_mux_state *state),
	    TP_ARGS(state),

	    TP_STRUCT__entry(__field_struct(struct typec_mux_state, state)),

	    TP_fast_assign(__entry->state = *state;),

	    TP_printk("state: %s", show_mux_state(__entry->state)));

TRACE_EVENT(atcphy_parsed_tunable,
	    TP_PROTO(const char *name, struct atcphy_tunable *tunable),
	    TP_ARGS(name, tunable),

	    TP_STRUCT__entry(__field(const char *, name)
				     __field(size_t, sz)),

	    TP_fast_assign(__entry->name = name; __entry->sz = tunable->sz;),

	    TP_printk("%s with %zu entries", __entry->name,
		      __entry->sz));

TRACE_EVENT(
	atcphy_fuses, TP_PROTO(struct apple_atcphy *atcphy), TP_ARGS(atcphy),
	TP_STRUCT__entry(__field(struct apple_atcphy *, atcphy)),
	TP_fast_assign(__entry->atcphy = atcphy;),
	TP_printk(
		"aus_cmn_shm_vreg_trim: 0x%02x; auspll_rodco_encap: 0x%02x; auspll_rodco_bias_adjust: 0x%02x; auspll_fracn_dll_start_capcode: 0x%02x; auspll_dtc_vreg_adjust: 0x%02x; cio3pll_dco_coarsebin: 0x%02x, 0x%02x; cio3pll_dll_start_capcode: 0x%02x, 0x%02x; cio3pll_dtc_vreg_adjust: 0x%02x",
		__entry->atcphy->fuses.aus_cmn_shm_vreg_trim,
		__entry->atcphy->fuses.auspll_rodco_encap,
		__entry->atcphy->fuses.auspll_rodco_bias_adjust,
		__entry->atcphy->fuses.auspll_fracn_dll_start_capcode,
		__entry->atcphy->fuses.auspll_dtc_vreg_adjust,
		__entry->atcphy->fuses.cio3pll_dco_coarsebin[0],
		__entry->atcphy->fuses.cio3pll_dco_coarsebin[1],
		__entry->atcphy->fuses.cio3pll_dll_start_capcode[0],
		__entry->atcphy->fuses.cio3pll_dll_start_capcode[1],
		__entry->atcphy->fuses.cio3pll_dtc_vreg_adjust));



TRACE_EVENT(atcphy_dp_configure,
	    TP_PROTO(struct apple_atcphy *atcphy, enum atcphy_dp_link_rate lr),
	    TP_ARGS(atcphy, lr),

	    TP_STRUCT__entry(__string(devname, dev_name(atcphy->dev))
				     __field(enum atcphy_dp_link_rate, lr)),

	    TP_fast_assign(__assign_str(devname, dev_name(atcphy->dev));
	     		  __entry->lr = lr;),

	    TP_printk("%s: link rate: %s", __get_str(devname),
		      show_dp_lr(__entry->lr)));

#endif /* _APPLETYPECPHY_TRACE_H_ */

/* This part must be outside protection */
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#include <trace/define_trace.h>
