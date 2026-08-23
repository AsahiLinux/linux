// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

/*
 * This file is intended to be included multiple times with IOMFB_VER
 * defined to declare DCP firmware version dependent structs.
 */

#ifdef DCP_FW_VER

#include <drm/drm_crtc.h>

#include <linux/types.h>

#include "iomfb.h"
#include "iomfb_plane.h"
#include "plane.h"
#include "version_utils.h"

struct DCP_FW_NAME(dcp_swap) {
	u64 ts1;
	u64 ts2;

	u64 unk_10;
	u64 unk_18;
	u64 ts64_unk;
	u64 unk_28;
	u64 ts3;
	u64 unk_38;

	u64 flags1;
	u64 flags2;
#if DCP_FW_VER >= DCP_FW_VERSION(14, 7, 0)
	u8 unk_v14_7[0x48];
#endif
	u32 swap_id;

	u32 surf_ids[SWAP_SURFACES];
	struct dcp_rect src_rect[SWAP_SURFACES];
	u32 surf_flags[SWAP_SURFACES];
	u32 surf_unk[SWAP_SURFACES];
	struct dcp_rect dst_rect[SWAP_SURFACES];
	u32 swap_enabled;
	u32 swap_completed;

	u32 bg_color;
	u8 unk_110[0x30];
	u32 active_region_en[SWAP_SURFACES];
	struct dcp_rect active_regions[SWAP_SURFACES];
	u8 unk_190[0x138];
	u32 unk_2c8;
#if DCP_FW_VER < DCP_FW_VERSION(14, 7, 0)
	u8 unk_2cc[0x14];
#else
	u8 unk_2cc[0x40];
#endif
#if DCP_FW_VER < DCP_FW_VERSION(14, 7, 0)
	u32 unk_2e0;
#else
	u32 bl_update;
#endif
#if DCP_FW_VER < DCP_FW_VERSION(13, 2, 0)
	u16 unk_2e2;
#else
	u8 unk_2e2[3];
#endif
#if DCP_FW_VER < DCP_FW_VERSION(14, 7 ,0)
	u64 bl_unk;
#else
	u32 bl_unk;
#endif
	u32 bl_value; // min value is 0x10000000
	u8  bl_power; // constant 0x40 for on
	u8 unk_2f3[0x2d];
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u8 unk_320[0x13f];
#if DCP_FW_VER >= DCP_FW_VERSION(14, 7, 0)
	u8 unk_14_7_2[0x30];
#endif
	u32 unk_flags;
	u32 unk_flags2;
#endif
} __packed;

/* Information describing a surface */
struct DCP_FW_NAME(dcp_surface) {
	struct dcp_surface base;
#if DCP_FW_VER < DCP_FW_VERSION(13, 2, 0)
	u8 padding[7];
#else
	u8 padding[47];
#endif
} __packed;

/* Prototypes */

struct DCP_FW_NAME(dcp_swap_submit_req) {
	struct DCP_FW_NAME(dcp_swap) swap;
	struct DCP_FW_NAME(dcp_surface) surf[SWAP_SURFACES];
	u64 surf_iova[SWAP_SURFACES];
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u64 unk_u64_a[SWAP_SURFACES];
	struct DCP_FW_NAME(dcp_surface) surf2[5];
	u64 surf2_iova[5];
#endif
	u8 unkbool;
	u64 unkdouble;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u64 unkU64;
	u8 unkbool2;
#endif
	u32 clear; // or maybe switch to default fb?
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u32 unkU32Ptr;
#endif
	u8 swap_null;
	u8 surf_null[SWAP_SURFACES];
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u8 surf2_null[5];
#endif
	u8 unkoutbool_null;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u8 unkU32Ptr_null;
	u8 unkU32out_null;
#endif
	u8 padding[1];
#if DCP_FW_VER >= DCP_FW_VERSION(14, 7, 0)
	u8 padding_14_7[0x1e9];
	u8 unk_14_7_zero[0x46];
	u32 unk_14_7_u32;
	u8 unk_bool;
#endif
} __packed;

struct DCP_FW_NAME(dcp_swap_submit_resp) {
	u8 unkoutbool;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u32 unkU32out;
#endif
	u32 ret;
	u8 padding[3];
} __packed;

struct DCP_FW_NAME(dc_swap_complete_resp) {
	u32 swap_id;
	u8 unkbool;
	u64 swap_data;
#if DCP_FW_VER < DCP_FW_VERSION(13, 2, 0)
	u8 swap_info[0x6c4];
#else
	u8 swap_info[0x6c5];
#endif
	u32 unkint;
	u8 swap_info_null;
} __packed;

struct DCP_FW_NAME(dcp_map_reg_req) {
	char obj[4];
	u32 index;
	u32 flags;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u8 unk_u64_null;
#endif
	u8 addr_null;
	u8 length_null;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u8 padding[1];
#else
	u8 padding[2];
#endif
} __packed;

struct DCP_FW_NAME(dcp_map_reg_resp) {
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u64 dva;
#endif
	u64 addr;
	u64 length;
	u32 ret;
} __packed;


struct apple_dcp;

int DCP_FW_NAME(iomfb_modeset)(struct apple_dcp *dcp,
			       struct drm_crtc_state *crtc_state);
void DCP_FW_NAME(iomfb_flush)(struct apple_dcp *dcp, struct drm_crtc *crtc, struct drm_atomic_commit *commit);
void DCP_FW_NAME(iomfb_poweron)(struct apple_dcp *dcp);
void DCP_FW_NAME(iomfb_poweroff)(struct apple_dcp *dcp);
void DCP_FW_NAME(iomfb_sleep)(struct apple_dcp *dcp);
void DCP_FW_NAME(iomfb_start)(struct apple_dcp *dcp);
void DCP_FW_NAME(iomfb_shutdown)(struct apple_dcp *dcp);

#endif
