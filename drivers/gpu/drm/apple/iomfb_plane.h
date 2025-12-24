// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright (C) The Asahi Linux Contributors
 */


#ifndef __APPLE_IOMFB_PLANE_H__
#define __APPLE_IOMFB_PLANE_H__

#include <linux/types.h>

#define DCP_SURF_MAX_PLANES 3

/* Information describing a plane of a planar compressed surface */
struct dcp_plane_info {
	u32 width;
	u32 height;
	u32 base;
	u32 offset;
	u32 stride;
	u32 size;
	u16 tile_size;
	u8 tile_w;
	u8 tile_h;
	u32 unk[13];
} __packed;

struct dcp_component_types {
	u8 count;
	u8 types[7];
} __packed;

/* Information describing a surface */
struct dcp_surface {
	u8 is_tiled;
	u8 is_tearing_allowed;
	u8 is_premultiplied;
	u32 plane_cnt;
	u32 plane_cnt2;
	u32 format; /* DCP fourcc */
	u32 ycbcr_matrix;
	u8 xfer_func;
	u8 colorspace;
	u32 stride;
	u16 pix_size;
	u8 pel_w;
	u8 pel_h;
	u32 offset;
	u32 width;
	u32 height;
	u32 buf_size;
	u64 protection_opts;
	u32 surface_id;
	struct dcp_component_types comp_types[DCP_SURF_MAX_PLANES];
	u64 has_comp;
	struct dcp_plane_info planes[DCP_SURF_MAX_PLANES];
	u64 has_planes;
	u32 compression_info[DCP_SURF_MAX_PLANES][13];
	u64 has_compr_info;
	u32 unk_num;
	u32 unk_denom;
} __packed;

#endif /* __APPLE_IOMFB_PLANE_H__ */
