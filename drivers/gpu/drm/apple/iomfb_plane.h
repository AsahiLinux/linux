// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright (C) The Asahi Linux Contributors
 */


#ifndef __APPLE_IOMFB_PLANE_H__
#define __APPLE_IOMFB_PLANE_H__

#include <drm/drm_fourcc.h>

#include <linux/types.h>

#define DCP_SURF_MAX_PLANES 3

#define DCP_FORMAT_BGRA		fourcc_code('A', 'R', 'G', 'B')
#define DCP_FORMAT_RGBA		fourcc_code('A', 'B', 'G', 'R')

#define DCP_FORMAT_W30R		fourcc_code('r', '0', '3', 'w')	// wide gamut packed 10-bit RGB without alpha
#define DCP_FORMAT_L10R		fourcc_code('r', '0', '1', 'l')	// full range packed 10-bit RGB with alpha

#define DCP_FORMAT_420V		fourcc_code('v', '0', '2', '4')	// NV12 video range 2 plane 8-bit YCbCr
#define DCP_FORMAT_420F		fourcc_code('f', '0', '2', '4')	// NV12 full range 2 plane 8-bit YCbCr
#define DCP_FORMAT_422V		fourcc_code('v', '2', '2', '4')	// NV16 video range 2 plane 8-bit YCbCr
#define DCP_FORMAT_422F		fourcc_code('f', '2', '2', '4')	// NV16 full range 2 plane 8-bit YCbCr
#define DCP_FORMAT_444V		fourcc_code('v', '4', '4', '4')	// NV24 video range 2 plane 8-bit YCbCr
#define DCP_FORMAT_444F		fourcc_code('f', '4', '4', '4')	// NV24 full range 2 plane 8-bit YCbCr

#define DCP_FORMAT_X420		fourcc_code('0', '2', '4', 'x')	// P010 video range 2 plane 10-bit YCbCR
#define DCP_FORMAT_X422		fourcc_code('2', '2', '4', 'x')	// P210 video range 2 plane 10-bit YCbCR
#define DCP_FORMAT_X444		fourcc_code('4', '4', '4', 'x')	// P410 video range 2 plane 10-bit YCbCR

#define DCP_FORMAT_XF20		fourcc_code('0', '2', 'f', 'x')	// P010 full range 2 plane 10-bit YCbCR
#define DCP_FORMAT_XF22		fourcc_code('2', '2', 'f', 'x')	// P210 full range 2 plane 10-bit YCbCR
#define DCP_FORMAT_XF44		fourcc_code('4', '4', 'f', 'x')	// P410 full range 2 plane 10-bit YCbCR

enum dcp_colorspace {
	DCP_COLORSPACE_BG_SRGB = 0,
	DCP_COLORSPACE_BT601 = 1,
	DCP_COLORSPACE_BT709 = 2,
	DCP_COLORSPACE_BG_BT2020 = 9,
	DCP_COLORSPACE_NATIVE = 12,
};

enum dcp_xfer_func {
	DCP_XFER_FUNC_BT601 = 1,
	DCP_XFER_FUNC_BT1886 = 2,
	DCP_XFER_FUNC_SDR = 13,
	DCP_XFER_FUNC_HDR = 16,
};

struct dcp_rect {
	u32 x;
	u32 y;
	u32 w;
	u32 h;
} __packed;

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
