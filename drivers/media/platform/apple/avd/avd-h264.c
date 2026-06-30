/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple AVD VPU codec driver
 *
 * Copyright The Asahi Linux Contributors
 * Copyright 2023 Eileen Yoon <eyn@gmx.com>
 *
 * Copyright (c) 2014 Rockchip Electronics Co., Ltd.
 *	Hertz Wong <hertz.wong@rock-chips.com>
 *	Herman Chen <herman.chen@rock-chips.com>
 *
 * Copyright (C) 2014 Google, Inc.
 *	Tomasz Figa <tfiga@chromium.org>
 */

#include "linux/dev_printk.h"
#include <linux/types.h>
#include <linux/iopoll.h>

#include <media/v4l2-h264.h>
#include <media/videobuf2-dma-contig.h>

#include "avd.h"
#include "avd-inst.h"

struct avd_h264_run {
	struct avd_run base;

	const struct v4l2_ctrl_h264_decode_params *decode_params;
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_slice_params *slice_params;

	const struct v4l2_ctrl_h264_scaling_matrix *scaling_matrix;
	const struct v4l2_ctrl_h264_pred_weights *pred_weights;

	struct run_addr {
		dma_addr_t y;
		dma_addr_t uv;
		dma_addr_t sl;
		dma_addr_t rvra;
		dma_addr_t sps;
	} addresses;

	s32 cur_poc;
	u8 num_valid;
};

/* state */
struct avd_h264_ctx {
	struct avd_h264_reflists {
		struct v4l2_h264_reference p[V4L2_H264_REF_LIST_LEN];
		struct v4l2_h264_reference b0[V4L2_H264_REF_LIST_LEN];
		struct v4l2_h264_reference b1[V4L2_H264_REF_LIST_LEN];
	} reflists;

	struct avd_h264_bufs {
		struct avd_buf pps_tile[5];
		struct avd_buf inst;
		struct avd_buf pipe_state;
	} bufs;
};

/* ffmpeg submits wrong timestamp, use first_mb_in_slice as a workaround */
#define is_new_frame(sl) (sl->first_mb_in_slice == 0) /* (ctx->fh.m2m_ctx->new_frame) */


/* scaling matrix */
static const u32 default_8x8_intra[] = {
	0x060a0d10, 0x0a0b1012, 0x0d101217, 0x10121719, 0x1217191b, 0x17191b1d,
	0x191b1d1f, 0x1b1d1f21, 0x1217191b, 0x17191b1d, 0x191b1d1f, 0x1b1d1f21,
	0x1d1f2124, 0x1f212426, 0x21242628, 0x2426282a,
};
static const u32 default_8x8_inter[] = {
	0x090d0f11, 0x0d0d1113, 0x0f111315, 0x11131516, 0x13151618, 0x15161819,
	0x1618191b, 0x18191b1c, 0x13151618, 0x15161819, 0x1618191b, 0x18191b1c,
	0x191b1c1e, 0x1b1c1e20, 0x1c1e2021, 0x1e202123,
};


static inline u32 sps_size(u32 w, u32 h)
{
	/* TODO: does it really need so much? */
	return (((w - 1) * (h - 1) / 0x10000) + 2) * 0x4000;
}

/* sorry for the formatting */

/* clang-format off */
static void stream_refs(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	const struct v4l2_ctrl_h264_decode_params *decode = run->decode_params;
	const struct v4l2_h264_dpb_entry *dpb = decode->dpb;
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct avd_dev *avd = ctx->dev;

	push(INST_DMA2, "cm3_dma_config_6");
	pusha(h264_ctx->bufs.pps_tile[4].addr, "hdr_9c_pps_tile_addr_lsb8", 7);
	pusha(run->addresses.sps, "hdr_bc_sps_tile_addr_lsb8", 0);

	push(INST_DMA3, "cm3_dma_config_7");
	push(INST_DMA3, "cm3_dma_config_8");
	push(INST_DMA3, "cm3_dma_config_9");
	push(INST_DMA3, "cm3_dma_config_a");

	for (int i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		if (!(dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;

		struct vb2_buffer *vb = vb2_find_buffer(
			&ctx->fh.m2m_ctx->cap_q_ctx.q, dpb[i].reference_ts);

		dma_addr_t rvra_addr = vb
			? vb2_dma_contig_plane_dma_addr(vb, 0)
				+ (vb->planes[0].length
						- sps_size(fmt_width(ctx), fmt_height(ctx))
						- ctx->rvra.size)
			: run->addresses.rvra; /* safe fallback */

		push(((run->num_valid - 1) & 0xf) << 28
				| 0x1000000
				| (boolify(dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) << 17)
				| (swrap(run->cur_poc - dpb[i].top_field_order_cnt, 1 << 17)),
				"hdr_d0_ref_hdr");

		push_rvra(avd, ctx, rvra_addr, ctx->rvra.offsets);
	}
}

static void stream_scaling(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	const struct v4l2_ctrl_h264_scaling_matrix *scaling = run->scaling_matrix;
	struct avd_dev *avd = ctx->dev;

	push(0x1000000 | ((64 / 4) << 5)
		| (((16 / 4) << 5) - 1), "hdr_4c_pic_scaling_list_dims");

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 16; j+=4)
			push((scaling->scaling_list_4x4[i][j + 0] << 24)
					| (scaling->scaling_list_4x4[i][j + 1] << 16)
					| (scaling->scaling_list_4x4[i][j + 2] << 8)
					| (scaling->scaling_list_4x4[i][j + 3]),
					"scl_46c_pic_scaling_matrix_4x4");

	/* Instead of 8x8 raster scan order avd expects 4 4x4 subblocks */
	static const u8 map[16] = {
		  0,  8, 16, 24, /* top left */
		  4, 12, 20, 28, /* top right */
		 32, 40, 48, 56, /* bottom left */
		 36, 44, 52, 60, /* bottom right */
	};

	/* 7.3.2.2, only matrix 0 and 1 are used if chroma_format_idc < 3 */
	if (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) {
		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 16; j++)
				push((scaling->scaling_list_8x8[i][map[j] + 0] << 24)
						| (scaling->scaling_list_8x8[i][map[j] + 1] << 16)
						| (scaling->scaling_list_8x8[i][map[j] + 2] << 8)
						| (scaling->scaling_list_8x8[i][map[j] + 3]),
						"scl_4cc_pic_scaling_matrix_8x8");

	} else {
		for (int i = 0; i < ARRAY_SIZE(default_8x8_intra); i++)
			push(default_8x8_intra[i], "scl_4cc_pic_scaling_matrix_8x8");
		for (int i = 0; i < ARRAY_SIZE(default_8x8_inter); i++)
			push(default_8x8_inter[i], "scl_4cc_pic_scaling_matrix_8x8");
	}
}

static void stream_hdr(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	const struct v4l2_ctrl_h264_decode_params *decode = run->decode_params;
	const struct v4l2_ctrl_h264_sps *sps = run->sps;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	struct avd_dev *avd = ctx->dev;
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	u32 bytesperline;
	u32 width = (sps->pic_width_in_mbs_minus1 + 1) * 16;
	u32 height = (sps->pic_height_in_map_units_minus1 + 1) * 16;

	push(0x2b000000
			| (ctx->fifo_idx << 4)
			| (avd->variant->revision == 3 ? 0x100 : 0x200),
			"inst_fifo_start");

	push(0x2db00000
			| 0x1000
			| ((decode->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) ? 0x2000 : 0)
			| 0x2e0
			| (avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE ? 0 : 0x80000)
			, "hdr_34_start_hdr");

	push(AVD_CODEC_H264 << 24, "hdr_38_mode");

	push(((height - 1) << 16) | (width - 1), "hdr_3c_height_width");

	push(0, "hdr_40_zero");

	push((((height - 1) >> 3) << 16) | ((width - 1) >> 3),
			"hdr_28_height_width_shift3");

	push((sps->chroma_format_idc & 3) << 24
		| (sps->bit_depth_luma_minus8 & 15) << 19
		| (sps->bit_depth_chroma_minus8 & 15) << 15
		| 0x2800
		| boolify(pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) << 7
		| boolify(sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE),
		"hdr_2c_sps_param");

	push(boolify(pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) << 20
		| !boolify(decode->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) << 21
		| boolify(pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED) << 19,
		"hdr_44_flags");


	push(swrap(pps->chroma_qp_index_offset, 32) << 5
		| swrap(pps->second_chroma_qp_index_offset, 32)
		, "hdr_48_chroma_qp_index_offset");

    push(0x30000a
			| (avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE ? 0 : 0x30),
			"hdr_58_const_3a");

	push(INST_DMA2, "cm3_dma_config_1");
	push(INST_DMA1, "cm3_dma_config_2");

	if (avd->variant->revision == 3)
		push(0, "zero");

	pusha(h264_ctx->bufs.pps_tile[0].addr, "hdr_9c_pps_tile_addr_lsb8", 0);

	push(INST_DMA2, "cm3_dma_config_3");
	push(INST_DMA2, "cm3_dma_config_4");

	if (avd->variant->revision == 3)
		push(0, "zero");
	else if (!(avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE))
		pusha(h264_ctx->bufs.pipe_state.addr, "pipe_state", 0);

	pusha(h264_ctx->bufs.pps_tile[1].addr, "hdr_9c_pps_tile_addr_lsb8", 1);
	pusha(h264_ctx->bufs.pps_tile[2].addr, "hdr_9c_pps_tile_addr_lsb8", 2);
	pusha(h264_ctx->bufs.pps_tile[3].addr, "hdr_9c_pps_tile_addr_lsb8", 3);
	push(INST_DMA3, "cm3_dma_config_5");

	push_rvra(avd, ctx, run->addresses.rvra, ctx->rvra.offsets);

	bytesperline = ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
	if (avd->variant->quirks & AVD_QUIRK_LSR)
		bytesperline = bytesperline >> 4;

	pusha(run->addresses.y, "hdr_210_y_addr_lsb8", 0);
	push(bytesperline, "hdr_218_width_align");
	pusha(run->addresses.uv,"hdr_214_uv_addr_lsb8", 0);
	push(bytesperline, "hdr_21c_width_align");

	push(0x0, "cm3_mark_end_section");
	push(((height - 1) << 16) | (width - 1), "hdr_54_height_width");

	if (!(decode->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC))
		stream_refs(ctx, run);

	if (pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT)
		stream_scaling(ctx, run);
	else
		push(0x0, "cm3_mark_end_section_scl");
}

static void stream_weights(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	int default_luma_weight, default_chroma_weight;
	struct v4l2_h264_weight_factors factors;
	const struct v4l2_ctrl_h264_pred_weights *weights = run->pred_weights;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	const struct v4l2_ctrl_h264_slice_params *sl = run->slice_params;
	struct avd_dev *avd = ctx->dev;

	bool pred_weight = V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(pps, sl);

	/* TODO: is there a better flag or something for the
	 * pps->weighted_bipred_idc == 2 checks? */
	push(0x2dd00000
		| ((pps->weighted_bipred_idc == 2) << 7)
		| (pred_weight << 6)
		| (weights->luma_log2_weight_denom << 3)
		| (weights->chroma_log2_weight_denom)
		/* default luma and chroma denom */
		| (pps->weighted_bipred_idc == 2 ? 0x5 | 0x5 << 3 : 0),
		"slc_76c_cmd_weights_denom");

	if (!pred_weight)
		return;

	default_luma_weight = 1 << weights->luma_log2_weight_denom;
	default_chroma_weight = 1 << weights->chroma_log2_weight_denom;


	for (int y = 0; y < 2; y++) {
		if (y == 1 && sl->slice_type != V4L2_H264_SLICE_TYPE_B)
			break;

		factors = weights->weight_factors[y];
		int to = y == 0 ? sl->num_ref_idx_l0_active_minus1
						: sl->num_ref_idx_l1_active_minus1;
		for (int i = 0; i < to + 1; i++) {
			/* Avd only expects offsets/weights if they are not the default
			 * ones, otherwise we get artifacts */
			if((factors.luma_weight[i] != default_luma_weight)
					|| (factors.luma_offset[i] != 0)) {
				push(0x2de00000
						| 1 << 14
						| y << 13
						| i << 9
						| ((factors.luma_weight[i] & 0x1ff)),
					"slc_luma_weights");
				push(0x2df00000
						| swrap(factors.luma_offset[i], 0x10000),
						"slc_luma_offsets");
			}

			if ((factors.chroma_weight[i][0] != default_chroma_weight)
					|| (factors.chroma_offset[i][0] != 0)
					|| (factors.chroma_weight[i][1] != default_chroma_weight)
					|| (factors.chroma_offset[i][1] != 0)) {
				push(0x2de00000
						| 2 << 14
						| y << 13
						| i << 9
						| (factors.chroma_weight[i][0] & 0x1ff),
						"slc_chroma_weights[0]");
				push(0x2df00000
						| swrap(factors.chroma_offset[i][0], 0x10000),
						"slc_chroma_offsets[0]");
				push(0x2de00000
						| 3 << 14
						| y << 13
						| i << 9
						| (factors.chroma_weight[i][1] & 0x1ff),
						"slc_chroma_weights[1]");
				push(0x2df00000
						| swrap(factors.chroma_offset[i][1], 0x10000),
						"slc_chroma_offsets[1]");
			}
		}
	}
}

static u32 stream_slice(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	const struct v4l2_ctrl_h264_decode_params *decode = run->decode_params;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	const struct v4l2_ctrl_h264_sps *sps = run->sps;
	const struct v4l2_ctrl_h264_slice_params *sl = run->slice_params;
	struct avd_dev *avd = ctx->dev;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	u32 payload_len = vb2_get_plane_payload(&src->vb2_buf, 0);
	bool en_mode = (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) ==
		       0;
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 x;

	u32 min_off = (sl->header_bit_size + (en_mode ? 0 : 7)) / 8;

	/* emulation byte. header_bit_size does not include this.
	 * I wonder if its worth it to just not support this. */
	u32 off = 2;
	u32 bytes_read = 2;

	while (bytes_read < min_off) {
		if (data[off - 2] != 0x00 || data[off - 1] != 0x00 ||
		    data[off] != 0x03)
			bytes_read++;
		off++;
	}


	dma_addr_t slc_a84 = run->addresses.sl + off;

	push(0x2d800000
			| ((en_mode ? (sl->header_bit_size % 8) : 0) << 15)
			| (u32)(slc_a84 >> 32), "slc_a7c_cmd_set_coded_slice");
	push((u32)(slc_a84 & 0xffffffff), "slc_a84_slice_addr_low");

	/* should not include trailing 0? */
	push(payload_len - off, "slc_a88_slice_hdr_size");

	push(0x2c000000
			| (sl->first_mb_in_slice / (sps->pic_width_in_mbs_minus1 + 1) << 12)
			|  sl->first_mb_in_slice % (sps->pic_width_in_mbs_minus1 + 1),
			"cm3_cmd_exec_mb_vp");

	push(0x2d900000
		| ( ((26 + pps->pic_init_qp_minus26 + sl->slice_qp_delta) * 0x400)
				& 0x1fc00),
			"slc_a70_cmd_quant_param");

	push(0x2da00000
			| (u32)(sl->disable_deblocking_filter_idc == 0
				? BIT(17) : 0)
			| (u32)(sl->disable_deblocking_filter_idc != 1
				? BIT(16)
				| swrap(sl->slice_beta_offset_div2, 16) << 12
				| swrap(sl->slice_alpha_c0_offset_div2, 16) << 8
				: 0),
			"slc_a74_cmd_deblocking_filter");

	if (sl->slice_type == V4L2_H264_SLICE_TYPE_P
			|| sl->slice_type == V4L2_H264_SLICE_TYPE_B) {

		u32 num_ref_idx_active = sl->num_ref_idx_l0_active_minus1 + 1;
		for (u32 i = 0; i < num_ref_idx_active; i++)
			push(0x2dc00000
			   | (0 << 8)
			   | ((i & 0xf) << 4)
			   | (sl->ref_pic_list0[i].index & 0xf),
			   "slc_6e8_cmd_ref_list_0");

		if (sl->slice_type == V4L2_H264_SLICE_TYPE_B) {
			u32 num_ref_idx_active = sl->num_ref_idx_l1_active_minus1 + 1;
			for (u32 i = 0; i < num_ref_idx_active; i++)
				push(0x2dc00000
						| (1 << 8)
						| ((i & 0xf) << 4)
						| (sl->ref_pic_list1[i].index & 0xf),
						"slc_6e8_cmd_ref_list_0");
		}
		stream_weights(ctx, run);
	}

	if (sl->first_mb_in_slice == 0) {
		push(0x2a000000, "cm3_cmd_set_mb_dims");
		push(((sps->pic_height_in_map_units_minus1) << 12)
				| (sps->pic_width_in_mbs_minus1), "cm3_set_mb_dims");
	}

	x = 0;
	if (sl->slice_type == V4L2_H264_SLICE_TYPE_I)
		x |= 0x20000;
	else if (sl->slice_type == V4L2_H264_SLICE_TYPE_P)
		x |= 0x10000;
	else if (sl->slice_type == V4L2_H264_SLICE_TYPE_B)
		x |= 0x40000;

	if ((sl->slice_type == V4L2_H264_SLICE_TYPE_P) ||
	    (sl->slice_type == V4L2_H264_SLICE_TYPE_B)) {
		if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
			x |= sl->cabac_init_idc << 5;

		if (sl->slice_type == V4L2_H264_SLICE_TYPE_B) {
			x |= sl->num_ref_idx_l1_active_minus1 << 7;
			if (!(sl->flags &
			      V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED))
				x |= BIT(15);
		}
		x |= sl->num_ref_idx_l0_active_minus1 << 11;
	}
	push(0x2d000000 | x, "slc_6e4_cmd_ref_type");

	if (sl->slice_type == V4L2_H264_SLICE_TYPE_B) {
		/* bidirectional reference of previous mv */
		struct vb2_buffer *vb = vb2_find_buffer(
				&ctx->fh.m2m_ctx->cap_q_ctx.q,
				 decode->dpb[sl->ref_pic_list1[0].index].reference_ts);

		dma_addr_t sps_tile_addr = vb
			? vb2_dma_contig_plane_dma_addr(vb, 0)
				+ (vb->planes[0].length - sps_size(fmt_width(ctx), fmt_height(ctx)))
			: run->addresses.sps;

		pusha(sps_tile_addr, "slc_a78_sps_tile_addr2_lsb8", 0);
	}

	/* only submit if this is the last slice */
	push(0x2b000000
			| !(src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF) << 10,
			"cm3_cmd_inst_fifo_end");

	return payload_len - off;
}
/* clang-format on */

static int avd_h264_alloc_bufs(struct avd_ctx *ctx)
{
	struct avd_dev *dev = ctx->dev;
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	int ret;

	ret = avd_buf_alloc(dev, &h264_ctx->bufs.inst, fifo_size());
	if (ret) {
		dev_err(dev->dev, "inst alloc failed\n");
		return ret;
	}

	if (!(dev->variant->quirks & AVD_QUIRK_NO_PIPE_STATE)) {
		ret = avd_buf_alloc(dev, &h264_ctx->bufs.pipe_state, 0x200);
		if (ret) {
			dev_err(dev->dev, "pipe state alloc failed\n");
			return ret;
		}
	}

	/* TODO: VERY ugly
	 * Does it actually need this much?
	 * */
	for (int i = 0; i < 5; i++) {
		u32 mul = fmt_width(ctx) / 16;
		u32 size = i == 0 ? 0x20000 :
			   i == 1 ? (16 + 8 + 8) * mul :
			   i == 2 ? ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline > 2048 ?
				    0xc000 :
				    (64 + 1 * 32 + 1 * 32) * mul :
				    32 * mul;
		ret = avd_buf_alloc(dev, &h264_ctx->bufs.pps_tile[i],
				    max(size, 0x8000));
		if (ret) {
			dev_err(dev->dev, "pps[%d] alloc failed\n", i);
			return ret;
		}
	}

	return 0;
}

static void avd_h264_free_bufs(struct avd_ctx *ctx)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct avd_dev *dev = ctx->dev;

	if (!h264_ctx)
		return;

	if (!(dev->variant->quirks & AVD_QUIRK_NO_PIPE_STATE))
		avd_buf_free(dev, &h264_ctx->bufs.pipe_state);
	avd_buf_free(dev, &h264_ctx->bufs.inst);

	for (int i = 0; i < 5; i++)
		avd_buf_free(dev, &h264_ctx->bufs.pps_tile[i]);

	kfree(h264_ctx);
}

static int avd_h264_validate_pps(struct avd_ctx *ctx,
				 const struct v4l2_ctrl_h264_pps *pps)
{
	if (pps->num_slice_groups_minus1 != 0) {
		dev_err(ctx->dev->dev, "pps->num_slice_groups_minus1 != 0");
		return -EINVAL;
	}

	return 0;
}

static int avd_h264_validate_sps(struct avd_ctx *ctx,
				 const struct v4l2_ctrl_h264_sps *sps)
{
	if (sps->chroma_format_idc > 2)
		/* Only 4:0:0, 4:2:0 and 4:2:2 are supported */
		return -EINVAL;
	if (sps->bit_depth_luma_minus8 != sps->bit_depth_chroma_minus8)
		/* Luma and chroma bit depth mismatch */
		return -EINVAL;
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
		/* no interlaced support */
		return -EINVAL;

	return 0;
}

static int avd_h264_start(struct avd_ctx *ctx)
{
	struct avd_h264_ctx *h264_ctx;
	struct v4l2_ctrl *ctrl;
	int ret;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_H264_SPS);
	if (!ctrl)
		return -EINVAL;

	ret = avd_h264_validate_sps(ctx, ctrl->p_new.p_h264_sps);
	if (ret)
		return ret;

	h264_ctx = kzalloc(sizeof(*h264_ctx), GFP_KERNEL);
	if (!h264_ctx)
		return -ENOMEM;

	ctx->priv = h264_ctx;

	ret = avd_h264_alloc_bufs(ctx);
	if (ret)
		goto err_free_ctx;

	return 0;

err_free_ctx:
	kfree(h264_ctx);
	ctx->priv = NULL;
	return ret;
}

static void avd_h264_stop(struct avd_ctx *ctx)
{
	avd_h264_free_bufs(ctx);

	/* needed for all so automatic? */
	free_vp_slot(ctx->dev, ctx);
	free_inst_slot(ctx->dev, ctx);
}

static void avd_h264_run_preamble(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	struct v4l2_ctrl *ctrl;
	u32 dst_len, sps_len;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	run->decode_params = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_H264_SPS);
	run->sps = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_H264_PPS);
	run->pps = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_H264_SLICE_PARAMS);
	run->slice_params = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_H264_SCALING_MATRIX);
	run->scaling_matrix = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_H264_PRED_WEIGHTS);
	run->pred_weights = ctrl ? ctrl->p_cur.p : NULL;

	avd_run_preamble(ctx, &run->base);

	dst_len = run->base.bufs.dst->vb2_buf.planes[0].length;

	run->addresses.y =
		vb2_dma_contig_plane_dma_addr(&run->base.bufs.dst->vb2_buf, 0);

	run->addresses.uv =
		run->addresses.y +
		ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline *
			ALIGN(ctx->decoded_fmt.fmt.pix_mp.height, 16);

	run->addresses.sl =
		vb2_dma_contig_plane_dma_addr(&run->base.bufs.src->vb2_buf, 0);
	sps_len = sps_size(fmt_width(ctx), fmt_height(ctx));

	run->addresses.rvra =
		run->addresses.y + (dst_len - sps_len - ctx->rvra.size);

	run->addresses.sps = run->addresses.y + (dst_len - sps_len);
}

static int avd_h264_run(struct avd_ctx *ctx)
{
	struct avd_dev *avd = ctx->dev;
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct v4l2_h264_reflist_builder reflist_builder;
	struct avd_h264_run run;
	u32 slice_size, slice_parsed, reg;
	int ret;

	avd_h264_run_preamble(ctx, &run);

	/* Build the P/B{0,1} ref lists. */
	v4l2_h264_init_reflist_builder(&reflist_builder, run.decode_params, run.sps,
				       run.decode_params->dpb);

	run.num_valid = reflist_builder.num_valid;
	run.cur_poc = reflist_builder.cur_pic_order_count;

	v4l2_h264_build_p_ref_list(&reflist_builder, h264_ctx->reflists.p);
	v4l2_h264_build_b_ref_lists(&reflist_builder, h264_ctx->reflists.b0,
				    h264_ctx->reflists.b1);

	avd_run_postamble(ctx, &run.base);

	if (is_new_frame(run.slice_params)) {
		ret = alloc_slots(avd, ctx, AVD_CODEC_H264);
		if (ret) {
			dev_err_ratelimited(avd->dev, "no free slots: %d", ret);
			return ret;
		}
		avd->variant->configure_stream(ctx->dev, h264_ctx->bufs.inst.addr,
				     ctx->fifo_idx, ctx->vp_slot);
		stream_hdr(ctx, &run);
	}

	if (ctx->vp_slot == VP_SLOT_NONE) {
		/* Only happens if its a multi slice frame and there was an error */
		dev_err_ratelimited(avd->dev, "no assigned VP slots: %04lx", avd->vp_slots);
		return -ENOMEM;
	}

	schedule_delayed_work(&ctx->watchdog_work, msecs_to_jiffies(2000));

	slice_size = stream_slice(ctx, &run);

	if (run.base.bufs.src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF) {
		if (avd->variant->revision == 3)
			reg = (0x18 | ctx->vp_slot << 12);
		else
			reg = (0x1018 | ctx->vp_slot << 8);

		/* seems to be take ~ slice_size / 16 us */
		ret = readl_poll_timeout(
			avd->ctrl + reg, slice_parsed,
			slice_parsed >= round_down(slice_size, 8), 5, 1000);

		if (ret) {
			dev_err(avd->dev, "VP%d: timed out (%02d)! size: %08x parsed: %08x",
					ctx->vp_slot, ctx->fifo_idx, slice_size, slice_parsed);
			avd_status(avd, ctx->vp_slot);
			return 0;
		}

		if (cancel_delayed_work(&ctx->watchdog_work))
			avd_job_finish(ctx, VB2_BUF_STATE_DONE);
	}

	return 0;
}

static enum avd_image_fmt avd_h264_get_image_fmt(struct avd_ctx *ctx,
						 struct v4l2_ctrl *ctrl)
{
	const struct v4l2_ctrl_h264_sps *sps = ctrl->p_new.p_h264_sps;

	if (ctrl->id != V4L2_CID_STATELESS_H264_SPS)
		return AVD_IMG_FMT_ANY;

	if (sps->bit_depth_luma_minus8 == 0) {
		if (sps->chroma_format_idc == 2)
			return AVD_IMG_FMT_422_8BIT;
		else
			return AVD_IMG_FMT_420_8BIT;
	} else if (sps->bit_depth_luma_minus8 == 2) {
		if (sps->chroma_format_idc == 2)
			return AVD_IMG_FMT_422_10BIT;
		else
			return AVD_IMG_FMT_420_10BIT;
	}

	return AVD_IMG_FMT_ANY;
}

static void avd_h264_adjust_decoded_fmt(struct avd_ctx *ctx,
		struct v4l2_pix_format_mplane *pix_mp)
{
	pix_mp->plane_fmt[0].sizeimage += sps_size(pix_mp->width, pix_mp->height);
}

static int avd_h264_try_ctrl(struct avd_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_H264_SPS)
		return avd_h264_validate_sps(ctx, ctrl->p_new.p_h264_sps);
	if (ctrl->id == V4L2_CID_STATELESS_H264_PPS)
		return avd_h264_validate_pps(ctx, ctrl->p_new.p_h264_pps);

	return 0;
}

const struct avd_coded_fmt_ops avd_h264_fmt_ops = {
	.adjust_decoded_fmt = avd_h264_adjust_decoded_fmt,
	.start = avd_h264_start,
	.stop = avd_h264_stop,
	.run = avd_h264_run,
	.try_ctrl = avd_h264_try_ctrl,
	.get_image_fmt = avd_h264_get_image_fmt,
};
