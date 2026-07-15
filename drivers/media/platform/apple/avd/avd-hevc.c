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
#include <linux/v4l2-controls.h>
#include <linux/delay.h>

#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "avd.h"
#include "avd-inst.h"

#define PPS_NUM 8

#define NEW_TILE_ID BIT(0)
#define NEW_SLICE BIT(1)

static inline u32 sps_size(u32 w, u32 h)
{
	/* TODO: does it really need so much? */
	return (((w - 1) * (h - 1) / 0x10000) + 2) * 0x4000;
}

struct avd_hevc_run {
	struct avd_run base;
	const struct v4l2_ctrl_hevc_slice_params *sl;
	const struct v4l2_ctrl_hevc_decode_params *decode;
	const struct v4l2_ctrl_hevc_sps *sps;
	const struct v4l2_ctrl_hevc_pps *pps;
	const struct v4l2_ctrl_hevc_scaling_matrix *scaling_matrix;
	const u32 *entry_point_offsets;

	int num_slices;

	struct run_addr {
		dma_addr_t y;
		dma_addr_t uv;
		dma_addr_t sl;
		dma_addr_t rvra;
		dma_addr_t sps;
	} addresses;
};

struct avd_hevc_ctx {
	struct v4l2_ctrl_hevc_scaling_matrix scaling_matrix_cache;

	struct avd_h264_bufs {
		struct avd_buf pps_tile[8];
		struct avd_buf inst;
		struct avd_buf pipe_state;
	} bufs;

	int submit_num;
};

/* clang-format off */
static void stream_refs(struct avd_ctx *ctx, struct avd_hevc_run *run)
{
	const struct v4l2_ctrl_hevc_decode_params *decode = run->decode;
	const struct v4l2_ctrl_hevc_slice_params *sl = &run->sl[0];
	const struct v4l2_hevc_dpb_entry *dpb;
	struct avd_hevc_ctx *hevc_ctx = ctx->priv;
	struct avd_dev *avd = ctx->dev;
	struct avd_decoded_buffer *dst, *ref_buf;

	dst = vb2_to_avd_decoded_buf(&run->base.bufs.dst->vb2_buf);

	push(INST_DMA2, "cm3_dma_config_6");
	pusha(hevc_ctx->bufs.pps_tile[1].addr, "hdr_9c_pps_tile_addr_lsb8", 7);
	pusha(run->addresses.sps, "hdr_bc_sps_tile_addr_lsb8",
			sl->slice_pic_order_cnt);

	push(INST_DMA3, "cm3_dma_config_7");
	push(INST_DMA3, "cm3_dma_config_8");
	push(INST_DMA3, "cm3_dma_config_9");
	push(INST_DMA3, "cm3_dma_config_a");

	for (int i = 0; i < decode->num_active_dpb_entries; i++) {
		dpb = &decode->dpb[i];

		ref_buf = get_ref_buf(ctx, &dst->base.vb, dpb->timestamp);

		dma_addr_t rvra_addr =
			vb2_dma_contig_plane_dma_addr(&ref_buf->base.vb.vb2_buf, 0)
				+ (ref_buf->base.vb.planes[0].length - sps_size(fmt_width(ctx), fmt_height(ctx)) - ref_buf->rvra.size);

		push((decode->num_active_dpb_entries - 1) << 28
				| 0x1000000
				| (!!(dpb->flags & V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE) << 17)
				| (((sl->slice_pic_order_cnt - dpb->pic_order_cnt_val) & 0x1ffff)),
				"hdr_d0_ref_hdr");

		push_rvra(avd, ctx, rvra_addr, ref_buf->rvra.offsets);
	}
}

static void set_scaling_lists(struct avd_ctx *ctx, struct avd_hevc_run *run)
{
	const struct v4l2_ctrl_hevc_scaling_matrix *s = run->scaling_matrix;
	struct avd_dev *avd = ctx->dev;
	int i,j,k;

	u8 (*dc_16x16)[3] = (u8(*)[3])s->scaling_list_dc_coef_16x16;
	u8 (*sc_4x4)[4][4] = (u8(*)[4][4])s->scaling_list_4x4;
	u8 (*sc_8x8)[2][4][8] = (u8(*)[2][4][8])s->scaling_list_8x8;
	u8 (*sc_16x16)[2][4][8] = (u8(*)[2][4][8])s->scaling_list_16x16;
	u8 (*sc_32x32)[2][4][8] = (u8(*)[2][4][8])s->scaling_list_32x32;

	/*
	 * this should presumably be how many of each are enabled?
	 * or some other scaling related thing
	 */
	push(0x127ffff, "hdr_7c_pps_scl_dims");

	for (i = 0; i < 2; i++)
		push(dc_16x16[i][0] << 16
				| dc_16x16[i][1] << 8
				| dc_16x16[i][2] << 0, "dc_16x16");

	for (i = 0; i < 2; i++)
		push(s->scaling_list_dc_coef_32x32[i] << 16, "dc_32x32");

	/* transposed in stride 4 */
	for (i = 0; i < 6; i++)
		for (j = 0; j < 4; j++)
			push(sc_4x4[i][0][j] << 24
					| sc_4x4[i][1][j] << 16
					| sc_4x4[i][2][j] << 8
					| sc_4x4[i][3][j], "scaling_4x4");

	/* transposed in stride 8 */
	for (i = 0; i < 6; i++)
		for (j = 0; j < 2; j++)
			for (k = 0; k < 8; k++)
			push(sc_8x8[i][j][0][k] << 24
					| sc_8x8[i][j][1][k] << 16
					| sc_8x8[i][j][2][k] << 8
					| sc_8x8[i][j][3][k], "scaling_8x8");

	for (i = 0; i < 6; i++)
		for (j = 0; j < 2; j++)
			for (k = 0; k < 8; k++)
			push(sc_16x16[i][j][0][k] << 24
					| sc_16x16[i][j][1][k] << 16
					| sc_16x16[i][j][2][k] << 8
					| sc_16x16[i][j][3][k], "scaling_16x16");

	for (i = 0; i < 2; i++)
		for (j = 0; j < 2; j++)
			for (k = 0; k < 8; k++)
				push(sc_32x32[i][j][0][k] << 24
						| sc_32x32[i][j][1][k] << 16
						| sc_32x32[i][j][2][k] << 8
						| sc_32x32[i][j][3][k], "scaling_32x32");

}

static void hevc_set_flags(struct avd_ctx *ctx, struct avd_hevc_run *run)
{
	const struct v4l2_ctrl_hevc_decode_params *decode = run->decode;
	const struct v4l2_ctrl_hevc_sps *sps = run->sps;
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	struct avd_dev *avd = ctx->dev;

	u32 log2_ctb_size = ((sps->log2_min_luma_coding_block_size_minus3 + 3) +
				sps->log2_diff_max_min_luma_coding_block_size);

	if (sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED)
		push(0x1000
			| sps->pcm_sample_bit_depth_luma_minus1 << 8
			| sps->pcm_sample_bit_depth_chroma_minus1 << 4
			| sps->log2_min_pcm_luma_coding_block_size_minus3 << 2
			| (sps->log2_diff_max_min_pcm_luma_coding_block_size
			  + sps->log2_min_pcm_luma_coding_block_size_minus3)
			, "hdr_30_sps_pcm");
	else
		push(0, "hdr_30_sps_pcm");

	/*
	 * RExt sets a few new flags here
	 */
	push((u32)BIT(3)
		| !!(sps->flags & V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED) << 9,
			"hdr_34_sps_flags");

	push((log2_ctb_size - 3) << 3
		| pps->log2_parallel_merge_level_minus2 << 9
		| !!(pps->flags & V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED) << 12
		| !!(pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) << 13
		| !!(pps->flags & V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED) << 14
		| ((log2_ctb_size - pps->diff_cu_qp_delta_depth - 3) & 3) << 15
		| !!(pps->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED) << 17
		| !!(pps->flags & V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED) << 18
		| !!(pps->flags & V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED) << 19
		| !!(pps->flags & V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED) << 20
		| !!(!(decode->flags & V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC)
			&& sps->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED) << 21
		, "hdr_5c_pps_flags");

	push(swrap(pps->pps_cb_qp_offset, 1 << 5) << 5
			| swrap(pps->pps_cr_qp_offset, 1 << 5),
			"hdr_60_pps_qp");

	/*
	 * something like segment features?
	 */
	push(0, "hdr_64_zero");
	push(0, "hdr_68_zero");
	push(0, "hdr_6c_zero");
	push(0, "hdr_70_zero");
	push(0, "hdr_74_zero");
	push(0, "hdr_78_zero");
}

static void set_header(struct avd_ctx *ctx, struct avd_hevc_run *run)
{
	const struct v4l2_ctrl_hevc_sps *sps = run->sps;
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	struct avd_dev *avd = ctx->dev;
	struct avd_hevc_ctx *hevc_ctx = ctx->priv;
	u32 bytesperline;
	u32 width = sps->pic_width_in_luma_samples;
	u32 height = sps->pic_height_in_luma_samples;

	bool is_intra = run->sl[0].slice_type == V4L2_HEVC_SLICE_TYPE_I;

	push(0x2b000000
		| ctx->fifo_idx << 4
		| (avd->variant->revision == 3 ? 0x100 : 0x200),
		"inst_fifo_start");

	push(0x2db00000
		| 0x1000
		| (is_intra ? 0x2000 : 0)
		| 0x2e0
		| (avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE ? 0 : 0x80000),
		"hdr_34_start_hdr");

	push(AVD_CODEC_HEVC, "hdr_50_mode");
	push(((height - 1) << 16) | (width - 1), "hdr_54_height_width");
	push(0, "hdr_58_pixfmt_zero");

	push((((height - 1) >> 3) << 16) | ((width - 1) >> 3),
			"hdr_28_height_width_shift3");

	push(sps->chroma_format_idc << 24
		| sps->bit_depth_chroma_minus8 << 19
		| sps->bit_depth_luma_minus8 << 15
		| sps->log2_min_luma_coding_block_size_minus3 << 13
		| (sps->log2_diff_max_min_luma_coding_block_size +
			sps->log2_min_luma_coding_block_size_minus3) << 11
		| sps->log2_min_luma_transform_block_size_minus2 << 9
		| (sps->log2_diff_max_min_luma_transform_block_size +
			+ sps->log2_min_luma_transform_block_size_minus2) << 7
		| sps->max_transform_hierarchy_depth_inter << 4
		| sps->max_transform_hierarchy_depth_intra << 1
		| !!(sps->flags & V4L2_HEVC_SPS_FLAG_AMP_ENABLED),
		 "hdr_2c_sps_txfm");

	hevc_set_flags(ctx, run);
	push(0x300000
			| (avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE ? 0 : 0x30),
			"hdr_98_const_30");

	push(INST_DMA2, "cm3_dma_config_1");
	push(INST_DMA1, "cm3_dma_config_2");

	if (avd->variant->revision == 3)
		push(0, "zero");

	push(INST_DMA1, "cm3_dma_config_2");
	push(INST_DMA2, "cm3_dma_config_3");

	if (avd->variant->revision == 3)
		push(0, "zero");
	else if (!(avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE))
		pusha(hevc_ctx->bufs.pipe_state.addr, "pipe_state", 0);

	pusha(hevc_ctx->bufs.pps_tile[0].addr, "hdr_dc_pps_tile_addr_lsb8", 0);
	pusha(hevc_ctx->bufs.pps_tile[2].addr, "hdr_dc_pps_tile_addr_lsb8", 1);
	pusha(hevc_ctx->bufs.pps_tile[3].addr, "hdr_dc_pps_tile_addr_lsb8", 2);

	if (pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) {
		pusha(hevc_ctx->bufs.pps_tile[4].addr, "hdr_dc_pps_tile_addr_lsb8", 3);
		pusha(hevc_ctx->bufs.pps_tile[5].addr, "hdr_dc_pps_tile_addr_lsb8", 4);
		pusha(hevc_ctx->bufs.pps_tile[6].addr, "hdr_dc_pps_tile_addr_lsb8", 8);
		pusha(hevc_ctx->bufs.pps_tile[7].addr, "hdr_dc_pps_tile_addr_lsb8", 9);
	} else {
		pusha(0, "", 3);
		pusha(0, "", 4);
		pusha(hevc_ctx->bufs.pps_tile[4].addr, "hdr_dc_pps_tile_addr_lsb8", 8);
		pusha(0, "", 9);
	}

	push(INST_DMA3, "cm3_dma_config_5");

	push_rvra(avd, ctx, run->addresses.rvra, ctx->rvra.offsets);

	push(0, "cm3_mark_end_section");

	if (avd->variant->revision != 3)
		push(0, "cm3_mark_end_section");

	bytesperline = ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
	if (avd->variant->quirks & AVD_QUIRK_LSR)
		bytesperline = bytesperline >> 4;

	pusha(run->addresses.y, "hdr_1b4_y_addr_lsb8", 0);
	push(bytesperline, "hdr_1bc_width_align");
	pusha(run->addresses.uv, "hdr_1b8_uv_addr_lsb8", 0);
	push(bytesperline, "hdr_1c0_width_align");
	push(0, "");
	push(((height - 1) << 16) | (width - 1), "hdr_54_height_width");

	if (!is_intra)
		stream_refs(ctx, run);

	if ((sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED))
		set_scaling_lists(ctx, run);
	else
		push(0, "cm3_mark_end_section");

}

static void stream_weights(struct avd_ctx *ctx, struct avd_hevc_run *run,
		const struct v4l2_ctrl_hevc_slice_params *sl)
{
	int luma_weight_denom, chroma_weight_denom;
	u8 chroma_log2_weight_denom;
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	const struct v4l2_hevc_pred_weight_table *pred = &sl->pred_weight_table;
	struct avd_dev *avd = ctx->dev;
	bool has_luma_weights = ((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED)
			&& sl->slice_type == V4L2_HEVC_SLICE_TYPE_P)
		|| ((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED)
				&& sl->slice_type == V4L2_HEVC_SLICE_TYPE_B);

	const s8 (*delta_chroma_weights)[2];
	const s8 (*chroma_offsets)[2];
	const s8 *delta_luma_weights;
	const s8 *luma_offsets;

	if (!has_luma_weights) {
		push(0x2dd00000, "slc_76c_cmd_weights_denom");
		return;
	}

	chroma_log2_weight_denom = pred->luma_log2_weight_denom +
		pred->delta_chroma_log2_weight_denom;

	/*
	 * TODO: this looks weird
	 */
	push(0x2dd00000
			| (!has_luma_weights << 7)
			| (has_luma_weights << 6)
			| (!has_luma_weights << 5)
			| (pred->luma_log2_weight_denom << 3)
			| (chroma_log2_weight_denom & 7)
			,"slc_76c_cmd_weights_denom");

	luma_weight_denom = 1 << pred->luma_log2_weight_denom;
	chroma_weight_denom = 1 << chroma_log2_weight_denom;

	/* comes from 7.4.7.3 */

	for (int y = 0; y < 2; y++) {
		if (y == 1 && sl->slice_type != V4L2_HEVC_SLICE_TYPE_B)
			break;
		luma_offsets = y == 0 ? pred->luma_offset_l0 : pred->luma_offset_l1;
		delta_luma_weights = (s8(*))(y == 0 ?
				pred->delta_luma_weight_l0 : pred->delta_luma_weight_l1);
		chroma_offsets = (s8(*)[2])(y == 0 ?
				pred->chroma_offset_l0 : pred->chroma_offset_l1);
		delta_chroma_weights = (s8(*)[2])(y == 0 ?
				pred->delta_chroma_weight_l0 : pred->delta_chroma_weight_l1);

		int to = y == 0 ? sl->num_ref_idx_l0_active_minus1
			: sl->num_ref_idx_l1_active_minus1;
		for (int i = 0; i < to + 1; i++) {
			/* Avd only expects offsets/weights if they are not the default
			 * ones, otherwise we get artifacts */
			if((delta_luma_weights[i]  != 0) || (luma_offsets[i] != 0)) {
				push(0x2de00000
						| 1 << 14
						| y << 13
						| i << 9
						| ((delta_luma_weights[i] + luma_weight_denom) & 0x1ff),
						"slc_luma_weights");
				push(0x2df00000
						| swrap(luma_offsets[i], 0x10000),
						"slc_luma_offsets");
			}

			if ((delta_chroma_weights[i][0] != 0)
					|| (chroma_offsets[i][0] != 0)
					|| (delta_chroma_weights[i][1] != 0)
					|| (chroma_offsets[i][1] != 0)) {
				push(0x2de00000
						| 2 << 14
						| y << 13
						| i << 9
						| ((delta_chroma_weights[i][0] + chroma_weight_denom) & 0x1ff),
						"slc_chroma_weights[0]");
				push(0x2df00000
						| swrap(chroma_offsets[i][0], 0x10000),
						"slc_chroma_offsets[0]");
				push(0x2de00000
						| 3 << 14
						| y << 13
						| i << 9
						| ((delta_chroma_weights[i][1] + chroma_weight_denom) & 0x1ff),
						"slc_chroma_weights[1]");
				push(0x2df00000
						| swrap(chroma_offsets[i][1], 0x10000),
						"slc_chroma_offsets[1]");
			}
		}
	}
}

static void stream_slice_dqtblk(struct avd_ctx *ctx, struct avd_hevc_run *run,
		const struct v4l2_ctrl_hevc_slice_params *sl)
{
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	const struct v4l2_ctrl_hevc_sps *sps = run->sps;
	struct avd_dev *avd = ctx->dev;

	push(0x2d900000
			| (((pps->init_qp_minus26 + 26 +  sl->slice_qp_delta) << 10) & 0x1fc00)
			| swrap(pps->pps_cb_qp_offset + sl->slice_cb_qp_offset, 32) << 5
			| swrap(pps->pps_cr_qp_offset + sl->slice_cr_qp_offset, 32)
			,"slc_bcc_cmd_quantization");

	push(0x2da00000
			| !!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA) << 6
		    | !!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA) << 7
			| swrap(sl->slice_tc_offset_div2, 16) << 8
			| swrap(sl->slice_beta_offset_div2, 16) << 12
			/* i wonder what this should actually be */
			| !!(!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED)
				&& (sps->flags & V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED
				|| sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA
				|| pps->flags & (V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED | V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT))) << 16
			| !!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED) << 17
			| !!(!(pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED)
				|| (pps->flags & V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED)) << 18
			| !!((sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED)
				&& ! (sps->flags & V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED)) << 19
			, "slc_bd0_cmd_deblocking_filter");

	if (sl->slice_type == V4L2_HEVC_SLICE_TYPE_B
			|| sl->slice_type == V4L2_HEVC_SLICE_TYPE_P) {
		for (int i = 0; i < sl->num_ref_idx_l0_active_minus1 + 1; i++)
			push(0x2dc00000
					| (0 << 8)
					| ((i & 0xf) << 4)
					| (sl->ref_idx_l0[i] & 0xf),
					"reference_frames_l0");
		if (sl->slice_type == V4L2_HEVC_SLICE_TYPE_B)
			for (int i = 0; i < sl->num_ref_idx_l1_active_minus1 + 1; i++)
				push(0x2dc00000
						| (1 << 8)
						| ((i & 0xf) << 4)
						| (sl->ref_idx_l1[i] & 0xf),
						"reference_frames_l1");

		stream_weights(ctx, run, sl);
	}
}
static void stream_slice_mv(struct avd_ctx *ctx, struct avd_hevc_run *run,
		const struct v4l2_ctrl_hevc_slice_params *sl, bool is_first)
{
	const struct v4l2_ctrl_hevc_decode_params *decode = run->decode;
	struct avd_dev *avd = ctx->dev;
	struct avd_decoded_buffer *dst, *ref;
	bool ref_valid;
	const u8 *ref_list;

	if (sl->slice_type == V4L2_HEVC_SLICE_TYPE_I) {
		push(0x2d000000
				| !!(sl->slice_type == V4L2_HEVC_SLICE_TYPE_I) << 17
				, "slc_a8c_cmd_ref_type");
		return;
	}
	/* bidirectional prediction */

	ref_list = sl->slice_type == V4L2_HEVC_SLICE_TYPE_P ? sl->ref_idx_l0
		: sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0
			? sl->ref_idx_l0 : sl->ref_idx_l1;

	dst = vb2_to_avd_decoded_buf(&run->base.bufs.dst->vb2_buf);
	ref = get_ref_buf(ctx, &dst->base.vb,
			decode->dpb[ref_list[sl->collocated_ref_idx]].timestamp);

	ref_valid =
		!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT)
		&& (sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED)
		&& is_first
		&& !ref->hevc.is_intra;

	u32 x = 0;

	x |= (5 - sl->five_minus_max_num_merge_cand) << 1;
	x |= !!((sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED)
			&& !(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0)) << 4;
	x |= !!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT) << 5;
	if (sl->slice_type == V4L2_HEVC_SLICE_TYPE_B) {
		x |= !!!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO) << 6;
		x |= sl->num_ref_idx_l1_active_minus1 << 7;
	}
	x |= sl->num_ref_idx_l0_active_minus1 << 11;

	x |= !!((sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED)
			|| (sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT)) << 15;

	push(0x2d000000
			| x
			| !!(sl->slice_type == V4L2_HEVC_SLICE_TYPE_P) << 16
			| ref_valid << 18
			, "slc_a8c_cmd_ref_type");

	if (ref_valid) {
		dma_addr_t sps_tile_addr = vb2_dma_contig_plane_dma_addr(&ref->base.vb.vb2_buf, 0)
			+ (ref->base.vb.planes[0].length - sps_size(fmt_width(ctx), fmt_height(ctx)));
		pusha(sps_tile_addr, "slc_bd4_sps_tile_addr2_lsb8",
				decode->dpb[ref_list[sl->collocated_ref_idx]].pic_order_cnt_val);
	}
}

static void set_slice(struct avd_ctx *ctx, struct avd_hevc_run *run,
		const struct v4l2_ctrl_hevc_slice_params *sl,
		u32 size, u32 offset, u32 flags)
{
	struct avd_dev *avd = ctx->dev;
	dma_addr_t slc_addr = run->addresses.sl + offset + sl->data_byte_offset;
	push(0x2d800000
			| flags
			| (u32)(slc_addr >> 32), "cm3_cmd_set_coded_slice");
	push((u32)(slc_addr & 0xffffffff), "slc_bd8_slice_addr");
	push(size, "slc_bdc_slice_size");
}

static int submit_slice_segment(struct avd_ctx *ctx, struct avd_hevc_run *run,
		const struct v4l2_ctrl_hevc_slice_params *sl,
		int row, int col, u32 col_bd[23], u32 row_bd[23],
		u32 pic_in_cts_width, u32 pic_in_cts_height,
		bool is_last, bool first_slice,
		bool hflip, bool vflip,
		u32 coded_flags, u32 last_tile_block)
{
	struct avd_dev *avd = ctx->dev;
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	u32 tb_x, tb_y, tile_block, tile_boundary;

	if (coded_flags & NEW_SLICE) {
		tb_x = sl->slice_segment_addr % pic_in_cts_width;
		tb_y = sl->slice_segment_addr / pic_in_cts_width;

		tile_block = tb_y << 12 | tb_x;

		if (!(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT))
			last_tile_block = tile_block;

		/*
		 * tile block start
		 * CABAC window
		 */
		push(0x2c000000 | last_tile_block, "cm3_cmd_set_cabac_xy");

		stream_slice_dqtblk(ctx, run, sl);
	} else {
		tile_boundary = row_bd[row] << 12 | col_bd[col];
	}

	if (coded_flags & NEW_TILE_ID) {
		push(0x2a000000 |
				(coded_flags & NEW_SLICE ? tile_block : tile_boundary),
				"cm3_cmd_set_ctb_xy");

		/* tile boundary end */
		if (pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED)
			push(((hflip ? 4 : 0) | (vflip ? 8 : 0)) << 28
					| col << 24
					| (row_bd[row + 1] - 1) << 12
					| (col_bd[col + 1] - 1),
					"cm3_set_ctb_xy");
		else /* first slice, one CTB */
			push((pic_in_cts_height - 1) << 12
					| (pic_in_cts_width - 1),
					"cm3_set_ctb_xy");
	}

	if (coded_flags & NEW_SLICE)
		stream_slice_mv(ctx, run, sl, first_slice);

	/* current tile block / boundary ?? */
	/* Unlike entropy, motion vector window resets every time */
	push(1 << 24 | (coded_flags & NEW_SLICE ? tile_block : tile_boundary),
			"cm3_set_mv_xy");

	push(0x2b000000 | is_last << 10, "cm3_cmd_inst_fifo_end");

	return last_tile_block;
}
/* clang-format on */

static void compute_tiles_uniform(struct avd_hevc_run *run,
				  u16 log2_min_cb_size, u16 width, u16 height,
				  s32 pic_in_cts_width, s32 pic_in_cts_height,
				  u16 *column_width, u16 *row_height)
{
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	int i;

	for (i = 0; i < pps->num_tile_columns_minus1 + 1; i++)
		column_width[i] = ((i + 1) * pic_in_cts_width) /
					  (pps->num_tile_columns_minus1 + 1) -
				  (i * pic_in_cts_width) /
					  (pps->num_tile_columns_minus1 + 1);

	for (i = 0; i < pps->num_tile_rows_minus1 + 1; i++)
		row_height[i] = ((i + 1) * pic_in_cts_height) /
					(pps->num_tile_rows_minus1 + 1) -
				(i * pic_in_cts_height) /
					(pps->num_tile_rows_minus1 + 1);
}

static void compute_tiles_non_uniform(struct avd_hevc_run *run,
				      u16 log2_min_cb_size, u16 width,
				      u16 height, s32 pic_in_cts_width,
				      s32 pic_in_cts_height, u16 *column_width,
				      u16 *row_height)
{
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	s32 sum = 0;
	int i;

	for (i = 0; i < pps->num_tile_columns_minus1; i++) {
		column_width[i] = pps->column_width_minus1[i] + 1;
		sum += column_width[i];
	}
	column_width[i] = pic_in_cts_width - sum;

	sum = 0;
	for (i = 0; i < pps->num_tile_rows_minus1; i++) {
		row_height[i] = pps->row_height_minus1[i] + 1;
		sum += row_height[i];
	}
	row_height[i] = pic_in_cts_height - sum;
}

static void compute_bd(struct avd_hevc_run *run, u32 *col_bd, u32 *row_bd,
		       u16 *column_width, u16 *row_height)
{
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	int i;

	for (col_bd[0] = 0, i = 0; i <= pps->num_tile_columns_minus1; i++)
		col_bd[i + 1] = col_bd[i] + column_width[i];

	for (row_bd[0] = 0, i = 0; i <= pps->num_tile_rows_minus1; i++)
		row_bd[i + 1] = row_bd[i] + row_height[i];
}

static void compute_rs_to_ts(struct avd_hevc_run *run, u32 pic_in_ctbs_size,
			     u32 pic_in_ctbs_width, u32 *col_bd, u32 *row_bd,
			     u16 *col_width, u16 *row_height,
			     u32 *ctb_addr_rs_to_ts)
{
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	int i, j, tb_x, tb_y, tile_x = 0, tile_y = 0;
	u32 ctb_addr_rs;

	for (ctb_addr_rs = 0; ctb_addr_rs < pic_in_ctbs_size; ctb_addr_rs++) {
		tb_x = ctb_addr_rs % pic_in_ctbs_width;
		tb_y = ctb_addr_rs / pic_in_ctbs_width;
		for (i = 0; i <= pps->num_tile_columns_minus1; i++)
			if (tb_x >= col_bd[i])
				tile_x = i;
		for (j = 0; j <= pps->num_tile_rows_minus1; j++)
			if (tb_y >= row_bd[j])
				tile_y = j;
		ctb_addr_rs_to_ts[ctb_addr_rs] = 0;
		for (i = 0; i < tile_x; i++)
			ctb_addr_rs_to_ts[ctb_addr_rs] += row_height[tile_y] * col_width[i];
		for (j = 0; j < tile_y; j++)
			ctb_addr_rs_to_ts[ctb_addr_rs] += pic_in_ctbs_width * row_height[j];
		ctb_addr_rs_to_ts[ctb_addr_rs] +=
			(tb_y - row_bd[tile_y]) * col_width[tile_x] + tb_x - col_bd[tile_x];
	}
}

static void compute_tile_ids(struct avd_hevc_run *run, u32 pic_in_ctbs_width,
			     u32 *col_bd, u32 *row_bd, u32 *ctb_addr_rs_to_ts,
			     u32 *tile_ids)
{
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	int j, i, y, x;
	u32 tile_idx;
	for (j = 0, tile_idx = 0; j <= pps->num_tile_rows_minus1; j++)
		for (i = 0; i <= pps->num_tile_columns_minus1; i++, tile_idx++)
			for (y = row_bd[j]; y < row_bd[j + 1]; y++)
				for (x = col_bd[i]; x < col_bd[i + 1]; x++)
					tile_ids[ctb_addr_rs_to_ts[y * pic_in_ctbs_width + x]]
						= tile_idx;
}

static int avd_wait_submission_queue(struct avd_ctx *ctx)
{
	struct avd_dev *avd = ctx->dev;
	u32 max = readl_relaxed(avd->ctrl +
				avd->variant->submit_queue_max_offset +
				(ctx->vp_slot) * 4);
	u32 cur = readl_relaxed(avd->ctrl +
				avd->variant->submit_queue_status_offset +
				(ctx->vp_slot) * 4);

	if (cur == max) {
		dev_err_ratelimited(avd->dev, "instruction que full! %d/%d",
				    cur, max);
		return 1;
	}

	if (cur >= max / 2) {
		/* TODO: to high? low? Has weird side effects??? */
		usleep_range(100, 150);
	}
	return 0;
}

struct sl_ctx {
	u32 ctx_col;
	u32 ctx_row;
	s32 q1_col;
	s32 q1_row;
};

/* TODO */
static void stream_slices(struct avd_ctx *ctx, struct avd_hevc_run *run)
{
	const struct v4l2_ctrl_hevc_sps *sps = run->sps;
	const struct v4l2_ctrl_hevc_pps *pps = run->pps;
	struct avd_hevc_ctx *hevc_ctx = ctx->priv;
	const struct v4l2_ctrl_hevc_slice_params *sl;
	bool tiles_enabled, is_last, first_slice, first_segment;
	bool hflip, vflip;
	int slice_segment_offset, entry_point_idx = 0, pos = 0, offset = 0;
	int row, col, i, s, to;
	int slice_flag, size, new_offset;
	int tile_id, last_tile_id;
	u16 log2_min_cb_size, width, height;
	s32 max_cu_width, pic_in_ctbs_width, pic_in_ctbs_height,
		pic_in_ctbs_size;
	u32 num_cols, last_tile_block = 0;
	u16 col_width[22] = {}, row_height[22] = {};
	u32 col_bd[23] = {}, row_bd[23] = {};
	u32 *ctb_addr_rs_to_ts = NULL, *tile_ids = NULL;
	struct sl_ctx last = {
		.q1_col = -1,
		.q1_row = -1,
	};

	width = sps->pic_width_in_luma_samples;
	height = sps->pic_height_in_luma_samples;

	log2_min_cb_size = sps->log2_min_luma_coding_block_size_minus3 + 3;

	num_cols = pps->num_tile_columns_minus1 + 1;

	max_cu_width = 1 << (sps->log2_diff_max_min_luma_coding_block_size +
			     log2_min_cb_size);
	pic_in_ctbs_width = (width + max_cu_width - 1) / max_cu_width;
	pic_in_ctbs_height = (height + max_cu_width - 1) / max_cu_width;
	pic_in_ctbs_size = pic_in_ctbs_height * pic_in_ctbs_width;

	tiles_enabled = !!(pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED);

	ctb_addr_rs_to_ts = kzalloc(
		sizeof(*ctb_addr_rs_to_ts) * pic_in_ctbs_size, GFP_KERNEL);
	if (!ctb_addr_rs_to_ts)
		goto done;

	tile_ids = kzalloc(sizeof(*tile_ids) * pic_in_ctbs_size, GFP_KERNEL);
	if (!tile_ids)
		goto done;

	if (tiles_enabled) {
		if (pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING) {
			compute_tiles_uniform(run, log2_min_cb_size, width,
					      height, pic_in_ctbs_width,
					      pic_in_ctbs_height, col_width,
					      row_height);
		} else {
			compute_tiles_non_uniform(run, log2_min_cb_size, width,
						  height, pic_in_ctbs_width,
						  pic_in_ctbs_height, col_width,
						  row_height);
		}
		compute_bd(run, col_bd, row_bd, col_width, row_height);

		/*
		 * 6.5.1 CTB raster and tile scanning conversion process
		 * (6-7) and (6-9)
		 *
		 * we really only need to know if tileidx != last tileidx
		 * so doing all this is stupid
		 */
		compute_rs_to_ts(run, pic_in_ctbs_size, pic_in_ctbs_width,
				 col_bd, row_bd, col_width, row_height,
				 ctb_addr_rs_to_ts);
		compute_tile_ids(run, pic_in_ctbs_width, col_bd, row_bd,
				 ctb_addr_rs_to_ts, tile_ids);
	}

	for (s = 0; s < run->num_slices; s++) {
		sl = &run->sl[s];
		slice_segment_offset = 0;
		to = tiles_enabled ? sl->num_entry_point_offsets + 1 : 1;

		for (i = 0; i < to; i++) {
			is_last = i == to - 1 && s == run->num_slices - 1;
			first_segment = i == 0;
			first_slice = s == 0;

			if (tiles_enabled && to > 1) {
				if (i < sl->num_entry_point_offsets) {
					size = run->entry_point_offsets[entry_point_idx++];
					new_offset = size;
				} else {
					size = sl->bit_size / 8 -
					       sl->data_byte_offset -
					       slice_segment_offset;
					new_offset =
						size + sl->data_byte_offset;
				}
			} else {
				size = (sl->bit_size) / 8 -
				       sl->data_byte_offset;
				new_offset = size + sl->data_byte_offset;
			}

			tile_id = tile_ids[ctb_addr_rs_to_ts[sl->slice_segment_addr]];
			last_tile_id =
				first_slice ?
					-1 :
					tile_ids[ctb_addr_rs_to_ts[run->sl[s - 1].slice_segment_addr]];

			slice_flag = 0;
			if (first_slice || tile_id != last_tile_id || !first_segment)
				slice_flag |= NEW_TILE_ID;

			if (first_segment)
				slice_flag |= NEW_SLICE;

			vflip = false;
			hflip = false;

			row = pos / num_cols;
			col = pos % num_cols;

			/*
			 * not a fan of this, in JCT-VC-HEVC_V1 only TILES_B_Cisco_1
			 * seems affected and it only seems to need hflip.
			 */
			if (slice_flag & NEW_TILE_ID) {
				if ((col >= last.ctx_col && row > last.ctx_row) ||
				    (col <= last.q1_col && row > last.q1_row))
					vflip = true;

				if (!(slice_flag & NEW_SLICE)) {
					if (row && row == last.ctx_row + 1) {
						hflip = true;
						if (!vflip) {
							last.q1_row = row;
							last.q1_col = col;
						}
					}
				} else {
					last.ctx_row = row;
					last.ctx_col = col;
					last.q1_row = -1;
					last.q1_col = -1;
				}
			}

			set_slice(
				ctx, run, sl, size,
				offset + slice_segment_offset,
				/* makes WPP more reliable? But not really ?? */
				(sl->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT ?
					 slice_flag & ~NEW_SLICE : slice_flag) << 13);

			last_tile_block = submit_slice_segment(
				ctx, run, sl, row, col, col_bd, row_bd,
				pic_in_ctbs_width, pic_in_ctbs_height, is_last,
				first_slice, hflip, vflip, slice_flag,
				last_tile_block);

			if (slice_flag & NEW_TILE_ID)
				pos++;

			slice_segment_offset += new_offset;

			if (avd_wait_submission_queue(ctx))
				goto done;
		}
		offset += sl->bit_size / 8;
	}

	hevc_ctx->submit_num = pos;

done:
	if (ctb_addr_rs_to_ts)
		kfree(ctb_addr_rs_to_ts);

	if (tile_ids)
		kfree(tile_ids);
}

static void update_dec_buf_info(struct avd_decoded_buffer *buf,
				const struct v4l2_ctrl_hevc_slice_params *sl)
{
	buf->hevc.is_intra = sl->slice_type == V4L2_HEVC_SLICE_TYPE_I;
}

static void avd_hevc_adjust_decoded_fmt(struct avd_ctx *ctx,
					struct v4l2_pix_format_mplane *pix_mp)
{
	pix_mp->plane_fmt[0].sizeimage +=
		sps_size(pix_mp->width, pix_mp->height);
}

static enum avd_image_fmt avd_hevc_get_image_fmt(struct avd_ctx *ctx,
						 struct v4l2_ctrl *ctrl)
{
	const struct v4l2_ctrl_hevc_sps *sps = ctrl->p_new.p_hevc_sps;

	if (ctrl->id != V4L2_CID_STATELESS_HEVC_SPS)
		return AVD_IMG_FMT_ANY;

	/*
	 * TODO: we can do up to 4:4:4 12 bit
	 * not sure if v4l2 supports RExt
	 */

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

static int avd_hevc_validate_sps(struct avd_ctx *ctx,
				 const struct v4l2_ctrl_hevc_sps *sps)
{
	if (sps->pic_width_in_luma_samples > ctx->coded_fmt.fmt.pix_mp.width ||
	    sps->pic_height_in_luma_samples > ctx->coded_fmt.fmt.pix_mp.height)
		return -EINVAL;

	return 0;
}

static int avd_hevc_alloc_bufs(struct avd_ctx *ctx)
{
	struct avd_dev *dev = ctx->dev;
	struct avd_hevc_ctx *hevc_ctx = ctx->priv;
	int ret;

	ret = avd_buf_alloc(dev, &hevc_ctx->bufs.inst, fifo_size());
	if (ret)
		return ret;

	ret = avd_buf_alloc(dev, &hevc_ctx->bufs.pipe_state, 0x200);
	if (ret)
		return ret;

	for (int i = 0; i < PPS_NUM; i++) {
		/* TODO: this is very waistfull */
		ret = avd_buf_alloc(dev, &hevc_ctx->bufs.pps_tile[i],
				    fmt_width(ctx) * fmt_height(ctx) / 8);
		if (ret)
			return ret;
	}

	return 0;
}

static int avd_hevc_start(struct avd_ctx *ctx)
{
	struct avd_hevc_ctx *hevc_ctx;
	int ret;

	hevc_ctx = kzalloc(sizeof(*hevc_ctx), GFP_KERNEL);
	if (!hevc_ctx)
		return -ENOMEM;

	ctx->priv = hevc_ctx;
	ret = avd_hevc_alloc_bufs(ctx);
	if (ret)
		goto err_free_ctx;

	return 0;

err_free_ctx:
	kfree(hevc_ctx);
	ctx->priv = NULL;
	return ret;
}

static void avd_hevc_stop(struct avd_ctx *ctx)
{
	struct avd_hevc_ctx *hevc_ctx = ctx->priv;
	struct avd_dev *avd = ctx->dev;

	if (!hevc_ctx)
		return;

	avd_buf_free(avd, &hevc_ctx->bufs.pipe_state);
	avd_buf_free(avd, &hevc_ctx->bufs.inst);

	for (int i = 0; i < PPS_NUM; i++)
		avd_buf_free(avd, &hevc_ctx->bufs.pps_tile[i]);

	free_vp_slot(avd, ctx);
	free_inst_slot(avd, ctx);

	kfree(hevc_ctx);
}

static void avd_hevc_run_preamble(struct avd_ctx *ctx, struct avd_hevc_run *run)
{
	struct v4l2_ctrl *ctrl;
	u32 dst_len, sps_len;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_HEVC_DECODE_PARAMS);
	run->decode = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_HEVC_SLICE_PARAMS);
	run->sl = ctrl ? ctrl->p_cur.p : NULL;
	run->num_slices = ctrl ? ctrl->new_elems : 0;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_HEVC_SPS);
	run->sps = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_HEVC_PPS);
	run->pps = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_HEVC_SCALING_MATRIX);
	run->scaling_matrix = ctrl ? ctrl->p_cur.p : NULL;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_HEVC_ENTRY_POINT_OFFSETS);
	run->entry_point_offsets = ctrl ? ctrl->p_cur.p : NULL;

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

static int avd_hevc_run(struct avd_ctx *ctx)
{
	struct avd_dev *avd = ctx->dev;
	struct avd_hevc_run run;
	struct avd_hevc_ctx *hevc_ctx;
	struct avd_decoded_buffer *dst;
	int ret;

	avd_hevc_run_preamble(ctx, &run);

	avd_run_postamble(ctx, &run.base);
	hevc_ctx = ctx->priv;
	dst = vb2_to_avd_decoded_buf(&run.base.bufs.dst->vb2_buf);
	update_dec_buf_info(dst, &run.sl[0]);

	ret = alloc_slots(avd, ctx, AVD_CODEC_HEVC);
	if (ret) {
		dev_err(avd->dev, "no free slots: %d", ret);
		return ret;
	}
	avd->variant->configure_stream(avd, hevc_ctx->bufs.inst.addr,
				       ctx->fifo_idx, ctx->vp_slot);
	set_header(ctx, &run);

	schedule_delayed_work(&ctx->watchdog_work, msecs_to_jiffies(2000));
	stream_slices(ctx, &run);

	return 0;
}

static int avd_hevc_try_ctrl(struct avd_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_HEVC_SPS)
		return avd_hevc_validate_sps(ctx, ctrl->p_new.p_hevc_sps);

	return 0;
}

static void avd_hevc_submit(struct avd_ctx *ctx)
{
	struct avd_hevc_ctx *hevc_ctx = ctx->priv;
	struct avd_dev *avd = ctx->dev;

	for (int i = 0; i < hevc_ctx->submit_num; i++) {
		writel(0x2b000000
				| (i == 0 ? (avd->variant->revision == 3 ? 0x100 : 0x200) : 0)
				| (ctx->fifo_idx << 4) | avd->variant->fifo_slots,
		       avd->ctrl + avd->variant->submit_offset);
	}
}

const struct avd_coded_fmt_ops avd_hevc_fmt_ops = {
	.adjust_decoded_fmt = avd_hevc_adjust_decoded_fmt,
	.start = avd_hevc_start,
	.stop = avd_hevc_stop,
	.run = avd_hevc_run,
	.submit = avd_hevc_submit,
	.try_ctrl = avd_hevc_try_ctrl,
	.get_image_fmt = avd_hevc_get_image_fmt,
};
