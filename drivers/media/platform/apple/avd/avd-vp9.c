/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple AVD VPU codec driver
 *
 * Copyright The Asahi Linux Contributors
 * Copyright 2023 Eileen Yoon <eyn@gmx.com>
 *
 * Copyright (C) 2019 Collabora, Ltd.
 *	Boris Brezillon <boris.brezillon@collabora.com>
 * Copyright (C) 2021 Collabora, Ltd.
 *	Andrzej Pietrasiewicz <andrzej.p@collabora.com>
 *
 * Copyright (C) 2016 Rockchip Electronics Co., Ltd.
 *	Alpha Lin <Alpha.Lin@rock-chips.com>
 */

#include "linux/v4l2-controls.h"
#include <linux/unaligned.h>
#include <linux/delay.h>

#include <media/v4l2-vp9.h>
#include <media/videobuf2-dma-contig.h>

#include "avd.h"
#include "avd-inst.h"

struct avd_vp9_seg_probs {
	u8 tree_probs[7];
	u8 pred_probs[3];
};

struct avd_vp9_probs {
	struct avd_vp9_seg_probs seg;
	u8 tx8[2][1];
	u8 tx16[2][2];
	u8 tx32[2][3];
	/* [4][2][2][k=6][(k == 0) ? 3 : 6][3] */
	u8 coef[1584];
	u8 skip[3];
	u8 inter_mode[7][3];
	u8 interp_filter[4][2];
	u8 is_inter[4];
	u8 comp_mode[5];
	u8 single_ref[5][2];
	u8 comp_ref[5];
	u8 y_mode[4][9];
	u8 uv_mode[10][9];
	u8 partition[16][3];
	u8 joint[3];
	struct mv_comp {
		u8 sign;
		u8 classes[10];
		u8 class0_bit;
		u8 bits[10];
	} mv_comp[2];
	struct mv_fr {
		u8 class0_fr[2][3];
		u8 fr[3];
	} mv_fr[2];
	struct mv_hp {
		u8 class0_hp;
		u8 hp;
	} mv_hp[2];
};
static_assert(sizeof(struct avd_vp9_probs) == 1905);

struct avd_vp9_frame_symbol_counts {
	u32 padding;
	u32 tx8p[2][2];
	u32 tx16p[2][3];
	u32 tx32p[2][4];
	/* [4][2][2][k=6][(k == 0) ? 3 : 6] */
	u32 eob_0[528];
	/*
	 * struct ref_cnt { u32 coef[3]; u32 eob_1; }
	 * struct ref_cnt cef_counts[4][2][2][k=6][(k == 0) ? 3 : 6];
	 */
	u32 ref_cnt[2112];
	u32 skip[3][2];
	u32 mv_mode[7][4];
	u32 filter[4][3];
	u32 intra_inter[4][2];
	u32 comp[5][2];
	u32 single_ref[5][2][2];
	u32 comp_ref[5][2];
	u32 y_mode[4][10];
	u32 uv_mode[10][10];
	u32 partition[16][4];
	u32 mv_joint[4];
	struct mv_comp_ctn {
		u32 sign[2];
		u32 classes[11];
		u32 class0[2];
		u32 bits[10][2];
	} mv_comp[2];
	struct mv_fr_cnt {
		u32 class0_fr[2][4];
		u32 fr[4];
	} mv_fr[2];
	struct mv_hp_cnt {
		u32 class0_hp[2];
		u32 hp[2];
	} mv_hp[2];
};
static_assert(sizeof(struct avd_vp9_frame_symbol_counts) == 12252);

struct avd_vp9_frame_info {
	u32 valid : 1;
	u32 segmapid : 1;
	u32 frame_context_idx : 2;
	u32 reference_mode : 2;
	u32 tx_mode : 3;
	u32 interpolation_filter : 3;
	u32 flags;
	u64 timestamp;
	struct v4l2_vp9_segmentation seg;
	struct v4l2_vp9_loop_filter lf;
};

struct avd_vp9_run {
	struct avd_run base;

	struct run_addr {
		dma_addr_t y;
		dma_addr_t uv;
		dma_addr_t sl;
		dma_addr_t rvra;
	} addresses;

	const struct v4l2_ctrl_vp9_frame *decode_params;
	const struct v4l2_ctrl_vp9_compressed_hdr *prob_updates;
};

struct avd_vp9_ctx {
	struct v4l2_vp9_frame_symbol_counts cnts;
	struct v4l2_vp9_frame_context probability_tables;
	struct v4l2_vp9_frame_context frame_context[4];
	struct avd_vp9_bufs {
		struct avd_buf inst;
		/* only affect tiles */
		struct avd_buf tiles[3];
		/* just a guess (this name exists in tunables) */
		struct avd_buf above_info;
		/* if above is true, state and color are left */
		struct avd_buf state;
		struct avd_buf color[2];
		struct avd_buf seg;
		struct avd_buf pipe_state;
		struct avd_buf counts;
		struct avd_buf probs;
	} bufs;
	struct avd_vp9_frame_info cur;
	struct avd_vp9_frame_info last;
	u8 submit_num;
};

static void set_refs(struct avd_ctx *ctx, struct avd_vp9_run *run)
{
	const struct v4l2_ctrl_vp9_frame *frame = run->decode_params;
	struct avd_dev *avd = ctx->dev;
	struct avd_decoded_buffer *dst, *ref_buf[4];
	dma_addr_t addr;

	dst = vb2_to_avd_decoded_buf(&run->base.bufs.dst->vb2_buf);

	ref_buf[0] = get_ref_buf(ctx, &dst->base.vb, frame->last_frame_ts);
	ref_buf[1] = get_ref_buf(ctx, &dst->base.vb, frame->golden_frame_ts);
	ref_buf[2] = get_ref_buf(ctx, &dst->base.vb, frame->alt_frame_ts);

	push(INST_DMA3, "cm3_dma_config_7");
	push(INST_DMA3, "cm3_dma_config_8");
	push(INST_DMA3, "cm3_dma_config_9");

	for (int i = 0; i < V4L2_VP9_NUM_FRAME_CTX - 1; i++) {

		addr = vb2_dma_contig_plane_dma_addr(&ref_buf[i]->base.vb.vb2_buf, 0)
			+ (ref_buf[i]->base.vb.planes[0].length - ref_buf[i]->rvra.size);

		/* TODO */
		push(0x1000000, "hdr_9c_ref_100");
		push((ref_buf[i]->vp9.height - 1) << 16 | (ref_buf[i]->vp9.width - 1),
				"hdr_70_ref_height_width");
		push(0x40004000, "hdr_7c_ref_align");

		push_rvra(avd, ctx, addr, ref_buf[i]->rvra.offsets);
	}
}

static u32 make_flags1(struct avd_ctx *ctx, struct avd_vp9_run *run)
{
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	const struct v4l2_ctrl_vp9_frame *frame = run->decode_params;
	bool has_ref = boolify(vp9_ctx->last.valid &&
				!(vp9_ctx->last.flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME)
				&& vp9_ctx->last.flags & V4L2_VP9_FRAME_FLAG_SHOW_FRAME);

	u32 flags = BIT(0)
		| boolify(frame->flags & V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE) << 14
		| !boolify(frame->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) << 15
		| boolify(frame->flags & V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV) << 19;

	if (!(frame->flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME)) {
		flags |= frame->interpolation_filter << 16;
		if (!(frame->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT))
			flags |= has_ref << 21;
		else
			flags |= BIT(20);
	}

	flags |= frame->ref_frame_sign_bias << 7;
	/* this seems wrong */
	flags |= !(frame->golden_frame_ts == 0) << 10;
	flags |= !(frame->ref_frame_sign_bias == 0) << 11;

	flags |= frame->reference_mode << 12;

	flags |=
		!!(frame->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_UPDATE_MAP) << 24
		| !!(frame->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_ENABLED) << 25;
	/* what?? */
	if (frame->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_UPDATE_DATA) {
		if (frame->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_TEMPORAL_UPDATE)
			flags |= BIT(23);
		else if (!vp9_ctx->last.valid)
			flags |= BIT(26);
	}
	return flags;
}

static u32 seg_features(struct avd_ctx *ctx, struct avd_vp9_run *run,
		unsigned int segid)
{
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	const struct v4l2_vp9_segmentation *seg = &vp9_ctx->cur.seg;
	s16 feature_val = 0;
	int feature_id = 0;
	u32 enabled = 0;

	feature_id = V4L2_VP9_SEG_LVL_ALT_Q;
	if (v4l2_vp9_seg_feat_enabled(seg->feature_enabled, feature_id, segid))
		feature_val = seg->feature_data[segid][feature_id];

	feature_id = V4L2_VP9_SEG_LVL_SKIP;
	if (v4l2_vp9_seg_feat_enabled(seg->feature_enabled, feature_id, segid))
		enabled |= 1;

	return (feature_val & 0x3ff) << 12 | enabled;
}

static void set_header(struct avd_ctx *ctx, struct avd_vp9_run *run)
{
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	const struct v4l2_ctrl_vp9_compressed_hdr *prob_updates = run->prob_updates;
	const struct v4l2_ctrl_vp9_frame *frame = run->decode_params;
	struct avd_dev *avd = ctx->dev;
	u32 bytesperline;

	bool intra_only = !!(frame->flags &
			(V4L2_VP9_FRAME_FLAG_KEY_FRAME |
			 V4L2_VP9_FRAME_FLAG_INTRA_ONLY));

	push(0x2b000000
			| (avd->variant->revision == 3 ? 0xfff100 : 0x200)
			| (ctx->fifo_idx << 4),
			"inst_fifo_start");

	push(0x2db00000
			| 0x1000
			| ((intra_only) ? 0x2000 : 0)
			| 0x2e0
			| (avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE ? 0 : 0x80000)
			, "hdr_34_start_hdr");

	push(AVD_CODEC_VP9 << 24, "hdr_38_mode");

	push(((frame->frame_height_minus_1) << 16) | (frame->frame_width_minus_1),
			"hdr_28_height_width_shift3");
	push(0, "cm3_dma_config_0");
	push(((frame->frame_height_minus_1) << 16) | (frame->frame_width_minus_1),
			"hdr_38_height_width_shift3");

	push(0x1000000 /* (sps->chroma_format_idc & 3) << 25 ? */
			| frame->profile << 19
			| (frame->bit_depth - 8) << 15
			| 0x1000 /* colorspace? */
			| 0x800
			| (min(prob_updates->tx_mode, 3) << 7)
			| boolify(prob_updates->tx_mode & V4L2_VP9_TX_MODE_SELECT)
			,"hdr_2c_txfm_mode");

	push(make_flags1(ctx, run), "hdr_40_flags1_pt1");

	for (int i = 0; i < 8; i++)
		push(seg_features(ctx, run, i), "seg");

	/* some kind of feature enable? h26{2,5} has 3 instread */
	push(0x20000, "unk_const");
	push(INST_DMA2, "cm3_dma_config_2");
	push(INST_DMA1, "cm3_dma_config_3");

	pusha(vp9_ctx->bufs.counts.addr, "frame_counts_addr", 0);
	pusha(vp9_ctx->bufs.probs.addr, "hdr_104_probs_addr_lsb8", 0);

	/* always used */
	pusha(vp9_ctx->bufs.state.addr, "hdr_118_pps0_tile_addr_lsb8", 0);

	 /* read / write segment buffers */
	pusha(vp9_ctx->bufs.seg.addr, "hdr_108_pps1_tile_addr_lsb8", 1);
	pusha(vp9_ctx->bufs.seg.addr, "hdr_108_pps1_tile_addr_lsb8", 2);
	/* ping pong buffers, not on intra frames (how apple uses them) */
	pusha(vp9_ctx->bufs.above_info.addr, "hdr_110_pps2_tile_addr_lsb8", 3);
	pusha(vp9_ctx->bufs.above_info.addr, "hdr_110_pps2_tile_addr_lsb8", 4);

	push(frame->quant.base_q_idx << 15
			| (frame->quant.delta_q_y_dc & 0x1f) << 10
			| (frame->quant.delta_q_uv_dc & 0x1f) << 5
			| (frame->quant.delta_q_uv_ac & 0x1f)
			, "hdr_4c_base_q_idx");
	/* filter related flags? */
	push(frame->lf.sharpness << 28
			| (frame->lf.flags & V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED ?
				(frame->lf.ref_deltas[0] & 0x7f) << 21
				| (frame->lf.ref_deltas[1] & 0x7f) << 14
				| (frame->lf.ref_deltas[2] & 0x7f) << 7
				| (frame->lf.ref_deltas[3] & 0x7f)
				: 0), "hdr_44_flags1_pt2");

	push(frame->lf.level << 14
			| (frame->lf.flags & V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED ?
			(frame->lf.mode_deltas[0] & 0x7f) << 7
			| (frame->lf.mode_deltas[1] & 0x7f)
			: 0),
			"hdr_48_loop_filter_level");

	push(INST_DMA2, "cm3_dma_config_4");
	push(INST_DMA2, "cm3_dma_config_5");

	if (avd->variant->revision == 3)
		push(0, "");
	if (!(avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE))
		pusha(vp9_ctx->bufs.pipe_state.addr, "pipe_state", 0);

	pusha(vp9_ctx->bufs.color[0].addr, "hdr_e8_sps0_tile_addr_lsb8", 0);
	pusha(vp9_ctx->bufs.color[1].addr, "hdr_e8_sps0_tile_addr_lsb8", 0);

	pusha((u64)0, "hdr_e8_sps0_tile_addr_lsb8", 0);

	/* not fatal */
	pusha(vp9_ctx->bufs.tiles[0].addr, "hdr_e8_sps0_tile_addr_lsb8", 0);
	pusha(vp9_ctx->bufs.tiles[1].addr, "hdr_e8_sps0_tile_addr_lsb8", 0);
	/* fatal if missing / wrong */
	pusha(vp9_ctx->bufs.tiles[2].addr, "hdr_e8_sps0_tile_addr_lsb8", 0);

	push(INST_DMA3, "cm3_dma_config_7");

	push_rvra(avd, ctx, run->addresses.rvra, ctx->rvra.offsets);

	/* confusing */
	pusha((u64)0, "hdr_f4_sps1_tile_addr_lsb8", 2);

	bytesperline = ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
	if (avd->variant->quirks & AVD_QUIRK_LSR)
		bytesperline = bytesperline >> 4;

	pusha(run->addresses.y, "hdr_168_y_addr_lsb8", 0);
	push(bytesperline, "hdr_170_width_align");
	pusha(run->addresses.uv, "hdr_16c_uv_addr_lsb8", 0);
	push(bytesperline, "hdr_174_width_align");
	push(0, "");
	push(((frame->frame_height_minus_1) << 16) | (frame->frame_width_minus_1),
			"cm3_height_width");

	if (!(intra_only))
		set_refs(ctx, run);
}


static void set_tiles(struct avd_ctx *ctx, struct avd_vp9_run *run)
{
	const struct v4l2_ctrl_vp9_frame *frame = run->decode_params;
	struct avd_dev *avd = ctx->dev;
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	bool is_last;

	u32 offset =
		frame->uncompressed_header_size + frame->compressed_header_size;
	u32 size = vb2_get_plane_payload(&src->vb2_buf, 0) - offset;
	u32 num_tile_rows = 1 << frame->tile_rows_log2;
	u32 num_tile_cols = 1 << frame->tile_cols_log2;
	u32 tile_size;

	/* 6.2.6 Compute image size syntax */
	u32 sb_64_cols = (((frame->frame_width_minus_1 + 8) >> 3) + 7) >> 3;
	u32 sb_64_rows = (((frame->frame_height_minus_1 + 8) >> 3) + 7) >> 3;

	for (int row = 0; row < num_tile_rows; row++)
		for (int col = 0; col < num_tile_cols; col++) {
			is_last = row == num_tile_rows - 1 && col == num_tile_cols - 1;
			if (row == num_tile_rows - 1 && col == num_tile_cols - 1) {
				tile_size = size;
			} else {
				tile_size = get_unaligned_be32(&data[offset]);
				/* i have crashed my computer because of this */
				if (tile_size > size - 4)
					return;
				offset += 4;
				size -= 4;
			}
			push(0x2d800000 | (u32)(run->addresses.sl >> 32),
					"cm3_cmd_set_slice_data");
			push((u32)((run->addresses.sl + offset) & 0xffffffff),
					"til_ab4_tile_addr_low");
			push(tile_size, "til_ab8_tile_size");
			push(0x2a000000
					| ((row * sb_64_rows) / num_tile_rows) << 12
					| (col * sb_64_cols) / num_tile_cols, "i");

			push(col << 24
					| (((row + 1) * sb_64_rows) / num_tile_rows - 1) << 12
					| (((col + 1) * sb_64_cols) / num_tile_cols - 1)
					, "til_ac0_tile_dims");
			push(0x2b000000
					| (is_last ? 0x400 : 0)
					| (avd->variant->revision == 3 ? 0xfff000 : 0)
					, "cm3_cmd_inst_fifo_end");
			offset += tile_size;
			size -= tile_size;
			vp9_ctx->submit_num++;
		}
}


static void update_dec_buf_info(struct avd_decoded_buffer *buf,
				const struct v4l2_ctrl_vp9_frame *dec_params)
{
	buf->vp9.width = dec_params->frame_width_minus_1 + 1;
	buf->vp9.height = dec_params->frame_height_minus_1 + 1;
	buf->vp9.bit_depth = dec_params->bit_depth;
}

static void update_ctx_cur_info(struct avd_vp9_ctx *vp9_ctx,
				struct avd_decoded_buffer *buf,
				const struct v4l2_ctrl_vp9_frame *dec_params)
{
	vp9_ctx->cur.valid = true;
	vp9_ctx->cur.reference_mode = dec_params->reference_mode;
	vp9_ctx->cur.interpolation_filter = dec_params->interpolation_filter;
	vp9_ctx->cur.flags = dec_params->flags;
	vp9_ctx->cur.timestamp = buf->base.vb.vb2_buf.timestamp;
	vp9_ctx->cur.seg = dec_params->seg;
	vp9_ctx->cur.lf = dec_params->lf;
}

static void update_ctx_last_info(struct avd_vp9_ctx *vp9_ctx)
{
	vp9_ctx->last = vp9_ctx->cur;
}

static void copy_vp9_frame_mv(struct avd_vp9_probs *avd_probs,
		const struct v4l2_vp9_frame_context *probs)
{
	memcpy(avd_probs->joint, probs->mv.joint, sizeof(avd_probs->joint));
	for (int i = 0; i < 2; i++) {
		avd_probs->mv_comp[i].sign = probs->mv.sign[i];
		memcpy(avd_probs->mv_comp[i].bits, probs->mv.bits[i],
				sizeof(avd_probs->mv_comp[i].bits));
		avd_probs->mv_comp[i].class0_bit = probs->mv.class0_bit[i];
		memcpy(avd_probs->mv_comp[i].classes, probs->mv.classes[i],
				sizeof(avd_probs->mv_comp[i].bits));

		memcpy(avd_probs->mv_fr[i].class0_fr, probs->mv.class0_fr[i],
				sizeof(avd_probs->mv_fr[i].class0_fr));
		memcpy(avd_probs->mv_fr[i].fr, probs->mv.fr[i],
				sizeof(avd_probs->mv_fr[i].fr));

		avd_probs->mv_hp[i].class0_hp = probs->mv.class0_hp[i];
		avd_probs->mv_hp[i].hp = probs->mv.hp[i];
	}
}

static void init_probs(struct avd_ctx *ctx,
		       const struct avd_vp9_run *run)
{
	const struct v4l2_ctrl_vp9_frame *dec_params;
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	struct avd_vp9_probs *avd_probs = vp9_ctx->bufs.probs.cpu;
	const struct v4l2_vp9_segmentation *seg;
	const struct v4l2_vp9_frame_context *probs;
	bool intra_only;
	int count = 0;

	dec_params = run->decode_params;
	probs = &vp9_ctx->probability_tables;
	seg = &dec_params->seg;

	memset(avd_probs, 0, sizeof(*avd_probs));

	intra_only = !!(dec_params->flags &
			(V4L2_VP9_FRAME_FLAG_KEY_FRAME | V4L2_VP9_FRAME_FLAG_INTRA_ONLY));

	memcpy(avd_probs->tx8, probs->tx8, sizeof(avd_probs->tx8));
	memcpy(avd_probs->tx16, probs->tx16, sizeof(avd_probs->tx16));
	memcpy(avd_probs->tx32, probs->tx32, sizeof(avd_probs->tx32));
	memcpy(avd_probs->skip, probs->skip, sizeof(avd_probs->skip));
	memcpy(avd_probs->inter_mode, probs->inter_mode,
			sizeof(avd_probs->inter_mode));
	memcpy(avd_probs->interp_filter, probs->interp_filter,
			sizeof(avd_probs->interp_filter));
	memcpy(avd_probs->is_inter, probs->is_inter, sizeof(avd_probs->is_inter));
	memcpy(avd_probs->comp_mode, probs->comp_mode,
			sizeof(avd_probs->comp_mode));
	memcpy(avd_probs->single_ref, probs->single_ref,
			sizeof(avd_probs->single_ref));
	memcpy(avd_probs->comp_ref, probs->comp_ref, sizeof(avd_probs->comp_ref));
	memcpy(avd_probs->y_mode, probs->y_mode, sizeof(avd_probs->y_mode));

	memcpy(avd_probs->partition,
	       intra_only ? v4l2_vp9_kf_partition_probs : probs->partition,
	       sizeof(avd_probs->partition));
	memcpy(avd_probs->uv_mode,
			intra_only ? v4l2_vp9_kf_uv_mode_prob : probs->uv_mode,
			sizeof(avd_probs->uv_mode));

	copy_vp9_frame_mv(avd_probs, probs);

	/* please mister gcc optimise this away */
	for (int t = 0; t < 4; t++)
		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 2; j++)
				for (int k = 0; k < 6; k++) {
					int max_l = (k == 0) ? 3 : 6;
					for (int l = 0; l < max_l; l++) {
						for (int n = 0; n < 3; n++)
							avd_probs->coef[count++] =
								probs->coef[t][i][j][k][l][n];
					}
				}

	memcpy(avd_probs->seg.pred_probs, seg->pred_probs,
			sizeof(avd_probs->seg.pred_probs));
	memcpy(avd_probs->seg.tree_probs, seg->tree_probs,
			sizeof(avd_probs->seg.tree_probs));
}

static int validate_dec_params(struct avd_ctx *ctx,
			       const struct v4l2_ctrl_vp9_frame *dec_params)
{
	unsigned int aligned_width, aligned_height;

	if (dec_params->frame_height_minus_1 + 1 < 64 ||
			dec_params->frame_width_minus_1 + 1 < 64)
		return -EINVAL;

	aligned_width = round_up(dec_params->frame_width_minus_1 + 1, 64);
	aligned_height = round_up(dec_params->frame_height_minus_1 + 1, 16);

	/*
	 * Userspace should update the capture/decoded format when the
	 * resolution changes.
	 */
	if (aligned_width != ctx->decoded_fmt.fmt.pix_mp.width ||
	    aligned_height != ctx->decoded_fmt.fmt.pix_mp.height) {
		dev_err(ctx->dev->dev, "unexpected bitstream resolution %dx%d\n",
			aligned_width, aligned_height);
		return -EINVAL;
	}

	return 0;
}

static int avd_vp9_alloc_bufs(struct avd_ctx *ctx)
{
	struct avd_dev *avd = ctx->dev;
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	int ret, size;

	ret = avd_buf_alloc(avd, &vp9_ctx->bufs.inst, fifo_size());
	if (ret)
		return ret;

	if (!(avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE)) {
		ret = avd_buf_alloc(avd, &vp9_ctx->bufs.pipe_state, 0x200);
		if (ret)
			return ret;
	}

	ret = avd_buf_alloc(avd, &vp9_ctx->bufs.seg, 0x200);
	if (ret)
		return ret;

	/* scratch buffer? */
	ret = avd_buf_alloc(avd, &vp9_ctx->bufs.tiles[1], 0x200);
	if (ret)
		return ret;

	/* TODO */
	size = (((ctx->decoded_fmt.fmt.pix_mp.height - 1)
		  * (ctx->decoded_fmt.fmt.pix_mp.height - 1) / 0x10000) + 2) * 0x4000;
	ret = avd_buf_alloc(avd, &vp9_ctx->bufs.tiles[0], size);
	if (ret)
		return ret;
	ret = avd_buf_alloc(avd, &vp9_ctx->bufs.tiles[2], size);
	if (ret)
		return ret;


	for (int i = 0; i < 2; i++) {
		ret = avd_buf_alloc(avd, &vp9_ctx->bufs.color[i], size);
		if (ret)
			return ret;
	}

	ret = avd_buf_alloc(avd, &vp9_ctx->bufs.above_info, size + 0xc000);
	if (ret)
		return ret;

	ret = avd_buf_alloc(avd, &vp9_ctx->bufs.state, size);
	if (ret)
		return ret;

	ret = avd_buf_alloc(avd, &vp9_ctx->bufs.probs,
			sizeof(struct avd_vp9_probs));
	if (ret)
		return ret;

	ret = avd_buf_alloc(avd, &vp9_ctx->bufs.counts,
			sizeof(struct avd_vp9_frame_symbol_counts));
	if (ret)
		return ret;

	return 0;
}

static int avd_vp9_run_preamble(struct avd_ctx *ctx,
				     struct avd_vp9_run *run)
{
	struct v4l2_ctrl *ctrl;
	const struct v4l2_ctrl_vp9_frame *dec_params;
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	unsigned int fctx_idx;
	u32 dst_len;
	int ret;

	avd_run_preamble(ctx, &run->base);

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_VP9_COMPRESSED_HDR);
	if (WARN_ON(!ctrl))
		return -EINVAL;
	run->prob_updates = ctrl->p_cur.p;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_VP9_FRAME);
	if (WARN_ON(!ctrl))
		return -EINVAL;
	dec_params = ctrl->p_cur.p;

	ret = validate_dec_params(ctx, dec_params);
	if (ret)
		return ret;

	run->decode_params = dec_params;

	vp9_ctx->cur.tx_mode = run->prob_updates->tx_mode;

	fctx_idx = v4l2_vp9_reset_frame_ctx(dec_params, vp9_ctx->frame_context);
	vp9_ctx->cur.frame_context_idx = fctx_idx;

	vp9_ctx->probability_tables = vp9_ctx->frame_context[fctx_idx];
	v4l2_vp9_fw_update_probs(&vp9_ctx->probability_tables, run->prob_updates,
			dec_params);

	run->addresses.sl =
		vb2_dma_contig_plane_dma_addr(&run->base.bufs.src->vb2_buf, 0);

	dst_len = run->base.bufs.dst->vb2_buf.planes[0].length;

	run->addresses.y =
		vb2_dma_contig_plane_dma_addr(&run->base.bufs.dst->vb2_buf, 0);

	run->addresses.uv = run->addresses.y +
		ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline *
			ALIGN(ctx->decoded_fmt.fmt.pix_mp.height, 16);

	run->addresses.rvra = run->addresses.y + (dst_len - ctx->rvra.size);

	return 0;
}

static int avd_vp9_run(struct avd_ctx *ctx)
{
	struct avd_dev *avd = ctx->dev;
	struct avd_vp9_run run;
	struct avd_vp9_ctx *vp9_ctx;
	struct avd_decoded_buffer *dst;
	int ret;

	ret = avd_vp9_run_preamble(ctx, &run);
	if (ret) {
		avd_run_postamble(ctx, &run.base);
		return ret;
	}

	init_probs(ctx, &run);

	vp9_ctx = ctx->priv;
	dst = vb2_to_avd_decoded_buf(&run.base.bufs.dst->vb2_buf);
	update_dec_buf_info(dst, run.decode_params);
	update_ctx_cur_info(vp9_ctx, dst, run.decode_params);

	ret = alloc_slots(avd, ctx, AVD_CODEC_VP9);
	if (ret) {
		dev_err(avd->dev, "no free slots: %d", ret);
		return ret;
	}

	schedule_delayed_work(&ctx->watchdog_work, msecs_to_jiffies(2000));

	avd->variant->configure_stream(avd, vp9_ctx->bufs.inst.addr,
			ctx->fifo_idx, ctx->vp_slot);

	set_header(ctx, &run);

	vp9_ctx->submit_num = 0;
	set_tiles(ctx, &run);
	avd_run_postamble(ctx, &run.base);

	return 0;
}

#define copy_tx_and_skip(p1, p2)				\
do {								\
	memcpy((p1)->tx8, (p2)->tx8, sizeof((p1)->tx8));	\
	memcpy((p1)->tx16, (p2)->tx16, sizeof((p1)->tx16));	\
	memcpy((p1)->tx32, (p2)->tx32, sizeof((p1)->tx32));	\
	memcpy((p1)->skip, (p2)->skip, sizeof((p1)->skip));	\
} while (0)


static void avd_vp9_done(struct avd_ctx *ctx,
			    struct vb2_v4l2_buffer *src_buf,
			    struct vb2_v4l2_buffer *dst_buf,
			    enum vb2_buffer_state result)
{
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	unsigned int fctx_idx;

	/* v4l2-specific stuff */
	if (result == VB2_BUF_STATE_ERROR)
		goto out_update_last;

	/*
	 * vp9 stuff
	 *
	 * 6.1.2 refresh_probs()
	 *
	 * In the spec a complementary condition goes last in 6.1.2 refresh_probs(),
	 * but it makes no sense to perform all the activities from the first "if"
	 * there if we actually are not refreshing the frame context. On top of that,
	 * because of 6.2 uncompressed_header() whenever error_resilient_mode == 1,
	 * refresh_frame_context == 0. Consequently, if we don't jump to out_update_last
	 * it means error_resilient_mode must be 0.
	 */
	if (!(vp9_ctx->cur.flags & V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX))
		goto out_update_last;

	fctx_idx = vp9_ctx->cur.frame_context_idx;

	if (!(vp9_ctx->cur.flags & V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE)) {
		/* error_resilient_mode == 0 && frame_parallel_decoding_mode == 0 */
		struct v4l2_vp9_frame_context *probs = &vp9_ctx->probability_tables;
		bool frame_is_intra = vp9_ctx->cur.flags &
		    (V4L2_VP9_FRAME_FLAG_KEY_FRAME | V4L2_VP9_FRAME_FLAG_INTRA_ONLY);
		struct tx_and_skip {
			u8 tx8[2][1];
			u8 tx16[2][2];
			u8 tx32[2][3];
			u8 skip[3];
		} _tx_skip, *tx_skip = &_tx_skip;
		struct v4l2_vp9_frame_symbol_counts *counts;

		/* buffer the forward-updated TX and skip probs */
		if (frame_is_intra)
			copy_tx_and_skip(tx_skip, probs);

		/* 6.1.2 refresh_probs(): load_probs() and load_probs2() */
		*probs = vp9_ctx->frame_context[fctx_idx];

		/* if FrameIsIntra then undo the effect of load_probs2() */
		if (frame_is_intra)
			copy_tx_and_skip(probs, tx_skip);

		counts = &vp9_ctx->cnts;

		v4l2_vp9_adapt_coef_probs(probs, counts,
					  !vp9_ctx->last.valid ||
					  vp9_ctx->last.flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME,
					  frame_is_intra);
		if (!frame_is_intra) {
			const struct avd_vp9_frame_symbol_counts *cnts;
			int i;
			u32 tx16p[2][4];
			u32 sign[2][2];
			u32 classes[2][11];
			u32 class0[2][2];
			u32 bits[2][10][2];
			u32 class0_fp[2][2][4];
			u32 fp[2][4];
			u32 class0_hp[2][2];
			u32 hp[2][2];

			cnts = vp9_ctx->bufs.counts.cpu;

			for (i = 0; i < ARRAY_SIZE(cnts->tx16p); ++i)
				memcpy(tx16p[i], cnts->tx16p[i], sizeof(tx16p[0]));

			for (i = 0; i < 2; i++) {
				memcpy(sign[i], cnts->mv_comp[i].sign, sizeof(sign[0]));
				memcpy(classes[i], cnts->mv_comp[i].classes, sizeof(classes[0]));
				memcpy(class0[i], cnts->mv_comp[i].class0, sizeof(class0[0]));
				memcpy(bits[i], cnts->mv_comp[i].bits, sizeof(bits[0]));
				memcpy(class0_fp[i], cnts->mv_fr[i].class0_fr,
						sizeof(class0_fp[0]));
				memcpy(fp[i], cnts->mv_fr[i].fr, sizeof(fp[0]));
				memcpy(class0_hp[i], cnts->mv_hp[i].class0_hp,
						sizeof(class0_hp[0]));
				memcpy(hp[i], cnts->mv_hp[i].hp, sizeof(hp[0]));
			}

			counts->tx16p = &tx16p;
			counts->sign = &sign;
			counts->classes = &classes;
			counts->class0 = &class0;
			counts->bits = &bits;
			counts->class0_fp = &class0_fp;
			counts->fp = &fp;
			counts->class0_hp = &class0_hp;
			counts->hp = &hp;

			/* load_probs2() already done */
			v4l2_vp9_adapt_noncoef_probs(&vp9_ctx->probability_tables, counts,
						     vp9_ctx->cur.reference_mode,
						     vp9_ctx->cur.interpolation_filter,
						     vp9_ctx->cur.tx_mode, vp9_ctx->cur.flags);
		}
	}

	/* 6.1.2 refresh_probs(): save_probs(fctx_idx) */
	vp9_ctx->frame_context[fctx_idx] = vp9_ctx->probability_tables;

out_update_last:
	update_ctx_last_info(vp9_ctx);
}

static noinline_for_stack void
avd_init_v4l2_vp9_count_tbl(struct avd_ctx *ctx)
{
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	struct avd_vp9_frame_symbol_counts *cnts = vp9_ctx->bufs.counts.cpu;
	int coeff_cnts = 0, eob_cnts = 0;

	vp9_ctx->cnts.intra_inter = &cnts->intra_inter;
	vp9_ctx->cnts.y_mode = &cnts->y_mode;
	vp9_ctx->cnts.uv_mode = &cnts->uv_mode;
	vp9_ctx->cnts.comp = &cnts->comp;
	vp9_ctx->cnts.comp_ref = &cnts->comp_ref;
	vp9_ctx->cnts.single_ref = &cnts->single_ref;
	vp9_ctx->cnts.filter = &cnts->filter;
	vp9_ctx->cnts.mv_mode = &cnts->mv_mode;
	vp9_ctx->cnts.mv_joint = &cnts->mv_joint;

	/* all of the mv use a different structure, so they must all be copied */

	vp9_ctx->cnts.tx8p = &cnts->tx8p;
	/* avd also uses "u32 tx16p[2][3]" instead of "u32 tx16p[2][4]" */
	vp9_ctx->cnts.tx32p = &cnts->tx32p;

	/* it would suck if we need to reverse partition */
	vp9_ctx->cnts.partition = &cnts->partition;
	/* these should work */
	vp9_ctx->cnts.skip = &cnts->skip;

	for (int t = 0; t < 4; t++)
		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 2; j++)
				for (int k = 0; k < 6; k++) {
					int max_l = (k == 0) ? 3 : 6;
					for (int l = 0; l < max_l; l++) {
						vp9_ctx->cnts.coeff[t][i][j][k][l] =
							(u32 (*)[3])&cnts->ref_cnt[coeff_cnts];
						coeff_cnts += 3;
						vp9_ctx->cnts.eob[t][i][j][k][l][0] =
							&cnts->eob_0[eob_cnts++];
						vp9_ctx->cnts.eob[t][i][j][k][l][1] =
							&cnts->ref_cnt[coeff_cnts++];
					}
				}
}

static int avd_vp9_start(struct avd_ctx *ctx)
{
	struct avd_vp9_ctx *vp9_ctx;
	int ret;

	vp9_ctx = kzalloc(sizeof(*vp9_ctx), GFP_KERNEL);
	if (!vp9_ctx)
		return -ENOMEM;

	ctx->priv = vp9_ctx;
	ret = avd_vp9_alloc_bufs(ctx);
	if (ret)
		goto err_free_ctx;

	avd_init_v4l2_vp9_count_tbl(ctx);

	return 0;

err_free_ctx:
	kfree(vp9_ctx);
	ctx->priv = NULL;
	return ret;
}

static void avd_vp9_stop(struct avd_ctx *ctx)
{
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	struct avd_dev *avd = ctx->dev;

	avd_buf_free(avd, &vp9_ctx->bufs.pipe_state);
	avd_buf_free(avd, &vp9_ctx->bufs.inst);
	avd_buf_free(avd, &vp9_ctx->bufs.probs);
	avd_buf_free(avd, &vp9_ctx->bufs.counts);
	avd_buf_free(avd, &vp9_ctx->bufs.seg);
	avd_buf_free(avd, &vp9_ctx->bufs.above_info);
	avd_buf_free(avd, &vp9_ctx->bufs.color[0]);
	avd_buf_free(avd, &vp9_ctx->bufs.color[1]);
	avd_buf_free(avd, &vp9_ctx->bufs.state);
	for (int i = 0; i < 3; i++)
		avd_buf_free(avd, &vp9_ctx->bufs.tiles[i]);

	kfree(vp9_ctx);
}

static enum avd_image_fmt avd_vp9_get_image_fmt(struct avd_ctx *ctx,
						 struct v4l2_ctrl *ctrl)
{
#define BIT_DEPTH(chroma) \
	(frame->bit_depth == 8 ? AVD_IMG_FMT_##chroma##_8BIT :  \
	 AVD_IMG_FMT_##chroma##_10BIT )
	const struct v4l2_ctrl_vp9_frame *frame = ctrl->p_new.p_vp9_frame;

	if (ctrl->id != V4L2_CID_STATELESS_VP9_FRAME)
		return AVD_IMG_FMT_ANY;

	/* 7.2.2 Color config semantics */
	if (frame->flags & V4L2_VP9_FRAME_FLAG_X_SUBSAMPLING) {
		if (frame->flags & V4L2_VP9_FRAME_FLAG_Y_SUBSAMPLING)
			return BIT_DEPTH(420);
		else
			return BIT_DEPTH(422);
	}

	return AVD_IMG_FMT_ANY;
#undef BIT_DEPTH
}

static void avd_vp9_submit(struct avd_ctx *ctx)
{
	struct avd_vp9_ctx *vp9_ctx = ctx->priv;
	struct avd_dev *avd = ctx->dev;
	u32 submit_mask = ctx->dev->variant->revision == 3 ? 0xfff000 : 0;

	writel(0x2b000000
			| submit_mask
			| (avd->variant->revision == 3 ? 0x100 : 0x200)
			| (ctx->fifo_idx << 4)
			| avd->variant->fifo_slots,
			avd->ctrl + avd->variant->submit_offset);
	for (int i = 0; i < vp9_ctx->submit_num - 1; i++)
		writel(0x2b000000
				| submit_mask
				| (ctx->fifo_idx << 4)
				| avd->variant->fifo_slots,
				avd->ctrl + avd->variant->submit_offset);
}

const struct avd_coded_fmt_ops avd_vp9_fmt_ops = {
	.start = avd_vp9_start,
	.stop = avd_vp9_stop,
	.run = avd_vp9_run,
	.done = avd_vp9_done,
	.submit = avd_vp9_submit,
	.get_image_fmt = avd_vp9_get_image_fmt,
};
