/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple AVD VPU codec driver
 *
 * Copyright The Asahi Linux Contributors
 * Copyright 2023 Eileen Yoon <eyn@gmx.com>
 *
 * Copyright (c) 2023, Collabora
 *	Benjamin Gaignard <benjamin.gaignard@collabora.com>
 *
 * Copyright (c) 2014 Rockchip Electronics Co., Ltd.
 *	Hertz Wong <hertz.wong@rock-chips.com>
 *	Herman Chen <herman.chen@rock-chips.com>
 *
 * Copyright (C) 2014 Google, Inc.
 *	Tomasz Figa <tfiga@chromium.org>
 */

#include "linux/v4l2-controls.h"
#include <linux/unaligned.h>
#include <linux/delay.h>

#include <media/videobuf2-dma-contig.h>

#include "avd.h"
#include "avd-av1-entropymode.h"
#include "avd-inst.h"

/*
 * This comes from
 * verisilicon/rockchip_vpu981_hw_av1_dec.c
 *
 * These 3 values aren't defined enum v4l2_av1_segment_feature because
 * they are not part of the specification
 */
#define V4L2_AV1_SEG_LVL_ALT_LF_Y_H	2
#define V4L2_AV1_SEG_LVL_ALT_LF_U	3
#define V4L2_AV1_SEG_LVL_ALT_LF_V	4

#define AV1_DIV_ROUND_UP_POW2(value, n)			\
({							\
	typeof(n) _n  = n;				\
	typeof(value) _value = value;			\
	(_value + (BIT(_n) >> 1)) >> _n;		\
})

#define AV1_DIV_ROUND_UP_POW2_SIGNED(value, n)				\
({									\
	typeof(n) _n_  = n;						\
	typeof(value) _value_ = value;					\
	(((_value_) < 0) ? -AV1_DIV_ROUND_UP_POW2(-(_value_), (_n_))	\
		: AV1_DIV_ROUND_UP_POW2((_value_), (_n_)));		\
})

#define DIV_LUT_BITS 8
#define DIV_LUT_NUM BIT(DIV_LUT_BITS)
#define DIV_LUT_PREC_BITS 14
#define WARPEDMODEL_PREC_BITS 16
#define WARP_PARAM_REDUCE_BITS 6

#define AV1_CODEC_MODE_INTRABC(v)	FIELD_PREP(BIT(28), !!(v))

#define AV1_HDR_JNT_COMP(v)		FIELD_PREP(BIT(0), !!(v))
#define AV1_HDR_DUAL_FILTER(v)		FIELD_PREP(BIT(5), !!(v))
#define AV1_HDR_MASKED_COMPOUND(v)	FIELD_PREP(BIT(6), !!(v))
#define AV1_HDR_INTRA_EDGE_FILTER(v)	FIELD_PREP(BIT(8), !!(v))
#define AV1_HDR_128X128_SUPERBLOCK(v)	FIELD_PREP(BIT(10), !!(v))
#define AV1_HDR_SCREEN_CONTENT_TOOLS(v)	FIELD_PREP(BIT(11), !!(v))

#define AV1_FLAGS_TX_MODE_LARGEST(v)		FIELD_PREP(BIT(0), !!(v))
#define AV1_FLAGS_TX_MODE_SELECT(v)		FIELD_PREP(BIT(1), !!(v))
#define AV1_FLAGS_TX_REDUCED_SET(v)		FIELD_PREP(BIT(2), !!(v))
#define AV1_FLAGS_ALLOW_WARPED_MOTION(v)	FIELD_PREP(BIT(3), !!(v))
#define AV1_FLAGS_SKIP_MODE(v)			FIELD_PREP(BIT(4), !!(v))
#define AV1_FLAGS_INTRABC(v)			FIELD_PREP(BIT(6), !!(v))
#define AV1_FLAGS_INTEGER_MV(v)			FIELD_PREP(BIT(7), !!(v))
#define AV1_FLAGS_DISABLE_CDF_UPDATE(v)		FIELD_PREP(BIT(8), !!(v))
#define AV1_FLAGS_TX_MODE_ONLY_4X4(v)		FIELD_PREP(BIT(9), !!(v))
#define AV1_FLAGS_HIGH_PRECISION_MV(v)		FIELD_PREP(BIT(10), !!(v))
#define AV1_FLAGS_SEG_TEMPORAL_UPDATE(v)	FIELD_PREP(BIT(13), !!(v))
#define AV1_FLAGS_SEG_UPDATE_MAP(v)		FIELD_PREP(BIT(14), !!(v))
#define AV1_FLAGS_SEG_ENABLED(v)		FIELD_PREP(BIT(15), !!(v))
#define AV1_FLAGS_SEG_UPDATE_DATA(v)		FIELD_PREP(BIT(16), !!(v))
#define AV1_FLAGS_INTRA(v)			FIELD_PREP(BIT(31), !!(v))
#define AV1_FLAGS_ORDER_HINT(v)			FIELD_PREP(GENMASK(30, 23), v)

#define AV1_FLAGS_REFERENCE_SELECT(v)		FIELD_PREP(BIT(28), !!(v))
#define AV1_FLAGS_INTERPOLATION_FILTER(v)	FIELD_PREP(GENMASK(31, 29), v)

#define AV1_SEG_ALT_Q(v)	FIELD_PREP(GENMASK(8, 0), v)
#define AV1_SEG_ALT_Q_EN(v)	FIELD_PREP(BIT(9), !!(v))
#define AV1_SEG_REF_FRAME(v)	FIELD_PREP(GENMASK(12, 10), v)
#define AV1_SEG_REF_FRAME_EN(v)	FIELD_PREP(BIT(13), !!(v))
#define AV1_SEG_REF_SKIP_EN(v)	FIELD_PREP(BIT(14), !!(v))
#define AV1_SEG_REF_GMV_EN(v)	FIELD_PREP(BIT(15), !!(v))

#define AV1_SEG_LF_V(v)		FIELD_PREP(GENMASK(6, 0), v)
#define AV1_SEG_LF_V_EN(v)	FIELD_PREP(BIT(7), !!(v))
#define AV1_SEG_LF_U(v)		FIELD_PREP(GENMASK(14, 8), v)
#define AV1_SEG_LF_U_EN(v)	FIELD_PREP(BIT(15), !!(v))
#define AV1_SEG_LF_Y_H(v)	FIELD_PREP(GENMASK(22, 16), v)
#define AV1_SEG_LF_Y_H_EN(v)	FIELD_PREP(BIT(23), !!(v))
#define AV1_SEG_LF_Y_V(v)	FIELD_PREP(GENMASK(30, 24), v)
#define AV1_SEG_LF_Y_V_EN(v)	FIELD_PREP(BIT(31), !!(v))

#define AV1_SEG_UNK0(v)		FIELD_PREP(BIT(16), !!(v))

#define AV1_QP_U_DC(v)		FIELD_PREP(GENMASK(13, 7), v)
#define AV1_QP_Y_DC(v)		FIELD_PREP(GENMASK(20, 14), v)
#define AV1_QP_BASE_IDX(v)	FIELD_PREP(GENMASK(28, 21), v)
#define AV1_QP_DELTA_RES(v)	FIELD_PREP(GENMASK(30, 29), v)
#define AV1_QP_PRESENT(v)	FIELD_PREP(BIT(31), !!(v))

#define AV1_GM_VALID(v)		FIELD_PREP(BIT(30), !!(v))
#define AV1_GM_TYPE(v)		FIELD_PREP(GENMASK(31, 30), v)
#define AV1_GM_PARAM0(v)	FIELD_PREP(GENMASK(29, 15), v)
#define AV1_GM_PARAM1(v)	FIELD_PREP(GENMASK(14, 0), v)

#define AV1_GM_PARAM_SHEAR0(v)	FIELD_PREP(GENMASK(15, 0), v)
#define AV1_GM_PARAM_SHEAR1(v)	FIELD_PREP(GENMASK(31, 16), v)

#define AV1_ORDER_HINT0(v)	FIELD_PREP(GENMASK(7, 0), v)
#define AV1_ORDER_HINT1(v)	FIELD_PREP(GENMASK(15, 8), v)
#define AV1_ORDER_HINT2(v)	FIELD_PREP(GENMASK(23, 16), v)
#define AV1_ORDER_HINT3(v)	FIELD_PREP(GENMASK(31, 24), v)

#define AV1_LF_DELTA_ENABLED(v)	FIELD_PREP(BIT(27), !!(v))
#define AV1_LF_LV3(v)		FIELD_PREP(GENMASK(5, 0), v)
#define AV1_LF_LV2(v)		FIELD_PREP(GENMASK(11, 6), v)
#define AV1_LF_LV1(v)		FIELD_PREP(GENMASK(17, 12), v)
#define AV1_LF_LV0(v)		FIELD_PREP(GENMASK(23, 18), v)

#define AV1_LF_SHARPNESS(v)	FIELD_PREP(GENMASK(31, 28), v)
#define AV1_LF_REF0(v)		FIELD_PREP(GENMASK(27, 21), v)
#define AV1_LF_REF1(v)		FIELD_PREP(GENMASK(20, 14), v)
#define AV1_LF_REF2(v)		FIELD_PREP(GENMASK(13, 7), v)
#define AV1_LF_REF3(v)		FIELD_PREP(GENMASK(6, 0), v)

#define AV1_LF_LV(v)		FIELD_PREP(GENMASK(31, 14), v)
#define AV1_LF_MODE0(v)		FIELD_PREP(GENMASK(13, 7), v)
#define AV1_LF_MODE1(v)		FIELD_PREP(GENMASK(6, 0), v)

#define AV1_CDEF_EN(v)		FIELD_PREP(BIT(28), !!(v))
#define AV1_CDEF_BITS(v)	FIELD_PREP(GENMASK(25, 24), v)
#define AV1_CDEF_DAMPING(v)	FIELD_PREP(GENMASK(27, 26), v)

#define AV1_CDEF_Y_PRI(v)	FIELD_PREP(GENMASK(11, 8), v)
#define AV1_CDEF_Y_SEC(v)	FIELD_PREP(GENMASK(7, 6), v)
#define AV1_CDEF_UV_PRI(v)	FIELD_PREP(GENMASK(5, 2), v)
#define AV1_CDEF_UV_SEC(v)	FIELD_PREP(GENMASK(1, 0), v)

#define AV1_CDEF_LO(v)	FIELD_PREP(GENMASK(11, 0), v)
#define AV1_CDEF_HI(v)	FIELD_PREP(GENMASK(23, 12), v)

#define AV1_LR_TYPE0(v)	FIELD_PREP(GENMASK(11, 10), v)
#define AV1_LR_TYPE1(v)	FIELD_PREP(GENMASK(9, 8), v)
#define AV1_LR_TYPE2(v)	FIELD_PREP(GENMASK(7, 6), v)

#define AV1_LR_UNIT0(v)	FIELD_PREP(GENMASK(1, 0), v)
#define AV1_LR_UNIT1(v)	FIELD_PREP(GENMASK(3, 2), v)
#define AV1_LR_UNIT2(v)	FIELD_PREP(GENMASK(5, 4), v)

struct avd_av1_run {
	struct avd_run base;

	struct {
		dma_addr_t y;
		dma_addr_t uv;
		dma_addr_t sl;
		dma_addr_t rvra;
	} addresses;

	const struct v4l2_ctrl_av1_sequence *seq;
	const struct v4l2_ctrl_av1_frame *frame;
	const struct v4l2_ctrl_av1_tile_group_entry *tile_group;
	const struct v4l2_ctrl_av1_film_grain *grain;
};

struct avd_av1_ctx {
	u8 submit_num;
	struct {
		struct avd_buf inst;
		struct avd_buf pipe_state;
		struct avd_buf unk[5];
		struct avd_buf const_unk[12];
		struct avd_buf ref;
		struct avd_buf probs[2];
	} bufs;
};

/*
 * from verisilicon/rockchip_vpu981_hw_av1_dec.c
 */
static const short div_lut[DIV_LUT_NUM + 1] = {
	16384, 16320, 16257, 16194, 16132, 16070, 16009, 15948, 15888, 15828, 15768,
	15709, 15650, 15592, 15534, 15477, 15420, 15364, 15308, 15252, 15197, 15142,
	15087, 15033, 14980, 14926, 14873, 14821, 14769, 14717, 14665, 14614, 14564,
	14513, 14463, 14413, 14364, 14315, 14266, 14218, 14170, 14122, 14075, 14028,
	13981, 13935, 13888, 13843, 13797, 13752, 13707, 13662, 13618, 13574, 13530,
	13487, 13443, 13400, 13358, 13315, 13273, 13231, 13190, 13148, 13107, 13066,
	13026, 12985, 12945, 12906, 12866, 12827, 12788, 12749, 12710, 12672, 12633,
	12596, 12558, 12520, 12483, 12446, 12409, 12373, 12336, 12300, 12264, 12228,
	12193, 12157, 12122, 12087, 12053, 12018, 11984, 11950, 11916, 11882, 11848,
	11815, 11782, 11749, 11716, 11683, 11651, 11619, 11586, 11555, 11523, 11491,
	11460, 11429, 11398, 11367, 11336, 11305, 11275, 11245, 11215, 11185, 11155,
	11125, 11096, 11067, 11038, 11009, 10980, 10951, 10923, 10894, 10866, 10838,
	10810, 10782, 10755, 10727, 10700, 10673, 10645, 10618, 10592, 10565, 10538,
	10512, 10486, 10460, 10434, 10408, 10382, 10356, 10331, 10305, 10280, 10255,
	10230, 10205, 10180, 10156, 10131, 10107, 10082, 10058, 10034, 10010, 9986,
	9963,  9939,  9916,  9892,  9869,  9846,  9823,  9800,  9777,  9754,  9732,
	9709,  9687,  9664,  9642,  9620,  9598,  9576,  9554,  9533,  9511,  9489,
	9468,  9447,  9425,  9404,  9383,  9362,  9341,  9321,  9300,  9279,  9259,
	9239,  9218,  9198,  9178,  9158,  9138,  9118,  9098,  9079,  9059,  9039,
	9020,  9001,  8981,  8962,  8943,  8924,  8905,  8886,  8867,  8849,  8830,
	8812,  8793,  8775,  8756,  8738,  8720,  8702,  8684,  8666,  8648,  8630,
	8613,  8595,  8577,  8560,  8542,  8525,  8508,  8490,  8473,  8456,  8439,
	8422,  8405,  8389,  8372,  8355,  8339,  8322,  8306,  8289,  8273,  8257,
	8240,  8224,  8208,  8192,
};

static inline int avd_av1_dec_get_msb(u32 n)
{
	if (n == 0)
		return 0;
	return 31 ^ __builtin_clz(n);
}

static short avd_av1_dec_resolve_divisor_32(u32 d, short *shift)
{
	int f;
	u64 e;

	*shift = avd_av1_dec_get_msb(d);
	/* e is obtained from D after resetting the most significant 1 bit. */
	e = d - ((u32)1 << *shift);
	/* Get the most significant DIV_LUT_BITS (8) bits of e into f */
	if (*shift > DIV_LUT_BITS)
		f = AV1_DIV_ROUND_UP_POW2(e, *shift - DIV_LUT_BITS);
	else
		f = e << (DIV_LUT_BITS - *shift);
	if (f > DIV_LUT_NUM)
		return -1;
	*shift += DIV_LUT_PREC_BITS;
	/* Use f as lookup into the precomputed table of multipliers */
	return div_lut[f];
}

static void
avd_av1_dec_get_shear_params(const u32 *params, s64 *alpha,
					 s64 *beta, s64 *gamma, s64 *delta)
{
	const int *mat = params;
	short shift;
	short y;
	long long gv, dv;

	if (mat[2] <= 0)
		return;

	*alpha = clamp_val(mat[2] - (1 << WARPEDMODEL_PREC_BITS), S16_MIN, S16_MAX);
	*beta = clamp_val(mat[3], S16_MIN, S16_MAX);

	y = avd_av1_dec_resolve_divisor_32(abs(mat[2]), &shift) * (mat[2] < 0 ? -1 : 1);

	gv = ((long long)mat[4] * (1 << WARPEDMODEL_PREC_BITS)) * y;

	*gamma = clamp_val((int)AV1_DIV_ROUND_UP_POW2_SIGNED(gv, shift), S16_MIN, S16_MAX);

	dv = ((long long)mat[3] * mat[4]) * y;
	*delta = clamp_val(mat[5] -
		(int)AV1_DIV_ROUND_UP_POW2_SIGNED(dv, shift) - (1 << WARPEDMODEL_PREC_BITS),
		S16_MIN, S16_MAX);

	*alpha = AV1_DIV_ROUND_UP_POW2_SIGNED(*alpha, WARP_PARAM_REDUCE_BITS)
		 * (1 << WARP_PARAM_REDUCE_BITS);
	*beta = AV1_DIV_ROUND_UP_POW2_SIGNED(*beta, WARP_PARAM_REDUCE_BITS)
		* (1 << WARP_PARAM_REDUCE_BITS);
	*gamma = AV1_DIV_ROUND_UP_POW2_SIGNED(*gamma, WARP_PARAM_REDUCE_BITS)
		 * (1 << WARP_PARAM_REDUCE_BITS);
	*delta = AV1_DIV_ROUND_UP_POW2_SIGNED(*delta, WARP_PARAM_REDUCE_BITS)
		* (1 << WARP_PARAM_REDUCE_BITS);
}

static void set_refs(struct avd_ctx *ctx, struct avd_av1_run *run)
{
	struct avd_av1_ctx *av1_ctx = ctx->priv;
	const struct v4l2_ctrl_av1_frame *frame = run->frame;
	const struct v4l2_av1_global_motion *gm = &frame->global_motion;
	struct avd_dev *avd = ctx->dev;
	int i, ref_idx;
	struct avd_decoded_buffer *dst, *ref;
	bool intrabc = !!(frame->flags & V4L2_AV1_FRAME_FLAG_ALLOW_INTRABC);
	dma_addr_t addr;
	s64 alpha = 0, beta = 0, gamma = 0, delta = 0;
	dst = vb2_to_avd_decoded_buf(&run->base.bufs.dst->vb2_buf);

	push(0, "ref_cnst0");
	pusha(av1_ctx->bufs.ref.addr, "unk_ref_buf", 0);


	for (i = 0; i < 4; i++)
		push(0, "ref_cnst1");

	for (i = 0; i < (intrabc ? 1 : V4L2_AV1_REFS_PER_FRAME); i++) {
		ref_idx = frame->ref_frame_idx[i] + 1;

		ref = avd_get_ref_buf(ctx, &dst->base.vb,
				frame->reference_frame_ts[ref_idx]);
		addr = vb2_dma_contig_plane_dma_addr(&ref->base.vb.vb2_buf, 0)
			+ (ref->base.vb.planes[0].length - ref->rvra.size);

		if (!intrabc) {
			int shift = (gm->flags[ref_idx] & V4L2_AV1_GLOBAL_MOTION_FLAG_IS_TRANSLATION)
				? WARPEDMODEL_PREC_BITS - 3 : 10 /* why? */;
			push(AV1_GM_TYPE(gm->type[ref_idx]) |
					AV1_GM_PARAM0(gm->params[ref_idx][0] >> shift) |
					AV1_GM_PARAM1(gm->params[ref_idx][1] >> shift)
					, "ref_gm_mv");
			/* TODO: precison or something, does not always fit */
			push(AV1_GM_VALID(!(V4L2_AV1_GLOBAL_MOTION_IS_INVALID(ref_idx) & gm->invalid)) |
					AV1_GM_PARAM0(AV1_DIV_ROUND_UP_POW2_SIGNED(gm->params[ref_idx][2], 1)) |
					AV1_GM_PARAM1(AV1_DIV_ROUND_UP_POW2_SIGNED(gm->params[ref_idx][3], 1))
					, "ref_gm_param");
			/* TODO: this does not quite fit */
			push(AV1_GM_PARAM1(gm->params[ref_idx][5] / 2) |
					AV1_GM_PARAM0(AV1_DIV_ROUND_UP_POW2_SIGNED(
							gm->params[ref_idx][4] -
							gm->params[ref_idx][3], 2))
					, "ref_gm_unk2");
			avd_av1_dec_get_shear_params(&gm->params[ref_idx][0],
					&alpha, &beta, &gamma, &delta);
			push(AV1_GM_PARAM_SHEAR0(beta) |
					AV1_GM_PARAM_SHEAR1(alpha), "ref_gm_beta_alpha");

			push(AV1_GM_PARAM_SHEAR0(delta) |
					AV1_GM_PARAM_SHEAR1(gamma), "ref_gm_delta_gamma");

		}
		push(AVD_REF_FLAG_CONST | frame->order_hints[ref_idx]
				, "ref_flag");
		push(AVD_HDR_HEIGHT(ref->av1.height) |
				AVD_HDR_WIDTH(ref->av1.width),
				"ref_height_width");
		push(0x40004000, "ref_cnst4");

		push_rvra(avd, ctx, addr, ref->rvra.offsets);
	}
}

static void set_ref_hints(struct avd_ctx *ctx, struct avd_av1_run *run)
{
	const struct v4l2_ctrl_av1_frame *frame = run->frame;
	struct avd_dev *avd = ctx->dev;
	int i, ref_idx;
	struct avd_decoded_buffer *dst, *ref;
	dst = vb2_to_avd_decoded_buf(&run->base.bufs.dst->vb2_buf);
	/* TODO: this is very wrong! */
	for (i = 0; i < V4L2_AV1_TOTAL_REFS_PER_FRAME / 2; i++) {
		ref_idx = frame->ref_frame_idx[i + 3] + 1;
		ref = avd_get_ref_buf(ctx, &dst->base.vb,
				frame->reference_frame_ts[ref_idx]);
#define ORDER_HINT(v)	ref->av1.order_hints[ref->av1.ref_frame_idx[v] + 1]
		/* TODO: high bits set */
		push(
				AV1_ORDER_HINT0(ORDER_HINT(0)) |
				AV1_ORDER_HINT1(ORDER_HINT(1)) |
				AV1_ORDER_HINT2(ORDER_HINT(2)) |
				AV1_ORDER_HINT3(ORDER_HINT(4))
				, "order_hints0");
		push(	AV1_ORDER_HINT0(ORDER_HINT(6)) |
				AV1_ORDER_HINT1(ORDER_HINT(5)) |
				AV1_ORDER_HINT2(ORDER_HINT(3))
				, "order_hints1");
#undef ORDER_HINT
	}
}

static void set_header(struct avd_ctx *ctx, struct avd_av1_run *run)
{
	struct avd_av1_ctx *av1_ctx = ctx->priv;
	const struct v4l2_ctrl_av1_frame *frame = run->frame;
	const struct v4l2_ctrl_av1_sequence *seq = run->seq;
	const struct v4l2_av1_loop_restoration *lr = &frame->loop_restoration;
	const struct v4l2_av1_cdef *cdef = &frame->cdef;
	const struct v4l2_av1_segmentation *seg = &frame->segmentation;
	struct avd_dev *avd = ctx->dev;
	u32 bytesperline;
	int i, segid;
	bool coded_lossless = frame->tx_mode == V4L2_AV1_TX_MODE_ONLY_4X4;
	bool intrabc = !!(frame->flags & V4L2_AV1_FRAME_FLAG_ALLOW_INTRABC);
	bool intra_only = !!((frame->frame_type == V4L2_AV1_KEY_FRAME ||
			frame->frame_type == V4L2_AV1_INTRA_ONLY_FRAME));
	push(AVD_OP_EXEC |
			AVD_OP_EXEC_FLAG_START_REV4(avd->variant->revision == 4) |
			AVD_OP_EXEC_FIFO_IDX(ctx->fifo_idx),
			"vp_start");
	push(AVD_OP_HDR |
			AVD_OP_HDR_FLAG0 |
			AVD_OP_HDR_FLAG_INTRA(intra_only && !intrabc) |
			AVD_OP_HDR_CONST |
			AVD_OP_HDR_FLAG_PIPE_STATE(1)
			, "op_hdr");

	push(AVD_HDR_CODEC_MODE(AVD_CODEC_AV1) |
			AV1_CODEC_MODE_INTRABC(intrabc), "codec");

	/*
	 * TODO: height_width_* are kinda weird
	 * on most samples this is true
	 */
	push(AVD_HDR_HEIGHT(frame->frame_height_minus_1) |
			AVD_HDR_WIDTH(frame->frame_width_minus_1),
			"height_width_1");
	push(0, "");
	push(AVD_HDR_HEIGHT(frame->frame_height_minus_1) |
			AVD_HDR_WIDTH(frame->frame_width_minus_1),
			"height_width_2");

	push(AVD_HDR_COMMON_CHROMA_FORMAT(1) |
			AVD_HDR_COMMON_BIT_DEPTH_C(seq->bit_depth - 8) |
			AVD_HDR_COMMON_BIT_DEPTH_L(seq->bit_depth - 8) |
			AV1_HDR_JNT_COMP(seq->flags & V4L2_AV1_SEQUENCE_FLAG_ENABLE_JNT_COMP) |
			AV1_HDR_DUAL_FILTER(seq->flags & V4L2_AV1_SEQUENCE_FLAG_ENABLE_DUAL_FILTER) |
			AV1_HDR_INTRA_EDGE_FILTER(seq->flags & V4L2_AV1_SEQUENCE_FLAG_ENABLE_MASKED_COMPOUND) |
			AV1_HDR_INTRA_EDGE_FILTER(seq->flags & V4L2_AV1_SEQUENCE_FLAG_ENABLE_INTRA_EDGE_FILTER) |
			AV1_HDR_128X128_SUPERBLOCK(seq->flags & V4L2_AV1_SEQUENCE_FLAG_USE_128X128_SUPERBLOCK) |
			AV1_HDR_SCREEN_CONTENT_TOOLS(frame->flags & V4L2_AV1_FRAME_FLAG_ALLOW_SCREEN_CONTENT_TOOLS) |

			/* TODO: this seems like a coincidence */
			!!!(seq->flags & V4L2_AV1_SEQUENCE_FLAG_ENABLE_ORDER_HINT) << 1 |
			!!(seq->flags & V4L2_AV1_SEQUENCE_FLAG_ENABLE_WARPED_MOTION) << 2 |
			!!(seq->flags & V4L2_AV1_SEQUENCE_FLAG_ENABLE_MASKED_COMPOUND) << 3 |

			/* i dont now which of these flags are the right ones */
			!!(seq->flags & (V4L2_AV1_SEQUENCE_FLAG_ENABLE_REF_FRAME_MVS | V4L2_AV1_SEQUENCE_FLAG_ENABLE_ORDER_HINT)) << 4 |
			!!(seq->flags & (V4L2_AV1_SEQUENCE_FLAG_ENABLE_INTERINTRA_COMPOUND | V4L2_AV1_SEQUENCE_FLAG_ENABLE_FILTER_INTRA)) << 7 |
			!!(seq->flags & (V4L2_AV1_SEQUENCE_FLAG_ENABLE_INTERINTRA_COMPOUND | V4L2_AV1_SEQUENCE_FLAG_ENABLE_FILTER_INTRA)) << 9
			, "hdr_common");


	push(AV1_FLAGS_TX_MODE_LARGEST(frame->tx_mode == V4L2_AV1_TX_MODE_LARGEST) |
			AV1_FLAGS_TX_MODE_SELECT(frame->tx_mode == V4L2_AV1_TX_MODE_SELECT)  |
			AV1_FLAGS_TX_REDUCED_SET(frame->flags & V4L2_AV1_FRAME_FLAG_REDUCED_TX_SET) |
			AV1_FLAGS_ALLOW_WARPED_MOTION(frame->flags & V4L2_AV1_FRAME_FLAG_ALLOW_WARPED_MOTION) |
			AV1_FLAGS_SKIP_MODE(frame->flags & (V4L2_AV1_FRAME_FLAG_SKIP_MODE_ALLOWED | V4L2_AV1_FRAME_FLAG_SKIP_MODE_PRESENT)) |
			AV1_FLAGS_INTRABC(frame->flags & V4L2_AV1_FRAME_FLAG_ALLOW_INTRABC) |
			AV1_FLAGS_INTEGER_MV(frame->flags & V4L2_AV1_FRAME_FLAG_FORCE_INTEGER_MV) |
			AV1_FLAGS_DISABLE_CDF_UPDATE(frame->flags & V4L2_AV1_FRAME_FLAG_DISABLE_CDF_UPDATE) |
			AV1_FLAGS_TX_MODE_ONLY_4X4(frame->tx_mode == V4L2_AV1_TX_MODE_ONLY_4X4) |
			AV1_FLAGS_HIGH_PRECISION_MV(frame->flags & V4L2_AV1_FRAME_FLAG_ALLOW_HIGH_PRECISION_MV) |
			AV1_FLAGS_SEG_TEMPORAL_UPDATE(frame->segmentation.flags & V4L2_AV1_SEGMENTATION_FLAG_TEMPORAL_UPDATE) |
			AV1_FLAGS_SEG_UPDATE_MAP(frame->segmentation.flags & V4L2_AV1_SEGMENTATION_FLAG_UPDATE_MAP) |
			AV1_FLAGS_SEG_ENABLED(frame->segmentation.flags & V4L2_AV1_SEGMENTATION_FLAG_ENABLED) |
			AV1_FLAGS_SEG_UPDATE_DATA(frame->segmentation.flags & V4L2_AV1_SEGMENTATION_FLAG_UPDATE_DATA) |
			AV1_FLAGS_ORDER_HINT(frame->order_hint) |
			AV1_FLAGS_INTRA(intra_only) |

			!!(frame->flags & (V4L2_AV1_FRAME_FLAG_USE_REF_FRAME_MVS | V4L2_AV1_FRAME_FLAG_IS_MOTION_MODE_SWITCHABLE)) << 12 |
			/*
			 * bit 20-22 are always set together
			 * bit 17-19 are also sat most of the time
			 * they dont seem to make a difference
			 */
			/* TODO not true */
			!(frame->frame_type == V4L2_AV1_KEY_FRAME) << 5
			, "av1_flags0");
	push(
			/*
			 * something skip mode / refs related?
			 * bits 0-20 are set sometimes
			 * with bit 7, 10, 13, 16 and 19 being the most common
			 */
			BIT(7) |
			BIT(25) |
			(true ? BIT(22) : BIT(23)) |

			AV1_FLAGS_REFERENCE_SELECT(frame->flags & V4L2_AV1_FRAME_FLAG_REFERENCE_SELECT) |
			AV1_FLAGS_INTERPOLATION_FILTER(frame->interpolation_filter)
			, "av1_flags1");

	int segval;
	for (segid = 0; segid < V4L2_AV1_MAX_SEGMENTS; segid++) {
#define SEG_FEAT_EN(feat)	(seg->feature_enabled[segid] & \
		V4L2_AV1_SEGMENT_FEATURE_ENABLED(feat))
		/* what? why? */
		segval = AV1_SEG_UNK0(frame->tx_mode == V4L2_AV1_TX_MODE_ONLY_4X4);

		segval |= AV1_SEG_ALT_Q(seg->feature_data[segid][V4L2_AV1_SEG_LVL_ALT_Q]);
		segval |= AV1_SEG_ALT_Q_EN(SEG_FEAT_EN(V4L2_AV1_SEG_LVL_ALT_Q));
		segval |= AV1_SEG_REF_FRAME(seg->feature_data[segid][V4L2_AV1_SEG_LVL_REF_FRAME]);
		segval |= AV1_SEG_REF_FRAME_EN(SEG_FEAT_EN(V4L2_AV1_SEG_LVL_REF_FRAME));
		segval |= AV1_SEG_REF_SKIP_EN(SEG_FEAT_EN(V4L2_AV1_SEG_LVL_REF_SKIP));
		segval |= AV1_SEG_REF_GMV_EN(SEG_FEAT_EN(V4L2_AV1_SEG_LVL_REF_GLOBALMV));
		push(segval, "seg_0");
		segval = 0;
		segval |= AV1_SEG_LF_V(seg->feature_data[segid][V4L2_AV1_SEG_LVL_ALT_LF_V]);
		segval |= AV1_SEG_LF_V_EN(SEG_FEAT_EN(V4L2_AV1_SEG_LVL_ALT_LF_V));
		segval |= AV1_SEG_LF_U(seg->feature_data[segid][V4L2_AV1_SEG_LVL_ALT_LF_U]);
		segval |= AV1_SEG_LF_U_EN(SEG_FEAT_EN(V4L2_AV1_SEG_LVL_ALT_LF_U));
		segval |= AV1_SEG_LF_Y_H(seg->feature_data[segid][V4L2_AV1_SEG_LVL_ALT_LF_Y_H]);
		segval |= AV1_SEG_LF_Y_H_EN(SEG_FEAT_EN(V4L2_AV1_SEG_LVL_ALT_LF_Y_H));
		segval |= AV1_SEG_LF_Y_V(seg->feature_data[segid][V4L2_AV1_SEG_LVL_ALT_LF_Y_V]);
		segval |= AV1_SEG_LF_Y_V_EN(SEG_FEAT_EN(V4L2_AV1_SEG_LVL_ALT_LF_Y_V));
		push(segval, "seg_1");
#undef SEG_FEAT_EN
	}

	push(AVD_HDR_FEAT_VP9, "feat_en");

	set_ref_hints(ctx, run);
	pusha(av1_ctx->bufs.probs[0].addr, "probs", 0);
	pusha(av1_ctx->bufs.probs[1].addr, "probs", 1);

	pusha(av1_ctx->bufs.const_unk[0].addr, "const_buf", 0);

	/* not always set */
	pusha(av1_ctx->bufs.unk[0].addr, "buf", 0);
	pusha(av1_ctx->bufs.unk[1].addr, "buf", 1);

	pusha(av1_ctx->bufs.const_unk[1].addr, "const_buf", 1);

	/* grain data? */
	/* bits 0-2 8-9 NOT 10 in flag1 are often set when these buffers are present */
	pusha(av1_ctx->bufs.unk[2].addr, "buf", 2);
	pusha(av1_ctx->bufs.unk[3].addr, "buf", 3);
	pusha(av1_ctx->bufs.unk[4].addr, "buf", 4);

	push(AV1_QP_PRESENT(frame->quantization.flags & V4L2_AV1_QUANTIZATION_FLAG_DELTA_Q_PRESENT) |
			AV1_QP_BASE_IDX(frame->quantization.base_q_idx) |
			AV1_QP_DELTA_RES(frame->quantization.delta_q_res) |
			AV1_QP_U_DC(frame->quantization.delta_q_u_dc) |
			AV1_QP_Y_DC(frame->quantization.delta_q_y_dc),
			"qp_base_q_idx");

	/* TODO: test294 test35 */
	push(0, "unk");
	/* TODO: bit 29-31 */
	push(AV1_LF_DELTA_ENABLED(frame->loop_filter.flags & V4L2_AV1_LOOP_FILTER_FLAG_DELTA_ENABLED) |
			AV1_LF_LV0(frame->loop_filter.level[0]) |
			AV1_LF_LV1(frame->loop_filter.level[1]) |
			AV1_LF_LV2(frame->loop_filter.level[2]) |
			AV1_LF_LV3(frame->loop_filter.level[3])
			, "lf_update");
	push(AV1_LF_SHARPNESS(frame->loop_filter.sharpness) |
			 AV1_LF_REF0(frame->loop_filter.ref_deltas[0]) |
			 AV1_LF_REF1(frame->loop_filter.ref_deltas[1]) |
			 AV1_LF_REF2(frame->loop_filter.ref_deltas[2]) |
			 AV1_LF_REF3(frame->loop_filter.ref_deltas[3])
			, "lf_sharp_ref_deltas");

	push(AV1_LF_REF0(frame->loop_filter.ref_deltas[4]) |
			 AV1_LF_REF1(frame->loop_filter.ref_deltas[5]) |
			 AV1_LF_REF2(frame->loop_filter.ref_deltas[6]) |
			 AV1_LF_REF3(frame->loop_filter.ref_deltas[7])
			, "lf_ref_deltas");
	/* TODO */
	push(0, "lf_unk");


#define AV1_CDEF_PACK_ONE(i) \
	(AV1_CDEF_Y_PRI(cdef->y_pri_strength[i]) | \
	 AV1_CDEF_Y_SEC(cdef->y_sec_strength[i]) | \
	 AV1_CDEF_UV_PRI(cdef->uv_pri_strength[i]) | \
	 AV1_CDEF_UV_SEC(cdef->uv_sec_strength[i]))

#define AV1_CDEF_PACK(i) \
	AV1_CDEF_HI(AV1_CDEF_PACK_ONE(((i) * 2))) | \
	AV1_CDEF_LO(AV1_CDEF_PACK_ONE(((i) * 2) + 1))
	bool cdef_enabled = !(!(seq->flags & V4L2_AV1_SEQUENCE_FLAG_ENABLE_CDEF)
		|| frame->flags & V4L2_AV1_FRAME_FLAG_ALLOW_INTRABC
		|| coded_lossless);
	push(AV1_CDEF_EN(cdef_enabled) |
			AV1_CDEF_BITS(frame->cdef.bits) |
			AV1_CDEF_DAMPING(frame->cdef.damping_minus_3) |
			AV1_CDEF_PACK(0),
			"cdef0");
	push(AV1_CDEF_PACK(1), "cdef1");
	push(AV1_CDEF_PACK(2), "cdef2");
	push(AV1_CDEF_PACK(3), "cdef3");
#undef AV1_CDEF_PACK_ONE
#undef AV1_CDEF_PACK

	/* TODO: this is wrong, maybe has something to do with superres */
	push((frame->flags & V4L2_AV1_FRAME_FLAG_USE_SUPERRES ?
			(frame->superres_denom - 1) << 28 : 0) |
			frame->upscaled_width - 1, "upscaled_width");
	/* something superres related? test35 */
	push(0x200000, "flag_unk0");
	push(0x200000, "flag_unk1");

	u8 restoration_unit_size[V4L2_AV1_NUM_PLANES_MAX] = { 3, 3, 3 };

	if (lr->flags & V4L2_AV1_LOOP_RESTORATION_FLAG_USES_LR) {
		restoration_unit_size[0] = 1 + lr->lr_unit_shift;
		restoration_unit_size[1] =
			1 + lr->lr_unit_shift - lr->lr_uv_shift;
		restoration_unit_size[2] =
			1 + lr->lr_unit_shift - lr->lr_uv_shift;
	}

	push(AV1_LR_TYPE0(frame->loop_restoration.frame_restoration_type[0]) |
			AV1_LR_TYPE1(frame->loop_restoration.frame_restoration_type[1]) |
			AV1_LR_TYPE2(frame->loop_restoration.frame_restoration_type[2]) |
			AV1_LR_UNIT0(restoration_unit_size[0]) |
			AV1_LR_UNIT1(restoration_unit_size[1]) |
			AV1_LR_UNIT2(restoration_unit_size[2]), "lr");

	push(0, "cnst3");
	push(0, "cnst3");
	pusha(av1_ctx->bufs.pipe_state.addr, "pipe_state", 0);
	for (i = 0; i < 10; i++)
		pusha(av1_ctx->bufs.const_unk[i + 2].addr, "const_unk", i + 2);

	push(0, "mark_section");

	push_rvra(avd, ctx, run->addresses.rvra, ctx->rvra.offsets);

	push(0, "");
	push(0, "mark_section");

	bytesperline = ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
	pusha(run->addresses.y, "y", 0);
	push(bytesperline, "bytesperline_y");
	pusha(run->addresses.uv, "uv", 0);
	push(bytesperline, "bytesperline_uv");

	push(0, "mark_section");

	push(AVD_HDR_HEIGHT(frame->frame_height_minus_1) |
			AVD_HDR_WIDTH(frame->frame_width_minus_1),
			"height_width_3");
	if (!intra_only || intrabc)
		set_refs(ctx, run);

}

static void set_tiles(struct avd_ctx *ctx, struct avd_av1_run *run)
{
	struct avd_av1_ctx *av1_ctx = ctx->priv;
	const struct v4l2_ctrl_av1_frame *frame = run->frame;
	const struct v4l2_ctrl_av1_tile_group_entry *tile_group;
	const struct v4l2_av1_tile_info *tile_info = &frame->tile_info;
	bool is_last;
	struct avd_dev *avd = ctx->dev;
	int col, row;

	/* TODO? */
	av1_ctx->submit_num = tile_info->tile_cols * tile_info->tile_rows;

	for (col = 0; col < tile_info->tile_cols; col++) {
		for (row = 0; row < tile_info->tile_rows; row++) {
			is_last = col == tile_info->tile_cols - 1 && row == tile_info->tile_rows - 1;
			int tile_id = row * tile_info->tile_cols + col;
			tile_group = &run->tile_group[tile_id];
			push(AVD_OP_CODED_DATA |
					AVD_OP_CODED_DATA_ADDR((run->addresses.sl + tile_group->tile_offset) >> 32),
					"set_coded_data");
			push((u32)((run->addresses.sl + tile_group->tile_offset) & 0xffffffff),
					"coded_data_addr");
			push(tile_group->tile_size, "tile_size");

			/* TODO */
			int row_start = tile_info->mi_row_starts[tile_group->tile_row];
			int col_start = tile_info->mi_col_starts[tile_group->tile_col];
			push(AVD_OP_SL_DIM_START |
					AVD_OP_SL_DIM_START_Y(row_start) |
					AVD_OP_SL_DIM_START_X(col_start), "sl_dim_start");

			push(0 | /* AVD_SL_DIM_END_ROW(TODO) |  */
					AVD_SL_DIM_END_Y(row_start + tile_info->height_in_sbs_minus_1[tile_group->tile_row]) |
					AVD_SL_DIM_END_X(col_start + tile_info->width_in_sbs_minus_1[tile_group->tile_col])
					, "sl_dim_end");
			push(AVD_OP_EXEC | AVD_OP_EXEC_FLAG_END(is_last), "end");
#ifdef DEBUG_INST
			pr_info("\n");
#endif
		}
	}
}

static void update_dec_buf_info(struct avd_decoded_buffer *buf,
				const struct v4l2_ctrl_av1_sequence *seq,
				const struct v4l2_ctrl_av1_frame *frame)
{
	int i;
	buf->av1.width = seq->max_frame_width_minus_1;
	buf->av1.height = seq->max_frame_height_minus_1;
	buf->av1.bit_depth = seq->bit_depth;

	for (i = 0; i < V4L2_AV1_TOTAL_REFS_PER_FRAME; i++)
		buf->av1.order_hints[i] = frame->order_hints[i];

	for (i = 0; i < V4L2_AV1_REFS_PER_FRAME; i++)
		buf->av1.ref_frame_idx[i] = frame->ref_frame_idx[i];
}

static int avd_av1_run_preamble(struct avd_ctx *ctx,
				     struct avd_av1_run *run)
{
	struct v4l2_ctrl *ctrl;
	u32 dst_len;

	avd_run_preamble(ctx, &run->base);

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_AV1_SEQUENCE);
	if (WARN_ON(!ctrl))
		return -EINVAL;
	run->seq = ctrl->p_cur.p;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_AV1_FRAME);
	if (WARN_ON(!ctrl))
		return -EINVAL;
	run->frame = ctrl->p_cur.p;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_AV1_TILE_GROUP_ENTRY);
	if (WARN_ON(!ctrl))
		return -EINVAL;
	run->tile_group = ctrl->p_cur.p;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			V4L2_CID_STATELESS_AV1_FILM_GRAIN);
	if (WARN_ON(!ctrl))
		return -EINVAL;
	run->grain = ctrl->p_cur.p;

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

static int avd_av1_run(struct avd_ctx *ctx)
{
	struct avd_dev *avd = ctx->dev;
	struct avd_av1_run run;
	struct avd_av1_ctx *av1_ctx;
	struct avd_decoded_buffer *dst;
	int ret;

	ret = avd_av1_run_preamble(ctx, &run);
	if (ret) {
		avd_run_postamble(ctx, &run.base);
		return ret;
	}

	av1_ctx = ctx->priv;
	dst = vb2_to_avd_decoded_buf(&run.base.bufs.dst->vb2_buf);
	update_dec_buf_info(dst, run.seq, run.frame);

	ret = alloc_slots(avd, ctx, AVD_CODEC_AV1);
	if (ret) {
		dev_err(avd->dev, "no free slots: %d", ret);
		return ret;
	}

	schedule_delayed_work(&ctx->watchdog_work, msecs_to_jiffies(2000));

	avd->variant->configure_stream(avd, av1_ctx->bufs.inst.addr,
			ctx->fifo_idx, ctx->vp_slot);

	avd_av1_default_coeff_probs(run.frame->quantization.base_q_idx, av1_ctx->bufs.probs[1].cpu);
	avd_av1_set_default_cdfs(av1_ctx->bufs.probs[1].cpu);

	set_header(ctx, &run);
	set_tiles(ctx, &run);

	avd_run_postamble(ctx, &run.base);

	return 0;
}

static int avd_av1_alloc_bufs(struct avd_ctx *ctx)
{
	struct avd_dev *avd = ctx->dev;
	struct avd_av1_ctx *av1_ctx = ctx->priv;
	int ret, i;

	ret = avd_buf_alloc(avd, &av1_ctx->bufs.inst, fifo_size());
	if (ret)
		return ret;

	/* no m1 to worry about :) */
	ret = avd_buf_alloc(avd, &av1_ctx->bufs.pipe_state, 0x200);
	if (ret)
		return ret;

	ret = avd_buf_alloc(avd, &av1_ctx->bufs.ref, 0x10000);
	if (ret)
		return ret;

	for (i = 0; i < 2; i++) {
		ret = avd_buf_alloc(avd, &av1_ctx->bufs.probs[i], sizeof(struct avd_av1_cdfs));
		if (ret)
			return ret;
	}

	for (i = 0; i < 5; i++) {
		ret = avd_buf_alloc(avd, &av1_ctx->bufs.unk[i], 0x4000);
		if (ret)
			return ret;
	}

	for (i = 0; i < 12; i++) {
		ret = avd_buf_alloc(avd, &av1_ctx->bufs.const_unk[i], 0x10000 * 4);
		if (ret)
			return ret;
	}

	return 0;
}

static int avd_av1_start(struct avd_ctx *ctx)
{
	struct avd_av1_ctx *av1_ctx;
	int ret;

	av1_ctx = kzalloc(sizeof(*av1_ctx), GFP_KERNEL);
	if (!av1_ctx)
		return -ENOMEM;

	ctx->priv = av1_ctx;
	ret = avd_av1_alloc_bufs(ctx);
	if (ret)
		goto err_free_ctx;

	return 0;

err_free_ctx:
	kfree(av1_ctx);
	ctx->priv = NULL;
	return ret;
}

static void avd_av1_stop(struct avd_ctx *ctx)
{
	struct avd_av1_ctx *av1_ctx = ctx->priv;
	struct avd_dev *avd = ctx->dev;

	avd_buf_free(avd, &av1_ctx->bufs.pipe_state);
	avd_buf_free(avd, &av1_ctx->bufs.inst);
	avd_buf_free(avd, &av1_ctx->bufs.ref);
	avd_buf_free(avd, &av1_ctx->bufs.probs[0]);
	avd_buf_free(avd, &av1_ctx->bufs.probs[1]);

	for (int i = 0; i < 5; i++)
		avd_buf_free(avd, &av1_ctx->bufs.unk[i]);
	for (int i = 0; i < 12; i++)
		avd_buf_free(avd, &av1_ctx->bufs.const_unk[i]);

	kfree(av1_ctx);
}

static enum avd_image_fmt avd_av1_get_image_fmt(struct avd_ctx *ctx,
						struct v4l2_ctrl *ctrl)
{
#define BIT_DEPTH(chroma) \
	(seq->bit_depth == 8 ? AVD_IMG_FMT_##chroma##_8BIT :  \
	 AVD_IMG_FMT_##chroma##_10BIT )

	const struct v4l2_ctrl_av1_sequence *seq = ctrl->p_new.p_av1_sequence;

	if (ctrl->id != V4L2_CID_STATELESS_AV1_SEQUENCE)
		return AVD_IMG_FMT_ANY;

	if (!(seq->flags & V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_X)) /* 4:4:4 */
		return AVD_IMG_FMT_ANY;
	if (!(seq->flags & V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_Y))
		return BIT_DEPTH(422);
	if (seq->flags & V4L2_AV1_SEQUENCE_FLAG_MONO_CHROME)
		return AVD_IMG_FMT_ANY; /* TODO 4:0:0 (did not test) */

	return BIT_DEPTH(420);
#undef BIT_DEPTH
}

static void avd_av1_submit(struct avd_ctx *ctx)
{
	struct avd_av1_ctx *av1_ctx = ctx->priv;
	struct avd_dev *avd = ctx->dev;

	writel(AVD_OP_EXEC |
			AVD_OP_EXEC_FLAG_START_REV4(avd->variant->revision == 4) |
			AVD_OP_EXEC_FIFO_IDX(ctx->fifo_idx) |
			AVD_OP_EXEC_FIFO_MASK(avd->variant->fifo_slots),
			avd->ctrl + avd->variant->submit_offset);
#ifdef DEBUG_INST
		pr_info("%8lx | %s %2d\n",
			AVD_OP_EXEC |
			AVD_OP_EXEC_FLAG_START_REV4(avd->variant->revision == 4) |
			AVD_OP_EXEC_FIFO_IDX(ctx->fifo_idx) |
			AVD_OP_EXEC_FIFO_MASK(avd->variant->fifo_slots),
				"submit", 0);
#endif
	for (int i = 0; i < av1_ctx->submit_num - 1; i++) {
#ifdef DEBUG_INST
		pr_info("%8lx | %s %2d\n",
				AVD_OP_EXEC |
				AVD_OP_EXEC_FIFO_IDX(ctx->fifo_idx) |
				AVD_OP_EXEC_FIFO_MASK(avd->variant->fifo_slots),
				"submit", i + 1);
#endif
		writel(AVD_OP_EXEC |
			AVD_OP_EXEC_FIFO_IDX(ctx->fifo_idx) |
			AVD_OP_EXEC_FIFO_MASK(avd->variant->fifo_slots),
				avd->ctrl + avd->variant->submit_offset);
	}
#ifdef DEBUG_INST
	pr_info("\n");
#endif
}

const struct avd_coded_fmt_ops avd_av1_fmt_ops = {
	.start = avd_av1_start,
	.stop = avd_av1_stop,
	.run = avd_av1_run,
	.submit = avd_av1_submit,
	.get_image_fmt = avd_av1_get_image_fmt,
};
