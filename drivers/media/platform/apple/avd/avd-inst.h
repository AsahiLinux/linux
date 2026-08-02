/* SPDX-License-Identifier: MIT */

#ifndef AVD_INST_H_
#define AVD_INST_H_
/* instruction stream things  */
#include <linux/types.h>

#include "avd.h"

/* i have no clue what this is */
#define INST_DMA1 0 /* (0x14 << 16 | 0x14) */
#define INST_DMA2 0 /* (0x4000000 | INST_DMA1) */
#define INST_DMA3 0 /* (0x07 << 16 | 0x07) */

#define AVD_OP_EXEC			FIELD_PREP(GENMASK(31, 24), 0x2b)
#define AVD_OP_EXEC_FIFO_MASK(v)	FIELD_PREP(GENMASK(3, 0), v)
#define AVD_OP_EXEC_FIFO_IDX(v)		FIELD_PREP(GENMASK(7, 4), v)
#define AVD_OP_EXEC_FLAG_START_REV3(v)	FIELD_PREP(BIT(8), !!(v))
#define AVD_OP_EXEC_FLAG_START_REV4(v)	FIELD_PREP(BIT(9), !!(v))
#define AVD_OP_EXEC_FLAG_END(v)		FIELD_PREP(BIT(10), !!(v))
#define AVD_OP_EXEC_REV3_VP9_MASK	FIELD_PREP(GENMASK(23, 12), 0xfff)

#define AVD_OP_HDR			FIELD_PREP(GENMASK(31, 20), 0x2db)
#define AVD_OP_HDR_CONST		FIELD_PREP(GENMASK(10, 0), 0x2e0)
#define AVD_OP_HDR_FLAG0		FIELD_PREP(BIT(12), 1)
#define AVD_OP_HDR_FLAG_INTRA(v)	FIELD_PREP(BIT(13), !!(v))
#define AVD_OP_HDR_FLAG_PIPE_STATE(v)	FIELD_PREP(BIT(19), !!(v))

#define AVD_OP_WEIGHTS_HDR		FIELD_PREP(GENMASK(31, 20), 0x2dd)
#define AVD_OP_WEIGHTS_HDR_CHROMA(v)	FIELD_PREP(GENMASK(2, 0), v)
#define AVD_OP_WEIGHTS_HDR_LUMA(v)	FIELD_PREP(GENMASK(5, 3), v)
#define AVD_OP_WEIGHTS_HDR_FLAG0(v)	FIELD_PREP(BIT(6), !!(v))
#define AVD_OP_WEIGHTS_HDR_FLAG1(v)	FIELD_PREP(BIT(7), !!(v))

#define AVD_OP_WEIGHTS			FIELD_PREP(GENMASK(31, 20), 0x2de)
#define AVD_OP_WEIGHTS_WEIGHT(v)	FIELD_PREP(GENMASK(8, 0), v)
#define AVD_OP_WEIGHTS_INDEX(v)		FIELD_PREP(GENMASK(12, 9), v)
#define AVD_OP_WEIGHTS_LIST_IDX(v)	FIELD_PREP(BIT(13), v)
/* 1 = luma, 2,3 = chroma[{0,1}] */
#define AVD_OP_WEIGHTS_IDENT(v)		FIELD_PREP(GENMASK(16, 14), v)

#define AVD_OP_OFFSETS			FIELD_PREP(GENMASK(31, 20), 0x2df)
#define AVD_OP_OFFSETS_OFFSET(v)	FIELD_PREP(GENMASK(15, 0), v)

#define AVD_OP_CODED_DATA		FIELD_PREP(GENMASK(31, 20), 0x2d8)
#define AVD_OP_CODED_DATA_ADDR(v)	FIELD_PREP(GENMASK(12, 0), v)
#define AVD_OP_CODED_DATA_FLAG0(v)	FIELD_PREP(BIT(13), !!(v))
#define AVD_OP_CODED_DATA_FLAG1(v)	FIELD_PREP(BIT(14), !!(v))
#define AVD_OP_CODED_DATA_BIT_OFF(v)	FIELD_PREP(GENMASK(18, 15), v)

/* not super happy with xy */
#define AVD_OP_SL_LOC			FIELD_PREP(GENMASK(31, 24), 0x2c)
#define AVD_OP_SL_LOC_X(v)		FIELD_PREP(GENMASK(11, 0), v)
#define AVD_OP_SL_LOC_Y(v)		FIELD_PREP(GENMASK(23, 12), v)

#define AVD_OP_SL_DIM_START		FIELD_PREP(GENMASK(31, 24), 0x2a)
#define AVD_OP_SL_DIM_START_X(v)	FIELD_PREP(GENMASK(11, 0), v)
#define AVD_OP_SL_DIM_START_Y(v)	FIELD_PREP(GENMASK(23, 12), v)

/* end is not an op */
#define AVD_SL_DIM_END_X(v)		FIELD_PREP(GENMASK(11, 0), v)
#define AVD_SL_DIM_END_Y(v)		FIELD_PREP(GENMASK(23, 12), v)
#define AVD_SL_DIM_END_COL(v)		FIELD_PREP(GENMASK(27, 24), v)
#define AVD_SL_DIM_END_ROW(v)		FIELD_PREP(GENMASK(31, 28), v)

#define AVD_OP_SL_REF			FIELD_PREP(GENMASK(31, 24), 0x2d)
#define AVD_OP_SL_REF_MAX_MERGE(v)	FIELD_PREP(GENMASK(3, 1), v)

#define AVD_OP_SL_REF_FLAG0(v)		FIELD_PREP(BIT(4), !!(v))
#define AVD_OP_SL_REF_FLAG_CABAC(v)	FIELD_PREP(BIT(5), !!(v))
#define AVD_OP_SL_REF_FLAG1(v)		FIELD_PREP(BIT(6), !!(v))
#define AVD_OP_SL_REF_NUM_L0(v)		FIELD_PREP(GENMASK(15, 11), v)
#define AVD_OP_SL_REF_NUM_L1(v)		FIELD_PREP(GENMASK(10, 7), v)
#define AVD_OP_SL_REF_FLAG2(v)		FIELD_PREP(BIT(15), !!(v))
#define AVD_OP_SL_REF_SLICE_P(v)	FIELD_PREP(BIT(16), !!(v))
#define AVD_OP_SL_REF_SLICE_I(v)	FIELD_PREP(BIT(17), !!(v))
/* not really kinda more like has_ref_and_ref_is_valid_ref */
#define AVD_OP_SL_REF_SLICE_B(v)	FIELD_PREP(BIT(18), !!(v))

#define AVD_OP_QP			FIELD_PREP(GENMASK(31, 20), 0x2d9)
#define AVD_OP_QP_CR_OFF(v)		FIELD_PREP(GENMASK(4, 0), v)
#define AVD_OP_QP_CB_OFF(v)		FIELD_PREP(GENMASK(9, 5), v)
#define AVD_OP_QP_VAL(v)		FIELD_PREP(GENMASK(17, 10), v)

#define AVD_OP_DBLK			FIELD_PREP(GENMASK(31, 20), 0x2da)
#define AVD_OP_DBLK_FLAG_SAO_CHROMA(v)	FIELD_PREP(BIT(6), !!(v))
#define AVD_OP_DBLK_FLAG_SAO_LUMA(v)	FIELD_PREP(BIT(7), !!(v))
#define AVD_OP_DBLK_OFF0(v)		FIELD_PREP(GENMASK(11, 8), v)
#define AVD_OP_DBLK_OFF1(v)		FIELD_PREP(GENMASK(16, 12), v)
#define AVD_OP_DBLK_FLAG_EN(v)		FIELD_PREP(BIT(16), !!(v))
#define AVD_OP_DBLK_FLAG_FULL_EN(v)	FIELD_PREP(BIT(17), !!(v))
#define AVD_OP_DBLK_FLAG_TILES_EN(v)	FIELD_PREP(BIT(18), !!(v))
#define AVD_OP_DBLK_FLAG_PCM_EN(v)	FIELD_PREP(BIT(19), !!(v))

#define AVD_OP_REF			FIELD_PREP(GENMASK(31, 20), 0x2dc)
/* same order as they where submitted */
#define AVD_OP_REF_DBP_IDX(v)		FIELD_PREP(GENMASK(3, 0), v)
#define AVD_OP_REF_LOOP_IDX(v)		FIELD_PREP(GENMASK(7, 4), v)
#define AVD_OP_REF_LIST_IDX(v)		FIELD_PREP(GENMASK(11, 8), v)

/*
 * now comes all common / shared
 */
#define AVD_HDR_CODEC_MODE(v)		FIELD_PREP(GENMASK(28, 24), v)
#define AVD_HDR_WIDTH(v)		FIELD_PREP(GENMASK(15, 0), v)
#define AVD_HDR_HEIGHT(v)		FIELD_PREP(GENMASK(31, 16), v)

#define AVD_HDR_FEAT_H264		FIELD_PREP(GENMASK(3, 0), 10)
#define AVD_HDR_FEAT_PIPE_STATE_EN(v)	FIELD_PREP(GENMASK(7, 4), (v) ? 3 : 0)
#define AVD_HDR_FEAT_H26X		FIELD_PREP(BIT(20), 1)
#define AVD_HDR_FEAT_COMMON		FIELD_PREP(BIT(21), 1)
#define AVD_HDR_FEAT_VP9		FIELD_PREP(BIT(17), 1)


#define AVD_HDR_COMMON_FLAG0(v)		FIELD_PREP(BIT(0), !!(v))
/*
 * this is hevc specifik
 * TODO
 * some of this is likeley wrong, it wont work without the overlap...
 */
#define AVD_HDR_COMMON_TR_INTRA(v)	FIELD_PREP(GENMASK(4, 1), v)
#define AVD_HDR_COMMON_TR_INTER(v)	FIELD_PREP(GENMASK(7, 4), v)
/* luma {transform,coding} block size */
#define AVD_HDR_COMMON_LUMA_TBS(v)	FIELD_PREP(GENMASK(8, 7), v)
#define AVD_HDR_COMMON_MIN_LUMA_TBS(v)	FIELD_PREP(GENMASK(10, 9), v)
#define AVD_HDR_COMMON_LUMA_CBS(v)	FIELD_PREP(GENMASK(12, 11), v)
#define AVD_HDR_COMMON_MIN_LUMA_CBS(v)	FIELD_PREP(GENMASK(14, 13), v)
#define AVD_HDR_COMMON_BIT_DEPTH_L(v)	FIELD_PREP(GENMASK(18, 15), v)
#define AVD_HDR_COMMON_BIT_DEPTH_C(v)	FIELD_PREP(GENMASK(23, 19), v)
/* TODO: did not bother checking max chroma idc */
#define AVD_HDR_COMMON_CHROMA_FORMAT(v)	FIELD_PREP(GENMASK(26, 24), v)

#define AVD_HDR_H26X_QP_OFFSET_CR(v)	FIELD_PREP(GENMASK(4, 0), v)
#define AVD_HDR_H26X_QP_OFFSET_CB(v)	FIELD_PREP(GENMASK(9, 5), v)

#define AVD_SCALING_I0(v)		FIELD_PREP(GENMASK(7, 0), v)
#define AVD_SCALING_I1(v)		FIELD_PREP(GENMASK(15, 8), v)
#define AVD_SCALING_I2(v)		FIELD_PREP(GENMASK(23, 16), v)
#define AVD_SCALING_I3(v)		FIELD_PREP(GENMASK(31, 24), v)

#define AVD_REF_NUM(v)			FIELD_PREP(GENMASK(31, 28), v)
#define AVD_REF_FLAG_CONST		FIELD_PREP(BIT(24), 1)
#define AVD_REF_FLAG_LONG(v)		FIELD_PREP(BIT(17), !!(v))
#define AVD_REF_DELTA_POC(v)		FIELD_PREP(GENMASK(16, 0), v)

static inline u32 fifo_size(void)
{
	return 0x100000 * 12;
}

static inline u32 swrap(u32 x, u32 w)
{
	return x & (w - 1);
}
static inline bool boolify(u32 v)
{
	return !!(v);
}

static inline void push(struct avd_dev *avd, struct avd_ctx *ctx, u32 inst)
{
	writel(inst, avd->ctrl + avd->variant->vp_slot_offset + ctx->vp_slot * 4);
}

static inline void push_address(struct avd_dev *avd, struct avd_ctx *ctx,
				dma_addr_t addr)
{
	if (avd->variant->quirks & AVD_QUIRK_LSR) {
		push(avd, ctx, (addr >> 8));
	} else {
		push(avd, ctx, (u32)(addr & 0xffffffff));
		push(avd, ctx, (u32)(addr >> 32));
	}
}
static inline void push_rvra(struct avd_dev *avd, struct avd_ctx *ctx,
		dma_addr_t addr, u32 offsets[4])
{
	if (avd->variant->quirks & AVD_QUIRK_LSR) {
		for (int i = 0; i < 4; i++)
			push(avd, ctx, (addr + offsets[i]) >> 7);
	} else {
		for (int i = 0; i < 4; i++)
			push_address(avd, ctx, (addr + offsets[i]));
	}
}

#ifdef DEBUG_INST
#define push(inst, name)                                           \
	do {                                                       \
		dev_info(ctx->dev->dev, "%8x | %s", (inst), name); \
		push(avd, ctx, inst);                              \
	} while (0)

#else
#define push(inst, name) push(avd, ctx, inst)
#endif

#ifdef DEBUG_INST_ADDR
#define pusha(inst, name, i)                                                   \
	do {                                                                   \
		dev_info(ctx->dev->dev, "%8llx | %s[%d]", (inst) & 0xffffffff, \
			 name, i);                                             \
		dev_info(ctx->dev->dev, "%8llx | %s[%d] (high)", (inst) >> 32, \
			 name, i);                                             \
		push_address(avd, ctx, inst);                                  \
	} while (0)

#else
#define pusha(inst, name, i) push_address(avd, ctx, inst)
#endif

#endif /* AVD_INST_H_ */
