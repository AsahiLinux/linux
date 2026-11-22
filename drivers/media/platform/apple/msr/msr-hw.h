/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Based on Rockship RGA driver
 * Copyright (C) Pengutronix e.K.
 * Author: Sven Püschel <s.pueschel@pengutronix.de>
 *
 * Copyright (C) 2026 Sofus Forstreuter <sofus.c@icloud.com>
 * Copyright (C) 2026 The Asahi Linux Contributors
 *
 */

#ifndef __MSR_HW_H__
#define __MSR_HW_H__

#include <linux/bits.h>
#include <linux/types.h>

#define MSR_MAX_PLANES	3 /* maybe 4 for src? */
#define MSR_COMPONENTS	4

#define MSR_DEGAMMA_TLB_LEN	1281
#define MSR_ENGAMMA_TLB_LEN	706

#define MSR_IRQ_DONE		BIT(0)
#define MSR_IRQ_DBGSTS_ERR	BIT(1) /* timeout */
#define MSR_IRQ_READ_ERR	BIT(3)
#define MSR_IRQ_WRITE_ERR	BIT(4)
#define MSR_IRQ_DECOMP_ERR	BIT(9)
#define MSR_IRQ_CTX_SWITCH	BIT(10)
#define MSR_IRQ_AXI_ERR		BIT(12)

/* XXX: what does 1 do? */
#define MSR_TIMEOUT_MILLIS(t)	FIELD_PREP(GENMASK(31, 1), (t) * 200000)

#define MSR_SCALE_FRAC	22
#define MSR_SCALE_EN	BIT(0)

/* components, bits per component, bits per block */
#define MSR_STORAGE_1C8B8		0x01
#define MSR_STORAGE_1C10B16		0x02
#define MSR_STORAGE_1C12B16		0x03
/* 1C14B16 hangs */
#define MSR_STORAGE_1C16B16		0x05
#define MSR_STORAGE_2C8B16		0x06
#define MSR_STORAGE_1C10B32		0x10 /* Packed 3 per block */
#define MSR_STORAGE_2C10B32		0x12
#define MSR_STORAGE_2C12B32		0x13
#define MSR_STORAGE_2C16B32		0x15
#define MSR_STORAGE_4C2101010B32	0x19
#define MSR_STORAGE_4C8B32		0x1e
#define MSR_STORAGE_2C10B64		0x21 /* Packed 3 per block */

/*
 * Only for luma? Why??
 */
#define MSR_STORAGE_AX_1C8B8		0x38
#define MSR_STORAGE_AX_1C10B32		0x3c

enum msr_modifier {
	MSR_MOD_NONE = 0,
	MSR_MOD_AX = 1, /* with metadata buffers */
	MSR_MOD_UNK = 2, /* without metadata, much smaller? */
};

#define MSR_DMA_SUBSAMPLING(v)	FIELD_PREP(GENMASK(3, 2), v)
#define MSR_DMA_ALPHA(v)	FIELD_PREP(BIT(0), !!(v))

#define MSR_DMA_STORAGE(v)	FIELD_PREP(GENMASK(7, 0), v)
#define MSR_DMA_MOD(v)		FIELD_PREP(GENMASK(9, 8), v)

#define MSR_FMT_COMP0(v)	FIELD_PREP(GENMASK(1, 0), v)
#define MSR_FMT_COMP1(v)	FIELD_PREP(GENMASK(9, 8), v)
#define MSR_FMT_COMP2(v)	FIELD_PREP(GENMASK(17, 16), v)
#define MSR_FMT_COMP3(v)	FIELD_PREP(GENMASK(25, 24), v)

#define MSR_CM_EN		BIT(0)
#define MSR_CMM_PRE		BIT(1)
#define MSR_CM_DEGAMMA		BIT(2)
#define MSR_CMM_POST_DEGAMMA	BIT(2)
#define MSR_CM_ENGAMMA		BIT(15)
#define MSR_CMM_POST		BIT(16)
/* POST_DEGAMMA has configurable pre clip signs */
#define MSR_CM_CSC_CLIP_MIN0	BIT(17)
#define MSR_CM_CSC_CLIP_MIN1	BIT(18)
#define MSR_CM_CSC_CLIP_MIN2	BIT(19)
#define MSR_CM_CSC_CLIP_MAX0	BIT(20)
#define MSR_CM_CSC_CLIP_MAX1	BIT(21)
#define MSR_CM_CSC_CLIP_MAX2	BIT(22)

#define MSR_ALPHA_WRITE_EN(v)	FIELD_PREP(BIT(31), !!(v))
#define MSR_ALPHA_WRITE(v)	FIELD_PREP(GENMASK(15, 0), v)

enum subsampling {
	SUBSAMPLING_NONE = 0,
	SUBSAMPLING_444 = 0,
	SUBSAMPLING_422 = 1,
	SUBSAMPLING_421 = 2, /* not tested */
	SUBSAMPLING_420 = 3,
};

enum msr_mat {
	MAT_PRE_SCALER = 0,
	MAT_PRE_CM,
	MAT_POST_DEGAMMA,
	MAT_POST_CM,
};

#define COMP_B	0
#define COMP_G	1
#define COMP_R	2
#define COMP_A	3

struct msr_fmt {
	u32 fourcc;
	bool alpha;
	enum subsampling subsampling;
	u8 comp_order[MSR_COMPONENTS];
	/* after changing order */
	u8 comp_bits[MSR_COMPONENTS];
	u8 storage[MSR_MAX_PLANES];
	enum msr_modifier mod[MSR_MAX_PLANES];
};

struct msr_matrix {
	u32 frac_bits;
	u32 pre[3];
	u32 mat[3][3];
	u32 post[3];
};

struct msr_clip_val {
	u32 min[3];
	u32 max[3];
};

struct msr_clip {
	struct msr_clip_val pre;
	struct msr_clip_val post;
};

#define PHASES	32
#define HTAPS	15
#define VTAPS	9

struct msr_hfilter {
	u16 block0[HTAPS][PHASES][2];
	u16 block1[HTAPS][PHASES];
};

struct msr_vfilter {
	u16 block0[VTAPS][PHASES][2];
	u16 block1[VTAPS][PHASES];
};

extern const u32 *degamma_tlb[V4L2_XFER_FUNC_LAST];
extern const u32 *engamma_tlb[V4L2_XFER_FUNC_LAST];

extern const struct msr_vfilter vfilter;
extern const struct msr_hfilter hfilter;

extern const struct msr_matrix *ycbcr_enc_tlb[V4L2_YCBCR_ENC_LAST][2];
extern const struct msr_matrix *ycbcr_dec_tlb[V4L2_YCBCR_ENC_LAST][2];

extern const
struct msr_matrix *csc_tlb[V4L2_COLORSPACE_LAST][V4L2_COLORSPACE_LAST];

#endif /* __MSR_HW_H__ */
