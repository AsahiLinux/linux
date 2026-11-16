// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on Rockship RGA driver
 * Copyright (C) 2025-2026 Pengutronix e.K.
 * Author: Sven Püschel <s.pueschel@pengutronix.de>
 *
 * Copyright (C) 2026 The Asahi Linux Contributors
 * Copyright (C) 2026 Sofus Forstreuter <sofus.c@icloud.com>
 */

#include <linux/videodev2.h>
#include <linux/pm_runtime.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/printk.h>

#include <media/v4l2-common.h>

#include "msr-hw.h"
#include "msr.h"
#include "piodma.h"

enum msr_scaling_dir {
	MSR_VERTICAL,
	MSR_HORIZONTAL
};

static const struct msr_clip defclip = {
	.pre = {
		.max = { 0x7fffffff, 0x7fffffff, 0x7fffffff },
		.min = { 0x80000001, 0x80000001, 0x80000001 },
	},
	.post = {
		.max = { 0x7fffffff, 0x7fffffff, 0x7fffffff },
		.min = { 0x80000001, 0x80000001, 0x80000001 },
	}
};

static void msr_write(struct msr_ctx *ctx, size_t off, const void *src,
		      size_t count)
{
	if (ctx->msr->piodma) {
		int ret = piodma_add_packet(ctx->msr->piodma, off, src, count);
		if (ret) {
			dev_err(ctx->msr->dev, "piodma_add_packet: %d\n", ret);
		}
	} else {
		void __iomem *dst = ctx->msr->base + off;

#ifdef CONFIG_ARM64
		__iowrite32_copy(dst, src, count);
#else
		memcpy_toio(dst, src, count * 4);
#endif
	}
}

#define msr_writel(ctx, val, off) do { \
	u32 tmp = val; msr_write(ctx, off, &tmp, 1); \
} while (false)
#define msr_writeq(ctx, val, off) do { \
	u64 tmp = val; msr_write(ctx, off, &tmp, 2); \
} while (false)

#define DIV_FIXED(v, d, frac)	(((u64)(v) << (frac)) / (d))

static void configure_scaling(struct msr_ctx *ctx, enum msr_scaling_dir dir)
{
	size_t base = (dir == MSR_VERTICAL ? 0x1000 : 0x2000);
	u32 srcdim, dstdim, config = 0;
	u32 ratio[3][2], dda[3];

	/* Runs before rotating */
	if (dir == MSR_VERTICAL) {
		srcdim = ctx->src.pix.height;
		dstdim = ctx->dst.pix.height;
	} else {
		srcdim = ctx->src.pix.width;
		dstdim = ctx->dst.pix.width;
	}

	if (srcdim == dstdim)
		goto configure;
	else
		config = MSR_SCALE_EN;

	/* DDA start offsets */
	dda[0] = (DIV_FIXED(srcdim, dstdim, MSR_SCALE_FRAC) -
			(1 << MSR_SCALE_FRAC)) / 2;
	dda[1] = (DIV_FIXED(srcdim, dstdim, MSR_SCALE_FRAC) -
			(1 << MSR_SCALE_FRAC)) / 2;
	dda[2] = (DIV_FIXED(srcdim, dstdim, MSR_SCALE_FRAC) -
			(1 << MSR_SCALE_FRAC)) / 2;

	ratio[0][0] = DIV_FIXED(srcdim, dstdim, MSR_SCALE_FRAC);
	ratio[0][1] = DIV_FIXED(dstdim, srcdim, MSR_SCALE_FRAC);

	ratio[1][0] = DIV_FIXED(srcdim, dstdim, MSR_SCALE_FRAC);
	ratio[1][1] = DIV_FIXED(dstdim, srcdim, MSR_SCALE_FRAC);

	ratio[2][0] = DIV_FIXED(srcdim, dstdim, MSR_SCALE_FRAC);
	ratio[2][1] = DIV_FIXED(dstdim, srcdim, MSR_SCALE_FRAC);

	msr_writel(ctx, dda[0], base  + 0x04);
	msr_writel(ctx, dda[1], base  + 0x08);
	msr_writel(ctx, ratio[0][0], base + 0x0c);
	msr_writel(ctx, ratio[1][0], base + 0x10);
	msr_writel(ctx, ratio[0][1], base + 0x14);
	msr_writel(ctx, ratio[1][1], base + 0x18);
	msr_writel(ctx, dda[2], base  + 0x1c);
	msr_writel(ctx, ratio[2][0], base + 0x20);
	msr_writel(ctx, ratio[2][1], base + 0x24);

	if (dir == MSR_VERTICAL) {
		msr_write(ctx, base + 0x400, vfilter.block0,
			  sizeof(vfilter.block0) / 4);
		msr_write(ctx, base + 0xc00, vfilter.block1,
			  sizeof(vfilter.block1) / 4);
	} else {
		msr_write(ctx, base + 0x400, hfilter.block0,
			  sizeof(hfilter.block0) / 4);
		msr_write(ctx, base + 0xc00, hfilter.block1,
			  sizeof(hfilter.block1) / 4);
	}

configure:
	msr_writel(ctx, config, base);
	msr_dbg(2, ctx->msr, "SCALE: %c mode %8x\n",
		dir == MSR_VERTICAL ? 'V' : 'H',
		readl_relaxed(ctx->msr->base + base));
}

static void configure_fmt(struct msr_ctx *ctx, struct msr_vb_buffer *vb,
			  struct msr_frame *frm)
{
	const struct msr_fmt *fmt = frm->fmt;
	void __iomem *base = ctx->msr->base;
	size_t reg = 0;
	int i;

	reg = IS_SRC(frm) ? 0x180 : 0x280;
	msr_writel(ctx, MSR_DMA_ALPHA(fmt->alpha) |
		   MSR_DMA_SUBSAMPLING(fmt->subsampling), reg);
	msr_dbg(2, ctx->msr, "%s: %x\n", FRM_STR(frm),
		readl_relaxed(base + reg));

	for (i = 0; i < MSR_MAX_PLANES; i++) {
		msr_writel(ctx, MSR_DMA_STORAGE(fmt->storage[i]),
			   reg + 4 + (i * 4));
		msr_dbg(2, ctx->msr, "%s[%d]: %05x\n", FRM_STR(frm), i,
			readl_relaxed(base + reg + 4 + (i * 4)));
	}

	/* XXX: can src have 4 planes? */
	reg = IS_SRC(frm) ? 0x198 : 0x290;
	for (i = 0; i < MSR_MAX_PLANES; i++) {
		msr_writeq(ctx, vb->dma_addrs[i], reg + i * 8);
		msr_dbg(2, ctx->msr, "%s[%d] dma:  %12llx\n", FRM_STR(frm), i,
			readq_relaxed(base + reg + i * 8));
	}

	reg = IS_SRC(frm) ? 0x1b8 : 0x2a8;
	for (i = 0; i < MSR_MAX_PLANES; i++) {
		msr_writeq(ctx, vb->dma_meta_addrs[i], reg + i * 8);
		msr_dbg(2, ctx->msr, "%s[%d] meta: %12llx\n", FRM_STR(frm), i,
			readq_relaxed(base + reg + i * 8));
	}

	reg = IS_SRC(frm) ? 0x1d8 : 0x2c0;
	for (i = 0; i < MSR_MAX_PLANES; i++)
		msr_writel(ctx, vb->stride[i], reg + i * 4);

	/* offsets? */
	reg = IS_SRC(frm) ? 0x1e8 : 0x2cc;

	/* component order, so hw can normalize */
	reg = IS_SRC(frm) ? 0x1f8 : 0x2d8;
	msr_writel(ctx, MSR_FMT_COMP0(fmt->comp_order[0]) |
			MSR_FMT_COMP1(fmt->comp_order[1]) |
			MSR_FMT_COMP2(fmt->comp_order[2]) |
			MSR_FMT_COMP3(fmt->comp_order[3]), reg);

	msr_writel(ctx, frm->pix.width, reg + 4);
	msr_writel(ctx, frm->pix.height, reg + 8);

	/* plane sizes */
	reg = IS_SRC(frm) ? 0x234 : 0x308;
	msr_writel(ctx, frm->pix.width, reg);
	msr_writel(ctx, frm->pix.width / frm->info->hdiv, reg + 4);
	msr_writel(ctx, frm->pix.width, reg + 8);

	reg = IS_SRC(frm) ? 0x244 : 0x314;
	msr_writel(ctx, frm->pix.height, reg);
	msr_writel(ctx, frm->pix.height / frm->info->vdiv, reg + 4);
	msr_writel(ctx, frm->pix.height, reg + 8);
}

static void configure_matrix(struct msr_ctx *ctx, enum msr_mat loc,
			     const struct msr_matrix *mat,
			     const struct msr_clip *clip)
{
	static const struct mat_info {
		u32 mat;
		u32 clip;
	} matrix_info[] = {
		/* enable << 31 in frac_bits, Q8.10 */
		[MAT_PRE_SCALER] =	{ .mat = 0x0824, .clip = 0x0000 },
		/* Q16.10 */
		[MAT_PRE_CM] =		{ .mat = 0x3804, .clip = 0xa98c },
		/* Q16.16 */
		[MAT_POST_DEGAMMA] =	{ .mat = 0x505c, .clip = 0xa9bc },
		/* Q22.10 */
		[MAT_POST_CM] =		{ .mat = 0xa94c, .clip = 0xaaac },
	};
	const struct mat_info *info = &matrix_info[loc];

	msr_write(ctx, info->mat, mat, sizeof(*mat) / 4);
	if (!info->clip)
		return;
	msr_write(ctx, info->clip, clip, sizeof(*clip) / 4);
}

static void configure_convert_revert(struct msr_ctx *ctx,
				     struct msr_frame *frm)
{
	const struct msr_fmt *fmt = frm->fmt;
	size_t base = 0x13800 + (IS_SRC(frm) ? 0 : 0x400);

	msr_writel(ctx, 1, base + 0);
	msr_writel(ctx, 0, base + 4);

	if (IS_SRC(frm)) {
		msr_writel(ctx, 0, base + 8);
		msr_writel(ctx, 0, base + 12);
	}

	msr_writel(ctx, fmt->comp_bits[0], base + 16);
	msr_writel(ctx, fmt->comp_bits[1], base + 20);
	msr_writel(ctx, fmt->comp_bits[2], base + 24);
	msr_writel(ctx, fmt->comp_bits[3], base + 28);
}

static void configure_chroma_sampling(struct msr_ctx *ctx)
{
	size_t reg = 0x800;
	const struct msr_fmt *src = ctx->src.fmt;
	const struct msr_fmt *dst = ctx->dst.fmt;
	bool enabled;
	u8 frac = 12;

	/* chroma upsampling */
	enabled = !!src->subsampling;
	msr_writel(ctx, frac | enabled, reg);

	/* some kinda chroma upsampling filter */
	msr_writel(ctx, 0, reg + 4);
	msr_writel(ctx, DIV_FIXED(1, 2, frac), reg + 8);
	msr_writel(ctx, DIV_FIXED(1, 2, frac), reg + 12);
	msr_writel(ctx, 0, reg + 16);

	msr_writel(ctx, 0, reg + 20);
	msr_writel(ctx, DIV_FIXED(1, 2, frac), reg + 24);
	msr_writel(ctx, DIV_FIXED(1, 2, frac), reg + 28);
	msr_writel(ctx, 0, reg + 32);

	/* chroma downsampling */
	msr_writel(ctx, dst->subsampling | (1 << 4) | (1 << 16), reg + 0x100);
}

static void configure_ase(struct msr_ctx *ctx)
{
	/* why does it deserve its on power domain?? */

	/* ASE enhancement */
	msr_writel(ctx, 0, 0x16800);

	/* ASE 3x1 transform */
	msr_writel(ctx, 0x0, 0x16080);
	msr_writel(ctx, 0xb710367, 0x16084);
	msr_writel(ctx, 0x128,  0x16088);

	/* ASE interpolation */
	msr_writel(ctx, 0x15, 0x16600);

	/* ASE angle detect */
	msr_writel(ctx, 0x2000500, 0x16504);
	msr_writel(ctx, 0x3200, 0x16508);
	msr_writel(ctx, 0x8, 0x16534);
	msr_writel(ctx, 0x851400, 0x1651c);
	msr_writel(ctx, 0x250500, 0x16568);
	msr_writel(ctx, 0x496513, 0x16588);

	/* ASE config */
	msr_writel(ctx, 0x0, 0x16000);
}

static bool csc_needed(struct msr_frame *src, struct msr_frame *dst)
{
	if (src->pix.colorspace != dst->pix.colorspace)
		return true;
	if (src->pix.xfer_func != dst->pix.xfer_func)
		return true;
	if (src->yuv != dst->yuv)
		return true;
	if (!(src->yuv && dst->yuv))
		return false;

	if (src->pix.ycbcr_enc != dst->pix.ycbcr_enc)
		return true;
	if (src->pix.quantization != dst->pix.quantization)
		return true;

	return false;
}

static const struct msr_matrix *get_ycbcr_enc_matrix(struct msr_frame *frm)
{
	enum v4l2_ycbcr_encoding enc = frm->pix.ycbcr_enc;
	bool full = frm->pix.quantization == V4L2_QUANTIZATION_FULL_RANGE;

	if (enc == V4L2_YCBCR_ENC_DEFAULT)
		enc = V4L2_MAP_YCBCR_ENC_DEFAULT(frm->pix.colorspace);

	return IS_SRC(frm) ?
		ycbcr_dec_tlb[enc][full] :
		ycbcr_enc_tlb[enc][full];
}

static void configure_colors(struct msr_ctx *ctx)
{
	struct msr_dev *msr = ctx->msr;
	struct msr_frame *dst = &ctx->dst, *src = &ctx->src;
	const u32 *de_gamma = NULL, *en_gamma = NULL;
	const struct msr_matrix *csc_m = NULL;
	const struct msr_matrix *ycbcr_enc;
	u32 config = 0;

	if (!csc_needed(src, dst))
		goto configure;
	else
		config = MSR_CM_EN;

	if (src->yuv) {
		ycbcr_enc = get_ycbcr_enc_matrix(src);
		if (ycbcr_enc) {
			configure_matrix(ctx, MAT_PRE_CM, ycbcr_enc, &defclip);
			config |= MSR_CMM_PRE;
		}
	}

	if (dst->yuv) {
		ycbcr_enc = get_ycbcr_enc_matrix(dst);
		if (ycbcr_enc) {
			configure_matrix(ctx, MAT_POST_CM, ycbcr_enc, &defclip);
			config |= MSR_CMM_POST;
		}
	}

	if (src->pix.colorspace == dst->pix.colorspace &&
	    src->pix.xfer_func == dst->pix.xfer_func)
		goto configure;

	de_gamma = degamma_tlb[src->pix.xfer_func];
	if (de_gamma) {
		config |= MSR_CM_DEGAMMA;
		for (int c = 0; c < MSR_MAX_PLANES; c++)
			msr_write(ctx, 0x20e00 + c * MSR_DEGAMMA_TLB_LEN *
				  sizeof(u32), de_gamma, MSR_DEGAMMA_TLB_LEN);
	}

	en_gamma = engamma_tlb[dst->pix.xfer_func];
	if (en_gamma) {
		config |= MSR_CM_ENGAMMA;
		for (int c = 0; c < MSR_MAX_PLANES; c++)
			msr_write(ctx, 0x8834 + c * MSR_ENGAMMA_TLB_LEN *
				  sizeof(u32), en_gamma, MSR_ENGAMMA_TLB_LEN);
	}

	csc_m = csc_tlb[src->pix.colorspace][dst->pix.colorspace];
	if (csc_m) {
		struct msr_clip csc_clip = {
			.pre = {
				.max = { 0xffffffff, 0xffffffff, 0xffffffff },
				.min = { 0xffffffff, 0xffffffff, 0xffffffff },
			},
			.post = {
				.max = { 0x7fffffff, 0x7fffffff, 0x7fffffff },
				.min = { 0x80000001, 0x80000001, 0x80000001 },
			}
		};

		config |= MSR_CMM_POST_DEGAMMA |
			MSR_CM_CSC_CLIP_MIN0 |
			MSR_CM_CSC_CLIP_MIN1 |
			MSR_CM_CSC_CLIP_MIN2;

		configure_matrix(ctx, MAT_POST_DEGAMMA, csc_m, &csc_clip);
	}


configure:
	msr_writel(ctx, config, 0x3800);
	msr_dbg(2, msr, "CM: config %8x\n",
		readl_relaxed(msr->base + 0x3800));
}

static void msr_hw_start(struct msr_ctx *ctx,
			 struct msr_vb_buffer *src, struct msr_vb_buffer *dst)
{
	struct msr_dev *msr = ctx->msr;
	const struct msr_fmt *dstf = ctx->dst.fmt, *srcf = ctx->src.fmt;

	/* reset */
	writel(1, msr->base + 0x4);
	writel(0, msr->base + 0x4);

	if (ctx->msr->piodma)
		piodma_start_packet(ctx->msr->piodma);

	msr_writel(ctx, 0xffffffff, 0x18); /* enable irq */

	/* transform config */
	msr_writel(ctx, (ctx->rotate / 90) | ctx->hflip << 2 | ctx->vflip << 3,
		   0x380);

	configure_fmt(ctx, src, &ctx->src);
	configure_fmt(ctx, dst, &ctx->dst);

	/* pixel averaging 1 and 2 does the same? */
	msr_writel(ctx, 0, 0xe4);

	configure_chroma_sampling(ctx);

	configure_scaling(ctx, MSR_VERTICAL);
	configure_scaling(ctx, MSR_HORIZONTAL);

	configure_ase(ctx);

	/* disable pseudo linear scaling */
	msr_writel(ctx, 0, 0x480);

	/* disabled reshape */
	msr_writel(ctx, 0, 0xe8);

	configure_colors(ctx);

	msr_writel(ctx, MSR_ALPHA_WRITE_EN(dstf->alpha && !srcf->alpha) |
			MSR_ALPHA_WRITE(0xffff), 0x8c);

	/* disable dither */
	msr_writel(ctx, 0, 0xa00);

	/* convert/revert map */
	configure_convert_revert(ctx, &ctx->src);
	configure_convert_revert(ctx, &ctx->dst);

	/* (don't) program histogram */
	msr_writel(ctx, 0x0, 0x3000);
	msr_writel(ctx, 0x0, 0x124);

	msr_dbg(2, msr, "tagging: %d\n", ctx->csequence);
	msr_writel(ctx, ctx->csequence, 0x110);

	msr_writel(ctx, MSR_TIMEOUT_MILLIS(500), 0x98);
	/* mode/start */
	msr_writel(ctx, 1, 0x80);

	if (ctx->msr->piodma)
		piodma_submit_packet(ctx->msr->piodma);
}

#define MSR_IRQ_DBG(v, ev)	(((v) & MSR_IRQ_##ev) ? #ev " " : "")

static bool msr_handle_irq(struct msr_dev *msr)
{
	u32 i = readl(msr->base + 0x1c);
	u32 unk = i & ~(MSR_IRQ_DONE | MSR_IRQ_DBGSTS_ERR | MSR_IRQ_READ_ERR |
			MSR_IRQ_WRITE_ERR | MSR_IRQ_DECOMP_ERR |
			MSR_IRQ_CTX_SWITCH | MSR_IRQ_AXI_ERR);

	writel(0, msr->base + 0x18); /* disable irq */
	msr_dbg(2, msr, "IRQ: GLBL_IRQSTS: %8x CTRL_DBGSTS: %8x\n",
		i, readl(msr->base + 0x9c));
	msr_dbg(2, msr, "DBG: %s%s%s%s%s%s%s\n",
		MSR_IRQ_DBG(i, DONE), MSR_IRQ_DBG(i, DBGSTS_ERR),
		MSR_IRQ_DBG(i, READ_ERR), MSR_IRQ_DBG(i, WRITE_ERR),
		MSR_IRQ_DBG(i, DECOMP_ERR), MSR_IRQ_DBG(i, CTX_SWITCH),
		MSR_IRQ_DBG(i, AXI_ERR));
	if (unk)
		dev_err(msr->dev, "IRQ: UNKNOWN: %x\n", unk);
	msr_dbg(2, msr, "TRANSFORM_TAG: %d\n", readl_relaxed(msr->base + 0x110));
	msr_dbg(2, msr, "FRAME_COUNT: %d\n", readl_relaxed(msr->base + 0x20));
	msr_dbg(2, msr, "RUNNING: %d\n", readl_relaxed(msr->base + 0xc));

	return i & MSR_IRQ_DONE;
}

static void msr_get_version(struct msr_dev *msr)
{
	msr->version = readl(msr->base);
}

#define COMP_ORDER(c0, c1, c2, c3)	\
	{COMP_##c0, COMP_##c1, COMP_##c2, COMP_##c3}
#define YCbCr_ORDER COMP_ORDER(R, G, B, A)

static struct msr_fmt formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.subsampling = SUBSAMPLING_420,
		.comp_order = YCbCr_ORDER,
		.comp_bits = {8, 8, 8},
		.storage = {
			MSR_STORAGE_1C8B8,
			MSR_STORAGE_2C8B16
		},
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.subsampling = SUBSAMPLING_422,
		.comp_order = YCbCr_ORDER,
		.comp_bits = {8, 8, 8},
		.storage = {
			MSR_STORAGE_1C8B8,
			MSR_STORAGE_2C8B16
		},
	},
	{
		.fourcc = V4L2_PIX_FMT_NV24,
		.subsampling = SUBSAMPLING_444,
		.comp_order = YCbCr_ORDER,
		.comp_bits = {8, 8, 8},
		.storage = {
			MSR_STORAGE_1C8B8,
			MSR_STORAGE_2C8B16
		},
	},
	{
		.fourcc = V4L2_PIX_FMT_P010,
		.subsampling = SUBSAMPLING_420,
		.comp_order = YCbCr_ORDER,
		.comp_bits = {10, 10, 10},
		.storage = {
			MSR_STORAGE_1C10B16,
			MSR_STORAGE_2C10B32
		},
	},
	{
		.fourcc = V4L2_PIX_FMT_P210,
		.subsampling = SUBSAMPLING_422,
		.comp_order = YCbCr_ORDER,
		.comp_bits = {10, 10, 10},
		.storage = {
			MSR_STORAGE_1C10B16,
			MSR_STORAGE_2C10B32
		},
	},
#ifdef V4L2_PIX_FMT_P410
	{
		.fourcc = V4L2_PIX_FMT_P410,
		.subsampling = SUBSAMPLING_444,
		.comp_order = YCbCr_ORDER,
		.comp_bits = {10, 10, 10},
		.storage = {
			MSR_STORAGE_1C10B16,
			MSR_STORAGE_2C10B32
		},
	},
#endif
	{
		.fourcc = V4L2_PIX_FMT_P012,
		.subsampling = SUBSAMPLING_420,
		.comp_order = YCbCr_ORDER,
		.comp_bits = {12, 12, 12},
		.storage = {
			MSR_STORAGE_1C12B16,
			MSR_STORAGE_2C12B32
		},
	},
#ifdef V4L2_PIX_FMT_P016
	{
		.fourcc = V4L2_PIX_FMT_P016,
		.subsampling = SUBSAMPLING_420,
		.comp_order = YCbCr_ORDER,
		.comp_bits = {16, 16, 16},
		.storage = {
			MSR_STORAGE_1C16B16,
			MSR_STORAGE_2C16B32
		},
	},
#endif
	{
		.fourcc = V4L2_PIX_FMT_RGBX32,
		.comp_order = COMP_ORDER(R, G, B, A),
		.comp_bits = {8, 8, 8, 8},
		.storage = {MSR_STORAGE_4C8B32},
	},
	{
		.fourcc = V4L2_PIX_FMT_RGBA32,
		.alpha = true,
		.comp_order = COMP_ORDER(R, G, B, A),
		.comp_bits = {8, 8, 8, 8},
		.storage = {MSR_STORAGE_4C8B32},
	},
	{
		.fourcc = V4L2_PIX_FMT_ABGR32,
		.alpha = true,
		.comp_order = COMP_ORDER(B, G, R, A),
		.comp_bits = {8, 8, 8, 8},
		.storage = {MSR_STORAGE_4C8B32},
	},
	{
		.fourcc = V4L2_PIX_FMT_RGBX1010102,
		.comp_order = COMP_ORDER(R, G, B, A),
		.comp_bits  = {10, 10, 10, 2},
		.storage = {MSR_STORAGE_4C2101010B32},
	},
	{
		.fourcc = V4L2_PIX_FMT_RGBA1010102,
		.alpha = true,
		.comp_order = COMP_ORDER(R, G, B, A),
		.comp_bits  = {10, 10, 10, 2},
		.storage = {MSR_STORAGE_4C2101010B32},
	},
	{
		.fourcc = V4L2_PIX_FMT_ARGB2101010,
		.alpha = true,
		.comp_order = COMP_ORDER(B, G, R, A),
		.comp_bits  = {10, 10, 10, 2},
		.storage = {MSR_STORAGE_4C2101010B32},
	},
};

static int msr_enum_format(struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	f->pixelformat = formats[f->index].fourcc;
	return 0;
}

static void *msr_adjust_and_map_format(struct msr_ctx *ctx,
				       struct v4l2_pix_format_mplane *format,
				       bool is_output)
{
	unsigned int i;

	if (!format)
		return &formats[0];

	for (i = 0; i < ARRAY_SIZE(formats); i++) {
		if (formats[i].fourcc == format->pixelformat)
			return &formats[i];
	}

	format->pixelformat = formats[0].fourcc;
	return &formats[0];
}

const struct msr_hw msr_hw = {
	.min_width = 1,
	.min_height = 1,
	.max_width = 1 << 16,
	.max_height = 1 << 16,
	.max_scaling_factor = 16,
	.stride_alignment = 16,
	.start = msr_hw_start,
	.handle_irq = msr_handle_irq,
	.get_version = msr_get_version,
	.enum_format = msr_enum_format,
	.adjust_and_map_format = msr_adjust_and_map_format,
};
