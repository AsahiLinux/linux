/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Based on Rockchip RGA driver
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Jacob Chen <jacob-chen@iotwrt.com>
 *
 * Copyright (C) 2026 The Asahi Linux Contributors
 * Copyright (C) 2026 Sofus Forstreuter <sofus.c@icloud.com>
 *
 */
#ifndef __MSR_H__
#define __MSR_H__

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

struct msr_fmt;

enum msr_frm_type {
	MSR_DST,
	MSR_SRC
};

struct msr_frame {
	enum msr_frm_type type;
	bool yuv;
	const struct msr_fmt *fmt;
	const struct v4l2_format_info *info;
	struct v4l2_pix_format_mplane pix;
	struct v4l2_rect crop;
};

#define IS_SRC(f)	((f)->type == MSR_SRC)
#define IS_DST(f)	((f)->type == MSR_DST)
/* logging helper */
#define FRM_STR(f)	(IS_SRC(f) ? "SRC" : "DST")

struct msr_ctx {
	struct v4l2_fh fh;
	struct msr_dev *msr;
	struct msr_frame src;
	struct msr_frame dst;
	struct v4l2_ctrl_handler ctrl_handler;

	int osequence;
	int csequence;

	/* Control values */
	u32 hflip;
	u32 vflip;
	u32 rotate;
	u32 fill_color;
};

static inline struct msr_ctx *file_to_msr_ctx(struct file *filp)
{
	return container_of(file_to_v4l2_fh(filp), struct msr_ctx, fh);
}

struct msr_hw;

struct msr_dev {
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct video_device *vfd;
	struct device *dev;
	void __iomem *base;
	u32 version;
	/* vfd lock */
	struct mutex mutex;
	/* ctrl parm lock */
	spinlock_t ctrl_lock;
	const struct msr_hw *hw;
};

struct msr_vb_buffer {
	struct vb2_v4l2_buffer vb_buf;
	struct list_head queue;

	/* Plane DMA addresses after the MMU mapping of the buffer */
	dma_addr_t dma_addrs[3];
	/* for compressed formats */
	dma_addr_t dma_meta_addrs[3];
	/* remove me */
	int stride[3];
};

static inline struct msr_vb_buffer *vb_to_msr(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct msr_vb_buffer, vb_buf);
}

struct msr_hw {
	u32 min_width, min_height;
	u32 max_width, max_height;
	u8 max_scaling_factor;
	u8 stride_alignment;
	void (*start)(struct msr_ctx *ctx,
		      struct msr_vb_buffer *src, struct msr_vb_buffer *dst);
	bool (*handle_irq)(struct msr_dev *msr);
	void (*get_version)(struct msr_dev *msr);
	void *(*adjust_and_map_format)(struct msr_ctx *ctx,
				       struct v4l2_pix_format_mplane *format,
				       bool is_output);
	int (*enum_format)(struct v4l2_fmtdesc *f);
};

extern int debug;

#define msr_dbg(level, msr, fmt, args...) \
	v4l2_dbg(level, debug, &(msr)->v4l2_dev, fmt, ##args)

extern const struct msr_hw msr_hw;

#endif
