/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple Video Decoder driver
 *
 * Copyright The Asahi Linux Contributors
 *
 * Based on rkvdec driver by Collabora, Ltd.
 * Copyright (C) 2019 Collabora, Ltd.
 * Based on rkvdec driver by Google LLC. (Tomasz Figa <tfiga@chromium.org>)
 * Based on s5p-mfc driver by Samsung Electronics Co., Ltd.
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 */

#ifndef AVD_H_
#define AVD_H_

#include "linux/bitmap.h"
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/iommu.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ioctl.h>

#define AVD_CAPABILITY_HEVC	BIT(0)
#define AVD_CAPABILITY_H264	BIT(1)
#define AVD_CAPABILITY_VP9	BIT(2)

/* Shifts addreses right and bytesperline?? */
#define AVD_QUIRK_LSR	BIT(0)
#define AVD_QUIRK_NO_PIPE_STATE	 BIT(1)

#define VP_SLOT_NONE		255
#define INST_FIFO_SLOT_NONE	255


struct avd_ctx;
struct avd_dev;

/* Matches hdr mode (inst stream) and register layout */
enum avd_codec {
	AVD_CODEC_HEVC = 0,
	AVD_CODEC_H264 = 1,
	AVD_CODEC_VP9 = 2,
	AVD_CODEC_AV1 = 3
};

struct avd_run {
	struct {
		struct vb2_v4l2_buffer *src; /* OUTPUT coded */
		struct vb2_v4l2_buffer *dst; /* CAPTURE decoded */
	} bufs;
};

struct avd_ctrl_desc {
	struct v4l2_ctrl_config cfg;
};

struct avd_ctrls {
	const struct avd_ctrl_desc *ctrls;
	unsigned int num_ctrls;
};

struct avd_rvra {
	u32 offsets[4]; /* sizes or offsets */
	u32 size;
};

/* TODO: change and use this */
struct avd_decoded_buffer {
	/* Must be the first field in this struct. */
	struct v4l2_m2m_buffer base;

	struct avd_rvra rvra;
};

static inline struct avd_decoded_buffer *
vb2_to_avd_decoded_buf(struct vb2_buffer *buf)
{
	return container_of(buf, struct avd_decoded_buffer, base.vb.vb2_buf);
}

struct avd_coded_fmt_ops {
	void (*adjust_decoded_fmt)(struct avd_ctx *ctx,
			struct v4l2_pix_format_mplane *pix_mp);
	int (*start)(struct avd_ctx *ctx);
	void (*stop)(struct avd_ctx *ctx);
	int (*run)(struct avd_ctx *ctx);
	void (*done)(struct avd_ctx *ctx, struct vb2_v4l2_buffer *src_buf,
		     struct vb2_v4l2_buffer *dst_buf,
		     enum vb2_buffer_state result);
	int (*try_ctrl)(struct avd_ctx *ctx, struct v4l2_ctrl *ctrl);
	enum avd_image_fmt (*get_image_fmt)(struct avd_ctx *ctx,
					    struct v4l2_ctrl *ctrl);
};

enum avd_image_fmt {
	AVD_IMG_FMT_ANY = 0,
	AVD_IMG_FMT_420_8BIT,
	AVD_IMG_FMT_420_10BIT,
	AVD_IMG_FMT_422_8BIT,
	AVD_IMG_FMT_422_10BIT,
};

struct avd_decoded_fmt_desc {
	u32 fourcc;
	enum avd_image_fmt image_fmt;
};

struct avd_coded_fmt_desc {
	u32 fourcc;
	struct v4l2_frmsize_stepwise frmsize;
	const struct avd_ctrls *ctrls;
	const struct avd_coded_fmt_ops *ops;
	unsigned int num_decoded_fmts;
	const struct avd_decoded_fmt_desc *decoded_fmts;
	u32 subsystem_flags;
	unsigned int capability;
};

struct avd_variant {
	unsigned int vp_slots[4];
	unsigned int fifo_slots;
	unsigned int capabilities;
	void (*configure_stream)(struct avd_dev *avd, dma_addr_t addr,
			u8 fifo_idx, u32 vp_slot);
	const char* fw_name;
	unsigned char revision; /* the same as the device tree */
	/* just for convenience */
	u32 vp_slot_offset;
	u32 submit_offset;
	unsigned int quirks;
};

struct avd_dev {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct platform_device *pdev;

	const struct firmware *fw; /* fw is lost on suspend */

	void __iomem *code;
	void __iomem *mbox;
	void __iomem *ctrl;

	struct iommu_domain *domain;
	struct iommu_domain *empty_domain;

	struct mutex vdev_lock;

	struct reset_control *rstc;

	unsigned long vp_slots;
	unsigned long inst_fifo_slots;

	const struct avd_variant *variant;
};

struct avd_ctx {
	struct v4l2_fh fh;
	struct avd_dev *dev;

	struct v4l2_format coded_fmt;
	struct v4l2_format decoded_fmt;

	const struct avd_coded_fmt_desc *coded_fmt_desc;
	struct v4l2_ctrl_handler ctrl_hdl;
	enum avd_image_fmt image_fmt;

	struct delayed_work watchdog_work;

	void *priv;

	/* reference VRA (video resolution adaptation) scaler buffer. */
	struct avd_rvra rvra;

	u8 fifo_idx;
	u8 vp_slot;
};

struct avd_buf {
	void *cpu;
	dma_addr_t addr;
	size_t size;
};

int avd_buf_alloc(struct avd_dev *avd, struct avd_buf *buf, size_t size);
void avd_buf_free(struct avd_dev *avd, struct avd_buf *buf);

void avd_reset_coded_fmt(struct avd_ctx *ctx);
void avd_reset_decoded_fmt(struct avd_ctx *ctx);
int avd_init_ctrls(struct avd_ctx *ctx);

void avd_job_finish_no_pm(struct avd_ctx *ctx, enum vb2_buffer_state result);
void avd_job_finish(struct avd_ctx *ctx, enum vb2_buffer_state result);

void avd_run_preamble(struct avd_ctx *ctx, struct avd_run *run);
void avd_run_postamble(struct avd_ctx *ctx, struct avd_run *run);

extern const struct avd_coded_fmt_ops avd_h264_fmt_ops;
extern const struct avd_coded_fmt_ops avd_hevc_fmt_ops;
extern const struct avd_coded_fmt_ops avd_vp9_fmt_ops;

extern const struct v4l2_ctrl_ops avd_ctrl_ops;
extern const struct v4l2_ioctl_ops avd_ioctl_ops;
extern const struct vb2_ops avd_queue_ops;

static inline u32 fmt_height(struct avd_ctx *ctx)
{
	return ctx->coded_fmt.fmt.pix_mp.height;
}
static inline u32 fmt_width(struct avd_ctx *ctx)
{
	return ctx->coded_fmt.fmt.pix_mp.width;
}

void fill_rvra(struct avd_rvra *rvra, enum avd_image_fmt image_fmt,
		u32 width, u32 height);
int alloc_slots(struct avd_dev *avd, struct avd_ctx *ctx, enum avd_codec codec);

static inline void free_vp_slot(struct avd_dev *avd, struct avd_ctx *ctx)
{
	clear_bit(ctx->vp_slot, &avd->vp_slots);
	ctx->vp_slot = VP_SLOT_NONE;
}

static inline void free_inst_slot(struct avd_dev *avd, struct avd_ctx *ctx)
{
	clear_bit(ctx->fifo_idx, &avd->inst_fifo_slots);
	ctx->fifo_idx = INST_FIFO_SLOT_NONE;
}

static inline struct avd_ctx *file_to_ctx(struct file *filp)
{
	return container_of(file_to_v4l2_fh(filp), struct avd_ctx, fh);
}

/* hw stuff */
int avd_boot(struct avd_dev *avd);
void avd_shutdown(struct avd_dev *avd);

void avd_status(struct avd_dev *avd, u32 vp);

void t8103_configure_stream(struct avd_dev *avd, dma_addr_t addr, u8 fifo_idx,
			  u32 vp_slot);
void t8112_configure_stream(struct avd_dev *avd, dma_addr_t addr, u8 fifo_idx,
			  u32 vp_slot);
void t8122_configure_stream (struct avd_dev *avd, dma_addr_t addr, u8 fifo_idx,
			  u32 vp_slot);

#endif /* AVD_H_ */
