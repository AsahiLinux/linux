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

#include <linux/pm_runtime.h>

#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>

#include "avd.h"

static bool avd_image_fmt_match(enum avd_image_fmt fmt1,
				enum avd_image_fmt fmt2)
{
	return fmt1 == fmt2 || fmt2 == AVD_IMG_FMT_ANY ||
	       fmt1 == AVD_IMG_FMT_ANY;
}

static bool avd_image_fmt_changed(struct avd_ctx *ctx,
				  enum avd_image_fmt image_fmt)
{
	if (image_fmt == AVD_IMG_FMT_ANY)
		return false;

	return ctx->image_fmt != image_fmt;
}

static u32 avd_enum_decoded_fmt(struct avd_ctx *ctx, int index,
				enum avd_image_fmt image_fmt)
{
	const struct avd_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	int fmt_idx = -1;
	unsigned int i;

	if (WARN_ON(!desc))
		return 0;

	for (i = 0; i < desc->num_decoded_fmts; i++) {
		if (!avd_image_fmt_match(desc->decoded_fmts[i].image_fmt,
					 image_fmt))
			continue;
		fmt_idx++;
		if (index == fmt_idx)
			return desc->decoded_fmts[i].fourcc;
	}

	return 0;
}

static bool avd_is_valid_fmt(struct avd_ctx *ctx, u32 fourcc,
			     enum avd_image_fmt image_fmt)
{
	const struct avd_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	unsigned int i;

	for (i = 0; i < desc->num_decoded_fmts; i++) {
		if (avd_image_fmt_match(desc->decoded_fmts[i].image_fmt,
					image_fmt) &&
		    desc->decoded_fmts[i].fourcc == fourcc)
			return true;
	}

	return false;
}

static void avd_fill_decoded_pixfmt(struct avd_ctx *ctx,
				    struct v4l2_pix_format_mplane *pix_mp)
{
	v4l2_fill_pixfmt_mp(pix_mp, pix_mp->pixelformat, pix_mp->width,
			    pix_mp->height);
	fill_rvra(&ctx->rvra, ctx->image_fmt, pix_mp->width, pix_mp->height);
	pix_mp->plane_fmt[0].sizeimage += ctx->rvra.size;
	if (ctx->coded_fmt_desc->ops->adjust_decoded_fmt)
		ctx->coded_fmt_desc->ops->adjust_decoded_fmt(ctx, pix_mp);
}

static void avd_reset_fmt(struct avd_ctx *ctx, struct v4l2_format *f,
			  u32 fourcc)
{
	memset(f, 0, sizeof(*f));
	f->fmt.pix_mp.pixelformat = fourcc;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
	f->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	f->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	f->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

void avd_reset_decoded_fmt(struct avd_ctx *ctx)
{
	struct v4l2_format *f = &ctx->decoded_fmt;
	u32 fourcc;

	fourcc = avd_enum_decoded_fmt(ctx, 0, ctx->image_fmt);
	avd_reset_fmt(ctx, f, fourcc);
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f->fmt.pix_mp.width = ctx->coded_fmt.fmt.pix_mp.width;
	f->fmt.pix_mp.height = ctx->coded_fmt.fmt.pix_mp.height;
	avd_fill_decoded_pixfmt(ctx, &f->fmt.pix_mp);
}

static int avd_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct avd_ctx *ctx =
		container_of(ctrl->handler, struct avd_ctx, ctrl_hdl);
	const struct avd_coded_fmt_desc *desc = ctx->coded_fmt_desc;

	if (desc->ops->try_ctrl)
		return desc->ops->try_ctrl(ctx, ctrl);

	return 0;
}

static int avd_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct avd_ctx *ctx =
		container_of(ctrl->handler, struct avd_ctx, ctrl_hdl);
	const struct avd_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	enum avd_image_fmt image_fmt;
	struct vb2_queue *vq;

	/* Check if this change requires a capture format reset */
	if (!desc->ops->get_image_fmt)
		return 0;

	image_fmt = desc->ops->get_image_fmt(ctx, ctrl);
	if (avd_image_fmt_changed(ctx, image_fmt)) {
		vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
				     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		if (vb2_is_busy(vq))
			return -EBUSY;

		ctx->image_fmt = image_fmt;
		avd_reset_decoded_fmt(ctx);
	}

	return 0;
}

const struct v4l2_ctrl_ops avd_ctrl_ops = {
	.try_ctrl = avd_try_ctrl,
	.s_ctrl = avd_s_ctrl,
};

static const struct avd_ctrl_desc avd_hevc_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
		.cfg.flags = V4L2_CTRL_FLAG_DYNAMIC_ARRAY,
		.cfg.type = V4L2_CTRL_TYPE_HEVC_SLICE_PARAMS,
		.cfg.dims = { 600 },
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SPS,
		.cfg.ops = &avd_ctrl_ops,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_PPS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_DECODE_MODE,
		.cfg.min = V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED,
		.cfg.max = V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED,
		.cfg.def = V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_START_CODE,
		.cfg.min = V4L2_STATELESS_HEVC_START_CODE_NONE,
		.cfg.def = V4L2_STATELESS_HEVC_START_CODE_NONE,
		.cfg.max = V4L2_STATELESS_HEVC_START_CODE_NONE,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
		.cfg.max = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10,
		.cfg.def = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
		.cfg.min = V4L2_MPEG_VIDEO_HEVC_LEVEL_1,
		.cfg.max = V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1,
	},
};

static const struct avd_ctrls avd_hevc_ctrls = {
	.ctrls = avd_hevc_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(avd_hevc_ctrl_descs),
};

static const struct avd_decoded_fmt_desc avd_hevc_decoded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.image_fmt = AVD_IMG_FMT_420_8BIT,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV15,
		.image_fmt = AVD_IMG_FMT_420_10BIT,
	},
};

static const struct avd_ctrl_desc avd_h264_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_H264_DECODE_PARAMS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_SPS,
		.cfg.ops = &avd_ctrl_ops,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_PPS,
		.cfg.ops = &avd_ctrl_ops,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_SCALING_MATRIX,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_PRED_WEIGHTS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_SLICE_PARAMS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.cfg.min = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		.cfg.max = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		.cfg.def = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_START_CODE,
		.cfg.min = V4L2_STATELESS_H264_START_CODE_NONE,
		.cfg.max = V4L2_STATELESS_H264_START_CODE_NONE,
		.cfg.def = V4L2_STATELESS_H264_START_CODE_NONE,
		/* annex b is also possibly but a bit more painfull */
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE,
		.cfg.max = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA,
		.cfg.menu_skip_mask =
			BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED) |
			BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE),
		.cfg.def = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.cfg.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.cfg.max = V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
	},
};

static const struct avd_ctrls avd_h264_ctrls = {
	.ctrls = avd_h264_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(avd_h264_ctrl_descs),
};

static const struct avd_decoded_fmt_desc avd_h264_decoded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.image_fmt = AVD_IMG_FMT_420_8BIT,
	},
	{
		.fourcc = V4L2_PIX_FMT_P010,
		.image_fmt = AVD_IMG_FMT_420_10BIT,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.image_fmt = AVD_IMG_FMT_422_8BIT,
	},
	{
		/* TODO: missing P210 */
		.fourcc = V4L2_PIX_FMT_P010,
		.image_fmt = AVD_IMG_FMT_422_10BIT,
	},
};

static const struct avd_ctrl_desc avd_vp9_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_VP9_FRAME,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_VP9_COMPRESSED_HDR,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_VP9_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
		.cfg.max = V4L2_MPEG_VIDEO_VP9_PROFILE_3,
		.cfg.def = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
	},
};

static const struct avd_ctrls avd_vp9_ctrls = {
	.ctrls = avd_vp9_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(avd_vp9_ctrl_descs),
};

static const struct avd_decoded_fmt_desc avd_vp9_decoded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.image_fmt = AVD_IMG_FMT_420_8BIT,
	},
	/*
	 * TODO: it looks like gstreamer uses h264 to decide what format to use
	 * ffmpeg handles it fine
	{
		.fourcc = V4L2_PIX_FMT_P010,
		.image_fmt = AVD_IMG_FMT_420_10BIT,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.image_fmt = AVD_IMG_FMT_422_8BIT,
	},
	{
		.fourcc = V4L2_PIX_FMT_P010,
		.image_fmt = AVD_IMG_FMT_422_10BIT,
	},
	*/
};

static const struct avd_coded_fmt_desc avd_coded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_HEVC_SLICE,
		.frmsize = {
			.min_width = 64,
			.max_width = 4096,
			.step_width = 16,
			.min_height = 64,
			.max_height = 4096,
			.step_height = 16,
		},
		.ctrls = &avd_hevc_ctrls,
		.ops = &avd_hevc_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(avd_hevc_decoded_fmts),
		.subsystem_flags = VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF,
		.decoded_fmts = avd_hevc_decoded_fmts,
		.capability = AVD_CAPABILITY_HEVC,
	},
	{
		.fourcc = V4L2_PIX_FMT_H264_SLICE,
		.frmsize = {
			.min_width = 64,
			.max_width = 4096,
			.step_width = 64,
			.min_height = 64,
			.max_height = 4096,
			.step_height = 16,
		},
		.ctrls = &avd_h264_ctrls,
		.ops = &avd_h264_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(avd_h264_decoded_fmts),
		.decoded_fmts = avd_h264_decoded_fmts,
		.subsystem_flags = VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF,
		.capability = AVD_CAPABILITY_H264,
	},
	{
		.fourcc = V4L2_PIX_FMT_VP9_FRAME,
		.frmsize = {
			.min_width = 64,
			.max_width = 4096,
			.step_width = 64,
			.min_height = 64,
			.max_height = 4096,
			.step_height = 16,
		},
		.ctrls = &avd_vp9_ctrls,
		.ops = &avd_vp9_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(avd_vp9_decoded_fmts),
		.decoded_fmts = avd_vp9_decoded_fmts,
		.capability = AVD_CAPABILITY_VP9,
	}
};

static bool avd_is_capable(struct avd_ctx *ctx, unsigned int capability)
{
	return capability & (AVD_CAPABILITY_H264 | AVD_CAPABILITY_VP9);
	/* return (ctx->dev->variant->capabilities & capability) == capability; */
}

static const struct avd_coded_fmt_desc *
avd_enum_coded_fmt_desc(struct avd_ctx *ctx, int index)
{
	int fmt_idx = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(avd_coded_fmts); i++) {
		if (!avd_is_capable(ctx, avd_coded_fmts[i].capability))
			continue;
		fmt_idx++;
		if (index == fmt_idx)
			return &avd_coded_fmts[i];
	}

	return NULL;
}

static const struct avd_coded_fmt_desc *
avd_find_coded_fmt_desc(struct avd_ctx *ctx, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(avd_coded_fmts); i++) {
		if (avd_is_capable(ctx, avd_coded_fmts[i].capability) &&
		    avd_coded_fmts[i].fourcc == fourcc)
			return &avd_coded_fmts[i];
	}

	return NULL;
}

void avd_reset_coded_fmt(struct avd_ctx *ctx)
{
	struct v4l2_format *f = &ctx->coded_fmt;

	ctx->coded_fmt_desc = avd_enum_coded_fmt_desc(ctx, 0);
	avd_reset_fmt(ctx, f, ctx->coded_fmt_desc->fourcc);

	f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	f->fmt.pix_mp.width = ctx->coded_fmt_desc->frmsize.min_width;
	f->fmt.pix_mp.height = ctx->coded_fmt_desc->frmsize.min_height;

	f->fmt.pix_mp.num_planes = 1;
	if (!f->fmt.pix_mp.plane_fmt[0].sizeimage)
		f->fmt.pix_mp.plane_fmt[0].sizeimage =
			f->fmt.pix_mp.width * f->fmt.pix_mp.height;
}

static int avd_enum_framesizes(struct file *file, void *priv,
			       struct v4l2_frmsizeenum *fsize)
{
	struct avd_ctx *ctx = file_to_ctx(file);
	const struct avd_coded_fmt_desc *desc;

	if (fsize->index != 0)
		return -EINVAL;

	desc = avd_find_coded_fmt_desc(ctx, fsize->pixel_format);
	if (!desc)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = 1;
	fsize->stepwise.max_width = desc->frmsize.max_width;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = 1;
	fsize->stepwise.max_height = desc->frmsize.max_height;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int avd_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	struct avd_dev *avd = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);

	strscpy(cap->driver, avd->dev->driver->name, sizeof(cap->driver));
	strscpy(cap->card, vdev->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 avd->dev->driver->name);
	return 0;
}

static int avd_try_capture_fmt(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct avd_ctx *ctx = file_to_ctx(file);
	const struct avd_coded_fmt_desc *coded_desc;

	/*
	 * The codec context should point to a coded format desc, if the format
	 * on the coded end has not been set yet, it should point to the
	 * default value.
	 */
	coded_desc = ctx->coded_fmt_desc;
	if (WARN_ON(!coded_desc)) {
		dev_err(ctx->dev->dev, "no coded desc!");
		return -EINVAL;
	}

	if (!avd_is_valid_fmt(ctx, pix_mp->pixelformat, ctx->image_fmt))
		pix_mp->pixelformat =
			avd_enum_decoded_fmt(ctx, 0, ctx->image_fmt);

	/* Always apply the frmsize constraint of the coded end. */
	pix_mp->width = max(pix_mp->width, ctx->coded_fmt.fmt.pix_mp.width);
	pix_mp->height = max(pix_mp->height, ctx->coded_fmt.fmt.pix_mp.height);
	v4l2_apply_frmsize_constraints(&pix_mp->width, &pix_mp->height,
				       &coded_desc->frmsize);

	avd_fill_decoded_pixfmt(ctx, pix_mp);
	pix_mp->field = V4L2_FIELD_NONE;

	return 0;
}

static int avd_try_output_fmt(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct avd_ctx *ctx = file_to_ctx(file);
	const struct avd_coded_fmt_desc *desc;

	desc = avd_find_coded_fmt_desc(ctx, pix_mp->pixelformat);
	if (!desc) {
		desc = avd_enum_coded_fmt_desc(ctx, 0);
		pix_mp->pixelformat = desc->fourcc;
	}

	v4l2_apply_frmsize_constraints(&pix_mp->width, &pix_mp->height,
				       &desc->frmsize);

	pix_mp->field = V4L2_FIELD_NONE;
	/* All coded formats are considered single planar for now. */
	pix_mp->num_planes = 1;

	if (!pix_mp->plane_fmt[0].sizeimage)
		pix_mp->plane_fmt[0].sizeimage =
			pix_mp->width * f->fmt.pix_mp.height;

	return 0;
}

static int avd_s_capture_fmt(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct avd_ctx *ctx = file_to_ctx(file);
	struct vb2_queue *vq;
	int ret;

	/* Change not allowed if queue is busy */
	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
			     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = avd_try_capture_fmt(file, priv, f);
	if (ret)
		return ret;

	ctx->decoded_fmt = *f;
	return 0;
}

static int avd_s_output_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct avd_ctx *ctx = file_to_ctx(file);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	const struct avd_coded_fmt_desc *desc;
	struct v4l2_format *cap_fmt;
	struct vb2_queue *peer_vq, *vq;
	int ret;

	/*
	 * In order to support dynamic resolution change, the decoder admits
	 * a resolution change, as long as the pixelformat remains. Can't be
	 * done if streaming.
	 */
	vq = v4l2_m2m_get_vq(m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (vb2_is_streaming(vq) ||
	    (vb2_is_busy(vq) && f->fmt.pix_mp.pixelformat !=
					ctx->coded_fmt.fmt.pix_mp.pixelformat))
		return -EBUSY;

	/*
	 * Since format change on the OUTPUT queue will reset the CAPTURE
	 * queue, we can't allow doing so when the CAPTURE queue has buffers
	 * allocated.
	 */
	peer_vq = v4l2_m2m_get_vq(m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vb2_is_busy(peer_vq))
		return -EBUSY;

	ret = avd_try_output_fmt(file, priv, f);
	if (ret)
		return ret;

	desc = avd_find_coded_fmt_desc(ctx, f->fmt.pix_mp.pixelformat);
	if (!desc)
		return -EINVAL;
	ctx->coded_fmt_desc = desc;
	ctx->coded_fmt = *f;

	/*
	 * Current decoded format might have become invalid with newly
	 * selected codec, so reset it to default just to be safe and
	 * keep internal driver state sane. User is mandated to set
	 * the decoded format again after we return, so we don't need
	 * anything smarter.
	 *
	 * Note that this will propagates any size changes to the decoded format.
	 */
	ctx->image_fmt = AVD_IMG_FMT_ANY;
	avd_reset_decoded_fmt(ctx);

	/* Propagate colorspace information to capture. */
	cap_fmt = &ctx->decoded_fmt;
	cap_fmt->fmt.pix_mp.colorspace = f->fmt.pix_mp.colorspace;
	cap_fmt->fmt.pix_mp.xfer_func = f->fmt.pix_mp.xfer_func;
	cap_fmt->fmt.pix_mp.ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	cap_fmt->fmt.pix_mp.quantization = f->fmt.pix_mp.quantization;

	/* Enable format specific queue features */
	vq->subsystem_flags |= desc->subsystem_flags;

	return 0;
}

static int avd_g_output_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct avd_ctx *ctx = file_to_ctx(file);

	*f = ctx->coded_fmt;
	return 0;
}

static int avd_g_capture_fmt(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct avd_ctx *ctx = file_to_ctx(file);

	*f = ctx->decoded_fmt;
	return 0;
}

static int avd_enum_output_fmt(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	struct avd_ctx *ctx = file_to_ctx(file);
	const struct avd_coded_fmt_desc *desc;

	desc = avd_enum_coded_fmt_desc(ctx, f->index);
	if (!desc)
		return -EINVAL;

	f->pixelformat = desc->fourcc;
	return 0;
}

static int avd_enum_capture_fmt(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	struct avd_ctx *ctx = file_to_ctx(file);
	u32 fourcc;

	fourcc = avd_enum_decoded_fmt(ctx, f->index, ctx->image_fmt);
	if (!fourcc)
		return -EINVAL;

	f->pixelformat = fourcc;
	return 0;
}

const struct v4l2_ioctl_ops avd_ioctl_ops = {
	.vidioc_querycap = avd_querycap,
	.vidioc_enum_framesizes = avd_enum_framesizes,

	.vidioc_try_fmt_vid_cap_mplane = avd_try_capture_fmt,
	.vidioc_try_fmt_vid_out_mplane = avd_try_output_fmt,
	.vidioc_s_fmt_vid_out_mplane = avd_s_output_fmt,
	.vidioc_s_fmt_vid_cap_mplane = avd_s_capture_fmt,
	.vidioc_g_fmt_vid_out_mplane = avd_g_output_fmt,
	.vidioc_g_fmt_vid_cap_mplane = avd_g_capture_fmt,
	.vidioc_enum_fmt_vid_out = avd_enum_output_fmt,
	.vidioc_enum_fmt_vid_cap = avd_enum_capture_fmt,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_decoder_cmd = v4l2_m2m_ioctl_stateless_decoder_cmd,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_stateless_try_decoder_cmd,
};

static int avd_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
			   unsigned int *num_planes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct avd_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_format *f;
	unsigned int i;

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		f = &ctx->coded_fmt;
	else
		f = &ctx->decoded_fmt;

	if (*num_planes) {
		if (*num_planes != f->fmt.pix_mp.num_planes)
			return -EINVAL;

		for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
			if (sizes[i] < f->fmt.pix_mp.plane_fmt[i].sizeimage)
				return -EINVAL;
		}
	} else {
		*num_planes = f->fmt.pix_mp.num_planes;
		for (i = 0; i < f->fmt.pix_mp.num_planes; i++)
			sizes[i] = f->fmt.pix_mp.plane_fmt[i].sizeimage;
	}

	return 0;
}

static int avd_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct avd_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_format *f;
	unsigned int i;

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		f = &ctx->coded_fmt;
	else
		f = &ctx->decoded_fmt;

	for (i = 0; i < f->fmt.pix_mp.num_planes; ++i) {
		u32 sizeimage = f->fmt.pix_mp.plane_fmt[i].sizeimage;

		if (vb2_plane_size(vb, i) < sizeimage)
			return -EINVAL;
	}

	/*
	 * Buffer's bytesused must be written by driver for CAPTURE buffers.
	 * (for OUTPUT buffers, if userspace passes 0 bytesused, v4l2-core sets
	 * it to buffer length).
	 */
	if (V4L2_TYPE_IS_CAPTURE(vq->type))
		vb2_set_plane_payload(vb, 0,
				      f->fmt.pix_mp.plane_fmt[0].sizeimage);

	return 0;
}

static void avd_buf_queue(struct vb2_buffer *vb)
{
	struct avd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int avd_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static void avd_buf_request_complete(struct vb2_buffer *vb)
{
	struct avd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->ctrl_hdl);
}

static int avd_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct avd_ctx *ctx = vb2_get_drv_priv(q);
	const struct avd_coded_fmt_desc *desc;
	int ret;

	if (V4L2_TYPE_IS_CAPTURE(q->type))
		return 0;

	desc = ctx->coded_fmt_desc;
	if (WARN_ON(!desc))
		return -EINVAL;

	if (desc->ops->start) {
		ret = desc->ops->start(ctx);
		if (ret)
			return ret;
	}

	return 0;
}

static void avd_queue_cleanup(struct vb2_queue *vq, u32 state)
{
	struct avd_ctx *ctx = vb2_get_drv_priv(vq);

	while (true) {
		struct vb2_v4l2_buffer *vbuf;

		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

		if (!vbuf)
			break;

		v4l2_ctrl_request_complete(vbuf->vb2_buf.req_obj.req,
					   &ctx->ctrl_hdl);
		v4l2_m2m_buf_done(vbuf, state);
	}
}

static void avd_stop_streaming(struct vb2_queue *q)
{
	struct avd_ctx *ctx = vb2_get_drv_priv(q);

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		const struct avd_coded_fmt_desc *desc = ctx->coded_fmt_desc;

		if (WARN_ON(!desc))
			return;

		if (desc->ops->stop)
			desc->ops->stop(ctx);
	}

	avd_queue_cleanup(q, VB2_BUF_STATE_ERROR);
}

const struct vb2_ops avd_queue_ops = {
	.queue_setup = avd_queue_setup,
	.buf_prepare = avd_buf_prepare,
	.buf_queue = avd_buf_queue,
	.buf_out_validate = avd_buf_out_validate,
	.buf_request_complete = avd_buf_request_complete,
	.start_streaming = avd_start_streaming,
	.stop_streaming = avd_stop_streaming,
};

void avd_job_finish_no_pm(struct avd_ctx *ctx, enum vb2_buffer_state result)
{
	if (ctx->coded_fmt_desc->ops->done) {
		struct vb2_v4l2_buffer *src_buf, *dst_buf;

		src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
		dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
		ctx->coded_fmt_desc->ops->done(ctx, src_buf, dst_buf, result);
	}

	v4l2_m2m_buf_done_and_job_finish(ctx->dev->m2m_dev, ctx->fh.m2m_ctx,
					 result);
}

void avd_job_finish(struct avd_ctx *ctx, enum vb2_buffer_state result)
{
	struct avd_dev *avd = ctx->dev;

	pm_runtime_put_autosuspend(avd->dev);
	avd_job_finish_no_pm(ctx, result);
}

void avd_run_preamble(struct avd_ctx *ctx, struct avd_run *run)
{
	struct media_request *src_req;
	struct avd_decoded_buffer *dst;

	memset(run, 0, sizeof(*run));

	run->bufs.src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	run->bufs.dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	dst = vb2_to_avd_decoded_buf(&run->bufs.dst->vb2_buf);
	memcpy(&dst->rvra, &ctx->rvra, sizeof(ctx->rvra));

	/* Apply request(s) controls if needed. */
	src_req = run->bufs.src->vb2_buf.req_obj.req;
	if (src_req)
		v4l2_ctrl_request_setup(src_req, &ctx->ctrl_hdl);

	v4l2_m2m_buf_copy_metadata(run->bufs.src, run->bufs.dst);
}

void avd_run_postamble(struct avd_ctx *ctx, struct avd_run *run)
{
	struct media_request *src_req = run->bufs.src->vb2_buf.req_obj.req;

	if (src_req)
		v4l2_ctrl_request_complete(src_req, &ctx->ctrl_hdl);
}

static int avd_add_ctrls(struct avd_ctx *ctx, const struct avd_ctrls *ctrls)
{
	unsigned int i;

	for (i = 0; i < ctrls->num_ctrls; i++) {
		const struct v4l2_ctrl_config *cfg = &ctrls->ctrls[i].cfg;

		v4l2_ctrl_new_custom(&ctx->ctrl_hdl, cfg, ctx);
		if (ctx->ctrl_hdl.error)
			return ctx->ctrl_hdl.error;
	}

	return 0;
}

int avd_init_ctrls(struct avd_ctx *ctx)
{
	unsigned int i, nctrls = 0;
	int ret;

	for (i = 0; i < ARRAY_SIZE(avd_coded_fmts); i++)
		if (avd_is_capable(ctx, avd_coded_fmts[i].capability))
			nctrls += avd_coded_fmts[i].ctrls->num_ctrls;

	v4l2_ctrl_handler_init(&ctx->ctrl_hdl, nctrls);

	for (i = 0; i < ARRAY_SIZE(avd_coded_fmts); i++) {
		if (avd_is_capable(ctx, avd_coded_fmts[i].capability)) {
			ret = avd_add_ctrls(ctx, avd_coded_fmts[i].ctrls);
			if (ret)
				goto err_free_handler;
		}
	}

	ret = v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);
	if (ret)
		goto err_free_handler;

	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	return ret;
}
