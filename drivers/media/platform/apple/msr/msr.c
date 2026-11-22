// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on Rockchip RGA driver
 * Copyright (C) Rockchip Electronics Co., Ltd.
 * Author: Jacob Chen <jacob-chen@iotwrt.com>
 *
 * Copyright (C) 2026 The Asahi Linux Contributors
 * Copyright (C) 2026 Sofus Forstreuter <sofus.c@icloud.com>
 *
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/platform_device.h>

#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "msr.h"

int debug;
module_param(debug, int, 0644);

static void device_run(void *prv)
{
	struct msr_ctx *ctx = prv;
	struct msr_dev *msr = ctx->msr;
	struct vb2_v4l2_buffer *src, *dst;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	src->sequence = ctx->osequence++;

	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	msr->hw->start(ctx, vb_to_msr(src), vb_to_msr(dst));
}

static void end_job(struct msr_ctx *ctx, enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *src, *dst;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	WARN_ON(!src);
	WARN_ON(!dst);

	v4l2_m2m_buf_copy_metadata(src, dst);

	dst->sequence = ctx->csequence++;

	v4l2_m2m_buf_done(src, state);
	v4l2_m2m_buf_done(dst, state);
	v4l2_m2m_job_finish(ctx->msr->m2m_dev, ctx->fh.m2m_ctx);
}

static irqreturn_t msr_irq_handler(int irq, void *prv)
{
	struct msr_dev *msr = prv;
	struct msr_ctx *ctx = v4l2_m2m_get_curr_priv(msr->m2m_dev);
	enum vb2_buffer_state state;

	if (msr->hw->handle_irq(msr))
		state = VB2_BUF_STATE_DONE;
	else
		state = VB2_BUF_STATE_ERROR;
	end_job(ctx, state);

	return IRQ_HANDLED;
}

static const struct v4l2_m2m_ops msr_m2m_ops = {
	.device_run = device_run,
};

static struct msr_frame *msr_get_frame(struct msr_ctx *ctx, enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->src;
	if (V4L2_TYPE_IS_CAPTURE(type))
		return &ctx->dst;
	return ERR_PTR(-EINVAL);
}

static int
msr_queue_setup(struct vb2_queue *vq,
		unsigned int *nbuffers, unsigned int *nplanes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	struct msr_ctx *ctx = vb2_get_drv_priv(vq);
	struct msr_frame *f = msr_get_frame(ctx, vq->type);
	const struct v4l2_pix_format_mplane *pix_fmt;
	int i;

	if (IS_ERR(f))
		return PTR_ERR(f);

	pix_fmt = &f->pix;

	if (*nplanes) {
		if (*nplanes != pix_fmt->num_planes)
			return -EINVAL;

		for (i = 0; i < pix_fmt->num_planes; i++)
			if (sizes[i] < pix_fmt->plane_fmt[i].sizeimage)
				return -EINVAL;

		return 0;
	}

	*nplanes = pix_fmt->num_planes;

	for (i = 0; i < pix_fmt->num_planes; i++)
		sizes[i] = pix_fmt->plane_fmt[i].sizeimage;

	return 0;
}

static bool is_ax_format(u32 pixfmt)
{
	return pixfmt == V4L2_PIX_FMT_APPLE_AX12 ||
		pixfmt == V4L2_PIX_FMT_APPLE_AX16 ||
		pixfmt == V4L2_PIX_FMT_APPLE_AX24 ||
		pixfmt == V4L2_PIX_FMT_APPLE_AX03 ||
		pixfmt == V4L2_PIX_FMT_APPLE_AX23 ||
		pixfmt == V4L2_PIX_FMT_APPLE_AX43;
}

static inline u8 v4l2_format_info_bpp(const struct v4l2_format_info *info,
				      int plane)
{
	return 8 * info->bpp[plane] / info->bpp_div[plane];
}

static int ax_get_plane_size(const struct v4l2_format_info *info, int plane,
			     const struct v4l2_pix_format_mplane *pix,
			     bool meta)
{
	u8 bpp = v4l2_format_info_bpp(info, plane);
	u8 tile_dim = plane ? 16 : 32;
	u8 meta_hdr_bytes = plane ? 8 : 32;
	unsigned int hdiv = plane ? info->hdiv : 1;
	unsigned int vdiv = plane ? info->vdiv : 1;
	unsigned int tw = DIV_ROUND_UP(pix->height / vdiv, tile_dim);
	unsigned int th = DIV_ROUND_UP(pix->width / hdiv, tile_dim);

	if (meta) {
		unsigned int meta_tw, meta_th;

		meta_tw = roundup_pow_of_two(tw);
		meta_th = roundup_pow_of_two(th);
		return ALIGN(meta_tw * meta_th * meta_hdr_bytes, 16);
	} else {
		unsigned int tile_bytes =
			DIV_ROUND_UP(tile_dim * tile_dim * bpp, 8);

		return ALIGN(tw * th * tile_bytes, 16);
	}
}

static inline unsigned int v4l2_format_block_width(const struct v4l2_format_info *info, int plane)
{
	if (!info->block_w[plane])
		return 1;
	return info->block_w[plane];
}

static inline unsigned int v4l2_format_plane_stride(const struct v4l2_format_info *info, int plane,
						    unsigned int width, unsigned int byte_alignment)
{
	unsigned int hdiv = plane ? info->hdiv : 1;
	unsigned int aligned_width =
		ALIGN(width, v4l2_format_block_width(info, plane));

	/*
	 * Formats with a single memory plane derive the stride of the
	 * other planes from the y stride. To avoid hardware or software
	 * deriving a different stride for the composite plane,
	 * multiply the alignment accordingly.
	 *
	 * It assumes the following format properties:
	 * - bpp_div[0] == bpp_div[1]
	 * - The multiplication factor doesn't differ between the non y planes
	 * - The multiplication factor is a power of 2
	 */
	if (info->mem_planes == 1 && info->comp_planes > 1) {
		if (plane == 0)
			byte_alignment *= DIV_ROUND_UP(info->hdiv * info->bpp[0], info->bpp[1]);
		else
			byte_alignment *= DIV_ROUND_UP(info->bpp[1], info->hdiv * info->bpp[0]);
	}

	return ALIGN(DIV_ROUND_UP(aligned_width, hdiv) * info->bpp[plane] /
			     info->bpp_div[plane],
		     byte_alignment);
}

static int get_plane_offset(struct msr_frame *f,
			    const struct v4l2_format_info *info,
			    int plane)
{
	u32 stride = f->pix.plane_fmt[0].bytesperline;

	if (plane == 0)
		return 0;
	if (plane == 1)
		return stride * f->pix.height;
	if (plane == 2)
		return stride * f->pix.height +
		       (stride * f->pix.height / info->hdiv / info->vdiv);

	return -EINVAL;
}

static int msr_buf_fill_ax(struct msr_frame *f, struct msr_vb_buffer *rbuf,
			   struct vb2_buffer *vb)
{
	int i;
	const struct v4l2_format_info *info =
		v4l2_format_info(f->pix.pixelformat);

	WARN_ON_ONCE(vb->num_planes != 1 ||  info->mem_planes != 1);

	vb2_set_plane_payload(vb, 0, f->pix.plane_fmt[0].sizeimage);
	rbuf->dma_addrs[0] = vb2_dma_contig_plane_dma_addr(vb, 0);
	rbuf->dma_meta_addrs[0] = rbuf->dma_addrs[0] +
		ax_get_plane_size(info, 0, &f->pix, false);


	for (i = 1; i < info->comp_planes; i++) {
		rbuf->dma_addrs[i] = rbuf->dma_meta_addrs[i - 1] +
			ax_get_plane_size(info, i - 1, &f->pix, true);
		rbuf->dma_meta_addrs[i] = rbuf->dma_addrs[i] +
			ax_get_plane_size(info, i, &f->pix, false);
	}

	for (i = 0; i < ARRAY_SIZE(rbuf->stride); i++)
		rbuf->stride[i] = v4l2_format_plane_stride(info, i, f->pix.width, 2);

	return 0;
}

static int msr_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct msr_vb_buffer *rbuf = vb_to_msr(vbuf);
	struct msr_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct msr_frame *f = msr_get_frame(ctx, vb->vb2_queue->type);
	int i;
	const struct v4l2_format_info *info;

	if (IS_ERR(f))
		return PTR_ERR(f);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vbuf->field == V4L2_FIELD_ANY)
			vbuf->field = V4L2_FIELD_NONE;
		if (vbuf->field != V4L2_FIELD_NONE)
			return -EINVAL;
	}

	if (WARN_ON_ONCE(vb->num_planes > ARRAY_SIZE(rbuf->dma_addrs)))
		return -EINVAL;

	if (f->comp)
		return msr_buf_fill_ax(f, rbuf, vb);

	for (i = 0; i < vb->num_planes; i++) {
		vb2_set_plane_payload(vb, i, f->pix.plane_fmt[i].sizeimage);
		rbuf->dma_addrs[i] = vb2_dma_contig_plane_dma_addr(vb, i);
	}

	/* Fill the remaining planes */
	info = v4l2_format_info(f->pix.pixelformat);
	for (i = info->mem_planes; i < info->comp_planes; i++)
		rbuf->dma_addrs[i] = rbuf->dma_addrs[0] + get_plane_offset(f, info, i);

	for (i = 0; i < ARRAY_SIZE(rbuf->stride); i++)
		rbuf->stride[i] = v4l2_format_plane_stride(info, i, f->pix.width, 2);

	for (i = 0; i < ARRAY_SIZE(rbuf->dma_meta_addrs); i++)
		rbuf->dma_meta_addrs[i] = 0;

	return 0;
}

static void msr_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct msr_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static void msr_buf_return_buffers(struct vb2_queue *q,
				   enum vb2_buffer_state state)
{
	struct msr_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			break;
		v4l2_m2m_buf_done(vbuf, state);
	}
}

static int msr_buf_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct msr_ctx *ctx = vb2_get_drv_priv(q);
	struct msr_dev *msr = ctx->msr;
	int ret;

	ret = pm_runtime_resume_and_get(msr->dev);
	if (ret < 0) {
		msr_buf_return_buffers(q, VB2_BUF_STATE_QUEUED);
		return ret;
	}

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		ctx->osequence = 0;
	else
		ctx->csequence = 0;

	return 0;
}

static void msr_buf_stop_streaming(struct vb2_queue *q)
{
	struct msr_ctx *ctx = vb2_get_drv_priv(q);
	struct msr_dev *msr = ctx->msr;

	msr_buf_return_buffers(q, VB2_BUF_STATE_ERROR);
	pm_runtime_put(msr->dev);
}

static const struct vb2_ops msr_qops = {
	.queue_setup = msr_queue_setup,
	.buf_prepare = msr_buf_prepare,
	.buf_queue = msr_buf_queue,
	.start_streaming = msr_buf_start_streaming,
	.stop_streaming = msr_buf_stop_streaming,
};

static int
queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct msr_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &msr_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct msr_vb_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->msr->mutex;
	src_vq->dev = ctx->msr->v4l2_dev.dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &msr_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct msr_vb_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->msr->mutex;
	dst_vq->dev = ctx->msr->v4l2_dev.dev;

	return vb2_queue_init(dst_vq);
}

static int msr_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct msr_ctx *ctx = container_of(ctrl->handler, struct msr_ctx,
					   ctrl_handler);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&ctx->msr->ctrl_lock, flags);
	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		ctx->hflip = ctrl->val;
		break;
	case V4L2_CID_VFLIP:
		ctx->vflip = ctrl->val;
		break;
	case V4L2_CID_ROTATE:
		ctx->rotate = ctrl->val;
		break;
	case V4L2_CID_BG_COLOR:
		ctx->fill_color = ctrl->val;
		break;
	}
	spin_unlock_irqrestore(&ctx->msr->ctrl_lock, flags);
	return ret;
}

static const struct v4l2_ctrl_ops msr_ctrl_ops = {
	.s_ctrl = msr_s_ctrl,
};

static int msr_setup_ctrls(struct msr_ctx *ctx)
{
	struct msr_dev *msr = ctx->msr;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 4);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &msr_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &msr_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &msr_ctrl_ops,
			  V4L2_CID_ROTATE, 0, 270, 90, 0);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &msr_ctrl_ops,
			  V4L2_CID_BG_COLOR, 0, 0xffffffff, 1, 0);

	if (ctx->ctrl_handler.error) {
		int err = ctx->ctrl_handler.error;

		v4l2_err(&msr->v4l2_dev, "%s failed\n", __func__);
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		return err;
	}

	return 0;
}

static void msr_fill_frame(struct msr_ctx *ctx, struct msr_frame *f)
{
	struct msr_dev *msr = ctx->msr;

	f->fmt = msr->hw->map_format(ctx, &f->pix);
	f->info = v4l2_format_info(f->pix.pixelformat);
	f->yuv = v4l2_is_format_yuv(f->info);
	f->comp = is_ax_format(f->pix.pixelformat);
}

static void msr_fill_pixfmt(struct msr_ctx *ctx,
			    struct v4l2_pix_format_mplane *pix_fmt)
{
	const struct msr_hw *hw = ctx->msr->hw;
	const struct v4l2_format_info *info =
		v4l2_format_info(pix_fmt->pixelformat);
	struct v4l2_frmsize_stepwise frmsize = {
		.min_width = hw->min_width,
		.max_width = hw->max_width,
		.min_height = hw->min_height,
		.max_height = hw->max_height,
		.step_width = 1,
		.step_height = 1,
	};

	if (v4l2_is_format_yuv(info)) {
		frmsize.step_width = 2;
		frmsize.step_height = 2;
	}

	v4l2_apply_frmsize_constraints(&pix_fmt->width, &pix_fmt->height, &frmsize);
	v4l2_fill_pixfmt_mp_aligned(pix_fmt, pix_fmt->pixelformat,
				    pix_fmt->width, pix_fmt->height, hw->stride_alignment);
	pix_fmt->field = V4L2_FIELD_NONE;

	if (is_ax_format(pix_fmt->pixelformat)) {
		WARN_ON_ONCE(info->mem_planes != 1);
		pix_fmt->plane_fmt[0].sizeimage = 0;
		for (int i = 0; i < info->comp_planes; i++) {
			pix_fmt->plane_fmt[0].sizeimage +=
				ax_get_plane_size(info, i, pix_fmt, false);
			pix_fmt->plane_fmt[0].sizeimage +=
				ax_get_plane_size(info, i, pix_fmt, true);
		}
	}
}

static int msr_open(struct file *file)
{
	struct msr_dev *msr = video_drvdata(file);
	struct msr_ctx *ctx = NULL;
	int ret = 0;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;

	ctx->msr = msr;
	/* Set default formats */
	ctx->src = (struct msr_frame){ .type = MSR_SRC, };
	ctx->dst = (struct msr_frame){ .type = MSR_DST, };

	ctx->src.fmt = msr->hw->map_format(ctx, &ctx->src.pix);
	v4l2_fill_pixfmt_mp_aligned(&ctx->src.pix, ctx->src.pix.pixelformat,
				    128, 128, msr->hw->stride_alignment);
	ctx->dst.fmt =
		msr->hw->map_format(ctx, &ctx->dst.pix);
	v4l2_fill_pixfmt_mp_aligned(&ctx->dst.pix, ctx->dst.pix.pixelformat,
				    128, 128, msr->hw->stride_alignment);

	msr_fill_frame(ctx, &ctx->src);
	msr_fill_frame(ctx, &ctx->dst);

	if (mutex_lock_interruptible(&msr->mutex)) {
		ret = -ERESTARTSYS;
		goto rel_cmdbuf;
	}
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(msr->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto unlock_mutex;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	v4l2_fh_add(&ctx->fh, file);

	msr_setup_ctrls(ctx);

	/* Write the default values to the ctx struct */
	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);

	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	mutex_unlock(&msr->mutex);

	return 0;

unlock_mutex:
	mutex_unlock(&msr->mutex);
rel_cmdbuf:
	kfree(ctx);
	return ret;
}

static int msr_release(struct file *file)
{
	struct msr_ctx *ctx = file_to_msr_ctx(file);
	struct msr_dev *msr = ctx->msr;

	mutex_lock(&msr->mutex);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);

	kfree(ctx);

	mutex_unlock(&msr->mutex);

	return 0;
}

static const struct v4l2_file_operations msr_fops = {
	.owner = THIS_MODULE,
	.open = msr_open,
	.release = msr_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int
vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "apple-msr", sizeof(cap->driver));
	strscpy(cap->card, "apple-msr", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:msr", sizeof(cap->bus_info));

	return 0;
}

static int vidioc_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	struct msr_dev *msr = video_drvdata(file);
	int ret;

	ret = msr->hw->enum_format(f);
	if (ret != 0)
		return ret;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return 0;

	/* allow changing the quantization and ycbcr_enc for YUV formats */
	if (v4l2_is_format_yuv(v4l2_format_info(f->pixelformat)))
		f->flags |= V4L2_FMT_FLAG_CSC_QUANTIZATION |
			    V4L2_FMT_FLAG_CSC_YCBCR_ENC;

	f->flags |= V4L2_FMT_FLAG_CSC_XFER_FUNC | V4L2_FMT_FLAG_CSC_COLORSPACE;

	return 0;
}

static int vidioc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_fmt = &f->fmt.pix_mp;
	struct msr_ctx *ctx = file_to_msr_ctx(file);
	struct msr_frame *frm;

	frm = msr_get_frame(ctx, f->type);
	if (IS_ERR(frm))
		return PTR_ERR(frm);

	*pix_fmt = frm->pix;
	pix_fmt->field = V4L2_FIELD_NONE;

	return 0;
}

static int vidioc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_fmt = &f->fmt.pix_mp;
	struct msr_ctx *ctx = file_to_msr_ctx(file);

	if (V4L2_TYPE_IS_CAPTURE(f->type)) {
		const struct msr_frame *frm;

		frm = msr_get_frame(ctx, f->type);
		if (IS_ERR(frm))
			return PTR_ERR(frm);

		if (!(pix_fmt->flags & V4L2_PIX_FMT_FLAG_SET_CSC)) {
			pix_fmt->quantization = frm->pix.quantization;
			pix_fmt->ycbcr_enc = frm->pix.ycbcr_enc;

			pix_fmt->colorspace = frm->pix.colorspace;
			pix_fmt->xfer_func = frm->pix.xfer_func;
		}
	}

	msr_fill_pixfmt(ctx, pix_fmt);

	return 0;
}

static int vidioc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_fmt = &f->fmt.pix_mp;
	struct msr_ctx *ctx = file_to_msr_ctx(file);
	struct msr_dev *msr = ctx->msr;
	struct vb2_queue *vq;
	struct msr_frame *frm;
	int ret = 0;
	int i;

	/* Adjust all values accordingly to the hardware capabilities
	 * and chosen format.
	 */
	ret = vidioc_try_fmt(file, priv, f);
	if (ret)
		return ret;
	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq)) {
		v4l2_err(&msr->v4l2_dev, "queue (%d) bust\n", f->type);
		return -EBUSY;
	}
	frm = msr_get_frame(ctx, f->type);
	if (IS_ERR(frm))
		return PTR_ERR(frm);

	/*
	 * Copy colorimetry from output to capture as required by the
	 * v4l2-compliance tests
	 */
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ctx->dst.pix.colorspace = pix_fmt->colorspace;
		ctx->dst.pix.ycbcr_enc = pix_fmt->ycbcr_enc;
		ctx->dst.pix.quantization = pix_fmt->quantization;
		ctx->dst.pix.xfer_func = pix_fmt->xfer_func;
	}

	frm->pix = *pix_fmt;
	msr_fill_frame(ctx, frm);

	msr_dbg(1, msr, "%s: fmt - %p4cc %dx%d (stride %d)\n",
		FRM_STR(frm), &pix_fmt->pixelformat, pix_fmt->width,
		pix_fmt->height, pix_fmt->plane_fmt[0].bytesperline);

	for (i = 0; i < pix_fmt->num_planes; i++) {
		msr_dbg(2, msr, "plane[%d]: size %d, bytesperline %d\n",
			i, pix_fmt->plane_fmt[i].sizeimage,
			pix_fmt->plane_fmt[i].bytesperline);
	}

	return 0;
}

static int vidioc_g_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct msr_ctx *ctx = file_to_msr_ctx(file);
	struct msr_frame *f;
	bool use_frame = false;

	f = msr_get_frame(ctx, s->type);
	if (IS_ERR(f))
		return PTR_ERR(f);

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		if (!V4L2_TYPE_IS_CAPTURE(s->type))
			return -EINVAL;
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		if (!V4L2_TYPE_IS_OUTPUT(s->type))
			return -EINVAL;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		if (!V4L2_TYPE_IS_CAPTURE(s->type))
			return -EINVAL;
		use_frame = true;
		break;
	case V4L2_SEL_TGT_CROP:
		if (!V4L2_TYPE_IS_OUTPUT(s->type))
			return -EINVAL;
		use_frame = true;
		break;
	default:
		return -EINVAL;
	}

	if (use_frame) {
		s->r = f->crop;
	} else {
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = f->pix.width;
		s->r.height = f->pix.height;
	}

	return 0;
}

static int vidioc_s_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct msr_ctx *ctx = file_to_msr_ctx(file);
	struct msr_dev *msr = ctx->msr;
	struct msr_frame *f;

	f = msr_get_frame(ctx, s->type);
	if (IS_ERR(f))
		return PTR_ERR(f);

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
		/*
		 * COMPOSE target is only valid for capture buffer type, return
		 * error for output buffer type
		 */
		if (!V4L2_TYPE_IS_CAPTURE(s->type))
			return -EINVAL;
		break;
	case V4L2_SEL_TGT_CROP:
		/*
		 * CROP target is only valid for output buffer type, return
		 * error for capture buffer type
		 */
		if (!V4L2_TYPE_IS_OUTPUT(s->type))
			return -EINVAL;
		break;
	/*
	 * bound and default crop/compose targets are invalid targets to
	 * try/set
	 */
	default:
		return -EINVAL;
	}

	if (s->r.top < 0 || s->r.left < 0) {
		v4l2_dbg(debug, 1, &msr->v4l2_dev,
			 "doesn't support negative values for top & left.\n");
		return -EINVAL;
	}

	if (s->r.left + s->r.width > f->pix.width ||
	    s->r.top + s->r.height > f->pix.height ||
	    s->r.width < msr->hw->min_width || s->r.height < msr->hw->min_height) {
		v4l2_dbg(debug, 1, &msr->v4l2_dev, "unsupported crop value.\n");
		return -EINVAL;
	}

	f->crop = s->r;

	return 0;
}

static const struct v4l2_ioctl_ops msr_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt,

	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt,
	.vidioc_g_fmt_vid_out_mplane = vidioc_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt,
	.vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt,

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

	.vidioc_g_selection = vidioc_g_selection,
	.vidioc_s_selection = vidioc_s_selection,
};

static const struct video_device msr_videodev = {
	.name = "apple-msr",
	.fops = &msr_fops,
	.ioctl_ops = &msr_ioctl_ops,
	.minor = -1,
	.release = video_device_release,
	.vfl_dir = VFL_DIR_M2M,
	.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING,
};

/*
 * The kernel is currently missing support for multi-core handling. Exposing
 * separate devices for each core to userspace is bad, since that does not
 * allow scheduling tasks properly (and creates ABI). With this workaround the
 * driver will only probe for the first core and early exit for the other
 * cores. Once the driver gains multi-core support, the same technique for
 * detecting the main core can be used to cluster all cores together.
 */
static int msr_disable_multicore(struct device *dev)
{
	struct device_node *node = NULL;
	const char *compatible;
	bool is_main_core;
	int ret;

	/* Intentionally ignores the fallback strings */
	ret = of_property_read_string(dev->of_node, "compatible", &compatible);
	if (ret)
		return ret;

	/* The first compatible and available node found is considered the main core */
	do {
		node = of_find_compatible_node(node, NULL, compatible);
		if (of_device_is_available(node))
			break;
	} while (node);

	if (!node)
		return -EINVAL;

	is_main_core = (dev->of_node == node);

	of_node_put(node);

	if (!is_main_core) {
		dev_info(dev, "missing multi-core support, ignoring this instance\n");
		return -ENODEV;
	}

	return 0;
}

static int msr_probe(struct platform_device *pdev)
{
	struct msr_dev *msr;
	struct video_device *vfd;
	int ret = 0;
	int irq;

	if (!pdev->dev.of_node)
		return -ENODEV;

	ret = msr_disable_multicore(&pdev->dev);
	if (ret)
		return ret;

	msr = devm_kzalloc(&pdev->dev, sizeof(*msr), GFP_KERNEL);
	if (!msr)
		return -ENOMEM;

	msr->hw = &msr_hw;
	msr->dev = &pdev->dev;
	spin_lock_init(&msr->ctrl_lock);
	mutex_init(&msr->mutex);

	pm_runtime_enable(msr->dev);

	msr->base = devm_platform_ioremap_resource_byname(pdev, "base");
	if (IS_ERR(msr->base)) {
		ret = PTR_ERR(msr->base);
		goto err_put_clk;
	}

	irq = platform_get_irq_byname(pdev, "base");
	if (irq < 0) {
		ret = irq;
		goto err_put_clk;
	}

	ret = devm_request_irq(msr->dev, irq, msr_irq_handler, IRQF_SHARED,
			       dev_name(msr->dev), msr);
	if (ret < 0) {
		dev_err(msr->dev, "failed to request irq\n");
		goto err_put_clk;
	}

	ret = dma_set_mask_and_coherent(msr->dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(msr->dev, "64-bit DMA not supported");
		goto err_put_clk;
	}

	ret = v4l2_device_register(&pdev->dev, &msr->v4l2_dev);
	if (ret)
		goto err_put_clk;
	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&msr->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto unreg_v4l2_dev;
	}
	*vfd = msr_videodev;
	vfd->lock = &msr->mutex;
	vfd->v4l2_dev = &msr->v4l2_dev;

	video_set_drvdata(vfd, msr);
	msr->vfd = vfd;

	platform_set_drvdata(pdev, msr);
	msr->m2m_dev = v4l2_m2m_init(&msr_m2m_ops);
	if (IS_ERR(msr->m2m_dev)) {
		v4l2_err(&msr->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(msr->m2m_dev);
		goto rel_vdev;
	}

	ret = pm_runtime_resume_and_get(msr->dev);
	if (ret < 0)
		goto rel_m2m;

	msr->hw->get_version(msr);

	v4l2_info(&msr->v4l2_dev, "HW Version: %x\n", msr->version);

	pm_runtime_put(msr->dev);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(&msr->v4l2_dev, "Failed to register video device\n");
		goto rel_m2m;
	}

	v4l2_info(&msr->v4l2_dev, "Registered %s as /dev/%s\n",
		  vfd->name, video_device_node_name(vfd));
	msr_dbg(1, msr, "Debug: %#x\n", debug);

	return 0;

rel_m2m:
	v4l2_m2m_release(msr->m2m_dev);
rel_vdev:
	video_device_release(vfd);
unreg_v4l2_dev:
	v4l2_device_unregister(&msr->v4l2_dev);
err_put_clk:
	pm_runtime_disable(msr->dev);

	return ret;
}

static void msr_remove(struct platform_device *pdev)
{
	struct msr_dev *msr = platform_get_drvdata(pdev);

	v4l2_info(&msr->v4l2_dev, "Removing\n");

	v4l2_m2m_release(msr->m2m_dev);
	video_unregister_device(msr->vfd);
	v4l2_device_unregister(&msr->v4l2_dev);

	pm_runtime_disable(msr->dev);
}

static int __maybe_unused msr_runtime_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused msr_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops msr_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(msr_runtime_suspend,
			   msr_runtime_resume, NULL)
};

static const struct of_device_id msr_dev_match[] = {
	// { .compatible = "apple,t8103-msr", },
	{ .compatible = "apple,t8112-msr", },
	{ .compatible = "apple,t8122-msr", },
	{ .compatible = "apple,t6000-msr", },
	{ .compatible = "apple,t6020-msr", },
	{ .compatible = "apple,t6030-msr", },
	{ .compatible = "apple,t6031-msr", },
	{},
};

MODULE_DEVICE_TABLE(of, msr_dev_match);

static struct platform_driver msr_pdrv = {
	.probe = msr_probe,
	.remove = msr_remove,
	.driver = {
		.name = "apple-msr",
		.pm = pm_ptr(&msr_pm),
		.of_match_table = msr_dev_match,
	},
};

module_platform_driver(msr_pdrv);

MODULE_DESCRIPTION("Apple MSR driver");
MODULE_LICENSE("GPL");
