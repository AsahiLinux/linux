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
#include <linux/iommu.h>
#include <linux/reset.h>

#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "avd.h"
#include "avd-regs.h"

void fill_rvra(struct avd_rvra *rvra, enum avd_image_fmt image_fmt,
		u32 width, u32 height)
{
	u32 size0, size1, size2;
	u32 hs = round_up(height, 32);

	size0 = (width * hs) + ((width * hs) / 4);
	size2 = size0;

	switch (image_fmt) {
	case AVD_IMG_FMT_420_8BIT:
	case AVD_IMG_FMT_420_10BIT:
		size2 /= 2;
		break;
	default:
		break;
	}

	size1 = max((roundup_pow_of_two(width) * roundup_pow_of_two(height)) / 32,
			 0x100u);
	/* TODO: how big? */

	rvra->size = round_up(size0 + size1 + size2, 0x4000);
	rvra->size +=
		(width < 1000 ? 0 : width < 1800 ? 2 : width < 3800 ? 3 : 9) * 0x4000;

	rvra->offsets[1] = 0;
	rvra->offsets[0] = size0;
	rvra->offsets[3] = size0 + size1;
	rvra->offsets[2] = size0 + size1 + size2;
}


int alloc_slots(struct avd_dev *avd, struct avd_ctx *ctx, enum avd_codec codec) {
	u32 free;
	u32 offset = 0;
	for (int i = 0; i < codec; i++)
		offset += avd->variant->vp_slots[i];

	free = find_next_zero_bit(&avd->vp_slots,
			offset + avd->variant->vp_slots[codec],
			offset);

	if (free >= offset + avd->variant->vp_slots[codec])
		return -ENOMEM;

	set_bit(free, &avd->vp_slots);
	ctx->vp_slot = free;

	ctx->fifo_idx = find_first_zero_bit(&avd->inst_fifo_slots,
			avd->variant->fifo_slots);

	if (WARN_ON(ctx->fifo_idx >= avd->variant->fifo_slots)) {
		clear_bit(free, &avd->vp_slots);
		return -ENOMEM;
	}
	set_bit(ctx->fifo_idx,  &avd->inst_fifo_slots);

	return 0;
}

int avd_buf_alloc(struct avd_dev *avd, struct avd_buf *buf, size_t size)
{
	buf->size = size;
	buf->cpu =
		dma_alloc_coherent(avd->dev, buf->size, &buf->addr, GFP_KERNEL);
	return buf->cpu ? 0 : -ENOMEM;
}

void avd_buf_free(struct avd_dev *avd, struct avd_buf *buf)
{
	if (buf->cpu)
		dma_free_coherent(avd->dev, buf->size, buf->cpu, buf->addr);
	memset(buf, 0, sizeof(*buf));
}

struct avd_decoded_buffer *
get_ref_buf(struct avd_ctx *ctx, struct vb2_v4l2_buffer *dst, u64 timestamp)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_queue *cap_q = &m2m_ctx->cap_q_ctx.q;
	struct vb2_buffer *buf;

	/*
	 * If a ref is unused or invalid, address of current destination
	 * buffer is returned.
	 */
	buf = vb2_find_buffer(cap_q, timestamp);
	if (!buf)
		buf = &dst->vb2_buf;

	return vb2_to_avd_decoded_buf(buf);
}


static int avd_reset(struct avd_dev *avd)
{
	int ret = 0;
	avd->vp_slots = 0;
	avd->inst_fifo_slots = 0;

	ret = pm_runtime_resume_and_get(avd->dev);
	if (ret < 0)
		return ret;

	ret = reset_control_reset(avd->rstc);
	if (ret)
		dev_err(avd->dev, "reset: failed: %d", ret);

	if (avd->empty_domain) {
		iommu_attach_device(avd->empty_domain, avd->dev);
		iommu_detach_device(avd->empty_domain, avd->dev);
	}

	ret = avd_boot(avd);
	if (ret)
		dev_err(avd->dev, "reset: failed to boot");

	pm_runtime_put_autosuspend(avd->dev);

	return ret;
}

static void avd_watchdog_func(struct work_struct *work)
{
	struct avd_dev *avd;
	struct avd_ctx *ctx;
	int ret;
	ctx = container_of(to_delayed_work(work), struct avd_ctx,
			   watchdog_work);
	if (!ctx)
		return;

	avd = ctx->dev;

	dev_err(avd->dev, "Frame processing timed out! Vp: %d (%02d)",
		ctx->vp_slot, ctx->fifo_idx);

	free_vp_slot(avd, ctx);
	free_inst_slot(avd, ctx);

	writel(0, avd->mbox + AVD_REG_MBOX_IRQ_ENABLE);
	ret = avd_reset(avd);
	if (ret)
		dev_err(avd->dev, "failed to reset: %d", ret);

	avd_job_finish(ctx, VB2_BUF_STATE_ERROR);
}

static irqreturn_t avd_irq_handler(int irq, void *data)
{
	struct avd_dev *avd = data;
	struct avd_ctx *ctx = v4l2_m2m_get_curr_priv(avd->m2m_dev);

	enum vb2_buffer_state state;
	u32 status;
	if (!ctx)
		return IRQ_HANDLED;

	status = readl(avd->mbox + AVD_REG_MBOX1_RETRIEVE);

	writel(AVD_MBOX1_NOT_EMPTY, avd->mbox + AVD_REG_MBOX_IRQ_CLR);

	if (status & 0x10000) { /* dbg */
		dev_warn(avd->dev, "no handler for IRQ: %3d", status &~0x10000);
		writel_relaxed(0, avd->mbox + AVD_REG_MBOX_IRQ_ENABLE);
		return IRQ_HANDLED;
	}

	if (status & 0x1000) {
		/* pp is done ! we are done */
		state = VB2_BUF_STATE_DONE;

		free_inst_slot(avd, ctx);
	} else if (status & 0x100) {
		free_vp_slot(avd, ctx);
		/* a vp is done, kick the pp and hope for the best */
		if(ctx->coded_fmt_desc->ops->submit)
			ctx->coded_fmt_desc->ops->submit(ctx);

		goto done;
	} else {
		dev_err(avd->dev, "H%d %02d error", status, ctx->fifo_idx);
		/* let watchdog handle */
		goto done;
	}

	/* if the watchdog_work has run the work has already been submitted */
	if (cancel_delayed_work(&ctx->watchdog_work))
		avd_job_finish(ctx, state);

done:
	return IRQ_HANDLED;
}

static void avd_device_run(void *priv)
{
	struct avd_ctx *ctx = priv;
	struct avd_dev *avd = ctx->dev;
	const struct avd_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	int ret;

	if (WARN_ON(!desc))
		return;

	ret = pm_runtime_resume_and_get(avd->dev);
	if (ret < 0) {
		avd_job_finish_no_pm(ctx, VB2_BUF_STATE_ERROR);
		return;
	}

	ret = desc->ops->run(ctx);
	if (ret)
		avd_job_finish(ctx, VB2_BUF_STATE_ERROR);
}

static int avd_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct avd_ctx *ctx = priv;
	int ret;

	/* TODO: what flags to give to dart */
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &avd_queue_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;

	src_vq->dma_attrs = 0 /* DMA_ATTR_NO_KERNEL_MAPPING */;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->dev->vdev_lock;
	src_vq->dev = ctx->dev->v4l2_dev.dev;
	src_vq->supports_requests = true;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->bidirectional = true;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->dma_attrs = DMA_ATTR_NO_KERNEL_MAPPING;
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &avd_queue_ops;
	dst_vq->buf_struct_size = sizeof(struct avd_decoded_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->dev->vdev_lock;
	dst_vq->dev = ctx->dev->v4l2_dev.dev;

	return vb2_queue_init(dst_vq);
}

static int avd_open(struct file *filp)
{
	struct avd_dev *avd = video_drvdata(filp);
	struct avd_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = avd;

	INIT_DELAYED_WORK(&ctx->watchdog_work, avd_watchdog_func);

	avd_reset_coded_fmt(ctx);
	avd_reset_decoded_fmt(ctx);

	v4l2_fh_init(&ctx->fh, video_devdata(filp));

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(avd->m2m_dev, ctx, avd_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free_ctx;
	}

	ret = avd_init_ctrls(ctx);
	if (ret)
		goto err_cleanup_m2m_ctx;

	v4l2_fh_add(&ctx->fh, filp);

	return 0;

err_cleanup_m2m_ctx:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

err_free_ctx:
	kfree(ctx);
	return ret;
}

static int avd_release(struct file *filp)
{
	struct avd_ctx *ctx = file_to_ctx(filp);

	v4l2_fh_del(&ctx->fh, filp);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations avd_fops = {
	.owner = THIS_MODULE,
	.open = avd_open,
	.release = avd_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_m2m_ops avd_m2m_ops = {
	.device_run = avd_device_run,
};

static const struct media_device_ops avd_media_ops = {
	.req_validate = vb2_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

static int avd_v4l2_init(struct avd_dev *avd)
{
	int ret;

	ret = v4l2_device_register(avd->dev, &avd->v4l2_dev);
	if (ret) {
		dev_err(avd->dev, "Failed to register V4L2 device\n");
		return ret;
	}

	avd->m2m_dev = v4l2_m2m_init(&avd_m2m_ops);
	if (IS_ERR(avd->m2m_dev)) {
		v4l2_err(&avd->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(avd->m2m_dev);
		goto err_unregister_v4l2;
	}

	avd->mdev.dev = avd->dev;
	strscpy(avd->mdev.model, "avd", sizeof(avd->mdev.model));
	strscpy(avd->mdev.bus_info, "platform:avd", sizeof(avd->mdev.bus_info));
	media_device_init(&avd->mdev);
	avd->mdev.ops = &avd_media_ops;
	avd->v4l2_dev.mdev = &avd->mdev;

	avd->vdev.lock = &avd->vdev_lock;
	avd->vdev.v4l2_dev = &avd->v4l2_dev;
	avd->vdev.fops = &avd_fops;
	avd->vdev.release = video_device_release_empty;
	avd->vdev.vfl_dir = VFL_DIR_M2M;
	avd->vdev.device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	avd->vdev.ioctl_ops = &avd_ioctl_ops;
	video_set_drvdata(&avd->vdev, avd);
	strscpy(avd->vdev.name, "avd", sizeof(avd->vdev.name));

	ret = video_register_device(&avd->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(&avd->v4l2_dev, "Failed to register video device\n");
		goto err_cleanup_mc;
	}

	ret = v4l2_m2m_register_media_controller(
		avd->m2m_dev, &avd->vdev, MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret) {
		v4l2_err(&avd->v4l2_dev,
			 "Failed to initialize V4L2 M2M media controller\n");
		goto err_unregister_vdev;
	}

	ret = media_device_register(&avd->mdev);
	if (ret) {
		v4l2_err(&avd->v4l2_dev, "Failed to register media device\n");
		goto err_unregister_mc;
	}

	return 0;

err_unregister_mc:
	v4l2_m2m_unregister_media_controller(avd->m2m_dev);

err_unregister_vdev:
	video_unregister_device(&avd->vdev);

err_cleanup_mc:
	media_device_cleanup(&avd->mdev);
	v4l2_m2m_release(avd->m2m_dev);

err_unregister_v4l2:
	v4l2_device_unregister(&avd->v4l2_dev);
	return ret;
}

static void avd_v4l2_cleanup(struct avd_dev *avd)
{
	media_device_unregister(&avd->mdev);
	v4l2_m2m_unregister_media_controller(avd->m2m_dev);
	video_unregister_device(&avd->vdev);
	media_device_cleanup(&avd->mdev);
	v4l2_m2m_release(avd->m2m_dev);
	v4l2_device_unregister(&avd->v4l2_dev);
}

static const struct avd_variant avd_t8103_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 2, /* no sure */
		[AVD_CODEC_H264] = 1,
		[AVD_CODEC_VP9] = 1,
	},
	.fifo_slots = 7,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9,
	.configure_stream = t8103_configure_stream,
	.fw_name = "apple/avd-fw-v2-t0.bin",
	.revision = 3,
	.quirks = AVD_QUIRK_LSR | AVD_QUIRK_NO_PIPE_STATE,
	.vp_slot_offset = 0x4004,
	.submit_offset = 0x4014,
};

static const struct avd_variant avd_t6000_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
	},
	.fifo_slots = 15,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9,
	.configure_stream = t8112_configure_stream,
	.fw_name = "apple/avd-fw-v3-t0.bin",
	.revision = 4,
	.quirks = AVD_QUIRK_LSR | AVD_QUIRK_NO_PIPE_STATE,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x30,
};

static const struct avd_variant avd_t8112_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
	},
	.fifo_slots = 15,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9,
	.configure_stream = t8112_configure_stream,
	.fw_name = "apple/avd-fw-v3-t1.bin",
	.revision = 4,
	.quirks = AVD_QUIRK_LSR,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x30,
};

static const struct avd_variant avd_t6020_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
	},
	.fifo_slots = 15,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9,
	.configure_stream = t8112_configure_stream,
	.fw_name = "apple/avd-fw-v3-t1.bin",
	.revision = 4,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x30,
};

static const struct avd_variant avd_t8122_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
		[AVD_CODEC_AV1] = 2,
	},
	.fifo_slots = 15,
	.configure_stream = t8122_configure_stream,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9 |
			AVD_CODEC_AV1,
	.fw_name = "apple/avd-fw-v4-t0.bin",
	.revision = 4,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x40,
};

static const struct avd_variant avd_t8140_variant = {
	/* This is filled in using the tunables, what vp/pp to use? */
	.vp_slots = {
		[AVD_CODEC_HEVC] = 2,
		[AVD_CODEC_H264] = 1,
		[AVD_CODEC_VP9] = 1,
		[AVD_CODEC_AV1] = 1,
	},
	.fifo_slots = 7,
	.configure_stream = t8122_configure_stream,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9 |
			AVD_CODEC_AV1,
	.fw_name = "apple/avd-fw-v5-t0.bin",
	.revision = 4,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x40,
};

static const struct avd_variant avd_t8132_variant = {
	.vp_slots = {
		[AVD_CODEC_HEVC] = 4,
		[AVD_CODEC_H264] = 4,
		[AVD_CODEC_VP9] = 1,
		[AVD_CODEC_AV1] = 3,
	},
	.fifo_slots = 15,
	.configure_stream = t8122_configure_stream,
	.capabilities = AVD_CAPABILITY_HEVC |
			AVD_CAPABILITY_H264 |
			AVD_CAPABILITY_VP9 |
			AVD_CODEC_AV1,
	.fw_name = "apple/avd-fw-v5-t1.bin",
	.revision = 4,
	.vp_slot_offset = 0xc,
	.submit_offset = 0x40,
};

/* can also be derived from a version register */
static const struct of_device_id avd_of_match[] = {
	{
		.compatible = "apple,t8103-avd",
		.data = &avd_t8103_variant
	},
	{
		.compatible = "apple,t6000-avd",
		.data = &avd_t6000_variant
	},
	{
		.compatible = "apple,t8112-avd",
		.data = &avd_t8112_variant
	},
	{
		.compatible = "apple,t6020-avd",
		.data = &avd_t6020_variant
	},
	{
		.compatible = "apple,t8122-avd",
		.data = &avd_t8122_variant
	},
	{
		.compatible = "apple,t8132-avd",
		.data = &avd_t8132_variant
	},
	{
		.compatible = "apple,t8140-avd",
		.data = &avd_t8140_variant
	},
	{},
};

MODULE_DEVICE_TABLE(of, avd_of_match);

static int avd_probe(struct platform_device *pdev)
{
	struct avd_dev *avd;
	const struct of_device_id *match;
	int ret, irq;

	avd = devm_kzalloc(&pdev->dev, sizeof(*avd), GFP_KERNEL);
	if (!avd)
		return -ENOMEM;

	platform_set_drvdata(pdev, avd);
	avd->dev = &pdev->dev;
	avd->pdev = pdev;

	mutex_init(&avd->vdev_lock);

	match = of_match_node(avd_of_match, pdev->dev.of_node);
	avd->variant = match->data;

	avd->rstc = devm_reset_control_get_exclusive(avd->dev, NULL);

	avd->code = devm_platform_ioremap_resource_byname(pdev, "code");
	if (IS_ERR(avd->code))
		return PTR_ERR(avd->code);
	avd->mbox = devm_platform_ioremap_resource_byname(pdev, "mbox");
	if (IS_ERR(avd->mbox))
		return PTR_ERR(avd->mbox);
	avd->ctrl = devm_platform_ioremap_resource_byname(pdev, "ctrl");
	if (IS_ERR(avd->ctrl))
		return PTR_ERR(avd->ctrl);

	avd->domain = iommu_get_domain_for_dev(avd->dev);
	if (avd->domain) {
		avd->empty_domain = iommu_paging_domain_alloc(avd->dev);
		if (IS_ERR(avd->empty_domain)) {
			avd->empty_domain = NULL;
			dev_warn(avd->dev, "cannot alloc new empty domain");
		}
	}

	ret = request_firmware(&avd->fw, avd->variant->fw_name, avd->dev);

	if (ret) {
		dev_err(avd->dev, "failed to load firmware: %d", ret);
		return ret;
	}

	ret = dma_set_mask_and_coherent(avd->dev,
			DMA_BIT_MASK((avd->variant->quirks & AVD_QUIRK_LSR) ? 38 : 64));

	if (ret) {
		dev_err(avd->dev, "Failed to set DMA mask");
		return ret;
	}

	irq = platform_get_irq(pdev, 1);
	if (irq < 0)
		return irq;
	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
			avd_irq_handler, IRQF_ONESHOT,
			dev_name(&pdev->dev), avd);
	if (ret) {
		dev_err(avd->dev, "Could not request IRQ 1");
		return ret;
	}

	pm_runtime_set_autosuspend_delay(avd->dev, 100);
	pm_runtime_use_autosuspend(avd->dev);
	pm_runtime_enable(avd->dev);

	ret = avd_v4l2_init(avd);
	if (ret)
		goto err_disable_runtime_pm;

	return 0;

err_disable_runtime_pm:
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return ret;
}

const struct avd_coded_fmt_ops avd_hevc_fmt_ops;

static void avd_remove(struct platform_device *pdev)
{
	struct avd_dev *avd = platform_get_drvdata(pdev);

	release_firmware(avd->fw);

	avd_v4l2_cleanup(avd);

	if (avd->empty_domain)
		iommu_domain_free(avd->empty_domain);

	pm_runtime_disable(avd->dev);
	pm_runtime_dont_use_autosuspend(avd->dev);
}

static int avd_runtime_resume(struct device *dev)
{
	int ret;
	struct avd_dev *avd = platform_get_drvdata(to_platform_device(dev));

	ret = avd_boot(avd);
	if (ret)
		dev_err(dev, "failed to boot");
	return ret;
}

static int avd_runtime_suspend(struct device *dev)
{
	struct avd_dev *avd = platform_get_drvdata(to_platform_device(dev));

	avd_shutdown(avd);
	return 0;
}

static const struct dev_pm_ops avd_pm_ops = { SET_SYSTEM_SLEEP_PM_OPS(
	pm_runtime_force_suspend,
	pm_runtime_force_resume) SET_RUNTIME_PM_OPS(avd_runtime_suspend,
						    avd_runtime_resume, NULL) };

static struct platform_driver avd_driver = {
	.probe = avd_probe,
	.remove = avd_remove,
	.driver = {
		.name = "avd",
		.of_match_table = avd_of_match,
		.pm = &avd_pm_ops,
	},
};
module_platform_driver(avd_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Apple avd v4l2 sl m2m");
