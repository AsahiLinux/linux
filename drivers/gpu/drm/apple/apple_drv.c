// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io> */
/* Based on meson driver which is
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 * Copyright (C) 2014 Endless Mobile
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>

#include <drm/drm_aperture.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_mode.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_fixed.h>

#include "dcp.h"

#define DRIVER_NAME     "apple"
#define DRIVER_DESC     "Apple display controller DRM driver"

#define FRAC_16_16(mult, div)    (((mult) << 16) / (div))

#define MAX_COPROCESSORS 2

struct apple_drm_private {
	struct drm_device drm;
};

DEFINE_DRM_GEM_DMA_FOPS(apple_fops);

static int apple_drm_gem_dumb_create(struct drm_file *file_priv,
                            struct drm_device *drm,
                            struct drm_mode_create_dumb *args)
{
        args->pitch = ALIGN(DIV_ROUND_UP(args->width * args->bpp, 8), 64);
        args->size = args->pitch * args->height;

	return drm_gem_dma_dumb_create_internal(file_priv, drm, args);
}

static const struct drm_driver apple_drm_driver = {
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(apple_drm_gem_dumb_create),
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= "20221106",
	.major			= 1,
	.minor			= 0,
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.fops			= &apple_fops,
};

static int apple_plane_atomic_check(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state;
	struct drm_crtc_state *crtc_state;

	new_plane_state = drm_atomic_get_new_plane_state(state, plane);

	if (!new_plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state, new_plane_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	/*
	 * DCP limits downscaling to 2x and upscaling to 4x. Attempting to
	 * scale outside these bounds errors out when swapping.
	 *
	 * This function also takes care of clipping the src/dest rectangles,
	 * which is required for correct operation. Partially off-screen
	 * surfaces may appear corrupted.
	 *
	 * DCP does not distinguish plane types in the hardware, so we set
	 * can_position. If the primary plane does not fill the screen, the
	 * hardware will fill in zeroes (black).
	 */
	return drm_atomic_helper_check_plane_state(new_plane_state,
						   crtc_state,
						   FRAC_16_16(1, 4),
						   FRAC_16_16(2, 1),
						   true, true);
}

static void apple_plane_atomic_update(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	/* Handled in atomic_flush */
}

static const struct drm_plane_helper_funcs apple_plane_helper_funcs = {
	.atomic_check	= apple_plane_atomic_check,
	.atomic_update	= apple_plane_atomic_update,
};

static const struct drm_plane_funcs apple_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

/*
 * Table of supported formats, mapping from DRM fourccs to DCP fourccs.
 *
 * For future work, DCP supports more formats not listed, including YUV
 * formats, an extra RGBA format, and a biplanar RGB10_A8 format (fourcc b3a8)
 * used for HDR.
 *
 * Note: we don't have non-alpha formats but userspace breaks without XRGB. It
 * doesn't matter for the primary plane, but cursors/overlays must not
 * advertise formats without alpha.
 */
static const u32 dcp_formats[] = {
	// DRM_FORMAT_XRGB2101010,
	// DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
};

u64 apple_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static struct drm_plane *apple_plane_init(struct drm_device *dev,
					  unsigned long possible_crtcs,
					  enum drm_plane_type type)
{
	int ret;
	struct drm_plane *plane;

	plane = devm_kzalloc(dev->dev, sizeof(*plane), GFP_KERNEL);

	ret = drm_universal_plane_init(dev, plane, possible_crtcs,
				       &apple_plane_funcs,
				       dcp_formats, ARRAY_SIZE(dcp_formats),
				       apple_format_modifiers, type, NULL);
	if (ret)
		return ERR_PTR(ret);

	drm_plane_helper_add(plane, &apple_plane_helper_funcs);

	return plane;
}

static enum drm_connector_status
apple_connector_detect(struct drm_connector *connector, bool force)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);

	return apple_connector->connected ? connector_status_connected :
						  connector_status_disconnected;
}

static void apple_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state;
	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (crtc_state->active_changed && crtc_state->active) {
		struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
		dev_dbg(&apple_crtc->dcp->dev, "%s", __func__);
		dcp_poweron(apple_crtc->dcp);
		dev_dbg(&apple_crtc->dcp->dev, "%s finished", __func__);
	}
}

static void apple_crtc_atomic_disable(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state;
	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (crtc_state->active_changed && !crtc_state->active) {
		struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
		dev_dbg(&apple_crtc->dcp->dev, "%s", __func__);
		dcp_poweroff(apple_crtc->dcp);
		dev_dbg(&apple_crtc->dcp->dev, "%s finished", __func__);
	}

	if (crtc->state->event && !crtc->state->active) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);

		crtc->state->event = NULL;
	}
}

static void apple_crtc_atomic_begin(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	unsigned long flags;

	if (crtc->state->event) {
		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		apple_crtc->event = crtc->state->event;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
		crtc->state->event = NULL;
	}
}

static void dcp_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *dev = old_state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, old_state);

	drm_atomic_helper_commit_modeset_enables(dev, old_state);

	drm_atomic_helper_commit_planes(dev, old_state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_fake_vblank(old_state);

	drm_atomic_helper_commit_hw_done(old_state);

	drm_atomic_helper_wait_for_flip_done(dev, old_state);

	drm_atomic_helper_cleanup_planes(dev, old_state);
}


static const struct drm_crtc_funcs apple_crtc_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.destroy		= drm_crtc_cleanup,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.set_config             = drm_atomic_helper_set_config,
};

static const struct drm_mode_config_funcs apple_mode_config_funcs = {
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
	.fb_create		= drm_gem_fb_create,
};

static const struct drm_mode_config_helper_funcs apple_mode_config_helpers = {
	.atomic_commit_tail	= dcp_atomic_commit_tail,
};

static const struct drm_connector_funcs apple_connector_funcs = {
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= drm_connector_cleanup,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
	.detect			= apple_connector_detect,
};

static const struct drm_connector_helper_funcs apple_connector_helper_funcs = {
	.get_modes		= dcp_get_modes,
	.mode_valid		= dcp_mode_valid,
};

static const struct drm_crtc_helper_funcs apple_crtc_helper_funcs = {
	.atomic_begin		= apple_crtc_atomic_begin,
	.atomic_check		= dcp_crtc_atomic_check,
	.atomic_flush		= dcp_flush,
	.atomic_enable		= apple_crtc_atomic_enable,
	.atomic_disable		= apple_crtc_atomic_disable,
	.mode_fixup		= dcp_crtc_mode_fixup,
};

static int apple_probe_per_dcp(struct device *dev,
			       struct drm_device *drm,
			       struct platform_device *dcp,
			       int num)
{
	struct apple_crtc *crtc;
	struct apple_connector *connector;
	struct drm_encoder *encoder;
	struct drm_plane *primary;
	int ret;

	primary = apple_plane_init(drm, 1U << num, DRM_PLANE_TYPE_PRIMARY);

	if (IS_ERR(primary))
		return PTR_ERR(primary);

	crtc = devm_kzalloc(dev, sizeof(*crtc), GFP_KERNEL);
	ret = drm_crtc_init_with_planes(drm, &crtc->base, primary, NULL,
					&apple_crtc_funcs, NULL);
	if (ret)
		return ret;

	drm_crtc_helper_add(&crtc->base, &apple_crtc_helper_funcs);

	encoder = devm_kzalloc(dev, sizeof(*encoder), GFP_KERNEL);
	encoder->possible_crtcs = drm_crtc_mask(&crtc->base);
	ret = drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_TMDS);
	if (ret)
		return ret;

	connector = devm_kzalloc(dev, sizeof(*connector), GFP_KERNEL);
	drm_connector_helper_add(&connector->base,
				 &apple_connector_helper_funcs);

	ret = drm_connector_init(drm, &connector->base, &apple_connector_funcs,
				 dcp_get_connector_type(dcp));
	if (ret)
		return ret;

	connector->base.polled = DRM_CONNECTOR_POLL_HPD;
	connector->connected = false;
	connector->dcp = dcp;

	INIT_WORK(&connector->hotplug_wq, dcp_hotplug);

	crtc->dcp = dcp;
	dcp_link(dcp, crtc, connector);

	return drm_connector_attach_encoder(&connector->base, encoder);
}

static int apple_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_drm_private *apple;
	struct platform_device *dcp[MAX_COPROCESSORS];
	int ret, nr_dcp, i;

	for (nr_dcp = 0; nr_dcp < MAX_COPROCESSORS; ++nr_dcp) {
		struct device_node *np;
		struct device_link *dcp_link;

		np = of_parse_phandle(dev->of_node, "apple,coprocessors",
				      nr_dcp);

		if (!np)
			break;

		dcp[nr_dcp] = of_find_device_by_node(np);

		if (!dcp[nr_dcp])
			return -ENODEV;

		dcp_link = device_link_add(dev, &dcp[nr_dcp]->dev,
					   DL_FLAG_AUTOREMOVE_CONSUMER);
		if (!dcp_link) {
			dev_err(dev, "Failed to link to DCP %d device", nr_dcp);
			return -EINVAL;
		}

		if (dcp_link->supplier->links.status != DL_DEV_DRIVER_BOUND)
			return -EPROBE_DEFER;
	}

	/* Need at least 1 DCP for a display subsystem */
	if (nr_dcp < 1)
		return -ENODEV;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;

	apple = devm_drm_dev_alloc(dev, &apple_drm_driver,
				   struct apple_drm_private, drm);
	if (IS_ERR(apple))
		return PTR_ERR(apple);

	dev_set_drvdata(dev, apple);

	ret = drmm_mode_config_init(&apple->drm);
	if (ret)
		goto err_unload;

	/*
	 * IOMFB::UPPipeDCP_H13P::verify_surfaces produces the error "plane
	 * requires a minimum of 32x32 for the source buffer" if smaller
	 */
	apple->drm.mode_config.min_width = 32;
	apple->drm.mode_config.min_height = 32;

	/*
	 * TODO: this is the max framebuffer size not the maximal supported
	 * output resolution. DCP reports the maximal framebuffer size take it
	 * from there.
	 * Hardcode it for now to the M1 Max DCP reported 'MaxSrcBufferWidth'
	 * and 'MaxSrcBufferHeight' of 16384.
	 */
	apple->drm.mode_config.max_width = 16384;
	apple->drm.mode_config.max_height = 16384;

	apple->drm.mode_config.funcs = &apple_mode_config_funcs;
	apple->drm.mode_config.helper_private = &apple_mode_config_helpers;

	for (i = 0; i < nr_dcp; ++i) {
		ret = apple_probe_per_dcp(dev, &apple->drm, dcp[i], i);

		if (ret)
			goto err_unload;

		ret = dcp_start(dcp[i]);

		if (ret)
			goto err_unload;
	}

	drm_mode_config_reset(&apple->drm);

	// remove before registering our DRM device
	ret = drm_aperture_remove_framebuffers(false, &apple_drm_driver);
	if (ret)
		return ret;

	ret = drm_dev_register(&apple->drm, 0);
	if (ret)
		goto err_unload;

	drm_fbdev_generic_setup(&apple->drm, 32);

	return 0;

err_unload:
	drm_dev_put(&apple->drm);
	return ret;
}

static int apple_platform_remove(struct platform_device *pdev)
{
	struct apple_drm_private *apple = platform_get_drvdata(pdev);

	drm_dev_unregister(&apple->drm);

	return 0;
}

static const struct of_device_id of_match[] = {
	{ .compatible = "apple,display-subsystem" },
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

#ifdef CONFIG_PM_SLEEP
static int apple_platform_suspend(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(&apple->drm);
}

static int apple_platform_resume(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);

	drm_mode_config_helper_resume(&apple->drm);
	return 0;
}

static const struct dev_pm_ops apple_platform_pm_ops = {
	.suspend	= apple_platform_suspend,
	.resume		= apple_platform_resume,
};
#endif

static struct platform_driver apple_platform_driver = {
	.driver	= {
		.name = "apple-drm",
		.of_match_table	= of_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &apple_platform_pm_ops,
#endif
	},
	.probe		= apple_platform_probe,
	.remove		= apple_platform_remove,
};

module_platform_driver(apple_platform_driver);

MODULE_AUTHOR("Alyssa Rosenzweig <alyssa@rosenzweig.io>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("Dual MIT/GPL");
