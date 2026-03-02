// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */
/* Based on meson driver which is
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 * Copyright (C) 2014 Endless Mobile
 */

#include <linux/aperture.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_mode.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_module.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_fixed.h>

#include "dcp.h"
#include "plane.h"

#define DRIVER_NAME     "apple"
#define DRIVER_DESC     "Apple display controller DRM driver"

#define MAX_COPROCESSORS 3

struct apple_drm_private {
	struct drm_device drm;
};

DEFINE_DRM_GEM_DMA_FOPS(apple_fops);

#define DART_PAGE_SIZE 16384

static int apple_drm_gem_dumb_create(struct drm_file *file_priv,
                            struct drm_device *drm,
                            struct drm_mode_create_dumb *args)
{
        args->pitch = ALIGN(DIV_ROUND_UP(args->width * args->bpp, 8), 64);
        args->size = round_up(args->pitch * args->height, DART_PAGE_SIZE);

	return drm_gem_dma_dumb_create_internal(file_priv, drm, args);
}

static const struct drm_driver apple_drm_driver = {
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(apple_drm_gem_dumb_create),
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.major			= 1,
	.minor			= 0,
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC | DRIVER_SYNCOBJ | DRIVER_SYNCOBJ_TIMELINE,
	.fops			= &apple_fops,
};

static enum drm_connector_status
apple_connector_detect(struct drm_connector *connector, bool force)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);

	return apple_connector->connected ? connector_status_connected :
						  connector_status_disconnected;
}

static void apple_connector_oob_hotplug(struct drm_connector *connector,
					enum drm_connector_status status)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);

	printk("#### oob_hotplug status:0x%x ####\n", (u32)status);

	if (status == connector_status_connected)
		dcp_dptx_connect_oob(apple_connector->dcp, 0);
	else if (status == connector_status_disconnected)
		dcp_dptx_disconnect_oob(apple_connector->dcp, 0);
	else
		dev_err(&apple_connector->dcp->dev, "unexpected connector status"
			":0x%x in oob_hotplug event\n", (u32)status);
}

static void apple_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state;
	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (crtc_state->active_changed && crtc_state->active) {
		struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
		dcp_poweron(apple_crtc->dcp);
		/* Force the CTM to be set on first swap */
		crtc_state->color_mgmt_changed = true;
	}

	if (crtc_state->active)
		dcp_crtc_atomic_modeset(crtc, state);
}

static void apple_crtc_atomic_disable(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state;
	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (crtc_state->active_changed && !crtc_state->active) {
		struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
		dcp_poweroff(apple_crtc->dcp);
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

static void apple_crtc_cleanup(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
	kfree(to_apple_crtc(crtc));
}

static int apple_crtc_parse_crc_source(const char *source, bool *enabled)
{
	int ret = 0;

	if (!source) {
		*enabled = false;
	} else if (strcmp(source, "auto") == 0) {
		*enabled = true;
	} else {
		*enabled = false;
		ret = -EINVAL;
	}

	return ret;
}

static int apple_crtc_set_crc_source(struct drm_crtc *crtc, const char *source)
{
	bool enabled = false;

	int ret = apple_crtc_parse_crc_source(source, &enabled);

	if (!ret)
		dcp_set_crc(crtc, enabled);

	return ret;
}

static int apple_crtc_verify_crc_source(struct drm_crtc *crtc,
					const char *source,
					size_t *values_cnt)
{
	bool enabled;

	if (apple_crtc_parse_crc_source(source, &enabled) < 0) {
		pr_warn("dcp: Invalid CRC source name %s\n", source);
		return -EINVAL;
	}

	*values_cnt = 1;

	return 0;
}

static const char * const apple_crtc_crc_sources[] = {"auto"};

static const char *const * apple_crtc_get_crc_sources(struct drm_crtc *crtc,
						      size_t *count)
{
	*count = ARRAY_SIZE(apple_crtc_crc_sources);
	return apple_crtc_crc_sources;
}

static const struct drm_crtc_funcs apple_crtc_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.destroy		= apple_crtc_cleanup,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.set_config             = drm_atomic_helper_set_config,
	.set_crc_source		= apple_crtc_set_crc_source,
	.verify_crc_source	= apple_crtc_verify_crc_source,
	.get_crc_sources	= apple_crtc_get_crc_sources,

};

static const struct drm_mode_config_funcs apple_mode_config_funcs = {
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
	.fb_create		= drm_gem_fb_create,
};

static const struct drm_mode_config_helper_funcs apple_mode_config_helpers = {
	.atomic_commit_tail	= drm_atomic_helper_commit_tail_rpm,
};

static void appledrm_connector_cleanup(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
	kfree(to_apple_connector(connector));
}

static const struct drm_connector_funcs apple_connector_funcs = {
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= appledrm_connector_cleanup,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
	.detect			= apple_connector_detect,
	.debugfs_init		= apple_connector_debugfs_init,
	.oob_hotplug_event	= apple_connector_oob_hotplug,
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
			       int num, bool dcp_ext)
{
	struct apple_crtc *crtc;
	struct apple_connector *connector;
	struct apple_encoder *enc;
	struct drm_plane *planes[DCP_MAX_PLANES];
	int ret, i;
	int immutable_zpos = 0;
	bool supports_l10r = !dcp_fw_compat_is_12_x(dcp);

	planes[0] = apple_plane_init(drm, 1U << num, supports_l10r,
				     DRM_PLANE_TYPE_PRIMARY);
	if (IS_ERR(planes[0]))
		return PTR_ERR(planes[0]);
	ret = drm_plane_create_zpos_immutable_property(planes[0], immutable_zpos);
	if (ret) {
		return ret;
	}


	/* Set up our other planes */
	for (i = 1; i < DCP_MAX_PLANES; i++) {
		planes[i] = apple_plane_init(drm, 1U << num, supports_l10r,
					     DRM_PLANE_TYPE_OVERLAY);
		if (IS_ERR(planes[i]))
			return PTR_ERR(planes[i]);
		immutable_zpos++;
		ret = drm_plane_create_zpos_immutable_property(planes[i], immutable_zpos);
		if (ret) {
			return ret;
		}
	}

	/*
	 * Even though we have an overlay plane, we cannot expose it to legacy
	 * userspace for cursors as we cannot make the same guarantees as ye olde
	 * hardware cursor planes such userspace would expect us to. Modern userspace
	 * knows what to do with overlays.
	 */
	crtc = kzalloc(sizeof(*crtc), GFP_KERNEL);
	ret = drm_crtc_init_with_planes(drm, &crtc->base, planes[0], NULL,
					&apple_crtc_funcs, NULL);
	if (ret)
		return ret;

	drm_crtc_helper_add(&crtc->base, &apple_crtc_helper_funcs);
	drm_crtc_enable_color_mgmt(&crtc->base, 0, true, 0);

	enc = drmm_simple_encoder_alloc(drm, struct apple_encoder, base,
					DRM_MODE_ENCODER_TMDS);
	if (IS_ERR(enc))
                return PTR_ERR(enc);
	enc->base.possible_crtcs = drm_crtc_mask(&crtc->base);

	connector = kzalloc(sizeof(*connector), GFP_KERNEL);
	mutex_init(&connector->chunk_lock);
	drm_connector_helper_add(&connector->base,
				 &apple_connector_helper_funcs);

	// HACK:
	if (dcp_ext)
		connector->base.fwnode = fwnode_handle_get(dcp->dev.fwnode);

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

	return drm_connector_attach_encoder(&connector->base, &enc->base);
}

static int apple_get_fb_resource(struct device *dev, const char *name,
				 struct resource *fb_r)
{
	int idx, ret = -ENODEV;
	struct device_node *node;

	idx = of_property_match_string(dev->of_node, "memory-region-names", name);

	node = of_parse_phandle(dev->of_node, "memory-region", idx);
	if (!node) {
		dev_err(dev, "reserved-memory node '%s' not found\n", name);
		return -ENODEV;
	}

	if (!of_device_is_available(node)) {
		dev_err(dev, "reserved-memory node '%s' is unavailable\n", name);
		goto err;
	}

	if (!of_device_is_compatible(node, "framebuffer")) {
		dev_err(dev, "reserved-memory node '%s' is incompatible\n",
			node->full_name);
		goto err;
	}

	ret = of_address_to_resource(node, 0, fb_r);

err:
	of_node_put(node);
	return ret;
}

static const struct of_device_id apple_dcp_id_tbl[] = {
	{ .compatible = "apple,dcp" },
	{ .compatible = "apple,dcpext" },
	{},
};

static int apple_drm_init_dcp(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);
	struct platform_device *dcp[MAX_COPROCESSORS];
	struct device_node *np;
	u64 timeout;
	int i, ret, num_dcp = 0;

	for_each_matching_node(np, apple_dcp_id_tbl) {
		bool dcp_ext;
		if (!of_device_is_available(np)) {
			of_node_put(np);
			continue;
		}
		dcp_ext = of_device_is_compatible(np, "apple,dcpext") ||
		          of_property_present(np, "phys");

		dcp[num_dcp] = of_find_device_by_node(np);
		of_node_put(np);
		if (!dcp[num_dcp])
			continue;

		device_link_add(dev, &dcp[num_dcp]->dev, DL_FLAG_AUTOREMOVE_SUPPLIER);

		ret = apple_probe_per_dcp(dev, &apple->drm, dcp[num_dcp],
					  num_dcp, dcp_ext);
		if (ret)
			continue;

		ret = dcp_start(dcp[num_dcp]);
		if (ret)
			continue;

		num_dcp++;
	}

	if (num_dcp < 1)
		return -ENODEV;

	/*
	 * Starting DPTX might take some time.
	 */
	timeout = get_jiffies_64() + msecs_to_jiffies(3000);

	for (i = 0; i < num_dcp; ++i) {
		u64 jiffies = get_jiffies_64();
		u64 wait = time_after_eq64(jiffies, timeout) ?
				   0 :
				   timeout - jiffies;
		ret = dcp_wait_ready(dcp[i], wait);
		/* There is nothing we can do if a dcp/dcpext does not boot
		 * (successfully). Ignoring it should not do any harm now.
		 * Needs to reevaluated when adding dcpext support.
		 */
		if (ret)
			dev_warn(dev, "DCP[%d] not ready: %d\n", i, ret);
	}
	/* HACK: Wait for dcp* to settle before a modeset */
	msleep(100);

	return 0;
}

static int apple_drm_init(struct device *dev)
{
	struct apple_drm_private *apple;
	struct resource fb_r;
	resource_size_t fb_size;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;

	ret = apple_get_fb_resource(dev, "framebuffer", &fb_r);
	if (ret)
		return ret;

	apple = devm_drm_dev_alloc(dev, &apple_drm_driver,
				   struct apple_drm_private, drm);
	if (IS_ERR(apple))
		return PTR_ERR(apple);

	dev_set_drvdata(dev, apple);

	ret = component_bind_all(dev, apple);
	if (ret)
		return ret;

	ret = drmm_mode_config_init(&apple->drm);
	if (ret)
		goto err_unbind;

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

	ret = apple_drm_init_dcp(dev);
	if (ret)
		goto err_unbind;

	drm_mode_config_reset(&apple->drm);

	fb_size = fb_r.end - fb_r.start + 1;
	ret = aperture_remove_conflicting_devices(fb_r.start, fb_size,
						  apple_drm_driver.name);
	if (ret) {
		dev_err(dev, "Failed remove fb: %d\n", ret);
		goto err_unbind;
	}

	ret = drm_dev_register(&apple->drm, 0);
	if (ret)
		goto err_unbind;

	drm_client_setup_with_fourcc(&apple->drm, DRM_FORMAT_XRGB8888);

	return 0;

err_unbind:
	component_unbind_all(dev, NULL);
	return ret;
}

static void apple_drm_uninit(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);

	drm_dev_unregister(&apple->drm);
	drm_atomic_helper_shutdown(&apple->drm);

	component_unbind_all(dev, NULL);

	dev_set_drvdata(dev, NULL);
}

static int apple_drm_bind(struct device *dev)
{
	return apple_drm_init(dev);
}

static void apple_drm_unbind(struct device *dev)
{
	apple_drm_uninit(dev);
}

const struct component_master_ops apple_drm_ops = {
	.bind	= apple_drm_bind,
	.unbind	= apple_drm_unbind,
};

static int add_dcp_components(struct device *dev,
			      struct component_match **matchptr)
{
	struct device_node *np, *endpoint, *port;
	int num = 0;

	for_each_matching_node(np, apple_dcp_id_tbl) {
		if (of_device_is_available(np)) {
			drm_of_component_match_add(dev, matchptr,
						   component_compare_of, np);
			num++;
			for_each_endpoint_of_node(np, endpoint) {
				port = of_graph_get_remote_port_parent(endpoint);
				if (!port)
					continue;

#if !IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
				if (of_device_is_compatible(port, "apple,dpaudio")) {
					of_node_put(port);
					continue;
				}
#endif

				/*
				 * The ATC phy driver is not part of the component
				 * collection for the Apple display-subsystem so
				 * ignore it here.
				 */
				if (of_device_is_compatible(port, "apple,t8103-atcphy")) {
					of_node_put(port);
					continue;
				}

				if (of_device_is_available(port))
					drm_of_component_match_add(dev, matchptr,
							   component_compare_of,
							   port);
				of_node_put(port);
			}
		}
		of_node_put(np);
	}

	return num;
}

static int apple_platform_probe(struct platform_device *pdev)
{
	struct device *mdev = &pdev->dev;
	struct component_match *match = NULL;
	int num_dcp;

	/* add DCP components, handle less than 1 as probe error */
	num_dcp = add_dcp_components(mdev, &match);
	if (num_dcp < 1)
		return -ENODEV;

	return component_master_add_with_match(mdev, &apple_drm_ops, match);
}

static void apple_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &apple_drm_ops);
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

	if (apple)
		return drm_mode_config_helper_suspend(&apple->drm);

	return 0;
}

static int apple_platform_resume(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);

	if (apple)
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



static int __init appledrm_register(void)
{
	if (drm_firmware_drivers_only())
		return -ENODEV;

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	dcp_audio_register();
#endif
	dcp_register();
	platform_driver_register(&apple_platform_driver);

	return 0;
}

static void __exit appledrm_unregister(void)
{
#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	dcp_audio_unregister();
#endif
	dcp_unregister();
	platform_driver_unregister(&apple_platform_driver);
}

module_init(appledrm_register);
module_exit(appledrm_unregister);

MODULE_AUTHOR("Asahi Linux contributors");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("Dual MIT/GPL");
