// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* "Copyright" 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io> */

#ifndef __APPLE_CONNECTOR_H__
#define __APPLE_CONNECTOR_H__

#include <linux/workqueue.h>

#include <drm/drm_atomic.h>
#include "drm/drm_connector.h"
#include "drm/drm_edid.h"

struct apple_connector;

#include "dcp-internal.h"

void dcp_hotplug(struct work_struct *work);

struct apple_connector {
	struct drm_connector base;
	bool connected;

	struct platform_device *dcp;

	const struct drm_edid *drm_edid;

	/* Workqueue for sending hotplug events to the associated device */
	struct work_struct hotplug_wq;

	struct mutex chunk_lock;

	struct dcp_chunks color_elements;
	struct dcp_chunks timing_elements;
	struct dcp_chunks display_attributes;
	struct dcp_chunks transport;
};

#define to_apple_connector(x) container_of(x, struct apple_connector, base)

void apple_connector_debugfs_init(struct drm_connector *connector, struct dentry *root);

void dcp_connector_update_dict(struct apple_connector *connector, const char *key,
			       struct dcp_chunks *chunks);
#endif
