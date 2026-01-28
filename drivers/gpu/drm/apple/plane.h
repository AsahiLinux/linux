// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright (C) The Asahi Linux Contributors
 */

#ifndef __APPLE_PLANE_H__
#define __APPLE_PLANE_H__

#include <drm/drm_plane.h>

#include <linux/types.h>

#include "iomfb_plane.h"

struct apple_plane_state {
	struct drm_plane_state base;
	struct dcp_surface surf;
	struct dcp_rect src_rect;
	struct dcp_rect dst_rect;
	u64 iova;
};

#define to_apple_plane_state(x) container_of(x, struct apple_plane_state, base)

struct drm_plane *apple_plane_init(struct drm_device *dev,
				   unsigned long possible_crtcs,
				   bool supports_l10r,
				   enum drm_plane_type type);

#endif /* __APPLE_PLANE_H__ */
