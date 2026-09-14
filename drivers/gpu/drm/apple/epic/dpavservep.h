// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright The Asahi Linux Contributors */

#ifndef _DRM_APPLE_EPIC_DPAVSERV_H
#define _DRM_APPLE_EPIC_DPAVSERV_H

#include <linux/completion.h>
#include <linux/types.h>

struct drm_edid;
struct apple_epic_service;

struct dcpavserv {
	bool enabled;
	struct completion enable_completion;
	u32 unit;
	struct apple_epic_service *service;
};

const struct drm_edid *dcpavserv_copy_edid(struct apple_epic_service *service);

/*
 * DDC/CI over the AV service. macOS reaches these through
 * IOAVServiceReadI2C / IOAVServiceWriteI2C.
 */
int dcpavserv_read_i2c(struct apple_epic_service *service, u32 chip_addr,
		       u32 data_addr, void *data, size_t len);
int dcpavserv_write_i2c(struct apple_epic_service *service, u32 chip_addr,
			u32 data_addr, const void *data, size_t len);

#endif /* _DRM_APPLE_EPIC_DPAVSERV_H */
