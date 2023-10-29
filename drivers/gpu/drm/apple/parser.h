// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io> */

#ifndef __APPLE_DCP_PARSER_H__
#define __APPLE_DCP_PARSER_H__

/* For mode parsing */
#include <drm/drm_modes.h>

struct apple_dcp;

struct dcp_parse_ctx {
	struct apple_dcp *dcp;
	void *blob;
	u32 pos, len;
};

/*
 * Represents a single display mode. These mode objects are populated at
 * runtime based on the TimingElements dictionary sent by the DCP.
 */
struct dcp_display_mode {
	struct drm_display_mode mode;
	u32 color_mode_id;
	u32 timing_mode_id;
};

int parse(void *blob, size_t size, struct dcp_parse_ctx *ctx);
struct dcp_display_mode *enumerate_modes(struct dcp_parse_ctx *handle,
					 unsigned int *count, int width_mm,
					 int height_mm, unsigned notch_height);
int parse_display_attributes(struct dcp_parse_ctx *handle, int *width_mm,
			     int *height_mm);

#endif
