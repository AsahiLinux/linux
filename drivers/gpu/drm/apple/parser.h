// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

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


struct dcp_sound_format_mask {
	u64 formats;			/* SNDRV_PCM_FMTBIT_* */
	unsigned int rates;		/* SNDRV_PCM_RATE_* */
	unsigned int nchans;
};

struct dcp_sound_cookie {
	u8 data[24];
};

struct snd_pcm_chmap_elem;
int parse_sound_constraints(struct dcp_parse_ctx *handle,
			    struct dcp_sound_format_mask *sieve,
			    struct dcp_sound_format_mask *hits);
int parse_sound_mode(struct dcp_parse_ctx *handle,
		     struct dcp_sound_format_mask *sieve,
		     struct snd_pcm_chmap_elem *chmap,
		     struct dcp_sound_cookie *cookie);

#endif
