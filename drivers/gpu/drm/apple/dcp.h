// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io> */

#ifndef __APPLE_DCP_H__
#define __APPLE_DCP_H__

#include <drm/drm_atomic.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>

#include "connector.h"
#include "dcp-internal.h"
#include "parser.h"

struct apple_crtc {
	struct drm_crtc base;
	struct drm_pending_vblank_event *event;
	bool vsync_disabled;

	/* Reference to the DCP device owning this CRTC */
	struct platform_device *dcp;
};

#define to_apple_crtc(x) container_of(x, struct apple_crtc, base)

struct apple_encoder {
	struct drm_encoder base;
};

#define to_apple_encoder(x) container_of(x, struct apple_encoder, base)

void dcp_poweroff(struct platform_device *pdev);
void dcp_poweron(struct platform_device *pdev);
int dcp_crtc_atomic_check(struct drm_crtc *crtc, struct drm_atomic_state *state);
int dcp_get_connector_type(struct platform_device *pdev);
void dcp_link(struct platform_device *pdev, struct apple_crtc *apple,
	      struct apple_connector *connector);
int dcp_start(struct platform_device *pdev);
int dcp_wait_ready(struct platform_device *pdev, u64 timeout);
void dcp_flush(struct drm_crtc *crtc, struct drm_atomic_state *state);
bool dcp_is_initialized(struct platform_device *pdev);
void apple_crtc_vblank(struct apple_crtc *apple);
void dcp_drm_crtc_vblank(struct apple_crtc *crtc);
int dcp_get_modes(struct drm_connector *connector);
int dcp_mode_valid(struct drm_connector *connector,
		   struct drm_display_mode *mode);
int dcp_crtc_atomic_modeset(struct drm_crtc *crtc,
			    struct drm_atomic_state *state);
bool dcp_crtc_mode_fixup(struct drm_crtc *crtc,
			 const struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode);
void dcp_set_dimensions(struct apple_dcp *dcp);
void dcp_send_message(struct apple_dcp *dcp, u8 endpoint, u64 message);

int dcp_dptx_connect_oob(struct platform_device *pdev, u32 port);
int dcp_dptx_disconnect_oob(struct platform_device *pdev, u32 port);

int iomfb_start_rtkit(struct apple_dcp *dcp);
void iomfb_shutdown(struct apple_dcp *dcp);
/* rtkit message handler for IOMFB messages */
void iomfb_recv_msg(struct apple_dcp *dcp, u64 message);

int systemep_init(struct apple_dcp *dcp);
int dptxep_init(struct apple_dcp *dcp);
int ibootep_init(struct apple_dcp *dcp);
int dpavservep_init(struct apple_dcp *dcp);
int avep_init(struct apple_dcp *dcp);


void __init dcp_audio_register(void);
void __exit dcp_audio_unregister(void);

#endif
