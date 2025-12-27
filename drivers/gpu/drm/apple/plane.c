// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright (C) The Asahi Linux Contributors
 */

#include "plane.h"

#include "iomfb_internal.h"

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_plane.h>

#define FRAC_16_16(mult, div)    (((mult) << 16) / (div))

static int apple_plane_atomic_check(struct drm_plane *plane,
				    struct drm_atomic_commit *commit)
{
	struct drm_plane_state *new_plane_state;
	struct drm_crtc_state *crtc_state;
	struct drm_rect *dst;
	int ret;

	new_plane_state = drm_atomic_get_new_plane_state(commit, plane);

	if (!new_plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(commit, new_plane_state->crtc);
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
	ret = drm_atomic_helper_check_plane_state(new_plane_state, crtc_state,
						  FRAC_16_16(1, 2),
						  FRAC_16_16(4, 1),
						  true, true);
	if (ret < 0)
		return ret;

	if (!new_plane_state->visible)
		return 0;

	/*
	 * DCP does not allow a surface to clip off the screen, and will crash
	 * if any blended surface is smaller than 32x32. Reject the atomic op
	 * if the plane will crash DCP.
	 *
	 * This is most pertinent to cursors. Userspace should fall back to
	 * software cursors if the plane check is rejected.
	 */
	dst = &new_plane_state->dst;
	if (drm_rect_width(dst) < 32 || drm_rect_height(dst) < 32) {
		dev_err_once(commit->dev->dev,
			"Plane operation would have crashed DCP! Rejected!\n\
			DCP requires 32x32 of every plane to be within screen space.\n\
			Your compositor asked to overlay [%dx%d, %dx%d] on %dx%d.\n\
			This is not supported, and your compositor should have\n\
			switched to software compositing when this operation failed.\n\
			You should not have noticed this at all. If your screen\n\
			froze/hitched, or your compositor crashed, please report\n\
			this to the your compositor's developers. We will not\n\
			throw this error again until you next reboot.\n",
			dst->x1, dst->y1, dst->x2, dst->y2,
			crtc_state->mode.hdisplay, crtc_state->mode.vdisplay);
		return -EINVAL;
	}

	return 0;
}

/*
 * DRM specifies rectangles as start and end coordinates.  DCP specifies
 * rectangles as a start coordinate and a width/height. Convert a DRM rectangle
 * to a DCP rectangle.
 */
static struct dcp_rect drm_to_dcp_rect(const struct drm_rect *rect)
{
	return (struct dcp_rect){ .x = rect->x1,
				  .y = rect->y1,
				  .w = drm_rect_width(rect),
				  .h = drm_rect_height(rect),
	};
}

static struct dcp_rect drm_to_dcp_rect_fp(const struct drm_rect *fp_rect)
{
	struct drm_rect rect;
	drm_rect_fp_to_int(&rect, fp_rect);
	return drm_to_dcp_rect(&rect);
}

static u32 drm_format_to_dcp(u32 drm)
{
	switch (drm) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return DCP_FORMAT_BGRA;

	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		return DCP_FORMAT_RGBA;

	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_ARGB2101010:
		return DCP_FORMAT_L10R;
	}

	pr_warn("DRM format %X not supported in DCP\n", drm);
	return 0;
}

static void apple_plane_atomic_update(struct drm_plane *plane,
				      struct drm_atomic_commit *commit)
{
	struct drm_plane_state *base = drm_atomic_get_new_plane_state(commit, plane);
	struct apple_plane_state *new_state;
	struct drm_gem_dma_object *obj;
	bool is_premultiplied = false;

	if (!base)
		return;

	new_state = to_apple_plane_state(base);

	if (!base->fb) {
		memset(&new_state->surf, 0, sizeof(new_state->surf));
		return;
	}

	struct drm_framebuffer *fb = base->fb;
	const struct drm_format_info *fmt = fb->format;
	/*
	 * DCP doesn't support XBGR8 / XRGB8 / XBGR2101010 natively. Blending as
	 * pre-multiplied alpha with a black background can be used as
	 * workaround for the bottommost plane.
	 */
	if (fmt->format == DRM_FORMAT_XRGB8888 ||
	    fmt->format == DRM_FORMAT_XBGR8888 ||
	    fmt->format == DRM_FORMAT_XBGR2101010)
		is_premultiplied = true;

	new_state->src_rect = drm_to_dcp_rect_fp(&base->src);
	new_state->dst_rect = drm_to_dcp_rect(&base->dst);

	new_state->surf = (struct dcp_surface){
		.is_premultiplied = is_premultiplied,
		.format = drm_format_to_dcp(fb->format->format),
		.xfer_func = DCP_XFER_FUNC_SDR,
		.colorspace = DCP_COLORSPACE_NATIVE,
		.stride = fb->pitches[0],
		.width = fb->width,
		.height = fb->height,
		.buf_size = fb->height * fb->pitches[0],
		// .surface_id = req->swap.surf_ids[l],

		/* Only used for compressed or multiplanar surfaces */
		.pix_size = 1,
		.pel_w = 1,
		.pel_h = 1,
		.has_comp = 1,
		.has_planes = 1,
	};

	/* the obvious helper call drm_fb_dma_get_gem_addr() adjusts
	 * the address for source x/y offsets. Since IOMFB has a direct
	 * support source position prefer that.
	 */
	obj = drm_fb_dma_get_gem_obj(base->fb, 0);
	if (obj)
		new_state->iova = obj->dma_addr + base->fb->offsets[0];
}

static const struct drm_plane_helper_funcs apple_primary_plane_helper_funcs = {
	.atomic_check	= apple_plane_atomic_check,
	.atomic_update	= apple_plane_atomic_update,
	.get_scanout_buffer = drm_fb_dma_get_scanout_buffer,
};

static const struct drm_plane_helper_funcs apple_plane_helper_funcs = {
	.atomic_check	= apple_plane_atomic_check,
	.atomic_update	= apple_plane_atomic_update,
};

// Duplicate drm_atomic_helper_plane_reset but allocate struct apple_plane_state
static void apple_plane_reset(struct drm_plane *plane)
{
        struct apple_plane_state *state = to_apple_plane_state(plane->state);
	if (state)
		__drm_atomic_helper_plane_destroy_state(&state->base);

	kfree(state);
	plane->state = NULL;
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state)
		__drm_atomic_helper_plane_reset(plane, &state->base);
}

static struct drm_plane_state *
apple_plane_duplicate_state(struct drm_plane *plane)
{
        struct apple_plane_state *apple_plane_state, *old_apple_plane_state;

	if (!plane->state)
		return NULL;

        old_apple_plane_state = to_apple_plane_state(plane->state);
        apple_plane_state = kzalloc(sizeof(*apple_plane_state), GFP_KERNEL);
        if (!apple_plane_state)
                return NULL;

        __drm_atomic_helper_plane_duplicate_state(plane, &apple_plane_state->base);

	apple_plane_state->surf = old_apple_plane_state->surf;

	return &apple_plane_state->base;
}

// void apple_plane_destroy_state(struct drm_plane *plane,
//                                      struct drm_plane_state *state)
// {
// 	drm_atomic_helper_plane_destroy_state(plane, state);
// }

static const struct drm_plane_funcs apple_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.reset			= apple_plane_reset,
	.atomic_duplicate_state = apple_plane_duplicate_state,
	// .atomic_destroy_state	= apple_plane_destroy_state,
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
static const u32 dcp_primary_formats[] = {
	DRM_FORMAT_XRGB2101010,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
};

static const u32 dcp_overlay_formats[] = {
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
};

u64 apple_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

struct apple_plane {
	struct drm_plane base;
};

struct drm_plane *apple_plane_init(struct drm_device *dev,
				   unsigned long possible_crtcs,
				   enum drm_plane_type type)
{
	struct apple_plane *plane;

	switch (type) {
	case DRM_PLANE_TYPE_PRIMARY:
		plane = drmm_universal_plane_alloc(dev, struct apple_plane, base, possible_crtcs,
				       &apple_plane_funcs,
				       dcp_primary_formats, ARRAY_SIZE(dcp_primary_formats),
				       apple_format_modifiers, type, NULL);
		break;
	case DRM_PLANE_TYPE_OVERLAY:
	case DRM_PLANE_TYPE_CURSOR:
		plane = drmm_universal_plane_alloc(dev, struct apple_plane, base, possible_crtcs,
				       &apple_plane_funcs,
				       dcp_overlay_formats, ARRAY_SIZE(dcp_overlay_formats),
				       apple_format_modifiers, type, NULL);
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR(plane))
		return ERR_PTR(PTR_ERR(plane));

	if (type == DRM_PLANE_TYPE_PRIMARY)
		drm_plane_helper_add(&plane->base, &apple_primary_plane_helper_funcs);
	else
		drm_plane_helper_add(&plane->base, &apple_plane_helper_funcs);

	return &plane->base;
}
