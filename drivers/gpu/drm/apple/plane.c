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
#include <drm/drm_gem.h>
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

	/*
	 * Pitches have to be 64-byte aligned.
	 */
	for (u32 i = 0; i < new_plane_state->fb->format->num_planes; i++)
		if (new_plane_state->fb->pitches[i] & 63)
			return -EINVAL;

	/*
	 * FIXME: dcp can currently only use multi-planar buffers using the same
	 *        object for all planes. It has a mandatory iommu so it should
	 *        be no problem to map multiple objects "linearly" into DCP
	 *        virtual address space and calculate the offsets accordingly.
	 *        Or maybe it can accept multiple BOs via the per plane field
	 *        `base`.
	 */
	if (new_plane_state->fb->format->num_planes > 1) {
		const struct drm_gem_object *first = new_plane_state->fb->obj[0];
		for (u32 i = 1; i < new_plane_state->fb->format->num_planes; i++)
			if (new_plane_state->fb->obj[i] != NULL &&
			    new_plane_state->fb->obj[i] != first)
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

static u32 drm_format_to_dcp(u32 drm, enum drm_color_range range)
{
	bool fr = range == DRM_COLOR_YCBCR_FULL_RANGE;
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

	/* semi planar YCbCr formats, limited and full range */
	case DRM_FORMAT_NV12:
		return fr ? DCP_FORMAT_420F : DCP_FORMAT_420V;
	case DRM_FORMAT_NV16:
		return fr ? DCP_FORMAT_422F : DCP_FORMAT_422V;
	case DRM_FORMAT_NV24:
		return fr ? DCP_FORMAT_444F : DCP_FORMAT_444V;

	/* semi planar 10-bit YCbCr formats, limited and full range */
	case DRM_FORMAT_P010:
		return fr ? DCP_FORMAT_XF20 : DCP_FORMAT_X420;
	case DRM_FORMAT_P210:
		return fr ? DCP_FORMAT_XF22 : DCP_FORMAT_X422;
	/*
	 * TODO: missing DRM fourcc for P410
	 */
#if defined(DRM_FORMAT_P410)
	case DRM_FORMAT_P410:
		return fr ? DCP_FORMAT_XF44 : DCP_FORMAT_X444;
#endif
	}

	pr_warn("DRM format %X not supported in DCP\n", drm);
	return 0;
}

static enum dcp_xfer_func get_xfer_func(bool is_yuv, enum drm_color_encoding enc)
{
	if (!is_yuv)
		return DCP_XFER_FUNC_SDR;

	switch (enc) {
	case DRM_COLOR_YCBCR_BT601:
		return DCP_XFER_FUNC_BT601;
	case DRM_COLOR_YCBCR_BT709:
	case DRM_COLOR_YCBCR_BT2020:
		return DCP_XFER_FUNC_BT1886;
	default:
		return DCP_XFER_FUNC_SDR;
	}
}

static enum dcp_colorspace get_colorspace(bool is_yuv,
					  enum drm_color_encoding enc)
{
	if (!is_yuv)
		return DCP_COLORSPACE_NATIVE;

	switch (enc) {
	case DRM_COLOR_YCBCR_BT601:
		return DCP_COLORSPACE_BT601;
	case DRM_COLOR_YCBCR_BT709:
		return DCP_COLORSPACE_BT709;
	case DRM_COLOR_YCBCR_BT2020:
		return DCP_COLORSPACE_BG_BT2020;
	default:
		return DCP_COLORSPACE_NATIVE;
	}
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
		.plane_cnt = fb->format->num_planes,
		.plane_cnt2 = fb->format->num_planes,
		.format = drm_format_to_dcp(fmt->format, base->color_range),
		.xfer_func = get_xfer_func(fmt->is_yuv, base->color_encoding),
		.colorspace = get_colorspace(fmt->is_yuv, base->color_encoding),
		.stride = fb->pitches[0],
		.width = fb->width,
		.height = fb->height,
		.buf_size = fb->height * fb->pitches[0],
		.surface_id = plane->base.id,

		/* Only used for compressed or multiplanar surfaces */
		.pix_size = 1,
		.pel_w = 1,
		.pel_h = 1,
		.has_comp = 1,
		.has_planes = 1,
	};

	/* Populate plane information for planar formats */
	struct dcp_surface *surf = &new_state->surf;
	for (int i = 0; fb->format->num_planes && i < fb->format->num_planes; i++) {
		u32 width = drm_format_info_plane_width(fb->format, fb->width, i);
		u32 height = drm_format_info_plane_height(fb->format, fb->height, i);
		u32 bh = drm_format_info_block_height(fb->format, i);
		u32 bw = drm_format_info_block_width(fb->format, i);

		surf->planes[i] = (struct dcp_plane_info){
			.width = width,
			.height = height,
			.base = fb->offsets[i] - fb->offsets[0],
			.offset = fb->offsets[i] - fb->offsets[0],
			.stride = fb->pitches[i],
			.size = height * fb->pitches[i],
			.tile_size = bw * bh,
			.tile_w = bw,
			.tile_h = bh,
		};

		if (i > 0)
			surf->buf_size += surf->planes[i].size;
	}

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
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
	DRM_FORMAT_P010,
	DRM_FORMAT_P210,
#if defined(DRM_FORMAT_P410)
	DRM_FORMAT_P410,
#endif
};

static const u32 dcp_overlay_formats[] = {
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
	DRM_FORMAT_P010,
	DRM_FORMAT_P210,
#if defined(DRM_FORMAT_P410)
	DRM_FORMAT_P410,
#endif
};

/*
 * Formats for the 12.x firmware which does not support "l10r" / ARGB2101010
 */
static const u32 dcp_primary_formats_12_x[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
	DRM_FORMAT_P010,
	DRM_FORMAT_P210,
#if defined(DRM_FORMAT_P410)
	DRM_FORMAT_P410,
#endif
};

static const u32 dcp_overlay_formats_12_x[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
	DRM_FORMAT_P010,
	DRM_FORMAT_P210,
#if defined(DRM_FORMAT_P410)
	DRM_FORMAT_P410,
#endif
};

u64 apple_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

struct drm_plane *apple_plane_init(struct drm_device *dev,
				   unsigned long possible_crtcs,
				   u32 iomfb_surf,
				   bool supports_l10r,
				   enum drm_plane_type type)
{
	struct apple_plane *plane;
	const u32 *fmts;
	u32 num_fmts;

	switch (type) {
	case DRM_PLANE_TYPE_PRIMARY:
		if (supports_l10r) {
			fmts = dcp_primary_formats;
			num_fmts = ARRAY_SIZE(dcp_primary_formats);
		} else {
			fmts = dcp_primary_formats_12_x;
			num_fmts = ARRAY_SIZE(dcp_primary_formats_12_x);
		}
		plane = drmm_universal_plane_alloc(dev, struct apple_plane, base, possible_crtcs,
				       &apple_plane_funcs, fmts, num_fmts,
				       apple_format_modifiers, type, NULL);
		break;
	case DRM_PLANE_TYPE_OVERLAY:
	case DRM_PLANE_TYPE_CURSOR:
		if (supports_l10r) {
			fmts = dcp_overlay_formats;
			num_fmts = ARRAY_SIZE(dcp_overlay_formats);
		} else {
			fmts = dcp_overlay_formats_12_x;
			num_fmts = ARRAY_SIZE(dcp_overlay_formats_12_x);
		}
		plane = drmm_universal_plane_alloc(dev, struct apple_plane, base, possible_crtcs,
				       &apple_plane_funcs, fmts, num_fmts,
				       apple_format_modifiers, type, NULL);
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR(plane))
		return ERR_PTR(PTR_ERR(plane));

	drm_plane_create_color_properties(&plane->base,
					  (1 << DRM_COLOR_ENCODING_MAX) - 1,
					  (1 << DRM_COLOR_RANGE_MAX) - 1,
					  DRM_COLOR_YCBCR_BT709,
					  DRM_COLOR_YCBCR_LIMITED_RANGE);

	if (type == DRM_PLANE_TYPE_PRIMARY)
		drm_plane_helper_add(&plane->base, &apple_primary_plane_helper_funcs);
	else
		drm_plane_helper_add(&plane->base, &apple_plane_helper_funcs);

	plane->iomfb_surf = iomfb_surf;

	return &plane->base;
}
