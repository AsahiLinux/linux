// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2022 Eileen Yoon <eyn@gmx.com> */

#ifndef __ANE_ACCEL_H__
#define __ANE_ACCEL_H__

#if defined(__cplusplus)
extern "C" {
#endif

#define ANE_TILE_COUNT	0x20
#define ANE_FIFO_NID	0x40
#define ANE_CMD_GRAN	0x10

#define DRM_ANE_BO_INIT 0x1
#define DRM_ANE_BO_FREE 0x2
#define DRM_ANE_SUBMIT	0x3

struct drm_ane_bo_init {
	__u32 handle;
	__u32 pad;
	__u64 size;
	__u64 offset;
};

struct drm_ane_bo_free {
	__u32 handle;
	__u32 pad;
};

struct drm_ane_submit {
	__u64 tsk_size;
	__u32 td_count;
	__u32 td_size;
	__u32 handles[ANE_TILE_COUNT];
	__u32 btsp_handle;
	__u32 pad;
};

#define DRM_IOCTL_ANE_BO_INIT \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_ANE_BO_INIT, struct drm_ane_bo_init)
#define DRM_IOCTL_ANE_BO_FREE \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_ANE_BO_FREE, struct drm_ane_bo_free)
#define DRM_IOCTL_ANE_SUBMIT \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_ANE_SUBMIT, struct drm_ane_submit)

#if defined(__cplusplus)
}
#endif

#endif /* __ANE_ACCEL_H__ */
