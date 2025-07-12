// SPDX-License-Identifier: GPL-2.0

#include <drm/drm_syncobj.h>

#ifdef CONFIG_DRM

void rust_helper_drm_syncobj_get(struct drm_syncobj *obj)
{
	drm_syncobj_get(obj);
}

void rust_helper_drm_syncobj_put(struct drm_syncobj *obj)
{
	drm_syncobj_put(obj);
}

struct dma_fence *rust_helper_drm_syncobj_fence_get(struct drm_syncobj *syncobj)
{
	return drm_syncobj_fence_get(syncobj);
}

#endif
