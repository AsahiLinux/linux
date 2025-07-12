// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#ifndef __ISP_IOMMU_H__
#define __ISP_IOMMU_H__

#include "isp-drv.h"

struct isp_surf *__isp_alloc_surface(struct apple_isp *isp, u64 size, bool gc);
#define isp_alloc_surface(isp, size)	(__isp_alloc_surface(isp, size, false))
#define isp_alloc_surface_gc(isp, size) (__isp_alloc_surface(isp, size, true))
struct isp_surf *isp_alloc_surface_vmap(struct apple_isp *isp, u64 size);
int isp_surf_vmap(struct apple_isp *isp, struct isp_surf *surf);
void isp_free_surface(struct apple_isp *isp, struct isp_surf *surf);

int apple_isp_iommu_map_sgt(struct apple_isp *isp, struct isp_surf *surf,
			    struct sg_table *sgt, u64 size);
void apple_isp_iommu_unmap_sgt(struct apple_isp *isp, struct isp_surf *surf);

#endif /* __ISP_IOMMU_H__ */
