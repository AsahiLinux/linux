// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#include <linux/iommu.h>
#include <linux/vmalloc.h>

#include "isp-iommu.h"

static void isp_surf_free_pages(struct isp_surf *surf)
{
	for (u32 i = 0; i < surf->num_pages && surf->pages[i] != NULL; i++) {
		__free_page(surf->pages[i]);
	}
	kvfree(surf->pages);
}

static int isp_surf_alloc_pages(struct isp_surf *surf)
{
	surf->pages = kvmalloc_array(surf->num_pages, sizeof(*surf->pages),
				     GFP_KERNEL);
	if (!surf->pages)
		return -ENOMEM;

	for (u32 i = 0; i < surf->num_pages; i++) {
		surf->pages[i] = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (surf->pages[i] == NULL)
			goto free_pages;
	}

	return 0;

free_pages:
	isp_surf_free_pages(surf);
	return -ENOMEM;
}

int isp_surf_vmap(struct apple_isp *isp, struct isp_surf *surf)
{
	surf->virt = vmap(surf->pages, surf->num_pages, VM_MAP,
			  pgprot_writecombine(PAGE_KERNEL));
	if (surf->virt == NULL) {
		dev_err(isp->dev, "failed to vmap size 0x%llx\n", surf->size);
		return -EINVAL;
	}

	return 0;
}

static void isp_surf_vunmap(struct apple_isp *isp, struct isp_surf *surf)
{
	if (surf->virt)
		vunmap(surf->virt);
	surf->virt = NULL;
}

static void isp_surf_unreserve_iova(struct apple_isp *isp,
				    struct isp_surf *surf)
{
	if (surf->mm) {
		mutex_lock(&isp->iovad_lock);
		drm_mm_remove_node(surf->mm);
		mutex_unlock(&isp->iovad_lock);
		kfree(surf->mm);
	}
	surf->mm = NULL;
}

static int isp_surf_reserve_iova(struct apple_isp *isp, struct isp_surf *surf)
{
	int err;

	surf->mm = kzalloc(sizeof(*surf->mm), GFP_KERNEL);
	if (!surf->mm)
		return -ENOMEM;

	mutex_lock(&isp->iovad_lock);
	err = drm_mm_insert_node_generic(&isp->iovad, surf->mm,
					 ALIGN(surf->size, 1UL << isp->shift),
					 1UL << isp->shift, 0, 0);
	mutex_unlock(&isp->iovad_lock);
	if (err < 0) {
		dev_err(isp->dev, "failed to reserve 0x%llx of iova space\n",
			surf->size);
		goto mm_free;
	}

	surf->iova = surf->mm->start;

	return 0;
mm_free:
	kfree(surf->mm);
	surf->mm = NULL;
	return err;
}

static void isp_surf_iommu_unmap(struct apple_isp *isp, struct isp_surf *surf)
{
	iommu_unmap(isp->domain, surf->iova, surf->size);
	sg_free_table(&surf->sgt);
}

static int isp_surf_iommu_map(struct apple_isp *isp, struct isp_surf *surf)
{
	unsigned long size;
	int err;

	err = sg_alloc_table_from_pages(&surf->sgt, surf->pages,
					surf->num_pages, 0, surf->size,
					GFP_KERNEL);
	if (err < 0) {
		dev_err(isp->dev, "failed to alloc sgt from pages\n");
		return err;
	}

	size = iommu_map_sgtable(isp->domain, surf->iova, &surf->sgt,
				 IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE);
	if (size < surf->size) {
		dev_err(isp->dev, "failed to iommu_map sgt to iova 0x%llx\n",
			surf->iova);
		sg_free_table(&surf->sgt);
		return -ENXIO;
	}

	return 0;
}

static void __isp_surf_init(struct apple_isp *isp, struct isp_surf *surf,
			    u64 size, bool gc)
{
	surf->mm = NULL;
	surf->virt = NULL;
	surf->size = ALIGN(size, 1UL << isp->shift);
	surf->num_pages = surf->size >> isp->shift;
	surf->gc = gc;
}

struct isp_surf *__isp_alloc_surface(struct apple_isp *isp, u64 size, bool gc)
{
	int err;

	struct isp_surf *surf = kzalloc(sizeof(struct isp_surf), GFP_KERNEL);
	if (!surf)
		return NULL;

	__isp_surf_init(isp, surf, size, gc);

	err = isp_surf_alloc_pages(surf);
	if (err < 0) {
		dev_err(isp->dev, "failed to allocate %d pages\n",
			surf->num_pages);
		goto free_surf;
	}

	err = isp_surf_reserve_iova(isp, surf);
	if (err < 0) {
		dev_err(isp->dev, "failed to reserve 0x%llx of iova space\n",
			surf->size);
		goto free_pages;
	}

	err = isp_surf_iommu_map(isp, surf);
	if (err < 0) {
		dev_err(isp->dev,
			"failed to iommu_map size 0x%llx to iova 0x%llx\n",
			surf->size, surf->iova);
		goto unreserve_iova;
	}

	refcount_set(&surf->refcount, 1);
	if (surf->gc)
		list_add_tail(&surf->head, &isp->gc);

	return surf;

unreserve_iova:
	isp_surf_unreserve_iova(isp, surf);
free_pages:
	isp_surf_free_pages(surf);
free_surf:
	kfree(surf);
	return NULL;
}

struct isp_surf *isp_alloc_surface_vmap(struct apple_isp *isp, u64 size)
{
	int err;

	struct isp_surf *surf = __isp_alloc_surface(isp, size, false);
	if (!surf)
		return NULL;

	err = isp_surf_vmap(isp, surf);
	if (err < 0) {
		dev_err(isp->dev, "failed to vmap iova 0x%llx - 0x%llx\n",
			surf->iova, surf->iova + surf->size);
		isp_free_surface(isp, surf);
		return NULL;
	}

	return surf;
}

void isp_free_surface(struct apple_isp *isp, struct isp_surf *surf)
{
	if (refcount_dec_and_test(&surf->refcount)) {
		isp_surf_vunmap(isp, surf);
		isp_surf_iommu_unmap(isp, surf);
		isp_surf_unreserve_iova(isp, surf);
		isp_surf_free_pages(surf);
		if (surf->gc)
			list_del(&surf->head);
		kfree(surf);
	}
}

int apple_isp_iommu_map_sgt(struct apple_isp *isp, struct isp_surf *surf,
			    struct sg_table *sgt, u64 size)
{
	int err;
	ssize_t mapped;

	// TODO userptr sends unaligned sizes
	surf->mm = NULL;
	surf->size = size;

	err = isp_surf_reserve_iova(isp, surf);
	if (err < 0) {
		dev_err(isp->dev, "failed to reserve 0x%llx of iova space\n",
			surf->size);
		return err;
	}

	mapped = iommu_map_sgtable(isp->domain, surf->iova, sgt,
				   IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE);
	if (mapped < surf->size) {
		dev_err(isp->dev, "failed to iommu_map sgt to iova 0x%llx\n",
			surf->iova);
		isp_surf_unreserve_iova(isp, surf);
		return -ENXIO;
	}
	surf->size = mapped;

	return 0;
}

void apple_isp_iommu_unmap_sgt(struct apple_isp *isp, struct isp_surf *surf)
{
	iommu_unmap(isp->domain, surf->iova, surf->size);
	isp_surf_unreserve_iova(isp, surf);
}
