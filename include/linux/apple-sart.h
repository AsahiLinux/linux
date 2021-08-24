// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SART device driver
 * Copyright (C) 2021 The Asahi Linux Contributors
 *
 * Apple SART is a simple address filter for DMA transactions.
 * Regions of physical memory must be added to the SART's allow
 * list before before any DMA can target these. Unlike a proper
 * IOMMU no remapping can be done.
 */

#ifndef _LINUX_APPLE_SART_H_
#define _LINUX_APPLE_SART_H_

#include <linux/device.h>
#include <linux/err.h>
#include <linux/types.h>

struct apple_sart;

#ifdef CONFIG_APPLE_SART

struct apple_sart *apple_sart_get(struct device *dev);
int apple_sart_add_allowed_region(struct apple_sart *sart, phys_addr_t paddr,
				  size_t size);
int apple_sart_remove_allowed_region(struct apple_sart *sart, phys_addr_t paddr,
				     size_t size);

#else

static inline struct apple_sart *apple_sart_get(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

static inline int apple_sart_add_allowed_region(struct apple_sart *sart,
						phys_addr_t paddr, size_t size)
{
	return -ENODEV;
}

static inline int apple_sart_remove_allowed_region(struct apple_sart *sart,
						   phys_addr_t paddr,
						   size_t size)
{
	return -ENODEV;
}

#endif

#endif
