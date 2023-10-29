// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#ifndef __ISP_FW_H__
#define __ISP_FW_H__

#include "isp-drv.h"

int apple_isp_alloc_firmware_surface(struct apple_isp *isp);
void apple_isp_free_firmware_surface(struct apple_isp *isp);

int apple_isp_firmware_boot(struct apple_isp *isp);
void apple_isp_firmware_shutdown(struct apple_isp *isp);

void *apple_isp_translate(struct apple_isp *isp, struct isp_surf *surf,
			  dma_addr_t iova, size_t size);

static inline void *apple_isp_ipc_translate(struct apple_isp *isp,
					    dma_addr_t iova, size_t size)
{
	return apple_isp_translate(isp, isp->ipc_surf, iova, size);
}

#endif /* __ISP_FW_H__ */
