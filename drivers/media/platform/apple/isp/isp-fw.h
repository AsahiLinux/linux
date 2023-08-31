// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#ifndef __ISP_FW_H__
#define __ISP_FW_H__

#include "isp-drv.h"

int apple_isp_firmware_boot(struct apple_isp *isp);
void apple_isp_firmware_shutdown(struct apple_isp *isp);

#endif /* __ISP_FW_H__ */
