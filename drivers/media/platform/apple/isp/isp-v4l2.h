// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#ifndef __ISP_V4L2_H__
#define __ISP_V4L2_H__

#include "isp-drv.h"

int apple_isp_setup_video(struct apple_isp *isp);
void apple_isp_remove_video(struct apple_isp *isp);
int ipc_bt_handle(struct apple_isp *isp, struct isp_channel *chan);

int apple_isp_video_suspend(struct apple_isp *isp);
int apple_isp_video_resume(struct apple_isp *isp);

#endif /* __ISP_V4L2_H__ */
