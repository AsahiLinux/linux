// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#ifndef __ISP_IPC_H__
#define __ISP_IPC_H__

#include "isp-drv.h"

#define ISP_IPC_CHAN_TYPE_COMMAND   0
#define ISP_IPC_CHAN_TYPE_REPLY	    1
#define ISP_IPC_CHAN_TYPE_REPORT    2

#define ISP_IPC_BUFEXC_STAT_SIZE    0x280
#define ISP_IPC_BUFEXC_FLAG_RENDER  0x10000000
#define ISP_IPC_BUFEXC_FLAG_COMMAND 0x30000000
#define ISP_IPC_BUFEXC_FLAG_ACK	    0x80000000

int ipc_chan_handle(struct apple_isp *isp, struct isp_channel *chan);
int ipc_chan_send(struct apple_isp *isp, struct isp_channel *chan,
		  unsigned long timeout);

int ipc_tm_handle(struct apple_isp *isp, struct isp_channel *chan);
int ipc_sm_handle(struct apple_isp *isp, struct isp_channel *chan);
int ipc_bt_handle(struct apple_isp *isp, struct isp_channel *chan);

#endif /* __ISP_IPC_H__ */
