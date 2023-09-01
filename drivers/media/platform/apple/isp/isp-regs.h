// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#ifndef __ISP_REGS_H__
#define __ISP_REGS_H__

#include "isp-drv.h"

#define ISP_ASC_PMGR_0	       0x738
#define ISP_ASC_PMGR_1	       0x798
#define ISP_ASC_PMGR_2	       0x7f8
#define ISP_ASC_PMGR_3	       0x858

#define ISP_ASC_RVBAR	       0x1050000
#define ISP_ASC_EDPRCR	       0x1010310
#define ISP_ASC_CONTROL	       0x1400044
#define ISP_ASC_STATUS	       0x1400048

#define ISP_ASC_IRQ_MASK_0     0x1400a00
#define ISP_ASC_IRQ_MASK_1     0x1400a04
#define ISP_ASC_IRQ_MASK_2     0x1400a08
#define ISP_ASC_IRQ_MASK_3     0x1400a0c
#define ISP_ASC_IRQ_MASK_4     0x1400a10
#define ISP_ASC_IRQ_MASK_5     0x1400a14

#define ISP_MBOX_IRQ_INTERRUPT 0x000
#define ISP_MBOX_IRQ_ENABLE    0x004
#define ISP_MBOX_IRQ_DOORBELL  0x3f0
#define ISP_MBOX_IRQ_ACK       0x3fc

#define ISP_GPIO_0	       0x00
#define ISP_GPIO_1	       0x04
#define ISP_GPIO_2	       0x08
#define ISP_GPIO_3	       0x0c
#define ISP_GPIO_4	       0x10
#define ISP_GPIO_5	       0x14
#define ISP_GPIO_6	       0x18
#define ISP_GPIO_7	       0x1c
#define ISP_GPIO_CLOCK_EN      0x20

static inline u32 isp_mbox_read32(struct apple_isp *isp, u32 reg)
{
	return readl(isp->mbox + reg);
}

static inline void isp_mbox_write32(struct apple_isp *isp, u32 reg, u32 val)
{
	writel(val, isp->mbox + reg);
}

#endif /* __ISP_REGS_H__ */
