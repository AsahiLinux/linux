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

#define ISP_CORE_IRQ_INTERRUPT 0x2104000
#define ISP_CORE_IRQ_ENABLE    0x2104004
#define ISP_CORE_IRQ_DOORBELL  0x21043f0
#define ISP_CORE_IRQ_ACK       0x21043fc

#define ISP_CORE_GPIO_0	       0x2104170
#define ISP_CORE_GPIO_1	       0x2104174
#define ISP_CORE_GPIO_2	       0x2104178
#define ISP_CORE_GPIO_3	       0x210417c
#define ISP_CORE_GPIO_4	       0x2104180
#define ISP_CORE_GPIO_5	       0x2104184
#define ISP_CORE_GPIO_6	       0x2104188
#define ISP_CORE_GPIO_7	       0x210418c

#define ISP_CORE_CLOCK_EN      0x2104190

#define ISP_CORE_DPE_CTRL_0    0x2504000
#define ISP_CORE_DPE_CTRL_1    0x2508000

static inline u32 isp_core_read32(struct apple_isp *isp, u32 reg)
{
	return readl(isp->core + reg - 0x2104000); // TODO this sucks
}

static inline void isp_core_write32(struct apple_isp *isp, u32 reg, u32 val)
{
	writel(val, isp->core + reg - 0x2104000);
}

static inline void isp_core_mask32(struct apple_isp *isp, u32 reg, u32 clear,
				   u32 set)
{
	isp_core_write32(isp, reg, isp_core_read32(isp, reg) & ~clear);
	isp_core_write32(isp, reg, isp_core_read32(isp, reg) | set);
}

#endif /* __ISP_REGS_H__ */
