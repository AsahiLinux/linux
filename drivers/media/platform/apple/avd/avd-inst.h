/* SPDX-License-Identifier: MIT */

#ifndef AVD_INST_H_
#define AVD_INST_H_
/* instruction stream things  */
#include <linux/types.h>

#include "avd.h"

/* i have no clue what this is */
#define INST_DMA1 0 /* (0x14 << 16 | 0x14) */
#define INST_DMA2 0 /* (0x4000000 | INST_DMA1) */
#define INST_DMA3 0 /* (0x07 << 16 | 0x07) */

static inline u32 fifo_size(void)
{
	return 0x100000 * 12;
}

static inline u32 swrap(u32 x, u32 w)
{
	return x & (w - 1);
}
static inline bool boolify(u32 v)
{
	return !!(v);
}

static inline void push(struct avd_dev *avd, struct avd_ctx *ctx, u32 inst)
{
	writel(inst, avd->ctrl + avd->variant->vp_slot_offset + ctx->vp_slot * 4);
}

static inline void push_address(struct avd_dev *avd, struct avd_ctx *ctx,
				dma_addr_t addr)
{
	if (avd->variant->quirks & AVD_QUIRK_LSR) {
		push(avd, ctx, (addr >> 8));
	} else {
		push(avd, ctx, (u32)(addr & 0xffffffff));
		push(avd, ctx, (u32)(addr >> 32));
	}
}
static inline void push_rvra(struct avd_dev *avd, struct avd_ctx *ctx,
		dma_addr_t addr, u32 offsets[4])
{
	if (avd->variant->quirks & AVD_QUIRK_LSR) {
		for (int i = 0; i < 4; i++)
			push(avd, ctx, (addr + offsets[i]) >> 7);
	} else {
		for (int i = 0; i < 4; i++)
			push_address(avd, ctx, (addr + offsets[i]));
	}
}

#ifdef DEBUG_INST
#define push(inst, name)                                           \
	do {                                                       \
		dev_info(ctx->dev->dev, "%8x | %s", (inst), name); \
		push(avd, ctx, inst);                              \
	} while (0)

#else
#define push(inst, name) push(avd, ctx, inst)
#endif

#ifdef DEBUG_INST_ADDR
#define pusha(inst, name, i)                                                   \
	do {                                                                   \
		dev_info(ctx->dev->dev, "%8llx | %s[%d]", (inst) & 0xffffffff, \
			 name, i);                                             \
		dev_info(ctx->dev->dev, "%8llx | %s[%d] (high)", (inst) >> 32, \
			 name, i);                                             \
		push_address(avd, ctx, inst);                                  \
	} while (0)

#else
#define pusha(inst, name, i) push_address(avd, ctx, inst)
#endif

#endif /* AVD_INST_H_ */
