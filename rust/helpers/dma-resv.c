// SPDX-License-Identifier: GPL-2.0

#include <linux/dma-resv.h>

int rust_helper_dma_resv_lock(struct dma_resv *obj, struct ww_acquire_ctx *ctx)
{
	return dma_resv_lock(obj, ctx);
}

void rust_helper_dma_resv_unlock(struct dma_resv *obj)
{
	dma_resv_unlock(obj);
}
