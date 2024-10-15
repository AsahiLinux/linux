// SPDX-License-Identifier: GPL-2.0

#include <linux/dma-fence.h>
#include <linux/dma-fence-chain.h>

#ifdef CONFIG_DMA_SHARED_BUFFER

void rust_helper_dma_fence_get(struct dma_fence *fence)
{
	dma_fence_get(fence);
}

void rust_helper_dma_fence_put(struct dma_fence *fence)
{
	dma_fence_put(fence);
}

struct dma_fence_chain *rust_helper_dma_fence_chain_alloc(void)
{
	return dma_fence_chain_alloc();
}

void rust_helper_dma_fence_chain_free(struct dma_fence_chain *chain)
{
	dma_fence_chain_free(chain);
}

void rust_helper_dma_fence_set_error(struct dma_fence *fence, int error)
{
	dma_fence_set_error(fence, error);
}

#endif
