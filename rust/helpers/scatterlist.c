// SPDX-License-Identifier: GPL-2.0

#include <linux/scatterlist.h>

dma_addr_t rust_helper_sg_dma_address(const struct scatterlist *sg)
{
	return sg_dma_address(sg);
}

int rust_helper_sg_dma_len(const struct scatterlist *sg)
{
	return sg_dma_len(sg);
}
