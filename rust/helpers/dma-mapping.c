// SPDX-License-Identifier: GPL-2.0

#include <linux/dma-mapping.h>

int rust_helper_dma_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	return dma_mapping_error(dev, dma_addr);
}
