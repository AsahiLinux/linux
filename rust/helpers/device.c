// SPDX-License-Identifier: GPL-2.0

#include <linux/device.h>
#include <linux/dma-mapping.h>

void *rust_helper_dev_get_drvdata(struct device *dev)
{
	return dev_get_drvdata(dev);
}

const char *rust_helper_dev_name(const struct device *dev)
{
	return dev_name(dev);
}

int rust_helper_dma_set_mask_and_coherent(struct device *dev, u64 mask)
{
	return dma_set_mask_and_coherent(dev, mask);
}
