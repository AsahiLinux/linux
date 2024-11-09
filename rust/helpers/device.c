// SPDX-License-Identifier: GPL-2.0

#include <linux/device.h>

void *rust_helper_dev_get_drvdata(struct device *dev)
{
	return dev_get_drvdata(dev);
}

int rust_helper_devm_add_action(struct device *dev,
				void (*action)(void *),
				void *data)
{
	return devm_add_action(dev, action, data);
}

void rust_helper_device_lock(struct device *dev)
{
	device_lock(dev);
}

void rust_helper_device_unlock(struct device *dev)
{
	device_unlock(dev);
}
