// SPDX-License-Identifier: GPL-2.0

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

void *
rust_helper_platform_get_drvdata(const struct platform_device *pdev)
{
	return platform_get_drvdata(pdev);
}

void
rust_helper_platform_set_drvdata(struct platform_device *pdev,
                                void *data)
{
	platform_set_drvdata(pdev, data);
}

const struct of_device_id *rust_helper_of_match_device(
               const struct of_device_id *matches, const struct device *dev)
{
	return of_match_device(matches, dev);
}
