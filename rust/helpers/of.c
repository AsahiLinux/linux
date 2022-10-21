// SPDX-License-Identifier: GPL-2.0

#include <linux/of.h>
#include <linux/of_device.h>

const struct of_device_id *rust_helper_of_match_device(
		const struct of_device_id *matches, const struct device *dev)
{
			return of_match_device(matches, dev);
}

#ifdef CONFIG_OF
bool rust_helper_of_node_is_root(const struct device_node *np)
{
	return of_node_is_root(np);
}
#endif

struct device_node *rust_helper_of_parse_phandle(const struct device_node *np,
               const char *phandle_name,
               int index)
{
	return of_parse_phandle(np, phandle_name, index);
}
