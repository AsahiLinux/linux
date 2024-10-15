// SPDX-License-Identifier: GPL-2.0

#include <linux/ioport.h>

resource_size_t rust_helper_resource_size(const struct resource *res)
{
	return resource_size(res);
}
