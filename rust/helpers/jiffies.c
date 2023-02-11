// SPDX-License-Identifier: GPL-2.0

#include <linux/jiffies.h>

unsigned long rust_helper_msecs_to_jiffies(const unsigned int m)
{
	return msecs_to_jiffies(m);
}
