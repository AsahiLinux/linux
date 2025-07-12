// SPDX-License-Identifier: GPL-2.0

#include <linux/siphash.h>

u64 rust_helper_siphash(const void *data, size_t len,
			const siphash_key_t *key)
{
	    return siphash(data, len, key);
}
