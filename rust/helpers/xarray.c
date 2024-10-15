
// SPDX-License-Identifier: GPL-2.0

#include <linux/xarray.h>

void rust_helper_xa_init_flags(struct xarray *xa, gfp_t flags)
{
	xa_init_flags(xa, flags);
}

bool rust_helper_xa_empty(struct xarray *xa)
{
	return xa_empty(xa);
}

int rust_helper_xa_alloc(struct xarray *xa, u32 *id, void *entry, struct xa_limit limit, gfp_t gfp)
{
	return xa_alloc(xa, id, entry, limit, gfp);
}

void rust_helper_xa_lock(struct xarray *xa)
{
	xa_lock(xa);
}

void rust_helper_xa_unlock(struct xarray *xa)
{
	xa_unlock(xa);
}

int rust_helper_xa_err(void *entry)
{
	return xa_err(entry);
}
