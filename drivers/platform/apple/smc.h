// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC internal core definitions
 * Copyright (C) The Asahi Linux Contributors
 */

#ifndef _SMC_H
#define _SMC_H

#include <linux/mfd/macsmc.h>

struct apple_smc_backend_ops {
	int (*read_key)(void *cookie, smc_key key, void *buf, size_t size);
	int (*write_key)(void *cookie, smc_key key, void *buf, size_t size);
	int (*write_key_atomic)(void *cookie, smc_key key, void *buf, size_t size);
	int (*rw_key)(void *cookie, smc_key key, void *wbuf, size_t wsize,
		      void *rbuf, size_t rsize);
	int (*get_key_by_index)(void *cookie, int index, smc_key *key);
	int (*get_key_info)(void *cookie, smc_key key, struct apple_smc_key_info *info);
};

int apple_smc_probe(struct device *dev, const struct apple_smc_backend_ops *ops, void *cookie);
void *apple_smc_get_cookie(struct apple_smc *smc);
int apple_smc_remove(struct apple_smc *smc);
void apple_smc_event_received(struct apple_smc *smc, uint32_t event);

#endif
