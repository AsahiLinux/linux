/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 Sofus Forstreuter <sofus.c@icloud.com>
 * Copyright (C) 2026 The Asahi Linux Contributors
 */

#ifndef __PIODMA_H__
#define __PIODMA_H__

#include <linux/platform_device.h>

enum piodma_completion_status {
	PIODMA_OK = 0,
	PIODMA_ERR
};

struct piodma_ctx;

/* TODO: there is no limit on resources */
struct piodma_ctx *piodma_init(struct platform_device * pdev,
			       const struct resource* res,
			       void (*cb)(void *prv,
					  enum piodma_completion_status status),
			       void *prv);
int piodma_start_packet(struct piodma_ctx *ctx);
int piodma_add_packet(struct piodma_ctx *ctx, size_t off, const u32 *data,
		      size_t count);
int piodma_submit_packet(struct piodma_ctx *ctx);
void piodma_free(struct piodma_ctx *ctx);

#endif
