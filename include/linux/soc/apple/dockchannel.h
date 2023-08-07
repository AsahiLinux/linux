/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple Dockchannel devices
 * Copyright (C) The Asahi Linux Contributors
 */
#ifndef _LINUX_APPLE_DOCKCHANNEL_H_
#define _LINUX_APPLE_DOCKCHANNEL_H_

#include <linux/device.h>
#include <linux/types.h>
#include <linux/of_platform.h>

#if IS_ENABLED(CONFIG_APPLE_DOCKCHANNEL)

struct dockchannel;

struct dockchannel *dockchannel_init(struct platform_device *pdev);

int dockchannel_send(struct dockchannel *dockchannel, const void *buf, size_t count);
int dockchannel_recv(struct dockchannel *dockchannel, void *buf, size_t count);
int dockchannel_await(struct dockchannel *dockchannel,
		      void (*callback)(void *cookie, size_t avail),
		      void *cookie, size_t count);

#endif
#endif
