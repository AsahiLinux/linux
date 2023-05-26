// SPDX-License-Identifier: GPL-2.0-only OR MIT
#ifndef __LEAP_H_
#define __LEAP_H_

#include <linux/types.h>

struct leap_cluster;
struct leap_core;

int leap_io_put(struct leap_cluster *cl, int port, u32 val);
int leap_io_update(struct leap_cluster *cl, int port, u32 val);
int leap_io_take(struct leap_cluster *cl, int port, u32 *val);
int leap_io_peek(struct leap_cluster *cl, int port, u32 *val);

struct leap_core *leap_get_core(struct leap_cluster *cluster, int index);
int leap_load_image(struct leap_core *core, void *image, size_t size, bool state_only);

void leap_enable(struct leap_core *core);
void leap_disable(struct leap_core *core);

#endif /* __LEAP_H__ */
