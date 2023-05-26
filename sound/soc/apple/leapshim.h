// SPDX-License-Identifier: GPL-2.0-only OR MIT
#ifndef __LEAPSHIM_H_
#define __LEAPSHIM_H_

struct dma_chan;
struct leapshim_data;

struct dma_chan *leapshim_request_dma_channel(struct leapshim_data *shim, int id);
struct platform_device *leapshim_find_leap_device(struct leapshim_data *shim);
void leapshim_enable_dma(struct leapshim_data *shim, int id);
void leapshim_disable_dma(struct leapshim_data *shim, int id);

#endif /* __LEAPSHIM_H_ */
