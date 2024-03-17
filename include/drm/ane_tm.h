// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2022 Eileen Yoon <eyn@gmx.com> */

#ifndef __ANE_TM_H__
#define __ANE_TM_H__

#include "ane.h"

void ane_tm_enable(struct ane_device *ane);
int ane_tm_enqueue(struct ane_device *ane, struct ane_request *req);
int ane_tm_execute(struct ane_device *ane, struct ane_request *req);

#endif /* __ANE_TM_H__ */
