// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2023 Daniel Berlin
 */

#ifndef _BRCMF_INTERFACE_CREATE_H_
#define _BRCMF_INTERFACE_CREATE_H_
#include "core.h"

int brcmf_cfg80211_request_sta_if(struct brcmf_if *ifp, u8 *macaddr);
int brcmf_cfg80211_request_ap_if(struct brcmf_if *ifp);

#endif /* _BRCMF_INTERFACE_CREATE_H_ */
