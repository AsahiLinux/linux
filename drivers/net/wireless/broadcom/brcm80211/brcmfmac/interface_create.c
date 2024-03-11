// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2023 Daniel Berlin
 */

/* This file handles firmware-side interface creation */

#include <linux/kernel.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <net/cfg80211.h>
#include <net/netlink.h>

#include <brcmu_utils.h>
#include <defs.h>
#include "cfg80211.h"
#include "debug.h"
#include "fwil.h"
#include "proto.h"
#include "bus.h"
#include "common.h"
#include "interface_create.h"

#define BRCMF_INTERFACE_CREATE_VER_1 1
#define BRCMF_INTERFACE_CREATE_VER_2 2
#define BRCMF_INTERFACE_CREATE_VER_3 3
#define BRCMF_INTERFACE_CREATE_VER_MAX BRCMF_INTERFACE_CREATE_VER_3

/* These sets of flags specify whether to use various fields in the interface create structures */

/* This is only used with version 0 or 1 */
#define BRCMF_INTERFACE_CREATE_STA (0 << 0)
#define BRCMF_INTERFACE_CREATE_AP (1 << 0)

#define BRCMF_INTERFACE_MAC_DONT_USE (0 << 1)
#define BRCMF_INTERFACE_MAC_USE (1 << 1)

#define BRCMF_INTERFACE_WLC_INDEX_DONT_USE (0 << 2)
#define BRCMF_INTERFACE_WLC_INDEX_USE (1 << 2)

#define BRCMF_INTERFACE_IF_INDEX_DONT_USE (0 << 3)
#define BRCMF_INTERFACE_IF_INDEX_USE (1 << 3)

#define BRCMF_INTERFACE_BSSID_DONT_USE (0 << 4)
#define BRCMF_INTERFACE_BSSID_USE (1 << 4)

/*
 * From revision >= 2 Bit 0 of flags field will not be used  for STA or AP interface creation.
 * "iftype" field shall be used for identifying the interface type.
 */
enum brcmf_interface_type {
	BRCMF_INTERFACE_TYPE_STA = 0,
	BRCMF_INTERFACE_TYPE_AP = 1,
	/* The missing number here is deliberate */
	BRCMF_INTERFACE_TYPE_NAN = 3,
	BRCMF_INTERFACE_TYPE_P2P_GO = 4,
	BRCMF_INTERFACE_TYPE_P2P_GC = 5,
	BRCMF_INTERFACE_TYPE_P2P_DISC = 6,
	BRCMF_INTERFACE_TYPE_IBSS = 7,
	BRCMF_INTERFACE_TYPE_MESH = 8
};


/* All sources treat these structures as being host endian.
 * However, firmware treats it as little endian, so we do as well */

struct brcmf_interface_create_v1 {
	__le16 ver; /* structure version */
	u8 pad1[2];
	__le32 flags; /* flags for operation */
	u8 mac_addr[ETH_ALEN]; /* MAC address */
	u8 pad2[2];
	__le32 wlc_index; /* optional for wlc index */
};

struct brcmf_interface_create_v2 {
	__le16 ver; /* structure version */
	u8 pad1[2];
	__le32 flags; /* flags for operation */
	u8 mac_addr[ETH_ALEN]; /* MAC address */
	u8 iftype; /* type of interface created */
	u8 pad2;
	u32 wlc_index; /* optional for wlc index */
};

struct brcmf_interface_create_v3 {
	__le16 ver; /* structure version */
	__le16 len; /* length of structure + data */
	__le16 fixed_len; /* length of structure */
	u8 iftype; /* type of interface created */
	u8 wlc_index; /* optional for wlc index */
	__le32 flags; /* flags for operation */
	u8 mac_addr[ETH_ALEN]; /* MAC address */
	u8 bssid[ETH_ALEN]; /* optional for BSSID */
	u8 if_index; /* interface index request */
	u8 pad[3];
	u8 data[]; /* Optional for specific data */
};

static int brcmf_get_first_free_bsscfgidx(struct brcmf_pub *drvr)
{
	int bsscfgidx;

	for (bsscfgidx = 0; bsscfgidx < BRCMF_MAX_IFS; bsscfgidx++) {
		/* bsscfgidx 1 is reserved for legacy P2P */
		if (bsscfgidx == 1)
			continue;
		if (!drvr->iflist[bsscfgidx])
			return bsscfgidx;
	}

	return -ENOMEM;
}

static void brcmf_set_vif_sta_macaddr(struct brcmf_if *ifp, u8 *mac_addr)
{
	u8 mac_idx = ifp->drvr->sta_mac_idx;

	/* set difference MAC address with locally administered bit */
	memcpy(mac_addr, ifp->mac_addr, ETH_ALEN);
	mac_addr[0] |= 0x02;
	mac_addr[3] ^= mac_idx ? 0xC0 : 0xA0;
	mac_idx++;
	mac_idx = mac_idx % 2;
	ifp->drvr->sta_mac_idx = mac_idx;
}

static int brcmf_cfg80211_request_if_internal(struct brcmf_if *ifp, u32 version,
					      enum brcmf_interface_type if_type,
					      u8 *macaddr)
{
	switch (version) {
	case BRCMF_INTERFACE_CREATE_VER_1: {
		struct brcmf_interface_create_v1 iface_v1 = {};
		u32 flags = if_type;

		iface_v1.ver = cpu_to_le16(BRCMF_INTERFACE_CREATE_VER_1);
		if (macaddr) {
			flags |= BRCMF_INTERFACE_MAC_USE;
			if (!is_zero_ether_addr(macaddr))
				memcpy(iface_v1.mac_addr, macaddr, ETH_ALEN);
			else
				brcmf_set_vif_sta_macaddr(ifp,
							  iface_v1.mac_addr);
		}
		iface_v1.flags = cpu_to_le32(flags);
		return brcmf_fil_iovar_data_get(ifp, "interface_create",
						&iface_v1, sizeof(iface_v1));
	}
	case BRCMF_INTERFACE_CREATE_VER_2: {
		struct brcmf_interface_create_v2 iface_v2 = {};
		u32 flags = 0;

		iface_v2.ver = cpu_to_le16(BRCMF_INTERFACE_CREATE_VER_2);
		iface_v2.iftype = if_type;
		if (macaddr) {
			flags = BRCMF_INTERFACE_MAC_USE;
			if (!is_zero_ether_addr(macaddr))
				memcpy(iface_v2.mac_addr, macaddr, ETH_ALEN);
			else
				brcmf_set_vif_sta_macaddr(ifp,
							  iface_v2.mac_addr);
		}
		iface_v2.flags = cpu_to_le32(flags);
		return brcmf_fil_iovar_data_get(ifp, "interface_create",
						&iface_v2, sizeof(iface_v2));
	}
	case BRCMF_INTERFACE_CREATE_VER_3: {
		struct brcmf_interface_create_v3 iface_v3 = {};
		u32 flags = 0;

		iface_v3.ver = cpu_to_le16(BRCMF_INTERFACE_CREATE_VER_3);
		iface_v3.iftype = if_type;
		iface_v3.len = cpu_to_le16(sizeof(iface_v3));
		iface_v3.fixed_len = cpu_to_le16(sizeof(iface_v3));
		if (macaddr) {
			flags = BRCMF_INTERFACE_MAC_USE;
			if (!is_zero_ether_addr(macaddr))
				memcpy(iface_v3.mac_addr, macaddr, ETH_ALEN);
			else
				brcmf_set_vif_sta_macaddr(ifp,
							  iface_v3.mac_addr);
		}
		iface_v3.flags = cpu_to_le32(flags);
		return brcmf_fil_iovar_data_get(ifp, "interface_create",
						&iface_v3, sizeof(iface_v3));
	}
	default:
		bphy_err(ifp->drvr, "Unknown interface create version:%d\n",
			 version);
		return -EINVAL;
	}
}
static int brcmf_cfg80211_request_if(struct brcmf_if *ifp,
				     enum brcmf_interface_type if_type,
				     u8 *macaddr)
{
	s32 err;
	u32 iface_create_ver;

	/* Query the creation version, see if the firmware knows */
	iface_create_ver = 0;
	err = brcmf_fil_bsscfg_int_get(ifp, "interface_create",
				       &iface_create_ver);
	if (!err) {
		err = brcmf_cfg80211_request_if_internal(ifp, iface_create_ver,
							 if_type, macaddr);
		if (!err) {
			brcmf_info("interface created (version %d)\n",
				   iface_create_ver);
		} else {
			bphy_err(ifp->drvr,
				 "failed to create interface (version %d):%d\n",
				 iface_create_ver, err);
		}
		return err;
	}
	/* Either version one or version two */
	err = brcmf_cfg80211_request_if_internal(
		ifp, if_type, BRCMF_INTERFACE_CREATE_VER_2, macaddr);
	if (!err) {
		brcmf_info("interface created (version 2)\n");
		return 0;
	}
	err = brcmf_cfg80211_request_if_internal(
		ifp, if_type, BRCMF_INTERFACE_CREATE_VER_1, macaddr);
	if (!err) {
		brcmf_info("interface created (version 1)\n");
		return 0;
	}
	bphy_err(ifp->drvr,
		 "interface creation failed, tried query, v2, v1: %d\n", err);
	return -EINVAL;
}

int brcmf_cfg80211_request_sta_if(struct brcmf_if *ifp, u8 *macaddr)
{
	return brcmf_cfg80211_request_if(ifp, BRCMF_INTERFACE_TYPE_STA,
					 macaddr);
}

int brcmf_cfg80211_request_ap_if(struct brcmf_if *ifp)
{
	int err;

	err = brcmf_cfg80211_request_if(ifp, BRCMF_INTERFACE_TYPE_AP, NULL);
	if (err) {
		struct brcmf_mbss_ssid_le mbss_ssid_le;
		int bsscfgidx;

		brcmf_info("Does not support interface_create (%d)\n", err);
		memset(&mbss_ssid_le, 0, sizeof(mbss_ssid_le));
		bsscfgidx = brcmf_get_first_free_bsscfgidx(ifp->drvr);
		if (bsscfgidx < 0)
			return bsscfgidx;

		mbss_ssid_le.bsscfgidx = cpu_to_le32(bsscfgidx);
		mbss_ssid_le.SSID_len = cpu_to_le32(5);
		sprintf(mbss_ssid_le.SSID, "ssid%d", bsscfgidx);

		err = brcmf_fil_bsscfg_data_set(ifp, "bsscfg:ssid",
						&mbss_ssid_le,
						sizeof(mbss_ssid_le));

		if (err < 0)
			bphy_err(ifp->drvr, "setting ssid failed %d\n", err);
	}
	return err;
}
