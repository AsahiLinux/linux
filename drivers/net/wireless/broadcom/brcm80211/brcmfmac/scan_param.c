// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2023 Daniel Berlin
 */
#include <linux/gcd.h>
#include <net/cfg80211.h>

#include "core.h"
#include "debug.h"
#include "fwil_types.h"
#include "cfg80211.h"
#include "scan_param.h"

static void brcmf_scan_param_set_defaults(u8 (*bssid)[ETH_ALEN], s8 *bss_type, __le32 *channel_num,
					  __le32 *nprobes, __le32 *active_time,
					  __le32 *passive_time,
					  __le32 *home_time)
{
	eth_broadcast_addr(*bssid);
	*bss_type = DOT11_BSSTYPE_ANY;
	*channel_num = 0;
	*nprobes = cpu_to_le32(-1);
	*active_time = cpu_to_le32(-1);
	*passive_time = cpu_to_le32(-1);
	*home_time = cpu_to_le32(-1);
}

static void brcmf_scan_param_copy_chanspecs(
	struct brcmf_cfg80211_info *cfg, __le16 (*dest_channels)[],
	struct ieee80211_channel **in_channels, u32 n_channels)
{
	int i;
	for (i = 0; i < n_channels; i++) {
		u32 chanspec =
			channel_to_chanspec(&cfg->d11inf, in_channels[i]);
		brcmf_dbg(SCAN, "Chan : %d, Channel spec: %x\n",
			  in_channels[i]->hw_value, chanspec);
		(*dest_channels)[i] = cpu_to_le16(chanspec);
	}
}

static void brcmf_scan_param_copy_ssids(char *dest_ssids,
					struct cfg80211_ssid *in_ssids,
					u32 n_ssids)
{
	int i;
	for (i = 0; i < n_ssids; i++) {
		struct brcmf_ssid_le ssid_le;
		memset(&ssid_le, 0, sizeof(ssid_le));
		ssid_le.SSID_len = cpu_to_le32(in_ssids[i].ssid_len);
		memcpy(ssid_le.SSID, in_ssids[i].ssid, in_ssids[i].ssid_len);
		if (!ssid_le.SSID_len)
			brcmf_dbg(SCAN, "%d: Broadcast scan\n", i);
		else
			brcmf_dbg(SCAN, "%d: scan for  %.32s size=%d\n", i,
				  ssid_le.SSID, ssid_le.SSID_len);
		memcpy(dest_ssids, &ssid_le, sizeof(ssid_le));
		dest_ssids += sizeof(ssid_le);
	}
}

/* The scan parameter structures have an array of SSID's that appears at the end in some cases.
 * In these cases, the chan list is really the lower half of a pair, the upper half is a ssid number,
 * and then after all of that there is an array of SSIDs */
static u32
brcmf_scan_param_tail_size(const struct cfg80211_scan_request *request,
			   u32 params_size)
{
	if (request != NULL) {
		/* Allocate space for populating ssid upper half in struct */
		params_size += sizeof(u32) * ((request->n_channels + 1) / 2);
		/* Allocate space for populating ssids in struct */
		params_size += sizeof(struct brcmf_ssid_le) * request->n_ssids;
	} else {
		params_size += sizeof(u16);
	}
	return params_size;
}

static u32 brcmf_nl80211_scan_flags_to_scan_flags(u32 nl80211_flags)
{
	u32 scan_flags = 0;
	if (nl80211_flags & NL80211_SCAN_FLAG_LOW_SPAN) {
		scan_flags |= BRCMF_SCANFLAGS_LOW_SPAN;
		brcmf_dbg(SCAN, "requested low span scan\n");
	}
	if (nl80211_flags & NL80211_SCAN_FLAG_HIGH_ACCURACY) {
		scan_flags |= BRCMF_SCANFLAGS_HIGH_ACCURACY;
		brcmf_dbg(SCAN, "requested high accuracy scan\n");
	}
	if (nl80211_flags & NL80211_SCAN_FLAG_LOW_POWER) {
		scan_flags |= BRCMF_SCANFLAGS_LOW_POWER;
		brcmf_dbg(SCAN, "requested low power scan\n");
	}
	if (nl80211_flags & NL80211_SCAN_FLAG_LOW_PRIORITY) {
		scan_flags |= BRCMF_SCANFLAGS_LOW_PRIO;
		brcmf_dbg(SCAN, "requested low priority scan\n");
	}
	return scan_flags;
}

static void *
brcmf_scan_param_get_prepped_struct_v1(struct brcmf_cfg80211_info *cfg,
				       u32 *struct_size,
				       struct cfg80211_scan_request *request)
{
	u32 n_ssids;
	u32 n_channels;
	u32 params_size = sizeof(struct brcmf_scan_params_le);
	u32 length;
	struct brcmf_scan_params_le *params_le = NULL;
	u8 scan_type = BRCMF_SCANTYPE_ACTIVE;

	length = offsetof(struct brcmf_scan_params_le, channel_list);
	params_size = brcmf_scan_param_tail_size(request, params_size);
	params_le = kzalloc(params_size, GFP_KERNEL);
	if (!params_le) {
		bphy_err(cfg, "Could not allocate scan params\n");
		return NULL;
	}
	brcmf_scan_param_set_defaults(&params_le->bssid,
		&params_le->bss_type, &params_le->channel_num,
		&params_le->nprobes, &params_le->active_time,
		&params_le->passive_time, &params_le->home_time);

	/* Scan abort */
	if (!request) {
		params_le->channel_num = cpu_to_le32(1);
		params_le->channel_list[0] = cpu_to_le16(-1);
		goto done;
	}

	n_ssids = request->n_ssids;
	n_channels = request->n_channels;

	/* Copy channel array if applicable */
	brcmf_dbg(SCAN, "### List of channelspecs to scan ### %d\n",
		  n_channels);
	if (n_channels > 0) {
		length += roundup(sizeof(u16) * n_channels, sizeof(u32));
		brcmf_scan_param_copy_chanspecs(cfg, &params_le->channel_list,
						request->channels, n_channels);
	} else {
		brcmf_dbg(SCAN, "Scanning all channels\n");
	}

	/* Copy ssid array if applicable */
	brcmf_dbg(SCAN, "### List of SSIDs to scan ### %d\n", n_ssids);
	if (n_ssids > 0) {
		s32 offset;
		char *ptr;

		offset =
			offsetof(struct brcmf_scan_params_le, channel_list) +
			n_channels * sizeof(u16);
		offset = roundup(offset, sizeof(u32));
		length += sizeof(struct brcmf_ssid_le) * n_ssids;
		ptr = (char *)params_le + offset;
		brcmf_scan_param_copy_ssids(ptr, request->ssids, n_ssids);
	} else {
		brcmf_dbg(SCAN, "Performing passive scan\n");
		scan_type = BRCMF_SCANTYPE_PASSIVE;
	}
	scan_type |= brcmf_nl80211_scan_flags_to_scan_flags(request->flags);
	params_le->scan_type =scan_type;
	/* Adding mask to channel numbers */
	params_le->channel_num =
		cpu_to_le32((n_ssids << BRCMF_SCAN_PARAMS_NSSID_SHIFT) |
			    (n_channels & BRCMF_SCAN_PARAMS_COUNT_MASK));
done:
	*struct_size = length;
	return params_le;
}

static void *
brcmf_scan_param_get_prepped_struct_v2(struct brcmf_cfg80211_info *cfg,
				       u32 *struct_size,
				       struct cfg80211_scan_request *request)
{
	u32 n_ssids;
	u32 n_channels;
	u32 params_size = sizeof(struct brcmf_scan_params_v2_le);
	u32 length;
	struct brcmf_scan_params_v2_le *params_le = NULL;
	u32 scan_type = BRCMF_SCANTYPE_ACTIVE;

	length = offsetof(struct brcmf_scan_params_v2_le, channel_list);
	params_size = brcmf_scan_param_tail_size(request, params_size);
	params_le = kzalloc(params_size, GFP_KERNEL);
	if (!params_le) {
		bphy_err(cfg, "Could not allocate scan params\n");
		return NULL;
	}
	params_le->version = cpu_to_le16(BRCMF_SCAN_PARAMS_VERSION_V2);
	brcmf_scan_param_set_defaults(&params_le->bssid,
		&params_le->bss_type, &params_le->channel_num,
		&params_le->nprobes, &params_le->active_time,
		&params_le->passive_time, &params_le->home_time);

	/* Scan abort */
	if (!request) {
		length += sizeof(u16);
		params_le->channel_num = cpu_to_le32(1);
		params_le->channel_list[0] = cpu_to_le16(-1);
		params_le->length = cpu_to_le16(length);
		goto done;
	}

	n_ssids = request->n_ssids;
	n_channels = request->n_channels;

	/* Copy channel array if applicable */
	brcmf_dbg(SCAN, "### List of channelspecs to scan ### %d\n",
		  n_channels);
	if (n_channels > 0) {
		length += roundup(sizeof(u16) * n_channels, sizeof(u32));
		brcmf_scan_param_copy_chanspecs(cfg, &params_le->channel_list,
						request->channels, n_channels);
	} else {
		brcmf_dbg(SCAN, "Scanning all channels\n");
	}

	/* Copy ssid array if applicable */
	brcmf_dbg(SCAN, "### List of SSIDs to scan ### %d\n", n_ssids);
	if (n_ssids > 0) {
		s32 offset;
		char *ptr;

		offset =
			offsetof(struct brcmf_scan_params_v2_le, channel_list) +
			n_channels * sizeof(u16);
		offset = roundup(offset, sizeof(u32));
		length += sizeof(struct brcmf_ssid_le) * n_ssids;
		ptr = (char *)params_le + offset;
		brcmf_scan_param_copy_ssids(ptr, request->ssids, n_ssids);

	} else {
		brcmf_dbg(SCAN, "Performing passive scan\n");
		scan_type = BRCMF_SCANTYPE_PASSIVE;
	}
	scan_type |= brcmf_nl80211_scan_flags_to_scan_flags(request->flags);
	params_le->scan_type = cpu_to_le32(scan_type);
	params_le->length = cpu_to_le16(length);
	/* Adding mask to channel numbers */
	params_le->channel_num =
		cpu_to_le32((n_ssids << BRCMF_SCAN_PARAMS_NSSID_SHIFT) |
			    (n_channels & BRCMF_SCAN_PARAMS_COUNT_MASK));
done:
	*struct_size = length;
	return params_le;
}

static void *
brcmf_scan_param_get_prepped_struct_v3(struct brcmf_cfg80211_info *cfg,
				       u32 *struct_size,
				       struct cfg80211_scan_request *request)
{
	u32 n_ssids;
	u32 n_channels;
	u32 params_size = sizeof(struct brcmf_scan_params_v3_le);
	u32 length;
	struct brcmf_scan_params_v3_le *params_le = NULL;
	u32 scan_type = BRCMF_SCANTYPE_ACTIVE;

	length = offsetof(struct brcmf_scan_params_v3_le, channel_list);
	params_size = brcmf_scan_param_tail_size(request, params_size);
	params_le = kzalloc(params_size, GFP_KERNEL);
	if (!params_le) {
		bphy_err(cfg, "Could not allocate scan params\n");
		return NULL;
	}

	params_le->version = cpu_to_le16(BRCMF_SCAN_PARAMS_VERSION_V3);
	params_le->ssid_type = 0;
	brcmf_scan_param_set_defaults(&params_le->bssid,
		&params_le->bss_type, &params_le->channel_num,
		&params_le->nprobes, &params_le->active_time,
		&params_le->passive_time, &params_le->home_time);

	/* Scan abort */
	if (!request) {
		length += sizeof(u16);
		params_le->channel_num = cpu_to_le32(1);
		params_le->channel_list[0] = cpu_to_le16(-1);
		params_le->length = cpu_to_le16(length);
		goto done;
	}

	n_ssids = request->n_ssids;
	n_channels = request->n_channels;

	/* Copy channel array if applicable */
	brcmf_dbg(SCAN, "### List of channelspecs to scan ### %d\n",
		  n_channels);
	if (n_channels > 0) {
		length += roundup(sizeof(u16) * n_channels, sizeof(u32));
		brcmf_scan_param_copy_chanspecs(cfg, &params_le->channel_list,
						request->channels, n_channels);

	} else {
		brcmf_dbg(SCAN, "Scanning all channels\n");
	}

	/* Copy ssid array if applicable */
	brcmf_dbg(SCAN, "### List of SSIDs to scan ### %d\n", n_ssids);
	if (n_ssids > 0) {
		s32 offset;
		char *ptr;

		offset =
			offsetof(struct brcmf_scan_params_v3_le, channel_list) +
			n_channels * sizeof(u16);
		offset = roundup(offset, sizeof(u32));
		length += sizeof(struct brcmf_ssid_le) * n_ssids;
		ptr = (char *)params_le + offset;
		brcmf_scan_param_copy_ssids(ptr, request->ssids, n_ssids);

	} else {
		brcmf_dbg(SCAN, "Performing passive scan\n");
		scan_type = BRCMF_SCANTYPE_PASSIVE;
	}
	scan_type |= brcmf_nl80211_scan_flags_to_scan_flags(request->flags);
	params_le->scan_type = cpu_to_le32(scan_type);
	params_le->length = cpu_to_le16(length);
	params_le->channel_num =
		cpu_to_le32((n_ssids << BRCMF_SCAN_PARAMS_NSSID_SHIFT) |
			    (n_channels & BRCMF_SCAN_PARAMS_COUNT_MASK));

	/* Include RNR results if requested */
	if (request->flags & NL80211_SCAN_FLAG_COLOCATED_6GHZ) {
		params_le->ssid_type |= BRCMF_SCANSSID_INC_RNR;
	}
	/* Adding mask to channel numbers */
done:
	*struct_size = length;
	return params_le;
}

static void *
brcmf_scan_param_get_prepped_struct_v4(struct brcmf_cfg80211_info *cfg,
				       u32 *struct_size,
				       struct cfg80211_scan_request *request)
{
	u32 n_ssids;
	u32 n_channels;
	u32 params_size = sizeof(struct brcmf_scan_params_v4_le);
	u32 length;
	struct brcmf_scan_params_v4_le *params_le = NULL;
	u32 scan_type = BRCMF_SCANTYPE_ACTIVE;

	length = offsetof(struct brcmf_scan_params_v4_le, channel_list);
	params_size = brcmf_scan_param_tail_size(request, params_size);
	params_le = kzalloc(params_size, GFP_KERNEL);
	if (!params_le) {
		bphy_err(cfg, "Could not allocate scan params\n");
		return NULL;
	}
	params_le->version = cpu_to_le16(BRCMF_SCAN_PARAMS_VERSION_V4);
	params_le->ssid_type = 0;
	brcmf_scan_param_set_defaults(&params_le->bssid,
		&params_le->bss_type, &params_le->channel_num,
		&params_le->nprobes, &params_le->active_time,
		&params_le->passive_time, &params_le->home_time);

	/* Scan abort */
	if (!request) {
		length += sizeof(u16);
		params_le->channel_num = cpu_to_le32(1);
		params_le->channel_list[0] = cpu_to_le16(-1);
		params_le->length = cpu_to_le16(length);
		goto done;
	}

	n_ssids = request->n_ssids;
	n_channels = request->n_channels;

	/* Copy channel array if applicable */
	brcmf_dbg(SCAN, "### List of channelspecs to scan ### %d\n",
		  n_channels);
	if (n_channels > 0) {
		length += roundup(sizeof(u16) * n_channels, sizeof(u32));
		brcmf_scan_param_copy_chanspecs(cfg, &params_le->channel_list,
						request->channels, n_channels);
	} else {
		brcmf_dbg(SCAN, "Scanning all channels\n");
	}

	/* Copy ssid array if applicable */
	brcmf_dbg(SCAN, "### List of SSIDs to scan ### %d\n", n_ssids);
	if (n_ssids > 0) {
		s32 offset;
		char *ptr;

		offset =
			offsetof(struct brcmf_scan_params_v4_le, channel_list) +
			n_channels * sizeof(u16);
		offset = roundup(offset, sizeof(u32));
		length += sizeof(struct brcmf_ssid_le) * n_ssids;
		ptr = (char *)params_le + offset;
		brcmf_scan_param_copy_ssids(ptr, request->ssids, n_ssids);
	} else {
		brcmf_dbg(SCAN, "Performing passive scan\n");
		scan_type = BRCMF_SCANTYPE_PASSIVE;
	}
	scan_type |= brcmf_nl80211_scan_flags_to_scan_flags(request->flags);
	params_le->scan_type = cpu_to_le32(scan_type);
	params_le->length = cpu_to_le16(length);
	/* Adding mask to channel numbers */
	params_le->channel_num =
		cpu_to_le32((n_ssids << BRCMF_SCAN_PARAMS_NSSID_SHIFT) |
			    (n_channels & BRCMF_SCAN_PARAMS_COUNT_MASK));
	/* Include RNR results if requested */
	if (request->flags & NL80211_SCAN_FLAG_COLOCATED_6GHZ) {
		params_le->ssid_type |= BRCMF_SCANSSID_INC_RNR;
	}
done:
	*struct_size = length;
	return params_le;
}

int brcmf_scan_param_setup_for_version(struct brcmf_pub *drvr, u8 version)
{
	drvr->scan_param_handler.version = version;
	switch (version) {
	case 1: {
		drvr->scan_param_handler.get_struct_for_request =
			brcmf_scan_param_get_prepped_struct_v1;
	} break;
	case 2: {
		drvr->scan_param_handler.get_struct_for_request =
			brcmf_scan_param_get_prepped_struct_v2;
	} break;
	case 3: {
		drvr->scan_param_handler.get_struct_for_request =
			brcmf_scan_param_get_prepped_struct_v3;
	} break;
	case 4: {
		drvr->scan_param_handler.get_struct_for_request =
			brcmf_scan_param_get_prepped_struct_v4;

	} break;
	default:
		return -EINVAL;
	}
	return 0;
}
