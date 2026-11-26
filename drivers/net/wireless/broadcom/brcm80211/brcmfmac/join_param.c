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
#include "join_param.h"

/* These defaults are the same as found in the DHD drivers, and represent
 * reasonable defaults for various scan dwell and probe times.   */
#define BRCMF_SCAN_JOIN_ACTIVE_DWELL_TIME_MS 320
#define BRCMF_SCAN_JOIN_PASSIVE_DWELL_TIME_MS 400
#define BRCMF_SCAN_JOIN_PROBE_INTERVAL_MS 20

/* Most of the actual structure fields we fill in are the same for various versions
 * However, due to various incompatible changes and variants, the fields are not always
 * in the same place.
 * This makes for code duplication, so we try to commonize setting fields where it makes sense.
 */

static void brcmf_joinscan_set_ssid(struct brcmf_ssid_le *ssid_le,
				    const u8 *ssid, u32 ssid_len)
{
	ssid_len = min_t(u32, ssid_len, IEEE80211_MAX_SSID_LEN);
	ssid_le->SSID_len = cpu_to_le32(ssid_len);
	memcpy(ssid_le->SSID, ssid, ssid_len);
}

static void brcmf_joinscan_set_bssid(u8 out_bssid[6], const u8 *in_bssid)
{
	if (in_bssid) {
		memcpy(out_bssid, in_bssid, ETH_ALEN);
	} else {
		eth_broadcast_addr(out_bssid);
	}
}

/* Create a single channel chanspec list from a wireless stack channel */
static void brcmf_joinscan_set_single_chanspec_from_channel(
	struct brcmf_cfg80211_info *cfg, struct ieee80211_channel *chan,
	__le32 *chanspec_count, __le16 (*chanspec_list)[])
{
	u16 chanspec = channel_to_chanspec(&cfg->d11inf, chan);
	*chanspec_count = cpu_to_le32(1);
	(*chanspec_list)[0] = cpu_to_le16(chanspec);
}

/* Create a single channel chanspec list from a wireless stack chandef */
static void brcmf_joinscan_set_single_chanspec_from_chandef(
	struct brcmf_cfg80211_info *cfg, struct cfg80211_chan_def *chandef,
	__le32 *chanspec_count, __le16 (*chanspec_list)[])
{
	u16 chanspec = chandef_to_chanspec(&cfg->d11inf, chandef);
	*chanspec_count = cpu_to_le32(1);
	(*chanspec_list)[0] = cpu_to_le16(chanspec);
}

static void *brcmf_get_struct_for_ibss_v0(struct brcmf_cfg80211_info *cfg,
					  u32 *struct_size,
					  struct cfg80211_ibss_params *params)
{
	struct brcmf_join_params *join_params;

	u32 join_params_size = struct_size(join_params, params_le.chanspec_list,
					   params->chandef.chan != NULL);

	*struct_size = join_params_size;
	join_params = kzalloc(join_params_size, GFP_KERNEL);
	if (!join_params) {
		bphy_err(cfg, "Unable to allocate memory for join params\n");
		return NULL;
	}
	brcmf_joinscan_set_ssid(&join_params->ssid_le, params->ssid,
				params->ssid_len);
	brcmf_joinscan_set_bssid(join_params->params_le.bssid, params->bssid);
	/* Channel */
	if (cfg->channel) {
		brcmf_joinscan_set_single_chanspec_from_chandef(
			cfg, &params->chandef,
			&join_params->params_le.chanspec_num,
			&join_params->params_le.chanspec_list);
	}
	return join_params;
}

static void *
brcmf_get_prepped_struct_for_ibss_v1(struct brcmf_cfg80211_info *cfg,
				     u32 *struct_size,
				     struct cfg80211_ibss_params *params)
{
	struct brcmf_join_params_v1 *join_params;
	u32 join_params_size = struct_size(join_params, params_le.chanspec_list,
					   params->chandef.chan != NULL);

	*struct_size = join_params_size;
	join_params = kzalloc(join_params_size, GFP_KERNEL);
	if (!join_params) {
		bphy_err(cfg, "Unable to allocate memory for join params\n");
		return NULL;
	}
	join_params->params_le.version = cpu_to_le16(1);
	brcmf_joinscan_set_ssid(&join_params->ssid_le, params->ssid,
				params->ssid_len);
	brcmf_joinscan_set_bssid(join_params->params_le.bssid, params->bssid);
	/* Channel */
	if (cfg->channel) {
		brcmf_joinscan_set_single_chanspec_from_chandef(
			cfg, &params->chandef,
			&join_params->params_le.chanspec_num,
			&join_params->params_le.chanspec_list);
	}
	return join_params;
}

static void
brcmf_joinscan_set_common_v0v1_params(struct brcmf_join_scan_params_le *scan_le,
				      bool have_channel)
{
	/* Set up join scan parameters */
	scan_le->scan_type = 0;
	scan_le->home_time = cpu_to_le32(-1);

	if (have_channel) {
		/* Increase dwell time to receive probe response or detect
		 * beacon from target AP at a noisy air only during connect
		 * command.
		 */
		scan_le->active_time =
			cpu_to_le32(BRCMF_SCAN_JOIN_ACTIVE_DWELL_TIME_MS);
		scan_le->passive_time =
			cpu_to_le32(BRCMF_SCAN_JOIN_PASSIVE_DWELL_TIME_MS);
		/* To sync with presence period of VSDB GO send probe request
		 * more frequently. Probe request will be stopped when it gets
		 * probe response from target AP/GO.
		 */
		scan_le->nprobes =
			cpu_to_le32(BRCMF_SCAN_JOIN_ACTIVE_DWELL_TIME_MS /
				    BRCMF_SCAN_JOIN_PROBE_INTERVAL_MS);
	} else {
		scan_le->active_time = cpu_to_le32(-1);
		scan_le->passive_time = cpu_to_le32(-1);
		scan_le->nprobes = cpu_to_le32(-1);
	}
}
static void *
brcmf_get_struct_for_connect_v0(struct brcmf_cfg80211_info *cfg,
				u32 *struct_size,
				struct cfg80211_connect_params *params)
{
	struct brcmf_ext_join_params_le *ext_v0;
	u32 join_params_size =
		struct_size(ext_v0, assoc_le.chanspec_list, cfg->channel != 0);

	*struct_size = join_params_size;
	ext_v0 = kzalloc(join_params_size, GFP_KERNEL);
	if (!ext_v0) {
		bphy_err(
			cfg,
			"Could not allocate memory for extended join parameters\n");
		return NULL;
	}
	brcmf_joinscan_set_ssid(&ext_v0->ssid_le, params->ssid,
				params->ssid_len);
	brcmf_joinscan_set_common_v0v1_params(&ext_v0->scan_le,
					      cfg->channel != 0);
	brcmf_joinscan_set_bssid(ext_v0->assoc_le.bssid, params->bssid);
	if (cfg->channel) {
		struct ieee80211_channel *chan = params->channel_hint ?
							 params->channel_hint :
							 params->channel;
		brcmf_joinscan_set_single_chanspec_from_channel(
			cfg, chan, &ext_v0->assoc_le.chanspec_num,
			&ext_v0->assoc_le.chanspec_list);
	}
	return ext_v0;
}

static void *
brcmf_get_struct_for_connect_v1(struct brcmf_cfg80211_info *cfg,
				u32 *struct_size,
				struct cfg80211_connect_params *params)
{
	struct brcmf_ext_join_params_v1_le *ext_v1;
	u32 join_params_size =
		struct_size(ext_v1, assoc_le.chanspec_list, cfg->channel != 0);

	*struct_size = join_params_size;
	ext_v1 = kzalloc(join_params_size, GFP_KERNEL);
	if (!ext_v1) {
		bphy_err(
			cfg,
			"Could not allocate memory for extended join parameters\n");
		return NULL;
	}
	ext_v1->version = cpu_to_le16(1);
	ext_v1->assoc_le.version = cpu_to_le16(1);
	brcmf_joinscan_set_ssid(&ext_v1->ssid_le, params->ssid,
				params->ssid_len);
	brcmf_joinscan_set_common_v0v1_params(&ext_v1->scan_le,
					      cfg->channel != 0);
	brcmf_joinscan_set_bssid(ext_v1->assoc_le.bssid, params->bssid);
	if (cfg->channel) {
		struct ieee80211_channel *chan = params->channel_hint ?
							 params->channel_hint :
							 params->channel;
		brcmf_joinscan_set_single_chanspec_from_channel(
			cfg, chan, &ext_v1->assoc_le.chanspec_num,
			&ext_v1->assoc_le.chanspec_list);
	}
	return ext_v1;
}

static void *brcmf_get_join_from_ext_join_v0(void *ext_join, u32 *struct_size)
{
	struct brcmf_ext_join_params_le *ext_join_v0 =
		(struct brcmf_ext_join_params_le *)ext_join;
	u32 chanspec_num = le32_to_cpu(ext_join_v0->assoc_le.chanspec_num);
	struct brcmf_join_params *join_params;
	u32 join_params_size =
		struct_size(join_params, params_le.chanspec_list, chanspec_num);
	u32 assoc_size = struct_size_t(struct brcmf_assoc_params_le,
				       chanspec_list, chanspec_num);

	*struct_size = join_params_size;
	join_params = kzalloc(join_params_size, GFP_KERNEL);
	if (!join_params) {
		return NULL;
	}
	memcpy(&join_params->ssid_le, &ext_join_v0->ssid_le,
	       sizeof(ext_join_v0->ssid_le));
	memcpy(&join_params->params_le, &ext_join_v0->assoc_le, assoc_size);

	return join_params;
}

static void *brcmf_get_join_from_ext_join_v1(void *ext_join, u32 *struct_size)
{
	struct brcmf_ext_join_params_v1_le *ext_join_v1 =
		(struct brcmf_ext_join_params_v1_le *)ext_join;
	u32 chanspec_num = le32_to_cpu(ext_join_v1->assoc_le.chanspec_num);
	struct brcmf_join_params_v1 *join_params;
	u32 join_params_size =
		struct_size(join_params, params_le.chanspec_list, chanspec_num);
	u32 assoc_size = struct_size_t(struct brcmf_assoc_params_le,
				       chanspec_list, chanspec_num);

	*struct_size = join_params_size;
	join_params = kzalloc(join_params_size, GFP_KERNEL);
	if (!join_params) {
		return NULL;
	}
	memcpy(&join_params->ssid_le, &ext_join_v1->ssid_le,
	       sizeof(ext_join_v1->ssid_le));
	memcpy(&join_params->params_le, &ext_join_v1->assoc_le, assoc_size);

	return join_params;
}

int brcmf_join_param_setup_for_version(struct brcmf_pub *drvr, u8 version)
{
	drvr->join_param_handler.version = version;
	switch (version) {
	case 0:
		drvr->join_param_handler.get_struct_for_ibss =
			brcmf_get_struct_for_ibss_v0;
		drvr->join_param_handler.get_struct_for_connect =
			brcmf_get_struct_for_connect_v0;
		drvr->join_param_handler.get_join_from_ext_join =
			brcmf_get_join_from_ext_join_v0;
		break;
	case 1:
		drvr->join_param_handler.get_struct_for_ibss =
			brcmf_get_prepped_struct_for_ibss_v1;
		drvr->join_param_handler.get_struct_for_connect =
			brcmf_get_struct_for_connect_v1;
		drvr->join_param_handler.get_join_from_ext_join =
			brcmf_get_join_from_ext_join_v1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
