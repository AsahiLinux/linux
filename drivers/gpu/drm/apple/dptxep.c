// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2022 Sven Peter <sven@svenpeter.dev> */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>

#include "afk.h"
#include "dcp.h"
#include "dptxep.h"
#include "parser.h"
#include "trace.h"

struct dcpdptx_connection_cmd {
	__le32 unk;
	__le32 target;
} __attribute__((packed));

struct dcpdptx_hotplug_cmd {
	u8 _pad0[16];
	__le32 unk;
} __attribute__((packed));

struct dptxport_apcall_link_rate {
	__le32 retcode;
	u8 _unk0[12];
	__le32 link_rate;
	u8 _unk1[12];
} __attribute__((packed));

struct dptxport_apcall_get_support {
	__le32 retcode;
	u8 _unk0[12];
	__le32 supported;
	u8 _unk1[12];
} __attribute__((packed));

struct dptxport_apcall_max_drive_settings {
	__le32 retcode;
	u8 _unk0[12];
	__le32 max_drive_settings[2];
	u8 _unk1[8];
};

int dptxport_validate_connection(struct apple_epic_service *service, u8 core,
				 u8 atc, u8 die)
{
	struct dptx_port *dptx = service->cookie;
	struct dcpdptx_connection_cmd cmd, resp;
	int ret;
	u32 target = FIELD_PREP(DCPDPTX_REMOTE_PORT_CORE, core) |
		     FIELD_PREP(DCPDPTX_REMOTE_PORT_ATC, atc) |
		     FIELD_PREP(DCPDPTX_REMOTE_PORT_DIE, die) |
		     DCPDPTX_REMOTE_PORT_CONNECTED;

	trace_dptxport_validate_connection(dptx, core, atc, die);

	cmd.target = cpu_to_le32(target);
	cmd.unk = cpu_to_le32(0x100);
	ret = afk_service_call(service, 0, 14, &cmd, sizeof(cmd), 40, &resp,
			       sizeof(resp), 40);
	if (ret)
		return ret;

	if (le32_to_cpu(resp.target) != target)
		return -EINVAL;
	if (le32_to_cpu(resp.unk) != 0x100)
		return -EINVAL;

	return 0;
}

int dptxport_connect(struct apple_epic_service *service, u8 core, u8 atc,
		     u8 die)
{
	struct dptx_port *dptx = service->cookie;
	struct dcpdptx_connection_cmd cmd, resp;
	int ret;
	u32 target = FIELD_PREP(DCPDPTX_REMOTE_PORT_CORE, core) |
		     FIELD_PREP(DCPDPTX_REMOTE_PORT_ATC, atc) |
		     FIELD_PREP(DCPDPTX_REMOTE_PORT_DIE, die) |
		     DCPDPTX_REMOTE_PORT_CONNECTED;

	trace_dptxport_connect(dptx, core, atc, die);

	cmd.target = cpu_to_le32(target);
	cmd.unk = cpu_to_le32(0x100);
	ret = afk_service_call(service, 0, 13, &cmd, sizeof(cmd), 24, &resp,
			       sizeof(resp), 24);
	if (ret)
		return ret;

	if (le32_to_cpu(resp.target) != target)
		return -EINVAL;
	if (le32_to_cpu(resp.unk) != 0x100)
		return -EINVAL;

	return 0;
}

int dptxport_request_display(struct apple_epic_service *service)
{
	return afk_service_call(service, 0, 8, NULL, 0, 16, NULL, 0, 16);
}

int dptxport_release_display(struct apple_epic_service *service)
{
	return afk_service_call(service, 0, 9, NULL, 0, 16, NULL, 0, 16);
}

int dptxport_set_hpd(struct apple_epic_service *service, bool hpd)
{
	struct dcpdptx_hotplug_cmd cmd, resp;
	int ret;

	memset(&cmd, 0, sizeof(cmd));

	if (hpd)
		cmd.unk = cpu_to_le32(1);

	ret = afk_service_call(service, 8, 10, &cmd, sizeof(cmd), 12, &resp,
			       sizeof(resp), 12);
	if (ret)
		return ret;
	if (le32_to_cpu(resp.unk) != 1)
		return -EINVAL;
	return 0;
}

static int
dptxport_call_get_max_drive_settings(struct apple_epic_service *service,
				     void *reply_, size_t reply_size)
{
	struct dptxport_apcall_max_drive_settings *reply = reply_;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	reply->retcode = cpu_to_le32(0);
	reply->max_drive_settings[0] = cpu_to_le32(0x3);
	reply->max_drive_settings[1] = cpu_to_le32(0x3);

	return 0;
}

static int dptxport_call_get_max_link_rate(struct apple_epic_service *service,
					   void *reply_, size_t reply_size)
{
	struct dptxport_apcall_link_rate *reply = reply_;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	reply->retcode = cpu_to_le32(0);
	reply->link_rate = cpu_to_le32(LINK_RATE_HBR3);

	return 0;
}

static int dptxport_call_get_link_rate(struct apple_epic_service *service,
				       void *reply_, size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	struct dptxport_apcall_link_rate *reply = reply_;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	reply->retcode = cpu_to_le32(0);
	reply->link_rate = cpu_to_le32(dptx->link_rate);

	return 0;
}

static int
dptxport_call_will_change_link_config(struct apple_epic_service *service)
{
	struct dptx_port *dptx = service->cookie;

	dptx->phy_ops.dp.set_lanes = 0;
	dptx->phy_ops.dp.set_rate = 0;
	dptx->phy_ops.dp.set_voltages = 0;

	return 0;
}

static int
dptxport_call_did_change_link_config(struct apple_epic_service *service)
{
	/* assume the link config did change and wait a little bit */
	mdelay(10);
	return 0;
}

static int dptxport_call_set_link_rate(struct apple_epic_service *service,
				       const void *data, size_t data_size,
				       void *reply_, size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	const struct dptxport_apcall_link_rate *request = data;
	struct dptxport_apcall_link_rate *reply = reply_;
	u32 link_rate, phy_link_rate;
	bool phy_set_rate = false;
	int ret;

	if (reply_size < sizeof(*reply))
		return -EINVAL;
	if (data_size < sizeof(*request))
		return -EINVAL;

	link_rate = le32_to_cpu(request->link_rate);
	trace_dptxport_call_set_link_rate(dptx, link_rate);

	switch (link_rate) {
	case LINK_RATE_RBR:
		phy_link_rate = 1620;
		phy_set_rate = true;
		break;
	case LINK_RATE_HBR:
		phy_link_rate = 2700;
		phy_set_rate = true;
		break;
	case LINK_RATE_HBR2:
		phy_link_rate = 5400;
		phy_set_rate = true;
		break;
	case LINK_RATE_HBR3:
		phy_link_rate = 8100;
		phy_set_rate = true;
		break;
	case 0:
		phy_link_rate = 0;
		phy_set_rate = true;
		break;
	default:
		dev_err(service->ep->dcp->dev,
			"DPTXPort: Unsupported link rate 0x%x requested\n",
			link_rate);
		link_rate = 0;
		phy_set_rate = false;
		break;
	}

	if (phy_set_rate) {
		dptx->phy_ops.dp.link_rate = phy_link_rate;
		dptx->phy_ops.dp.set_rate = 1;

		if (dptx->atcphy) {
			ret = phy_configure(dptx->atcphy, &dptx->phy_ops);
			if (ret)
				return ret;
		}

		//if (dptx->phy_ops.dp.set_rate)
		dptx->link_rate = dptx->pending_link_rate = link_rate;

	}

	//dptx->pending_link_rate = link_rate;
	reply->retcode = cpu_to_le32(0);
	reply->link_rate = cpu_to_le32(link_rate);

	return 0;
}

static int dptxport_call_get_supports_hpd(struct apple_epic_service *service,
					  void *reply_, size_t reply_size)
{
	struct dptxport_apcall_get_support *reply = reply_;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	reply->retcode = cpu_to_le32(0);
	reply->supported = cpu_to_le32(0);
	return 0;
}

static int
dptxport_call_get_supports_downspread(struct apple_epic_service *service,
				      void *reply_, size_t reply_size)
{
	struct dptxport_apcall_get_support *reply = reply_;

	if (reply_size < sizeof(*reply))
		return -EINVAL;

	reply->retcode = cpu_to_le32(0);
	reply->supported = cpu_to_le32(0);
	return 0;
}

static int dptxport_call(struct apple_epic_service *service, u32 idx,
			 const void *data, size_t data_size, void *reply,
			 size_t reply_size)
{
	struct dptx_port *dptx = service->cookie;
	trace_dptxport_apcall(dptx, idx, data_size);

	switch (idx) {
	case DPTX_APCALL_WILL_CHANGE_LINKG_CONFIG:
		return dptxport_call_will_change_link_config(service);
	case DPTX_APCALL_DID_CHANGE_LINK_CONFIG:
		return dptxport_call_did_change_link_config(service);
	case DPTX_APCALL_GET_MAX_LINK_RATE:
		return dptxport_call_get_max_link_rate(service, reply,
						       reply_size);
	case DPTX_APCALL_GET_LINK_RATE:
		return dptxport_call_get_link_rate(service, reply, reply_size);
	case DPTX_APCALL_SET_LINK_RATE:
		return dptxport_call_set_link_rate(service, data, data_size,
						   reply, reply_size);
	case DPTX_APCALL_GET_SUPPORTS_HPD:
		return dptxport_call_get_supports_hpd(service, reply,
						      reply_size);
	case DPTX_APCALL_GET_SUPPORTS_DOWN_SPREAD:
		return dptxport_call_get_supports_downspread(service, reply,
							     reply_size);
	case DPTX_APCALL_GET_MAX_DRIVE_SETTINGS:
		return dptxport_call_get_max_drive_settings(service, reply,
							    reply_size);
	default:
		/* just try to ACK and hope for the best... */
		dev_info(service->ep->dcp->dev, "DPTXPort: acking unhandled call %u\n",
			idx);
		fallthrough;
	/* we can silently ignore and just ACK these calls */
	case DPTX_APCALL_ACTIVATE:
	case DPTX_APCALL_DEACTIVATE:
	case DPTX_APCALL_SET_DRIVE_SETTINGS:
	case DPTX_APCALL_GET_DRIVE_SETTINGS:
		memcpy(reply, data, min(reply_size, data_size));
		if (reply_size > 4)
			memset(reply, 0, 4);
		return 0;
	}
}

static void dptxport_init(struct apple_epic_service *service, const char *name,
			  const char *class, s64 unit)
{

	if (strcmp(name, "dcpdptx-port-epic"))
		return;
	if (strcmp(class, "AppleDCPDPTXRemotePort"))
		return;

	trace_dptxport_init(service->ep->dcp, unit);

	switch (unit) {
	case 0:
	case 1:
		if (service->ep->dcp->dptxport[unit].enabled) {
			dev_err(service->ep->dcp->dev,
				"DPTXPort: unit %lld already exists\n", unit);
			return;
		}
		service->ep->dcp->dptxport[unit].unit = unit;
		service->ep->dcp->dptxport[unit].service = service;
		service->ep->dcp->dptxport[unit].enabled = true;
		service->cookie = (void *)&service->ep->dcp->dptxport[unit];
		complete(&service->ep->dcp->dptxport[unit].enable_completion);
		break;
	default:
		dev_err(service->ep->dcp->dev, "DPTXPort: invalid unit %lld\n",
			unit);
	}
}

static const struct apple_epic_service_ops dptxep_ops[] = {
	{
		.name = "AppleDCPDPTXRemotePort",
		.init = dptxport_init,
		.call = dptxport_call,
	},
	{}
};

int dptxep_init(struct apple_dcp *dcp)
{
	int ret;
	u32 port;
	unsigned long timeout = msecs_to_jiffies(1000);

	init_completion(&dcp->dptxport[0].enable_completion);
	init_completion(&dcp->dptxport[1].enable_completion);

	dcp->dptxep = afk_init(dcp, DPTX_ENDPOINT, dptxep_ops);
	if (IS_ERR(dcp->dptxep))
		return PTR_ERR(dcp->dptxep);

	ret = afk_start(dcp->dptxep);
	if (ret)
		return ret;

	for (port = 0; port < dcp->hw.num_dptx_ports; port++) {
		ret = wait_for_completion_timeout(&dcp->dptxport[port].enable_completion,
						timeout);
		if (!ret)
			return -ETIMEDOUT;
		else if (ret < 0)
			return ret;
		timeout = ret;
	}

	return 0;
}
