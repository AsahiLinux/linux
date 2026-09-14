// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright The Asahi Linux Contributors */

#include "dpavservep.h"

#include <drm/drm_edid.h>

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/types.h>

#include "../afk.h"
#include "../dcp.h"
#include "../dcp-internal.h"
#include "../trace.h"

static void dcpavserv_init(struct apple_epic_service *service, const char *name,
			  const char *class, s64 unit)
{
	struct apple_dcp *dcp = service->ep->dcp;
	trace_dcpavserv_init(dcp, unit);

	if (unit == 0 && name && !strcmp(name, "dcpav-service-epic")) {
		if (dcp->dcpavserv.enabled) {
			dev_err(dcp->dev,
				"DCPAVSERV: unit %lld already exists\n", unit);
			return;
		}
		dcp->dcpavserv.service = service;
		dcp->dcpavserv.enabled = true;
		service->cookie = &dcp->dcpavserv;
		complete(&dcp->dcpavserv.enable_completion);
	}
}

static void dcpavserv_teardown(struct apple_epic_service *service)
{
	struct apple_dcp *dcp = service->ep->dcp;
	service->enabled = false;

	if (dcp->dcpavserv.enabled) {
		dcp->dcpavserv.enabled = false;
		dcp->dcpavserv.service = NULL;
		service->cookie = NULL;
		reinit_completion(&dcp->dcpavserv.enable_completion);
	}
}

static void dcpdpserv_init(struct apple_epic_service *service, const char *name,
			  const char *class, s64 unit)
{
}

static void dcpdpserv_teardown(struct apple_epic_service *service)
{
	service->enabled = false;
}

struct dcpavserv_status_report {
	u32 unk00[4];
	u8 flag0;
	u8 flag1;
	u8 flag2;
	u8 flag3;
	u32 unk14[3];
	u32 status;
	u32 unk24[3];
} __packed;

struct dpavserv_copy_edid_cmd {
	__le64 max_size;
	u8 _pad1[24];
	__le64 used_size;
	u8 _pad2[8];
} __packed;

#define EDID_LEADING_DATA_SIZE		8
#define EDID_BLOCK_SIZE			128
#define EDID_EXT_BLOCK_COUNT_OFFSET	0x7E
#define EDID_MAX_SIZE			SZ_32K
#define EDID_BUF_SIZE			(EDID_LEADING_DATA_SIZE + EDID_MAX_SIZE)

struct dpavserv_copy_edid_resp {
	__le64 max_size;
	u8 _pad1[24];
	__le64 used_size;
	u8 _pad2[8];
	u8 data[];
} __packed;

static int parse_report(struct apple_epic_service *service, enum epic_subtype type,
			 const void *data, size_t data_size)
{
#if defined(DEBUG)
	struct apple_dcp *dcp = service->ep->dcp;
	const struct epic_service_call *call;
	const void *payload;
	size_t payload_size;

	dev_dbg(dcp->dev, "dcpavserv[ch:%u]: report type:%02x len:%zu\n",
		service->channel, type, data_size);

	if (type != EPIC_SUBTYPE_STD_SERVICE)
		return 0;

	if (data_size < sizeof(*call))
		return 0;

	call = data;

	if (le32_to_cpu(call->magic) != EPIC_SERVICE_CALL_MAGIC) {
		dev_warn(dcp->dev, "dcpavserv[ch:%u]: report magic 0x%08x != 0x%08x\n",
			service->channel, le32_to_cpu(call->magic), EPIC_SERVICE_CALL_MAGIC);
		return 0;
	}

	payload_size = data_size - sizeof(*call);
	if (payload_size < le32_to_cpu(call->data_len)) {
		dev_warn(dcp->dev, "dcpavserv[ch:%u]: report payload size %zu call len %u\n",
			service->channel, payload_size, le32_to_cpu(call->data_len));
		return 0;
	}
	payload_size = le32_to_cpu(call->data_len);
	payload = data + sizeof(*call);

	if (le16_to_cpu(call->group) == 2 && le16_to_cpu(call->command) == 0) {
		if (payload_size == sizeof(struct dcpavserv_status_report)) {
			const struct dcpavserv_status_report *stat = payload;
			dev_info(dcp->dev, "dcpavserv[ch:%u]: flags: 0x%02x,0x%02x,0x%02x,0x%02x status:%u\n",
				service->channel, stat->flag0, stat->flag1,
				stat->flag2, stat->flag3, stat->status);
		} else {
			dev_dbg(dcp->dev, "dcpavserv[ch:%u]: report payload size %zu\n", service->channel, payload_size);
		}
	} else {
		print_hex_dump(KERN_DEBUG, "dcpavserv report: ", DUMP_PREFIX_NONE,
			       16, 1, payload, payload_size, true);
	}
#endif

	return 0;
}

static int dcpavserv_report(struct apple_epic_service *service,
			    enum epic_subtype type, const void *data,
			    size_t data_size)
{
	return parse_report(service, type, data, data_size);
}

static int dcpdpserv_report(struct apple_epic_service *service,
			    enum epic_subtype type, const void *data,
			    size_t data_size)
{
	return parse_report(service, type, data, data_size);
}

const struct drm_edid *dcpavserv_copy_edid(struct apple_epic_service *service)
{
	struct dpavserv_copy_edid_cmd cmd;
	struct dpavserv_copy_edid_resp *resp __free(kfree) = NULL;
	int num_blocks;
	u64 data_size;
	int ret;

	memset(&cmd, 0, sizeof(cmd));
	cmd.max_size = cpu_to_le64(EDID_BUF_SIZE);
	resp = kzalloc(sizeof(*resp) + EDID_BUF_SIZE, GFP_KERNEL);
	if (!resp)
		return ERR_PTR(-ENOMEM);

	ret = afk_service_call(service, 1, 7, &cmd, sizeof(cmd), EDID_BUF_SIZE, resp,
			       sizeof(resp) + EDID_BUF_SIZE, 0);
	if (ret < 0)
		return ERR_PTR(ret);

	if (le64_to_cpu(resp->max_size) != EDID_BUF_SIZE)
		return ERR_PTR(-EIO);

	// print_hex_dump(KERN_DEBUG, "dpavserv EDID cmd: ", DUMP_PREFIX_NONE,
	// 	       16, 1, resp, 192, true);

	data_size = le64_to_cpu(resp->used_size);
	if (data_size < EDID_LEADING_DATA_SIZE + EDID_BLOCK_SIZE)
		return ERR_PTR(-EIO);

	num_blocks = resp->data[EDID_LEADING_DATA_SIZE + EDID_EXT_BLOCK_COUNT_OFFSET];
	if ((1 + num_blocks) * EDID_BLOCK_SIZE != data_size - EDID_LEADING_DATA_SIZE)
		return ERR_PTR(-EIO);

	return drm_edid_alloc(resp->data + EDID_LEADING_DATA_SIZE,
			      data_size - EDID_LEADING_DATA_SIZE);
}

static const struct apple_epic_service_ops dpavservep_ops[] = {
	{
		.name = "dcpav-service-epic",
		.init = dcpavserv_init,
		.teardown = dcpavserv_teardown,
		.report = dcpavserv_report,
	},
	{
		.name = "dcpdp-service-epic",
		.init = dcpdpserv_init,
		.teardown = dcpdpserv_teardown,
		.report = dcpdpserv_report,
	},
	{},
};

int dpavservep_init(struct apple_dcp *dcp)
{
	int ret;

	init_completion(&dcp->dcpavserv.enable_completion);

	dcp->dcpavservep = afk_init(dcp, DPAVSERV_ENDPOINT, dpavservep_ops);
	if (IS_ERR(dcp->dcpavservep))
		return PTR_ERR(dcp->dcpavservep);

	dcp->dcpavservep->match_epic_name = true;

	ret = afk_start(dcp->dcpavservep);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&dcp->dcpavserv.enable_completion,
					  msecs_to_jiffies(1000));
	if (ret >= 0)
		return 0;

	return ret;
}

/*
 * I2C over the DCP AV service.
 *
 * The DCP firmware owns the DisplayPort AUX channel, so the AP cannot drive
 * I2C-over-AUX itself. macOS asks the firmware to run the transaction instead:
 * IOAVServiceReadI2C() and IOAVServiceWriteI2C() end up in
 * DCPAVServiceProxy::readI2C() / ::writeI2C() (DCPAVFamilyProxy.kext), which
 * issue EPIC service calls on this same "dcpav-service-epic" service:
 *
 *	read	group 1, command 9
 *	write	group 1, command 10
 *
 * Both carry a 0x40 byte parameter block, one parameter per 16 byte slot,
 * followed by the payload. The firmware fills in the result code, and for
 * reads the data, in the reply. The group is 1 whenever a payload is present,
 * matching copy_edid (group 1, command 7) above.
 *
 * Verified against the macOS 13.5 (22G74) kernelcache for Mac14,2, which is
 * the DCP firmware version this driver loads.
 */

#define DPAVSERV_CMD_READ_I2C		9
#define DPAVSERV_CMD_WRITE_I2C		10

/* DDC/CI needs 32 bytes, an EDID block 128. */
#define DPAVSERV_I2C_MAX_LEN		256

struct dpavserv_read_i2c_cmd {
	__le32 chip_addr;
	u8 _pad0[12];
	__le32 data_addr;
	u8 _pad1[12];
	__le32 size;
	u8 _pad2[12];
	__le32 retcode;
	u8 _pad3[12];
	u8 data[];
} __packed;
static_assert(sizeof(struct dpavserv_read_i2c_cmd) == 0x40);

struct dpavserv_write_i2c_cmd {
	__le32 retcode;
	u8 _pad0[12];
	__le32 chip_addr;
	u8 _pad1[12];
	__le32 data_addr;
	u8 _pad2[12];
	__le32 size;
	u8 _pad3[12];
	u8 data[];
} __packed;
static_assert(sizeof(struct dpavserv_write_i2c_cmd) == 0x40);

int dcpavserv_read_i2c(struct apple_epic_service *service, u32 chip_addr,
		       u32 data_addr, void *data, size_t len)
{
	struct dpavserv_read_i2c_cmd *cmd __free(kfree) = NULL;
	size_t cmd_size;
	int ret;

	if (!len || len > DPAVSERV_I2C_MAX_LEN)
		return -EINVAL;

	cmd_size = sizeof(*cmd) + len;
	cmd = kzalloc(cmd_size, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->chip_addr = cpu_to_le32(chip_addr);
	cmd->data_addr = cpu_to_le32(data_addr);
	cmd->size = cpu_to_le32(len);

	ret = afk_service_call(service, 1, DPAVSERV_CMD_READ_I2C, cmd, cmd_size,
			       0, cmd, cmd_size, 0);
	if (ret)
		return ret;

	if (le32_to_cpu(cmd->retcode))
		return -EIO;

	memcpy(data, cmd->data, len);

	return 0;
}

int dcpavserv_write_i2c(struct apple_epic_service *service, u32 chip_addr,
			u32 data_addr, const void *data, size_t len)
{
	struct dpavserv_write_i2c_cmd *cmd __free(kfree) = NULL;
	size_t cmd_size;
	int ret;

	if (len > DPAVSERV_I2C_MAX_LEN)
		return -EINVAL;

	cmd_size = sizeof(*cmd) + len;
	cmd = kzalloc(cmd_size, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->chip_addr = cpu_to_le32(chip_addr);
	cmd->data_addr = cpu_to_le32(data_addr);
	cmd->size = cpu_to_le32(len);
	if (len)
		memcpy(cmd->data, data, len);

	ret = afk_service_call(service, len ? 1 : 0, DPAVSERV_CMD_WRITE_I2C,
			       cmd, cmd_size, 0, cmd, cmd_size, 0);
	if (ret)
		return ret;

	if (le32_to_cpu(cmd->retcode))
		return -EIO;

	return 0;
}
