/*
 * SPDX-License-Identifier: GPL-2.0 OR MIT
 *
 * Apple DockChannel HID transport driver
 *
 * Copyright The Asahi Linux Contributors
 */
#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/hid.h>
#include <linux/slab.h>
#include <linux/soc/apple/dockchannel.h>
#include <linux/of.h>
#include "../hid-ids.h"

#define COMMAND_TIMEOUT_MS 1000
#define START_TIMEOUT_MS 2000

#define MAX_INTERFACES 16

/* Data + checksum */
#define MAX_PKT_SIZE (0xffff + 4)

#define DCHID_CHANNEL_CMD 0x11
#define DCHID_CHANNEL_REPORT 0x12

struct dchid_hdr {
	u8 hdr_len;
	u8 channel;
	u16 length;
	u8 seq;
	u8 iface;
	u16 pad;
} __packed;

#define IFACE_COMM 0

#define FLAGS_GROUP GENMASK(7, 6)
#define FLAGS_REQ GENMASK(5, 0)

#define GROUP_INPUT 0
#define GROUP_OUTPUT 1
#define GROUP_CMD 2

#define REQ_SET_REPORT 0
#define REQ_GET_REPORT 1

struct dchid_subhdr {
	u8 flags;
	u8 unk;
	u16 length;
	u32 retcode;
} __packed;

#define EVENT_GPIO_CMD	0xa0
#define EVENT_INIT	0xf0
#define EVENT_READY	0xf1

struct dchid_init_hdr {
	u8 type;
	u8 unk1;
	u8 unk2;
	u8 iface;
	char name[16];
	u8 more_packets;
	u8 unkpad;
} __packed;

#define INIT_HID_DESCRIPTOR	0
#define INIT_GPIO_REQUEST	1
#define INIT_TERMINATOR		2
#define INIT_PRODUCT_NAME	7

#define CMD_RESET_INTERFACE 0x40
#define CMD_SEND_FIRMWARE 0x95
#define CMD_ENABLE_INTERFACE 0xb4
#define CMD_ACK_GPIO_CMD 0xa1

struct dchid_init_block_hdr {
	u16 type;
	u16 length;
} __packed;

#define MAX_GPIO_NAME 32

struct dchid_gpio_request {
	u16 unk;
	u16 id;
	char name[MAX_GPIO_NAME];
} __packed;

struct dchid_gpio_cmd {
	u8 type;
	u8 iface;
	u8 gpio;
	u8 unk;
	u8 cmd;
} __packed;

struct dchid_gpio_ack {
	u8 type;
	u32 retcode;
	u8 cmd[];
} __packed;

#define STM_REPORT_ID		0x10
#define STM_REPORT_SERIAL	0x11
#define STM_REPORT_KEYBTYPE	0x14

struct dchid_stm_id {
	u8 unk;
	u16 vendor_id;
	u16 product_id;
	u16 version_number;
	u8 unk2;
	u8 unk3;
	u8 keyboard_type;
	u8 serial_length;
	/* Serial follows, but we grab it with a different report. */
} __packed;

#define FW_MAGIC 0x46444948
#define FW_VER 1

struct fw_header {
	u32 magic;
	u32 version;
	u32 hdr_length;
	u32 data_length;
	u32 iface_offset;
} __packed;

struct dchid_work {
	struct work_struct work;
	struct dchid_iface *iface;

	struct dchid_hdr hdr;
	u8 data[];
};

struct dchid_iface {
	struct dockchannel_hid *dchid;
	struct hid_device *hid;
	struct workqueue_struct *wq;

	bool creating;
	struct work_struct create_work;

	int index;
	const char *name;
	const struct device_node *of_node;

	uint8_t tx_seq;
	bool deferred;
	bool starting;
	bool open;
	struct completion ready;

	void *hid_desc;
	size_t hid_desc_len;

	struct gpio_desc *gpio;
	char gpio_name[MAX_GPIO_NAME];
	int gpio_id;

	struct mutex out_mutex;
	u32 out_flags;
	int out_report;
	u32 retcode;
	void *resp_buf;
	size_t resp_size;
	struct completion out_complete;

	u32 keyboard_layout_id;
};

struct dockchannel_hid {
	struct device *dev;
	struct dockchannel *dc;
	struct device_link *helper_link;

	bool id_ready;
	struct dchid_stm_id device_id;
	char serial[64];

	struct dchid_iface *comm;
	struct dchid_iface *ifaces[MAX_INTERFACES];

	u8 pkt_buf[MAX_PKT_SIZE];

	/* Workqueue to asynchronously create HID devices */
	struct workqueue_struct *new_iface_wq;
};

static ssize_t apple_layout_id_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct dchid_iface *iface = hdev->driver_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", iface->keyboard_layout_id);
}

static DEVICE_ATTR_RO(apple_layout_id);

static struct dchid_iface *
dchid_get_interface(struct dockchannel_hid *dchid, int index, const char *name)
{
	struct dchid_iface *iface;

	if (index >= MAX_INTERFACES) {
		dev_err(dchid->dev, "Interface index %d out of range\n", index);
		return NULL;
	}

	if (dchid->ifaces[index])
		return dchid->ifaces[index];

	iface = devm_kzalloc(dchid->dev, sizeof(struct dchid_iface), GFP_KERNEL);
	if (!iface)
		return NULL;

	iface->index = index;
	iface->name = devm_kstrdup(dchid->dev, name, GFP_KERNEL);
	iface->dchid = dchid;
	iface->out_report= -1;
	init_completion(&iface->out_complete);
	init_completion(&iface->ready);
	mutex_init(&iface->out_mutex);
	iface->wq = alloc_ordered_workqueue("dchid-%s", WQ_MEM_RECLAIM, iface->name);
	if (!iface->wq)
		return NULL;

	/* Comm is not a HID subdevice */
	if (!strcmp(name, "comm")) {
		dchid->ifaces[index] = iface;
		return iface;
	}

	iface->of_node = of_get_child_by_name(dchid->dev->of_node, name);
	if (!iface->of_node) {
		dev_warn(dchid->dev, "No OF node for subdevice %s, ignoring.", name);
		return NULL;
	}

	dchid->ifaces[index] = iface;
	return iface;
}

static u32 dchid_checksum(void *p, size_t length)
{
	u32 sum = 0;

	while (length >= 4) {
		sum += get_unaligned_le32(p);
		p += 4;
		length -= 4;
	}

	WARN_ON_ONCE(length);
	return sum;
}

static int dchid_send(struct dchid_iface *iface, u32 flags, void *msg, size_t size)
{
	u32 checksum = 0xffffffff;
	size_t wsize = round_down(size, 4);
	size_t tsize = size - wsize;
	int ret;
	struct {
		struct dchid_hdr hdr;
		struct dchid_subhdr sub;
	} __packed h;

	memset(&h, 0, sizeof(h));
	h.hdr.hdr_len = sizeof(h.hdr);
	h.hdr.channel = DCHID_CHANNEL_CMD;
	h.hdr.length = round_up(size, 4) + sizeof(h.sub);
	h.hdr.seq = iface->tx_seq;
	h.hdr.iface = iface->index;
	h.sub.flags = flags;
	h.sub.length = size;

	ret = dockchannel_send(iface->dchid->dc, &h, sizeof(h));
	if (ret < 0)
		return ret;
	checksum -= dchid_checksum(&h, sizeof(h));

	ret = dockchannel_send(iface->dchid->dc, msg, wsize);
	if (ret < 0)
		return ret;
	checksum -= dchid_checksum(msg, wsize);

	if (tsize) {
		u8 tail[4] = {0, 0, 0, 0};

		memcpy(tail, msg + wsize, tsize);
		ret = dockchannel_send(iface->dchid->dc, tail, sizeof(tail));
		if (ret < 0)
			return ret;
		checksum -= dchid_checksum(tail, sizeof(tail));
	}

	ret = dockchannel_send(iface->dchid->dc, &checksum, sizeof(checksum));
	if (ret < 0)
		return ret;

	return 0;
}

static int dchid_cmd(struct dchid_iface *iface, u32 type, u32 req,
		     void *data, size_t size, void *resp_buf, size_t resp_size)
{
	int ret;
	int report_id = *(u8*)data;

	mutex_lock(&iface->out_mutex);

	WARN_ON(iface->out_report != -1);
	iface->out_report = report_id;
	iface->out_flags = FIELD_PREP(FLAGS_GROUP, type) | FIELD_PREP(FLAGS_REQ, req);
	iface->resp_buf = resp_buf;
	iface->resp_size = resp_size;
	reinit_completion(&iface->out_complete);

	ret = dchid_send(iface, iface->out_flags, data, size);
	if (ret < 0)
		goto done;

	if (!wait_for_completion_timeout(&iface->out_complete, msecs_to_jiffies(COMMAND_TIMEOUT_MS))) {
		dev_err(iface->dchid->dev, "output report 0x%x to iface  %d (%s) timed out\n",
			report_id, iface->index, iface->name);
		ret = -ETIMEDOUT;
		goto done;
	}

	ret = iface->resp_size;
	if (iface->retcode) {
		dev_err(iface->dchid->dev,
			"output report 0x%x to iface %d (%s) failed with err 0x%x\n",
			report_id, iface->index, iface->name, iface->retcode);
		ret = -EIO;
	}

done:
	iface->tx_seq++;
	iface->out_report = -1;
	iface->out_flags = 0;
	iface->resp_buf = NULL;
	iface->resp_size = 0;
	mutex_unlock(&iface->out_mutex);
	return ret;
}

static int dchid_comm_cmd(struct dockchannel_hid *dchid, void *cmd, size_t size)
{
	return dchid_cmd(dchid->comm, GROUP_CMD, REQ_SET_REPORT, cmd, size, NULL, 0);
}

static int dchid_enable_interface(struct dchid_iface *iface)
{
	u8 msg[] = { CMD_ENABLE_INTERFACE, iface->index };

	return dchid_comm_cmd(iface->dchid, msg, sizeof(msg));
}

static int dchid_reset_interface(struct dchid_iface *iface, int state)
{
	u8 msg[] = { CMD_RESET_INTERFACE, 1, iface->index, state };

	return dchid_comm_cmd(iface->dchid, msg, sizeof(msg));
}

static int dchid_send_firmware(struct dchid_iface *iface, void *firmware, size_t size)
{
	struct {
		u8 cmd;
		u8 unk1;
		u8 unk2;
		u8 iface;
		u64 addr;
		u32 size;
	} __packed msg = {
		.cmd = CMD_SEND_FIRMWARE,
		.unk1 = 2,
		.unk2 = 0,
		.iface = iface->index,
		.size = size,
	};
	dma_addr_t addr;
	void *buf = dmam_alloc_coherent(iface->dchid->dev, size, &addr, GFP_KERNEL);

	if (IS_ERR_OR_NULL(buf))
		return buf ? PTR_ERR(buf) : -ENOMEM;

	msg.addr = addr;
	memcpy(buf, firmware, size);
	wmb();

	return dchid_comm_cmd(iface->dchid, &msg, sizeof(msg));
}

static int dchid_get_firmware(struct dchid_iface *iface, void **firmware, size_t *size)
{
	int ret;
	const char *fw_name;
	const struct firmware *fw;
	struct fw_header *hdr;
	u8 *fw_data;

	ret = of_property_read_string(iface->of_node, "firmware-name", &fw_name);
	if (ret) {
		/* Firmware is only for some devices */
		*firmware = NULL;
		*size = 0;
		return 0;
	}

	ret = request_firmware(&fw, fw_name, iface->dchid->dev);
	if (ret)
		return ret;

	hdr = (struct fw_header *)fw->data;

	if (hdr->magic != FW_MAGIC || hdr->version != FW_VER ||
		hdr->hdr_length < sizeof(*hdr) || hdr->hdr_length > fw->size ||
		(hdr->hdr_length + (size_t)hdr->data_length) > fw->size ||
		hdr->iface_offset >= hdr->data_length) {
		dev_warn(iface->dchid->dev, "%s: invalid firmware header\n",
			 fw_name);
		ret = -EINVAL;
		goto done;
	}

	fw_data = devm_kmemdup(iface->dchid->dev, fw->data + hdr->hdr_length,
			       hdr->data_length, GFP_KERNEL);
	if (!fw_data) {
		ret = -ENOMEM;
		goto done;
	}

	if (hdr->iface_offset)
		fw_data[hdr->iface_offset] = iface->index;

	*firmware = fw_data;
	*size = hdr->data_length;

done:
	release_firmware(fw);
	return ret;
}

static int dchid_request_gpio(struct dchid_iface *iface)
{
	char prop_name[MAX_GPIO_NAME + 16];

	if (iface->gpio)
		return 0;

	dev_info(iface->dchid->dev, "Requesting GPIO %s#%d: %s\n",
		 iface->name, iface->gpio_id, iface->gpio_name);

	snprintf(prop_name, sizeof(prop_name), "apple,%s", iface->gpio_name);

	iface->gpio = devm_gpiod_get_index(iface->dchid->dev, prop_name, 0, GPIOD_OUT_LOW);

	if (IS_ERR_OR_NULL(iface->gpio)) {
		dev_err(iface->dchid->dev, "Failed to request GPIO %s-gpios\n", prop_name);
		iface->gpio = NULL;
		return -1;
	}

	return 0;
}

static int dchid_start_interface(struct dchid_iface *iface)
{
	void *fw;
	size_t size;
	int ret;

	if (iface->starting) {
		dev_warn(iface->dchid->dev, "Interface %s is already starting", iface->name);
		return -EINPROGRESS;
	}

	dev_info(iface->dchid->dev, "Starting interface %s\n", iface->name);

	iface->starting = true;

	/* Look to see if we need firmware */
	ret = dchid_get_firmware(iface, &fw, &size);
	if (ret < 0)
		goto err;

	/* If we need a GPIO, make sure we have it. */
	if (iface->gpio_id) {
		ret = dchid_request_gpio(iface);
		if (ret < 0)
			goto err;
	}

	/* Only multi-touch has firmware */
	if (fw && size) {

		/* Send firmware to the device */
		dev_info(iface->dchid->dev, "Sending firmware for %s\n", iface->name);
		ret = dchid_send_firmware(iface, fw, size);
		if (ret < 0) {
			dev_err(iface->dchid->dev, "Failed to send %s firmwareS", iface->name);
			goto err;
		}

		/* After loading firmware, multi-touch needs a reset */
		dev_info(iface->dchid->dev, "Resetting %s\n", iface->name);
		dchid_reset_interface(iface, 0);
		dchid_reset_interface(iface, 2);
	}

	return 0;

err:
	iface->starting = false;
	return ret;
}

static int dchid_start(struct hid_device *hdev)
{
	struct dchid_iface *iface = hdev->driver_data;

	if (iface->keyboard_layout_id) {
		int ret = device_create_file(&hdev->dev, &dev_attr_apple_layout_id);
		if (ret) {
			dev_warn(iface->dchid->dev, "Failed to create apple_layout_id: %d", ret);
			iface->keyboard_layout_id = 0;
		}
	}

	return 0;
};

static void dchid_stop(struct hid_device *hdev)
{
	struct dchid_iface *iface = hdev->driver_data;

	if (iface->keyboard_layout_id)
		device_remove_file(&hdev->dev, &dev_attr_apple_layout_id);
}

static int dchid_open(struct hid_device *hdev)
{
	struct dchid_iface *iface = hdev->driver_data;
	int ret;

	if (!completion_done(&iface->ready)) {
		ret = dchid_start_interface(iface);
		if (ret < 0)
			return ret;

		if (!wait_for_completion_timeout(&iface->ready, msecs_to_jiffies(START_TIMEOUT_MS))) {
			dev_err(iface->dchid->dev, "iface %s start timed out\n", iface->name);
			return -ETIMEDOUT;
		}
	}

	iface->open = true;
	return 0;
}

static void dchid_close(struct hid_device *hdev)
{
	struct dchid_iface *iface = hdev->driver_data;

	iface->open = false;
}

static int dchid_parse(struct hid_device *hdev)
{
	struct dchid_iface *iface = hdev->driver_data;

	return hid_parse_report(hdev, iface->hid_desc, iface->hid_desc_len);
}

/* Note: buf excludes report number! For ease of fetching strings/etc. */
static int dchid_get_report_cmd(struct dchid_iface *iface, u8 reportnum, void *buf, size_t len)
{
	int ret = dchid_cmd(iface, GROUP_CMD, REQ_GET_REPORT, &reportnum, 1, buf, len);

	return ret <= 0 ? ret : ret - 1;
}

/* Note: buf includes report number! */
static int dchid_set_report(struct dchid_iface *iface, void *buf, size_t len)
{
	return dchid_cmd(iface, GROUP_OUTPUT, REQ_SET_REPORT, buf, len, NULL, 0);
}

static int dchid_raw_request(struct hid_device *hdev,
				unsigned char reportnum, __u8 *buf, size_t len,
				unsigned char rtype, int reqtype)
{
	struct dchid_iface *iface = hdev->driver_data;

	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		buf[0] = reportnum;
		return dchid_cmd(iface, GROUP_OUTPUT, REQ_GET_REPORT, &reportnum, 1, buf + 1, len - 1);
	case HID_REQ_SET_REPORT:
		return dchid_set_report(iface, buf, len);
	default:
		return -EIO;
	}

	return 0;
}

static struct hid_ll_driver dchid_ll = {
	.start = &dchid_start,
	.stop = &dchid_stop,
	.open = &dchid_open,
	.close = &dchid_close,
	.parse = &dchid_parse,
	.raw_request = &dchid_raw_request,
};

static void dchid_create_interface_work(struct work_struct *ws)
{
	struct dchid_iface *iface = container_of(ws, struct dchid_iface, create_work);
	struct dockchannel_hid *dchid = iface->dchid;
	struct hid_device *hid;
	int ret;

	if (iface->hid) {
		dev_warn(dchid->dev, "Interface %s already created!\n",
			 iface->name);
		return;
	}

	dev_info(dchid->dev, "New interface %s\n", iface->name);

	/* Start the interface. This is not the entire init process, as firmware is loaded later on device open. */
	ret = dchid_enable_interface(iface);
	if (ret < 0) {
		dev_warn(dchid->dev, "Failed to enable %s: %d\n", iface->name, ret);
		return;
	}

	iface->deferred = false;

	hid = hid_allocate_device();
	if (IS_ERR(hid))
		return;

	snprintf(hid->name, sizeof(hid->name), "Apple MTP %s", iface->name);
	snprintf(hid->phys, sizeof(hid->phys), "%s.%d (%s)",
		 dev_name(dchid->dev), iface->index, iface->name);
	strscpy(hid->uniq, dchid->serial, sizeof(hid->uniq));

	hid->ll_driver = &dchid_ll;
	hid->bus = BUS_HOST;
	hid->vendor = dchid->device_id.vendor_id;
	hid->product = dchid->device_id.product_id;
	hid->version = dchid->device_id.version_number;
	hid->type = HID_TYPE_OTHER;
	if (!strcmp(iface->name, "multi-touch")) {
		hid->type = HID_TYPE_SPI_MOUSE;
	} else if (!strcmp(iface->name, "keyboard")) {
		u32 country_code = 0;

		hid->type = HID_TYPE_SPI_KEYBOARD;

		/*
		 * We have to get the country code from the device tree, since the
		 * device provides no reliable way to get this info.
		 */
		if (!of_property_read_u32(iface->of_node, "hid-country-code", &country_code))
			hid->country = country_code;

		of_property_read_u32(iface->of_node, "apple,keyboard-layout-id",
			&iface->keyboard_layout_id);
	}

	hid->dev.parent = iface->dchid->dev;
	hid->driver_data = iface;

	iface->hid = hid;

	ret = hid_add_device(hid);
	if (ret < 0) {
		iface->hid = NULL;
		hid_destroy_device(hid);
		dev_warn(iface->dchid->dev, "Failed to register hid device %s", iface->name);
	}
}

static int dchid_create_interface(struct dchid_iface *iface)
{
	if (iface->creating)
		return -EBUSY;

	iface->creating = true;
	INIT_WORK(&iface->create_work, dchid_create_interface_work);
	return queue_work(iface->dchid->new_iface_wq, &iface->create_work);
}

static void dchid_handle_descriptor(struct dchid_iface *iface, void *hid_desc, size_t desc_len)
{
	if (iface->hid) {
		dev_warn(iface->dchid->dev, "Tried to initialize already started interface %s!\n",
			 iface->name);
		return;
	}

	iface->hid_desc = devm_kmemdup(iface->dchid->dev, hid_desc, desc_len, GFP_KERNEL);
	if (!iface->hid_desc)
		return;

	iface->hid_desc_len = desc_len;
}

static void dchid_handle_ready(struct dockchannel_hid *dchid, void *data, size_t length)
{
	struct dchid_iface *iface;
	u8 *pkt = data;
	u8 index;
	int i, ret;

	if (length < 2) {
		dev_err(dchid->dev, "Bad length for ready message: %zu\n", length);
		return;
	}

	index = pkt[1];

	if (index >= MAX_INTERFACES) {
		dev_err(dchid->dev, "Got ready notification for bad iface %d\n", index);
		return;
	}

	iface = dchid->ifaces[index];
	if (!iface) {
		dev_err(dchid->dev, "Got ready notification for unknown iface %d\n", index);
		return;
	}

	dev_info(dchid->dev, "Interface %s is now ready\n", iface->name);
	complete_all(&iface->ready);

	/* When STM is ready, grab global device info */
	if (!strcmp(iface->name, "stm")) {
		ret = dchid_get_report_cmd(iface, STM_REPORT_ID, &dchid->device_id,
					   sizeof(dchid->device_id));
		if (ret < sizeof(dchid->device_id)) {
			dev_warn(iface->dchid->dev, "Failed to get device ID from STM!\n");
			/* Fake it and keep going. Things might still work... */
			memset(&dchid->device_id, 0, sizeof(dchid->device_id));
			dchid->device_id.vendor_id = HOST_VENDOR_ID_APPLE;
		}
		ret = dchid_get_report_cmd(iface, STM_REPORT_SERIAL, dchid->serial,
					   sizeof(dchid->serial) - 1);
		if (ret < 0) {
			dev_warn(iface->dchid->dev, "Failed to get serial from STM!\n");
			dchid->serial[0] = 0;
		}

		dchid->id_ready = true;
		for (i = 0; i < MAX_INTERFACES; i++) {
			if (!dchid->ifaces[i] || !dchid->ifaces[i]->deferred)
				continue;
			dchid_create_interface(dchid->ifaces[i]);
		}
	}
}

static void dchid_handle_init(struct dockchannel_hid *dchid, void *data, size_t length)
{
	struct dchid_init_hdr *hdr = data;
	struct dchid_iface *iface;
	struct dchid_init_block_hdr *blk;

	if (length < sizeof(*hdr))
		return;

	iface = dchid_get_interface(dchid, hdr->iface, hdr->name);
	if (!iface)
		return;

	data += sizeof(*hdr);
	length -= sizeof(*hdr);

	while (length >= sizeof(*blk)) {
		blk = data;
		data += sizeof(*blk);
		length -= sizeof(*blk);

		if (blk->length > length)
			break;

		switch (blk->type) {
		case INIT_HID_DESCRIPTOR:
			dchid_handle_descriptor(iface, data, blk->length);
			break;

		case INIT_GPIO_REQUEST: {
			struct dchid_gpio_request *req = data;

			if (sizeof(*req) > length)
				break;

			if (iface->gpio_id) {
				dev_err(dchid->dev,
					"Cannot request more than one GPIO per interface!\n");
				break;
			}

			strlcpy(iface->gpio_name, req->name, MAX_GPIO_NAME);
			iface->gpio_id = req->id;
			break;
		}

		case INIT_TERMINATOR:
			break;

		case INIT_PRODUCT_NAME: {
			char *product = data;

			if (product[blk->length - 1] != 0) {
				dev_warn(dchid->dev, "Unterminated product name for %s\n",
					 iface->name);
			} else {
				dev_info(dchid->dev, "Product name for %s: %s\n",
					 iface->name, product);
			}
			break;
		}

		default:
			dev_warn(dchid->dev, "Unknown init packet %d for %s\n",
				 blk->type, iface->name);
			break;
		}

		data += blk->length;
		length -= blk->length;

		if (blk->type == INIT_TERMINATOR)
			break;
	}

	if (hdr->more_packets)
		return;

	/* We need to enable STM first, since it'll give us the device IDs */
	if (iface->dchid->id_ready || !strcmp(iface->name, "stm")) {
		dchid_create_interface(iface);
	} else {
		iface->deferred = true;
	}
}

static void dchid_handle_gpio(struct dockchannel_hid *dchid, void *data, size_t length)
{
	struct dchid_gpio_cmd *cmd = data;
	struct dchid_iface *iface;
	u32 retcode = 0xe000f00d; /* Give it a random Apple-style error code */
	struct dchid_gpio_ack *ack;

	if (length < sizeof(*cmd))
		return;

	if (cmd->iface >= MAX_INTERFACES || !(iface = dchid->ifaces[cmd->iface])) {
		dev_err(dchid->dev, "Got GPIO command for bad inteface %d\n", cmd->iface);
		goto err;
	}

	if (dchid_request_gpio(iface) < 0)
		goto err;

	if (!iface->gpio || cmd->gpio != iface->gpio_id) {
		dev_err(dchid->dev, "Got GPIO command for bad GPIO %s#%d\n",
			iface->name, cmd->gpio);
		goto err;
	}

	dev_info(dchid->dev, "GPIO command: %s#%d: %d\n", iface->name, cmd->gpio, cmd->cmd);

	switch (cmd->cmd) {
	case 3:
		/* Pulse.  */
		gpiod_set_value_cansleep(iface->gpio, 1);
		msleep(10); /* Random guess... */
		gpiod_set_value_cansleep(iface->gpio, 0);
		retcode = 0;
		break;
	default:
		dev_err(dchid->dev, "Unknown GPIO command %d\n", cmd->cmd	);
		break;
	}

err:
	/* Ack it */
	ack = kzalloc(sizeof(*ack) + length, GFP_KERNEL);
	if (!ack)
		return;

	ack->type = CMD_ACK_GPIO_CMD;
	ack->retcode = retcode;
	memcpy(ack->cmd, data, length);

	if (dchid_comm_cmd(dchid, ack, sizeof(*ack) + length) < 0)
		dev_err(dchid->dev, "Failed to ACK GPIO command\n");

	kfree(ack);
}

static void dchid_handle_event(struct dockchannel_hid *dchid, void *data, size_t length)
{
	u8 *p = data;
	switch (*p) {
	case EVENT_INIT:
		dchid_handle_init(dchid, data, length);
		break;
	case EVENT_READY:
		dchid_handle_ready(dchid, data, length);
		break;
	case EVENT_GPIO_CMD:
		dchid_handle_gpio(dchid, data, length);
		break;
	}
}

static void dchid_handle_report(struct dchid_iface *iface, void *data, size_t length)
{
	struct dockchannel_hid *dchid = iface->dchid;

	if (!iface->hid) {
		dev_warn(dchid->dev, "Report received but %s is not initialized!\n", iface->name);
		return;
	}

	if (!iface->open)
		return;

	hid_input_report(iface->hid, HID_INPUT_REPORT, data, length, 1);
}

static void dchid_packet_work(struct work_struct *ws)
{
	struct dchid_work *work = container_of(ws, struct dchid_work, work);
	struct dchid_subhdr *shdr = (void *)work->data;
	struct dockchannel_hid *dchid = work->iface->dchid;
	int type = FIELD_GET(FLAGS_GROUP, shdr->flags);
	u8 *payload = work->data + sizeof(*shdr);

	if (shdr->length + sizeof(*shdr) > work->hdr.length) {
		dev_err(dchid->dev, "Bad sub header length (%d > %zu)\n",
			shdr->length, work->hdr.length - sizeof(*shdr));
		return;
	}

	switch (type) {
	case GROUP_INPUT:
		if (work->hdr.iface == IFACE_COMM)
			dchid_handle_event(dchid, payload, shdr->length);
		else
			dchid_handle_report(work->iface, payload, shdr->length);
		break;
	default:
		dev_err(dchid->dev, "Received unknown packet type %d\n", type);
		break;
	}

	kfree(work);
}

static void dchid_handle_ack(struct dchid_iface *iface, struct dchid_hdr *hdr, void *data)
{
	struct dchid_subhdr *shdr = (void *)data;
	u8 *payload = data + sizeof(*shdr);

	if (shdr->length + sizeof(*shdr) > hdr->length) {
		dev_err(iface->dchid->dev, "Bad sub header length (%d > %ld)\n",
			shdr->length, hdr->length - sizeof(*shdr));
		return;
	}
	if (shdr->flags != iface->out_flags) {
		dev_err(iface->dchid->dev,
			"Received unexpected flags 0x%x on ACK channel (expFected 0x%x)\n",
			shdr->flags, iface->out_flags);
		return;
	}

	if (shdr->length < 1) {
		dev_err(iface->dchid->dev, "Received length 0 output report ack\n");
		return;
	}
	if (iface->tx_seq != hdr->seq) {
		dev_err(iface->dchid->dev, "Received ACK with bad seq (expected %d, got %d)\n",
			iface->tx_seq, hdr->seq);
		return;
	}
	if (iface->out_report != payload[0]) {
		dev_err(iface->dchid->dev, "Received ACK with bad report (expected %d, got %d\n",
			iface->out_report, payload[0]);
		return;
	}

	if (iface->resp_buf && iface->resp_size)
		memcpy(iface->resp_buf, payload + 1, min((size_t)shdr->length - 1, iface->resp_size));

	iface->resp_size = shdr->length;
	iface->out_report = -1;
	iface->retcode = shdr->retcode;
	complete(&iface->out_complete);
}

static void dchid_handle_packet(void *cookie, size_t avail)
{
	struct dockchannel_hid *dchid = cookie;
	struct dchid_hdr hdr;
	struct dchid_work *work;
	struct dchid_iface *iface;
	u32 checksum;

	if (dockchannel_recv(dchid->dc, &hdr, sizeof(hdr)) != sizeof(hdr)) {
		dev_err(dchid->dev, "Read failed (header)\n");
		return;
	}

	if (hdr.hdr_len != sizeof(hdr)) {
		dev_err(dchid->dev, "Bad header length %d\n", hdr.hdr_len);
		goto done;
	}

	if (dockchannel_recv(dchid->dc, dchid->pkt_buf, hdr.length + 4) != (hdr.length + 4)) {
		dev_err(dchid->dev, "Read failed (body)\n");
		goto done;
	}

	checksum = dchid_checksum(&hdr, sizeof(hdr));
	checksum += dchid_checksum(dchid->pkt_buf, hdr.length + 4);

	if (checksum != 0xffffffff) {
		dev_err(dchid->dev, "Checksum mismatch (iface %d): 0x%08x != 0xffffffff\n",
			hdr.iface, checksum);
		goto done;
	}


	if (hdr.iface >= MAX_INTERFACES) {
		dev_err(dchid->dev, "Bad iface %d\n", hdr.iface);
	}

	iface = dchid->ifaces[hdr.iface];

	if (!iface) {
		dev_err(dchid->dev, "Received packet for uninitialized iface %d\n", hdr.iface);
		goto done;
	}

	switch (hdr.channel) {
		case DCHID_CHANNEL_CMD:
			dchid_handle_ack(iface, &hdr, dchid->pkt_buf);
			goto done;
		case DCHID_CHANNEL_REPORT:
			break;
		default:
			dev_warn(dchid->dev, "Unknown channel 0x%x, treating as report...\n",
				 hdr.channel);
			break;
	}

	work = kzalloc(sizeof(*work) + hdr.length, GFP_KERNEL);
	if (!work)
		return;

	work->hdr = hdr;
	work->iface = iface;
	memcpy(work->data, dchid->pkt_buf, hdr.length);
	INIT_WORK(&work->work, dchid_packet_work);

	queue_work(iface->wq, &work->work);

done:
	dockchannel_await(dchid->dc, dchid_handle_packet, dchid, sizeof(struct dchid_hdr));
}

static int dockchannel_hid_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dockchannel_hid *dchid;
	struct device_node *child, *helper;
	struct platform_device *helper_pdev;
	struct property *prop;
	int ret;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	dchid = devm_kzalloc(dev, sizeof(*dchid), GFP_KERNEL);
	if (!dchid) {
		return -ENOMEM;
	}

	dchid->dev = dev;

	/*
	 * First make sure all the GPIOs are available, in cased we need to defer.
	 * This is necessary because MTP will request them by name later, and by then
	 * it's too late to defer the probe.
	 */

	for_each_child_of_node(dev->of_node, child) {
		for_each_property_of_node(child, prop) {
			size_t len = strlen(prop->name);
			struct gpio_desc *gpio;

			if (len < 12 || strncmp("apple,", prop->name, 6) ||
			    strcmp("-gpios", prop->name + len - 6))
				continue;

			gpio = fwnode_gpiod_get_index(&child->fwnode, prop->name, 0, GPIOD_ASIS,
						      prop->name);
			if (IS_ERR_OR_NULL(gpio)) {
				if (PTR_ERR(gpio) == -EPROBE_DEFER) {
					of_node_put(child);
					return -EPROBE_DEFER;
				}
			} else {
				gpiod_put(gpio);
			}
		}
	}

	/*
	 * Make sure we also have the MTP coprocessor available, and
	 * defer probe if the helper hasn't probed yet.
	 */
	helper = of_parse_phandle(dev->of_node, "apple,helper-cpu", 0);
	if (!helper) {
		dev_err(dev, "Missing apple,helper-cpu property");
		return -EINVAL;
	}

	helper_pdev = of_find_device_by_node(helper);
	of_node_put(helper);
	if (!helper_pdev) {
		dev_err(dev, "Failed to find helper device");
		return -EINVAL;
	}

	dchid->helper_link = device_link_add(dev, &helper_pdev->dev,
					     DL_FLAG_AUTOREMOVE_CONSUMER);
	put_device(&helper_pdev->dev);
	if (!dchid->helper_link) {
		dev_err(dev, "Failed to link to helper device");
		return -EINVAL;
	}

	if (dchid->helper_link->supplier->links.status != DL_DEV_DRIVER_BOUND)
		return -EPROBE_DEFER;

	/* Now it is safe to begin initializing */
	dchid->dc = dockchannel_init(pdev);
	if (IS_ERR_OR_NULL(dchid->dc)) {
		return PTR_ERR(dchid->dc);
	}
	dchid->new_iface_wq = alloc_workqueue("dchid-new", WQ_MEM_RECLAIM, 0);
	if (!dchid->new_iface_wq)
		return -ENOMEM;

	dchid->comm = dchid_get_interface(dchid, IFACE_COMM, "comm");
	if (!dchid->comm) {
		dev_err(dchid->dev, "Failed to initialize comm interface");
		return -EIO;
	}

	dev_info(dchid->dev, "Initialized, awaiting packets\n");
	dockchannel_await(dchid->dc, dchid_handle_packet, dchid, sizeof(struct dchid_hdr));

	return 0;
}

static int dockchannel_hid_remove(struct platform_device *pdev)
{
	BUG_ON(1);
	return 0;
}

static const struct of_device_id dockchannel_hid_of_match[] = {
	{ .compatible = "apple,dockchannel-hid" },
	{},
};
MODULE_DEVICE_TABLE(of, dockchannel_hid_of_match);
MODULE_FIRMWARE("apple/tpmtfw-*.bin");

static struct platform_driver dockchannel_hid_driver = {
	.driver = {
		.name = "dockchannel-hid",
		.of_match_table = dockchannel_hid_of_match,
	},
	.probe = dockchannel_hid_probe,
	.remove = dockchannel_hid_remove,
};
module_platform_driver(dockchannel_hid_driver);

MODULE_DESCRIPTION("Apple DockChannel HID transport driver");
MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_LICENSE("Dual MIT/GPL");
