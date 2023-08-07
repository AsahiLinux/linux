/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Apple SPI HID transport driver
 *
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on: drivers/input/applespi.c
 *
 * MacBook (Pro) SPI keyboard and touchpad driver
 *
 * Copyright (c) 2015-2018 Federico Lorenzi
 * Copyright (c) 2017-2018 Ronald Tschalär
 *
 */

//#define DEBUG 2

#include <asm/unaligned.h>
#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/device/driver.h>
#include <linux/hid.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>

#include "spi-hid-apple.h"

#define SPIHID_DEF_WAIT msecs_to_jiffies(1000)

#define SPIHID_MAX_INPUT_REPORT_SIZE 0x800

/* support only keyboard, trackpad and management dev for now */
#define SPIHID_MAX_DEVICES 3

#define SPIHID_DEVICE_ID_MNGT 0x0
#define SPIHID_DEVICE_ID_KBD 0x1
#define SPIHID_DEVICE_ID_TP 0x2
#define SPIHID_DEVICE_ID_INFO 0xd0

#define SPIHID_READ_PACKET 0x20
#define SPIHID_WRITE_PACKET 0x40

#define SPIHID_DESC_MAX 512

#define SPIHID_SET_LEDS 0x0151 /* caps lock */

#define SPI_RW_CHG_DELAY_US 200 /* 'Inter Stage Us'? */

static const u8 spi_hid_apple_booted[4] = { 0xa0, 0x80, 0x00, 0x00 };
static const u8 spi_hid_apple_status_ok[4] = { 0xac, 0x27, 0x68, 0xd5 };

struct spihid_interface {
	struct hid_device *hid;
	u8 *hid_desc;
	u32 hid_desc_len;
	u32 id;
	unsigned country;
	u32 max_control_report_len;
	u32 max_input_report_len;
	u32 max_output_report_len;
	u8 name[32];
	bool ready;
};

struct spihid_input_report {
	u8 *buf;
	u32 length;
	u32 offset;
	u8 device;
	u8 flags;
};

struct spihid_apple {
	struct spi_device *spidev;

	struct spihid_apple_ops *ops;

	struct spihid_interface mngt;
	struct spihid_interface kbd;
	struct spihid_interface tp;

	wait_queue_head_t wait;
	struct mutex tx_lock; //< protects against concurrent SPI writes

	struct spi_message rx_msg;
	struct spi_message tx_msg;
	struct spi_transfer rx_transfer;
	struct spi_transfer tx_transfer;
	struct spi_transfer status_transfer;

	u8 *rx_buf;
	u8 *tx_buf;
	u8 *status_buf;

	u8 vendor[32];
	u8 product[64];
	u8 serial[32];

	u32 num_devices;

	u32 vendor_id;
	u32 product_id;
	u32 version_number;

	u8 msg_id;

	/* fragmented HID report */
	struct spihid_input_report report;

	/* state tracking flags */
	bool status_booted;

#ifdef IRQ_WAKE_SUPPORT
	bool irq_wake_enabled;
#endif
};

/**
 * struct spihid_msg_hdr - common header of protocol messages.
 *
 * Each message begins with fixed header, followed by a message-type specific
 * payload, and ends with a 16-bit crc. Because of the varying lengths of the
 * payload, the crc is defined at the end of each payload struct, rather than
 * in this struct.
 *
 * @unknown0:	request type? output, input (0x10), feature, protocol
 * @unknown1:	maybe report id?
 * @unknown2:	mostly zero, in info request maybe device num
 * @msgid:	incremented on each message, rolls over after 255; there is a
 *		separate counter for each message type.
 * @rsplen:	response length (the exact nature of this field is quite
 *		speculative). On a request/write this is often the same as
 *		@length, though in some cases it has been seen to be much larger
 *		(e.g. 0x400); on a response/read this the same as on the
 *		request; for reads that are not responses it is 0.
 * @length:	length of the remainder of the data in the whole message
 *		structure (after re-assembly in case of being split over
 *		multiple spi-packets), minus the trailing crc. The total size
 *		of a message is therefore @length + 10.
 */

struct spihid_msg_hdr {
	u8 unknown0;
	u8 unknown1;
	u8 unknown2;
	u8 id;
	__le16 rsplen;
	__le16 length;
};

/**
 * struct spihid_transfer_packet - a complete spi packet; always 256 bytes. This carries
 * the (parts of the) message in the data. But note that this does not
 * necessarily contain a complete message, as in some cases (e.g. many
 * fingers pressed) the message is split over multiple packets (see the
 * @offset, @remain, and @length fields). In general the data parts in
 * spihid_transfer_packet's are concatenated until @remaining is 0, and the
 * result is an message.
 *
 * @flags:	0x40 = write (to device), 0x20 = read (from device); note that
 *		the response to a write still has 0x40.
 * @device:	1 = keyboard, 2 = touchpad
 * @offset:	specifies the offset of this packet's data in the complete
 *		message; i.e. > 0 indicates this is a continuation packet (in
 *		the second packet for a message split over multiple packets
 *		this would then be the same as the @length in the first packet)
 * @remain:	number of message bytes remaining in subsequents packets (in
 *		the first packet of a message split over two packets this would
 *		then be the same as the @length in the second packet)
 * @length:	length of the valid data in the @data in this packet
 * @data:	all or part of a message
 * @crc16:	crc over this whole structure minus this @crc16 field. This
 *		covers just this packet, even on multi-packet messages (in
 *		contrast to the crc in the message).
 */
struct spihid_transfer_packet {
	u8 flags;
	u8 device;
	__le16 offset;
	__le16 remain;
	__le16 length;
	u8 data[246];
	__le16 crc16;
};

/*
 * how HID is mapped onto the protocol is not fully clear. This are the known
 * reports/request:
 *
 *			pkt.flags	pkt.dev?	msg.u0	msg.u1	msg.u2
 * info			0x40		0xd0		0x20	0x01	0xd0
 *
 * info mngt:		0x40		0xd0		0x20	0x10	0x00
 * info kbd:		0x40		0xd0		0x20	0x10	0x01
 * info tp:		0x40		0xd0		0x20	0x10	0x02
 *
 * desc kbd:		0x40		0xd0		0x20	0x10	0x01
 * desc trackpad:	0x40		0xd0		0x20	0x10	0x02
 *
 * mt mode:		0x40		0x02		0x52	0x02	0x00	set protocol?
 * capslock led		0x40		0x01		0x51	0x01	0x00	output report
 *
 * report kbd:		0x20		0x01		0x10	0x01	0x00	input report
 * report tp:		0x20		0x02		0x10	0x02	0x00	input report
 *
 */


static int spihid_apple_request(struct spihid_apple *spihid, u8 target, u8 unk0,
				u8 unk1, u8 unk2, u16 resp_len, u8 *buf,
				    size_t len)
{
	struct spihid_transfer_packet *pkt;
	struct spihid_msg_hdr *hdr;
	u16 crc;
	int err;

	/* know reports are small enoug to fit in a single packet */
	if (len > sizeof(pkt->data) - sizeof(*hdr) - sizeof(__le16))
		return -EINVAL;

	err = mutex_lock_interruptible(&spihid->tx_lock);
	if (err < 0)
		return err;

	pkt = (struct spihid_transfer_packet *)spihid->tx_buf;

	memset(pkt, 0, sizeof(*pkt));
	pkt->flags = SPIHID_WRITE_PACKET;
	pkt->device = target;
	pkt->length = cpu_to_le16(sizeof(*hdr) + len + sizeof(__le16));

	hdr = (struct spihid_msg_hdr *)&pkt->data[0];
	hdr->unknown0 = unk0;
	hdr->unknown1 = unk1;
	hdr->unknown2 = unk2;
	hdr->id = spihid->msg_id++;
	hdr->rsplen = cpu_to_le16(resp_len);
	hdr->length = cpu_to_le16(len);

	if (len)
		memcpy(pkt->data + sizeof(*hdr), buf, len);
	crc = crc16(0, &pkt->data[0], sizeof(*hdr) + len);
	put_unaligned_le16(crc, pkt->data + sizeof(*hdr) + len);

	pkt->crc16 = cpu_to_le16(crc16(0, spihid->tx_buf,
				 offsetof(struct spihid_transfer_packet, crc16)));

	memset(spihid->status_buf, 0, sizeof(spi_hid_apple_status_ok));

	err = spi_sync(spihid->spidev, &spihid->tx_msg);

	if (memcmp(spihid->status_buf, spi_hid_apple_status_ok,
		   sizeof(spi_hid_apple_status_ok))) {
		u8 *b = spihid->status_buf;
		dev_warn_ratelimited(&spihid->spidev->dev, "status message "
				     "mismatch: %02x %02x %02x %02x\n",
				     b[0], b[1], b[2], b[3]);
	}
	mutex_unlock(&spihid->tx_lock);
	if (err < 0)
		return err;

	return (int)len;
}

static struct spihid_apple *spihid_get_data(struct spihid_interface *idev)
{
	switch (idev->id) {
	case SPIHID_DEVICE_ID_KBD:
		return container_of(idev, struct spihid_apple, kbd);
	case SPIHID_DEVICE_ID_TP:
		return container_of(idev, struct spihid_apple, tp);
	default:
		return NULL;
	}
}

static int apple_ll_start(struct hid_device *hdev)
{
	/* no-op SPI transport is already setup */
	return 0;
};

static void apple_ll_stop(struct hid_device *hdev)
{
	/* no-op, devices will be desstroyed on driver destruction */
}

static int apple_ll_open(struct hid_device *hdev)
{
	struct spihid_apple *spihid;
	struct spihid_interface *idev = hdev->driver_data;

	if (idev->hid_desc_len == 0) {
		spihid = spihid_get_data(idev);
		dev_warn(&spihid->spidev->dev,
			 "HID descriptor missing for dev %u", idev->id);
	} else
		idev->ready = true;

	return 0;
}

static void apple_ll_close(struct hid_device *hdev)
{
	struct spihid_interface *idev = hdev->driver_data;
	idev->ready = false;
}

static int apple_ll_parse(struct hid_device *hdev)
{
	struct spihid_interface *idev = hdev->driver_data;

	return hid_parse_report(hdev, idev->hid_desc, idev->hid_desc_len);
}

static int apple_ll_raw_request(struct hid_device *hdev,
				unsigned char reportnum, __u8 *buf, size_t len,
				unsigned char rtype, int reqtype)
{
	struct spihid_interface *idev = hdev->driver_data;
	struct spihid_apple *spihid = spihid_get_data(idev);

	dev_dbg(&spihid->spidev->dev,
		"apple_ll_raw_request: device:%u reportnum:%hhu rtype:%hhu",
		idev->id, reportnum, rtype);

	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		return -EINVAL; // spihid_get_raw_report();
	case HID_REQ_SET_REPORT:
		if (buf[0] != reportnum)
			return -EINVAL;
		if (reportnum != idev->id) {
			dev_warn(&spihid->spidev->dev,
				 "device:%u reportnum:"
				 "%hhu mismatch",
				 idev->id, reportnum);
			return -EINVAL;
		}
		return spihid_apple_request(spihid, idev->id, 0x52, reportnum, 0x00, 2, buf, len);
	default:
		return -EIO;
	}
}

static int apple_ll_output_report(struct hid_device *hdev, __u8 *buf,
				  size_t len)
{
	struct spihid_interface *idev = hdev->driver_data;
	struct spihid_apple *spihid = spihid_get_data(idev);
	if (!spihid)
		return -1;

	dev_dbg(&spihid->spidev->dev,
		"apple_ll_output_report: device:%u len:%zu:",
		idev->id, len);
	// second idev->id should maybe be buf[0]?
	return spihid_apple_request(spihid, idev->id, 0x51, idev->id, 0x00, 0, buf, len);
}

static struct hid_ll_driver apple_hid_ll = {
	.start = &apple_ll_start,
	.stop = &apple_ll_stop,
	.open = &apple_ll_open,
	.close = &apple_ll_close,
	.parse = &apple_ll_parse,
	.raw_request = &apple_ll_raw_request,
	.output_report = &apple_ll_output_report,
};

static struct spihid_interface *spihid_get_iface(struct spihid_apple *spihid,
						 u32 iface)
{
	switch (iface) {
	case SPIHID_DEVICE_ID_MNGT:
		return &spihid->mngt;
	case SPIHID_DEVICE_ID_KBD:
		return &spihid->kbd;
	case SPIHID_DEVICE_ID_TP:
		return &spihid->tp;
	default:
		return NULL;
	}
}

static int spihid_verify_msg(struct spihid_apple *spihid, u8 *buf, size_t len)
{
	u16 msg_crc, crc;
	struct device *dev = &spihid->spidev->dev;

	crc = crc16(0, buf, len - sizeof(__le16));
	msg_crc = get_unaligned_le16(buf + len - sizeof(__le16));
	if (crc != msg_crc) {
		dev_warn_ratelimited(dev, "Read message crc mismatch\n");
		return 0;
	}
	return 1;
}

static bool spihid_status_report(struct spihid_apple *spihid, u8 *pl,
				 size_t len)
{
	struct device *dev = &spihid->spidev->dev;
	dev_dbg(dev, "%s: len: %zu", __func__, len);
	if (len == 5 && pl[0] == 0xe0)
		return true;

	return false;
}

static bool spihid_process_input_report(struct spihid_apple *spihid, u32 device,
					struct spihid_msg_hdr *hdr, u8 *payload,
					size_t len)
{
	//dev_dbg(&spihid>spidev->dev, "input report: req:%hx iface:%u ", hdr->unknown0, device);
	if (hdr->unknown0 != 0x10)
		return false;

	/* HID device as well but Vendor usage only, handle it internally for now */
	if (device == 0) {
		if (hdr->unknown1 == 0xe0) {
			return spihid_status_report(spihid, payload, len);
		}
	} else if (device < SPIHID_MAX_DEVICES) {
		struct spihid_interface *iface =
			spihid_get_iface(spihid, device);
		if (iface && iface->hid && iface->ready) {
			hid_input_report(iface->hid, HID_INPUT_REPORT, payload,
					 len, 1);
			return true;
		}
	} else
		dev_dbg(&spihid->spidev->dev,
			"unexpected iface:%u for input report", device);

	return false;
}

struct spihid_device_info {
	__le16 u0[2];
	__le16 num_devices;
	__le16 vendor_id;
	__le16 product_id;
	__le16 version_number;
	__le16 vendor_str[2]; //< offset and string length
	__le16 product_str[2]; //< offset and string length
	__le16 serial_str[2]; //< offset and string length
};

static bool spihid_process_device_info(struct spihid_apple *spihid, u32 iface,
				       u8 *payload, size_t len)
{
	struct device *dev = &spihid->spidev->dev;

	if (iface != SPIHID_DEVICE_ID_INFO)
		return false;

	if (spihid->vendor_id == 0 &&
	    len >= sizeof(struct spihid_device_info)) {
		struct spihid_device_info *info =
			(struct spihid_device_info *)payload;
		u16 voff, vlen, poff, plen, soff, slen;
		u32 num_devices;

		num_devices = __le16_to_cpu(info->num_devices);

		if (num_devices < SPIHID_MAX_DEVICES) {
			dev_err(dev,
				"Device info reports %u devices, expecting at least 3",
				num_devices);
			return false;
		}
		spihid->num_devices = num_devices;

		if (spihid->num_devices > SPIHID_MAX_DEVICES) {
			dev_info(
				dev,
				"limiting the number of devices to mngt, kbd and mouse");
			spihid->num_devices = SPIHID_MAX_DEVICES;
		}

		spihid->vendor_id = __le16_to_cpu(info->vendor_id);
		spihid->product_id = __le16_to_cpu(info->product_id);
		spihid->version_number = __le16_to_cpu(info->version_number);

		voff = __le16_to_cpu(info->vendor_str[0]);
		vlen = __le16_to_cpu(info->vendor_str[1]);

		if (voff < len && vlen <= len - voff &&
		    vlen < sizeof(spihid->vendor)) {
			memcpy(spihid->vendor, payload + voff, vlen);
			spihid->vendor[vlen] = '\0';
		}

		poff = __le16_to_cpu(info->product_str[0]);
		plen = __le16_to_cpu(info->product_str[1]);

		if (poff < len && plen <= len - poff &&
		    plen < sizeof(spihid->product)) {
			memcpy(spihid->product, payload + poff, plen);
			spihid->product[plen] = '\0';
		}

		soff = __le16_to_cpu(info->serial_str[0]);
		slen = __le16_to_cpu(info->serial_str[1]);

		if (soff < len && slen <= len - soff &&
		    slen < sizeof(spihid->serial)) {
			memcpy(spihid->vendor, payload + soff, slen);
			spihid->serial[slen] = '\0';
		}

		wake_up_interruptible(&spihid->wait);
	}
	return true;
}

struct spihid_iface_info {
	u8 u_0;
	u8 interface_num;
	u8 u_2;
	u8 u_3;
	u8 u_4;
	u8 country_code;
	__le16 max_input_report_len;
	__le16 max_output_report_len;
	__le16 max_control_report_len;
	__le16 name_offset;
	__le16 name_length;
};

static bool spihid_process_iface_info(struct spihid_apple *spihid, u32 num,
				      u8 *payload, size_t len)
{
	struct spihid_iface_info *info;
	struct spihid_interface *iface = spihid_get_iface(spihid, num);
	u32 name_off, name_len;

	if (!iface)
		return false;

	if (!iface->max_input_report_len) {
		if (len < sizeof(*info))
			return false;

		info = (struct spihid_iface_info *)payload;

		iface->max_input_report_len =
			le16_to_cpu(info->max_input_report_len);
		iface->max_output_report_len =
			le16_to_cpu(info->max_output_report_len);
		iface->max_control_report_len =
			le16_to_cpu(info->max_control_report_len);
		iface->country = info->country_code;

		name_off = le16_to_cpu(info->name_offset);
		name_len = le16_to_cpu(info->name_length);

		if (name_off < len && name_len <= len - name_off &&
		    name_len < sizeof(iface->name)) {
			memcpy(iface->name, payload + name_off, name_len);
			iface->name[name_len] = '\0';
		}

		dev_dbg(&spihid->spidev->dev, "Info for %s, country code: 0x%x",
			iface->name, iface->country);

		wake_up_interruptible(&spihid->wait);
	}

	return true;
}

static int spihid_register_hid_device(struct spihid_apple *spihid,
				      struct spihid_interface *idev, u8 device);

static bool spihid_process_iface_hid_report_desc(struct spihid_apple *spihid,
						 u32 num, u8 *payload,
						 size_t len)
{
	struct spihid_interface *iface = spihid_get_iface(spihid, num);

	if (!iface)
		return false;

	if (iface->hid_desc_len == 0) {
		if (len > SPIHID_DESC_MAX)
			return false;
		memcpy(iface->hid_desc, payload, len);
		iface->hid_desc_len = len;

		/* do not register the mngt iface as HID device */
		if (num > 0)
			spihid_register_hid_device(spihid, iface, num);

		wake_up_interruptible(&spihid->wait);
	}
	return true;
}

static bool spihid_process_response(struct spihid_apple *spihid,
				    struct spihid_msg_hdr *hdr, u8 *payload,
				    size_t len)
{
	if (hdr->unknown0 == 0x20) {
		switch (hdr->unknown1) {
		case 0x01:
			return spihid_process_device_info(spihid, hdr->unknown2,
							  payload, len);
		case 0x02:
			return spihid_process_iface_info(spihid, hdr->unknown2,
							 payload, len);
		case 0x10:
			return spihid_process_iface_hid_report_desc(
				spihid, hdr->unknown2, payload, len);
		default:
			break;
		}
	}

	return false;
}

static void spihid_process_message(struct spihid_apple *spihid, u8 *data,
				   size_t length, u8 device, u8 flags)
{
	struct device *dev = &spihid->spidev->dev;
	struct spihid_msg_hdr *hdr;
	bool handled = false;
	u8 *payload;

	if (!spihid_verify_msg(spihid, data, length))
		return;

	hdr = (struct spihid_msg_hdr *)data;

	if (hdr->length == 0)
		return;

	payload = data + sizeof(struct spihid_msg_hdr);

	switch (flags) {
	case SPIHID_READ_PACKET:
		handled = spihid_process_input_report(spihid, device, hdr,
						      payload, le16_to_cpu(hdr->length));
		break;
	case SPIHID_WRITE_PACKET:
		handled = spihid_process_response(spihid, hdr, payload,
						  le16_to_cpu(hdr->length));
		break;
	default:
		break;
	}

#if defined(DEBUG) && DEBUG > 1
	{
		dev_dbg(dev,
			"R msg: req:%02hhx rep:%02hhx dev:%02hhx id:%hu len:%hu\n",
			hdr->unknown0, hdr->unknown1, hdr->unknown2, hdr->id,
			hdr->length);
		print_hex_dump_debug("spihid msg: ", DUMP_PREFIX_OFFSET, 16, 1,
				     payload, le16_to_cpu(hdr->length), true);
	}
#else
	if (!handled) {
		dev_dbg(dev,
			"R unhandled msg: req:%02hhx rep:%02hhx dev:%02hhx id:%hu len:%hu\n",
			hdr->unknown0, hdr->unknown1, hdr->unknown2, hdr->id,
			hdr->length);
		print_hex_dump_debug("spihid msg: ", DUMP_PREFIX_OFFSET, 16, 1,
				     payload, le16_to_cpu(hdr->length), true);
	}
#endif
}

static void spihid_assemble_message(struct spihid_apple *spihid,
				    struct spihid_transfer_packet *pkt)
{
	size_t length, offset, remain;
	struct device *dev = &spihid->spidev->dev;
	struct spihid_input_report *rep = &spihid->report;

	length = le16_to_cpu(pkt->length);
	remain = le16_to_cpu(pkt->remain);
	offset = le16_to_cpu(pkt->offset);

	if (offset + length + remain > U16_MAX) {
		return;
	}

	if (pkt->device != rep->device || pkt->flags != rep->flags ||
	    offset != rep->offset) {
		rep->device = 0;
		rep->flags = 0;
		rep->offset = 0;
		rep->length = 0;
	}

	if (offset == 0) {
		if (rep->offset != 0) {
			dev_warn(dev, "incomplete report off:%u len:%u",
				 rep->offset, rep->length);
		}
		memcpy(rep->buf, pkt->data, length);
		rep->offset = length;
		rep->length = length + remain;
		rep->device = pkt->device;
		rep->flags = pkt->flags;
	} else if (offset == rep->offset) {
		if (offset + length + remain != rep->length) {
			dev_warn(dev, "incomplete report off:%u len:%u",
				 rep->offset, rep->length);
			return;
		}
		memcpy(rep->buf + offset, pkt->data, length);
		rep->offset += length;

		if (rep->offset == rep->length) {
			spihid_process_message(spihid, rep->buf, rep->length,
					       rep->device, rep->flags);
			rep->device = 0;
			rep->flags = 0;
			rep->offset = 0;
			rep->length = 0;
		}
	}
}

static void spihid_process_read(struct spihid_apple *spihid)
{
	u16 crc;
	size_t length;
	struct device *dev = &spihid->spidev->dev;
	struct spihid_transfer_packet *pkt;

	pkt = (struct spihid_transfer_packet *)spihid->rx_buf;

	/* check transfer packet crc */
	crc = crc16(0, spihid->rx_buf,
		    offsetof(struct spihid_transfer_packet, crc16));
	if (crc != le16_to_cpu(pkt->crc16)) {
		dev_warn_ratelimited(dev, "Read package crc mismatch\n");
		return;
	}

	length = le16_to_cpu(pkt->length);

	if (length < sizeof(struct spihid_msg_hdr) + 2) {
		if (length == sizeof(spi_hid_apple_booted) &&
		    !memcmp(pkt->data, spi_hid_apple_booted, length)) {
			if (!spihid->status_booted) {
				spihid->status_booted = true;
				wake_up_interruptible(&spihid->wait);
			}
		} else {
			dev_info(dev, "R short packet: len:%zu\n", length);
			print_hex_dump(KERN_INFO, "spihid pkt:",
				       DUMP_PREFIX_OFFSET, 16, 1, pkt->data,
				       length, false);
		}
		return;
	}

#if defined(DEBUG) && DEBUG > 1
	dev_dbg(dev,
		"R pkt: flags:%02hhx dev:%02hhx off:%hu remain:%hu, len:%zu\n",
		pkt->flags, pkt->device, pkt->offset, pkt->remain, length);
#if defined(DEBUG) && DEBUG > 2
	print_hex_dump_debug("spihid pkt: ", DUMP_PREFIX_OFFSET, 16, 1,
			     spihid->rx_buf,
			     sizeof(struct spihid_transfer_packet), true);
#endif
#endif

	if (length > sizeof(pkt->data)) {
		dev_warn_ratelimited(dev, "Invalid pkt len:%zu", length);
		return;
	}

	/* short message */
	if (pkt->offset == 0 && pkt->remain == 0) {
		spihid_process_message(spihid, pkt->data, length, pkt->device,
				       pkt->flags);
	} else {
		spihid_assemble_message(spihid, pkt);
	}
}

static void spihid_read_packet_sync(struct spihid_apple *spihid)
{
	int err;

	err = spi_sync(spihid->spidev, &spihid->rx_msg);
	if (!err) {
		spihid_process_read(spihid);
	} else {
		dev_warn(&spihid->spidev->dev, "RX failed: %d\n", err);
	}
}

irqreturn_t spihid_apple_core_irq(int irq, void *data)
{
	struct spi_device *spi = data;
	struct spihid_apple *spihid = spi_get_drvdata(spi);

	spihid_read_packet_sync(spihid);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(spihid_apple_core_irq);

static void spihid_apple_setup_spi_msgs(struct spihid_apple *spihid)
{
	memset(&spihid->rx_transfer, 0, sizeof(spihid->rx_transfer));

	spihid->rx_transfer.rx_buf = spihid->rx_buf;
	spihid->rx_transfer.len = sizeof(struct spihid_transfer_packet);

	spi_message_init(&spihid->rx_msg);
	spi_message_add_tail(&spihid->rx_transfer, &spihid->rx_msg);

	memset(&spihid->tx_transfer, 0, sizeof(spihid->rx_transfer));
	memset(&spihid->status_transfer, 0, sizeof(spihid->status_transfer));

	spihid->tx_transfer.tx_buf = spihid->tx_buf;
	spihid->tx_transfer.len = sizeof(struct spihid_transfer_packet);
	spihid->tx_transfer.delay.unit = SPI_DELAY_UNIT_USECS;
	spihid->tx_transfer.delay.value = SPI_RW_CHG_DELAY_US;

	spihid->status_transfer.rx_buf = spihid->status_buf;
	spihid->status_transfer.len = sizeof(spi_hid_apple_status_ok);

	spi_message_init(&spihid->tx_msg);
	spi_message_add_tail(&spihid->tx_transfer, &spihid->tx_msg);
	spi_message_add_tail(&spihid->status_transfer, &spihid->tx_msg);
}

static int spihid_apple_setup_spi(struct spihid_apple *spihid)
{
	spihid_apple_setup_spi_msgs(spihid);

	return spihid->ops->power_on(spihid->ops);
}

static int spihid_register_hid_device(struct spihid_apple *spihid,
				      struct spihid_interface *iface, u8 device)
{
	int ret;
	struct hid_device *hid;

	iface->id = device;

	hid = hid_allocate_device();
	if (IS_ERR(hid))
		return PTR_ERR(hid);

	strscpy(hid->name, spihid->product, sizeof(hid->name));
	snprintf(hid->phys, sizeof(hid->phys), "%s (%hhx)",
		 dev_name(&spihid->spidev->dev), device);
	strscpy(hid->uniq, spihid->serial, sizeof(hid->uniq));

	hid->ll_driver = &apple_hid_ll;
	hid->bus = BUS_SPI;
	hid->vendor = spihid->vendor_id;
	hid->product = spihid->product_id;
	hid->version = spihid->version_number;

	if (device == SPIHID_DEVICE_ID_KBD)
		hid->type = HID_TYPE_SPI_KEYBOARD;
	else if (device == SPIHID_DEVICE_ID_TP)
		hid->type = HID_TYPE_SPI_MOUSE;

	hid->country = iface->country;
	hid->dev.parent = &spihid->spidev->dev;
	hid->driver_data = iface;

	ret = hid_add_device(hid);
	if (ret < 0) {
		hid_destroy_device(hid);
		dev_warn(&spihid->spidev->dev,
			 "Failed to register hid device %hhu", device);
		return ret;
	}

	iface->hid = hid;

	return 0;
}

static void spihid_destroy_hid_device(struct spihid_interface *iface)
{
	if (iface->hid) {
		hid_destroy_device(iface->hid);
		iface->hid = NULL;
	}
	iface->ready = false;
}

int spihid_apple_core_probe(struct spi_device *spi, struct spihid_apple_ops *ops)
{
	struct device *dev = &spi->dev;
	struct spihid_apple *spihid;
	int err, i;

	if (!ops || !ops->power_on || !ops->power_off || !ops->enable_irq || !ops->disable_irq)
		return -EINVAL;

	spihid = devm_kzalloc(dev, sizeof(*spihid), GFP_KERNEL);
	if (!spihid)
		return -ENOMEM;

	spihid->ops = ops;
	spihid->spidev = spi;

	// init spi
	spi_set_drvdata(spi, spihid);

	/* allocate SPI buffers */
	spihid->rx_buf = devm_kmalloc(
		&spi->dev, sizeof(struct spihid_transfer_packet), GFP_KERNEL);
	spihid->tx_buf = devm_kmalloc(
		&spi->dev, sizeof(struct spihid_transfer_packet), GFP_KERNEL);
	spihid->status_buf = devm_kmalloc(
		&spi->dev, sizeof(spi_hid_apple_status_ok), GFP_KERNEL);

	if (!spihid->rx_buf || !spihid->tx_buf || !spihid->status_buf)
		return -ENOMEM;

	spihid->report.buf =
		devm_kmalloc(dev, SPIHID_MAX_INPUT_REPORT_SIZE, GFP_KERNEL);

	spihid->kbd.hid_desc = devm_kmalloc(dev, SPIHID_DESC_MAX, GFP_KERNEL);
	spihid->tp.hid_desc = devm_kmalloc(dev, SPIHID_DESC_MAX, GFP_KERNEL);

	if (!spihid->report.buf || !spihid->kbd.hid_desc ||
	    !spihid->tp.hid_desc)
		return -ENOMEM;

	init_waitqueue_head(&spihid->wait);

	mutex_init(&spihid->tx_lock);

	/* Init spi transfer buffers and power device on */
	err = spihid_apple_setup_spi(spihid);
	if (err < 0)
		goto error;

	/* enable HID irq */
	spihid->ops->enable_irq(spihid->ops);

	// wait for boot message
	err = wait_event_interruptible_timeout(spihid->wait,
					       spihid->status_booted,
					       msecs_to_jiffies(1000));
	if (err == 0)
		err = -ENODEV;
	if (err < 0) {
		dev_err(dev, "waiting for device boot failed: %d", err);
		goto error;
	}

	/* request device information */
	dev_dbg(dev, "request device info");
	spihid_apple_request(spihid, 0xd0, 0x20, 0x01, 0xd0, 0, NULL, 0);
	err = wait_event_interruptible_timeout(spihid->wait, spihid->vendor_id,
					       SPIHID_DEF_WAIT);
	if (err == 0)
		err = -ENODEV;
	if (err < 0) {
		dev_err(dev, "waiting for device info failed: %d", err);
		goto error;
	}

	/* request interface information */
	for (i = 0; i < spihid->num_devices; i++) {
		struct spihid_interface *iface = spihid_get_iface(spihid, i);
		if (!iface)
			continue;
		dev_dbg(dev, "request interface info 0x%02x", i);
		spihid_apple_request(spihid, 0xd0, 0x20, 0x02, i,
				     SPIHID_DESC_MAX, NULL, 0);
		err = wait_event_interruptible_timeout(
			spihid->wait, iface->max_input_report_len,
			SPIHID_DEF_WAIT);
	}

	/* request HID report descriptors */
	for (i = 1; i < spihid->num_devices; i++) {
		struct spihid_interface *iface = spihid_get_iface(spihid, i);
		if (!iface)
			continue;
		dev_dbg(dev, "request hid report desc 0x%02x", i);
		spihid_apple_request(spihid, 0xd0, 0x20, 0x10, i,
				     SPIHID_DESC_MAX, NULL, 0);
		wait_event_interruptible_timeout(
			spihid->wait, iface->hid_desc_len, SPIHID_DEF_WAIT);
	}

	return 0;
error:
	return err;
}
EXPORT_SYMBOL_GPL(spihid_apple_core_probe);

void spihid_apple_core_remove(struct spi_device *spi)
{
	struct spihid_apple *spihid = spi_get_drvdata(spi);

	/* destroy input devices */

	spihid_destroy_hid_device(&spihid->tp);
	spihid_destroy_hid_device(&spihid->kbd);

	/* disable irq */
	spihid->ops->disable_irq(spihid->ops);

	/* power SPI device down */
	spihid->ops->power_off(spihid->ops);
}
EXPORT_SYMBOL_GPL(spihid_apple_core_remove);

void spihid_apple_core_shutdown(struct spi_device *spi)
{
	struct spihid_apple *spihid = spi_get_drvdata(spi);

	/* disable irq */
	spihid->ops->disable_irq(spihid->ops);

	/* power SPI device down */
	spihid->ops->power_off(spihid->ops);
}
EXPORT_SYMBOL_GPL(spihid_apple_core_shutdown);

#ifdef CONFIG_PM_SLEEP
static int spihid_apple_core_suspend(struct device *dev)
{
	int ret;
#ifdef IRQ_WAKE_SUPPORT
	int wake_status;
#endif
	struct spihid_apple *spihid = spi_get_drvdata(to_spi_device(dev));

	if (spihid->tp.hid) {
		ret = hid_driver_suspend(spihid->tp.hid, PMSG_SUSPEND);
		if (ret < 0)
			return ret;
	}

	if (spihid->kbd.hid) {
		ret = hid_driver_suspend(spihid->kbd.hid, PMSG_SUSPEND);
		if (ret < 0) {
			if (spihid->tp.hid)
				hid_driver_resume(spihid->tp.hid);
			return ret;
		}
	}

	/* Save some power */
	spihid->ops->disable_irq(spihid->ops);

#ifdef IRQ_WAKE_SUPPORT
	if (device_may_wakeup(dev)) {
		wake_status = spihid->ops->enable_irq_wake(spihid->ops);
		if (!wake_status)
			spihid->irq_wake_enabled = true;
		else
			dev_warn(dev, "Failed to enable irq wake: %d\n",
				wake_status);
	} else {
		spihid->ops->power_off(spihid->ops);
	}
#else
	spihid->ops->power_off(spihid->ops);
#endif

	return 0;
}

static int spihid_apple_core_resume(struct device *dev)
{
	int ret_tp = 0, ret_kbd = 0;
	struct spihid_apple *spihid = spi_get_drvdata(to_spi_device(dev));
#ifdef IRQ_WAKE_SUPPORT
	int wake_status;

	if (!device_may_wakeup(dev)) {
		spihid->ops->power_on(spihid->ops);
	} else if (spihid->irq_wake_enabled) {
		wake_status = spihid->ops->disable_irq_wake(spihid->ops);
		if (!wake_status)
			spihid->irq_wake_enabled = false;
		else
			dev_warn(dev, "Failed to disable irq wake: %d\n",
				wake_status);
	}
#endif

	spihid->ops->enable_irq(spihid->ops);
	spihid->ops->power_on(spihid->ops);

	if (spihid->tp.hid)
		ret_tp = hid_driver_reset_resume(spihid->tp.hid);
	if (spihid->kbd.hid)
		ret_kbd = hid_driver_reset_resume(spihid->kbd.hid);

	if (ret_tp < 0)
		return ret_tp;

	return ret_kbd;
}
#endif

const struct dev_pm_ops spihid_apple_core_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(spihid_apple_core_suspend,
				spihid_apple_core_resume)
};
EXPORT_SYMBOL_GPL(spihid_apple_core_pm);

MODULE_DESCRIPTION("Apple SPI HID transport driver");
MODULE_AUTHOR("Janne Grunau <j@jannau.net>");
MODULE_LICENSE("GPL");
