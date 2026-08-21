// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/soc/apple/actuator.h>
#include <linux/module.h>

#include "hid-ids.h"

int apple_taptic_send(struct hid_device *haptic_hdev, u16 effect_type, u8 strength, u8 softness)
{
	int ret;
	u8 *msg;
	int msg_length;
	u8 event_click[] = {0x53, 0x01, strength, softness, 0x03, 0x02, 0x22, 0x48, 0x49,
			    0x01, 0x04, 0x60, 0x17, 0x2D, 0x03, 0x02, 0x18, 0x32, 0x1E};
	u8 event_deep_click[] = {0x53, 0x01, strength, softness, 0x01, 0x07, 0x18, 0x18, 0x4F,
				 0x02, 0x02, 0x60, 0x0C, 0x25};
	u8 event_release[] = {0x53, 0x01, strength, softness};

	switch (effect_type) {
	case HID_HP_WAVEFORMPRESS & HID_USAGE:
		msg = event_click;
		msg_length = sizeof(event_click);
		break;
	case APPLE_HP_WAVEFORMDEEPCLICK & HID_USAGE:
		msg = event_deep_click;
		msg_length = sizeof(event_deep_click);
		break;
	case HID_HP_WAVEFORMRELEASE & HID_USAGE:
		msg = event_release;
		msg_length = sizeof(event_release);
		break;
	default:
		return -EINVAL;
	}

	msg = kmemdup(msg, msg_length, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	ret = hid_hw_raw_request(haptic_hdev, msg[0], msg, msg_length,
				 HID_OUTPUT_REPORT, HID_REQ_SET_REPORT);
	kfree(msg);

	if (ret < 0)
		return ret;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_taptic_send);

int apple_taptic_switch_modes(struct hid_device *haptic_hdev, bool enable_host_controlled)
{
	int ret;
	u8 *buf;
	u8 msg[] = { 0x21, enable_host_controlled };

	buf = kmemdup(msg, sizeof(msg), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = hid_hw_raw_request(haptic_hdev, buf[0], buf, sizeof(msg),
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	kfree(buf);

	if (ret < 0)
		return ret;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_taptic_switch_modes);

static int apple_actuator_probe(struct hid_device *haptic_hdev, const struct hid_device_id *id)
{
	int ret;

	ret = hid_parse(haptic_hdev);
	if (ret) {
		hid_err(haptic_hdev, "apple_haptic hid parse failed\n");
		return ret;
	}

	ret = hid_hw_start(haptic_hdev, 0);
	if (ret) {
		hid_err(haptic_hdev, "apple_haptic hw start failed\n");
		return ret;
	}

	return 0;
}

static bool actuator_match(struct hid_device *hdev, bool ignore_special_drivers)
{
	return (strcmp(hdev->name, "Apple MTP actuator") == 0);
}

static void actuator_remove(struct hid_device *haptic_hdev)
{
	apple_taptic_switch_modes(haptic_hdev, 0);
	hid_hw_stop(haptic_hdev);
}

static const struct hid_device_id apple_haptic_devices[] = {
	{ HID_DEVICE(BUS_HOST, HID_GROUP_ANY, HOST_VENDOR_ID_APPLE,
		     HID_ANY_ID), .driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(hid, apple_haptic_devices);

static struct hid_driver apple_actuator_driver = {
	.name = "hid-apple-mtp-haptic",
	.id_table = apple_haptic_devices,
	.probe = apple_actuator_probe,
	.match = actuator_match,
	.remove = actuator_remove
};
module_hid_driver(apple_actuator_driver);

MODULE_DESCRIPTION("Apple MTP Haptic Actuator driver");
MODULE_LICENSE("GPL");
