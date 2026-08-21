// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/soc/apple/actuator.h>
#include <linux/module.h>
#include <linux/workqueue.h>

#include "hid-ids.h"
#include "hid-haptic.h"

#define APPLE_HP_WAVEFORMDEEPCLICK	0x000e2001
#define APPLE_HP_SWITCH_MODE_PACKET_ID	0x21
#define APPLE_HP_PLAY_PACKET_ID		0x53

static unsigned char softness = 0x90;
module_param(softness, byte, 0644);
MODULE_PARM_DESC(softness, "Overwrite default haptic click softness value (0x90), range 0-255");

struct effect_job {
	struct work_struct work;
	struct hid_device *actuator_hdev;
	u16 effect_type;
	u8 strength;
	u8 softness;
};

struct actuator {
	struct hid_haptic_device haptic_dev;
	struct hid_device *hid_device;
	struct effect_job *haptic_effects;
};

static bool is_apple_actuator(struct hid_device *hdev)
{
	/* When adding support for new actuators, just add their alternative name here. */
	return !strcmp(hdev->name, "Apple MTP actuator");
}

static int apple_actuator_match_dev(struct device *dev, const void *data)
{
	if (dev->bus != &hid_bus_type)
		return 0;

	return is_apple_actuator(to_hid_device(dev));
}

static bool apple_actuator_match(struct hid_device *hdev, bool ignore_special_drivers)
{
	return is_apple_actuator(hdev);
}

static int apple_actuator_send(struct hid_device *actuator_hid_dev, u16 effect_type,
			       u8 strength, u8 softness)
{
	int ret;
	u8 *msg;
	int msg_length;
	u8 event_click[] = {APPLE_HP_PLAY_PACKET_ID, 0x01, strength, softness, 0x03, 0x02, 0x22,
			    0x48, 0x49, 0x01, 0x04, 0x60, 0x17, 0x2D, 0x03, 0x02, 0x18, 0x32, 0x1E};
	u8 event_deep_click[] = {APPLE_HP_PLAY_PACKET_ID, 0x01, strength, softness, 0x01, 0x07,
				 0x18, 0x18, 0x4F, 0x02, 0x02, 0x60, 0x0C, 0x25};
	u8 event_release[] = {APPLE_HP_PLAY_PACKET_ID, 0x01, strength, softness};

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

	ret = hid_hw_raw_request(actuator_hid_dev, msg[0], msg, msg_length,
				 HID_OUTPUT_REPORT, HID_REQ_SET_REPORT);
	kfree(msg);

	if (ret < 0)
		return ret;
	return 0;
}

static int apple_actuator_switch_modes(struct actuator *actuator,
				       bool enable_host_controlled)
{
	int ret;
	u8 *buf;
	u8 msg[] = { APPLE_HP_SWITCH_MODE_PACKET_ID, enable_host_controlled };

	buf = kmemdup(msg, sizeof(msg), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = hid_hw_raw_request(actuator->hid_device, buf[0], buf, sizeof(msg),
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	kfree(buf);

	if (ret < 0) {
		hid_err(actuator->hid_device, "Unable to switch to mode %d.\n",
			enable_host_controlled);

		return ret;
	}

	actuator->haptic_dev.mode =  enable_host_controlled;
	return 0;
}

static int apple_actuator_upload_effects(struct input_dev *trackpad_idev,
					 struct ff_effect *effect, struct ff_effect *old)
{
	struct actuator *actuator = trackpad_idev->ff->private;
	int ret = 0;

	switch (effect->u.haptic.hid_usage) {
	case (HID_HP_WAVEFORMPRESS & HID_USAGE):
	case (HID_HP_WAVEFORMRELEASE & HID_USAGE):
	case (APPLE_HP_WAVEFORMDEEPCLICK & HID_USAGE):
		break;
	default:
		return -EINVAL;
	}

	actuator->haptic_effects[effect->id].actuator_hdev = actuator->hid_device;
	actuator->haptic_effects[effect->id].effect_type = (effect->u.haptic.hid_usage) & HID_USAGE;
	actuator->haptic_effects[effect->id].strength =
		(min(100, (effect->u.haptic.intensity)) * 255) / 100;
	actuator->haptic_effects[effect->id].softness = softness;

	if (actuator->haptic_dev.mode == HID_HAPTIC_MODE_DEVICE) {
		ret = apple_actuator_switch_modes(actuator, HID_HAPTIC_MODE_HOST);

		if (ret)
			actuator->haptic_effects[effect->id].effect_type = 0;
	}
	return ret;
}

static void haptic_playback_worker(struct work_struct *ws)
{
	struct effect_job *job = container_of(ws, struct effect_job, work);
	struct hid_device *actuator_hdev = job->actuator_hdev;

	if (job->effect_type && apple_actuator_send(actuator_hdev, job->effect_type,
						    job->strength, job->softness))
		hid_err(job->actuator_hdev, "apple-haptic: unable to send haptic event.\n");
}

static int apple_actuator_playback(struct input_dev *trackpad_idev, int effect_id, int value)
{
	struct actuator *actuator = trackpad_idev->ff->private;

	if (value)
		queue_work(actuator->haptic_dev.wq, &actuator->haptic_effects[effect_id].work);

	return 0;
}

static int apple_actuator_erase(struct input_dev *trackpad_idev, int effect_id)
{
	struct actuator *actuator = trackpad_idev->ff->private;
	int i;

	actuator->haptic_effects[effect_id].effect_type = 0;

	for (i = 0; i < FF_MAX_EFFECTS; i++) {
		if (actuator->haptic_effects[i].effect_type != 0)
			return 0;
	}

	/* Return to device-controlled mode if there are no effects left */
	if (actuator->haptic_dev.mode == HID_HAPTIC_MODE_HOST) {
		flush_workqueue(actuator->haptic_dev.wq);

		return apple_actuator_switch_modes(actuator, HID_HAPTIC_MODE_DEVICE);
	}

	return 0;
}

static void apple_actuator_destroy(struct ff_device *ff)
{
	struct actuator *actuator = ff->private;

	if (actuator->haptic_dev.mode == HID_HAPTIC_MODE_HOST) {
		flush_workqueue(actuator->haptic_dev.wq);

		apple_actuator_switch_modes(actuator, HID_HAPTIC_MODE_DEVICE);
		/* The return value is not used as function is void. */
	}

	destroy_workqueue(actuator->haptic_dev.wq);

	kfree(actuator->haptic_effects);

	hid_set_drvdata(actuator->hid_device, NULL);
	put_device(&actuator->hid_device->dev);
}

static int apple_actuator_init(struct input_dev *trackpad_idev,
			       struct actuator *actuator)
{
	struct hid_device *trackpad_hdev = input_get_drvdata(trackpad_idev);
	struct ff_device *ff;
	int ret, i;

	actuator->haptic_effects = kzalloc_objs(struct effect_job, FF_MAX_EFFECTS);
	if (!actuator->haptic_effects)
		return -ENOMEM;

	actuator->haptic_dev.wq = create_singlethread_workqueue("Apple actuator workqueue");
	if (!actuator->haptic_dev.wq) {
		kfree(actuator->haptic_effects);

		return -ENOMEM;
	}

	for (i = 0; i < FF_MAX_EFFECTS; i++)
		INIT_WORK(&actuator->haptic_effects[i].work, haptic_playback_worker);

	ret = input_ff_create(trackpad_idev, FF_MAX_EFFECTS);
	if (ret) {
		kfree(actuator->haptic_effects);

		destroy_workqueue(actuator->haptic_dev.wq);

		return ret;
	}

	ff = trackpad_idev->ff;
	ff->private = actuator;
	ff->upload = apple_actuator_upload_effects;
	ff->playback = apple_actuator_playback;
	ff->erase = apple_actuator_erase;
	ff->destroy = apple_actuator_destroy;

	input_set_capability(trackpad_idev, EV_FF, FF_HAPTIC);

	hid_dbg(trackpad_hdev, "Successfully init'd the haptics\n");
	return 0;
}

int apple_actuator_init_haptics(struct input_dev *trackpad_idev, struct hid_device *trackpad_hdev)
{
	struct device *actuator_dev;
	struct hid_device *actuator_hid_dev;
	struct actuator *actuator;

	int ret;

	/* Check if actuator is allocated */
	actuator_dev = device_find_child(trackpad_hdev->dev.parent, NULL, apple_actuator_match_dev);
	if (actuator_dev) {
		actuator_hid_dev = to_hid_device(actuator_dev);

		hid_dbg(trackpad_hdev, "Successfully bound trackpad drivers to the actuator\n");

		actuator = kzalloc_obj(*actuator, GFP_KERNEL);

		if (!actuator) {
			put_device(actuator_dev);

			return -ENOMEM;
		}

		actuator->hid_device = actuator_hid_dev;
		hid_set_drvdata(actuator_hid_dev, actuator);

		ret = apple_actuator_init(trackpad_idev, actuator);

		if (ret) {
			hid_set_drvdata(actuator_hid_dev, NULL);
			put_device(actuator_dev);
			kfree(actuator);

			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(apple_actuator_init_haptics);

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

static void apple_actuator_remove(struct hid_device *haptic_hdev)
{
	struct actuator *actuator = hid_get_drvdata(haptic_hdev);

	if (actuator)
		apple_actuator_switch_modes(actuator, HID_HAPTIC_MODE_DEVICE);

	hid_hw_stop(haptic_hdev);
}

static const struct hid_device_id apple_haptic_devices[] = {
	{ HID_DEVICE(BUS_HOST, HID_GROUP_ANY, HOST_VENDOR_ID_APPLE,
		     HID_ANY_ID), .driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(hid, apple_haptic_devices);

static struct hid_driver apple_actuator_driver = {
	.name = "hid-apple-haptic",
	.id_table = apple_haptic_devices,
	.probe = apple_actuator_probe,
	.match = apple_actuator_match,
	.remove = apple_actuator_remove
};
module_hid_driver(apple_actuator_driver);

MODULE_DESCRIPTION("Apple Haptic Actuator driver");
MODULE_LICENSE("GPL");
