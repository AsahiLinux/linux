// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *   Apple "Magic" Wireless Mouse driver
 *
 *   Copyright (c) 2010 Michael Poole <mdpoole@troilus.org>
 *   Copyright (c) 2010 Chase Douglas <chase.douglas@canonical.com>
 */

/*
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "hid-ids.h"

static bool emulate_3button = true;
module_param(emulate_3button, bool, 0644);
MODULE_PARM_DESC(emulate_3button, "Emulate a middle button");

static int middle_button_start = -350;
static int middle_button_stop = +350;

static bool emulate_scroll_wheel = true;
module_param(emulate_scroll_wheel, bool, 0644);
MODULE_PARM_DESC(emulate_scroll_wheel, "Emulate a scroll wheel");

static unsigned int scroll_speed = 32;
static int param_set_scroll_speed(const char *val,
				  const struct kernel_param *kp) {
	unsigned long speed;
	if (!val || kstrtoul(val, 0, &speed) || speed > 63)
		return -EINVAL;
	scroll_speed = speed;
	return 0;
}
module_param_call(scroll_speed, param_set_scroll_speed, param_get_uint, &scroll_speed, 0644);
MODULE_PARM_DESC(scroll_speed, "Scroll speed, value from 0 (slow) to 63 (fast)");

static bool scroll_acceleration = false;
module_param(scroll_acceleration, bool, 0644);
MODULE_PARM_DESC(scroll_acceleration, "Accelerate sequential scroll events");

static bool report_undeciphered;
module_param(report_undeciphered, bool, 0644);
MODULE_PARM_DESC(report_undeciphered, "Report undeciphered multi-touch state field using a MSC_RAW event");

#define TRACKPAD2_2021_BT_VERSION 0x110

#define TRACKPAD_REPORT_ID 0x28
#define TRACKPAD2_USB_REPORT_ID 0x02
#define TRACKPAD2_BT_REPORT_ID 0x31
#define MOUSE_REPORT_ID    0x29
#define MOUSE2_REPORT_ID   0x12
#define DOUBLE_REPORT_ID   0xf7
#define SPI_REPORT_ID      0x02
#define SPI_RESET_REPORT_ID 0x60
#define MTP_REPORT_ID      0x75
#define USB_BATTERY_TIMEOUT_MS 60000

#define MAX_CONTACTS 16

/* These definitions are not precise, but they're close enough.  (Bits
 * 0x03 seem to indicate the aspect ratio of the touch, bits 0x70 seem
 * to be some kind of bit mask -- 0x20 may be a near-field reading,
 * and 0x40 is actual contact, and 0x10 may be a start/stop or change
 * indication.)
 */
#define TOUCH_STATE_MASK  0xf0
#define TOUCH_STATE_NONE  0x00
#define TOUCH_STATE_START 0x30
#define TOUCH_STATE_DRAG  0x40

/* Number of high-resolution events for each low-resolution detent. */
#define SCROLL_HR_STEPS 10
#define SCROLL_HR_MULT (120 / SCROLL_HR_STEPS)
#define SCROLL_HR_THRESHOLD 90 /* units */
#define SCROLL_ACCEL_DEFAULT 7

/* Touch surface information. Dimension is in hundredths of a mm, min and max
 * are in units. */
#define MOUSE_DIMENSION_X (float)9056
#define MOUSE_MIN_X -1100
#define MOUSE_MAX_X 1258
#define MOUSE_RES_X ((MOUSE_MAX_X - MOUSE_MIN_X) / (MOUSE_DIMENSION_X / 100))
#define MOUSE_DIMENSION_Y (float)5152
#define MOUSE_MIN_Y -1589
#define MOUSE_MAX_Y 2047
#define MOUSE_RES_Y ((MOUSE_MAX_Y - MOUSE_MIN_Y) / (MOUSE_DIMENSION_Y / 100))

#define TRACKPAD_DIMENSION_X (float)13000
#define TRACKPAD_MIN_X -2909
#define TRACKPAD_MAX_X 3167
#define TRACKPAD_RES_X \
	((TRACKPAD_MAX_X - TRACKPAD_MIN_X) / (TRACKPAD_DIMENSION_X / 100))
#define TRACKPAD_DIMENSION_Y (float)11000
#define TRACKPAD_MIN_Y -2456
#define TRACKPAD_MAX_Y 2565
#define TRACKPAD_RES_Y \
	((TRACKPAD_MAX_Y - TRACKPAD_MIN_Y) / (TRACKPAD_DIMENSION_Y / 100))

#define TRACKPAD2_DIMENSION_X (float)16000
#define TRACKPAD2_MIN_X -3678
#define TRACKPAD2_MAX_X 3934
#define TRACKPAD2_RES_X \
	((TRACKPAD2_MAX_X - TRACKPAD2_MIN_X) / (TRACKPAD2_DIMENSION_X / 100))
#define TRACKPAD2_DIMENSION_Y (float)11490
#define TRACKPAD2_MIN_Y -2478
#define TRACKPAD2_MAX_Y 2587
#define TRACKPAD2_RES_Y \
	((TRACKPAD2_MAX_Y - TRACKPAD2_MIN_Y) / (TRACKPAD2_DIMENSION_Y / 100))

#define J314_TP_DIMENSION_X (float)13000
#define J314_TP_MIN_X -5900
#define J314_TP_MAX_X 6500
#define J314_TP_RES_X \
	((J314_TP_MAX_X - J314_TP_MIN_X) / (J314_TP_DIMENSION_X / 100))
#define J314_TP_DIMENSION_Y (float)8100
#define J314_TP_MIN_Y -200
#define J314_TP_MAX_Y 7400
#define J314_TP_RES_Y \
	((J314_TP_MAX_Y - J314_TP_MIN_Y) / (J314_TP_DIMENSION_Y / 100))

#define J314_TP_MAX_FINGER_ORIENTATION 16384

struct magicmouse_input_ops {
	int (*raw_event)(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size);
	int (*setup_input)(struct input_dev *input, struct hid_device *hdev);
};

/**
 * struct magicmouse_sc - Tracks Magic Mouse-specific data.
 * @input: Input device through which we report events.
 * @quirks: Currently unused.
 * @ntouches: Number of touches in most recent touch report.
 * @scroll_accel: Number of consecutive scroll motions.
 * @scroll_jiffies: Time of last scroll motion.
 * @touches: Most recent data for a touch, indexed by tracking ID.
 * @tracking_ids: Mapping of current touch input data to @touches.
 */
struct magicmouse_sc {
	struct input_dev *input;
	unsigned long quirks;

	int ntouches;
	int scroll_accel;
	unsigned long scroll_jiffies;

	struct input_mt_pos pos[MAX_CONTACTS];
	struct {
		short scroll_x;
		short scroll_y;
		short scroll_x_hr;
		short scroll_y_hr;
		u8 size;
		bool scroll_x_active;
		bool scroll_y_active;
	} touches[MAX_CONTACTS];
	int tracking_ids[MAX_CONTACTS];

	struct hid_device *hdev;
	struct delayed_work work;
	struct timer_list battery_timer;
	struct magicmouse_input_ops input_ops;
};

static int magicmouse_enable_multitouch(struct hid_device *hdev)
{
	const u8 *feature;
	const u8 feature_mt[] = { 0xD7, 0x01 };
	const u8 feature_mt_mouse2[] = { 0xF1, 0x02, 0x01 };
	const u8 feature_mt_trackpad2_usb[] = { 0x02, 0x01 };
	const u8 feature_mt_trackpad2_bt[] = { 0xF1, 0x02, 0x01 };
	u8 *buf;
	int ret;
	int feature_size;

	if (hdev->product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) {
		if (hdev->vendor == BT_VENDOR_ID_APPLE) {
			feature_size = sizeof(feature_mt_trackpad2_bt);
			feature = feature_mt_trackpad2_bt;
		} else { /* USB_VENDOR_ID_APPLE */
			feature_size = sizeof(feature_mt_trackpad2_usb);
			feature = feature_mt_trackpad2_usb;
		}
	} else if (hdev->vendor == SPI_VENDOR_ID_APPLE) {
		feature_size = sizeof(feature_mt_trackpad2_usb);
		feature = feature_mt_trackpad2_usb;
	} else if (hdev->product == USB_DEVICE_ID_APPLE_MAGICMOUSE2) {
		feature_size = sizeof(feature_mt_mouse2);
		feature = feature_mt_mouse2;
	} else {
		feature_size = sizeof(feature_mt);
		feature = feature_mt;
	}

	buf = kmemdup(feature, feature_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = hid_hw_raw_request(hdev, buf[0], buf, feature_size,
				HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	kfree(buf);
	return ret;
}

static void magicmouse_enable_mt_work(struct work_struct *work)
{
	struct magicmouse_sc *msc =
		container_of(work, struct magicmouse_sc, work.work);
	int ret;

	ret = magicmouse_enable_multitouch(msc->hdev);
	if (ret < 0)
		hid_err(msc->hdev, "unable to request touch data (%d)\n", ret);
}

static int magicmouse_open(struct input_dev *dev)
{
	struct hid_device *hdev = input_get_drvdata(dev);
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);
	int ret;

	ret = hid_hw_open(hdev);
	if (ret)
		return ret;

	/*
	 * Some devices repond with 'invalid report id' when feature
	 * report switching it into multitouch mode is sent to it.
	 *
	 * This results in -EIO from the _raw low-level transport callback,
	 * but there seems to be no other way of switching the mode.
	 * Thus the super-ugly hacky success check below.
	 */
	ret = magicmouse_enable_multitouch(hdev);
	if (ret == -EIO && hdev->product == USB_DEVICE_ID_APPLE_MAGICMOUSE2) {
		schedule_delayed_work(&msc->work, msecs_to_jiffies(500));
		return 0;
	}
	if (ret < 0)
		hid_err(hdev, "unable to request touch data (%d)\n", ret);

	/*
	 * MT enable is usually not required after the first time, so don't
	 * consider it fatal.
	 */
	return 0;
}

static void magicmouse_close(struct input_dev *dev)
{
	struct hid_device *hdev = input_get_drvdata(dev);

	hid_hw_close(hdev);
}

static int magicmouse_firm_touch(struct magicmouse_sc *msc)
{
	int touch = -1;
	int ii;

	/* If there is only one "firm" touch, set touch to its
	 * tracking ID.
	 */
	for (ii = 0; ii < msc->ntouches; ii++) {
		int idx = msc->tracking_ids[ii];
		if (msc->touches[idx].size < 8) {
			/* Ignore this touch. */
		} else if (touch >= 0) {
			touch = -1;
			break;
		} else {
			touch = idx;
		}
	}

	return touch;
}

static void magicmouse_emit_buttons(struct magicmouse_sc *msc, int state)
{
	int last_state = test_bit(BTN_LEFT, msc->input->key) << 0 |
		test_bit(BTN_RIGHT, msc->input->key) << 1 |
		test_bit(BTN_MIDDLE, msc->input->key) << 2;

	if (emulate_3button) {
		int id;

		/* If some button was pressed before, keep it held
		 * down.  Otherwise, if there's exactly one firm
		 * touch, use that to override the mouse's guess.
		 */
		if (state == 0) {
			/* The button was released. */
		} else if (last_state != 0) {
			state = last_state;
		} else if ((id = magicmouse_firm_touch(msc)) >= 0) {
			int x = msc->pos[id].x;
			if (x < middle_button_start)
				state = 1;
			else if (x > middle_button_stop)
				state = 2;
			else
				state = 4;
		} /* else: we keep the mouse's guess */

		input_report_key(msc->input, BTN_MIDDLE, state & 4);
	}

	input_report_key(msc->input, BTN_LEFT, state & 1);
	input_report_key(msc->input, BTN_RIGHT, state & 2);

	if (state != last_state)
		msc->scroll_accel = SCROLL_ACCEL_DEFAULT;
}

static void magicmouse_emit_touch(struct magicmouse_sc *msc, int raw_id, u8 *tdata)
{
	struct input_dev *input = msc->input;
	int id, x, y, size, orientation, touch_major, touch_minor, state, down;
	int pressure = 0;

	if (input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE ||
	    input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE2) {
		id = (tdata[6] << 2 | tdata[5] >> 6) & 0xf;
		x = (tdata[1] << 28 | tdata[0] << 20) >> 20;
		y = -((tdata[2] << 24 | tdata[1] << 16) >> 20);
		size = tdata[5] & 0x3f;
		orientation = (tdata[6] >> 2) - 32;
		touch_major = tdata[3];
		touch_minor = tdata[4];
		state = tdata[7] & TOUCH_STATE_MASK;
		down = state != TOUCH_STATE_NONE;
	} else if (input->id.product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) {
		id = tdata[8] & 0xf;
		x = (tdata[1] << 27 | tdata[0] << 19) >> 19;
		y = -((tdata[3] << 30 | tdata[2] << 22 | tdata[1] << 14) >> 19);
		size = tdata[6];
		orientation = (tdata[8] >> 5) - 4;
		touch_major = tdata[4];
		touch_minor = tdata[5];
		pressure = tdata[7];
		state = tdata[3] & 0xC0;
		down = state == 0x80;
	} else { /* USB_DEVICE_ID_APPLE_MAGICTRACKPAD */
		id = (tdata[7] << 2 | tdata[6] >> 6) & 0xf;
		x = (tdata[1] << 27 | tdata[0] << 19) >> 19;
		y = -((tdata[3] << 30 | tdata[2] << 22 | tdata[1] << 14) >> 19);
		size = tdata[6] & 0x3f;
		orientation = (tdata[7] >> 2) - 32;
		touch_major = tdata[4];
		touch_minor = tdata[5];
		state = tdata[8] & TOUCH_STATE_MASK;
		down = state != TOUCH_STATE_NONE;
	}

	/* Store tracking ID and other fields. */
	msc->tracking_ids[raw_id] = id;
	msc->pos[id].x = x;
	msc->pos[id].y = y;
	msc->touches[id].size = size;

	/* If requested, emulate a scroll wheel by detecting small
	 * vertical touch motions.
	 */
	if (emulate_scroll_wheel && (input->id.product !=
			USB_DEVICE_ID_APPLE_MAGICTRACKPAD2)) {
		unsigned long now = jiffies;
		int step_x = msc->touches[id].scroll_x - x;
		int step_y = msc->touches[id].scroll_y - y;
		int step_hr =
			max_t(int,
			      ((64 - (int)scroll_speed) * msc->scroll_accel) /
					SCROLL_HR_STEPS,
			      1);
		int step_x_hr = msc->touches[id].scroll_x_hr - x;
		int step_y_hr = msc->touches[id].scroll_y_hr - y;

		/* Calculate and apply the scroll motion. */
		switch (state) {
		case TOUCH_STATE_START:
			msc->touches[id].scroll_x = x;
			msc->touches[id].scroll_y = y;
			msc->touches[id].scroll_x_hr = x;
			msc->touches[id].scroll_y_hr = y;
			msc->touches[id].scroll_x_active = false;
			msc->touches[id].scroll_y_active = false;

			/* Reset acceleration after half a second. */
			if (scroll_acceleration && time_before(now,
						msc->scroll_jiffies + HZ / 2))
				msc->scroll_accel = max_t(int,
						msc->scroll_accel - 1, 1);
			else
				msc->scroll_accel = SCROLL_ACCEL_DEFAULT;

			break;
		case TOUCH_STATE_DRAG:
			step_x /= (64 - (int)scroll_speed) * msc->scroll_accel;
			if (step_x != 0) {
				msc->touches[id].scroll_x -= step_x *
					(64 - scroll_speed) * msc->scroll_accel;
				msc->scroll_jiffies = now;
				input_report_rel(input, REL_HWHEEL, -step_x);
			}

			step_y /= (64 - (int)scroll_speed) * msc->scroll_accel;
			if (step_y != 0) {
				msc->touches[id].scroll_y -= step_y *
					(64 - scroll_speed) * msc->scroll_accel;
				msc->scroll_jiffies = now;
				input_report_rel(input, REL_WHEEL, step_y);
			}

			if (!msc->touches[id].scroll_x_active &&
			    abs(step_x_hr) > SCROLL_HR_THRESHOLD) {
				msc->touches[id].scroll_x_active = true;
				msc->touches[id].scroll_x_hr = x;
				step_x_hr = 0;
			}

			step_x_hr /= step_hr;
			if (step_x_hr != 0 &&
			    msc->touches[id].scroll_x_active) {
				msc->touches[id].scroll_x_hr -= step_x_hr *
					step_hr;
				input_report_rel(input,
						 REL_HWHEEL_HI_RES,
						 -step_x_hr * SCROLL_HR_MULT);
			}

			if (!msc->touches[id].scroll_y_active &&
			    abs(step_y_hr) > SCROLL_HR_THRESHOLD) {
				msc->touches[id].scroll_y_active = true;
				msc->touches[id].scroll_y_hr = y;
				step_y_hr = 0;
			}

			step_y_hr /= step_hr;
			if (step_y_hr != 0 &&
			    msc->touches[id].scroll_y_active) {
				msc->touches[id].scroll_y_hr -= step_y_hr *
					step_hr;
				input_report_rel(input,
						 REL_WHEEL_HI_RES,
						 step_y_hr * SCROLL_HR_MULT);
			}
			break;
		}
	}

	if (down)
		msc->ntouches++;

	input_mt_slot(input, id);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, down);

	/* Generate the input events for this touch. */
	if (down) {
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, touch_major << 2);
		input_report_abs(input, ABS_MT_TOUCH_MINOR, touch_minor << 2);
		input_report_abs(input, ABS_MT_ORIENTATION, -orientation);
		input_report_abs(input, ABS_MT_POSITION_X, x);
		input_report_abs(input, ABS_MT_POSITION_Y, y);

		if (input->id.product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2)
			input_report_abs(input, ABS_MT_PRESSURE, pressure);

		if (report_undeciphered) {
			if (input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE ||
			    input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE2)
				input_event(input, EV_MSC, MSC_RAW, tdata[7]);
			else if (input->id.product !=
					USB_DEVICE_ID_APPLE_MAGICTRACKPAD2)
				input_event(input, EV_MSC, MSC_RAW, tdata[8]);
		}
	}
}

static int magicmouse_raw_event(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);

	return msc->input_ops.raw_event(hdev, report, data, size);
}

static int magicmouse_raw_event_usb(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);
	struct input_dev *input = msc->input;
	int x = 0, y = 0, ii, clicks = 0, npoints;

	switch (data[0]) {
	case TRACKPAD_REPORT_ID:
	case TRACKPAD2_BT_REPORT_ID:
		/* Expect four bytes of prefix, and N*9 bytes of touch data. */
		if (size < 4 || ((size - 4) % 9) != 0)
			return 0;
		npoints = (size - 4) / 9;
		if (npoints > 15) {
			hid_warn(hdev, "invalid size value (%d) for TRACKPAD_REPORT_ID\n",
					size);
			return 0;
		}
		msc->ntouches = 0;
		for (ii = 0; ii < npoints; ii++)
			magicmouse_emit_touch(msc, ii, data + ii * 9 + 4);

		clicks = data[1];

		/* The following bits provide a device specific timestamp. They
		 * are unused here.
		 *
		 * ts = data[1] >> 6 | data[2] << 2 | data[3] << 10;
		 */
		break;
	case TRACKPAD2_USB_REPORT_ID:
		/* Expect twelve bytes of prefix and N*9 bytes of touch data. */
		if (size < 12 || ((size - 12) % 9) != 0)
			return 0;
		npoints = (size - 12) / 9;
		if (npoints > 15) {
			hid_warn(hdev, "invalid size value (%d) for TRACKPAD2_USB_REPORT_ID\n",
					size);
			return 0;
		}
		msc->ntouches = 0;
		for (ii = 0; ii < npoints; ii++)
			magicmouse_emit_touch(msc, ii, data + ii * 9 + 12);

		clicks = data[1];
		break;
	case MOUSE_REPORT_ID:
		/* Expect six bytes of prefix, and N*8 bytes of touch data. */
		if (size < 6 || ((size - 6) % 8) != 0)
			return 0;
		npoints = (size - 6) / 8;
		if (npoints > 15) {
			hid_warn(hdev, "invalid size value (%d) for MOUSE_REPORT_ID\n",
					size);
			return 0;
		}
		msc->ntouches = 0;
		for (ii = 0; ii < npoints; ii++)
			magicmouse_emit_touch(msc, ii, data + ii * 8 + 6);

		/* When emulating three-button mode, it is important
		 * to have the current touch information before
		 * generating a click event.
		 */
		x = (int)(((data[3] & 0x0c) << 28) | (data[1] << 22)) >> 22;
		y = (int)(((data[3] & 0x30) << 26) | (data[2] << 22)) >> 22;
		clicks = data[3];

		/* The following bits provide a device specific timestamp. They
		 * are unused here.
		 *
		 * ts = data[3] >> 6 | data[4] << 2 | data[5] << 10;
		 */
		break;
	case MOUSE2_REPORT_ID:
		/* Size is either 8 or (14 + 8 * N) */
		if (size != 8 && (size < 14 || (size - 14) % 8 != 0))
			return 0;
		npoints = (size - 14) / 8;
		if (npoints > 15) {
			hid_warn(hdev, "invalid size value (%d) for MOUSE2_REPORT_ID\n",
					size);
			return 0;
		}
		msc->ntouches = 0;
		for (ii = 0; ii < npoints; ii++)
			magicmouse_emit_touch(msc, ii, data + ii * 8 + 14);

		/* When emulating three-button mode, it is important
		 * to have the current touch information before
		 * generating a click event.
		 */
		x = (int)((data[3] << 24) | (data[2] << 16)) >> 16;
		y = (int)((data[5] << 24) | (data[4] << 16)) >> 16;
		clicks = data[1];

		/* The following bits provide a device specific timestamp. They
		 * are unused here.
		 *
		 * ts = data[11] >> 6 | data[12] << 2 | data[13] << 10;
		 */
		break;
	case DOUBLE_REPORT_ID:
		/* Sometimes the trackpad sends two touch reports in one
		 * packet.
		 */
		magicmouse_raw_event(hdev, report, data + 2, data[1]);
		magicmouse_raw_event(hdev, report, data + 2 + data[1],
			size - 2 - data[1]);
		return 0;
	default:
		return 0;
	}

	if (input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE ||
	    input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE2) {
		magicmouse_emit_buttons(msc, clicks & 3);
		input_report_rel(input, REL_X, x);
		input_report_rel(input, REL_Y, y);
	} else if (input->id.product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) {
		input_mt_sync_frame(input);
		input_report_key(input, BTN_MOUSE, clicks & 1);
	} else { /* USB_DEVICE_ID_APPLE_MAGICTRACKPAD */
		input_report_key(input, BTN_MOUSE, clicks & 1);
		input_mt_report_pointer_emulation(input, true);
	}

	input_sync(input);
	return 1;
}

/**
 * struct tp_finger - single trackpad finger structure, le16-aligned
 *
 * @unknown1:		unknown
 * @unknown2:		unknown
 * @abs_x:		absolute x coordinate
 * @abs_y:		absolute y coordinate
 * @rel_x:		relative x coordinate
 * @rel_y:		relative y coordinate
 * @tool_major:		tool area, major axis
 * @tool_minor:		tool area, minor axis
 * @orientation:	16384 when point, else 15 bit angle
 * @touch_major:	touch area, major axis
 * @touch_minor:	touch area, minor axis
 * @unused:		zeros
 * @pressure:		pressure on forcetouch touchpad
 * @multi:		one finger: varies, more fingers: constant
 * @crc16:		on last finger: crc over the whole message struct
 *			(i.e. message header + this struct) minus the last
 *			@crc16 field; unknown on all other fingers.
 */
struct tp_finger {
	__le16 unknown1;
	__le16 unknown2;
	__le16 abs_x;
	__le16 abs_y;
	__le16 rel_x;
	__le16 rel_y;
	__le16 tool_major;
	__le16 tool_minor;
	__le16 orientation;
	__le16 touch_major;
	__le16 touch_minor;
	__le16 unused[2];
	__le16 pressure;
	__le16 multi;
} __attribute__((packed, aligned(2)));

/**
 * vendor trackpad report
 *
 * @num_fingers:	the number of fingers being reported in @fingers
 * @buttons:		same as HID buttons
 */
struct tp_header {
	// HID vendor part, up to 1751 bytes
	u8 unknown[22];
	u8 num_fingers;
	u8 buttons;
	u8 unknown3[14];
};

/**
 * standard HID mouse report
 *
 * @report_id:		reportid
 * @buttons:		HID Usage Buttons 3 1-bit reports
 */
struct tp_mouse_report {
	// HID mouse report
	u8 report_id;
	u8 buttons;
	u8 rel_x;
	u8 rel_y;
	u8 padding[4];
};

static inline int le16_to_int(__le16 x)
{
	return (signed short)le16_to_cpu(x);
}

static void report_finger_data(struct input_dev *input, int slot,
			       const struct input_mt_pos *pos,
			       const struct tp_finger *f)
{
	input_mt_slot(input, slot);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, true);

	input_report_abs(input, ABS_MT_TOUCH_MAJOR,
			 le16_to_int(f->touch_major) << 1);
	input_report_abs(input, ABS_MT_TOUCH_MINOR,
			 le16_to_int(f->touch_minor) << 1);
	input_report_abs(input, ABS_MT_WIDTH_MAJOR,
			 le16_to_int(f->tool_major) << 1);
	input_report_abs(input, ABS_MT_WIDTH_MINOR,
			 le16_to_int(f->tool_minor) << 1);
	input_report_abs(input, ABS_MT_ORIENTATION,
			 J314_TP_MAX_FINGER_ORIENTATION - le16_to_int(f->orientation));
	input_report_abs(input, ABS_MT_PRESSURE, le16_to_int(f->pressure));
	input_report_abs(input, ABS_MT_POSITION_X, pos->x);
	input_report_abs(input, ABS_MT_POSITION_Y, pos->y);
}

static int magicmouse_raw_event_mtp(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);
	struct input_dev *input = msc->input;
	struct tp_header *tp_hdr;
	struct tp_finger *f;
	int i, n;
	u32 npoints;
	const size_t hdr_sz = sizeof(struct tp_header);
	const size_t touch_sz = sizeof(struct tp_finger);
	u8 map_contacs[MAX_CONTACTS];

	// hid_warn(hdev, "%s\n", __func__);
	// print_hex_dump_debug("appleft ev: ", DUMP_PREFIX_OFFSET, 16, 1, data,
	// 		     size, false);

	/* Expect 46 bytes of prefix, and N * 30 bytes of touch data. */
	if (size < hdr_sz || ((size - hdr_sz) % touch_sz) != 0)
		return 0;

	tp_hdr = (struct tp_header *)data;

	npoints = (size - hdr_sz) / touch_sz;
	if (npoints < tp_hdr->num_fingers || npoints > MAX_CONTACTS) {
		hid_warn(hdev,
			 "unexpected number of touches (%u) for "
			 "report\n",
			 npoints);
		return 0;
	}

	n = 0;
	for (i = 0; i < tp_hdr->num_fingers; i++) {
		f = (struct tp_finger *)(data + hdr_sz + i * touch_sz);
		if (le16_to_int(f->touch_major) == 0)
			continue;

		hid_dbg(hdev, "ev x:%04x y:%04x\n", le16_to_int(f->abs_x),
			le16_to_int(f->abs_y));
		msc->pos[n].x = le16_to_int(f->abs_x);
		msc->pos[n].y = -le16_to_int(f->abs_y);
		map_contacs[n] = i;
		n++;
	}

	input_mt_assign_slots(input, msc->tracking_ids, msc->pos, n, 0);

	for (i = 0; i < n; i++) {
		int idx = map_contacs[i];
		f = (struct tp_finger *)(data + hdr_sz + idx * touch_sz);
		report_finger_data(input, msc->tracking_ids[i], &msc->pos[i], f);
	}

	input_mt_sync_frame(input);
	input_report_key(input, BTN_MOUSE, tp_hdr->buttons & 1);

	input_sync(input);
	return 1;
}

static int magicmouse_raw_event_spi(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);
	const size_t hdr_sz = sizeof(struct tp_mouse_report);

	if (!size)
		return 0;

	if (data[0] == SPI_RESET_REPORT_ID) {
		hid_info(hdev, "Touch controller was reset, re-enabling touch mode\n");
		schedule_delayed_work(&msc->work, msecs_to_jiffies(10));
		return 1;
	}

	if (data[0] != TRACKPAD2_USB_REPORT_ID || size < hdr_sz)
		return 0;

	return magicmouse_raw_event_mtp(hdev, report, data + hdr_sz, size - hdr_sz);
}

static int magicmouse_event(struct hid_device *hdev, struct hid_field *field,
		struct hid_usage *usage, __s32 value)
{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);
	if (msc->input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE2 &&
	    field->report->id == MOUSE2_REPORT_ID) {
		/*
		 * magic_mouse_raw_event has done all the work. Skip hidinput.
		 *
		 * Specifically, hidinput may modify BTN_LEFT and BTN_RIGHT,
		 * breaking emulate_3button.
		 */
		return 1;
	}
	return 0;
}


static int magicmouse_setup_input(struct input_dev *input,
				  struct hid_device *hdev)
{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);

	return msc->input_ops.setup_input(input, hdev);
}

static int magicmouse_setup_input_usb(struct input_dev *input,
				      struct hid_device *hdev)
{
	int error;
	int mt_flags = 0;

	__set_bit(EV_KEY, input->evbit);

	if (input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE ||
	    input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE2) {
		__set_bit(BTN_LEFT, input->keybit);
		__set_bit(BTN_RIGHT, input->keybit);
		if (emulate_3button)
			__set_bit(BTN_MIDDLE, input->keybit);

		__set_bit(EV_REL, input->evbit);
		__set_bit(REL_X, input->relbit);
		__set_bit(REL_Y, input->relbit);
		if (emulate_scroll_wheel) {
			__set_bit(REL_WHEEL, input->relbit);
			__set_bit(REL_HWHEEL, input->relbit);
			__set_bit(REL_WHEEL_HI_RES, input->relbit);
			__set_bit(REL_HWHEEL_HI_RES, input->relbit);
		}
	} else if (input->id.product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) {
		/* If the trackpad has been connected to a Mac, the name is
		 * automatically personalized, e.g., "José Expósito's Trackpad".
		 * When connected through Bluetooth, the personalized name is
		 * reported, however, when connected through USB the generic
		 * name is reported.
		 * Set the device name to ensure the same driver settings get
		 * loaded, whether connected through bluetooth or USB.
		 */
		if (hdev->vendor == BT_VENDOR_ID_APPLE) {
			if (input->id.version == TRACKPAD2_2021_BT_VERSION)
				input->name = "Apple Inc. Magic Trackpad";
			else
				input->name = "Apple Inc. Magic Trackpad 2";
		} else { /* USB_VENDOR_ID_APPLE */
			input->name = hdev->name;
		}

		__clear_bit(EV_MSC, input->evbit);
		__clear_bit(BTN_0, input->keybit);
		__clear_bit(BTN_RIGHT, input->keybit);
		__clear_bit(BTN_MIDDLE, input->keybit);
		__set_bit(BTN_MOUSE, input->keybit);
		__set_bit(INPUT_PROP_BUTTONPAD, input->propbit);
		__set_bit(BTN_TOOL_FINGER, input->keybit);

		mt_flags = INPUT_MT_POINTER | INPUT_MT_DROP_UNUSED |
				INPUT_MT_TRACK;
	} else { /* USB_DEVICE_ID_APPLE_MAGICTRACKPAD */
		/* input->keybit is initialized with incorrect button info
		 * for Magic Trackpad. There really is only one physical
		 * button (BTN_LEFT == BTN_MOUSE). Make sure we don't
		 * advertise buttons that don't exist...
		 */
		__clear_bit(BTN_RIGHT, input->keybit);
		__clear_bit(BTN_MIDDLE, input->keybit);
		__set_bit(BTN_MOUSE, input->keybit);
		__set_bit(BTN_TOOL_FINGER, input->keybit);
		__set_bit(BTN_TOOL_DOUBLETAP, input->keybit);
		__set_bit(BTN_TOOL_TRIPLETAP, input->keybit);
		__set_bit(BTN_TOOL_QUADTAP, input->keybit);
		__set_bit(BTN_TOOL_QUINTTAP, input->keybit);
		__set_bit(BTN_TOUCH, input->keybit);
		__set_bit(INPUT_PROP_POINTER, input->propbit);
		__set_bit(INPUT_PROP_BUTTONPAD, input->propbit);
	}


	__set_bit(EV_ABS, input->evbit);

	error = input_mt_init_slots(input, MAX_CONTACTS, mt_flags);
	if (error)
		return error;
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255 << 2,
			     4, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255 << 2,
			     4, 0);

	/* Note: Touch Y position from the device is inverted relative
	 * to how pointer motion is reported (and relative to how USB
	 * HID recommends the coordinates work).  This driver keeps
	 * the origin at the same position, and just uses the additive
	 * inverse of the reported Y.
	 */
	if (input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE ||
	    input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE2) {
		input_set_abs_params(input, ABS_MT_ORIENTATION, -31, 32, 1, 0);
		input_set_abs_params(input, ABS_MT_POSITION_X,
				     MOUSE_MIN_X, MOUSE_MAX_X, 4, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y,
				     MOUSE_MIN_Y, MOUSE_MAX_Y, 4, 0);

		input_abs_set_res(input, ABS_MT_POSITION_X,
				  MOUSE_RES_X);
		input_abs_set_res(input, ABS_MT_POSITION_Y,
				  MOUSE_RES_Y);
	} else if (input->id.product ==  USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) {
		input_set_abs_params(input, ABS_MT_PRESSURE, 0, 253, 0, 0);
		input_set_abs_params(input, ABS_PRESSURE, 0, 253, 0, 0);
		input_set_abs_params(input, ABS_MT_ORIENTATION, -3, 4, 0, 0);
		input_set_abs_params(input, ABS_X, TRACKPAD2_MIN_X,
				     TRACKPAD2_MAX_X, 0, 0);
		input_set_abs_params(input, ABS_Y, TRACKPAD2_MIN_Y,
				     TRACKPAD2_MAX_Y, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_X,
				     TRACKPAD2_MIN_X, TRACKPAD2_MAX_X, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y,
				     TRACKPAD2_MIN_Y, TRACKPAD2_MAX_Y, 0, 0);

		input_abs_set_res(input, ABS_X, TRACKPAD2_RES_X);
		input_abs_set_res(input, ABS_Y, TRACKPAD2_RES_Y);
		input_abs_set_res(input, ABS_MT_POSITION_X, TRACKPAD2_RES_X);
		input_abs_set_res(input, ABS_MT_POSITION_Y, TRACKPAD2_RES_Y);
	} else { /* USB_DEVICE_ID_APPLE_MAGICTRACKPAD */
		input_set_abs_params(input, ABS_MT_ORIENTATION, -31, 32, 1, 0);
		input_set_abs_params(input, ABS_X, TRACKPAD_MIN_X,
				     TRACKPAD_MAX_X, 4, 0);
		input_set_abs_params(input, ABS_Y, TRACKPAD_MIN_Y,
				     TRACKPAD_MAX_Y, 4, 0);
		input_set_abs_params(input, ABS_MT_POSITION_X,
				     TRACKPAD_MIN_X, TRACKPAD_MAX_X, 4, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y,
				     TRACKPAD_MIN_Y, TRACKPAD_MAX_Y, 4, 0);

		input_abs_set_res(input, ABS_X, TRACKPAD_RES_X);
		input_abs_set_res(input, ABS_Y, TRACKPAD_RES_Y);
		input_abs_set_res(input, ABS_MT_POSITION_X,
				  TRACKPAD_RES_X);
		input_abs_set_res(input, ABS_MT_POSITION_Y,
				  TRACKPAD_RES_Y);
	}

	input_set_events_per_packet(input, 60);

	if (report_undeciphered &&
	    input->id.product != USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) {
		__set_bit(EV_MSC, input->evbit);
		__set_bit(MSC_RAW, input->mscbit);
	}

	/*
	 * hid-input may mark device as using autorepeat, but neither
	 * the trackpad, nor the mouse actually want it.
	 */
	__clear_bit(EV_REP, input->evbit);

	/*
	 * This isn't strictly speaking needed for USB, but enabling MT on
	 * device open is probably more robust than only doing it once on probe
	 * even if USB devices are not known to suffer from the SPI reset issue.
	 */
	input->open = magicmouse_open;
	input->close = magicmouse_close;
	return 0;
}

static int magicmouse_setup_input_mtp(struct input_dev *input,
				      struct hid_device *hdev)
{
	int error;
	int mt_flags = 0;

	__set_bit(INPUT_PROP_BUTTONPAD, input->propbit);
	__clear_bit(BTN_0, input->keybit);
	__clear_bit(BTN_RIGHT, input->keybit);
	__clear_bit(BTN_MIDDLE, input->keybit);
	__clear_bit(EV_REL, input->evbit);
	__clear_bit(REL_X, input->relbit);
	__clear_bit(REL_Y, input->relbit);

	mt_flags = INPUT_MT_POINTER | INPUT_MT_DROP_UNUSED | INPUT_MT_TRACK;

	/* finger touch area */
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 5000, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 5000, 0, 0);

	/* finger approach area */
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 5000, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MINOR, 0, 5000, 0, 0);

	/* Note: Touch Y position from the device is inverted relative
	 * to how pointer motion is reported (and relative to how USB
	 * HID recommends the coordinates work).  This driver keeps
	 * the origin at the same position, and just uses the additive
	 * inverse of the reported Y.
	 */

	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 6000, 0, 0);

	/*
	 * This makes libinput recognize this as a PressurePad and
	 * stop trying to use pressure for touch size. Pressure unit
	 * seems to be ~grams on these touchpads.
	 */
	input_abs_set_res(input, ABS_MT_PRESSURE, 1);

	/* finger orientation */
	input_set_abs_params(input, ABS_MT_ORIENTATION, -J314_TP_MAX_FINGER_ORIENTATION,
			     J314_TP_MAX_FINGER_ORIENTATION, 0, 0);

	/* finger position */
	input_set_abs_params(input, ABS_MT_POSITION_X, J314_TP_MIN_X, J314_TP_MAX_X,
			     0, 0);
	/* Y axis is inverted */
	input_set_abs_params(input, ABS_MT_POSITION_Y, -J314_TP_MAX_Y, -J314_TP_MIN_Y,
			     0, 0);

	/* X/Y resolution */
	input_abs_set_res(input, ABS_MT_POSITION_X, J314_TP_RES_X);
	input_abs_set_res(input, ABS_MT_POSITION_Y, J314_TP_RES_Y);

	input_set_events_per_packet(input, 60);

	/* touchpad button */
	input_set_capability(input, EV_KEY, BTN_MOUSE);

	/*
	 * hid-input may mark device as using autorepeat, but the trackpad does
	 * not actually want it.
	 */
	__clear_bit(EV_REP, input->evbit);

	error = input_mt_init_slots(input, MAX_CONTACTS, mt_flags);
	if (error)
		return error;

	return 0;
}

static int magicmouse_setup_input_spi(struct input_dev *input,
				      struct hid_device *hdev)
{
	int ret = magicmouse_setup_input_mtp(input, hdev);
	if (ret)
		return ret;

	/*
	 * Override the default input->open function to send the MT
	 * enable every time the device is opened. This ensures it works
	 * even if we missed a reset event due to the device being closed.
	 * input->close is overridden for symmetry.
	 */
	input->open = magicmouse_open;
	input->close = magicmouse_close;

	return 0;
}

static int magicmouse_input_mapping(struct hid_device *hdev,
		struct hid_input *hi, struct hid_field *field,
		struct hid_usage *usage, unsigned long **bit, int *max)
{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);

	if (!msc->input)
		msc->input = hi->input;

	/* Magic Trackpad does not give relative data after switching to MT */
	if ((hi->input->id.product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD ||
	     hi->input->id.product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) &&
	    field->flags & HID_MAIN_ITEM_RELATIVE)
		return -1;

	return 0;
}

static int magicmouse_input_configured(struct hid_device *hdev,
		struct hid_input *hi)

{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);
	int ret;

	ret = magicmouse_setup_input(msc->input, hdev);
	if (ret) {
		hid_err(hdev, "magicmouse setup input failed (%d)\n", ret);
		/* clean msc->input to notify probe() of the failure */
		msc->input = NULL;
		return ret;
	}

	return 0;
}

static int magicmouse_fetch_battery(struct hid_device *hdev)
{
#ifdef CONFIG_HID_BATTERY_STRENGTH
	struct hid_report_enum *report_enum;
	struct hid_report *report;

	if (!hdev->battery || hdev->vendor != USB_VENDOR_ID_APPLE ||
	    (hdev->product != USB_DEVICE_ID_APPLE_MAGICMOUSE2 &&
	     hdev->product != USB_DEVICE_ID_APPLE_MAGICTRACKPAD2))
		return -1;

	report_enum = &hdev->report_enum[hdev->battery_report_type];
	report = report_enum->report_id_hash[hdev->battery_report_id];

	if (!report || report->maxfield < 1)
		return -1;

	if (hdev->battery_capacity == hdev->battery_max)
		return -1;

	hid_hw_request(hdev, report, HID_REQ_GET_REPORT);
	return 0;
#else
	return -1;
#endif
}

static void magicmouse_battery_timer_tick(struct timer_list *t)
{
	struct magicmouse_sc *msc = from_timer(msc, t, battery_timer);
	struct hid_device *hdev = msc->hdev;

	if (magicmouse_fetch_battery(hdev) == 0) {
		mod_timer(&msc->battery_timer,
			  jiffies + msecs_to_jiffies(USB_BATTERY_TIMEOUT_MS));
	}
}

static int magicmouse_probe(struct hid_device *hdev,
	const struct hid_device_id *id)
{
	struct magicmouse_sc *msc;
	struct hid_report *report;
	int ret;

	if ((id->bus == BUS_SPI || id->bus == BUS_HOST) && id->vendor == SPI_VENDOR_ID_APPLE &&
	    hdev->type != HID_TYPE_SPI_MOUSE)
		return -ENODEV;

	msc = devm_kzalloc(&hdev->dev, sizeof(*msc), GFP_KERNEL);
	if (msc == NULL) {
		hid_err(hdev, "can't alloc magicmouse descriptor\n");
		return -ENOMEM;
	}

	// internal trackpad use a data format use input ops to avoid
	// conflicts with the report ID.
	if (id->bus == BUS_HOST) {
		msc->input_ops.raw_event = magicmouse_raw_event_mtp;
		msc->input_ops.setup_input = magicmouse_setup_input_mtp;
	} else if (id->bus == BUS_SPI) {
		msc->input_ops.raw_event = magicmouse_raw_event_spi;
		msc->input_ops.setup_input = magicmouse_setup_input_spi;

	} else {
		msc->input_ops.raw_event = magicmouse_raw_event_usb;
		msc->input_ops.setup_input = magicmouse_setup_input_usb;
	}

	msc->scroll_accel = SCROLL_ACCEL_DEFAULT;
	msc->hdev = hdev;
	INIT_DEFERRABLE_WORK(&msc->work, magicmouse_enable_mt_work);

	msc->quirks = id->driver_data;
	hid_set_drvdata(hdev, msc);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "magicmouse hid parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "magicmouse hw start failed\n");
		return ret;
	}

	timer_setup(&msc->battery_timer, magicmouse_battery_timer_tick, 0);
	mod_timer(&msc->battery_timer,
		  jiffies + msecs_to_jiffies(USB_BATTERY_TIMEOUT_MS));
	magicmouse_fetch_battery(hdev);

	if (id->vendor == USB_VENDOR_ID_APPLE &&
	    (id->product == USB_DEVICE_ID_APPLE_MAGICMOUSE2 ||
	     (id->product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2 && hdev->type != HID_TYPE_USBMOUSE)))
		return 0;

	if (!msc->input) {
		hid_err(hdev, "magicmouse input not registered\n");
		ret = -ENOMEM;
		goto err_stop_hw;
	}

	if (id->product == USB_DEVICE_ID_APPLE_MAGICMOUSE)
		report = hid_register_report(hdev, HID_INPUT_REPORT,
			MOUSE_REPORT_ID, 0);
	else if (id->product == USB_DEVICE_ID_APPLE_MAGICMOUSE2)
		report = hid_register_report(hdev, HID_INPUT_REPORT,
			MOUSE2_REPORT_ID, 0);
	else if (id->product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) {
		if (id->vendor == BT_VENDOR_ID_APPLE)
			report = hid_register_report(hdev, HID_INPUT_REPORT,
				TRACKPAD2_BT_REPORT_ID, 0);
		else /* USB_VENDOR_ID_APPLE */
			report = hid_register_report(hdev, HID_INPUT_REPORT,
				TRACKPAD2_USB_REPORT_ID, 0);
	} else if (id->bus == BUS_SPI) {
		report = hid_register_report(hdev, HID_INPUT_REPORT, SPI_REPORT_ID, 0);
	} else if (id->bus == BUS_HOST) {
		report = hid_register_report(hdev, HID_INPUT_REPORT, MTP_REPORT_ID, 0);
	} else { /* USB_DEVICE_ID_APPLE_MAGICTRACKPAD */
		report = hid_register_report(hdev, HID_INPUT_REPORT,
			TRACKPAD_REPORT_ID, 0);
		report = hid_register_report(hdev, HID_INPUT_REPORT,
			DOUBLE_REPORT_ID, 0);
	}

	if (!report) {
		hid_err(hdev, "unable to register touch report\n");
		ret = -ENOMEM;
		goto err_stop_hw;
	}
	report->size = 6;

	/* MTP devices do not need the MT enable, this is handled by the MTP driver */
	if (id->bus == BUS_HOST)
		return 0;

	/* SPI devices need to watch for reset events to re-send the MT enable */
	if (id->bus == BUS_SPI) {
		report = hid_register_report(hdev, HID_INPUT_REPORT, SPI_RESET_REPORT_ID, 0);
		report->size = 2;
	}

	return 0;
err_stop_hw:
	del_timer_sync(&msc->battery_timer);
	hid_hw_stop(hdev);
	return ret;
}

static void magicmouse_remove(struct hid_device *hdev)
{
	struct magicmouse_sc *msc = hid_get_drvdata(hdev);

	if (msc) {
		cancel_delayed_work_sync(&msc->work);
		del_timer_sync(&msc->battery_timer);
	}

	hid_hw_stop(hdev);
}

static __u8 *magicmouse_report_fixup(struct hid_device *hdev, __u8 *rdesc,
				     unsigned int *rsize)
{
	/*
	 * Change the usage from:
	 *   0x06, 0x00, 0xff, // Usage Page (Vendor Defined Page 1)  0
	 *   0x09, 0x0b,       // Usage (Vendor Usage 0x0b)           3
	 * To:
	 *   0x05, 0x01,       // Usage Page (Generic Desktop)        0
	 *   0x09, 0x02,       // Usage (Mouse)                       2
	 */
	if (hdev->vendor == USB_VENDOR_ID_APPLE &&
	    (hdev->product == USB_DEVICE_ID_APPLE_MAGICMOUSE2 ||
	     hdev->product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) &&
	    *rsize == 83 && rdesc[46] == 0x84 && rdesc[58] == 0x85) {
		hid_info(hdev,
			 "fixing up magicmouse battery report descriptor\n");
		*rsize = *rsize - 1;
		rdesc = kmemdup(rdesc + 1, *rsize, GFP_KERNEL);
		if (!rdesc)
			return NULL;

		rdesc[0] = 0x05;
		rdesc[1] = 0x01;
		rdesc[2] = 0x09;
		rdesc[3] = 0x02;
	}

	return rdesc;
}

static const struct hid_device_id magic_mice[] = {
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICMOUSE), .driver_data = 0 },
	{ HID_BLUETOOTH_DEVICE(BT_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICMOUSE2), .driver_data = 0 },
	{ HID_USB_DEVICE(USB_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICMOUSE2), .driver_data = 0 },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICTRACKPAD), .driver_data = 0 },
	{ HID_BLUETOOTH_DEVICE(BT_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICTRACKPAD2), .driver_data = 0 },
	{ HID_USB_DEVICE(USB_VENDOR_ID_APPLE,
		USB_DEVICE_ID_APPLE_MAGICTRACKPAD2), .driver_data = 0 },
	{ HID_SPI_DEVICE(SPI_VENDOR_ID_APPLE, HID_ANY_ID),
	  .driver_data = 0 },
	{ HID_DEVICE(BUS_HOST, HID_GROUP_ANY, HOST_VENDOR_ID_APPLE,
                     HID_ANY_ID), .driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(hid, magic_mice);

#ifdef CONFIG_PM
static int magicmouse_reset_resume(struct hid_device *hdev)
{
	if (hdev->bus == BUS_SPI)
		return magicmouse_enable_multitouch(hdev);

	return 0;
}
#endif

static struct hid_driver magicmouse_driver = {
	.name = "magicmouse",
	.id_table = magic_mice,
	.probe = magicmouse_probe,
	.remove = magicmouse_remove,
	.report_fixup = magicmouse_report_fixup,
	.raw_event = magicmouse_raw_event,
	.event = magicmouse_event,
	.input_mapping = magicmouse_input_mapping,
	.input_configured = magicmouse_input_configured,
#ifdef CONFIG_PM
        .reset_resume = magicmouse_reset_resume,
#endif

};
module_hid_driver(magicmouse_driver);

MODULE_LICENSE("GPL");
