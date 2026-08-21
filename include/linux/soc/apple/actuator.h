// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef __APPLE_ACTUATOR_H__
#define __APPLE_ACTUATOR_H__
#include <linux/hid.h>

#if IS_REACHABLE(CONFIG_HID_APPLE_HAPTIC)
int apple_actuator_init_haptics(struct input_dev *trackpad_idev, struct hid_device *trackpad_hdev);

#else
static inline int apple_actuator_init_haptics(struct input_dev *trackpad_idev,
					      struct hid_device *trackpad_hdev){ return 0; }
#endif
#endif /* __APPLE_ACTUATOR_H__ */
