// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef __APPLE_MTP_ACTUATOR_H__
#define __APPLE_MTP_ACTUATOR_H__
#include <linux/hid.h>

#define APPLE_HP_WAVEFORMDEEPCLICK	0x000e2001

#if IS_ENABLED(CONFIG_HID_APPLE_MTP_HAPTIC)
int apple_taptic_send(struct hid_device *hdev, u16 effect_type, u8 strength, u8 softness);
int apple_taptic_switch_modes(struct hid_device *hdev, bool currently_host_controlled);

#else
static inline int apple_taptic_send(struct hid_device *hdev, u16 effect_type,
				    u8 strength, u8 softness) { return -ENODEV; }
static inline int apple_taptic_switch_modes(struct hid_device *hdev,
					    bool enable_host_controlled) { return -ENODEV; }
#endif
#endif /* __APPLE_MTP_ACTUATOR_H__ */
