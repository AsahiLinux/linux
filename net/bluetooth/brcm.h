/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 The Asahi Linux Contributors
 */

#if IS_ENABLED(CONFIG_BT_BRCMEXT)

int brcm_set_high_priority(struct hci_dev *hdev, u16 handle, bool enable);

#else

static inline int brcm_set_high_priority(struct hci_dev *hdev, u16 handle, bool enable)
{
	return 0;
}

#endif
