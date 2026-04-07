// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 The Asahi Linux Contributors
 */

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "brcm.h"

int brcm_set_high_priority(struct hci_dev *hdev, u16 handle, bool enable)
{
	struct sk_buff *skb;
	u8 cmd[3];

	if (!hdev->brcm_capable)
		return 0;

	cmd[0] = handle;
	cmd[1] = handle >> 8;
	cmd[2] = !!enable;

	skb = hci_cmd_sync(hdev, 0xfc57, sizeof(cmd), cmd, HCI_CMD_TIMEOUT);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	kfree_skb(skb);
	return 0;
}
