/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2019-2020 Linaro Limited */

#ifndef XHCI_PCI_H
#define XHCI_PCI_H

int xhci_pci_common_probe(struct pci_dev *dev, const struct pci_device_id *id);
void xhci_pci_remove(struct pci_dev *dev);

struct xhci_driver_data {
	u64 quirks;
	const char *firmware;
};

#if IS_ENABLED(CONFIG_USB_XHCI_PCI_ASMEDIA)
int asmedia_xhci_check_request_fw(struct pci_dev *dev,
				  const struct pci_device_id *id);

#else
static inline int asmedia_xhci_check_request_fw(struct pci_dev *dev,
						const struct pci_device_id *id)
{
	return 0;
}

#endif

#endif
