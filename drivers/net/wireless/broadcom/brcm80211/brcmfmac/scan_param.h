// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2023 Daniel Berlin
 */

#ifndef _BRCMF_SCAN_PARAM_H
#define _BRCMF_SCAN_PARAM_H

struct brcmf_pub;

/**
 * brcmf_scan_param_setup_for_version() - Setup the driver to handle join structures
 *
 * There are a number of different structures and interface versions for scanning info
 * This sets up the driver to handle a particular interface version.
 *
 * @drvr Driver structure to setup
 * @ver Interface version
 * Return: %0 if okay, error code otherwise
 */
int brcmf_scan_param_setup_for_version(struct brcmf_pub *, u8 ver);
#endif /* _BRCMF_SCAN_PARAM_H */
