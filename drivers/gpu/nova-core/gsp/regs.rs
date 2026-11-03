// SPDX-License-Identifier: GPL-2.0

use kernel::io::register;

// PGSP

register! {
    pub(super) NV_PGSP_QUEUE_HEAD(u32) @ 0x00110c00 {
        31:0    address;
    }
}
