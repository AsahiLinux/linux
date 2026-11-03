// SPDX-License-Identifier: GPL-2.0

use kernel::io::register;

// PDISP

register! {
    pub(super) NV_PDISP_VGA_WORKSPACE_BASE(u32) @ 0x00625f04 {
        /// VGA workspace base address divided by 0x10000.
        31:8    addr;
        /// Set if the `addr` field is valid.
        3:3     status_valid => bool;
    }
}

impl NV_PDISP_VGA_WORKSPACE_BASE {
    /// Returns the base address of the VGA workspace, or `None` if none exists.
    pub(super) fn vga_workspace_addr(self) -> Option<u64> {
        if self.status_valid() {
            Some(u64::from(self.addr()) << 16)
        } else {
            None
        }
    }
}
