// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Bridge for PMPv1 drivers

use crate::prelude::*;

/// Allows report-entry drivers to send power on/off messages through RTKit.
pub trait DevPwrBridge: Send + Sync {
    /// Sends a device power state request.
    fn send_devpwr(&self, dev: u64, enable: bool) -> Result<()>;

    /// Waits for the bridge to be ready to accept requests.
    fn ready(&self) -> bool;
}
