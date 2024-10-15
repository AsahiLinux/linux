// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM subsystem abstractions.

pub mod device;
pub mod drv;
pub mod file;
pub mod gem;
#[cfg(CONFIG_DRM_GPUVM = "y")]
pub mod gpuvm;
pub mod ioctl;
pub mod mm;
pub mod sched;
pub mod syncobj;
