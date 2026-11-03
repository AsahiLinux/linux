// SPDX-License-Identifier: GPL-2.0 or MIT
//! GEM buffer object management for the Tyr driver.
//!
//! This module provides buffer object (BO) management functionality using
//! DRM's GEM subsystem with shmem backing.

use kernel::{
    drm::gem,
    prelude::*, //
};

use crate::driver::{
    TyrDrmDevice,
    TyrDrmDriver, //
};

/// Tyr's DriverObject type for GEM objects.
#[pin_data]
pub(crate) struct BoData {
    flags: u32,
}

/// Provides a way to pass arguments when creating BoData
/// as required by the gem::DriverObject trait.
pub(crate) struct BoCreateArgs {
    flags: u32,
}

#[vtable]
impl gem::DriverObject for BoData {
    type Driver = TyrDrmDriver;
    type Args = BoCreateArgs;

    fn new(_dev: &TyrDrmDevice, _size: usize, args: BoCreateArgs) -> impl PinInit<Self, Error> {
        try_pin_init!(Self { flags: args.flags })
    }

    fn create_imported(_dev: &TyrDrmDevice, _size: usize) -> impl PinInit<Self, Error> {
        try_pin_init!(Self { flags: 0 })
    }
}
