// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM device.
//!
//! C header: [`include/linux/drm/drm_device.h`](../../../../include/linux/drm/drm_device.h)

use crate::{
    bindings, device, drm,
    types::{AlwaysRefCounted, ForeignOwnable},
};
use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::ptr::NonNull;

/// A typed DRM device with a specific driver. The device is always reference-counted.
#[repr(transparent)]
pub struct Device<T: drm::drv::Driver> {
    pub(super) drm: UnsafeCell<bindings::drm_device>,
    _p: PhantomData<T>,
}

impl<T: drm::drv::Driver> Device<T> {
    #[allow(dead_code, clippy::mut_from_ref)]
    pub(crate) unsafe fn raw_mut(&self) -> &mut bindings::drm_device {
        unsafe { &mut *self.drm.get() }
    }

    // Not intended to be called externally, except via declare_drm_ioctls!()
    #[doc(hidden)]
    pub unsafe fn borrow<'a>(raw: *const bindings::drm_device) -> &'a Self {
        unsafe { &*(raw as *const Self) }
    }

    /// Returns a borrowed reference to the user data associated with this Device.
    pub fn data(&self) -> <T::Data as ForeignOwnable>::Borrowed<'_> {
        // SAFETY: dev_private is guaranteed to be initialized for all
        // Device objects exposed to users.
        unsafe { T::Data::borrow((*self.drm.get()).dev_private) }
    }
}

// SAFETY: DRM device objects are always reference counted and the get/put functions
// satisfy the requirements.
unsafe impl<T: drm::drv::Driver> AlwaysRefCounted for Device<T> {
    fn inc_ref(&self) {
        unsafe { bindings::drm_dev_get(&self.drm as *const _ as *mut _) };
    }

    unsafe fn dec_ref(obj: NonNull<Self>) {
        // SAFETY: The Device<T> type has the same layout as drm_device,
        // so we can just cast.
        unsafe { bindings::drm_dev_put(obj.as_ptr() as *mut _) };
    }
}

// SAFETY: `Device` only holds a pointer to a C device, which is safe to be used from any thread.
unsafe impl<T: drm::drv::Driver> Send for Device<T> {}

// SAFETY: `Device` only holds a pointer to a C device, references to which are safe to be used
// from any thread.
unsafe impl<T: drm::drv::Driver> Sync for Device<T> {}

// Make drm::Device work for dev_info!() and friends
unsafe impl<T: drm::drv::Driver> device::RawDevice for Device<T> {
    fn raw_device(&self) -> *mut bindings::device {
        // SAFETY: dev is initialized by C for all Device objects
        unsafe { (*self.drm.get()).dev }
    }
}
