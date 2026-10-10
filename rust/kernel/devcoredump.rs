// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Device coredump support.
//!
//! C header: [`include/linux/devcoredump.h`](../../../../include/linux/devcoredump.h)

use crate::{
    alloc, bindings, device,
    error::{code::EIO, from_result},
    prelude::Result,
    time::Jiffies,
    types::ForeignOwnable,
    ThisModule,
};

use core::ops::Deref;

/// The default timeout for device coredumps.
pub const DEFAULT_TIMEOUT: Jiffies = bindings::DEVCD_TIMEOUT as Jiffies;

/// Trait to implement reading from a device coredump.
///
/// Users must implement this trait to provide device coredump support.
pub trait DevCoreDump {
    /// Returns the IOVA (virtual address) of the buffer from RTKit's point of view, or an error if
    /// unavailable.
    fn read(&self, buf: &mut [u8], offset: usize) -> Result<usize>;
}

unsafe extern "C" fn read_callback<
    'a,
    T: ForeignOwnable<Borrowed<'a>: Deref<Target = D>>,
    D: DevCoreDump,
>(
    buffer: *mut crate::ffi::c_char,
    offset: bindings::loff_t,
    count: usize,
    data: *mut crate::ffi::c_void,
    _datalen: usize,
) -> isize {
    // SAFETY: This pointer came from into_foreign() below.
    let coredump = unsafe { T::borrow(data.cast()) };
    // The caller does not initialize `buffer`, and copies as many bytes to userspace as we return.
    // Zero it, so that an implementation returning more than it wrote cannot leak stale data.
    //
    // SAFETY: The caller guarantees `buffer` points to at least `count` bytes.
    unsafe { buffer.write_bytes(0, count) };
    // SAFETY: The caller guarantees `buffer` points to at least `count` bytes.
    let buf = unsafe { core::slice::from_raw_parts_mut(buffer, count) };

    from_result(|| {
        let len = coredump.read(buf, offset.try_into()?)?;

        // The caller copies `len` bytes out of `buffer`, which only holds `count` bytes.
        if len > count {
            return Err(EIO);
        }

        Ok(len.try_into()?)
    })
}

unsafe extern "C" fn free_callback<
    'a,
    T: ForeignOwnable<Borrowed<'a>: Deref<Target = D>>,
    D: DevCoreDump,
>(
    data: *mut crate::ffi::c_void,
) {
    // SAFETY: This pointer came from into_foreign() below.
    unsafe {
        T::from_foreign(data.cast());
    }
}

/// Registers a coredump for the given device.
pub fn dev_coredump<'a, T, D>(
    dev: &device::Device,
    module: &'static ThisModule,
    coredump: T,
    gfp: alloc::Flags,
    timeout: Jiffies,
) where
    // The coredump outlives the caller, and is dropped from a workqueue.
    T: ForeignOwnable<Borrowed<'a>: Deref<Target = D>> + Send + 'static,
    // Userspace may read the coredump from several threads at once.
    D: DevCoreDump + Sync,
{
    // SAFETY: Call upholds dev_coredumpm lifetime requirements.
    unsafe {
        bindings::dev_coredumpm_timeout(
            dev.as_raw(),
            module.0,
            coredump.into_foreign() as *mut _,
            0,
            gfp.as_raw(),
            Some(read_callback::<'a, T, D>),
            Some(free_callback::<'a, T, D>),
            timeout,
        )
    }
}
