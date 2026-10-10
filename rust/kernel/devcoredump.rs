// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Device coredump support.
//!
//! C header: [`include/linux/devcoredump.h`](srctree/include/linux/devcoredump.h)

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
    /// Reads part of the coredump into `buf`, starting at `offset`.
    ///
    /// Returns how many bytes were written. Return `Ok(0)` once there is nothing left to read.
    ///
    /// Userspace can read from several threads at once, so this may run concurrently.
    fn read(&self, buf: &mut [u8], offset: usize) -> Result<usize>;
}

/// # Safety
///
/// `buffer` must point to `count` writable bytes, and `data` must come from `T::into_foreign()`
/// and not have been freed yet.
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
    // SAFETY: `data` comes from `T::into_foreign()` and has not been freed yet.
    let coredump = unsafe { T::borrow(data.cast()) };
    // The core hands us an uninitialized buffer and copies back however many bytes we report.
    // Zero it first, so a sloppy `read()` cannot leak old heap contents to userspace.
    //
    // SAFETY: `buffer` points to `count` writable bytes.
    unsafe { buffer.write_bytes(0, count) };
    // SAFETY: Same as above, and the buffer is initialized now.
    let buf = unsafe { core::slice::from_raw_parts_mut(buffer, count) };

    from_result(|| {
        let len = coredump.read(buf, offset.try_into()?)?;

        if len > count {
            return Err(EIO);
        }

        Ok(len.try_into()?)
    })
}

/// # Safety
///
/// `data` must come from `T::into_foreign()`, and must not be used again after this call.
unsafe extern "C" fn free_callback<
    'a,
    T: ForeignOwnable<Borrowed<'a>: Deref<Target = D>>,
    D: DevCoreDump,
>(
    data: *mut crate::ffi::c_void,
) {
    // SAFETY: `data` comes from `T::into_foreign()`, and we own it from here on.
    unsafe {
        T::from_foreign(data.cast());
    }
}

/// Registers a coredump for the given device.
///
/// The core takes ownership of `coredump` and drops it when userspace dismisses the dump or
/// `timeout` runs out. If the dump cannot be registered, for example because the previous one for
/// `dev` is still pending, it is dropped right away.
///
/// `module` should be the module implementing [`DevCoreDump`]: it stays pinned for as long as the
/// dump is around.
pub fn dev_coredump<'a, T, D>(
    dev: &device::Device,
    module: &'static ThisModule,
    coredump: T,
    gfp: alloc::Flags,
    timeout: Jiffies,
) where
    // The dump outlives the caller and gets dropped from a workqueue.
    T: ForeignOwnable<Borrowed<'a>: Deref<Target = D>> + Send + 'static,
    D: DevCoreDump + Sync,
{
    // SAFETY: `dev` and `module` are valid, and the core takes its own references to them. The
    // pointer from `into_foreign()` only ever reaches our two callbacks, and `free_callback` runs
    // last and exactly once.
    unsafe {
        bindings::dev_coredumpm_timeout(
            dev.as_raw(),
            module.0,
            coredump.into_foreign(),
            0,
            gfp.as_raw(),
            Some(read_callback::<'a, T, D>),
            Some(free_callback::<'a, T, D>),
            timeout,
        )
    }
}
