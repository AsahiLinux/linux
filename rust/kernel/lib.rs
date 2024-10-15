// SPDX-License-Identifier: GPL-2.0

//! The `kernel` crate.
//!
//! This crate contains the kernel APIs that have been ported or wrapped for
//! usage by Rust code in the kernel and is shared by all of them.
//!
//! In other words, all the rest of the Rust code in the kernel (e.g. kernel
//! modules written in Rust) depends on [`core`], [`alloc`] and this crate.
//!
//! If you need a kernel C API that is not ported or wrapped yet here, then
//! do so first instead of bypassing this crate.

#![no_std]
#![feature(associated_type_defaults)]
#![feature(coerce_unsized)]
#![feature(const_mut_refs)]
#![feature(const_refs_to_cell)]
#![feature(dispatch_from_dyn)]
#![feature(duration_constants)]
#![feature(new_uninit)]
#![feature(receiver_trait)]
#![feature(type_alias_impl_trait)]
#![feature(unsize)]
#![warn(clippy::undocumented_unsafe_blocks)]

// Ensure conditional compilation based on the kernel configuration works;
// otherwise we may silently break things like initcall handling.
#[cfg(not(CONFIG_RUST))]
compile_error!("Missing kernel configuration for conditional compilation");

// Allow proc-macros to refer to `::kernel` inside the `kernel` crate (this crate).
extern crate self as kernel;

pub mod alloc;
#[cfg(CONFIG_BLOCK)]
pub mod block;
mod build_assert;
pub mod delay;
pub mod device;
#[cfg(CONFIG_DMA_SHARED_BUFFER)]
pub mod dma_fence;
pub mod driver;
#[cfg(CONFIG_DRM = "y")]
pub mod drm;
pub mod error;
#[cfg(CONFIG_RUST_FW_LOADER_ABSTRACTIONS)]
pub mod firmware;
pub mod init;
pub mod io_buffer;
pub mod io_mem;
pub mod io_pgtable;
pub mod ioctl;
#[cfg(CONFIG_KUNIT)]
pub mod kunit;
pub mod module_param;
#[cfg(CONFIG_NET)]
pub mod net;
pub mod of;
pub mod page;
pub mod platform;
pub mod prelude;
pub mod print;
pub mod siphash;
pub mod soc;
mod static_assert;
#[doc(hidden)]
pub mod std_vendor;
pub mod str;
pub mod sync;
pub mod task;
pub mod time;
pub mod types;
pub mod uaccess;
pub mod user_ptr;
pub mod workqueue;
pub mod xarray;

#[doc(hidden)]
pub use bindings;
pub use macros;
pub use uapi;

#[doc(hidden)]
pub use build_error::build_error;

pub(crate) mod private {
    #[allow(unreachable_pub)]
    pub trait Sealed {}
}

/// Page size defined in terms of the `PAGE_SHIFT` macro from C.
///
/// [`PAGE_SHIFT`]: ../../../include/asm-generic/page.h
pub const PAGE_SIZE: usize = 1 << bindings::PAGE_SHIFT;

/// Prefix to appear before log messages printed from within the `kernel` crate.
const __LOG_PREFIX: &[u8] = b"rust_kernel\0";

/// The top level entrypoint to implementing a kernel module.
///
/// For any teardown or cleanup operations, your type may implement [`Drop`].
pub trait Module: Sized + Sync + Send {
    /// Called at module initialization time.
    ///
    /// Use this method to perform whatever setup or registration your module
    /// should do.
    ///
    /// Equivalent to the `module_init` macro in the C API.
    fn init(name: &'static crate::str::CStr, module: &'static ThisModule) -> error::Result<Self>;
}

/// Equivalent to `THIS_MODULE` in the C API.
///
/// C header: [`include/linux/export.h`](srctree/include/linux/export.h)
pub struct ThisModule(*mut bindings::module);

// SAFETY: `THIS_MODULE` may be used from all threads within a module.
unsafe impl Sync for ThisModule {}

impl ThisModule {
    /// Creates a [`ThisModule`] given the `THIS_MODULE` pointer.
    ///
    /// # Safety
    ///
    /// The pointer must be equal to the right `THIS_MODULE`.
    pub const unsafe fn from_ptr(ptr: *mut bindings::module) -> ThisModule {
        ThisModule(ptr)
    }

    /// Access the raw pointer for this module.
    ///
    /// It is up to the user to use it correctly.
    pub const fn as_ptr(&self) -> *mut bindings::module {
        self.0
    }

    /// Locks the module parameters to access them.
    ///
    /// Returns a [`KParamGuard`] that will release the lock when dropped.
    pub fn kernel_param_lock(&self) -> KParamGuard<'_> {
        // SAFETY: `kernel_param_lock` will check if the pointer is null and
        // use the built-in mutex in that case.
        #[cfg(CONFIG_SYSFS)]
        unsafe {
            bindings::kernel_param_lock(self.0)
        }

        KParamGuard {
            #[cfg(CONFIG_SYSFS)]
            this_module: self,
            phantom: core::marker::PhantomData,
        }
    }
}

/// Scoped lock on the kernel parameters of [`ThisModule`].
///
/// Lock will be released when this struct is dropped.
pub struct KParamGuard<'a> {
    #[cfg(CONFIG_SYSFS)]
    this_module: &'a ThisModule,
    phantom: core::marker::PhantomData<&'a ()>,
}

#[cfg(CONFIG_SYSFS)]
impl<'a> Drop for KParamGuard<'a> {
    fn drop(&mut self) {
        // SAFETY: `kernel_param_lock` will check if the pointer is null and
        // use the built-in mutex in that case. The existence of `self`
        // guarantees that the lock is held.
        unsafe { bindings::kernel_param_unlock(self.this_module.0) }
    }
}

#[cfg(not(any(testlib, test)))]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    pr_emerg!("{}\n", info);
    // SAFETY: FFI call.
    unsafe { bindings::BUG() };
}

/// Produces a pointer to an object from a pointer to one of its fields.
///
/// # Safety
///
/// The pointer passed to this macro, and the pointer returned by this macro, must both be in
/// bounds of the same allocation.
///
/// # Examples
///
/// ```
/// # use kernel::container_of;
/// struct Test {
///     a: u64,
///     b: u32,
/// }
///
/// let test = Test { a: 10, b: 20 };
/// let b_ptr = &test.b;
/// // SAFETY: The pointer points at the `b` field of a `Test`, so the resulting pointer will be
/// // in-bounds of the same allocation as `b_ptr`.
/// let test_alias = unsafe { container_of!(b_ptr, Test, b) };
/// assert!(core::ptr::eq(&test, test_alias));
/// ```
#[macro_export]
macro_rules! container_of {
    ($ptr:expr, $type:ty, $($f:tt)*) => {{
        let ptr = $ptr as *const _ as *const u8;
        let offset: usize = ::core::mem::offset_of!($type, $($f)*);
        ptr.sub(offset) as *const $type
    }}
}
