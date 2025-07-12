// SPDX-License-Identifier: GPL-2.0

//! Abstractions for system resources.
//!
//! C header: [`include/linux/ioport.h`](srctree/include/linux/ioport.h)

use core::ops::Deref;
use core::ptr::NonNull;

use crate::str::CStr;
use crate::types::Opaque;

#[cfg(CONFIG_HAS_IOPORT)]
/// Returns a reference to the global `ioport_resource` variable.
pub fn ioport_resource() -> &'static Resource {
    // SAFETY: `bindings::ioport_resoure` has global lifetime and is of type Resource.
    unsafe { Resource::as_ref(&raw mut bindings::ioport_resource) }
}

#[cfg(CONFIG_HAS_IOMEM)]
/// Returns a reference to the global `iomem_resource` variable.
pub fn iomem_resource() -> &'static Resource {
    // SAFETY: `bindings::iomem_resoure` has global lifetime and is of type Resource.
    unsafe { Resource::as_ref(&raw mut bindings::iomem_resource) }
}

/// Resource Size type.
///
/// This is a type alias to `u64` depending on the config option
/// `CONFIG_PHYS_ADDR_T_64BIT`.
#[cfg(CONFIG_PHYS_ADDR_T_64BIT)]
pub type ResourceSize = u64;

/// Resource Size type.
///
/// This is a type alias to `u32` depending on the config option
/// `CONFIG_PHYS_ADDR_T_64BIT`.
#[cfg(not(CONFIG_PHYS_ADDR_T_64BIT))]
pub type ResourceSize = u32;

/// A region allocated from a parent [`Resource`].
///
/// # Invariants
///
/// - `self.0` points to a valid `bindings::resource` that was obtained through
///   `bindings::__request_region`.
pub struct Region(NonNull<bindings::resource>);

impl Deref for Region {
    type Target = Resource;

    fn deref(&self) -> &Self::Target {
        // SAFETY: Safe as per the invariant of `Region`
        unsafe { Resource::as_ref(self.0.as_ptr()) }
    }
}

impl Drop for Region {
    fn drop(&mut self) {
        // SAFETY: Safe as per the invariant of `Region`
        let res = unsafe { Resource::as_ref(self.0.as_ptr()) };
        let flags = res.flags();

        let release_fn = if flags.contains(flags::IORESOURCE_MEM) {
            bindings::release_mem_region
        } else {
            bindings::release_region
        };

        // SAFETY: Safe as per the invariant of `Region`
        unsafe { release_fn(res.start(), res.size()) };
    }
}

// SAFETY: `Region` only holds a pointer to a C `struct resource`, which is safe to be used from
// any thread.
unsafe impl Send for Region {}

// SAFETY: `Region` only holds a pointer to a C `struct resource`, references to which are
// safe to be used from any thread.
unsafe impl Sync for Region {}

/// A resource abstraction.
///
/// # Invariants
///
/// [`Resource`] is a transparent wrapper around a valid `bindings::resource`.
#[repr(transparent)]
pub struct Resource(Opaque<bindings::resource>);

impl Resource {
    /// Create a new zeroed [`Resource`]
    pub(crate) fn zeroed() -> Self {
        Resource { 0: Opaque::<bindings::resource>::zeroed(), }
    }

    /// Gets the raw pointer to the wrapped `bindings::resource`.
    pub(crate) fn as_raw(&self) -> *mut bindings::resource {
        self.0.get()
    }

    /// Creates a reference to a [`Resource`] from a valid pointer.
    ///
    /// # Safety
    ///
    /// The caller must ensure that for the duration of 'a, the pointer will
    /// point at a valid `bindings::resource`.
    ///
    /// The caller must also ensure that the [`Resource`] is only accessed via the
    /// returned reference for the duration of 'a.
    pub(crate) const unsafe fn as_ref<'a>(ptr: *mut bindings::resource) -> &'a Self {
        // SAFETY: Self is a transparent wrapper around `Opaque<bindings::resource>`.
        unsafe { &*ptr.cast() }
    }

    /// Requests a resource region.
    ///
    /// Exclusive access will be given and the region will be marked as busy.
    /// Further calls to [`Self::request_region`] will return [`None`] if
    /// the region, or a part of it, is already in use.
    pub fn request_region(
        &self,
        start: ResourceSize,
        size: ResourceSize,
        name: &'static CStr,
        flags: Flags,
    ) -> Option<Region> {
        // SAFETY: Safe as per the invariant of `Resource`
        let region = unsafe {
            bindings::__request_region(
                self.0.get(),
                start,
                size,
                name.as_char_ptr(),
                flags.0 as i32,
            )
        };

        Some(Region(NonNull::new(region)?))
    }

    /// Returns the size of the resource.
    pub fn size(&self) -> ResourceSize {
        let inner = self.0.get();
        // SAFETY: safe as per the invariants of `Resource`
        unsafe { bindings::resource_size(inner) }
    }

    /// Returns the start address of the resource.
    pub fn start(&self) -> u64 {
        let inner = self.0.get();
        // SAFETY: safe as per the invariants of `Resource`
        unsafe { *inner }.start
    }

    /// Returns the name of the resource.
    pub fn name(&self) -> &'static CStr {
        let inner = self.0.get();
        // SAFETY: safe as per the invariants of `Resource`
        unsafe { CStr::from_char_ptr((*inner).name) }
    }

    /// Returns the flags associated with the resource.
    pub fn flags(&self) -> Flags {
        let inner = self.0.get();
        // SAFETY: safe as per the invariants of `Resource`
        let flags = unsafe { *inner }.flags;

        Flags(flags)
    }
}

// SAFETY: `Resource` only holds a pointer to a C `struct resource`, which is safe to be used from
// any thead.
unsafe impl Send for Resource {}

// SAFETY: `Resource` only holds a pointer to a C `struct resource`, references to which are
// safe to be used from any thead.
unsafe impl Sync for Resource {}

/// Resource flags as stored in the C `struct resource::flags` field.
///
/// They can be combined with the operators `|`, `&`, and `!`.
///
/// Values can be used from the [`flags`] module.
#[derive(Clone, Copy, PartialEq)]
pub struct Flags(usize);

impl Flags {
    /// Check whether `flags` is contained in `self`.
    pub fn contains(self, flags: Flags) -> bool {
        (self & flags) == flags
    }
}

impl core::ops::BitOr for Flags {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitAnd for Flags {
    type Output = Self;
    fn bitand(self, rhs: Self) -> Self::Output {
        Self(self.0 & rhs.0)
    }
}

impl core::ops::Not for Flags {
    type Output = Self;
    fn not(self) -> Self::Output {
        Self(!self.0)
    }
}

/// Resource flags as stored in the `struct resource::flags` field.
pub mod flags {
    use super::Flags;

    /// PCI/ISA I/O ports.
    pub const IORESOURCE_IO: Flags = Flags(bindings::IORESOURCE_IO as usize);

    /// Resource is software muxed.
    pub const IORESOURCE_MUXED: Flags = Flags(bindings::IORESOURCE_MUXED as usize);

    /// Resource represents a memory region.
    pub const IORESOURCE_MEM: Flags = Flags(bindings::IORESOURCE_MEM as usize);

    /// Resource represents a memory region that must be ioremaped using `ioremap_np`.
    pub const IORESOURCE_MEM_NONPOSTED: Flags = Flags(bindings::IORESOURCE_MEM_NONPOSTED as usize);
}
