// SPDX-License-Identifier: GPL-2.0

//! Generic memory-mapped IO.

use core::ops::Deref;
use core::ptr::NonNull;

use crate::device::Bound;
use crate::device::Device;
use crate::devres::Devres;
use crate::io;
use crate::io::resource::Region;
use crate::io::resource::Resource;
use crate::io::Io;
use crate::io::IoRaw;
use crate::prelude::*;
use crate::types::declare_flags_type;

/// An exclusive memory-mapped IO region.
///
/// # Invariants
///
/// - [`ExclusiveIoMem`] has exclusive access to the underlying [`IoMem`].
pub struct ExclusiveIoMem<const SIZE: usize> {
    /// The region abstraction. This represents exclusive access to the
    /// range represented by the underlying `iomem`.
    ///
    /// It's placed first to ensure that the region is released before it is
    /// unmapped as a result of the drop order.
    ///
    /// This field is needed for ownership of the region.
    _region: Region,
    /// The underlying `IoMem` instance.
    iomem: IoMem<SIZE>,
}

impl<const SIZE: usize> ExclusiveIoMem<SIZE> {
    /// Creates a new `ExclusiveIoMem` instance.
    pub(crate) fn ioremap(resource: &Resource) -> Result<Self> {
        let iomem = IoMem::ioremap(resource)?;

        let start = resource.start();
        let size = resource.size();
        let name = resource.name();

        let region = resource
            .request_region(start, size, name, io::resource::flags::IORESOURCE_MEM)
            .ok_or(EBUSY)?;

        let iomem = ExclusiveIoMem {
            iomem,
            _region: region,
        };

        Ok(iomem)
    }

    pub(crate) fn new(resource: &Resource, device: &Device<Bound>) -> Result<Devres<Self>> {
        let iomem = Self::ioremap(resource)?;
        let devres = Devres::new(device, iomem, GFP_KERNEL)?;

        Ok(devres)
    }
}

impl<const SIZE: usize> Deref for ExclusiveIoMem<SIZE> {
    type Target = Io<SIZE>;

    fn deref(&self) -> &Self::Target {
        &self.iomem
    }
}

/// A generic memory-mapped IO region.
///
/// Accesses to the underlying region is checked either at compile time, if the
/// region's size is known at that point, or at runtime otherwise.
///
/// # Invariants
///
/// [`IoMem`] always holds an [`IoRaw`] instance that holds a valid pointer to the
/// start of the I/O memory mapped region.
pub struct IoMem<const SIZE: usize = 0> {
    io: IoRaw<SIZE>,
}

impl<const SIZE: usize> IoMem<SIZE> {
    fn ioremap(resource: &Resource) -> Result<Self> {
        let size = resource.size();
        if size == 0 {
            return Err(EINVAL);
        }

        let res_start = resource.start();

        let addr = if resource
            .flags()
            .contains(io::resource::flags::IORESOURCE_MEM_NONPOSTED)
        {
            // SAFETY:
            // - `res_start` and `size` are read from a presumably valid `struct resource`.
            // - `size` is known not to be zero at this point.
            unsafe { bindings::ioremap_np(res_start, size as usize) }
        } else {
            // SAFETY:
            // - `res_start` and `size` are read from a presumably valid `struct resource`.
            // - `size` is known not to be zero at this point.
            unsafe { bindings::ioremap(res_start, size as usize) }
        };

        if addr.is_null() {
            return Err(ENOMEM);
        }

        let io = IoRaw::new(addr as usize, size as usize)?;
        let io = IoMem { io };

        Ok(io)
    }

    /// Creates a new `IoMem` instance.
    pub(crate) fn new(resource: &Resource, device: &Device<Bound>) -> Result<Devres<Self>> {
        let io = Self::ioremap(resource)?;
        let devres = Devres::new(device, io, GFP_KERNEL)?;

        Ok(devres)
    }
}

impl<const SIZE: usize> Drop for IoMem<SIZE> {
    fn drop(&mut self) {
        // SAFETY: Safe as by the invariant of `Io`.
        unsafe { bindings::iounmap(self.io.addr() as *mut core::ffi::c_void) }
    }
}

impl<const SIZE: usize> Deref for IoMem<SIZE> {
    type Target = Io<SIZE>;

    fn deref(&self) -> &Self::Target {
        // SAFETY: Safe as by the invariant of `IoMem`.
        unsafe { Io::from_raw(&self.io) }
    }
}

declare_flags_type! {
    /// Flags to be used when remapping memory.
    ///
    /// They can be combined with the operators `|`, `&`, and `!`.
    pub struct MemFlags(crate::ffi::c_ulong) = 0;
}

impl MemFlags {
    /// Matches the default mapping for System RAM on the architecture.
    ///
    /// This is usually a read-allocate write-back cache. Moreover, if this flag is specified and
    /// the requested remap region is RAM, memremap() will bypass establishing a new mapping and
    /// instead return a pointer into the direct map.
    pub const WB: MemFlags = MemFlags(bindings::MEMREMAP_WB as _);

    /// Establish a mapping whereby writes either bypass the cache or are written through to memory
    /// and never exist in a cache-dirty state with respect to program visibility.
    ///
    /// Attempts to map System RAM with this mapping type will fail.
    pub const WT: MemFlags = MemFlags(bindings::MEMREMAP_WT as _);
    /// Establish a writecombine mapping, whereby writes may be coalesced together  (e.g. in the
    /// CPU's write buffers), but is otherwise uncached.
    ///
    /// Attempts to map System RAM with this mapping type will fail.
    pub const WC: MemFlags = MemFlags(bindings::MEMREMAP_WC as _);

    // Note: Skipping MEMREMAP_ENC/DEC since they are under-documented and have zero
    // users outside of arch/x86.
}

/// Represents a non-MMIO memory block. This is like [`IoMem`], but for cases where it is known
/// that the resource being mapped does not have I/O side effects.
// Invariants:
// `ptr` is a non-null and valid address of at least `usize` bytes and returned by a `memremap`
// call.
// ```
pub struct Mem {
    ptr: NonNull<crate::ffi::c_void>,
    size: usize,
}

impl Mem {
    /// Tries to create a new instance of a memory block from a Resource.
    ///
    /// The resource described by `res` is mapped into the CPU's address space so that it can be
    /// accessed directly. It is also consumed by this function so that it can't be mapped again
    /// to a different address.
    ///
    /// If multiple caching flags are specified, the different mapping types will be attempted in
    /// the order [`MemFlags::WB`], [`MemFlags::WT`], [`MemFlags::WC`].
    ///
    /// # Flags
    ///
    /// * [`MemFlags::WB`]: Matches the default mapping for System RAM on the architecture.
    ///   This is usually a read-allocate write-back cache. Moreover, if this flag is specified and
    ///   the requested remap region is RAM, memremap() will bypass establishing a new mapping and
    ///   instead return a pointer into the direct map.
    ///
    /// * [`MemFlags::WT`]: Establish a mapping whereby writes either bypass the cache or are written
    ///   through to memory and never exist in a cache-dirty state with respect to program visibility.
    ///   Attempts to map System RAM with this mapping type will fail.
    /// * [`MemFlags::WC`]: Establish a writecombine mapping, whereby writes may be coalesced together
    ///   (e.g. in the CPU's write buffers), but is otherwise uncached. Attempts to map System RAM with
    ///   this mapping type will fail.
    ///
    /// # Safety
    ///
    /// Callers must ensure that either (a) the resulting interface cannot be used to initiate DMA
    /// operations, or (b) that DMA operations initiated via the returned interface use DMA handles
    /// allocated through the `dma` module.
    pub unsafe fn try_new(res: Resource, flags: MemFlags) -> Result<Self> {
        let size: usize = res.size().try_into()?;

        let addr = unsafe { bindings::memremap(res.start(), size, flags.as_raw()) };
        let ptr = NonNull::new(addr).ok_or(ENOMEM)?;
        // INVARIANT: `ptr` is non-null and was returned by `memremap`, so it is valid.
        Ok(Self { ptr, size })
    }

    /// Returns the base address of the memory mapping as a raw pointer.
    ///
    /// It is up to the caller to use this pointer safely, depending on the requirements of the
    /// hardware backing this memory block.
    pub fn ptr(&self) -> *mut u8 {
        self.ptr.cast().as_ptr()
    }

    /// Returns the size of this mapped memory block.
    pub fn size(&self) -> usize {
        self.size
    }
}

impl Drop for Mem {
    fn drop(&mut self) {
        // SAFETY: By the type invariant, `self.ptr` is a value returned by a previous successful
        // call to `memremap`.
        unsafe { bindings::memunmap(self.ptr.as_ptr()) };
    }
}
