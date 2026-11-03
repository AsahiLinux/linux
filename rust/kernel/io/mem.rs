// SPDX-License-Identifier: GPL-2.0

//! Generic memory-mapped IO.

use core::ptr::NonNull;

use crate::{
    device::{
        Bound,
        Device, //
    },
    devres::Devres,
    impl_flags,
    io::{
        self,
        resource::{
            Region,
            Resource, //
        },
        IoBase,
        Mmio,
        MmioBackend,
        MmioRaw, //
    },
    prelude::*,
};

/// An IO request for a specific device and resource.
pub struct IoRequest<'a> {
    device: &'a Device<Bound>,
    resource: &'a Resource,
}

impl<'a> IoRequest<'a> {
    /// Creates a new [`IoRequest`] instance.
    ///
    /// # Safety
    ///
    /// Callers must ensure that `resource` is valid for `device` during the
    /// lifetime `'a`.
    pub(crate) unsafe fn new(device: &'a Device<Bound>, resource: &'a Resource) -> Self {
        IoRequest { device, resource }
    }

    /// Maps an [`IoRequest`] where the size is known at compile time.
    ///
    /// This uses the [`ioremap()`] C API.
    ///
    /// [`ioremap()`]: https://docs.kernel.org/driver-api/device-io.html#getting-access-to-the-device
    ///
    /// # Examples
    ///
    /// The following example uses a [`kernel::platform::Device`] for
    /// illustration purposes.
    ///
    /// ```no_run
    /// use kernel::{
    ///     bindings,
    ///     device::Core,
    ///     io::Io,
    ///     of,
    ///     platform,
    /// };
    /// struct SampleDriver;
    ///
    /// impl platform::Driver for SampleDriver {
    ///    # type IdInfo = ();
    ///    # type Data<'bound> = Self;
    ///
    ///    fn probe<'bound>(
    ///       pdev: &'bound platform::Device<Core<'_>>,
    ///       info: Option<&'bound Self::IdInfo>,
    ///    ) -> impl PinInit<Self, Error> + 'bound {
    ///       let offset = 0; // Some offset.
    ///
    ///       // If the size is known at compile time, use [`Self::iomap_sized`].
    ///       //
    ///       // No runtime checks will apply when reading and writing.
    ///       let request = pdev.io_request_by_index(0).ok_or(ENODEV)?;
    ///       let iomem = request.iomap_sized::<42>()?;
    ///
    ///       // Read and write a 32-bit value at `offset`.
    ///       let data = iomem.read32(offset);
    ///
    ///       iomem.write32(data, offset);
    ///
    ///       # Ok(SampleDriver)
    ///     }
    /// }
    /// ```
    pub fn iomap_sized<const SIZE: usize>(self) -> Result<IoMem<'a, SIZE>> {
        IoMem::ioremap(self.device, self.resource)
    }

    /// Same as [`Self::iomap_sized`] but with exclusive access to the
    /// underlying region.
    ///
    /// This uses the [`ioremap()`] C API.
    ///
    /// [`ioremap()`]: https://docs.kernel.org/driver-api/device-io.html#getting-access-to-the-device
    pub fn iomap_exclusive_sized<const SIZE: usize>(self) -> Result<ExclusiveIoMem<'a, SIZE>> {
        ExclusiveIoMem::ioremap(self.device, self.resource)
    }

    /// Maps an [`IoRequest`] where the size is not known at compile time,
    ///
    /// This uses the [`ioremap()`] C API.
    ///
    /// [`ioremap()`]: https://docs.kernel.org/driver-api/device-io.html#getting-access-to-the-device
    ///
    /// # Examples
    ///
    /// The following example uses a [`kernel::platform::Device`] for
    /// illustration purposes.
    ///
    /// ```no_run
    /// use kernel::{
    ///     bindings,
    ///     device::Core,
    ///     io::Io,
    ///     of,
    ///     platform,
    /// };
    /// struct SampleDriver;
    ///
    /// impl platform::Driver for SampleDriver {
    ///    # type IdInfo = ();
    ///    # type Data<'bound> = Self;
    ///
    ///    fn probe<'bound>(
    ///       pdev: &'bound platform::Device<Core<'_>>,
    ///       info: Option<&'bound Self::IdInfo>,
    ///    ) -> impl PinInit<Self, Error> + 'bound {
    ///       let offset = 0; // Some offset.
    ///
    ///       // Unlike [`Self::iomap_sized`], here the size of the memory region
    ///       // is not known at compile time, so only the `try_read*` and `try_write*`
    ///       // family of functions should be used, leading to runtime checks on every
    ///       // access.
    ///       let request = pdev.io_request_by_index(0).ok_or(ENODEV)?;
    ///       let iomem = request.iomap()?;
    ///
    ///       let data = iomem.try_read32(offset)?;
    ///
    ///       iomem.try_write32(data, offset)?;
    ///
    ///       # Ok(SampleDriver)
    ///     }
    /// }
    /// ```
    pub fn iomap(self) -> Result<IoMem<'a>> {
        self.iomap_sized::<0>()
    }

    /// Same as [`Self::iomap`] but with exclusive access to the underlying
    /// region.
    pub fn iomap_exclusive(self) -> Result<ExclusiveIoMem<'a, 0>> {
        self.iomap_exclusive_sized::<0>()
    }
}

/// An exclusive memory-mapped IO region.
///
/// # Invariants
///
/// - [`ExclusiveIoMem`] has exclusive access to the underlying [`IoMem`].
pub struct ExclusiveIoMem<'a, const SIZE: usize> {
    /// The underlying `IoMem` instance.
    iomem: IoMem<'a, SIZE>,

    /// The region abstraction. This represents exclusive access to the
    /// range represented by the underlying `iomem`.
    ///
    /// This field is needed for ownership of the region.
    _region: Region,
}

impl<'a, const SIZE: usize> ExclusiveIoMem<'a, SIZE> {
    /// Creates a new `ExclusiveIoMem` instance.
    fn ioremap(dev: &'a Device<Bound>, resource: &Resource) -> Result<Self> {
        let start = resource.start();
        let size = resource.size();
        let name = resource.name().unwrap_or_default();

        let region = resource
            .request_region(
                start,
                size,
                name.to_cstring()?,
                io::resource::Flags::IORESOURCE_MEM,
            )
            .ok_or(EBUSY)?;

        let iomem = IoMem::ioremap(dev, resource)?;

        Ok(ExclusiveIoMem {
            iomem,
            _region: region,
        })
    }

    /// Consume the `ExclusiveIoMem` and register it as a device-managed resource.
    ///
    /// The returned `Devres<ExclusiveIoMem<'static, SIZE>>` can outlive the original lifetime
    /// `'a`. Access to the I/O memory is revoked when the device is unbound.
    pub fn into_devres(self) -> Result<Devres<ExclusiveIoMem<'static, SIZE>>> {
        // SAFETY: Casting to `'static` is sound because `Devres` guarantees the
        // `ExclusiveIoMem` does not actually outlive the device -- access is revoked and the
        // resource is released when the device is unbound.
        let iomem: ExclusiveIoMem<'static, SIZE> = unsafe { core::mem::transmute(self) };
        let dev = iomem.iomem.dev;
        Devres::new(dev, iomem)
    }
}

impl<'a, const SIZE: usize> IoBase<'a> for &'a ExclusiveIoMem<'_, SIZE> {
    type Backend = MmioBackend;
    type Target = super::Region<SIZE>;

    #[inline]
    fn as_view(self) -> Mmio<'a, Self::Target> {
        self.iomem.as_view()
    }
}

/// A generic memory-mapped IO region.
///
/// Accesses to the underlying region is checked either at compile time, if the
/// region's size is known at that point, or at runtime otherwise.
///
/// # Invariants
///
/// [`IoMem`] always holds an [`MmioRaw`] instance that holds a valid pointer to the
/// start of the I/O memory mapped region.
pub struct IoMem<'a, const SIZE: usize = 0> {
    dev: &'a Device<Bound>,
    io: MmioRaw<super::Region<SIZE>>,
}

impl<'a, const SIZE: usize> IoMem<'a, SIZE> {
    fn ioremap(dev: &'a Device<Bound>, resource: &Resource) -> Result<Self> {
        // Note: Some ioremap() implementations use types that depend on the CPU
        // word width rather than the bus address width.
        //
        // TODO: Properly address this in the C code to avoid this `try_into`.
        let size = resource.size().try_into()?;
        if size == 0 {
            return Err(EINVAL);
        }

        let res_start = resource.start();

        let addr = if resource
            .flags()
            .contains(io::resource::Flags::IORESOURCE_MEM_NONPOSTED)
        {
            // SAFETY:
            // - `res_start` and `size` are read from a presumably valid `struct resource`.
            // - `size` is known not to be zero at this point.
            unsafe { bindings::ioremap_np(res_start, size) }
        } else {
            // SAFETY:
            // - `res_start` and `size` are read from a presumably valid `struct resource`.
            // - `size` is known not to be zero at this point.
            unsafe { bindings::ioremap(res_start, size) }
        };

        if addr.is_null() {
            return Err(ENOMEM);
        }

        let io = MmioRaw::new_region(addr as usize, size)?;
        Ok(IoMem { dev, io })
    }

    /// Consume the `IoMem` and register it as a device-managed resource.
    ///
    /// The returned `Devres<IoMem<'static, SIZE>>` can outlive the original
    /// lifetime `'a`. Access to the I/O memory is revoked when the device
    /// is unbound.
    pub fn into_devres(self) -> Result<Devres<IoMem<'static, SIZE>>> {
        // SAFETY: Casting to `'static` is sound because `Devres` guarantees the `IoMem` does not
        // actually outlive the device -- access is revoked and the resource is released when the
        // device is unbound.
        let iomem: IoMem<'static, SIZE> = unsafe { core::mem::transmute(self) };
        let dev = iomem.dev;
        Devres::new(dev, iomem)
    }
}

impl<const SIZE: usize> Drop for IoMem<'_, SIZE> {
    fn drop(&mut self) {
        // SAFETY: Safe as by the invariant of `Io`.
        unsafe { bindings::iounmap(self.io.addr() as *mut c_void) }
    }
}

impl<'a, const SIZE: usize> IoBase<'a> for &'a IoMem<'_, SIZE> {
    type Backend = MmioBackend;
    type Target = super::Region<SIZE>;

    #[inline]
    fn as_view(self) -> Mmio<'a, Self::Target> {
        // SAFETY: Safe as by the invariant of `IoMem`.
        unsafe { Mmio::from_raw(self.io) }
    }
}

impl_flags!(
    /// Flags to be used when remapping memory.
    #[derive(Debug, Clone, Default, Copy, PartialEq, Eq)]
    pub struct MemFlags(usize);

    /// Enum mirroring the C MEMREMAP_* eum values
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum MemFlag {
        /// Matches the default mapping for System RAM on the architecture.
        ///
        /// This is usually a read-allocate write-back cache. Moreover, if this flag is specified and
        /// the requested remap region is RAM, memremap() will bypass establishing a new mapping and
        /// instead return a pointer into the direct map.
        WB = bindings::MEMREMAP_WB as usize,

        /// Establish a mapping whereby writes either bypass the cache or are written through to memory
        /// and never exist in a cache-dirty state with respect to program visibility.
        ///
        /// Attempts to map System RAM with this mapping type will fail.
        WT = bindings::MEMREMAP_WT as usize,

        /// Establish a writecombine mapping, whereby writes may be coalesced together  (e.g. in the
        /// CPU's write buffers), but is otherwise uncached.
        ///
        /// Attempts to map System RAM with this mapping type will fail.
        WC = bindings::MEMREMAP_WC as usize,
        // Note: Skipping MEMREMAP_ENC/DEC since they are under-documented and have zero
        // users outside of arch/x86.
    }
);

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
    /// the order [`MemFlag::WB`], [`MemFlag::WT`], [`MemFlag::WC`].
    ///
    /// # Flags
    ///
    /// * [`MemFlag::WB`]: Matches the default mapping for System RAM on the architecture.
    ///   This is usually a read-allocate write-back cache. Moreover, if this flag is specified and
    ///   the requested remap region is RAM, memremap() will bypass establishing a new mapping and
    ///   instead return a pointer into the direct map.
    ///
    /// * [`MemFlag::WT`]: Establish a mapping whereby writes either bypass the cache or are written
    ///   through to memory and never exist in a cache-dirty state with respect to program visibility.
    ///   Attempts to map System RAM with this mapping type will fail.
    /// * [`MemFlag::WC`]: Establish a writecombine mapping, whereby writes may be coalesced together
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

        let addr = unsafe { bindings::memremap(res.start(), size, flags.into()) };
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
