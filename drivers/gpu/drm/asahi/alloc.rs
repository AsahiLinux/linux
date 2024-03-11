// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU kernel object allocator.
//!
//! This kernel driver needs to manage a large number of GPU objects, in both firmware/kernel
//! address space and user address space. This module implements a simple grow-only heap allocator
//! based on the DRM MM range allocator, and a debug allocator that allocates each object as a
//! separate GEM object.
//!
//! Allocations may optionally have debugging enabled, which adds preambles that store metadata
//! about the allocation. This is useful for live debugging using the hypervisor or postmortem
//! debugging with a GPU memory snapshot, since it makes it easier to identify use-after-free and
//! caching issues.

use kernel::{drm::mm, error::Result, prelude::*, str::CString};

use crate::debug::*;
use crate::driver::{AsahiDevRef, AsahiDevice};
use crate::fw::types::Zeroable;
use crate::mmu;
use crate::object::{GpuArray, GpuObject, GpuOnlyArray, GpuStruct, GpuWeakPointer};

use core::cmp::Ordering;
use core::fmt;
use core::fmt::{Debug, Formatter};
use core::marker::PhantomData;
use core::mem;
use core::mem::MaybeUninit;
use core::ptr::NonNull;

const DEBUG_CLASS: DebugFlags = DebugFlags::Alloc;

#[cfg(not(CONFIG_DRM_ASAHI_DEBUG_ALLOCATOR))]
/// The driver-global allocator type
pub(crate) type DefaultAllocator = HeapAllocator;

#[cfg(not(CONFIG_DRM_ASAHI_DEBUG_ALLOCATOR))]
/// The driver-global allocation type
pub(crate) type DefaultAllocation = HeapAllocation;

#[cfg(CONFIG_DRM_ASAHI_DEBUG_ALLOCATOR)]
/// The driver-global allocator type
pub(crate) type DefaultAllocator = SimpleAllocator;

#[cfg(CONFIG_DRM_ASAHI_DEBUG_ALLOCATOR)]
/// The driver-global allocation type
pub(crate) type DefaultAllocation = SimpleAllocation;

/// Represents a raw allocation (without any type information).
pub(crate) trait RawAllocation {
    /// Returns the CPU-side pointer (if CPU mapping is enabled) as a byte non-null pointer.
    fn ptr(&self) -> Option<NonNull<u8>>;
    /// Returns the GPU VA pointer as a u64.
    fn gpu_ptr(&self) -> u64;
    /// Returns the size of the allocation in bytes.
    fn size(&self) -> usize;
    /// Returns the AsahiDevice that owns this allocation.
    fn device(&self) -> &AsahiDevice;
}

/// Represents a typed allocation.
pub(crate) trait Allocation<T>: Debug {
    /// Returns the typed CPU-side pointer (if CPU mapping is enabled).
    fn ptr(&self) -> Option<NonNull<T>>;
    /// Returns the GPU VA pointer as a u64.
    fn gpu_ptr(&self) -> u64;
    /// Returns the size of the allocation in bytes.
    fn size(&self) -> usize;
    /// Returns the AsahiDevice that owns this allocation.
    fn device(&self) -> &AsahiDevice;
}

/// A generic typed allocation wrapping a RawAllocation.
///
/// This is currently the only Allocation implementation, since it is shared by all allocators.
pub(crate) struct GenericAlloc<T, U: RawAllocation> {
    alloc: U,
    alloc_size: usize,
    debug_offset: usize,
    padding: usize,
    tag: u32,
    pad_word: u32,
    _p: PhantomData<T>,
}

impl<T, U: RawAllocation> Allocation<T> for GenericAlloc<T, U> {
    fn ptr(&self) -> Option<NonNull<T>> {
        self.alloc
            .ptr()
            .map(|p| unsafe { NonNull::new_unchecked(p.as_ptr().add(self.debug_offset) as *mut T) })
    }
    fn gpu_ptr(&self) -> u64 {
        self.alloc.gpu_ptr() + self.debug_offset as u64
    }
    fn size(&self) -> usize {
        self.alloc_size
    }
    fn device(&self) -> &AsahiDevice {
        self.alloc.device()
    }
}

impl<T, U: RawAllocation> Debug for GenericAlloc<T, U> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct(core::any::type_name::<GenericAlloc<T, U>>())
            .field("ptr", &format_args!("{:?}", self.ptr()))
            .field("gpu_ptr", &format_args!("{:#X?}", self.gpu_ptr()))
            .field("size", &format_args!("{:#X?}", self.size()))
            .finish()
    }
}

/// Debugging data associated with an allocation, when debugging is enabled.
#[repr(C)]
struct AllocDebugData {
    state: u32,
    tag: u32,
    size: u64,
    base_gpuva: u64,
    obj_gpuva: u64,
    name: [u8; 0x20],
}

/// Magic flag indicating a live allocation.
const STATE_LIVE: u32 = u32::from_le_bytes(*b"LIVE");
/// Magic flag indicating a freed allocation.
const STATE_DEAD: u32 = u32::from_le_bytes(*b"DEAD");

/// Marker byte to identify when firmware/GPU write beyond the end of an allocation.
const GUARD_MARKER: u32 = 0x93939393;

impl<T, U: RawAllocation> Drop for GenericAlloc<T, U> {
    fn drop(&mut self) {
        let debug_len = mem::size_of::<AllocDebugData>();
        if self.debug_offset >= debug_len {
            if let Some(p) = self.alloc.ptr() {
                unsafe {
                    let p = p.as_ptr().add(self.debug_offset - debug_len);
                    (p as *mut u32).write(STATE_DEAD);
                }
            }
        }
        if debug_enabled(DebugFlags::FillAllocations) {
            if let Some(p) = self.ptr() {
                unsafe { (p.as_ptr() as *mut u8).write_bytes(0xde, self.size()) };
            }
        }
        if self.padding != 0 {
            if let Some(p) = self.ptr() {
                let guard = unsafe {
                    core::slice::from_raw_parts(
                        (p.as_ptr() as *mut u8 as *const u8).add(self.size()),
                        self.padding,
                    )
                };
                let mut first_err = None;
                let mut last_err = 0;
                for (i, p) in guard.iter().enumerate() {
                    if *p != (self.pad_word >> (8 * (i & 3))) as u8 {
                        if first_err.is_none() {
                            first_err = Some(i);
                        }
                        last_err = i;
                    }
                }
                if let Some(start) = first_err {
                    dev_warn!(
                        self.device(),
                        "Allocator: Corruption after object of type {}/{:#x} at {:#x}:{:#x} + {:#x}..={:#x}\n",
                        core::any::type_name::<T>(),
                        self.tag,
                        self.gpu_ptr(),
                        self.size(),
                        start,
                        last_err,
                    );
                }
            }
        }
    }
}

static_assert!(mem::size_of::<AllocDebugData>() == 0x40);

/// A trait representing an allocator.
pub(crate) trait Allocator {
    /// The raw allocation type used by this allocator.
    type Raw: RawAllocation;
    // TODO: Needs associated_type_defaults
    // type Allocation<T> = GenericAlloc<T, Self::Raw>;

    /// Returns the `AsahiDevice` associated with this allocator.
    fn device(&self) -> &AsahiDevice;
    /// Returns whether CPU-side mapping is enabled.
    fn cpu_maps(&self) -> bool;
    /// Returns the minimum alignment for allocations.
    fn min_align(&self) -> usize;
    /// Allocate an object of the given size in bytes with the given alignment.
    fn alloc(&mut self, size: usize, align: usize) -> Result<Self::Raw>;

    /// Returns a tuple of (count, size) of how much garbage (freed but not yet reusable objects)
    /// exists in this allocator. Optional.
    fn garbage(&self) -> (usize, usize) {
        (0, 0)
    }
    /// Collect garbage for this allocator, up to the given object count. Optional.
    fn collect_garbage(&mut self, _count: usize) {}

    /// Allocate a new GpuStruct object. See [`GpuObject::new`].
    #[inline(never)]
    fn new_object<T: GpuStruct>(
        &mut self,
        inner: T,
        callback: impl for<'a> FnOnce(&'a T) -> T::Raw<'a>,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>> {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new(self.alloc_object()?, inner, callback)
    }

    /// Allocate a new GpuStruct object. See [`GpuObject::new_boxed`].
    #[inline(never)]
    fn new_boxed<T: GpuStruct>(
        &mut self,
        inner: Box<T>,
        callback: impl for<'a> FnOnce(
            &'a T,
            &'a mut MaybeUninit<T::Raw<'a>>,
        ) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>> {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new_boxed(self.alloc_object()?, inner, callback)
    }

    /// Allocate a new GpuStruct object. See [`GpuObject::new_inplace`].
    #[inline(never)]
    fn new_inplace<T: GpuStruct>(
        &mut self,
        inner: T,
        callback: impl for<'a> FnOnce(
            &'a T,
            &'a mut MaybeUninit<T::Raw<'a>>,
        ) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>> {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new_inplace(
            self.alloc_object()?,
            inner,
            callback,
        )
    }

    /// Allocate a new GpuStruct object. See [`GpuObject::new_default`].
    #[inline(never)]
    fn new_default<T: GpuStruct + Default>(
        &mut self,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>>
    where
        for<'a> <T as GpuStruct>::Raw<'a>: Default + Zeroable,
    {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new_default(self.alloc_object()?)
    }

    /// Allocate a new GpuStruct object. See [`GpuObject::new_init_prealloc`].
    #[inline(never)]
    fn new_init_prealloc<
        'a,
        T: GpuStruct,
        I: Init<T, kernel::error::Error>,
        R: PinInit<T::Raw<'a>, kernel::error::Error>,
    >(
        &mut self,
        inner_init: impl FnOnce(GpuWeakPointer<T>) -> I,
        raw_init: impl FnOnce(&'a T, GpuWeakPointer<T>) -> R,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>> {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new_init_prealloc(
            self.alloc_object()?,
            inner_init,
            raw_init,
        )
    }

    /// Allocate a new GpuStruct object. See [`GpuObject::new_init`].
    #[inline(never)]
    fn new_init<'a, T: GpuStruct, R: PinInit<T::Raw<'a>, F>, E, F>(
        &mut self,
        inner_init: impl Init<T, E>,
        raw_init: impl FnOnce(&'a T, GpuWeakPointer<T>) -> R,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>>
    where
        kernel::error::Error: core::convert::From<E>,
        kernel::error::Error: core::convert::From<F>,
    {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new_init_prealloc(
            self.alloc_object()?,
            |_p| inner_init,
            raw_init,
        )
    }

    /// Allocate a generic buffer of the given size and alignment, applying the debug features if
    /// enabled to tag it and detect overflows.
    fn alloc_generic<T>(
        &mut self,
        size: usize,
        align: usize,
        tag: Option<u32>,
    ) -> Result<GenericAlloc<T, Self::Raw>> {
        let padding = if debug_enabled(DebugFlags::DetectOverflows) {
            size
        } else {
            0
        };

        let ret: GenericAlloc<T, Self::Raw> =
            if self.cpu_maps() && debug_enabled(debug::DebugFlags::DebugAllocations) {
                let debug_align = self.min_align().max(align);
                let debug_len = mem::size_of::<AllocDebugData>();
                let debug_offset = (debug_len * 2 + debug_align - 1) & !(debug_align - 1);

                let alloc = self.alloc(size + debug_offset + padding, align)?;

                let mut debug = AllocDebugData {
                    state: STATE_LIVE,
                    tag: tag.unwrap_or(0),
                    size: size as u64,
                    base_gpuva: alloc.gpu_ptr(),
                    obj_gpuva: alloc.gpu_ptr() + debug_offset as u64,
                    name: [0; 0x20],
                };

                let name = core::any::type_name::<T>().as_bytes();
                let len = name.len().min(debug.name.len() - 1);
                debug.name[..len].copy_from_slice(&name[..len]);

                if let Some(p) = alloc.ptr() {
                    unsafe {
                        let p = p.as_ptr();
                        p.write_bytes(0x42, debug_offset - 2 * debug_len);
                        let cur = p.add(debug_offset - debug_len) as *mut AllocDebugData;
                        let prev = p.add(debug_offset - 2 * debug_len) as *mut AllocDebugData;
                        prev.copy_from(cur, 1);
                        cur.copy_from(&debug, 1);
                    };
                }

                GenericAlloc {
                    alloc,
                    alloc_size: size,
                    debug_offset,
                    tag: tag.unwrap_or(0),
                    pad_word: tag.unwrap_or(GUARD_MARKER) | 0x81818181,
                    padding,
                    _p: PhantomData,
                }
            } else {
                GenericAlloc {
                    alloc: self.alloc(size + padding, align)?,
                    alloc_size: size,
                    debug_offset: 0,
                    tag: tag.unwrap_or(0),
                    pad_word: tag.unwrap_or(GUARD_MARKER) | 0x81818181,
                    padding,
                    _p: PhantomData,
                }
            };

        if debug_enabled(DebugFlags::FillAllocations) {
            if let Some(p) = ret.ptr() {
                unsafe { (p.as_ptr() as *mut u8).write_bytes(0xaa, ret.size()) };
            }
        }

        if padding != 0 {
            if let Some(p) = ret.ptr() {
                let guard = unsafe {
                    core::slice::from_raw_parts_mut(
                        (p.as_ptr() as *mut u8).add(ret.size()),
                        padding,
                    )
                };
                for (i, p) in guard.iter_mut().enumerate() {
                    *p = (ret.pad_word >> (8 * (i & 3))) as u8;
                }
            }
        }

        Ok(ret)
    }

    /// Allocate an object of a given type, without actually initializing the allocation.
    ///
    /// This is useful to directly call [`GpuObject::new_*`], without borrowing a reference to the
    /// allocator for the entire duration (e.g. if further allocations need to happen inside the
    /// callbacks).
    fn alloc_object<T: GpuStruct>(&mut self) -> Result<GenericAlloc<T, Self::Raw>> {
        let size = mem::size_of::<T::Raw<'static>>();
        let align = mem::align_of::<T::Raw<'static>>();

        self.alloc_generic(size, align, None)
    }

    /// Allocate an empty `GpuArray` of a given type and length.
    fn array_empty<T: Sized + Default>(
        &mut self,
        count: usize,
    ) -> Result<GpuArray<T, GenericAlloc<T, Self::Raw>>> {
        let size = mem::size_of::<T>() * count;
        let align = mem::align_of::<T>();

        let alloc = self.alloc_generic(size, align, None)?;
        GpuArray::<T, GenericAlloc<T, Self::Raw>>::empty(alloc, count)
    }

    /// Allocate an empty `GpuArray` of a given type and length.
    fn array_empty_tagged<T: Sized + Default>(
        &mut self,
        count: usize,
        tag: &[u8; 4],
    ) -> Result<GpuArray<T, GenericAlloc<T, Self::Raw>>> {
        let size = mem::size_of::<T>() * count;
        let align = mem::align_of::<T>();

        let alloc = self.alloc_generic(size, align, Some(u32::from_le_bytes(*tag)))?;
        GpuArray::<T, GenericAlloc<T, Self::Raw>>::empty(alloc, count)
    }

    /// Allocate an empty `GpuOnlyArray` of a given type and length.
    fn array_gpuonly<T: Sized + Default>(
        &mut self,
        count: usize,
    ) -> Result<GpuOnlyArray<T, GenericAlloc<T, Self::Raw>>> {
        let size = mem::size_of::<T>() * count;
        let align = mem::align_of::<T>();

        let alloc = self.alloc_generic(size, align, None)?;
        GpuOnlyArray::<T, GenericAlloc<T, Self::Raw>>::new(alloc, count)
    }
}

/// A simple allocation backed by a separate GEM object.
///
/// # Invariants
/// `ptr` is either None or a valid, non-null pointer to the CPU view of the object.
/// `gpu_ptr` is the GPU-side VA of the object.
pub(crate) struct SimpleAllocation {
    dev: AsahiDevRef,
    ptr: Option<NonNull<u8>>,
    gpu_ptr: u64,
    size: usize,
    vm: mmu::Vm,
    obj: crate::gem::ObjectRef,
}

/// SAFETY: `SimpleAllocation` just points to raw memory and should be safe to send across threads.
unsafe impl Send for SimpleAllocation {}
unsafe impl Sync for SimpleAllocation {}

impl Drop for SimpleAllocation {
    fn drop(&mut self) {
        mod_dev_dbg!(
            self.device(),
            "SimpleAllocator: drop object @ {:#x}\n",
            self.gpu_ptr()
        );
        if debug_enabled(DebugFlags::FillAllocations) {
            if let Ok(vmap) = self.obj.vmap() {
                vmap.as_mut_slice().fill(0x42);
            }
        }
        self.obj.drop_vm_mappings(self.vm.id());
    }
}

impl RawAllocation for SimpleAllocation {
    fn ptr(&self) -> Option<NonNull<u8>> {
        self.ptr
    }
    fn gpu_ptr(&self) -> u64 {
        self.gpu_ptr
    }
    fn size(&self) -> usize {
        self.size
    }

    fn device(&self) -> &AsahiDevice {
        &self.dev
    }
}

/// A simple allocator that allocates each object as its own GEM object, aligned to the end of a
/// page.
///
/// This is very slow, but it has the advantage that over-reads by the firmware or GPU will fault on
/// the guard page after the allocation, which can be useful to validate that the firmware's or
/// GPU's idea of object size what we expect.
pub(crate) struct SimpleAllocator {
    dev: AsahiDevRef,
    start: u64,
    end: u64,
    prot: u32,
    vm: mmu::Vm,
    min_align: usize,
    cpu_maps: bool,
}

impl SimpleAllocator {
    /// Create a new `SimpleAllocator` for a given address range and `Vm`.
    #[allow(dead_code)]
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        dev: &AsahiDevice,
        vm: &mmu::Vm,
        start: u64,
        end: u64,
        min_align: usize,
        prot: u32,
        _block_size: usize,
        mut cpu_maps: bool,
        _name: fmt::Arguments<'_>,
        _keep_garbage: bool,
    ) -> Result<SimpleAllocator> {
        if debug_enabled(DebugFlags::ForceCPUMaps) {
            cpu_maps = true;
        }
        Ok(SimpleAllocator {
            dev: dev.into(),
            vm: vm.clone(),
            start,
            end,
            prot,
            min_align,
            cpu_maps,
        })
    }
}

impl Allocator for SimpleAllocator {
    type Raw = SimpleAllocation;

    fn device(&self) -> &AsahiDevice {
        &self.dev
    }

    fn cpu_maps(&self) -> bool {
        self.cpu_maps
    }

    fn min_align(&self) -> usize {
        self.min_align
    }

    #[inline(never)]
    fn alloc(&mut self, size: usize, align: usize) -> Result<SimpleAllocation> {
        let size_aligned = (size + mmu::UAT_PGSZ - 1) & !mmu::UAT_PGMSK;
        let align = self.min_align.max(align);
        let offset = (size_aligned - size) & !(align - 1);

        mod_dev_dbg!(
            &self.dev,
            "SimpleAllocator::new: size={:#x} size_al={:#x} al={:#x} off={:#x}\n",
            size,
            size_aligned,
            align,
            offset
        );

        let mut obj = crate::gem::new_kernel_object(&self.dev, size_aligned)?;
        let p = obj.vmap()?.as_mut_ptr() as *mut u8;
        if debug_enabled(DebugFlags::FillAllocations) {
            obj.vmap()?.as_mut_slice().fill(0xde);
        }
        let iova = obj.map_into_range(
            &self.vm,
            self.start,
            self.end,
            self.min_align.max(mmu::UAT_PGSZ) as u64,
            self.prot,
            true,
        )?;

        let ptr = unsafe { p.add(offset) };
        let gpu_ptr = (iova + offset) as u64;

        mod_dev_dbg!(
            &self.dev,
            "SimpleAllocator::new -> {:#?} / {:#?} | {:#x} / {:#x}\n",
            p,
            ptr,
            iova,
            gpu_ptr
        );

        Ok(SimpleAllocation {
            dev: self.dev.clone(),
            ptr: NonNull::new(ptr),
            gpu_ptr,
            size,
            vm: self.vm.clone(),
            obj,
        })
    }
}

/// Inner data for an allocation from the heap allocator.
///
/// This is wrapped in an `mm::Node`.
pub(crate) struct HeapAllocationInner {
    dev: AsahiDevRef,
    ptr: Option<NonNull<u8>>,
    real_size: usize,
}

/// SAFETY: `SimpleAllocation` just points to raw memory and should be safe to send across threads.
unsafe impl Send for HeapAllocationInner {}
unsafe impl Sync for HeapAllocationInner {}

/// Outer view of a heap allocation.
///
/// This uses an Option<> so we can move the internal `Node` into the garbage pool when it gets
/// dropped.
///
/// # Invariants
/// The `Option` must always be `Some(...)` while this object is alive.
pub(crate) struct HeapAllocation(Option<mm::Node<HeapAllocatorInner, HeapAllocationInner>>);

impl Drop for HeapAllocation {
    fn drop(&mut self) {
        let node = self.0.take().unwrap();
        let size = node.size();
        let alloc = node.alloc_ref();

        alloc.with(|a| {
            if let Some(garbage) = a.garbage.as_mut() {
                if garbage.try_push(node).is_err() {
                    dev_err!(
                        &a.dev,
                        "HeapAllocation[{}]::drop: Failed to keep garbage\n",
                        &*a.name,
                    );
                }
                a.total_garbage += size as usize;
                None
            } else {
                // We need to ensure node survives this scope, since dropping it
                // will try to take the mm lock and deadlock us
                Some(node)
            }
        });
    }
}

impl mm::AllocInner<HeapAllocationInner> for HeapAllocatorInner {
    fn drop_object(
        &mut self,
        start: u64,
        _size: u64,
        _color: usize,
        obj: &mut HeapAllocationInner,
    ) {
        /* real_size == 0 means it's a guard node */
        if obj.real_size > 0 {
            mod_dev_dbg!(
                obj.dev,
                "HeapAllocator[{}]: drop object @ {:#x} ({} bytes)\n",
                &*self.name,
                start,
                obj.real_size,
            );
            self.allocated -= obj.real_size;
        }
    }
}

impl RawAllocation for HeapAllocation {
    // SAFETY: This function must always return a valid pointer.
    // Since the HeapAllocation contains a reference to the
    // backing_objects array that contains the object backing this pointer,
    // and objects are only ever added to it, this pointer is guaranteed to
    // remain valid for the lifetime of the HeapAllocation.
    fn ptr(&self) -> Option<NonNull<u8>> {
        self.0.as_ref().unwrap().ptr
    }
    // SAFETY: This function must always return a valid GPU pointer.
    // See the explanation in ptr().
    fn gpu_ptr(&self) -> u64 {
        self.0.as_ref().unwrap().start()
    }
    fn size(&self) -> usize {
        self.0.as_ref().unwrap().size() as usize
    }
    fn device(&self) -> &AsahiDevice {
        &self.0.as_ref().unwrap().dev
    }
}

/// Inner data for a heap allocator which uses the DRM MM range allocator to manage the heap.
///
/// This is wrapped by an `mm::Allocator`.
struct HeapAllocatorInner {
    dev: AsahiDevRef,
    allocated: usize,
    backing_objects: Vec<(crate::gem::ObjectRef, u64)>,
    garbage: Option<Vec<mm::Node<HeapAllocatorInner, HeapAllocationInner>>>,
    total_garbage: usize,
    name: CString,
    vm_id: u64,
}

/// A heap allocator which uses the DRM MM range allocator to manage its objects.
///
/// The heap is composed of a series of GEM objects. This implementation only ever grows the heap,
/// never shrinks it.
pub(crate) struct HeapAllocator {
    dev: AsahiDevRef,
    start: u64,
    end: u64,
    top: u64,
    prot: u32,
    vm: mmu::Vm,
    min_align: usize,
    block_size: usize,
    cpu_maps: bool,
    guard_nodes: Vec<mm::Node<HeapAllocatorInner, HeapAllocationInner>>,
    mm: mm::Allocator<HeapAllocatorInner, HeapAllocationInner>,
    name: CString,
}

impl HeapAllocator {
    /// Create a new HeapAllocator for a given `Vm` and address range.
    #[allow(dead_code)]
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        dev: &AsahiDevice,
        vm: &mmu::Vm,
        start: u64,
        end: u64,
        min_align: usize,
        prot: u32,
        block_size: usize,
        mut cpu_maps: bool,
        name: fmt::Arguments<'_>,
        keep_garbage: bool,
    ) -> Result<HeapAllocator> {
        if !min_align.is_power_of_two() {
            return Err(EINVAL);
        }
        if debug_enabled(DebugFlags::ForceCPUMaps) {
            cpu_maps = true;
        }

        let name = CString::try_from_fmt(name)?;

        let inner = HeapAllocatorInner {
            dev: dev.into(),
            allocated: 0,
            backing_objects: Vec::new(),
            // TODO: This clearly needs a try_clone() or similar
            name: CString::try_from_fmt(fmt!("{}", &*name))?,
            vm_id: vm.id(),
            garbage: if keep_garbage { Some(Vec::new()) } else { None },
            total_garbage: 0,
        };

        let mm = mm::Allocator::new(start, end - start + 1, inner)?;

        Ok(HeapAllocator {
            dev: dev.into(),
            vm: vm.clone(),
            start,
            end,
            top: start,
            prot,
            min_align,
            block_size: block_size.max(min_align),
            cpu_maps,
            guard_nodes: Vec::new(),
            mm,
            name,
        })
    }

    /// Add a new backing block of the given size to this heap.
    ///
    /// If CPU mapping is enabled, this also adds a guard node to the range allocator to ensure that
    /// objects cannot straddle backing block boundaries, since we cannot easily create a contiguous
    /// CPU VA mapping for them. This can create some fragmentation. If CPU mapping is disabled, we
    /// skip the guard blocks, since the GPU view of the heap is always contiguous.
    fn add_block(&mut self, size: usize) -> Result {
        let size_aligned = (size + mmu::UAT_PGSZ - 1) & !mmu::UAT_PGMSK;

        mod_dev_dbg!(
            &self.dev,
            "HeapAllocator[{}]::add_block: size={:#x} size_al={:#x}\n",
            &*self.name,
            size,
            size_aligned,
        );

        if self.top.saturating_add(size_aligned as u64) >= self.end {
            dev_err!(
                &self.dev,
                "HeapAllocator[{}]::add_block: Exhausted VA space\n",
                &*self.name,
            );
        }

        let mut obj = crate::gem::new_kernel_object(&self.dev, size_aligned)?;
        if self.cpu_maps && debug_enabled(DebugFlags::FillAllocations) {
            obj.vmap()?.as_mut_slice().fill(0xde);
        }

        let gpu_ptr = self.top;
        if let Err(e) = obj.map_at(&self.vm, gpu_ptr, self.prot, self.cpu_maps) {
            dev_err!(
                &self.dev,
                "HeapAllocator[{}]::add_block: Failed to map at {:#x} ({:?})\n",
                &*self.name,
                gpu_ptr,
                e
            );
            return Err(e);
        }

        self.mm
            .with_inner(|inner| inner.backing_objects.try_reserve(1))?;

        let mut new_top = self.top + size_aligned as u64;
        if self.cpu_maps {
            let guard = self.min_align.max(mmu::UAT_PGSZ);
            mod_dev_dbg!(
                &self.dev,
                "HeapAllocator[{}]::add_block: Adding guard node {:#x}:{:#x}\n",
                &*self.name,
                new_top,
                guard
            );

            let inner = HeapAllocationInner {
                dev: self.dev.clone(),
                ptr: None,
                real_size: 0,
            };

            let node = match self.mm.reserve_node(inner, new_top, guard as u64, 0) {
                Ok(a) => a,
                Err(a) => {
                    dev_err!(
                        &self.dev,
                        "HeapAllocator[{}]::add_block: Failed to reserve guard node {:#x}:{:#x}: {:?}\n",
                        &*self.name,
                        guard,
                        new_top,
                        a
                    );
                    return Err(EIO);
                }
            };

            self.guard_nodes.try_push(node)?;

            new_top += guard as u64;
        }
        mod_dev_dbg!(
            &self.dev,
            "HeapAllocator[{}]::add_block: top={:#x}\n",
            &*self.name,
            new_top
        );

        self.mm
            .with_inner(|inner| inner.backing_objects.try_push((obj, gpu_ptr)))?;

        self.top = new_top;

        cls_dev_dbg!(
            MemStats,
            &self.dev,
            "{} Heap: grow to {} bytes\n",
            &*self.name,
            self.top - self.start
        );

        Ok(())
    }

    /// Find the backing object index that backs a given GPU address.
    fn find_obj(&mut self, addr: u64) -> Result<usize> {
        self.mm.with_inner(|inner| {
            inner
                .backing_objects
                .binary_search_by(|obj| {
                    let start = obj.1;
                    let end = obj.1 + obj.0.size() as u64;
                    if start > addr {
                        Ordering::Greater
                    } else if end <= addr {
                        Ordering::Less
                    } else {
                        Ordering::Equal
                    }
                })
                .or(Err(ENOENT))
        })
    }
}

impl Allocator for HeapAllocator {
    type Raw = HeapAllocation;

    fn device(&self) -> &AsahiDevice {
        &self.dev
    }

    fn cpu_maps(&self) -> bool {
        self.cpu_maps
    }

    fn min_align(&self) -> usize {
        self.min_align
    }

    fn alloc(&mut self, size: usize, align: usize) -> Result<HeapAllocation> {
        if align != 0 && !align.is_power_of_two() {
            return Err(EINVAL);
        }
        let align = self.min_align.max(align);
        let size_aligned = (size + align - 1) & !(align - 1);

        mod_dev_dbg!(
            &self.dev,
            "HeapAllocator[{}]::new: size={:#x} size_al={:#x}\n",
            &*self.name,
            size,
            size_aligned,
        );

        let inner = HeapAllocationInner {
            dev: self.dev.clone(),
            ptr: None,
            real_size: size,
        };

        let mut node = match self.mm.insert_node_generic(
            inner,
            size_aligned as u64,
            align as u64,
            0,
            mm::InsertMode::Best,
        ) {
            Ok(a) => a,
            Err(a) => {
                dev_err!(
                    &self.dev,
                    "HeapAllocator[{}]::new: Failed to insert node of size {:#x} / align {:#x}: {:?}\n",
                    &*self.name, size_aligned, align, a
                );
                return Err(a);
            }
        };

        self.mm.with_inner(|inner| inner.allocated += size);

        let mut new_object = false;
        let start = node.start();
        let end = start + node.size();
        if end > self.top {
            if start > self.top {
                dev_warn!(
                    self.dev,
                    "HeapAllocator[{}]::alloc: top={:#x}, start={:#x}\n",
                    &*self.name,
                    self.top,
                    start
                );
            }
            let block_size = self.block_size.max((end - self.top) as usize);
            self.add_block(block_size)?;
            new_object = true;
        }
        assert!(end <= self.top);

        if self.cpu_maps {
            mod_dev_dbg!(
                self.dev,
                "HeapAllocator[{}]::alloc: mapping to CPU\n",
                &*self.name
            );

            let idx = if new_object {
                None
            } else {
                Some(match self.find_obj(start) {
                    Ok(a) => a,
                    Err(_) => {
                        dev_warn!(
                            self.dev,
                            "HeapAllocator[{}]::alloc: Failed to find object at {:#x}\n",
                            &*self.name,
                            start
                        );
                        return Err(EIO);
                    }
                })
            };
            let (obj_start, obj_size, p) = self.mm.with_inner(|inner| -> Result<_> {
                let idx = idx.unwrap_or(inner.backing_objects.len() - 1);
                let obj = &mut inner.backing_objects[idx];
                let p = obj.0.vmap()?.as_mut_ptr() as *mut u8;
                Ok((obj.1, obj.0.size(), p))
            })?;
            assert!(obj_start <= start);
            assert!(obj_start + obj_size as u64 >= end);
            node.as_mut().inner_mut().ptr =
                NonNull::new(unsafe { p.add((start - obj_start) as usize) });
            mod_dev_dbg!(
                self.dev,
                "HeapAllocator[{}]::alloc: CPU pointer = {:?}\n",
                &*self.name,
                node.ptr
            );
        }

        mod_dev_dbg!(
            self.dev,
            "HeapAllocator[{}]::alloc: Allocated {:#x} bytes @ {:#x}\n",
            &*self.name,
            end - start,
            start
        );

        Ok(HeapAllocation(Some(node)))
    }

    fn garbage(&self) -> (usize, usize) {
        self.mm.with_inner(|inner| {
            if let Some(g) = inner.garbage.as_ref() {
                (g.len(), inner.total_garbage)
            } else {
                (0, 0)
            }
        })
    }

    fn collect_garbage(&mut self, count: usize) {
        // Take the garbage out of the inner block, so we can safely drop it without deadlocking
        let mut garbage = Vec::new();

        if garbage.try_reserve(count).is_err() {
            dev_crit!(
                self.dev,
                "HeapAllocator[{}]:collect_garbage: failed to reserve space\n",
                &*self.name,
            );
            return;
        }

        self.mm.with_inner(|inner| {
            if let Some(g) = inner.garbage.as_mut() {
                for node in g.drain(0..count) {
                    inner.total_garbage -= node.size() as usize;
                    garbage
                        .try_push(node)
                        .expect("try_push() failed after reserve()");
                }
            }
        });
    }
}

impl Drop for HeapAllocatorInner {
    fn drop(&mut self) {
        mod_dev_dbg!(
            self.dev,
            "HeapAllocator[{}]: dropping allocator\n",
            &*self.name
        );
        if self.allocated > 0 {
            // This should never happen
            dev_crit!(
                self.dev,
                "HeapAllocator[{}]: dropping with {} bytes allocated\n",
                &*self.name,
                self.allocated
            );
        } else {
            for mut obj in self.backing_objects.drain(..) {
                obj.0.drop_vm_mappings(self.vm_id);
            }
        }
    }
}
