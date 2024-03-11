// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Asahi GPU object model
//!
//! The AGX GPU includes a coprocessor that uses a large number of shared memory structures to
//! communicate with the driver. These structures contain GPU VA pointers to each other, which are
//! directly dereferenced by the firmware and are expected to always be valid for the usage
//! lifetime of the containing struct (which is an implicit contract, not explicitly managed).
//! Any faults cause an unrecoverable firmware crash, requiring a full system reboot.
//!
//! In order to manage this complexity safely, we implement a GPU object model using Rust's type
//! system to enforce GPU object lifetime relationships. GPU objects represent an allocated piece
//! of memory of a given type, mapped to the GPU (and usually also the CPU). On the CPU side,
//! these objects are associated with a pure Rust structure that contains the objects it depends
//! on (or references to them). This allows us to map Rust lifetimes into the GPU object model
//! system. Then, GPU VA pointers also inherit those lifetimes, which means the Rust borrow checker
//! can ensure that all pointers are assigned an address that is guaranteed to outlive the GPU
//! object it points to.
//!
//! Since the firmware object model does have self-referencing pointers (and there is of course no
//! underlying revocability mechanism to make it safe), we must have an escape hatch. GPU pointers
//! can be weak pointers, which do not enforce lifetimes. In those cases, it is the user's
//! responsibility to ensure that lifetime requirements are met.
//!
//! In other words, the model is necessarily leaky and there is no way to fully map Rust safety to
//! GPU firmware object safety. The goal of the model is to make it easy to model the lifetimes of
//! GPU objects and have the compiler help in avoiding mistakes, rather than to guarantee safety
//! 100% of the time as would be the case for CPU-side Rust code.

// TODO: There is a fundamental soundness issue with sharing memory with the GPU (that even affects
// C code too). Since the GPU is free to mutate that memory at any time, normal reference invariants
// cannot be enforced on the CPU side. For example, the compiler could perform an optimization that
// assumes that a given memory location does not change between two reads, and causes UB otherwise,
// and then the GPU could mutate that memory out from under the CPU.
//
// For cases where we *expect* this to happen, we use atomic types, which avoid this issue. However,
// doing so for every single field of every type is a non-starter. Right now, there seems to be no
// good solution for this that does not come with significant performance or ergonomics downsides.
//
// In *practice* we are almost always only writing GPU memory, and only reading from atomics, so the
// chances of this actually triggering UB (e.g. a security issue that can be triggered from the GPU
// side) due to a compiler optimization are very slim.
//
// Further discussion: https://github.com/rust-lang/unsafe-code-guidelines/issues/152

use kernel::{error::code::*, prelude::*};

use alloc::boxed::Box;
use core::fmt;
use core::fmt::Debug;
use core::fmt::Formatter;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::num::NonZeroU64;
use core::ops::{Deref, DerefMut, Index, IndexMut};
use core::{mem, ptr, slice};

use crate::alloc::Allocation;
use crate::debug::*;
use crate::fw::types::Zeroable;

const DEBUG_CLASS: DebugFlags = DebugFlags::Object;

/// A GPU-side strong pointer, which is a 64-bit non-zero VA with an associated lifetime.
///
/// In rare cases these pointers are not aligned, so this is `packed(1)`.
#[repr(C, packed(1))]
pub(crate) struct GpuPointer<'a, T: ?Sized>(NonZeroU64, PhantomData<&'a T>);

impl<'a, T: ?Sized> GpuPointer<'a, T> {
    /// Logical OR the pointer with an arbitrary `u64`. This is used when GPU struct fields contain
    /// misc flag fields in the upper bits. The lifetime is retained. This is GPU-unsafe in
    /// principle, but we assert that only non-implemented address bits are touched, which is safe
    /// for pointers used by the GPU (not by firmware).
    pub(crate) fn or(&self, other: u64) -> GpuPointer<'a, T> {
        // This will fail for kernel-half pointers, which should not be ORed.
        assert_eq!(self.0.get() & other, 0);
        // Assert that we only touch the high bits.
        assert_eq!(other & 0xffffffffff, 0);
        GpuPointer(self.0 | other, PhantomData)
    }

    /// Add an arbitrary offset to the pointer. This is not safe (from the GPU perspective), and
    /// should only be used via the `inner_ptr` macro to get pointers to inner fields, hence we mark
    /// it `unsafe` to discourage direct use.
    // NOTE: The third argument is a type inference hack.
    pub(crate) unsafe fn offset<U>(&self, off: usize, _: *const U) -> GpuPointer<'a, U> {
        GpuPointer::<'a, U>(
            NonZeroU64::new(self.0.get() + (off as u64)).unwrap(),
            PhantomData,
        )
    }
}

impl<'a, T: ?Sized> Debug for GpuPointer<'a, T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let val = self.0;
        f.write_fmt(format_args!("{:#x} ({})", val, core::any::type_name::<T>()))
    }
}

impl<'a, T: ?Sized> From<GpuPointer<'a, T>> for u64 {
    fn from(value: GpuPointer<'a, T>) -> Self {
        value.0.get()
    }
}

/// Take a pointer to a sub-field within a structure pointed to by a GpuPointer, keeping the
/// lifetime.
#[macro_export]
macro_rules! inner_ptr {
    ($gpuva:expr, $($f:tt)*) => ({
        // This mirrors kernel::offset_of(), except we use type inference to avoid having to know
        // the type of the pointer explicitly.
        fn uninit_from<T: GpuStruct>(_: GpuPointer<'_, T>) -> core::mem::MaybeUninit<T::Raw<'static>> {
            core::mem::MaybeUninit::uninit()
        }
        let tmp = uninit_from($gpuva);
        let outer = tmp.as_ptr();
        // SAFETY: The pointer is valid and aligned, just not initialised; `addr_of` ensures that
        // we don't actually read from `outer` (which would be UB) nor create an intermediate
        // reference.
        let p: *const _ = unsafe { core::ptr::addr_of!((*outer).$($f)*) };
        let inner = p as *const u8;
        // SAFETY: The two pointers are within the same allocation block.
        let off = unsafe { inner.offset_from(outer as *const u8) };
        // SAFETY: The resulting pointer is guaranteed to point to valid memory within the outer
        // object.
        unsafe { $gpuva.offset(off.try_into().unwrap(), p) }
    })
}

/// A GPU-side weak pointer, which is a 64-bit non-zero VA with no lifetime.
///
/// In rare cases these pointers are not aligned, so this is `packed(1)`.
#[repr(C, packed(1))]
pub(crate) struct GpuWeakPointer<T: ?Sized>(NonZeroU64, PhantomData<*const T>);

/// SAFETY: GPU weak pointers are always safe to share between threads.
unsafe impl<T: ?Sized> Send for GpuWeakPointer<T> {}
unsafe impl<T: ?Sized> Sync for GpuWeakPointer<T> {}

// Weak pointers can be copied/cloned regardless of their target type.
impl<T: ?Sized> Copy for GpuWeakPointer<T> {}

impl<T: ?Sized> Clone for GpuWeakPointer<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T: ?Sized> GpuWeakPointer<T> {
    /// Add an arbitrary offset to the pointer. This is not safe (from the GPU perspective), and
    /// should only be used via the `inner_ptr` macro to get pointers to inner fields, hence we mark
    /// it `unsafe` to discourage direct use.
    // NOTE: The third argument is a type inference hack.
    pub(crate) unsafe fn offset<U>(&self, off: usize, _: *const U) -> GpuWeakPointer<U> {
        GpuWeakPointer::<U>(
            NonZeroU64::new(self.0.get() + (off as u64)).unwrap(),
            PhantomData,
        )
    }

    /// Upgrade a weak pointer into a strong pointer. This is not considered safe from the GPU
    /// perspective.
    pub(crate) unsafe fn upgrade<'a>(&self) -> GpuPointer<'a, T> {
        GpuPointer(self.0, PhantomData)
    }
}

impl<T: ?Sized> From<GpuWeakPointer<T>> for u64 {
    fn from(value: GpuWeakPointer<T>) -> Self {
        value.0.get()
    }
}

impl<T: ?Sized> Debug for GpuWeakPointer<T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let val = self.0;
        f.write_fmt(format_args!("{:#x} ({})", val, core::any::type_name::<T>()))
    }
}

/// Take a pointer to a sub-field within a structure pointed to by a GpuWeakPointer.
#[macro_export]
macro_rules! inner_weak_ptr {
    ($gpuva:expr, $($f:tt)*) => ({
        // See inner_ptr()
        fn uninit_from<T: GpuStruct>(_: GpuWeakPointer<T>) -> core::mem::MaybeUninit<T::Raw<'static>> {
            core::mem::MaybeUninit::uninit()
        }
        let tmp = uninit_from($gpuva);
        let outer = tmp.as_ptr();
        // SAFETY: The pointer is valid and aligned, just not initialised; `addr_of` ensures that
        // we don't actually read from `outer` (which would be UB) nor create an intermediate
        // reference.
        let p: *const _ = unsafe { core::ptr::addr_of!((*outer).$($f)*) };
        let inner = p as *const u8;
        // SAFETY: The two pointers are within the same allocation block.
        let off = unsafe { inner.offset_from(outer as *const u8) };
        // SAFETY: The resulting pointer is guaranteed to point to valid memory within the outer
        // object.
        unsafe { $gpuva.offset(off.try_into().unwrap(), p) }
    })
}

/// Types that implement this trait represent a GPU structure from the CPU side.
///
/// The `Raw` type represents the actual raw structure definition on the GPU side.
///
/// Types implementing [`GpuStruct`] must have fields owning any objects (or strong references
/// to them) that GPU pointers in the `Raw` structure point to. This mechanism is used to enforce
/// lifetimes.
pub(crate) trait GpuStruct: 'static {
    /// The type of the GPU-side structure definition representing the firmware struct layout.
    type Raw<'a>;
}

/// An instance of a GPU object in memory.
///
/// # Invariants
/// `raw` must point to a valid mapping of the `T::Raw` type associated with the `alloc` allocation.
/// `gpu_ptr` must be the GPU address of the same object.
pub(crate) struct GpuObject<T: GpuStruct, U: Allocation<T>> {
    raw: *mut T::Raw<'static>,
    alloc: U,
    gpu_ptr: GpuWeakPointer<T>,
    inner: Box<T>,
}

impl<T: GpuStruct, U: Allocation<T>> GpuObject<T, U> {
    /// Create a new GpuObject given an allocator and the inner data (a type implementing
    /// GpuStruct).
    ///
    /// The caller passes a closure that constructs the `T::Raw` type given a reference to the
    /// `GpuStruct`. This is the mechanism used to enforce lifetimes.
    pub(crate) fn new(
        alloc: U,
        inner: T,
        callback: impl for<'a> FnOnce(&'a T) -> T::Raw<'a>,
    ) -> Result<Self> {
        let size = mem::size_of::<T::Raw<'static>>();
        if size > 0x1000 {
            dev_crit!(
                alloc.device(),
                "Allocating {} of size {:#x}, with new, please use new_boxed!\n",
                core::any::type_name::<T>(),
                size
            );
        }
        if alloc.size() < size {
            return Err(ENOMEM);
        }
        let gpu_ptr =
            GpuWeakPointer::<T>(NonZeroU64::new(alloc.gpu_ptr()).ok_or(EINVAL)?, PhantomData);
        mod_dev_dbg!(
            alloc.device(),
            "Allocating {} @ {:#x}\n",
            core::any::type_name::<T>(),
            alloc.gpu_ptr()
        );
        let p = alloc.ptr().ok_or(EINVAL)?.as_ptr() as *mut T::Raw<'static>;
        let mut raw = callback(&inner);
        // SAFETY: `p` is guaranteed to be valid per the Allocation invariant, and the type is
        // identical to the type of `raw` other than the lifetime.
        unsafe { p.copy_from(&mut raw as *mut _ as *mut u8 as *mut _, 1) };
        mem::forget(raw);
        Ok(Self {
            raw: p,
            gpu_ptr,
            alloc,
            inner: Box::try_new(inner)?,
        })
    }

    /// Create a new GpuObject given an allocator and the boxed inner data (a type implementing
    /// GpuStruct).
    ///
    /// The caller passes a closure that initializes the `T::Raw` type given a reference to the
    /// `GpuStruct` and a `MaybeUninit<T::Raw>`. This is intended to be used with the place!()
    /// macro to avoid constructing the whole `T::Raw` object on the stack.
    pub(crate) fn new_boxed(
        alloc: U,
        inner: Box<T>,
        callback: impl for<'a> FnOnce(
            &'a T,
            &'a mut MaybeUninit<T::Raw<'a>>,
        ) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<Self> {
        if alloc.size() < mem::size_of::<T::Raw<'static>>() {
            return Err(ENOMEM);
        }
        let gpu_ptr =
            GpuWeakPointer::<T>(NonZeroU64::new(alloc.gpu_ptr()).ok_or(EINVAL)?, PhantomData);
        mod_dev_dbg!(
            alloc.device(),
            "Allocating {} @ {:#x}\n",
            core::any::type_name::<T>(),
            alloc.gpu_ptr()
        );
        let p = alloc.ptr().ok_or(EINVAL)?.as_ptr() as *mut MaybeUninit<T::Raw<'_>>;
        // SAFETY: `p` is guaranteed to be valid per the Allocation invariant.
        let raw = callback(&inner, unsafe { &mut *p })?;
        if p as *mut T::Raw<'_> != raw as *mut _ {
            dev_err!(
                alloc.device(),
                "Allocation callback returned a mismatched reference ({})\n",
                core::any::type_name::<T>(),
            );
            return Err(EINVAL);
        }
        Ok(Self {
            raw: p as *mut u8 as *mut T::Raw<'static>,
            gpu_ptr,
            alloc,
            inner,
        })
    }

    /// Create a new GpuObject given an allocator and the inner data (a type implementing
    /// GpuStruct).
    ///
    /// The caller passes a closure that initializes the `T::Raw` type given a reference to the
    /// `GpuStruct` and a `MaybeUninit<T::Raw>`. This is intended to be used with the place!()
    /// macro to avoid constructing the whole `T::Raw` object on the stack.
    pub(crate) fn new_inplace(
        alloc: U,
        inner: T,
        callback: impl for<'a> FnOnce(
            &'a T,
            &'a mut MaybeUninit<T::Raw<'a>>,
        ) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<Self> {
        GpuObject::<T, U>::new_boxed(alloc, Box::try_new(inner)?, callback)
    }

    /// Create a new GpuObject given an allocator and the boxed inner data (a type implementing
    /// GpuStruct).
    ///
    /// The caller passes a closure that initializes the `T::Raw` type given a reference to the
    /// `GpuStruct` and a `MaybeUninit<T::Raw>`. This is intended to be used with the place!()
    /// macro to avoid constructing the whole `T::Raw` object on the stack.
    pub(crate) fn new_init_prealloc<'a, I: Init<T, E>, R: PinInit<T::Raw<'a>, F>, E, F>(
        alloc: U,
        inner_init: impl FnOnce(GpuWeakPointer<T>) -> I,
        raw_init: impl FnOnce(&'a T, GpuWeakPointer<T>) -> R,
    ) -> Result<Self>
    where
        kernel::error::Error: core::convert::From<E>,
        kernel::error::Error: core::convert::From<F>,
    {
        if alloc.size() < mem::size_of::<T::Raw<'static>>() {
            return Err(ENOMEM);
        }
        let gpu_ptr =
            GpuWeakPointer::<T>(NonZeroU64::new(alloc.gpu_ptr()).ok_or(EINVAL)?, PhantomData);
        mod_dev_dbg!(
            alloc.device(),
            "Allocating {} @ {:#x}\n",
            core::any::type_name::<T>(),
            alloc.gpu_ptr()
        );
        let inner = inner_init(gpu_ptr);
        let p = alloc.ptr().ok_or(EINVAL)?.as_ptr() as *mut T::Raw<'_>;
        let ret = Self {
            raw: p as *mut u8 as *mut T::Raw<'static>,
            gpu_ptr,
            alloc,
            inner: Box::init(inner)?,
        };
        let q = &*ret.inner as *const T;
        // SAFETY: `p` is guaranteed to be valid per the Allocation invariant.
        unsafe { raw_init(&*q, gpu_ptr).__pinned_init(p) }?;
        Ok(ret)
    }

    /// Returns the GPU VA of this object (as a raw [`NonZeroU64`])
    pub(crate) fn gpu_va(&self) -> NonZeroU64 {
        self.gpu_ptr.0
    }

    /// Returns a strong GPU pointer to this object, with a lifetime.
    pub(crate) fn gpu_pointer(&self) -> GpuPointer<'_, T> {
        GpuPointer(self.gpu_ptr.0, PhantomData)
    }

    /// Returns a weak GPU pointer to this object, with no lifetime.
    pub(crate) fn weak_pointer(&self) -> GpuWeakPointer<T> {
        GpuWeakPointer(self.gpu_ptr.0, PhantomData)
    }

    /// Perform a mutation to the inner `Raw` data given a user-supplied callback.
    ///
    /// The callback gets a mutable reference to the `GpuStruct` type.
    pub(crate) fn with_mut<RetVal>(
        &mut self,
        callback: impl for<'a> FnOnce(&'a mut <T as GpuStruct>::Raw<'a>, &'a mut T) -> RetVal,
    ) -> RetVal {
        // SAFETY: `self.raw` is valid per the type invariant, and the second half is just
        // converting lifetimes.
        unsafe { callback(&mut *self.raw, &mut *(&mut *self.inner as *mut _)) }
    }

    /// Access the inner `Raw` data given a user-supplied callback.
    ///
    /// The callback gets a reference to the `GpuStruct` type.
    pub(crate) fn with<RetVal>(
        &self,
        callback: impl for<'a> FnOnce(&'a <T as GpuStruct>::Raw<'a>, &'a T) -> RetVal,
    ) -> RetVal {
        // SAFETY: `self.raw` is valid per the type invariant, and the second half is just
        // converting lifetimes.
        unsafe { callback(&*self.raw, &*(&*self.inner as *const _)) }
    }
}

impl<T: GpuStruct, U: Allocation<T>> Deref for GpuObject<T, U> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T: GpuStruct, U: Allocation<T>> DerefMut for GpuObject<T, U> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<T: GpuStruct + Debug, U: Allocation<T>> Debug for GpuObject<T, U>
where
    <T as GpuStruct>::Raw<'static>: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct(core::any::type_name::<T>())
            // SAFETY: `self.raw` is valid per the type invariant.
            .field("raw", &format_args!("{:#X?}", unsafe { &*self.raw }))
            .field("inner", &format_args!("{:#X?}", &self.inner))
            .field("alloc", &format_args!("{:?}", &self.alloc))
            .finish()
    }
}

impl<T: GpuStruct + Default, U: Allocation<T>> GpuObject<T, U>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Default + Zeroable,
{
    /// Create a new GpuObject with default data. `T` must implement `Default` and `T::Raw` must
    /// implement `Zeroable`, since the GPU-side memory is initialized by zeroing.
    pub(crate) fn new_default(alloc: U) -> Result<Self> {
        GpuObject::<T, U>::new_inplace(alloc, Default::default(), |_inner, raw| {
            // SAFETY: `raw` is valid here, and `T::Raw` implements `Zeroable`.
            Ok(unsafe {
                ptr::write_bytes(raw, 0, 1);
                (*raw).assume_init_mut()
            })
        })
    }
}

impl<T: GpuStruct, U: Allocation<T>> Drop for GpuObject<T, U> {
    fn drop(&mut self) {
        mod_dev_dbg!(
            self.alloc.device(),
            "Dropping {} @ {:?}\n",
            core::any::type_name::<T>(),
            self.gpu_pointer()
        );
    }
}

// SAFETY: GpuObjects are Send as long as the GpuStruct itself is Send
unsafe impl<T: GpuStruct + Send, U: Allocation<T>> Send for GpuObject<T, U> {}
// SAFETY: GpuObjects are Send as long as the GpuStruct itself is Send
unsafe impl<T: GpuStruct + Sync, U: Allocation<T>> Sync for GpuObject<T, U> {}

/// Trait used to erase the type of a GpuObject, used when we need to keep a list of heterogenous
/// objects around.
pub(crate) trait OpaqueGpuObject: Send + Sync {
    fn gpu_va(&self) -> NonZeroU64;
}

impl<T: GpuStruct + Sync + Send, U: Allocation<T>> OpaqueGpuObject for GpuObject<T, U> {
    fn gpu_va(&self) -> NonZeroU64 {
        Self::gpu_va(self)
    }
}

/// An array of raw GPU objects that is only accessible to the GPU (no CPU-side mapping required).
///
/// This must necessarily be uninitialized as far as the GPU is concerned, so it cannot be used
/// when initialization is required.
///
/// # Invariants
///
/// `alloc` is valid and at least as large as `len` times the size of one `T`.
/// `gpu_ptr` is valid and points to the allocation start.
pub(crate) struct GpuOnlyArray<T, U: Allocation<T>> {
    len: usize,
    alloc: U,
    gpu_ptr: NonZeroU64,
    _p: PhantomData<T>,
}

impl<T, U: Allocation<T>> GpuOnlyArray<T, U> {
    /// Allocate a new GPU-only array with the given length.
    pub(crate) fn new(alloc: U, count: usize) -> Result<GpuOnlyArray<T, U>> {
        let bytes = count * mem::size_of::<T>();
        let gpu_ptr = NonZeroU64::new(alloc.gpu_ptr()).ok_or(EINVAL)?;
        if alloc.size() < bytes {
            return Err(ENOMEM);
        }
        Ok(Self {
            len: count,
            alloc,
            gpu_ptr,
            _p: PhantomData,
        })
    }

    /// Returns the GPU VA of this arraw (as a raw [`NonZeroU64`])
    pub(crate) fn gpu_va(&self) -> NonZeroU64 {
        self.gpu_ptr
    }

    /// Returns a strong GPU pointer to this array, with a lifetime.
    pub(crate) fn gpu_pointer(&self) -> GpuPointer<'_, &'_ [T]> {
        GpuPointer(self.gpu_ptr, PhantomData)
    }

    /// Returns a weak GPU pointer to this array, with no lifetime.
    pub(crate) fn weak_pointer(&self) -> GpuWeakPointer<[T]> {
        GpuWeakPointer(self.gpu_ptr, PhantomData)
    }

    /// Returns a pointer to an offset within the array (as a subslice).
    pub(crate) fn gpu_offset_pointer(&self, offset: usize) -> GpuPointer<'_, &'_ [T]> {
        if offset > self.len {
            panic!("Index {} out of bounds (len: {})", offset, self.len);
        }
        GpuPointer(
            NonZeroU64::new(self.gpu_ptr.get() + (offset * mem::size_of::<T>()) as u64).unwrap(),
            PhantomData,
        )
    }

    /* Not used yet
    /// Returns a weak pointer to an offset within the array (as a subslice).
    pub(crate) fn weak_offset_pointer(&self, offset: usize) -> GpuWeakPointer<[T]> {
        if offset > self.len {
            panic!("Index {} out of bounds (len: {})", offset, self.len);
        }
        GpuWeakPointer(
            NonZeroU64::new(self.gpu_ptr.get() + (offset * mem::size_of::<T>()) as u64).unwrap(),
            PhantomData,
        )
    }

    /// Returns a pointer to an element within the array.
    pub(crate) fn gpu_item_pointer(&self, index: usize) -> GpuPointer<'_, &'_ T> {
        if index >= self.len {
            panic!("Index {} out of bounds (len: {})", index, self.len);
        }
        GpuPointer(
            NonZeroU64::new(self.gpu_ptr.get() + (index * mem::size_of::<T>()) as u64).unwrap(),
            PhantomData,
        )
    }
    */

    /// Returns a weak pointer to an element within the array.
    pub(crate) fn weak_item_pointer(&self, index: usize) -> GpuWeakPointer<T> {
        if index >= self.len {
            panic!("Index {} out of bounds (len: {})", index, self.len);
        }
        GpuWeakPointer(
            NonZeroU64::new(self.gpu_ptr.get() + (index * mem::size_of::<T>()) as u64).unwrap(),
            PhantomData,
        )
    }

    /// Returns the length of the array.
    pub(crate) fn len(&self) -> usize {
        self.len
    }
}

impl<T: Debug, U: Allocation<T>> Debug for GpuOnlyArray<T, U> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct(core::any::type_name::<T>())
            .field("len", &format_args!("{:#X?}", self.len()))
            .finish()
    }
}

impl<T, U: Allocation<T>> Drop for GpuOnlyArray<T, U> {
    fn drop(&mut self) {
        mod_dev_dbg!(
            self.alloc.device(),
            "Dropping {} @ {:?}\n",
            core::any::type_name::<T>(),
            self.gpu_pointer()
        );
    }
}

/// An array of raw GPU objects that is also CPU-accessible.
///
/// # Invariants
///
/// `raw` is valid and points to the CPU-side view of the array (which must have one).
pub(crate) struct GpuArray<T, U: Allocation<T>> {
    raw: *mut T,
    array: GpuOnlyArray<T, U>,
}

/* Not used yet
impl<T: Copy, U: Allocation<T>> GpuArray<T, U> {
    /// Allocate a new GPU array, copying the contents from a slice.
    pub(crate) fn new(alloc: U, data: &[T]) -> Result<GpuArray<T, U>> {
        let p = alloc.ptr().ok_or(EINVAL)?.as_ptr();
        let inner = GpuOnlyArray::new(alloc, data.len())?;
        // SAFETY: `p` is valid per the Allocation type invariant, and GpuOnlyArray guarantees
        // that its size is at least as large as `data.len()`.
        unsafe { ptr::copy(data.as_ptr(), p, data.len()) };
        Ok(Self {
            raw: p,
            array: inner,
        })
    }
}
*/

impl<T: Default, U: Allocation<T>> GpuArray<T, U> {
    /// Allocate a new GPU array, initializing each element to its default.
    pub(crate) fn empty(alloc: U, count: usize) -> Result<GpuArray<T, U>> {
        let p = alloc.ptr().ok_or(EINVAL)?.as_ptr();
        let inner = GpuOnlyArray::new(alloc, count)?;
        let mut pi = p;
        for _i in 0..count {
            // SAFETY: `pi` is valid per the Allocation type invariant, and GpuOnlyArray guarantees
            // that it can never iterate beyond the buffer length.
            unsafe {
                pi.write(Default::default());
                pi = pi.add(1);
            }
        }
        Ok(Self {
            raw: p,
            array: inner,
        })
    }
}

impl<T, U: Allocation<T>> GpuArray<T, U> {
    /// Get a slice view of the array contents.
    pub(crate) fn as_slice(&self) -> &[T] {
        // SAFETY: self.raw / self.len are valid per the type invariant
        unsafe { slice::from_raw_parts(self.raw, self.len) }
    }

    /// Get a mutable slice view of the array contents.
    pub(crate) fn as_mut_slice(&mut self) -> &mut [T] {
        // SAFETY: self.raw / self.len are valid per the type invariant
        unsafe { slice::from_raw_parts_mut(self.raw, self.len) }
    }
}

impl<T, U: Allocation<T>> Deref for GpuArray<T, U> {
    type Target = GpuOnlyArray<T, U>;

    fn deref(&self) -> &GpuOnlyArray<T, U> {
        &self.array
    }
}

impl<T, U: Allocation<T>> Index<usize> for GpuArray<T, U> {
    type Output = T;

    fn index(&self, index: usize) -> &T {
        if index >= self.len {
            panic!("Index {} out of bounds (len: {})", index, self.len);
        }
        // SAFETY: This is bounds checked above
        unsafe { &*(self.raw.add(index)) }
    }
}

impl<T, U: Allocation<T>> IndexMut<usize> for GpuArray<T, U> {
    fn index_mut(&mut self, index: usize) -> &mut T {
        if index >= self.len {
            panic!("Index {} out of bounds (len: {})", index, self.len);
        }
        // SAFETY: This is bounds checked above
        unsafe { &mut *(self.raw.add(index)) }
    }
}

// SAFETY: GpuArray are Send as long as the contained type itself is Send
unsafe impl<T: Send, U: Allocation<T>> Send for GpuArray<T, U> {}
// SAFETY: GpuArray are Sync as long as the contained type itself is Sync
unsafe impl<T: Sync, U: Allocation<T>> Sync for GpuArray<T, U> {}

impl<T: Debug, U: Allocation<T>> Debug for GpuArray<T, U> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct(core::any::type_name::<T>())
            .field("array", &format_args!("{:#X?}", self.as_slice()))
            .finish()
    }
}
