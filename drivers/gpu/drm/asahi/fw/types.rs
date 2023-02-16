// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Common types for firmware structure definitions

use crate::{alloc, object};
use core::fmt;
use core::ops::{Deref, DerefMut, Index, IndexMut};

pub(crate) use crate::event::EventValue;
pub(crate) use crate::object::{GpuPointer, GpuStruct, GpuWeakPointer};
pub(crate) use crate::{f32, float::F32};

pub(crate) use core::fmt::Debug;
pub(crate) use core::marker::PhantomData;
pub(crate) use core::sync::atomic::{AtomicI32, AtomicU32, AtomicU64};
pub(crate) use kernel::init::Zeroable;
pub(crate) use kernel::macros::versions;

// Make the trait visible
pub(crate) use crate::alloc::Allocator as _Allocator;

/// General allocator type used for the driver
pub(crate) type Allocator = alloc::DefaultAllocator;

/// General GpuObject type used for the driver
pub(crate) type GpuObject<T> =
    object::GpuObject<T, alloc::GenericAlloc<T, alloc::DefaultAllocation>>;

/// General GpuArray type used for the driver
pub(crate) type GpuArray<T> = object::GpuArray<T, alloc::GenericAlloc<T, alloc::DefaultAllocation>>;

/// General GpuOnlyArray type used for the driver
pub(crate) type GpuOnlyArray<T> =
    object::GpuOnlyArray<T, alloc::GenericAlloc<T, alloc::DefaultAllocation>>;

/// A stamp slot that is shared between firmware and the driver.
#[derive(Debug, Default)]
#[repr(transparent)]
pub(crate) struct Stamp(pub(crate) AtomicU32);

/// A stamp slot that is for private firmware use.
///
/// This is a separate type to guard against pointer type confusion.
#[derive(Debug, Default)]
#[repr(transparent)]
pub(crate) struct FwStamp(pub(crate) AtomicU32);

/// An unaligned u64 type.
///
/// This is useful to avoid having to pack firmware structures entirely, since that is incompatible
/// with `#[derive(Debug)]` and atomics.
#[derive(Copy, Clone, Default)]
#[repr(C, packed(1))]
pub(crate) struct U64(pub(crate) u64);

unsafe impl Zeroable for U64 {}

impl fmt::Debug for U64 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let v = self.0;
        f.write_fmt(format_args!("{:#x}", v))
    }
}

/// An unaligned u32 type.
///
/// This is useful to avoid having to pack firmware structures entirely, since that is incompatible
/// with `#[derive(Debug)]` and atomics.
#[derive(Copy, Clone, Default)]
#[repr(C, packed(1))]
pub(crate) struct U32(pub(crate) u32);

unsafe impl Zeroable for U32 {}

impl fmt::Debug for U32 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let v = self.0;
        f.write_fmt(format_args!("{:#x}", v))
    }
}

/// Create a dummy `Debug` implementation, for when we need it but it's too painful to write by
/// hand or not very useful.
#[macro_export]
macro_rules! no_debug {
    ($type:ty) => {
        impl ::core::fmt::Debug for $type {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(f, "...")
            }
        }
    };
}

/// Implement Zeroable for a given type (and Default along with it).
///
/// # Safety
///
/// This macro must only be used if a type only contains primitive types which can be
/// zero-initialized, FFI structs intended to be zero-initialized, or other types which
/// impl Zeroable.
#[macro_export]
macro_rules! default_zeroed {
    (<$($lt:lifetime),*>, $type:ty) => {
        impl<$($lt),*> Default for $type {
            fn default() -> $type {
                ::kernel::init::Zeroable::zeroed()
            }
        }
        // SAFETY: The user is responsible for ensuring this is safe.
        unsafe impl<$($lt),*> ::kernel::init::Zeroable for $type {}
    };
    ($type:ty) => {
        impl Default for $type {
            fn default() -> $type {
                ::kernel::init::Zeroable::zeroed()
            }
        }
        // SAFETY: The user is responsible for ensuring this is safe.
        unsafe impl ::kernel::init::Zeroable for $type {}
    };
}

/// A convenience type for a number of padding bytes. Hidden from Debug formatting.
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub(crate) struct Pad<const N: usize>([u8; N]);

/// SAFETY: Primitive type, safe to zero-init.
unsafe impl<const N: usize> Zeroable for Pad<N> {}

impl<const N: usize> Default for Pad<N> {
    fn default() -> Self {
        Zeroable::zeroed()
    }
}

impl<const N: usize> fmt::Debug for Pad<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_fmt(format_args!("<pad>"))
    }
}

/// A convenience type for a fixed-sized array with Default/Zeroable impls.
#[derive(Copy, Clone)]
#[repr(C)]
pub(crate) struct Array<const N: usize, T>([T; N]);

impl<const N: usize, T> Array<N, T> {
    pub(crate) fn new(data: [T; N]) -> Self {
        Self(data)
    }
}

// SAFETY: Arrays of Zeroable values can be safely Zeroable.
unsafe impl<const N: usize, T: Zeroable> Zeroable for Array<N, T> {}

impl<const N: usize, T: Zeroable> Default for Array<N, T> {
    fn default() -> Self {
        Zeroable::zeroed()
    }
}

impl<const N: usize, T> Index<usize> for Array<N, T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl<const N: usize, T> IndexMut<usize> for Array<N, T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

impl<const N: usize, T> Deref for Array<N, T> {
    type Target = [T; N];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize, T> DerefMut for Array<N, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const N: usize, T: Sized + fmt::Debug> fmt::Debug for Array<N, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

/// Convenience macro to define an identically-named trivial GpuStruct with no inner fields for a
/// given raw type name.
#[macro_export]
macro_rules! trivial_gpustruct {
    ($type:ident) => {
        #[derive(Debug)]
        pub(crate) struct $type {}

        impl GpuStruct for $type {
            type Raw<'a> = raw::$type;
        }
        $crate::default_zeroed!($type);
    };
}
