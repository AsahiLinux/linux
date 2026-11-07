// SPDX-License-Identifier: GPL-2.0

//! DMA buffer API
//!
//! C header: [`include/linux/dma-buf.h`](srctree/include/linux/dma-buf.h)

use bindings;
use kernel::types::*;

/// A DMA buffer object.
///
/// # Invariants
///
/// The data layout of this type is equivalent to that of `struct dma_buf`.
#[repr(transparent)]
pub struct DmaBuf(Opaque<bindings::dma_buf>);

// SAFETY: `struct dma_buf` is thread-safe
unsafe impl Send for DmaBuf {}
// SAFETY: `struct dma_buf` is thread-safe
unsafe impl Sync for DmaBuf {}

impl DmaBuf {
    /// Convert from a `*mut bindings::dma_buf` to a [`DmaBuf`].
    ///
    /// # Safety
    ///
    /// The caller guarantees that `self_ptr` points to a valid initialized `struct dma_buf` for the
    /// duration of the lifetime of `'a`, and promises to not violate rust's data aliasing rules
    /// using the reference provided by this function.
    pub(crate) unsafe fn as_ref<'a>(self_ptr: *mut bindings::dma_buf) -> &'a Self {
        // SAFETY: Our data layout is equivalent to `dma_buf` .
        unsafe { &*self_ptr.cast() }
    }

    pub(crate) fn as_raw(&self) -> *mut bindings::dma_buf {
        self.0.get()
    }
}
