// SPDX-License-Identifier: GPL-2.0

//! Scatterlist
//!
//! C header: [`include/linux/scatterlist.h`](srctree/include/linux/scatterlist.h)

use crate::{
    bindings,
    device::{Bound, Device},
    dma::DmaDataDirection,
    error::{Error, Result},
    page::Page,
    types::{ARef, Opaque},
};

/// A single scatter-gather entry, representing a span of pages in the device's DMA address space.
///
/// This interface is accessible only via the `SGTable` iterators. When using the API safely, certain
/// methods are only available depending on a specific state of operation of the scatter-gather table,
/// i.e. setting page entries is done internally only during construction while retrieving the DMA address
/// is only possible when the `SGTable` is already mapped for DMA via a device.
///
/// # Invariants
///
/// The `scatterlist` pointer is valid for the lifetime of an SGEntry instance.
#[repr(transparent)]
pub struct SGEntry(Opaque<bindings::scatterlist>);

impl SGEntry {
    /// Convert a raw `struct scatterlist *` to a `&'a SGEntry`.
    ///
    /// This is meant as a helper for other kernel subsystems and not to be used by device drivers directly.
    ///
    /// # Safety
    ///
    /// Callers must ensure that the `struct scatterlist` pointed to by `ptr` is valid for the lifetime
    /// of the returned reference.
    pub(crate) unsafe fn as_ref<'a>(ptr: *mut bindings::scatterlist) -> &'a Self {
        // SAFETY: The pointer is valid and guaranteed by the safety requirements of the function.
        unsafe { &*ptr.cast() }
    }

    /// Convert a raw `struct scatterlist *` to a `&'a mut SGEntry`.
    ///
    /// This is meant as a helper for other kernel subsystems and not to be used by device drivers directly.
    ///
    /// # Safety
    ///
    /// See safety requirements of [`SGEntry::as_ref`]. In addition, callers must ensure that only
    /// a single mutable reference can be taken from the same raw pointer, i.e. for the lifetime of the
    /// returned reference, no other call to this function on the same `struct scatterlist *` should
    /// be permitted.
    pub(crate) unsafe fn as_mut<'a>(ptr: *mut bindings::scatterlist) -> &'a mut Self {
        // SAFETY: The pointer is valid and guaranteed by the safety requirements of the function.
        unsafe { &mut *ptr.cast() }
    }

    /// Obtain the raw `struct scatterlist *`.
    pub(crate) fn as_raw(&self) -> *mut bindings::scatterlist {
        self.0.get()
    }

    /// Returns the DMA address of this SG entry.
    pub fn dma_address(&self) -> bindings::dma_addr_t {
        // SAFETY: By the type invariant of `SGEntry`, ptr is valid.
        unsafe { bindings::sg_dma_address(self.0.get()) }
    }

    /// Returns the length of this SG entry.
    pub fn dma_len(&self) -> u32 {
        // SAFETY: By the type invariant of `SGEntry`, ptr is valid.
        unsafe { bindings::sg_dma_len(self.0.get()) }
    }

    /// Internal constructor helper to set this entry to point at a given page. Not to be used directly.
    fn set_page(&mut self, page: &Page, length: u32, offset: u32) {
        let c: *mut bindings::scatterlist = self.0.get();
        // SAFETY: according to the `SGEntry` invariant, the scatterlist pointer is valid.
        // `Page` invariant also ensure the pointer is valid.
        unsafe { bindings::sg_set_page(c, page.as_ptr(), length, offset) };
    }
}

/// A scatter-gather table of DMA address spans.
///
/// This structure represents the Rust abstraction for a C `struct sg_table`. This implementation
/// abstracts the usage of an already existing C `struct sg_table` within Rust code that we get
/// passed from the C side.
///
/// Note that constructing a new scatter-gather object using [`SGTable::alloc_table`] enforces safe
/// and proper use of the API.
///
/// # Invariants
///
/// The `sg_table` pointer is valid for the lifetime of an SGTable instance.
#[repr(transparent)]
pub struct SGTable(Opaque<bindings::sg_table>);

impl SGTable {
    /// Convert a raw `struct sg_table *` to a `&'a SGTable`.
    ///
    /// This is meant as a helper for other kernel subsystems and not to be used by device drivers directly.
    ///
    /// # Safety
    ///
    /// Callers must ensure that the `struct sg_table` pointed to by `ptr` is initialized and valid for
    /// the lifetime of the returned reference.
    #[allow(unused)]
    pub(crate) unsafe fn as_ref<'a>(ptr: *mut bindings::sg_table) -> &'a Self {
        // SAFETY: Guaranteed by the safety requirements of the function.
        unsafe { &*ptr.cast() }
    }

    /// Convert a raw `struct sg_table *` to a `&'a mut SGTable`.
    ///
    /// This is meant as a helper for other kernel subsystems and not to be used by device drivers directly.
    ///
    /// # Safety
    ///
    /// See safety requirements of [`SGTable::as_ref`]. In addition, callers must ensure that only
    /// a single mutable reference can be taken from the same raw pointer, i.e. for the lifetime of the
    /// returned reference, no other call to this function on the same `struct sg_table *` should
    /// be permitted.
    #[allow(unused)]
    pub(crate) unsafe fn as_mut<'a>(ptr: *mut bindings::sg_table) -> &'a mut Self {
        // SAFETY: Guaranteed by the safety requirements of the function.
        unsafe { &mut *ptr.cast() }
    }

    /// Obtain the raw `struct sg_table *`.
    pub(crate) fn as_raw(&self) -> *mut bindings::sg_table {
        self.0.get()
    }

    /// Allocate and build a new scatter-gather table from an existing list of `pages`. This method
    /// moves the ownership of `pages` to the table.
    ///
    /// To build a scatter-gather table, provide the `pages` object which must implement the
    /// `SGTablePages` trait.
    ///
    ///# Examples
    ///
    /// ```
    /// use kernel::{device::Device, scatterlist::*, page::*, prelude::*};
    ///
    /// struct PagesArray(KVec<Page>);
    /// impl SGTablePages for PagesArray {
    ///     fn iter<'a>(&'a self) -> impl Iterator<Item = (&'a Page, usize, usize)> {
    ///         self.0.iter().map(|page| (page, kernel::page::PAGE_SIZE, 0))
    ///     }
    ///
    ///     fn entries(&self) -> usize {
    ///         self.0.len()
    ///     }
    /// }
    ///
    /// let mut pages = KVec::new();
    /// let _ = pages.push(Page::alloc_page(GFP_KERNEL)?, GFP_KERNEL);
    /// let _ = pages.push(Page::alloc_page(GFP_KERNEL)?, GFP_KERNEL);
    /// let sgt = SGTable::alloc_table(PagesArray(pages), GFP_KERNEL)?;
    /// # Ok::<(), Error>(())
    /// ```
    pub fn alloc_table<P: SGTablePages>(
        pages: P,
        flags: kernel::alloc::Flags,
    ) -> Result<SGTablePageList<P>> {
        let sgt: Opaque<bindings::sg_table> = Opaque::uninit();

        // SAFETY: The sgt pointer is from the Opaque-wrapped `sg_table` object hence is valid.
        let ret =
            unsafe { bindings::sg_alloc_table(sgt.get(), pages.entries() as u32, flags.as_raw()) };
        if ret != 0 {
            return Err(Error::from_errno(ret));
        }
        let mut sgtable = Self(sgt);
        let sgentries = sgtable.iter_mut().zip(pages.iter());
        for entry in sgentries {
            let page = entry.1;
            entry.0.set_page(page.0, page.1 as u32, page.2 as u32)
        }

        // INVARIANT: We just successfully allocated and built the table from the page entries.
        Ok(SGTablePageList { sg: sgtable, pages })
    }

    /// Map this scatter-gather table describing a buffer for DMA by the `Device`.
    ///
    /// This is meant as a helper for other kernel subsystems and not to be used by device drivers directly.
    /// See also the safe version [`SGTablePageList::dma_map`] which enforces the safety requirements below.
    ///
    /// # Safety
    ///
    /// * The caller takes responsibility that the sg entries in the scatter-gather table object are
    ///   already populated beforehand.
    /// * The caller takes responsibility that the table does not get mapped more than once to prevent UB.
    pub(crate) unsafe fn dma_map_raw(&self, dev: &Device<Bound>, dir: DmaDataDirection) -> Result {
        // SAFETY: Invariants on `Device` and `SGTable` ensures that the pointers are valid.
        let ret = unsafe {
            bindings::dma_map_sgtable(
                dev.as_raw(),
                self.as_raw(),
                dir as i32,
                bindings::DMA_ATTR_NO_WARN as usize,
            )
        };
        if ret != 0 {
            return Err(Error::from_errno(ret));
        }
        Ok(())
    }

    /// Returns an immutable iterator over the scatter-gather table that is mapped for DMA. The iterator
    /// serves as an interface to retrieve the DMA address of the entries
    ///
    /// This is meant as a helper for other kernel subsystems and not to be used by device drivers directly.
    /// See also the safe version [`DeviceSGTable::iter`] which enforces the safety requirement below.
    ///
    /// # Safety
    ///
    /// Callers take responsibility that `self` is already mapped for DMA by a device.
    pub(crate) unsafe fn iter_raw(&self) -> SGTableIter<'_> {
        SGTableIter {
            // SAFETY: dereferenced pointer is valid due to the type invariants on `SGTable`.
            pos: Some(unsafe { SGEntry::as_ref((*self.0.get()).sgl) }),
        }
    }

    /// Internal helper to create a mutable iterator for the constructor when building the table. Not
    /// to be used directly.
    fn iter_mut(&mut self) -> SGTableIterMut<'_> {
        SGTableIterMut {
            // SAFETY: dereferenced pointer is valid due to the type invariants on `SGTable`. This call
            // is within a private method called only within the `[SGTable::alloc_table]` constructor
            // ensuring that the mutable reference can only be obtained once for this object.
            pos: Some(unsafe { SGEntry::as_mut((*self.0.get()).sgl) }),
        }
    }
}

/// Provides a list of pages that can be used to build a `SGTable`.
pub trait SGTablePages {
    /// Returns an iterator to the pages providing the backing memory of `self`.
    ///
    /// Implementers should return an iterator which provides information regarding each page entry to
    /// build the `SGTable`. The first element in the tuple is a reference to the Page, the second element
    /// as the offset into the page, and the third as the length of data. The fields correspond to the
    /// first three fields of the C `struct scatterlist`.
    fn iter<'a>(&'a self) -> impl Iterator<Item = (&'a Page, usize, usize)>;

    /// Returns the number of pages in the list.
    fn entries(&self) -> usize;
}

/// A scatter-gather table that owns the page entries.
///
/// # Invariants
///
/// The scatter-gather table is valid and initialized with sg entries.
pub struct SGTablePageList<P: SGTablePages> {
    sg: SGTable,
    pages: P,
}

impl<P: SGTablePages> SGTablePageList<P> {
    /// Map this scatter-gather table describing a buffer for DMA by the `Device`.
    ///
    /// To prevent the table from being mapped more than once, this call consumes `self` and transfers
    /// ownership of resources to the new `DeviceSGTable` object.
    pub fn dma_map(self, dev: &Device<Bound>, dir: DmaDataDirection) -> Result<DeviceSGTable<P>> {
        // SAFETY: By the type invariant, `self.sg` is already built with valid sg entries. Since we are
        // in a method that consumes self, it also ensures that dma_map_raw is only called once for
        // this `SGTable`.
        unsafe {
            self.sg.dma_map_raw(dev, dir)?;
        }

        // INVARIANT: We just successfully mapped the table for DMA.
        Ok(DeviceSGTable {
            sg: self.sg,
            dir,
            dev: dev.into(),
            _pages: self.pages,
        })
    }
}

/// A scatter-gather table that is mapped for DMA operation.
///
/// # Invariants
///
/// The scatter-gather table is mapped for DMA operation.
pub struct DeviceSGTable<P: SGTablePages> {
    sg: SGTable,
    dir: DmaDataDirection,
    dev: ARef<Device>,
    _pages: P,
}

impl<P: SGTablePages> DeviceSGTable<P> {
    /// Returns an immutable iterator over the scather-gather table that is mapped for DMA. The iterator
    /// serves as an interface to retrieve the DMA address of the entries
    pub fn iter(&self) -> SGTableIter<'_> {
        // SAFETY: By the type invariant, `self.sg` is already mapped for DMA.
        unsafe { self.sg.iter_raw() }
    }
}

impl<P: SGTablePages> Drop for DeviceSGTable<P> {
    fn drop(&mut self) {
        // SAFETY: Invariants on `Device<Bound>` and `SGTable` ensures that the `self.dev` and `self.sg`
        // pointers are valid.
        unsafe {
            bindings::dma_unmap_sgtable(self.dev.as_raw(), self.sg.as_raw(), self.dir as i32, 0)
        };
    }
}

/// SAFETY: A `SGTable<Mapped>` is an immutable interface and should be safe to `Send` across threads.
unsafe impl Send for SGTable {}

/// A mutable iterator through `SGTable` entries.
pub struct SGTableIterMut<'a> {
    pos: Option<&'a mut SGEntry>,
}

impl<'a> Iterator for SGTableIterMut<'a> {
    type Item = &'a mut SGEntry;

    fn next(&mut self) -> Option<Self::Item> {
        self.pos.take().map(|entry| {
            let sg = entry.as_raw();
            // SAFETY: `sg` is guaranteed to be valid and non-NULL while inside this closure.
            let next = unsafe { bindings::sg_next(sg) };
            self.pos = (!next.is_null()).then(||
                                              // SAFETY: `SGEntry::as_mut` is called on `next` only once,
                                              // which is valid and non-NULL
                                              // inside the closure.
                                              unsafe { SGEntry::as_mut(next) });
            // SAFETY: `SGEntry::as_mut` is called on `sg` only once, which is valid and non-NULL
            // inside the closure.
            unsafe { SGEntry::as_mut(sg) }
        })
    }
}

impl<'a> IntoIterator for &'a mut SGTable {
    type Item = &'a mut SGEntry;
    type IntoIter = SGTableIterMut<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}

/// An iterator through `SGTable` entries.
pub struct SGTableIter<'a> {
    pos: Option<&'a SGEntry>,
}

impl<'a> Iterator for SGTableIter<'a> {
    type Item = &'a SGEntry;

    fn next(&mut self) -> Option<Self::Item> {
        let entry = self.pos;
        // SAFETY: `sg` is an immutable reference and is equivalent to `scatterlist` via its type
        // invariants, so its safe to use with sg_next.
        let next = unsafe { bindings::sg_next(self.pos?.as_raw()) };

        // SAFETY: `sg_next` returns either a valid pointer to a `scatterlist`, or null if we
        // are at the end of the scatterlist.
        self.pos = (!next.is_null()).then(|| unsafe { SGEntry::as_ref(next) });
        entry
    }
}

impl<'a, P: SGTablePages> IntoIterator for &'a DeviceSGTable<P> {
    type Item = &'a SGEntry;
    type IntoIter = SGTableIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl Drop for SGTable {
    fn drop(&mut self) {
        // SAFETY: Invariant on `SGTable` ensures that the sg_table is valid.
        unsafe { bindings::sg_free_table(self.as_raw()) };
    }
}
