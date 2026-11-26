// SPDX-License-Identifier: GPL-2.0 or MIT

// Copied from tyr's mmu/vm/range.rs

//! Range allocator.
//!
//! This module allows you to search for unused ranges to store GEM objects.

use kernel::alloc::Flags;
use kernel::maple_tree::MapleTreeAlloc;
use kernel::prelude::*;
use kernel::sync::Arc;

use core::ops::Range;

/// The actual storage for the ranges.
///
/// All ranges must fit within the `range` field.
///
/// The implementation is different on 32-bit and 64-bit cpus. On 64-bit, the 64-bit addresses are
/// stored directly in the maple tree, but on 32-bit, the maple tree stores the ranges translated
/// in the range zero until `range.end-range.start`. This is done because the maple tree uses
/// unsigned long as its address type, which is too small to store the 64-bit address directly on
/// 32-bit machines.
#[pin_data]
struct RangeAllocInner {
    #[pin]
    maple: MapleTreeAlloc<()>,
    range: Range<u64>,
}

/// This object allows you to allocate ranges on the inner maple tree.
pub(crate) struct RangeAlloc {
    inner: Arc<RangeAllocInner>,
}

/// Represents a live range in the maple tree.
///
/// The destructor removes the range from the maple tree, allowing others to allocate it in the
/// future.
pub(crate) struct LiveRange {
    inner: Arc<RangeAllocInner>,
    offset: u64,
    size: usize,
}

impl RangeAlloc {
    pub(crate) fn new(start: u64, end: u64, gfp: Flags) -> Result<Self> {
        if end < start {
            return Err(EINVAL);
        }

        let inner = Arc::pin_init(
            try_pin_init!(RangeAllocInner {
                maple <- MapleTreeAlloc::new(),
                range: start..end,
            }),
            gfp,
        )?;

        Ok(RangeAlloc { inner })
    }

    pub(crate) fn allocate(&self, size: usize, gfp: Flags) -> Result<LiveRange> {
        let maple_start = self.inner.range.start as usize;
        let maple_end = self.inner.range.end as usize;

        let offset = self
            .inner
            .maple
            .alloc_range(size, (), maple_start..maple_end, gfp)?;

        Ok(LiveRange {
            inner: self.inner.clone(),
            offset: offset as u64,
            size,
        })
    }

    pub(crate) fn insert(&self, start: u64, end: u64, gfp: Flags) -> Result<LiveRange> {
        if end < start {
            return Err(EINVAL);
        }
        if start < self.inner.range.start {
            return Err(EINVAL);
        }
        if end > self.inner.range.end {
            return Err(EINVAL);
        }

        self.inner
            .maple
            .insert_range(start as usize..end as usize, (), gfp)?;

        Ok(LiveRange {
            inner: self.inner.clone(),
            offset: start,
            size: (end - start) as usize,
        })
    }
}

impl LiveRange {
    pub(crate) fn size(&self) -> usize {
        self.size
    }

    pub(crate) fn start(&self) -> u64 {
        self.offset
    }

    pub(crate) fn end(&self) -> u64 {
        self.offset + self.size as u64
    }

    pub(crate) fn range(&self) -> Range<u64> {
        self.start()..self.end()
    }
}

impl Drop for LiveRange {
    fn drop(&mut self) {
        self.inner.maple.erase(self.offset as usize);
    }
}
