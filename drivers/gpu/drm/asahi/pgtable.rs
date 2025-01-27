// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! UAT Page Table management
//!
//! AGX GPUs use an MMU called the UAT, which is largely compatible with the ARM64 page table
//! format. This module manages the actual page tables by allocating raw memory pages from
//! the kernel page allocator.

use core::fmt::Debug;
use core::mem::size_of;
use core::ops::Range;
use core::sync::atomic::{AtomicU64, Ordering};

use kernel::uapi::{PF_R, PF_W, PF_X};
use kernel::{addr::PhysicalAddr, error::Result, page::Page, prelude::*, types::Owned};

use crate::debug::*;
use crate::util::align;

const DEBUG_CLASS: DebugFlags = DebugFlags::PgTable;

/// Number of bits in a page offset.
pub(crate) const UAT_PGBIT: usize = 14;
/// UAT page size.
pub(crate) const UAT_PGSZ: usize = 1 << UAT_PGBIT;
/// UAT page offset mask.
pub(crate) const UAT_PGMSK: usize = UAT_PGSZ - 1;

type Pte = AtomicU64;

const PTE_BIT: usize = 3; // log2(sizeof(Pte))
const PTE_SIZE: usize = 1 << PTE_BIT;

/// Number of PTEs per page.
const UAT_NPTE: usize = UAT_PGSZ / size_of::<Pte>();

/// Number of address bits to address a level
const UAT_LVBIT: usize = UAT_PGBIT - PTE_BIT;
/// Number of entries per level
const UAT_LVSZ: usize = UAT_NPTE;
/// Mask of level bits
const UAT_LVMSK: u64 = (UAT_LVSZ - 1) as u64;

const UAT_LEVELS: usize = 3;

/// UAT input address space
pub(crate) const UAT_IAS: usize = 39;
const UAT_IASMSK: u64 = ((1u64 << UAT_IAS) - 1) as u64;

const PTE_TYPE_BITS: u64 = 3;
const PTE_TYPE_LEAF_TABLE: u64 = 3;

const UAT_AP_SHIFT: u32 = 6;
const UAT_AP_BITS: u64 = 3 << UAT_AP_SHIFT;
const UAT_HIGH_BITS_SHIFT: u32 = 52;
const UAT_HIGH_BITS: u64 = 0xfff << UAT_HIGH_BITS_SHIFT;
const UAT_MEMATTR_SHIFT: u32 = 2;
const UAT_MEMATTR_BITS: u64 = 7 << UAT_MEMATTR_SHIFT;

const UAT_PROT_BITS: u64 = UAT_AP_BITS | UAT_MEMATTR_BITS | UAT_HIGH_BITS;

const UAT_AF: u64 = 1 << 10;

const MEMATTR_CACHED: u8 = 0;
const MEMATTR_DEV: u8 = 1;
const MEMATTR_UNCACHED: u8 = 2;

const AP_FW_GPU: u8 = 0;
const AP_FW: u8 = 1;
const AP_GPU: u8 = 2;

const HIGH_BITS_PXN: u16 = 1 << 1;
const HIGH_BITS_UXN: u16 = 1 << 2;
const HIGH_BITS_GPU_ACCESS: u16 = 1 << 3;

pub(crate) const PTE_ADDR_BITS: u64 = (!0u64) & (!UAT_PGMSK as u64) & (!UAT_HIGH_BITS);

#[derive(Debug, Copy, Clone)]
pub(crate) struct Prot {
    memattr: u8,
    ap: u8,
    high_bits: u16,
}

// Firmware + GPU access
const PROT_FW_GPU_NA: Prot = Prot::from_bits(AP_FW_GPU, 0, 0);
const _PROT_FW_GPU_RO: Prot = Prot::from_bits(AP_FW_GPU, 0, 1);
const _PROT_FW_GPU_WO: Prot = Prot::from_bits(AP_FW_GPU, 1, 0);
const PROT_FW_GPU_RW: Prot = Prot::from_bits(AP_FW_GPU, 1, 1);

// Firmware only access
const PROT_FW_RO: Prot = Prot::from_bits(AP_FW, 0, 0);
const _PROT_FW_NA: Prot = Prot::from_bits(AP_FW, 0, 1);
const PROT_FW_RW: Prot = Prot::from_bits(AP_FW, 1, 0);
const PROT_FW_RW_GPU_RO: Prot = Prot::from_bits(AP_FW, 1, 1);

// GPU only access
const PROT_GPU_RO: Prot = Prot::from_bits(AP_GPU, 0, 0);
const PROT_GPU_WO: Prot = Prot::from_bits(AP_GPU, 0, 1);
const PROT_GPU_RW: Prot = Prot::from_bits(AP_GPU, 1, 0);
const _PROT_GPU_NA: Prot = Prot::from_bits(AP_GPU, 1, 1);

const PF_RW: u32 = PF_R | PF_W;
const PF_RX: u32 = PF_R | PF_X;

// For crash dumps
const PROT_TO_PERMS_FW: [[u32; 4]; 4] = [
    [0, 0, 0, PF_RW],
    [0, PF_RW, 0, PF_RW],
    [PF_RX, PF_RX, 0, PF_R],
    [PF_RX, PF_RW, 0, PF_R],
];
const PROT_TO_PERMS_OS: [[u32; 4]; 4] = [
    [0, PF_R, PF_W, PF_RW],
    [PF_R, 0, PF_RW, PF_RW],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
];

pub(crate) mod prot {
    pub(crate) use super::Prot;
    use super::*;

    /// Firmware MMIO R/W
    pub(crate) const PROT_FW_MMIO_RW: Prot = PROT_FW_RW.memattr(MEMATTR_DEV);
    /// Firmware MMIO R/O
    pub(crate) const PROT_FW_MMIO_RO: Prot = PROT_FW_RO.memattr(MEMATTR_DEV);
    /// Firmware shared (uncached) RW
    pub(crate) const PROT_FW_SHARED_RW: Prot = PROT_FW_RW.memattr(MEMATTR_UNCACHED);
    /// Firmware shared (uncached) RO
    pub(crate) const PROT_FW_SHARED_RO: Prot = PROT_FW_RO.memattr(MEMATTR_UNCACHED);
    /// Firmware private (cached) RW
    pub(crate) const PROT_FW_PRIV_RW: Prot = PROT_FW_RW.memattr(MEMATTR_CACHED);
    /// Firmware/GPU shared (uncached) RW
    pub(crate) const PROT_GPU_FW_SHARED_RW: Prot = PROT_FW_GPU_RW.memattr(MEMATTR_UNCACHED);
    /// Firmware/GPU shared (private) RW
    pub(crate) const PROT_GPU_FW_PRIV_RW: Prot = PROT_FW_GPU_RW.memattr(MEMATTR_CACHED);
    /// Firmware-RW/GPU-RO shared (private) RW
    pub(crate) const PROT_GPU_RO_FW_PRIV_RW: Prot = PROT_FW_RW_GPU_RO.memattr(MEMATTR_CACHED);
    /// GPU shared/coherent RW
    pub(crate) const PROT_GPU_SHARED_RW: Prot = PROT_GPU_RW;
    /// GPU shared/coherent RO
    pub(crate) const PROT_GPU_SHARED_RO: Prot = PROT_GPU_RO;
    /// GPU shared/coherent WO
    pub(crate) const PROT_GPU_SHARED_WO: Prot = PROT_GPU_WO;
}

impl Prot {
    const fn from_bits(ap: u8, uxn: u16, pxn: u16) -> Self {
        assert!(uxn <= 1);
        assert!(pxn <= 1);
        assert!(ap <= 3);

        Prot {
            high_bits: HIGH_BITS_GPU_ACCESS | (pxn * HIGH_BITS_PXN) | (uxn * HIGH_BITS_UXN),
            memattr: 0,
            ap,
        }
    }

    pub(crate) const fn from_pte(pte: u64) -> Self {
        Prot {
            high_bits: (pte >> UAT_HIGH_BITS_SHIFT) as u16,
            ap: ((pte & UAT_AP_BITS) >> UAT_AP_SHIFT) as u8,
            memattr: ((pte & UAT_MEMATTR_BITS) >> UAT_MEMATTR_SHIFT) as u8,
        }
    }

    pub(crate) const fn elf_flags(&self) -> u32 {
        let ap = (self.ap & 3) as usize;
        let uxn = if self.high_bits & HIGH_BITS_UXN != 0 {
            1
        } else {
            0
        };
        let pxn = if self.high_bits & HIGH_BITS_PXN != 0 {
            1
        } else {
            0
        };
        let gpu = self.high_bits & HIGH_BITS_GPU_ACCESS != 0;

        // Format:
        // [12 top bits of PTE] [12 bottom bits of PTE] [5 bits pad] [ELF RWX]
        let mut perms = if gpu {
            PROT_TO_PERMS_OS[ap][(uxn << 1) | pxn]
        } else {
            PROT_TO_PERMS_FW[ap][(uxn << 1) | pxn]
        };

        perms |= ((self.as_pte() >> 52) << 20) as u32;
        perms |= ((self.as_pte() & 0xfff) << 8) as u32;

        perms
    }

    const fn memattr(&self, memattr: u8) -> Self {
        Self { memattr, ..*self }
    }

    const fn as_pte(&self) -> u64 {
        (self.ap as u64) << UAT_AP_SHIFT
            | (self.high_bits as u64) << UAT_HIGH_BITS_SHIFT
            | (self.memattr as u64) << UAT_MEMATTR_SHIFT
            | UAT_AF
    }

    pub(crate) const fn is_cached_noncoherent(&self) -> bool {
        self.ap != AP_GPU && self.memattr == MEMATTR_CACHED
    }

    pub(crate) const fn as_uncached(&self) -> Self {
        self.memattr(MEMATTR_UNCACHED)
    }
}

impl Default for Prot {
    fn default() -> Self {
        PROT_FW_GPU_NA
    }
}

pub(crate) struct DumpedPage {
    pub(crate) iova: u64,
    pub(crate) pte: u64,
    pub(crate) data: Option<Owned<Page>>,
}

pub(crate) struct UatPageTable {
    ttb: PhysicalAddr,
    ttb_owned: bool,
    va_range: Range<u64>,
    oas_mask: u64,
}

impl UatPageTable {
    pub(crate) fn new(oas: usize) -> Result<Self> {
        mod_pr_debug!("UATPageTable::new: oas={}\n", oas);
        let ttb_page = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
        let ttb = Page::into_phys(ttb_page);
        Ok(UatPageTable {
            ttb,
            ttb_owned: true,
            va_range: 0..(1u64 << UAT_IAS),
            oas_mask: (1u64 << oas) - 1,
        })
    }

    pub(crate) fn new_with_ttb(
        ttb: PhysicalAddr,
        va_range: Range<u64>,
        oas: usize,
    ) -> Result<Self> {
        mod_pr_debug!(
            "UATPageTable::new_with_ttb: ttb={:#x} range={:#x?} oas={}\n",
            ttb,
            va_range,
            oas
        );
        if ttb & (UAT_PGMSK as PhysicalAddr) != 0 {
            return Err(EINVAL);
        }
        if (va_range.start | va_range.end) & (UAT_PGMSK as u64) != 0 {
            return Err(EINVAL);
        }
        // SAFETY: The TTB is should remain valid (if properly mapped), as it is bootloader-managed.
        if unsafe { Page::borrow_phys(&ttb) }.is_none() {
            pr_err!(
                "UATPageTable::new_with_ttb: ttb at {:#x} is not mapped (DT using no-map?)\n",
                ttb
            );
            return Err(EIO);
        }

        Ok(UatPageTable {
            ttb,
            ttb_owned: false,
            va_range,
            oas_mask: (1 << oas) - 1,
        })
    }

    pub(crate) fn ttb(&self) -> PhysicalAddr {
        self.ttb
    }

    fn with_pages<F>(
        &mut self,
        iova_range: Range<u64>,
        alloc: bool,
        free: bool,
        mut cb: F,
    ) -> Result
    where
        F: FnMut(u64, &[Pte]) -> Result,
    {
        mod_pr_debug!(
            "UATPageTable::with_pages: {:#x?} alloc={} free={}\n",
            iova_range,
            alloc,
            free
        );
        if (iova_range.start | iova_range.end) & (UAT_PGMSK as u64) != 0 {
            pr_err!(
                "UATPageTable::with_pages: iova range not aligned: {:#x?}\n",
                iova_range
            );
            return Err(EINVAL);
        }

        if iova_range.is_empty() {
            return Ok(());
        }

        let mut iova = iova_range.start & UAT_IASMSK;
        let mut last_iova = iova;
        // Handle the case where iova_range.end is just at the top boundary of the IAS
        let end = ((iova_range.end - 1) & UAT_IASMSK) + 1;

        let mut pt_addr: [Option<PhysicalAddr>; UAT_LEVELS] = Default::default();
        pt_addr[UAT_LEVELS - 1] = Some(self.ttb);

        'outer: while iova < end {
            mod_pr_debug!("UATPageTable::with_pages: iova={:#x}\n", iova);
            let addr_diff = last_iova ^ iova;
            for level in (0..UAT_LEVELS - 1).rev() {
                // If the iova has changed at this level or above, invalidate the physaddr
                if addr_diff & !((1 << (UAT_PGBIT + (level + 1) * UAT_LVBIT)) - 1) != 0 {
                    if let Some(phys) = pt_addr[level].take() {
                        if free {
                            mod_pr_debug!(
                                "UATPageTable::with_pages: free level {} {:#x?}\n",
                                level,
                                phys
                            );
                            // SAFETY: Page tables for our VA ranges always come from Page::into_phys().
                            unsafe { Page::from_phys(phys) };
                        }
                        mod_pr_debug!("UATPageTable::with_pages: invalidate level {}\n", level);
                    }
                }
            }
            last_iova = iova;
            for level in (0..UAT_LEVELS - 1).rev() {
                // Fetch the page table base address for this level
                if pt_addr[level].is_none() {
                    let phys = pt_addr[level + 1].unwrap();
                    mod_pr_debug!(
                        "UATPageTable::with_pages: need level {}, parent phys {:#x}\n",
                        level,
                        phys
                    );
                    let upidx = ((iova >> (UAT_PGBIT + (level + 1) * UAT_LVBIT) as u64) & UAT_LVMSK)
                        as usize;
                    // SAFETY: Page table addresses are either allocated by us, or
                    // firmware-managed and safe to borrow a struct page from.
                    let upt = unsafe { Page::borrow_phys_unchecked(&phys) };
                    mod_pr_debug!("UATPageTable::with_pages: borrowed phys {:#x}\n", phys);
                    pt_addr[level] =
                        upt.with_pointer_into_page(upidx * PTE_SIZE, PTE_SIZE, |p| {
                            let uptep = p as *const _ as *const Pte;
                            // SAFETY: with_pointer_into_page() ensures the pointer is valid,
                            // and our index is aligned so it is safe to deref as an AtomicU64.
                            let upte = unsafe { &*uptep };
                            let mut upte_val = upte.load(Ordering::Relaxed);
                            // Allocate if requested
                            if upte_val == 0 && alloc {
                                let pt_page = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
                                mod_pr_debug!("UATPageTable::with_pages: alloc PT at {:#x}\n", pt_page.phys());
                                let pt_paddr = Page::into_phys(pt_page);
                                upte_val = pt_paddr | PTE_TYPE_LEAF_TABLE;
                                upte.store(upte_val, Ordering::Relaxed);
                            }
                            if upte_val & PTE_TYPE_BITS == PTE_TYPE_LEAF_TABLE {
                                Ok(Some(upte_val & self.oas_mask & (!UAT_PGMSK as u64)))
                            } else if upte_val == 0 || (!alloc && !free) {
                                mod_pr_debug!("UATPageTable::with_pages: no level {}\n", level);
                                Ok(None)
                            } else {
                                pr_err!("UATPageTable::with_pages: Unexpected Table PTE value {:#x} at iova {:#x} index {} phys {:#x}\n", upte_val,
                                        iova, level + 1, phys + ((upidx * PTE_SIZE) as PhysicalAddr));
                                Ok(None)
                            }
                        })?;
                    mod_pr_debug!(
                        "UATPageTable::with_pages: level {} PT {:#x?}\n",
                        level,
                        pt_addr[level]
                    );
                }
                // If we don't have a page table, skip this entire level
                if pt_addr[level].is_none() {
                    let block = 1 << (UAT_PGBIT + UAT_LVBIT * (level + 1));
                    let old = iova;
                    iova = align(iova + 1, block);
                    mod_pr_debug!(
                        "UATPageTable::with_pages: skip {:#x} {:#x} -> {:#x}\n",
                        block,
                        old,
                        iova
                    );
                    continue 'outer;
                }
            }

            let idx = ((iova >> UAT_PGBIT as u64) & UAT_LVMSK) as usize;
            let max_count = UAT_NPTE - idx;
            let count = (((end - iova) >> UAT_PGBIT) as usize).min(max_count);
            let phys = pt_addr[0].unwrap();
            mod_pr_debug!(
                "UATPageTable::with_pages: leaf PT at {:#x} idx {:#x} count {:#x} iova {:#x}\n",
                phys,
                idx,
                count,
                iova
            );
            // SAFETY: Page table addresses are either allocated by us, or
            // firmware-managed and safe to borrow a struct page from.
            let pt = unsafe { Page::borrow_phys_unchecked(&phys) };
            pt.with_pointer_into_page(idx * PTE_SIZE, count * PTE_SIZE, |p| {
                let ptep = p as *const _ as *const Pte;
                // SAFETY: We know this is a valid pointer to PTEs and the range is valid and
                // checked by with_pointer_into_page().
                let ptes = unsafe { core::slice::from_raw_parts(ptep, count) };
                cb(iova, ptes)?;
                Ok(())
            })?;

            let block = 1 << (UAT_PGBIT + UAT_LVBIT);
            iova = align(iova + 1, block);
        }

        if free {
            for level in (0..UAT_LEVELS - 1).rev() {
                if let Some(phys) = pt_addr[level] {
                    mod_pr_debug!(
                        "UATPageTable::with_pages: free level {} {:#x?}\n",
                        level,
                        phys
                    );
                    // SAFETY: Page tables for our VA ranges always come from Page::into_phys().
                    unsafe { Page::from_phys(phys) };
                }
            }
        }

        Ok(())
    }

    pub(crate) fn alloc_pages(&mut self, iova_range: Range<u64>) -> Result {
        mod_pr_debug!("UATPageTable::alloc_pages: {:#x?}\n", iova_range);
        self.with_pages(iova_range, true, false, |_, _| Ok(()))
    }

    pub(crate) fn map_pages(
        &mut self,
        iova_range: Range<u64>,
        mut phys: PhysicalAddr,
        prot: Prot,
        one_page: bool,
    ) -> Result {
        mod_pr_debug!(
            "UATPageTable::map_pages: {:#x?} {:#x?} {:?}\n",
            iova_range,
            phys,
            prot
        );
        if phys & (UAT_PGMSK as PhysicalAddr) != 0 {
            pr_err!("UATPageTable::map_pages: phys not aligned: {:#x?}\n", phys);
            return Err(EINVAL);
        }

        self.with_pages(iova_range, true, false, |iova, ptes| {
            for (idx, pte) in ptes.iter().enumerate() {
                let ptev = pte.load(Ordering::Relaxed);
                if ptev != 0 {
                    pr_err!(
                        "UATPageTable::map_pages: Page at IOVA {:#x} is mapped (PTE: {:#x})\n",
                        iova + (idx * UAT_PGSZ) as u64,
                        ptev
                    );
                }
                pte.store(
                    phys | prot.as_pte() | PTE_TYPE_LEAF_TABLE,
                    Ordering::Relaxed,
                );
                if !one_page {
                    phys += UAT_PGSZ as PhysicalAddr;
                }
            }
            Ok(())
        })
    }

    pub(crate) fn reprot_pages(&mut self, iova_range: Range<u64>, prot: Prot) -> Result {
        mod_pr_debug!(
            "UATPageTable::reprot_pages: {:#x?} {:?}\n",
            iova_range,
            prot
        );
        self.with_pages(iova_range, true, false, |iova, ptes| {
            for (idx, pte) in ptes.iter().enumerate() {
                let ptev = pte.load(Ordering::Relaxed);
                if ptev & PTE_TYPE_BITS != PTE_TYPE_LEAF_TABLE {
                    pr_err!(
                        "UATPageTable::reprot_pages: Page at IOVA {:#x} is unmapped (PTE: {:#x})\n",
                        iova + (idx * UAT_PGSZ) as u64,
                        ptev
                    );
                    continue;
                }
                pte.store((ptev & !UAT_PROT_BITS) | prot.as_pte(), Ordering::Relaxed);
            }
            Ok(())
        })
    }

    pub(crate) fn unmap_pages(&mut self, iova_range: Range<u64>) -> Result {
        mod_pr_debug!("UATPageTable::unmap_pages: {:#x?}\n", iova_range);
        self.with_pages(iova_range, false, false, |iova, ptes| {
            for (idx, pte) in ptes.iter().enumerate() {
                if pte.load(Ordering::Relaxed) & PTE_TYPE_LEAF_TABLE == 0 {
                    pr_err!(
                        "UATPageTable::unmap_pages: Page at IOVA {:#x} already unmapped\n",
                        iova + (idx * UAT_PGSZ) as u64
                    );
                }
                pte.store(0, Ordering::Relaxed);
            }
            Ok(())
        })
    }

    pub(crate) fn dump_pages(&mut self, iova_range: Range<u64>) -> Result<KVVec<DumpedPage>> {
        let mut pages = KVVec::new();
        let oas_mask = self.oas_mask;
        let iova_base = self.va_range.start & !UAT_IASMSK;
        self.with_pages(iova_range, false, false, |iova, ptes| {
            let iova = iova | iova_base;
            for (idx, ppte) in ptes.iter().enumerate() {
                let pte = ppte.load(Ordering::Relaxed);
                if (pte & PTE_TYPE_LEAF_TABLE) != PTE_TYPE_LEAF_TABLE {
                    continue;
                }
                let memattr = ((pte & UAT_MEMATTR_BITS) >> UAT_MEMATTR_SHIFT) as u8;

                if !(memattr == MEMATTR_CACHED || memattr == MEMATTR_UNCACHED) {
                    pages.push(
                        DumpedPage {
                            iova: iova + (idx * UAT_PGSZ) as u64,
                            pte,
                            data: None,
                        },
                        GFP_KERNEL,
                    )?;
                    continue;
                }
                let phys = pte & oas_mask & (!UAT_PGMSK as u64);
                // SAFETY: GPU pages are either firmware/preallocated pages
                // (which the kernel isn't concerned with and are either in
                // the page map or not, and if they aren't, borrow_phys()
                // will fail), or GPU page table pages (which we own),
                // or GEM buffer pages (which are locked while they are
                // mapped in the page table), so they should be safe to
                // borrow.
                //
                // This does trust the firmware not to have any weird
                // mappings in its own internal page tables, but since
                // those are managed by the uPPL which is privileged anyway,
                // this trust does not actually extend any trust boundary.
                let src_page = match unsafe { Page::borrow_phys(&phys) } {
                    Some(page) => page,
                    None => {
                        pages.push(
                            DumpedPage {
                                iova: iova + (idx * UAT_PGSZ) as u64,
                                pte,
                                data: None,
                            },
                            GFP_KERNEL,
                        )?;
                        continue;
                    }
                };
                let dst_page = Page::alloc_page(GFP_KERNEL)?;
                src_page.with_page_mapped(|psrc| -> Result {
                    // SAFETY: This could technically still have a data
                    // race with the firmware or other driver code (or
                    // even userspace with timestamp buffers), but while
                    // the Rust language technically says this is UB, in
                    // the real world, using atomic reads for this is
                    // guaranteed to never cause any harmful effects
                    // other than possibly reading torn/unreliable data.
                    // At least on ARM64 anyway.
                    //
                    // (Yes, I checked with Rust people about this. ~~ Lina)
                    //
                    let src_items = unsafe {
                        core::slice::from_raw_parts(
                            psrc as *const AtomicU64,
                            UAT_PGSZ / core::mem::size_of::<AtomicU64>(),
                        )
                    };
                    dst_page.with_page_mapped(|pdst| -> Result {
                        let dst_items = unsafe {
                            core::slice::from_raw_parts_mut(
                                pdst as *mut u64,
                                UAT_PGSZ / core::mem::size_of::<u64>(),
                            )
                        };
                        for (si, di) in src_items.iter().zip(dst_items.iter_mut()) {
                            *di = si.load(Ordering::Relaxed);
                        }
                        Ok(())
                    })?;
                    Ok(())
                })?;
                pages.push(
                    DumpedPage {
                        iova: iova + (idx * UAT_PGSZ) as u64,
                        pte,
                        data: Some(dst_page),
                    },
                    GFP_KERNEL,
                )?;
            }
            Ok(())
        })?;
        Ok(pages)
    }
}

impl Drop for UatPageTable {
    fn drop(&mut self) {
        mod_pr_debug!("UATPageTable::drop range: {:#x?}\n", &self.va_range);
        if self
            .with_pages(self.va_range.clone(), false, true, |iova, ptes| {
                for (idx, pte) in ptes.iter().enumerate() {
                    if pte.load(Ordering::Relaxed) != 0 {
                        pr_err!(
                            "UATPageTable::drop: Leaked page at IOVA {:#x}\n",
                            iova + (idx * UAT_PGSZ) as u64
                        );
                    }
                }
                Ok(())
            })
            .is_err()
        {
            pr_err!("UATPageTable::drop failed to free page tables\n",);
        }
        if self.ttb_owned {
            // SAFETY: If we own the ttb, it was allocated with Page::into_phys().
            mod_pr_debug!("UATPageTable::drop: Free TTB {:#x}\n", self.ttb);
            unsafe {
                Page::from_phys(self.ttb);
            }
        }
    }
}
