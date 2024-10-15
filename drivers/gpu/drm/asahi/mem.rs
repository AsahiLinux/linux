// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! ARM64 low level memory operations.
//!
//! This GPU uses CPU-side `tlbi` outer-shareable instructions to manage its TLBs.
//! Yes, really. Even though the VA address spaces are unrelated.
//!
//! Right now we pick our own ASIDs and don't coordinate with the CPU. This might result
//! in needless TLB shootdowns on the CPU side... TODO: fix this.

use core::arch::asm;
use core::cmp::min;

use crate::debug::*;
use crate::mmu;

type Asid = u8;

/// Invalidate the entire GPU TLB.
#[inline(always)]
pub(crate) fn tlbi_all() {
    unsafe {
        asm!(".arch armv8.4-a", "tlbi vmalle1os",);
    }
}

/// Invalidate all TLB entries for a given ASID.
#[inline(always)]
pub(crate) fn tlbi_asid(asid: Asid) {
    if debug_enabled(DebugFlags::ConservativeTlbi) {
        tlbi_all();
        sync();
        return;
    }

    unsafe {
        asm!(
            ".arch armv8.4-a",
            "tlbi aside1os, {x}",
            x = in(reg) ((asid as u64) << 48)
        );
    }
}

/// Invalidate a single page for a given ASID.
#[inline(always)]
pub(crate) fn tlbi_page(asid: Asid, va: usize) {
    if debug_enabled(DebugFlags::ConservativeTlbi) {
        tlbi_all();
        sync();
        return;
    }

    let val: u64 = ((asid as u64) << 48) | ((va as u64 >> 12) & 0xffffffffffc);
    unsafe {
        asm!(
            ".arch armv8.4-a",
            "tlbi vae1os, {x}",
            x = in(reg) val
        );
    }
}

/// Invalidate a range of pages for a given ASID.
#[inline(always)]
pub(crate) fn tlbi_range(asid: Asid, va: usize, len: usize) {
    if debug_enabled(DebugFlags::ConservativeTlbi) {
        tlbi_all();
        sync();
        return;
    }

    if len == 0 {
        return;
    }

    let start_pg = va >> mmu::UAT_PGBIT;
    let end_pg = (va + len + mmu::UAT_PGMSK) >> mmu::UAT_PGBIT;

    let mut val: u64 = ((asid as u64) << 48) | (2 << 46) | (start_pg as u64 & 0x1fffffffff);
    let pages = end_pg - start_pg;

    // Guess? It's possible that the page count is in terms of 4K pages
    // when the CPU is in 4K mode...
    #[cfg(CONFIG_ARM64_4K_PAGES)]
    let pages = 4 * pages;

    if pages == 1 {
        tlbi_page(asid, va);
        return;
    }

    // Page count is always in units of 2
    let num = ((pages + 1) >> 1) as u64;
    // base: 5 bits
    // exp: 2 bits
    // pages = (base + 1) << (5 * exp + 1)
    // 0:00000 ->                     2 pages = 2 << 0
    // 0:11111 ->                32 * 2 pages = 2 << 5
    // 1:00000 ->            1 * 32 * 2 pages = 2 << 5
    // 1:11111 ->           32 * 32 * 2 pages = 2 << 10
    // 2:00000 ->       1 * 32 * 32 * 2 pages = 2 << 10
    // 2:11111 ->      32 * 32 * 32 * 2 pages = 2 << 15
    // 3:00000 ->  1 * 32 * 32 * 32 * 2 pages = 2 << 15
    // 3:11111 -> 32 * 32 * 32 * 32 * 2 pages = 2 << 20
    let exp = min(3, (64 - num.leading_zeros()) / 5);
    let bits = 5 * exp;
    let mut base = (num + (1 << bits) - 1) >> bits;

    val |= (exp as u64) << 44;

    while base > 32 {
        unsafe {
            asm!(
                ".arch armv8.4-a",
                "tlbi rvae1os, {x}",
                x = in(reg) val | (31 << 39)
            );
        }
        base -= 32;
    }

    unsafe {
        asm!(
            ".arch armv8.4-a",
            "tlbi rvae1os, {x}",
            x = in(reg) val | ((base - 1) << 39)
        );
    }
}

/// Issue a memory barrier (`dsb sy`).
#[inline(always)]
pub(crate) fn sync() {
    unsafe {
        asm!("dsb sy");
    }
}
