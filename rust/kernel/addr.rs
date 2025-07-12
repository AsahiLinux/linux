// SPDX-License-Identifier: GPL-2.0

//! Kernel core address types.

use bindings;
use core::ffi;

/// A physical memory address (which may be wider than the CPU pointer size)
pub type PhysicalAddr = bindings::phys_addr_t;
/// A DMA memory address (which may be narrower than `PhysicalAddr` on some systems)
pub type DmaAddr = bindings::dma_addr_t;
/// A physical resource size, typically the same width as `PhysicalAddr`
pub type ResourceSize = bindings::resource_size_t;
/// A raw page frame number, not to be confused with the C `pfn_t` which also encodes flags.
pub type Pfn = ffi::c_ulong;
