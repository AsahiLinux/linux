// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU UAT (MMU) management
//!
//! AGX GPUs use an MMU called the UAT, which is largely compatible with the ARM64 page table
//! format. This module manages the global MMU structures, including a shared handoff structure
//! that is used to coordinate VM management operations with the firmware, the TTBAT which points
//! to currently active GPU VM contexts, as well as the individual `Vm` operations to map and
//! unmap buffer objects into a single user or kernel address space.
//!
//! The actual page table management is delegated to the common kernel `io_pgtable` code.

use core::fmt::Debug;
use core::mem::size_of;
use core::num::NonZeroUsize;
use core::ops::Range;
use core::ptr::NonNull;
use core::sync::atomic::{fence, AtomicU32, AtomicU64, AtomicU8, Ordering};
use core::time::Duration;

use kernel::{
    bindings, c_str, delay, device, drm,
    drm::{gem::BaseObject, gpuvm, mm},
    error::{to_result, Result},
    io_pgtable,
    io_pgtable::{prot, AppleUAT, IoPageTable},
    prelude::*,
    static_lock_class,
    sync::{
        lock::{mutex::MutexBackend, Guard},
        Arc, Mutex,
    },
    time::{clock, Now},
    types::{ARef, ForeignOwnable},
};

use crate::debug::*;
use crate::no_debug;
use crate::{driver, fw, gem, hw, mem, slotalloc, util::RangeExt};

const DEBUG_CLASS: DebugFlags = DebugFlags::Mmu;

/// PPL magic number for the handoff region
const PPL_MAGIC: u64 = 0x4b1d000000000002;

/// Number of supported context entries in the TTBAT
const UAT_NUM_CTX: usize = 64;
/// First context available for users
const UAT_USER_CTX_START: usize = 1;
/// Number of available user contexts
const UAT_USER_CTX: usize = UAT_NUM_CTX - UAT_USER_CTX_START;

/// Number of bits in a page offset.
pub(crate) const UAT_PGBIT: usize = 14;
/// UAT page size.
pub(crate) const UAT_PGSZ: usize = 1 << UAT_PGBIT;
/// UAT page offset mask.
pub(crate) const UAT_PGMSK: usize = UAT_PGSZ - 1;

type Pte = AtomicU64;

/// Number of PTEs per page.
const UAT_NPTE: usize = UAT_PGSZ / size_of::<Pte>();

/// UAT input address space (user)
pub(crate) const UAT_IAS: usize = 39;
/// "Fake" kernel UAT input address space (one page level lower)
pub(crate) const UAT_IAS_KERN: usize = 36;

/// Lower/user base VA
pub(crate) const IOVA_USER_BASE: u64 = UAT_PGSZ as u64;
/// Lower/user top VA
pub(crate) const IOVA_USER_TOP: u64 = 1 << (UAT_IAS as u64);
/// Lower/user VA range
pub(crate) const IOVA_USER_RANGE: Range<u64> = IOVA_USER_BASE..IOVA_USER_TOP;

/// Upper/kernel base VA
// const IOVA_TTBR1_BASE: usize = 0xffffff8000000000;
/// Driver-managed kernel base VA
const IOVA_KERN_BASE: u64 = 0xffffffa000000000;
/// Driver-managed kernel top VA
const IOVA_KERN_TOP: u64 = 0xffffffb000000000;
/// Lower/user VA range
const IOVA_KERN_RANGE: Range<u64> = IOVA_KERN_BASE..IOVA_KERN_TOP;

const TTBR_VALID: u64 = 0x1; // BIT(0)
const TTBR_ASID_SHIFT: usize = 48;

const PTE_TABLE: u64 = 0x3; // BIT(0) | BIT(1)

/// Address of a special dummy page?
//const IOVA_UNK_PAGE: u64 = 0x6f_ffff8000;
pub(crate) const IOVA_UNK_PAGE: u64 = IOVA_USER_TOP - 2 * UAT_PGSZ as u64;
/// User VA range excluding the unk page
pub(crate) const IOVA_USER_USABLE_RANGE: Range<u64> = IOVA_USER_BASE..IOVA_UNK_PAGE;

// KernelMapping protection types

// Note: prot::CACHE means "cache coherency", which for UAT means *uncached*,
// since uncached mappings from the GFX ASC side are cache coherent with the AP cache.
// Not having that flag means *cached noncoherent*.

/// Firmware MMIO R/W
pub(crate) const PROT_FW_MMIO_RW: u32 =
    prot::PRIV | prot::READ | prot::WRITE | prot::CACHE | prot::MMIO;
/// Firmware MMIO R/O
pub(crate) const PROT_FW_MMIO_RO: u32 = prot::PRIV | prot::READ | prot::CACHE | prot::MMIO;
/// Firmware shared (uncached) RW
pub(crate) const PROT_FW_SHARED_RW: u32 = prot::PRIV | prot::READ | prot::WRITE | prot::CACHE;
/// Firmware shared (uncached) RO
pub(crate) const PROT_FW_SHARED_RO: u32 = prot::PRIV | prot::READ | prot::CACHE;
/// Firmware private (cached) RW
pub(crate) const PROT_FW_PRIV_RW: u32 = prot::PRIV | prot::READ | prot::WRITE;
/*
/// Firmware private (cached) RO
pub(crate) const PROT_FW_PRIV_RO: u32 = prot::PRIV | prot::READ;
*/
/// Firmware/GPU shared (uncached) RW
pub(crate) const PROT_GPU_FW_SHARED_RW: u32 = prot::READ | prot::WRITE | prot::CACHE;
/// Firmware/GPU shared (private) RW
pub(crate) const PROT_GPU_FW_PRIV_RW: u32 = prot::READ | prot::WRITE;
/// Firmware-RW/GPU-RO shared (private) RW
pub(crate) const PROT_GPU_RO_FW_PRIV_RW: u32 = prot::PRIV | prot::WRITE;
/// GPU shared/coherent RW
pub(crate) const PROT_GPU_SHARED_RW: u32 = prot::READ | prot::WRITE | prot::CACHE | prot::NOEXEC;
/// GPU shared/coherent RO
pub(crate) const PROT_GPU_SHARED_RO: u32 = prot::READ | prot::CACHE | prot::NOEXEC;
/// GPU shared/coherent WO
pub(crate) const PROT_GPU_SHARED_WO: u32 = prot::WRITE | prot::CACHE | prot::NOEXEC;
/*
/// GPU private/noncoherent RW
pub(crate) const PROT_GPU_PRIV_RW: u32 = prot::READ | prot::WRITE | prot::NOEXEC;
/// GPU private/noncoherent RO
pub(crate) const PROT_GPU_PRIV_RO: u32 = prot::READ | prot::NOEXEC;
*/

type PhysAddr = bindings::phys_addr_t;

/// A pre-allocated memory region for UAT management
struct UatRegion {
    base: PhysAddr,
    map: NonNull<core::ffi::c_void>,
}

/// It's safe to share UAT region records across threads.
unsafe impl Send for UatRegion {}
unsafe impl Sync for UatRegion {}

/// Handoff region flush info structure
#[repr(C)]
struct FlushInfo {
    state: AtomicU64,
    addr: AtomicU64,
    size: AtomicU64,
}

/// UAT Handoff region layout
#[repr(C)]
struct Handoff {
    magic_ap: AtomicU64,
    magic_fw: AtomicU64,

    lock_ap: AtomicU8,
    lock_fw: AtomicU8,
    // Implicit padding: 2 bytes
    turn: AtomicU32,
    cur_slot: AtomicU32,
    // Implicit padding: 4 bytes
    flush: [FlushInfo; UAT_NUM_CTX + 1],

    unk2: AtomicU8,
    // Implicit padding: 7 bytes
    unk3: AtomicU64,
}

const HANDOFF_SIZE: usize = size_of::<Handoff>();

/// One VM slot in the TTBAT
#[repr(C)]
struct SlotTTBS {
    ttb0: AtomicU64,
    ttb1: AtomicU64,
}

const SLOTS_SIZE: usize = UAT_NUM_CTX * size_of::<SlotTTBS>();

// We need at least page 0 (ttb0)
const PAGETABLES_SIZE: usize = UAT_PGSZ;

/// Inner data for a Vm instance. This is reference-counted by the outer Vm object.
struct VmInner {
    dev: driver::AsahiDevRef,
    is_kernel: bool,
    va_range: Range<u64>,
    page_table: AppleUAT<Uat>,
    mm: mm::Allocator<(), KernelMappingInner>,
    uat_inner: Arc<UatInner>,
    binding: Arc<Mutex<VmBinding>>,
    id: u64,
}

/// Slot binding-related inner data for a Vm instance.
struct VmBinding {
    active_users: usize,
    binding: Option<slotalloc::Guard<SlotInner>>,
    bind_token: Option<slotalloc::SlotToken>,
    ttb: u64,
}

/// Data associated with a VM <=> BO pairing
#[pin_data]
struct VmBo {
    #[pin]
    sgt: Mutex<Option<gem::SGTable>>,
}

impl gpuvm::DriverGpuVmBo for VmBo {
    fn new() -> impl PinInit<Self> {
        pin_init!(VmBo {
            sgt <- Mutex::new_named(None, c_str!("VmBinding")),
        })
    }
}

#[derive(Default)]
struct StepContext {
    new_va: Option<Pin<Box<gpuvm::GpuVa<VmInner>>>>,
    prev_va: Option<Pin<Box<gpuvm::GpuVa<VmInner>>>>,
    next_va: Option<Pin<Box<gpuvm::GpuVa<VmInner>>>>,
    vm_bo: Option<ARef<gpuvm::GpuVmBo<VmInner>>>,
    prot: u32,
}

impl gpuvm::DriverGpuVm for VmInner {
    type Driver = driver::AsahiDriver;
    type GpuVmBo = VmBo;
    type StepContext = StepContext;

    fn step_map(
        self: &mut gpuvm::UpdatingGpuVm<'_, Self>,
        op: &mut gpuvm::OpMap<Self>,
        ctx: &mut Self::StepContext,
    ) -> Result {
        let mut iova = op.addr();
        let mut left = op.range() as usize;
        let mut offset = op.offset() as usize;

        let bo = ctx.vm_bo.as_ref().expect("step_map with no BO");

        let guard = bo.inner().sgt.lock();
        for range in guard.as_ref().expect("step_map with no SGT").iter() {
            let mut addr = range.dma_address();
            let mut len = range.dma_len();

            if left == 0 {
                break;
            }

            if offset > 0 {
                let skip = len.min(offset);
                addr += skip;
                len -= skip;
                offset -= skip;
            }

            if len == 0 {
                continue;
            }

            assert!(offset == 0);

            len = len.min(left);

            mod_dev_dbg!(
                self.dev,
                "MMU: map: {:#x}:{:#x} -> {:#x}\n",
                addr,
                len,
                iova
            );

            self.map_pages(iova, addr, UAT_PGSZ, len >> UAT_PGBIT, ctx.prot)?;

            left -= len;
            iova += len as u64;
        }

        let gpuva = ctx.new_va.take().expect("Multiple step_map calls");

        if op
            .map_and_link_va(
                self,
                gpuva,
                ctx.vm_bo.as_ref().expect("step_map with no BO"),
            )
            .is_err()
        {
            dev_err!(
                self.dev,
                "map_and_link_va failed: {:#x} [{:#x}] -> {:#x}\n",
                op.offset(),
                op.range(),
                op.addr()
            );
            return Err(EINVAL);
        }
        Ok(())
    }
    fn step_unmap(
        self: &mut gpuvm::UpdatingGpuVm<'_, Self>,
        op: &mut gpuvm::OpUnMap<Self>,
        _ctx: &mut Self::StepContext,
    ) -> Result {
        let va = op.va().expect("step_unmap: missing VA");

        mod_dev_dbg!(self.dev, "MMU: unmap: {:#x}:{:#x}\n", va.addr(), va.range());

        self.unmap_pages(va.addr(), UAT_PGSZ, (va.range() >> UAT_PGBIT) as usize)?;

        if let Some(asid) = self.slot() {
            mem::tlbi_range(asid as u8, va.addr() as usize, va.range() as usize);
            mod_dev_dbg!(
                self.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                va.addr(),
                va.range(),
            );
            mem::sync();
        }

        if op.unmap_and_unlink_va().is_none() {
            dev_err!(self.dev, "step_unmap: could not unlink gpuva");
        }
        Ok(())
    }
    fn step_remap(
        self: &mut gpuvm::UpdatingGpuVm<'_, Self>,
        op: &mut gpuvm::OpReMap<Self>,
        ctx: &mut Self::StepContext,
    ) -> Result {
        let prev_gpuva = ctx.prev_va.take().expect("Multiple step_remap calls");
        let next_gpuva = ctx.next_va.take().expect("Multiple step_remap calls");
        let va = op.unmap().va().expect("No previous VA");
        let orig_addr = va.addr();
        let orig_range = va.range();
        let vm_bo = va.vm_bo();

        // Only unmap the hole between prev/next, if they exist
        let unmap_start = if let Some(op) = op.prev_map() {
            op.addr() + op.range()
        } else {
            orig_addr
        };

        let unmap_end = if let Some(op) = op.next_map() {
            op.addr()
        } else {
            orig_addr + orig_range
        };

        let unmap_range = unmap_end - unmap_start;

        mod_dev_dbg!(
            self.dev,
            "MMU: unmap for remap: {:#x}:{:#x} (from {:#x}:{:#x})\n",
            unmap_start,
            unmap_range,
            orig_addr,
            orig_range
        );

        self.unmap_pages(unmap_start, UAT_PGSZ, (unmap_range >> UAT_PGBIT) as usize)?;

        if op.unmap().unmap_and_unlink_va().is_none() {
            dev_err!(self.dev, "step_unmap: could not unlink gpuva");
        }

        if let Some(prev_op) = op.prev_map() {
            if prev_op.map_and_link_va(self, prev_gpuva, &vm_bo).is_err() {
                dev_err!(self.dev, "step_remap: could not relink prev gpuva");
                return Err(EINVAL);
            }
        }

        if let Some(next_op) = op.next_map() {
            if next_op.map_and_link_va(self, next_gpuva, &vm_bo).is_err() {
                dev_err!(self.dev, "step_remap: could not relink next gpuva");
                return Err(EINVAL);
            }
        }

        Ok(())
    }
}

impl VmInner {
    /// Returns the slot index, if this VM is bound.
    fn slot(&self) -> Option<u32> {
        if self.is_kernel {
            // The GFX ASC does not care about the ASID. Pick an arbitrary one.
            // TODO: This needs to be a persistently reserved ASID once we integrate
            // with the ARM64 kernel ASID machinery to avoid overlap.
            Some(0)
        } else {
            // We don't check whether we lost the slot, which could cause unnecessary
            // invalidations against another Vm. However, this situation should be very
            // rare (e.g. a Vm lost its slot, which means 63 other Vms bound in the
            // interim, and then it gets killed / drops its mappings without doing any
            // final rendering). Anything doing active maps/unmaps is probably also
            // rendering and therefore likely bound.
            self.binding
                .lock()
                .bind_token
                .as_ref()
                .map(|token| (token.last_slot() + UAT_USER_CTX_START as u32))
        }
    }

    /// Returns the translation table base for this Vm
    fn ttb(&self) -> u64 {
        self.page_table.cfg().ttbr
    }

    /// Map an IOVA to the shifted address the underlying io_pgtable uses.
    fn map_iova(&self, iova: u64, size: usize) -> Result<u64> {
        if !self.va_range.is_superset(iova..(iova + size as u64)) {
            Err(EINVAL)
        } else if self.is_kernel {
            Ok(iova - self.va_range.start)
        } else {
            Ok(iova)
        }
    }

    /// Map a contiguous range of virtual->physical pages.
    fn map_pages(
        &mut self,
        mut iova: u64,
        mut paddr: usize,
        pgsize: usize,
        pgcount: usize,
        prot: u32,
    ) -> Result<usize> {
        let mut left = pgcount;
        while left > 0 {
            let mapped_iova = self.map_iova(iova, pgsize * left)?;
            let mapped =
                self.page_table
                    .map_pages(mapped_iova as usize, paddr, pgsize, left, prot)?;
            assert!(mapped <= left * pgsize);

            left -= mapped / pgsize;
            paddr += mapped;
            iova += mapped as u64;
        }
        Ok(pgcount * pgsize)
    }

    /// Unmap a contiguous range of pages.
    fn unmap_pages(&mut self, mut iova: u64, pgsize: usize, pgcount: usize) -> Result<usize> {
        let mut left = pgcount;
        while left > 0 {
            let mapped_iova = self.map_iova(iova, pgsize * left)?;
            let mut unmapped = self
                .page_table
                .unmap_pages(mapped_iova as usize, pgsize, left);
            if unmapped == 0 {
                dev_err!(
                    self.dev,
                    "unmap_pages {:#x}:{:#x} returned 0\n",
                    mapped_iova,
                    left
                );
                unmapped = pgsize; // Pretend we unmapped one page and try again...
            }
            assert!(unmapped <= left * pgsize);

            left -= unmapped / pgsize;
            iova += unmapped as u64;
        }

        Ok(pgcount * pgsize)
    }

    /// Map an `mm::Node` representing an mapping in VA space.
    fn map_node(&mut self, node: &mm::Node<(), KernelMappingInner>, prot: u32) -> Result {
        let mut iova = node.start();
        let guard = node.bo.as_ref().ok_or(EINVAL)?.inner().sgt.lock();
        let sgt = guard.as_ref().ok_or(EINVAL)?;

        for range in sgt.iter() {
            let addr = range.dma_address();
            let len = range.dma_len();

            if (addr | len | iova as usize) & UAT_PGMSK != 0 {
                dev_err!(
                    self.dev,
                    "MMU: KernelMapping {:#x}:{:#x} -> {:#x} is not page-aligned\n",
                    addr,
                    len,
                    iova
                );
                return Err(EINVAL);
            }

            mod_dev_dbg!(
                self.dev,
                "MMU: map: {:#x}:{:#x} -> {:#x}\n",
                addr,
                len,
                iova
            );

            self.map_pages(iova, addr, UAT_PGSZ, len >> UAT_PGBIT, prot)?;

            iova += len as u64;
        }
        Ok(())
    }
}

/// Shared reference to a virtual memory address space ([`Vm`]).
#[derive(Clone)]
pub(crate) struct Vm {
    id: u64,
    inner: ARef<gpuvm::GpuVm<VmInner>>,
    dummy_obj: drm::gem::ObjectRef<gem::Object>,
    binding: Arc<Mutex<VmBinding>>,
}
no_debug!(Vm);

/// Slot data for a [`Vm`] slot (nothing, we only care about the indices).
pub(crate) struct SlotInner();

impl slotalloc::SlotItem for SlotInner {
    type Data = ();
}

/// Represents a single user of a binding of a [`Vm`] to a slot.
///
/// The number of users is counted, and the slot will be freed when it drops to 0.
#[derive(Debug)]
pub(crate) struct VmBind(Vm, u32);

impl VmBind {
    /// Returns the slot that this `Vm` is bound to.
    pub(crate) fn slot(&self) -> u32 {
        self.1
    }
}

impl Drop for VmBind {
    fn drop(&mut self) {
        let mut binding = self.0.binding.lock();

        assert_ne!(binding.active_users, 0);
        binding.active_users -= 1;
        mod_pr_debug!(
            "MMU: slot {} active users {}\n",
            self.1,
            binding.active_users
        );
        if binding.active_users == 0 {
            binding.binding = None;
        }
    }
}

impl Clone for VmBind {
    fn clone(&self) -> VmBind {
        let mut binding = self.0.binding.lock();

        binding.active_users += 1;
        mod_pr_debug!(
            "MMU: slot {} active users {}\n",
            self.1,
            binding.active_users
        );
        VmBind(self.0.clone(), self.1)
    }
}

/// Inner data required for an object mapping into a [`Vm`].
pub(crate) struct KernelMappingInner {
    // Drop order matters:
    // - Drop the GpuVmBo first, which resv locks its BO and drops a GpuVm reference
    // - Drop the GEM BO next, since BO free can take the resv lock itself
    // - Drop the owner GpuVm last, since that again can take resv locks when the refcount drops to 0
    bo: Option<ARef<gpuvm::GpuVmBo<VmInner>>>,
    _gem: Option<drm::gem::ObjectRef<gem::Object>>,
    owner: ARef<gpuvm::GpuVm<VmInner>>,
    uat_inner: Arc<UatInner>,
    prot: u32,
    mapped_size: usize,
}

/// An object mapping into a [`Vm`], which reserves the address range from use by other mappings.

pub(crate) struct KernelMapping(mm::Node<(), KernelMappingInner>);

impl KernelMapping {
    /// Returns the IOVA base of this mapping
    pub(crate) fn iova(&self) -> u64 {
        self.0.start()
    }

    /// Returns the size of this mapping in bytes
    pub(crate) fn size(&self) -> usize {
        self.0.mapped_size
    }

    /// Remap a cached mapping as uncached, then synchronously flush that range of VAs from the
    /// coprocessor cache. This is required to safely unmap cached/private mappings.
    fn remap_uncached_and_flush(&mut self) {
        let mut owner = self
            .0
            .owner
            .exec_lock(None)
            .expect("Failed to exec_lock in remap_uncached_and_flush");

        mod_dev_dbg!(
            owner.dev,
            "MMU: remap as uncached {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        // The IOMMU API does not allow us to remap things in-place...
        // just do an unmap and map again for now.
        // Do not try to unmap guard page (-1)
        if owner
            .unmap_pages(self.iova(), UAT_PGSZ, self.size() >> UAT_PGBIT)
            .is_err()
        {
            dev_err!(
                owner.dev,
                "MMU: unmap for remap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
        }

        let prot = self.0.prot | prot::CACHE;
        if owner.map_node(&self.0, prot).is_err() {
            dev_err!(
                owner.dev,
                "MMU: remap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
        }

        // If we don't have (and have never had) a VM slot, just return
        let slot = match owner.slot() {
            None => return,
            Some(slot) => slot,
        };

        let flush_slot = if owner.is_kernel {
            // If this is a kernel mapping, always flush on index 64
            UAT_NUM_CTX as u32
        } else {
            // Otherwise, check if this slot is the active one, otherwise return
            // Also check that we actually own this slot
            let ttb = owner.ttb() | TTBR_VALID | (slot as u64) << TTBR_ASID_SHIFT;

            let uat_inner = self.0.uat_inner.lock();
            uat_inner.handoff().lock();
            let cur_slot = uat_inner.handoff().current_slot();
            let ttb_cur = uat_inner.ttbs()[slot as usize].ttb0.load(Ordering::Relaxed);
            uat_inner.handoff().unlock();
            if cur_slot == Some(slot) && ttb_cur == ttb {
                slot
            } else {
                return;
            }
        };

        // FIXME: There is a race here, though it'll probably never happen in practice.
        // In theory, it's possible for the ASC to finish using our slot, whatever command
        // it was processing to complete, the slot to be lost to another context, and the ASC
        // to begin using it again with a different page table, thus faulting when it gets a
        // flush request here. In practice, the chance of this happening is probably vanishingly
        // small, as all 62 other slots would have to be recycled or in use before that slot can
        // be reused, and the ASC using user contexts at all is very rare.

        // Still, the locking around UAT/Handoff/TTBs should probably be redesigned to better
        // model the interactions with the firmware and avoid these races.
        // Possibly TTB changes should be tied to slot locks:

        // Flush:
        //  - Can early check handoff here (no need to lock).
        //      If user slot and it doesn't match the active ASC slot,
        //      we can elide the flush as the ASC guarantees it flushes
        //      TLBs/caches when it switches context. We just need a
        //      barrier to ensure ordering.
        //  - Lock TTB slot
        //      - If user ctx:
        //          - Lock handoff AP-side
        //              - Lock handoff dekker
        //                  - Check TTB & handoff cur ctx
        //      - Perform flush if necessary
        //          - This implies taking the fwring lock
        //
        // TTB change:
        //  - lock TTB slot
        //      - lock handoff AP-side
        //          - lock handoff dekker
        //              change TTB

        // Lock this flush slot, and write the range to it
        let flush = self.0.uat_inner.lock_flush(flush_slot);
        let pages = self.size() >> UAT_PGBIT;
        flush.begin_flush(self.iova(), self.size() as u64);
        if pages >= 0x10000 {
            dev_err!(owner.dev, "MMU: Flush too big ({:#x} pages))\n", pages);
        }

        let cmd = fw::channels::FwCtlMsg {
            addr: fw::types::U64(self.iova()),
            unk_8: 0,
            slot: flush_slot,
            page_count: pages as u16,
            unk_12: 2, // ?
        };

        // Tell the firmware to do a cache flush
        if let Err(e) = owner.dev.data().gpu.fwctl(cmd) {
            dev_err!(
                owner.dev,
                "MMU: ASC cache flush {:#x}:{:#x} failed (err: {:?})\n",
                self.iova(),
                self.size(),
                e
            );
        }

        // Finish the flush
        flush.end_flush();

        // Slot is unlocked here
    }
}
no_debug!(KernelMapping);

impl Drop for KernelMapping {
    fn drop(&mut self) {
        // This is the main unmap function for UAT mappings.
        // The sequence of operations here is finicky, due to the interaction
        // between cached GFX ASC mappings and the page tables. These mappings
        // always have to be flushed from the cache before being unmapped.

        // For uncached mappings, just unmapping and flushing the TLB is sufficient.

        // For cached mappings, this is the required sequence:
        // 1. Remap it as uncached
        // 2. Flush the TLB range
        // 3. If kernel VA mapping OR user VA mapping and handoff.current_slot() == slot:
        //    a. Take a lock for this slot
        //    b. Write the flush range to the right context slot in handoff area
        //    c. Issue a cache invalidation request via FwCtl queue
        //    d. Poll for completion via queue
        //    e. Check for completion flag in the handoff area
        //    f. Drop the lock
        // 4. Unmap
        // 5. Flush the TLB range again

        // prot::CACHE means "cache coherent" which means *uncached* here.
        if self.0.prot & prot::CACHE == 0 {
            self.remap_uncached_and_flush();
        }

        let mut owner = self
            .0
            .owner
            .exec_lock(None)
            .expect("exec_lock failed in KernelMapping::drop");
        mod_dev_dbg!(
            owner.dev,
            "MMU: unmap {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        if owner
            .unmap_pages(self.iova(), UAT_PGSZ, self.size() >> UAT_PGBIT)
            .is_err()
        {
            dev_err!(
                owner.dev,
                "MMU: unmap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
        }

        if let Some(asid) = owner.slot() {
            mem::tlbi_range(asid as u8, self.iova() as usize, self.size());
            mod_dev_dbg!(
                owner.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                self.iova(),
                self.size()
            );
            mem::sync();
        }
    }
}

/// Shared UAT global data structures
struct UatShared {
    kernel_ttb1: u64,
    map_kernel_to_user: bool,
    handoff_rgn: UatRegion,
    ttbs_rgn: UatRegion,
}

impl UatShared {
    /// Returns the handoff region area
    fn handoff(&self) -> &Handoff {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.handoff_rgn.map.as_ptr() as *mut Handoff).as_ref() }.unwrap()
    }

    /// Returns the TTBAT area
    fn ttbs(&self) -> &[SlotTTBS; UAT_NUM_CTX] {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.ttbs_rgn.map.as_ptr() as *mut [SlotTTBS; UAT_NUM_CTX]).as_ref() }.unwrap()
    }
}

// SAFETY: Nothing here is unsafe to send across threads.
unsafe impl Send for UatShared {}

/// Inner data for the top-level UAT instance.
#[pin_data]
struct UatInner {
    #[pin]
    shared: Mutex<UatShared>,
    #[pin]
    handoff_flush: [Mutex<HandoffFlush>; UAT_NUM_CTX + 1],
}

impl UatInner {
    /// Take the lock on the shared data and return the guard.
    fn lock(&self) -> Guard<'_, UatShared, MutexBackend> {
        self.shared.lock()
    }

    /// Take a lock on a handoff flush slot and return the guard.
    fn lock_flush(&self, slot: u32) -> Guard<'_, HandoffFlush, MutexBackend> {
        self.handoff_flush[slot as usize].lock()
    }
}

/// Top-level UAT manager object
pub(crate) struct Uat {
    dev: driver::AsahiDevRef,
    cfg: &'static hw::HwConfig,
    pagetables_rgn: UatRegion,

    inner: Arc<UatInner>,
    slots: slotalloc::SlotAllocator<SlotInner>,

    kernel_vm: Vm,
    kernel_lower_vm: Vm,
}

impl Drop for UatRegion {
    fn drop(&mut self) {
        // SAFETY: the pointer is valid by the type invariant
        unsafe { bindings::memunmap(self.map.as_ptr()) };
    }
}

impl Handoff {
    /// Lock the handoff region from firmware access
    fn lock(&self) {
        self.lock_ap.store(1, Ordering::Relaxed);
        fence(Ordering::SeqCst);

        while self.lock_fw.load(Ordering::Relaxed) != 0 {
            if self.turn.load(Ordering::Relaxed) != 0 {
                self.lock_ap.store(0, Ordering::Relaxed);
                while self.turn.load(Ordering::Relaxed) != 0 {}
                self.lock_ap.store(1, Ordering::Relaxed);
                fence(Ordering::SeqCst);
            }
        }
        fence(Ordering::Acquire);
    }

    /// Unlock the handoff region, allowing firmware access
    fn unlock(&self) {
        self.turn.store(1, Ordering::Relaxed);
        self.lock_ap.store(0, Ordering::Release);
    }

    /// Returns the current Vm slot mapped by the firmware for lower/unprivileged access, if any.
    fn current_slot(&self) -> Option<u32> {
        let slot = self.cur_slot.load(Ordering::Relaxed);
        if slot == 0 || slot == u32::MAX {
            None
        } else {
            Some(slot)
        }
    }

    /// Initialize the handoff region
    fn init(&self) -> Result {
        self.magic_ap.store(PPL_MAGIC, Ordering::Relaxed);
        self.cur_slot.store(0, Ordering::Relaxed);
        self.unk3.store(0, Ordering::Relaxed);
        fence(Ordering::SeqCst);

        let start = clock::KernelTime::now();
        const TIMEOUT: Duration = Duration::from_millis(1000);

        self.lock();
        while start.elapsed() < TIMEOUT {
            if self.magic_fw.load(Ordering::Relaxed) == PPL_MAGIC {
                break;
            } else {
                self.unlock();
                delay::coarse_sleep(Duration::from_millis(10));
                self.lock();
            }
        }

        if self.magic_fw.load(Ordering::Relaxed) != PPL_MAGIC {
            self.unlock();
            pr_err!("Handoff: Failed to initialize (firmware not running?)\n");
            return Err(EIO);
        }

        self.unlock();

        for i in 0..=UAT_NUM_CTX {
            self.flush[i].state.store(0, Ordering::Relaxed);
            self.flush[i].addr.store(0, Ordering::Relaxed);
            self.flush[i].size.store(0, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        Ok(())
    }
}

/// Represents a single flush info slot in the handoff region.
///
/// # Invariants
/// The pointer is valid and there is no aliasing HandoffFlush instance.
struct HandoffFlush(*const FlushInfo);

// SAFETY: These pointers are safe to send across threads.
unsafe impl Send for HandoffFlush {}

impl HandoffFlush {
    /// Set up a flush operation for the coprocessor
    fn begin_flush(&self, start: u64, size: u64) {
        let flush = unsafe { self.0.as_ref().unwrap() };

        let state = flush.state.load(Ordering::Relaxed);
        if state != 0 {
            pr_err!("Handoff: expected flush state 0, got {}\n", state);
        }
        flush.addr.store(start, Ordering::Relaxed);
        flush.size.store(size, Ordering::Relaxed);
        flush.state.store(1, Ordering::Relaxed);
    }

    /// Complete a flush operation for the coprocessor
    fn end_flush(&self) {
        let flush = unsafe { self.0.as_ref().unwrap() };
        let state = flush.state.load(Ordering::Relaxed);
        if state != 2 {
            pr_err!("Handoff: expected flush state 2, got {}\n", state);
        }
        flush.state.store(0, Ordering::Relaxed);
    }
}

// We do not implement FlushOps, since we flush manually in this module after
// page table operations. Just provide dummy implementations.
impl io_pgtable::FlushOps for Uat {
    type Data = ();

    fn tlb_flush_all(_data: <Self::Data as ForeignOwnable>::Borrowed<'_>) {}
    fn tlb_flush_walk(
        _data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        _iova: usize,
        _size: usize,
        _granule: usize,
    ) {
    }
    fn tlb_add_page(
        _data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        _iova: usize,
        _granule: usize,
    ) {
    }
}

impl Vm {
    /// Create a new virtual memory address space
    fn new(
        dev: &driver::AsahiDevice,
        uat_inner: Arc<UatInner>,
        kernel_range: Range<u64>,
        cfg: &'static hw::HwConfig,
        is_kernel: bool,
        id: u64,
    ) -> Result<Vm> {
        let dummy_obj = gem::new_kernel_object(dev, 0x4000)?;

        let page_table = AppleUAT::new(
            dev,
            io_pgtable::Config {
                pgsize_bitmap: UAT_PGSZ,
                ias: if is_kernel { UAT_IAS_KERN } else { UAT_IAS },
                oas: cfg.uat_oas,
                coherent_walk: true,
                quirks: 0,
            },
            (),
        )?;
        let (va_range, gpuvm_range) = if is_kernel {
            (IOVA_KERN_RANGE, kernel_range.clone())
        } else {
            (IOVA_USER_RANGE, IOVA_USER_USABLE_RANGE)
        };

        let mm = mm::Allocator::new(va_range.start, va_range.range(), ())?;

        let binding = Arc::pin_init(
            Mutex::new_named(
                VmBinding {
                    binding: None,
                    bind_token: None,
                    active_users: 0,
                    ttb: page_table.cfg().ttbr,
                },
                c_str!("VmBinding"),
            ),
            GFP_KERNEL,
        )?;

        let binding_clone = binding.clone();
        Ok(Vm {
            id,
            dummy_obj: dummy_obj.gem.clone(),
            inner: gpuvm::GpuVm::new(
                c_str!("Asahi::GpuVm"),
                dev,
                &*(dummy_obj.gem),
                gpuvm_range,
                kernel_range,
                init!(VmInner {
                    dev: dev.into(),
                    va_range,
                    is_kernel,
                    page_table,
                    mm,
                    uat_inner,
                    binding: binding_clone,
                    id,
                }),
            )?,
            binding,
        })
    }

    /// Get the translation table base for this Vm
    fn ttb(&self) -> u64 {
        self.binding.lock().ttb
    }

    /// Map a GEM object (using its `SGTable`) into this Vm at a free address in a given range.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn map_in_range(
        &self,
        size: usize,
        gem: &gem::Object,
        alignment: u64,
        range: Range<u64>,
        prot: u32,
        guard: bool,
    ) -> Result<KernelMapping> {
        let sgt = gem.sg_table()?;
        let mut inner = self.inner.exec_lock(Some(gem))?;
        let vm_bo = inner.obtain_bo()?;

        let mut vm_bo_guard = vm_bo.inner().sgt.lock();
        if vm_bo_guard.is_none() {
            vm_bo_guard.replace(sgt);
        }
        core::mem::drop(vm_bo_guard);

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.insert_node_in_range(
            KernelMappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                bo: Some(vm_bo),
                _gem: Some(gem.reference()),
                mapped_size: size,
            },
            (size + if guard { UAT_PGSZ } else { 0 }) as u64, // Add guard page
            alignment,
            0,
            range.start,
            range.end,
            mm::InsertMode::Best,
        )?;

        inner.map_node(&node, prot)?;
        Ok(KernelMapping(node))
    }

    /// Map a GEM object into this Vm at a specific address.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn map_at(
        &self,
        addr: u64,
        size: usize,
        gem: &gem::Object,
        prot: u32,
        guard: bool,
    ) -> Result<KernelMapping> {
        let sgt = gem.sg_table()?;
        let mut inner = self.inner.exec_lock(Some(gem))?;

        let vm_bo = inner.obtain_bo()?;

        let mut vm_bo_guard = vm_bo.inner().sgt.lock();
        if vm_bo_guard.is_none() {
            vm_bo_guard.replace(sgt);
        }
        core::mem::drop(vm_bo_guard);

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.reserve_node(
            KernelMappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                bo: Some(vm_bo),
                _gem: Some(gem.reference()),
                mapped_size: size,
            },
            addr,
            (size + if guard { UAT_PGSZ } else { 0 }) as u64, // Add guard page
            0,
        )?;

        inner.map_node(&node, prot)?;
        Ok(KernelMapping(node))
    }

    /// Map a range of a GEM object into this Vm using GPUVM.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn bind_object(
        &self,
        gem: &gem::Object,
        addr: u64,
        size: u64,
        offset: u64,
        prot: u32,
    ) -> Result {
        // Mapping needs a complete context
        let mut ctx = StepContext {
            new_va: Some(gpuvm::GpuVa::<VmInner>::new(init::default())?),
            prev_va: Some(gpuvm::GpuVa::<VmInner>::new(init::default())?),
            next_va: Some(gpuvm::GpuVa::<VmInner>::new(init::default())?),
            vm_bo: None,
            prot,
        };

        let sgt = gem.sg_table()?;
        let mut inner = self.inner.exec_lock(Some(gem))?;

        let vm_bo = inner.obtain_bo()?;

        let mut vm_bo_guard = vm_bo.inner().sgt.lock();
        if vm_bo_guard.is_none() {
            vm_bo_guard.replace(sgt);
        }
        core::mem::drop(vm_bo_guard);

        ctx.vm_bo = Some(vm_bo);

        if (addr | size | offset) & (UAT_PGMSK as u64) != 0 {
            dev_err!(
                inner.dev,
                "MMU: Map step {:#x} [{:#x}] -> {:#x} is not page-aligned\n",
                offset,
                size,
                addr
            );
            return Err(EINVAL);
        }

        mod_dev_dbg!(
            inner.dev,
            "MMU: sm_map: {:#x} [{:#x}] -> {:#x}\n",
            offset,
            size,
            addr
        );
        inner.sm_map(&mut ctx, addr, size, offset)
    }

    /// Add a direct MMIO mapping to this Vm at a free address.
    pub(crate) fn map_io(
        &self,
        iova: u64,
        phys: usize,
        size: usize,
        prot: u32,
    ) -> Result<KernelMapping> {
        let mut inner = self.inner.exec_lock(None)?;

        if (iova as usize | phys | size) & UAT_PGMSK != 0 {
            dev_err!(
                inner.dev,
                "MMU: KernelMapping {:#x}:{:#x} -> {:#x} is not page-aligned\n",
                phys,
                size,
                iova
            );
            return Err(EINVAL);
        }

        dev_info!(
            inner.dev,
            "MMU: IO map: {:#x}:{:#x} -> {:#x}\n",
            phys,
            size,
            iova
        );

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.reserve_node(
            KernelMappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                bo: None,
                _gem: None,
                mapped_size: size,
            },
            iova,
            size as u64,
            0,
        )?;

        inner.map_pages(iova, phys, UAT_PGSZ, size >> UAT_PGBIT, prot)?;

        Ok(KernelMapping(node))
    }

    /// Unmap everything in an address range.
    pub(crate) fn unmap_range(&self, iova: u64, size: u64) -> Result {
        // Unmapping a range can only do a single split, so just preallocate
        // the prev and next GpuVas
        let mut ctx = StepContext {
            prev_va: Some(gpuvm::GpuVa::<VmInner>::new(init::default())?),
            next_va: Some(gpuvm::GpuVa::<VmInner>::new(init::default())?),
            ..Default::default()
        };

        let mut inner = self.inner.exec_lock(None)?;

        mod_dev_dbg!(inner.dev, "MMU: sm_unmap: {:#x}:{:#x}\n", iova, size);
        inner.sm_unmap(&mut ctx, iova, size)
    }

    /// Drop mappings for a given bo.
    pub(crate) fn drop_mappings(&self, gem: &gem::Object) -> Result {
        // Removing whole mappings only does unmaps, so no preallocated VAs
        let mut ctx = Default::default();

        let mut inner = self.inner.exec_lock(Some(gem))?;

        if let Some(bo) = inner.find_bo() {
            mod_dev_dbg!(inner.dev, "MMU: bo_unmap\n");
            inner.bo_unmap(&mut ctx, &bo)?;
            mod_dev_dbg!(inner.dev, "MMU: bo_unmap done\n");
            // We need to drop the exec_lock first, then the GpuVmBo since that will take the lock itself.
            core::mem::drop(inner);
            core::mem::drop(bo);
        }

        Ok(())
    }

    /// Returns the dummy GEM object used to hold the shared DMA reservation locks
    pub(crate) fn get_resv_obj(&self) -> drm::gem::ObjectRef<gem::Object> {
        self.dummy_obj.clone()
    }

    /// Check whether an object is external to this GpuVm
    pub(crate) fn is_extobj(&self, gem: &gem::Object) -> bool {
        self.inner.is_extobj(gem)
    }
}

impl Drop for VmInner {
    fn drop(&mut self) {
        let mut binding = self.binding.lock();
        assert_eq!(binding.active_users, 0);

        mod_pr_debug!(
            "VmInner::Drop [{}]: bind_token={:?}\n",
            self.id,
            binding.bind_token
        );

        // Make sure this VM is not mapped to a TTB if it was
        if let Some(token) = binding.bind_token.take() {
            let idx = (token.last_slot() as usize) + UAT_USER_CTX_START;
            let ttb = self.ttb() | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT;

            let uat_inner = self.uat_inner.lock();
            uat_inner.handoff().lock();
            let handoff_cur = uat_inner.handoff().current_slot();
            let ttb_cur = uat_inner.ttbs()[idx].ttb0.load(Ordering::SeqCst);
            let inval = ttb_cur == ttb;
            if inval {
                if handoff_cur == Some(idx as u32) {
                    pr_err!(
                        "VmInner::drop owning slot {}, but it is currently in use by the ASC?\n",
                        idx
                    );
                }
                uat_inner.ttbs()[idx].ttb0.store(0, Ordering::SeqCst);
                uat_inner.ttbs()[idx].ttb1.store(0, Ordering::SeqCst);
            }
            uat_inner.handoff().unlock();
            core::mem::drop(uat_inner);

            // In principle we dropped all the KernelMappings already, but we might as
            // well play it safe and invalidate the whole ASID.
            if inval {
                mod_pr_debug!(
                    "VmInner::Drop [{}]: need inval for ASID {:#x}\n",
                    self.id,
                    idx
                );
                mem::tlbi_asid(idx as u8);
                mem::sync();
            }
        }
    }
}

impl Uat {
    /// Map a bootloader-preallocated memory region
    fn map_region(
        dev: &dyn device::RawDevice,
        name: &CStr,
        size: usize,
        cached: bool,
    ) -> Result<UatRegion> {
        let rdev = dev.raw_device();

        let mut res = core::mem::MaybeUninit::<bindings::resource>::uninit();

        let res = unsafe {
            let idx = bindings::of_property_match_string(
                (*rdev).of_node,
                c_str!("memory-region-names").as_char_ptr(),
                name.as_char_ptr(),
            );
            to_result(idx)?;

            let np = bindings::of_parse_phandle(
                (*rdev).of_node,
                c_str!("memory-region").as_char_ptr(),
                idx,
            );
            if np.is_null() {
                dev_err!(dev, "Missing {} region\n", name);
                return Err(EINVAL);
            }
            let ret = bindings::of_address_to_resource(np, 0, res.as_mut_ptr());
            #[cfg(CONFIG_OF_DYNAMIC)]
            bindings::of_node_put(np);

            if ret < 0 {
                dev_err!(dev, "Failed to get {} region\n", name);
                to_result(ret)?
            }

            res.assume_init()
        };

        let rgn_size: usize = unsafe { bindings::resource_size(&res) } as usize;

        if size > rgn_size {
            dev_err!(
                dev,
                "Region {} is too small (expected {}, got {})\n",
                name,
                size,
                rgn_size
            );
            return Err(ENOMEM);
        }

        let flags = if cached {
            bindings::MEMREMAP_WB
        } else {
            bindings::MEMREMAP_WC
        };
        let map = unsafe { bindings::memremap(res.start, rgn_size, flags.into()) };
        let map = NonNull::new(map);

        match map {
            None => {
                dev_err!(dev, "Failed to remap {} region\n", name);
                Err(ENOMEM)
            }
            Some(map) => Ok(UatRegion {
                base: res.start,
                map,
            }),
        }
    }

    /// Returns a view into the root kernel (upper half) page table
    fn kpt0(&self) -> &[Pte; UAT_NPTE] {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.pagetables_rgn.map.as_ptr() as *mut [Pte; UAT_NPTE]).as_ref() }.unwrap()
    }

    /// Returns a reference to the global kernel (upper half) `Vm`
    pub(crate) fn kernel_vm(&self) -> &Vm {
        &self.kernel_vm
    }

    /// Returns a reference to the local kernel (lower half) `Vm`
    pub(crate) fn kernel_lower_vm(&self) -> &Vm {
        &self.kernel_lower_vm
    }

    /// Returns the base physical address of the TTBAT region.
    pub(crate) fn ttb_base(&self) -> u64 {
        let inner = self.inner.lock();

        inner.ttbs_rgn.base
    }

    /// Binds a `Vm` to a slot, preferring the last used one.
    pub(crate) fn bind(&self, vm: &Vm) -> Result<VmBind> {
        let mut binding = vm.binding.lock();

        if binding.binding.is_none() {
            assert_eq!(binding.active_users, 0);

            let isolation = {
                let lock = crate::THIS_MODULE.kernel_param_lock();
                *crate::robust_isolation.read(&lock)
            };

            self.slots.set_limit(if isolation {
                NonZeroUsize::new(1)
            } else {
                None
            });

            let slot = self.slots.get(binding.bind_token)?;
            if slot.changed() {
                mod_pr_debug!("Vm Bind [{}]: bind_token={:?}\n", vm.id, slot.token(),);
                let idx = (slot.slot() as usize) + UAT_USER_CTX_START;
                let ttb = binding.ttb | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT;

                let uat_inner = self.inner.lock();

                let ttb1 = if uat_inner.map_kernel_to_user {
                    uat_inner.kernel_ttb1 | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT
                } else {
                    0
                };

                let ttbs = uat_inner.ttbs();
                uat_inner.handoff().lock();
                if uat_inner.handoff().current_slot() == Some(idx as u32) {
                    pr_err!(
                        "Vm::bind to slot {}, but it is currently in use by the ASC?\n",
                        idx
                    );
                }
                ttbs[idx].ttb0.store(ttb, Ordering::Relaxed);
                ttbs[idx].ttb1.store(ttb1, Ordering::Relaxed);
                uat_inner.handoff().unlock();
                core::mem::drop(uat_inner);

                // Make sure all TLB entries from the previous owner of this ASID are gone
                mem::tlbi_asid(idx as u8);
                mem::sync();
            }

            binding.bind_token = Some(slot.token());
            binding.binding = Some(slot);
        }

        binding.active_users += 1;

        let slot = binding.binding.as_ref().unwrap().slot() + UAT_USER_CTX_START as u32;
        mod_pr_debug!("MMU: slot {} active users {}\n", slot, binding.active_users);
        Ok(VmBind(vm.clone(), slot))
    }

    /// Creates a new `Vm` linked to this UAT.
    pub(crate) fn new_vm(&self, id: u64, kernel_range: Range<u64>) -> Result<Vm> {
        Vm::new(
            &self.dev,
            self.inner.clone(),
            kernel_range,
            self.cfg,
            false,
            id,
        )
    }

    /// Creates the reference-counted inner data for a new `Uat` instance.
    #[inline(never)]
    fn make_inner(dev: &driver::AsahiDevice) -> Result<Arc<UatInner>> {
        let handoff_rgn = Self::map_region(dev, c_str!("handoff"), HANDOFF_SIZE, false)?;
        let ttbs_rgn = Self::map_region(dev, c_str!("ttbs"), SLOTS_SIZE, false)?;

        let handoff = unsafe { &(handoff_rgn.map.as_ptr() as *mut Handoff).as_ref().unwrap() };

        dev_info!(dev, "MMU: Initializing kernel page table\n");

        Arc::pin_init(
            try_pin_init!(UatInner {
                handoff_flush <- init::pin_init_array_from_fn(|i| {
                    Mutex::new_named(HandoffFlush(&handoff.flush[i]), c_str!("handoff_flush"))
                }),
                shared <- Mutex::new_named(
                    UatShared {
                        kernel_ttb1: 0,
                        map_kernel_to_user: false,
                        handoff_rgn,
                        ttbs_rgn,
                    },
                    c_str!("uat_shared")
                ),
            }),
            GFP_KERNEL,
        )
    }

    /// Creates a new `Uat` instance given the relevant hardware config.
    #[inline(never)]
    pub(crate) fn new(
        dev: &driver::AsahiDevice,
        cfg: &'static hw::HwConfig,
        map_kernel_to_user: bool,
    ) -> Result<Self> {
        dev_info!(dev, "MMU: Initializing...\n");

        let inner = Self::make_inner(dev)?;

        let pagetables_rgn = Self::map_region(dev, c_str!("pagetables"), PAGETABLES_SIZE, true)?;

        dev_info!(dev, "MMU: Creating kernel page tables\n");
        let kernel_lower_vm = Vm::new(dev, inner.clone(), IOVA_USER_RANGE, cfg, false, 1)?;
        let kernel_vm = Vm::new(dev, inner.clone(), IOVA_KERN_RANGE, cfg, true, 0)?;

        dev_info!(dev, "MMU: Kernel page tables created\n");

        let ttb0 = kernel_lower_vm.ttb();
        let ttb1 = kernel_vm.ttb();

        let uat = Self {
            dev: dev.into(),
            cfg,
            pagetables_rgn,
            kernel_vm,
            kernel_lower_vm,
            inner,
            slots: slotalloc::SlotAllocator::new(
                UAT_USER_CTX as u32,
                (),
                |_inner, _slot| Some(SlotInner()),
                c_str!("Uat::SlotAllocator"),
                static_lock_class!(),
                static_lock_class!(),
            )?,
        };

        let mut inner = uat.inner.lock();

        inner.map_kernel_to_user = map_kernel_to_user;
        inner.kernel_ttb1 = uat.pagetables_rgn.base;

        inner.handoff().init()?;

        dev_info!(dev, "MMU: Initializing TTBs\n");

        inner.handoff().lock();

        let ttbs = inner.ttbs();

        ttbs[0].ttb0.store(ttb0 | TTBR_VALID, Ordering::Relaxed);
        ttbs[0]
            .ttb1
            .store(uat.pagetables_rgn.base | TTBR_VALID, Ordering::Relaxed);

        for ctx in &ttbs[1..] {
            ctx.ttb0.store(0, Ordering::Relaxed);
            ctx.ttb1.store(0, Ordering::Relaxed);
        }

        inner.handoff().unlock();

        core::mem::drop(inner);

        uat.kpt0()[2].store(ttb1 | PTE_TABLE, Ordering::Relaxed);

        dev_info!(dev, "MMU: initialized\n");

        Ok(uat)
    }
}

impl Drop for Uat {
    fn drop(&mut self) {
        // Unmap what we mapped
        self.kpt0()[2].store(0, Ordering::Relaxed);

        // Make sure we flush the TLBs
        fence(Ordering::SeqCst);
        mem::tlbi_all();
        mem::sync();
    }
}
