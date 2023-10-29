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
use core::ptr::NonNull;
use core::sync::atomic::{fence, AtomicU32, AtomicU64, AtomicU8, Ordering};
use core::time::Duration;

use kernel::{
    bindings, c_str, delay, device,
    drm::mm,
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
    types::ForeignOwnable,
};

use crate::debug::*;
use crate::no_debug;
use crate::{driver, fw, gem, hw, mem, slotalloc};

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
const IOVA_USER_BASE: usize = UAT_PGSZ;
/// Lower/user top VA
const IOVA_USER_TOP: usize = (1 << UAT_IAS) - 1;
/// Upper/kernel base VA
// const IOVA_TTBR1_BASE: usize = 0xffffff8000000000;
/// Driver-managed kernel base VA
const IOVA_KERN_BASE: usize = 0xffffffa000000000;
/// Driver-managed kernel top VA
const IOVA_KERN_TOP: usize = 0xffffffafffffffff;

const TTBR_VALID: u64 = 0x1; // BIT(0)
const TTBR_ASID_SHIFT: usize = 48;

const PTE_TABLE: u64 = 0x3; // BIT(0) | BIT(1)

// Mapping protection types

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
    min_va: usize,
    max_va: usize,
    page_table: AppleUAT<Uat>,
    mm: mm::Allocator<(), MappingInner>,
    uat_inner: Arc<UatInner>,
    active_users: usize,
    binding: Option<slotalloc::Guard<SlotInner>>,
    bind_token: Option<slotalloc::SlotToken>,
    id: u64,
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
            self.bind_token
                .as_ref()
                .map(|token| (token.last_slot() + UAT_USER_CTX_START as u32))
        }
    }

    /// Returns the translation table base for this Vm
    fn ttb(&self) -> u64 {
        self.page_table.cfg().ttbr
    }

    /// Map an IOVA to the shifted address the underlying io_pgtable uses.
    fn map_iova(&self, iova: usize, size: usize) -> Result<usize> {
        if iova < self.min_va || (iova + size - 1) > self.max_va {
            Err(EINVAL)
        } else if self.is_kernel {
            Ok(iova - self.min_va)
        } else {
            Ok(iova)
        }
    }

    /// Map a contiguous range of virtual->physical pages.
    fn map_pages(
        &mut self,
        mut iova: usize,
        mut paddr: usize,
        pgsize: usize,
        pgcount: usize,
        prot: u32,
    ) -> Result<usize> {
        let mut left = pgcount;
        while left > 0 {
            let mapped_iova = self.map_iova(iova, pgsize * left)?;
            let mapped = self
                .page_table
                .map_pages(mapped_iova, paddr, pgsize, left, prot)?;
            assert!(mapped <= left * pgsize);

            left -= mapped / pgsize;
            paddr += mapped;
            iova += mapped;
        }
        Ok(pgcount * pgsize)
    }

    /// Unmap a contiguous range of pages.
    fn unmap_pages(&mut self, mut iova: usize, pgsize: usize, pgcount: usize) -> Result<usize> {
        let mut left = pgcount;
        while left > 0 {
            let mapped_iova = self.map_iova(iova, pgsize * left)?;
            let unmapped = self.page_table.unmap_pages(mapped_iova, pgsize, left);
            assert!(unmapped <= left * pgsize);

            left -= unmapped / pgsize;
            iova += unmapped;
        }

        Ok(pgcount * pgsize)
    }

    /// Map an `mm::Node` representing an mapping in VA space.
    fn map_node(&mut self, node: &mm::Node<(), MappingInner>, prot: u32) -> Result {
        let mut iova = node.start() as usize;
        let sgt = node.sgt.as_ref().ok_or(EINVAL)?;

        for range in sgt.iter() {
            let addr = range.dma_address();
            let len = range.dma_len();

            if (addr | len | iova) & UAT_PGMSK != 0 {
                dev_err!(
                    self.dev,
                    "MMU: Mapping {:#x}:{:#x} -> {:#x} is not page-aligned\n",
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

            iova += len;
        }
        Ok(())
    }
}

/// Shared reference to a virtual memory address space ([`Vm`]).
#[derive(Clone)]
pub(crate) struct Vm {
    id: u64,
    file_id: u64,
    inner: Arc<Mutex<VmInner>>,
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
        let mut inner = self.0.inner.lock();

        assert_ne!(inner.active_users, 0);
        inner.active_users -= 1;
        mod_pr_debug!("MMU: slot {} active users {}\n", self.1, inner.active_users);
        if inner.active_users == 0 {
            inner.binding = None;
        }
    }
}

impl Clone for VmBind {
    fn clone(&self) -> VmBind {
        let mut inner = self.0.inner.lock();

        inner.active_users += 1;
        mod_pr_debug!("MMU: slot {} active users {}\n", self.1, inner.active_users);
        VmBind(self.0.clone(), self.1)
    }
}

/// Inner data required for an object mapping into a [`Vm`].
pub(crate) struct MappingInner {
    owner: Arc<Mutex<VmInner>>,
    uat_inner: Arc<UatInner>,
    prot: u32,
    mapped_size: usize,
    sgt: Option<gem::SGTable>,
}

/// An object mapping into a [`Vm`], which reserves the address range from use by other mappings.
pub(crate) struct Mapping(mm::Node<(), MappingInner>);

impl Mapping {
    /// Returns the IOVA base of this mapping
    pub(crate) fn iova(&self) -> usize {
        self.0.start() as usize
    }

    /// Returns the size of this mapping in bytes
    pub(crate) fn size(&self) -> usize {
        self.0.mapped_size
    }

    /// Remap a cached mapping as uncached, then synchronously flush that range of VAs from the
    /// coprocessor cache. This is required to safely unmap cached/private mappings.
    fn remap_uncached_and_flush(&mut self) {
        let mut owner = self.0.owner.lock();
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
        flush.begin_flush(self.iova() as u64, self.size() as u64);
        if pages >= 0x10000 {
            dev_err!(owner.dev, "MMU: Flush too big ({:#x} pages))\n", pages);
        }

        let cmd = fw::channels::FwCtlMsg {
            addr: fw::types::U64(self.iova() as u64),
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

impl Drop for Mapping {
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

        let mut owner = self.0.owner.lock();
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
            mem::tlbi_range(asid as u8, self.iova(), self.size());
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
        cfg: &'static hw::HwConfig,
        is_kernel: bool,
        id: u64,
        file_id: u64,
    ) -> Result<Vm> {
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
        let min_va = if is_kernel {
            IOVA_KERN_BASE
        } else {
            IOVA_USER_BASE
        };
        let max_va = if is_kernel {
            IOVA_KERN_TOP
        } else {
            IOVA_USER_TOP
        };

        let mm = mm::Allocator::new(min_va as u64, (max_va - min_va + 1) as u64, ())?;

        Ok(Vm {
            id,
            file_id,
            inner: Arc::pin_init(Mutex::new_named(
                VmInner {
                    dev: dev.into(),
                    min_va,
                    max_va,
                    is_kernel,
                    page_table,
                    mm,
                    uat_inner,
                    binding: None,
                    bind_token: None,
                    active_users: 0,
                    id,
                },
                c_str!("VmInner"),
            ))?,
        })
    }

    /// Get the translation table base for this Vm
    fn ttb(&self) -> u64 {
        self.inner.lock().ttb()
    }

    /// Map a GEM object (using its `SGTable`) into this Vm at a free address in a given range.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn map_in_range(
        &self,
        size: usize,
        sgt: gem::SGTable,
        alignment: u64,
        start: u64,
        end: u64,
        prot: u32,
        guard: bool,
    ) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.insert_node_in_range(
            MappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                sgt: Some(sgt),
                mapped_size: size,
            },
            (size + if guard { UAT_PGSZ } else { 0 }) as u64, // Add guard page
            alignment,
            0,
            start,
            end,
            mm::InsertMode::Best,
        )?;

        inner.map_node(&node, prot)?;
        Ok(Mapping(node))
    }

    /// Map a GEM object (using its `SGTable`) into this Vm at a specific address.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn map_at(
        &self,
        addr: u64,
        size: usize,
        sgt: gem::SGTable,
        prot: u32,
        guard: bool,
    ) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.reserve_node(
            MappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                sgt: Some(sgt),
                mapped_size: size,
            },
            addr,
            (size + if guard { UAT_PGSZ } else { 0 }) as u64, // Add guard page
            0,
        )?;

        inner.map_node(&node, prot)?;
        Ok(Mapping(node))
    }

    /// Add a direct MMIO mapping to this Vm at a free address.
    pub(crate) fn map_io(&self, iova: u64, phys: usize, size: usize, prot: u32) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        if (iova as usize | phys | size) & UAT_PGMSK != 0 {
            dev_err!(
                inner.dev,
                "MMU: Mapping {:#x}:{:#x} -> {:#x} is not page-aligned\n",
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
            MappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                sgt: None,
                mapped_size: size,
            },
            iova,
            size as u64,
            0,
        )?;

        inner.map_pages(iova as usize, phys, UAT_PGSZ, size >> UAT_PGBIT, prot)?;

        Ok(Mapping(node))
    }

    /// Returns the unique ID of this Vm
    pub(crate) fn id(&self) -> u64 {
        self.id
    }

    /// Returns the unique File ID of the owner of this Vm
    pub(crate) fn file_id(&self) -> u64 {
        self.file_id
    }
}

impl Drop for VmInner {
    fn drop(&mut self) {
        assert_eq!(self.active_users, 0);

        mod_pr_debug!(
            "VmInner::Drop [{}]: bind_token={:?}\n",
            self.id,
            self.bind_token
        );

        // Make sure this VM is not mapped to a TTB if it was
        if let Some(token) = self.bind_token.take() {
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

            // In principle we dropped all the Mappings already, but we might as
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
        let mut inner = vm.inner.lock();

        if inner.binding.is_none() {
            assert_eq!(inner.active_users, 0);

            let slot = self.slots.get(inner.bind_token)?;
            if slot.changed() {
                mod_pr_debug!("Vm Bind [{}]: bind_token={:?}\n", vm.id, slot.token(),);
                let idx = (slot.slot() as usize) + UAT_USER_CTX_START;
                let ttb = inner.ttb() | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT;

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

            inner.bind_token = Some(slot.token());
            inner.binding = Some(slot);
        }

        inner.active_users += 1;

        let slot = inner.binding.as_ref().unwrap().slot() + UAT_USER_CTX_START as u32;
        mod_pr_debug!("MMU: slot {} active users {}\n", slot, inner.active_users);
        Ok(VmBind(vm.clone(), slot))
    }

    /// Creates a new `Vm` linked to this UAT.
    pub(crate) fn new_vm(&self, id: u64, file_id: u64) -> Result<Vm> {
        Vm::new(&self.dev, self.inner.clone(), self.cfg, false, id, file_id)
    }

    /// Creates the reference-counted inner data for a new `Uat` instance.
    #[inline(never)]
    fn make_inner(dev: &driver::AsahiDevice) -> Result<Arc<UatInner>> {
        let handoff_rgn = Self::map_region(dev, c_str!("handoff"), HANDOFF_SIZE, false)?;
        let ttbs_rgn = Self::map_region(dev, c_str!("ttbs"), SLOTS_SIZE, false)?;

        let handoff = unsafe { &(handoff_rgn.map.as_ptr() as *mut Handoff).as_ref().unwrap() };

        dev_info!(dev, "MMU: Initializing kernel page table\n");

        Arc::pin_init(try_pin_init!(UatInner {
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
        }))
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
        let kernel_lower_vm = Vm::new(dev, inner.clone(), cfg, false, 1, 0)?;
        let kernel_vm = Vm::new(dev, inner.clone(), cfg, true, 0, 0)?;

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
