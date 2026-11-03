// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU UAT (MMU) management
//!
//! AGX GPUs use an MMU called the UAT, which is largely compatible with the ARM64 page table
//! format. This module manages the global MMU structures, including a shared handoff structure
//! that is used to coordinate VM management operations with the firmware, the TTBAT which points
//! to currently active GPU VM contexts, as well as the individual `Vm` operations to map and
//! unmap buffer objects into a single user or kernel address space.
//!
//! The actual page table management is in the `pt` module.

use core::fmt::Debug;
use core::mem::size_of;
use core::num::NonZeroUsize;
use core::ops::Range;
use core::sync::atomic::{
    fence,
    AtomicU32,
    AtomicU64,
    AtomicU8,
    Ordering, //
};

use kernel::{
    addr::{
        DmaAddr,
        PhysicalAddr, //
    },
    c_str,
    device::{
        self, //
        Bound,
    },
    dma::DmaAddress,
    drm::{
        gpuvm::{
            self,
            GpuVaAlloc,
            GpuVaFlag,
            GpuVaFlags,
            OpMap,
            OpMapRequest,
            OpMapped,
            OpRemap,
            OpRemapped,
            OpUnmap,
            OpUnmapped, //
        },
        mm,
        Registered, //
    },
    error::Result,
    io::{
        self,
        ResourceSize, //
    },
    new_mutex,
    prelude::*,
    static_lock_class,
    sync::{
        aref::ARef,
        lock::{
            mutex::MutexBackend,
            Guard, //
        },
        Arc,
        Mutex, //
    },
    time::{
        delay::fsleep,
        Delta,
        Instant,
        Monotonic, //
    }, //
};

use crate::debug::*;
use crate::module_parameters;
use crate::no_debug;
use crate::{
    driver::{
        self,
        AsahiDrmDevice,
        AsahiDrmDriver, //
    },
    fw,
    gem,
    hw,
    mem,
    pgtable,
    slotalloc,
    util::RangeExt, //
};

// KernelMapping protection types
pub(crate) use crate::pgtable::Prot;
pub(crate) use pgtable::prot::*;
pub(crate) use pgtable::{
    UatPageTable,
    UAT_PGBIT,
    UAT_PGMSK,
    UAT_PGSZ, //
};

use pgtable::UAT_IAS;

use pin_init;

const DEBUG_CLASS: DebugFlags = DebugFlags::Mmu;

/// PPL magic number for the handoff region
const PPL_MAGIC: u64 = 0x4b1d000000000002;

/// Number of supported context entries in the TTBAT
const UAT_NUM_CTX: usize = 64;
/// First context available for users
const UAT_USER_CTX_START: usize = 1;
/// Number of available user contexts
const UAT_USER_CTX: usize = UAT_NUM_CTX - UAT_USER_CTX_START;

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

/// Address of a special dummy page?
//const IOVA_UNK_PAGE: u64 = 0x6f_ffff8000;
pub(crate) const IOVA_UNK_PAGE: u64 = IOVA_USER_TOP - 2 * UAT_PGSZ as u64;
/// User VA range excluding the unk page
pub(crate) const IOVA_USER_USABLE_RANGE: Range<u64> = IOVA_USER_BASE..IOVA_UNK_PAGE;

/// A pre-allocated memory region for UAT management
struct UatRegion {
    base: PhysicalAddr,
    map: io::mem::Mem,
}

/// SAFETY: It's safe to share UAT region records across threads.
unsafe impl Send for UatRegion {}
/// SAFETY: It's safe to share UAT region records across threads.
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

/// Inner data for a Vm instance.
struct VmInner {
    dev: driver::AsahiDevRef,
    is_kernel: bool,
    page_table: UatPageTable,
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
struct VmBoData {
    #[pin]
    segments: Mutex<Option<KVVec<(DmaAddress, ResourceSize)>>>,
}

/// Preallocated resources needed to execute a VM operation.
///
/// VM operations may require allocating new GPUVA objects to track mappings.
/// To avoid allocation failures during the operation, preallocate the
/// maximum number of GPUVAs that might be needed.
struct VmOpResources {
    /// Preallocated GPUVA objects for remap operations.
    ///
    /// Partial unmap requests or map requests overlapping existing mappings
    /// will trigger a remap call, which needs to register up to three VA
    /// objects (one for the new mapping, and two for the previous and next
    /// mappings).
    preallocated_gpuvas: [Option<GpuVaAlloc<VmInner>>; 3],
}

struct StepContext<'ctx> {
    dev: Option<&'ctx device::Device<Bound>>,
    prot: Prot,

    /// Preallocated resources that can be used when executing the request.
    resources: &'ctx mut VmOpResources,
}

impl<'ctx> StepContext<'ctx> {
    fn new(
        dev: Option<&'ctx device::Device<Bound>>,
        prot: Prot,
        resources: &'ctx mut VmOpResources,
    ) -> Result<StepContext<'ctx>> {
        Ok(Self {
            dev,
            prot,
            resources,
        })
    }

    /// Finds one of our pre-allocated VAs.
    fn preallocated_gpuva(&mut self) -> Result<GpuVaAlloc<VmInner>> {
        self.resources
            .preallocated_gpuvas
            .iter_mut()
            .find_map(|f| f.take())
            .ok_or(EINVAL)
    }

    /// Returns an unused GPUVA object to the preallocated pool.
    /// If the pool is already full, the unused allocation is simply dropped.
    fn return_preallocated_gpuva(&mut self, gpuva: GpuVaAlloc<VmInner>) {
        if let Some(slot) = self
            .resources
            .preallocated_gpuvas
            .iter_mut()
            .find(|slot| slot.is_none())
        {
            *slot = Some(gpuva);
        }
    }
}

impl gpuvm::DriverGpuVm for VmInner {
    type Driver = AsahiDrmDriver;
    type Object = gem::Object;
    type VaData = ();
    type VmBoData = VmBoData;
    type SmContext<'ctx>
        = StepContext<'ctx>
    where
        Self: 'ctx;

    /// Create a new mapping.
    fn sm_step_map<'op, 'ctx>(
        &mut self,
        op: OpMap<'op, Self>,
        ctx: &mut Self::SmContext<'ctx>,
    ) -> Result<OpMapped<'op, Self>, Error> {
        let mut iova = op.addr();
        let mut left = op.length() as usize;
        let mut offset = op.gem_offset() as usize;

        let one_page = op.flags().contains(gpuvm::GpuVaFlag::REPEAT);

        let Some(dev) = ctx.dev else {
            dev_err!(self.dev.as_ref(), "StepContext missed Bound device!");
            return Err(ENODEV);
        };

        let sgt = op.obj().sg_table(dev).inspect_err(|e| {
            dev_err!(
                self.dev.as_ref(),
                "sm_step_map() Failed to get SG table: {:?}",
                e
            );
        })?;
        for sgt_entry in sgt.iter() {
            // TODO: proper DMA address/length handling
            let mut addr = sgt_entry.dma_address() as usize;
            let mut len: usize = sgt_entry.dma_len() as usize;

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

            if one_page {
                len = left;
            } else {
                len = len.min(left);
            }

            mod_dev_dbg!(
                self.dev,
                "MMU: map: {:#x}:{:#x} -> {:#x} [OP={}]\n",
                addr,
                len,
                iova,
                one_page
            );

            self.page_table.map_pages(
                iova..(iova + len as u64),
                addr as PhysicalAddr,
                ctx.prot,
                one_page,
            )?;

            left -= len;
            iova += len as u64;
        }

        let gpuva = ctx.preallocated_gpuva()?;
        let op = op.insert(gpuva, pin_init::init_zeroed());

        Ok(op)
    }

    /// Unmap existing mapping.
    fn sm_step_unmap<'op, 'ctx>(
        &mut self,
        op: OpUnmap<'op, Self>,
        _ctx: &mut Self::SmContext<'ctx>,
    ) -> Result<OpUnmapped<'op, Self>, Error> {
        let iova = op.va().addr();
        let length = op.va().length();

        mod_dev_dbg!(self.dev, "MMU: unmap: {:#x}:{:#x}\n", iova, length);

        self.page_table.unmap_pages(iova..(iova + length))?;

        if let Some(asid) = self.slot() {
            fence(Ordering::SeqCst);
            mem::tlbi_range(asid as u8, iova as usize, length as usize);
            mod_dev_dbg!(
                self.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                iova,
                length,
            );
            mem::sync();
        }

        let (op_unmapped, _va_removed) = op.remove();

        Ok(op_unmapped)
    }

    /// Truncate or split exisiting mapping.
    fn sm_step_remap<'op, 'ctx>(
        &mut self,
        op: OpRemap<'op, Self>,
        ctx: &mut Self::SmContext<'ctx>,
    ) -> Result<OpRemapped<'op, Self>, Error> {
        // Only unmap the hole between prev/next, if they exist
        let unmap_start = if let Some(op) = op.prev() {
            op.addr() + op.length()
        } else {
            op.va_to_unmap().addr()
        };

        let unmap_end = if let Some(op) = op.next() {
            op.addr()
        } else {
            op.va_to_unmap().addr() + op.va_to_unmap().length()
        };

        mod_dev_dbg!(
            self.dev,
            "MMU: unmap for remap: {:#x}..{:#x} (from {:#x}:{:#x})\n",
            unmap_start,
            unmap_end,
            op.va_to_unmap().addr(),
            op.va_to_unmap().addr() + op.va_to_unmap().length()
        );

        let unmap_range = unmap_end - unmap_start;

        self.page_table.unmap_pages(unmap_start..unmap_end)?;

        if let Some(asid) = self.slot() {
            fence(Ordering::SeqCst);
            mem::tlbi_range(asid as u8, unmap_start as usize, unmap_range as usize);
            mod_dev_dbg!(
                self.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}\n",
                asid,
                unmap_start,
                unmap_range,
            );
            mem::sync();
        }

        let prev_gpuva = ctx.preallocated_gpuva()?;
        let next_gpuva = ctx.preallocated_gpuva()?;

        let (op_remapped, remap_ret) = op.remap(
            [prev_gpuva, next_gpuva],
            pin_init::init_zeroed(),
            pin_init::init_zeroed(),
        );

        if let Some(unused_va) = remap_ret.unused_va {
            ctx.return_preallocated_gpuva(unused_va);
        }

        Ok(op_remapped)
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
                .map(|token| token.last_slot() + UAT_USER_CTX_START as u32)
        }
    }

    /// Returns the translation table base for this Vm
    fn ttb(&self) -> u64 {
        self.page_table.ttb()
    }

    /// Map an `mm::Node` representing an mapping in VA space.
    fn map_node(&mut self, node: &mm::Node<(), KernelMappingInner>, prot: Prot) -> Result {
        let mut iova = node.start();
        let segments = node.bo.as_ref().ok_or(EINVAL)?.data().segments.lock();
        let mut offset = node.offset;
        let mut left = node.mapped_size;

        if let Some(segments) = &*segments {
            for (dma_address, dma_len) in segments.iter() {
                if left == 0 {
                    break;
                }

                // TODO: proper DMA address/length handling
                let mut addr = *dma_address as usize;
                let mut len: usize = *dma_len as usize;

                if (offset | addr | len | iova as usize) & UAT_PGMSK != 0 {
                    dev_err!(
                        self.dev.as_ref(),
                        "MMU: KernelMapping {:#x}:{:#x} -> {:#x} is not page-aligned\n",
                        addr,
                        len,
                        iova
                    );
                    return Err(EINVAL);
                }

                if offset > 0 {
                    let skip = len.min(offset);
                    addr += skip;
                    len -= skip;
                    offset -= skip;
                }

                len = len.min(left);

                if len == 0 {
                    continue;
                }

                mod_dev_dbg!(
                    self.dev,
                    "MMU: map: {:#x}:{:#x} -> {:#x}\n",
                    addr,
                    len,
                    iova
                );

                self.page_table.map_pages(
                    iova..(iova + len as u64),
                    addr as PhysicalAddr,
                    prot,
                    false,
                )?;

                iova += len as u64;
                left -= len;
            }
        }
        Ok(())
    }
}

/// Shared reference to a virtual memory address space ([`Vm`]).
#[derive(Clone)]
pub(crate) struct Vm {
    dev: ARef<device::Device>,
    id: u64,
    gpuvm: ARef<gpuvm::GpuVm<VmInner>>,
    dummy_obj: ARef<gem::Object>,
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
    owner: ARef<gpuvm::GpuVm<VmInner>>,
    _gem: Option<ARef<gem::Object>>,
    bo: Option<ARef<gpuvm::GpuVmBo<VmInner>>>,
    uat_inner: Arc<UatInner>,
    prot: Prot,
    offset: usize,
    mapped_size: usize,
}

impl Drop for KernelMappingInner {
    fn drop(self: &mut Self) {
        self.bo = None;
        self.owner.deferred_cleanup();
    }
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

    /// Returns the IOVA base of this mapping
    pub(crate) fn iova_range(&self) -> Range<u64> {
        self.0.start()..(self.0.start() + self.0.mapped_size as u64)
    }

    /// Remap a cached mapping as uncached, then synchronously flush that range of VAs from the
    /// coprocessor cache. This is required to safely unmap cached/private mappings.
    fn remap_uncached_and_flush(&mut self, owner: &mut VmInner) {
        mod_dev_dbg!(
            owner.dev,
            "MMU: remap as uncached {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        // Remap in-place as uncached.
        // Do not try to unmap the guard page (-1)
        let prot = self.0.prot.as_uncached();
        if owner
            .page_table
            .reprot_pages(self.iova_range(), prot)
            .is_err()
        {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: remap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
        }
        fence(Ordering::SeqCst);

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
            dev_err!(
                owner.dev.as_ref(),
                "MMU: Flush too big ({:#x} pages))\n",
                pages
            );
        }

        let cmd = fw::channels::FwCtlMsg {
            addr: fw::types::U64(self.iova()),
            unk_8: 0,
            slot: flush_slot,
            page_count: pages as u16,
            unk_12: 2, // ?
        };

        let dev = owner.dev.clone();
        let guard = dev.registration_guard().expect("Device is not registered!");

        // Tell the firmware to do a cache flush
        guard.registration_data_with(|reg_data| {
            if let Err(e) = reg_data.gpu.fwctl(cmd) {
                dev_err!(
                    owner.dev.as_ref(),
                    "MMU: ASC cache flush {:#x}:{:#x} failed (err: {:?})\n",
                    self.iova(),
                    self.size(),
                    e
                );
            }
        });

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

        let mut vm_exec = unsafe {
            self.0
                .owner
                .exec_lock(None, false)
                .expect("exec_lock failed in KernelMapping::drop")
        };
        let owner = vm_exec.unique_gpuvm.data();

        if self.0.prot.is_cached_noncoherent() {
            mod_pr_debug!(
                "MMU: remap as uncached {:#x}:{:#x}\n",
                self.iova(),
                self.size()
            );
            self.remap_uncached_and_flush(owner);
        }

        mod_dev_dbg!(
            owner.dev,
            "MMU: unmap {:#x}:{:#x}\n",
            self.iova(),
            self.size()
        );

        if owner.page_table.unmap_pages(self.iova_range()).is_err() {
            dev_err!(
                owner.dev.as_ref(),
                "MMU: unmap {:#x}:{:#x} failed\n",
                self.iova(),
                self.size()
            );
        }

        if let Some(asid) = owner.slot() {
            fence(Ordering::SeqCst);
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
        unsafe { (self.handoff_rgn.map.ptr() as *mut Handoff).as_ref() }.unwrap()
    }

    /// Returns the TTBAT area
    fn ttbs(&self) -> &[SlotTTBS; UAT_NUM_CTX] {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.ttbs_rgn.map.ptr() as *mut [SlotTTBS; UAT_NUM_CTX]).as_ref() }.unwrap()
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

    inner: Arc<UatInner>,
    slots: slotalloc::SlotAllocator<SlotInner>,

    kernel_vm: Vm,
    kernel_lower_vm: Vm,
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

        let start = Instant::<Monotonic>::now();
        const TIMEOUT: Delta = Delta::from_millis(1000);

        self.lock();
        while start.elapsed() < TIMEOUT {
            if self.magic_fw.load(Ordering::Relaxed) == PPL_MAGIC {
                break;
            } else {
                self.unlock();
                fsleep(Delta::from_millis(10));
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
        // SAFETY: Per the type invariant, this is safe
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
        // SAFETY: Per the type invariant, this is safe
        let flush = unsafe { self.0.as_ref().unwrap() };
        let state = flush.state.load(Ordering::Relaxed);
        if state != 2 {
            pr_err!("Handoff: expected flush state 2, got {}\n", state);
        }
        flush.state.store(0, Ordering::Relaxed);
    }
}

fn prefetch_sgt_segments(
    dev: &device::Device<Bound>,
    gem: &gem::Object,
) -> Result<KVVec<(DmaAddr, ResourceSize)>> {
    let sgt = gem.sg_table(dev)?;

    // self.segments.
    let mut segments = KVVec::with_capacity(sgt.iter().size_hint().0, GFP_KERNEL)?;
    for sgt_entry in sgt.iter() {
        segments.push((sgt_entry.dma_address(), sgt_entry.dma_len()), GFP_KERNEL)?;
    }

    Ok(segments)
}

impl Vm {
    /// Create a new virtual memory address space
    fn new(
        dev: driver::AsahiDevice,
        uat_inner: Arc<UatInner>,
        kernel_range: Range<u64>,
        cfg: &'static hw::HwConfig,
        ttb: Option<PhysicalAddr>,
        id: u64,
    ) -> Result<Vm> {
        let dummy_obj = gem::new_kernel_object(&dev, UAT_PGSZ)?;
        let is_kernel = ttb.is_some();

        let page_table = if let Some(ttb) = ttb {
            UatPageTable::new_with_ttb(ttb, IOVA_KERN_RANGE, cfg.uat_oas)?
        } else {
            UatPageTable::new(cfg.uat_oas)?
        };

        let (va_range, gpuvm_range) = if is_kernel {
            (IOVA_KERN_RANGE, kernel_range.clone())
        } else {
            (IOVA_USER_RANGE, IOVA_USER_USABLE_RANGE)
        };

        let mm = mm::Allocator::new(va_range.start, va_range.range(), ())?;

        let binding = Arc::pin_init(
            new_mutex!(
                VmBinding {
                    binding: None,
                    bind_token: None,
                    active_users: 0,
                    ttb: page_table.ttb(),
                },
                "VmBinding",
            ),
            GFP_KERNEL,
        )?;

        let unique_gpuvm = gpuvm::GpuVm::new::<Error, _>(
            c"Asahi::GpuVm",
            &dev,
            &*dummy_obj.gem,
            gpuvm_range,
            kernel_range,
            VmInner {
                dev: dev.clone(),
                is_kernel,
                page_table,
                mm,
                uat_inner,
                binding: binding.clone(),
                id,
            },
        )?;
        let gpuvm = ARef::from(&*unique_gpuvm);

        Ok(Vm {
            dev: dev.as_ref().as_ref().into(),
            id,
            gpuvm,
            dummy_obj: dummy_obj.gem.clone(),
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
        gem: &gem::Object,
        object_range: Range<usize>,
        alignment: u64,
        range: Range<u64>,
        prot: Prot,
        guard: bool,
    ) -> Result<KernelMapping> {
        // TODO: hack, populate the GEM object sg table first
        {
            let dev = unsafe { self.dev.as_bound() };
            let _sgt = gem.sg_table(dev)?;
        }
        let vm_bo = self
            .gpuvm
            .obtain(gem, pin_init!(VmBoData { segments <- new_mutex!(None) }))?;

        let size = object_range.range();
        let mut vm_exec = unsafe {
            self.gpuvm.exec_lock(/*Some(gem)*/ None, false)?
        };
        let inner = vm_exec.unique_gpuvm.data();

        let mut vm_bo_guard = vm_bo.data().segments.lock();
        if vm_bo_guard.is_none() {
            let dev = unsafe { self.dev.as_bound() };
            let segments = prefetch_sgt_segments(dev, gem)?;
            vm_bo_guard.replace(segments);
        }
        core::mem::drop(vm_bo_guard);

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.insert_node_in_range(
            KernelMappingInner {
                owner: self.gpuvm.clone(),
                uat_inner,
                prot,
                bo: Some(vm_bo),
                _gem: Some(gem.into()),
                offset: object_range.start,
                mapped_size: size,
            },
            (size + if guard { UAT_PGSZ } else { 0 }) as u64, // Add guard page
            alignment,
            0,
            range.start,
            range.end,
            mm::InsertMode::Best,
        )?;

        let ret = inner.map_node(&node, prot);
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        core::mem::drop(vm_exec);
        ret?;
        Ok(KernelMapping(node))
    }

    /// Map a GEM object into this Vm at a specific address.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn map_at(
        &self,
        addr: u64,
        size: usize,
        gem: &gem::Object,
        prot: Prot,
        guard: bool,
    ) -> Result<KernelMapping> {
        // TODO: hack, populate the GEM object sg table first
        {
            let dev = unsafe { self.dev.as_bound() };
            let _sgt = gem.sg_table(dev)?;
        }
        let vm_bo = self
            .gpuvm
            .obtain(&gem, pin_init!(VmBoData { segments <-new_mutex!(None) }))?;

        // SAFETY: no other UniqRefGpuVm exists so `GpuVm::exec_lock()` is safe
        let mut vm_exec = unsafe {
            self.gpuvm
                .exec_lock(/*Some(&gem)*/ None, false)
                .inspect_err(|_| {
                    dev_err!(self.dev, "map_at(): Failed to exec_lock gpuvm\n");
                })?
        };
        let inner = vm_exec.unique_gpuvm.data();

        let mut vm_bo_guard = vm_bo.data().segments.lock();
        if vm_bo_guard.is_none() {
            let dev = unsafe { self.dev.as_bound() };
            let segments = prefetch_sgt_segments(dev, gem)?;
            vm_bo_guard.replace(segments);
        }
        core::mem::drop(vm_bo_guard);

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.reserve_node(
            KernelMappingInner {
                owner: self.gpuvm.clone(),
                uat_inner,
                prot,
                bo: Some(vm_bo),
                _gem: Some(gem.into()),
                offset: 0,
                mapped_size: size,
            },
            addr,
            (size + if guard { UAT_PGSZ } else { 0 }) as u64, // Add guard page
            0,
        )?;

        let ret = inner.map_node(&node, prot);
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        core::mem::drop(vm_exec);
        ret?;
        Ok(KernelMapping(node))
    }

    /// Map a range of a GEM object into this Vm using GPUVM.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn bind_object<'drm>(
        &self,
        device: &'drm AsahiDrmDevice<Registered>,
        gem: &gem::Object,
        addr: u64,
        size: u64,
        offset: u64,
        prot: Prot,
        single_page: bool,
    ) -> Result {
        // TODO: hack, populate the GEM object sg table first
        {
            let dev = unsafe { self.dev.as_bound() };
            let _sgt = gem.sg_table(dev)?;
        }
        let vm_bo = self
            .gpuvm
            .obtain(gem, pin_init!(VmBoData { segments <- new_mutex!(None) }))?;

        let mut resources = VmOpResources {
            preallocated_gpuvas: [
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
            ],
        };

        // Mapping needs a complete context
        let mut ctx = StepContext::new(Some(device.as_ref().as_ref()), prot, &mut resources)?;

        let mut vm_exec = unsafe {
            self.gpuvm.exec_lock(/*Some(&gem)*/ None, false)?
        };
        let inner = vm_exec.unique_gpuvm.data();

        let mut vm_bo_guard = vm_bo.data().segments.lock();
        if vm_bo_guard.is_none() {
            let dev = unsafe { self.dev.as_bound() };
            let segments = prefetch_sgt_segments(dev, gem)?;
            vm_bo_guard.replace(segments);
        }
        core::mem::drop(vm_bo_guard);

        // Preallocate the page tables, to fail early if we ENOMEM
        inner.page_table.alloc_pages(addr..(addr + size))?;

        if (addr | size | offset) & (UAT_PGMSK as u64) != 0 {
            dev_err!(
                inner.dev.as_ref(),
                "MMU: Map step {:#x} [{:#x}] -> {:#x} is not page-aligned\n",
                offset,
                size,
                addr
            );
            return Err(EINVAL);
        }

        let (flags, gem_range) = if single_page {
            ((GpuVaFlag::REPEAT).into(), UAT_PGSZ as u32)
        } else {
            (GpuVaFlags::empty(), 0u32)
        };
        let _gem_range = 0u32;

        mod_dev_dbg!(
            inner.dev,
            "MMU: sm_map: {:#x} [{:#x}] -> {:#x}\n",
            offset,
            size,
            addr
        );

        let req = OpMapRequest {
            addr,
            range: size,
            gem_offset: offset,
            gem_range,
            vm_bo: &vm_bo,
            context: &mut ctx,
            flags,
        };
        vm_exec.unique_gpuvm.sm_map(req)
    }

    /// Add a direct MMIO mapping to this Vm at a free address.
    pub(crate) fn map_io(
        &self,
        dev: &device::Device,
        iova: u64,
        phys: usize,
        size: usize,
        prot: Prot,
    ) -> Result<KernelMapping> {
        let mut vm_exec = unsafe { self.gpuvm.exec_lock(None, false)? };

        if (iova as usize | phys | size) & UAT_PGMSK != 0 {
            dev_err!(
                dev,
                "MMU: KernelMapping {:#x}:{:#x} -> {:#x} is not page-aligned\n",
                phys,
                size,
                iova
            );
            return Err(EINVAL);
        }

        dev_info!(dev, "MMU: IO map: {:#x}:{:#x} -> {:#x}\n", phys, size, iova);

        let inner = vm_exec.unique_gpuvm.data();
        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.reserve_node(
            KernelMappingInner {
                owner: self.gpuvm.clone(),
                uat_inner,
                prot,
                bo: None,
                _gem: None,
                offset: 0,
                mapped_size: size,
            },
            iova,
            size as u64,
            0,
        )?;

        let ret = inner.page_table.map_pages(
            iova..(iova + size as u64),
            phys as PhysicalAddr,
            prot,
            false,
        );
        // Drop the exec_lock first, so that if map_node failed the
        // KernelMappingInner destructur does not deadlock.
        let _ = inner;
        ret?;
        Ok(KernelMapping(node))
    }

    /// Unmap everything in an address range.
    pub(crate) fn unmap_range<'drm>(
        &self,
        dev: Option<&'drm device::Device<Bound>>,
        iova: u64,
        size: u64,
    ) -> Result {
        // Unmapping a range can only do a single split, so just preallocate
        // two GPUVAs.
        let mut resources = VmOpResources {
            preallocated_gpuvas: [
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                None,
            ],
        };
        let mut ctx = StepContext::new(dev, Prot::default(), &mut resources)?;

        let mut vm_exec = unsafe { self.gpuvm.exec_lock(None, false)? };

        mod_dev_dbg!(self.dev, "MMU: sm_unmap: {:#x}:{:#x}\n", iova, size);
        vm_exec.unique_gpuvm.sm_unmap(iova, size, &mut ctx)
    }

    /// Drop mappings for a given bo.
    pub(crate) fn drop_mappings(&self, gem: &gem::Object) -> Result {
        // Removing whole mappings only does unmaps, so no preallocated VAs
        let mut resources = VmOpResources {
            preallocated_gpuvas: [None, None, None],
        };
        let mut ctx = StepContext::new(None, Prot::default(), &mut resources)?;

        let mut vm_exec = unsafe { self.gpuvm.exec_lock(None, false)? };

        if let Some(bo) = vm_exec.unique_gpuvm.find_bo(gem) {
            mod_dev_dbg!(self.dev, "MMU: bo_unmap\n");
            vm_exec.unique_gpuvm.bo_unmap(&mut ctx, &bo)?;
            mod_dev_dbg!(self.dev, "MMU: bo_unmap done\n");
            // We need to drop the exec_lock first, then the GpuVmBo since that will take the lock itself.
            core::mem::drop(vm_exec);
            core::mem::drop(bo);
        }
        Ok(())
    }

    /// Returns the dummy GEM object used to hold the shared DMA reservation locks
    pub(crate) fn get_resv_obj(&self) -> ARef<gem::Object> {
        self.dummy_obj.clone()
    }

    /// Check whether an object is external to this GpuVm
    pub(crate) fn is_extobj(&self, gem: &gem::Object) -> bool {
        self.gpuvm.is_extobj(gem)
    }

    /// Perform deferred cleanup for this GpuVm
    pub(crate) fn bo_deferred_cleanup(&self) {
        self.gpuvm.deferred_cleanup()
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
        dev: &device::Device,
        name: &CStr,
        size: usize,
        cached: bool,
    ) -> Result<UatRegion> {
        let of_node = dev.of_node().ok_or(EINVAL)?;
        let res = of_node.reserved_mem_region_to_resource_byname(name)?;
        let base = res.start();
        let res_size = res.size().try_into()?;

        if size > res_size {
            dev_err!(
                dev,
                "Region {} is too small (expected {}, got {})\n",
                name,
                size,
                res_size
            );
            return Err(ENOMEM);
        }

        let flags = if cached {
            io::mem::MemFlag::WB
        } else {
            io::mem::MemFlag::WC
        };

        // SAFETY: The safety of this operation hinges on the correctness of
        // much of this file and also the `pgtable` module, so it is difficult
        // to prove in a single safety comment. Such is life with raw GPU
        // page table management...
        let map = unsafe { io::mem::Mem::try_new(res, flags.into()) }.inspect_err(|_| {
            dev_err!(dev, "Failed to remap {} mem resource\n", name);
        })?;

        Ok(UatRegion { base, map })
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

            let isolation = *module_parameters::robust_isolation.value() != 0;

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
                ttbs[idx].ttb0.store(ttb, Ordering::Release);
                ttbs[idx].ttb1.store(ttb1, Ordering::Release);
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
            self.dev.clone(),
            self.inner.clone(),
            kernel_range,
            self.cfg,
            None,
            id,
        )
    }

    /// Creates the reference-counted inner data for a new `Uat` instance.
    #[inline(never)]
    fn make_inner(dev: driver::AsahiDevice) -> Result<Arc<UatInner>> {
        let handoff_rgn = Self::map_region(dev.as_ref().as_ref(), c"handoff", HANDOFF_SIZE, true)?;
        let ttbs_rgn = Self::map_region(dev.as_ref().as_ref(), c"ttbs", SLOTS_SIZE, true)?;

        // SAFETY: The Handoff struct layout matches the firmware's view of memory at this address,
        // and the region is at least large enough per the size specified above.
        let handoff = unsafe { &(handoff_rgn.map.ptr() as *mut Handoff).as_ref().unwrap() };

        dev_info!(dev.as_ref(), "MMU: Initializing kernel page table\n");

        Arc::pin_init(
            try_pin_init!(UatInner {
                handoff_flush <- pin_init::pin_init_array_from_fn(|i| {
                    new_mutex!(HandoffFlush(&handoff.flush[i]), "handoff_flush")
                }),
                shared <- new_mutex!(
                    UatShared {
                        kernel_ttb1: 0,
                        map_kernel_to_user: false,
                        handoff_rgn,
                        ttbs_rgn,
                    },
                    "uat_shared"
                ),
            }),
            GFP_KERNEL,
        )
    }

    /// Creates a new `Uat` instance given the relevant hardware config.
    #[inline(never)]
    pub(crate) fn new(
        dev: driver::AsahiDevice,
        cfg: &'static hw::HwConfig,
        map_kernel_to_user: bool,
    ) -> Result<Self> {
        dev_info!(dev.as_ref(), "MMU: Initializing...\n");

        let inner = Self::make_inner(dev.clone())?;

        let of_node = dev.as_ref().as_ref().of_node().ok_or(EINVAL)?;
        let res = of_node.reserved_mem_region_to_resource_byname(c_str!("pagetables"))?;
        let ttb1 = res.start();
        let ttb1size: usize = res.size().try_into()?;

        if ttb1size < PAGETABLES_SIZE {
            dev_err!(dev.as_ref(), "MMU: Pagetables region is too small\n");
            return Err(ENOMEM);
        }

        dev_info!(dev.as_ref(), "MMU: Creating kernel page tables\n");
        let kernel_lower_vm = Vm::new(dev.clone(), inner.clone(), IOVA_USER_RANGE, cfg, None, 1)?;
        let kernel_vm = Vm::new(
            dev.clone(),
            inner.clone(),
            IOVA_KERN_RANGE,
            cfg,
            Some(ttb1),
            0,
        )?;

        dev_info!(dev.as_ref(), "MMU: Kernel page tables created\n");

        let ttb0 = kernel_lower_vm.ttb();

        let uat = Self {
            dev: dev.clone(),
            cfg,
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
        inner.kernel_ttb1 = ttb1;

        inner.handoff().init()?;

        dev_info!(dev.as_ref(), "MMU: Initializing TTBs\n");

        inner.handoff().lock();

        let ttbs = inner.ttbs();

        ttbs[0].ttb0.store(ttb0 | TTBR_VALID, Ordering::SeqCst);
        ttbs[0].ttb1.store(ttb1 | TTBR_VALID, Ordering::SeqCst);

        for ctx in &ttbs[1..] {
            ctx.ttb0.store(0, Ordering::Relaxed);
            ctx.ttb1.store(0, Ordering::Relaxed);
        }

        inner.handoff().unlock();

        core::mem::drop(inner);

        dev_info!(dev.as_ref(), "MMU: initialized\n");

        Ok(uat)
    }
}

impl Drop for Uat {
    fn drop(&mut self) {
        // Make sure we flush the TLBs
        fence(Ordering::SeqCst);
        mem::tlbi_all();
        mem::sync();
    }
}
