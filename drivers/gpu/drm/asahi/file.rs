// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(clippy::unusual_byte_groupings)]

//! File implementation, which represents a single DRM client.
//!
//! This is in charge of managing the resources associated with one GPU client, including an
//! arbitrary number of submission queues and Vm objects, and reporting hardware/driver
//! information to userspace and accepting submissions.

use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::{
    alloc, buffer, driver, gem, mmu, module_parameters, queue,
    util::{align, align_down, gcd, AnyBitPattern, RangeExt, Reader},
};
use core::mem::MaybeUninit;
use core::ops::Range;
use kernel::dma_fence::RawDmaFence;
use kernel::drm::gem::BaseObject;
use kernel::error::code::*;
use kernel::prelude::*;
use kernel::sync::{Arc, Mutex};
use kernel::time::NSEC_PER_SEC;
use kernel::types::ForeignOwnable;
use kernel::uaccess::{UserPtr, UserSlice};
use kernel::{dma_fence, drm, uapi, xarray};

const DEBUG_CLASS: DebugFlags = DebugFlags::File;

pub(crate) const MAX_COMMANDS_PER_SUBMISSION: u32 = 64;

/// A client instance of an `mmu::Vm` address space.
struct Vm {
    ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
    ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
    vm: mmu::Vm,
    kernel_range: Range<u64>,
    _dummy_mapping: mmu::KernelMapping,
}

impl Drop for Vm {
    fn drop(&mut self) {
        // When the user Vm is dropped, unmap everything in the user range
        let left_range = VM_USER_RANGE.start..self.kernel_range.start;
        let right_range = self.kernel_range.end..VM_USER_RANGE.end;

        if !left_range.is_empty()
            && self
                .vm
                .unmap_range(left_range.start, left_range.range())
                .is_err()
        {
            pr_err!("Vm::Drop: vm.unmap_range() failed\n");
        }
        if !right_range.is_empty()
            && self
                .vm
                .unmap_range(right_range.start, right_range.range())
                .is_err()
        {
            pr_err!("Vm::Drop: vm.unmap_range() failed\n");
        }
    }
}

/// Sync object from userspace.
pub(crate) struct SyncItem {
    pub(crate) syncobj: drm::syncobj::SyncObj,
    pub(crate) fence: Option<dma_fence::Fence>,
    pub(crate) chain_fence: Option<dma_fence::FenceChain>,
    pub(crate) timeline_value: u64,
}

impl SyncItem {
    fn parse_one(file: &DrmFile, data: uapi::drm_asahi_sync, out: bool) -> Result<SyncItem> {
        match data.sync_type {
            uapi::drm_asahi_sync_type_DRM_ASAHI_SYNC_SYNCOBJ => {
                if data.timeline_value != 0 {
                    cls_pr_debug!(Errors, "Non-timeline sync object with a nonzero value\n");
                    return Err(EINVAL);
                }
                let syncobj = drm::syncobj::SyncObj::lookup_handle(file, data.handle)?;

                Ok(SyncItem {
                    fence: if out {
                        None
                    } else {
                        Some(syncobj.fence_get().ok_or_else(|| {
                            cls_pr_debug!(Errors, "Failed to get fence from sync object\n");
                            EINVAL
                        })?)
                    },
                    syncobj,
                    chain_fence: None,
                    timeline_value: data.timeline_value,
                })
            }
            uapi::drm_asahi_sync_type_DRM_ASAHI_SYNC_TIMELINE_SYNCOBJ => {
                let syncobj = drm::syncobj::SyncObj::lookup_handle(file, data.handle)?;
                let fence = if out {
                    None
                } else {
                    syncobj
                        .fence_get()
                        .ok_or_else(|| {
                            cls_pr_debug!(
                                Errors,
                                "Failed to get fence from timeline sync object\n"
                            );
                            EINVAL
                        })?
                        .chain_find_seqno(data.timeline_value)?
                };

                Ok(SyncItem {
                    fence,
                    syncobj,
                    chain_fence: if out {
                        Some(dma_fence::FenceChain::new()?)
                    } else {
                        None
                    },
                    timeline_value: data.timeline_value,
                })
            }
            _ => {
                cls_pr_debug!(Errors, "Invalid sync type {}\n", data.sync_type);
                Err(EINVAL)
            }
        }
    }

    fn parse_array(
        file: &DrmFile,
        ptr: u64,
        in_count: u32,
        out_count: u32,
    ) -> Result<KVec<SyncItem>> {
        let count = in_count + out_count;
        let mut vec = KVec::with_capacity(count as usize, GFP_KERNEL)?;

        const STRIDE: usize = core::mem::size_of::<uapi::drm_asahi_sync>();
        let size = STRIDE * count as usize;

        // SAFETY: We only read this once, so there are no TOCTOU issues.
        let mut reader = UserSlice::new(ptr as UserPtr, size).reader();

        for i in 0..count {
            let mut sync: MaybeUninit<uapi::drm_asahi_sync> = MaybeUninit::uninit();

            // SAFETY: The size of `sync` is STRIDE
            reader.read_raw(unsafe {
                core::slice::from_raw_parts_mut(sync.as_mut_ptr() as *mut MaybeUninit<u8>, STRIDE)
            })?;

            // SAFETY: All bit patterns in the struct are valid
            let sync = unsafe { sync.assume_init() };

            vec.push(SyncItem::parse_one(file, sync, i >= in_count)?, GFP_KERNEL)?;
        }

        Ok(vec)
    }
}

pub(crate) enum Object {
    TimestampBuffer(Arc<mmu::KernelMapping>),
}

/// State associated with a client.
pub(crate) struct File {
    id: u64,
    vms: xarray::XArray<KBox<Vm>>,
    queues: xarray::XArray<Arc<Mutex<KBox<dyn queue::Queue>>>>,
    objects: xarray::XArray<KBox<Object>>,
}

/// Convenience type alias for our DRM `File` type.
pub(crate) type DrmFile = drm::file::File<File>;

/// Available VM range for the user
const VM_USER_RANGE: Range<u64> = mmu::IOVA_USER_USABLE_RANGE;

/// Minimum reserved AS for kernel mappings
const VM_KERNEL_MIN_SIZE: u64 = 0x20000000;

impl drm::file::DriverFile for File {
    kernel::define_driver_file_types!(driver::AsahiDriver);

    /// Create a new `File` instance for a fresh client.
    fn open(
        device: &AsahiDevice,
        dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
    ) -> Result<Pin<KBox<Self>>> {
        debug::update_debug_flags();

        let gpu = &dev_data.gpu;
        let id = gpu.ids().file.next();

        mod_dev_dbg!(device, "[File {}]: DRM device opened\n", id);
        Ok(KBox::pin(
            Self {
                id,
                vms: xarray::XArray::new(xarray::flags::ALLOC1),
                queues: xarray::XArray::new(xarray::flags::ALLOC1),
                objects: xarray::XArray::new(xarray::flags::ALLOC1),
            },
            GFP_KERNEL,
        )?)
    }
}

// SAFETY: All bit patterns are valid by construction.
unsafe impl AnyBitPattern for uapi::drm_asahi_gem_bind_op {}

impl File {
    fn vms(self: Pin<&Self>) -> Pin<&xarray::XArray<KBox<Vm>>> {
        // SAFETY: Structural pinned projection for vms.
        // We never move out of this field.
        unsafe { self.map_unchecked(|s| &s.vms) }
    }

    #[allow(clippy::type_complexity)]
    fn queues(self: Pin<&Self>) -> Pin<&xarray::XArray<Arc<Mutex<KBox<dyn queue::Queue>>>>> {
        // SAFETY: Structural pinned projection for queues.
        // We never move out of this field.
        unsafe { self.map_unchecked(|s| &s.queues) }
    }

    fn objects(self: Pin<&Self>) -> Pin<&xarray::XArray<KBox<Object>>> {
        // SAFETY: Structural pinned projection for objects.
        // We never move out of this field.
        unsafe { self.map_unchecked(|s| &s.objects) }
    }

    /// IOCTL: get_param: Get a driver parameter value.
    pub(crate) fn get_params(
        device: &AsahiDevice,
        dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &uapi::drm_asahi_get_params,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(device, "[File {}]: IOCTL: get_params\n", file.inner().id);

        let gpu = &dev_data.gpu;

        if data.param_group != 0 || data.pad != 0 {
            cls_pr_debug!(Errors, "get_params: Invalid arguments\n");
            return Err(EINVAL);
        }

        if gpu.is_crashed() {
            return Err(ENODEV);
        }

        let mut params = uapi::drm_asahi_params_global {
            features: 0,

            gpu_generation: gpu.get_dyncfg().id.gpu_gen as u32,
            gpu_variant: gpu.get_dyncfg().id.gpu_variant as u32,
            gpu_revision: gpu.get_dyncfg().id.gpu_rev as u32,
            chip_id: gpu.get_cfg().chip_id,

            num_dies: gpu.get_cfg().num_dies,
            num_clusters_total: gpu.get_dyncfg().id.num_clusters,
            num_cores_per_cluster: gpu.get_dyncfg().id.num_cores,
            core_masks: [0; uapi::DRM_ASAHI_MAX_CLUSTERS as usize],

            vm_start: VM_USER_RANGE.start,
            vm_end: VM_USER_RANGE.end,
            vm_kernel_min_size: VM_KERNEL_MIN_SIZE,

            max_commands_per_submission: MAX_COMMANDS_PER_SUBMISSION,
            max_attachments: crate::microseq::MAX_ATTACHMENTS as u32,
            max_frequency_khz: gpu.get_dyncfg().pwr.max_frequency_khz(),

            command_timestamp_frequency_hz: 1_000_000_000, // User timestamps always in nanoseconds
        };

        for (i, mask) in gpu.get_dyncfg().id.core_masks.iter().enumerate() {
            *(params.core_masks.get_mut(i).ok_or(EIO)?) = (*mask).into();
        }

        if *module_parameters::fault_control.get() == 0xb {
            params.features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_SOFT_FAULTS as u64;
        }

        let size = core::mem::size_of::<uapi::drm_asahi_params_global>().min(data.size.try_into()?);

        // SAFETY: We only write to this userptr once, so there are no TOCTOU issues.
        let mut params_writer = UserSlice::new(data.pointer as UserPtr, size).writer();

        // SAFETY: `size` is at most the sizeof of `params`
        params_writer.write_slice(unsafe {
            core::slice::from_raw_parts(&params as *const _ as *const u8, size)
        })?;

        Ok(0)
    }

    /// IOCTL: vm_create: Create a new `Vm`.
    pub(crate) fn vm_create(
        device: &AsahiDevice,
        dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_vm_create,
        file: &DrmFile,
    ) -> Result<u32> {
        let kernel_range = data.kernel_start..data.kernel_end;

        // Validate requested kernel range
        if !VM_USER_RANGE.is_superset(kernel_range.clone())
            || kernel_range.range() < VM_KERNEL_MIN_SIZE
            || kernel_range.start & (mmu::UAT_PGMSK as u64) != 0
            || kernel_range.end & (mmu::UAT_PGMSK as u64) != 0
        {
            cls_pr_debug!(Errors, "vm_create: Invalid kernel range\n");
            return Err(EINVAL);
        }

        // Align to buffer::PAGE_SIZE so the allocators are happy
        let kernel_range = align(kernel_range.start, buffer::PAGE_SIZE as u64)
            ..align_down(kernel_range.end, buffer::PAGE_SIZE as u64);

        let kernel_half_size = align_down(kernel_range.range() >> 1, buffer::PAGE_SIZE as u64);
        let kernel_gpu_range = kernel_range.start..(kernel_range.start + kernel_half_size);
        let kernel_gpufw_range = kernel_gpu_range.end..kernel_range.end;

        let gpu = &dev_data.gpu;
        let file_id = file.inner().id;
        let vm = gpu.new_vm(kernel_range.clone())?;

        let resv = file.inner().vms().reserve()?;
        let id: u32 = resv.index().try_into()?;

        mod_dev_dbg!(device, "[File {} VM {}]: VM Create\n", file_id, id);
        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating allocators\n",
            file_id,
            id
        );
        let ualloc = Arc::pin_init(
            Mutex::new(alloc::DefaultAllocator::new(
                device,
                &vm,
                kernel_gpu_range,
                buffer::PAGE_SIZE,
                mmu::PROT_GPU_SHARED_RW,
                512 * 1024,
                true,
                fmt!("File {} VM {} GPU Shared", file_id, id),
                false,
            )?),
            GFP_KERNEL,
        )?;
        let ualloc_priv = Arc::pin_init(
            Mutex::new(alloc::DefaultAllocator::new(
                device,
                &vm,
                kernel_gpufw_range,
                buffer::PAGE_SIZE,
                mmu::PROT_GPU_FW_PRIV_RW,
                64 * 1024,
                true,
                fmt!("File {} VM {} GPU FW Private", file_id, id),
                false,
            )?),
            GFP_KERNEL,
        )?;

        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating dummy object\n",
            file_id,
            id
        );
        let mut dummy_obj = gem::new_kernel_object(device, 0x4000)?;
        dummy_obj.vmap()?.as_mut_slice().fill(0);
        let dummy_mapping =
            dummy_obj.map_at(&vm, mmu::IOVA_UNK_PAGE, mmu::PROT_GPU_SHARED_RW, true)?;

        mod_dev_dbg!(device, "[File {} VM {}]: VM created\n", file_id, id);
        resv.store(KBox::new(
            Vm {
                ualloc,
                ualloc_priv,
                vm,
                kernel_range,
                _dummy_mapping: dummy_mapping,
            },
            GFP_KERNEL,
        )?)?;

        data.vm_id = id;

        Ok(0)
    }

    /// IOCTL: vm_destroy: Destroy a `Vm`.
    pub(crate) fn vm_destroy(
        _device: &AsahiDevice,
        _dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_vm_destroy,
        file: &DrmFile,
    ) -> Result<u32> {
        if file.inner().vms().remove(data.vm_id as usize).is_none() {
            Err(ENOENT)
        } else {
            Ok(0)
        }
    }

    /// IOCTL: gem_create: Create a new GEM object.
    pub(crate) fn gem_create(
        device: &AsahiDevice,
        _dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_gem_create,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_create size={:#x?}\n",
            file.inner().id,
            data.size
        );

        if (data.flags
            & !(uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_WRITEBACK
                | uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE))
            != 0
            || (data.flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE == 0
                && data.vm_id != 0)
        {
            cls_pr_debug!(Errors, "gem_create: Invalid arguments\n");
            return Err(EINVAL);
        }

        let resv_obj = if data.flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE != 0 {
            Some(
                file.inner()
                    .vms()
                    .get(data.vm_id.try_into()?)
                    .ok_or(ENOENT)?
                    .borrow()
                    .vm
                    .get_resv_obj(),
            )
        } else {
            None
        };

        let bo = gem::new_object(device, data.size.try_into()?, data.flags, resv_obj.as_ref())?;

        let handle = bo.gem.create_handle(file)?;
        data.handle = handle;

        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_create size={:#x} handle={:#x?}\n",
            file.inner().id,
            data.size,
            data.handle
        );

        Ok(0)
    }

    /// IOCTL: gem_mmap_offset: Assign an mmap offset to a GEM object.
    pub(crate) fn gem_mmap_offset(
        device: &AsahiDevice,
        _dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_gem_mmap_offset,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_mmap_offset handle={:#x?}\n",
            file.inner().id,
            data.handle
        );

        if data.flags != 0 {
            cls_pr_debug!(Errors, "gem_mmap_offset: Unexpected flags\n");
            return Err(EINVAL);
        }

        let bo = gem::lookup_handle(file, data.handle)?;
        data.offset = bo.gem.create_mmap_offset()?;
        Ok(0)
    }

    /// IOCTL: vm_bind: Map or unmap memory into a Vm.
    pub(crate) fn vm_bind(
        device: &AsahiDevice,
        _dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &uapi::drm_asahi_vm_bind,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {} VM {}]: IOCTL: vm_bind\n",
            file.inner().id,
            data.vm_id,
        );

        if data.stride == 0 || data.pad != 0 {
            cls_pr_debug!(Errors, "vm_bind: Unexpected headers\n");
            return Err(EINVAL);
        }

        let vm_id = data.vm_id.try_into()?;

        let mut vec = KVec::new();
        let size = (data.stride * data.num_binds) as usize;
        let reader = UserSlice::new(data.userptr as UserPtr, size).reader();
        reader.read_all(&mut vec, GFP_KERNEL)?;
        let mut reader = Reader::new(&vec);

        for _i in 0..data.num_binds {
            let bind: uapi::drm_asahi_gem_bind_op = reader.read_up_to(data.stride as usize)?;
            Self::do_gem_bind_unbind(vm_id, &bind, file)?;
        }

        Ok(0)
    }

    pub(crate) fn do_gem_bind_unbind(
        vm_id: usize,
        data: &uapi::drm_asahi_gem_bind_op,
        file: &DrmFile,
    ) -> Result<u32> {
        if (data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_UNBIND) != 0 {
            Self::do_gem_unbind(vm_id, data, file)
        } else {
            Self::do_gem_bind(vm_id, data, file)
        }
    }

    pub(crate) fn do_gem_bind(
        vm_id: usize,
        data: &uapi::drm_asahi_gem_bind_op,
        file: &DrmFile,
    ) -> Result<u32> {
        if (data.addr | data.range | data.offset) as usize & mmu::UAT_PGMSK != 0 {
            cls_pr_debug!(
                Errors,
                "gem_bind: Addr/range/offset not page aligned: {:#x} {:#x}\n",
                data.addr,
                data.range
            );
            return Err(EINVAL); // Must be page aligned
        }

        if (data.flags
            & !(uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_READ
                | uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_WRITE
                | uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_SINGLE_PAGE))
            != 0
        {
            cls_pr_debug!(Errors, "gem_bind: Invalid flags {:#x}\n", data.flags);
            return Err(EINVAL);
        }

        let single_page = data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_SINGLE_PAGE != 0;

        let bo = gem::lookup_handle(file, data.handle)?;

        let start = data.addr;
        let end = data.addr.checked_add(data.range).ok_or(EINVAL)?;
        let range = start..end;

        let bo_accessed_size = if single_page {
            mmu::UAT_PGMSK as u64
        } else {
            data.range
        };
        let end_off = data.offset.checked_add(bo_accessed_size).ok_or(EINVAL)?;
        if end_off as usize > bo.size() {
            return Err(EINVAL);
        }

        if !VM_USER_RANGE.is_superset(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid map range {:#x}..{:#x} (not contained in user range)\n",
                start,
                end
            );
            return Err(EINVAL); // Invalid map range
        }

        let prot = if data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_READ != 0 {
            if data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_WRITE != 0 {
                mmu::PROT_GPU_SHARED_RW
            } else {
                mmu::PROT_GPU_SHARED_RO
            }
        } else if data.flags & uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_WRITE != 0 {
            mmu::PROT_GPU_SHARED_WO
        } else {
            cls_pr_debug!(
                Errors,
                "gem_bind: Must specify read or write (flags: {:#x})\n",
                data.flags
            );
            return Err(EINVAL); // Must specify one of DRM_ASAHI_BIND_{READ,WRITE}
        };

        let guard = file.inner().vms().get(vm_id).ok_or(ENOENT)?;

        // Clone it immediately so we aren't holding the XArray lock
        let vm = guard.borrow().vm.clone();
        let kernel_range = guard.borrow().kernel_range.clone();
        core::mem::drop(guard);

        if kernel_range.overlaps(range) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid map range {:#x}..{:#x} (intrudes in kernel range)\n",
                start,
                end
            );
            return Err(EINVAL);
        }

        vm.bind_object(
            &bo.gem,
            data.addr,
            data.range,
            data.offset,
            prot,
            single_page,
        )?;

        Ok(0)
    }

    pub(crate) fn do_gem_unbind(
        vm_id: usize,
        data: &uapi::drm_asahi_gem_bind_op,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.offset != 0
            || data.flags != uapi::drm_asahi_bind_flags_DRM_ASAHI_BIND_UNBIND
            || data.handle != 0
        {
            cls_pr_debug!(Errors, "gem_unbind: offset/flags/handle not zero\n");
            return Err(EINVAL);
        }

        if (data.addr | data.range) as usize & mmu::UAT_PGMSK != 0 {
            cls_pr_debug!(
                Errors,
                "gem_bind: Addr/range/offset not page aligned: {:#x} {:#x}\n",
                data.addr,
                data.range
            );
            return Err(EINVAL); // Must be page aligned
        }

        let start = data.addr;
        let end = data.addr.checked_add(data.range).ok_or(EINVAL)?;
        let range = start..end;

        if !VM_USER_RANGE.is_superset(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid unmap range {:#x}..{:#x} (not contained in user range)\n",
                start,
                end
            );
            return Err(EINVAL); // Invalid map range
        }

        let guard = file.inner().vms().get(vm_id).ok_or(ENOENT)?;

        // Clone it immediately so we aren't holding the XArray lock
        let vm = guard.borrow().vm.clone();
        let kernel_range = guard.borrow().kernel_range.clone();
        core::mem::drop(guard);

        if kernel_range.overlaps(range.clone()) {
            cls_pr_debug!(
                Errors,
                "gem_bind: Invalid unmap range {:#x}..{:#x} (intrudes in kernel range)\n",
                start,
                end
            );
            return Err(EINVAL);
        }

        vm.unmap_range(range.start, range.range())?;

        Ok(0)
    }

    pub(crate) fn unbind_gem_object(file: &DrmFile, bo: &gem::Object) -> Result {
        let mut index = 0;
        loop {
            let item = file
                .inner()
                .vms()
                .find(index, xarray::XArray::<KBox<Vm>>::MAX);
            match item {
                Some((idx, file_vm)) => {
                    // Clone since we can't hold the xarray spinlock while
                    // calling drop_mappings()
                    let vm = file_vm.borrow().vm.clone();
                    core::mem::drop(file_vm);
                    vm.drop_mappings(bo)?;
                    if idx == xarray::XArray::<KBox<Vm>>::MAX {
                        break;
                    }
                    index = idx + 1;
                }
                None => break,
            }
        }
        Ok(())
    }

    /// IOCTL: gem_bind_object: Map or unmap a GEM object as a special object.
    pub(crate) fn gem_bind_object(
        device: &AsahiDevice,
        dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_gem_bind_object,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {} VM {}]: IOCTL: gem_bind_object op={:?} handle={:#x?} flags={:#x?} {:#x?}:{:#x?} object_handle={:#x?}\n",
            file.inner().id,
            data.vm_id,
            data.op,
            data.handle,
            data.flags,
            data.offset,
            data.range,
            data.object_handle
        );

        if data.pad != 0 {
            cls_pr_debug!(Errors, "gem_bind_object: Unexpected pad\n");
            return Err(EINVAL);
        }

        if data.vm_id != 0 {
            cls_pr_debug!(Errors, "gem_bind_object: Unexpected vm_id\n");
            return Err(EINVAL);
        }

        match data.op {
            uapi::drm_asahi_bind_object_op_DRM_ASAHI_BIND_OBJECT_OP_BIND => {
                Self::do_gem_bind_object(device, dev_data, data, file)
            }
            uapi::drm_asahi_bind_object_op_DRM_ASAHI_BIND_OBJECT_OP_UNBIND => {
                Self::do_gem_unbind_object(device, dev_data, data, file)
            }
            _ => {
                cls_pr_debug!(Errors, "gem_bind_object: Invalid op {}\n", data.op);
                Err(EINVAL)
            }
        }
    }

    pub(crate) fn do_gem_bind_object(
        _device: &AsahiDevice,
        dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_gem_bind_object,
        file: &DrmFile,
    ) -> Result<u32> {
        if (data.range | data.offset) as usize & mmu::UAT_PGMSK != 0 {
            cls_pr_debug!(
                Errors,
                "gem_bind_object: Range/offset not page aligned: {:#x} {:#x}\n",
                data.range,
                data.offset
            );
            return Err(EINVAL); // Must be page aligned
        }

        if data.flags != uapi::drm_asahi_bind_object_flags_DRM_ASAHI_BIND_OBJECT_USAGE_TIMESTAMPS {
            cls_pr_debug!(Errors, "gem_bind_object: Invalid flags {:#x}\n", data.flags);
            return Err(EINVAL);
        }

        let offset = data.offset.try_into()?;
        let end_offset = data
            .offset
            .checked_add(data.range)
            .ok_or(EINVAL)?
            .try_into()?;
        let bo = gem::lookup_handle(file, data.handle)?;

        let mapping = Arc::new(
            dev_data.gpu.map_timestamp_buffer(bo, offset..end_offset)?,
            GFP_KERNEL,
        )?;
        let obj = KBox::new(Object::TimestampBuffer(mapping), GFP_KERNEL)?;
        let handle = file.inner().objects().alloc(obj)? as u64;

        data.object_handle = handle as u32;
        Ok(0)
    }

    pub(crate) fn do_gem_unbind_object(
        _device: &AsahiDevice,
        _dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_gem_bind_object,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.range != 0 || data.offset != 0 {
            cls_pr_debug!(
                Errors,
                "gem_unbind_object: Range/offset not zero: {:#x} {:#x}\n",
                data.range,
                data.offset
            );
            return Err(EINVAL);
        }

        if data.flags != 0 {
            cls_pr_debug!(
                Errors,
                "gem_unbind_object: Invalid flags {:#x}\n",
                data.flags
            );
            return Err(EINVAL);
        }

        if data.handle != 0 {
            cls_pr_debug!(
                Errors,
                "gem_unbind_object: Invalid handle {}\n",
                data.handle
            );
            return Err(EINVAL);
        }

        if file
            .inner()
            .objects()
            .remove(data.object_handle as usize)
            .is_none()
        {
            Err(ENOENT)
        } else {
            Ok(0)
        }
    }

    /// IOCTL: queue_create: Create a new command submission queue of a given type.
    pub(crate) fn queue_create(
        device: &AsahiDevice,
        dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_queue_create,
        file: &DrmFile,
    ) -> Result<u32> {
        let file_id = file.inner().id;

        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating queue prio={:?} flags={:#x?}\n",
            file_id,
            data.vm_id,
            data.priority,
            data.flags,
        );

        if data.flags != 0 || data.priority > uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_REALTIME {
            cls_pr_debug!(Errors, "queue_create: Invalid arguments\n");
            return Err(EINVAL);
        }

        // TODO: Allow with CAP_SYS_NICE
        if data.priority >= uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_HIGH {
            cls_pr_debug!(Errors, "queue_create: Invalid priority\n");
            return Err(EINVAL);
        }

        let resv = file.inner().queues().reserve()?;
        let file_vm = file
            .inner()
            .vms()
            .get(data.vm_id.try_into()?)
            .ok_or(ENOENT)?;
        let vm = file_vm.borrow().vm.clone();
        let ualloc = file_vm.borrow().ualloc.clone();
        let ualloc_priv = file_vm.borrow().ualloc_priv.clone();
        // Drop the vms lock eagerly
        core::mem::drop(file_vm);

        let queue = dev_data.gpu.new_queue(
            vm,
            ualloc,
            ualloc_priv,
            // TODO: Plumb deeper the enum
            uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_REALTIME - data.priority,
            data.usc_exec_base,
        )?;

        data.queue_id = resv.index().try_into()?;
        resv.store(Arc::pin_init(Mutex::new(queue), GFP_KERNEL)?)?;

        Ok(0)
    }

    /// IOCTL: queue_destroy: Destroy a command submission queue.
    pub(crate) fn queue_destroy(
        _device: &AsahiDevice,
        _dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_queue_destroy,
        file: &DrmFile,
    ) -> Result<u32> {
        if file
            .inner()
            .queues()
            .remove(data.queue_id as usize)
            .is_none()
        {
            Err(ENOENT)
        } else {
            Ok(0)
        }
    }

    /// IOCTL: submit: Submit GPU work to a command submission queue.
    pub(crate) fn submit(
        device: &AsahiDevice,
        dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_submit,
        file: &DrmFile,
    ) -> Result<u32> {
        debug::update_debug_flags();

        if data.flags != 0 || data.pad != 0 {
            cls_pr_debug!(Errors, "submit: Invalid arguments\n");
            return Err(EINVAL);
        }

        let gpu = &dev_data.gpu;
        gpu.update_globals();

        // Upgrade to Arc<T> to drop the XArray lock early
        let queue: Arc<Mutex<KBox<dyn queue::Queue>>> = file
            .inner()
            .queues()
            .get(data.queue_id.try_into()?)
            .ok_or(ENOENT)?
            .borrow()
            .into();

        let id = gpu.ids().submission.next();
        mod_dev_dbg!(
            device,
            "[File {} Queue {}]: IOCTL: submit (submission ID: {})\n",
            file.inner().id,
            data.queue_id,
            id
        );

        mod_dev_dbg!(
            device,
            "[File {} Queue {}]: IOCTL: submit({}): Parsing syncs\n",
            file.inner().id,
            data.queue_id,
            id
        );
        let syncs =
            SyncItem::parse_array(file, data.syncs, data.in_sync_count, data.out_sync_count)?;

        mod_dev_dbg!(
            device,
            "[File {} Queue {}]: IOCTL: submit({}): Parsing commands\n",
            file.inner().id,
            data.queue_id,
            id
        );

        let mut vec = KVec::new();

        // Copy the command buffer into the kernel. Because we need to iterate
        // the command buffer twice, we do this in one big copy_from_user to
        // avoid TOCTOU issues.
        let reader = UserSlice::new(data.cmdbuf as UserPtr, data.cmdbuf_size as usize).reader();
        reader.read_all(&mut vec, GFP_KERNEL)?;

        let objects = file.inner().objects();
        let ret = queue
            .lock()
            .submit(id, syncs, data.in_sync_count as usize, &vec, objects);

        match ret {
            Err(ERESTARTSYS) => Err(ERESTARTSYS),
            Err(e) => {
                dev_info!(
                    device.as_ref(),
                    "[File {} Queue {}]: IOCTL: submit failed! (submission ID: {} err: {:?})\n",
                    file.inner().id,
                    data.queue_id,
                    id,
                    e
                );
                Err(e)
            }
            Ok(()) => Ok(0),
        }
    }

    /// IOCTL: get_time: Get the current GPU timer value.
    pub(crate) fn get_time(
        _device: &AsahiDevice,
        dev_data: <Self as drm::file::DriverFile>::BorrowedData<'_>,
        data: &mut uapi::drm_asahi_get_time,
        _file: &DrmFile,
    ) -> Result<u32> {
        if data.flags != 0 {
            cls_pr_debug!(Errors, "get_time: Unexpected flags\n");
            return Err(EINVAL);
        }

        // TODO: Do this on device-init for perf.
        let gpu = &dev_data.gpu;
        let frequency_hz = gpu.get_cfg().base_clock_hz as u64;
        let ts_gcd = gcd(frequency_hz, NSEC_PER_SEC as u64);

        let num = (NSEC_PER_SEC as u64) / ts_gcd;
        let den = frequency_hz / ts_gcd;

        let raw: u64;

        // SAFETY: Assembly only loads the timer
        unsafe {
            core::arch::asm!(
                "mrs {x}, CNTPCT_EL0",
                x = out(reg) raw
            );
        }

        data.gpu_timestamp = (raw * num) / den;

        Ok(0)
    }
}

impl Drop for File {
    fn drop(&mut self) {
        mod_pr_debug!("[File {}]: Closing...\n", self.id);
    }
}
