// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(clippy::unusual_byte_groupings)]

//! File implementation, which represents a single DRM client.
//!
//! This is in charge of managing the resources associated with one GPU client, including an
//! arbitrary number of submission queues and Vm objects, and reporting hardware/driver
//! information to userspace and accepting submissions.

use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::{alloc, buffer, driver, gem, mmu, queue};
use core::mem::MaybeUninit;
use kernel::dma_fence::RawDmaFence;
use kernel::drm::gem::BaseObject;
use kernel::io_buffer::{IoBufferReader, IoBufferWriter};
use kernel::prelude::*;
use kernel::sync::{Arc, Mutex};
use kernel::user_ptr::UserSlicePtr;
use kernel::{dma_fence, drm, uapi, xarray};

const DEBUG_CLASS: DebugFlags = DebugFlags::File;

const MAX_SYNCS_PER_SUBMISSION: u32 = 64;
const MAX_COMMANDS_PER_SUBMISSION: u32 = 64;
pub(crate) const MAX_COMMANDS_IN_FLIGHT: u32 = 1024;

/// A client instance of an `mmu::Vm` address space.
struct Vm {
    ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
    ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
    vm: mmu::Vm,
    dummy_obj: gem::ObjectRef,
}

impl Drop for Vm {
    fn drop(&mut self) {
        // Mappings create a reference loop, make sure to break it.
        self.dummy_obj.drop_vm_mappings(self.vm.id());
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
        if data.extensions != 0 {
            return Err(EINVAL);
        }

        match data.sync_type {
            uapi::drm_asahi_sync_type_DRM_ASAHI_SYNC_SYNCOBJ => {
                if data.timeline_value != 0 {
                    return Err(EINVAL);
                }
                let syncobj = drm::syncobj::SyncObj::lookup_handle(file, data.handle)?;

                Ok(SyncItem {
                    fence: if out {
                        None
                    } else {
                        Some(syncobj.fence_get().ok_or(EINVAL)?)
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
                    Some(
                        syncobj
                            .fence_get()
                            .ok_or(EINVAL)?
                            .chain_find_seqno(data.timeline_value)?,
                    )
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
            _ => Err(EINVAL),
        }
    }

    fn parse_array(file: &DrmFile, ptr: u64, count: u32, out: bool) -> Result<Vec<SyncItem>> {
        let mut vec = Vec::try_with_capacity(count as usize)?;

        const STRIDE: usize = core::mem::size_of::<uapi::drm_asahi_sync>();
        let size = STRIDE * count as usize;

        // SAFETY: We only read this once, so there are no TOCTOU issues.
        let mut reader = unsafe { UserSlicePtr::new(ptr as usize as *mut _, size).reader() };

        for _i in 0..count {
            let mut sync: MaybeUninit<uapi::drm_asahi_sync> = MaybeUninit::uninit();

            // SAFETY: The size of `sync` is STRIDE
            unsafe { reader.read_raw(sync.as_mut_ptr() as *mut u8, STRIDE)? };

            // SAFETY: All bit patterns in the struct are valid
            let sync = unsafe { sync.assume_init() };

            vec.try_push(SyncItem::parse_one(file, sync, out)?)?;
        }

        Ok(vec)
    }
}

/// State associated with a client.
pub(crate) struct File {
    id: u64,
    vms: xarray::XArray<Box<Vm>>,
    queues: xarray::XArray<Arc<Mutex<Box<dyn queue::Queue>>>>,
}

/// Convenience type alias for our DRM `File` type.
pub(crate) type DrmFile = drm::file::File<File>;

/// Start address of the 32-bit USC address space.
const VM_SHADER_START: u64 = 0x11_00000000;
/// End address of the 32-bit USC address space.
const VM_SHADER_END: u64 = 0x11_ffffffff;
/// Start address of the general user mapping region.
const VM_USER_START: u64 = 0x20_00000000;
/// End address of the general user mapping region.
const VM_USER_END: u64 = 0x5f_ffffffff;

/// Start address of the kernel-managed GPU-only mapping region.
const VM_DRV_GPU_START: u64 = 0x60_00000000;
/// End address of the kernel-managed GPU-only mapping region.
const VM_DRV_GPU_END: u64 = 0x60_ffffffff;
/// Start address of the kernel-managed GPU/FW shared mapping region.
const VM_DRV_GPUFW_START: u64 = 0x61_00000000;
/// End address of the kernel-managed GPU/FW shared mapping region.
const VM_DRV_GPUFW_END: u64 = 0x61_ffffffff;
/// Address of a special dummy page?
const VM_UNK_PAGE: u64 = 0x6f_ffff8000;

impl drm::file::DriverFile for File {
    type Driver = driver::AsahiDriver;

    /// Create a new `File` instance for a fresh client.
    fn open(device: &AsahiDevice) -> Result<Pin<Box<Self>>> {
        debug::update_debug_flags();

        let gpu = &device.data().gpu;
        let id = gpu.ids().file.next();

        mod_dev_dbg!(device, "[File {}]: DRM device opened\n", id);
        Ok(Box::into_pin(Box::try_new(Self {
            id,
            vms: xarray::XArray::new(xarray::flags::ALLOC1),
            queues: xarray::XArray::new(xarray::flags::ALLOC1),
        })?))
    }
}

impl File {
    fn vms(self: Pin<&Self>) -> Pin<&xarray::XArray<Box<Vm>>> {
        // SAFETY: Structural pinned projection for vms.
        // We never move out of this field.
        unsafe { self.map_unchecked(|s| &s.vms) }
    }

    #[allow(clippy::type_complexity)]
    fn queues(self: Pin<&Self>) -> Pin<&xarray::XArray<Arc<Mutex<Box<dyn queue::Queue>>>>> {
        // SAFETY: Structural pinned projection for queues.
        // We never move out of this field.
        unsafe { self.map_unchecked(|s| &s.queues) }
    }

    /// IOCTL: get_param: Get a driver parameter value.
    pub(crate) fn get_params(
        device: &AsahiDevice,
        data: &mut uapi::drm_asahi_get_params,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(device, "[File {}]: IOCTL: get_params\n", file.inner().id);

        let gpu = &device.data().gpu;

        if data.extensions != 0 || data.param_group != 0 || data.pad != 0 {
            return Err(EINVAL);
        }

        if gpu.is_crashed() {
            return Err(ENODEV);
        }

        let mut params = uapi::drm_asahi_params_global {
            unstable_uabi_version: uapi::DRM_ASAHI_UNSTABLE_UABI_VERSION,
            pad0: 0,

            feat_compat: gpu.get_cfg().gpu_feat_compat,
            feat_incompat: gpu.get_cfg().gpu_feat_incompat,

            gpu_generation: gpu.get_dyncfg().id.gpu_gen as u32,
            gpu_variant: gpu.get_dyncfg().id.gpu_variant as u32,
            gpu_revision: gpu.get_dyncfg().id.gpu_rev as u32,
            chip_id: gpu.get_cfg().chip_id,

            num_dies: gpu.get_cfg().num_dies,
            num_clusters_total: gpu.get_dyncfg().id.num_clusters,
            num_cores_per_cluster: gpu.get_dyncfg().id.num_cores,
            num_frags_per_cluster: gpu.get_dyncfg().id.num_frags,
            num_gps_per_cluster: gpu.get_dyncfg().id.num_gps,
            num_cores_total_active: gpu.get_dyncfg().id.total_active_cores,
            core_masks: [0; uapi::DRM_ASAHI_MAX_CLUSTERS as usize],

            vm_page_size: mmu::UAT_PGSZ as u32,
            pad1: 0,
            vm_user_start: VM_USER_START,
            vm_user_end: VM_USER_END,
            vm_shader_start: VM_SHADER_START,
            vm_shader_end: VM_SHADER_END,

            max_syncs_per_submission: MAX_SYNCS_PER_SUBMISSION,
            max_commands_per_submission: MAX_COMMANDS_PER_SUBMISSION,
            max_commands_in_flight: MAX_COMMANDS_IN_FLIGHT,
            max_attachments: crate::microseq::MAX_ATTACHMENTS as u32,

            timer_frequency_hz: gpu.get_cfg().base_clock_hz,
            min_frequency_khz: gpu.get_dyncfg().pwr.min_frequency_khz(),
            max_frequency_khz: gpu.get_dyncfg().pwr.max_frequency_khz(),
            max_power_mw: gpu.get_dyncfg().pwr.max_power_mw,

            result_render_size: core::mem::size_of::<uapi::drm_asahi_result_render>() as u32,
            result_compute_size: core::mem::size_of::<uapi::drm_asahi_result_compute>() as u32,
        };

        for (i, mask) in gpu.get_dyncfg().id.core_masks.iter().enumerate() {
            *(params.core_masks.get_mut(i).ok_or(EIO)?) = (*mask).try_into()?;
        }

        let size = core::mem::size_of::<uapi::drm_asahi_params_global>().min(data.size.try_into()?);

        // SAFETY: We only write to this userptr once, so there are no TOCTOU issues.
        let mut params_writer =
            unsafe { UserSlicePtr::new(data.pointer as usize as *mut _, size).writer() };

        // SAFETY: `size` is at most the sizeof of `params`
        unsafe { params_writer.write_raw(&params as *const _ as *const u8, size)? };

        Ok(0)
    }

    /// IOCTL: vm_create: Create a new `Vm`.
    pub(crate) fn vm_create(
        device: &AsahiDevice,
        data: &mut uapi::drm_asahi_vm_create,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.extensions != 0 {
            return Err(EINVAL);
        }

        let gpu = &device.data().gpu;
        let file_id = file.inner().id;
        let vm = gpu.new_vm(file_id)?;

        let resv = file.inner().vms().reserve()?;
        let id: u32 = resv.index().try_into()?;

        mod_dev_dbg!(device, "[File {} VM {}]: VM Create\n", file_id, id);
        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating allocators\n",
            file_id,
            id
        );
        let ualloc = Arc::pin_init(Mutex::new(alloc::DefaultAllocator::new(
            device,
            &vm,
            VM_DRV_GPU_START,
            VM_DRV_GPU_END,
            buffer::PAGE_SIZE,
            mmu::PROT_GPU_SHARED_RW,
            512 * 1024,
            true,
            fmt!("File {} VM {} GPU Shared", file_id, id),
            false,
        )?))?;
        let ualloc_priv = Arc::pin_init(Mutex::new(alloc::DefaultAllocator::new(
            device,
            &vm,
            VM_DRV_GPUFW_START,
            VM_DRV_GPUFW_END,
            buffer::PAGE_SIZE,
            mmu::PROT_GPU_FW_PRIV_RW,
            64 * 1024,
            true,
            fmt!("File {} VM {} GPU FW Private", file_id, id),
            false,
        )?))?;

        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating dummy object\n",
            file_id,
            id
        );
        let mut dummy_obj = gem::new_kernel_object(device, 0x4000)?;
        dummy_obj.vmap()?.as_mut_slice().fill(0);
        dummy_obj.map_at(&vm, VM_UNK_PAGE, mmu::PROT_GPU_SHARED_RW, true)?;

        mod_dev_dbg!(device, "[File {} VM {}]: VM created\n", file_id, id);
        resv.store(Box::try_new(Vm {
            ualloc,
            ualloc_priv,
            vm,
            dummy_obj,
        })?)?;

        data.vm_id = id;

        Ok(0)
    }

    /// IOCTL: vm_destroy: Destroy a `Vm`.
    pub(crate) fn vm_destroy(
        _device: &AsahiDevice,
        data: &mut uapi::drm_asahi_vm_destroy,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.extensions != 0 {
            return Err(EINVAL);
        }

        if file.inner().vms().remove(data.vm_id as usize).is_none() {
            Err(ENOENT)
        } else {
            Ok(0)
        }
    }

    /// IOCTL: gem_create: Create a new GEM object.
    pub(crate) fn gem_create(
        device: &AsahiDevice,
        data: &mut uapi::drm_asahi_gem_create,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_create size={:#x?}\n",
            file.inner().id,
            data.size
        );

        if data.extensions != 0
            || (data.flags & !(uapi::ASAHI_GEM_WRITEBACK | uapi::ASAHI_GEM_VM_PRIVATE)) != 0
            || (data.flags & uapi::ASAHI_GEM_VM_PRIVATE == 0 && data.vm_id != 0)
        {
            return Err(EINVAL);
        }

        let vm_id = if data.flags & uapi::ASAHI_GEM_VM_PRIVATE != 0 {
            Some(
                file.inner()
                    .vms()
                    .get(data.vm_id.try_into()?)
                    .ok_or(ENOENT)?
                    .borrow()
                    .vm
                    .id(),
            )
        } else {
            None
        };

        let bo = gem::new_object(device, data.size.try_into()?, data.flags, vm_id)?;

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
        data: &mut uapi::drm_asahi_gem_mmap_offset,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_mmap_offset handle={:#x?}\n",
            file.inner().id,
            data.handle
        );

        if data.extensions != 0 || data.flags != 0 {
            return Err(EINVAL);
        }

        let bo = gem::lookup_handle(file, data.handle)?;
        data.offset = bo.gem.create_mmap_offset()?;
        Ok(0)
    }

    /// IOCTL: gem_bind: Map or unmap a GEM object into a Vm.
    pub(crate) fn gem_bind(
        device: &AsahiDevice,
        data: &mut uapi::drm_asahi_gem_bind,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {} VM {}]: IOCTL: gem_bind op={:?} handle={:#x?} flags={:#x?} {:#x?}:{:#x?} -> {:#x?}\n",
            file.inner().id,
            data.vm_id,
            data.op,
            data.handle,
            data.flags,
            data.offset,
            data.range,
            data.addr
        );

        if data.extensions != 0 {
            return Err(EINVAL);
        }

        match data.op {
            uapi::drm_asahi_bind_op_ASAHI_BIND_OP_BIND => Self::do_gem_bind(device, data, file),
            uapi::drm_asahi_bind_op_ASAHI_BIND_OP_UNBIND => Err(ENOTSUPP),
            uapi::drm_asahi_bind_op_ASAHI_BIND_OP_UNBIND_ALL => {
                Self::do_gem_unbind_all(device, data, file)
            }
            _ => Err(EINVAL),
        }
    }

    pub(crate) fn do_gem_bind(
        _device: &AsahiDevice,
        data: &mut uapi::drm_asahi_gem_bind,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.offset != 0 {
            return Err(EINVAL); // Not supported yet
        }

        if (data.addr | data.range) as usize & mmu::UAT_PGMSK != 0 {
            return Err(EINVAL); // Must be page aligned
        }

        if (data.flags & !(uapi::ASAHI_BIND_READ | uapi::ASAHI_BIND_WRITE)) != 0 {
            return Err(EINVAL);
        }

        let mut bo = gem::lookup_handle(file, data.handle)?;

        if data.range != bo.size().try_into()? {
            return Err(EINVAL); // Not supported yet
        }

        let start = data.addr;
        let end = data.addr + data.range - 1;

        if (VM_SHADER_START..=VM_SHADER_END).contains(&start) {
            if !(VM_SHADER_START..=VM_SHADER_END).contains(&end) {
                return Err(EINVAL); // Invalid map range
            }
        } else if (VM_USER_START..=VM_USER_END).contains(&start) {
            if !(VM_USER_START..=VM_USER_END).contains(&end) {
                return Err(EINVAL); // Invalid map range
            }
        } else {
            return Err(EINVAL); // Invalid map range
        }

        // Just in case
        if end >= VM_DRV_GPU_START {
            return Err(EINVAL);
        }

        let prot = if data.flags & uapi::ASAHI_BIND_READ != 0 {
            if data.flags & uapi::ASAHI_BIND_WRITE != 0 {
                mmu::PROT_GPU_SHARED_RW
            } else {
                mmu::PROT_GPU_SHARED_RO
            }
        } else if data.flags & uapi::ASAHI_BIND_WRITE != 0 {
            mmu::PROT_GPU_SHARED_WO
        } else {
            return Err(EINVAL); // Must specify one of ASAHI_BIND_{READ,WRITE}
        };

        // Clone it immediately so we aren't holding the XArray lock
        let vm = file
            .inner()
            .vms()
            .get(data.vm_id.try_into()?)
            .ok_or(ENOENT)?
            .borrow()
            .vm
            .clone();

        bo.map_at(&vm, start, prot, true)?;

        Ok(0)
    }

    pub(crate) fn do_gem_unbind_all(
        _device: &AsahiDevice,
        data: &mut uapi::drm_asahi_gem_bind,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.flags != 0 || data.offset != 0 || data.range != 0 || data.addr != 0 {
            return Err(EINVAL);
        }

        let mut bo = gem::lookup_handle(file, data.handle)?;

        if data.vm_id == 0 {
            bo.drop_file_mappings(file.inner().id);
        } else {
            let vm_id = file
                .inner()
                .vms()
                .get(data.vm_id.try_into()?)
                .ok_or(ENOENT)?
                .borrow()
                .vm
                .id();
            bo.drop_vm_mappings(vm_id);
        }

        Ok(0)
    }

    /// IOCTL: queue_create: Create a new command submission queue of a given type.
    pub(crate) fn queue_create(
        device: &AsahiDevice,
        data: &mut uapi::drm_asahi_queue_create,
        file: &DrmFile,
    ) -> Result<u32> {
        let file_id = file.inner().id;

        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating queue caps={:?} prio={:?} flags={:#x?}\n",
            file_id,
            data.vm_id,
            data.queue_caps,
            data.priority,
            data.flags,
        );

        if data.extensions != 0
            || data.flags != 0
            || data.priority > 3
            || data.queue_caps == 0
            || (data.queue_caps
                & !(uapi::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_RENDER
                    | uapi::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_BLIT
                    | uapi::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_COMPUTE))
                != 0
        {
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

        let queue =
            device
                .data()
                .gpu
                .new_queue(vm, ualloc, ualloc_priv, data.priority, data.queue_caps)?;

        data.queue_id = resv.index().try_into()?;
        resv.store(Arc::pin_init(Mutex::new(queue))?)?;

        Ok(0)
    }

    /// IOCTL: queue_destroy: Destroy a command submission queue.
    pub(crate) fn queue_destroy(
        _device: &AsahiDevice,
        data: &mut uapi::drm_asahi_queue_destroy,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.extensions != 0 {
            return Err(EINVAL);
        }

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
        data: &mut uapi::drm_asahi_submit,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.extensions != 0
            || data.flags != 0
            || data.in_sync_count > MAX_SYNCS_PER_SUBMISSION
            || data.out_sync_count > MAX_SYNCS_PER_SUBMISSION
            || data.command_count > MAX_COMMANDS_PER_SUBMISSION
        {
            return Err(EINVAL);
        }

        debug::update_debug_flags();

        let gpu = &device.data().gpu;
        gpu.update_globals();

        // Upgrade to Arc<T> to drop the XArray lock early
        let queue: Arc<Mutex<Box<dyn queue::Queue>>> = file
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
            "[File {} Queue {}]: IOCTL: submit({}): Parsing in_syncs\n",
            file.inner().id,
            data.queue_id,
            id
        );
        let in_syncs = SyncItem::parse_array(file, data.in_syncs, data.in_sync_count, false)?;
        mod_dev_dbg!(
            device,
            "[File {} Queue {}]: IOCTL: submit({}): Parsing out_syncs\n",
            file.inner().id,
            data.queue_id,
            id
        );
        let out_syncs = SyncItem::parse_array(file, data.out_syncs, data.out_sync_count, true)?;

        let result_buf = if data.result_handle != 0 {
            mod_dev_dbg!(
                device,
                "[File {} Queue {}]: IOCTL: submit({}): Looking up result_handle {}\n",
                file.inner().id,
                data.queue_id,
                id,
                data.result_handle
            );
            Some(gem::lookup_handle(file, data.result_handle)?)
        } else {
            None
        };

        mod_dev_dbg!(
            device,
            "[File {} Queue {}]: IOCTL: submit({}): Parsing commands\n",
            file.inner().id,
            data.queue_id,
            id
        );
        let mut commands = Vec::try_with_capacity(data.command_count as usize)?;

        const STRIDE: usize = core::mem::size_of::<uapi::drm_asahi_command>();
        let size = STRIDE * data.command_count as usize;

        // SAFETY: We only read this once, so there are no TOCTOU issues.
        let mut reader =
            unsafe { UserSlicePtr::new(data.commands as usize as *mut _, size).reader() };

        for _i in 0..data.command_count {
            let mut cmd: MaybeUninit<uapi::drm_asahi_command> = MaybeUninit::uninit();

            // SAFETY: The size of `sync` is STRIDE
            unsafe { reader.read_raw(cmd.as_mut_ptr() as *mut u8, STRIDE)? };

            // SAFETY: All bit patterns in the struct are valid
            commands.try_push(unsafe { cmd.assume_init() })?;
        }

        let ret = queue
            .lock()
            .submit(id, in_syncs, out_syncs, result_buf, commands);

        match ret {
            Err(ERESTARTSYS) => Err(ERESTARTSYS),
            Err(e) => {
                dev_info!(
                    device,
                    "[File {} Queue {}]: IOCTL: submit failed! (submission ID: {} err: {:?})\n",
                    file.inner().id,
                    data.queue_id,
                    id,
                    e
                );
                Err(e)
            }
            Ok(_) => Ok(0),
        }
    }

    /// Returns the unique file ID for this `File`.
    pub(crate) fn file_id(&self) -> u64 {
        self.id
    }
}

impl Drop for File {
    fn drop(&mut self) {
        mod_pr_debug!("[File {}]: Closing...\n", self.id);
    }
}
