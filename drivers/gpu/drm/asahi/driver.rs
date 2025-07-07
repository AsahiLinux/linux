// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Top-level GPU driver implementation.

use kernel::{
    c_str, device::Core, drm, drm::ioctl, error::Result, of, platform, prelude::*, sync::Arc,
};

use crate::{debug, file, gem, gpu, hw, regs};

use kernel::macros::vtable;
use kernel::types::ARef;

/// Holds a reference to the top-level `GpuManager` object.
// pub(crate) struct AsahiData {
//     pub(crate) dev: ARef<device::Device>,
//     pub(crate) gpu: Arc<dyn gpu::GpuManager>,
// }

#[pin_data]
pub(crate) struct AsahiData {
    #[pin]
    pub(crate) gpu: Arc<dyn gpu::GpuManager>,
    pub(crate) pdev: ARef<platform::Device>,
    pub(crate) resources: regs::Resources,
}

unsafe impl Send for AsahiData {}
unsafe impl Sync for AsahiData {}

pub(crate) struct AsahiDriver {
    #[allow(dead_code)]
    drm: ARef<drm::Device<Self>>,
}

unsafe impl Send for AsahiDriver {}
unsafe impl Sync for AsahiDriver {}

/// Convenience type alias for the DRM device type for this driver.
pub(crate) type AsahiDevice = drm::device::Device<AsahiDriver>;
pub(crate) type AsahiDevRef = ARef<AsahiDevice>;

/// DRM Driver metadata
const INFO: drm::driver::DriverInfo = drm::driver::DriverInfo {
    major: 0,
    minor: 0,
    patchlevel: 0,
    name: c_str!("asahi"),
    desc: c_str!("Apple AGX Graphics"),
};

/// DRM Driver implementation for `AsahiDriver`.
#[vtable]
impl drm::driver::Driver for AsahiDriver {
    /// Our `DeviceData` type, reference-counted
    type Data = AsahiData;
    /// Our `File` type.
    type File = file::File;
    /// Our `Object` type.
    type Object = gem::AsahiObject;

    const INFO: drm::driver::DriverInfo = INFO;
    const FEATURES: u32 = drm::driver::FEAT_GEM
        | drm::driver::FEAT_RENDER
        | drm::driver::FEAT_SYNCOBJ
        | drm::driver::FEAT_SYNCOBJ_TIMELINE
        | drm::driver::FEAT_GEM_GPUVA;

    kernel::declare_drm_ioctls! {
        (ASAHI_GET_PARAMS,      drm_asahi_get_params,
                          ioctl::RENDER_ALLOW, crate::file::File::get_params),
        (ASAHI_GET_TIME,        drm_asahi_get_time,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::get_time),
        (ASAHI_VM_CREATE,       drm_asahi_vm_create,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::vm_create),
        (ASAHI_VM_DESTROY,      drm_asahi_vm_destroy,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::vm_destroy),
        (ASAHI_VM_BIND,         drm_asahi_vm_bind,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::vm_bind),
        (ASAHI_GEM_CREATE,      drm_asahi_gem_create,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_create),
        (ASAHI_GEM_MMAP_OFFSET, drm_asahi_gem_mmap_offset,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_mmap_offset),
        (ASAHI_GEM_BIND_OBJECT, drm_asahi_gem_bind_object,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_bind_object),
        (ASAHI_QUEUE_CREATE,    drm_asahi_queue_create,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::queue_create),
        (ASAHI_QUEUE_DESTROY,   drm_asahi_queue_destroy,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::queue_destroy),
        (ASAHI_SUBMIT,          drm_asahi_submit,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::submit),
    }
}

// OF Device ID table.s
kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    <AsahiDriver as platform::Driver>::IdInfo,
    [
        (
            of::DeviceId::new(c_str!("apple,agx-t8103")),
            &hw::t8103::HWCONFIG
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t8112")),
            &hw::t8112::HWCONFIG
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6000")),
            &hw::t600x::HWCONFIG_T6000
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6001")),
            &hw::t600x::HWCONFIG_T6001
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6002")),
            &hw::t600x::HWCONFIG_T6002
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6020")),
            &hw::t602x::HWCONFIG_T6020
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6021")),
            &hw::t602x::HWCONFIG_T6021
        ),
        (
            of::DeviceId::new(c_str!("apple,agx-t6022")),
            &hw::t602x::HWCONFIG_T6022
        ),
    ]
);

/// Platform Driver implementation for `AsahiDriver`.
impl platform::Driver for AsahiDriver {
    type IdInfo = &'static hw::HwConfig;
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    /// Device probe function.
    fn probe(
        pdev: &platform::Device<Core>,
        info: Option<&Self::IdInfo>,
    ) -> Result<Pin<KBox<Self>>> {
        debug::update_debug_flags();

        dev_info!(pdev.as_ref(), "Probing...\n");

        let cfg = info.ok_or(ENODEV)?;

        pdev.as_ref()
            .dma_set_mask_and_coherent((1 << cfg.uat_oas) - 1)?;

        let res = regs::Resources::new(pdev)?;

        // Initialize misc MMIO
        res.init_mmio()?;

        // Start the coprocessor CPU, so UAT can initialize the handoff
        res.start_cpu()?;

        let node = pdev.as_ref().of_node().ok_or(EIO)?;
        let compat: KVec<u32> = node.get_property(c_str!("apple,firmware-compat"))?;

        let raw_drm = unsafe { drm::device::Device::<AsahiDriver>::new_uninit(pdev.as_ref())? };

        let drm: AsahiDevRef = unsafe { ARef::from_raw(raw_drm) };

        let gpu = match (cfg.gpu_gen, cfg.gpu_variant, compat.as_slice()) {
            (hw::GpuGen::G13, _, &[12, 3, 0]) => {
                gpu::GpuManagerG13V12_3::new(&drm, &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G14, hw::GpuVariant::G, &[12, 4, 0]) => {
                gpu::GpuManagerG14V12_4::new(&drm, &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G13, _, &[13, 5, 0]) => {
                gpu::GpuManagerG13V13_5::new(&drm, &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G14, hw::GpuVariant::G, &[13, 5, 0]) => {
                gpu::GpuManagerG14V13_5::new(&drm, &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G14, _, &[13, 5, 0]) => {
                gpu::GpuManagerG14XV13_5::new(&drm, &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            _ => {
                dev_info!(
                    pdev.as_ref(),
                    "Unsupported GPU/firmware combination ({:?}, {:?}, {:?})\n",
                    cfg.gpu_gen,
                    cfg.gpu_variant,
                    compat
                );
                return Err(ENODEV);
            }
        };

        let data = try_pin_init!(AsahiData {
            gpu,
            pdev: pdev.into(),
            resources: res,
        });

        let drm = unsafe { AsahiDevice::init_data(raw_drm, data)? };

        (*drm).gpu.init()?;

        drm::driver::Registration::new_foreign_owned(&drm, pdev.as_ref(), 0)?;

        Ok(KBox::new(Self { drm }, GFP_KERNEL)?.into())
    }
}
