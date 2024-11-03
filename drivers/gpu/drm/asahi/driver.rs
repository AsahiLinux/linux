// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Top-level GPU driver implementation.

use kernel::{
    c_str, device, drm, drm::drv, drm::ioctl, error::code, error::Result, of, platform, prelude::*,
    sync::Arc,
};

use crate::{debug, file, gem, gpu, hw, regs};

use kernel::macros::vtable;
use kernel::types::ARef;

/// Driver metadata
const INFO: drv::DriverInfo = drv::DriverInfo {
    major: 0,
    minor: 0,
    patchlevel: 0,
    name: c_str!("asahi"),
    desc: c_str!("Apple AGX Graphics"),
    date: c_str!("20220831"),
};

/// Device data for the driver registration.
///
/// Holds a reference to the top-level `GpuManager` object.
pub(crate) struct AsahiData {
    pub(crate) dev: ARef<device::Device>,
    pub(crate) gpu: Arc<dyn gpu::GpuManager>,
}

/// Convenience type alias for the `device::Data` type for this driver.
type DeviceData = device::Data<drv::Registration<AsahiDriver>, regs::Resources, AsahiData>;

/// Empty struct representing this driver.
pub(crate) struct AsahiDriver;

/// Convenience type alias for the DRM device type for this driver.
pub(crate) type AsahiDevice = kernel::drm::device::Device<AsahiDriver>;
pub(crate) type AsahiDevRef = ARef<AsahiDevice>;

/// DRM Driver implementation for `AsahiDriver`.
#[vtable]
impl drv::Driver for AsahiDriver {
    /// Our `DeviceData` type, reference-counted
    type Data = Arc<DeviceData>;
    /// Our `File` type.
    type File = file::File;
    /// Our `Object` type.
    type Object = gem::Object;

    const INFO: drv::DriverInfo = INFO;
    const FEATURES: u32 = drv::FEAT_GEM
        | drv::FEAT_RENDER
        | drv::FEAT_SYNCOBJ
        | drv::FEAT_SYNCOBJ_TIMELINE
        | drv::FEAT_GEM_GPUVA;

    kernel::declare_drm_ioctls! {
        (ASAHI_GET_PARAMS,      drm_asahi_get_params,
                          ioctl::RENDER_ALLOW, crate::file::File::get_params),
        (ASAHI_VM_CREATE,       drm_asahi_vm_create,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::vm_create),
        (ASAHI_VM_DESTROY,      drm_asahi_vm_destroy,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::vm_destroy),
        (ASAHI_GEM_CREATE,      drm_asahi_gem_create,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_create),
        (ASAHI_GEM_MMAP_OFFSET, drm_asahi_gem_mmap_offset,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_mmap_offset),
        (ASAHI_GEM_BIND,        drm_asahi_gem_bind,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::gem_bind),
        (ASAHI_QUEUE_CREATE,    drm_asahi_queue_create,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::queue_create),
        (ASAHI_QUEUE_DESTROY,   drm_asahi_queue_destroy,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::queue_destroy),
        (ASAHI_SUBMIT,          drm_asahi_submit,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::submit),
        (ASAHI_GET_TIME,        drm_asahi_get_time,
            ioctl::AUTH | ioctl::RENDER_ALLOW, crate::file::File::get_time),
    }
}

// OF Device ID table.
kernel::define_of_id_table! {ASAHI_ID_TABLE, &'static hw::HwConfig, [
    (of::DeviceId::Compatible(b"apple,agx-t8103"), Some(&hw::t8103::HWCONFIG)),
    (of::DeviceId::Compatible(b"apple,agx-t8112"), Some(&hw::t8112::HWCONFIG)),
    (of::DeviceId::Compatible(b"apple,agx-t6000"), Some(&hw::t600x::HWCONFIG_T6000)),
    (of::DeviceId::Compatible(b"apple,agx-t6001"), Some(&hw::t600x::HWCONFIG_T6001)),
    (of::DeviceId::Compatible(b"apple,agx-t6002"), Some(&hw::t600x::HWCONFIG_T6002)),
    (of::DeviceId::Compatible(b"apple,agx-t6020"), Some(&hw::t602x::HWCONFIG_T6020)),
    (of::DeviceId::Compatible(b"apple,agx-t6021"), Some(&hw::t602x::HWCONFIG_T6021)),
    (of::DeviceId::Compatible(b"apple,agx-t6022"), Some(&hw::t602x::HWCONFIG_T6022)),
]}

/// Platform Driver implementation for `AsahiDriver`.
impl platform::Driver for AsahiDriver {
    /// Our `DeviceData` type, reference-counted
    type Data = Arc<DeviceData>;
    /// Data associated with each hardware ID.
    type IdInfo = &'static hw::HwConfig;

    // Assign the above OF ID table to this driver.
    kernel::driver_of_id_table!(ASAHI_ID_TABLE);

    /// Device probe function.
    fn probe(
        pdev: &mut platform::Device,
        id_info: Option<&Self::IdInfo>,
    ) -> Result<Arc<DeviceData>> {
        debug::update_debug_flags();

        let dev = device::Device::from_dev(pdev);

        dev_info!(dev, "Probing...\n");

        let cfg = id_info.ok_or(ENODEV)?;

        pdev.set_dma_masks((1 << cfg.uat_oas) - 1)?;

        let res = regs::Resources::new(pdev)?;

        // Initialize misc MMIO
        res.init_mmio()?;

        // Start the coprocessor CPU, so UAT can initialize the handoff
        res.start_cpu()?;

        let node = dev.of_node().ok_or(EIO)?;
        let compat: Vec<u32> = node.get_property(c_str!("apple,firmware-compat"))?;

        let reg = drm::drv::Registration::<AsahiDriver>::new(&dev)?;
        let gpu = match (cfg.gpu_gen, cfg.gpu_variant, compat.as_slice()) {
            (hw::GpuGen::G13, _, &[12, 3, 0]) => {
                gpu::GpuManagerG13V12_3::new(reg.device(), &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G14, hw::GpuVariant::G, &[12, 4, 0]) => {
                gpu::GpuManagerG14V12_4::new(reg.device(), &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G13, _, &[13, 5, 0]) => {
                gpu::GpuManagerG13V13_5::new(reg.device(), &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G14, hw::GpuVariant::G, &[13, 5, 0]) => {
                gpu::GpuManagerG14V13_5::new(reg.device(), &res, cfg)? as Arc<dyn gpu::GpuManager>
            }
            (hw::GpuGen::G14, _, &[13, 5, 0]) => {
                gpu::GpuManagerG14XV13_5::new(reg.device(), &res, cfg)? as Arc<dyn gpu::GpuManager>
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

        let data =
            kernel::new_device_data!(reg, res, AsahiData { dev, gpu }, "Asahi::Registrations")?;

        let data: Arc<DeviceData> = data.into();

        data.gpu.init()?;

        kernel::drm_device_register!(
            // TODO: Expose an API to get a pinned reference here
            unsafe { Pin::new_unchecked(&mut *data.registrations().ok_or(ENXIO)?) },
            data.clone(),
            0
        )?;

        dev_info!(data.dev, "Probed!\n");
        Ok(data)
    }
}

// Export the OF ID table as a module ID table, to make modpost/autoloading work.
kernel::module_of_id_table!(MOD_TABLE, ASAHI_ID_TABLE);
