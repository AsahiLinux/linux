// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple PMPv1 driver
//!
//! Copyright (C) The Asahi Linux Contributors

use kernel::{
    bindings,
    device::{
        self,
        Core,
    },
    platform,
    dma,
    io::{
        Io,
        IoBase,
        IoSysMap, //
    },
    module_platform_driver,
    new_mutex,
    of,
    prelude::*,
    soc::apple::{
        rtkit::{
            self,
            ASC_CPU_CONTROL, //
        },
        pmp_v1_bridge
    },
    sync::{
        Arc,
        aref::ARef,
        Mutex,
        Completion
    },
    types::ForeignOwnable,
    time::msecs_to_jiffies,
};

const ASC_MMIO_SIZE: usize = 0x60000;
const PMP_ENDPOINT: u8 = 0x20;
const OPC_STARTUP: u64 = 0;
const OPC_CONFIGURE: u64 = 0x10;
const OPC_CONFIGURE_ACK: u64 = 0x20;
const OPC_INIT1: u64 = 0x200;
const OPC_INIT1_ACK: u64 = 0x201;
const OPC_INIT2: u64 = 0x202;
const OPC_INIT2_ACK: u64 = 0x203;
const OPC_DEVPWR: u64 = 0x20e;
const OPC_SHIFT: u32 = 44;

enum InitEntry {
    Range { addr: u64, size: u64 },
    Dva { addr: u64 },
}

const INIT_TABLE: &[InitEntry] = &[
    InitEntry::Dva { addr: 0xc0000000 },

    InitEntry::Range { addr: 0x00024000, size: 0x4000 },
    InitEntry::Range { addr: 0x00064000, size: 0x4000 },
    InitEntry::Range { addr: 0x000a4000, size: 0x4000 },
    InitEntry::Range { addr: 0x000e4000, size: 0x4000 },
    InitEntry::Range { addr: 0x00158000, size: 0x4000 },
    InitEntry::Range { addr: 0x00204000, size: 0x4000 },
    InitEntry::Range { addr: 0x00244000, size: 0x4000 },
    InitEntry::Range { addr: 0x00284000, size: 0x4000 },
    InitEntry::Range { addr: 0x002c4000, size: 0x4000 },
    InitEntry::Range { addr: 0x00000000, size: 0x0000 },
    InitEntry::Range { addr: 0x00000000, size: 0x0000 },
    InitEntry::Range { addr: 0x00000000, size: 0x0000 },
    InitEntry::Range { addr: 0x00000000, size: 0x0000 },
    InitEntry::Range { addr: 0x00000000, size: 0x0000 },

    InitEntry::Dva { addr: 0xc1000000 },

    InitEntry::Range { addr: 0x04d10000, size: 0x4000 },

    InitEntry::Dva { addr: 0xc2000000 },

    InitEntry::Range { addr: 0x10058000, size: 0x4000 },
    InitEntry::Range { addr: 0x10158000, size: 0x4000 },
    InitEntry::Range { addr: 0x10258000, size: 0x4000 },
    InitEntry::Range { addr: 0x10358000, size: 0x4000 },
    InitEntry::Range { addr: 0x10e20000, size: 0x60000 },
    InitEntry::Range { addr: 0x10e48000, size: 0x4000 },

    InitEntry::Dva { addr: 0xc3000000 },

    InitEntry::Range { addr: 0x11058000, size: 0x4000 },
    InitEntry::Range { addr: 0x11158000, size: 0x4000 },
    InitEntry::Range { addr: 0x11258000, size: 0x4000 },
    InitEntry::Range { addr: 0x11358000, size: 0x4000 },
    InitEntry::Range { addr: 0x11e20000, size: 0x60000 },
    InitEntry::Range { addr: 0x11e48000, size: 0x4000 },

    InitEntry::Dva { addr: 0xc4000000 },

    InitEntry::Range { addr: 0x3d100000, size: 0x14000 },
    InitEntry::Range { addr: 0x3d128000, size: 0x30000 },
    InitEntry::Range { addr: 0x3d0d8000, size: 0x4000 },

    InitEntry::Dva { addr: 0xc5000000 },

    InitEntry::Range { addr: 0x6b90c000, size: 0x4000 },
    InitEntry::Range { addr: 0x00000000, size: 0x0000 },

    InitEntry::Dva { addr: 0xc0024000 },

    InitEntry::Range { addr: 0x00170000, size: 0x4000 },
    InitEntry::Range { addr: 0x00000000, size: 0x0000 },

    InitEntry::Dva { addr: 0xc6000000 },

    InitEntry::Range { addr: 0x3c100000, size: 0x4000 },

    InitEntry::Dva { addr: 0xc1004000 },

    InitEntry::Range { addr: 0x04e20000, size: 0x4000 },

    InitEntry::Dva { addr: 0xc3074000 },

    InitEntry::Range { addr: 0x11ee0000, size: 0x8000 },
    InitEntry::Range { addr: 0x11ee8000, size: 0x8000 },
    InitEntry::Range { addr: 0x11ef0000, size: 0x8000 },

    InitEntry::Dva { addr: 0xc2074000 },

    InitEntry::Range { addr: 0x10ee0000, size: 0x8000 },

    InitEntry::Dva { addr: 0xc1008000 },

    InitEntry::Range { addr: 0x04d80000, size: 0x8000 },
];

type ShMem = dma::Coherent<[u8]>;

#[pin_data]
struct PmpData {
    dev: ARef<device::Device>,
    #[pin]
    rtkit: Mutex<Option<rtkit::RtKit<PmpData>>>,
    shmem: ShMem,
    #[pin]
    pub ready: Completion,
}

fn build_shmem(dev: &platform::Device<device::Core<'_>>) -> Result<ShMem> {
    dma::Coherent::<u8>::zeroed_slice(dev.as_ref(), 0x10000, GFP_KERNEL)
}

fn send_dram_config(dev: &ARef<device::Device>, shmem: &mut ShMem) -> Result<()> {
   let node = dev.fwnode().ok_or(EIO)?;

    let n_entries = node.property_count_elem::<u8>(c"apple,energy-model-dram-configs")?;

    let dram_config = node
        .property_read_array_vec::<u8>(c"apple,energy-model-dram-configs", n_entries)?
        .required_by(dev)?;

    unsafe { shmem.as_mut()[0x2000..][..dram_config.len()].copy_from_slice(&dram_config); }

    Ok(())
}

fn send_init_config(dev: &ARef<device::Device>, shmem: &mut ShMem) -> Result<u64> {
    send_dram_config(dev, shmem)?;

    let mut maps: KVec<u8> = KVec::<u8>::new();

    let domain = unsafe { bindings::iommu_get_domain_for_dev(dev.as_raw()) };

    let mut dva: u64 = 0;

    for entry in INIT_TABLE {
        match entry {
            InitEntry::Dva { addr } => dva = *addr,
            InitEntry::Range { addr, size } => {
                let addr = (1u64 << 33) + addr;
                if *size == 0 {
                    maps.extend_from_slice(&[0u8; 16], GFP_KERNEL)?;
                    continue;
                }

                dev_info!(dev, "map 0x{:x} -> 0x{:x}", addr, dva);

                unsafe {
                    let err = bindings::iommu_map(
                        domain,
                        dva as usize,
                        addr,
                        *size as usize,
                        (bindings::IOMMU_READ | bindings::IOMMU_WRITE | bindings::IOMMU_MMIO) as i32,
                        bindings::GFP_KERNEL,
                    );

                    if err != 0 {
                        return Err(Error::from_errno(err));
                    }
                }

                maps.extend_from_slice(&dva.to_le_bytes(), GFP_KERNEL)?;
                maps.extend_from_slice(&size.to_le_bytes(), GFP_KERNEL)?;

                dva += size.next_multiple_of(0x4000);
            }
        }
    }

    unsafe { shmem.as_mut()[0xe000..][..maps.len()].copy_from_slice(&maps); }

    Ok(0)
}

impl PmpData {
    fn new(pdev: &platform::Device<Core<'_>>) -> Result<Arc<PmpData>> {
        let dev = pdev.as_ref().into();
        let mut shmem = build_shmem(pdev)?;

        send_init_config(&dev, &mut shmem)?;

        Arc::pin_init(
            try_pin_init!(
                PmpData {
                    dev,
                    rtkit <- new_mutex!(None),
                    shmem,
                    ready <- Completion::new(),
                }
            ),
            GFP_KERNEL,
        )
    }

    fn start(&self) -> Result<()> {
        let mut guard = self.rtkit.lock();
        let mut rtk = guard.as_mut().as_pin_mut().unwrap();
        rtk.as_mut().wake()?;
        rtk.start_endpoint(PMP_ENDPOINT)
    }

    fn startup(&self) -> Result<u64> {
        let configure_msg = (OPC_CONFIGURE << OPC_SHIFT) | self.shmem.dma_handle();

        Ok(configure_msg)
    }

    fn recv_message(&self, msg: u64) -> Result<()> {
        let opc = msg >> OPC_SHIFT;
        let reply = match opc {
            OPC_STARTUP => self.startup()?,
            OPC_CONFIGURE_ACK =>
                (OPC_INIT1 << OPC_SHIFT) | (1 << 16) + 0x3,
            OPC_INIT1_ACK =>
                (OPC_INIT2 << OPC_SHIFT) | (1 << 16),
            OPC_INIT2_ACK => {
                self.ready.complete_all();
                0
            },
            0x110 => 0,
            _ => {
                dev_err!(self.dev, "Got unknown message 0x{:x}", msg);
                return Err(EIO);
            }
        };

        if reply != 0 {
            let mut rtk_guard = self.rtkit.lock();
            let rtk = rtk_guard.as_mut().as_pin_mut().unwrap();
            rtk.send_message(PMP_ENDPOINT, reply)?;
        }

        Ok(())
    }

    fn rtk_send_devpwr(&self, dev: u64, enable: bool) -> Result<()> {
        let msg = (OPC_DEVPWR << OPC_SHIFT) + (dev << 16) + enable as u64;

        let mut rtk_guard = self.rtkit.lock();
        rtk_guard.as_mut().as_pin_mut().unwrap()
            .send_message(PMP_ENDPOINT, msg)?;

        Ok(())
    }
}

impl pmp_v1_bridge::DevPwrBridge for PmpData {
    fn send_devpwr(&self, dev: u64, enable: bool) -> Result<()> {
        self.rtk_send_devpwr(dev, enable)
    }

    fn ready(&self) -> bool {
        self.ready.wait_for_completion_timeout(msecs_to_jiffies(50))
    }
}

#[repr(transparent)]
struct PmpDriver(Arc<dyn pmp_v1_bridge::DevPwrBridge>);

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    (),
    [(of::DeviceId::new(c"apple,t8103-pmp-v1"), ())]
);

unsafe impl Send for PmpData {}
unsafe impl Sync for PmpData {}

struct NoBuffer;
impl rtkit::Buffer for NoBuffer {
    fn iova(&self) -> Result<usize> {
        unreachable!()
    }

    fn buf(&mut self) -> Result<IoSysMap<'_, u8>> {
        unreachable!()
    }
}

#[vtable]
impl rtkit::Operations for PmpData {
    type Data = Arc<PmpData>;
    type Buffer = NoBuffer;

    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, _ep: u8, msg: u64) {
        let ret = data.recv_message(msg);
        if let Err(e) = ret {
            dev_err!(data.dev, "Failed to handle rtkit message, error: {:?}", e);
        }
    }

    fn crashed(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, _crashlog: Option<&[u8]>) {
        dev_err!(data.dev, "PMP firmware crashed");
    }
}

impl platform::Driver for PmpDriver {
    type IdInfo = ();
    type Data<'bound> = PmpDriver;

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
            pdev: &'bound platform::Device<Core<'_>>,
            _info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let dev: ARef<device::Device> = pdev.as_ref().into();
        let asc_req = pdev.io_request_by_name(c"asc").ok_or(EINVAL)?;
        let asc_mmio = asc_req.iomap_sized::<ASC_MMIO_SIZE>()?;
        let asc_mmio = asc_mmio.as_view().relaxed();

        unsafe {
            let err = bindings::devm_of_platform_populate(dev.as_raw());
            if err != 0 {
                return Err(Error::from_errno(err));
            }
        }

        let data = PmpData::new(pdev)?;

        let rtkit = rtkit::RtKit::<PmpData>::new(&dev, None, 0, data.clone())?;
        data.rtkit.lock().as_mut().set(Some(rtkit));
        asc_mmio.update(ASC_CPU_CONTROL, |r| r.with_const_cpu_run::<1>());
        data.start()?;

        let data = data as Arc<dyn pmp_v1_bridge::DevPwrBridge>;

        Ok(Self(data))
    }
}

module_platform_driver! {
    type: PmpDriver,
    name: "apple_pmp_v1",
    license: "Dual MIT/GPL",
}
