// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple SEP driver
//!
//! Copyright (C) The Asahi Linux Contributors

use core::sync::atomic::{AtomicBool, Ordering};

use kernel::{
    bindings, c_str, device, dma, module_platform_driver, new_mutex, of, platform,
    prelude::*,
    soc::apple::mailbox::{MailCallback, Mailbox, Message},
    sync::{Arc, Mutex},
    types::{ARef, ForeignOwnable},
    workqueue::{self, impl_has_work, new_work, Work, WorkItem},
};

const SHMEM_SIZE: usize = 0x30000;
const MSG_BOOT_TZ0: u64 = 0x5;
const MSG_BOOT_IMG4: u64 = 0x6;
const MSG_SET_SHMEM: u64 = 0x18;
const MSG_BOOT_TZ0_ACK1: u64 = 0x69;
const MSG_BOOT_TZ0_ACK2: u64 = 0xD2;
const MSG_BOOT_IMG4_ACK: u64 = 0x6A;
const MSG_ADVERTISE_EP: u64 = 0;
const EP_DISCOVER: u64 = 0xFD;
const EP_SHMEM: u64 = 0xFE;
const EP_BOOT: u64 = 0xFF;

const MSG_TYPE_SHIFT: u32 = 16;
const MSG_TYPE_MASK: u64 = 0xFF;
//const MSG_PARAM_SHIFT: u32 = 24;
//const MSG_PARAM_MASK: u64 = 0xFF;

const MSG_EP_MASK: u64 = 0xFF;
const MSG_DATA_SHIFT: u32 = 32;

const IOVA_SHIFT: u32 = 0xC;

type ShMem = dma::CoherentAllocation<u8>;

fn align_up(v: usize, a: usize) -> usize {
    (v + a - 1) & !(a - 1)
}

fn memcpy_to_iomem(iomem: &ShMem, off: usize, src: &[u8]) -> Result<()> {
    // SAFETY:
    // as_slice_mut() checks that off and src.len() are whithin iomem's limits.
    // memcpy_to_iomem is only called from within probe() ansuring there are no
    // concurrent read and write accesses to the same region while the slice is
    // alive per as_slice_mut()'s requiremnts.
    unsafe {
        let target = iomem.as_slice_mut(off, src.len())?;
        target.copy_from_slice(src);
    }
    Ok(())
}

fn build_shmem(dev: &platform::Device<device::Core>) -> Result<ShMem> {
    let fwnode = dev.as_ref().fwnode().ok_or(EIO)?;
    let iomem =
        dma::CoherentAllocation::<u8>::alloc_coherent(dev.as_ref(), SHMEM_SIZE, GFP_KERNEL)?;

    let panic_offset = 0x4000;
    let panic_size = 0x8000;
    memcpy_to_iomem(&iomem, panic_offset, &1u32.to_le_bytes())?;

    let lpol_offset = panic_offset + panic_size;
    let lpol_prop_name = c_str!("local-policy-manifest");
    let lpol_prop_size = fwnode.property_count_elem::<u8>(lpol_prop_name)?;
    let lpol = fwnode
        .property_read_array_vec(lpol_prop_name, lpol_prop_size)?
        .required_by(dev.as_ref())?;
    memcpy_to_iomem(&iomem, lpol_offset, &(lpol_prop_size as u32).to_le_bytes())?;
    memcpy_to_iomem(&iomem, lpol_offset + 4, &lpol)?;
    let lpol_size = align_up(lpol_prop_size + 4, 0x4000);

    let ibot_offset = lpol_offset + lpol_size;
    let ibot_prop_name = c_str!("iboot-manifest");
    let ibot_prop_size = fwnode.property_count_elem::<u8>(ibot_prop_name)?;
    let ibot = fwnode
        .property_read_array_vec(ibot_prop_name, ibot_prop_size)?
        .required_by(dev.as_ref())?;
    memcpy_to_iomem(&iomem, ibot_offset, &(ibot_prop_size as u32).to_le_bytes())?;
    memcpy_to_iomem(&iomem, ibot_offset + 4, &ibot)?;
    let ibot_size = align_up(ibot_prop_size + 4, 0x4000);

    memcpy_to_iomem(&iomem, 0, b"CNIP")?;
    memcpy_to_iomem(&iomem, 4, &(panic_size as u32).to_le_bytes())?;
    memcpy_to_iomem(&iomem, 8, &(panic_offset as u32).to_le_bytes())?;

    memcpy_to_iomem(&iomem, 16, b"OPLA")?;
    memcpy_to_iomem(&iomem, 16 + 4, &(lpol_size as u32).to_le_bytes())?;
    memcpy_to_iomem(&iomem, 16 + 8, &(lpol_offset as u32).to_le_bytes())?;

    memcpy_to_iomem(&iomem, 32, b"IPIS")?;
    memcpy_to_iomem(&iomem, 32 + 4, &(ibot_size as u32).to_le_bytes())?;
    memcpy_to_iomem(&iomem, 32 + 8, &(ibot_offset as u32).to_le_bytes())?;

    memcpy_to_iomem(&iomem, 48, b"llun")?;
    Ok(iomem)
}

#[pin_data]
struct SepReceiveWork {
    data: Arc<SepData>,
    msg: Message,
    #[pin]
    work: Work<SepReceiveWork>,
}

impl_has_work! {
    impl HasWork<Self, 0> for SepReceiveWork { self.work }
}

impl SepReceiveWork {
    fn new(data: Arc<SepData>, msg: Message) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(SepReceiveWork {
                data,
                msg,
                work <- new_work!("SepReceiveWork::work"),
            }),
            GFP_ATOMIC,
        )
    }
}

impl WorkItem for SepReceiveWork {
    type Pointer = Arc<SepReceiveWork>;

    fn run(this: Arc<SepReceiveWork>) {
        this.data.process_message(this.msg);
    }
}

struct FwRegionParams {
    addr: u64,
    size: usize,
}

#[pin_data]
struct SepData {
    dev: ARef<device::Device>,
    #[pin]
    mbox: Mutex<Option<Mailbox<SepData>>>,
    shmem: ShMem,
    region_params: FwRegionParams,
    fw_mapped: AtomicBool,
}

impl SepData {
    fn new(
        dev: &platform::Device<device::Core>,
        region_params: FwRegionParams,
    ) -> Result<Arc<SepData>> {
        Arc::pin_init(
            try_pin_init!(SepData {
                shmem: build_shmem(dev)?,
                dev: ARef::<device::Device>::from(dev.as_ref()),
                mbox <- new_mutex!(None),
                region_params,
                fw_mapped: AtomicBool::new(false),
            }),
            GFP_KERNEL,
        )
    }
    fn start(&self) -> Result<()> {
        self.mbox.lock().as_ref().unwrap().send(
            Message {
                msg0: EP_BOOT | (MSG_BOOT_TZ0 << MSG_TYPE_SHIFT),
                msg1: 0,
            },
            false,
        )
    }
    fn load_fw_and_shmem(&self) -> Result<()> {
        let fw_addr = unsafe {
            let res = bindings::dma_map_resource(
                self.dev.as_raw(),
                self.region_params.addr,
                self.region_params.size,
                bindings::dma_data_direction_DMA_TO_DEVICE,
                0,
            );
            if bindings::dma_mapping_error(self.dev.as_raw(), res) != 0 {
                dev_err!(self.dev, "Failed to map firmware");
                return Err(ENOMEM);
            }
            self.fw_mapped.store(true, Ordering::Relaxed);
            res >> IOVA_SHIFT
        };
        let guard = self.mbox.lock();
        let mbox = guard.as_ref().unwrap();
        mbox.send(
            Message {
                msg0: EP_BOOT | (MSG_BOOT_IMG4 << MSG_TYPE_SHIFT) | (fw_addr << MSG_DATA_SHIFT),
                msg1: 0,
            },
            false,
        )?;
        let shm_addr = self.shmem.dma_handle() >> IOVA_SHIFT;
        mbox.send(
            Message {
                msg0: EP_SHMEM | (MSG_SET_SHMEM << MSG_TYPE_SHIFT) | (shm_addr << MSG_DATA_SHIFT),
                msg1: 0,
            },
            false,
        )?;
        Ok(())
    }
    fn process_boot_msg(&self, msg: Message) {
        let ty = (msg.msg0 >> MSG_TYPE_SHIFT) & MSG_TYPE_MASK;
        match ty {
            MSG_BOOT_TZ0_ACK1 => {}
            MSG_BOOT_TZ0_ACK2 => {
                let res = self.load_fw_and_shmem();
                if let Err(e) = res {
                    dev_err!(self.dev, "Unable to load firmware: {:?}", e);
                }
            }
            MSG_BOOT_IMG4_ACK => {}
            _ => {
                dev_err!(self.dev, "Unknown boot message type: {}", ty);
            }
        }
    }
    fn process_discover_msg(&self, msg: Message) {
        let ty = (msg.msg0 >> MSG_TYPE_SHIFT) & MSG_TYPE_MASK;
        //let data = (msg.msg0 >> MSG_DATA_SHIFT) as u32;
        //let param = (msg.msg0 >> MSG_PARAM_SHIFT) & MSG_PARAM_MASK;
        match ty {
            MSG_ADVERTISE_EP => {
                /*dev_info!(
                    self.dev,
                    "Got endpoint {:?} at {}",
                    core::str::from_utf8(&data.to_be_bytes()),
                    param
                );*/
            }
            _ => {
                //dev_warn!(self.dev, "Unknown discovery message type: {}", ty);
            }
        }
    }
    fn process_message(&self, msg: Message) {
        let ep = msg.msg0 & MSG_EP_MASK;
        match ep {
            EP_BOOT => self.process_boot_msg(msg),
            EP_DISCOVER => self.process_discover_msg(msg),
            _ => {} // dev_warn!(self.dev, "Message from unknown endpoint: {}", ep),
        }
    }
    fn remove(&self) {
        *self.mbox.lock() = None;
        if self.fw_mapped.load(Ordering::Relaxed) {
            unsafe {
                bindings::dma_unmap_resource(
                    self.dev.as_raw(),
                    self.region_params.addr,
                    self.region_params.size,
                    bindings::dma_data_direction_DMA_TO_DEVICE,
                    0,
                );
            }
        }
    }
}

impl MailCallback for SepData {
    type Data = Arc<SepData>;
    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, msg: Message) {
        let work = SepReceiveWork::new(data.into(), msg);
        if let Ok(work) = work {
            let res = workqueue::system().enqueue(work);
            if res.is_err() {
                dev_err!(
                    data.dev,
                    "Unable to schedule work item for message {}",
                    msg.msg0
                );
            }
        } else {
            dev_err!(
                data.dev,
                "Unable to allocate work item for message {}",
                msg.msg0
            );
        }
    }
}

unsafe impl Send for SepData {}
unsafe impl Sync for SepData {}

struct SepDriver(Arc<SepData>);

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    (),
    [(of::DeviceId::new(c_str!("apple,sep")), ())]
);

impl platform::Driver for SepDriver {
    type IdInfo = ();

    const OF_ID_TABLE: Option<of::IdTable<()>> = Some(&OF_TABLE);

    fn probe(
        pdev: &platform::Device<device::Core>,
        _info: Option<&()>,
    ) -> Result<Pin<KBox<SepDriver>>> {
        let of = pdev.as_ref().of_node().ok_or(EIO)?;
        let res = of.reserved_mem_region_to_resource_byname(c_str!("sepfw"))?;
        let data = SepData::new(
            pdev,
            FwRegionParams {
                addr: res.start(),
                size: res.size().try_into()?,
            },
        )?;
        *data.mbox.lock() = Some(Mailbox::new_byname(
            pdev.as_ref(),
            c_str!("mbox"),
            data.clone(),
        )?);
        data.start()?;
        Ok(KBox::pin(SepDriver(data), GFP_KERNEL)?)
    }
}

impl Drop for SepDriver {
    fn drop(&mut self) {
        self.0.remove();
    }
}

module_platform_driver! {
    type: SepDriver,
    name: "apple_sep",
    description: "Secure enclave processor stub driver",
    license: "Dual MIT/GPL",
}
