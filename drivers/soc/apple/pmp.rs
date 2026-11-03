// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple PMP driver
//!
//! Copyright (C) The Asahi Linux Contributors

use core::mem;

use kernel::{
    bindings,
    device::{
        self,
        Core, //
    },
    dma::{
        Coherent,
        CoherentBox, //
    },
    io::{
        io_project,
        Io,
        IoBase,
        IoSysMap,
        Mmio,
        Region, //
    },
    kvec,
    module_platform_driver,
    new_mutex,
    of,
    platform,
    prelude::*,
    soc::apple::rtkit::{
        self,
        ASC_CPU_CONTROL, //
    },
    str::CString,
    sync::{
        aref::ARef,
        Arc,
        Mutex, //
    },
    transmute::{
        AsBytes,
        FromBytes, //
    },
    types::ForeignOwnable, //
};

const PMP_MMIO_SIZE: usize = 0x80000;
const ASC_MMIO_SIZE: usize = 0x4000;
const BOOTARGS_OFFSET: usize = 0x22c;
const BOOTARGS_SIZE: usize = 0x230;
const PMP_ENDPOINT: u8 = 0x20;
const OPC_GET_IOVA_TABLE: u64 = 0x10;
const OPC_MALLOC: u64 = 0x12;
const OPC_FREE: u64 = 0x14;
const OPC_SET_BUF: u64 = 0x30;
const OPC_REGISTER_IOREG: u64 = 0x32;
const OPC_SET_IOREG: u64 = 0x34;
const OPC_ACK_MASK: u64 = 0x1;
const OPC_SHIFT: u32 = 48;
const MALLOC_SIZE_MASK: u64 = 0xFFFFFF;
const MSG_IOVA_MASK: u64 = 0xFFFFFFFFFFFF;
const SET_IOREG_INDEX_MASK: u64 = 0xFFFF;
const PIO_VM_BASE: u64 = 0xc0000000;
const PIO_GRANULARITY: u64 = 0x1000000;

const fn from_fourcc(b: &[u8; 4]) -> u32 {
    b[3] as u32 | (b[2] as u32) << 8 | (b[1] as u32) << 16 | (b[0] as u32) << 24
}

struct PmpAllocation {
    addr: u64,
    alloc: Coherent<[u8]>,
}

struct IovaTableEntry {
    _host_addr: u64,
    _pio_base: u64,
    _size: u64,
}

// SAFETY: TODO:
unsafe impl AsBytes for IovaTableEntry {}
// SAFETY: TODO:
unsafe impl FromBytes for IovaTableEntry {}

struct PmpState {
    iova_table: Option<Coherent<[IovaTableEntry]>>,
    allocs: KVec<PmpAllocation>,
    value_buf: Option<u64>,
    ioreg_entries: KVec<u32>,
}

impl PmpState {
    fn new() -> Result<PmpState> {
        Ok(PmpState {
            iova_table: None,
            allocs: KVec::with_capacity(10, GFP_KERNEL)?,
            value_buf: None,
            ioreg_entries: KVec::with_capacity(340, GFP_KERNEL)?,
        })
    }
    fn find_alloc(&self, addr: u64) -> Option<usize> {
        // Due to how pmp manages memory, iterating in reverse will
        // usually result in us getting the right one on the first try
        for (i, e) in self.allocs.iter().enumerate().rev() {
            if e.addr == addr {
                return Some(i);
            }
        }
        None
    }
    fn get_buf(&mut self, addr: u64) -> Option<&mut Coherent<[u8]>> {
        let idx = self.find_alloc(addr)?;
        Some(&mut self.allocs[idx].alloc)
    }
}

#[pin_data]
struct PmpData {
    dev: ARef<device::Device>,
    #[pin]
    rtkit: Mutex<Option<rtkit::RtKit<PmpData>>>,
    #[pin]
    state: Mutex<PmpState>,
}

impl PmpData {
    fn new(dev: &platform::Device<Core<'_>>) -> Result<Arc<PmpData>> {
        Arc::pin_init(
            try_pin_init!(
                PmpData {
                    dev: dev.as_ref().into(),
                    rtkit <- new_mutex!(None),
                    state <- new_mutex!(PmpState::new()?)
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
    fn patch_bootargs(
        &self,
        pmp_mmio: &Mmio<'_, Region<PMP_MMIO_SIZE>>,
        patches: &[(u32, u32)],
    ) -> Result<()> {
        let io = pmp_mmio.as_view().relaxed();
        let offset = io.read32(BOOTARGS_OFFSET) as usize;
        let size = io.read32(BOOTARGS_SIZE) as usize;
        let mut arg_bytes = kvec![0u8; size]?;
        let pmp_io_array: Mmio<'_, [u8; PMP_MMIO_SIZE]> = pmp_mmio.try_cast()?;
        let bootargs = io_project!(pmp_io_array, [try: offset .. offset+size]);
        bootargs.copy_to_slice(&mut arg_bytes);
        let mut idx = 0;
        while idx < size {
            let key = u32::from_le_bytes(arg_bytes[idx..idx + 4].try_into().unwrap());
            let size = u32::from_le_bytes(arg_bytes[idx + 4..idx + 8].try_into().unwrap()) as usize;
            idx += 8;
            for (k, v) in patches.iter() {
                if *k != key {
                    continue;
                }
                arg_bytes[idx..idx + size].copy_from_slice(&(*v as u64).to_le_bytes()[..size]);
                break;
            }
            idx += size;
        }
        bootargs.copy_from_slice(&arg_bytes);
        Ok(())
    }
    fn get_iova_table(&self) -> Result<u64> {
        let mut state = self.state.lock();
        if state.iova_table.is_some() {
            dev_err!(self.dev, "Asked for iova table with existing buffer");
            return Err(EIO);
        }
        let node = self.dev.fwnode().ok_or(EIO)?;
        let mut pio_base = PIO_VM_BASE;
        let prop_name = c"apple,pio-ranges";
        if !node.property_present(prop_name) {
            return Ok((OPC_GET_IOVA_TABLE | OPC_ACK_MASK) << OPC_SHIFT);
        }
        let n_entries = node.property_count_elem::<u64>(prop_name)? / 2;
        let ranges = node
            .property_read_array_vec::<u64>(prop_name, n_entries * 2)?
            .required_by(&self.dev)?;
        // SAFETY: TODO: ensure self.dev is bound
        let bound_dev = unsafe { self.dev.as_bound() };
        let mut table = CoherentBox::<[IovaTableEntry]>::zeroed_slice(bound_dev, 170, GFP_KERNEL)?;

        let domain = unsafe { bindings::iommu_get_domain_for_dev(self.dev.as_raw()) };
        for i in 0..n_entries {
            let host_addr = ranges[i * 2];
            let size = ranges[i * 2 + 1];
            unsafe {
                let err = bindings::iommu_map(
                    domain,
                    pio_base as usize,
                    host_addr,
                    size as usize,
                    (bindings::IOMMU_READ | bindings::IOMMU_WRITE | bindings::IOMMU_MMIO) as i32,
                    bindings::GFP_KERNEL,
                );
                if err != 0 {
                    return Err(Error::from_errno(err));
                }
            }
            table[i] = IovaTableEntry {
                _host_addr: host_addr,
                _pio_base: pio_base,
                _size: size,
            };
            pio_base += PIO_GRANULARITY;
        }
        let table: Coherent<[IovaTableEntry]> = table.into();
        let msg = (OPC_GET_IOVA_TABLE | OPC_ACK_MASK) << OPC_SHIFT | table.dma_handle();
        state.iova_table = Some(table);
        Ok(msg)
    }
    fn malloc(&self, size: u64) -> Result<u64> {
        // SAFETY: TODO: ensure self.dev is bound
        let bound_dev = unsafe { self.dev.as_bound() };
        let iomem = Coherent::<u8>::zeroed_slice(bound_dev, size as usize, GFP_KERNEL)?;
        let mut state = self.state.lock();
        let addr = iomem.dma_handle();
        let msg = (OPC_MALLOC | OPC_ACK_MASK) << OPC_SHIFT | addr;
        state.allocs.push(
            PmpAllocation {
                addr: addr,
                alloc: iomem,
            },
            GFP_KERNEL,
        )?;
        Ok(msg)
    }
    fn free(&self, addr: u64) -> Result<u64> {
        let mut state = self.state.lock();
        if let Some(idx) = state.find_alloc(addr) {
            state.allocs.swap_remove(idx);
        } else {
            dev_err!(
                self.dev,
                "Attempted to free memory that was not allocated {}",
                addr
            );
            return Err(EIO);
        }
        let msg = (OPC_FREE | OPC_ACK_MASK) << OPC_SHIFT;
        Ok(msg)
    }
    fn set_buf(&self, addr: u64) -> Result<u64> {
        let mut state = self.state.lock();
        if state.value_buf.is_some() {
            dev_err!(self.dev, "Setting a buffer when one exists");
            return Err(EIO);
        }
        let ptr_buf = if let Some(s) = state.get_buf(addr) {
            s
        } else {
            dev_err!(self.dev, "Unable to find buffer");
            return Err(EIO);
        };
        if ptr_buf.size() < mem::size_of::<u64>() {
            dev_err!(self.dev, "Buffer too small");
            return Err(EIO);
        }
        let ptr = unsafe { *(ptr_buf.as_ptr() as *const u64) };
        state.value_buf = Some(ptr);
        let msg = (OPC_SET_BUF | OPC_ACK_MASK) << OPC_SHIFT;
        Ok(msg)
    }
    fn register_ioreg(&self, addr: u64) -> Result<u64> {
        let mut state = self.state.lock();
        let msg_buf = if let Some(s) = state.get_buf(addr) {
            s
        } else {
            dev_err!(self.dev, "Unable to find buffer");
            return Err(EIO);
        };
        if msg_buf.size() < 0x44 {
            dev_err!(self.dev, "Buffer too small");
            return Err(EIO);
        }
        // SAFETY: TODO
        let mut size = u32::from_le_bytes(unsafe {
            <&[u8] as TryInto<[u8; 4]>>::try_into(&msg_buf.as_ref()[0x40..0x44]).unwrap()
        });
        if size == 0 {
            let mut name_vec = KVec::with_capacity(0x31, GFP_KERNEL)?;
            name_vec
                .extend_from_slice(unsafe { &msg_buf.as_ref()[0..0x30] }, GFP_KERNEL)
                .unwrap();
            name_vec.push(0, GFP_KERNEL).unwrap();
            let name_str = CStr::from_bytes_until_nul(&name_vec).unwrap();
            let name_str = CString::try_from_fmt(fmt!("apple,tunable-{name_str}"))?;
            let node = self.dev.fwnode().ok_or(EIO)?;
            if state.value_buf.is_none() {
                dev_err!(self.dev, "Value buf not set");
                return Err(EIO);
            }
            let val_buf_addr = state.value_buf.unwrap();
            let val_buf = if let Some(s) = state.get_buf(val_buf_addr) {
                s
            } else {
                dev_err!(self.dev, "Unable to find value buffer");
                return Err(EIO);
            };
            if node.property_present(&name_str) {
                let len = node.property_count_elem::<u8>(&name_str)?;
                let data = node
                    .property_read_array_vec::<u8>(&name_str, len)?
                    .required_by(&self.dev)?;
                unsafe {
                    val_buf.as_mut()[0..len].copy_from_slice(&data);
                }
                size = len as u32;
            } else {
                dev_info!(self.dev, "unknown property {:?}", name_str);
            }
        }
        state.ioreg_entries.push(size, GFP_KERNEL)?;
        let index = state.ioreg_entries.len() as u64;
        let msg = (OPC_REGISTER_IOREG | OPC_ACK_MASK) << OPC_SHIFT | (index << 32) | size as u64;
        Ok(msg)
    }
    fn set_ioreg(&self, index: u64) -> Result<u64> {
        let len = *self
            .state
            .lock()
            .ioreg_entries
            .get(index as usize)
            .ok_or(EIO)? as u64;
        let msg = (OPC_SET_IOREG | OPC_ACK_MASK) << OPC_SHIFT | len;
        Ok(msg)
    }
    fn recv_message(&self, msg: u64) -> Result<()> {
        let opc = (msg >> OPC_SHIFT) & 0xFF;
        let reply = match opc {
            OPC_GET_IOVA_TABLE => self.get_iova_table()?,
            OPC_MALLOC => self.malloc(msg & MALLOC_SIZE_MASK)?,
            OPC_FREE => self.free(msg & MSG_IOVA_MASK)?,
            OPC_SET_BUF => self.set_buf(msg & MSG_IOVA_MASK)?,
            OPC_REGISTER_IOREG => self.register_ioreg(msg & MSG_IOVA_MASK)?,
            OPC_SET_IOREG => self.set_ioreg(msg & SET_IOREG_INDEX_MASK)?,
            _ => {
                dev_err!(self.dev, "Got unknown message {}", msg);
                return Err(EIO);
            }
        };
        let mut rtk_guard = self.rtkit.lock();
        let rtk = rtk_guard.as_mut().as_pin_mut().unwrap();
        rtk.send_message(PMP_ENDPOINT, reply)?;
        Ok(())
    }
}

unsafe impl Send for PmpData {}
unsafe impl Sync for PmpData {}

struct NoBuffer;
impl rtkit::Buffer for NoBuffer {
    fn iova(&self) -> Result<usize> {
        unreachable!()
    }
    fn buf(&mut self) -> Result<IoSysMap<'_, Region>> {
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

#[allow(dead_code)]
struct PmpDriver(Arc<PmpData>);

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    (),
    [(of::DeviceId::new(c"apple,t6000-pmp-v2"), ())]
);

impl platform::Driver for PmpDriver {
    type IdInfo = ();
    type Data<'bound> = PmpDriver;

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        _info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let dev: ARef<device::Device> = pdev.as_ref().into();
        let pmp_req = pdev.io_request_by_name(c"pmp").ok_or(EINVAL)?;
        let pmp_mmio = pmp_req.iomap_sized::<PMP_MMIO_SIZE>()?;
        let pmp_mmio = pmp_mmio.as_view();
        let asc_req = pdev.io_request_by_name(c"asc").ok_or(EINVAL)?;
        let asc_mmio = asc_req.iomap_sized::<ASC_MMIO_SIZE>()?;
        let asc_mmio = asc_mmio.as_view().relaxed();
        let data = PmpData::new(pdev)?;
        let node = dev.fwnode().ok_or(EIO)?;
        let dvid = node
            .property_read(c"apple,dram-vendor-id")
            .required_by(&dev)?;
        let bdid = node.property_read(c"apple,board-id").required_by(&dev)?;
        match node.property_read(c"apple,dram-capacity").optional() {
            Some(dcap) => data.patch_bootargs(
                &pmp_mmio,
                &[
                    (from_fourcc(b"BDID"), bdid),
                    (from_fourcc(b"DCAP"), dcap),
                    (from_fourcc(b"DVID"), dvid),
                ],
            )?,
            None => data.patch_bootargs(
                &pmp_mmio,
                &[(from_fourcc(b"BDID"), bdid), (from_fourcc(b"DVID"), dvid)],
            )?,
        };
        let rtkit = rtkit::RtKit::<PmpData>::new(&dev, None, 0, data.clone())?;
        data.rtkit.lock().as_mut().set(Some(rtkit));
        asc_mmio.update(ASC_CPU_CONTROL, |r| r.with_const_cpu_run::<1>());
        data.start()?;
        Ok(PmpDriver(data))
    }
}

module_platform_driver! {
    type: PmpDriver,
    name: "apple_pmp",
    description: "Apple Power Management Processor",
    license: "Dual MIT/GPL",
}
