// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Top-level GPU manager
//!
//! This module is the root of all GPU firmware management for a given driver instance. It is
//! responsible for initialization, owning the top-level managers (events, UAT, etc.), and
//! communicating with the raw RtKit endpoints to send and receive messages to/from the GPU
//! firmware.
//!
//! It is also the point where diverging driver firmware/GPU variants (using the versions macro)
//! are unified, so that the top level of the driver itself (in `driver`) does not have to concern
//! itself with version dependence.

use core::any::Any;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use core::time::Duration;

use kernel::{
    c_str,
    delay::coarse_sleep,
    error::code::*,
    macros::versions,
    prelude::*,
    soc::apple::rtkit,
    sync::{
        lock::{mutex::MutexBackend, Guard},
        Arc, Mutex, UniqueArc,
    },
    time::{clock, Now},
    types::ForeignOwnable,
};

use crate::alloc::Allocator;
use crate::debug::*;
use crate::driver::{AsahiDevRef, AsahiDevice};
use crate::fw::channels::PipeType;
use crate::fw::types::{U32, U64};
use crate::{
    alloc, buffer, channel, event, fw, gem, hw, initdata, mem, mmu, queue, regs, workqueue,
};

const DEBUG_CLASS: DebugFlags = DebugFlags::Gpu;

/// Firmware endpoint for init & incoming notifications.
const EP_FIRMWARE: u8 = 0x20;

/// Doorbell endpoint for work/message submissions.
const EP_DOORBELL: u8 = 0x21;

/// Initialize the GPU firmware.
const MSG_INIT: u64 = 0x81 << 48;
const INIT_DATA_MASK: u64 = (1 << 44) - 1;

/// TX channel doorbell.
const MSG_TX_DOORBELL: u64 = 0x83 << 48;
/// Firmware control channel doorbell.
const MSG_FWCTL: u64 = 0x84 << 48;
// /// Halt the firmware (?).
// const MSG_HALT: u64 = 0x85 << 48;

/// Receive channel doorbell notification.
const MSG_RX_DOORBELL: u64 = 0x42 << 48;

/// Doorbell number for firmware kicks/wakeups.
const DOORBELL_KICKFW: u64 = 0x10;
/// Doorbell number for device control channel kicks.
const DOORBELL_DEVCTRL: u64 = 0x11;

// Upper kernel half VA address ranges.
/// Private (cached) firmware structure VA range base.
const IOVA_KERN_PRIV_BASE: u64 = 0xffffffa000000000;
/// Private (cached) firmware structure VA range top.
const IOVA_KERN_PRIV_TOP: u64 = 0xffffffa5ffffffff;
/// Private (cached) GPU-RO firmware structure VA range base.
const IOVA_KERN_GPU_RO_BASE: u64 = 0xffffffa600000000;
/// Private (cached) GPU-RO firmware structure VA range top.
const IOVA_KERN_GPU_RO_TOP: u64 = 0xffffffa7ffffffff;
/// Shared (uncached) firmware structure VA range base.
const IOVA_KERN_SHARED_BASE: u64 = 0xffffffa800000000;
/// Shared (uncached) firmware structure VA range top.
const IOVA_KERN_SHARED_TOP: u64 = 0xffffffa9ffffffff;
/// Shared (uncached) read-only firmware structure VA range base.
const IOVA_KERN_SHARED_RO_BASE: u64 = 0xffffffaa00000000;
/// Shared (uncached) read-only firmware structure VA range top.
const IOVA_KERN_SHARED_RO_TOP: u64 = 0xffffffabffffffff;
/// GPU/FW shared structure VA range base.
const IOVA_KERN_GPU_BASE: u64 = 0xffffffac00000000;
/// GPU/FW shared structure VA range top.
const IOVA_KERN_GPU_TOP: u64 = 0xffffffadffffffff;
/// GPU/FW shared structure VA range base.
const IOVA_KERN_RTKIT_BASE: u64 = 0xffffffae00000000;
/// GPU/FW shared structure VA range top.
const IOVA_KERN_RTKIT_TOP: u64 = 0xffffffae0fffffff;

/// GPU/FW buffer manager control address (context 0 low)
pub(crate) const IOVA_KERN_GPU_BUFMGR_LOW: u64 = 0x20_0000_0000;
/// GPU/FW buffer manager control address (context 0 high)
pub(crate) const IOVA_KERN_GPU_BUFMGR_HIGH: u64 = 0xffffffaeffff0000;

/// Timeout for entering the halt state after a fault or request.
const HALT_ENTER_TIMEOUT: Duration = Duration::from_millis(100);

/// Maximum amount of firmware-private memory garbage allowed before collection.
/// Collection flushes the FW cache and is expensive, so this needs to be
/// reasonably high.
const MAX_FW_ALLOC_GARBAGE: usize = 16 * 1024 * 1024;

/// Global allocators used for kernel-half structures.
pub(crate) struct KernelAllocators {
    pub(crate) private: alloc::DefaultAllocator,
    pub(crate) shared: alloc::DefaultAllocator,
    pub(crate) shared_ro: alloc::DefaultAllocator,
    #[allow(dead_code)]
    pub(crate) gpu: alloc::DefaultAllocator,
    pub(crate) gpu_ro: alloc::DefaultAllocator,
}

/// Receive (GPU->driver) ring buffer channels.
#[versions(AGX)]
#[pin_data]
struct RxChannels {
    event: channel::EventChannel::ver,
    fw_log: channel::FwLogChannel,
    ktrace: channel::KTraceChannel,
    stats: channel::StatsChannel::ver,
}

/// GPU work submission pipe channels (driver->GPU).
#[versions(AGX)]
struct PipeChannels {
    pub(crate) vtx: Vec<Pin<Box<Mutex<channel::PipeChannel::ver>>>>,
    pub(crate) frag: Vec<Pin<Box<Mutex<channel::PipeChannel::ver>>>>,
    pub(crate) comp: Vec<Pin<Box<Mutex<channel::PipeChannel::ver>>>>,
}

/// Misc command transmit (driver->GPU) channels.
#[versions(AGX)]
#[pin_data]
struct TxChannels {
    pub(crate) device_control: channel::DeviceControlChannel::ver,
}

/// Number of work submission pipes per type, one for each priority level.
const NUM_PIPES: usize = 4;

/// A generic monotonically incrementing ID used to uniquely identify object instances within the
/// driver.
pub(crate) struct ID(AtomicU64);

impl ID {
    /// Create a new ID counter with a given value.
    fn new(val: u64) -> ID {
        ID(AtomicU64::new(val))
    }

    /// Fetch the next unique ID.
    pub(crate) fn next(&self) -> u64 {
        self.0.fetch_add(1, Ordering::Relaxed)
    }
}

impl Default for ID {
    /// IDs default to starting at 2, as 0/1 are considered reserved for the system.
    fn default() -> Self {
        Self::new(2)
    }
}

/// A guard representing one active submission on the GPU. When dropped, decrements the active
/// submission count.
pub(crate) struct OpGuard(Arc<dyn GpuManagerPriv>);

impl Drop for OpGuard {
    fn drop(&mut self) {
        self.0.end_op();
    }
}

/// Set of global sequence IDs used in the driver.
#[derive(Default)]
pub(crate) struct SequenceIDs {
    /// `File` instance ID.
    pub(crate) file: ID,
    /// `Vm` instance ID.
    pub(crate) vm: ID,
    /// Submission instance ID.
    pub(crate) submission: ID,
    /// `Queue` instance ID.
    pub(crate) queue: ID,
}

/// Top-level GPU manager that owns all the global state relevant to the driver instance.
#[versions(AGX)]
#[pin_data]
pub(crate) struct GpuManager {
    dev: AsahiDevRef,
    cfg: &'static hw::HwConfig,
    dyncfg: hw::DynConfig,
    pub(crate) initdata: fw::types::GpuObject<fw::initdata::InitData::ver>,
    uat: mmu::Uat,
    crashed: AtomicBool,
    #[pin]
    alloc: Mutex<KernelAllocators>,
    io_mappings: Vec<mmu::Mapping>,
    #[pin]
    rtkit: Mutex<Option<rtkit::RtKit<GpuManager::ver>>>,
    #[pin]
    rx_channels: Mutex<RxChannels::ver>,
    #[pin]
    tx_channels: Mutex<TxChannels::ver>,
    #[pin]
    fwctl_channel: Mutex<channel::FwCtlChannel>,
    pipes: PipeChannels::ver,
    event_manager: Arc<event::EventManager>,
    buffer_mgr: buffer::BufferManager::ver,
    ids: SequenceIDs,
    #[pin]
    garbage_work: Mutex<Vec<Box<dyn workqueue::GenSubmittedWork>>>,
    #[allow(clippy::vec_box)]
    #[pin]
    garbage_contexts: Mutex<Vec<Box<fw::types::GpuObject<fw::workqueue::GpuContextData>>>>,
}

/// Trait used to abstract the firmware/GPU-dependent variants of the GpuManager.
pub(crate) trait GpuManager: Send + Sync {
    /// Cast as an Any type.
    fn as_any(&self) -> &dyn Any;
    /// Cast Arc<Self> as an Any type.
    fn arc_as_any(self: Arc<Self>) -> Arc<dyn Any + Sync + Send>;
    /// Initialize the GPU.
    fn init(&self) -> Result;
    /// Update the GPU globals from global info
    ///
    /// TODO: Unclear what can and cannot be updated like this.
    fn update_globals(&self);
    /// Get a reference to the KernelAllocators.
    fn alloc(&self) -> Guard<'_, KernelAllocators, MutexBackend>;
    /// Create a new `Vm` given a unique `File` ID.
    fn new_vm(&self, file_id: u64) -> Result<mmu::Vm>;
    /// Bind a `Vm` to an available slot and return the `VmBind`.
    fn bind_vm(&self, vm: &mmu::Vm) -> Result<mmu::VmBind>;
    /// Create a new user command queue.
    fn new_queue(
        &self,
        vm: mmu::Vm,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        caps: u32,
    ) -> Result<Box<dyn queue::Queue>>;
    /// Return a reference to the global `SequenceIDs` instance.
    fn ids(&self) -> &SequenceIDs;
    /// Kick the firmware (wake it up if asleep).
    ///
    /// This should be useful to reduce latency on work submission, so we can ask the firmware to
    /// wake up while we do some preparatory work for the work submission.
    fn kick_firmware(&self) -> Result;
    /// Flush the entire firmware cache.
    ///
    /// TODO: Does this actually work?
    fn flush_fw_cache(&self) -> Result;
    /// Handle a GPU work timeout event.
    fn handle_timeout(&self, counter: u32, event_slot: i32);
    /// Handle a GPU fault event.
    fn handle_fault(&self);
    /// Acknowledge a Buffer grow op.
    fn ack_grow(&self, buffer_slot: u32, vm_slot: u32, counter: u32);
    /// Wait for the GPU to become idle and power off.
    fn wait_for_poweroff(&self, timeout: usize) -> Result;
    /// Send a firmware control command (secure cache flush).
    fn fwctl(&self, msg: fw::channels::FwCtlMsg) -> Result;
    /// Get the static GPU configuration for this SoC.
    fn get_cfg(&self) -> &'static hw::HwConfig;
    /// Get the dynamic GPU configuration for this SoC.
    fn get_dyncfg(&self) -> &hw::DynConfig;
    /// Register completed work as garbage
    fn add_completed_work(&self, work: Vec<Box<dyn workqueue::GenSubmittedWork>>);
    /// Register an unused context as garbage
    fn free_context(&self, data: Box<fw::types::GpuObject<fw::workqueue::GpuContextData>>);
    /// Check whether the GPU is crashed
    fn is_crashed(&self) -> bool;
}

/// Private generic trait for functions that don't need to escape this module.
trait GpuManagerPriv {
    /// Decrement the pending submission counter.
    fn end_op(&self);
}

#[versions(AGX)]
#[vtable]
impl rtkit::Operations for GpuManager::ver {
    type Data = Arc<GpuManager::ver>;
    type Buffer = gem::ObjectRef;

    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, ep: u8, msg: u64) {
        let dev = &data.dev;
        //dev_info!(dev, "RtKit message: {:#x}:{:#x}\n", ep, msg);

        if ep != EP_FIRMWARE || msg != MSG_RX_DOORBELL {
            dev_err!(dev, "Unknown message: {:#x}:{:#x}\n", ep, msg);
            return;
        }

        let mut ch = data.rx_channels.lock();

        ch.fw_log.poll();
        ch.ktrace.poll();
        ch.stats.poll();
        ch.event.poll();
    }

    fn crashed(data: <Self::Data as ForeignOwnable>::Borrowed<'_>) {
        let dev = &data.dev;

        data.crashed.store(true, Ordering::Relaxed);

        if debug_enabled(DebugFlags::OopsOnGpuCrash) {
            panic!("GPU firmware crashed");
        } else {
            dev_err!(dev, "GPU firmware crashed, failing all jobs\n");
            data.event_manager.fail_all(workqueue::WorkError::NoDevice);
        }
    }

    fn shmem_alloc(
        data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        size: usize,
    ) -> Result<Self::Buffer> {
        let dev = &data.dev;
        mod_dev_dbg!(dev, "shmem_alloc() {:#x} bytes\n", size);

        let mut obj = gem::new_kernel_object(dev, size)?;
        obj.vmap()?;
        let iova = obj.map_into_range(
            data.uat.kernel_vm(),
            IOVA_KERN_RTKIT_BASE,
            IOVA_KERN_RTKIT_TOP,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_FW_SHARED_RW,
            true,
        )?;
        mod_dev_dbg!(dev, "shmem_alloc() -> VA {:#x}\n", iova);
        Ok(obj)
    }
}

#[versions(AGX)]
impl GpuManager::ver {
    /// Create a new GpuManager of this version/GPU combination.
    #[inline(never)]
    pub(crate) fn new(
        dev: &AsahiDevice,
        res: &regs::Resources,
        cfg: &'static hw::HwConfig,
    ) -> Result<Arc<GpuManager::ver>> {
        let uat = Self::make_uat(dev, cfg)?;
        let dyncfg = Self::make_dyncfg(dev, res, cfg, &uat)?;

        let mut alloc = KernelAllocators {
            private: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                IOVA_KERN_PRIV_BASE,
                IOVA_KERN_PRIV_TOP,
                0x80,
                mmu::PROT_FW_PRIV_RW,
                1024 * 1024,
                true,
                fmt!("Kernel Private"),
                true,
            )?,
            shared: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                IOVA_KERN_SHARED_BASE,
                IOVA_KERN_SHARED_TOP,
                0x80,
                mmu::PROT_FW_SHARED_RW,
                1024 * 1024,
                true,
                fmt!("Kernel Shared"),
                false,
            )?,
            shared_ro: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                IOVA_KERN_SHARED_RO_BASE,
                IOVA_KERN_SHARED_RO_TOP,
                0x80,
                mmu::PROT_FW_SHARED_RO,
                64 * 1024,
                true,
                fmt!("Kernel RO Shared"),
                false,
            )?,
            gpu: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                IOVA_KERN_GPU_BASE,
                IOVA_KERN_GPU_TOP,
                0x80,
                mmu::PROT_GPU_FW_SHARED_RW,
                64 * 1024,
                true,
                fmt!("Kernel GPU Shared"),
                false,
            )?,
            gpu_ro: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                IOVA_KERN_GPU_RO_BASE,
                IOVA_KERN_GPU_RO_TOP,
                0x80,
                mmu::PROT_GPU_RO_FW_PRIV_RW,
                1024 * 1024,
                true,
                fmt!("Kernel GPU RO Shared"),
                true,
            )?,
        };

        let event_manager = Self::make_event_manager(&mut alloc)?;
        let mut initdata = Self::make_initdata(dev, cfg, &dyncfg, &mut alloc)?;

        initdata.runtime_pointers.buffer_mgr_ctl.map_at(
            uat.kernel_lower_vm(),
            IOVA_KERN_GPU_BUFMGR_LOW,
            mmu::PROT_GPU_SHARED_RW,
            false,
        )?;
        initdata.runtime_pointers.buffer_mgr_ctl.map_at(
            uat.kernel_vm(),
            IOVA_KERN_GPU_BUFMGR_HIGH,
            mmu::PROT_FW_SHARED_RW,
            false,
        )?;

        let mut mgr = Self::make_mgr(dev, cfg, dyncfg, uat, alloc, event_manager, initdata)?;

        {
            let fwctl = mgr.fwctl_channel.lock();
            let p_fwctl = fwctl.to_raw();
            core::mem::drop(fwctl);

            mgr.as_mut()
                .initdata_mut()
                .fw_status
                .with_mut(|raw, _inner| {
                    raw.fwctl_channel = p_fwctl;
                });
        }

        {
            let txc = mgr.tx_channels.lock();
            let p_device_control = txc.device_control.to_raw();
            core::mem::drop(txc);

            let rxc = mgr.rx_channels.lock();
            let p_event = rxc.event.to_raw();
            let p_fw_log = rxc.fw_log.to_raw();
            let p_ktrace = rxc.ktrace.to_raw();
            let p_stats = rxc.stats.to_raw();
            let p_fwlog_buf = rxc.fw_log.get_buf();
            core::mem::drop(rxc);

            mgr.as_mut()
                .initdata_mut()
                .runtime_pointers
                .with_mut(|raw, _inner| {
                    raw.device_control = p_device_control;
                    raw.event = p_event;
                    raw.fw_log = p_fw_log;
                    raw.ktrace = p_ktrace;
                    raw.stats = p_stats;
                    raw.fwlog_buf = Some(p_fwlog_buf);
                });
        }

        let mut p_pipes: Vec<fw::initdata::raw::PipeChannels::ver> = Vec::new();

        for ((v, f), c) in mgr
            .pipes
            .vtx
            .iter()
            .zip(&mgr.pipes.frag)
            .zip(&mgr.pipes.comp)
        {
            p_pipes.try_push(fw::initdata::raw::PipeChannels::ver {
                vtx: v.lock().to_raw(),
                frag: f.lock().to_raw(),
                comp: c.lock().to_raw(),
            })?;
        }

        mgr.as_mut()
            .initdata_mut()
            .runtime_pointers
            .with_mut(|raw, _inner| {
                for (i, p) in p_pipes.into_iter().enumerate() {
                    raw.pipes[i].vtx = p.vtx;
                    raw.pipes[i].frag = p.frag;
                    raw.pipes[i].comp = p.comp;
                }
            });

        for (i, map) in cfg.io_mappings.iter().enumerate() {
            if let Some(map) = map.as_ref() {
                Self::iomap(&mut mgr, i, map)?;
            }
        }

        #[ver(V >= V13_0B4)]
        if let Some(base) = cfg.sram_base {
            let size = cfg.sram_size.unwrap() as usize;

            let mapping =
                mgr.uat
                    .kernel_vm()
                    .map_io(base as usize, size, mmu::PROT_FW_SHARED_RW)?;

            mgr.as_mut()
                .initdata_mut()
                .runtime_pointers
                .hwdata_b
                .with_mut(|raw, _| {
                    raw.sgx_sram_ptr = U64(mapping.iova() as u64);
                });

            mgr.as_mut().io_mappings_mut().try_push(mapping)?;
        }

        let mgr = Arc::from(mgr);

        let rtkit = rtkit::RtKit::<GpuManager::ver>::new(dev, None, 0, mgr.clone())?;

        *mgr.rtkit.lock() = Some(rtkit);

        {
            let mut rxc = mgr.rx_channels.lock();
            rxc.event.set_manager(mgr.clone());
        }

        Ok(mgr)
    }

    /// Return a mutable reference to the initdata member
    fn initdata_mut(
        self: Pin<&mut Self>,
    ) -> &mut fw::types::GpuObject<fw::initdata::InitData::ver> {
        // SAFETY: initdata does not require structural pinning.
        unsafe { &mut self.get_unchecked_mut().initdata }
    }

    /// Return a mutable reference to the io_mappings member
    fn io_mappings_mut(self: Pin<&mut Self>) -> &mut Vec<mmu::Mapping> {
        // SAFETY: io_mappings does not require structural pinning.
        unsafe { &mut self.get_unchecked_mut().io_mappings }
    }

    /// Build the entire GPU InitData structure tree and return it as a boxed GpuObject.
    fn make_initdata(
        dev: &AsahiDevice,
        cfg: &'static hw::HwConfig,
        dyncfg: &hw::DynConfig,
        alloc: &mut KernelAllocators,
    ) -> Result<Box<fw::types::GpuObject<fw::initdata::InitData::ver>>> {
        let mut builder = initdata::InitDataBuilder::ver::new(dev, alloc, cfg, dyncfg);
        builder.build()
    }

    /// Create a fresh boxed Uat instance.
    ///
    /// Force disable inlining to avoid blowing up the stack.
    #[inline(never)]
    fn make_uat(dev: &AsahiDevice, cfg: &'static hw::HwConfig) -> Result<Box<mmu::Uat>> {
        // G14X has a new thing in the Scene structure that unfortunately requires
        // write access from user contexts. Hopefully it's not security-sensitive.
        #[ver(G >= G14X)]
        let map_kernel_to_user = true;
        #[ver(G < G14X)]
        let map_kernel_to_user = false;

        Ok(Box::try_new(mmu::Uat::new(dev, cfg, map_kernel_to_user)?)?)
    }

    /// Actually create the final GpuManager instance, as a UniqueArc.
    ///
    /// Force disable inlining to avoid blowing up the stack.
    #[inline(never)]
    fn make_mgr(
        dev: &AsahiDevice,
        cfg: &'static hw::HwConfig,
        dyncfg: Box<hw::DynConfig>,
        uat: Box<mmu::Uat>,
        mut alloc: KernelAllocators,
        event_manager: Arc<event::EventManager>,
        initdata: Box<fw::types::GpuObject<fw::initdata::InitData::ver>>,
    ) -> Result<Pin<UniqueArc<GpuManager::ver>>> {
        let mut pipes = PipeChannels::ver {
            vtx: Vec::new(),
            frag: Vec::new(),
            comp: Vec::new(),
        };

        for _i in 0..=NUM_PIPES - 1 {
            pipes.vtx.try_push(Box::pin_init(Mutex::new_named(
                channel::PipeChannel::ver::new(dev, &mut alloc)?,
                c_str!("pipe_vtx"),
            ))?)?;
            pipes.frag.try_push(Box::pin_init(Mutex::new_named(
                channel::PipeChannel::ver::new(dev, &mut alloc)?,
                c_str!("pipe_frag"),
            ))?)?;
            pipes.comp.try_push(Box::pin_init(Mutex::new_named(
                channel::PipeChannel::ver::new(dev, &mut alloc)?,
                c_str!("pipe_comp"),
            ))?)?;
        }

        let fwctl_channel = channel::FwCtlChannel::new(dev, &mut alloc)?;

        let buffer_mgr = buffer::BufferManager::ver::new()?;
        let event_manager_clone = event_manager.clone();
        let buffer_mgr_clone = buffer_mgr.clone();
        let alloc_ref = &mut alloc;
        let rx_channels = Box::init(try_init!(RxChannels::ver {
            event: channel::EventChannel::ver::new(
                dev,
                alloc_ref,
                event_manager_clone,
                buffer_mgr_clone,
            )?,
            fw_log: channel::FwLogChannel::new(dev, alloc_ref)?,
            ktrace: channel::KTraceChannel::new(dev, alloc_ref)?,
            stats: channel::StatsChannel::ver::new(dev, alloc_ref)?,
        }))?;

        let alloc_ref = &mut alloc;
        let tx_channels = Box::init(try_init!(TxChannels::ver {
            device_control: channel::DeviceControlChannel::ver::new(dev, alloc_ref)?,
        }))?;

        let x = UniqueArc::pin_init(try_pin_init!(GpuManager::ver {
            dev: dev.into(),
            cfg,
            dyncfg: *dyncfg,
            initdata: *initdata,
            uat: *uat,
            io_mappings: Vec::new(),
            rtkit <- Mutex::new_named(None, c_str!("rtkit")),
            crashed: AtomicBool::new(false),
            event_manager,
            alloc <- Mutex::new_named(alloc, c_str!("alloc")),
            fwctl_channel <- Mutex::new_named(fwctl_channel, c_str!("fwctl_channel")),
            rx_channels <- Mutex::new_named(*rx_channels, c_str!("rx_channels")),
            tx_channels <- Mutex::new_named(*tx_channels, c_str!("tx_channels")),
            pipes,
            buffer_mgr,
            ids: Default::default(),
            garbage_work <- Mutex::new_named(Vec::new(), c_str!("garbage_work")),
            garbage_contexts <- Mutex::new_named(Vec::new(), c_str!("garbage_contexts")),
        }))?;

        Ok(x)
    }

    /// Fetch and validate the GPU dynamic configuration from the device tree and hardware.
    ///
    /// Force disable inlining to avoid blowing up the stack.
    #[inline(never)]
    fn make_dyncfg(
        dev: &AsahiDevice,
        res: &regs::Resources,
        cfg: &'static hw::HwConfig,
        uat: &mmu::Uat,
    ) -> Result<Box<hw::DynConfig>> {
        let gpu_id = res.get_gpu_id()?;

        dev_info!(dev, "GPU Information:\n");
        dev_info!(
            dev,
            "  Type: {:?}{:?}\n",
            gpu_id.gpu_gen,
            gpu_id.gpu_variant
        );
        dev_info!(dev, "  Clusters: {}\n", gpu_id.num_clusters);
        dev_info!(
            dev,
            "  Cores: {} ({})\n",
            gpu_id.num_cores,
            gpu_id.num_cores * gpu_id.num_clusters
        );
        dev_info!(
            dev,
            "  Frags: {} ({})\n",
            gpu_id.num_frags,
            gpu_id.num_frags * gpu_id.num_clusters
        );
        dev_info!(
            dev,
            "  GPs: {} ({})\n",
            gpu_id.num_gps,
            gpu_id.num_gps * gpu_id.num_clusters
        );
        dev_info!(dev, "  Core masks: {:#x?}\n", gpu_id.core_masks);
        dev_info!(dev, "  Active cores: {}\n", gpu_id.total_active_cores);

        dev_info!(dev, "Getting configuration from device tree...\n");
        let pwr_cfg = hw::PwrConfig::load(dev, cfg)?;
        dev_info!(dev, "Dynamic configuration fetched\n");

        if gpu_id.gpu_gen != cfg.gpu_gen || gpu_id.gpu_variant != cfg.gpu_variant {
            dev_err!(
                dev,
                "GPU type mismatch (expected {:?}{:?}, found {:?}{:?})\n",
                cfg.gpu_gen,
                cfg.gpu_variant,
                gpu_id.gpu_gen,
                gpu_id.gpu_variant
            );
            return Err(EIO);
        }
        if gpu_id.num_clusters > cfg.max_num_clusters {
            dev_err!(
                dev,
                "Too many clusters ({} > {})\n",
                gpu_id.num_clusters,
                cfg.max_num_clusters
            );
            return Err(EIO);
        }
        if gpu_id.num_cores > cfg.max_num_cores {
            dev_err!(
                dev,
                "Too many cores ({} > {})\n",
                gpu_id.num_cores,
                cfg.max_num_cores
            );
            return Err(EIO);
        }
        if gpu_id.num_frags > cfg.max_num_frags {
            dev_err!(
                dev,
                "Too many frags ({} > {})\n",
                gpu_id.num_frags,
                cfg.max_num_frags
            );
            return Err(EIO);
        }
        if gpu_id.num_gps > cfg.max_num_gps {
            dev_err!(
                dev,
                "Too many GPs ({} > {})\n",
                gpu_id.num_gps,
                cfg.max_num_gps
            );
            return Err(EIO);
        }

        Ok(Box::try_new(hw::DynConfig {
            pwr: pwr_cfg,
            uat_ttb_base: uat.ttb_base(),
            id: gpu_id,
        })?)
    }

    /// Create the global GPU event manager, and return an `Arc<>` to it.
    fn make_event_manager(alloc: &mut KernelAllocators) -> Result<Arc<event::EventManager>> {
        Ok(Arc::try_new(event::EventManager::new(alloc)?)?)
    }

    /// Create a new MMIO mapping and add it to the mappings list in initdata at the specified
    /// index.
    fn iomap(
        this: &mut Pin<UniqueArc<GpuManager::ver>>,
        index: usize,
        map: &hw::IOMapping,
    ) -> Result {
        let off = map.base & mmu::UAT_PGMSK;
        let base = map.base - off;
        let end = (map.base + map.size + mmu::UAT_PGMSK) & !mmu::UAT_PGMSK;
        let mapping = this.uat.kernel_vm().map_io(
            base,
            end - base,
            if map.writable {
                mmu::PROT_FW_MMIO_RW
            } else {
                mmu::PROT_FW_MMIO_RO
            },
        )?;

        this.as_mut()
            .initdata_mut()
            .runtime_pointers
            .hwdata_b
            .with_mut(|raw, _| {
                raw.io_mappings[index] = fw::initdata::raw::IOMapping {
                    phys_addr: U64(map.base as u64),
                    virt_addr: U64((mapping.iova() + off) as u64),
                    size: map.size as u32,
                    range_size: map.range_size as u32,
                    readwrite: U64(map.writable as u64),
                };
            });

        this.as_mut().io_mappings_mut().try_push(mapping)?;
        Ok(())
    }

    /// Mark work associated with currently in-progress event slots as failed, after a fault or
    /// timeout.
    fn mark_pending_events(&self, culprit_slot: Option<u32>, error: workqueue::WorkError) {
        dev_err!(self.dev, "  Pending events:\n");

        self.initdata.globals.with(|raw, _inner| {
            for (index, i) in raw.pending_stamps.iter().enumerate() {
                let info = i.info.load(Ordering::Relaxed);
                let wait_value = i.wait_value.load(Ordering::Relaxed);

                if info & 1 != 0 {
                    #[ver(V >= V13_5)]
                    let slot = (info >> 4) & 0x7f;
                    #[ver(V < V13_5)]
                    let slot = (info >> 3) & 0x7f;
                    #[ver(V >= V13_5)]
                    let flags = info & 0xf;
                    #[ver(V < V13_5)]
                    let flags = info & 0x7;
                    dev_err!(
                        self.dev,
                        "    [{}:{}] flags={} value={:#x}\n",
                        index,
                        slot,
                        flags,
                        wait_value
                    );
                    let error = if culprit_slot.is_some() && culprit_slot != Some(slot) {
                        workqueue::WorkError::Killed
                    } else {
                        error
                    };
                    self.event_manager.mark_error(slot, wait_value, error);
                    i.info.store(0, Ordering::Relaxed);
                    i.wait_value.store(0, Ordering::Relaxed);
                }
            }
        });
    }

    /// Fetch the GPU MMU fault information from the hardware registers.
    fn get_fault_info(&self) -> Option<regs::FaultInfo> {
        let data = self.dev.data();

        let res = match data.resources() {
            Some(res) => res,
            None => {
                dev_err!(self.dev, "  Failed to acquire resources\n");
                return None;
            }
        };

        let info = res.get_fault_info(self.cfg);
        if info.is_some() {
            dev_err!(self.dev, "  Fault info: {:#x?}\n", info.as_ref().unwrap());
        }
        info
    }

    /// Resume the GPU firmware after it halts (due to a timeout, fault, or request).
    fn recover(&self) {
        self.initdata.fw_status.with(|raw, _inner| {
            let halt_count = raw.flags.halt_count.load(Ordering::Relaxed);
            let mut halted = raw.flags.halted.load(Ordering::Relaxed);
            dev_err!(self.dev, "  Halt count: {}\n", halt_count);
            dev_err!(self.dev, "  Halted: {}\n", halted);

            if halted == 0 {
                let start = clock::KernelTime::now();
                while start.elapsed() < HALT_ENTER_TIMEOUT {
                    halted = raw.flags.halted.load(Ordering::Relaxed);
                    if halted != 0 {
                        break;
                    }
                    mem::sync();
                }
                halted = raw.flags.halted.load(Ordering::Relaxed);
            }

            if debug_enabled(DebugFlags::NoGpuRecovery) {
                dev_crit!(self.dev, "  GPU recovery is disabled, wedging forever!\n");
            } else if halted != 0 {
                dev_err!(self.dev, "  Attempting recovery...\n");
                raw.flags.halted.store(0, Ordering::SeqCst);
                raw.flags.resume.store(1, Ordering::SeqCst);
            } else {
                dev_err!(self.dev, "  Cannot recover.\n");
            }
        });
    }

    /// Return the packed GPU enabled core masks.
    // Only used for some versions
    #[allow(dead_code)]
    pub(crate) fn core_masks_packed(&self) -> &[u32] {
        self.dyncfg.id.core_masks_packed.as_slice()
    }

    /// Kick a submission pipe for a submitted job to tell the firmware to start processing it.
    pub(crate) fn run_job(&self, job: workqueue::JobSubmission::ver<'_>) -> Result {
        mod_dev_dbg!(self.dev, "GPU: run_job\n");

        let pipe_type = job.pipe_type();
        mod_dev_dbg!(self.dev, "GPU: run_job: pipe_type={:?}\n", pipe_type);

        let pipes = match pipe_type {
            PipeType::Vertex => &self.pipes.vtx,
            PipeType::Fragment => &self.pipes.frag,
            PipeType::Compute => &self.pipes.comp,
        };

        let index: usize = job.priority() as usize;
        let mut pipe = pipes.get(index).ok_or(EIO)?.lock();

        mod_dev_dbg!(self.dev, "GPU: run_job: run()\n");
        job.run(&mut pipe);
        mod_dev_dbg!(self.dev, "GPU: run_job: ring doorbell\n");

        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().unwrap();
        rtk.send_message(
            EP_DOORBELL,
            MSG_TX_DOORBELL | pipe_type as u64 | ((index as u64) << 2),
        )?;
        mod_dev_dbg!(self.dev, "GPU: run_job: done\n");

        Ok(())
    }

    pub(crate) fn start_op(self: &Arc<GpuManager::ver>) -> Result<OpGuard> {
        if self.is_crashed() {
            return Err(ENODEV);
        }

        let val = self
            .initdata
            .globals
            .with(|raw, _inner| raw.pending_submissions.fetch_add(1, Ordering::Acquire));

        mod_dev_dbg!(self.dev, "OP start (pending: {})\n", val + 1);
        self.kick_firmware()?;
        Ok(OpGuard(self.clone()))
    }

    fn invalidate_context(
        &self,
        context: &fw::types::GpuObject<fw::workqueue::GpuContextData>,
    ) -> Result {
        mod_dev_dbg!(
            self.dev,
            "Invalidating GPU context @ {:?}\n",
            context.weak_pointer()
        );

        if self.is_crashed() {
            return Err(ENODEV);
        }

        let mut guard = self.alloc.lock();
        let (garbage_count, _) = guard.private.garbage();
        let (garbage_count_gpuro, _) = guard.gpu_ro.garbage();

        let dc = context.with(
            |raw, _inner| fw::channels::DeviceControlMsg::ver::DestroyContext {
                unk_4: 0,
                ctx_23: raw.unk_23,
                #[ver(V < V13_3)]
                __pad0: Default::default(),
                unk_c: U32(0),
                unk_10: U32(0),
                ctx_0: raw.unk_0,
                ctx_1: raw.unk_1,
                ctx_4: raw.unk_4,
                #[ver(V < V13_3)]
                __pad1: Default::default(),
                #[ver(V < V13_3)]
                unk_18: 0,
                gpu_context: Some(context.weak_pointer()),
                __pad2: Default::default(),
            },
        );

        mod_dev_dbg!(self.dev, "Context invalidation command: {:?}\n", &dc);

        let mut txch = self.tx_channels.lock();

        let token = txch.device_control.send(&dc);

        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        }

        txch.device_control.wait_for(token)?;

        mod_dev_dbg!(
            self.dev,
            "GPU context invalidated: {:?}\n",
            context.weak_pointer()
        );

        // The invalidation does a cache flush, so it is okay to collect garbage
        guard.private.collect_garbage(garbage_count);
        guard.gpu_ro.collect_garbage(garbage_count_gpuro);

        Ok(())
    }
}

#[versions(AGX)]
impl GpuManager for GpuManager::ver {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn arc_as_any(self: Arc<Self>) -> Arc<dyn Any + Sync + Send> {
        self as Arc<dyn Any + Sync + Send>
    }

    fn init(&self) -> Result {
        self.tx_channels.lock().device_control.send(
            &fw::channels::DeviceControlMsg::ver::Initialize(Default::default()),
        );

        let initdata = self.initdata.gpu_va().get();
        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().unwrap();

        rtk.boot()?;
        rtk.start_endpoint(EP_FIRMWARE)?;
        rtk.start_endpoint(EP_DOORBELL)?;
        rtk.send_message(EP_FIRMWARE, MSG_INIT | (initdata & INIT_DATA_MASK))?;
        rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        core::mem::drop(guard);

        self.kick_firmware()?;
        Ok(())
    }

    fn update_globals(&self) {
        let mut timeout: u32 = 2;
        if debug_enabled(DebugFlags::WaitForPowerOff) {
            timeout = 0;
        } else if debug_enabled(DebugFlags::KeepGpuPowered) {
            timeout = 5000;
        }

        self.initdata.globals.with(|raw, _inner| {
            raw.idle_off_delay_ms.store(timeout, Ordering::Relaxed);
        });
    }

    fn alloc(&self) -> Guard<'_, KernelAllocators, MutexBackend> {
        /*
         * TODO: This should be done in a workqueue or something.
         * Clean up completed jobs
         */
        self.garbage_work.lock().clear();

        /* Clean up idle contexts */
        let mut garbage_ctx = Vec::new();
        core::mem::swap(&mut *self.garbage_contexts.lock(), &mut garbage_ctx);

        for ctx in garbage_ctx {
            if self.invalidate_context(&ctx).is_err() {
                dev_err!(self.dev, "GpuContext: Failed to invalidate GPU context!\n");
                if debug_enabled(DebugFlags::OopsOnGpuCrash) {
                    panic!("GPU firmware timed out");
                }
            }
        }

        let mut guard = self.alloc.lock();
        let (garbage_count, garbage_bytes) = guard.private.garbage();
        if garbage_bytes > MAX_FW_ALLOC_GARBAGE {
            mod_dev_dbg!(
                self.dev,
                "Collecting kalloc/private garbage ({} objects, {} bytes)\n",
                garbage_count,
                garbage_bytes
            );
            if self.flush_fw_cache().is_err() {
                dev_err!(self.dev, "Failed to flush FW cache\n");
            } else {
                guard.private.collect_garbage(garbage_count);
            }
        }

        let (garbage_count, garbage_bytes) = guard.gpu_ro.garbage();
        if garbage_bytes > MAX_FW_ALLOC_GARBAGE {
            mod_dev_dbg!(
                self.dev,
                "Collecting kalloc/gpuro garbage ({} objects, {} bytes)\n",
                garbage_count,
                garbage_bytes
            );
            if self.flush_fw_cache().is_err() {
                dev_err!(self.dev, "Failed to flush FW cache\n");
            } else {
                guard.gpu_ro.collect_garbage(garbage_count);
            }
        }

        guard
    }

    fn new_vm(&self, file_id: u64) -> Result<mmu::Vm> {
        self.uat.new_vm(self.ids.vm.next(), file_id)
    }

    fn bind_vm(&self, vm: &mmu::Vm) -> Result<mmu::VmBind> {
        self.uat.bind(vm)
    }

    fn new_queue(
        &self,
        vm: mmu::Vm,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        caps: u32,
    ) -> Result<Box<dyn queue::Queue>> {
        let mut kalloc = self.alloc();
        let id = self.ids.queue.next();
        Ok(Box::try_new(queue::Queue::ver::new(
            &self.dev,
            vm,
            &mut kalloc,
            ualloc,
            ualloc_priv,
            self.event_manager.clone(),
            &self.buffer_mgr,
            id,
            priority,
            caps,
        )?)?)
    }

    fn kick_firmware(&self) -> Result {
        if self.is_crashed() {
            return Err(ENODEV);
        }

        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().unwrap();
        rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_KICKFW)?;

        Ok(())
    }

    fn flush_fw_cache(&self) -> Result {
        mod_dev_dbg!(self.dev, "Flushing coprocessor data cache\n");

        if self.is_crashed() {
            return Err(ENODEV);
        }

        // ctx_0 == 0xff or ctx_1 == 0xff cause no effect on context,
        // but this command does a full cache flush too, so abuse it
        // for that.

        let dc = fw::channels::DeviceControlMsg::ver::DestroyContext {
            unk_4: 0,

            ctx_23: 0,
            #[ver(V < V13_3)]
            __pad0: Default::default(),
            unk_c: U32(0),
            unk_10: U32(0),
            ctx_0: 0xff,
            ctx_1: 0xff,
            ctx_4: 0,
            #[ver(V < V13_3)]
            __pad1: Default::default(),
            #[ver(V < V13_3)]
            unk_18: 0,
            gpu_context: None,
            __pad2: Default::default(),
        };

        let mut txch = self.tx_channels.lock();

        let token = txch.device_control.send(&dc);
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        }

        txch.device_control.wait_for(token)?;
        Ok(())
    }

    fn ids(&self) -> &SequenceIDs {
        &self.ids
    }

    fn handle_timeout(&self, counter: u32, event_slot: i32) {
        dev_err!(self.dev, " (\\________/) \n");
        dev_err!(self.dev, "  |        |  \n");
        dev_err!(self.dev, "'.| \\  , / |.'\n");
        dev_err!(self.dev, "--| / (( \\ |--\n");
        dev_err!(self.dev, ".'|  _-_-  |'.\n");
        dev_err!(self.dev, "  |________|  \n");
        dev_err!(self.dev, "** GPU timeout nya~!!!!! **\n");
        dev_err!(self.dev, "  Event slot: {}\n", event_slot);
        dev_err!(self.dev, "  Timeout count: {}\n", counter);

        // If we have fault info, consider it a fault.
        let error = match self.get_fault_info() {
            Some(info) => workqueue::WorkError::Fault(info),
            None => workqueue::WorkError::Timeout,
        };
        self.mark_pending_events(event_slot.try_into().ok(), error);
        self.recover();
    }

    fn handle_fault(&self) {
        dev_err!(self.dev, " (\\________/) \n");
        dev_err!(self.dev, "  |        |  \n");
        dev_err!(self.dev, "'.| \\  , / |.'\n");
        dev_err!(self.dev, "--| / (( \\ |--\n");
        dev_err!(self.dev, ".'|  _-_-  |'.\n");
        dev_err!(self.dev, "  |________|  \n");
        dev_err!(self.dev, "GPU fault nya~!!!!!\n");
        let error = match self.get_fault_info() {
            Some(info) => workqueue::WorkError::Fault(info),
            None => workqueue::WorkError::Unknown,
        };
        self.mark_pending_events(None, error);
        self.recover();
    }

    fn ack_grow(&self, buffer_slot: u32, vm_slot: u32, counter: u32) {
        let dc = fw::channels::DeviceControlMsg::ver::GrowTVBAck {
            unk_4: 1,
            buffer_slot,
            vm_slot,
            counter,
            subpipe: 0, // TODO
            __pad: Default::default(),
        };

        mod_dev_dbg!(self.dev, "TVB Grow Ack command: {:?}\n", &dc);

        let mut txch = self.tx_channels.lock();

        txch.device_control.send(&dc);
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            if rtk
                .send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)
                .is_err()
            {
                dev_err!(self.dev, "Failed to send TVB Grow Ack command\n");
            }
        }
    }

    fn wait_for_poweroff(&self, timeout: usize) -> Result {
        self.initdata.runtime_pointers.hwdata_a.with(|raw, _inner| {
            for _i in 0..timeout {
                if raw.pwr_status.load(Ordering::Relaxed) == 4 {
                    return Ok(());
                }
                coarse_sleep(Duration::from_millis(1));
            }
            Err(ETIMEDOUT)
        })
    }

    fn fwctl(&self, msg: fw::channels::FwCtlMsg) -> Result {
        if self.is_crashed() {
            return Err(ENODEV);
        }

        let mut fwctl = self.fwctl_channel.lock();
        let token = fwctl.send(&msg);
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_FWCTL)?;
        }
        fwctl.wait_for(token)?;
        Ok(())
    }

    fn get_cfg(&self) -> &'static hw::HwConfig {
        self.cfg
    }

    fn get_dyncfg(&self) -> &hw::DynConfig {
        &self.dyncfg
    }

    fn add_completed_work(&self, work: Vec<Box<dyn workqueue::GenSubmittedWork>>) {
        let mut garbage = self.garbage_work.lock();

        if garbage.try_reserve(work.len()).is_err() {
            dev_err!(
                self.dev,
                "Failed to reserve space for completed work, deadlock possible.\n"
            );
            return;
        }

        for i in work {
            garbage
                .try_push(i)
                .expect("try_push() failed after try_reserve()");
        }
    }

    fn free_context(&self, ctx: Box<fw::types::GpuObject<fw::workqueue::GpuContextData>>) {
        let mut garbage = self.garbage_contexts.lock();

        if garbage.try_push(ctx).is_err() {
            dev_err!(
                self.dev,
                "Failed to reserve space for freed context, deadlock possible.\n"
            );
        }
    }

    fn is_crashed(&self) -> bool {
        self.crashed.load(Ordering::Relaxed)
    }
}

#[versions(AGX)]
impl GpuManagerPriv for GpuManager::ver {
    fn end_op(&self) {
        let val = self
            .initdata
            .globals
            .with(|raw, _inner| raw.pending_submissions.fetch_sub(1, Ordering::Release));

        mod_dev_dbg!(self.dev, "OP end (pending: {})\n", val - 1);
    }
}
