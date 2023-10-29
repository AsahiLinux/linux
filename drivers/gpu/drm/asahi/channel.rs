// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU ring buffer channels
//!
//! The GPU firmware use a set of ring buffer channels to receive commands from the driver and send
//! it notifications and status messages.
//!
//! These ring buffers mostly follow uniform conventions, so they share the same base
//! implementation.

use crate::debug::*;
use crate::driver::{AsahiDevRef, AsahiDevice};
use crate::fw::channels::*;
use crate::fw::initdata::{raw, ChannelRing};
use crate::fw::types::*;
use crate::{buffer, event, gpu, mem};
use core::time::Duration;
use kernel::{
    c_str,
    delay::coarse_sleep,
    prelude::*,
    sync::Arc,
    time::{clock, Now},
};

pub(crate) use crate::fw::channels::PipeType;

/// A receive (FW->driver) channel.
pub(crate) struct RxChannel<T: RxChannelState, U: Copy + Default>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default + Zeroable,
{
    ring: ChannelRing<T, U>,
    // FIXME: needs feature(generic_const_exprs)
    //rptr: [u32; T::SUB_CHANNELS],
    rptr: [u32; 6],
    count: u32,
}

impl<T: RxChannelState, U: Copy + Default> RxChannel<T, U>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default + Zeroable,
{
    /// Allocates a new receive channel with a given message count.
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators, count: usize) -> Result<RxChannel<T, U>> {
        Ok(RxChannel {
            ring: ChannelRing {
                state: alloc.shared.new_default()?,
                ring: alloc.shared.array_empty(T::SUB_CHANNELS * count)?,
            },
            rptr: Default::default(),
            count: count as u32,
        })
    }

    /// Receives a message on the specified sub-channel index, optionally leaving in the ring
    /// buffer.
    ///
    /// Returns None if the channel is empty.
    fn get_or_peek(&mut self, index: usize, peek: bool) -> Option<U> {
        self.ring.state.with(|raw, _inner| {
            let wptr = T::wptr(raw, index);
            let rptr = &mut self.rptr[index];
            if wptr == *rptr {
                None
            } else {
                let off = self.count as usize * index;
                let msg = self.ring.ring[off + *rptr as usize];
                if !peek {
                    *rptr = (*rptr + 1) % self.count;
                    T::set_rptr(raw, index, *rptr);
                }
                Some(msg)
            }
        })
    }

    /// Receives a message on the specified sub-channel index, and dequeues it from the ring buffer.
    ///
    /// Returns None if the channel is empty.
    pub(crate) fn get(&mut self, index: usize) -> Option<U> {
        self.get_or_peek(index, false)
    }

    /// Peeks a message on the specified sub-channel index, leaving it in the ring buffer.
    ///
    /// Returns None if the channel is empty.
    pub(crate) fn peek(&mut self, index: usize) -> Option<U> {
        self.get_or_peek(index, true)
    }
}

/// A transmit (driver->FW) channel.
pub(crate) struct TxChannel<T: TxChannelState, U: Copy + Default>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default + Zeroable,
{
    ring: ChannelRing<T, U>,
    wptr: u32,
    count: u32,
}

impl<T: TxChannelState, U: Copy + Default> TxChannel<T, U>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default + Zeroable,
{
    /// Allocates a new cached transmit channel with a given message count.
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators, count: usize) -> Result<TxChannel<T, U>> {
        Ok(TxChannel {
            ring: ChannelRing {
                state: alloc.shared.new_default()?,
                ring: alloc.private.array_empty(count)?,
            },
            wptr: 0,
            count: count as u32,
        })
    }

    /// Allocates a new uncached transmit channel with a given message count.
    pub(crate) fn new_uncached(
        alloc: &mut gpu::KernelAllocators,
        count: usize,
    ) -> Result<TxChannel<T, U>> {
        Ok(TxChannel {
            ring: ChannelRing {
                state: alloc.shared.new_default()?,
                ring: alloc.shared.array_empty(count)?,
            },
            wptr: 0,
            count: count as u32,
        })
    }

    /// Send a message to the ring, returning a cookie with the ring buffer position.
    ///
    /// This will poll/block if the ring is full, which we don't really expect to happen.
    pub(crate) fn put(&mut self, msg: &U) -> u32 {
        self.ring.state.with(|raw, _inner| {
            let next_wptr = (self.wptr + 1) % self.count;
            let mut rptr = T::rptr(raw);
            if next_wptr == rptr {
                pr_err!(
                    "TX ring buffer is full! Waiting... ({}, {})\n",
                    next_wptr,
                    rptr
                );
                // TODO: block properly on incoming messages?
                while next_wptr == rptr {
                    coarse_sleep(Duration::from_millis(8));
                    rptr = T::rptr(raw);
                }
            }
            self.ring.ring[self.wptr as usize] = *msg;
            mem::sync();
            T::set_wptr(raw, next_wptr);
            self.wptr = next_wptr;
        });
        self.wptr
    }

    /// Wait for a previously submitted message to be popped off of the ring by the GPU firmware.
    ///
    /// This busy-loops, and is intended to be used for rare cases when we need to block for
    /// completion of a cache management or invalidation operation synchronously (which
    /// the firmware normally completes fast enough not to be worth sleeping for).
    /// If the poll takes longer than 10ms, this switches to sleeping between polls.
    pub(crate) fn wait_for(&mut self, wptr: u32, timeout_ms: u64) -> Result {
        const MAX_FAST_POLL: u64 = 10;
        let start = clock::KernelTime::now();
        let timeout_fast = Duration::from_millis(timeout_ms.min(MAX_FAST_POLL));
        let timeout_slow = Duration::from_millis(timeout_ms);
        self.ring.state.with(|raw, _inner| {
            while start.elapsed() < timeout_fast {
                if T::rptr(raw) == wptr {
                    return Ok(());
                }
                mem::sync();
            }
            while start.elapsed() < timeout_slow {
                if T::rptr(raw) == wptr {
                    return Ok(());
                }
                coarse_sleep(Duration::from_millis(5));
                mem::sync();
            }
            Err(ETIMEDOUT)
        })
    }
}

/// Device Control channel for global device management commands.
#[versions(AGX)]
pub(crate) struct DeviceControlChannel {
    dev: AsahiDevRef,
    ch: TxChannel<ChannelState, DeviceControlMsg::ver>,
}

#[versions(AGX)]
impl DeviceControlChannel::ver {
    const COMMAND_TIMEOUT_MS: u64 = 1000;

    /// Allocate a new Device Control channel.
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<DeviceControlChannel::ver> {
        Ok(DeviceControlChannel::ver {
            dev: dev.into(),
            ch: TxChannel::<ChannelState, DeviceControlMsg::ver>::new(alloc, 0x100)?,
        })
    }

    /// Returns the raw `ChannelRing` structure to pass to firmware.
    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, DeviceControlMsg::ver> {
        self.ch.ring.to_raw()
    }

    /// Submits a Device Control command.
    pub(crate) fn send(&mut self, msg: &DeviceControlMsg::ver) -> u32 {
        cls_dev_dbg!(DeviceControlCh, self.dev, "DeviceControl: {:?}\n", msg);
        self.ch.put(msg)
    }

    /// Waits for a previously submitted Device Control command to complete.
    pub(crate) fn wait_for(&mut self, wptr: u32) -> Result {
        self.ch.wait_for(wptr, Self::COMMAND_TIMEOUT_MS)
    }
}

/// Pipe channel to submit WorkQueue execution requests.
#[versions(AGX)]
pub(crate) struct PipeChannel {
    dev: AsahiDevRef,
    ch: TxChannel<ChannelState, PipeMsg::ver>,
}

#[versions(AGX)]
impl PipeChannel::ver {
    /// Allocate a new Pipe submission channel.
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<PipeChannel::ver> {
        Ok(PipeChannel::ver {
            dev: dev.into(),
            ch: TxChannel::<ChannelState, PipeMsg::ver>::new(alloc, 0x100)?,
        })
    }

    /// Returns the raw `ChannelRing` structure to pass to firmware.
    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, PipeMsg::ver> {
        self.ch.ring.to_raw()
    }

    /// Submits a Pipe kick command to the firmware.
    pub(crate) fn send(&mut self, msg: &PipeMsg::ver) {
        cls_dev_dbg!(PipeCh, self.dev, "Pipe: {:?}\n", msg);
        self.ch.put(msg);
    }
}

/// Firmware Control channel, used for secure cache flush requests.
pub(crate) struct FwCtlChannel {
    dev: AsahiDevRef,
    ch: TxChannel<FwCtlChannelState, FwCtlMsg>,
}

impl FwCtlChannel {
    const COMMAND_TIMEOUT_MS: u64 = 1000;

    /// Allocate a new Firmware Control channel.
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<FwCtlChannel> {
        Ok(FwCtlChannel {
            dev: dev.into(),
            ch: TxChannel::<FwCtlChannelState, FwCtlMsg>::new_uncached(alloc, 0x100)?,
        })
    }

    /// Returns the raw `ChannelRing` structure to pass to firmware.
    pub(crate) fn to_raw(&self) -> raw::ChannelRing<FwCtlChannelState, FwCtlMsg> {
        self.ch.ring.to_raw()
    }

    /// Submits a Firmware Control command to the firmware.
    pub(crate) fn send(&mut self, msg: &FwCtlMsg) -> u32 {
        cls_dev_dbg!(FwCtlCh, self.dev, "FwCtl: {:?}\n", msg);
        self.ch.put(msg)
    }

    /// Waits for a previously submitted Firmware Control command to complete.
    pub(crate) fn wait_for(&mut self, wptr: u32) -> Result {
        self.ch.wait_for(wptr, Self::COMMAND_TIMEOUT_MS)
    }
}

/// Event channel, used to notify the driver of command completions, GPU faults and errors, and
/// other events.
#[versions(AGX)]
pub(crate) struct EventChannel {
    dev: AsahiDevRef,
    ch: RxChannel<ChannelState, RawEventMsg>,
    ev_mgr: Arc<event::EventManager>,
    buf_mgr: buffer::BufferManager::ver,
    gpu: Option<Arc<dyn gpu::GpuManager>>,
}

#[versions(AGX)]
impl EventChannel::ver {
    /// Allocate a new Event channel.
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
        ev_mgr: Arc<event::EventManager>,
        buf_mgr: buffer::BufferManager::ver,
    ) -> Result<EventChannel::ver> {
        Ok(EventChannel::ver {
            dev: dev.into(),
            ch: RxChannel::<ChannelState, RawEventMsg>::new(alloc, 0x100)?,
            ev_mgr,
            buf_mgr,
            gpu: None,
        })
    }

    /// Registers the managing `Gpu` instance that will handle events on this channel.
    pub(crate) fn set_manager(&mut self, gpu: Arc<dyn gpu::GpuManager>) {
        self.gpu = Some(gpu);
    }

    /// Returns the raw `ChannelRing` structure to pass to firmware.
    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, RawEventMsg> {
        self.ch.ring.to_raw()
    }

    /// Polls for new Event messages on this ring.
    pub(crate) fn poll(&mut self) {
        while let Some(msg) = self.ch.get(0) {
            let tag = unsafe { msg.raw.0 };
            match tag {
                0..=EVENT_MAX => {
                    let msg = unsafe { msg.msg };

                    cls_dev_dbg!(EventCh, self.dev, "Event: {:?}\n", msg);
                    match msg {
                        EventMsg::Fault => match self.gpu.as_ref() {
                            Some(gpu) => gpu.handle_fault(),
                            None => {
                                dev_crit!(self.dev, "EventChannel: No GPU manager available!\n")
                            }
                        },
                        EventMsg::Timeout {
                            counter,
                            event_slot,
                            ..
                        } => match self.gpu.as_ref() {
                            Some(gpu) => gpu.handle_timeout(counter, event_slot),
                            None => {
                                dev_crit!(self.dev, "EventChannel: No GPU manager available!\n")
                            }
                        },
                        EventMsg::Flag { firing, .. } => {
                            for (i, flags) in firing.iter().enumerate() {
                                for j in 0..32 {
                                    if flags & (1u32 << j) != 0 {
                                        self.ev_mgr.signal((i * 32 + j) as u32);
                                    }
                                }
                            }
                        }
                        EventMsg::GrowTVB {
                            vm_slot,
                            buffer_slot,
                            counter,
                            ..
                        } => match self.gpu.as_ref() {
                            Some(gpu) => {
                                self.buf_mgr.grow(buffer_slot);
                                gpu.ack_grow(buffer_slot, vm_slot, counter);
                            }
                            None => {
                                dev_crit!(self.dev, "EventChannel: No GPU manager available!\n")
                            }
                        },
                        msg => {
                            dev_crit!(self.dev, "Unknown event message: {:?}\n", msg);
                        }
                    }
                }
                _ => {
                    dev_warn!(self.dev, "Unknown event message: {:?}\n", unsafe {
                        msg.raw
                    });
                }
            }
        }
    }
}

/// Firmware Log channel. This one is pretty special, since it has 6 sub-channels (for different log
/// levels), and it also uses a side buffer to actually hold the log messages, only passing around
/// pointers in the main buffer.
pub(crate) struct FwLogChannel {
    dev: AsahiDevRef,
    ch: RxChannel<FwLogChannelState, RawFwLogMsg>,
    payload_buf: GpuArray<RawFwLogPayloadMsg>,
}

impl FwLogChannel {
    const RING_SIZE: usize = 0x100;
    const BUF_SIZE: usize = 0x100;

    /// Allocate a new Firmware Log channel.
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<FwLogChannel> {
        Ok(FwLogChannel {
            dev: dev.into(),
            ch: RxChannel::<FwLogChannelState, RawFwLogMsg>::new(alloc, Self::RING_SIZE)?,
            payload_buf: alloc
                .shared
                .array_empty(Self::BUF_SIZE * FwLogChannelState::SUB_CHANNELS)?,
        })
    }

    /// Returns the raw `ChannelRing` structure to pass to firmware.
    pub(crate) fn to_raw(&self) -> raw::ChannelRing<FwLogChannelState, RawFwLogMsg> {
        self.ch.ring.to_raw()
    }

    /// Returns the GPU pointers to the firmware log payload buffer.
    pub(crate) fn get_buf(&self) -> GpuWeakPointer<[RawFwLogPayloadMsg]> {
        self.payload_buf.weak_pointer()
    }

    /// Polls for new log messages on all sub-rings.
    pub(crate) fn poll(&mut self) {
        for i in 0..=FwLogChannelState::SUB_CHANNELS - 1 {
            while let Some(msg) = self.ch.peek(i) {
                cls_dev_dbg!(FwLogCh, self.dev, "FwLog{}: {:?}\n", i, msg);
                if msg.msg_type != 2 {
                    dev_warn!(self.dev, "Unknown FWLog{} message: {:?}\n", i, msg);
                    self.ch.get(i);
                    continue;
                }
                if msg.msg_index.0 as usize >= Self::BUF_SIZE {
                    dev_warn!(
                        self.dev,
                        "FWLog{} message index out of bounds: {:?}\n",
                        i,
                        msg
                    );
                    self.ch.get(i);
                    continue;
                }
                let index = Self::BUF_SIZE * i + msg.msg_index.0 as usize;
                let payload = &self.payload_buf.as_slice()[index];
                if payload.msg_type != 3 {
                    dev_warn!(self.dev, "Unknown FWLog{} payload: {:?}\n", i, payload);
                    self.ch.get(i);
                    continue;
                }
                let msg = if let Some(end) = payload.msg.iter().position(|&r| r == 0) {
                    CStr::from_bytes_with_nul(&(*payload.msg)[..end + 1])
                        .unwrap_or(c_str!("cstr_err"))
                } else {
                    dev_warn!(
                        self.dev,
                        "FWLog{} payload not NUL-terminated: {:?}\n",
                        i,
                        payload
                    );
                    self.ch.get(i);
                    continue;
                };
                match i {
                    0 => dev_dbg!(self.dev, "FWLog: {}\n", msg),
                    1 => dev_info!(self.dev, "FWLog: {}\n", msg),
                    2 => dev_notice!(self.dev, "FWLog: {}\n", msg),
                    3 => dev_warn!(self.dev, "FWLog: {}\n", msg),
                    4 => dev_err!(self.dev, "FWLog: {}\n", msg),
                    5 => dev_crit!(self.dev, "FWLog: {}\n", msg),
                    _ => (),
                };
                self.ch.get(i);
            }
        }
    }
}

pub(crate) struct KTraceChannel {
    dev: AsahiDevRef,
    ch: RxChannel<ChannelState, RawKTraceMsg>,
}

/// KTrace channel, used to receive detailed execution trace markers from the firmware.
/// We currently disable this in initdata, so no messages are expected here at this time.
impl KTraceChannel {
    /// Allocate a new KTrace channel.
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<KTraceChannel> {
        Ok(KTraceChannel {
            dev: dev.into(),
            ch: RxChannel::<ChannelState, RawKTraceMsg>::new(alloc, 0x200)?,
        })
    }

    /// Returns the raw `ChannelRing` structure to pass to firmware.
    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, RawKTraceMsg> {
        self.ch.ring.to_raw()
    }

    /// Polls for new KTrace messages on this ring.
    pub(crate) fn poll(&mut self) {
        while let Some(msg) = self.ch.get(0) {
            cls_dev_dbg!(KTraceCh, self.dev, "KTrace: {:?}\n", msg);
        }
    }
}

/// Statistics channel, reporting power-related statistics to the driver.
/// Not really implemented other than debug logs yet...
#[versions(AGX)]
pub(crate) struct StatsChannel {
    dev: AsahiDevRef,
    ch: RxChannel<ChannelState, RawStatsMsg::ver>,
}

#[versions(AGX)]
impl StatsChannel::ver {
    /// Allocate a new Statistics channel.
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<StatsChannel::ver> {
        Ok(StatsChannel::ver {
            dev: dev.into(),
            ch: RxChannel::<ChannelState, RawStatsMsg::ver>::new(alloc, 0x100)?,
        })
    }

    /// Returns the raw `ChannelRing` structure to pass to firmware.
    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, RawStatsMsg::ver> {
        self.ch.ring.to_raw()
    }

    /// Polls for new statistics messages on this ring.
    pub(crate) fn poll(&mut self) {
        while let Some(msg) = self.ch.get(0) {
            let tag = unsafe { msg.raw.0 };
            match tag {
                0..=STATS_MAX::ver => {
                    let msg = unsafe { msg.msg };
                    cls_dev_dbg!(StatsCh, self.dev, "Stats: {:?}\n", msg);
                }
                _ => {
                    pr_warn!("Unknown stats message: {:?}\n", unsafe { msg.raw });
                }
            }
        }
    }
}
