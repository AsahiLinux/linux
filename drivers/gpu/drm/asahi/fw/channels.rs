// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU communication channel firmware structures (ring buffers)

use super::types::*;
use crate::default_zeroed;
use core::sync::atomic::Ordering;
use kernel::static_assert;

pub(crate) mod raw {
    use super::*;

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct ChannelState<'a> {
        pub(crate) read_ptr: AtomicU32,
        __pad0: Pad<0x1c>,
        pub(crate) write_ptr: AtomicU32,
        __pad1: Pad<0xc>,
        _p: PhantomData<&'a ()>,
    }
    default_zeroed!(<'a>, ChannelState<'a>);

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct FwCtlChannelState<'a> {
        pub(crate) read_ptr: AtomicU32,
        __pad0: Pad<0xc>,
        pub(crate) write_ptr: AtomicU32,
        __pad1: Pad<0xc>,
        _p: PhantomData<&'a ()>,
    }
    default_zeroed!(<'a>, FwCtlChannelState<'a>);
}

pub(crate) trait RxChannelState: GpuStruct + Debug + Default
where
    for<'a> <Self as GpuStruct>::Raw<'a>: Default + Zeroable,
{
    const SUB_CHANNELS: usize;

    fn wptr(raw: &Self::Raw<'_>, index: usize) -> u32;
    fn set_rptr(raw: &Self::Raw<'_>, index: usize, rptr: u32);
}

#[derive(Debug, Default)]
pub(crate) struct ChannelState {}

impl GpuStruct for ChannelState {
    type Raw<'a> = raw::ChannelState<'a>;
}

impl RxChannelState for ChannelState {
    const SUB_CHANNELS: usize = 1;

    fn wptr(raw: &Self::Raw<'_>, _index: usize) -> u32 {
        raw.write_ptr.load(Ordering::Acquire)
    }

    fn set_rptr(raw: &Self::Raw<'_>, _index: usize, rptr: u32) {
        raw.read_ptr.store(rptr, Ordering::Release);
    }
}

#[derive(Debug, Default)]
pub(crate) struct FwLogChannelState {}

impl GpuStruct for FwLogChannelState {
    type Raw<'a> = Array<6, raw::ChannelState<'a>>;
}

impl RxChannelState for FwLogChannelState {
    const SUB_CHANNELS: usize = 6;

    fn wptr(raw: &Self::Raw<'_>, index: usize) -> u32 {
        raw[index].write_ptr.load(Ordering::Acquire)
    }

    fn set_rptr(raw: &Self::Raw<'_>, index: usize, rptr: u32) {
        raw[index].read_ptr.store(rptr, Ordering::Release);
    }
}

#[derive(Debug, Default)]
pub(crate) struct FwCtlChannelState {}

impl GpuStruct for FwCtlChannelState {
    type Raw<'a> = raw::FwCtlChannelState<'a>;
}

pub(crate) trait TxChannelState: GpuStruct + Debug + Default {
    fn rptr(raw: &Self::Raw<'_>) -> u32;
    fn set_wptr(raw: &Self::Raw<'_>, wptr: u32);
}

impl TxChannelState for ChannelState {
    fn rptr(raw: &Self::Raw<'_>) -> u32 {
        raw.read_ptr.load(Ordering::Acquire)
    }

    fn set_wptr(raw: &Self::Raw<'_>, wptr: u32) {
        raw.write_ptr.store(wptr, Ordering::Release);
    }
}

impl TxChannelState for FwCtlChannelState {
    fn rptr(raw: &Self::Raw<'_>) -> u32 {
        raw.read_ptr.load(Ordering::Acquire)
    }

    fn set_wptr(raw: &Self::Raw<'_>, wptr: u32) {
        raw.write_ptr.store(wptr, Ordering::Release);
    }
}

#[derive(Debug, Copy, Clone, Default)]
#[repr(u32)]
pub(crate) enum PipeType {
    #[default]
    Vertex = 0,
    Fragment = 1,
    Compute = 2,
}

#[versions(AGX)]
#[derive(Debug, Copy, Clone, Default)]
#[repr(C)]
pub(crate) struct RunWorkQueueMsg {
    pub(crate) pipe_type: PipeType,
    pub(crate) work_queue: Option<GpuWeakPointer<super::workqueue::QueueInfo::ver>>,
    pub(crate) wptr: u32,
    pub(crate) event_slot: u32,
    pub(crate) is_new: bool,
    #[ver(V >= V13_2 && G == G14)]
    pub(crate) __pad: Pad<0x2b>,
    #[ver(V < V13_2 || G != G14)]
    pub(crate) __pad: Pad<0x1b>,
}

#[versions(AGX)]
pub(crate) type PipeMsg = RunWorkQueueMsg::ver;

#[versions(AGX)]
pub(crate) const DEVICECONTROL_SZ: usize = {
    #[ver(V < V13_2 || G != G14)]
    {
        0x2c
    }
    #[ver(V >= V13_2 && G == G14)]
    {
        0x3c
    }
};

// TODO: clean up when arbitrary_enum_discriminant is stable
// https://github.com/rust-lang/rust/issues/60553

#[versions(AGX)]
#[derive(Debug, Copy, Clone)]
#[repr(C, u32)]
#[allow(dead_code)]
pub(crate) enum DeviceControlMsg {
    Unk00(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk01(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk02(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk03(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk04(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk05(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk06(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk07(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk08(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk09(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk0a(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk0b(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk0c(Array<DEVICECONTROL_SZ::ver, u8>),
    #[ver(V >= V13_3)]
    Unk0d(Array<DEVICECONTROL_SZ::ver, u8>),
    GrowTVBAck {
        unk_4: u32,
        buffer_slot: u32,
        vm_slot: u32,
        counter: u32,
        subpipe: u32,
        halt_count: U64,
        __pad: Pad<{ DEVICECONTROL_SZ::ver - 0x1c }>,
    },
    Unk0e(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk0f(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk10(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk11(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk12(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk13(Array<DEVICECONTROL_SZ::ver, u8>),
    Unk14(Array<DEVICECONTROL_SZ::ver, u8>), // Init?
    Unk15(Array<DEVICECONTROL_SZ::ver, u8>), // Enable something
    Unk16(Array<DEVICECONTROL_SZ::ver, u8>), // Disable something
    DestroyContext {
        unk_4: u32,
        ctx_23: u8,
        #[ver(V < V13_3)]
        __pad0: Pad<3>,
        unk_c: U32,
        unk_10: U32,
        ctx_0: u8,
        ctx_1: u8,
        ctx_4: u8,
        #[ver(V < V13_3)]
        __pad1: Pad<1>,
        #[ver(V < V13_3)]
        unk_18: u32,
        gpu_context: Option<GpuWeakPointer<super::workqueue::GpuContextData>>,
        #[ver(V < V13_3)]
        __pad2: Pad<{ DEVICECONTROL_SZ::ver - 0x20 }>,
        #[ver(V >= V13_3)]
        __pad2: Pad<{ DEVICECONTROL_SZ::ver - 0x18 }>,
    },
    Unk18(Array<DEVICECONTROL_SZ::ver, u8>),
    Initialize(Pad<DEVICECONTROL_SZ::ver>), // Update RegionC
}

#[versions(AGX)]
static_assert!(core::mem::size_of::<DeviceControlMsg::ver>() == 4 + DEVICECONTROL_SZ::ver);

#[versions(AGX)]
default_zeroed!(DeviceControlMsg::ver);

#[derive(Copy, Clone, Default, Debug)]
#[repr(C)]
#[allow(dead_code)]
pub(crate) struct FwCtlMsg {
    pub(crate) addr: U64,
    pub(crate) unk_8: u32,
    pub(crate) slot: u32,
    pub(crate) page_count: u16,
    pub(crate) unk_12: u16,
}

pub(crate) const EVENT_SZ: usize = 0x34;

#[derive(Debug, Copy, Clone)]
#[repr(C, u32)]
#[allow(dead_code)]
pub(crate) enum EventMsg {
    Fault,
    Flag {
        firing: [u32; 4],
        unk_14: u16,
    },
    Unk2(Array<EVENT_SZ, u8>),
    Unk3(Array<EVENT_SZ, u8>),
    Timeout {
        counter: u32,
        unk_8: u32,
        event_slot: i32,
    },
    Unk5(Array<EVENT_SZ, u8>),
    Unk6(Array<EVENT_SZ, u8>),
    GrowTVB {
        vm_slot: u32,
        buffer_slot: u32,
        counter: u32,
    }, // Max discriminant: 0x7
}

static_assert!(core::mem::size_of::<EventMsg>() == 4 + EVENT_SZ);

pub(crate) const EVENT_MAX: u32 = 0x7;

#[derive(Copy, Clone)]
#[repr(C)]
pub(crate) union RawEventMsg {
    pub(crate) raw: (u32, Array<EVENT_SZ, u8>),
    pub(crate) msg: EventMsg,
}

default_zeroed!(RawEventMsg);

#[derive(Debug, Copy, Clone, Default)]
#[repr(C)]
pub(crate) struct RawFwLogMsg {
    pub(crate) msg_type: u32,
    __pad0: u32,
    pub(crate) msg_index: U64,
    __pad1: Pad<0x28>,
}

#[derive(Debug, Copy, Clone, Default)]
#[repr(C)]
pub(crate) struct RawFwLogPayloadMsg {
    pub(crate) msg_type: u32,
    pub(crate) seq_no: u32,
    pub(crate) timestamp: U64,
    pub(crate) msg: Array<0xc8, u8>,
}

#[derive(Debug, Copy, Clone, Default)]
#[repr(C)]
pub(crate) struct RawKTraceMsg {
    pub(crate) msg_type: u32,
    pub(crate) timestamp: U64,
    pub(crate) args: Array<4, U64>,
    pub(crate) code: u8,
    pub(crate) channel: u8,
    __pad: Pad<1>,
    pub(crate) thread: u8,
    pub(crate) unk_flag: U64,
}

#[versions(AGX)]
pub(crate) const STATS_SZ: usize = {
    #[ver(V < V13_0B4)]
    {
        0x2c
    }
    #[ver(V >= V13_0B4)]
    {
        0x3c
    }
};

#[versions(AGX)]
#[derive(Debug, Copy, Clone)]
#[repr(C, u32)]
#[allow(dead_code)]
pub(crate) enum StatsMsg {
    Power {
        // 0x00
        __pad: Pad<0x18>,
        power: U64,
    },
    Unk1(Array<{ STATS_SZ::ver }, u8>),
    PowerOn {
        // 0x02
        off_time: U64,
    },
    PowerOff {
        // 0x03
        on_time: U64,
    },
    Utilization {
        // 0x04
        timestamp: U64,
        util1: u32,
        util2: u32,
        util3: u32,
        util4: u32,
    },
    Unk5(Array<{ STATS_SZ::ver }, u8>),
    Unk6(Array<{ STATS_SZ::ver }, u8>),
    Unk7(Array<{ STATS_SZ::ver }, u8>),
    Unk8(Array<{ STATS_SZ::ver }, u8>),
    AvgPower {
        // 0x09
        active_cs: U64,
        unk2: u32,
        unk3: u32,
        unk4: u32,
        avg_power: u32,
    },
    Temperature {
        // 0x0a
        __pad: Pad<0x8>,
        raw_value: u32,
        scale: u32,
        tmin: u32,
        tmax: u32,
    },
    PowerState {
        // 0x0b
        timestamp: U64,
        last_busy_ts: U64,
        active: u32,
        poweroff: u32,
        unk1: u32,
        pstate: u32,
        unk2: u32,
        unk3: u32,
    },
    FwBusy {
        // 0x0c
        timestamp: U64,
        busy: u32,
    },
    PState {
        // 0x0d
        __pad: Pad<0x8>,
        ps_min: u32,
        unk1: u32,
        ps_max: u32,
        unk2: u32,
    },
    TempSensor {
        // 0x0e
        __pad: Pad<0x4>,
        sensor_id: u32,
        raw_value: u32,
        scale: u32,
        tmin: u32,
        tmax: u32,
    }, // Max discriminant: 0xe
}

#[versions(AGX)]
static_assert!(core::mem::size_of::<StatsMsg::ver>() == 4 + STATS_SZ::ver);

#[versions(AGX)]
pub(crate) const STATS_MAX: u32 = 0xe;

#[versions(AGX)]
#[derive(Copy, Clone)]
#[repr(C)]
pub(crate) union RawStatsMsg {
    pub(crate) raw: (u32, Array<{ STATS_SZ::ver }, u8>),
    pub(crate) msg: StatsMsg::ver,
}

#[versions(AGX)]
default_zeroed!(RawStatsMsg::ver);
