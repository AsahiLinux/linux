// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU firmware microsequence operations

use super::types::*;
use super::{buffer, compute, fragment, initdata, job, vertex, workqueue};
use crate::default_zeroed;

pub(crate) trait Operation {}

#[derive(Debug, Copy, Clone)]
#[repr(u32)]
enum OpCode {
    WaitForIdle = 0x01,
    WaitForIdle2 = 0x02,
    RetireStamp = 0x18,
    #[allow(dead_code)]
    Timestamp = 0x19,
    StartVertex = 0x22,
    FinalizeVertex = 0x23,
    StartFragment = 0x24,
    FinalizeFragment = 0x25,
    StartCompute = 0x29,
    FinalizeCompute = 0x2a,
}

#[derive(Debug, Copy, Clone)]
#[repr(u32)]
pub(crate) enum Pipe {
    Vertex = 1 << 0,
    Fragment = 1 << 8,
    Compute = 1 << 15,
}

pub(crate) const MAX_ATTACHMENTS: usize = 16;

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub(crate) struct Attachment {
    pub(crate) address: U64,
    pub(crate) size: u32,
    pub(crate) unk_c: u16,
    pub(crate) unk_e: u16,
}
default_zeroed!(Attachment);

#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub(crate) struct Attachments {
    pub(crate) list: Array<MAX_ATTACHMENTS, Attachment>,
    pub(crate) count: u32,
}

#[derive(Debug, Copy, Clone)]
#[repr(transparent)]
pub(crate) struct OpHeader(u32);

impl OpHeader {
    const fn new(opcode: OpCode) -> OpHeader {
        OpHeader(opcode as u32)
    }
    const fn with_args(opcode: OpCode, args: u32) -> OpHeader {
        OpHeader(opcode as u32 | args)
    }
}

macro_rules! simple_op {
    ($name:ident) => {
        #[derive(Debug, Copy, Clone)]
        pub(crate) struct $name(OpHeader);

        impl $name {
            pub(crate) const HEADER: $name = $name(OpHeader::new(OpCode::$name));
        }
    };
}

pub(crate) mod op {
    use super::*;

    simple_op!(StartVertex);
    simple_op!(FinalizeVertex);
    simple_op!(StartFragment);
    simple_op!(FinalizeFragment);
    simple_op!(StartCompute);
    simple_op!(FinalizeCompute);
    simple_op!(WaitForIdle2);

    #[derive(Debug, Copy, Clone)]
    pub(crate) struct RetireStamp(OpHeader);
    impl RetireStamp {
        pub(crate) const HEADER: RetireStamp =
            RetireStamp(OpHeader::with_args(OpCode::RetireStamp, 0x40000000));
    }

    #[derive(Debug, Copy, Clone)]
    pub(crate) struct WaitForIdle(OpHeader);
    impl WaitForIdle {
        pub(crate) const fn new(pipe: Pipe) -> WaitForIdle {
            WaitForIdle(OpHeader::with_args(OpCode::WaitForIdle, (pipe as u32) << 8))
        }
    }

    #[derive(Debug, Copy, Clone)]
    pub(crate) struct Timestamp(OpHeader);
    impl Timestamp {
        #[allow(dead_code)]
        pub(crate) const fn new(flag: bool) -> Timestamp {
            Timestamp(OpHeader::with_args(OpCode::Timestamp, (flag as u32) << 31))
        }
    }
}

#[derive(Debug)]
#[repr(C)]
pub(crate) struct WaitForIdle {
    pub(crate) header: op::WaitForIdle,
}

impl Operation for WaitForIdle {}

#[derive(Debug)]
#[repr(C)]
pub(crate) struct WaitForIdle2 {
    pub(crate) header: op::WaitForIdle2,
}

impl Operation for WaitForIdle2 {}

#[derive(Debug)]
#[repr(C)]
pub(crate) struct RetireStamp {
    pub(crate) header: op::RetireStamp,
}

impl Operation for RetireStamp {}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
pub(crate) struct Timestamp<'a> {
    pub(crate) header: op::Timestamp,
    pub(crate) cur_ts: GpuWeakPointer<U64>,
    pub(crate) start_ts: GpuWeakPointer<Option<GpuPointer<'a, AtomicU64>>>,
    pub(crate) update_ts: GpuWeakPointer<Option<GpuPointer<'a, AtomicU64>>>,
    pub(crate) work_queue: GpuWeakPointer<workqueue::QueueInfo::ver>,
    pub(crate) unk_24: U64,

    #[ver(V >= V13_0B4)]
    pub(crate) unk_ts: GpuWeakPointer<U64>,

    pub(crate) uuid: u32,
    pub(crate) unk_30_padding: u32,
}

#[versions(AGX)]
impl<'a> Operation for Timestamp::ver<'a> {}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
pub(crate) struct StartVertex<'a> {
    pub(crate) header: op::StartVertex,
    pub(crate) tiling_params: Option<GpuWeakPointer<vertex::raw::TilingParameters>>,
    pub(crate) job_params1: Option<GpuWeakPointer<vertex::raw::JobParameters1::ver<'a>>>,
    #[ver(G >= G14X)]
    pub(crate) registers: GpuWeakPointer<job::raw::RegisterArray>,
    pub(crate) buffer: GpuWeakPointer<buffer::Info::ver>,
    pub(crate) scene: GpuWeakPointer<buffer::Scene::ver>,
    pub(crate) stats: GpuWeakPointer<initdata::raw::GpuStatsVtx>,
    pub(crate) work_queue: GpuWeakPointer<workqueue::QueueInfo::ver>,
    pub(crate) vm_slot: u32,
    pub(crate) unk_38: u32,
    pub(crate) event_generation: u32,
    pub(crate) buffer_slot: u32,
    pub(crate) unk_44: u32,
    pub(crate) event_seq: U64,
    pub(crate) unk_50: u32,
    pub(crate) unk_pointer: GpuWeakPointer<u32>,
    pub(crate) unk_job_buf: GpuWeakPointer<U64>,
    pub(crate) unk_64: u32,
    pub(crate) unk_68: u32,
    pub(crate) uuid: u32,
    pub(crate) attachments: Attachments,
    pub(crate) padding: u32,

    #[ver(V >= V13_0B4)]
    pub(crate) counter: U64,

    #[ver(V >= V13_0B4)]
    pub(crate) notifier_buf: GpuWeakPointer<Array<0x8, u8>>,

    pub(crate) unk_178: u32,
}

#[versions(AGX)]
impl<'a> Operation for StartVertex::ver<'a> {}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
pub(crate) struct FinalizeVertex {
    pub(crate) header: op::FinalizeVertex,
    pub(crate) scene: GpuWeakPointer<buffer::Scene::ver>,
    pub(crate) buffer: GpuWeakPointer<buffer::Info::ver>,
    pub(crate) stats: GpuWeakPointer<initdata::raw::GpuStatsVtx>,
    pub(crate) work_queue: GpuWeakPointer<workqueue::QueueInfo::ver>,
    pub(crate) vm_slot: u32,
    pub(crate) unk_28: u32,
    pub(crate) unk_pointer: GpuWeakPointer<u32>,
    pub(crate) unk_34: u32,
    pub(crate) uuid: u32,
    pub(crate) fw_stamp: GpuWeakPointer<FwStamp>,
    pub(crate) stamp_value: EventValue,
    pub(crate) unk_48: U64,
    pub(crate) unk_50: u32,
    pub(crate) unk_54: u32,
    pub(crate) unk_58: U64,
    pub(crate) unk_60: u32,
    pub(crate) unk_64: u32,
    pub(crate) unk_68: u32,

    #[ver(G >= G14 && V < V13_0B4)]
    pub(crate) unk_68_g14: U64,

    pub(crate) restart_branch_offset: i32,
    pub(crate) has_attachments: u32, // Check DCMP errors bits 2,3 1=ktrace 2=log 3=panic

    #[ver(V >= V13_0B4)]
    pub(crate) unk_74: Array<0x10, u8>,
}

#[versions(AGX)]
impl Operation for FinalizeVertex::ver {}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
pub(crate) struct StartFragment<'a> {
    pub(crate) header: op::StartFragment,
    pub(crate) job_params2: Option<GpuWeakPointer<fragment::raw::JobParameters2>>,
    pub(crate) job_params1: Option<GpuWeakPointer<fragment::raw::JobParameters1::ver<'a>>>,
    #[ver(G >= G14X)]
    pub(crate) registers: GpuWeakPointer<job::raw::RegisterArray>,
    pub(crate) scene: GpuPointer<'a, buffer::Scene::ver>,
    pub(crate) stats: GpuWeakPointer<initdata::raw::GpuStatsFrag>,
    pub(crate) busy_flag: GpuWeakPointer<u32>,
    pub(crate) tvb_overflow_count: GpuWeakPointer<u32>,
    pub(crate) unk_pointer: GpuWeakPointer<u32>,
    pub(crate) work_queue: GpuWeakPointer<workqueue::QueueInfo::ver>,
    pub(crate) work_item: GpuWeakPointer<fragment::RunFragment::ver>,
    pub(crate) vm_slot: u32,
    pub(crate) unk_50: u32,
    pub(crate) event_generation: u32,
    pub(crate) buffer_slot: u32,
    pub(crate) sync_grow: u32,
    pub(crate) event_seq: U64,
    pub(crate) unk_68: u32,
    pub(crate) unk_758_flag: GpuWeakPointer<u32>,
    pub(crate) unk_job_buf: GpuWeakPointer<U64>,
    #[ver(V >= V13_3)]
    pub(crate) unk_7c_0: U64,
    pub(crate) unk_7c: u32,
    pub(crate) unk_80: u32,
    pub(crate) unk_84: u32,
    pub(crate) uuid: u32,
    pub(crate) attachments: Attachments,
    pub(crate) padding: u32,

    #[ver(V >= V13_0B4)]
    pub(crate) counter: U64,

    #[ver(V >= V13_0B4)]
    pub(crate) notifier_buf: GpuWeakPointer<Array<0x8, u8>>,
}

#[versions(AGX)]
impl<'a> Operation for StartFragment::ver<'a> {}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
pub(crate) struct FinalizeFragment {
    pub(crate) header: op::FinalizeFragment,
    pub(crate) uuid: u32,
    pub(crate) unk_8: u32,
    pub(crate) fw_stamp: GpuWeakPointer<FwStamp>,
    pub(crate) stamp_value: EventValue,
    pub(crate) unk_18: u32,
    pub(crate) scene: GpuWeakPointer<buffer::Scene::ver>,
    pub(crate) buffer: GpuWeakPointer<buffer::Info::ver>,
    pub(crate) unk_2c: U64,
    pub(crate) stats: GpuWeakPointer<initdata::raw::GpuStatsFrag>,
    pub(crate) unk_pointer: GpuWeakPointer<u32>,
    pub(crate) busy_flag: GpuWeakPointer<u32>,
    pub(crate) work_queue: GpuWeakPointer<workqueue::QueueInfo::ver>,
    pub(crate) work_item: GpuWeakPointer<fragment::RunFragment::ver>,
    pub(crate) vm_slot: u32,
    pub(crate) unk_60: u32,
    pub(crate) unk_758_flag: GpuWeakPointer<u32>,
    #[ver(V >= V13_3)]
    pub(crate) unk_6c_0: U64,
    pub(crate) unk_6c: U64,
    pub(crate) unk_74: U64,
    pub(crate) unk_7c: U64,
    pub(crate) unk_84: U64,
    pub(crate) unk_8c: U64,

    #[ver(G == G14 && V < V13_0B4)]
    pub(crate) unk_8c_g14: U64,

    pub(crate) restart_branch_offset: i32,
    pub(crate) has_attachments: u32, // Check DCMP errors bits 2,3 1=ktrace 2=log 3=panic

    #[ver(V >= V13_0B4)]
    pub(crate) unk_9c: Array<0x10, u8>,
}

#[versions(AGX)]
impl Operation for FinalizeFragment::ver {}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
pub(crate) struct StartCompute<'a> {
    pub(crate) header: op::StartCompute,
    pub(crate) unk_pointer: GpuWeakPointer<u32>,
    pub(crate) job_params1: Option<GpuWeakPointer<compute::raw::JobParameters1<'a>>>,
    #[ver(G >= G14X)]
    pub(crate) registers: GpuWeakPointer<job::raw::RegisterArray>,
    pub(crate) stats: GpuWeakPointer<initdata::GpuStatsComp>,
    pub(crate) work_queue: GpuWeakPointer<workqueue::QueueInfo::ver>,
    pub(crate) vm_slot: u32,
    pub(crate) unk_28: u32,
    pub(crate) event_generation: u32,
    pub(crate) event_seq: U64,
    pub(crate) unk_38: u32,
    pub(crate) job_params2: GpuWeakPointer<compute::raw::JobParameters2::ver<'a>>,
    pub(crate) unk_44: u32,
    pub(crate) uuid: u32,
    pub(crate) attachments: Attachments,
    pub(crate) padding: u32,

    #[ver(V >= V13_0B4)]
    pub(crate) unk_flag: GpuWeakPointer<U32>,

    #[ver(V >= V13_0B4)]
    pub(crate) counter: U64,

    #[ver(V >= V13_0B4)]
    pub(crate) notifier_buf: GpuWeakPointer<Array<0x8, u8>>,
}

#[versions(AGX)]
impl<'a> Operation for StartCompute::ver<'a> {}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
pub(crate) struct FinalizeCompute<'a> {
    pub(crate) header: op::FinalizeCompute,
    pub(crate) stats: GpuWeakPointer<initdata::GpuStatsComp>,
    pub(crate) work_queue: GpuWeakPointer<workqueue::QueueInfo::ver>,
    pub(crate) vm_slot: u32,
    #[ver(V < V13_0B4)]
    pub(crate) unk_18: u32,
    pub(crate) job_params2: GpuWeakPointer<compute::raw::JobParameters2::ver<'a>>,
    pub(crate) unk_24: u32,
    pub(crate) uuid: u32,
    pub(crate) fw_stamp: GpuWeakPointer<FwStamp>,
    pub(crate) stamp_value: EventValue,
    pub(crate) unk_38: u32,
    pub(crate) unk_3c: u32,
    pub(crate) unk_40: u32,
    pub(crate) unk_44: u32,
    pub(crate) unk_48: u32,
    pub(crate) unk_4c: u32,
    pub(crate) unk_50: u32,
    pub(crate) unk_54: u32,
    pub(crate) unk_58: u32,

    #[ver(G == G14 && V < V13_0B4)]
    pub(crate) unk_5c_g14: U64,

    pub(crate) restart_branch_offset: i32,
    pub(crate) has_attachments: u32, // Check DCMP errors bits 2,3 1=ktrace 2=log 3=panic

    #[ver(V >= V13_0B4)]
    pub(crate) unk_64: Array<0xd, u8>,

    #[ver(V >= V13_0B4)]
    pub(crate) unk_flag: GpuWeakPointer<U32>,

    #[ver(V >= V13_0B4)]
    pub(crate) unk_79: Array<0x7, u8>,
}

#[versions(AGX)]
impl<'a> Operation for FinalizeCompute::ver<'a> {}
