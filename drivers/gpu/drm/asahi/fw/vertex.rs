// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU vertex job firmware structures

use super::types::*;
use super::{event, job, workqueue};
use crate::{buffer, fw, microseq, mmu};
use kernel::sync::Arc;

pub(crate) mod raw {
    use super::*;

    #[derive(Debug, Default, Copy, Clone)]
    #[repr(C)]
    pub(crate) struct TilingParameters {
        pub(crate) rgn_size: u32,
        pub(crate) unk_4: u32,
        pub(crate) ppp_ctrl: u32,
        pub(crate) x_max: u16,
        pub(crate) y_max: u16,
        pub(crate) te_screen: u32,
        pub(crate) te_mtile1: u32,
        pub(crate) te_mtile2: u32,
        pub(crate) tiles_per_mtile: u32,
        pub(crate) tpc_stride: u32,
        pub(crate) unk_24: u32,
        pub(crate) unk_28: u32,
        pub(crate) __pad: Pad<0x74>,
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct JobParameters1<'a> {
        pub(crate) unk_0: U64,
        pub(crate) unk_8: F32,
        pub(crate) unk_c: F32,
        pub(crate) tvb_tilemap: GpuPointer<'a, &'a [u8]>,
        #[ver(G < G14)]
        pub(crate) tvb_cluster_tilemaps: Option<GpuPointer<'a, &'a [u8]>>,
        pub(crate) tpc: GpuPointer<'a, &'a [u8]>,
        pub(crate) tvb_heapmeta: GpuPointer<'a, &'a [u8]>,
        pub(crate) iogpu_unk_54: U64,
        pub(crate) iogpu_unk_56: U64,
        #[ver(G < G14)]
        pub(crate) tvb_cluster_meta1: Option<GpuPointer<'a, &'a [u8]>>,
        pub(crate) utile_config: u32,
        pub(crate) unk_4c: u32,
        pub(crate) ppp_multisamplectl: U64,
        pub(crate) tvb_heapmeta_2: GpuPointer<'a, &'a [u8]>,
        #[ver(G < G14)]
        pub(crate) unk_60: U64,
        #[ver(G < G14)]
        pub(crate) core_mask: Array<2, u32>,
        pub(crate) preempt_buf1: GpuPointer<'a, &'a [u8]>,
        pub(crate) preempt_buf2: GpuPointer<'a, &'a [u8]>,
        pub(crate) unk_80: U64,
        pub(crate) preempt_buf3: GpuPointer<'a, &'a [u8]>,
        pub(crate) encoder_addr: U64,
        #[ver(G < G14)]
        pub(crate) tvb_cluster_meta2: Option<GpuPointer<'a, &'a [u8]>>,
        #[ver(G < G14)]
        pub(crate) tvb_cluster_meta3: Option<GpuPointer<'a, &'a [u8]>>,
        #[ver(G < G14)]
        pub(crate) tiling_control: u32,
        #[ver(G < G14)]
        pub(crate) unk_ac: u32,
        pub(crate) unk_b0: Array<6, U64>,
        pub(crate) pipeline_base: U64,
        #[ver(G < G14)]
        pub(crate) tvb_cluster_meta4: Option<GpuPointer<'a, &'a [u8]>>,
        #[ver(G < G14)]
        pub(crate) unk_f0: U64,
        pub(crate) unk_f8: U64,
        pub(crate) unk_100: Array<3, U64>,
        pub(crate) unk_118: u32,
        #[ver(G >= G14)]
        pub(crate) __pad: Pad<{ 8 * 9 + 0x268 }>,
        #[ver(G < G14)]
        pub(crate) __pad: Pad<0x268>,
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct JobParameters2<'a> {
        pub(crate) unk_480: Array<4, u32>,
        pub(crate) unk_498: U64,
        pub(crate) unk_4a0: u32,
        pub(crate) preempt_buf1: GpuPointer<'a, &'a [u8]>,
        pub(crate) unk_4ac: u32,
        pub(crate) unk_4b0: U64,
        pub(crate) unk_4b8: u32,
        pub(crate) unk_4bc: U64,
        pub(crate) unk_4c4_padding: Array<0x48, u8>,
        pub(crate) unk_50c: u32,
        pub(crate) unk_510: U64,
        pub(crate) unk_518: U64,
        pub(crate) unk_520: U64,
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct RunVertex<'a> {
        pub(crate) tag: workqueue::CommandType,

        #[ver(V >= V13_0B4)]
        pub(crate) counter: U64,

        pub(crate) vm_slot: u32,
        pub(crate) unk_8: u32,
        pub(crate) notifier: GpuPointer<'a, event::Notifier::ver>,
        pub(crate) buffer_slot: u32,
        pub(crate) unk_1c: u32,
        pub(crate) buffer: GpuPointer<'a, fw::buffer::Info::ver>,
        pub(crate) scene: GpuPointer<'a, fw::buffer::Scene::ver>,
        pub(crate) unk_buffer_buf: GpuWeakPointer<[u8]>,
        pub(crate) unk_34: u32,

        #[ver(G < G14X)]
        pub(crate) job_params1: JobParameters1::ver<'a>,
        #[ver(G < G14X)]
        pub(crate) tiling_params: TilingParameters,
        #[ver(G >= G14X)]
        pub(crate) registers: job::raw::RegisterArray,

        pub(crate) tpc: GpuPointer<'a, &'a [u8]>,
        pub(crate) tpc_size: U64,
        pub(crate) microsequence: GpuPointer<'a, &'a [u8]>,
        pub(crate) microsequence_size: u32,
        pub(crate) fragment_stamp_slot: u32,
        pub(crate) fragment_stamp_value: EventValue,
        pub(crate) unk_pointee: u32,
        pub(crate) unk_pad: u32,
        pub(crate) job_params2: JobParameters2<'a>,
        pub(crate) encoder_params: job::raw::EncoderParams,
        pub(crate) unk_55c: u32,
        pub(crate) unk_560: u32,
        pub(crate) sync_grow: u32,
        pub(crate) unk_568: u32,
        pub(crate) unk_56c: u32,
        pub(crate) meta: job::raw::JobMeta,
        pub(crate) unk_after_meta: u32,
        pub(crate) unk_buf_0: U64,
        pub(crate) unk_buf_8: U64,
        pub(crate) unk_buf_10: U64,
        pub(crate) cur_ts: U64,
        pub(crate) start_ts: Option<GpuPointer<'a, AtomicU64>>,
        pub(crate) end_ts: Option<GpuPointer<'a, AtomicU64>>,
        pub(crate) unk_5c4: u32,
        pub(crate) unk_5c8: u32,
        pub(crate) unk_5cc: u32,
        pub(crate) unk_5d0: u32,
        pub(crate) client_sequence: u8,
        pub(crate) pad_5d5: Array<3, u8>,
        pub(crate) unk_5d8: u32,
        pub(crate) unk_5dc: u8,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_ts: U64,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_5dd_8: Array<0x1b, u8>,
    }
}

#[versions(AGX)]
#[derive(Debug)]
pub(crate) struct RunVertex {
    pub(crate) notifier: Arc<GpuObject<event::Notifier::ver>>,
    pub(crate) scene: Arc<buffer::Scene::ver>,
    pub(crate) micro_seq: microseq::MicroSequence,
    pub(crate) vm_bind: mmu::VmBind,
    pub(crate) timestamps: Arc<GpuObject<job::RenderTimestamps>>,
}

#[versions(AGX)]
impl GpuStruct for RunVertex::ver {
    type Raw<'a> = raw::RunVertex::ver<'a>;
}

#[versions(AGX)]
impl workqueue::Command for RunVertex::ver {}
