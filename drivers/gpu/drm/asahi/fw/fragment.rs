// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU fragment job firmware structures

use super::types::*;
use super::{event, job, workqueue};
use crate::{buffer, fw, microseq, mmu};
use kernel::sync::Arc;

pub(crate) mod raw {
    use super::*;

    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub(crate) struct ClearPipelineBinding {
        pub(crate) pipeline_bind: U64,
        pub(crate) address: U64,
    }

    #[derive(Debug, Clone, Copy, Default)]
    #[repr(C)]
    pub(crate) struct StorePipelineBinding {
        pub(crate) unk_0: U64,
        pub(crate) unk_8: u32,
        pub(crate) pipeline_bind: u32,
        pub(crate) unk_10: u32,
        pub(crate) address: u32,
        pub(crate) unk_18: u32,
        pub(crate) unk_1c_padding: u32,
    }

    impl StorePipelineBinding {
        pub(crate) fn new(pipeline_bind: u32, address: u32) -> StorePipelineBinding {
            StorePipelineBinding {
                pipeline_bind,
                address,
                ..Default::default()
            }
        }
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct ArrayAddr {
        pub(crate) ptr: U64,
        pub(crate) unk_padding: U64,
    }

    #[versions(AGX)]
    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub(crate) struct AuxFBInfo {
        pub(crate) iogpu_unk_214: u32,
        pub(crate) unk2: u32,
        pub(crate) width: u32,
        pub(crate) height: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk3: U64,
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct JobParameters1<'a> {
        pub(crate) utile_config: u32,
        pub(crate) unk_4: u32,
        pub(crate) clear_pipeline: ClearPipelineBinding,
        pub(crate) ppp_multisamplectl: U64,
        pub(crate) scissor_array: U64,
        pub(crate) depth_bias_array: U64,
        pub(crate) aux_fb_info: AuxFBInfo::ver,
        pub(crate) depth_dimensions: U64,
        pub(crate) visibility_result_buffer: U64,
        pub(crate) zls_ctrl: U64,

        #[ver(G >= G14)]
        pub(crate) unk_58_g14_0: U64,
        #[ver(G >= G14)]
        pub(crate) unk_58_g14_8: U64,

        pub(crate) depth_buffer_ptr1: U64,
        pub(crate) depth_buffer_ptr2: U64,
        pub(crate) stencil_buffer_ptr1: U64,
        pub(crate) stencil_buffer_ptr2: U64,

        #[ver(G >= G14)]
        pub(crate) unk_68_g14_0: Array<0x20, u8>,

        pub(crate) unk_78: Array<0x4, U64>,
        pub(crate) depth_meta_buffer_ptr1: U64,
        pub(crate) unk_a0: U64,
        pub(crate) depth_meta_buffer_ptr2: U64,
        pub(crate) unk_b0: U64,
        pub(crate) stencil_meta_buffer_ptr1: U64,
        pub(crate) unk_c0: U64,
        pub(crate) stencil_meta_buffer_ptr2: U64,
        pub(crate) unk_d0: U64,
        pub(crate) tvb_tilemap: GpuPointer<'a, &'a [u8]>,
        pub(crate) tvb_layermeta: GpuPointer<'a, &'a [u8]>,
        pub(crate) mtile_stride_dwords: U64,
        pub(crate) tvb_heapmeta: GpuPointer<'a, &'a [u8]>,
        pub(crate) tile_config: U64,
        pub(crate) aux_fb: GpuPointer<'a, &'a [u8]>,
        pub(crate) unk_108: Array<0x6, U64>,
        pub(crate) pipeline_base: U64,
        pub(crate) unk_140: U64,
        pub(crate) helper_program: u32,
        pub(crate) unk_14c: u32,
        pub(crate) helper_arg: U64,
        pub(crate) unk_158: U64,
        pub(crate) unk_160: U64,

        #[ver(G < G14)]
        pub(crate) __pad: Pad<0x1d8>,
        #[ver(G >= G14)]
        pub(crate) __pad: Pad<0x1a8>,
        #[ver(V < V13_0B4)]
        pub(crate) __pad1: Pad<0x8>,
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct JobParameters2 {
        pub(crate) store_pipeline_bind: u32,
        pub(crate) store_pipeline_addr: u32,
        pub(crate) unk_8: u32,
        pub(crate) unk_c: u32,
        pub(crate) merge_upper_x: F32,
        pub(crate) merge_upper_y: F32,
        pub(crate) unk_18: U64,
        pub(crate) utiles_per_mtile_y: u16,
        pub(crate) utiles_per_mtile_x: u16,
        pub(crate) unk_24: u32,
        pub(crate) tile_counts: u32,
        pub(crate) tib_blocks: u32,
        pub(crate) isp_bgobjdepth: u32,
        pub(crate) isp_bgobjvals: u32,
        pub(crate) unk_38: u32,
        pub(crate) unk_3c: u32,
        pub(crate) unk_40: u32,
        pub(crate) __pad: Pad<0xac>,
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct JobParameters3 {
        pub(crate) depth_bias_array: ArrayAddr,
        pub(crate) scissor_array: ArrayAddr,
        pub(crate) visibility_result_buffer: U64,
        pub(crate) unk_118: U64,
        pub(crate) unk_120: Array<0x25, U64>,
        pub(crate) unk_reload_pipeline: ClearPipelineBinding,
        pub(crate) unk_258: U64,
        pub(crate) unk_260: U64,
        pub(crate) unk_268: U64,
        pub(crate) unk_270: U64,
        pub(crate) reload_pipeline: ClearPipelineBinding,
        pub(crate) zls_ctrl: U64,
        pub(crate) unk_290: U64,
        pub(crate) depth_buffer_ptr1: U64,
        pub(crate) unk_2a0: U64,
        pub(crate) unk_2a8: U64,
        pub(crate) depth_buffer_ptr2: U64,
        pub(crate) depth_buffer_ptr3: U64,
        pub(crate) depth_meta_buffer_ptr3: U64,
        pub(crate) stencil_buffer_ptr1: U64,
        pub(crate) unk_2d0: U64,
        pub(crate) unk_2d8: U64,
        pub(crate) stencil_buffer_ptr2: U64,
        pub(crate) stencil_buffer_ptr3: U64,
        pub(crate) stencil_meta_buffer_ptr3: U64,
        pub(crate) unk_2f8: Array<2, U64>,
        pub(crate) tib_blocks: u32,
        pub(crate) unk_30c: u32,
        pub(crate) aux_fb_info: AuxFBInfo::ver,
        pub(crate) tile_config: U64,
        pub(crate) unk_328_padding: Array<0x8, u8>,
        pub(crate) unk_partial_store_pipeline: StorePipelineBinding,
        pub(crate) partial_store_pipeline: StorePipelineBinding,
        pub(crate) isp_bgobjdepth: u32,
        pub(crate) isp_bgobjvals: u32,
        pub(crate) sample_size: u32,
        pub(crate) unk_37c: u32,
        pub(crate) unk_380: U64,
        pub(crate) unk_388: U64,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_390_0: U64,

        pub(crate) depth_dimensions: U64,
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct RunFragment<'a> {
        pub(crate) tag: workqueue::CommandType,

        #[ver(V >= V13_0B4)]
        pub(crate) counter: U64,

        pub(crate) vm_slot: u32,
        pub(crate) unk_8: u32,
        pub(crate) microsequence: GpuPointer<'a, &'a [u8]>,
        pub(crate) microsequence_size: u32,
        pub(crate) notifier: GpuPointer<'a, event::Notifier::ver>,
        pub(crate) buffer: GpuPointer<'a, fw::buffer::Info::ver>,
        pub(crate) scene: GpuPointer<'a, fw::buffer::Scene::ver>,
        pub(crate) unk_buffer_buf: GpuWeakPointer<[u8]>,
        pub(crate) tvb_tilemap: GpuPointer<'a, &'a [u8]>,
        pub(crate) ppp_multisamplectl: U64,
        pub(crate) samples: u32,
        pub(crate) tiles_per_mtile_y: u16,
        pub(crate) tiles_per_mtile_x: u16,
        pub(crate) unk_50: U64,
        pub(crate) unk_58: U64,
        pub(crate) merge_upper_x: F32,
        pub(crate) merge_upper_y: F32,
        pub(crate) unk_68: U64,
        pub(crate) tile_count: U64,

        #[ver(G < G14X)]
        pub(crate) job_params1: JobParameters1::ver<'a>,
        #[ver(G < G14X)]
        pub(crate) job_params2: JobParameters2,
        #[ver(G >= G14X)]
        pub(crate) registers: job::raw::RegisterArray,

        pub(crate) job_params3: JobParameters3::ver,
        pub(crate) unk_758_flag: u32,
        pub(crate) unk_75c_flag: u32,
        pub(crate) unk_buf: Array<0x110, u8>,
        pub(crate) busy_flag: u32,
        pub(crate) tvb_overflow_count: u32,
        pub(crate) unk_878: u32,
        pub(crate) encoder_params: job::raw::EncoderParams,
        pub(crate) process_empty_tiles: u32,
        pub(crate) no_clear_pipeline_textures: u32,
        pub(crate) msaa_zs: u32,
        pub(crate) unk_pointee: u32,
        #[ver(V >= V13_3)]
        pub(crate) unk_v13_3: u32,
        pub(crate) meta: job::raw::JobMeta,
        pub(crate) unk_after_meta: u32,
        pub(crate) unk_buf_0: U64,
        pub(crate) unk_buf_8: U64,
        pub(crate) unk_buf_10: U64,
        pub(crate) cur_ts: U64,
        pub(crate) start_ts: Option<GpuPointer<'a, AtomicU64>>,
        pub(crate) end_ts: Option<GpuPointer<'a, AtomicU64>>,
        pub(crate) unk_914: u32,
        pub(crate) unk_918: U64,
        pub(crate) unk_920: u32,
        pub(crate) client_sequence: u8,
        pub(crate) pad_925: Array<3, u8>,
        pub(crate) unk_928: u32,
        pub(crate) unk_92c: u8,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_ts: U64,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_92d_8: Array<0x1b, u8>,
    }
}

#[versions(AGX)]
#[derive(Debug)]
pub(crate) struct RunFragment {
    pub(crate) notifier: Arc<GpuObject<event::Notifier::ver>>,
    pub(crate) scene: Arc<buffer::Scene::ver>,
    pub(crate) micro_seq: microseq::MicroSequence,
    pub(crate) vm_bind: mmu::VmBind,
    pub(crate) aux_fb: GpuArray<u8>,
    pub(crate) timestamps: Arc<GpuObject<job::RenderTimestamps>>,
}

#[versions(AGX)]
impl GpuStruct for RunFragment::ver {
    type Raw<'a> = raw::RunFragment::ver<'a>;
}

#[versions(AGX)]
impl workqueue::Command for RunFragment::ver {}
