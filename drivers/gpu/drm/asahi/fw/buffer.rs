// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU tiled vertex buffer control firmware structures

use super::types::*;
use super::workqueue;
use crate::{default_zeroed, no_debug, trivial_gpustruct};
use kernel::sync::Arc;

pub(crate) mod raw {
    use super::*;

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct BlockControl {
        pub(crate) total: AtomicU32,
        pub(crate) wptr: AtomicU32,
        pub(crate) unk: AtomicU32,
        pub(crate) pad: Pad<0x34>,
    }
    default_zeroed!(BlockControl);

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct Counter {
        pub(crate) count: AtomicU32,
        __pad: Pad<0x3c>,
    }
    default_zeroed!(Counter);

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct Stats {
        pub(crate) max_pages: AtomicU32,
        pub(crate) max_b: AtomicU32,
        pub(crate) overflow_count: AtomicU32,
        pub(crate) gpu_c: AtomicU32,
        pub(crate) __pad0: Pad<0x10>,
        pub(crate) reset: AtomicU32,
        pub(crate) __pad1: Pad<0x1c>,
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct Info<'a> {
        pub(crate) gpu_counter: u32,
        pub(crate) unk_4: u32,
        pub(crate) last_id: i32,
        pub(crate) cur_id: i32,
        pub(crate) unk_10: u32,
        pub(crate) gpu_counter2: u32,
        pub(crate) unk_18: u32,

        #[ver(V < V13_0B4 || G >= G14X)]
        pub(crate) unk_1c: u32,

        pub(crate) page_list: GpuPointer<'a, &'a [u32]>,
        pub(crate) page_list_size: u32,
        pub(crate) page_count: AtomicU32,
        pub(crate) max_blocks: u32,
        pub(crate) block_count: AtomicU32,
        pub(crate) unk_38: u32,
        pub(crate) block_list: GpuPointer<'a, &'a [u32]>,
        pub(crate) block_ctl: GpuPointer<'a, super::BlockControl>,
        pub(crate) last_page: AtomicU32,
        pub(crate) gpu_page_ptr1: u32,
        pub(crate) gpu_page_ptr2: u32,
        pub(crate) unk_58: u32,
        pub(crate) block_size: u32,
        pub(crate) unk_60: U64,
        pub(crate) counter: GpuPointer<'a, super::Counter>,
        pub(crate) unk_70: u32,
        pub(crate) unk_74: u32,
        pub(crate) unk_78: u32,
        pub(crate) unk_7c: u32,
        pub(crate) unk_80: u32,
        pub(crate) max_pages: u32,
        pub(crate) max_pages_nomemless: u32,
        pub(crate) unk_8c: u32,
        pub(crate) unk_90: Array<0x30, u8>,
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct Scene<'a> {
        #[ver(G >= G14X)]
        pub(crate) control_word: GpuPointer<'a, &'a [u32]>,
        #[ver(G >= G14X)]
        pub(crate) control_word2: GpuPointer<'a, &'a [u32]>,
        pub(crate) pass_page_count: AtomicU32,
        pub(crate) unk_4: u32,
        pub(crate) unk_8: U64,
        pub(crate) unk_10: U64,
        pub(crate) user_buffer: GpuPointer<'a, &'a [u8]>,
        pub(crate) unk_20: u32,
        #[ver(V >= V13_3)]
        pub(crate) unk_28: U64,
        pub(crate) stats: GpuWeakPointer<super::Stats>,
        pub(crate) total_page_count: AtomicU32,
        #[ver(G < G14X)]
        pub(crate) unk_30: U64, // pad
        #[ver(G < G14X)]
        pub(crate) unk_38: U64, // pad
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct InitBuffer<'a> {
        pub(crate) tag: workqueue::CommandType,
        pub(crate) vm_slot: u32,
        pub(crate) buffer_slot: u32,
        pub(crate) unk_c: u32,
        pub(crate) block_count: u32,
        pub(crate) buffer: GpuPointer<'a, super::Info::ver>,
        pub(crate) stamp_value: EventValue,
    }
}

trivial_gpustruct!(BlockControl);
trivial_gpustruct!(Counter);
trivial_gpustruct!(Stats);

#[versions(AGX)]
#[derive(Debug)]
pub(crate) struct Info {
    pub(crate) block_ctl: GpuObject<BlockControl>,
    pub(crate) counter: GpuObject<Counter>,
    pub(crate) page_list: GpuArray<u32>,
    pub(crate) block_list: GpuArray<u32>,
}

#[versions(AGX)]
impl GpuStruct for Info::ver {
    type Raw<'a> = raw::Info::ver<'a>;
}

pub(crate) struct ClusterBuffers {
    pub(crate) tilemaps: GpuArray<u8>,
    pub(crate) meta: GpuArray<u8>,
}

#[versions(AGX)]
pub(crate) struct Scene {
    pub(crate) user_buffer: GpuArray<u8>,
    pub(crate) buffer: crate::buffer::Buffer::ver,
    pub(crate) tvb_heapmeta: GpuArray<u8>,
    pub(crate) tvb_tilemap: GpuArray<u8>,
    pub(crate) tpc: Arc<GpuArray<u8>>,
    pub(crate) clustering: Option<ClusterBuffers>,
    pub(crate) preempt_buf: GpuArray<u8>,
    #[ver(G >= G14X)]
    pub(crate) control_word: GpuArray<u32>,
}

#[versions(AGX)]
no_debug!(Scene::ver);

#[versions(AGX)]
impl GpuStruct for Scene::ver {
    type Raw<'a> = raw::Scene::ver<'a>;
}

#[versions(AGX)]
pub(crate) struct InitBuffer {
    pub(crate) scene: Arc<crate::buffer::Scene::ver>,
}

#[versions(AGX)]
no_debug!(InitBuffer::ver);

#[versions(AGX)]
impl workqueue::Command for InitBuffer::ver {}

#[versions(AGX)]
impl GpuStruct for InitBuffer::ver {
    type Raw<'a> = raw::InitBuffer::ver<'a>;
}
