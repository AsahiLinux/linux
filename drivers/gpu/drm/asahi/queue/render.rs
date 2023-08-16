// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(clippy::unusual_byte_groupings)]

//! Render work queue.
//!
//! A render queue consists of two underlying WorkQueues, one for vertex and one for fragment work.
//! This module is in charge of creating all of the firmware structures required to submit 3D
//! rendering work to the GPU, based on the userspace command buffer.

use super::common;
use crate::alloc::Allocator;
use crate::debug::*;
use crate::fw::types::*;
use crate::gpu::GpuManager;
use crate::util::*;
use crate::workqueue::WorkError;
use crate::{buffer, fw, gpu, microseq, workqueue};
use crate::{inner_ptr, inner_weak_ptr};
use core::mem::MaybeUninit;
use core::sync::atomic::Ordering;
use kernel::dma_fence::RawDmaFence;
use kernel::drm::sched::Job;
use kernel::io_buffer::IoBufferReader;
use kernel::new_mutex;
use kernel::prelude::*;
use kernel::sync::Arc;
use kernel::uapi;
use kernel::user_ptr::UserSlicePtr;

const DEBUG_CLASS: DebugFlags = DebugFlags::Render;

/// Tiling/Vertex control bit to disable using more than one GPU cluster. This results in decreased
/// throughput but also less latency, which is probably desirable for light vertex loads where the
/// overhead of clustering/merging would exceed the time it takes to just run the job on one
/// cluster.
const TILECTL_DISABLE_CLUSTERING: u32 = 1u32 << 0;

struct RenderResult {
    result: uapi::drm_asahi_result_render,
    vtx_complete: bool,
    frag_complete: bool,
    vtx_error: Option<workqueue::WorkError>,
    frag_error: Option<workqueue::WorkError>,
    writer: super::ResultWriter,
}

impl RenderResult {
    fn commit(&mut self) {
        if !self.vtx_complete || !self.frag_complete {
            return;
        }

        let mut error = self.vtx_error.take();
        if let Some(frag_error) = self.frag_error.take() {
            if error.is_none() || error == Some(WorkError::Killed) {
                error = Some(frag_error);
            }
        }

        if let Some(err) = error {
            self.result.info = err.into();
        } else {
            self.result.info.status = uapi::drm_asahi_status_DRM_ASAHI_STATUS_COMPLETE;
        }

        self.writer.write(self.result);
    }
}

#[versions(AGX)]
impl super::Queue::ver {
    /// Get the appropriate tiling parameters for a given userspace command buffer.
    fn get_tiling_params(
        cmdbuf: &uapi::drm_asahi_cmd_render,
        num_clusters: u32,
    ) -> Result<buffer::TileInfo> {
        let width: u32 = cmdbuf.fb_width;
        let height: u32 = cmdbuf.fb_height;
        let layers: u32 = cmdbuf.layers;

        if width > 65536 || height > 65536 {
            return Err(EINVAL);
        }

        if layers == 0 || layers > 2048 {
            return Err(EINVAL);
        }

        let tile_width = 32u32;
        let tile_height = 32u32;

        let utile_width = cmdbuf.utile_width;
        let utile_height = cmdbuf.utile_height;

        match (utile_width, utile_height) {
            (32, 32) | (32, 16) | (16, 16) => (),
            _ => return Err(EINVAL),
        };

        let utiles_per_tile_x = tile_width / utile_width;
        let utiles_per_tile_y = tile_height / utile_height;

        let utiles_per_tile = utiles_per_tile_x * utiles_per_tile_y;

        let tiles_x = (width + tile_width - 1) / tile_width;
        let tiles_y = (height + tile_height - 1) / tile_height;
        let tiles = tiles_x * tiles_y;

        let mtiles_x = 4u32;
        let mtiles_y = 4u32;
        let mtiles = mtiles_x * mtiles_y;

        let tiles_per_mtile_x = align(div_ceil(tiles_x, mtiles_x), 4);
        let tiles_per_mtile_y = align(div_ceil(tiles_y, mtiles_y), 4);
        let tiles_per_mtile = tiles_per_mtile_x * tiles_per_mtile_y;

        let mtile_x1 = tiles_per_mtile_x;
        let mtile_x2 = 2 * tiles_per_mtile_x;
        let mtile_x3 = 3 * tiles_per_mtile_x;

        let mtile_y1 = tiles_per_mtile_y;
        let mtile_y2 = 2 * tiles_per_mtile_y;
        let mtile_y3 = 3 * tiles_per_mtile_y;

        let rgn_entry_size = 5;
        // Macrotile stride in 32-bit words
        let rgn_size = align(rgn_entry_size * tiles_per_mtile * utiles_per_tile, 4) / 4;
        let tilemap_size = (4 * rgn_size * mtiles * layers) as usize;

        let tpc_entry_size = 8;
        // TPC stride in 32-bit words
        let tpc_mtile_stride = tpc_entry_size * utiles_per_tile * tiles_per_mtile / 4;
        let tpc_size = (num_clusters * (4 * tpc_mtile_stride * mtiles) * layers) as usize;

        // No idea where this comes from, but it fits what macOS does...
        // GUESS: Number of 32K heap blocks to fit a 5-byte region header/pointer per tile?
        // That would make a ton of sense...
        // TODO: Layers? Why the sample count factor here?
        let meta1_blocks = if num_clusters > 1 {
            div_ceil(
                align(tiles_x, 2) * align(tiles_y, 4) * utiles_per_tile,
                0x1980,
            )
        } else {
            0
        };

        let mut min_tvb_blocks = align(div_ceil(tiles_x * tiles_y, 128), 8);

        if num_clusters > 1 {
            min_tvb_blocks = min_tvb_blocks.max(7 + 2 * layers);
        }

        Ok(buffer::TileInfo {
            tiles_x,
            tiles_y,
            tiles,
            utile_width,
            utile_height,
            //mtiles_x,
            //mtiles_y,
            tiles_per_mtile_x,
            tiles_per_mtile_y,
            //tiles_per_mtile,
            utiles_per_mtile_x: tiles_per_mtile_x * utiles_per_tile_x,
            utiles_per_mtile_y: tiles_per_mtile_y * utiles_per_tile_y,
            //utiles_per_mtile: tiles_per_mtile * utiles_per_tile,
            tilemap_size,
            tpc_size,
            meta1_blocks,
            layermeta_size: if layers > 1 { 0x100 } else { 0 },
            min_tvb_blocks: min_tvb_blocks as usize,
            params: fw::vertex::raw::TilingParameters {
                rgn_size,
                unk_4: 0x88,
                ppp_ctrl: cmdbuf.ppp_ctrl,
                x_max: (width - 1) as u16,
                y_max: (height - 1) as u16,
                te_screen: ((tiles_y - 1) << 12) | (tiles_x - 1),
                te_mtile1: mtile_x3 | (mtile_x2 << 9) | (mtile_x1 << 18),
                te_mtile2: mtile_y3 | (mtile_y2 << 9) | (mtile_y1 << 18),
                tiles_per_mtile,
                tpc_stride: tpc_mtile_stride,
                unk_24: 0x100,
                unk_28: if layers > 1 {
                    0xe000 | (layers - 1)
                } else {
                    0x8000
                },
                __pad: Default::default(),
            },
        })
    }

    /// Submit work to a render queue.
    pub(super) fn submit_render(
        &self,
        job: &mut Job<super::QueueJob::ver>,
        cmd: &uapi::drm_asahi_command,
        result_writer: Option<super::ResultWriter>,
        id: u64,
        flush_stamps: bool,
    ) -> Result {
        if cmd.cmd_type != uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER {
            return Err(EINVAL);
        }

        mod_dev_dbg!(self.dev, "[Submission {}] Render!\n", id);

        let mut cmdbuf_reader = unsafe {
            UserSlicePtr::new(
                cmd.cmd_buffer as usize as *mut _,
                core::mem::size_of::<uapi::drm_asahi_cmd_render>(),
            )
            .reader()
        };

        let mut cmdbuf: MaybeUninit<uapi::drm_asahi_cmd_render> = MaybeUninit::uninit();
        unsafe {
            cmdbuf_reader.read_raw(
                cmdbuf.as_mut_ptr() as *mut u8,
                core::mem::size_of::<uapi::drm_asahi_cmd_render>(),
            )?;
        }
        let cmdbuf = unsafe { cmdbuf.assume_init() };

        if cmdbuf.flags
            & !(uapi::ASAHI_RENDER_NO_CLEAR_PIPELINE_TEXTURES
                | uapi::ASAHI_RENDER_SET_WHEN_RELOADING_Z_OR_S
                | uapi::ASAHI_RENDER_SYNC_TVB_GROWTH
                | uapi::ASAHI_RENDER_PROCESS_EMPTY_TILES
                | uapi::ASAHI_RENDER_NO_VERTEX_CLUSTERING
                | uapi::ASAHI_RENDER_MSAA_ZS) as u64
            != 0
        {
            return Err(EINVAL);
        }

        if cmdbuf.fb_width == 0
            || cmdbuf.fb_height == 0
            || cmdbuf.fb_width > 16384
            || cmdbuf.fb_height > 16384
        {
            mod_dev_dbg!(
                self.dev,
                "[Submission {}] Invalid dimensions {}x{}\n",
                id,
                cmdbuf.fb_width,
                cmdbuf.fb_height
            );
            return Err(EINVAL);
        }

        let mut unks: uapi::drm_asahi_cmd_render_unknowns = Default::default();

        let mut ext_ptr = cmdbuf.extensions;
        while ext_ptr != 0 {
            let ext_type = u32::from_ne_bytes(
                unsafe { UserSlicePtr::new(ext_ptr as usize as *mut _, 4) }
                    .read_all()?
                    .try_into()
                    .or(Err(EINVAL))?,
            );

            match ext_type {
                uapi::ASAHI_RENDER_EXT_UNKNOWNS => {
                    if !debug_enabled(debug::DebugFlags::AllowUnknownOverrides) {
                        return Err(EINVAL);
                    }
                    let mut ext_reader = unsafe {
                        UserSlicePtr::new(
                            ext_ptr as usize as *mut _,
                            core::mem::size_of::<uapi::drm_asahi_cmd_render_unknowns>(),
                        )
                        .reader()
                    };
                    unsafe {
                        ext_reader.read_raw(
                            &mut unks as *mut _ as *mut u8,
                            core::mem::size_of::<uapi::drm_asahi_cmd_render_unknowns>(),
                        )?;
                    }

                    ext_ptr = unks.next;
                }
                _ => return Err(EINVAL),
            }
        }

        if unks.pad != 0 {
            return Err(EINVAL);
        }

        let dev = self.dev.data();
        let gpu = match dev.gpu.as_any().downcast_ref::<gpu::GpuManager::ver>() {
            Some(gpu) => gpu,
            None => {
                dev_crit!(self.dev.as_ref(), "GpuManager mismatched with Queue!\n");
                return Err(EIO);
            }
        };

        let nclusters = gpu.get_dyncfg().id.num_clusters;

        // Can be set to false to disable clustering (for simpler jobs), but then the
        // core masks below should be adjusted to cover a single rolling cluster.
        let mut clustering = nclusters > 1;

        if debug_enabled(debug::DebugFlags::DisableClustering)
            || cmdbuf.flags & uapi::ASAHI_RENDER_NO_VERTEX_CLUSTERING as u64 != 0
        {
            clustering = false;
        }

        #[ver(G != G14)]
        let mut tiling_control = {
            let render_cfg = gpu.get_cfg().render;
            let mut tiling_control = render_cfg.tiling_control;

            if !clustering {
                tiling_control |= TILECTL_DISABLE_CLUSTERING;
            }
            tiling_control
        };

        let mut alloc = gpu.alloc();
        let kalloc = &mut *alloc;

        // This sequence number increases per new client/VM? assigned to some slot,
        // but it's unclear *which* slot...
        let slot_client_seq: u8 = (self.id & 0xff) as u8;

        let tile_info = Self::get_tiling_params(&cmdbuf, if clustering { nclusters } else { 1 })?;

        let buffer = self.buffer.as_ref().ok_or(EINVAL)?;

        let notifier = self.notifier.clone();

        let tvb_autogrown = buffer.auto_grow()?;
        if tvb_autogrown {
            let new_size = buffer.block_count() as usize;
            cls_dev_dbg!(
                TVBStats,
                &self.dev,
                "[Submission {}] TVB grew to {} bytes ({} blocks) due to overflows\n",
                id,
                new_size * buffer::BLOCK_SIZE,
                new_size,
            );
        }

        let tvb_grown = buffer.ensure_blocks(tile_info.min_tvb_blocks)?;
        if tvb_grown {
            cls_dev_dbg!(
                TVBStats,
                &self.dev,
                "[Submission {}] TVB grew to {} bytes ({} blocks) due to dimensions ({}x{})\n",
                id,
                tile_info.min_tvb_blocks * buffer::BLOCK_SIZE,
                tile_info.min_tvb_blocks,
                cmdbuf.fb_width,
                cmdbuf.fb_height
            );
        }

        let scene = Arc::new(buffer.new_scene(kalloc, &tile_info)?, GFP_KERNEL)?;

        let vm_bind = job.vm_bind.clone();

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] VM slot = {}\n",
            id,
            vm_bind.slot()
        );

        let ev_vtx = job.get_vtx()?.event_info();
        let ev_frag = job.get_frag()?.event_info();

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Vert event #{} -> {:#x?}\n",
            id,
            ev_vtx.slot,
            ev_vtx.value.next(),
        );
        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Frag event #{} -> {:#x?}\n",
            id,
            ev_frag.slot,
            ev_frag.value.next(),
        );

        let uuid_3d = cmdbuf.cmd_3d_id;
        let uuid_ta = cmdbuf.cmd_ta_id;

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Vert UUID = {:#x?}\n",
            id,
            uuid_ta
        );
        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Frag UUID = {:#x?}\n",
            id,
            uuid_3d
        );

        let fence = job.fence.clone();
        let frag_job = job.get_frag()?;

        mod_dev_dbg!(self.dev, "[Submission {}] Create Barrier\n", id);
        let barrier = kalloc.private.new_init(
            kernel::init::zeroed::<fw::workqueue::Barrier>(),
            |_inner, _p| {
                try_init!(fw::workqueue::raw::Barrier {
                    tag: fw::workqueue::CommandType::Barrier,
                    wait_stamp: ev_vtx.fw_stamp_pointer,
                    wait_value: ev_vtx.value.next(),
                    wait_slot: ev_vtx.slot,
                    stamp_self: ev_frag.value.next(),
                    uuid: uuid_3d,
                    barrier_type: 0,
                    padding: Default::default(),
                })
            },
        )?;

        mod_dev_dbg!(self.dev, "[Submission {}] Add Barrier\n", id);
        frag_job.add(barrier, vm_bind.slot())?;

        let timestamps = Arc::new(
            kalloc.shared.new_default::<fw::job::RenderTimestamps>()?,
            GFP_KERNEL,
        )?;

        let unk1 = unks.flags & uapi::ASAHI_RENDER_UNK_UNK1 as u64 != 0;

        let mut tile_config: u64 = 0;
        if !unk1 {
            tile_config |= 0x280;
        }
        if cmdbuf.layers > 1 {
            tile_config |= 1;
        }
        if cmdbuf.flags & uapi::ASAHI_RENDER_PROCESS_EMPTY_TILES as u64 != 0 {
            tile_config |= 0x10000;
        }

        let mut utile_config =
            ((tile_info.utile_width / 16) << 12) | ((tile_info.utile_height / 16) << 14);
        utile_config |= match cmdbuf.samples {
            1 => 0,
            2 => 1,
            4 => 2,
            _ => return Err(EINVAL),
        };

        #[ver(G >= G14X)]
        let mut frg_tilecfg = 0x0000000_00036011
            | (((tile_info.tiles_x - 1) as u64) << 44)
            | (((tile_info.tiles_y - 1) as u64) << 53)
            | (if unk1 { 0 } else { 0x20_00000000 })
            | ((utile_config as u64 & 0xf000) << 28);

        let frag_result = result_writer
            .map(|writer| {
                let mut result = RenderResult {
                    result: Default::default(),
                    vtx_complete: false,
                    frag_complete: false,
                    vtx_error: None,
                    frag_error: None,
                    writer,
                };

                if tvb_autogrown {
                    result.result.flags |= uapi::DRM_ASAHI_RESULT_RENDER_TVB_GROW_OVF as u64;
                }
                if tvb_grown {
                    result.result.flags |= uapi::DRM_ASAHI_RESULT_RENDER_TVB_GROW_MIN as u64;
                }
                result.result.tvb_size_bytes = buffer.size() as u64;

                Arc::pin_init(new_mutex!(result, "render result"), GFP_KERNEL)
            })
            .transpose()?;

        let vtx_result = frag_result.clone();

        // TODO: check
        #[ver(V >= V13_0B4)]
        let count_frag = self.counter.fetch_add(2, Ordering::Relaxed);
        #[ver(V >= V13_0B4)]
        let count_vtx = count_frag + 1;

        // Unknowns handling

        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_TILE_CONFIG as u64 != 0 {
            tile_config = unks.tile_config;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_UTILE_CONFIG as u64 != 0 {
            utile_config = unks.utile_config as u32;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_AUX_FB_UNK as u64 == 0 {
            unks.aux_fb_unk = 0x100000;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_G14_UNK as u64 == 0 {
            #[ver(G >= G14)]
            unks.g14_unk = 0x4040404;
            #[ver(G < G14)]
            unks.g14_unk = 0;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_FRG_UNK_140 as u64 == 0 {
            unks.frg_unk_140 = 0x8c60;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_FRG_UNK_158 as u64 == 0 {
            unks.frg_unk_158 = 0x1c;
        }
        #[ver(G >= G14X)]
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_FRG_TILECFG as u64 != 0 {
            frg_tilecfg = unks.frg_tilecfg;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_LOAD_BGOBJVALS as u64 == 0 {
            unks.load_bgobjvals = cmdbuf.isp_bgobjvals.into();
            #[ver(G < G14)]
            unks.load_bgobjvals |= 0x400;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_FRG_UNK_38 as u64 == 0 {
            unks.frg_unk_38 = 0;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_FRG_UNK_3C as u64 == 0 {
            unks.frg_unk_3c = 1;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_RELOAD_ZLSCTRL as u64 == 0 {
            unks.reload_zlsctrl = cmdbuf.zls_ctrl;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_UNK_BUF_10 as u64 == 0 {
            #[ver(G < G14X)]
            unks.unk_buf_10 = 1;
            #[ver(G >= G14X)]
            unks.unk_buf_10 = 0;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_FRG_UNK_MASK as u64 == 0 {
            unks.frg_unk_mask = 0xffffffff;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_IOGPU_UNK54 == 0 {
            unks.iogpu_unk54 = 0x3a0012006b0003;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_IOGPU_UNK56 == 0 {
            unks.iogpu_unk56 = 1;
        }
        #[ver(G != G14)]
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_TILING_CONTROL != 0 {
            tiling_control = unks.tiling_control as u32;
        }
        #[ver(G != G14)]
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_TILING_CONTROL_2 == 0 {
            #[ver(G < G14X)]
            unks.tiling_control_2 = 0;
            #[ver(G >= G14X)]
            unks.tiling_control_2 = 4;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_VTX_UNK_F0 == 0 {
            unks.vtx_unk_f0 = 0x1c;
            #[ver(G < G14X)]
            unks.vtx_unk_f0 += align(tile_info.meta1_blocks, 4) as u64;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_VTX_UNK_F8 == 0 {
            unks.vtx_unk_f8 = 0x8c60;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_VTX_UNK_118 == 0 {
            unks.vtx_unk_118 = 0x1c;
        }
        if unks.flags & uapi::ASAHI_RENDER_UNK_SET_VTX_UNK_MASK == 0 {
            unks.vtx_unk_mask = 0xffffffff;
        }

        mod_dev_dbg!(self.dev, "[Submission {}] Create Frag\n", id);
        let frag = GpuObject::new_init_prealloc(
            kalloc.gpu_ro.alloc_object()?,
            |ptr: GpuWeakPointer<fw::fragment::RunFragment::ver>| {
                let has_result = frag_result.is_some();
                let scene = scene.clone();
                let notifier = notifier.clone();
                let vm_bind = vm_bind.clone();
                let timestamps = timestamps.clone();
                let private = &mut kalloc.private;
                try_init!(fw::fragment::RunFragment::ver {
                    micro_seq: {
                        let mut builder = microseq::Builder::new();

                        let stats = inner_weak_ptr!(
                            gpu.initdata.runtime_pointers.stats.frag.weak_pointer(),
                            stats
                        );

                        let start_frag = builder.add(microseq::StartFragment::ver {
                            header: microseq::op::StartFragment::HEADER,
                            #[ver(G < G14X)]
                            job_params2: Some(inner_weak_ptr!(ptr, job_params2)),
                            #[ver(G < G14X)]
                            job_params1: Some(inner_weak_ptr!(ptr, job_params1)),
                            #[ver(G >= G14X)]
                            job_params1: None,
                            #[ver(G >= G14X)]
                            job_params2: None,
                            #[ver(G >= G14X)]
                            registers: inner_weak_ptr!(ptr, registers),
                            scene: scene.gpu_pointer(),
                            stats,
                            busy_flag: inner_weak_ptr!(ptr, busy_flag),
                            tvb_overflow_count: inner_weak_ptr!(ptr, tvb_overflow_count),
                            unk_pointer: inner_weak_ptr!(ptr, unk_pointee),
                            work_queue: ev_frag.info_ptr,
                            work_item: ptr,
                            vm_slot: vm_bind.slot(),
                            unk_50: 0x1, // fixed
                            event_generation: self.id as u32,
                            buffer_slot: scene.slot(),
                            sync_grow: (cmdbuf.flags & uapi::ASAHI_RENDER_SYNC_TVB_GROWTH as u64
                                != 0) as u32,
                            event_seq: U64(ev_frag.event_seq),
                            unk_68: 0,
                            unk_758_flag: inner_weak_ptr!(ptr, unk_758_flag),
                            unk_job_buf: inner_weak_ptr!(ptr, unk_buf_0),
                            #[ver(V >= V13_3)]
                            unk_7c_0: U64(0),
                            unk_7c: 0,
                            unk_80: 0,
                            unk_84: unk1.into(),
                            uuid: uuid_3d,
                            attachments: common::build_attachments(
                                cmdbuf.fragment_attachments,
                                cmdbuf.fragment_attachment_count,
                            )?,
                            padding: 0,
                            #[ver(V >= V13_0B4)]
                            counter: U64(count_frag),
                            #[ver(V >= V13_0B4)]
                            notifier_buf: inner_weak_ptr!(notifier.weak_pointer(), state.unk_buf),
                        })?;

                        if has_result {
                            builder.add(microseq::Timestamp::ver {
                                header: microseq::op::Timestamp::new(true),
                                cur_ts: inner_weak_ptr!(ptr, cur_ts),
                                start_ts: inner_weak_ptr!(ptr, start_ts),
                                update_ts: inner_weak_ptr!(ptr, start_ts),
                                work_queue: ev_frag.info_ptr,
                                unk_24: U64(0),
                                #[ver(V >= V13_0B4)]
                                unk_ts: inner_weak_ptr!(ptr, unk_ts),
                                uuid: uuid_3d,
                                unk_30_padding: 0,
                            })?;
                        }

                        #[ver(G < G14X)]
                        builder.add(microseq::WaitForIdle {
                            header: microseq::op::WaitForIdle::new(microseq::Pipe::Fragment),
                        })?;
                        #[ver(G >= G14X)]
                        builder.add(microseq::WaitForIdle2 {
                            header: microseq::op::WaitForIdle2::HEADER,
                        })?;

                        if has_result {
                            builder.add(microseq::Timestamp::ver {
                                header: microseq::op::Timestamp::new(false),
                                cur_ts: inner_weak_ptr!(ptr, cur_ts),
                                start_ts: inner_weak_ptr!(ptr, start_ts),
                                update_ts: inner_weak_ptr!(ptr, end_ts),
                                work_queue: ev_frag.info_ptr,
                                unk_24: U64(0),
                                #[ver(V >= V13_0B4)]
                                unk_ts: inner_weak_ptr!(ptr, unk_ts),
                                uuid: uuid_3d,
                                unk_30_padding: 0,
                            })?;
                        }

                        let off = builder.offset_to(start_frag);
                        builder.add(microseq::FinalizeFragment::ver {
                            header: microseq::op::FinalizeFragment::HEADER,
                            uuid: uuid_3d,
                            unk_8: 0,
                            fw_stamp: ev_frag.fw_stamp_pointer,
                            stamp_value: ev_frag.value.next(),
                            unk_18: 0,
                            scene: scene.weak_pointer(),
                            buffer: scene.weak_buffer_pointer(),
                            unk_2c: U64(1),
                            stats,
                            unk_pointer: inner_weak_ptr!(ptr, unk_pointee),
                            busy_flag: inner_weak_ptr!(ptr, busy_flag),
                            work_queue: ev_frag.info_ptr,
                            work_item: ptr,
                            vm_slot: vm_bind.slot(),
                            unk_60: 0,
                            unk_758_flag: inner_weak_ptr!(ptr, unk_758_flag),
                            #[ver(V >= V13_3)]
                            unk_6c_0: U64(0),
                            unk_6c: U64(0),
                            unk_74: U64(0),
                            unk_7c: U64(0),
                            unk_84: U64(0),
                            unk_8c: U64(0),
                            #[ver(G == G14 && V < V13_0B4)]
                            unk_8c_g14: U64(0),
                            restart_branch_offset: off,
                            has_attachments: (cmdbuf.fragment_attachment_count > 0) as u32,
                            #[ver(V >= V13_0B4)]
                            unk_9c: Default::default(),
                        })?;

                        builder.add(microseq::RetireStamp {
                            header: microseq::op::RetireStamp::HEADER,
                        })?;

                        builder.build(private)?
                    },
                    notifier,
                    scene,
                    vm_bind,
                    aux_fb: self.ualloc.lock().array_empty_tagged(0x8000, b"AXFB")?,
                    timestamps,
                })
            },
            |inner, _ptr| {
                let vm_slot = vm_bind.slot();
                let aux_fb_info = fw::fragment::raw::AuxFBInfo::ver {
                    iogpu_unk_214: cmdbuf.iogpu_unk_214,
                    unk2: 0,
                    width: cmdbuf.fb_width,
                    height: cmdbuf.fb_height,
                    #[ver(V >= V13_0B4)]
                    unk3: U64(unks.aux_fb_unk),
                };

                try_init!(fw::fragment::raw::RunFragment::ver {
                    tag: fw::workqueue::CommandType::RunFragment,
                    #[ver(V >= V13_0B4)]
                    counter: U64(count_frag),
                    vm_slot,
                    unk_8: 0,
                    microsequence: inner.micro_seq.gpu_pointer(),
                    microsequence_size: inner.micro_seq.len() as u32,
                    notifier: inner.notifier.gpu_pointer(),
                    buffer: inner.scene.buffer_pointer(),
                    scene: inner.scene.gpu_pointer(),
                    unk_buffer_buf: inner.scene.kernel_buffer_pointer(),
                    tvb_tilemap: inner.scene.tvb_tilemap_pointer(),
                    ppp_multisamplectl: U64(cmdbuf.ppp_multisamplectl),
                    samples: cmdbuf.samples,
                    tiles_per_mtile_y: tile_info.tiles_per_mtile_y as u16,
                    tiles_per_mtile_x: tile_info.tiles_per_mtile_x as u16,
                    unk_50: U64(0),
                    unk_58: U64(0),
                    merge_upper_x: F32::from_bits(cmdbuf.merge_upper_x),
                    merge_upper_y: F32::from_bits(cmdbuf.merge_upper_y),
                    unk_68: U64(0),
                    tile_count: U64(tile_info.tiles as u64),
                    #[ver(G < G14X)]
                    job_params1 <- try_init!(fw::fragment::raw::JobParameters1::ver {
                        utile_config,
                        unk_4: 0,
                        clear_pipeline: fw::fragment::raw::ClearPipelineBinding {
                            pipeline_bind: U64(cmdbuf.load_pipeline_bind as u64),
                            address: U64(cmdbuf.load_pipeline as u64),
                        },
                        ppp_multisamplectl: U64(cmdbuf.ppp_multisamplectl),
                        scissor_array: U64(cmdbuf.scissor_array),
                        depth_bias_array: U64(cmdbuf.depth_bias_array),
                        aux_fb_info,
                        depth_dimensions: U64(cmdbuf.depth_dimensions as u64),
                        visibility_result_buffer: U64(cmdbuf.visibility_result_buffer),
                        zls_ctrl: U64(cmdbuf.zls_ctrl),
                        #[ver(G >= G14)]
                        unk_58_g14_0: U64(unks.g14_unk),
                        #[ver(G >= G14)]
                        unk_58_g14_8: U64(0),
                        depth_buffer_ptr1: U64(cmdbuf.depth_buffer_load),
                        depth_buffer_ptr2: U64(cmdbuf.depth_buffer_store),
                        stencil_buffer_ptr1: U64(cmdbuf.stencil_buffer_load),
                        stencil_buffer_ptr2: U64(cmdbuf.stencil_buffer_store),
                        #[ver(G >= G14)]
                        unk_68_g14_0: Default::default(),
                        unk_78: Default::default(),
                        depth_meta_buffer_ptr1: U64(cmdbuf.depth_meta_buffer_load),
                        unk_a0: Default::default(),
                        depth_meta_buffer_ptr2: U64(cmdbuf.depth_meta_buffer_store),
                        unk_b0: Default::default(),
                        stencil_meta_buffer_ptr1: U64(cmdbuf.stencil_meta_buffer_load),
                        unk_c0: Default::default(),
                        stencil_meta_buffer_ptr2: U64(cmdbuf.stencil_meta_buffer_store),
                        unk_d0: Default::default(),
                        tvb_tilemap: inner.scene.tvb_tilemap_pointer(),
                        tvb_layermeta: inner.scene.tvb_layermeta_pointer(),
                        mtile_stride_dwords: U64((4 * tile_info.params.rgn_size as u64) << 24),
                        tvb_heapmeta: inner.scene.tvb_heapmeta_pointer(),
                        tile_config: U64(tile_config),
                        aux_fb: inner.aux_fb.gpu_pointer(),
                        unk_108: Default::default(),
                        pipeline_base: U64(0x11_00000000),
                        unk_140: U64(unks.frg_unk_140),
                        unk_148: U64(0x0),
                        unk_150: U64(0x0),
                        unk_158: U64(unks.frg_unk_158),
                        unk_160: U64(0),
                        __pad: Default::default(),
                        #[ver(V < V13_0B4)]
                        __pad1: Default::default(),
                    }),
                    #[ver(G < G14X)]
                    job_params2 <- try_init!(fw::fragment::raw::JobParameters2 {
                        store_pipeline_bind: cmdbuf.store_pipeline_bind,
                        store_pipeline_addr: cmdbuf.store_pipeline,
                        unk_8: 0x0,
                        unk_c: 0x0,
                        merge_upper_x: F32::from_bits(cmdbuf.merge_upper_x),
                        merge_upper_y: F32::from_bits(cmdbuf.merge_upper_y),
                        unk_18: U64(0x0),
                        utiles_per_mtile_y: tile_info.utiles_per_mtile_y as u16,
                        utiles_per_mtile_x: tile_info.utiles_per_mtile_x as u16,
                        unk_24: 0x0,
                        tile_counts: ((tile_info.tiles_y - 1) << 12) | (tile_info.tiles_x - 1),
                        tib_blocks: cmdbuf.tib_blocks,
                        isp_bgobjdepth: cmdbuf.isp_bgobjdepth,
                        // TODO: does this flag need to be exposed to userspace?
                        isp_bgobjvals: unks.load_bgobjvals as u32,
                        unk_38: unks.frg_unk_38 as u32,
                        unk_3c: unks.frg_unk_3c as u32,
                        unk_40: unks.frg_unk_40 as u32,
                        __pad: Default::default(),
                    }),
                    #[ver(G >= G14X)]
                    registers: fw::job::raw::RegisterArray::new(
                        inner_weak_ptr!(_ptr, registers.registers),
                        |r| {
                            r.add(0x1739, 1);
                            r.add(0x10009, utile_config.into());
                            r.add(0x15379, cmdbuf.store_pipeline_bind.into());
                            r.add(0x15381, cmdbuf.store_pipeline.into());
                            r.add(0x15369, cmdbuf.load_pipeline_bind.into());
                            r.add(0x15371, cmdbuf.load_pipeline.into());
                            r.add(0x15131, cmdbuf.merge_upper_x.into());
                            r.add(0x15139, cmdbuf.merge_upper_y.into());
                            r.add(0x100a1, 0);
                            r.add(0x15069, 0);
                            r.add(0x15071, 0); // pointer
                            r.add(0x16058, 0);
                            r.add(0x10019, cmdbuf.ppp_multisamplectl);
                            let isp_mtile_size = (tile_info.utiles_per_mtile_y
                                | (tile_info.utiles_per_mtile_x << 16))
                                .into();
                            r.add(0x100b1, isp_mtile_size); // ISP_MTILE_SIZE
                            r.add(0x16030, isp_mtile_size); // ISP_MTILE_SIZE
                            r.add(
                                0x100d9,
                                (((tile_info.tiles_y - 1) << 12) | (tile_info.tiles_x - 1)).into(),
                            ); // TE_SCREEN
                            r.add(0x16098, inner.scene.tvb_heapmeta_pointer().into());
                            r.add(0x15109, cmdbuf.scissor_array); // ISP_SCISSOR_BASE
                            r.add(0x15101, cmdbuf.depth_bias_array); // ISP_DBIAS_BASE
                            r.add(0x15021, cmdbuf.iogpu_unk_214.into()); // aux_fb_info.unk_1
                            r.add(
                                0x15211,
                                ((cmdbuf.fb_height as u64) << 32) | cmdbuf.fb_width as u64,
                            ); // aux_fb_info.{width, heigh
                            r.add(0x15049, unks.aux_fb_unk); // s2.aux_fb_info.unk3
                            r.add(0x10051, cmdbuf.tib_blocks.into()); // s1.unk_2c
                            r.add(0x15321, cmdbuf.depth_dimensions.into()); // ISP_ZLS_PIXELS
                            r.add(0x15301, cmdbuf.isp_bgobjdepth.into()); // ISP_BGOBJDEPTH
                            r.add(0x15309, unks.load_bgobjvals); // ISP_BGOBJVALS
                            r.add(0x15311, cmdbuf.visibility_result_buffer); // ISP_OCLQRY_BASE
                            r.add(0x15319, cmdbuf.zls_ctrl); // ISP_ZLSCTL
                            r.add(0x15349, unks.g14_unk); // s2.unk_58_g14_0
                            r.add(0x15351, 0); // s2.unk_58_g14_8
                            r.add(0x15329, cmdbuf.depth_buffer_load); // ISP_ZLOAD_BASE
                            r.add(0x15331, cmdbuf.depth_buffer_store); // ISP_ZSTORE_BASE
                            r.add(0x15339, cmdbuf.stencil_buffer_load); // ISP_STENCIL_LOAD_BASE
                            r.add(0x15341, cmdbuf.stencil_buffer_store); // ISP_STENCIL_STORE_BASE
                            r.add(0x15231, 0);
                            r.add(0x15221, 0);
                            r.add(0x15239, 0);
                            r.add(0x15229, 0);
                            r.add(0x15401, 0);
                            r.add(0x15421, 0);
                            r.add(0x15409, 0);
                            r.add(0x15429, 0);
                            r.add(0x153c1, cmdbuf.depth_meta_buffer_load);
                            r.add(0x15411, 0);
                            r.add(0x153c9, cmdbuf.depth_meta_buffer_store);
                            r.add(0x15431, 0);
                            r.add(0x153d1, cmdbuf.stencil_meta_buffer_load);
                            r.add(0x15419, 0);
                            r.add(0x153d9, cmdbuf.stencil_meta_buffer_store);
                            r.add(0x15439, 0);
                            r.add(0x16429, inner.scene.tvb_tilemap_pointer().into());
                            r.add(0x16060, inner.scene.tvb_layermeta_pointer().into());
                            r.add(0x16431, (4 * tile_info.params.rgn_size as u64) << 24); // ISP_RGN?
                            r.add(0x10039, tile_config); // tile_config ISP_CTL?
                            r.add(0x16451, 0x0); // ISP_RENDER_ORIGIN
                            r.add(0x11821, 0x0); // some shader?
                            r.add(0x11829, 0);
                            r.add(0x11f79, 0);
                            r.add(0x15359, 0);
                            r.add(0x10069, 0x11_00000000); // USC_EXEC_BASE_ISP
                            r.add(0x16020, 0);
                            r.add(0x16461, inner.aux_fb.gpu_pointer().into());
                            r.add(0x16090, inner.aux_fb.gpu_pointer().into());
                            r.add(0x120a1, unks.frg_unk_158);
                            r.add(0x160a8, 0);
                            r.add(0x16068, frg_tilecfg);
                            r.add(0x160b8, 0x0);
                            /*
                            r.add(0x10201, 0x100); // Some kind of counter?? Does this matter?
                            r.add(0x10428, 0x100); // Some kind of counter?? Does this matter?
                            r.add(0x1c838, 1);  // ?
                            r.add(0x1ca28, 0x1502960f00); // ??
                            r.add(0x1731, 0x1); // ??
                            */
                        }
                    ),
                    job_params3 <- try_init!(fw::fragment::raw::JobParameters3::ver {
                        depth_bias_array: fw::fragment::raw::ArrayAddr {
                            ptr: U64(cmdbuf.depth_bias_array),
                            unk_padding: U64(0),
                        },
                        scissor_array: fw::fragment::raw::ArrayAddr {
                            ptr: U64(cmdbuf.scissor_array),
                            unk_padding: U64(0),
                        },
                        visibility_result_buffer: U64(cmdbuf.visibility_result_buffer),
                        unk_118: U64(0x0),
                        unk_120: Default::default(),
                        unk_reload_pipeline: fw::fragment::raw::ClearPipelineBinding {
                            pipeline_bind: U64(cmdbuf.partial_reload_pipeline_bind as u64),
                            address: U64(cmdbuf.partial_reload_pipeline as u64),
                        },
                        unk_258: U64(0),
                        unk_260: U64(0),
                        unk_268: U64(0),
                        unk_270: U64(0),
                        reload_pipeline: fw::fragment::raw::ClearPipelineBinding {
                            pipeline_bind: U64(cmdbuf.partial_reload_pipeline_bind as u64),
                            address: U64(cmdbuf.partial_reload_pipeline as u64),
                        },
                        zls_ctrl: U64(unks.reload_zlsctrl),
                        unk_290: U64(unks.g14_unk),
                        depth_buffer_ptr1: U64(cmdbuf.depth_buffer_load),
                        unk_2a0: U64(0x0),
                        unk_2a8: U64(0x0),
                        depth_buffer_ptr2: U64(cmdbuf.depth_buffer_store),
                        depth_buffer_ptr3: U64(cmdbuf.depth_buffer_partial),
                        depth_meta_buffer_ptr3: U64(cmdbuf.depth_meta_buffer_partial),
                        stencil_buffer_ptr1: U64(cmdbuf.stencil_buffer_load),
                        unk_2d0: U64(0x0),
                        unk_2d8: U64(0x0),
                        stencil_buffer_ptr2: U64(cmdbuf.stencil_buffer_store),
                        stencil_buffer_ptr3: U64(cmdbuf.stencil_buffer_partial),
                        stencil_meta_buffer_ptr3: U64(cmdbuf.stencil_meta_buffer_partial),
                        unk_2f8: Default::default(),
                        tib_blocks: cmdbuf.tib_blocks,
                        unk_30c: 0x0,
                        aux_fb_info,
                        tile_config: U64(tile_config),
                        unk_328_padding: Default::default(),
                        unk_partial_store_pipeline: fw::fragment::raw::StorePipelineBinding::new(
                            cmdbuf.partial_store_pipeline_bind,
                            cmdbuf.partial_store_pipeline
                        ),
                        partial_store_pipeline: fw::fragment::raw::StorePipelineBinding::new(
                            cmdbuf.partial_store_pipeline_bind,
                            cmdbuf.partial_store_pipeline
                        ),
                        isp_bgobjdepth: cmdbuf.isp_bgobjdepth,
                        isp_bgobjvals: cmdbuf.isp_bgobjvals,
                        sample_size: cmdbuf.sample_size,
                        unk_37c: 0x0,
                        unk_380: U64(0x0),
                        unk_388: U64(0x0),
                        #[ver(V >= V13_0B4)]
                        unk_390_0: U64(0x0),
                        depth_dimensions: U64(cmdbuf.depth_dimensions as u64),
                    }),
                    unk_758_flag: 0,
                    unk_75c_flag: 0,
                    unk_buf: Default::default(),
                    busy_flag: 0,
                    tvb_overflow_count: 0,
                    unk_878: 0,
                    encoder_params <- try_init!(fw::job::raw::EncoderParams {
                        unk_8: (cmdbuf.flags & uapi::ASAHI_RENDER_SET_WHEN_RELOADING_Z_OR_S as u64
                            != 0) as u32,
                        sync_grow: (cmdbuf.flags & uapi::ASAHI_RENDER_SYNC_TVB_GROWTH as u64
                                    != 0) as u32,
                        unk_10: 0x0, // fixed
                        encoder_id: cmdbuf.encoder_id,
                        unk_18: 0x0, // fixed
                        unk_mask: unks.frg_unk_mask as u32,
                        sampler_array: U64(cmdbuf.fragment_sampler_array),
                        sampler_count: cmdbuf.fragment_sampler_count,
                        sampler_max: cmdbuf.fragment_sampler_max,
                    }),
                    process_empty_tiles: (cmdbuf.flags
                        & uapi::ASAHI_RENDER_PROCESS_EMPTY_TILES as u64
                        != 0) as u32,
                    no_clear_pipeline_textures: (cmdbuf.flags
                        & uapi::ASAHI_RENDER_NO_CLEAR_PIPELINE_TEXTURES as u64
                        != 0) as u32,
                    msaa_zs: (cmdbuf.flags & uapi::ASAHI_RENDER_MSAA_ZS as u64 != 0) as u32,
                    unk_pointee: 0,
                    #[ver(V >= V13_3)]
                    unk_v13_3: 0,
                    meta <- try_init!(fw::job::raw::JobMeta {
                        unk_0: 0,
                        unk_2: 0,
                        no_preemption: 0,
                        stamp: ev_frag.stamp_pointer,
                        fw_stamp: ev_frag.fw_stamp_pointer,
                        stamp_value: ev_frag.value.next(),
                        stamp_slot: ev_frag.slot,
                        evctl_index: 0, // fixed
                        flush_stamps: flush_stamps as u32,
                        uuid: uuid_3d,
                        event_seq: ev_frag.event_seq as u32,
                    }),
                    unk_after_meta: unk1.into(),
                    unk_buf_0: U64(0),
                    unk_buf_8: U64(0),
                    #[ver(G < G14X)]
                    unk_buf_10: U64(1),
                    #[ver(G >= G14X)]
                    unk_buf_10: U64(0),
                    cur_ts: U64(0),
                    start_ts: Some(inner_ptr!(inner.timestamps.gpu_pointer(), frag.start)),
                    end_ts: Some(inner_ptr!(inner.timestamps.gpu_pointer(), frag.end)),
                    unk_914: 0,
                    unk_918: U64(0),
                    unk_920: 0,
                    client_sequence: slot_client_seq,
                    pad_925: Default::default(),
                    unk_928: 0,
                    unk_92c: 0,
                    #[ver(V >= V13_0B4)]
                    unk_ts: U64(0),
                    #[ver(V >= V13_0B4)]
                    unk_92d_8: Default::default(),
                })
            },
        )?;

        mod_dev_dbg!(self.dev, "[Submission {}] Add Frag\n", id);
        fence.add_command();

        frag_job.add_cb(frag, vm_bind.slot(), move |cmd, error| {
            if let Some(err) = error {
                fence.set_error(err.into());
            }
            if let Some(mut res) = frag_result.as_ref().map(|a| a.lock()) {
                cmd.timestamps.with(|raw, _inner| {
                    res.result.fragment_ts_start = raw.frag.start.load(Ordering::Relaxed);
                    res.result.fragment_ts_end = raw.frag.end.load(Ordering::Relaxed);
                });
                cmd.with(|raw, _inner| {
                    res.result.num_tvb_overflows = raw.tvb_overflow_count;
                });
                res.frag_error = error;
                res.frag_complete = true;
                res.commit();
            }
            fence.command_complete();
        })?;

        let fence = job.fence.clone();
        let vtx_job = job.get_vtx()?;

        if scene.rebind() || tvb_grown || tvb_autogrown {
            mod_dev_dbg!(self.dev, "[Submission {}] Create Bind Buffer\n", id);
            let bind_buffer = kalloc.private.new_init(
                {
                    let scene = scene.clone();
                    try_init!(fw::buffer::InitBuffer::ver { scene })
                },
                |inner, _ptr| {
                    let vm_slot = vm_bind.slot();
                    try_init!(fw::buffer::raw::InitBuffer::ver {
                        tag: fw::workqueue::CommandType::InitBuffer,
                        vm_slot,
                        buffer_slot: inner.scene.slot(),
                        unk_c: 0,
                        block_count: buffer.block_count(),
                        buffer: inner.scene.buffer_pointer(),
                        stamp_value: ev_vtx.value.next(),
                    })
                },
            )?;

            mod_dev_dbg!(self.dev, "[Submission {}] Add Bind Buffer\n", id);
            vtx_job.add(bind_buffer, vm_bind.slot())?;
        }

        mod_dev_dbg!(self.dev, "[Submission {}] Create Vertex\n", id);
        let vtx = GpuObject::new_init_prealloc(
            kalloc.gpu_ro.alloc_object()?,
            |ptr: GpuWeakPointer<fw::vertex::RunVertex::ver>| {
                let has_result = vtx_result.is_some();
                let scene = scene.clone();
                let vm_bind = vm_bind.clone();
                let timestamps = timestamps.clone();
                let private = &mut kalloc.private;
                try_init!(fw::vertex::RunVertex::ver {
                    micro_seq: {
                        let mut builder = microseq::Builder::new();

                        let stats = inner_weak_ptr!(
                            gpu.initdata.runtime_pointers.stats.vtx.weak_pointer(),
                            stats
                        );

                        let start_vtx = builder.add(microseq::StartVertex::ver {
                            header: microseq::op::StartVertex::HEADER,
                            #[ver(G < G14X)]
                            tiling_params: Some(inner_weak_ptr!(ptr, tiling_params)),
                            #[ver(G < G14X)]
                            job_params1: Some(inner_weak_ptr!(ptr, job_params1)),
                            #[ver(G >= G14X)]
                            tiling_params: None,
                            #[ver(G >= G14X)]
                            job_params1: None,
                            #[ver(G >= G14X)]
                            registers: inner_weak_ptr!(ptr, registers),
                            buffer: scene.weak_buffer_pointer(),
                            scene: scene.weak_pointer(),
                            stats,
                            work_queue: ev_vtx.info_ptr,
                            vm_slot: vm_bind.slot(),
                            unk_38: 1, // fixed
                            event_generation: self.id as u32,
                            buffer_slot: scene.slot(),
                            unk_44: 0,
                            event_seq: U64(ev_vtx.event_seq),
                            unk_50: 0,
                            unk_pointer: inner_weak_ptr!(ptr, unk_pointee),
                            unk_job_buf: inner_weak_ptr!(ptr, unk_buf_0),
                            unk_64: 0x0, // fixed
                            unk_68: unk1.into(),
                            uuid: uuid_ta,
                            attachments: common::build_attachments(
                                cmdbuf.vertex_attachments,
                                cmdbuf.vertex_attachment_count,
                            )?,
                            padding: 0,
                            #[ver(V >= V13_0B4)]
                            counter: U64(count_vtx),
                            #[ver(V >= V13_0B4)]
                            notifier_buf: inner_weak_ptr!(notifier.weak_pointer(), state.unk_buf),
                            #[ver(V < V13_0B4)]
                            unk_178: 0x0, // padding?
                            #[ver(V >= V13_0B4)]
                            unk_178: (!clustering) as u32,
                        })?;

                        if has_result {
                            builder.add(microseq::Timestamp::ver {
                                header: microseq::op::Timestamp::new(true),
                                cur_ts: inner_weak_ptr!(ptr, cur_ts),
                                start_ts: inner_weak_ptr!(ptr, start_ts),
                                update_ts: inner_weak_ptr!(ptr, start_ts),
                                work_queue: ev_vtx.info_ptr,
                                unk_24: U64(0),
                                #[ver(V >= V13_0B4)]
                                unk_ts: inner_weak_ptr!(ptr, unk_ts),
                                uuid: uuid_ta,
                                unk_30_padding: 0,
                            })?;
                        }

                        #[ver(G < G14X)]
                        builder.add(microseq::WaitForIdle {
                            header: microseq::op::WaitForIdle::new(microseq::Pipe::Vertex),
                        })?;
                        #[ver(G >= G14X)]
                        builder.add(microseq::WaitForIdle2 {
                            header: microseq::op::WaitForIdle2::HEADER,
                        })?;

                        if has_result {
                            builder.add(microseq::Timestamp::ver {
                                header: microseq::op::Timestamp::new(false),
                                cur_ts: inner_weak_ptr!(ptr, cur_ts),
                                start_ts: inner_weak_ptr!(ptr, start_ts),
                                update_ts: inner_weak_ptr!(ptr, end_ts),
                                work_queue: ev_vtx.info_ptr,
                                unk_24: U64(0),
                                #[ver(V >= V13_0B4)]
                                unk_ts: inner_weak_ptr!(ptr, unk_ts),
                                uuid: uuid_ta,
                                unk_30_padding: 0,
                            })?;
                        }

                        let off = builder.offset_to(start_vtx);
                        builder.add(microseq::FinalizeVertex::ver {
                            header: microseq::op::FinalizeVertex::HEADER,
                            scene: scene.weak_pointer(),
                            buffer: scene.weak_buffer_pointer(),
                            stats,
                            work_queue: ev_vtx.info_ptr,
                            vm_slot: vm_bind.slot(),
                            unk_28: 0x0, // fixed
                            unk_pointer: inner_weak_ptr!(ptr, unk_pointee),
                            unk_34: 0x0, // fixed
                            uuid: uuid_ta,
                            fw_stamp: ev_vtx.fw_stamp_pointer,
                            stamp_value: ev_vtx.value.next(),
                            unk_48: U64(0x0), // fixed
                            unk_50: 0x0,      // fixed
                            unk_54: 0x0,      // fixed
                            unk_58: U64(0x0), // fixed
                            unk_60: 0x0,      // fixed
                            unk_64: 0x0,      // fixed
                            unk_68: 0x0,      // fixed
                            #[ver(G >= G14 && V < V13_0B4)]
                            unk_68_g14: U64(0),
                            restart_branch_offset: off,
                            has_attachments: (cmdbuf.vertex_attachment_count > 0) as u32,
                            #[ver(V >= V13_0B4)]
                            unk_74: Default::default(), // Ventura
                        })?;

                        builder.add(microseq::RetireStamp {
                            header: microseq::op::RetireStamp::HEADER,
                        })?;
                        builder.build(private)?
                    },
                    notifier,
                    scene,
                    vm_bind,
                    timestamps,
                })
            },
            |inner, _ptr| {
                let vm_slot = vm_bind.slot();
                #[ver(G < G14)]
                let core_masks = gpu.core_masks_packed();

                try_init!(fw::vertex::raw::RunVertex::ver {
                    tag: fw::workqueue::CommandType::RunVertex,
                    #[ver(V >= V13_0B4)]
                    counter: U64(count_vtx),
                    vm_slot,
                    unk_8: 0,
                    notifier: inner.notifier.gpu_pointer(),
                    buffer_slot: inner.scene.slot(),
                    unk_1c: 0,
                    buffer: inner.scene.buffer_pointer(),
                    scene: inner.scene.gpu_pointer(),
                    unk_buffer_buf: inner.scene.kernel_buffer_pointer(),
                    unk_34: 0,
                    #[ver(G < G14X)]
                    job_params1 <- try_init!(fw::vertex::raw::JobParameters1::ver {
                        unk_0: U64(if unk1 { 0 } else { 0x200 }), // sometimes 0
                        unk_8: f32!(1e-20),                       // fixed
                        unk_c: f32!(1e-20),                       // fixed
                        tvb_tilemap: inner.scene.tvb_tilemap_pointer(),
                        #[ver(G < G14)]
                        tvb_cluster_tilemaps: inner.scene.cluster_tilemaps_pointer(),
                        tpc: inner.scene.tpc_pointer(),
                        tvb_heapmeta: inner.scene.tvb_heapmeta_pointer().or(0x8000_0000_0000_0000),
                        iogpu_unk_54: U64(unks.iogpu_unk54), // fixed
                        iogpu_unk_56: U64(unks.iogpu_unk56), // fixed
                        #[ver(G < G14)]
                        tvb_cluster_meta1: inner
                            .scene
                            .meta_1_pointer()
                            .map(|x| x.or((tile_info.meta1_blocks as u64) << 50)),
                        utile_config,
                        unk_4c: 0,
                        ppp_multisamplectl: U64(cmdbuf.ppp_multisamplectl), // fixed
                        tvb_layermeta: inner.scene.tvb_layermeta_pointer(),
                        #[ver(G < G14)]
                        unk_60: U64(0x0), // fixed
                        #[ver(G < G14)]
                        core_mask: Array::new([
                            *core_masks.first().unwrap_or(&0),
                            *core_masks.get(1).unwrap_or(&0),
                        ]),
                        preempt_buf1: inner.scene.preempt_buf_1_pointer(),
                        preempt_buf2: inner.scene.preempt_buf_2_pointer(),
                        unk_80: U64(0x1), // fixed
                        preempt_buf3: inner.scene.preempt_buf_3_pointer().or(0x4_0000_0000_0000), // check
                        encoder_addr: U64(cmdbuf.encoder_ptr),
                        #[ver(G < G14)]
                        tvb_cluster_meta2: inner.scene.meta_2_pointer(),
                        #[ver(G < G14)]
                        tvb_cluster_meta3: inner.scene.meta_3_pointer(),
                        #[ver(G < G14)]
                        tiling_control,
                        #[ver(G < G14)]
                        unk_ac: unks.tiling_control_2 as u32, // fixed
                        unk_b0: Default::default(), // fixed
                        pipeline_base: U64(0x11_00000000),
                        #[ver(G < G14)]
                        tvb_cluster_meta4: inner
                            .scene
                            .meta_4_pointer()
                            .map(|x| x.or(0x3000_0000_0000_0000)),
                        #[ver(G < G14)]
                        unk_f0: U64(unks.vtx_unk_f0),
                        unk_f8: U64(unks.vtx_unk_f8),     // fixed
                        unk_100: Default::default(),      // fixed
                        unk_118: unks.vtx_unk_118 as u32, // fixed
                        __pad: Default::default(),
                    }),
                    #[ver(G < G14X)]
                    tiling_params: tile_info.params,
                    #[ver(G >= G14X)]
                    registers: fw::job::raw::RegisterArray::new(
                        inner_weak_ptr!(_ptr, registers.registers),
                        |r| {
                            r.add(0x10141, if unk1 { 0 } else { 0x200 }); // s2.unk_0
                            r.add(0x1c039, inner.scene.tvb_tilemap_pointer().into());
                            r.add(0x1c9c8, inner.scene.tvb_tilemap_pointer().into());

                            let cl_tilemaps_ptr = inner
                                .scene
                                .cluster_tilemaps_pointer()
                                .map_or(0, |a| a.into());
                            r.add(0x1c041, cl_tilemaps_ptr);
                            r.add(0x1c9d0, cl_tilemaps_ptr);
                            r.add(0x1c0a1, inner.scene.tpc_pointer().into()); // TE_TPC_ADDR

                            let tvb_heapmeta_ptr = inner
                                .scene
                                .tvb_heapmeta_pointer()
                                .or(0x8000_0000_0000_0000)
                                .into();
                            r.add(0x1c031, tvb_heapmeta_ptr);
                            r.add(0x1c9c0, tvb_heapmeta_ptr);
                            r.add(0x1c051, unks.iogpu_unk54); // iogpu_unk_54/55
                            r.add(0x1c061, unks.iogpu_unk56); // iogpu_unk_56
                            r.add(0x10149, utile_config.into()); // s2.unk_48 utile_config
                            r.add(0x10139, cmdbuf.ppp_multisamplectl); // PPP_MULTISAMPLECTL
                            r.add(0x10111, inner.scene.preempt_buf_1_pointer().into());
                            r.add(0x1c9b0, inner.scene.preempt_buf_1_pointer().into());
                            r.add(0x10119, inner.scene.preempt_buf_2_pointer().into());
                            r.add(0x1c9b8, inner.scene.preempt_buf_2_pointer().into());
                            r.add(0x1c958, 1); // s2.unk_80
                            r.add(
                                0x1c950,
                                inner
                                    .scene
                                    .preempt_buf_3_pointer()
                                    .or(0x4_0000_0000_0000)
                                    .into(),
                            );
                            r.add(0x1c930, 0); // VCE related addr, lsb to enable
                            r.add(0x1c880, cmdbuf.encoder_ptr); // VDM_CTRL_STREAM_BASE
                            r.add(0x1c898, 0x0); // if lsb set, faults in UL1C0, possibly missing addr.
                            r.add(
                                0x1c948,
                                inner.scene.meta_2_pointer().map_or(0, |a| a.into()),
                            ); // tvb_cluster_meta2
                            r.add(
                                0x1c888,
                                inner.scene.meta_3_pointer().map_or(0, |a| a.into()),
                            ); // tvb_cluster_meta3
                            r.add(0x1c890, tiling_control.into()); // tvb_tiling_control
                            r.add(0x1c918, unks.tiling_control_2);
                            r.add(0x1c079, inner.scene.tvb_layermeta_pointer().into());
                            r.add(0x1c9d8, inner.scene.tvb_layermeta_pointer().into());
                            r.add(0x1c089, 0);
                            r.add(0x1c9e0, 0);
                            let cl_meta_4_pointer =
                                inner.scene.meta_4_pointer().map_or(0, |a| a.into());
                            r.add(0x16c41, cl_meta_4_pointer); // tvb_cluster_meta4
                            r.add(0x1ca40, cl_meta_4_pointer); // tvb_cluster_meta4
                            r.add(0x1c9a8, unks.vtx_unk_f0); // + meta1_blocks? min_free_tvb_pages?
                            r.add(
                                0x1c920,
                                inner.scene.meta_1_pointer().map_or(0, |a| a.into()),
                            ); // ??? | meta1_blocks?
                            r.add(0x10151, 0);
                            r.add(0x1c199, 0);
                            r.add(0x1c1a1, 0);
                            r.add(0x1c1a9, 0); // 0x10151 bit 1 enables
                            r.add(0x1c1b1, 0);
                            r.add(0x1c1b9, 0);
                            r.add(0x10061, 0x11_00000000); // USC_EXEC_BASE_TA
                            r.add(0x11801, 0); // some shader?
                            r.add(0x11809, 0); // maybe arg?
                            r.add(0x11f71, 0);
                            r.add(0x1c0b1, tile_info.params.rgn_size.into()); // TE_PSG
                            r.add(0x1c850, tile_info.params.rgn_size.into());
                            r.add(0x10131, tile_info.params.unk_4.into());
                            r.add(0x10121, tile_info.params.ppp_ctrl.into()); // PPP_CTRL
                            r.add(
                                0x10129,
                                tile_info.params.x_max as u64
                                    | ((tile_info.params.y_max as u64) << 16),
                            ); // PPP_SCREEN
                            r.add(0x101b9, tile_info.params.te_screen.into()); // TE_SCREEN
                            r.add(0x1c069, tile_info.params.te_mtile1.into()); // TE_MTILE1
                            r.add(0x1c071, tile_info.params.te_mtile2.into()); // TE_MTILE2
                            r.add(0x1c081, tile_info.params.tiles_per_mtile.into()); // TE_MTILE
                            r.add(0x1c0a9, tile_info.params.tpc_stride.into()); // TE_TPC
                            r.add(0x10171, tile_info.params.unk_24.into());
                            r.add(0x10169, tile_info.params.unk_28.into()); // TA_RENDER_TARGET_MAX
                            r.add(0x12099, unks.vtx_unk_118);
                            r.add(0x1c9e8, 0);
                            /*
                            r.add(0x10209, 0x100); // Some kind of counter?? Does this matter?
                            r.add(0x1c9f0, 0x100); // Some kind of counter?? Does this matter?
                            r.add(0x1c830, 1); // ?
                            r.add(0x1ca30, 0x1502960e60); // ?
                            r.add(0x16c39, 0x1502960e60); // ?
                            r.add(0x1c910, 0xa0000b011d); // ?
                            r.add(0x1c8e0, 0xff); // cluster mask
                            r.add(0x1c8e8, 0); // ?
                            */
                        }
                    ),
                    tpc: inner.scene.tpc_pointer(),
                    tpc_size: U64(tile_info.tpc_size as u64),
                    microsequence: inner.micro_seq.gpu_pointer(),
                    microsequence_size: inner.micro_seq.len() as u32,
                    fragment_stamp_slot: ev_frag.slot,
                    fragment_stamp_value: ev_frag.value.next(),
                    unk_pointee: 0,
                    unk_pad: 0,
                    job_params2 <- try_init!(fw::vertex::raw::JobParameters2 {
                        unk_480: Default::default(), // fixed
                        unk_498: U64(0x0),           // fixed
                        unk_4a0: 0x0,                // fixed
                        preempt_buf1: inner.scene.preempt_buf_1_pointer(),
                        unk_4ac: 0x0,      // fixed
                        unk_4b0: U64(0x0), // fixed
                        unk_4b8: 0x0,      // fixed
                        unk_4bc: U64(0x0), // fixed
                        unk_4c4_padding: Default::default(),
                        unk_50c: 0x0,      // fixed
                        unk_510: U64(0x0), // fixed
                        unk_518: U64(0x0), // fixed
                        unk_520: U64(0x0), // fixed
                    }),
                    encoder_params <- try_init!(fw::job::raw::EncoderParams {
                        unk_8: 0x0,     // fixed
                        sync_grow: 0x0, // fixed
                        unk_10: 0x0,    // fixed
                        encoder_id: cmdbuf.encoder_id,
                        unk_18: 0x0, // fixed
                        unk_mask: unks.vtx_unk_mask as u32,
                        sampler_array: U64(cmdbuf.vertex_sampler_array),
                        sampler_count: cmdbuf.vertex_sampler_count,
                        sampler_max: cmdbuf.vertex_sampler_max,
                    }),
                    unk_55c: 0,
                    unk_560: 0,
                    sync_grow: (cmdbuf.flags
                        & uapi::ASAHI_RENDER_SYNC_TVB_GROWTH as u64
                        != 0) as u32,
                    unk_568: 0,
                    unk_56c: 0,
                    meta <- try_init!(fw::job::raw::JobMeta {
                        unk_0: 0,
                        unk_2: 0,
                        no_preemption: 0,
                        stamp: ev_vtx.stamp_pointer,
                        fw_stamp: ev_vtx.fw_stamp_pointer,
                        stamp_value: ev_vtx.value.next(),
                        stamp_slot: ev_vtx.slot,
                        evctl_index: 0, // fixed
                        flush_stamps: flush_stamps as u32,
                        uuid: uuid_ta,
                        event_seq: ev_vtx.event_seq as u32,
                    }),
                    unk_after_meta: unk1.into(),
                    unk_buf_0: U64(0),
                    unk_buf_8: U64(0),
                    unk_buf_10: U64(0),
                    cur_ts: U64(0),
                    start_ts: Some(inner_ptr!(inner.timestamps.gpu_pointer(), vtx.start)),
                    end_ts: Some(inner_ptr!(inner.timestamps.gpu_pointer(), vtx.end)),
                    unk_5c4: 0,
                    unk_5c8: 0,
                    unk_5cc: 0,
                    unk_5d0: 0,
                    client_sequence: slot_client_seq,
                    pad_5d5: Default::default(),
                    unk_5d8: 0,
                    unk_5dc: 0,
                    #[ver(V >= V13_0B4)]
                    unk_ts: U64(0),
                    #[ver(V >= V13_0B4)]
                    unk_5dd_8: Default::default(),
                })
            },
        )?;

        core::mem::drop(alloc);

        mod_dev_dbg!(self.dev, "[Submission {}] Add Vertex\n", id);
        fence.add_command();
        vtx_job.add_cb(vtx, vm_bind.slot(), move |cmd, error| {
            if let Some(err) = error {
                fence.set_error(err.into())
            }
            if let Some(mut res) = vtx_result.as_ref().map(|a| a.lock()) {
                cmd.timestamps.with(|raw, _inner| {
                    res.result.vertex_ts_start = raw.vtx.start.load(Ordering::Relaxed);
                    res.result.vertex_ts_end = raw.vtx.end.load(Ordering::Relaxed);
                });
                res.result.tvb_usage_bytes = cmd.scene.used_bytes() as u64;
                if cmd.scene.overflowed() {
                    res.result.flags |= uapi::DRM_ASAHI_RESULT_RENDER_TVB_OVERFLOWED as u64;
                }
                res.vtx_error = error;
                res.vtx_complete = true;
                res.commit();
            }
            fence.command_complete();
        })?;

        mod_dev_dbg!(self.dev, "[Submission {}] Increment counters\n", id);
        self.notifier.threshold.with(|raw, _inner| {
            raw.increment();
            raw.increment();
        });

        // TODO: handle rollbacks, move to job submit?
        buffer.increment();

        job.get_vtx()?.next_seq();
        job.get_frag()?.next_seq();

        Ok(())
    }
}
