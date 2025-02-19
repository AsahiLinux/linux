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
use crate::driver::AsahiDriver;
use crate::fw::types::*;
use crate::gpu::GpuManager;
use crate::util::*;
use crate::{buffer, file, fw, gpu, microseq};
use crate::{inner_ptr, inner_weak_ptr};
use core::sync::atomic::Ordering;
use kernel::dma_fence::RawDmaFence;
use kernel::drm::sched::Job;
use kernel::prelude::*;
use kernel::sync::Arc;
use kernel::types::ForeignOwnable;
use kernel::uapi;
use kernel::xarray;

const DEBUG_CLASS: DebugFlags = DebugFlags::Render;

/// Tiling/Vertex control bit to disable using more than one GPU cluster. This results in decreased
/// throughput but also less latency, which is probably desirable for light vertex loads where the
/// overhead of clustering/merging would exceed the time it takes to just run the job on one
/// cluster.
const TILECTL_DISABLE_CLUSTERING: u32 = 1u32 << 0;

#[versions(AGX)]
impl super::QueueInner::ver {
    /// Get the appropriate tiling parameters for a given userspace command buffer.
    fn get_tiling_params(
        cmdbuf: &uapi::drm_asahi_cmd_render,
        num_clusters: u32,
    ) -> Result<buffer::TileInfo> {
        let width: u32 = cmdbuf.width_px as u32;
        let height: u32 = cmdbuf.height_px as u32;
        let layers: u32 = cmdbuf.layers as u32;

        if layers == 0 || layers > 2048 {
            cls_pr_debug!(Errors, "Layer count invalid ({})\n", layers);
            return Err(EINVAL);
        }

        // This is overflow safe: all these calculations are done in u32.
        // At 64Kx64K max dimensions above, this is 2**32 pixels max.
        // In terms of tiles that are always larger than one pixel,
        // this can never overflow. Note that real actual dimensions
        // are limited to 16K * 16K below anyway.
        //
        // Once we multiply by the layer count, then we need to check
        // for overflow or use u64.

        let tile_width = 32u32;
        let tile_height = 32u32;

        let utile_width = cmdbuf.utile_width_px as u32;
        let utile_height = cmdbuf.utile_height_px as u32;

        match (utile_width, utile_height) {
            (32, 32) | (32, 16) | (16, 16) => (),
            _ => {
                cls_pr_debug!(
                    Errors,
                    "uTile size invalid ({} x {})\n",
                    utile_width,
                    utile_height
                );
                return Err(EINVAL);
            }
        };

        let utiles_per_tile_x = tile_width / utile_width;
        let utiles_per_tile_y = tile_height / utile_height;

        let utiles_per_tile = utiles_per_tile_x * utiles_per_tile_y;

        let tiles_x = width.div_ceil(tile_width);
        let tiles_y = height.div_ceil(tile_height);
        let tiles = tiles_x * tiles_y;

        let mtiles_x = 4u32;
        let mtiles_y = 4u32;
        let mtiles = mtiles_x * mtiles_y;

        let tiles_per_mtile_x = align(tiles_x.div_ceil(mtiles_x), 4);
        let tiles_per_mtile_y = align(tiles_y.div_ceil(mtiles_y), 4);
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
        let tilemap_size = (4 * rgn_size * mtiles) as usize * layers as usize;

        let tpc_entry_size = 8;
        // TPC stride in 32-bit words
        let tpc_mtile_stride = tpc_entry_size * utiles_per_tile * tiles_per_mtile / 4;
        let tpc_size =
            (4 * tpc_mtile_stride * mtiles) as usize * layers as usize * num_clusters as usize;

        // No idea where this comes from, but it fits what macOS does...
        // GUESS: Number of 32K heap blocks to fit a 5-byte region header/pointer per tile?
        // That would make a ton of sense...
        let meta1_layer_stride = if num_clusters > 1 {
            (align(tiles_x, 2) * align(tiles_y, 4) * utiles_per_tile).div_ceil(0x1980)
        } else {
            0
        };

        let mut min_tvb_blocks = align((tiles_x * tiles_y).div_ceil(128), 8);

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
            meta1_layer_stride,
            #[ver(G < G14X)]
            meta1_blocks: meta1_layer_stride * (cmdbuf.layers as u32),
            #[ver(G >= G14X)]
            meta1_blocks: meta1_layer_stride,
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
                helper_cfg: cmdbuf.vertex_helper.cfg,
                __pad: Default::default(),
            },
        })
    }

    /// Submit work to a render queue.
    pub(super) fn submit_render(
        &self,
        job: &mut Job<super::QueueJob::ver>,
        cmdbuf: &uapi::drm_asahi_cmd_render,
        vertex_attachments: &microseq::Attachments,
        fragment_attachments: &microseq::Attachments,
        objects: Pin<&xarray::XArray<KBox<file::Object>>>,
        id: u64,
        flush_stamps: bool,
    ) -> Result {
        mod_dev_dbg!(self.dev, "[Submission {}] Render!\n", id);

        if cmdbuf.flags
            & !(uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_VERTEX_SCRATCH
                | uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES
                | uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_NO_VERTEX_CLUSTERING
                | uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_DBIAS_IS_INT) as u32
            != 0
        {
            cls_pr_debug!(Errors, "Invalid flags ({:#x})\n", cmdbuf.flags);
            return Err(EINVAL);
        }

        if cmdbuf.width_px == 0
            || cmdbuf.height_px == 0
            || cmdbuf.width_px > 16384
            || cmdbuf.height_px > 16384
        {
            cls_pr_debug!(
                Errors,
                "Invalid dimensions ({}x{})\n",
                cmdbuf.width_px,
                cmdbuf.height_px
            );
            return Err(EINVAL);
        }

        let mut vtx_user_timestamps: fw::job::UserTimestamps = Default::default();
        let mut frg_user_timestamps: fw::job::UserTimestamps = Default::default();

        vtx_user_timestamps.start = common::get_timestamp_object(objects, cmdbuf.ts_vtx.start)?;
        vtx_user_timestamps.end = common::get_timestamp_object(objects, cmdbuf.ts_vtx.end)?;
        frg_user_timestamps.start = common::get_timestamp_object(objects, cmdbuf.ts_frag.start)?;
        frg_user_timestamps.end = common::get_timestamp_object(objects, cmdbuf.ts_frag.end)?;

        let data = unsafe { &<KBox<AsahiDriver>>::borrow(self.dev.as_ref().get_drvdata()).data };
        let gpu = match data.gpu.as_any().downcast_ref::<gpu::GpuManager::ver>() {
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
            || cmdbuf.flags
                & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_NO_VERTEX_CLUSTERING as u32
                != 0
        {
            clustering = false;
        }

        #[ver(G != G14)]
        let tiling_control = {
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

        let buffer = &self.buffer;
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
                cmdbuf.width_px,
                cmdbuf.height_px
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

        let uuid_3d = 0;
        let uuid_ta = 0;

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
                    external_barrier: 0,
                    internal_barrier_type: 0,
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

        let unk1 = false;

        let mut tile_config: u64 = 0;
        if !unk1 {
            tile_config |= 0x280;
        }
        if cmdbuf.layers > 1 {
            tile_config |= 1;
        }
        if cmdbuf.flags & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES as u32
            != 0
        {
            tile_config |= 0x10000;
        }

        let samples_log2 = match cmdbuf.samples {
            1 => 0,
            2 => 1,
            4 => 2,
            _ => {
                cls_pr_debug!(Errors, "Invalid sample count {}\n", cmdbuf.samples);
                return Err(EINVAL);
            }
        };

        let utile_config = ((tile_info.utile_width / 16) << 12)
            | ((tile_info.utile_height / 16) << 14)
            | samples_log2;

        // Calculate the number of 2KiB blocks to allocate per utile. This is
        // just a bit of dimensional analysis.
        let pixels_per_utile: u32 =
            (cmdbuf.utile_width_px as u32) * (cmdbuf.utile_height_px as u32);
        let samples_per_utile: u32 = pixels_per_utile << samples_log2;
        let utile_size_bytes: u32 = (cmdbuf.sample_size_B as u32) * samples_per_utile;
        let block_size_bytes: u32 = 2048;
        let blocks_per_utile: u32 = utile_size_bytes.div_ceil(block_size_bytes);

        #[ver(G >= G14X)]
        let frg_tilecfg = 0x0000000_00036011
            | (((tile_info.tiles_x - 1) as u64) << 44)
            | (((tile_info.tiles_y - 1) as u64) << 53)
            | (if unk1 { 0 } else { 0x20_00000000 })
            | (if cmdbuf.layers > 1 { 0x1_00000000 } else { 0 })
            | ((utile_config as u64 & 0xf000) << 28);

        // TODO: check
        #[ver(V >= V13_0B4)]
        let count_frag = self.counter.fetch_add(2, Ordering::Relaxed);
        #[ver(V >= V13_0B4)]
        let count_vtx = count_frag + 1;

        // Unknowns handling

        #[ver(G >= G14)]
        let g14_unk = 0x4040404;
        #[ver(G < G14)]
        let g14_unk = 0;
        #[ver(G < G14X)]
        let frg_unk_140 = 0x8c60;
        let frg_unk_158 = 0x1c;
        #[ver(G >= G14)]
        let load_bgobjvals = cmdbuf.isp_bgobjvals as u64;
        #[ver(G < G14)]
        let load_bgobjvals = cmdbuf.isp_bgobjvals as u64 | 0x400;
        let reload_zlsctrl = cmdbuf.zls_ctrl;
        let iogpu_unk54 = 0x3a0012006b0003;
        let iogpu_unk56 = 1;
        #[ver(G < G14)]
        let tiling_control_2 = 0;
        #[ver(G >= G14X)]
        let tiling_control_2 = 4;
        #[ver(G >= G14X)]
        let vtx_unk_f0 = 0x1c;
        #[ver(G < G14)]
        let vtx_unk_f0 = 0x1c + (align(tile_info.meta1_blocks, 4) as u64);
        let vtx_unk_118 = 0x1c;

        // DRM_ASAHI_RENDER_DBIAS_IS_INT chosen to match hardware bit.
        let isp_ctl = 0xc000u32
            | (cmdbuf.flags & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_DBIAS_IS_INT as u32);

        // Always allow preemption at the UAPI level
        let no_preemption = false;

        mod_dev_dbg!(self.dev, "[Submission {}] Create Frag\n", id);
        let frag = GpuObject::new_init_prealloc(
            kalloc.gpu_ro.alloc_object()?,
            |ptr: GpuWeakPointer<fw::fragment::RunFragment::ver>| {
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
                            sync_grow: 0,
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
                            attachments: *fragment_attachments,
                            padding: 0,
                            #[ver(V >= V13_0B4)]
                            counter: U64(count_frag),
                            #[ver(V >= V13_0B4)]
                            notifier_buf: inner_weak_ptr!(notifier.weak_pointer(), state.unk_buf),
                        })?;

                        if frg_user_timestamps.any() {
                            builder.add(microseq::Timestamp::ver {
                                header: microseq::op::Timestamp::new(true),
                                command_time: inner_weak_ptr!(ptr, command_time),
                                ts_pointers: inner_weak_ptr!(ptr, timestamp_pointers),
                                update_ts: inner_weak_ptr!(ptr, timestamp_pointers.start_addr),
                                work_queue: ev_frag.info_ptr,
                                user_ts_pointers: inner_weak_ptr!(ptr, user_timestamp_pointers),
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

                        if frg_user_timestamps.any() {
                            builder.add(microseq::Timestamp::ver {
                                header: microseq::op::Timestamp::new(false),
                                command_time: inner_weak_ptr!(ptr, command_time),
                                ts_pointers: inner_weak_ptr!(ptr, timestamp_pointers),
                                update_ts: inner_weak_ptr!(ptr, timestamp_pointers.end_addr),
                                work_queue: ev_frag.info_ptr,
                                user_ts_pointers: inner_weak_ptr!(ptr, user_timestamp_pointers),
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
                            has_attachments: (fragment_attachments.count > 0) as u32,
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
                    user_timestamps: frg_user_timestamps,
                })
            },
            |inner, _ptr| {
                let vm_slot = vm_bind.slot();
                let aux_fb_info = fw::fragment::raw::AuxFBInfo::ver {
                    isp_ctl: isp_ctl,
                    unk2: 0,
                    width: cmdbuf.width_px as u32,
                    height: cmdbuf.height_px as u32,
                    #[ver(V >= V13_0B4)]
                    unk3: U64(0x100000),
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
                    samples: cmdbuf.samples as u32,
                    tiles_per_mtile_y: tile_info.tiles_per_mtile_y as u16,
                    tiles_per_mtile_x: tile_info.tiles_per_mtile_x as u16,
                    unk_50: U64(0),
                    unk_58: U64(0),
                    isp_merge_upper_x: F32::from_bits(cmdbuf.isp_merge_upper_x),
                    isp_merge_upper_y: F32::from_bits(cmdbuf.isp_merge_upper_y),
                    unk_68: U64(0),
                    tile_count: U64(tile_info.tiles as u64),
                    #[ver(G < G14X)]
                    job_params1 <- try_init!(fw::fragment::raw::JobParameters1::ver {
                        utile_config,
                        unk_4: 0,
                        bg: fw::fragment::raw::BackgroundProgram {
                            rsrc_spec: U64(cmdbuf.bg.rsrc_spec as u64),
                            address: U64(cmdbuf.bg.usc as u64),
                        },
                        ppp_multisamplectl: U64(cmdbuf.ppp_multisamplectl),
                        isp_scissor_base: U64(cmdbuf.isp_scissor_base),
                        isp_dbias_base: U64(cmdbuf.isp_dbias_base),
                        isp_oclqry_base: U64(cmdbuf.isp_oclqry_base),
                        aux_fb_info,
                        isp_zls_pixels: U64(cmdbuf.isp_zls_pixels as u64),
                        zls_ctrl: U64(cmdbuf.zls_ctrl),
                        #[ver(G >= G14)]
                        unk_58_g14_0: U64(g14_unk),
                        #[ver(G >= G14)]
                        unk_58_g14_8: U64(0),
                        z_load: U64(cmdbuf.depth.base),
                        z_store: U64(cmdbuf.depth.base),
                        s_load: U64(cmdbuf.stencil.base),
                        s_store: U64(cmdbuf.stencil.base),
                        #[ver(G >= G14)]
                        unk_68_g14_0: Default::default(),
                        z_load_stride: U64(cmdbuf.depth.stride as u64),
                        z_store_stride: U64(cmdbuf.depth.stride as u64),
                        s_load_stride: U64(cmdbuf.stencil.stride as u64),
                        s_store_stride: U64(cmdbuf.stencil.stride as u64),
                        z_load_comp: U64(cmdbuf.depth.comp_base),
                        z_load_comp_stride: U64(cmdbuf.depth.comp_stride as u64),
                        z_store_comp: U64(cmdbuf.depth.comp_base),
                        z_store_comp_stride: U64(cmdbuf.depth.comp_stride as u64),
                        s_load_comp: U64(cmdbuf.stencil.comp_base),
                        s_load_comp_stride: U64(cmdbuf.stencil.comp_stride as u64),
                        s_store_comp: U64(cmdbuf.stencil.comp_base),
                        s_store_comp_stride: U64(cmdbuf.stencil.comp_stride as u64),
                        tvb_tilemap: inner.scene.tvb_tilemap_pointer(),
                        tvb_layermeta: inner.scene.tvb_layermeta_pointer(),
                        mtile_stride_dwords: U64((4 * tile_info.params.rgn_size as u64) << 24),
                        tvb_heapmeta: inner.scene.tvb_heapmeta_pointer(),
                        tile_config: U64(tile_config),
                        aux_fb: inner.aux_fb.gpu_pointer(),
                        unk_108: Default::default(),
                        usc_exec_base_isp: U64(self.usc_exec_base),
                        unk_140: U64(frg_unk_140),
                        helper_program: cmdbuf.fragment_helper.binary,
                        unk_14c: 0,
                        helper_arg: U64(cmdbuf.fragment_helper.data),
                        unk_158: U64(frg_unk_158),
                        unk_160: U64(0),
                        __pad: Default::default(),
                        #[ver(V < V13_0B4)]
                        __pad1: Default::default(),
                    }),
                    #[ver(G < G14X)]
                    job_params2 <- try_init!(fw::fragment::raw::JobParameters2 {
                        eot_rsrc_spec: cmdbuf.eot.rsrc_spec,
                        eot_usc: cmdbuf.eot.usc,
                        unk_8: 0x0,
                        unk_c: 0x0,
                        isp_merge_upper_x: F32::from_bits(cmdbuf.isp_merge_upper_x),
                        isp_merge_upper_y: F32::from_bits(cmdbuf.isp_merge_upper_y),
                        unk_18: U64(0x0),
                        utiles_per_mtile_y: tile_info.utiles_per_mtile_y as u16,
                        utiles_per_mtile_x: tile_info.utiles_per_mtile_x as u16,
                        unk_24: 0x0,
                        tile_counts: ((tile_info.tiles_y - 1) << 12) | (tile_info.tiles_x - 1),
                        tib_blocks: blocks_per_utile,
                        isp_bgobjdepth: cmdbuf.isp_bgobjdepth,
                        // TODO: does this flag need to be exposed to userspace?
                        isp_bgobjvals: load_bgobjvals as u32,
                        unk_38: 0x0,
                        unk_3c: 0x1,
                        helper_cfg: cmdbuf.fragment_helper.cfg,
                        __pad: Default::default(),
                    }),
                    #[ver(G >= G14X)]
                    registers: fw::job::raw::RegisterArray::new(
                        inner_weak_ptr!(_ptr, registers.registers),
                        |r| {
                            r.add(0x1739, 1);
                            r.add(0x10009, utile_config.into());
                            r.add(0x15379, cmdbuf.eot.rsrc_spec.into());
                            r.add(0x15381, cmdbuf.eot.usc.into());
                            r.add(0x15369, cmdbuf.bg.rsrc_spec.into());
                            r.add(0x15371, cmdbuf.bg.usc.into());
                            r.add(0x15131, cmdbuf.isp_merge_upper_x.into());
                            r.add(0x15139, cmdbuf.isp_merge_upper_y.into());
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
                            r.add(0x15109, cmdbuf.isp_scissor_base); // ISP_SCISSOR_BASE
                            r.add(0x15101, cmdbuf.isp_dbias_base); // ISP_DBIAS_BASE
                            r.add(0x15021, isp_ctl.into()); // aux_fb_info.unk_1
                            r.add(
                                0x15211,
                                ((cmdbuf.height_px as u64) << 32) | cmdbuf.width_px as u64,
                            ); // aux_fb_info.{width, heigh
                            r.add(0x15049, 0x100000); // s2.aux_fb_info.unk3
                            r.add(0x10051, blocks_per_utile.into()); // s1.unk_2c
                            r.add(0x15321, cmdbuf.isp_zls_pixels.into()); // ISP_ZLS_PIXELS
                            r.add(0x15301, cmdbuf.isp_bgobjdepth.into()); // ISP_BGOBJDEPTH
                            r.add(0x15309, load_bgobjvals); // ISP_BGOBJVALS
                            r.add(0x15311, cmdbuf.isp_oclqry_base); // ISP_OCLQRY_BASE
                            r.add(0x15319, cmdbuf.zls_ctrl); // ISP_ZLSCTL
                            r.add(0x15349, g14_unk); // s2.unk_58_g14_0
                            r.add(0x15351, 0); // s2.unk_58_g14_8
                            r.add(0x15329, cmdbuf.depth.base); // ISP_ZLOAD_BASE
                            r.add(0x15331, cmdbuf.depth.base); // ISP_ZSTORE_BASE
                            r.add(0x15339, cmdbuf.stencil.base); // ISP_STENCIL_LOAD_BASE
                            r.add(0x15341, cmdbuf.stencil.base); // ISP_STENCIL_STORE_BASE
                            r.add(0x15231, 0);
                            r.add(0x15221, 0);
                            r.add(0x15239, 0);
                            r.add(0x15229, 0);
                            r.add(0x15401, cmdbuf.depth.stride as u64); // load
                            r.add(0x15421, cmdbuf.depth.stride as u64); // store
                            r.add(0x15409, cmdbuf.stencil.stride as u64); // load
                            r.add(0x15429, cmdbuf.stencil.stride as u64);
                            r.add(0x153c1, cmdbuf.depth.comp_base); // load
                            r.add(0x15411, cmdbuf.depth.comp_stride as u64); // load
                            r.add(0x153c9, cmdbuf.depth.comp_base); // store
                            r.add(0x15431, cmdbuf.depth.comp_stride as u64); // store
                            r.add(0x153d1, cmdbuf.stencil.comp_base); // load
                            r.add(0x15419, cmdbuf.stencil.comp_stride as u64); // load
                            r.add(0x153d9, cmdbuf.stencil.comp_base); // store
                            r.add(0x15439, cmdbuf.stencil.comp_stride as u64); // store
                            r.add(0x16429, inner.scene.tvb_tilemap_pointer().into());
                            r.add(0x16060, inner.scene.tvb_layermeta_pointer().into());
                            r.add(0x16431, (4 * tile_info.params.rgn_size as u64) << 24); // ISP_RGN?
                            r.add(0x10039, tile_config); // tile_config ISP_CTL?
                            r.add(0x16451, 0x0); // ISP_RENDER_ORIGIN
                            r.add(0x11821, cmdbuf.fragment_helper.binary.into());
                            r.add(0x11829, cmdbuf.fragment_helper.data);
                            r.add(0x11f79, cmdbuf.fragment_helper.cfg.into());
                            r.add(0x15359, 0);
                            r.add(0x10069, self.usc_exec_base); // frag; USC_EXEC_BASE_ISP
                            r.add(0x16020, 0);
                            r.add(0x16461, inner.aux_fb.gpu_pointer().into());
                            r.add(0x16090, inner.aux_fb.gpu_pointer().into());
                            r.add(0x120a1, frg_unk_158);
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
                        isp_dbias_base: fw::fragment::raw::ArrayAddr {
                            ptr: U64(cmdbuf.isp_dbias_base),
                            unk_padding: U64(0),
                        },
                        isp_scissor_base: fw::fragment::raw::ArrayAddr {
                            ptr: U64(cmdbuf.isp_scissor_base),
                            unk_padding: U64(0),
                        },
                        isp_oclqry_base: U64(cmdbuf.isp_oclqry_base),
                        unk_118: U64(0x0),
                        unk_120: Default::default(),
                        unk_partial_bg: fw::fragment::raw::BackgroundProgram {
                            rsrc_spec: U64(cmdbuf.partial_bg.rsrc_spec as u64),
                            address: U64(cmdbuf.partial_bg.usc as u64),
                        },
                        unk_258: U64(0),
                        unk_260: U64(0),
                        unk_268: U64(0),
                        unk_270: U64(0),
                        partial_bg: fw::fragment::raw::BackgroundProgram {
                            rsrc_spec: U64(cmdbuf.partial_bg.rsrc_spec as u64),
                            address: U64(cmdbuf.partial_bg.usc as u64),
                        },
                        zls_ctrl: U64(reload_zlsctrl),
                        unk_290: U64(g14_unk),
                        z_load: U64(cmdbuf.depth.base),
                        z_partial_stride: U64(cmdbuf.depth.stride as u64),
                        z_partial_comp_stride: U64(cmdbuf.depth.comp_stride as u64),
                        z_store: U64(cmdbuf.depth.base),
                        z_partial: U64(cmdbuf.depth.base),
                        z_partial_comp: U64(cmdbuf.depth.comp_base),
                        s_load: U64(cmdbuf.stencil.base),
                        s_partial_stride: U64(cmdbuf.stencil.stride as u64),
                        s_partial_comp_stride: U64(cmdbuf.stencil.comp_stride as u64),
                        s_store: U64(cmdbuf.stencil.base),
                        s_partial: U64(cmdbuf.stencil.base),
                        s_partial_comp: U64(cmdbuf.stencil.comp_base),
                        unk_2f8: Default::default(),
                        tib_blocks: blocks_per_utile,
                        unk_30c: 0x0,
                        aux_fb_info,
                        tile_config: U64(tile_config),
                        unk_328_padding: Default::default(),
                        unk_partial_eot: fw::fragment::raw::EotProgram::new(
                            cmdbuf.partial_eot.rsrc_spec,
                            cmdbuf.partial_eot.usc
                        ),
                        partial_eot: fw::fragment::raw::EotProgram::new(
                            cmdbuf.partial_eot.rsrc_spec,
                            cmdbuf.partial_eot.usc
                        ),
                        isp_bgobjdepth: cmdbuf.isp_bgobjdepth,
                        isp_bgobjvals: cmdbuf.isp_bgobjvals,
                        sample_size: cmdbuf.sample_size_B as u32,
                        unk_37c: 0x0,
                        unk_380: U64(0x0),
                        unk_388: U64(0x0),
                        #[ver(V >= V13_0B4)]
                        unk_390_0: U64(0x0),
                        isp_zls_pixels: U64(cmdbuf.isp_zls_pixels as u64),
                    }),
                    unk_758_flag: 0,
                    unk_75c_flag: 0,
                    unk_buf: Default::default(),
                    busy_flag: 0,
                    tvb_overflow_count: 0,
                    unk_878: 0,
                    encoder_params <- try_init!(fw::job::raw::EncoderParams {
                        // Maybe set when reloading z/s?
                        unk_8: 0,
                        sync_grow: 0,
                        unk_10: 0x0, // fixed
                        encoder_id: 0,
                        unk_18: 0x0, // fixed
                        unk_mask: 0xffffffffu32,
                        sampler_array: U64(cmdbuf.sampler_heap),
                        sampler_count: cmdbuf.sampler_count as u32,
                        sampler_max: (cmdbuf.sampler_count as u32) + 1,
                    }),
                    process_empty_tiles: (cmdbuf.flags
                        & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES as u32
                        != 0) as u32,
                    // TODO: needs to be investigated
                    no_clear_pipeline_textures: 1,
                    // TODO: needs to be investigated
                    msaa_zs: 0,
                    unk_pointee: 0,
                    #[ver(V >= V13_3)]
                    unk_v13_3: 0,
                    meta <- try_init!(fw::job::raw::JobMeta {
                        unk_0: 0,
                        unk_2: 0,
                        no_preemption: no_preemption as u8,
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
                    command_time: U64(0),
                    timestamp_pointers <- try_init!(fw::job::raw::TimestampPointers {
                        start_addr: Some(inner_ptr!(inner.timestamps.gpu_pointer(), frag.start)),
                        end_addr: Some(inner_ptr!(inner.timestamps.gpu_pointer(), frag.end)),
                    }),
                    user_timestamp_pointers: inner.user_timestamps.pointers()?,
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

        frag_job.add_cb(frag, vm_bind.slot(), move |error| {
            if let Some(err) = error {
                fence.set_error(err.into());
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
                            attachments: *vertex_attachments,
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

                        if vtx_user_timestamps.any() {
                            builder.add(microseq::Timestamp::ver {
                                header: microseq::op::Timestamp::new(true),
                                command_time: inner_weak_ptr!(ptr, command_time),
                                ts_pointers: inner_weak_ptr!(ptr, timestamp_pointers),
                                update_ts: inner_weak_ptr!(ptr, timestamp_pointers.start_addr),
                                work_queue: ev_vtx.info_ptr,
                                user_ts_pointers: inner_weak_ptr!(ptr, user_timestamp_pointers),
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

                        if vtx_user_timestamps.any() {
                            builder.add(microseq::Timestamp::ver {
                                header: microseq::op::Timestamp::new(false),
                                command_time: inner_weak_ptr!(ptr, command_time),
                                ts_pointers: inner_weak_ptr!(ptr, timestamp_pointers),
                                update_ts: inner_weak_ptr!(ptr, timestamp_pointers.end_addr),
                                work_queue: ev_vtx.info_ptr,
                                user_ts_pointers: inner_weak_ptr!(ptr, user_timestamp_pointers),
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
                            has_attachments: (vertex_attachments.count > 0) as u32,
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
                    user_timestamps: vtx_user_timestamps,
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
                        iogpu_unk_54: U64(iogpu_unk54), // fixed
                        iogpu_unk_56: U64(iogpu_unk56), // fixed
                        #[ver(G < G14)]
                        tvb_cluster_meta1: inner
                            .scene
                            .meta_1_pointer()
                            .map(|x| x.or((tile_info.meta1_layer_stride as u64) << 50)),
                        utile_config,
                        unk_4c: 0,
                        ppp_multisamplectl: U64(cmdbuf.ppp_multisamplectl), // fixed
                        tvb_layermeta: inner.scene.tvb_layermeta_pointer(),
                        #[ver(G < G14)]
                        tvb_cluster_layermeta: inner.scene.tvb_cluster_layermeta_pointer(),
                        #[ver(G < G14)]
                        core_mask: Array::new([
                            *core_masks.first().unwrap_or(&0),
                            *core_masks.get(1).unwrap_or(&0),
                        ]),
                        preempt_buf1: inner.scene.preempt_buf_1_pointer(),
                        preempt_buf2: inner.scene.preempt_buf_2_pointer(),
                        unk_80: U64(0x1), // fixed
                        preempt_buf3: inner.scene.preempt_buf_3_pointer().or(0x4_0000_0000_0000), // check
                        vdm_ctrl_stream_base: U64(cmdbuf.vdm_ctrl_stream_base),
                        #[ver(G < G14)]
                        tvb_cluster_meta2: inner.scene.meta_2_pointer(),
                        #[ver(G < G14)]
                        tvb_cluster_meta3: inner.scene.meta_3_pointer(),
                        #[ver(G < G14)]
                        tiling_control,
                        #[ver(G < G14)]
                        unk_ac: tiling_control_2 as u32, // fixed
                        unk_b0: Default::default(), // fixed
                        usc_exec_base_ta: U64(self.usc_exec_base),
                        #[ver(G < G14)]
                        tvb_cluster_meta4: inner
                            .scene
                            .meta_4_pointer()
                            .map(|x| x.or(0x3000_0000_0000_0000)),
                        #[ver(G < G14)]
                        unk_f0: U64(vtx_unk_f0),
                        unk_f8: U64(0x8c60),     // fixed
                        helper_program: cmdbuf.vertex_helper.binary,
                        unk_104: 0,
                        helper_arg: U64(cmdbuf.vertex_helper.data),
                        unk_110: Default::default(),      // fixed
                        unk_118: vtx_unk_118 as u32, // fixed
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
                            r.add(0x1c051, iogpu_unk54); // iogpu_unk_54/55
                            r.add(0x1c061, iogpu_unk56); // iogpu_unk_56
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
                            r.add(0x1c880, cmdbuf.vdm_ctrl_stream_base); // VDM_CTRL_STREAM_BASE
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
                            r.add(0x1c918, tiling_control_2);
                            r.add(0x1c079, inner.scene.tvb_layermeta_pointer().into());
                            r.add(0x1c9d8, inner.scene.tvb_layermeta_pointer().into());
                            let cl_layermeta_pointer =
                                inner.scene.tvb_cluster_layermeta_pointer().map_or(0, |a| a.into());
                            r.add(0x1c089, cl_layermeta_pointer);
                            r.add(0x1c9e0, cl_layermeta_pointer);
                            let cl_meta_4_pointer =
                                inner.scene.meta_4_pointer().map_or(0, |a| a.into());
                            r.add(0x16c41, cl_meta_4_pointer); // tvb_cluster_meta4
                            r.add(0x1ca40, cl_meta_4_pointer); // tvb_cluster_meta4
                            r.add(0x1c9a8, vtx_unk_f0); // + meta1_blocks? min_free_tvb_pages?
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
                            r.add(0x10061, self.usc_exec_base); // USC_EXEC_BASE_TA
                            r.add(0x11801, cmdbuf.vertex_helper.binary.into());
                            r.add(0x11809, cmdbuf.vertex_helper.data);
                            r.add(0x11f71, cmdbuf.vertex_helper.cfg.into());
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
                            r.add(0x12099, vtx_unk_118);
                            r.add(0x1c9e8, (tile_info.params.unk_28 & 0x4fff).into());
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
                        encoder_id: 0,
                        unk_18: 0x0, // fixed
                        unk_mask: 0xffffffffu32,
                        sampler_array: U64(cmdbuf.sampler_heap),
                        sampler_count: cmdbuf.sampler_count as u32,
                        sampler_max: (cmdbuf.sampler_count as u32) + 1,
                    }),
                    unk_55c: 0,
                    unk_560: 0,
                    sync_grow: 0,
                    unk_568: 0,
                    uses_scratch: (cmdbuf.flags
                        & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_VERTEX_SCRATCH as u32
                        != 0) as u32,
                    meta <- try_init!(fw::job::raw::JobMeta {
                        unk_0: 0,
                        unk_2: 0,
                        no_preemption: no_preemption as u8,
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
                    command_time: U64(0),
                    timestamp_pointers <- try_init!(fw::job::raw::TimestampPointers {
                        start_addr: Some(inner_ptr!(inner.timestamps.gpu_pointer(), vtx.start)),
                        end_addr: Some(inner_ptr!(inner.timestamps.gpu_pointer(), vtx.end)),
                    }),
                    user_timestamp_pointers: inner.user_timestamps.pointers()?,
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
        vtx_job.add_cb(vtx, vm_bind.slot(), move |error| {
            if let Some(err) = error {
                fence.set_error(err.into())
            }

            fence.command_complete();
        })?;

        mod_dev_dbg!(self.dev, "[Submission {}] Increment counters\n", id);

        // TODO: handle rollbacks, move to job submit?
        buffer.increment();

        job.get_vtx()?.next_seq();
        job.get_frag()?.next_seq();

        Ok(())
    }
}
