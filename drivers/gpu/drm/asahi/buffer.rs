// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Tiled Vertex Buffer management
//!
//! This module manages the Tiled Vertex Buffer, also known as the Parameter Buffer (in imgtec
//! parlance) or the tiler heap (on other architectures). This buffer holds transformed primitive
//! data between the vertex/tiling stage and the fragment stage.
//!
//! On AGX, the buffer is a heap of 128K blocks split into 32K pages (which must be aligned to a
//! multiple of 32K in VA space). The buffer can be shared between multiple render jobs, and each
//! will allocate pages from it during vertex processing and return them during fragment processing.
//!
//! If the buffer runs out of free pages, the vertex pass stops and a partial fragment pass occurs,
//! spilling the intermediate render target state to RAM (a partial render). This is all managed
//! transparently by the firmware. Since partial renders are less efficient, the kernel must grow
//! the heap in response to feedback from the firmware to avoid partial renders in the future.
//! Currently, we only ever grow the heap, and never shrink it.
//!
//! AGX also supports memoryless render targets, which can be used for intermediate results within
//! a render pass. To support partial renders, it seems the GPU/firmware has the ability to borrow
//! pages from the TVB buffer as a temporary render target buffer. Since this happens during a
//! partial render itself, if the buffer runs out of space, it requires synchronous growth in
//! response to a firmware interrupt. This is not currently supported, but may be in the future,
//! though it is unclear whether it is worth the effort.
//!
//! This module is also in charge of managing the temporary objects associated with a single render
//! pass, which includes the top-level tile array, the tail pointer cache, preemption buffers, and
//! other miscellaneous structures collectively managed as a "scene".
//!
//! To avoid runaway memory usage, there is a maximum size for buffers (at that point it's unlikely
//! that partial renders will incur much overhead over the buffer data access itself). This is
//! different depending on whether memoryless render targets are in use, and is currently hardcoded.
//! to the most common value used by macOS.

use crate::debug::*;
use crate::fw::buffer;
use crate::fw::types::*;
use crate::util::*;
use crate::{alloc, fw, gpu, hw, mmu, slotalloc};
use core::sync::atomic::Ordering;
use kernel::prelude::*;
use kernel::sync::{Arc, Mutex};
use kernel::{c_str, static_lock_class};

const DEBUG_CLASS: DebugFlags = DebugFlags::Buffer;

/// There are 127 GPU/firmware-side buffer manager slots (yes, 127, not 128).
const NUM_BUFFERS: u32 = 127;

/// Page size bits for buffer pages (32K). VAs must be aligned to this size.
pub(crate) const PAGE_SHIFT: usize = 15;
/// Page size for buffer pages.
pub(crate) const PAGE_SIZE: usize = 1 << PAGE_SHIFT;
/// Number of pages in a buffer block, which should be contiguous in VA space.
pub(crate) const PAGES_PER_BLOCK: usize = 4;
/// Size of a buffer block.
pub(crate) const BLOCK_SIZE: usize = PAGE_SIZE * PAGES_PER_BLOCK;

/// Metadata about the tiling configuration for a scene. This is computed in the `render` module.
/// based on dimensions, tile size, and other info.
pub(crate) struct TileInfo {
    /// Tile count in the X dimension. Tiles are always 32x32.
    pub(crate) tiles_x: u32,
    /// Tile count in the Y dimension. Tiles are always 32x32.
    pub(crate) tiles_y: u32,
    /// Total tile count.
    pub(crate) tiles: u32,
    /// Micro-tile width (16 or 32).
    pub(crate) utile_width: u32,
    /// Micro-tile height (16 or 32).
    pub(crate) utile_height: u32,
    // Macro-tiles in the X dimension. Always 4.
    //pub(crate) mtiles_x: u32,
    // Macro-tiles in the Y dimension. Always 4.
    //pub(crate) mtiles_y: u32,
    /// Tiles per macro-tile in the X dimension.
    pub(crate) tiles_per_mtile_x: u32,
    /// Tiles per macro-tile in the Y dimension.
    pub(crate) tiles_per_mtile_y: u32,
    // Total tiles per macro-tile.
    //pub(crate) tiles_per_mtile: u32,
    /// Micro-tiles per macro-tile in the X dimension.
    pub(crate) utiles_per_mtile_x: u32,
    /// Micro-tiles per macro-tile in the Y dimension.
    pub(crate) utiles_per_mtile_y: u32,
    // Total micro-tiles per macro-tile.
    //pub(crate) utiles_per_mtile: u32,
    /// Size of the top-level tilemap, in bytes (for all layers, one cluster).
    pub(crate) tilemap_size: usize,
    /// Size of the Tail Pointer Cache, in bytes (for all layers * clusters).
    pub(crate) tpc_size: usize,
    /// Number of blocks in the clustering meta buffer (for clustering).
    pub(crate) meta1_blocks: u32,
    /// Minimum number of TVB blocks for this render.
    pub(crate) min_tvb_blocks: usize,
    /// Tiling parameter structure passed to firmware.
    pub(crate) params: fw::vertex::raw::TilingParameters,
}

/// A single scene, representing a render pass and its required buffers.
#[versions(AGX)]
#[derive(Debug)]
pub(crate) struct Scene {
    object: GpuObject<buffer::Scene::ver>,
    slot: u32,
    rebind: bool,
    preempt2_off: usize,
    preempt3_off: usize,
    // Note: these are dead code only on some version variants.
    // It's easier to do this than to propagate the version conditionals everywhere.
    #[allow(dead_code)]
    meta2_off: usize,
    #[allow(dead_code)]
    meta3_off: usize,
    #[allow(dead_code)]
    meta4_off: usize,
}

#[versions(AGX)]
impl Scene::ver {
    /// Returns true if the buffer was bound to a fresh manager slot, and therefore needs an init
    /// command before a render.
    pub(crate) fn rebind(&self) -> bool {
        self.rebind
    }

    /// Returns the buffer manager slot this scene's buffer was bound to.
    pub(crate) fn slot(&self) -> u32 {
        self.slot
    }

    /// Returns the GPU pointer to the [`buffer::Scene::ver`].
    pub(crate) fn gpu_pointer(&self) -> GpuPointer<'_, buffer::Scene::ver> {
        self.object.gpu_pointer()
    }

    /// Returns the GPU weak pointer to the [`buffer::Scene::ver`].
    pub(crate) fn weak_pointer(&self) -> GpuWeakPointer<buffer::Scene::ver> {
        self.object.weak_pointer()
    }

    /// Returns the GPU weak pointer to the kernel-side temp buffer.
    /// (purpose unknown...)
    pub(crate) fn kernel_buffer_pointer(&self) -> GpuWeakPointer<[u8]> {
        self.object.buffer.inner.lock().kernel_buffer.weak_pointer()
    }

    /// Returns the GPU pointer to the `buffer::Info::ver` object associated with this Scene.
    pub(crate) fn buffer_pointer(&self) -> GpuPointer<'_, buffer::Info::ver> {
        // We can't return the strong pointer directly since its lifetime crosses a lock, but we know
        // its lifetime will be valid as long as &self since we hold a reference to the buffer,
        // so just construct the strong pointer with the right lifetime here.
        unsafe { self.weak_buffer_pointer().upgrade() }
    }

    /// Returns the GPU weak pointer to the `buffer::Info::ver` object associated with this Scene.
    pub(crate) fn weak_buffer_pointer(&self) -> GpuWeakPointer<buffer::Info::ver> {
        self.object.buffer.inner.lock().info.weak_pointer()
    }

    /// Returns the GPU pointer to the TVB heap metadata buffer.
    pub(crate) fn tvb_heapmeta_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object.tvb_heapmeta.gpu_pointer()
    }

    /// Returns the GPU pointer to the top-level TVB tilemap buffer.
    pub(crate) fn tvb_tilemap_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object.tvb_tilemap.gpu_pointer()
    }

    /// Returns the GPU pointer to the Tail Pointer Cache buffer.
    pub(crate) fn tpc_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object.tpc.gpu_pointer()
    }

    /// Returns the GPU pointer to the first preemption scratch buffer.
    pub(crate) fn preempt_buf_1_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object.preempt_buf.gpu_pointer()
    }

    /// Returns the GPU pointer to the second preemption scratch buffer.
    pub(crate) fn preempt_buf_2_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object
            .preempt_buf
            .gpu_offset_pointer(self.preempt2_off)
    }

    /// Returns the GPU pointer to the third preemption scratch buffer.
    pub(crate) fn preempt_buf_3_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object
            .preempt_buf
            .gpu_offset_pointer(self.preempt3_off)
    }

    /// Returns the GPU pointer to the per-cluster tilemap buffer, if clustering is enabled.
    #[allow(dead_code)]
    pub(crate) fn cluster_tilemaps_pointer(&self) -> Option<GpuPointer<'_, &'_ [u8]>> {
        self.object
            .clustering
            .as_ref()
            .map(|c| c.tilemaps.gpu_pointer())
    }

    /// Returns the GPU pointer to the clustering metadata 1 buffer, if clustering is enabled.
    #[allow(dead_code)]
    pub(crate) fn meta_1_pointer(&self) -> Option<GpuPointer<'_, &'_ [u8]>> {
        self.object
            .clustering
            .as_ref()
            .map(|c| c.meta.gpu_pointer())
    }

    /// Returns the GPU pointer to the clustering metadata 2 buffer, if clustering is enabled.
    #[allow(dead_code)]
    pub(crate) fn meta_2_pointer(&self) -> Option<GpuPointer<'_, &'_ [u8]>> {
        self.object
            .clustering
            .as_ref()
            .map(|c| c.meta.gpu_offset_pointer(self.meta2_off))
    }

    /// Returns the GPU pointer to the clustering metadata 3 buffer, if clustering is enabled.
    #[allow(dead_code)]
    pub(crate) fn meta_3_pointer(&self) -> Option<GpuPointer<'_, &'_ [u8]>> {
        self.object
            .clustering
            .as_ref()
            .map(|c| c.meta.gpu_offset_pointer(self.meta3_off))
    }

    /// Returns the GPU pointer to the clustering metadata 4 buffer, if clustering is enabled.
    #[allow(dead_code)]
    pub(crate) fn meta_4_pointer(&self) -> Option<GpuPointer<'_, &'_ [u8]>> {
        self.object
            .clustering
            .as_ref()
            .map(|c| c.meta.gpu_offset_pointer(self.meta4_off))
    }

    /// Returns the number of TVB bytes used for this scene.
    pub(crate) fn used_bytes(&self) -> usize {
        self.object
            .with(|raw, _inner| raw.total_page_count.load(Ordering::Relaxed) as usize * PAGE_SIZE)
    }

    /// Returns whether the TVB overflowed while rendering this scene.
    pub(crate) fn overflowed(&self) -> bool {
        self.object.with(|raw, _inner| {
            raw.total_page_count.load(Ordering::Relaxed)
                > raw.pass_page_count.load(Ordering::Relaxed)
        })
    }
}

#[versions(AGX)]
impl Drop for Scene::ver {
    fn drop(&mut self) {
        let mut inner = self.object.buffer.inner.lock();
        assert_ne!(inner.active_scenes, 0);
        inner.active_scenes -= 1;

        if inner.active_scenes == 0 {
            mod_pr_debug!(
                "Buffer: no scenes left, dropping slot {}",
                inner.active_slot.take().unwrap().slot()
            );
            inner.active_slot = None;
        }
    }
}

/// Inner data for a single TVB buffer object.
#[versions(AGX)]
struct BufferInner {
    info: GpuObject<buffer::Info::ver>,
    ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
    ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
    blocks: KVec<GpuOnlyArray<u8>>,
    max_blocks: usize,
    max_blocks_nomemless: usize,
    mgr: BufferManager::ver,
    active_scenes: usize,
    active_slot: Option<slotalloc::Guard<BufferSlotInner::ver>>,
    last_token: Option<slotalloc::SlotToken>,
    tpc: Option<Arc<GpuArray<u8>>>,
    kernel_buffer: GpuArray<u8>,
    stats: GpuObject<buffer::Stats>,
    cfg: &'static hw::HwConfig,
    preempt1_size: usize,
    preempt2_size: usize,
    preempt3_size: usize,
    num_clusters: usize,
}

/// Locked and reference counted TVB buffer.
#[versions(AGX)]
pub(crate) struct Buffer {
    inner: Arc<Mutex<BufferInner::ver>>,
}

#[versions(AGX)]
impl Buffer::ver {
    /// Create a new Buffer for a given VM, given the per-VM allocators.
    pub(crate) fn new(
        gpu: &dyn gpu::GpuManager,
        alloc: &mut gpu::KernelAllocators,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        mgr: &BufferManager::ver,
    ) -> Result<Buffer::ver> {
        // These are the typical max numbers on macOS.
        // 8GB machines have this halved.
        let max_size: usize = 862_322_688; // bytes
        let max_size_nomemless = max_size / 3;

        let max_blocks = max_size / BLOCK_SIZE;
        let max_blocks_nomemless = max_size_nomemless / BLOCK_SIZE;
        let max_pages = max_blocks * PAGES_PER_BLOCK;
        let max_pages_nomemless = max_blocks_nomemless * PAGES_PER_BLOCK;

        let num_clusters = gpu.get_dyncfg().id.num_clusters as usize;
        let num_clusters_adj = if num_clusters > 1 {
            num_clusters + 1
        } else {
            1
        };

        let preempt1_size = num_clusters_adj * gpu.get_cfg().preempt1_size;
        let preempt2_size = num_clusters_adj * gpu.get_cfg().preempt2_size;
        let preempt3_size = num_clusters_adj * gpu.get_cfg().preempt3_size;

        let shared = &mut alloc.shared;
        let info = alloc.private.new_init(
            {
                let ualloc_priv = &ualloc_priv;
                try_init!(buffer::Info::ver {
                    block_ctl: shared.new_default::<buffer::BlockControl>()?,
                    counter: shared.new_default::<buffer::Counter>()?,
                    page_list: ualloc_priv.lock().array_empty(max_pages)?,
                    block_list: ualloc_priv.lock().array_empty(max_blocks * 2)?,
                })
            },
            |inner, _p| {
                try_init!(buffer::raw::Info::ver {
                    gpu_counter: 0x0,
                    unk_4: 0,
                    last_id: 0x0,
                    cur_id: -1,
                    unk_10: 0x0,
                    gpu_counter2: 0x0,
                    unk_18: 0x0,
                    #[ver(V < V13_0B4 || G >= G14X)]
                    unk_1c: 0x0,
                    page_list: inner.page_list.gpu_pointer(),
                    page_list_size: (4 * max_pages).try_into()?,
                    page_count: AtomicU32::new(0),
                    max_blocks: max_blocks.try_into()?,
                    block_count: AtomicU32::new(0),
                    unk_38: 0x0,
                    block_list: inner.block_list.gpu_pointer(),
                    block_ctl: inner.block_ctl.gpu_pointer(),
                    last_page: AtomicU32::new(0),
                    gpu_page_ptr1: 0x0,
                    gpu_page_ptr2: 0x0,
                    unk_58: 0x0,
                    block_size: BLOCK_SIZE as u32,
                    unk_60: U64(0x0),
                    counter: inner.counter.gpu_pointer(),
                    unk_70: 0x0,
                    unk_74: 0x0,
                    unk_78: 0x0,
                    unk_7c: 0x0,
                    unk_80: 0x1,
                    max_pages: max_pages.try_into()?,
                    max_pages_nomemless: max_pages_nomemless.try_into()?,
                    unk_8c: 0x0,
                    unk_90: Default::default(),
                })
            },
        )?;

        // Technically similar to Scene below, let's play it safe.
        let kernel_buffer = alloc.shared.array_empty(0x40)?;
        let stats = alloc
            .shared
            .new_object(Default::default(), |_inner| buffer::raw::Stats {
                reset: AtomicU32::from(1),
                ..Default::default()
            })?;

        Ok(Buffer::ver {
            inner: Arc::pin_init(
                Mutex::new(BufferInner::ver {
                    info,
                    ualloc,
                    ualloc_priv,
                    blocks: KVec::new(),
                    max_blocks,
                    max_blocks_nomemless,
                    mgr: mgr.clone(),
                    active_scenes: 0,
                    active_slot: None,
                    last_token: None,
                    tpc: None,
                    kernel_buffer,
                    stats,
                    cfg: gpu.get_cfg(),
                    preempt1_size,
                    preempt2_size,
                    preempt3_size,
                    num_clusters,
                }),
                GFP_KERNEL,
            )?,
        })
    }

    /// Returns the total block count allocated to this Buffer.
    pub(crate) fn block_count(&self) -> u32 {
        self.inner.lock().blocks.len() as u32
    }

    /// Returns the total size in bytes allocated to this Buffer.
    pub(crate) fn size(&self) -> usize {
        self.block_count() as usize * BLOCK_SIZE
    }

    /// Automatically grow the Buffer based on feedback from the statistics.
    pub(crate) fn auto_grow(&self) -> Result<bool> {
        let inner = self.inner.lock();

        let used_pages = inner.stats.with(|raw, _inner| {
            let used = raw.max_pages.load(Ordering::Relaxed);
            raw.reset.store(1, Ordering::Release);
            used as usize
        });

        let need_blocks = div_ceil(used_pages * 2, PAGES_PER_BLOCK).min(inner.max_blocks_nomemless);
        let want_blocks = div_ceil(used_pages * 3, PAGES_PER_BLOCK).min(inner.max_blocks_nomemless);

        let cur_count = inner.blocks.len();

        if need_blocks <= cur_count {
            Ok(false)
        } else {
            // Grow to 3x requested size (same logic as macOS)
            core::mem::drop(inner);
            self.ensure_blocks(want_blocks)?;
            Ok(true)
        }
    }

    /// Synchronously grow the Buffer.
    pub(crate) fn sync_grow(&self) {
        let inner = self.inner.lock();

        let cur_count = inner.blocks.len();
        core::mem::drop(inner);
        if self.ensure_blocks(cur_count + 10).is_err() {
            pr_err!("BufferManager: Failed to grow buffer synchronously\n");
        }
    }

    /// Ensure that the buffer has at least a certain minimum size in blocks.
    pub(crate) fn ensure_blocks(&self, min_blocks: usize) -> Result<bool> {
        let mut inner = self.inner.lock();

        let cur_count = inner.blocks.len();
        if cur_count >= min_blocks {
            return Ok(false);
        }
        if min_blocks > inner.max_blocks {
            return Err(ENOMEM);
        }

        let add_blocks = min_blocks - cur_count;
        let new_count = min_blocks;

        let mut new_blocks: KVec<GpuOnlyArray<u8>> = KVec::new();

        // Allocate the new blocks first, so if it fails they will be dropped
        let mut ualloc = inner.ualloc.lock();
        for _i in 0..add_blocks {
            new_blocks.push(ualloc.array_gpuonly(BLOCK_SIZE)?, GFP_KERNEL)?;
        }
        core::mem::drop(ualloc);

        // Then actually commit them
        inner.blocks.reserve(add_blocks, GFP_KERNEL)?;

        for (i, block) in new_blocks.into_iter().enumerate() {
            let page_num = (block.gpu_va().get() >> PAGE_SHIFT) as u32;

            inner
                .blocks
                .push(block, GFP_KERNEL)
                .expect("push() failed after reserve()");
            inner.info.block_list[2 * (cur_count + i)] = page_num;
            for j in 0..PAGES_PER_BLOCK {
                inner.info.page_list[(cur_count + i) * PAGES_PER_BLOCK + j] = page_num + j as u32;
            }
        }

        inner.info.block_ctl.with(|raw, _inner| {
            raw.total.store(new_count as u32, Ordering::SeqCst);
            raw.wptr.store(new_count as u32, Ordering::SeqCst);
        });

        /* Only do this update if the buffer manager is idle (which means we own it) */
        if inner.active_scenes == 0 {
            let page_count = (new_count * PAGES_PER_BLOCK) as u32;
            inner.info.with(|raw, _inner| {
                raw.page_count.store(page_count, Ordering::Relaxed);
                raw.block_count.store(new_count as u32, Ordering::Relaxed);
                raw.last_page.store(page_count - 1, Ordering::Relaxed);
            });
        }

        Ok(true)
    }

    /// Create a new [`Scene::ver`] (render pass) using this buffer.
    pub(crate) fn new_scene(
        &self,
        alloc: &mut gpu::KernelAllocators,
        tile_info: &TileInfo,
    ) -> Result<Scene::ver> {
        let mut inner = self.inner.lock();

        let tilemap_size = tile_info.tilemap_size;
        let tpc_size = tile_info.tpc_size;

        // TODO: what is this exactly?
        mod_pr_debug!("Buffer: Allocating TVB buffers\n");

        // This seems to be a list, with 4x2 bytes of headers and 8 bytes per entry.
        // On single-cluster devices, the used length always seems to be 1.
        // On M1 Ultra, it can grow and usually doesn't exceed 64 entries.
        // macOS allocates a whole 64K * 0x80 for this, so let's go with
        // that to be safe...
        let user_buffer = inner.ualloc.lock().array_empty(if inner.num_clusters > 1 {
            0x10080
        } else {
            0x80
        })?;

        let tvb_heapmeta = inner.ualloc.lock().array_empty(0x200)?;
        let tvb_tilemap = inner.ualloc.lock().array_empty(tilemap_size)?;

        mod_pr_debug!("Buffer: Allocating misc buffers\n");
        let preempt_buf = inner
            .ualloc
            .lock()
            .array_empty(inner.preempt1_size + inner.preempt2_size + inner.preempt3_size)?;

        let tpc = match inner.tpc.as_ref() {
            Some(buf) if buf.len() >= tpc_size => buf.clone(),
            _ => {
                // MacOS allocates this as shared GPU+FW, but
                // priv seems to work and might be faster?
                // Needs to be FW-writable anyway, so ualloc
                // won't work.
                let buf = Arc::try_new(
                    inner
                        .ualloc_priv
                        .lock()
                        .array_empty((tpc_size + mmu::UAT_PGMSK) & !mmu::UAT_PGMSK)?,
                )?;
                inner.tpc = Some(buf.clone());
                buf
            }
        };

        let mut meta1_size = 0;
        let mut meta2_size = 0;
        let mut meta3_size = 0;

        let clustering = if inner.num_clusters > 1 {
            let cfg = inner.cfg.clustering.as_ref().unwrap();

            // Maybe: (4x4 macro tiles + 1 global page)*n, 32bit each (17*4*n)
            // Unused on t602x?
            meta1_size = align(tile_info.meta1_blocks as usize * cfg.meta1_blocksize, 0x80);
            meta2_size = align(cfg.meta2_size, 0x80);
            meta3_size = align(cfg.meta3_size, 0x80);
            let meta4_size = cfg.meta4_size;

            let meta_size = meta1_size + meta2_size + meta3_size + meta4_size;

            mod_pr_debug!("Buffer: Allocating clustering buffers\n");
            let tilemaps = inner
                .ualloc
                .lock()
                .array_empty(cfg.max_splits * tilemap_size)?;
            let meta = inner.ualloc.lock().array_empty(meta_size)?;
            Some(buffer::ClusterBuffers { tilemaps, meta })
        } else {
            None
        };

        // Could be made strong, but we wind up with a deadlock if we try to grab the
        // pointer through the inner.buffer path inside the closure.
        let stats_pointer = inner.stats.weak_pointer();

        let _gpu = &mut alloc.gpu;

        // macOS allocates this as private. However, the firmware does not
        // DC CIVAC this before reading it (like it does most other things),
        // which causes odd cache incoherency bugs when combined with
        // speculation on the firmware side (maybe). This doesn't happen
        // on macOS because these structs are a circular pool that is mapped
        // already initialized. Just mark this shared for now.
        let scene = alloc.shared.new_init(
            try_init!(buffer::Scene::ver {
                user_buffer: user_buffer,
                buffer: self.clone(),
                tvb_heapmeta: tvb_heapmeta,
                tvb_tilemap: tvb_tilemap,
                tpc: tpc,
                clustering: clustering,
                preempt_buf: preempt_buf,
                #[ver(G >= G14X)]
                control_word: _gpu.array_empty(1)?,
            }),
            |inner, _p| {
                try_init!(buffer::raw::Scene::ver {
                    #[ver(G >= G14X)]
                    control_word: inner.control_word.gpu_pointer(),
                    #[ver(G >= G14X)]
                    control_word2: inner.control_word.gpu_pointer(),
                    pass_page_count: AtomicU32::new(0),
                    unk_4: 0,
                    unk_8: U64(0),
                    unk_10: U64(0),
                    user_buffer: inner.user_buffer.gpu_pointer(),
                    unk_20: 0,
                    #[ver(V >= V13_3)]
                    unk_28: U64(0),
                    stats: stats_pointer,
                    total_page_count: AtomicU32::new(0),
                    #[ver(G < G14X)]
                    unk_30: U64(0),
                    #[ver(G < G14X)]
                    unk_38: U64(0),
                })
            },
        )?;

        let mut rebind = false;

        if inner.active_slot.is_none() {
            assert_eq!(inner.active_scenes, 0);

            let slot = inner.mgr.0.get_inner(inner.last_token, |inner, mgr| {
                inner.owners[mgr.slot() as usize] = Some(self.clone());
                Ok(())
            })?;
            rebind = slot.changed();

            mod_pr_debug!("Buffer: assigning slot {} (rebind={})", slot.slot(), rebind);

            inner.last_token = Some(slot.token());
            inner.active_slot = Some(slot);
        }

        inner.active_scenes += 1;

        Ok(Scene::ver {
            object: scene,
            slot: inner.active_slot.as_ref().unwrap().slot(),
            rebind,
            preempt2_off: inner.preempt1_size,
            preempt3_off: inner.preempt1_size + inner.preempt2_size,
            meta2_off: meta1_size,
            meta3_off: meta1_size + meta2_size,
            meta4_off: meta1_size + meta2_size + meta3_size,
        })
    }

    /// Increment the buffer manager usage count. Should we done once we know the Scene is ready
    /// to be committed and used in commands submitted to the GPU.
    pub(crate) fn increment(&self) {
        let inner = self.inner.lock();
        inner.info.counter.with(|raw, _inner| {
            // We could use fetch_add, but the non-LSE atomic
            // sequence Rust produces confuses the hypervisor.
            // We have inner locked anyway, so this is not racy.
            let v = raw.count.load(Ordering::Relaxed);
            raw.count.store(v + 1, Ordering::Relaxed);
        });
    }

    pub(crate) fn any_ref(&self) -> Arc<dyn core::any::Any + Send + Sync> {
        self.inner.clone()
    }
}

#[versions(AGX)]
impl Clone for Buffer::ver {
    fn clone(&self) -> Self {
        Buffer::ver {
            inner: self.inner.clone(),
        }
    }
}

#[versions(AGX)]
struct BufferSlotInner();

#[versions(AGX)]
impl slotalloc::SlotItem for BufferSlotInner::ver {
    type Data = BufferManagerInner::ver;

    fn release(&mut self, data: &mut Self::Data, slot: u32) {
        mod_pr_debug!("EventManager: Released slot {}\n", slot);
        data.owners[slot as usize] = None;
    }
}

/// Inner data for the event manager, to be protected by the SlotAllocator lock.
#[versions(AGX)]
pub(crate) struct BufferManagerInner {
    owners: KVec<Option<Buffer::ver>>,
}

/// The GPU-global buffer manager, used to allocate and release buffer slots from the pool.
#[versions(AGX)]
pub(crate) struct BufferManager(slotalloc::SlotAllocator<BufferSlotInner::ver>);

#[versions(AGX)]
impl BufferManager::ver {
    pub(crate) fn new() -> Result<BufferManager::ver> {
        let mut owners = KVec::new();
        for _i in 0..(NUM_BUFFERS as usize) {
            owners.push(None, GFP_KERNEL)?;
        }
        Ok(BufferManager::ver(slotalloc::SlotAllocator::new(
            NUM_BUFFERS,
            BufferManagerInner::ver { owners },
            |_inner, _slot| BufferSlotInner::ver(),
            c_str!("BufferManager::SlotAllocator"),
            static_lock_class!(),
            static_lock_class!(),
        )?))
    }

    /// Signals a Buffer to synchronously grow.
    pub(crate) fn grow(&self, slot: u32) {
        match self
            .0
            .with_inner(|inner| inner.owners[slot as usize].as_ref().cloned())
        {
            Some(owner) => {
                pr_info!(
                    "BufferManager: Received synchronous grow request for slot {}, this is not generally expected\n",
                    slot
                );
                owner.sync_grow();
            }
            None => {
                pr_err!(
                    "BufferManager: Received grow request for empty slot {}\n",
                    slot
                );
            }
        }
    }
}

#[versions(AGX)]
impl Clone for BufferManager::ver {
    fn clone(&self) -> Self {
        BufferManager::ver(self.0.clone())
    }
}
