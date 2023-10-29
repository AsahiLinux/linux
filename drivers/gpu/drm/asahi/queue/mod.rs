// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Submission queue management
//!
//! This module implements the userspace view of submission queues and the logic to map userspace
//! submissions to firmware queues.

use kernel::dma_fence::*;
use kernel::prelude::*;
use kernel::{
    c_str, dma_fence,
    drm::gem::shmem::VMap,
    drm::sched,
    macros::versions,
    sync::{Arc, Mutex},
    uapi,
};

use crate::alloc::Allocator;
use crate::debug::*;
use crate::driver::{AsahiDevRef, AsahiDevice};
use crate::fw::types::*;
use crate::gpu::GpuManager;
use crate::inner_weak_ptr;
use crate::{alloc, buffer, channel, event, file, fw, gem, gpu, mmu, workqueue};

use core::sync::atomic::{AtomicU64, Ordering};

const DEBUG_CLASS: DebugFlags = DebugFlags::Queue;

const WQ_SIZE: u32 = 0x500;

mod common;
mod compute;
mod render;

/// Trait implemented by all versioned queues.
pub(crate) trait Queue: Send + Sync {
    fn submit(
        &mut self,
        id: u64,
        in_syncs: Vec<file::SyncItem>,
        out_syncs: Vec<file::SyncItem>,
        result_buf: Option<gem::ObjectRef>,
        commands: Vec<uapi::drm_asahi_command>,
    ) -> Result;
}

#[versions(AGX)]
struct SubQueue {
    wq: Arc<workqueue::WorkQueue::ver>,
}

#[versions(AGX)]
impl SubQueue::ver {
    fn new_job(&mut self, fence: dma_fence::Fence) -> SubQueueJob::ver {
        SubQueueJob::ver {
            wq: self.wq.clone(),
            fence: Some(fence),
            job: None,
        }
    }
}

#[versions(AGX)]
struct SubQueueJob {
    wq: Arc<workqueue::WorkQueue::ver>,
    job: Option<workqueue::Job::ver>,
    fence: Option<dma_fence::Fence>,
}

#[versions(AGX)]
impl SubQueueJob::ver {
    fn get(&mut self) -> Result<&mut workqueue::Job::ver> {
        if self.job.is_none() {
            mod_pr_debug!("SubQueueJob: Creating {:?} job\n", self.wq.pipe_type());
            self.job
                .replace(self.wq.new_job(self.fence.take().unwrap())?);
        }
        Ok(self.job.as_mut().expect("expected a Job"))
    }

    fn commit(&mut self) -> Result {
        match self.job.as_mut() {
            Some(job) => job.commit(),
            None => Ok(()),
        }
    }

    fn can_submit(&self) -> Option<Fence> {
        self.job.as_ref().and_then(|job| job.can_submit())
    }
}

#[versions(AGX)]
pub(crate) struct Queue {
    dev: AsahiDevRef,
    _sched: sched::Scheduler<QueueJob::ver>,
    entity: sched::Entity<QueueJob::ver>,
    vm: mmu::Vm,
    ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
    q_vtx: Option<SubQueue::ver>,
    q_frag: Option<SubQueue::ver>,
    q_comp: Option<SubQueue::ver>,
    buffer: Option<buffer::Buffer::ver>,
    gpu_context: Arc<workqueue::GpuContext>,
    notifier_list: Arc<GpuObject<fw::event::NotifierList>>,
    notifier: Arc<GpuObject<fw::event::Notifier::ver>>,
    id: u64,
    fence_ctx: FenceContexts,
    #[ver(V >= V13_0B4)]
    counter: AtomicU64,
}

#[versions(AGX)]
#[derive(Default)]
pub(crate) struct JobFence {
    id: u64,
    pending: AtomicU64,
}

#[versions(AGX)]
impl JobFence::ver {
    fn add_command(self: &FenceObject<Self>) {
        self.pending.fetch_add(1, Ordering::Relaxed);
    }

    fn command_complete(self: &FenceObject<Self>) {
        let remain = self.pending.fetch_sub(1, Ordering::Relaxed) - 1;
        mod_pr_debug!(
            "JobFence[{}]: Command complete (remain: {})\n",
            self.id,
            remain
        );
        if remain == 0 {
            mod_pr_debug!("JobFence[{}]: Signaling\n", self.id);
            if self.signal().is_err() {
                pr_err!("JobFence[{}]: Fence signal failed\n", self.id);
            }
        }
    }
}

#[versions(AGX)]
#[vtable]
impl dma_fence::FenceOps for JobFence::ver {
    const USE_64BIT_SEQNO: bool = true;

    fn get_driver_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("asahi")
    }
    fn get_timeline_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("queue")
    }
}

#[versions(AGX)]
pub(crate) struct QueueJob {
    dev: AsahiDevRef,
    vm_bind: mmu::VmBind,
    op_guard: Option<gpu::OpGuard>,
    sj_vtx: Option<SubQueueJob::ver>,
    sj_frag: Option<SubQueueJob::ver>,
    sj_comp: Option<SubQueueJob::ver>,
    fence: UserFence<JobFence::ver>,
    did_run: bool,
    id: u64,
}

#[versions(AGX)]
impl QueueJob::ver {
    fn get_vtx(&mut self) -> Result<&mut workqueue::Job::ver> {
        self.sj_vtx
            .as_mut()
            .ok_or_else(|| {
                cls_pr_debug!(Errors, "No vertex queue\n");
                EINVAL
            })?
            .get()
    }
    fn get_frag(&mut self) -> Result<&mut workqueue::Job::ver> {
        self.sj_frag
            .as_mut()
            .ok_or_else(|| {
                cls_pr_debug!(Errors, "No fragment queue\n");
                EINVAL
            })?
            .get()
    }
    fn get_comp(&mut self) -> Result<&mut workqueue::Job::ver> {
        self.sj_comp
            .as_mut()
            .ok_or_else(|| {
                cls_pr_debug!(Errors, "No compute queue\n");
                EINVAL
            })?
            .get()
    }

    fn commit(&mut self) -> Result {
        mod_dev_dbg!(self.dev, "QueueJob: Committing\n");

        self.sj_vtx.as_mut().map(|a| a.commit()).unwrap_or(Ok(()))?;
        self.sj_frag
            .as_mut()
            .map(|a| a.commit())
            .unwrap_or(Ok(()))?;
        self.sj_comp.as_mut().map(|a| a.commit()).unwrap_or(Ok(()))
    }
}

#[versions(AGX)]
impl sched::JobImpl for QueueJob::ver {
    fn prepare(job: &mut sched::Job<Self>) -> Option<Fence> {
        mod_dev_dbg!(job.dev, "QueueJob {}: Checking runnability\n", job.id);

        if let Some(sj) = job.sj_vtx.as_ref() {
            if let Some(fence) = sj.can_submit() {
                mod_dev_dbg!(
                    job.dev,
                    "QueueJob {}: Blocking due to vertex queue full\n",
                    job.id
                );
                return Some(fence);
            }
        }
        if let Some(sj) = job.sj_frag.as_ref() {
            if let Some(fence) = sj.can_submit() {
                mod_dev_dbg!(
                    job.dev,
                    "QueueJob {}: Blocking due to fragment queue full\n",
                    job.id
                );
                return Some(fence);
            }
        }
        if let Some(sj) = job.sj_comp.as_ref() {
            if let Some(fence) = sj.can_submit() {
                mod_dev_dbg!(
                    job.dev,
                    "QueueJob {}: Blocking due to compute queue full\n",
                    job.id
                );
                return Some(fence);
            }
        }
        None
    }

    #[allow(unused_assignments)]
    fn run(job: &mut sched::Job<Self>) -> Result<Option<dma_fence::Fence>> {
        mod_dev_dbg!(job.dev, "QueueJob {}: Running Job\n", job.id);

        let dev = job.dev.data();
        let gpu = match dev
            .gpu
            .clone()
            .arc_as_any()
            .downcast::<gpu::GpuManager::ver>()
        {
            Ok(gpu) => gpu,
            Err(_) => {
                dev_crit!(job.dev, "GpuManager mismatched with QueueJob!\n");
                return Err(EIO);
            }
        };

        if job.op_guard.is_none() {
            job.op_guard = Some(gpu.start_op()?);
        }

        // First submit all the commands for each queue. This can fail.

        let mut frag_job = None;
        let mut frag_sub = None;
        if let Some(sj) = job.sj_frag.as_mut() {
            frag_job = sj.job.take();
            if let Some(wqjob) = frag_job.as_mut() {
                mod_dev_dbg!(job.dev, "QueueJob {}: Submit fragment\n", job.id);
                frag_sub = Some(wqjob.submit()?);
            }
        }

        let mut vtx_job = None;
        let mut vtx_sub = None;
        if let Some(sj) = job.sj_vtx.as_mut() {
            vtx_job = sj.job.take();
            if let Some(wqjob) = vtx_job.as_mut() {
                mod_dev_dbg!(job.dev, "QueueJob {}: Submit vertex\n", job.id);
                vtx_sub = Some(wqjob.submit()?);
            }
        }

        let mut comp_job = None;
        let mut comp_sub = None;
        if let Some(sj) = job.sj_comp.as_mut() {
            comp_job = sj.job.take();
            if let Some(wqjob) = comp_job.as_mut() {
                mod_dev_dbg!(job.dev, "QueueJob {}: Submit compute\n", job.id);
                comp_sub = Some(wqjob.submit()?);
            }
        }

        // Now we fully commit to running the job
        mod_dev_dbg!(job.dev, "QueueJob {}: Run fragment\n", job.id);
        frag_sub.map(|a| gpu.run_job(a)).transpose()?;

        mod_dev_dbg!(job.dev, "QueueJob {}: Run vertex\n", job.id);
        vtx_sub.map(|a| gpu.run_job(a)).transpose()?;

        mod_dev_dbg!(job.dev, "QueueJob {}: Run compute\n", job.id);
        comp_sub.map(|a| gpu.run_job(a)).transpose()?;

        mod_dev_dbg!(job.dev, "QueueJob {}: Drop compute job\n", job.id);
        core::mem::drop(comp_job);
        mod_dev_dbg!(job.dev, "QueueJob {}: Drop vertex job\n", job.id);
        core::mem::drop(vtx_job);
        mod_dev_dbg!(job.dev, "QueueJob {}: Drop fragment job\n", job.id);
        core::mem::drop(frag_job);

        job.did_run = true;

        Ok(Some(Fence::from_fence(&job.fence)))
    }

    fn timed_out(job: &mut sched::Job<Self>) -> sched::Status {
        // FIXME: Handle timeouts properly
        dev_err!(
            job.dev,
            "QueueJob {}: Job timed out on the DRM scheduler, things will probably break (ran: {})\n",
            job.id, job.did_run
        );
        sched::Status::NoDevice
    }
}

#[versions(AGX)]
impl Drop for QueueJob::ver {
    fn drop(&mut self) {
        mod_dev_dbg!(self.dev, "QueueJob {}: Dropping\n", self.id);
    }
}

struct ResultWriter {
    vmap: VMap<gem::DriverObject>,
    offset: usize,
    len: usize,
}

impl ResultWriter {
    fn write<T>(&mut self, mut value: T) {
        let p: *mut u8 = &mut value as *mut _ as *mut u8;
        // SAFETY: We know `p` points to a type T of that size, and UAPI types must have
        // no padding and all bit patterns valid.
        let slice = unsafe { core::slice::from_raw_parts_mut(p, core::mem::size_of::<T>()) };
        let len = slice.len().min(self.len);
        self.vmap.as_mut_slice()[self.offset..self.offset + len].copy_from_slice(&slice[..len]);
    }
}

static QUEUE_NAME: &CStr = c_str!("asahi_fence");
static QUEUE_CLASS_KEY: kernel::sync::LockClassKey = kernel::static_lock_class!();

#[versions(AGX)]
impl Queue::ver {
    /// Create a new user queue.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        dev: &AsahiDevice,
        vm: mmu::Vm,
        alloc: &mut gpu::KernelAllocators,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        event_manager: Arc<event::EventManager>,
        mgr: &buffer::BufferManager::ver,
        id: u64,
        priority: u32,
        caps: u32,
    ) -> Result<Queue::ver> {
        mod_dev_dbg!(dev, "[Queue {}] Creating queue\n", id);

        let data = dev.data();

        let mut notifier_list = alloc.private.new_default::<fw::event::NotifierList>()?;

        let self_ptr = notifier_list.weak_pointer();
        notifier_list.with_mut(|raw, _inner| {
            raw.list_head.next = Some(inner_weak_ptr!(self_ptr, list_head));
        });

        let threshold = alloc.shared.new_default::<fw::event::Threshold>()?;

        let notifier: Arc<GpuObject<fw::event::Notifier::ver>> =
            Arc::try_new(alloc.private.new_init(
                try_init!(fw::event::Notifier::ver { threshold }),
                |inner, _p| {
                    try_init!(fw::event::raw::Notifier::ver {
                        threshold: inner.threshold.gpu_pointer(),
                        generation: AtomicU32::new(id as u32),
                        cur_count: AtomicU32::new(0),
                        unk_10: AtomicU32::new(0x50),
                        state: Default::default()
                    })
                },
            )?)?;

        let sched = sched::Scheduler::new(dev, WQ_SIZE, 0, 100000, c_str!("asahi_sched"))?;
        // Priorities are handled by the AGX scheduler, there is no meaning within a
        // per-queue scheduler.
        let entity = sched::Entity::new(&sched, sched::Priority::Normal)?;

        let buffer = if caps & uapi::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_RENDER != 0 {
            Some(buffer::Buffer::ver::new(
                &*data.gpu,
                alloc,
                ualloc.clone(),
                ualloc_priv,
                mgr,
            )?)
        } else {
            None
        };

        let mut ret = Queue::ver {
            dev: dev.into(),
            _sched: sched,
            entity,
            vm,
            ualloc,
            q_vtx: None,
            q_frag: None,
            q_comp: None,
            gpu_context: Arc::try_new(workqueue::GpuContext::new(
                dev,
                alloc,
                buffer.as_ref().map(|b| b.any_ref()),
            )?)?,
            buffer,
            notifier_list: Arc::try_new(notifier_list)?,
            notifier,
            id,
            fence_ctx: FenceContexts::new(1, QUEUE_NAME, QUEUE_CLASS_KEY)?,
            #[ver(V >= V13_0B4)]
            counter: AtomicU64::new(0),
        };

        // Rendering structures
        if caps & uapi::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_RENDER != 0 {
            let tvb_blocks = {
                let lock = crate::THIS_MODULE.kernel_param_lock();
                *crate::initial_tvb_size.read(&lock)
            };

            ret.buffer.as_ref().unwrap().ensure_blocks(tvb_blocks)?;

            ret.q_vtx = Some(SubQueue::ver {
                wq: workqueue::WorkQueue::ver::new(
                    dev,
                    alloc,
                    event_manager.clone(),
                    ret.gpu_context.clone(),
                    ret.notifier_list.clone(),
                    channel::PipeType::Vertex,
                    id,
                    priority,
                    WQ_SIZE,
                )?,
            });
        }

        // Rendering & blit structures
        if caps
            & (uapi::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_RENDER
                | uapi::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_BLIT)
            != 0
        {
            ret.q_frag = Some(SubQueue::ver {
                wq: workqueue::WorkQueue::ver::new(
                    dev,
                    alloc,
                    event_manager.clone(),
                    ret.gpu_context.clone(),
                    ret.notifier_list.clone(),
                    channel::PipeType::Fragment,
                    id,
                    priority,
                    WQ_SIZE,
                )?,
            });
        }

        // Compute structures
        if caps & uapi::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_COMPUTE != 0 {
            ret.q_comp = Some(SubQueue::ver {
                wq: workqueue::WorkQueue::ver::new(
                    dev,
                    alloc,
                    event_manager,
                    ret.gpu_context.clone(),
                    ret.notifier_list.clone(),
                    channel::PipeType::Compute,
                    id,
                    priority,
                    WQ_SIZE,
                )?,
            });
        }

        mod_dev_dbg!(dev, "[Queue {}] Queue created\n", id);
        Ok(ret)
    }
}

const SQ_RENDER: usize = uapi::drm_asahi_subqueue_DRM_ASAHI_SUBQUEUE_RENDER as usize;
const SQ_COMPUTE: usize = uapi::drm_asahi_subqueue_DRM_ASAHI_SUBQUEUE_COMPUTE as usize;
const SQ_COUNT: usize = uapi::drm_asahi_subqueue_DRM_ASAHI_SUBQUEUE_COUNT as usize;

#[versions(AGX)]
impl Queue for Queue::ver {
    fn submit(
        &mut self,
        id: u64,
        in_syncs: Vec<file::SyncItem>,
        out_syncs: Vec<file::SyncItem>,
        result_buf: Option<gem::ObjectRef>,
        commands: Vec<uapi::drm_asahi_command>,
    ) -> Result {
        let dev = self.dev.data();
        let gpu = match dev
            .gpu
            .clone()
            .arc_as_any()
            .downcast::<gpu::GpuManager::ver>()
        {
            Ok(gpu) => gpu,
            Err(_) => {
                dev_crit!(self.dev, "GpuManager mismatched with JobImpl!\n");
                return Err(EIO);
            }
        };

        mod_dev_dbg!(self.dev, "[Submission {}] Submit job\n", id);

        if gpu.is_crashed() {
            dev_err!(
                self.dev,
                "[Submission {}] GPU is crashed, cannot submit\n",
                id
            );
            return Err(ENODEV);
        }

        // Empty submissions are not legal
        if commands.is_empty() {
            cls_pr_debug!(Errors, "Empty submission\n");
            return Err(EINVAL);
        }

        let op_guard = if !in_syncs.is_empty() {
            Some(gpu.start_op()?)
        } else {
            None
        };

        let mut events: [Vec<Option<workqueue::QueueEventInfo::ver>>; SQ_COUNT] =
            Default::default();

        events[SQ_RENDER].try_push(self.q_frag.as_ref().and_then(|a| a.wq.event_info()))?;
        events[SQ_COMPUTE].try_push(self.q_comp.as_ref().and_then(|a| a.wq.event_info()))?;

        let vm_bind = gpu.bind_vm(&self.vm)?;
        let vm_slot = vm_bind.slot();

        mod_dev_dbg!(self.dev, "[Submission {}] Creating job\n", id);

        let fence: UserFence<JobFence::ver> = self
            .fence_ctx
            .new_fence::<JobFence::ver>(
                0,
                JobFence::ver {
                    id,
                    pending: Default::default(),
                },
            )?
            .into();

        let mut job = self.entity.new_job(QueueJob::ver {
            dev: self.dev.clone(),
            vm_bind,
            op_guard,
            sj_vtx: self
                .q_vtx
                .as_mut()
                .map(|a| a.new_job(Fence::from_fence(&fence))),
            sj_frag: self
                .q_frag
                .as_mut()
                .map(|a| a.new_job(Fence::from_fence(&fence))),
            sj_comp: self
                .q_comp
                .as_mut()
                .map(|a| a.new_job(Fence::from_fence(&fence))),
            fence,
            did_run: false,
            id,
        })?;

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Adding {} in_syncs\n",
            id,
            in_syncs.len()
        );
        for sync in in_syncs {
            job.add_dependency(sync.fence.expect("in_sync missing fence"))?;
        }

        let mut last_render = None;
        let mut last_compute = None;

        for (i, cmd) in commands.iter().enumerate() {
            match cmd.cmd_type {
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER => last_render = Some(i),
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE => last_compute = Some(i),
                _ => {
                    cls_pr_debug!(Errors, "Unknown command type {}\n", cmd.cmd_type);
                    return Err(EINVAL);
                }
            }
        }

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Submitting {} commands\n",
            id,
            commands.len()
        );
        for (i, cmd) in commands.into_iter().enumerate() {
            for (queue_idx, index) in cmd.barriers.iter().enumerate() {
                if *index == uapi::DRM_ASAHI_BARRIER_NONE as u32 {
                    continue;
                }
                if let Some(event) = events[queue_idx].get(*index as usize).ok_or_else(|| {
                    cls_pr_debug!(Errors, "Invalid barrier #{}: {}\n", queue_idx, index);
                    EINVAL
                })? {
                    let mut alloc = gpu.alloc();
                    let queue_job = match cmd.cmd_type {
                        uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER => job.get_vtx()?,
                        uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE => job.get_comp()?,
                        _ => return Err(EINVAL),
                    };
                    mod_dev_dbg!(self.dev, "[Submission {}] Create Explicit Barrier\n", id);
                    let barrier = alloc.private.new_init(
                        kernel::init::zeroed::<fw::workqueue::Barrier>(),
                        |_inner, _p| {
                            let queue_job = &queue_job;
                            try_init!(fw::workqueue::raw::Barrier {
                                tag: fw::workqueue::CommandType::Barrier,
                                wait_stamp: event.fw_stamp_pointer,
                                wait_value: event.value,
                                wait_slot: event.slot,
                                stamp_self: queue_job.event_info().value.next(),
                                uuid: 0xffffbbbb,
                                barrier_type: 0,
                                padding: Default::default(),
                            })
                        },
                    )?;
                    mod_dev_dbg!(self.dev, "[Submission {}] Add Explicit Barrier\n", id);
                    queue_job.add(barrier, vm_slot)?;
                } else {
                    assert!(*index == 0);
                }
            }

            let result_writer = match result_buf.as_ref() {
                None => {
                    if cmd.result_offset != 0 || cmd.result_size != 0 {
                        cls_pr_debug!(Errors, "No result buffer but result requested\n");
                        return Err(EINVAL);
                    }
                    None
                }
                Some(buf) => {
                    if cmd.result_size != 0 {
                        let end_offset = cmd
                            .result_offset
                            .checked_add(cmd.result_size)
                            .ok_or_else(|| {
                                cls_pr_debug!(Errors, "result_offset + result_size overflow\n");
                                EINVAL
                            })?;
                        if end_offset > buf.size() as u64 {
                            cls_pr_debug!(
                                Errors,
                                "Result buffer overflow ({} + {} > {})\n",
                                cmd.result_offset,
                                cmd.result_size,
                                buf.size()
                            );

                            return Err(EINVAL);
                        }
                        Some(ResultWriter {
                            vmap: buf.gem.vmap()?,
                            offset: cmd.result_offset.try_into()?,
                            len: cmd.result_size.try_into()?,
                        })
                    } else {
                        None
                    }
                }
            };

            match cmd.cmd_type {
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER => {
                    self.submit_render(
                        &mut job,
                        &cmd,
                        result_writer,
                        id,
                        last_render.unwrap() == i,
                    )?;
                    events[SQ_RENDER].try_push(Some(
                        job.sj_frag
                            .as_ref()
                            .expect("No frag queue?")
                            .job
                            .as_ref()
                            .expect("No frag job?")
                            .event_info(),
                    ))?;
                }
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE => {
                    self.submit_compute(
                        &mut job,
                        &cmd,
                        result_writer,
                        id,
                        last_compute.unwrap() == i,
                    )?;
                    events[SQ_COMPUTE].try_push(Some(
                        job.sj_comp
                            .as_ref()
                            .expect("No comp queue?")
                            .job
                            .as_ref()
                            .expect("No comp job?")
                            .event_info(),
                    ))?;
                }
                _ => return Err(EINVAL),
            }
        }

        mod_dev_dbg!(self.dev, "Queue: Committing job\n");
        job.commit()?;

        mod_dev_dbg!(self.dev, "Queue: Arming job\n");
        let job = job.arm();
        let out_fence = job.fences().finished();
        mod_dev_dbg!(self.dev, "Queue: Pushing job\n");
        job.push();

        mod_dev_dbg!(self.dev, "Queue: Adding {} out_syncs\n", out_syncs.len());
        for mut sync in out_syncs {
            if let Some(chain) = sync.chain_fence.take() {
                sync.syncobj
                    .add_point(chain, &out_fence, sync.timeline_value);
            } else {
                sync.syncobj.replace_fence(Some(&out_fence));
            }
        }

        Ok(())
    }
}

#[versions(AGX)]
impl Drop for Queue::ver {
    fn drop(&mut self) {
        mod_dev_dbg!(self.dev, "[Queue {}] Dropping queue\n", self.id);
    }
}
