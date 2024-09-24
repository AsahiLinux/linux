// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU command execution queues
//!
//! The AGX GPU firmware schedules GPU work commands out of work queues, which are ring buffers of
//! pointers to work commands. There can be an arbitrary number of work queues. Work queues have an
//! associated type (vertex, fragment, or compute) and may only contain generic commands or commands
//! specific to that type.
//!
//! This module manages queueing work commands into a work queue and submitting them for execution
//! by the firmware. An active work queue needs an event to signal completion of its work, which is
//! owned by what we call a batch. This event then notifies the work queue when work is completed,
//! and that triggers freeing of all resources associated with that work. An idle work queue gives
//! up its associated event.

use crate::debug::*;
use crate::fw::channels::{ChannelErrorType, PipeType};
use crate::fw::types::*;
use crate::fw::workqueue::*;
use crate::no_debug;
use crate::object::OpaqueGpuObject;
use crate::regs::FaultReason;
use crate::{channel, driver, event, fw, gpu, object, regs};
use core::any::Any;
use core::num::NonZeroU64;
use core::sync::atomic::Ordering;
use kernel::{
    c_str, dma_fence,
    error::code::*,
    prelude::*,
    sync::{
        lock::{mutex::MutexBackend, Guard},
        Arc, Mutex,
    },
    uapi,
};

const DEBUG_CLASS: DebugFlags = DebugFlags::WorkQueue;

const MAX_JOB_SLOTS: u32 = 127;

/// An enum of possible errors that might cause a piece of work to fail execution.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum WorkError {
    /// GPU timeout (command execution took too long).
    Timeout,
    /// GPU MMU fault (invalid access).
    Fault(regs::FaultInfo),
    /// Work failed due to an error caused by other concurrent GPU work.
    Killed,
    /// Channel error
    ChannelError(ChannelErrorType),
    /// The GPU crashed.
    NoDevice,
    /// Unknown reason.
    Unknown,
}

impl From<WorkError> for uapi::drm_asahi_result_info {
    fn from(err: WorkError) -> Self {
        match err {
            WorkError::Fault(info) => Self {
                status: uapi::drm_asahi_status_DRM_ASAHI_STATUS_FAULT,
                fault_type: match info.reason {
                    FaultReason::Unmapped => uapi::drm_asahi_fault_DRM_ASAHI_FAULT_UNMAPPED,
                    FaultReason::AfFault => uapi::drm_asahi_fault_DRM_ASAHI_FAULT_AF_FAULT,
                    FaultReason::WriteOnly => uapi::drm_asahi_fault_DRM_ASAHI_FAULT_WRITE_ONLY,
                    FaultReason::ReadOnly => uapi::drm_asahi_fault_DRM_ASAHI_FAULT_READ_ONLY,
                    FaultReason::NoAccess => uapi::drm_asahi_fault_DRM_ASAHI_FAULT_NO_ACCESS,
                    FaultReason::Unknown(_) => uapi::drm_asahi_fault_DRM_ASAHI_FAULT_UNKNOWN,
                },
                unit: info.unit_code.into(),
                sideband: info.sideband.into(),
                level: info.level,
                extra: info.unk_5.into(),
                is_read: info.read as u8,
                pad: 0,
                address: info.address,
            },
            a => Self {
                status: match a {
                    WorkError::Timeout => uapi::drm_asahi_status_DRM_ASAHI_STATUS_TIMEOUT,
                    WorkError::Killed => uapi::drm_asahi_status_DRM_ASAHI_STATUS_KILLED,
                    WorkError::ChannelError(_) => {
                        uapi::drm_asahi_status_DRM_ASAHI_STATUS_CHANNEL_ERROR
                    }
                    WorkError::NoDevice => uapi::drm_asahi_status_DRM_ASAHI_STATUS_NO_DEVICE,
                    _ => uapi::drm_asahi_status_DRM_ASAHI_STATUS_UNKNOWN_ERROR,
                },
                ..Default::default()
            },
        }
    }
}

impl From<WorkError> for kernel::error::Error {
    fn from(err: WorkError) -> Self {
        match err {
            WorkError::Timeout => ETIMEDOUT,
            // Not EFAULT because that's for userspace faults
            WorkError::Fault(_) => EIO,
            WorkError::Unknown => ENODATA,
            WorkError::Killed => ECANCELED,
            WorkError::NoDevice => ENODEV,
            WorkError::ChannelError(_) => EIO,
        }
    }
}

/// A GPU context tracking structure, which must be explicitly invalidated when dropped.
pub(crate) struct GpuContext {
    dev: driver::AsahiDevRef,
    data: Option<KBox<GpuObject<fw::workqueue::GpuContextData>>>,
}
no_debug!(GpuContext);

impl GpuContext {
    /// Allocate a new GPU context.
    pub(crate) fn new(
        dev: &driver::AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
        buffer: Option<Arc<dyn core::any::Any + Send + Sync>>,
    ) -> Result<GpuContext> {
        Ok(GpuContext {
            dev: dev.into(),
            data: Some(KBox::new(
                alloc.shared.new_object(
                    fw::workqueue::GpuContextData { _buffer: buffer },
                    |_inner| Default::default(),
                )?,
                GFP_KERNEL,
            )?),
        })
    }

    /// Returns the GPU pointer to the inner GPU context data structure.
    pub(crate) fn gpu_pointer(&self) -> GpuPointer<'_, fw::workqueue::GpuContextData> {
        self.data.as_ref().unwrap().gpu_pointer()
    }
}

impl Drop for GpuContext {
    fn drop(&mut self) {
        mod_dev_dbg!(self.dev, "GpuContext: Freeing GPU context\n");
        let dev = self.dev.data();
        let data = self.data.take().unwrap();
        dev.gpu.free_context(data);
    }
}

struct SubmittedWork<O, C>
where
    O: OpaqueGpuObject,
    C: FnOnce(&mut O, Option<WorkError>) + Send + Sync + 'static,
{
    object: O,
    value: EventValue,
    error: Option<WorkError>,
    wptr: u32,
    vm_slot: u32,
    callback: Option<C>,
    fence: dma_fence::Fence,
}

pub(crate) trait GenSubmittedWork: Send + Sync {
    fn gpu_va(&self) -> NonZeroU64;
    fn value(&self) -> event::EventValue;
    fn wptr(&self) -> u32;
    fn set_wptr(&mut self, wptr: u32);
    fn mark_error(&mut self, error: WorkError);
    fn complete(&mut self);
    fn get_fence(&self) -> dma_fence::Fence;
}

impl<O: OpaqueGpuObject, C: FnOnce(&mut O, Option<WorkError>) + Send + Sync> GenSubmittedWork
    for SubmittedWork<O, C>
{
    fn gpu_va(&self) -> NonZeroU64 {
        self.object.gpu_va()
    }

    fn value(&self) -> event::EventValue {
        self.value
    }

    fn wptr(&self) -> u32 {
        self.wptr
    }

    fn set_wptr(&mut self, wptr: u32) {
        self.wptr = wptr;
    }

    fn complete(&mut self) {
        if let Some(cb) = self.callback.take() {
            cb(&mut self.object, self.error);
        }
    }

    fn mark_error(&mut self, error: WorkError) {
        mod_pr_debug!("WorkQueue: Command at value {:#x?} failed\n", self.value);
        self.error = Some(match error {
            WorkError::Fault(info) if info.vm_slot != self.vm_slot => WorkError::Killed,
            err => err,
        });
    }

    fn get_fence(&self) -> dma_fence::Fence {
        self.fence.clone()
    }
}

/// Inner data for managing a single work queue.
#[versions(AGX)]
struct WorkQueueInner {
    dev: driver::AsahiDevRef,
    event_manager: Arc<event::EventManager>,
    info: GpuObject<QueueInfo::ver>,
    new: bool,
    pipe_type: PipeType,
    size: u32,
    wptr: u32,
    pending: Vec<Box<dyn GenSubmittedWork>>,
    last_token: Option<event::Token>,
    pending_jobs: usize,
    last_submitted: Option<event::EventValue>,
    last_completed: Option<event::EventValue>,
    event: Option<(event::Event, event::EventValue)>,
    priority: u32,
    commit_seq: u64,
    submit_seq: u64,
    event_seq: u64,
}

/// An instance of a work queue.
#[versions(AGX)]
#[pin_data]
pub(crate) struct WorkQueue {
    info_pointer: GpuWeakPointer<QueueInfo::ver>,
    #[pin]
    inner: Mutex<WorkQueueInner::ver>,
}

#[versions(AGX)]
impl WorkQueueInner::ver {
    /// Return the GPU done pointer, representing how many work items have been completed by the
    /// GPU.
    fn doneptr(&self) -> u32 {
        self.info
            .state
            .with(|raw, _inner| raw.gpu_doneptr.load(Ordering::Acquire))
    }
}

#[versions(AGX)]
#[derive(Copy, Clone)]
pub(crate) struct QueueEventInfo {
    pub(crate) stamp_pointer: GpuWeakPointer<Stamp>,
    pub(crate) fw_stamp_pointer: GpuWeakPointer<FwStamp>,
    pub(crate) slot: u32,
    pub(crate) value: event::EventValue,
    pub(crate) cmd_seq: u64,
    pub(crate) event_seq: u64,
    pub(crate) info_ptr: GpuWeakPointer<QueueInfo::ver>,
}

#[versions(AGX)]
pub(crate) struct Job {
    wq: Arc<WorkQueue::ver>,
    event_info: QueueEventInfo::ver,
    start_value: EventValue,
    pending: Vec<Box<dyn GenSubmittedWork>>,
    committed: bool,
    submitted: bool,
    event_count: usize,
    fence: dma_fence::Fence,
}

#[versions(AGX)]
pub(crate) struct JobSubmission<'a> {
    inner: Option<Guard<'a, WorkQueueInner::ver, MutexBackend>>,
    wptr: u32,
    event_count: usize,
    command_count: usize,
}

#[versions(AGX)]
impl Job::ver {
    pub(crate) fn event_info(&self) -> QueueEventInfo::ver {
        let mut info = self.event_info;
        info.cmd_seq += self.pending.len() as u64;
        info.event_seq += self.event_count as u64;

        info
    }

    pub(crate) fn next_seq(&mut self) {
        self.event_count += 1;
        self.event_info.value.increment();
    }

    pub(crate) fn add<O: object::OpaqueGpuObject + 'static>(
        &mut self,
        command: O,
        vm_slot: u32,
    ) -> Result {
        self.add_cb(command, vm_slot, |_, _| {})
    }

    pub(crate) fn add_cb<O: object::OpaqueGpuObject + 'static>(
        &mut self,
        command: O,
        vm_slot: u32,
        callback: impl FnOnce(&mut O, Option<WorkError>) + Sync + Send + 'static,
    ) -> Result {
        if self.committed {
            pr_err!("WorkQueue: Tried to mutate committed Job\n");
            return Err(EINVAL);
        }

        self.pending.push(
            Box::new(
                SubmittedWork::<_, _> {
                    object: command,
                    value: self.event_info.value.next(),
                    error: None,
                    callback: Some(callback),
                    wptr: 0,
                    vm_slot,
                    fence: self.fence.clone(),
                },
                GFP_KERNEL,
            )?,
            GFP_KERNEL,
        )?;

        Ok(())
    }

    pub(crate) fn commit(&mut self) -> Result {
        if self.committed {
            pr_err!("WorkQueue: Tried to commit committed Job\n");
            return Err(EINVAL);
        }

        if self.pending.is_empty() {
            pr_err!("WorkQueue: Job::commit() with no commands\n");
            return Err(EINVAL);
        }

        let mut inner = self.wq.inner.lock();

        let ev = inner.event.as_mut().expect("WorkQueue: Job lost its event");

        if ev.1 != self.start_value {
            pr_err!(
                "WorkQueue: Job::commit() out of order (event slot {} {:?} != {:?}\n",
                ev.0.slot(),
                ev.1,
                self.start_value
            );
            return Err(EINVAL);
        }

        ev.1 = self.event_info.value;
        inner.commit_seq += self.pending.len() as u64;
        inner.event_seq += self.event_count as u64;
        self.committed = true;

        Ok(())
    }

    pub(crate) fn can_submit(&self) -> Option<dma_fence::Fence> {
        let inner = self.wq.inner.lock();
        if inner.free_slots() > self.event_count && inner.free_space() > self.pending.len() {
            None
        } else if let Some(work) = inner.pending.first() {
            Some(work.get_fence())
        } else {
            pr_err!(
                "WorkQueue: Cannot submit, but queue is empty? {} > {}, {} > {} (pend={} ls={:#x?} lc={:#x?}) ev={:#x?} cur={:#x?} slot {:?}\n",
                inner.free_slots(),
                self.event_count,
                inner.free_space(),
                self.pending.len(),
                inner.pending.len(),
                inner.last_submitted,
                inner.last_completed,
                inner.event.as_ref().map(|a| a.1),
                inner.event.as_ref().map(|a| a.0.current()),
                inner.event.as_ref().map(|a| a.0.slot()),
            );
            None
        }
    }

    pub(crate) fn submit(&mut self) -> Result<JobSubmission::ver<'_>> {
        if !self.committed {
            pr_err!("WorkQueue: Tried to submit uncommitted Job\n");
            return Err(EINVAL);
        }

        if self.submitted {
            pr_err!("WorkQueue: Tried to submit Job twice\n");
            return Err(EINVAL);
        }

        if self.pending.is_empty() {
            pr_err!("WorkQueue: Job::submit() with no commands\n");
            return Err(EINVAL);
        }

        let mut inner = self.wq.inner.lock();

        if inner.submit_seq != self.event_info.cmd_seq {
            pr_err!(
                "WorkQueue: Job::submit() out of order (submit_seq {} != {})\n",
                inner.submit_seq,
                self.event_info.cmd_seq
            );
            return Err(EINVAL);
        }

        if inner.commit_seq < (self.event_info.cmd_seq + self.pending.len() as u64) {
            pr_err!(
                "WorkQueue: Job::submit() out of order (commit_seq {} != {})\n",
                inner.commit_seq,
                (self.event_info.cmd_seq + self.pending.len() as u64)
            );
            return Err(EINVAL);
        }

        let mut wptr = inner.wptr;
        let command_count = self.pending.len();

        if inner.free_space() <= command_count {
            pr_err!("WorkQueue: Job does not fit in ring buffer\n");
            return Err(EBUSY);
        }

        inner.pending.reserve(command_count, GFP_KERNEL)?;

        inner.last_submitted = Some(self.event_info.value);
        mod_dev_dbg!(
            inner.dev,
            "WorkQueue: submitting {} cmds at {:#x?}, lc {:#x?}, cur {:#x?}, pending {}, events {}\n",
            self.pending.len(),
            inner.last_submitted,
            inner.last_completed,
            inner.event.as_ref().map(|a| a.0.current()),
            inner.pending.len(),
            self.event_count,
        );

        for mut command in self.pending.drain(..) {
            command.set_wptr(wptr);

            let next_wptr = (wptr + 1) % inner.size;
            assert!(inner.doneptr() != next_wptr);
            inner.info.ring[wptr as usize] = command.gpu_va().get();
            wptr = next_wptr;

            // Cannot fail, since we did a reserve(1) above
            inner
                .pending
                .push(command, GFP_KERNEL)
                .expect("push() failed after reserve()");
        }

        self.submitted = true;

        Ok(JobSubmission::ver {
            inner: Some(inner),
            wptr,
            command_count,
            event_count: self.event_count,
        })
    }
}

#[versions(AGX)]
impl<'a> JobSubmission::ver<'a> {
    pub(crate) fn run(mut self, channel: &mut channel::PipeChannel::ver) {
        let command_count = self.command_count;
        let mut inner = self.inner.take().expect("No inner?");
        let wptr = self.wptr;
        core::mem::forget(self);

        inner
            .info
            .state
            .with(|raw, _inner| raw.cpu_wptr.store(wptr, Ordering::Release));

        inner.wptr = wptr;

        let event = inner.event.as_mut().expect("JobSubmission lost its event");

        let event_slot = event.0.slot();

        let msg = fw::channels::RunWorkQueueMsg::ver {
            pipe_type: inner.pipe_type,
            work_queue: Some(inner.info.weak_pointer()),
            wptr: inner.wptr,
            event_slot,
            is_new: inner.new,
            __pad: Default::default(),
        };
        channel.send(&msg);
        inner.new = false;

        inner.submit_seq += command_count as u64;
    }

    pub(crate) fn pipe_type(&self) -> PipeType {
        self.inner.as_ref().expect("No inner?").pipe_type
    }

    pub(crate) fn priority(&self) -> u32 {
        self.inner.as_ref().expect("No inner?").priority
    }
}

#[versions(AGX)]
impl Drop for Job::ver {
    fn drop(&mut self) {
        mod_pr_debug!("WorkQueue: Dropping Job\n");
        let mut inner = self.wq.inner.lock();

        if !self.committed {
            pr_info!(
                "WorkQueue: Dropping uncommitted job with {} events\n",
                self.event_count
            );
        }

        if self.committed && !self.submitted {
            let pipe_type = inner.pipe_type;
            let event = inner.event.as_mut().expect("Job lost its event");
            pr_info!(
                "WorkQueue({:?}): Roll back {} events (slot {} val {:#x?}) and {} commands\n",
                pipe_type,
                self.event_count,
                event.0.slot(),
                event.1,
                self.pending.len()
            );
            event.1.sub(self.event_count as u32);
            inner.commit_seq -= self.pending.len() as u64;
            inner.event_seq -= self.event_count as u64;
        }

        inner.pending_jobs -= 1;

        if inner.pending.is_empty() && inner.pending_jobs == 0 {
            mod_pr_debug!("WorkQueue({:?}): Dropping event\n", inner.pipe_type);
            inner.event = None;
            inner.last_submitted = None;
            inner.last_completed = None;
        }
        mod_pr_debug!("WorkQueue({:?}): Dropped Job\n", inner.pipe_type);
    }
}

#[versions(AGX)]
impl<'a> Drop for JobSubmission::ver<'a> {
    fn drop(&mut self) {
        let inner = self.inner.as_mut().expect("No inner?");
        mod_pr_debug!("WorkQueue({:?}): Dropping JobSubmission\n", inner.pipe_type);

        let new_len = inner.pending.len() - self.command_count;
        inner.pending.truncate(new_len);

        let pipe_type = inner.pipe_type;
        let event = inner.event.as_mut().expect("JobSubmission lost its event");
        pr_info!(
            "WorkQueue({:?}): JobSubmission: Roll back {} events (slot {} val {:#x?}) and {} commands\n",
            pipe_type,
            self.event_count,
            event.0.slot(),
            event.1,
            self.command_count
        );
        event.1.sub(self.event_count as u32);
        let val = event.1;
        inner.commit_seq -= self.command_count as u64;
        inner.event_seq -= self.event_count as u64;
        inner.last_submitted = Some(val);
        mod_pr_debug!("WorkQueue({:?}): Dropped JobSubmission\n", inner.pipe_type);
    }
}

#[versions(AGX)]
impl WorkQueueInner::ver {
    /// Return the number of free entries in the workqueue
    pub(crate) fn free_space(&self) -> usize {
        self.size as usize - self.pending.len() - 1
    }

    pub(crate) fn free_slots(&self) -> usize {
        let busy_slots = if let Some(ls) = self.last_submitted {
            let lc = self
                .last_completed
                .expect("last_submitted but not completed?");
            ls.delta(&lc)
        } else {
            0
        };

        ((MAX_JOB_SLOTS as i32) - busy_slots).max(0) as usize
    }
}

#[versions(AGX)]
impl WorkQueue::ver {
    /// Create a new WorkQueue of a given type and priority.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        dev: &driver::AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
        event_manager: Arc<event::EventManager>,
        gpu_context: Arc<GpuContext>,
        notifier_list: Arc<GpuObject<fw::event::NotifierList>>,
        pipe_type: PipeType,
        id: u64,
        priority: u32,
        size: u32,
    ) -> Result<Arc<WorkQueue::ver>> {
        let gpu_buf = alloc.private.array_empty_tagged(0x2c18, b"GPBF")?;
        let mut state = alloc.shared.new_default::<RingState>()?;
        let ring = alloc.shared.array_empty(size as usize)?;
        let inner = WorkQueueInner::ver {
            dev: dev.into(),
            event_manager,
            // Use shared (coherent) state with verbose faults so we can dump state correctly
            info: if debug_enabled(DebugFlags::VerboseFaults) {
                &mut alloc.shared
            } else {
                &mut alloc.private
            }
            .new_init(
                try_init!(QueueInfo::ver {
                    state: {
                        state.with_mut(|raw, _inner| {
                            raw.rb_size = size;
                        });
                        state
                    },
                    ring,
                    gpu_buf,
                    notifier_list: notifier_list,
                    gpu_context: gpu_context,
                }),
                |inner, _p| {
                    try_init!(raw::QueueInfo::ver {
                        state: inner.state.gpu_pointer(),
                        ring: inner.ring.gpu_pointer(),
                        notifier_list: inner.notifier_list.gpu_pointer(),
                        gpu_buf: inner.gpu_buf.gpu_pointer(),
                        gpu_rptr1: Default::default(),
                        gpu_rptr2: Default::default(),
                        gpu_rptr3: Default::default(),
                        event_id: AtomicI32::new(-1),
                        priority: *raw::PRIORITY.get(priority as usize).ok_or(EINVAL)?,
                        unk_4c: -1,
                        uuid: id as u32,
                        unk_54: -1,
                        unk_58: Default::default(),
                        busy: Default::default(),
                        __pad: Default::default(),
                        #[ver(V >= V13_2 && G < G14X)]
                        unk_84_0: 0,
                        unk_84_state: Default::default(),
                        error_count: Default::default(),
                        unk_8c: 0,
                        unk_90: 0,
                        unk_94: 0,
                        pending: Default::default(),
                        unk_9c: 0,
                        gpu_context: inner.gpu_context.gpu_pointer(),
                        unk_a8: Default::default(),
                        #[ver(V >= V13_2 && G < G14X)]
                        unk_b0: 0,
                    })
                },
            )?,
            new: true,
            pipe_type,
            size,
            wptr: 0,
            pending: KVec::new(),
            last_token: None,
            event: None,
            priority,
            pending_jobs: 0,
            commit_seq: 0,
            submit_seq: 0,
            event_seq: 0,
            last_completed: None,
            last_submitted: None,
        };

        let info_pointer = inner.info.weak_pointer();

        Arc::pin_init(
            pin_init!(Self {
                info_pointer,
                inner <- match pipe_type {
                    PipeType::Vertex => Mutex::new_named(inner, c_str!("WorkQueue::inner (Vertex)")),
                    PipeType::Fragment => Mutex::new_named(inner, c_str!("WorkQueue::inner (Fragment)")),
                    PipeType::Compute => Mutex::new_named(inner, c_str!("WorkQueue::inner (Compute)")),
                },
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn event_info(&self) -> Option<QueueEventInfo::ver> {
        let inner = self.inner.lock();

        inner.event.as_ref().map(|ev| QueueEventInfo::ver {
            stamp_pointer: ev.0.stamp_pointer(),
            fw_stamp_pointer: ev.0.fw_stamp_pointer(),
            slot: ev.0.slot(),
            value: ev.1,
            cmd_seq: inner.commit_seq,
            event_seq: inner.event_seq,
            info_ptr: self.info_pointer,
        })
    }

    pub(crate) fn new_job(self: &Arc<Self>, fence: dma_fence::Fence) -> Result<Job::ver> {
        let mut inner = self.inner.lock();

        if inner.event.is_none() {
            mod_pr_debug!("WorkQueue({:?}): Grabbing event\n", inner.pipe_type);
            let event = inner.event_manager.get(inner.last_token, self.clone())?;
            let cur = event.current();
            inner.last_token = Some(event.token());
            mod_pr_debug!(
                "WorkQueue({:?}): Grabbed event slot {}: {:#x?}\n",
                inner.pipe_type,
                event.slot(),
                cur
            );
            inner.event = Some((event, cur));
            inner.last_submitted = Some(cur);
            inner.last_completed = Some(cur);
        }

        inner.pending_jobs += 1;

        let ev = &inner.event.as_ref().unwrap();

        mod_pr_debug!(
            "WorkQueue({:?}): New job at value {:#x?} slot {}\n",
            inner.pipe_type,
            ev.1,
            ev.0.slot()
        );
        Ok(Job::ver {
            wq: self.clone(),
            event_info: QueueEventInfo::ver {
                stamp_pointer: ev.0.stamp_pointer(),
                fw_stamp_pointer: ev.0.fw_stamp_pointer(),
                slot: ev.0.slot(),
                value: ev.1,
                cmd_seq: inner.commit_seq,
                event_seq: inner.event_seq,
                info_ptr: self.info_pointer,
            },
            start_value: ev.1,
            pending: KVec::new(),
            event_count: 0,
            committed: false,
            submitted: false,
            fence,
        })
    }

    pub(crate) fn pipe_type(&self) -> PipeType {
        self.inner.lock().pipe_type
    }

    pub(crate) fn dump_info(&self) {
        pr_info!("WorkQueue @ {:?}:", self.info_pointer);
        self.inner.lock().info.with(|raw, _inner| {
            pr_info!("  GPU rptr1: {:#x}", raw.gpu_rptr1.load(Ordering::Relaxed));
            pr_info!("  GPU rptr1: {:#x}", raw.gpu_rptr2.load(Ordering::Relaxed));
            pr_info!("  GPU rptr1: {:#x}", raw.gpu_rptr3.load(Ordering::Relaxed));
            pr_info!("  Event ID: {:#x}", raw.event_id.load(Ordering::Relaxed));
            pr_info!("  Busy: {:#x}", raw.busy.load(Ordering::Relaxed));
            pr_info!("  Unk 84: {:#x}", raw.unk_84_state.load(Ordering::Relaxed));
            pr_info!(
                "  Error count: {:#x}",
                raw.error_count.load(Ordering::Relaxed)
            );
            pr_info!("  Pending: {:#x}", raw.pending.load(Ordering::Relaxed));
        });
    }

    pub(crate) fn info_pointer(&self) -> GpuWeakPointer<QueueInfo::ver> {
        self.info_pointer
    }
}

/// Trait used to erase the version-specific type of WorkQueues, to avoid leaking
/// version-specificity into the event module.
pub(crate) trait WorkQueue {
    /// Cast as an Any type.
    fn as_any(&self) -> &dyn Any;

    fn signal(&self) -> bool;
    fn mark_error(&self, value: event::EventValue, error: WorkError);
    fn fail_all(&self, error: WorkError);
}

#[versions(AGX)]
impl WorkQueue for WorkQueue::ver {
    fn as_any(&self) -> &dyn Any {
        self
    }

    /// Signal a workqueue that some work was completed.
    ///
    /// This will check the event stamp value to find out exactly how many commands were processed.
    fn signal(&self) -> bool {
        let mut inner = self.inner.lock();
        let event = inner.event.as_ref();
        let value = match event {
            None => {
                mod_pr_debug!("WorkQueue: signal() called but no event?\n");

                if inner.pending_jobs > 0 || !inner.pending.is_empty() {
                    pr_crit!("WorkQueue: signal() called with no event and pending jobs.\n");
                }
                return true;
            }
            Some(event) => event.0.current(),
        };

        if let Some(lc) = inner.last_completed {
            if value < lc {
                pr_err!(
                    "WorkQueue: event rolled back? cur {:#x?}, lc {:#x?}, ls {:#x?}",
                    value,
                    inner.last_completed,
                    inner.last_submitted
                );
            }
        } else {
            pr_crit!("WorkQueue: signal() called with no last_completed.\n");
        }
        inner.last_completed = Some(value);

        mod_pr_debug!(
            "WorkQueue({:?}): Signaling event {:?} value {:#x?}\n",
            inner.pipe_type,
            inner.last_token,
            value
        );

        let mut completed_commands: usize = 0;

        for cmd in inner.pending.iter() {
            if cmd.value() <= value {
                mod_pr_debug!(
                    "WorkQueue({:?}): Command at value {:#x?} complete\n",
                    inner.pipe_type,
                    cmd.value()
                );
                completed_commands += 1;
            } else {
                break;
            }
        }

        if completed_commands == 0 {
            return inner.pending.is_empty();
        }

        let mut completed = Vec::new();

        if completed.reserve(completed_commands, GFP_KERNEL).is_err() {
            pr_crit!(
                "WorkQueue({:?}): Failed to allocate space for {} completed commands\n",
                inner.pipe_type,
                completed_commands
            );
        }

        let pipe_type = inner.pipe_type;

        for cmd in inner.pending.drain(..completed_commands) {
            if completed.push(cmd, GFP_KERNEL).is_err() {
                pr_crit!(
                    "WorkQueue({:?}): Failed to signal a completed command\n",
                    pipe_type,
                );
            }
        }

        mod_pr_debug!(
            "WorkQueue({:?}): Completed {} commands, left pending {}, ls {:#x?}, lc {:#x?}\n",
            inner.pipe_type,
            completed_commands,
            inner.pending.len(),
            inner.last_submitted,
            inner.last_completed,
        );

        if let Some(i) = completed.last() {
            inner
                .info
                .state
                .with(|raw, _inner| raw.cpu_freeptr.store(i.wptr(), Ordering::Release));
        }

        let empty = inner.pending.is_empty();
        if empty && inner.pending_jobs == 0 {
            inner.event = None;
            inner.last_submitted = None;
            inner.last_completed = None;
        }

        let dev = inner.dev.clone();
        core::mem::drop(inner);

        for cmd in completed.iter_mut() {
            cmd.complete();
        }

        let gpu = &dev.data().gpu;
        gpu.add_completed_work(completed);

        empty
    }

    /// Mark this queue's work up to a certain stamp value as having failed.
    fn mark_error(&self, value: event::EventValue, error: WorkError) {
        // If anything is marked completed, we can consider it successful
        // at this point, even if we didn't get the signal event yet.
        self.signal();

        let mut inner = self.inner.lock();

        if inner.event.is_none() {
            mod_pr_debug!("WorkQueue: signal_fault() called but no event?\n");

            if inner.pending_jobs > 0 || !inner.pending.is_empty() {
                pr_crit!("WorkQueue: signal_fault() called with no event and pending jobs.\n");
            }
            return;
        }

        mod_pr_debug!(
            "WorkQueue({:?}): Signaling fault for event {:?} at value {:#x?}\n",
            inner.pipe_type,
            inner.last_token,
            value
        );

        for cmd in inner.pending.iter_mut() {
            if cmd.value() <= value {
                cmd.mark_error(error);
            } else {
                break;
            }
        }
    }

    /// Mark all of this queue's work as having failed, and complete it.
    fn fail_all(&self, error: WorkError) {
        // If anything is marked completed, we can consider it successful
        // at this point, even if we didn't get the signal event yet.
        self.signal();

        let mut inner = self.inner.lock();

        if inner.event.is_none() {
            mod_pr_debug!("WorkQueue: fail_all() called but no event?\n");

            if inner.pending_jobs > 0 || !inner.pending.is_empty() {
                pr_crit!("WorkQueue: fail_all() called with no event and pending jobs.\n");
            }
            return;
        }

        mod_pr_debug!(
            "WorkQueue({:?}): Failing all jobs {:?}\n",
            inner.pipe_type,
            error
        );

        let mut cmds = KVec::new();

        core::mem::swap(&mut inner.pending, &mut cmds);

        if inner.pending_jobs == 0 {
            inner.event = None;
        }

        core::mem::drop(inner);

        for mut cmd in cmds {
            cmd.mark_error(error);
            cmd.complete();
        }
    }
}
