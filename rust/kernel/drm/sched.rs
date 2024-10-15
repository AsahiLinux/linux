// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM Scheduler
//!
//! C header: [`include/drm/gpu_scheduler.h`](../../../../include/drm/gpu_scheduler.h)

use crate::{
    alloc::{box_ext::BoxExt, flags::*},
    bindings, device,
    dma_fence::*,
    error::{to_result, Result},
    prelude::*,
    sync::{Arc, UniqueArc},
};
use alloc::boxed::Box;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::ops::{Deref, DerefMut};
use core::ptr::addr_of_mut;

/// Scheduler status after timeout recovery
#[repr(u32)]
pub enum Status {
    /// Device recovered from the timeout and can execute jobs again
    Nominal = bindings::drm_gpu_sched_stat_DRM_GPU_SCHED_STAT_NOMINAL,
    /// Device is no longer available
    NoDevice = bindings::drm_gpu_sched_stat_DRM_GPU_SCHED_STAT_ENODEV,
}

/// Scheduler priorities
#[repr(u32)]
pub enum Priority {
    /// Low userspace priority
    Low = bindings::drm_sched_priority_DRM_SCHED_PRIORITY_LOW,
    /// Normal userspace priority
    Normal = bindings::drm_sched_priority_DRM_SCHED_PRIORITY_NORMAL,
    /// High userspace priority
    High = bindings::drm_sched_priority_DRM_SCHED_PRIORITY_HIGH,
    /// Kernel priority (highest)
    Kernel = bindings::drm_sched_priority_DRM_SCHED_PRIORITY_KERNEL,
}

/// Trait to be implemented by driver job objects.
pub trait JobImpl: Sized {
    /// Called when the scheduler is considering scheduling this job next, to get another Fence
    /// for this job to block on. Once it returns None, run() may be called.
    fn prepare(_job: &mut Job<Self>) -> Option<Fence> {
        None // Equivalent to NULL function pointer
    }

    /// Called to execute the job once all of the dependencies have been resolved. This may be
    /// called multiple times, if timed_out() has happened and drm_sched_job_recovery() decides
    /// to try it again.
    fn run(job: &mut Job<Self>) -> Result<Option<Fence>>;

    /// Called when a job has taken too long to execute, to trigger GPU recovery.
    ///
    /// This method is called in a workqueue context.
    fn timed_out(job: &mut Job<Self>) -> Status;
}

unsafe extern "C" fn prepare_job_cb<T: JobImpl>(
    sched_job: *mut bindings::drm_sched_job,
    _s_entity: *mut bindings::drm_sched_entity,
) -> *mut bindings::dma_fence {
    // SAFETY: All of our jobs are Job<T>.
    let p = unsafe { crate::container_of!(sched_job, Job<T>, job) as *mut Job<T> };

    // SAFETY: All of our jobs are Job<T>.
    match T::prepare(unsafe { &mut *p }) {
        None => core::ptr::null_mut(),
        Some(fence) => fence.into_raw(),
    }
}

unsafe extern "C" fn run_job_cb<T: JobImpl>(
    sched_job: *mut bindings::drm_sched_job,
) -> *mut bindings::dma_fence {
    // SAFETY: All of our jobs are Job<T>.
    let p = unsafe { crate::container_of!(sched_job, Job<T>, job) as *mut Job<T> };

    // SAFETY: All of our jobs are Job<T>.
    match T::run(unsafe { &mut *p }) {
        Err(e) => e.to_ptr(),
        Ok(None) => core::ptr::null_mut(),
        Ok(Some(fence)) => fence.into_raw(),
    }
}

unsafe extern "C" fn timedout_job_cb<T: JobImpl>(
    sched_job: *mut bindings::drm_sched_job,
) -> bindings::drm_gpu_sched_stat {
    // SAFETY: All of our jobs are Job<T>.
    let p = unsafe { crate::container_of!(sched_job, Job<T>, job) as *mut Job<T> };

    // SAFETY: All of our jobs are Job<T>.
    T::timed_out(unsafe { &mut *p }) as bindings::drm_gpu_sched_stat
}

unsafe extern "C" fn free_job_cb<T: JobImpl>(sched_job: *mut bindings::drm_sched_job) {
    // SAFETY: All of our jobs are Job<T>.
    let p = unsafe { crate::container_of!(sched_job, Job<T>, job) as *mut Job<T> };

    // Convert the job back to a Box and drop it
    // SAFETY: All of our Job<T>s are created inside a box.
    unsafe { drop(Box::from_raw(p)) };
}

/// A DRM scheduler job.
pub struct Job<T: JobImpl> {
    job: bindings::drm_sched_job,
    inner: T,
}

impl<T: JobImpl> Deref for Job<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T: JobImpl> DerefMut for Job<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<T: JobImpl> Drop for Job<T> {
    fn drop(&mut self) {
        // SAFETY: At this point the job has either been submitted and this is being called from
        // `free_job_cb` above, or it hasn't and it is safe to call `drm_sched_job_cleanup`.
        unsafe { bindings::drm_sched_job_cleanup(&mut self.job) };
    }
}

/// A pending DRM scheduler job (not yet armed)
pub struct PendingJob<'a, T: JobImpl>(Box<Job<T>>, PhantomData<&'a T>);

impl<'a, T: JobImpl> PendingJob<'a, T> {
    /// Add a fence as a dependency to the job
    pub fn add_dependency(&mut self, fence: Fence) -> Result {
        // SAFETY: C call with correct arguments
        to_result(unsafe {
            bindings::drm_sched_job_add_dependency(&mut self.0.job, fence.into_raw())
        })
    }

    /// Arm the job to make it ready for execution
    pub fn arm(mut self) -> ArmedJob<'a, T> {
        // SAFETY: C call with correct arguments
        unsafe { bindings::drm_sched_job_arm(&mut self.0.job) };
        ArmedJob(self.0, PhantomData)
    }
}

impl<'a, T: JobImpl> Deref for PendingJob<'a, T> {
    type Target = Job<T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<'a, T: JobImpl> DerefMut for PendingJob<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// An armed DRM scheduler job (not yet submitted)
pub struct ArmedJob<'a, T: JobImpl>(Box<Job<T>>, PhantomData<&'a T>);

impl<'a, T: JobImpl> ArmedJob<'a, T> {
    /// Returns the job fences
    pub fn fences(&self) -> JobFences<'_> {
        // SAFETY: s_fence is always a valid drm_sched_fence pointer
        JobFences(unsafe { &mut *self.0.job.s_fence })
    }

    /// Push the job for execution into the scheduler
    pub fn push(self) {
        // After this point, the job is submitted and owned by the scheduler
        let ptr = match self {
            ArmedJob(job, _) => Box::<Job<T>>::into_raw(job),
        };

        // SAFETY: We are passing in ownership of a valid Box raw pointer.
        unsafe { bindings::drm_sched_entity_push_job(addr_of_mut!((*ptr).job)) };
    }
}
impl<'a, T: JobImpl> Deref for ArmedJob<'a, T> {
    type Target = Job<T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<'a, T: JobImpl> DerefMut for ArmedJob<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Reference to the bundle of fences attached to a DRM scheduler job
pub struct JobFences<'a>(&'a mut bindings::drm_sched_fence);

impl<'a> JobFences<'a> {
    /// Returns a new reference to the job scheduled fence.
    pub fn scheduled(&mut self) -> Fence {
        // SAFETY: self.0.scheduled is always a valid fence
        unsafe { Fence::get_raw(&mut self.0.scheduled) }
    }

    /// Returns a new reference to the job finished fence.
    pub fn finished(&mut self) -> Fence {
        // SAFETY: self.0.finished is always a valid fence
        unsafe { Fence::get_raw(&mut self.0.finished) }
    }
}

struct EntityInner<T: JobImpl> {
    entity: bindings::drm_sched_entity,
    // TODO: Allow users to share guilty flag between entities
    sched: Arc<SchedulerInner<T>>,
    guilty: bindings::atomic_t,
    _p: PhantomData<T>,
}

impl<T: JobImpl> Drop for EntityInner<T> {
    fn drop(&mut self) {
        // SAFETY: The EntityInner is initialized. This will cancel/free all jobs.
        unsafe { bindings::drm_sched_entity_destroy(&mut self.entity) };
    }
}

// SAFETY: TODO
unsafe impl<T: JobImpl> Sync for EntityInner<T> {}
// SAFETY: TODO
unsafe impl<T: JobImpl> Send for EntityInner<T> {}

/// A DRM scheduler entity.
pub struct Entity<T: JobImpl>(Pin<Box<EntityInner<T>>>);

impl<T: JobImpl> Entity<T> {
    /// Create a new scheduler entity.
    pub fn new(sched: &Scheduler<T>, priority: Priority) -> Result<Self> {
        let mut entity: Box<MaybeUninit<EntityInner<T>>> =
            Box::new_uninit(GFP_KERNEL | __GFP_ZERO)?;

        let mut sched_ptr = &sched.0.sched as *const _ as *mut _;

        // SAFETY: The Box is allocated above and valid.
        unsafe {
            bindings::drm_sched_entity_init(
                addr_of_mut!((*entity.as_mut_ptr()).entity),
                priority as _,
                &mut sched_ptr,
                1,
                addr_of_mut!((*entity.as_mut_ptr()).guilty),
            )
        };

        // SAFETY: The Box is allocated above and valid.
        unsafe { addr_of_mut!((*entity.as_mut_ptr()).sched).write(sched.0.clone()) };

        // SAFETY: entity is now initialized.
        Ok(Self(Pin::from(unsafe { entity.assume_init() })))
    }

    /// Create a new job on this entity.
    ///
    /// The entity must outlive the pending job until it transitions into the submitted state,
    /// after which the scheduler owns it. Since jobs must be submitted in creation order,
    /// this requires a mutable reference to the entity, ensuring that only one new job can be
    /// in flight at once.
    pub fn new_job(&mut self, credits: u32, inner: T) -> Result<PendingJob<'_, T>> {
        let mut job: Box<MaybeUninit<Job<T>>> = Box::new_uninit(GFP_KERNEL | __GFP_ZERO)?;

        // SAFETY: We hold a reference to the entity (which is a valid pointer),
        // and the job object was just allocated above.
        to_result(unsafe {
            bindings::drm_sched_job_init(
                addr_of_mut!((*job.as_mut_ptr()).job),
                &self.0.as_ref().get_ref().entity as *const _ as *mut _,
                credits,
                core::ptr::null_mut(),
            )
        })?;

        // SAFETY: The Box pointer is valid, and this initializes the inner member.
        unsafe { addr_of_mut!((*job.as_mut_ptr()).inner).write(inner) };

        // SAFETY: All fields of the Job<T> are now initialized.
        Ok(PendingJob(unsafe { job.assume_init() }, PhantomData))
    }
}

/// DRM scheduler inner data
pub struct SchedulerInner<T: JobImpl> {
    sched: bindings::drm_gpu_scheduler,
    _p: PhantomData<T>,
}

impl<T: JobImpl> Drop for SchedulerInner<T> {
    fn drop(&mut self) {
        // SAFETY: The scheduler is valid. This assumes drm_sched_fini() will take care of
        // freeing all in-progress jobs.
        unsafe { bindings::drm_sched_fini(&mut self.sched) };
    }
}

// SAFETY: TODO
unsafe impl<T: JobImpl> Sync for SchedulerInner<T> {}
// SAFETY: TODO
unsafe impl<T: JobImpl> Send for SchedulerInner<T> {}

/// A DRM Scheduler
pub struct Scheduler<T: JobImpl>(Arc<SchedulerInner<T>>);

impl<T: JobImpl> Scheduler<T> {
    const OPS: bindings::drm_sched_backend_ops = bindings::drm_sched_backend_ops {
        prepare_job: Some(prepare_job_cb::<T>),
        run_job: Some(run_job_cb::<T>),
        timedout_job: Some(timedout_job_cb::<T>),
        free_job: Some(free_job_cb::<T>),
        update_job_credits: None,
    };
    /// Creates a new DRM Scheduler object
    // TODO: Shared timeout workqueues & scores
    pub fn new(
        device: &impl device::RawDevice,
        num_rqs: u32,
        credit_limit: u32,
        hang_limit: u32,
        timeout_ms: usize,
        name: &'static CStr,
    ) -> Result<Scheduler<T>> {
        let mut sched: UniqueArc<MaybeUninit<SchedulerInner<T>>> =
            UniqueArc::new_uninit(GFP_KERNEL)?;

        // SAFETY: zero sched->sched_rq as drm_sched_init() uses it to exit early withoput initialisation
        // TODO: allocate sched zzeroed instead
        unsafe {
            (*sched.as_mut_ptr()).sched.sched_rq = core::ptr::null_mut();
        };

        // SAFETY: The drm_sched pointer is valid and pinned as it was just allocated above.
        to_result(unsafe {
            bindings::drm_sched_init(
                addr_of_mut!((*sched.as_mut_ptr()).sched),
                &Self::OPS,
                core::ptr::null_mut(),
                num_rqs,
                credit_limit,
                hang_limit,
                bindings::msecs_to_jiffies(timeout_ms.try_into()?).try_into()?,
                core::ptr::null_mut(),
                core::ptr::null_mut(),
                name.as_char_ptr(),
                device.raw_device(),
            )
        })?;

        // SAFETY: All fields of SchedulerInner are now initialized.
        Ok(Scheduler(unsafe { sched.assume_init() }.into()))
    }
}
