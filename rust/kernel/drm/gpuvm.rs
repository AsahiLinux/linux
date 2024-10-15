// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM Sync Objects
//!
//! C header: [`include/drm/drm_gpuvm.h`](../../../../include/drm/drm_gpuvm.h)

#![allow(missing_docs)]

use crate::{
    alloc::flags::*,
    bindings,
    drm::{device, drv},
    error::{
        code::{EINVAL, ENOMEM},
        from_result, to_result, Result,
    },
    init,
    prelude::*,
    types::{ARef, AlwaysRefCounted, Opaque},
};

use crate::drm::gem::IntoGEMObject;
use core::cell::UnsafeCell;
use core::marker::{PhantomData, PhantomPinned};
use core::mem::ManuallyDrop;
use core::ops::{Deref, DerefMut, Range};
use core::ptr::NonNull;

/// Trait that must be implemented by DRM drivers to represent a DRM GpuVm (a GPU address space).
pub trait DriverGpuVm: Sized {
    /// The parent `Driver` implementation for this `DriverGpuVm`.
    type Driver: drv::Driver;
    type GpuVa: DriverGpuVa = ();
    type GpuVmBo: DriverGpuVmBo = ();
    type StepContext = ();

    fn step_map(
        self: &mut UpdatingGpuVm<'_, Self>,
        op: &mut OpMap<Self>,
        ctx: &mut Self::StepContext,
    ) -> Result;
    fn step_unmap(
        self: &mut UpdatingGpuVm<'_, Self>,
        op: &mut OpUnMap<Self>,
        ctx: &mut Self::StepContext,
    ) -> Result;
    fn step_remap(
        self: &mut UpdatingGpuVm<'_, Self>,
        op: &mut OpReMap<Self>,
        ctx: &mut Self::StepContext,
    ) -> Result;
}

struct StepContext<'a, T: DriverGpuVm> {
    gpuvm: &'a GpuVm<T>,
    ctx: &'a mut T::StepContext,
}

/// Trait that must be implemented by DRM drivers to represent a DRM GpuVa (a mapping in GPU address space).
pub trait DriverGpuVa: Sized {}

impl DriverGpuVa for () {}

/// Trait that must be implemented by DRM drivers to represent a DRM GpuVmBo (a connection between a BO and a VM).
pub trait DriverGpuVmBo: Sized {
    fn new() -> impl PinInit<Self>;
}

/// Provide a default implementation for trivial types
impl<T: Default> DriverGpuVmBo for T {
    fn new() -> impl PinInit<Self> {
        init::default()
    }
}

#[repr(transparent)]
pub struct OpMap<T: DriverGpuVm>(bindings::drm_gpuva_op_map, PhantomData<T>);
#[repr(transparent)]
pub struct OpUnMap<T: DriverGpuVm>(bindings::drm_gpuva_op_unmap, PhantomData<T>);
#[repr(transparent)]
pub struct OpReMap<T: DriverGpuVm>(bindings::drm_gpuva_op_remap, PhantomData<T>);

impl<T: DriverGpuVm> OpMap<T> {
    pub fn addr(&self) -> u64 {
        self.0.va.addr
    }
    pub fn range(&self) -> u64 {
        self.0.va.range
    }
    pub fn offset(&self) -> u64 {
        self.0.gem.offset
    }
    pub fn object(&self) -> &<T::Driver as drv::Driver>::Object {
        // SAFETY: The GEM object is only ever passed as a Driver object below, so
        // the type must be correct.
        let p = unsafe {
            <<T::Driver as drv::Driver>::Object as IntoGEMObject>::from_gem_obj(self.0.gem.obj)
        };
        // SAFETY: The GEM object has an active reference for the lifetime of this op
        unsafe { &*p }
    }
    pub fn map_and_link_va(
        &mut self,
        gpuvm: &mut UpdatingGpuVm<'_, T>,
        gpuva: Pin<Box<GpuVa<T>>>,
        gpuvmbo: &ARef<GpuVmBo<T>>,
    ) -> Result<(), Pin<Box<GpuVa<T>>>> {
        // SAFETY: We are handing off the GpuVa ownership and it will not be moved.
        let p = Box::leak(unsafe { Pin::into_inner_unchecked(gpuva) });
        // SAFETY: These C functions are called with the correct invariants
        unsafe {
            bindings::drm_gpuva_init_from_op(&mut p.gpuva, &mut self.0);
            if bindings::drm_gpuva_insert(gpuvm.0.gpuvm() as *mut _, &mut p.gpuva) != 0 {
                // EEXIST, return the GpuVa to the caller as an error
                return Err(Pin::new_unchecked(Box::from_raw(p)));
            };
            // SAFETY: This takes a new reference to the gpuvmbo.
            bindings::drm_gpuva_link(&mut p.gpuva, &gpuvmbo.bo as *const _ as *mut _);
        }
        Ok(())
    }
}

impl<T: DriverGpuVm> OpUnMap<T> {
    pub fn va(&self) -> Option<&GpuVa<T>> {
        if self.0.va.is_null() {
            return None;
        }
        // SAFETY: Container invariant is guaranteed for ops structs created for our types.
        let p = unsafe { crate::container_of!(self.0.va, GpuVa<T>, gpuva) as *mut GpuVa<T> };
        // SAFETY: The GpuVa object reference is valid per the op_unmap contract
        Some(unsafe { &*p })
    }
    pub fn unmap_and_unlink_va(&mut self) -> Option<Pin<Box<GpuVa<T>>>> {
        if self.0.va.is_null() {
            return None;
        }
        // SAFETY: Container invariant is guaranteed for ops structs created for our types.
        let p = unsafe { crate::container_of!(self.0.va, GpuVa<T>, gpuva) as *mut GpuVa<T> };

        // SAFETY: The GpuVa object reference is valid per the op_unmap contract
        unsafe {
            bindings::drm_gpuva_unmap(&mut self.0);
            bindings::drm_gpuva_unlink(self.0.va);
        }

        // Unlinking/unmapping relinquishes ownership of the GpuVa object,
        // so clear the pointer
        self.0.va = core::ptr::null_mut();
        // SAFETY: The GpuVa object reference is valid per the op_unmap contract
        Some(unsafe { Pin::new_unchecked(Box::from_raw(p)) })
    }
}

impl<T: DriverGpuVm> OpReMap<T> {
    pub fn prev_map(&mut self) -> Option<&mut OpMap<T>> {
        // SAFETY: The prev pointer must be valid if not-NULL per the op_remap contract
        unsafe { (self.0.prev as *mut OpMap<T>).as_mut() }
    }
    pub fn next_map(&mut self) -> Option<&mut OpMap<T>> {
        // SAFETY: The next pointer must be valid if not-NULL per the op_remap contract
        unsafe { (self.0.next as *mut OpMap<T>).as_mut() }
    }
    pub fn unmap(&mut self) -> &mut OpUnMap<T> {
        // SAFETY: The unmap pointer is always valid per the op_remap contract
        unsafe { (self.0.unmap as *mut OpUnMap<T>).as_mut().unwrap() }
    }
}

/// A base GPU VA.
#[repr(C)]
#[pin_data]
pub struct GpuVa<T: DriverGpuVm> {
    #[pin]
    gpuva: bindings::drm_gpuva,
    #[pin]
    inner: T::GpuVa,
    #[pin]
    _p: PhantomPinned,
}

// SAFETY: This type is safe to zero-init (as far as C is concerned).
unsafe impl init::Zeroable for bindings::drm_gpuva {}

impl<T: DriverGpuVm> GpuVa<T> {
    pub fn new<E>(inner: impl PinInit<T::GpuVa, E>) -> Result<Pin<Box<GpuVa<T>>>>
    where
        Error: From<E>,
    {
        Box::try_pin_init(
            try_pin_init!(Self {
                gpuva <- init::zeroed(),
                inner <- inner,
                _p: PhantomPinned
            }),
            GFP_KERNEL,
        )
    }

    pub fn addr(&self) -> u64 {
        self.gpuva.va.addr
    }
    pub fn range(&self) -> u64 {
        self.gpuva.va.range
    }
    pub fn vm_bo(&self) -> ARef<GpuVmBo<T>> {
        // SAFETY: Container invariant is guaranteed for ops structs created for our types.
        let p =
            unsafe { crate::container_of!(self.gpuva.vm_bo, GpuVmBo<T>, bo) as *mut GpuVmBo<T> };

        // SAFETY: We incref and wrap in an ARef, so the reference count is consistent
        unsafe {
            bindings::drm_gpuvm_bo_get(self.gpuva.vm_bo);
            ARef::from_raw(NonNull::new_unchecked(p))
        }
    }
    pub fn offset(&self) -> u64 {
        self.gpuva.gem.offset
    }
}

/// A base GpuVm BO.
#[repr(C)]
#[pin_data]
pub struct GpuVmBo<T: DriverGpuVm> {
    #[pin]
    bo: bindings::drm_gpuvm_bo,
    #[pin]
    inner: T::GpuVmBo,
    #[pin]
    _p: PhantomPinned,
}

impl<T: DriverGpuVm> GpuVmBo<T> {
    /// Return a reference to the inner driver data for this GpuVmBo
    pub fn inner(&self) -> &T::GpuVmBo {
        &self.inner
    }
}

// SAFETY: DRM GpuVmBo objects are always reference counted and the get/put functions
// satisfy the requirements.
unsafe impl<T: DriverGpuVm> AlwaysRefCounted for GpuVmBo<T> {
    fn inc_ref(&self) {
        // SAFETY: The drm_gpuvm_get function satisfies the requirements for inc_ref().
        unsafe { bindings::drm_gpuvm_bo_get(&self.bo as *const _ as *mut _) };
    }

    unsafe fn dec_ref(mut obj: NonNull<Self>) {
        // SAFETY: drm_gpuvm_bo_put() requires holding the gpuva lock, which is the dma_resv lock by default.
        // The drm_gpuvm_put function satisfies the requirements for dec_ref().
        // (We do not support custom locks yet.)
        unsafe {
            let resv = (*obj.as_mut().bo.obj).resv;
            bindings::dma_resv_lock(resv, core::ptr::null_mut());
            bindings::drm_gpuvm_bo_put(&mut obj.as_mut().bo);
            bindings::dma_resv_unlock(resv);
        }
    }
}

/// A base GPU VM.
#[repr(C)]
#[pin_data]
pub struct GpuVm<T: DriverGpuVm> {
    #[pin]
    gpuvm: Opaque<bindings::drm_gpuvm>,
    #[pin]
    inner: UnsafeCell<T>,
    #[pin]
    _p: PhantomPinned,
}

pub(super) unsafe extern "C" fn vm_free_callback<T: DriverGpuVm>(
    raw_gpuvm: *mut bindings::drm_gpuvm,
) {
    // SAFETY: Container invariant is guaranteed for objects using our callback.
    let p = unsafe {
        crate::container_of!(
            raw_gpuvm as *mut Opaque<bindings::drm_gpuvm>,
            GpuVm<T>,
            gpuvm
        ) as *mut GpuVm<T>
    };

    // SAFETY: p is guaranteed to be valid for drm_gpuvm objects using this callback.
    unsafe { drop(Box::from_raw(p)) };
}

pub(super) unsafe extern "C" fn vm_bo_alloc_callback<T: DriverGpuVm>() -> *mut bindings::drm_gpuvm_bo
{
    let obj: Result<Pin<Box<GpuVmBo<T>>>> = Box::try_pin_init(
        try_pin_init!(GpuVmBo::<T> {
            bo <- init::default(),
            inner <- T::GpuVmBo::new(),
            _p: PhantomPinned
        }),
        GFP_KERNEL,
    );

    match obj {
        Ok(obj) =>
        // SAFETY: The DRM core will keep this object pinned
        unsafe {
            let p = Box::leak(Pin::into_inner_unchecked(obj));
            &mut p.bo
        },
        Err(_) => core::ptr::null_mut(),
    }
}

pub(super) unsafe extern "C" fn vm_bo_free_callback<T: DriverGpuVm>(
    raw_vm_bo: *mut bindings::drm_gpuvm_bo,
) {
    // SAFETY: Container invariant is guaranteed for objects using this callback.
    let p = unsafe { crate::container_of!(raw_vm_bo, GpuVmBo<T>, bo) as *mut GpuVmBo<T> };

    // SAFETY: p is guaranteed to be valid for drm_gpuvm_bo objects using this callback.
    unsafe { drop(Box::from_raw(p)) };
}

pub(super) unsafe extern "C" fn step_map_callback<T: DriverGpuVm>(
    op: *mut bindings::drm_gpuva_op,
    _priv: *mut core::ffi::c_void,
) -> core::ffi::c_int {
    // SAFETY: We know this is a map op, and OpMap is a transparent wrapper.
    let map = unsafe { &mut *((&mut (*op).__bindgen_anon_1.map) as *mut _ as *mut OpMap<T>) };
    // SAFETY: This is a pointer to a StepContext created inline in sm_map(), which is
    // guaranteed to outlive this function.
    let ctx = unsafe { &mut *(_priv as *mut StepContext<'_, T>) };

    from_result(|| {
        UpdatingGpuVm(ctx.gpuvm).step_map(map, ctx.ctx)?;
        Ok(0)
    })
}

pub(super) unsafe extern "C" fn step_remap_callback<T: DriverGpuVm>(
    op: *mut bindings::drm_gpuva_op,
    _priv: *mut core::ffi::c_void,
) -> core::ffi::c_int {
    // SAFETY: We know this is a map op, and OpReMap is a transparent wrapper.
    let remap = unsafe { &mut *((&mut (*op).__bindgen_anon_1.remap) as *mut _ as *mut OpReMap<T>) };
    // SAFETY: This is a pointer to a StepContext created inline in sm_map(), which is
    // guaranteed to outlive this function.
    let ctx = unsafe { &mut *(_priv as *mut StepContext<'_, T>) };

    from_result(|| {
        UpdatingGpuVm(ctx.gpuvm).step_remap(remap, ctx.ctx)?;
        Ok(0)
    })
}
pub(super) unsafe extern "C" fn step_unmap_callback<T: DriverGpuVm>(
    op: *mut bindings::drm_gpuva_op,
    _priv: *mut core::ffi::c_void,
) -> core::ffi::c_int {
    // SAFETY: We know this is a map op, and OpUnMap is a transparent wrapper.
    let unmap = unsafe { &mut *((&mut (*op).__bindgen_anon_1.unmap) as *mut _ as *mut OpUnMap<T>) };
    // SAFETY: This is a pointer to a StepContext created inline in sm_map(), which is
    // guaranteed to outlive this function.
    let ctx = unsafe { &mut *(_priv as *mut StepContext<'_, T>) };

    from_result(|| {
        UpdatingGpuVm(ctx.gpuvm).step_unmap(unmap, ctx.ctx)?;
        Ok(0)
    })
}

pub(super) unsafe extern "C" fn exec_lock_gem_object(
    vm_exec: *mut bindings::drm_gpuvm_exec,
) -> core::ffi::c_int {
    // SAFETY: The gpuvm_exec object is valid and priv_ is a GEM object pointer
    // when this callback is used
    unsafe { bindings::drm_exec_lock_obj(&mut (*vm_exec).exec, (*vm_exec).extra.priv_ as *mut _) }
}

impl<T: DriverGpuVm> GpuVm<T> {
    const OPS: bindings::drm_gpuvm_ops = bindings::drm_gpuvm_ops {
        vm_free: Some(vm_free_callback::<T>),
        op_alloc: None,
        op_free: None,
        vm_bo_alloc: Some(vm_bo_alloc_callback::<T>),
        vm_bo_free: Some(vm_bo_free_callback::<T>),
        vm_bo_validate: None,
        sm_step_map: Some(step_map_callback::<T>),
        sm_step_remap: Some(step_remap_callback::<T>),
        sm_step_unmap: Some(step_unmap_callback::<T>),
    };

    fn gpuvm(&self) -> *const bindings::drm_gpuvm {
        self.gpuvm.get()
    }

    pub fn new<E>(
        name: &'static CStr,
        dev: &device::Device<T::Driver>,
        r_obj: &<T::Driver as drv::Driver>::Object,
        range: Range<u64>,
        reserve_range: Range<u64>,
        inner: impl PinInit<T, E>,
    ) -> Result<ARef<GpuVm<T>>>
    where
        Error: From<E>,
    {
        let obj: Pin<Box<Self>> = Box::try_pin_init(
            try_pin_init!(Self {
                // SAFETY: drm_gpuvm_init cannot fail and always initializes the member
                gpuvm <- unsafe {
                    init::pin_init_from_closure(move |slot: *mut Opaque<bindings::drm_gpuvm> | {
                        // Zero-init required by drm_gpuvm_init
                        *slot = Opaque::zeroed();
                        bindings::drm_gpuvm_init(
                            Opaque::raw_get(slot),
                            name.as_char_ptr(),
                            0,
                            dev.raw_mut(),
                            r_obj.gem_obj() as *const _ as *mut _,
                            range.start,
                            range.end - range.start,
                            reserve_range.start,
                            reserve_range.end - reserve_range.start,
                            &Self::OPS
                        );
                        Ok(())
                    })
                },
                // SAFETY: Just passing through to the initializer argument
                inner <- unsafe {
                    init::pin_init_from_closure(move |slot: *mut UnsafeCell<T> | {
                        inner.__pinned_init(slot as *mut _)
                    })
                },
                _p: PhantomPinned
            }),
            GFP_KERNEL,
        )?;

        // SAFETY: We never move out of the object
        let vm_ref = unsafe {
            ARef::from_raw(NonNull::new_unchecked(Box::leak(
                Pin::into_inner_unchecked(obj),
            )))
        };

        Ok(vm_ref)
    }

    pub fn exec_lock<'a, 'b>(
        &'a self,
        obj: Option<&'b <T::Driver as drv::Driver>::Object>,
    ) -> Result<LockedGpuVm<'a, 'b, T>> {
        // Do not try to lock the object if it is internal (since it is already locked).
        let is_ext = obj.map(|a| self.is_extobj(a)).unwrap_or(false);

        let mut guard = ManuallyDrop::new(LockedGpuVm {
            gpuvm: self,
            // vm_exec needs to be pinned, so stick it in a Box.
            vm_exec: Box::init(
                init!(bindings::drm_gpuvm_exec {
                    vm: self.gpuvm() as *mut _,
                    flags: bindings::BINDINGS_DRM_EXEC_INTERRUPTIBLE_WAIT,
                    exec: Default::default(),
                    extra: match (is_ext, obj) {
                        (true, Some(obj)) => bindings::drm_gpuvm_exec__bindgen_ty_1 {
                            fn_: Some(exec_lock_gem_object),
                            priv_: obj.gem_obj() as *const _ as *mut _,
                        },
                        _ => Default::default(),
                    },
                    num_fences: 0,
                }),
                GFP_KERNEL,
            )?,
            obj,
        });

        // SAFETY: The object is valid and was initialized above
        to_result(unsafe { bindings::drm_gpuvm_exec_lock(&mut *guard.vm_exec) })?;

        Ok(ManuallyDrop::into_inner(guard))
    }

    /// Returns true if the given object is external to the GPUVM
    /// (that is, if it does not share the DMA reservation object of the GPUVM).
    pub fn is_extobj(&self, obj: &impl IntoGEMObject) -> bool {
        let gem = obj.gem_obj() as *const _ as *mut _;
        // SAFETY: This is safe to call as long as the arguments are valid pointers.
        unsafe { bindings::drm_gpuvm_is_extobj(self.gpuvm() as *mut _, gem) }
    }
}

// SAFETY: DRM GpuVm objects are always reference counted and the get/put functions
// satisfy the requirements.
unsafe impl<T: DriverGpuVm> AlwaysRefCounted for GpuVm<T> {
    fn inc_ref(&self) {
        // SAFETY: The drm_gpuvm_get function satisfies the requirements for inc_ref().
        unsafe { bindings::drm_gpuvm_get(&self.gpuvm as *const _ as *mut _) };
    }

    unsafe fn dec_ref(obj: NonNull<Self>) {
        // SAFETY: The drm_gpuvm_put function satisfies the requirements for dec_ref().
        unsafe { bindings::drm_gpuvm_put(Opaque::raw_get(&(*obj.as_ptr()).gpuvm)) };
    }
}

pub struct LockedGpuVm<'a, 'b, T: DriverGpuVm> {
    gpuvm: &'a GpuVm<T>,
    vm_exec: Box<bindings::drm_gpuvm_exec>,
    obj: Option<&'b <T::Driver as drv::Driver>::Object>,
}

impl<T: DriverGpuVm> LockedGpuVm<'_, '_, T> {
    pub fn find_bo(&mut self) -> Option<ARef<GpuVmBo<T>>> {
        let obj = self.obj?;
        // SAFETY: LockedGpuVm implies the right locks are held.
        let p = unsafe {
            bindings::drm_gpuvm_bo_find(
                self.gpuvm.gpuvm() as *mut _,
                obj.gem_obj() as *const _ as *mut _,
            )
        };
        if p.is_null() {
            None
        } else {
            // SAFETY: All the drm_gpuvm_bo objects in this GpuVm are always allocated by us as GpuVmBo<T>.
            let p = unsafe { crate::container_of!(p, GpuVmBo<T>, bo) as *mut GpuVmBo<T> };
            // SAFETY: We checked for NULL above, and the types ensure that
            // this object was created by vm_bo_alloc_callback<T>.
            Some(unsafe { ARef::from_raw(NonNull::new_unchecked(p)) })
        }
    }

    pub fn obtain_bo(&mut self) -> Result<ARef<GpuVmBo<T>>> {
        let obj = self.obj.ok_or(EINVAL)?;
        // SAFETY: LockedGpuVm implies the right locks are held.
        let p = unsafe {
            bindings::drm_gpuvm_bo_obtain(
                self.gpuvm.gpuvm() as *mut _,
                obj.gem_obj() as *const _ as *mut _,
            )
        };
        if p.is_null() {
            Err(ENOMEM)
        } else {
            // SAFETY: Container invariant is guaranteed for GpuVmBo objects for this GpuVm.
            let p = unsafe { crate::container_of!(p, GpuVmBo<T>, bo) as *mut GpuVmBo<T> };
            // SAFETY: We checked for NULL above, and the types ensure that
            // this object was created by vm_bo_alloc_callback<T>.
            Ok(unsafe { ARef::from_raw(NonNull::new_unchecked(p)) })
        }
    }

    pub fn sm_map(
        &mut self,
        ctx: &mut T::StepContext,
        req_addr: u64,
        req_range: u64,
        req_offset: u64,
    ) -> Result {
        let obj = self.obj.ok_or(EINVAL)?;
        let mut ctx = StepContext {
            ctx,
            gpuvm: self.gpuvm,
        };
        // SAFETY: LockedGpuVm implies the right locks are held.
        to_result(unsafe {
            bindings::drm_gpuvm_sm_map(
                self.gpuvm.gpuvm() as *mut _,
                &mut ctx as *mut _ as *mut _,
                req_addr,
                req_range,
                obj.gem_obj() as *const _ as *mut _,
                req_offset,
            )
        })
    }

    pub fn sm_unmap(&mut self, ctx: &mut T::StepContext, req_addr: u64, req_range: u64) -> Result {
        let mut ctx = StepContext {
            ctx,
            gpuvm: self.gpuvm,
        };
        // SAFETY: LockedGpuVm implies the right locks are held.
        to_result(unsafe {
            bindings::drm_gpuvm_sm_unmap(
                self.gpuvm.gpuvm() as *mut _,
                &mut ctx as *mut _ as *mut _,
                req_addr,
                req_range,
            )
        })
    }

    pub fn bo_unmap(&mut self, ctx: &mut T::StepContext, bo: &ARef<GpuVmBo<T>>) -> Result {
        let mut ctx = StepContext {
            ctx,
            gpuvm: self.gpuvm,
        };
        // SAFETY: LockedGpuVm implies the right locks are held.
        to_result(unsafe {
            bindings::drm_gpuvm_bo_unmap(&bo.bo as *const _ as *mut _, &mut ctx as *mut _ as *mut _)
        })
    }
}

impl<T: DriverGpuVm> Deref for LockedGpuVm<'_, '_, T> {
    type Target = T;

    fn deref(&self) -> &T {
        // SAFETY: The existence of this LockedGpuVm implies the lock is held,
        // so this is the only reference
        unsafe { &*self.gpuvm.inner.get() }
    }
}

impl<T: DriverGpuVm> DerefMut for LockedGpuVm<'_, '_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: The existence of this UpdatingGpuVm implies the lock is held,
        // so this is the only reference
        unsafe { &mut *self.gpuvm.inner.get() }
    }
}

impl<T: DriverGpuVm> Drop for LockedGpuVm<'_, '_, T> {
    fn drop(&mut self) {
        // SAFETY: We hold the lock, so it's safe to unlock
        unsafe {
            bindings::drm_gpuvm_exec_unlock(&mut *self.vm_exec);
        }
    }
}

pub struct UpdatingGpuVm<'a, T: DriverGpuVm>(&'a GpuVm<T>);

impl<T: DriverGpuVm> UpdatingGpuVm<'_, T> {}

impl<T: DriverGpuVm> Deref for UpdatingGpuVm<'_, T> {
    type Target = T;

    fn deref(&self) -> &T {
        // SAFETY: The existence of this UpdatingGpuVm implies the lock is held,
        // so this is the only reference
        unsafe { &*self.0.inner.get() }
    }
}

impl<T: DriverGpuVm> DerefMut for UpdatingGpuVm<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: The existence of this UpdatingGpuVm implies the lock is held,
        // so this is the only reference
        unsafe { &mut *self.0.inner.get() }
    }
}

impl<T: DriverGpuVm> core::ops::Receiver for UpdatingGpuVm<'_, T> {}

// SAFETY: All our trait methods take locks
unsafe impl<T: DriverGpuVm> Sync for GpuVm<T> {}
// SAFETY: All our trait methods take locks
unsafe impl<T: DriverGpuVm> Send for GpuVm<T> {}

// SAFETY: All our trait methods take locks
unsafe impl<T: DriverGpuVm> Sync for GpuVmBo<T> {}
// SAFETY: All our trait methods take locks
unsafe impl<T: DriverGpuVm> Send for GpuVmBo<T> {}
