// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM Sync Objects
//!
//! C header: [`include/drm/drm_gpuvm.h`](../../../../include/drm/drm_gpuvm.h)

#![allow(missing_docs)]

use crate::{
    bindings, drm,
    drm::device,
    error::{
        code::{EINVAL, ENOMEM},
        from_result, to_result, Error, Result,
    },
    prelude::*,
    types::{ARef, AlwaysRefCounted, Opaque},
};

use crate::drm::gem::BaseDriverObject;
use crate::drm::gem::IntoGEMObject;
use core::cell::UnsafeCell;
use core::marker::{PhantomData, PhantomPinned};
use core::mem::ManuallyDrop;
use core::ops::{Deref, DerefMut, Range};
use core::ptr::NonNull;
use pin_init;

/// GpuVaFlags to be used for a GpuVa.
///
/// They can be combined with the operators `|`, `&`, and `!`.
#[derive(Clone, Copy, PartialEq, Default)]
pub struct GpuVaFlags(u32);

impl GpuVaFlags {
    /// No GpuVaFlags (zero)
    pub const NONE: GpuVaFlags = GpuVaFlags(0);

    /// The backing GEM is invalidated.
    pub const INVALIDATED: GpuVaFlags = GpuVaFlags(bindings::drm_gpuva_flags_DRM_GPUVA_INVALIDATED);

    /// The GpuVa is a sparse mapping.
    pub const SPARSE: GpuVaFlags = GpuVaFlags(bindings::drm_gpuva_flags_DRM_GPUVA_SPARSE);

    /// The GpuVa is a sparse mapping.
    pub const SINGLE_PAGE: GpuVaFlags = GpuVaFlags(bindings::drm_gpuva_flags_DRM_GPUVA_SPARSE);

    /// Construct a driver-specific GpuVaFlag.
    ///
    /// The argument must be a flag index in the range [0..28].
    pub const fn user_flag(index: u32) -> GpuVaFlags {
        let flags = bindings::drm_gpuva_flags_DRM_GPUVA_USERBITS << index;
        assert!(flags != 0);
        GpuVaFlags(flags)
    }

    /// Get the raw representation of this flag.
    pub(crate) fn as_raw(self) -> u32 {
        self.0
    }

    /// Check whether `flags` is contained in `self`.
    pub fn contains(self, flags: GpuVaFlags) -> bool {
        (self & flags) == flags
    }
}

impl core::ops::BitOr for GpuVaFlags {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitAnd for GpuVaFlags {
    type Output = Self;
    fn bitand(self, rhs: Self) -> Self::Output {
        Self(self.0 & rhs.0)
    }
}

impl core::ops::Not for GpuVaFlags {
    type Output = Self;
    fn not(self) -> Self::Output {
        Self(!self.0)
    }
}

/// Trait that must be implemented by DRM drivers to represent a DRM GpuVm (a GPU address space).
pub trait DriverGpuVm: Sized {
    /// The parent `Driver` implementation for this `DriverGpuVm`.
    type Driver: drm::Driver;
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
        vm_bo: &GpuVmBo<Self>,
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
        pin_init::default()
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
    pub fn flags(&self) -> GpuVaFlags {
        GpuVaFlags(self.0.flags)
    }
    pub fn object(&self) -> &<<T::Driver as drm::Driver>::Object as BaseDriverObject>::Object {
        let p = unsafe {
            <<<T::Driver as drm::Driver>::Object as BaseDriverObject>::Object as IntoGEMObject>::as_ref(self.0.gem.obj)
        };
        // SAFETY: The GEM object has an active reference for the lifetime of this op
        &*p
    }
    pub fn map_and_link_va(
        &mut self,
        gpuvm: &mut UpdatingGpuVm<'_, T>,
        gpuva: Pin<KBox<GpuVa<T>>>,
        gpuvmbo: &GpuVmBo<T>,
    ) -> Result<(), Pin<KBox<GpuVa<T>>>> {
        // SAFETY: We are handing off the GpuVa ownership and it will not be moved.
        let p = KBox::leak(unsafe { Pin::into_inner_unchecked(gpuva) });
        // SAFETY: These C functions are called with the correct invariants
        unsafe {
            bindings::drm_gpuva_init_from_op(&mut p.gpuva, &mut self.0);
            if bindings::drm_gpuva_insert(gpuvm.0.gpuvm() as *mut _, &mut p.gpuva) != 0 {
                // EEXIST, return the GpuVa to the caller as an error
                return Err(Pin::new_unchecked(KBox::from_raw(p)));
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
    pub fn unmap_and_unlink_va(&mut self) -> Option<Pin<KBox<GpuVa<T>>>> {
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
        Some(unsafe { Pin::new_unchecked(KBox::from_raw(p)) })
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

impl<T: DriverGpuVm> GpuVa<T> {
    pub fn new<E>(inner: impl PinInit<T::GpuVa, E>) -> Result<Pin<KBox<GpuVa<T>>>>
    where
        Error: From<E>,
    {
        KBox::try_pin_init(
            try_pin_init!(Self {
                gpuva <- pin_init::zeroed(),
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
    pub fn offset(&self) -> u64 {
        self.gpuva.gem.offset
    }
    pub fn flags(&self) -> GpuVaFlags {
        GpuVaFlags(self.gpuva.flags)
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
    unsafe { drop(KBox::from_raw(p)) };
}

pub(super) unsafe extern "C" fn vm_bo_alloc_callback<T: DriverGpuVm>() -> *mut bindings::drm_gpuvm_bo
{
    let obj: Result<Pin<KBox<GpuVmBo<T>>>> = KBox::try_pin_init(
        try_pin_init!(GpuVmBo::<T> {
            bo <- pin_init::default(),
            inner <- T::GpuVmBo::new(),
            _p: PhantomPinned
        }),
        GFP_KERNEL,
    );

    match obj {
        Ok(obj) =>
        // SAFETY: The DRM core will keep this object pinned
        unsafe {
            let p = KBox::leak(Pin::into_inner_unchecked(obj));
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
    unsafe { drop(KBox::from_raw(p)) };
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

    let p_vm_bo = remap.unmap().va().unwrap().gpuva.vm_bo;

    let res = {
        // SAFETY: vm_bo pointer must be valid and non-null by the step_remap invariants.
        // Since we grab a ref, this reference's lifetime is until the decref.
        let vm_bo_ref = unsafe {
            bindings::drm_gpuvm_bo_get(p_vm_bo);
            &*(crate::container_of!(p_vm_bo, GpuVmBo<T>, bo) as *mut GpuVmBo<T>)
        };

        from_result(|| {
            UpdatingGpuVm(ctx.gpuvm).step_remap(remap, vm_bo_ref, ctx.ctx)?;
            Ok(0)
        })
    };

    // SAFETY: We incremented the refcount above, and the Rust reference we took is
    // no longer in scope.
    unsafe { bindings::drm_gpuvm_bo_put(p_vm_bo) };

    res
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
        r_obj: ARef<<<T::Driver as drm::Driver>::Object as BaseDriverObject>::Object>,
        range: Range<u64>,
        reserve_range: Range<u64>,
        inner: impl PinInit<T, E>,
    ) -> Result<ARef<GpuVm<T>>>
    where
        Error: From<E>,
    {
        let obj: Pin<KBox<Self>> = KBox::try_pin_init(
            try_pin_init!(Self {
                // SAFETY: drm_gpuvm_init cannot fail and always initializes the member
                gpuvm <- unsafe {
                    pin_init::pin_init_from_closure(move |slot: *mut Opaque<bindings::drm_gpuvm> | {
                        // Zero-init required by drm_gpuvm_init
                        *slot = Opaque::zeroed();
                        bindings::drm_gpuvm_init(
                            Opaque::raw_get(slot),
                            name.as_char_ptr(),
                            0,
                            dev.as_raw(),
                            r_obj.as_raw() as *const _ as *mut _,
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
                    pin_init::pin_init_from_closure(move |slot: *mut UnsafeCell<T> | {
                        inner.__pinned_init(slot as *mut _)
                    })
                },
                _p: PhantomPinned
            }),
            GFP_KERNEL,
        )?;

        // SAFETY: We never move out of the object
        let vm_ref = unsafe {
            ARef::from_raw(NonNull::new_unchecked(KBox::leak(
                Pin::into_inner_unchecked(obj),
            )))
        };

        Ok(vm_ref)
    }

    pub fn exec_lock<'a, 'b>(
        &'a self,
        obj: Option<&'b <<T::Driver as drm::Driver>::Object as BaseDriverObject>::Object>,
        interruptible: bool,
    ) -> Result<LockedGpuVm<'a, 'b, T>> {
        // Do not try to lock the object if it is internal (since it is already locked).
        let is_ext = obj.map(|a| self.is_extobj(a)).unwrap_or(false);

        let mut guard = ManuallyDrop::new(LockedGpuVm {
            gpuvm: self,
            // vm_exec needs to be pinned, so stick it in a Box.
            vm_exec: KBox::init(
                init!(bindings::drm_gpuvm_exec {
                    vm: self.gpuvm() as *mut _,
                    flags: if interruptible {
                        bindings::DRM_EXEC_INTERRUPTIBLE_WAIT
                    } else {
                        0
                    },
                    exec: Default::default(),
                    extra: match (is_ext, obj) {
                        (true, Some(obj)) => bindings::drm_gpuvm_exec__bindgen_ty_1 {
                            fn_: Some(exec_lock_gem_object),
                            priv_: obj.as_raw() as *const _ as *mut _,
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
        let gem = obj.as_raw() as *const _ as *mut _;
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
    vm_exec: KBox<bindings::drm_gpuvm_exec>,
    obj: Option<&'b <<T::Driver as drm::Driver>::Object as BaseDriverObject>::Object>,
}

impl<T: DriverGpuVm> LockedGpuVm<'_, '_, T> {
    pub fn find_bo(&mut self) -> Option<ARef<GpuVmBo<T>>> {
        let obj = self.obj?;
        // SAFETY: LockedGpuVm implies the right locks are held.
        let p = unsafe {
            bindings::drm_gpuvm_bo_find(
                self.gpuvm.gpuvm() as *mut _,
                obj.as_raw() as *const _ as *mut _,
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
                obj.as_raw() as *const _ as *mut _,
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
        flags: GpuVaFlags,
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
                obj.as_raw() as *const _ as *mut _,
                req_offset,
                flags.as_raw(),
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

    pub fn bo_unmap(&mut self, ctx: &mut T::StepContext, bo: &GpuVmBo<T>) -> Result {
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

// SAFETY: All our trait methods take locks
unsafe impl<T: DriverGpuVm> Sync for GpuVm<T> {}
// SAFETY: All our trait methods take locks
unsafe impl<T: DriverGpuVm> Send for GpuVm<T> {}

// SAFETY: All our trait methods take locks
unsafe impl<T: DriverGpuVm> Sync for GpuVmBo<T> {}
// SAFETY: All our trait methods take locks
unsafe impl<T: DriverGpuVm> Send for GpuVmBo<T> {}
