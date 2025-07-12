// SPDX-License-Identifier: GPL-2.0

//! DRM GEM shmem helper objects
//!
//! C header: [`include/linux/drm/drm_gem_shmem_helper.h`](srctree/include/linux/drm/drm_gem_shmem_helper.h)

// TODO:
// - There are a number of spots here that manually acquire/release the DMA reservation lock using
//   dma_resv_(un)lock(). In the future we should add support for ww mutex, expose a method to
//   acquire a reference to the WwMutex, and then use that directly instead of the C functions here.

use crate::{
    drm::{
        device,
        driver,
        gem,
        private::Sealed
    },
    error::{from_err_ptr, to_result},
    prelude::*,
    types::{ARef, Opaque},
    container_of,
    scatterlist::SGTable,
};
use core::{
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    ptr::{addr_of_mut, NonNull},
    slice,
};
use gem::{
    BaseObject,
    BaseObjectPrivate,
    BaseDriverObject,
    OpaqueObject,
    IntoGEMObject
};

/// A struct for controlling the creation of shmem-backed GEM objects.
///
/// This is used with [`Object::new()`] to control various properties that can only be set when
/// initially creating a shmem-backed GEM object.
#[derive(Default)]
pub struct ObjectConfig<'a, T: BaseDriverObject> {
    /// Whether to set the write-combine map flag.
    pub map_wc: bool,

    /// Reuse the DMA reservation from another GEM object.
    ///
    /// The newly created [`Object`] will hold an owned refcount to `parent_resv_obj` if specified.
    pub parent_resv_obj: Option<&'a OpaqueObject<T::Driver>>,
}

/// A shmem-backed GEM object.
///
/// # Invariants
///
/// The DRM core ensures that `dev` will remain valid for as long as the object.
#[repr(C)]
#[pin_data]
pub struct Object<T: BaseDriverObject + Send + Sync> {
    #[pin]
    obj: Opaque<bindings::drm_gem_shmem_object>,
    dev: NonNull<device::Device<T::Driver>>,
    // Parent object that owns this object's DMA reservation object
    parent_resv_obj: Option<ARef<OpaqueObject<T::Driver>>>,
    #[pin]
    inner: T,
}

unsafe impl<T: BaseDriverObject> Sync for Object<T> {}
unsafe impl<T: BaseDriverObject> Send for Object<T> {}

super::impl_as_opaque!(Object<T> where T: BaseDriverObject);

impl<T: BaseDriverObject> Object<T> {
    /// `drm_gem_object_funcs` vtable suitable for GEM shmem objects.
    const VTABLE: bindings::drm_gem_object_funcs = bindings::drm_gem_object_funcs {
        free: Some(Self::free_callback),
        open: Some(super::open_callback::<T>),
        close: Some(super::close_callback::<T>),
        print_info: Some(bindings::drm_gem_shmem_object_print_info),
        export: if T::HAS_EXPORT {
            Some(super::export_callback::<T>)
        } else {
            None
        },
        pin: Some(bindings::drm_gem_shmem_object_pin),
        unpin: Some(bindings::drm_gem_shmem_object_unpin),
        get_sg_table: Some(bindings::drm_gem_shmem_object_get_sg_table),
        vmap: Some(bindings::drm_gem_shmem_object_vmap),
        vunmap: Some(bindings::drm_gem_shmem_object_vunmap),
        mmap: Some(bindings::drm_gem_shmem_object_mmap),
        status: None,
        rss: None,
        // SAFETY: `drm_gem_shmem_vm_ops` is static const on the C side, so immutable references are
        // safe here and such references shall be valid forever
        vm_ops: unsafe { &bindings::drm_gem_shmem_vm_ops },
        evict: None,
    };

    /// Return a raw pointer to the embedded drm_gem_shmem_object.
    fn as_shmem(&self) -> *mut bindings::drm_gem_shmem_object {
        self.obj.get()
    }

    /// Create a new shmem-backed DRM object of the given size.
    ///
    /// Additional config options can be specified using `config`.
    pub fn new(
        dev: &device::Device<T::Driver>,
        size: usize,
        config: ObjectConfig<'_, T>,
        args: T::Args,
    ) -> Result<ARef<Self>> {
        let new: Pin<KBox<Self>> = KBox::try_pin_init(
            try_pin_init!(Self {
                obj <- pin_init::zeroed(),
                dev: NonNull::from(dev),
                parent_resv_obj: config.parent_resv_obj.map(|p| p.into()),
                inner <- T::new(dev, size, args),
            }),
            GFP_KERNEL
        )?;

        // SAFETY: `obj.as_raw()` is guaranteed to be valid by the initialization above.
        unsafe { (*new.as_raw()).funcs = &Self::VTABLE };

        // SAFETY: The arguments are all valid via the type invariants.
        to_result(unsafe { bindings::drm_gem_shmem_init(dev.as_raw(), new.as_shmem(), size) })?;

        // SAFETY: We never move out of `self`.
        let new = KBox::into_raw(unsafe { Pin::into_inner_unchecked(new) });

        // SAFETY: We're taking over the owned refcount from `drm_gem_shmem_init`.
        let obj = unsafe { ARef::from_raw(NonNull::new_unchecked(new)) };

        // Start filling out values from `config`
        if let Some(parent_resv) = config.parent_resv_obj {
            // SAFETY: We have yet to expose the new gem object outside of this function, so it is
            // safe to modify this field.
            unsafe { (*obj.obj.get()).base.resv = parent_resv.raw_dma_resv() };
        }

        // SAFETY: We have yet to expose this object outside of this function, so we're guaranteed
        // to have exclusive access - thus making this safe to hold a mutable reference to.
        let shmem = unsafe { &mut *obj.as_shmem() };
        shmem.set_map_wc(config.map_wc);

        Ok(obj)
    }

    /// Returns the `Device` that owns this GEM object.
    pub fn dev(&self) -> &device::Device<T::Driver> {
        // SAFETY: We are guaranteed that `dev` is valid for as long as this object is valid by our
        // type invariants
        unsafe { self.dev.as_ref() }
    }

    extern "C" fn free_callback(obj: *mut bindings::drm_gem_object) {
        // SAFETY:
        // - DRM always passes a valid gem object here
        // - We used drm_gem_shmem_create() in our create_gem_object callback, so we know that
        //   `obj` is contained within a drm_gem_shmem_object
        let shmem = unsafe { container_of!(obj, bindings::drm_gem_shmem_object, base) };

        let ptr: *mut Opaque<bindings::drm_gem_shmem_object> = shmem.cast();
        // SAFETY:
        // - We verified above that `obj` is valid, which makes `ptr` valid
        // - This function is set in AllocOps, so we know that `ptr` is contained within a
        //   `Object<T>`
        let this = unsafe { container_of!(ptr, Self, obj) };

        // SAFETY:
        // - We're in free_callback - so this function is safe to call.
        // - We won't be using the gem resources on `shmem` after this call.
        unsafe { bindings::drm_gem_shmem_release(shmem) };

        // SAFETY: We're recovering the Kbox<> we created in gem_create_object()
        let _ = unsafe { KBox::from_raw(this) };
    }

    /// Creates (if necessary) and returns an immutable reference to a scatter-gather table of DMA
    /// pages for this object.
    ///
    /// This will pin the object in memory.
    #[inline]
    pub fn sg_table(&self) -> Result<&SGTable> {
        // SAFETY:
        // - drm_gem_shmem_get_pages_sgt is thread-safe.
        // - drm_gem_shmem_get_pages_sgt returns either a valid pointer to a scatterlist, or an
        //   error pointer.
        let sgt = from_err_ptr(unsafe { bindings::drm_gem_shmem_get_pages_sgt(self.as_shmem()) })?;

        // SAFETY: We checked above that `sgt` is not an error pointer, so it must be a valid
        // pointer to a scatterlist
        Ok(unsafe { SGTable::as_ref(sgt) })
    }

    /// Creates (if necessary) and returns an owned scatter-gather table of DMA pages for this
    /// object.
    ///
    /// This is the same as [`sg_table`](Self::sg_table), except that it instead returns a
    /// [`OwnedSGTable`] which holds a reference to the associated gem object.
    ///
    /// This will pin the object in memory.
    pub fn owned_sg_table(&self) -> Result<OwnedSGTable<T>> {
        Ok(OwnedSGTable {
            sgt: self.sg_table()?.into(),
            // INVARIANT: We take an owned refcount to `self` here, ensuring that `sgt` remains
            // valid for as long as this `OwnedSGTable`.
            _owner: self.into(),
        })
    }

    /// Creates and returns a virtual kernel memory mapping for this object.
    pub fn vmap(&self) -> Result<VMap<T>> {
        let mut map: MaybeUninit<bindings::iosys_map> = MaybeUninit::uninit();

        // SAFETY:
        // - drm_gem_shmem_vmap can be called with the DMA reservation lock held
        // - Our ARef is proof that `obj` is safe to deref
        to_result(unsafe {
            // TODO: see top of file
            bindings::dma_resv_lock(self.raw_dma_resv(), core::ptr::null_mut());
            let ret = bindings::drm_gem_shmem_vmap(self.as_shmem(), map.as_mut_ptr());
            bindings::dma_resv_unlock(self.raw_dma_resv());
            ret
        })?;

        // SAFETY: if drm_gem_shmem_vmap did not fail, map is initialized now
        let map = unsafe { map.assume_init() };

        Ok(VMap {
            map,
            owner: self.into(),
        })
    }
}

impl<T: BaseDriverObject> Deref for Object<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T: BaseDriverObject> DerefMut for Object<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<T: BaseDriverObject> Sealed for Object<T> {}

impl<T: BaseDriverObject> gem::IntoGEMObject for Object<T> {
    fn as_raw(&self) -> *mut bindings::drm_gem_object {
        // SAFETY: Our immutable reference is proof that this is afe to dereference
        unsafe { addr_of_mut!((*self.obj.get()).base) }
    }

    unsafe fn as_ref<'a>(obj: *mut bindings::drm_gem_object) -> &'a Object<T> {
        // SAFETY: The safety contract of from_gem_obj() guarantees that `obj` is contained within
        // `Self`
        unsafe {
            let obj = container_of!(obj, bindings::drm_gem_shmem_object, base);
            let obj: *mut Opaque<bindings::drm_gem_shmem_object> = obj.cast();

            &*container_of!(obj, Object<T>, obj)
        }
    }
}

impl<T: BaseDriverObject> driver::AllocImpl for Object<T> {
    type Driver = T::Driver;

    const ALLOC_OPS: driver::AllocOps = driver::AllocOps {
        gem_create_object: None,
        prime_handle_to_fd: None,
        prime_fd_to_handle: None,
        gem_prime_import: None,
        gem_prime_import_sg_table: Some(bindings::drm_gem_shmem_prime_import_sg_table),
        dumb_create: Some(bindings::drm_gem_shmem_dumb_create),
        dumb_map_offset: None,
    };
}

/// A virtual mapping for a shmem-backed GEM object in kernel address space.
pub struct VMap<T: BaseDriverObject> {
    map: bindings::iosys_map,
    owner: ARef<Object<T>>,
}

impl<T: BaseDriverObject> VMap<T> {
    /// Returns a const raw pointer to the start of the mapping.
    pub fn as_ptr(&self) -> *const core::ffi::c_void {
        // SAFETY: The shmem helpers always return non-iomem maps
        unsafe { self.map.__bindgen_anon_1.vaddr }
    }

    /// Returns a mutable raw pointer to the start of the mapping.
    pub fn as_mut_ptr(&mut self) -> *mut core::ffi::c_void {
        // SAFETY: The shmem helpers always return non-iomem maps
        unsafe { self.map.__bindgen_anon_1.vaddr }
    }

    /// Returns a byte slice view of the mapping.
    pub fn as_slice(&self) -> &[u8] {
        // SAFETY: The vmap maps valid memory up to the owner size
        unsafe { slice::from_raw_parts(self.as_ptr() as *const u8, self.owner.size()) }
    }

    /// Returns mutable a byte slice view of the mapping.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        // SAFETY: The vmap maps valid memory up to the owner size
        unsafe { slice::from_raw_parts_mut(self.as_mut_ptr() as *mut u8, self.owner.size()) }
    }

    /// Borrows a reference to the object that owns this virtual mapping.
    pub fn owner(&self) -> &ARef<Object<T>> {
        &self.owner
    }
}

impl<T: BaseDriverObject> Drop for VMap<T> {
    fn drop(&mut self) {
        // SAFETY:
        // - This function is safe to call with the DMA reservation lock held
        // - Our `ARef` is proof that the underlying gem object here is initialized and thus safe to
        //   dereference.
        unsafe {
            let resv = self.owner.raw_dma_resv();

            // TODO: see top of file
            bindings::dma_resv_lock(resv, core::ptr::null_mut());
            bindings::drm_gem_shmem_vunmap(self.owner.as_shmem(), &mut self.map);
            bindings::dma_resv_unlock(resv);
        }
    }
}

/// SAFETY: `iosys_map` objects are safe to send across threads.
unsafe impl<T: BaseDriverObject> Send for VMap<T> {}
/// SAFETY: `iosys_map` objects are safe to send across threads.
unsafe impl<T: BaseDriverObject> Sync for VMap<T> {}

/// An owned scatter-gather table of DMA address spans for a GEM shmem object.
///
/// # Invariants
///
/// - `sgt` is kept alive by `_owner`, ensuring it remains valid for as long as `Self`.
/// - `sgt` corresponds to the owned object in `_owner`.
/// - This object is only exposed in situations where we know the underlying `SGTable` will not be
///   modified for the lifetime of this object.
pub struct OwnedSGTable<T: BaseDriverObject> {
    sgt: NonNull<SGTable>,
    _owner: ARef<Object<T>>,
}

// SAFETY: This object is only exposed in situations where we know the underlying `SGTable` will not
// be modified for the lifetime of this object.
unsafe impl<T: BaseDriverObject> Send for OwnedSGTable<T> {}
// SAFETY: This object is only exposed in situations where we know the underlying `SGTable` will not
// be modified for the lifetime of this object.
unsafe impl<T: BaseDriverObject> Sync for OwnedSGTable<T> {}

impl<T: BaseDriverObject> Deref for OwnedSGTable<T> {
    type Target = SGTable;

    fn deref(&self) -> &Self::Target {
        // SAFETY: Creating an immutable reference to this is safe via our type invariants.
        unsafe { self.sgt.as_ref() }
    }
}
