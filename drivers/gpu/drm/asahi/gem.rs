// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Asahi driver GEM object implementation
//!
//! Basic wrappers and adaptations between generic GEM shmem objects and this driver's
//! view of what a GPU buffer object is. It is in charge of keeping track of all mappings for
//! each GEM object so we can remove them when a client (File) or a Vm are destroyed, as well as
//! implementing RTKit buffers on top of GEM objects for firmware use.

use kernel::{
    drm::{gem, gem::shmem},
    error::Result,
    prelude::*,
    uapi,
};

use kernel::drm::gem::BaseObject;

use core::ops::Range;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::{debug::*, driver::AsahiDevice, file, file::DrmFile, mmu, util::*};

const DEBUG_CLASS: DebugFlags = DebugFlags::Gem;

/// Represents the inner data of a GEM object for this driver.
#[pin_data]
pub(crate) struct DriverObject {
    /// Whether this is a kernel-created object.
    kernel: bool,
    /// Object creation flags.
    flags: u32,
    /// ID for debug
    id: u64,
}

/// Type alias for the shmem GEM object type for this driver.
pub(crate) type Object = shmem::Object<DriverObject>;

/// Type alias for the SGTable type for this driver.
pub(crate) type SGTable = shmem::SGTable<DriverObject>;

/// A shared reference to a GEM object for this driver.
pub(crate) struct ObjectRef {
    /// The underlying GEM object reference
    pub(crate) gem: gem::ObjectRef<shmem::Object<DriverObject>>,
    /// The kernel-side VMap of this object, if needed
    vmap: Option<shmem::VMap<DriverObject>>,
}

crate::no_debug!(ObjectRef);

static GEM_ID: AtomicU64 = AtomicU64::new(0);

impl ObjectRef {
    /// Create a new wrapper for a raw GEM object reference.
    pub(crate) fn new(gem: gem::ObjectRef<shmem::Object<DriverObject>>) -> ObjectRef {
        ObjectRef { gem, vmap: None }
    }

    /// Return the `VMap` for this object, creating it if necessary.
    pub(crate) fn vmap(&mut self) -> Result<&mut shmem::VMap<DriverObject>> {
        if self.vmap.is_none() {
            self.vmap = Some(self.gem.vmap()?);
        }
        Ok(self.vmap.as_mut().unwrap())
    }

    /// Returns the size of an object in bytes
    pub(crate) fn size(&self) -> usize {
        self.gem.size()
    }

    /// Maps an object into a given `Vm` at any free address within a given range.
    pub(crate) fn map_into_range(
        &mut self,
        vm: &crate::mmu::Vm,
        range: Range<u64>,
        alignment: u64,
        prot: u32,
        guard: bool,
    ) -> Result<crate::mmu::KernelMapping> {
        // Only used for kernel objects now
        if !self.gem.kernel {
            return Err(EINVAL);
        }
        vm.map_in_range(self.gem.size(), &self.gem, alignment, range, prot, guard)
    }

    /// Maps an object into a given `Vm` at a specific address.
    ///
    /// Returns Err(ENOSPC) if the requested address is already busy.
    pub(crate) fn map_at(
        &mut self,
        vm: &crate::mmu::Vm,
        addr: u64,
        prot: u32,
        guard: bool,
    ) -> Result<crate::mmu::KernelMapping> {
        if self.gem.flags & uapi::ASAHI_GEM_VM_PRIVATE != 0 && vm.is_extobj(&self.gem) {
            return Err(EINVAL);
        }

        vm.map_at(addr, self.gem.size(), &self.gem, prot, guard)
    }
}

/// Create a new kernel-owned GEM object.
pub(crate) fn new_kernel_object(dev: &AsahiDevice, size: usize) -> Result<ObjectRef> {
    let mut gem = shmem::Object::<DriverObject>::new(dev, align(size, mmu::UAT_PGSZ))?;
    gem.kernel = true;
    gem.flags = 0;

    gem.set_exportable(false);

    mod_pr_debug!("DriverObject new kernel object id={}\n", gem.id);
    Ok(ObjectRef::new(gem.into_ref()))
}

/// Create a new user-owned GEM object with the given flags.
pub(crate) fn new_object(
    dev: &AsahiDevice,
    size: usize,
    flags: u32,
    parent_object: Option<&gem::ObjectRef<Object>>,
) -> Result<ObjectRef> {
    if (flags & uapi::ASAHI_GEM_VM_PRIVATE != 0) != parent_object.is_some() {
        return Err(EINVAL);
    }

    let mut gem = shmem::Object::<DriverObject>::new(dev, align(size, mmu::UAT_PGSZ))?;
    gem.kernel = false;
    gem.flags = flags;

    gem.set_exportable(parent_object.is_none());
    gem.set_wc(flags & uapi::ASAHI_GEM_WRITEBACK == 0);
    if let Some(parent) = parent_object {
        gem.share_dma_resv(&**parent)?;
    }

    mod_pr_debug!("DriverObject new user object: id={}\n", gem.id);
    Ok(ObjectRef::new(gem.into_ref()))
}

/// Look up a GEM object handle for a `File` and return an `ObjectRef` for it.
pub(crate) fn lookup_handle(file: &DrmFile, handle: u32) -> Result<ObjectRef> {
    Ok(ObjectRef::new(shmem::Object::lookup_handle(file, handle)?))
}

impl gem::BaseDriverObject<Object> for DriverObject {
    type Initializer = impl PinInit<Self, Error>;

    /// Callback to create the inner data of a GEM object
    fn new(_dev: &AsahiDevice, _size: usize) -> Self::Initializer {
        let id = GEM_ID.fetch_add(1, Ordering::Relaxed);
        mod_pr_debug!("DriverObject::new id={}\n", id);
        try_pin_init!(DriverObject {
            kernel: false,
            flags: 0,
            id,
        })
    }

    /// Callback to drop all mappings for a GEM object owned by a given `File`
    fn close(obj: &Object, file: &DrmFile) {
        mod_pr_debug!("DriverObject::close id={}\n", obj.id);
        if file::File::unbind_gem_object(file, obj).is_err() {
            pr_err!("DriverObject::close: Failed to unbind GEM object\n");
        }
    }
}

impl shmem::DriverObject for DriverObject {
    type Driver = crate::driver::AsahiDriver;
}
