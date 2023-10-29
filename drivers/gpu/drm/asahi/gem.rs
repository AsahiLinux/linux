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
    soc::apple::rtkit,
    sync::Mutex,
    uapi,
};

use kernel::drm::gem::BaseObject;

use core::sync::atomic::{AtomicU64, Ordering};

use crate::{debug::*, driver::AsahiDevice, file::DrmFile, mmu, util::*};

const DEBUG_CLASS: DebugFlags = DebugFlags::Gem;

/// Represents the inner data of a GEM object for this driver.
#[pin_data]
pub(crate) struct DriverObject {
    /// Whether this is a kernel-created object.
    kernel: bool,
    /// Object creation flags.
    flags: u32,
    /// VM ID for VM-private objects.
    vm_id: Option<u64>,
    /// Locked list of mapping tuples: (file_id, vm_id, mapping)
    #[pin]
    mappings: Mutex<Vec<(u64, u64, crate::mmu::Mapping)>>,
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

impl DriverObject {
    /// Drop all object mappings for a given file ID.
    ///
    /// Used on file close.
    fn drop_file_mappings(&self, file_id: u64) {
        let mut mappings = self.mappings.lock();
        for (index, (mapped_fid, _mapped_vmid, _mapping)) in mappings.iter().enumerate() {
            if *mapped_fid == file_id {
                mappings.swap_remove(index);
                return;
            }
        }
    }

    /// Drop all object mappings for a given VM ID.
    ///
    /// Used on VM destroy.
    fn drop_vm_mappings(&self, vm_id: u64) {
        let mut mappings = self.mappings.lock();
        for (index, (_mapped_fid, mapped_vmid, _mapping)) in mappings.iter().enumerate() {
            if *mapped_vmid == vm_id {
                mappings.swap_remove(index);
                return;
            }
        }
    }
}

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

    /// Return the IOVA of this object at which it is mapped in a given `Vm` identified by its ID,
    /// if it is mapped in that `Vm`.
    pub(crate) fn iova(&self, vm_id: u64) -> Option<usize> {
        let mappings = self.gem.mappings.lock();
        for (_mapped_fid, mapped_vmid, mapping) in mappings.iter() {
            if *mapped_vmid == vm_id {
                return Some(mapping.iova());
            }
        }

        None
    }

    /// Returns the size of an object in bytes
    pub(crate) fn size(&self) -> usize {
        self.gem.size()
    }

    /// Maps an object into a given `Vm` at any free address within a given range.
    ///
    /// Returns Err(EBUSY) if there is already a mapping.
    pub(crate) fn map_into_range(
        &mut self,
        vm: &crate::mmu::Vm,
        start: u64,
        end: u64,
        alignment: u64,
        prot: u32,
        guard: bool,
    ) -> Result<usize> {
        let vm_id = vm.id();

        if self.gem.vm_id.is_some() && self.gem.vm_id != Some(vm_id) {
            return Err(EINVAL);
        }

        let mut mappings = self.gem.mappings.lock();
        for (_mapped_fid, mapped_vmid, _mapping) in mappings.iter() {
            if *mapped_vmid == vm_id {
                return Err(EBUSY);
            }
        }

        let sgt = self.gem.sg_table()?;
        let new_mapping =
            vm.map_in_range(self.gem.size(), sgt, alignment, start, end, prot, guard)?;

        let iova = new_mapping.iova();
        mappings.try_push((vm.file_id(), vm_id, new_mapping))?;
        Ok(iova)
    }

    /// Maps an object into a given `Vm` at a specific address.
    ///
    /// Returns Err(EBUSY) if there is already a mapping.
    /// Returns Err(ENOSPC) if the requested address is already busy.
    pub(crate) fn map_at(
        &mut self,
        vm: &crate::mmu::Vm,
        addr: u64,
        prot: u32,
        guard: bool,
    ) -> Result {
        let vm_id = vm.id();

        if self.gem.vm_id.is_some() && self.gem.vm_id != Some(vm_id) {
            return Err(EINVAL);
        }

        let mut mappings = self.gem.mappings.lock();
        for (_mapped_fid, mapped_vmid, _mapping) in mappings.iter() {
            if *mapped_vmid == vm_id {
                return Err(EBUSY);
            }
        }

        let sgt = self.gem.sg_table()?;
        let new_mapping = vm.map_at(addr, self.gem.size(), sgt, prot, guard)?;

        let iova = new_mapping.iova();
        assert!(iova == addr as usize);
        mappings.try_push((vm.file_id(), vm_id, new_mapping))?;
        Ok(())
    }

    /// Drop all mappings for this object owned by a given `Vm` identified by its ID.
    pub(crate) fn drop_vm_mappings(&mut self, vm_id: u64) {
        self.gem.drop_vm_mappings(vm_id);
    }

    /// Drop all mappings for this object owned by a given `File` identified by its ID.
    pub(crate) fn drop_file_mappings(&mut self, file_id: u64) {
        self.gem.drop_file_mappings(file_id);
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
    vm_id: Option<u64>,
) -> Result<ObjectRef> {
    let mut gem = shmem::Object::<DriverObject>::new(dev, align(size, mmu::UAT_PGSZ))?;
    gem.kernel = false;
    gem.flags = flags;
    gem.vm_id = vm_id;

    gem.set_exportable(vm_id.is_none());
    gem.set_wc(flags & uapi::ASAHI_GEM_WRITEBACK == 0);

    mod_pr_debug!(
        "DriverObject new user object: vm_id={:?} id={}\n",
        vm_id,
        gem.id
    );
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
            vm_id: None,
            mappings <- Mutex::new(Vec::new()),
            id,
        })
    }

    /// Callback to drop all mappings for a GEM object owned by a given `File`
    fn close(obj: &Object, file: &DrmFile) {
        mod_pr_debug!("DriverObject::close vm_id={:?} id={}\n", obj.vm_id, obj.id);
        obj.drop_file_mappings(file.inner().file_id());
    }
}

impl shmem::DriverObject for DriverObject {
    type Driver = crate::driver::AsahiDriver;
}

impl rtkit::Buffer for ObjectRef {
    fn iova(&self) -> Result<usize> {
        self.iova(0).ok_or(EIO)
    }
    fn buf(&mut self) -> Result<&mut [u8]> {
        let vmap = self.vmap.as_mut().ok_or(ENOMEM)?;
        Ok(vmap.as_mut_slice())
    }
}
