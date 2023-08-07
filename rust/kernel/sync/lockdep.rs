// SPDX-License-Identifier: GPL-2.0

//! Lockdep utilities.
//!
//! This module abstracts the parts of the kernel lockdep API relevant to Rust
//! modules, including lock classes.

use crate::{
    c_str, fmt,
    init::InPlaceInit,
    new_mutex,
    prelude::{Box, Result, Vec},
    str::{CStr, CString},
    sync::Mutex,
    types::Opaque,
};

use core::hash::{Hash, Hasher};
use core::pin::Pin;
use core::sync::atomic::{AtomicPtr, Ordering};

/// Represents a lockdep class. It's a wrapper around C's `lock_class_key`.
#[repr(transparent)]
pub struct StaticLockClassKey(Opaque<bindings::lock_class_key>);

impl StaticLockClassKey {
    /// Creates a new lock class key.
    pub const fn new() -> Self {
        Self(Opaque::uninit())
    }

    /// Returns the lock class key reference for this static lock class.
    pub const fn key(&self) -> LockClassKey {
        LockClassKey(self.0.get())
    }
}

// SAFETY: `bindings::lock_class_key` just represents an opaque memory location, and is never
// actually dereferenced.
unsafe impl Sync for StaticLockClassKey {}

/// A reference to a lock class key. This is a raw pointer to a lock_class_key,
/// which is required to have a static lifetime.
#[derive(Copy, Clone)]
pub struct LockClassKey(*mut bindings::lock_class_key);

impl LockClassKey {
    pub(crate) fn as_ptr(&self) -> *mut bindings::lock_class_key {
        self.0
    }
}

// SAFETY: `bindings::lock_class_key` just represents an opaque memory location, and is never
// actually dereferenced.
unsafe impl Send for LockClassKey {}
unsafe impl Sync for LockClassKey {}

// Location is 'static but not really, since module unloads will
// invalidate existing static Locations within that module.
// To avoid breakage, we maintain our own location struct which is
// dynamically allocated on first reference. We store a hash of the
// whole location (including the filename string), as well as the
// line and column separately. The assumption is that this whole
// struct is highly unlikely to ever collide with a reasonable
// hash (this saves us from having to check the filename string
// itself).
#[derive(PartialEq, Debug)]
struct LocationKey {
    hash: u64,
    line: u32,
    column: u32,
}

struct DynLockClassKey {
    key: Opaque<bindings::lock_class_key>,
    loc: LocationKey,
    name: CString,
}

impl LocationKey {
    fn new(loc: &'static core::panic::Location<'static>) -> Self {
        let mut hasher = crate::siphash::SipHasher::new();
        loc.hash(&mut hasher);

        LocationKey {
            hash: hasher.finish(),
            line: loc.line(),
            column: loc.column(),
        }
    }
}

impl DynLockClassKey {
    fn key(&'static self) -> LockClassKey {
        LockClassKey(self.key.get())
    }

    fn name(&'static self) -> &CStr {
        &self.name
    }
}

const LOCK_CLASS_BUCKETS: usize = 1024;

#[track_caller]
fn caller_lock_class_inner() -> Result<&'static DynLockClassKey> {
    // This is just a hack to make the below static array initialization work.
    #[allow(clippy::declare_interior_mutable_const)]
    const ATOMIC_PTR: AtomicPtr<Mutex<Vec<&'static DynLockClassKey>>> =
        AtomicPtr::new(core::ptr::null_mut());

    #[allow(clippy::complexity)]
    static LOCK_CLASSES: [AtomicPtr<Mutex<Vec<&'static DynLockClassKey>>>; LOCK_CLASS_BUCKETS] =
        [ATOMIC_PTR; LOCK_CLASS_BUCKETS];

    let loc = core::panic::Location::caller();
    let loc_key = LocationKey::new(loc);

    let index = (loc_key.hash % (LOCK_CLASS_BUCKETS as u64)) as usize;
    let slot = &LOCK_CLASSES[index];

    let mut ptr = slot.load(Ordering::Relaxed);
    if ptr.is_null() {
        let new_element = Box::pin_init(new_mutex!(Vec::new()))?;

        if let Err(e) = slot.compare_exchange(
            core::ptr::null_mut(),
            // SAFETY: We never move out of this Box
            Box::into_raw(unsafe { Pin::into_inner_unchecked(new_element) }),
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            // SAFETY: We just got this pointer from `into_raw()`
            unsafe { Box::from_raw(e) };
        }

        ptr = slot.load(Ordering::Relaxed);
        assert!(!ptr.is_null());
    }

    // SAFETY: This mutex was either just created above or previously allocated,
    // and we never free these objects so the pointer is guaranteed to be valid.
    let mut guard = unsafe { (*ptr).lock() };

    for i in guard.iter() {
        if i.loc == loc_key {
            return Ok(i);
        }
    }

    // We immediately leak the class, so it becomes 'static
    let new_class = Box::leak(Box::try_new(DynLockClassKey {
        key: Opaque::zeroed(),
        loc: loc_key,
        name: CString::try_from_fmt(fmt!("{}:{}:{}", loc.file(), loc.line(), loc.column()))?,
    })?);

    // SAFETY: This is safe to call with a pointer to a dynamically allocated lockdep key,
    // and we never free the objects so it is safe to never unregister the key.
    unsafe { bindings::lockdep_register_key(new_class.key.get()) };

    guard.try_push(new_class)?;

    Ok(new_class)
}

#[track_caller]
pub(crate) fn caller_lock_class() -> (LockClassKey, &'static CStr) {
    match caller_lock_class_inner() {
        Ok(a) => (a.key(), a.name()),
        Err(_) => {
            crate::pr_err!(
                "Failed to dynamically allocate lock class, lockdep may be unreliable.\n"
            );

            let loc = core::panic::Location::caller();
            // SAFETY: LockClassKey is opaque and the lockdep implementation only needs
            // unique addresses for statically allocated keys, so it is safe to just cast
            // the Location reference directly into a LockClassKey. However, this will
            // result in multiple keys for the same callsite due to monomorphization,
            // as well as spuriously destroyed keys when the static key is allocated in
            // the wrong module, which is what makes this unreliable.
            (
                LockClassKey(loc as *const _ as *mut _),
                c_str!("fallback_lock_class"),
            )
        }
    }
}

pub(crate) struct LockdepMap(Opaque<bindings::lockdep_map>);
pub(crate) struct LockdepGuard<'a>(&'a LockdepMap);

#[allow(dead_code)]
impl LockdepMap {
    #[track_caller]
    pub(crate) fn new() -> Self {
        let map = Opaque::uninit();
        let (key, name) = caller_lock_class();

        unsafe {
            bindings::lockdep_init_map_type(
                map.get(),
                name.as_char_ptr(),
                key.as_ptr(),
                0,
                bindings::lockdep_wait_type_LD_WAIT_INV as _,
                bindings::lockdep_wait_type_LD_WAIT_INV as _,
                bindings::lockdep_lock_type_LD_LOCK_NORMAL as _,
            )
        };

        LockdepMap(map)
    }

    #[inline(always)]
    pub(crate) fn lock(&self) -> LockdepGuard<'_> {
        unsafe { bindings::lock_acquire_ret(self.0.get(), 0, 0, 1, 1, core::ptr::null_mut()) };

        LockdepGuard(self)
    }
}

impl<'a> Drop for LockdepGuard<'a> {
    #[inline(always)]
    fn drop(&mut self) {
        unsafe { bindings::lock_release_ret(self.0 .0.get()) };
    }
}
