// SPDX-License-Identifier: GPL-2.0

//! Dummy lockdep utilities.
//!
//! Takes the place of the `lockdep` module when lockdep is disabled.

use crate::{c_str, str::CStr};

/// A dummy, zero-sized lock class.
pub struct StaticLockClassKey();

impl StaticLockClassKey {
    /// Creates a new dummy lock class key.
    pub const fn new() -> Self {
        Self()
    }

    /// Returns the lock class key reference for this static lock class.
    pub const fn key(&self) -> LockClassKey {
        LockClassKey()
    }
}

/// A dummy reference to a lock class key.
#[derive(Copy, Clone)]
pub struct LockClassKey();

impl LockClassKey {
    pub(crate) fn as_ptr(&self) -> *mut bindings::lock_class_key {
        core::ptr::null_mut()
    }
}

pub(crate) fn caller_lock_class() -> (LockClassKey, &'static CStr) {
    static DUMMY_LOCK_CLASS: StaticLockClassKey = StaticLockClassKey::new();

    (DUMMY_LOCK_CLASS.key(), c_str!("dummy"))
}
