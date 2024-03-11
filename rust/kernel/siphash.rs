// SPDX-License-Identifier: GPL-2.0

//! A core::hash::Hasher wrapper for the kernel siphash implementation.
//!
//! This module allows Rust code to use the kernel's siphash implementation
//! to hash Rust objects.

use core::hash::Hasher;

/// A Hasher implementation that uses the kernel siphash implementation.
#[derive(Default)]
pub struct SipHasher {
    // SipHash state is 4xu64, but the Linux implementation
    // doesn't expose incremental hashing so let's just chain
    // individual SipHash calls for now, which return a u64
    // hash.
    state: u64,
}

impl SipHasher {
    /// Create a new SipHasher with zeroed state.
    pub fn new() -> Self {
        SipHasher { state: 0 }
    }
}

impl Hasher for SipHasher {
    fn finish(&self) -> u64 {
        self.state
    }

    fn write(&mut self, bytes: &[u8]) {
        let key = bindings::siphash_key_t {
            key: [self.state, 0],
        };

        self.state = unsafe { bindings::siphash(bytes.as_ptr() as *const _, bytes.len(), &key) };
    }
}
