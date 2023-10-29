// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU Micro operation sequence builder
//!
//! As part of a single job submisssion to the GPU, the GPU firmware interprets a sequence of
//! commands that we call a "microsequence". These are responsible for setting up the job execution,
//! timestamping the process, waiting for completion, tearing up any resources, and signaling
//! completion to the driver via the event stamp mechanism.
//!
//! Although the microsequences used by the macOS driver are usually quite uniform and simple, the
//! firmware actually implements enough operations to make this interpreter Turing-complete (!).
//! Most of those aren't implemented yet, since we don't need them, but they could come in handy in
//! the future to do strange things or work around firmware bugs...
//!
//! This module simply implements a collection of microsequence operations that can be appended to
//! and later concatenated into one buffer, ready for firmware execution.

use crate::fw::microseq;
pub(crate) use crate::fw::microseq::*;
use crate::fw::types::*;
use kernel::prelude::*;

/// MicroSequence object type, which is just an opaque byte array.
pub(crate) type MicroSequence = GpuArray<u8>;

/// MicroSequence builder.
pub(crate) struct Builder {
    ops: Vec<u8>,
}

impl Builder {
    /// Create a new Builder object
    pub(crate) fn new() -> Builder {
        Builder { ops: Vec::new() }
    }

    /// Get the relative offset from the current pointer to a given target offset.
    ///
    /// Used for relative jumps.
    pub(crate) fn offset_to(&self, target: i32) -> i32 {
        target - self.ops.len() as i32
    }

    /// Add an operation to the end of the sequence.
    pub(crate) fn add<T: microseq::Operation>(&mut self, op: T) -> Result<i32> {
        let off = self.ops.len();
        let p: *const T = &op;
        let p: *const u8 = p as *const u8;
        let s: &[u8] = unsafe { core::slice::from_raw_parts(p, core::mem::size_of::<T>()) };
        self.ops.try_extend_from_slice(s)?;
        Ok(off as i32)
    }

    /// Collect all submitted operations into a finalized GPU object.
    pub(crate) fn build(self, alloc: &mut Allocator) -> Result<MicroSequence> {
        let mut array = alloc.array_empty::<u8>(self.ops.len())?;

        array.as_mut_slice().clone_from_slice(self.ops.as_slice());
        Ok(array)
    }
}
