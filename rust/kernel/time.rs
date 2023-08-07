// SPDX-License-Identifier: GPL-2.0

//! Timekeeping functions.
//!
//! C header: [`include/linux/ktime.h`](../../../../include/linux/ktime.h)
//! C header: [`include/linux/timekeeping.h`](../../../../include/linux/timekeeping.h)

use crate::{bindings, pr_err};
use core::marker::PhantomData;
use core::time::Duration;

/// Represents a clock, that is, a unique time source.
pub trait Clock: Sized {}

/// A time source that can be queried for the current time.
pub trait Now: Clock {
    /// Returns the current time for this clock.
    fn now() -> Instant<Self>;
}

/// Marker trait for clock sources that are guaranteed to be monotonic.
pub trait Monotonic {}

/// Marker trait for clock sources that represent a calendar (wall clock)
/// relative to the UNIX epoch.
pub trait WallTime {}

/// An instant in time associated with a given clock source.
#[derive(Debug)]
pub struct Instant<T: Clock> {
    nanoseconds: i64,
    _type: PhantomData<T>,
}

impl<T: Clock> Clone for Instant<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T: Clock> Copy for Instant<T> {}

impl<T: Clock> Instant<T> {
    fn new(nanoseconds: i64) -> Self {
        Instant {
            nanoseconds,
            _type: PhantomData,
        }
    }

    /// Returns the time elapsed since an earlier Instant<t>, or
    /// None if the argument is a later Instant.
    pub fn since(&self, earlier: Instant<T>) -> Option<Duration> {
        if earlier.nanoseconds > self.nanoseconds {
            None
        } else {
            // Casting to u64 and subtracting is guaranteed to give the right
            // result for all inputs, as long as the condition we checked above
            // holds.
            Some(Duration::from_nanos(
                self.nanoseconds as u64 - earlier.nanoseconds as u64,
            ))
        }
    }
}

impl<T: Clock + Now + Monotonic> Instant<T> {
    /// Returns the time elapsed since this Instant<T>.
    ///
    /// This is guaranteed to return a positive result, since
    /// it is only implemented for monotonic clocks.
    pub fn elapsed(&self) -> Duration {
        T::now().since(*self).unwrap_or_else(|| {
            pr_err!(
                "Monotonic clock {} went backwards!",
                core::any::type_name::<T>()
            );
            Duration::ZERO
        })
    }
}

/// Contains the various clock source types available to the kernel.
pub mod clock {
    use super::*;

    /// A clock representing the default kernel time source.
    ///
    /// This is `CLOCK_MONOTONIC` (though it is not the only
    /// monotonic clock) and also the default clock used by
    /// `ktime_get()` in the C API.
    ///
    /// This is like `BootTime`, but does not include time
    /// spent sleeping.

    pub struct KernelTime;

    impl Clock for KernelTime {}
    impl Monotonic for KernelTime {}
    impl Now for KernelTime {
        fn now() -> Instant<Self> {
            Instant::<Self>::new(unsafe { bindings::ktime_get() })
        }
    }

    /// A clock representing the time elapsed since boot.
    ///
    /// This is `CLOCK_MONOTONIC` (though it is not the only
    /// monotonic clock) and also the default clock used by
    /// `ktime_get()` in the C API.
    ///
    /// This is like `KernelTime`, but does include time
    /// spent sleeping.
    pub struct BootTime;

    impl Clock for BootTime {}
    impl Monotonic for BootTime {}
    impl Now for BootTime {
        fn now() -> Instant<Self> {
            Instant::<Self>::new(unsafe { bindings::ktime_get_boottime() })
        }
    }

    /// A clock representing TAI time.
    ///
    /// This clock is not monotonic and can be changed from userspace.
    /// However, it is not affected by leap seconds.
    pub struct TaiTime;

    impl Clock for TaiTime {}
    impl WallTime for TaiTime {}
    impl Now for TaiTime {
        fn now() -> Instant<Self> {
            Instant::<Self>::new(unsafe { bindings::ktime_get_clocktai() })
        }
    }

    /// A clock representing wall clock time.
    ///
    /// This clock is not monotonic and can be changed from userspace.
    pub struct RealTime;

    impl Clock for RealTime {}
    impl WallTime for RealTime {}
    impl Now for RealTime {
        fn now() -> Instant<Self> {
            Instant::<Self>::new(unsafe { bindings::ktime_get_real() })
        }
    }
}
