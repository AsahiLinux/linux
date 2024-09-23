// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(dead_code)]

//! Debug enable/disable flags and convenience macros

#[allow(unused_imports)]
pub(crate) use super::{cls_dev_dbg, cls_pr_debug, debug, mod_dev_dbg, mod_pr_debug};
use core::sync::atomic::{AtomicU64, Ordering};

static DEBUG_FLAGS: AtomicU64 = AtomicU64::new(0);

/// Debug flag bit indices
pub(crate) enum DebugFlags {
    // 0-3: Memory-related debug
    Mmu = 0,
    Alloc = 1,
    Gem = 2,
    Object = 3,

    // 4-7: Firmware objects and resources
    Event = 4,
    Buffer = 5,
    WorkQueue = 6,

    // 8-13: DRM interface, rendering, compute, GPU globals
    Gpu = 8,
    File = 9,
    Queue = 10,
    Render = 11,
    Compute = 12,
    Errors = 13,

    // 14-15: Misc stats
    MemStats = 14,
    TVBStats = 15,

    // 16-22: Channels
    FwLogCh = 16,
    KTraceCh = 17,
    StatsCh = 18,
    EventCh = 19,
    PipeCh = 20,
    DeviceControlCh = 21,
    FwCtlCh = 22,

    // 32-35: Allocator debugging
    FillAllocations = 32,
    DebugAllocations = 33,
    DetectOverflows = 34,
    ForceCPUMaps = 35,

    // 36-: Behavior flags
    ConservativeTlbi = 36,
    KeepGpuPowered = 37,
    WaitForPowerOff = 38,
    NoGpuRecovery = 39,
    DisableClustering = 40,

    // 48-: Misc
    Debug0 = 48,
    Debug1 = 49,
    Debug2 = 50,
    Debug3 = 51,
    Debug4 = 52,
    Debug5 = 53,
    Debug6 = 54,
    Debug7 = 55,

    VerboseFaults = 61,
    AllowUnknownOverrides = 62,
    OopsOnGpuCrash = 63,
}

/// Update the cached global debug flags from the module parameter
pub(crate) fn update_debug_flags() {
    let flags = {
        let lock = crate::THIS_MODULE.kernel_param_lock();
        *crate::debug_flags.read(&lock)
    };

    DEBUG_FLAGS.store(flags, Ordering::Relaxed);
}

/// Check whether debug is enabled for a given flag
#[inline(always)]
pub(crate) fn debug_enabled(flag: DebugFlags) -> bool {
    DEBUG_FLAGS.load(Ordering::Relaxed) & 1 << (flag as usize) != 0
}

/// Run some code only if debug is enabled for the calling module
#[macro_export]
macro_rules! debug {
    ($($arg:tt)*) => {
        if $crate::debug::debug_enabled(DEBUG_CLASS) {
            $($arg)*
        }
    };
}

/// pr_info!() if debug is enabled for the calling module
#[macro_export]
macro_rules! mod_pr_debug (
    ($($arg:tt)*) => (
        $crate::debug! { ::kernel::pr_info! ( $($arg)* ); }
    )
);

/// dev_info!() if debug is enabled for the calling module
#[macro_export]
macro_rules! mod_dev_dbg (
    ($($arg:tt)*) => (
        $crate::debug! { ::kernel::dev_info! ( $($arg)* ); }
    )
);

/// pr_info!() if debug is enabled for a specific module
#[macro_export]
macro_rules! cls_pr_debug (
    ($cls:ident, $($arg:tt)*) => (
        if $crate::debug::debug_enabled($crate::debug::DebugFlags::$cls) {
            ::kernel::pr_info! ( $($arg)* );
        }
    )
);

/// dev_info!() if debug is enabled for a specific module
#[macro_export]
macro_rules! cls_dev_dbg (
    ($cls:ident, $($arg:tt)*) => (
        if $crate::debug::debug_enabled($crate::debug::DebugFlags::$cls) {
            ::kernel::dev_info! ( $($arg)* );
        }
    )
);
