// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple SoC drivers

#[cfg(CONFIG_RUST_APPLE_RTKIT = "y")]
pub mod rtkit;

#[cfg(any(CONFIG_APPLE_AOP = "y", CONFIG_APPLE_AOP = "m"))]
pub mod aop;

#[cfg(CONFIG_RUST_APPLE_MAILBOX = "y")]
pub mod mailbox;

#[cfg(CONFIG_APPLE_PMP_V1 = "y")]
pub mod pmp_v1_bridge;
#[cfg(CONFIG_APPLE_PMP_V1_REPORT = "y")]
pub mod pmdomain;
