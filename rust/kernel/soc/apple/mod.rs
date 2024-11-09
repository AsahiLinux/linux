// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple SoC drivers

#[cfg(CONFIG_APPLE_RTKIT = "y")]
pub mod rtkit;

#[cfg(any(CONFIG_APPLE_AOP = "y", CONFIG_APPLE_AOP = "m"))]
pub mod aop;
