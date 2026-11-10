// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Power domain abstractions.

use crate::{
    bindings,
    device::{Device, property::FwNode},
    error::{to_result, Result},
    of,
};

/// Binding for `generic_pm_domain`
pub type GenericPmDomain = bindings::generic_pm_domain;
/// Binding for `of_phandle_args`
pub type OfPhandleArgs = bindings::of_phandle_args;

/// Binding for `MAX_PHANDLE_ARGS`
pub const MAX_PHANDLE_ARGS: u32 = bindings::MAX_PHANDLE_ARGS;
/// Binding for `GENPD_FLAG_ACTIVE_WAKEUP`
pub const GENPD_FLAG_ACTIVE_WAKEUP: u32 = bindings::GENPD_FLAG_ACTIVE_WAKEUP;

/// Removes a device from its generic PM domain.
pub fn pm_genpd_remove_device(dev: &Device) -> Result {
    // SAFETY: `dev` is valid.
    to_result(unsafe { bindings::pm_genpd_remove_device(dev.as_raw()) })
}

/// Initialize a generic PM domain.
pub fn pm_genpd_init(
    genpd: *mut GenericPmDomain,
    gov: Option<*mut bindings::dev_power_governor>,
    is_off: bool,
) -> Result {
    // SAFETY: caller guarantees `genpd` and `gov` are valid.
    to_result(
        unsafe { bindings::pm_genpd_init(genpd, gov.unwrap_or(core::ptr::null_mut()), is_off) }
    )
}

/// Remove a generic PM domain.
pub fn pm_genpd_remove(genpd: *mut GenericPmDomain) -> Result {
    // SAFETY: Caller guarantees `genpd` is valid.
    to_result(unsafe { bindings::pm_genpd_remove(genpd) })
}

/// Adds a simple OF generic PM domain provider.
pub fn of_genpd_add_provider_simple(node: &FwNode, genpd: *mut GenericPmDomain) -> Result {
    // SAFETY: Caller guarantees `genpd` is valid, and `node` is valid.
    to_result(unsafe { bindings::of_genpd_add_provider_simple(of::to_of_node(node).cast_mut(), genpd) })
}

/// Removes an OF generic PM domain provider.
pub fn of_genpd_del_provider(node: &FwNode) {
    // SAFETY: `node` is valid.
    unsafe { bindings::of_genpd_del_provider(of::to_of_node(node).cast_mut()) };
}

/// Add a subdomain from phandle args.
pub fn of_genpd_add_subdomain(
    parent_spec: &OfPhandleArgs,
    subdomain_spec: &OfPhandleArgs,
) -> Result {
    // SAFETY: Caller guarantees `parent_spec` and `subdomain_spec` are valid.
    to_result(unsafe { bindings::of_genpd_add_subdomain(parent_spec, subdomain_spec) })
}
