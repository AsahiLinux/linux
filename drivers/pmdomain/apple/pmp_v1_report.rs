// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple SoC PMPv1 power state reporting driver

use kernel::{
    container_of,
    device::{
        self,
        Core,
    },
    module_platform_driver,
    of,
    platform,
    prelude::*,
    soc::apple::{
        pmdomain,
        pmp_v1_bridge
    },
    sync::{Arc, aref::ARef},
    str::CString,
    types::{ScopeGuard, ForeignOwnable},
};

#[repr(C)]
struct ReportEntry {
    label: CString,
    dev: ARef<device::Device>,
    pub genpd: pmdomain::GenericPmDomain,
    id: u32,
}

impl ReportEntry {
    fn genpd_ptr(&self) -> *mut pmdomain::GenericPmDomain {
        &raw const self.genpd as *mut _
    }

    unsafe fn from_genpd<'a>(genpd: *mut pmdomain::GenericPmDomain) -> &'a mut Self {
        unsafe { &mut *container_of!(genpd, ReportEntry, genpd) }
    }
}

#[pin_data]
struct PmpReportData {
    #[pin]
    dev: ARef<device::Device>,
    #[pin]
    entry: Pin<KBox<ReportEntry>>,
}

impl PmpReportData {
    fn new(pdev: &platform::Device<Core<'_>>,
        entry: Pin<KBox<ReportEntry>>) -> Result<Arc<Self>>
    {
        Arc::pin_init(
            try_pin_init!(
                PmpReportData {
                    dev: pdev.as_ref().into(),
                    entry
                }
            ),
            GFP_KERNEL,
        )
    }
}

unsafe impl Send for PmpReportData {}
unsafe impl Sync for PmpReportData {}

struct PmpV1ReportDriver(Arc<PmpReportData>);

impl Drop for PmpV1ReportDriver {
    fn drop(&mut self) {
        let node = self.0.dev.fwnode().unwrap();
        pmdomain::of_genpd_del_provider(&node);

        let genpd = self.0.entry.genpd_ptr();
        let _ = pmdomain::pm_genpd_remove(genpd);
    }
}

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    (),
    [(of::DeviceId::new(c"apple,t8103-pmp-v1-report-entry"), ())]
);

unsafe extern "C" fn report_entry_set_state(
    genpd: *mut pmdomain::GenericPmDomain,
    enable: bool
) -> c_int {
    let ent = unsafe { ReportEntry::from_genpd(genpd) };

    let parent = ent.dev.as_ref().parent().unwrap();

    dev_dbg!(ent.dev, "Setting state '{}' for device {}", enable, ent.id);

    // SAFETY: our parent is PmpDriver, and its repr(transparent) for Arc<dyn DevPwrBridge>
    let pdata_ptr = unsafe {
        Pin::<KBox<Arc<dyn pmp_v1_bridge::DevPwrBridge>>>::borrow(parent.get_drvdata())
    };
    let bridge = (&*pdata_ptr).clone();

    if !bridge.ready() {
        // TODO: put correct erro
        return 1;
    }

    dev_dbg!(ent.dev, "Bridge ready");

    // TODO: handle errors
    bridge.send_devpwr(ent.id as u64, enable).unwrap();

    0
}

unsafe extern "C" fn report_entry_power_on(genpd: *mut pmdomain::GenericPmDomain) -> c_int {
    unsafe { report_entry_set_state(genpd, true) }
}

unsafe extern "C" fn report_entry_power_off(genpd: *mut pmdomain::GenericPmDomain) -> c_int {
    unsafe { report_entry_set_state(genpd, false) }
}

impl platform::Driver for PmpV1ReportDriver {
    type IdInfo = ();
    type Data<'bound> = PmpV1ReportDriver;

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe(
        pdev: &platform::Device<Core<'_>>,
        _info: Option<&()>,
    ) -> impl PinInit<Self, Error> {
        let dev: ARef<device::Device> = pdev.as_ref().into();
        let node = dev.fwnode().ok_or(ENODEV)?;

        let id = node.property_read::<u32>(c"reg").required_by(&dev)?;

        let label = node.property_read::<CString>(c"label").required_by(&dev)?;

        let mut entry = KBox::into_pin(KBox::new(
            ReportEntry {
                label,
                dev: dev.clone(),
                // SAFETY: valid in C so also valid here.
                genpd: unsafe { core::mem::zeroed() },
                id,
            },
            GFP_KERNEL,
        )?);

        let entry_mut = unsafe {
            Pin::get_unchecked_mut(entry.as_mut())
        };

        let genpd = &raw mut entry_mut.genpd;

        if node.property_read_bool(c"apple,always-on") {
            entry_mut.genpd.flags |= pmdomain::GENPD_FLAG_ACTIVE_WAKEUP;
        }
        entry_mut.genpd.name = entry_mut.label.as_char_ptr();
        entry_mut.genpd.power_on = Some(report_entry_power_on);
        entry_mut.genpd.power_off = Some(report_entry_power_off);

        pmdomain::pm_genpd_init(genpd, None, true)?;
        pmdomain::of_genpd_add_provider_simple(node, genpd)?;

        let remove_device = ScopeGuard::new(|| {
            let _ = pmdomain::pm_genpd_remove_device(pdev.as_ref());
        });

        let fwnode_of_node = unsafe { of::to_of_node(node).cast_mut() };

        let mut index = 0;

        loop {
            let args = match node.property_get_reference_args(
                c"power-domains",
                device::property::NArgs::Prop(c"#power-domain-cells"),
                index
            ) {
                Ok(args) => args,
                Err(e) if e == ENOENT => break,
                Err(e) => return Err(e),
            };

            let mut parent_args = [0u32; pmdomain::MAX_PHANDLE_ARGS as usize];

            for (i, arg) in args.as_slice().into_iter().enumerate() {
                // `i` < MAX_PHANDLE_ARGS guaranteed by `property_get_reference_args`
                parent_args[i] = (*arg).try_into().unwrap();
            }

            let parent_spec = pmdomain::OfPhandleArgs {
                np: unsafe { of::to_of_node(args.fwnode().unwrap()).cast_mut() },
                args_count: args.len().try_into().unwrap(),
                args: parent_args
            };

            let subdomain_spec = pmdomain::OfPhandleArgs {
                np: fwnode_of_node,
                args_count: 0,
                args: [0; pmdomain::MAX_PHANDLE_ARGS as usize],
            };

            if let Err(e) = pmdomain::of_genpd_add_subdomain(&parent_spec, &subdomain_spec) {
                dev_err!(entry.dev, "failed to add to parent domain");
                return Err(e);
            }

            index += 1;
        }

        let data = PmpReportData::new(pdev, entry)?;

        remove_device.dismiss();

        Ok(Self(data))
    }
}

module_platform_driver! {
    type: PmpV1ReportDriver,
    name: "apple_pmp_v1_report",
    license: "Dual MIT/GPL",
}
