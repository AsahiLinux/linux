// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Per-SoC hardware configuration structures
//!
//! This module contains the definitions used to store per-GPU and per-SoC configuration data.

use crate::driver::AsahiDevice;
use crate::fw::types::*;
use alloc::vec::Vec;
use kernel::c_str;
use kernel::device::RawDevice;
use kernel::prelude::*;

const MAX_POWERZONES: usize = 5;

pub(crate) mod t600x;
pub(crate) mod t602x;
pub(crate) mod t8103;
pub(crate) mod t8112;

/// GPU generation enumeration. Note: Part of the UABI.
#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(u32)]
pub(crate) enum GpuGen {
    G13 = 13,
    G14 = 14,
}

/// GPU variant enumeration. Note: Part of the UABI.
#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(u32)]
pub(crate) enum GpuVariant {
    P = 'P' as u32,
    G = 'G' as u32,
    S = 'S' as u32,
    C = 'C' as u32,
    D = 'D' as u32,
}

/// GPU revision enumeration. Note: Part of the UABI.
#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(u32)]
pub(crate) enum GpuRevision {
    A0 = 0x00,
    A1 = 0x01,
    B0 = 0x10,
    B1 = 0x11,
    C0 = 0x20,
    C1 = 0x21,
}

/// GPU core type enumeration. Note: Part of the firmware ABI.
#[derive(Debug, Copy, Clone)]
#[repr(u32)]
pub(crate) enum GpuCore {
    // Unknown = 0,
    // G5P = 1,
    // G5G = 2,
    // G9P = 3,
    // G9G = 4,
    // G10P = 5,
    // G11P = 6,
    // G11M = 7,
    // G11G = 8,
    // G12P = 9,
    // G13P = 10,
    G13G = 11,
    G13S = 12,
    G13C = 13,
    // G14P = 14,
    G14G = 15,
    G14S = 16,
    G14C = 17,
    G14D = 18, // Split out, unlike G13D
}

/// GPU revision ID. Note: Part of the firmware ABI.
#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(u32)]
pub(crate) enum GpuRevisionID {
    // Unknown = 0,
    A0 = 1,
    A1 = 2,
    B0 = 3,
    B1 = 4,
    C0 = 5,
    C1 = 6,
}

/// GPU driver/hardware features, from the UABI.
pub(crate) mod feat {
    /// Backwards-compatible features.
    pub(crate) mod compat {}

    /// Backwards-incompatible features.
    pub(crate) mod incompat {
        use kernel::uapi;

        /// Hardware requires Z/S compression to be mandatorily enabled.
        pub(crate) const MANDATORY_ZS_COMPRESSION: u64 =
            uapi::drm_asahi_feat_incompat_DRM_ASAHI_FEAT_MANDATORY_ZS_COMPRESSION as u64;
    }
}

/// A single performance state of the GPU.
#[derive(Debug)]
pub(crate) struct PState {
    /// Voltage in millivolts, per GPU cluster.
    pub(crate) volt_mv: Vec<u32>,
    /// Frequency in hertz.
    pub(crate) freq_hz: u32,
    /// Maximum power consumption of the GPU at this pstate, in milliwatts.
    pub(crate) pwr_mw: u32,
}

impl PState {
    pub(crate) fn max_volt_mv(&self) -> u32 {
        *self.volt_mv.iter().max().expect("No voltages")
    }
}

/// A power zone definition (we have no idea what this is but Apple puts them in the DT).
#[allow(missing_docs)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct PowerZone {
    pub(crate) target: u32,
    pub(crate) target_offset: u32,
    pub(crate) filter_tc: u32,
}

/// An MMIO mapping used by the firmware.
#[derive(Debug, Copy, Clone)]
pub(crate) struct IOMapping {
    /// Base physical address of the mapping.
    pub(crate) base: usize,
    /// Whether this mapping should be replicated to all dies
    pub(crate) per_die: bool,
    /// Number of mappings.
    pub(crate) count: usize,
    /// Size of one mapping.
    pub(crate) size: usize,
    /// Stride between mappings.
    pub(crate) stride: usize,
    /// Whether the mapping should be writable.
    pub(crate) writable: bool,
}

impl IOMapping {
    /// Convenience constructor for a new IOMapping.
    pub(crate) const fn new(
        base: usize,
        per_die: bool,
        count: usize,
        size: usize,
        stride: usize,
        writable: bool,
    ) -> IOMapping {
        IOMapping {
            base,
            per_die,
            count,
            size,
            stride,
            writable,
        }
    }
}

/// Unknown HwConfigA fields that vary from SoC to SoC.
#[allow(missing_docs)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct HwConfigA {
    pub(crate) unk_87c: i32,
    pub(crate) unk_8cc: u32,
    pub(crate) unk_e24: u32,
}

/// Unknown HwConfigB fields that vary from SoC to SoC.
#[allow(missing_docs)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct HwConfigB {
    pub(crate) unk_454: u32,
    pub(crate) unk_4e0: u64,
    pub(crate) unk_534: u32,
    pub(crate) unk_ab8: u32,
    pub(crate) unk_abc: u32,
    pub(crate) unk_b30: u32,
}

/// Render command configs that vary from SoC to SoC.
#[derive(Debug, Copy, Clone)]
pub(crate) struct HwRenderConfig {
    /// Vertex/tiling-related configuration register (lsb: disable clustering)
    pub(crate) tiling_control: u32,
}

#[derive(Debug)]
pub(crate) struct HwConfigShared2Curves {
    pub(crate) t1_coef: u32,
    pub(crate) t2: &'static [i16],
    pub(crate) t3_coefs: &'static [u32],
    pub(crate) t3_scales: &'static [u32],
}

/// Static hardware clustering configuration for multi-cluster SoCs.
#[derive(Debug)]
pub(crate) struct HwClusteringConfig {
    pub(crate) meta1_blocksize: usize,
    pub(crate) meta2_size: usize,
    pub(crate) meta3_size: usize,
    pub(crate) meta4_size: usize,
    pub(crate) max_splits: usize,
}

/// Static hardware configuration for a given SoC model.
#[derive(Debug)]
pub(crate) struct HwConfig {
    /// Chip ID in hex format (e.g. 0x8103 for t8103).
    pub(crate) chip_id: u32,
    /// GPU generation.
    pub(crate) gpu_gen: GpuGen,
    /// GPU variant type.
    pub(crate) gpu_variant: GpuVariant,
    /// GPU core type ID (as known by the firmware).
    pub(crate) gpu_core: GpuCore,
    /// Compatible feature bitmask for this GPU.
    pub(crate) gpu_feat_compat: u64,
    /// Incompatible feature bitmask for this GPU.
    pub(crate) gpu_feat_incompat: u64,

    /// Base clock used used for timekeeping.
    pub(crate) base_clock_hz: u32,
    /// Output address space for the UAT on this SoC.
    pub(crate) uat_oas: usize,
    /// Number of dies on this SoC.
    pub(crate) num_dies: u32,
    /// Maximum number of clusters on this SoC.
    pub(crate) max_num_clusters: u32,
    /// Maximum number of cores per cluster for this GPU.
    pub(crate) max_num_cores: u32,
    /// Maximum number of frags per cluster for this GPU.
    pub(crate) max_num_frags: u32,
    /// Maximum number of GPs per cluster for this GPU.
    pub(crate) max_num_gps: u32,

    /// Required size of the first preemption buffer.
    pub(crate) preempt1_size: usize,
    /// Required size of the second preemption buffer.
    pub(crate) preempt2_size: usize,
    /// Required size of the third preemption buffer.
    pub(crate) preempt3_size: usize,

    /// Required size of the compute preemption buffer.
    pub(crate) compute_preempt1_size: usize,

    pub(crate) clustering: Option<HwClusteringConfig>,

    /// Rendering-relevant configuration.
    pub(crate) render: HwRenderConfig,

    /// Misc HWDataA field values.
    pub(crate) da: HwConfigA,
    /// Misc HWDataB field values.
    pub(crate) db: HwConfigB,
    /// HwDataShared1.table.
    pub(crate) shared1_tab: &'static [i32],
    /// HwDataShared1.unk_a4.
    pub(crate) shared1_a4: u32,
    /// HwDataShared2.table.
    pub(crate) shared2_tab: &'static [i32],
    /// HwDataShared2.unk_508.
    pub(crate) shared2_unk_508: u32,
    /// HwDataShared2.unk_508.
    pub(crate) shared2_curves: Option<HwConfigShared2Curves>,

    /// HwDataShared3.unk_8.
    pub(crate) shared3_unk: u32,
    /// HwDataShared3.table.
    pub(crate) shared3_tab: &'static [u32],

    /// Globals.idle_off_standby_timer.
    pub(crate) idle_off_standby_timer_default: u32,
    /// Globals.unk_hws2_4.
    pub(crate) unk_hws2_4: Option<[F32; 8]>,
    /// Globals.unk_hws2_24.
    pub(crate) unk_hws2_24: u32,
    /// Globals.unk_54
    pub(crate) global_unk_54: u16,

    /// Constant related to SRAM voltages.
    pub(crate) sram_k: F32,
    /// Unknown per-cluster coefficients 1.
    pub(crate) unk_coef_a: &'static [&'static [F32]],
    /// Unknown per-cluster coefficients 2.
    pub(crate) unk_coef_b: &'static [&'static [F32]],
    /// Unknown table in Global struct.
    pub(crate) global_tab: Option<&'static [u8]>,
    /// Whether this GPU has CS/AFR performance states
    pub(crate) has_csafr: bool,

    /// Temperature sensor list (8 bits per sensor).
    pub(crate) fast_sensor_mask: [u64; 2],
    /// Temperature sensor list (alternate).
    pub(crate) fast_sensor_mask_alt: [u64; 2],
    /// Temperature sensor present bitmask.
    pub(crate) fast_die0_sensor_present: u32,
    /// Required MMIO mappings for this GPU/firmware.
    pub(crate) io_mappings: &'static [Option<IOMapping>],
    /// SRAM base
    pub(crate) sram_base: Option<u64>,
    /// SRAM size
    pub(crate) sram_size: Option<u64>,
}

/// Dynamic (fetched from hardware/DT) configuration.
#[derive(Debug)]
pub(crate) struct DynConfig {
    /// Base physical address of the UAT TTB (from DT reserved memory region).
    pub(crate) uat_ttb_base: u64,
    /// GPU ID configuration read from hardware.
    pub(crate) id: GpuIdConfig,
    /// Power calibration configuration for this specific chip/device.
    pub(crate) pwr: PwrConfig,
    /// Firmware version.
    pub(crate) firmware_version: Vec<u32>,
}

/// Specific GPU ID configuration fetched from SGX MMIO registers.
#[derive(Debug)]
pub(crate) struct GpuIdConfig {
    /// GPU generation (should match static config).
    pub(crate) gpu_gen: GpuGen,
    /// GPU variant type (should match static config).
    pub(crate) gpu_variant: GpuVariant,
    /// GPU silicon revision.
    pub(crate) gpu_rev: GpuRevision,
    /// GPU silicon revision ID (firmware enum).
    pub(crate) gpu_rev_id: GpuRevisionID,
    /// Total number of GPU clusters.
    pub(crate) num_clusters: u32,
    /// Maximum number of GPU cores per cluster.
    pub(crate) num_cores: u32,
    /// Number of frags per cluster.
    pub(crate) num_frags: u32,
    /// Number of GPs per cluster.
    pub(crate) num_gps: u32,
    /// Total number of active cores for the whole GPU.
    pub(crate) total_active_cores: u32,
    /// Mask of active cores per cluster.
    pub(crate) core_masks: Vec<u32>,
    /// Packed mask of all active cores.
    pub(crate) core_masks_packed: Vec<u32>,
}

/// Configurable CS/AFR GPU power settings from the device tree.
#[derive(Debug)]
pub(crate) struct CsAfrPwrConfig {
    /// GPU CS performance state list.
    pub(crate) perf_states_cs: Vec<PState>,
    /// GPU AFR performance state list.
    pub(crate) perf_states_afr: Vec<PState>,

    /// CS leakage coefficient per die.
    pub(crate) leak_coef_cs: Vec<F32>,
    /// AFR leakage coefficient per die.
    pub(crate) leak_coef_afr: Vec<F32>,

    /// Minimum voltage for the CS/AFR SRAM power domain in microvolts.
    pub(crate) min_sram_microvolt: u32,
}

/// Configurable GPU power settings from the device tree.
#[derive(Debug)]
pub(crate) struct PwrConfig {
    /// GPU performance state list.
    pub(crate) perf_states: Vec<PState>,
    /// GPU power zone list.
    pub(crate) power_zones: Vec<PowerZone>,

    /// Core leakage coefficient per cluster.
    pub(crate) core_leak_coef: Vec<F32>,
    /// SRAM leakage coefficient per cluster.
    pub(crate) sram_leak_coef: Vec<F32>,

    pub(crate) csafr: Option<CsAfrPwrConfig>,

    /// Maximum total power of the GPU in milliwatts.
    pub(crate) max_power_mw: u32,
    /// Maximum frequency of the GPU in megahertz.
    pub(crate) max_freq_mhz: u32,

    /// Minimum performance state to start at.
    pub(crate) perf_base_pstate: u32,
    /// Maximum enabled performance state.
    pub(crate) perf_max_pstate: u32,

    /// Minimum voltage for the SRAM power domain in microvolts.
    pub(crate) min_sram_microvolt: u32,

    // Most of these fields are just named after Apple ADT property names and we don't fully
    // understand them. They configure various power-related PID loops and filters.
    /// Average power filter time constant in milliseconds.
    pub(crate) avg_power_filter_tc_ms: u32,
    /// Average power filter PID integral gain?
    pub(crate) avg_power_ki_only: F32,
    /// Average power filter PID proportional gain?
    pub(crate) avg_power_kp: F32,
    pub(crate) avg_power_min_duty_cycle: u32,
    /// Average power target filter time constant in periods.
    pub(crate) avg_power_target_filter_tc: u32,
    /// "Fast die0" (temperature?) PID integral gain.
    pub(crate) fast_die0_integral_gain: F32,
    /// "Fast die0" (temperature?) PID proportional gain.
    pub(crate) fast_die0_proportional_gain: F32,
    pub(crate) fast_die0_prop_tgt_delta: u32,
    pub(crate) fast_die0_release_temp: u32,
    /// Delay from the fender (?) becoming idle to powerdown
    pub(crate) fender_idle_off_delay_ms: u32,
    /// Timeout from firmware early wake to sleep if no work was submitted (?)
    pub(crate) fw_early_wake_timeout_ms: u32,
    /// Delay from the GPU becoming idle to powerdown
    pub(crate) idle_off_delay_ms: u32,
    /// Related to the above?
    pub(crate) idle_off_standby_timer: u32,
    /// Percent?
    pub(crate) perf_boost_ce_step: u32,
    /// Minimum utilization before performance state is increased in %.
    pub(crate) perf_boost_min_util: u32,
    pub(crate) perf_filter_drop_threshold: u32,
    /// Performance PID filter time constant? (periods?)
    pub(crate) perf_filter_time_constant: u32,
    /// Performance PID filter time constant 2? (periods?)
    pub(crate) perf_filter_time_constant2: u32,
    /// Performance PID integral gain.
    pub(crate) perf_integral_gain: F32,
    /// Performance PID integral gain 2 (?).
    pub(crate) perf_integral_gain2: F32,
    pub(crate) perf_integral_min_clamp: u32,
    /// Performance PID proportional gain.
    pub(crate) perf_proportional_gain: F32,
    /// Performance PID proportional gain 2 (?).
    pub(crate) perf_proportional_gain2: F32,
    pub(crate) perf_reset_iters: u32,
    /// Target GPU utilization for the performance controller in %.
    pub(crate) perf_tgt_utilization: u32,
    /// Power sampling period in milliseconds.
    pub(crate) power_sample_period: u32,
    /// PPM (?) filter time constant in milliseconds.
    pub(crate) ppm_filter_time_constant_ms: u32,
    /// PPM (?) filter PID integral gain.
    pub(crate) ppm_ki: F32,
    /// PPM (?) filter PID proportional gain.
    pub(crate) ppm_kp: F32,
    /// Power consumption filter time constant (periods?)
    pub(crate) pwr_filter_time_constant: u32,
    /// Power consumption filter PID integral gain.
    pub(crate) pwr_integral_gain: F32,
    pub(crate) pwr_integral_min_clamp: u32,
    pub(crate) pwr_min_duty_cycle: u32,
    pub(crate) pwr_proportional_gain: F32,
    /// Power sample period in base clocks, used when not an integer number of ms
    pub(crate) pwr_sample_period_aic_clks: u32,

    pub(crate) se_engagement_criteria: i32,
    pub(crate) se_filter_time_constant: u32,
    pub(crate) se_filter_time_constant_1: u32,
    pub(crate) se_inactive_threshold: u32,
    pub(crate) se_ki: F32,
    pub(crate) se_ki_1: F32,
    pub(crate) se_kp: F32,
    pub(crate) se_kp_1: F32,
    pub(crate) se_reset_criteria: u32,
}

impl PwrConfig {
    fn load_opp(
        dev: &AsahiDevice,
        name: &CStr,
        cfg: &HwConfig,
        is_main: bool,
    ) -> Result<Vec<PState>> {
        let mut perf_states = Vec::new();

        let node = dev.of_node().ok_or(EIO)?;
        let opps = node.parse_phandle(name, 0).ok_or(EIO)?;

        for opp in opps.children() {
            let freq_hz: u64 = opp.get_property(c_str!("opp-hz"))?;
            let mut volt_uv: Vec<u32> = opp.get_property(c_str!("opp-microvolt"))?;
            let pwr_uw: u32 = if is_main {
                opp.get_property(c_str!("opp-microwatt"))?
            } else {
                0
            };

            let voltage_count = if is_main {
                cfg.max_num_clusters
            } else {
                cfg.num_dies
            };

            if volt_uv.len() != voltage_count as usize {
                dev_err!(
                    dev,
                    "Invalid opp-microvolt length (expected {}, got {})\n",
                    voltage_count,
                    volt_uv.len()
                );
                return Err(EINVAL);
            }

            volt_uv.iter_mut().for_each(|a| *a /= 1000);
            let volt_mv = volt_uv;

            let pwr_mw = pwr_uw / 1000;

            perf_states.try_push(PState {
                freq_hz: freq_hz.try_into()?,
                volt_mv,
                pwr_mw,
            })?;
        }

        if perf_states.is_empty() {
            Err(EINVAL)
        } else {
            Ok(perf_states)
        }
    }

    /// Load the GPU power configuration from the device tree.
    pub(crate) fn load(dev: &AsahiDevice, cfg: &HwConfig) -> Result<PwrConfig> {
        let perf_states = Self::load_opp(dev, c_str!("operating-points-v2"), cfg, true)?;
        let node = dev.of_node().ok_or(EIO)?;

        macro_rules! prop {
            ($prop:expr, $default:expr) => {{
                node.get_opt_property(c_str!($prop))
                    .map_err(|e| {
                        dev_err!(dev, "Error reading property {}: {:?}\n", $prop, e);
                        e
                    })?
                    .unwrap_or($default)
            }};
            ($prop:expr) => {{
                node.get_property(c_str!($prop)).map_err(|e| {
                    dev_err!(dev, "Error reading property {}: {:?}\n", $prop, e);
                    e
                })?
            }};
        }

        let pz_data = prop!("apple,power-zones", Vec::new());

        if pz_data.len() > 3 * MAX_POWERZONES || pz_data.len() % 3 != 0 {
            dev_err!(dev, "Invalid apple,power-zones value\n");
            return Err(EINVAL);
        }

        let pz_count = pz_data.len() / 3;
        let mut power_zones = Vec::new();
        for i in (0..pz_count).step_by(3) {
            power_zones.try_push(PowerZone {
                target: pz_data[i],
                target_offset: pz_data[i + 1],
                filter_tc: pz_data[i + 2],
            })?;
        }

        let core_leak_coef: Vec<F32> = prop!("apple,core-leak-coef");
        let sram_leak_coef: Vec<F32> = prop!("apple,sram-leak-coef");

        if core_leak_coef.len() != cfg.max_num_clusters as usize {
            dev_err!(dev, "Invalid apple,core-leak-coef\n");
            return Err(EINVAL);
        }
        if sram_leak_coef.len() != cfg.max_num_clusters as usize {
            dev_err!(dev, "Invalid apple,sram_leak_coef\n");
            return Err(EINVAL);
        }

        let csafr = if cfg.has_csafr {
            Some(CsAfrPwrConfig {
                perf_states_cs: Self::load_opp(dev, c_str!("apple,cs-opp"), cfg, false)?,
                perf_states_afr: Self::load_opp(dev, c_str!("apple,afr-opp"), cfg, false)?,
                leak_coef_cs: prop!("apple,cs-leak-coef"),
                leak_coef_afr: prop!("apple,afr-leak-coef"),
                min_sram_microvolt: prop!("apple,csafr-min-sram-microvolt"),
            })
        } else {
            None
        };

        let power_sample_period: u32 = prop!("apple,power-sample-period");

        Ok(PwrConfig {
            core_leak_coef,
            sram_leak_coef,

            max_power_mw: perf_states.iter().map(|a| a.pwr_mw).max().unwrap(),
            max_freq_mhz: perf_states.iter().map(|a| a.freq_hz).max().unwrap() / 1_000_000,

            perf_base_pstate: prop!("apple,perf-base-pstate", 1),
            perf_max_pstate: perf_states.len() as u32 - 1,
            min_sram_microvolt: prop!("apple,min-sram-microvolt"),

            avg_power_filter_tc_ms: prop!("apple,avg-power-filter-tc-ms"),
            avg_power_ki_only: prop!("apple,avg-power-ki-only"),
            avg_power_kp: prop!("apple,avg-power-kp"),
            avg_power_min_duty_cycle: prop!("apple,avg-power-min-duty-cycle"),
            avg_power_target_filter_tc: prop!("apple,avg-power-target-filter-tc"),
            fast_die0_integral_gain: prop!("apple,fast-die0-integral-gain"),
            fast_die0_proportional_gain: prop!("apple,fast-die0-proportional-gain"),
            fast_die0_prop_tgt_delta: prop!("apple,fast-die0-prop-tgt-delta", 0),
            fast_die0_release_temp: prop!("apple,fast-die0-release-temp", 80),
            fender_idle_off_delay_ms: prop!("apple,fender-idle-off-delay-ms", 40),
            fw_early_wake_timeout_ms: prop!("apple,fw-early-wake-timeout-ms", 5),
            idle_off_delay_ms: prop!("apple,idle-off-delay-ms", 2),
            idle_off_standby_timer: prop!(
                "apple,idleoff-standby-timer",
                cfg.idle_off_standby_timer_default
            ),
            perf_boost_ce_step: prop!("apple,perf-boost-ce-step", 25),
            perf_boost_min_util: prop!("apple,perf-boost-min-util", 100),
            perf_filter_drop_threshold: prop!("apple,perf-filter-drop-threshold"),
            perf_filter_time_constant2: prop!("apple,perf-filter-time-constant2"),
            perf_filter_time_constant: prop!("apple,perf-filter-time-constant"),
            perf_integral_gain2: prop!("apple,perf-integral-gain2"),
            perf_integral_gain: prop!("apple,perf-integral-gain", f32!(7.8956833)),
            perf_integral_min_clamp: prop!("apple,perf-integral-min-clamp"),
            perf_proportional_gain2: prop!("apple,perf-proportional-gain2"),
            perf_proportional_gain: prop!("apple,perf-proportional-gain", f32!(14.707963)),
            perf_reset_iters: prop!("apple,perf-reset-iters", 6),
            perf_tgt_utilization: prop!("apple,perf-tgt-utilization"),
            power_sample_period,
            ppm_filter_time_constant_ms: prop!("apple,ppm-filter-time-constant-ms"),
            ppm_ki: prop!("apple,ppm-ki"),
            ppm_kp: prop!("apple,ppm-kp"),
            pwr_filter_time_constant: prop!("apple,pwr-filter-time-constant", 313),
            pwr_integral_gain: prop!("apple,pwr-integral-gain", f32!(0.0202129)),
            pwr_integral_min_clamp: prop!("apple,pwr-integral-min-clamp", 0),
            pwr_min_duty_cycle: prop!("apple,pwr-min-duty-cycle"),
            pwr_proportional_gain: prop!("apple,pwr-proportional-gain", f32!(5.2831855)),
            pwr_sample_period_aic_clks: prop!(
                "apple,pwr-sample-period-aic-clks",
                cfg.base_clock_hz / 1000 * power_sample_period
            ),
            se_engagement_criteria: prop!("apple,se-engagement-criteria", -1),
            se_filter_time_constant: prop!("apple,se-filter-time-constant", 9),
            se_filter_time_constant_1: prop!("apple,se-filter-time-constant-1", 3),
            se_inactive_threshold: prop!("apple,se-inactive-threshold", 2500),
            se_ki: prop!("apple,se-ki", f32!(-50.0)),
            se_ki_1: prop!("apple,se-ki-1", f32!(-100.0)),
            se_kp: prop!("apple,se-kp", f32!(-5.0)),
            se_kp_1: prop!("apple,se-kp-1", f32!(-10.0)),
            se_reset_criteria: prop!("apple,se-reset-criteria", 50),

            perf_states,
            power_zones,
            csafr,
        })
    }

    pub(crate) fn min_frequency_khz(&self) -> u32 {
        self.perf_states[self.perf_base_pstate as usize].freq_hz / 1000
    }

    pub(crate) fn max_frequency_khz(&self) -> u32 {
        self.perf_states[self.perf_max_pstate as usize].freq_hz / 1000
    }
}
