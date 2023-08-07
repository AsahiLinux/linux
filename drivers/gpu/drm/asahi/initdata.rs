// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(clippy::unusual_byte_groupings)]

//! GPU initialization data builder.
//!
//! The root of all interaction between the GPU firmware and the host driver is a complex set of
//! nested structures that we call InitData. This includes both GPU hardware/firmware configuration
//! and the pointers to the ring buffers and global data fields that are used for communication at
//! runtime.
//!
//! Many of these structures are poorly understood, so there are lots of hardcoded unknown values
//! derived from observing the InitData structures that macOS generates.

use crate::f32;
use crate::fw::initdata::*;
use crate::fw::types::*;
use crate::{driver::AsahiDevice, gem, gpu, hw, mmu};
use alloc::vec::Vec;
use kernel::error::{Error, Result};
use kernel::macros::versions;
use kernel::{init, init::Init, try_init};

/// Builder helper for the global GPU InitData.
#[versions(AGX)]
pub(crate) struct InitDataBuilder<'a> {
    dev: &'a AsahiDevice,
    alloc: &'a mut gpu::KernelAllocators,
    cfg: &'static hw::HwConfig,
    dyncfg: &'a hw::DynConfig,
}

#[versions(AGX)]
impl<'a> InitDataBuilder::ver<'a> {
    /// Create a new InitData builder
    pub(crate) fn new(
        dev: &'a AsahiDevice,
        alloc: &'a mut gpu::KernelAllocators,
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
    ) -> InitDataBuilder::ver<'a> {
        InitDataBuilder::ver {
            dev,
            alloc,
            cfg,
            dyncfg,
        }
    }

    /// Create the HwDataShared1 structure, which is used in two places in InitData.
    fn hw_shared1(cfg: &'static hw::HwConfig) -> impl Init<raw::HwDataShared1> {
        init::chain(
            init!(raw::HwDataShared1 {
                unk_a4: cfg.shared1_a4,
                ..Zeroable::zeroed()
            }),
            |ret| {
                for (i, val) in cfg.shared1_tab.iter().enumerate() {
                    ret.table[i] = *val;
                }
                Ok(())
            },
        )
    }

    fn init_curve(
        curve: &mut raw::HwDataShared2Curve,
        unk_0: u32,
        unk_4: u32,
        t1: &[u16],
        t2: &[i16],
        t3: &[Vec<i32>],
    ) {
        curve.unk_0 = unk_0;
        curve.unk_4 = unk_4;
        (*curve.t1)[..t1.len()].copy_from_slice(t1);
        (*curve.t1)[t1.len()..].fill(t1[0]);
        (*curve.t2)[..t2.len()].copy_from_slice(t2);
        (*curve.t2)[t2.len()..].fill(t2[0]);
        for (i, a) in curve.t3.iter_mut().enumerate() {
            a.fill(0x3ffffff);
            if i < t3.len() {
                let b = &t3[i];
                (**a)[..b.len()].copy_from_slice(b);
            }
        }
    }

    /// Create the HwDataShared2 structure, which is used in two places in InitData.
    fn hw_shared2(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
    ) -> impl Init<raw::HwDataShared2, Error> + 'a {
        init::chain(
            try_init!(raw::HwDataShared2 {
                unk_28: Array::new([0xff; 16]),
                g14: Default::default(),
                unk_508: cfg.shared2_unk_508,
                ..Zeroable::zeroed()
            }),
            |ret| {
                for (i, val) in cfg.shared2_tab.iter().enumerate() {
                    ret.table[i] = *val;
                }

                let curve_cfg = match cfg.shared2_curves.as_ref() {
                    None => return Ok(()),
                    Some(a) => a,
                };

                let mut t1 = Vec::new();
                let mut t3 = Vec::new();

                for _ in 0..curve_cfg.t3_scales.len() {
                    t3.try_push(Vec::new())?;
                }

                for (i, ps) in dyncfg.pwr.perf_states.iter().enumerate() {
                    let t3_coef = curve_cfg.t3_coefs[i];
                    if t3_coef == 0 {
                        t1.try_push(0xffff)?;
                        for j in t3.iter_mut() {
                            j.try_push(0x3ffffff)?;
                        }
                        continue;
                    }

                    let f_mhz = (ps.freq_hz / 1000) as u64;
                    let v_max = ps.max_volt_mv() as u64;

                    t1.try_push(
                        (1000000000 * (curve_cfg.t1_coef as u64) / (f_mhz * v_max))
                            .try_into()
                            .unwrap(),
                    )?;

                    for (j, scale) in curve_cfg.t3_scales.iter().enumerate() {
                        t3[j].try_push(
                            (t3_coef as u64 * 1000000000 * *scale as u64 / (f_mhz * v_max * 6))
                                .try_into()
                                .unwrap(),
                        )?;
                    }
                }

                ret.g14.unk_14 = 0x6000000;
                Self::init_curve(
                    &mut ret.g14.curve1,
                    0,
                    0x20000000,
                    &[0xffff],
                    &[0x0f07],
                    &[],
                );
                Self::init_curve(&mut ret.g14.curve2, 7, 0x80000000, &t1, curve_cfg.t2, &t3);

                Ok(())
            },
        )
    }

    /// Create the HwDataShared3 structure, which is used in two places in InitData.
    fn hw_shared3(cfg: &'static hw::HwConfig) -> impl Init<raw::HwDataShared3> {
        init::chain(init::zeroed::<raw::HwDataShared3>(), |ret| {
            if !cfg.shared3_tab.is_empty() {
                ret.unk_0 = 1;
                ret.unk_4 = 500;
                ret.unk_8 = cfg.shared3_unk;
                ret.table.copy_from_slice(cfg.shared3_tab);
                ret.unk_4c = 1;
            }
            Ok(())
        })
    }

    /// Create an unknown T81xx-specific data structure.
    fn t81xx_data(
        cfg: &'static hw::HwConfig,
        dyncfg: &'a hw::DynConfig,
    ) -> impl Init<raw::T81xxData> {
        let _perf_max_pstate = dyncfg.pwr.perf_max_pstate;

        init::chain(init::zeroed::<raw::T81xxData>(), move |_ret| {
            match cfg.chip_id {
                0x8103 | 0x8112 => {
                    #[ver(V < V13_3)]
                    {
                        _ret.unk_d8c = 0x80000000;
                        _ret.unk_d90 = 4;
                        _ret.unk_d9c = f32!(0.6);
                        _ret.unk_da4 = f32!(0.4);
                        _ret.unk_dac = f32!(0.38552);
                        _ret.unk_db8 = f32!(65536.0);
                        _ret.unk_dbc = f32!(13.56);
                        _ret.max_pstate_scaled = 100 * _perf_max_pstate;
                    }
                }
                _ => (),
            }
            Ok(())
        })
    }

    /// Create the HwDataA structure. This mostly contains power-related configuration.
    fn hwdata_a(&mut self) -> Result<GpuObject<HwDataA::ver>> {
        let pwr = &self.dyncfg.pwr;
        let period_ms = pwr.power_sample_period;
        let period_s = F32::from(period_ms) / f32!(1000.0);
        let ppm_filter_tc_periods = pwr.ppm_filter_time_constant_ms / period_ms;
        #[ver(V >= V13_0B4)]
        let ppm_filter_tc_ms_rounded = ppm_filter_tc_periods * period_ms;
        let ppm_filter_a = f32!(1.0) / ppm_filter_tc_periods.into();
        let perf_filter_a = f32!(1.0) / pwr.perf_filter_time_constant.into();
        let perf_filter_a2 = f32!(1.0) / pwr.perf_filter_time_constant2.into();
        let avg_power_target_filter_a = f32!(1.0) / pwr.avg_power_target_filter_tc.into();
        let avg_power_filter_tc_periods = pwr.avg_power_filter_tc_ms / period_ms;
        #[ver(V >= V13_0B4)]
        let avg_power_filter_tc_ms_rounded = avg_power_filter_tc_periods * period_ms;
        let avg_power_filter_a = f32!(1.0) / avg_power_filter_tc_periods.into();
        let pwr_filter_a = f32!(1.0) / pwr.pwr_filter_time_constant.into();

        let base_ps = pwr.perf_base_pstate;
        let base_ps_scaled = 100 * base_ps;
        let max_ps = pwr.perf_max_pstate;
        let max_ps_scaled = 100 * max_ps;
        let boost_ps_count = max_ps - base_ps;

        #[allow(unused_variables)]
        let base_clock_khz = self.cfg.base_clock_hz / 1000;
        let clocks_per_period = pwr.pwr_sample_period_aic_clks;

        #[allow(unused_variables)]
        let clocks_per_period_coarse = self.cfg.base_clock_hz / 1000 * pwr.power_sample_period;

        self.alloc.private.new_init(init::zeroed(), |_inner, _ptr| {
            let cfg = &self.cfg;
            let dyncfg = &self.dyncfg;
            init::chain(
                try_init!(raw::HwDataA::ver {
                    clocks_per_period: clocks_per_period,
                    #[ver(V >= V13_0B4)]
                    clocks_per_period_2: clocks_per_period,
                    pwr_status: AtomicU32::new(4),
                    unk_10: f32!(1.0),
                    actual_pstate: 1,
                    tgt_pstate: 1,
                    base_pstate_scaled: base_ps_scaled,
                    unk_40: 1,
                    max_pstate_scaled: max_ps_scaled,
                    min_pstate_scaled: 100,
                    unk_64c: 625,
                    pwr_filter_a_neg: f32!(1.0) - pwr_filter_a,
                    pwr_filter_a: pwr_filter_a,
                    pwr_integral_gain: pwr.pwr_integral_gain,
                    pwr_integral_min_clamp: pwr.pwr_integral_min_clamp.into(),
                    max_power_1: pwr.max_power_mw.into(),
                    pwr_proportional_gain: pwr.pwr_proportional_gain,
                    pwr_pstate_related_k: -F32::from(max_ps_scaled) / pwr.max_power_mw.into(),
                    pwr_pstate_max_dc_offset: pwr.pwr_min_duty_cycle as i32 - max_ps_scaled as i32,
                    max_pstate_scaled_2: max_ps_scaled,
                    max_power_2: pwr.max_power_mw,
                    max_pstate_scaled_3: max_ps_scaled,
                    ppm_filter_tc_periods_x4: ppm_filter_tc_periods * 4,
                    ppm_filter_a_neg: f32!(1.0) - ppm_filter_a,
                    ppm_filter_a: ppm_filter_a,
                    ppm_ki_dt: pwr.ppm_ki * period_s,
                    unk_6fc: f32!(65536.0),
                    ppm_kp: pwr.ppm_kp,
                    pwr_min_duty_cycle: pwr.pwr_min_duty_cycle,
                    max_pstate_scaled_4: max_ps_scaled,
                    unk_71c: f32!(0.0),
                    max_power_3: pwr.max_power_mw,
                    cur_power_mw_2: 0x0,
                    ppm_filter_tc_ms: pwr.ppm_filter_time_constant_ms,
                    #[ver(V >= V13_0B4)]
                    ppm_filter_tc_clks: ppm_filter_tc_ms_rounded * base_clock_khz,
                    perf_tgt_utilization: pwr.perf_tgt_utilization,
                    perf_boost_min_util: pwr.perf_boost_min_util,
                    perf_boost_ce_step: pwr.perf_boost_ce_step,
                    perf_reset_iters: pwr.perf_reset_iters,
                    unk_774: 6,
                    unk_778: 1,
                    perf_filter_drop_threshold: pwr.perf_filter_drop_threshold,
                    perf_filter_a_neg: f32!(1.0) - perf_filter_a,
                    perf_filter_a2_neg: f32!(1.0) - perf_filter_a2,
                    perf_filter_a: perf_filter_a,
                    perf_filter_a2: perf_filter_a2,
                    perf_ki: pwr.perf_integral_gain,
                    perf_ki2: pwr.perf_integral_gain2,
                    perf_integral_min_clamp: pwr.perf_integral_min_clamp.into(),
                    unk_79c: f32!(95.0),
                    perf_kp: pwr.perf_proportional_gain,
                    perf_kp2: pwr.perf_proportional_gain2,
                    boost_state_unk_k: F32::from(boost_ps_count) / f32!(0.95),
                    base_pstate_scaled_2: base_ps_scaled,
                    max_pstate_scaled_5: max_ps_scaled,
                    base_pstate_scaled_3: base_ps_scaled,
                    perf_tgt_utilization_2: pwr.perf_tgt_utilization,
                    base_pstate_scaled_4: base_ps_scaled,
                    unk_7fc: f32!(65536.0),
                    pwr_min_duty_cycle_2: pwr.pwr_min_duty_cycle.into(),
                    max_pstate_scaled_6: max_ps_scaled.into(),
                    max_freq_mhz: pwr.max_freq_mhz,
                    pwr_min_duty_cycle_3: pwr.pwr_min_duty_cycle,
                    min_pstate_scaled_4: f32!(100.0),
                    max_pstate_scaled_7: max_ps_scaled,
                    unk_alpha_neg: f32!(0.8),
                    unk_alpha: f32!(0.2),
                    fast_die0_sensor_mask: U64(cfg.fast_sensor_mask[0]),
                    fast_die0_release_temp_cc: 100 * pwr.fast_die0_release_temp,
                    unk_87c: cfg.da.unk_87c,
                    unk_880: 0x4,
                    unk_894: f32!(1.0),

                    fast_die0_ki_dt: pwr.fast_die0_integral_gain * period_s,
                    unk_8a8: f32!(65536.0),
                    fast_die0_kp: pwr.fast_die0_proportional_gain,
                    pwr_min_duty_cycle_4: pwr.pwr_min_duty_cycle,
                    max_pstate_scaled_8: max_ps_scaled,
                    max_pstate_scaled_9: max_ps_scaled,
                    fast_die0_prop_tgt_delta: 100 * pwr.fast_die0_prop_tgt_delta,
                    unk_8cc: cfg.da.unk_8cc,
                    max_pstate_scaled_10: max_ps_scaled,
                    max_pstate_scaled_11: max_ps_scaled,
                    unk_c2c: 1,
                    power_zone_count: pwr.power_zones.len() as u32,
                    max_power_4: pwr.max_power_mw,
                    max_power_5: pwr.max_power_mw,
                    max_power_6: pwr.max_power_mw,
                    avg_power_target_filter_a_neg: f32!(1.0) - avg_power_target_filter_a,
                    avg_power_target_filter_a: avg_power_target_filter_a,
                    avg_power_target_filter_tc_x4: 4 * pwr.avg_power_target_filter_tc,
                    avg_power_target_filter_tc_xperiod: period_ms * pwr.avg_power_target_filter_tc,
                    #[ver(V >= V13_0B4)]
                    avg_power_target_filter_tc_clks: period_ms
                        * pwr.avg_power_target_filter_tc
                        * base_clock_khz,
                    avg_power_filter_tc_periods_x4: 4 * avg_power_filter_tc_periods,
                    avg_power_filter_a_neg: f32!(1.0) - avg_power_filter_a,
                    avg_power_filter_a: avg_power_filter_a,
                    avg_power_ki_dt: pwr.avg_power_ki_only * period_s,
                    unk_d20: f32!(65536.0),
                    avg_power_kp: pwr.avg_power_kp,
                    avg_power_min_duty_cycle: pwr.avg_power_min_duty_cycle,
                    max_pstate_scaled_12: max_ps_scaled,
                    max_pstate_scaled_13: max_ps_scaled,
                    max_power_7: pwr.max_power_mw.into(),
                    max_power_8: pwr.max_power_mw,
                    avg_power_filter_tc_ms: pwr.avg_power_filter_tc_ms,
                    #[ver(V >= V13_0B4)]
                    avg_power_filter_tc_clks: avg_power_filter_tc_ms_rounded * base_clock_khz,
                    max_pstate_scaled_14: max_ps_scaled,
                    t81xx_data <- Self::t81xx_data(cfg, dyncfg),
                    #[ver(V >= V13_0B4)]
                    unk_e10_0 <- {
                        let filter_a = f32!(1.0) / pwr.se_filter_time_constant.into();
                        let filter_1_a = f32!(1.0) / pwr.se_filter_time_constant_1.into();
                        try_init!(raw::HwDataA130Extra {
                            unk_38: 4,
                            unk_3c: 8000,
                            gpu_se_inactive_threshold: pwr.se_inactive_threshold,
                            gpu_se_engagement_criteria: pwr.se_engagement_criteria,
                            gpu_se_reset_criteria: pwr.se_reset_criteria,
                            unk_54: 50,
                            unk_58: 0x1,
                            gpu_se_filter_a_neg: f32!(1.0) - filter_a,
                            gpu_se_filter_1_a_neg: f32!(1.0) - filter_1_a,
                            gpu_se_filter_a: filter_a,
                            gpu_se_filter_1_a: filter_1_a,
                            gpu_se_ki_dt: pwr.se_ki * period_s,
                            gpu_se_ki_1_dt: pwr.se_ki_1 * period_s,
                            unk_7c: f32!(65536.0),
                            gpu_se_kp: pwr.se_kp,
                            gpu_se_kp_1: pwr.se_kp_1,

                            #[ver(V >= V13_3)]
                            unk_8c: 100,
                            #[ver(V < V13_3)]
                            unk_8c: 40,

                            max_pstate_scaled_1: max_ps_scaled,
                            unk_9c: f32!(8000.0),
                            unk_a0: 1400,
                            gpu_se_filter_time_constant_ms: pwr.se_filter_time_constant * period_ms,
                            gpu_se_filter_time_constant_1_ms: pwr.se_filter_time_constant_1
                                * period_ms,
                            gpu_se_filter_time_constant_clks: U64((pwr.se_filter_time_constant
                                * clocks_per_period_coarse)
                                .into()),
                            gpu_se_filter_time_constant_1_clks: U64((pwr
                                .se_filter_time_constant_1
                                * clocks_per_period_coarse)
                                .into()),
                            unk_c4: f32!(65536.0),
                            unk_114: f32!(65536.0),
                            unk_124: 40,
                            max_pstate_scaled_2: max_ps_scaled,
                            ..Zeroable::zeroed()
                        })
                    },
                    fast_die0_sensor_mask_2: U64(cfg.fast_sensor_mask[0]),
                    unk_e24: cfg.da.unk_e24,
                    unk_e28: 1,
                    fast_die0_sensor_mask_alt: U64(cfg.fast_sensor_mask_alt[0]),
                    #[ver(V < V13_0B4)]
                    fast_die0_sensor_present: U64(cfg.fast_die0_sensor_present as u64),
                    unk_163c: 1,
                    unk_3644: 0,
                    hws1 <- Self::hw_shared1(cfg),
                    hws2 <- Self::hw_shared2(cfg, dyncfg),
                    hws3 <- Self::hw_shared3(cfg),
                    unk_3ce8: 1,
                    ..Zeroable::zeroed()
                }),
                |raw| {
                    for i in 0..self.dyncfg.pwr.perf_states.len() {
                        raw.sram_k[i] = self.cfg.sram_k;
                    }

                    for (i, coef) in pwr.core_leak_coef.iter().enumerate() {
                        raw.core_leak_coef[i] = *coef;
                    }

                    for (i, coef) in pwr.sram_leak_coef.iter().enumerate() {
                        raw.sram_leak_coef[i] = *coef;
                    }

                    #[ver(V >= V13_0B4)]
                    if let Some(csafr) = pwr.csafr.as_ref() {
                        for (i, coef) in csafr.leak_coef_afr.iter().enumerate() {
                            raw.aux_leak_coef.cs_1[i] = *coef;
                            raw.aux_leak_coef.cs_2[i] = *coef;
                        }

                        for (i, coef) in csafr.leak_coef_cs.iter().enumerate() {
                            raw.aux_leak_coef.afr_1[i] = *coef;
                            raw.aux_leak_coef.afr_2[i] = *coef;
                        }
                    }

                    for i in 0..self.dyncfg.id.num_clusters as usize {
                        if let Some(coef_a) = self.cfg.unk_coef_a.get(i) {
                            (*raw.unk_coef_a1[i])[..coef_a.len()].copy_from_slice(coef_a);
                            (*raw.unk_coef_a2[i])[..coef_a.len()].copy_from_slice(coef_a);
                        }
                        if let Some(coef_b) = self.cfg.unk_coef_b.get(i) {
                            (*raw.unk_coef_b1[i])[..coef_b.len()].copy_from_slice(coef_b);
                            (*raw.unk_coef_b2[i])[..coef_b.len()].copy_from_slice(coef_b);
                        }
                    }

                    for (i, pz) in pwr.power_zones.iter().enumerate() {
                        raw.power_zones[i].target = pz.target;
                        raw.power_zones[i].target_off = pz.target - pz.target_offset;
                        raw.power_zones[i].filter_tc_x4 = 4 * pz.filter_tc;
                        raw.power_zones[i].filter_tc_xperiod = period_ms * pz.filter_tc;
                        let filter_a = f32!(1.0) / pz.filter_tc.into();
                        raw.power_zones[i].filter_a = filter_a;
                        raw.power_zones[i].filter_a_neg = f32!(1.0) - filter_a;
                        #[ver(V >= V13_0B4)]
                        raw.power_zones[i].unk_10 = 1320000000;
                    }

                    #[ver(V >= V13_0B4 && G >= G14X)]
                    for (i, j) in raw.hws2.g14.curve2.t1.iter().enumerate() {
                        raw.unk_hws2[i] = if *j == 0xffff { 0 } else { j / 2 };
                    }

                    Ok(())
                },
            )
        })
    }

    /// Create the HwDataB structure. This mostly contains GPU-related configuration.
    fn hwdata_b(&mut self) -> Result<GpuObject<HwDataB::ver>> {
        self.alloc.private.new_init(init::zeroed(), |_inner, _ptr| {
            let cfg = &self.cfg;
            let dyncfg = &self.dyncfg;
            init::chain(
                try_init!(raw::HwDataB::ver {
                    // Userspace VA map related
                    #[ver(V < V13_0B4)]
                    unk_0: U64(0x13_00000000),
                    unk_8: U64(0x14_00000000),
                    #[ver(V < V13_0B4)]
                    unk_10: U64(0x1_00000000),
                    unk_18: U64(0xffc00000),
                    unk_20: U64(0x11_00000000),
                    unk_28: U64(0x11_00000000),
                    // userspace address?
                    unk_30: U64(0x6f_ffff8000),
                    // unmapped?
                    unkptr_38: U64(0xffffffa0_11800000),
                    // TODO: yuv matrices
                    chip_id: cfg.chip_id,
                    unk_454: cfg.db.unk_454,
                    unk_458: 0x1,
                    unk_460: 0x1,
                    unk_464: 0x1,
                    unk_468: 0x1,
                    unk_47c: 0x1,
                    unk_484: 0x1,
                    unk_48c: 0x1,
                    base_clock_khz: cfg.base_clock_hz / 1000,
                    power_sample_period: dyncfg.pwr.power_sample_period,
                    unk_49c: 0x1,
                    unk_4a0: 0x1,
                    unk_4a4: 0x1,
                    unk_4c0: 0x1f,
                    unk_4e0: U64(cfg.db.unk_4e0),
                    unk_4f0: 0x1,
                    unk_4f4: 0x1,
                    unk_504: 0x31,
                    unk_524: 0x1, // use_secure_cache_flush
                    unk_534: cfg.db.unk_534,
                    num_frags: dyncfg.id.num_frags * dyncfg.id.num_clusters,
                    unk_554: 0x1,
                    uat_ttb_base: U64(dyncfg.uat_ttb_base),
                    gpu_core_id: cfg.gpu_core as u32,
                    gpu_rev_id: dyncfg.id.gpu_rev_id as u32,
                    num_cores: dyncfg.id.num_cores * dyncfg.id.num_clusters,
                    max_pstate: dyncfg.pwr.perf_states.len() as u32 - 1,
                    #[ver(V < V13_0B4)]
                    num_pstates: dyncfg.pwr.perf_states.len() as u32,
                    #[ver(V < V13_0B4)]
                    min_sram_volt: dyncfg.pwr.min_sram_microvolt / 1000,
                    #[ver(V < V13_0B4)]
                    unk_ab8: cfg.db.unk_ab8,
                    #[ver(V < V13_0B4)]
                    unk_abc: cfg.db.unk_abc,
                    #[ver(V < V13_0B4)]
                    unk_ac0: 0x1020,

                    #[ver(V >= V13_0B4)]
                    unk_ae4: Array::new([0x0, 0x3, 0x7, 0x7]),
                    #[ver(V < V13_0B4)]
                    unk_ae4: Array::new([0x0, 0xf, 0x3f, 0x3f]),
                    unk_b10: 0x1,
                    timer_offset: U64(0),
                    unk_b24: 0x1,
                    unk_b28: 0x1,
                    unk_b2c: 0x1,
                    unk_b30: cfg.db.unk_b30,
                    #[ver(V >= V13_0B4)]
                    unk_b38_0: 1,
                    #[ver(V >= V13_0B4)]
                    unk_b38_4: 1,
                    unk_b38: Array::new([0xffffffff; 12]),
                    #[ver(V >= V13_0B4 && V < V13_3)]
                    unk_c3c: 0x19,
                    #[ver(V >= V13_3)]
                    unk_c3c: 0x1a,
                    ..Zeroable::zeroed()
                }),
                |raw| {
                    #[ver(V >= V13_3)]
                    for i in 0..16 {
                        raw.unk_arr_0[i] = i as u32;
                    }

                    let base_ps = self.dyncfg.pwr.perf_base_pstate as usize;
                    let max_ps = self.dyncfg.pwr.perf_max_pstate as usize;
                    let base_freq = self.dyncfg.pwr.perf_states[base_ps].freq_hz;
                    let max_freq = self.dyncfg.pwr.perf_states[max_ps].freq_hz;

                    for (i, ps) in self.dyncfg.pwr.perf_states.iter().enumerate() {
                        raw.frequencies[i] = ps.freq_hz / 1000000;
                        for (j, mv) in ps.volt_mv.iter().enumerate() {
                            let sram_mv = (*mv).max(self.dyncfg.pwr.min_sram_microvolt / 1000);
                            raw.voltages[i][j] = *mv;
                            raw.voltages_sram[i][j] = sram_mv;
                        }
                        for j in ps.volt_mv.len()..raw.voltages[i].len() {
                            raw.voltages[i][j] = raw.voltages[i][0];
                            raw.voltages_sram[i][j] = raw.voltages_sram[i][0];
                        }
                        raw.sram_k[i] = self.cfg.sram_k;
                        raw.rel_max_powers[i] = ps.pwr_mw * 100 / self.dyncfg.pwr.max_power_mw;
                        raw.rel_boost_freqs[i] = if i > base_ps {
                            (ps.freq_hz - base_freq) / ((max_freq - base_freq) / 100)
                        } else {
                            0
                        };
                    }

                    #[ver(V >= V13_0B4)]
                    if let Some(csafr) = self.dyncfg.pwr.csafr.as_ref() {
                        let aux = &mut raw.aux_ps;
                        aux.cs_max_pstate = (csafr.perf_states_cs.len() - 1).try_into()?;
                        aux.afr_max_pstate = (csafr.perf_states_afr.len() - 1).try_into()?;

                        for (i, ps) in csafr.perf_states_cs.iter().enumerate() {
                            aux.cs_frequencies[i] = ps.freq_hz / 1000000;
                            for (j, mv) in ps.volt_mv.iter().enumerate() {
                                let sram_mv = (*mv).max(csafr.min_sram_microvolt / 1000);
                                aux.cs_voltages[i][j] = *mv;
                                aux.cs_voltages_sram[i][j] = sram_mv;
                            }
                        }

                        for (i, ps) in csafr.perf_states_afr.iter().enumerate() {
                            aux.afr_frequencies[i] = ps.freq_hz / 1000000;
                            for (j, mv) in ps.volt_mv.iter().enumerate() {
                                let sram_mv = (*mv).max(csafr.min_sram_microvolt / 1000);
                                aux.afr_voltages[i][j] = *mv;
                                aux.afr_voltages_sram[i][j] = sram_mv;
                            }
                        }
                    }

                    // Special case override for T602x
                    #[ver(G == G14X)]
                    if dyncfg.id.gpu_rev_id == hw::GpuRevisionID::B1 {
                        raw.gpu_rev_id = hw::GpuRevisionID::B0 as u32;
                    }

                    Ok(())
                },
            )
        })
    }

    /// Create the Globals structure, which contains global firmware config including more power
    /// configuration data and globals used to exchange state between the firmware and driver.
    fn globals(&mut self) -> Result<GpuObject<Globals::ver>> {
        self.alloc.private.new_init(init::zeroed(), |_inner, _ptr| {
            let cfg = &self.cfg;
            let dyncfg = &self.dyncfg;
            let pwr = &dyncfg.pwr;
            let period_ms = pwr.power_sample_period;
            let period_s = F32::from(period_ms) / f32!(1000.0);
            let avg_power_filter_tc_periods = pwr.avg_power_filter_tc_ms / period_ms;

            let max_ps = pwr.perf_max_pstate;
            let max_ps_scaled = 100 * max_ps;

            init::chain(
                try_init!(raw::Globals::ver {
                    //ktrace_enable: 0xffffffff,
                    ktrace_enable: 0,
                    #[ver(V >= V13_2)]
                    unk_24_0: 3000,
                    unk_24: 0,
                    #[ver(V >= V13_0B4)]
                    debug: 0,
                    unk_28: 1,
                    #[ver(G >= G14X)]
                    unk_2c_0: 1,
                    #[ver(V >= V13_0B4 && G < G14X)]
                    unk_2c_0: 0,
                    unk_2c: 1,
                    unk_30: 0,
                    unk_34: 120,
                    sub <- try_init!(raw::GlobalsSub::ver {
                        unk_54: cfg.global_unk_54,
                        unk_56: 40,
                        unk_58: 0xffff,
                        unk_5e: U32(1),
                        unk_66: U32(1),
                        ..Zeroable::zeroed()
                    }),
                    unk_8900: 1,
                    pending_submissions: AtomicU32::new(0),
                    max_power: pwr.max_power_mw,
                    max_pstate_scaled: max_ps_scaled,
                    max_pstate_scaled_2: max_ps_scaled,
                    max_pstate_scaled_3: max_ps_scaled,
                    power_zone_count: pwr.power_zones.len() as u32,
                    avg_power_filter_tc_periods: avg_power_filter_tc_periods,
                    avg_power_ki_dt: pwr.avg_power_ki_only * period_s,
                    avg_power_kp: pwr.avg_power_kp,
                    avg_power_min_duty_cycle: pwr.avg_power_min_duty_cycle,
                    avg_power_target_filter_tc: pwr.avg_power_target_filter_tc,
                    unk_89bc: cfg.da.unk_8cc,
                    fast_die0_release_temp: 100 * pwr.fast_die0_release_temp,
                    unk_89c4: cfg.da.unk_87c,
                    fast_die0_prop_tgt_delta: 100 * pwr.fast_die0_prop_tgt_delta,
                    fast_die0_kp: pwr.fast_die0_proportional_gain,
                    fast_die0_ki_dt: pwr.fast_die0_integral_gain * period_s,
                    unk_89e0: 1,
                    max_power_2: pwr.max_power_mw,
                    ppm_kp: pwr.ppm_kp,
                    ppm_ki_dt: pwr.ppm_ki * period_s,
                    #[ver(V >= V13_0B4)]
                    unk_89f4_8: 1,
                    unk_89f4: 0,
                    hws1 <- Self::hw_shared1(cfg),
                    hws2 <- Self::hw_shared2(cfg, dyncfg),
                    hws3 <- Self::hw_shared3(cfg),
                    #[ver(V >= V13_0B4)]
                    unk_hws2_0: cfg.unk_hws2_0,
                    #[ver(V >= V13_0B4)]
                    unk_hws2_4: cfg.unk_hws2_4.map(Array::new).unwrap_or_default(),
                    #[ver(V >= V13_0B4)]
                    unk_hws2_24: cfg.unk_hws2_24,
                    unk_900c: 1,
                    #[ver(V >= V13_0B4)]
                    unk_9010_0: 1,
                    #[ver(V >= V13_0B4)]
                    unk_903c: 1,
                    #[ver(V < V13_0B4)]
                    unk_903c: 0,
                    fault_control: *crate::fault_control.read(),
                    do_init: 1,
                    unk_11020: 40,
                    unk_11024: 10,
                    unk_11028: 250,
                    #[ver(V >= V13_0B4)]
                    unk_1102c_0: 1,
                    #[ver(V >= V13_0B4)]
                    unk_1102c_4: 1,
                    #[ver(V >= V13_0B4)]
                    unk_1102c_8: 100,
                    #[ver(V >= V13_0B4)]
                    unk_1102c_c: 1,
                    idle_off_delay_ms: AtomicU32::new(pwr.idle_off_delay_ms),
                    fender_idle_off_delay_ms: pwr.fender_idle_off_delay_ms,
                    fw_early_wake_timeout_ms: pwr.fw_early_wake_timeout_ms,
                    unk_118e0: 40,
                    #[ver(V >= V13_0B4)]
                    unk_118e4_0: 50,
                    #[ver(V >= V13_0B4)]
                    unk_11edc: 0,
                    #[ver(V >= V13_0B4)]
                    unk_11efc: 0,
                    ..Zeroable::zeroed()
                }),
                |raw| {
                    for (i, pz) in self.dyncfg.pwr.power_zones.iter().enumerate() {
                        raw.power_zones[i].target = pz.target;
                        raw.power_zones[i].target_off = pz.target - pz.target_offset;
                        raw.power_zones[i].filter_tc = pz.filter_tc;
                    }

                    if let Some(tab) = self.cfg.global_tab.as_ref() {
                        for (i, x) in tab.iter().enumerate() {
                            raw.unk_118ec[i] = *x;
                        }
                        raw.unk_118e8 = 1;
                    }
                    Ok(())
                },
            )
        })
    }

    /// Create the RuntimePointers structure, which contains pointers to most of the other
    /// structures including the ring buffer channels, statistics structures, and HwDataA/HwDataB.
    fn runtime_pointers(&mut self) -> Result<GpuObject<RuntimePointers::ver>> {
        let hwa = self.hwdata_a()?;
        let hwb = self.hwdata_b()?;

        let mut buffer_mgr_ctl = gem::new_kernel_object(self.dev, 0x4000)?;
        buffer_mgr_ctl.vmap()?.as_mut_slice().fill(0);

        GpuObject::new_init_prealloc(
            self.alloc.private.alloc_object()?,
            |_ptr| {
                let alloc = &mut *self.alloc;
                try_init!(RuntimePointers::ver {
                    stats <- {
                        let alloc = &mut *alloc;
                        try_init!(Stats::ver {
                            vtx: alloc.private.new_default::<GpuGlobalStatsVtx>()?,
                            frag: alloc.private.new_init(
                                init::zeroed::<GpuGlobalStatsFrag::ver>(),
                                |_inner, _ptr| {
                                    try_init!(raw::GpuGlobalStatsFrag::ver {
                                        total_cmds: 0,
                                        unk_4: 0,
                                        stats: Default::default(),
                                    })
                                }
                            )?,
                            comp: alloc.private.new_default::<GpuStatsComp>()?,
                        })
                    },

                    hwdata_a: hwa,
                    unkptr_190: alloc.private.array_empty(0x80)?,
                    unkptr_198: alloc.private.array_empty(0xc0)?,
                    hwdata_b: hwb,

                    unkptr_1b8: alloc.private.array_empty(0x1000)?,
                    unkptr_1c0: alloc.private.array_empty(0x300)?,
                    unkptr_1c8: alloc.private.array_empty(0x1000)?,

                    buffer_mgr_ctl,
                })
            },
            |inner, _ptr| {
                try_init!(raw::RuntimePointers::ver {
                    pipes: Default::default(),
                    device_control: Default::default(),
                    event: Default::default(),
                    fw_log: Default::default(),
                    ktrace: Default::default(),
                    stats: Default::default(),

                    stats_vtx: inner.stats.vtx.gpu_pointer(),
                    stats_frag: inner.stats.frag.gpu_pointer(),
                    stats_comp: inner.stats.comp.gpu_pointer(),

                    hwdata_a: inner.hwdata_a.gpu_pointer(),
                    unkptr_190: inner.unkptr_190.gpu_pointer(),
                    unkptr_198: inner.unkptr_198.gpu_pointer(),
                    hwdata_b: inner.hwdata_b.gpu_pointer(),
                    hwdata_b_2: inner.hwdata_b.gpu_pointer(),

                    fwlog_buf: None,

                    unkptr_1b8: inner.unkptr_1b8.gpu_pointer(),

                    #[ver(G < G14X)]
                    unkptr_1c0: inner.unkptr_1c0.gpu_pointer(),
                    #[ver(G < G14X)]
                    unkptr_1c8: inner.unkptr_1c8.gpu_pointer(),

                    buffer_mgr_ctl_gpu_addr: U64(gpu::IOVA_KERN_GPU_BUFMGR_LOW),
                    buffer_mgr_ctl_fw_addr: U64(gpu::IOVA_KERN_GPU_BUFMGR_HIGH),

                    __pad0: Default::default(),
                    unk_160: U64(0),
                    unk_168: U64(0),
                    unk_1d0: 0,
                    unk_1d4: 0,
                    unk_1d8: Default::default(),

                    __pad1: Default::default(),
                    gpu_scratch: raw::RuntimeScratch::ver {
                        unk_6b38: 0xff,
                        ..Default::default()
                    },
                })
            },
        )
    }

    /// Create the FwStatus structure, which is used to coordinate the firmware halt state between
    /// the firmware and the driver.
    fn fw_status(&mut self) -> Result<GpuObject<FwStatus>> {
        self.alloc
            .shared
            .new_object(Default::default(), |_inner| Default::default())
    }

    /// Create one UatLevelInfo structure, which describes one level of translation for the UAT MMU.
    fn uat_level_info(
        cfg: &'static hw::HwConfig,
        index_shift: usize,
        num_entries: usize,
    ) -> raw::UatLevelInfo {
        raw::UatLevelInfo {
            index_shift: index_shift as _,
            unk_1: 14,
            unk_2: 14,
            unk_3: 8,
            unk_4: 0x4000,
            num_entries: num_entries as _,
            unk_8: U64(1),
            unk_10: U64(((1u64 << cfg.uat_oas) - 1) & !(mmu::UAT_PGMSK as u64)),
            index_mask: U64(((num_entries - 1) << index_shift) as u64),
        }
    }

    /// Build the top-level InitData object.
    #[inline(never)]
    pub(crate) fn build(&mut self) -> Result<Box<GpuObject<InitData::ver>>> {
        let runtime_pointers = self.runtime_pointers()?;
        let globals = self.globals()?;
        let fw_status = self.fw_status()?;
        let shared_ro = &mut self.alloc.shared_ro;

        let obj = self.alloc.private.new_init(
            try_init!(InitData::ver {
                unk_buf: shared_ro.array_empty(0x4000)?,
                runtime_pointers,
                globals,
                fw_status,
            }),
            |inner, _ptr| {
                let cfg = &self.cfg;
                try_init!(raw::InitData::ver {
                    #[ver(V == V13_5 && G != G14X)]
                    ver_info: Array::new([0x6ba0, 0x1f28, 0x601, 0xb0]),
                    #[ver(V == V13_5 && G == G14X)]
                    ver_info: Array::new([0xb390, 0x70f8, 0x601, 0xb0]),
                    unk_buf: inner.unk_buf.gpu_pointer(),
                    unk_8: 0,
                    unk_c: 0,
                    runtime_pointers: inner.runtime_pointers.gpu_pointer(),
                    globals: inner.globals.gpu_pointer(),
                    fw_status: inner.fw_status.gpu_pointer(),
                    uat_page_size: 0x4000,
                    uat_page_bits: 14,
                    uat_num_levels: 3,
                    uat_level_info: Array::new([
                        Self::uat_level_info(cfg, 36, 8),
                        Self::uat_level_info(cfg, 25, 2048),
                        Self::uat_level_info(cfg, 14, 2048),
                    ]),
                    __pad0: Default::default(),
                    host_mapped_fw_allocations: 1,
                    unk_ac: 0,
                    unk_b0: 0,
                    unk_b4: 0,
                    unk_b8: 0,
                })
            },
        )?;
        Ok(Box::try_new(obj)?)
    }
}
