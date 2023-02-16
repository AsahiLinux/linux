// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hardware configuration for t8112 platforms (M2).

use crate::f32;

use super::*;

pub(crate) const HWCONFIG: super::HwConfig = HwConfig {
    chip_id: 0x8112,
    gpu_gen: GpuGen::G14,
    gpu_variant: GpuVariant::G,
    gpu_core: GpuCore::G14G,
    gpu_feat_compat: 0,
    gpu_feat_incompat: 0,

    base_clock_hz: 24_000_000,
    uat_oas: 40,
    num_dies: 1,
    max_num_clusters: 1,
    max_num_cores: 10,
    max_num_frags: 10,
    max_num_gps: 4,

    preempt1_size: 0x540,
    preempt2_size: 0x280,
    preempt3_size: 0x20,
    compute_preempt1_size: 0x10000, // TODO: Check
    clustering: None,

    render: HwRenderConfig {
        // TODO: this is unused here, may be present in newer FW
        tiling_control: 0xa041,
    },

    da: HwConfigA {
        unk_87c: 900,
        unk_8cc: 11000,
        unk_e24: 125,
    },
    db: HwConfigB {
        unk_4e0: 4,
        unk_534: 0,
        unk_ab8: 0x2048,
        unk_abc: 0x4000,
        unk_b30: 1,
    },
    shared1_tab: &[
        0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
        0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
    ],
    shared1_a4: 0,
    shared2_tab: &[-1, -1, -1, -1, -1, -1, -1, -1, 0xaa5aa, 0],
    shared2_unk_508: 0xc00000,
    shared2_curves: Some(HwConfigShared2Curves {
        t1_coef: 7200,
        t2: &[
            0xf07, 0x4c0, 0x6c0, 0x8c0, 0xac0, 0xc40, 0xdc0, 0xec0, 0xf80,
        ],
        t3_coefs: &[0, 20, 28, 36, 44, 50, 56, 60, 63],
        t3_scales: &[9, 3209, 10400],
    }),
    shared3_unk: 5,
    shared3_tab: &[
        10700, 10700, 10700, 10700, 10700, 6000, 1000, 1000, 1000, 10700, 10700, 10700, 10700,
        10700, 10700, 10700,
    ],
    unk_hws2_0: 0,
    unk_hws2_4: None,
    unk_hws2_24: 0,
    global_unk_54: 0xffff,

    sram_k: f32!(1.02),
    // 13.2: last coef changed from 6.6 to 5.3, assuming that was a fix we can backport
    unk_coef_a: &[&f32!([0.0, 0.0, 0.0, 0.0, 5.3, 0.0, 5.3, /*6.6*/ 5.3])],
    unk_coef_b: &[&f32!([0.0, 0.0, 0.0, 0.0, 5.3, 0.0, 5.3, /*6.6*/ 5.3])],
    global_tab: None,
    has_csafr: false,
    fast_sensor_mask: [0x6800, 0],
    fast_sensor_mask_alt: [0x6800, 0],
    fast_die0_sensor_present: 0x02,
    io_mappings: &[
        Some(IOMapping::new(0x204d00000, 0x14000, 0x14000, true)), // Fender
        Some(IOMapping::new(0x20e100000, 0x4000, 0x4000, false)),  // AICTimer
        Some(IOMapping::new(0x23b0c4000, 0x4000, 0x4000, true)),   // AICSWInt
        Some(IOMapping::new(0x204000000, 0x20000, 0x20000, true)), // RGX
        None,                                                      // UVD
        None,                                                      // unused
        None,                                                      // DisplayUnderrunWA
        Some(IOMapping::new(0x23b2c0000, 0x1000, 0x1000, false)),  // AnalogTempSensorControllerRegs
        None,                                                      // PMPDoorbell
        Some(IOMapping::new(0x204d80000, 0x8000, 0x8000, true)),   // MetrologySensorRegs
        Some(IOMapping::new(0x204d61000, 0x1000, 0x1000, true)),   // GMGIFAFRegs
        Some(IOMapping::new(0x200000000, 0xd6400, 0xd6400, true)), // MCache registers
        None,                                                      // AICBankedRegisters
        None,                                                      // PMGRScratch
        None, // NIA Special agent idle register die 0
        None, // NIA Special agent idle register die 1
        Some(IOMapping::new(0x204e00000, 0x10000, 0x10000, true)), // CRE registers
        Some(IOMapping::new(0x27d050000, 0x4000, 0x4000, true)), // Streaming codec registers
        Some(IOMapping::new(0x23b3d0000, 0x1000, 0x1000, true)), //
        Some(IOMapping::new(0x23b3c0000, 0x1000, 0x1000, true)), //
    ],
    sram_base: None,
    sram_size: None,
};
