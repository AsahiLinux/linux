// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hardware configuration for t600x (M1 Pro/Max/Ultra) platforms.

use crate::f32;

use super::*;

const fn iomaps(mcc_count: usize, has_die1: bool) -> [Option<IOMapping>; 20] {
    [
        Some(IOMapping::new(0x404d00000, 0x1c000, 0x1c000, true)), // Fender
        Some(IOMapping::new(0x20e100000, 0x4000, 0x4000, false)),  // AICTimer
        Some(IOMapping::new(0x28e104000, 0x4000, 0x4000, true)),   // AICSWInt
        Some(IOMapping::new(0x404000000, 0x20000, 0x20000, true)), // RGX
        None,                                                      // UVD
        None,                                                      // unused
        None,                                                      // DisplayUnderrunWA
        Some(IOMapping::new(0x28e494000, 0x1000, 0x1000, false)),  // AnalogTempSensorControllerRegs
        None,                                                      // PMPDoorbell
        Some(IOMapping::new(0x404d80000, 0x8000, 0x8000, true)),   // MetrologySensorRegs
        Some(IOMapping::new(0x204d61000, 0x1000, 0x1000, true)),   // GMGIFAFRegs
        Some(IOMapping::new(
            0x200000000,
            mcc_count * 0xd8000,
            0xd8000,
            true,
        )), // MCache registers
        None,                                                      // AICBankedRegisters
        None,                                                      // PMGRScratch
        Some(IOMapping::new(0x2643c4000, 0x1000, 0x1000, true)), // NIA Special agent idle register die 0
        if has_die1 {
            // NIA Special agent idle register die 1
            Some(IOMapping::new(0x22643c4000, 0x1000, 0x1000, true))
        } else {
            None
        },
        None,                                                     // CRE registers
        None,                                                     // Streaming codec registers
        Some(IOMapping::new(0x28e3d0000, 0x1000, 0x1000, true)),  // ?
        Some(IOMapping::new(0x28e3c0000, 0x1000, 0x1000, false)), // ?
    ]
}

pub(crate) const HWCONFIG_T6002: super::HwConfig = HwConfig {
    chip_id: 0x6002,
    gpu_gen: GpuGen::G13,
    gpu_variant: GpuVariant::D,
    gpu_core: GpuCore::G13C,
    gpu_feat_compat: 0,
    gpu_feat_incompat: feat::incompat::MANDATORY_ZS_COMPRESSION,

    base_clock_hz: 24_000_000,
    uat_oas: 42,
    num_dies: 2,
    max_num_clusters: 8,
    max_num_cores: 8,
    max_num_frags: 8,
    max_num_gps: 4,

    preempt1_size: 0x540,
    preempt2_size: 0x280,
    preempt3_size: 0x20,
    compute_preempt1_size: 0x3bd00,
    clustering: Some(HwClusteringConfig {
        meta1_blocksize: 0x44,
        meta2_size: 0xc0 * 8,
        meta3_size: 0x280 * 8,
        meta4_size: 0x30 * 16,
        max_splits: 16,
    }),

    render: HwRenderConfig {
        tiling_control: 0xa540,
    },

    da: HwConfigA {
        unk_87c: 900,
        unk_8cc: 11000,
        unk_e24: 125,
    },
    db: HwConfigB {
        unk_4e0: 4,
        unk_534: 1,
        unk_ab8: 0x2084,
        unk_abc: 0x80,
        unk_b30: 0,
    },
    shared1_tab: &[
        0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
        0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
    ],
    shared1_a4: 0xffff,
    shared2_tab: &[-1, -1, -1, -1, 0x2aa, 0xaaa, -1, -1, 0, 0],
    shared2_unk_508: 0xcc00001,
    shared2_curves: None,
    shared3_unk: 0,
    shared3_tab: &[],
    unk_hws2_0: 0,
    unk_hws2_4: None,
    unk_hws2_24: 0,
    global_unk_54: 0xffff,
    sram_k: f32!(1.02),
    unk_coef_a: &[
        &f32!([9.838]),
        &f32!([9.819]),
        &f32!([9.826]),
        &f32!([9.799]),
        &f32!([9.799]),
        &f32!([9.826]),
        &f32!([9.819]),
        &f32!([9.838]),
    ],
    unk_coef_b: &[
        &f32!([13.0]),
        &f32!([13.0]),
        &f32!([13.0]),
        &f32!([13.0]),
        &f32!([13.0]),
        &f32!([13.0]),
        &f32!([13.0]),
        &f32!([13.0]),
    ],
    global_tab: Some(&[
        0, 1, 2, 1, 1, 90, 75, 1, 1, 1, 2, 90, 75, 1, 1, 1, 1, 90, 75, 1, 1,
    ]),
    has_csafr: false,
    fast_sensor_mask: [0x8080808080808080, 0],
    fast_sensor_mask_alt: [0x9090909090909090, 0],
    fast_die0_sensor_present: 0xff,
    io_mappings: &iomaps(16, true),
    sram_base: None,
    sram_size: None,
};

pub(crate) const HWCONFIG_T6001: super::HwConfig = HwConfig {
    chip_id: 0x6001,
    gpu_variant: GpuVariant::C,
    gpu_core: GpuCore::G13C,

    num_dies: 1,
    max_num_clusters: 4,
    fast_sensor_mask: [0x80808080, 0],
    fast_sensor_mask_alt: [0x90909090, 0],
    fast_die0_sensor_present: 0x0f,
    io_mappings: &iomaps(8, false),
    ..HWCONFIG_T6002
};

pub(crate) const HWCONFIG_T6000: super::HwConfig = HwConfig {
    chip_id: 0x6000,
    gpu_variant: GpuVariant::S,
    gpu_core: GpuCore::G13S,

    max_num_clusters: 2,
    fast_sensor_mask: [0x8080, 0],
    fast_sensor_mask_alt: [0x9090, 0],
    fast_die0_sensor_present: 0x03,
    io_mappings: &iomaps(4, false),
    ..HWCONFIG_T6001
};
