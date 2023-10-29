// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hardware configuration for t600x (M1 Pro/Max/Ultra) platforms.

use crate::f32;

use super::*;

const fn iomaps(chip_id: u32, mcc_count: usize) -> [Option<IOMapping>; 24] {
    [
        Some(IOMapping::new(0x404d00000, false, 1, 0x144000, 0, true)), // Fender
        Some(IOMapping::new(0x20e100000, false, 1, 0x4000, 0, false)),  // AICTimer
        Some(IOMapping::new(0x28e106000, false, 1, 0x4000, 0, true)),   // AICSWInt
        Some(IOMapping::new(0x404000000, false, 1, 0x20000, 0, true)),  // RGX
        None,                                                           // UVD
        None,                                                           // unused
        None,                                                           // DisplayUnderrunWA
        Some(match chip_id {
            0x6020 => IOMapping::new(0x28e460000, true, 1, 0x4000, 0, false),
            _ => IOMapping::new(0x28e478000, true, 1, 0x4000, 0, false),
        }), // AnalogTempSensorControllerRegs
        None,                                                           // PMPDoorbell
        Some(IOMapping::new(0x404e08000, false, 1, 0x8000, 0, true)),   // MetrologySensorRegs
        None,                                                           // GMGIFAFRegs
        Some(IOMapping::new(
            0x200000000,
            true,
            mcc_count,
            0xd8000,
            0x1000000,
            true,
        )), // MCache registers
        Some(IOMapping::new(0x28e118000, false, 1, 0x4000, 0, false)),  // AICBankedRegisters
        None,                                                           // PMGRScratch
        None, // NIA Special agent idle register die 0
        None, // NIA Special agent idle register die 1
        None, // CRE registers
        None, // Streaming codec registers
        Some(IOMapping::new(0x28e3d0000, false, 1, 0x4000, 0, true)), // ?
        Some(IOMapping::new(0x28e3c0000, false, 1, 0x4000, 0, false)), // ?
        Some(IOMapping::new(0x28e3d8000, false, 1, 0x4000, 0, true)), // ?
        Some(IOMapping::new(0x404eac000, true, 1, 0x4000, 0, true)), // ?
        None,
        None,
    ]
}

// TODO: Tentative
pub(crate) const HWCONFIG_T6022: super::HwConfig = HwConfig {
    chip_id: 0x6022,
    gpu_gen: GpuGen::G14,
    gpu_variant: GpuVariant::D,
    gpu_core: GpuCore::G14D,
    gpu_feat_compat: 0,
    gpu_feat_incompat: feat::incompat::MANDATORY_ZS_COMPRESSION,

    base_clock_hz: 24_000_000,
    uat_oas: 42,
    num_dies: 2,
    max_num_clusters: 8,
    max_num_cores: 10,
    max_num_frags: 10,
    max_num_gps: 4,

    preempt1_size: 0x540,
    preempt2_size: 0x280,
    preempt3_size: 0x40,
    compute_preempt1_size: 0x25980 * 2, // Conservative guess
    clustering: Some(HwClusteringConfig {
        meta1_blocksize: 0x44,
        meta2_size: 0xc0 * 16,
        meta3_size: 0x280 * 16,
        meta4_size: 0x10 * 128,
        max_splits: 64,
    }),

    render: HwRenderConfig {
        tiling_control: 0x180340,
    },

    da: HwConfigA {
        unk_87c: 500,
        unk_8cc: 11000,
        unk_e24: 125,
    },
    db: HwConfigB {
        unk_454: 1,
        unk_4e0: 4,
        unk_534: 0,
        unk_ab8: 0, // Unused
        unk_abc: 0, // Unused
        unk_b30: 0,
    },
    shared1_tab: &[
        0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
        0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
    ],
    shared1_a4: 0,
    shared2_tab: &[0x800, 0x1555, -1, -1, -1, -1, -1, -1, 0xaaaaa, 0],
    shared2_unk_508: 0xc00007,
    shared2_curves: Some(HwConfigShared2Curves {
        t1_coef: 11000,
        t2: &[
            0xf07, 0x4c0, 0x680, 0x8c0, 0xa80, 0xc40, 0xd80, 0xec0, 0xf40,
        ],
        t3_coefs: &[0, 20, 27, 36, 43, 50, 55, 60, 62],
        t3_scales: &[9, 3209, 10400],
    }),
    shared3_unk: 8,
    shared3_tab: &[
        125, 125, 125, 125, 125, 125, 125, 125, 7500, 125, 125, 125, 125, 125, 125, 125,
    ],
    idle_off_standby_timer_default: 700,
    unk_hws2_4: Some(f32!([1.0, 0.8, 0.2, 0.9, 0.1, 0.25, 0.5, 0.9])),
    unk_hws2_24: 6,
    global_unk_54: 4000,
    sram_k: f32!(1.02),
    unk_coef_a: &[
        &f32!([0.0, 8.2, 0.0, 6.9, 6.9]),
        &f32!([0.0, 0.0, 0.0, 6.9, 6.9]),
        &f32!([0.0, 8.2, 0.0, 6.9, 0.0]),
        &f32!([0.0, 0.0, 0.0, 6.9, 0.0]),
        &f32!([0.0, 0.0, 0.0, 6.9, 0.0]),
        &f32!([0.0, 8.2, 0.0, 6.9, 0.0]),
        &f32!([0.0, 0.0, 0.0, 6.9, 6.9]),
        &f32!([0.0, 8.2, 0.0, 6.9, 6.9]),
    ],
    unk_coef_b: &[
        &f32!([0.0, 9.0, 0.0, 8.0, 8.0]),
        &f32!([0.0, 0.0, 0.0, 8.0, 8.0]),
        &f32!([0.0, 9.0, 0.0, 8.0, 0.0]),
        &f32!([0.0, 0.0, 0.0, 8.0, 0.0]),
        &f32!([0.0, 0.0, 0.0, 8.0, 0.0]),
        &f32!([0.0, 9.0, 0.0, 8.0, 0.0]),
        &f32!([0.0, 0.0, 0.0, 8.0, 8.0]),
        &f32!([0.0, 9.0, 0.0, 8.0, 8.0]),
    ],
    global_tab: Some(&[
        0, 2, 2, 1, 1, 90, 75, 1, 1, 1, 2, 90, 75, 1, 1, 1, 2, 90, 75, 1, 1, 1, 1, 90, 75, 1, 1,
    ]),
    has_csafr: true,
    fast_sensor_mask: [0x40005000c000d00, 0xd000c0005000400],
    // Apple typo? Should probably be 0x140015001c001d00
    fast_sensor_mask_alt: [0x140015001d001d00, 0x1d001c0015001400],
    fast_die0_sensor_present: 0, // Unused
    io_mappings: &iomaps(0x6022, 8),
    sram_base: Some(0x404d60000),
    sram_size: Some(0x20000),
};

pub(crate) const HWCONFIG_T6021: super::HwConfig = HwConfig {
    chip_id: 0x6021,
    gpu_variant: GpuVariant::C,
    gpu_core: GpuCore::G14C,

    num_dies: 1,
    max_num_clusters: 4,
    compute_preempt1_size: 0x25980,
    unk_hws2_4: Some(f32!([1.0, 0.8, 0.2, 0.9, 0.1, 0.25, 0.7, 0.9])),
    fast_sensor_mask: [0x40005000c000d00, 0],
    fast_sensor_mask_alt: [0x140015001d001d00, 0],
    io_mappings: &iomaps(0x6021, 8),
    ..HWCONFIG_T6022
};

pub(crate) const HWCONFIG_T6020: super::HwConfig = HwConfig {
    chip_id: 0x6020,
    gpu_variant: GpuVariant::S,
    gpu_core: GpuCore::G14S,

    db: HwConfigB {
        unk_454: 0,
        ..HWCONFIG_T6021.db
    },

    max_num_clusters: 2,
    fast_sensor_mask: [0xc000d00, 0],
    fast_sensor_mask_alt: [0x1d001d00, 0],
    io_mappings: &iomaps(0x6020, 4),
    ..HWCONFIG_T6021
};
