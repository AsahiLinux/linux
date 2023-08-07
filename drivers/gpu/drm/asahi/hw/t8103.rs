// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hardware configuration for t8103 platforms (M1).

use crate::f32;

use super::*;

pub(crate) const HWCONFIG: super::HwConfig = HwConfig {
    chip_id: 0x8103,
    gpu_gen: GpuGen::G13,
    gpu_variant: GpuVariant::G,
    gpu_core: GpuCore::G13G,
    gpu_feat_compat: 0,
    gpu_feat_incompat: 0,

    base_clock_hz: 24_000_000,
    uat_oas: 40,
    num_dies: 1,
    max_num_clusters: 1,
    max_num_cores: 8,
    max_num_frags: 8,
    max_num_gps: 4,

    preempt1_size: 0x540,
    preempt2_size: 0x280,
    preempt3_size: 0x20,
    compute_preempt1_size: 0x7f80,
    clustering: None,

    render: HwRenderConfig {
        // bit 0: disable clustering (always)
        tiling_control: 0xa041,
    },

    da: HwConfigA {
        unk_87c: -220,
        unk_8cc: 9880,
        unk_e24: 112,
    },
    db: HwConfigB {
        unk_454: 1,
        unk_4e0: 0,
        unk_534: 0,
        unk_ab8: 0x48,
        unk_abc: 0x8,
        unk_b30: 0,
    },
    shared1_tab: &[
        -1, 0x7282, 0x50ea, 0x370a, 0x25be, 0x1c1f, 0x16fb, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    ],
    shared1_a4: 0xffff,
    shared2_tab: &[0x800, 0x1555, -1, -1, -1, -1, -1, -1, 0, 0],
    shared2_unk_508: 0xc00007,
    shared2_curves: None,
    shared3_unk: 0,
    shared3_tab: &[],
    unk_hws2_0: 0,
    unk_hws2_4: None,
    unk_hws2_24: 0,
    global_unk_54: 0xffff,
    sram_k: f32!(1.02),
    unk_coef_a: &[],
    unk_coef_b: &[],
    global_tab: None,
    has_csafr: false,
    fast_sensor_mask: [0x12, 0],
    fast_sensor_mask_alt: [0x12, 0],
    fast_die0_sensor_present: 0x01,
    io_mappings: &[
        Some(IOMapping::new(0x204d00000, 0x1c000, 0x1c000, true)), // Fender
        Some(IOMapping::new(0x20e100000, 0x4000, 0x4000, false)),  // AICTimer
        Some(IOMapping::new(0x23b104000, 0x4000, 0x4000, true)),   // AICSWInt
        Some(IOMapping::new(0x204000000, 0x20000, 0x20000, true)), // RGX
        None,                                                      // UVD
        None,                                                      // unused
        None,                                                      // DisplayUnderrunWA
        Some(IOMapping::new(0x23b2e8000, 0x1000, 0x1000, false)),  // AnalogTempSensorControllerRegs
        Some(IOMapping::new(0x23bc00000, 0x1000, 0x1000, true)),   // PMPDoorbell
        Some(IOMapping::new(0x204d80000, 0x5000, 0x5000, true)),   // MetrologySensorRegs
        Some(IOMapping::new(0x204d61000, 0x1000, 0x1000, true)),   // GMGIFAFRegs
        Some(IOMapping::new(0x200000000, 0xd6400, 0xd6400, true)), // MCache registers
        None,                                                      // AICBankedRegisters
        Some(IOMapping::new(0x23b738000, 0x1000, 0x1000, true)),   // PMGRScratch
        None, // NIA Special agent idle register die 0
        None, // NIA Special agent idle register die 1
        None, // CRE registers
        None, // Streaming codec registers
        None, //
        None, //
    ],
    sram_base: None,
    sram_size: None,
};
