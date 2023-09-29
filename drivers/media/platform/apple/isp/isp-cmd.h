// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#ifndef __ISP_CMD_H__
#define __ISP_CMD_H__

#include "isp-drv.h"

#define CISP_CMD_START					     0x0000
#define CISP_CMD_STOP					     0x0001
#define CISP_CMD_CONFIG_GET				     0x0003
#define CISP_CMD_PRINT_ENABLE				     0x0004
#define CISP_CMD_BUILDINFO				     0x0006
#define CISP_CMD_GET_BES_PARAM				     0x000f
#define CISP_CMD_POWER_DOWN				     0x0010
#define CISP_CMD_SET_ISP_PMU_BASE			     0x0011
#define CISP_CMD_PMP_CTRL_SET				     0x001c
#define CISP_CMD_TRACE_ENABLE				     0x001d
#define CISP_CMD_SUSPEND				     0x0021
#define CISP_CMD_FID_ENTER				     0x0022
#define CISP_CMD_FID_EXIT				     0x0023
#define CISP_CMD_FLICKER_SENSOR_SET			     0x0024
#define CISP_CMD_CH_START				     0x0100
#define CISP_CMD_CH_STOP				     0x0101
#define CISP_CMD_CH_BUFFER_RETURN			     0x0104
#define CISP_CMD_CH_CAMERA_CONFIG_CURRENT_GET		     0x0105
#define CISP_CMD_CH_CAMERA_CONFIG_GET			     0x0106
#define CISP_CMD_CH_CAMERA_CONFIG_SELECT		     0x0107
#define CISP_CMD_CH_INFO_GET				     0x010d
#define CISP_CMD_CH_BUFFER_RECYCLE_MODE_SET		     0x010e
#define CISP_CMD_CH_BUFFER_RECYCLE_START		     0x010f
#define CISP_CMD_CH_BUFFER_RECYCLE_STOP			     0x0110
#define CISP_CMD_CH_SET_FILE_LOAD			     0x0111
#define CISP_CMD_CH_SIF_PIXEL_FORMAT_SET		     0x0115
#define CISP_CMD_CH_BUFFER_POOL_CONFIG_GET		     0x0116
#define CISP_CMD_CH_BUFFER_POOL_CONFIG_SET		     0x0117
#define CISP_CMD_CH_CAMERA_MIPI_FREQUENCY_GET		     0x011a
#define CISP_CMD_CH_CAMERA_PIX_FREQUENCY_GET		     0x011f
#define CISP_CMD_CH_PROPERTY_WRITE			     0x0122
#define CISP_CMD_CH_PROPERTY_READ			     0x0123
#define CISP_CMD_CH_LOCAL_RAW_BUFFER_ENABLE		     0x0125
#define CISP_CMD_CH_META_DATA_ENABLE			     0x0126
#define CISP_CMD_CH_CAMERA_MIPI_FREQUENCY_TOTAL_GET	     0x0133
#define CISP_CMD_CH_SBS_ENABLE				     0x013b
#define CISP_CMD_CH_LSC_POLYNOMIAL_COEFF_GET		     0x0142
#define CISP_CMD_CH_SET_META_DATA_REQUIRED		     0x014f
#define CISP_CMD_CH_BUFFER_POOL_RETURN			     0x015b
#define CISP_CMD_CH_CAMERA_AGILE_FREQ_ARRAY_CURRENT_GET	     0x015e
#define CISP_CMD_CH_AE_START				     0x0200
#define CISP_CMD_CH_AE_STOP				     0x0201
#define CISP_CMD_CH_AE_FRAME_RATE_MAX_GET		     0x0207
#define CISP_CMD_CH_AE_FRAME_RATE_MAX_SET		     0x0208
#define CISP_CMD_CH_AE_FRAME_RATE_MIN_GET		     0x0209
#define CISP_CMD_CH_AE_FRAME_RATE_MIN_SET		     0x020a
#define CISP_CMD_CH_AE_STABILITY_SET			     0x021a
#define CISP_CMD_CH_AE_STABILITY_TO_STABLE_SET		     0x0229
#define CISP_CMD_CH_SENSOR_NVM_GET			     0x0501
#define CISP_CMD_CH_SENSOR_PERMODULE_LSC_INFO_GET	     0x0507
#define CISP_CMD_CH_SENSOR_PERMODULE_LSC_GRID_GET	     0x0511
#define CISP_CMD_CH_LPDP_HS_RECEIVER_TUNING_SET		     0x051b
#define CISP_CMD_CH_FOCUS_LIMITS_GET			     0x0701
#define CISP_CMD_CH_CROP_GET				     0x0800
#define CISP_CMD_CH_CROP_SET				     0x0801
#define CISP_CMD_CH_SCALER_CROP_SET			     0x080a
#define CISP_CMD_CH_CROP_SCL1_GET			     0x080b
#define CISP_CMD_CH_CROP_SCL1_SET			     0x080c
#define CISP_CMD_CH_SCALER_CROP_SCL1_SET		     0x080d
#define CISP_CMD_CH_ALS_ENABLE				     0x0a1c
#define CISP_CMD_CH_ALS_DISABLE				     0x0a1d
#define CISP_CMD_CH_CNR_START				     0x0a2f
#define CISP_CMD_CH_MBNR_ENABLE				     0x0a3a
#define CISP_CMD_CH_OUTPUT_CONFIG_SET			     0x0b01
#define CISP_CMD_CH_OUTPUT_CONFIG_SCL1_SET		     0x0b09
#define CISP_CMD_CH_PREVIEW_STREAM_SET			     0x0b0d
#define CISP_CMD_CH_SEMANTIC_VIDEO_ENABLE		     0x0b17
#define CISP_CMD_CH_SEMANTIC_AWB_ENABLE			     0x0b18
#define CISP_CMD_CH_FACE_DETECTION_START		     0x0d00
#define CISP_CMD_CH_FACE_DETECTION_STOP			     0x0d01
#define CISP_CMD_CH_FACE_DETECTION_CONFIG_GET		     0x0d02
#define CISP_CMD_CH_FACE_DETECTION_CONFIG_SET		     0x0d03
#define CISP_CMD_CH_FACE_DETECTION_DISABLE		     0x0d04
#define CISP_CMD_CH_FACE_DETECTION_ENABLE		     0x0d05
#define CISP_CMD_CH_FID_START				     0x3000
#define CISP_CMD_CH_FID_STOP				     0x3001
#define CISP_CMD_IPC_ENDPOINT_SET2			     0x300c
#define CISP_CMD_IPC_ENDPOINT_UNSET2			     0x300d
#define CISP_CMD_SET_DSID_CLR_REG_BASE2			     0x3204
#define CISP_CMD_SET_DSID_CLR_REG_BASE			     0x3205
#define CISP_CMD_APPLE_CH_AE_METERING_MODE_SET		     0x8206
#define CISP_CMD_APPLE_CH_AE_FD_SCENE_METERING_CONFIG_SET    0x820e
#define CISP_CMD_APPLE_CH_AE_FLICKER_FREQ_UPDATE_CURRENT_SET 0x8212
#define CISP_CMD_APPLE_CH_TEMPORAL_FILTER_START		     0xc100
#define CISP_CMD_APPLE_CH_TEMPORAL_FILTER_STOP		     0xc101
#define CISP_CMD_APPLE_CH_MOTION_HISTORY_START		     0xc102
#define CISP_CMD_APPLE_CH_MOTION_HISTORY_STOP		     0xc103
#define CISP_CMD_APPLE_CH_TEMPORAL_FILTER_ENABLE	     0xc113
#define CISP_CMD_APPLE_CH_TEMPORAL_FILTER_DISABLE	     0xc114

#define CISP_POOL_TYPE_META				     0x0
#define CISP_POOL_TYPE_RENDERED				     0x1
#define CISP_POOL_TYPE_FD				     0x2
#define CISP_POOL_TYPE_RAW				     0x3
#define CISP_POOL_TYPE_STAT				     0x4
#define CISP_POOL_TYPE_RAW_AUX				     0x5
#define CISP_POOL_TYPE_YCC				     0x6
#define CISP_POOL_TYPE_CAPTURE_FULL_RES			     0x7
#define CISP_POOL_TYPE_META_CAPTURE			     0x8
#define CISP_POOL_TYPE_RENDERED_SCL1			     0x9
#define CISP_POOL_TYPE_STAT_PIXELOUTPUT			     0x11
#define CISP_POOL_TYPE_FSCL				     0x12
#define CISP_POOL_TYPE_CAPTURE_FULL_RES_YCC		     0x13
#define CISP_POOL_TYPE_RENDERED_RAW			     0x14
#define CISP_POOL_TYPE_CAPTURE_PDC_RAW			     0x16
#define CISP_POOL_TYPE_FPC_DATA				     0x17
#define CISP_POOL_TYPE_AICAM_SEG			     0x19
#define CISP_POOL_TYPE_SPD				     0x1a
#define CISP_POOL_TYPE_META_DEPTH			     0x1c
#define CISP_POOL_TYPE_JASPER_DEPTH			     0x1d
#define CISP_POOL_TYPE_RAW_SIFR				     0x1f
#define CISP_POOL_TYPE_FEP_THUMBNAIL_DYNAMIC_POOL_RAW	     0x21

#define CISP_COLORSPACE_REC709				     0x1
#define CISP_OUTPUT_FORMAT_YUV_2PLANE			     0x0
#define CISP_OUTPUT_FORMAT_YUV_1PLANE			     0x1
#define CISP_OUTPUT_FORMAT_RGB				     0x2
#define CISP_BUFFER_RECYCLE_MODE_EMPTY_ONLY		     0x1

struct cmd_start {
	u64 opcode;
	u32 mode;
} __packed;
static_assert(sizeof(struct cmd_start) == 0xc);

struct cmd_stop {
	u64 opcode;
	u32 mode;
} __packed;
static_assert(sizeof(struct cmd_stop) == 0xc);

struct cmd_power_down {
	u64 opcode;
} __packed;
static_assert(sizeof(struct cmd_power_down) == 0x8);

struct cmd_suspend {
	u64 opcode;
} __packed;
static_assert(sizeof(struct cmd_suspend) == 0x8);

struct cmd_print_enable {
	u64 opcode;
	u32 enable;
} __packed;
static_assert(sizeof(struct cmd_print_enable) == 0xc);

struct cmd_trace_enable {
	u64 opcode;
	u32 enable;
} __packed;
static_assert(sizeof(struct cmd_trace_enable) == 0xc);

struct cmd_config_get {
	u64 opcode;
	u32 timestamp_freq;
	u32 num_channels;
	u32 unk_10;
	u32 unk_14;
	u32 unk_18;
} __packed;
static_assert(sizeof(struct cmd_config_get) == 0x1c);

struct cmd_set_isp_pmu_base {
	u64 opcode;
	u64 pmu_base;
} __packed;
static_assert(sizeof(struct cmd_set_isp_pmu_base) == 0x10);

struct cmd_set_dsid_clr_req_base2 {
	u64 opcode;
	u64 dsid_clr_base0;
	u64 dsid_clr_base1;
	u64 dsid_clr_base2;
	u64 dsid_clr_base3;
	u32 dsid_clr_range0;
	u32 dsid_clr_range1;
	u32 dsid_clr_range2;
	u32 dsid_clr_range3;
} __packed;
static_assert(sizeof(struct cmd_set_dsid_clr_req_base2) == 0x38);

struct cmd_set_dsid_clr_req_base {
	u64 opcode;
	u64 dsid_clr_base;
	u32 dsid_clr_range;
} __packed;
static_assert(sizeof(struct cmd_set_dsid_clr_req_base) == 0x14);

struct cmd_pmp_ctrl_set {
	u64 opcode;
	u64 clock_scratch;
	u64 clock_base;
	u8 clock_bit;
	u8 clock_size;
	u16 clock_pad;
	u64 bandwidth_scratch;
	u64 bandwidth_base;
	u8 bandwidth_bit;
	u8 bandwidth_size;
	u16 bandwidth_pad;
} __packed;
static_assert(sizeof(struct cmd_pmp_ctrl_set) == 0x30);

struct cmd_fid_enter {
	u64 opcode;
} __packed;
static_assert(sizeof(struct cmd_fid_enter) == 0x8);

struct cmd_fid_exit {
	u64 opcode;
} __packed;
static_assert(sizeof(struct cmd_fid_exit) == 0x8);

struct cmd_ipc_endpoint_set2 {
	u64 opcode;
	u32 unk;
	u64 addr1;
	u32 size1;
	u64 addr2;
	u32 size2;
	u64 regs;
	u32 unk2;
} __packed;
static_assert(sizeof(struct cmd_ipc_endpoint_set2) == 0x30);

int isp_cmd_start(struct apple_isp *isp, u32 mode);
int isp_cmd_stop(struct apple_isp *isp, u32 mode);
int isp_cmd_power_down(struct apple_isp *isp);
int isp_cmd_suspend(struct apple_isp *isp);
int isp_cmd_print_enable(struct apple_isp *isp, u32 enable);
int isp_cmd_trace_enable(struct apple_isp *isp, u32 enable);
int isp_cmd_config_get(struct apple_isp *isp, struct cmd_config_get *args);
int isp_cmd_set_isp_pmu_base(struct apple_isp *isp, u64 pmu_base);
int isp_cmd_set_dsid_clr_req_base(struct apple_isp *isp, u64 dsid_clr_base,
				  u32 dsid_clr_range);
int isp_cmd_set_dsid_clr_req_base2(struct apple_isp *isp, u64 dsid_clr_base0,
				   u64 dsid_clr_base1, u64 dsid_clr_base2,
				   u64 dsid_clr_base3, u32 dsid_clr_range0,
				   u32 dsid_clr_range1, u32 dsid_clr_range2,
				   u32 dsid_clr_range3);
int isp_cmd_pmp_ctrl_set(struct apple_isp *isp, u64 clock_scratch,
			 u64 clock_base, u8 clock_bit, u8 clock_size,
			 u64 bandwidth_scratch, u64 bandwidth_base,
			 u8 bandwidth_bit, u8 bandwidth_size);
int isp_cmd_fid_enter(struct apple_isp *isp);
int isp_cmd_fid_exit(struct apple_isp *isp);

struct cmd_ch_start {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_ch_start) == 0xc);

struct cmd_ch_stop {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_ch_stop) == 0xc);

struct cmd_ch_info {
	u64 opcode;
	u32 chan;
	u32 unk_c;  // 0x7da0001, 0x7db0001
	u32 unk_10; // 0x300ac, 0x5006d
	u32 unk_14; // 0x40007, 0x10007
	u32 unk_18; // 0x5, 0x2
	u32 unk_1c; // 0x1, 0x1
	u32 version;
	u32 unk_24; // 0x7, 0x9
	u32 unk_28; // 0x1, 0x1410
	u32 unk_2c; // 0x7, 0x2
	u32 pad_30[7];
	u32 unk_4c; // 0x10000, 0x50000
	u32 unk_50; // 0x1, 0x1
	u32 unk_54; // 0x0, 0x0
	u32 unk_58; // 0x4, 0x4
	u32 unk_5c; // 0x10, 0x20
	u32 num_presets;
	u32 unk_64; // 0x0, 0x0
	u32 unk_68; // 0x44c0, 0x4680
	u32 unk_6c; // 0x40, 0x40
	u32 unk_70; // 0x1, 0x1
	u32 unk_74; // 0x2, 0x2
	u32 unk_78; // 0x4000, 0x4000
	u32 unk_7c; // 0x40, 0x40
	u32 unk_80; // 0x1, 0x1
	u32 pad_84[2];
	u32 unk_8c; // 0x36, 0x36
	u32 pad_90[2];
	u32 timestamp_freq;
	u16 pad_9c;
	char module_sn[20];
	u16 pad_b0;
	u32 unk_b4; // 0x8, 0x8
	u32 pad_b8[2];
	u32 unk_c0; // 0x4, 0x1
	u32 unk_c4; // 0x0, 0x0
	u32 unk_c8; // 0x0, 0x100
	u32 pad_cc[4];
	u32 unk_dc; // 0xff0000, 0xff0000
	u32 unk_e0; // 0xc00, 0xc00
	u32 unk_e4; // 0x0, 0x0
	u32 unk_e8; // 0x1c, 0x1c
	u32 unk_ec; // 0x640, 0x680
	u32 unk_f0; // 0x4, 0x4
	u32 unk_f4; // 0x4, 0x4
	u32 pad_f8[6];
	u32 unk_110; // 0x0, 0x7800000
	u32 unk_114; // 0x0, 0x780
} __packed;
static_assert(sizeof(struct cmd_ch_info) == 0x118);

struct cmd_ch_camera_config {
	u64 opcode;
	u32 chan;
	u32 preset;
	u16 in_width;
	u16 in_height;
	u16 out_width;
	u16 out_height;
	u32 unk_28;
	u32 unk_2c;
	u32 unk_30[16];
	u32 sensor_clk;
	u32 unk_64[4];
	u32 timestamp_freq;
	u32 unk_78[2];
	u32 unk_80[16];
	u32 in_width2; // repeated in u32??
	u32 in_height2;
	u32 unk_c8[3];
	u32 out_width2;
	u32 out_height2;
} __packed;
static_assert(sizeof(struct cmd_ch_camera_config) == 0xdc);

struct cmd_ch_camera_config_select {
	u64 opcode;
	u32 chan;
	u32 preset;
} __packed;
static_assert(sizeof(struct cmd_ch_camera_config_select) == 0x10);

struct cmd_ch_set_file_load {
	u64 opcode;
	u32 chan;
	u32 addr;
	u32 size;
} __packed;
static_assert(sizeof(struct cmd_ch_set_file_load) == 0x14);

struct cmd_ch_set_file_load64 {
	u64 opcode;
	u32 chan;
	u64 addr;
	u32 size;
} __packed;
static_assert(sizeof(struct cmd_ch_set_file_load64) == 0x18);

struct cmd_ch_buffer_return {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_ch_buffer_return) == 0xc);

struct cmd_ch_sbs_enable {
	u64 opcode;
	u32 chan;
	u32 enable;
} __packed;
static_assert(sizeof(struct cmd_ch_sbs_enable) == 0x10);

struct cmd_ch_crop_set {
	u64 opcode;
	u32 chan;
	u32 x1;
	u32 y1;
	u32 x2;
	u32 y2;
} __packed;
static_assert(sizeof(struct cmd_ch_crop_set) == 0x1c);

struct cmd_ch_output_config_set {
	u64 opcode;
	u32 chan;
	u32 width;
	u32 height;
	u32 colorspace;
	u32 format;
	u32 strides[3];
	u32 padding_rows;
	u32 unk_h0;
	u32 compress;
	u32 unk_w2;
} __packed;
static_assert(sizeof(struct cmd_ch_output_config_set) == 0x38);

struct cmd_ch_preview_stream_set {
	u64 opcode;
	u32 chan;
	u32 stream;
} __packed;
static_assert(sizeof(struct cmd_ch_preview_stream_set) == 0x10);

struct cmd_ch_als_disable {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_ch_als_disable) == 0xc);

struct cmd_ch_cnr_start {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_ch_cnr_start) == 0xc);

struct cmd_ch_mbnr_enable {
	u64 opcode;
	u32 chan;
	u32 use_case;
	u32 mode;
	u32 enable_chroma;
} __packed;
static_assert(sizeof(struct cmd_ch_mbnr_enable) == 0x18);

struct cmd_ch_sif_pixel_format_set {
	u64 opcode;
	u32 chan;
	u8 format;
	u8 type;
	u16 compress;
	u32 unk_10;
} __packed;
static_assert(sizeof(struct cmd_ch_sif_pixel_format_set) == 0x14);

struct cmd_ch_lpdp_hs_receiver_tuning_set {
	u64 opcode;
	u32 chan;
	u32 unk1;
	u32 unk2;
} __packed;
static_assert(sizeof(struct cmd_ch_lpdp_hs_receiver_tuning_set) == 0x14);

struct cmd_ch_property_write {
	u64 opcode;
	u32 chan;
	u32 prop;
	u32 val;
	u32 unk1;
	u32 unk2;
} __packed;
static_assert(sizeof(struct cmd_ch_property_write) == 0x1c);

int isp_cmd_ch_start(struct apple_isp *isp, u32 chan);
int isp_cmd_ch_stop(struct apple_isp *isp, u32 chan);
int isp_cmd_ch_info_get(struct apple_isp *isp, u32 chan,
			struct cmd_ch_info *args);
int isp_cmd_ch_camera_config_get(struct apple_isp *isp, u32 chan, u32 preset,
				 struct cmd_ch_camera_config *args);
int isp_cmd_ch_camera_config_current_get(struct apple_isp *isp, u32 chan,
					 struct cmd_ch_camera_config *args);
int isp_cmd_ch_camera_config_select(struct apple_isp *isp, u32 chan,
				    u32 preset);
int isp_cmd_ch_set_file_load(struct apple_isp *isp, u32 chan, u64 addr,
			     u32 size);
int isp_cmd_ch_buffer_return(struct apple_isp *isp, u32 chan);
int isp_cmd_ch_sbs_enable(struct apple_isp *isp, u32 chan, u32 enable);
int isp_cmd_ch_crop_set(struct apple_isp *isp, u32 chan, u32 x1, u32 y1, u32 x2,
			u32 y2);
int isp_cmd_ch_output_config_set(struct apple_isp *isp, u32 chan, u32 width,
				 u32 height, u32 strides[3], u32 colorspace, u32 format);
int isp_cmd_ch_preview_stream_set(struct apple_isp *isp, u32 chan, u32 stream);
int isp_cmd_ch_als_disable(struct apple_isp *isp, u32 chan);
int isp_cmd_ch_cnr_start(struct apple_isp *isp, u32 chan);
int isp_cmd_ch_mbnr_enable(struct apple_isp *isp, u32 chan, u32 use_case,
			   u32 mode, u32 enable_chroma);
int isp_cmd_ch_sif_pixel_format_set(struct apple_isp *isp, u32 chan);
int isp_cmd_ch_lpdp_hs_receiver_tuning_set(struct apple_isp *isp, u32 chan, u32 unk1, u32 unk2);

int isp_cmd_ch_property_read(struct apple_isp *isp, u32 chan, u32 prop, u32 *val);
int isp_cmd_ch_property_write(struct apple_isp *isp, u32 chan, u32 prop, u32 val);

enum isp_mbnr_mode {
	ISP_MBNR_MODE_DISABLE = 0,
	ISP_MBNR_MODE_ENABLE = 1,
	ISP_MBNR_MODE_BYPASS = 2,
};

struct cmd_ch_buffer_recycle_mode_set {
	u64 opcode;
	u32 chan;
	u32 mode;
} __packed;
static_assert(sizeof(struct cmd_ch_buffer_recycle_mode_set) == 0x10);

struct cmd_ch_buffer_recycle_start {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_ch_buffer_recycle_start) == 0xc);

struct cmd_ch_buffer_pool_config_set {
	u64 opcode;
	u32 chan;
	u16 type;
	u16 count;
	u32 meta_size0;
	u32 meta_size1;
	u64 unk0;
	u64 unk1;
	u64 unk2;
	u32 zero[0x19];
	u32 data_blocks;
	u32 compress;
} __packed;
static_assert(sizeof(struct cmd_ch_buffer_pool_config_set) == 0x9c);

struct cmd_ch_buffer_pool_return {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_ch_buffer_pool_return) == 0xc);

int isp_cmd_ch_buffer_recycle_mode_set(struct apple_isp *isp, u32 chan,
				       u32 mode);
int isp_cmd_ch_buffer_recycle_start(struct apple_isp *isp, u32 chan);
int isp_cmd_ch_buffer_pool_config_set(struct apple_isp *isp, u32 chan,
				      u16 type);
int isp_cmd_ch_buffer_pool_config_get(struct apple_isp *isp, u32 chan,
				      u16 type);
int isp_cmd_ch_buffer_pool_return(struct apple_isp *isp, u32 chan);

struct cmd_apple_ch_temporal_filter_start {
	u64 opcode;
	u32 chan;
	u32 unk_c;
	u32 unk_10;
} __packed;
static_assert(sizeof(struct cmd_apple_ch_temporal_filter_start) == 0x14);

struct cmd_apple_ch_temporal_filter_stop {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_apple_ch_temporal_filter_stop) == 0xc);

struct cmd_apple_ch_motion_history_start {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_apple_ch_motion_history_start) == 0xc);

struct cmd_apple_ch_motion_history_stop {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_apple_ch_motion_history_stop) == 0xc);

struct cmd_apple_ch_temporal_filter_enable {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_apple_ch_temporal_filter_enable) == 0xc);

struct cmd_apple_ch_temporal_filter_disable {
	u64 opcode;
	u32 chan;
} __packed;
static_assert(sizeof(struct cmd_apple_ch_temporal_filter_disable) == 0xc);

int isp_cmd_apple_ch_temporal_filter_start(struct apple_isp *isp, u32 chan);
int isp_cmd_apple_ch_temporal_filter_stop(struct apple_isp *isp, u32 chan);
int isp_cmd_apple_ch_motion_history_start(struct apple_isp *isp, u32 chan);
int isp_cmd_apple_ch_motion_history_stop(struct apple_isp *isp, u32 chan);
int isp_cmd_apple_ch_temporal_filter_enable(struct apple_isp *isp, u32 chan);
int isp_cmd_apple_ch_temporal_filter_disable(struct apple_isp *isp, u32 chan);

struct cmd_ch_ae_stability_set {
	u64 opcode;
	u32 chan;
	u32 stability;
} __packed;
static_assert(sizeof(struct cmd_ch_ae_stability_set) == 0x10);

struct cmd_ch_ae_stability_to_stable_set {
	u64 opcode;
	u32 chan;
	u32 stability;
} __packed;
static_assert(sizeof(struct cmd_ch_ae_stability_to_stable_set) == 0x10);

struct cmd_ch_ae_frame_rate_max_get {
	u64 opcode;
	u32 chan;
	u32 framerate;
} __packed;
static_assert(sizeof(struct cmd_ch_ae_frame_rate_max_get) == 0x10);

struct cmd_ch_ae_frame_rate_max_set {
	u64 opcode;
	u32 chan;
	u32 framerate;
} __packed;
static_assert(sizeof(struct cmd_ch_ae_frame_rate_max_set) == 0x10);

struct cmd_ch_ae_frame_rate_min_set {
	u64 opcode;
	u32 chan;
	u32 framerate;
} __packed;
static_assert(sizeof(struct cmd_ch_ae_frame_rate_min_set) == 0x10);

struct cmd_apple_ch_ae_fd_scene_metering_config_set {
	u64 opcode;
	u32 chan;
	u32 unk_c;
	u32 unk_10;
	u32 unk_14;
	u32 unk_18;
	u32 unk_1c;
	u32 unk_20;
} __packed;
static_assert(sizeof(struct cmd_apple_ch_ae_fd_scene_metering_config_set) ==
	      0x24);

struct cmd_apple_ch_ae_metering_mode_set {
	u64 opcode;
	u32 chan;
	u32 mode;
} __packed;
static_assert(sizeof(struct cmd_apple_ch_ae_metering_mode_set) == 0x10);

struct cmd_apple_ch_ae_flicker_freq_update_current_set {
	u64 opcode;
	u32 chan;
	u32 freq;
} __packed;
static_assert(sizeof(struct cmd_apple_ch_ae_flicker_freq_update_current_set) ==
	      0x10);

int isp_cmd_ch_ae_stability_set(struct apple_isp *isp, u32 chan, u32 stability);
int isp_cmd_ch_ae_stability_to_stable_set(struct apple_isp *isp, u32 chan,
					  u32 stability);
int isp_cmd_ch_ae_frame_rate_max_get(struct apple_isp *isp, u32 chan,
				     struct cmd_ch_ae_frame_rate_max_get *args);
int isp_cmd_ch_ae_frame_rate_max_set(struct apple_isp *isp, u32 chan,
				     u32 framerate);
int isp_cmd_ch_ae_frame_rate_min_set(struct apple_isp *isp, u32 chan,
				     u32 framerate);
int isp_cmd_apple_ch_ae_fd_scene_metering_config_set(struct apple_isp *isp,
						     u32 chan);
int isp_cmd_apple_ch_ae_metering_mode_set(struct apple_isp *isp, u32 chan,
					  u32 mode);
int isp_cmd_apple_ch_ae_flicker_freq_update_current_set(struct apple_isp *isp,
							u32 chan, u32 freq);

struct cmd_ch_semantic_video_enable {
	u64 opcode;
	u32 chan;
	u32 enable;
} __packed;
static_assert(sizeof(struct cmd_ch_semantic_video_enable) == 0x10);

struct cmd_ch_semantic_awb_enable {
	u64 opcode;
	u32 chan;
	u32 enable;
} __packed;
static_assert(sizeof(struct cmd_ch_semantic_awb_enable) == 0x10);

int isp_cmd_ch_semantic_video_enable(struct apple_isp *isp, u32 chan,
				     u32 enable);
int isp_cmd_ch_semantic_awb_enable(struct apple_isp *isp, u32 chan, u32 enable);

#endif /* __ISP_CMD_H__ */
