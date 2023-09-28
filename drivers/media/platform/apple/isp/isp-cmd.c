// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */

#include "isp-cmd.h"
#include "isp-iommu.h"
#include "isp-ipc.h"

#define CISP_OPCODE_SHIFT     32UL
#define CISP_OPCODE(x)	      (((u64)(x)) << CISP_OPCODE_SHIFT)
#define CISP_OPCODE_GET(x)    (((u64)(x)) >> CISP_OPCODE_SHIFT)

#define CISP_TIMEOUT	      msecs_to_jiffies(3000)
#define CISP_SEND_IN(x, a)    (cisp_send((x), &(a), sizeof(a), 0, CISP_TIMEOUT))
#define CISP_SEND_INOUT(x, a) (cisp_send((x), &(a), sizeof(a), sizeof(a), CISP_TIMEOUT))
#define CISP_SEND_OUT(x, a)   (cisp_send_read((x), (a), sizeof(*a), sizeof(*a)))
#define CISP_POST_IN(x, a)    (cisp_send((x), &(a), sizeof(a), 0, 0))

static int cisp_send(struct apple_isp *isp, void *args, u32 insize, u32 outsize, int timeout)
{
	struct isp_channel *chan = isp->chan_io;
	struct isp_message *req = &chan->req;
	int err;

	req->arg0 = isp->cmd_iova;
	req->arg1 = insize;
	req->arg2 = outsize;

	memcpy(isp->cmd_virt, args, insize);
	err = ipc_chan_send(isp, chan, timeout);
	if (err) {
		u64 opcode;
		memcpy(&opcode, args, sizeof(opcode));
		dev_err(isp->dev,
			"%s: failed to send OPCODE 0x%04llx: [0x%llx, 0x%llx, 0x%llx]\n",
			chan->name, CISP_OPCODE_GET(opcode), req->arg0,
			req->arg1, req->arg2);
	}

	return err;
}

static int cisp_send_read(struct apple_isp *isp, void *args, u32 insize,
			  u32 outsize)
{
	/* TODO do I need to lock the iova space? */
	int err = cisp_send(isp, args, insize, outsize, CISP_TIMEOUT);
	if (err)
		return err;

	memcpy(args, isp->cmd_virt, outsize);
	return 0;
}

int isp_cmd_start(struct apple_isp *isp, u32 mode)
{
	struct cmd_start args = {
		.opcode = CISP_OPCODE(CISP_CMD_START),
		.mode = mode,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_stop(struct apple_isp *isp, u32 mode)
{
	struct cmd_stop args = {
		.opcode = CISP_OPCODE(CISP_CMD_STOP),
		.mode = mode,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_power_down(struct apple_isp *isp)
{
	struct cmd_power_down args = {
		.opcode = CISP_OPCODE(CISP_CMD_POWER_DOWN),
	};
	return CISP_POST_INOUT(isp, args);
}

int isp_cmd_suspend(struct apple_isp *isp)
{
	struct cmd_suspend args = {
		.opcode = CISP_OPCODE(CISP_CMD_SUSPEND),
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_print_enable(struct apple_isp *isp, u32 enable)
{
	struct cmd_print_enable args = {
		.opcode = CISP_OPCODE(CISP_CMD_PRINT_ENABLE),
		.enable = enable,
	};
	return CISP_SEND_INOUT(isp, args);
}

int isp_cmd_trace_enable(struct apple_isp *isp, u32 enable)
{
	struct cmd_trace_enable args = {
		.opcode = CISP_OPCODE(CISP_CMD_TRACE_ENABLE),
		.enable = enable,
	};
	return CISP_SEND_INOUT(isp, args);
}

int isp_cmd_config_get(struct apple_isp *isp, struct cmd_config_get *args)
{
	args->opcode = CISP_OPCODE(CISP_CMD_CONFIG_GET);
	return CISP_SEND_OUT(isp, args);
}

int isp_cmd_set_isp_pmu_base(struct apple_isp *isp, u64 pmu_base)
{
	struct cmd_set_isp_pmu_base args = {
		.opcode = CISP_OPCODE(CISP_CMD_SET_ISP_PMU_BASE),
		.pmu_base = pmu_base,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_set_dsid_clr_req_base2(struct apple_isp *isp, u64 dsid_clr_base0,
				   u64 dsid_clr_base1, u64 dsid_clr_base2,
				   u64 dsid_clr_base3, u32 dsid_clr_range0,
				   u32 dsid_clr_range1, u32 dsid_clr_range2,
				   u32 dsid_clr_range3)
{
	struct cmd_set_dsid_clr_req_base2 args = {
		.opcode = CISP_OPCODE(CISP_CMD_SET_DSID_CLR_REG_BASE2),
		.dsid_clr_base0 = dsid_clr_base0,
		.dsid_clr_base1 = dsid_clr_base1,
		.dsid_clr_base2 = dsid_clr_base2,
		.dsid_clr_base3 = dsid_clr_base3,
		.dsid_clr_range0 = dsid_clr_range0,
		.dsid_clr_range1 = dsid_clr_range1,
		.dsid_clr_range2 = dsid_clr_range2,
		.dsid_clr_range3 = dsid_clr_range3,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_set_dsid_clr_req_base(struct apple_isp *isp, u64 dsid_clr_base,
				  u32 dsid_clr_range)
{
	struct cmd_set_dsid_clr_req_base args = {
		.opcode = CISP_OPCODE(CISP_CMD_SET_DSID_CLR_REG_BASE),
		.dsid_clr_base = dsid_clr_base,
		.dsid_clr_range = dsid_clr_range,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_pmp_ctrl_set(struct apple_isp *isp, u64 clock_scratch,
			 u64 clock_base, u8 clock_bit, u8 clock_size,
			 u64 bandwidth_scratch, u64 bandwidth_base,
			 u8 bandwidth_bit, u8 bandwidth_size)
{
	struct cmd_pmp_ctrl_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_PMP_CTRL_SET),
		.clock_scratch = clock_scratch,
		.clock_base = clock_base,
		.clock_bit = clock_bit,
		.clock_size = clock_size,
		.clock_pad = 0,
		.bandwidth_scratch = bandwidth_scratch,
		.bandwidth_base = bandwidth_base,
		.bandwidth_bit = bandwidth_bit,
		.bandwidth_size = bandwidth_size,
		.bandwidth_pad = 0,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_fid_enter(struct apple_isp *isp)
{
	struct cmd_fid_enter args = {
		.opcode = CISP_OPCODE(CISP_CMD_FID_ENTER),
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_fid_exit(struct apple_isp *isp)
{
	struct cmd_fid_exit args = {
		.opcode = CISP_OPCODE(CISP_CMD_FID_EXIT),
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_start(struct apple_isp *isp, u32 chan)
{
	struct cmd_ch_start args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_START),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_stop(struct apple_isp *isp, u32 chan)
{
	struct cmd_ch_stop args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_STOP),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_info_get(struct apple_isp *isp, u32 chan,
			struct cmd_ch_info *args)
{
	args->opcode = CISP_OPCODE(CISP_CMD_CH_INFO_GET);
	args->chan = chan;
	return CISP_SEND_OUT(isp, args);
}

int isp_cmd_ch_camera_config_get(struct apple_isp *isp, u32 chan, u32 preset,
				 struct cmd_ch_camera_config *args)
{
	args->opcode = CISP_OPCODE(CISP_CMD_CH_CAMERA_CONFIG_GET);
	args->preset = preset;
	args->chan = chan;
	return CISP_SEND_OUT(isp, args);
}

int isp_cmd_ch_camera_config_current_get(struct apple_isp *isp, u32 chan,
					 struct cmd_ch_camera_config *args)
{
	args->opcode = CISP_OPCODE(CISP_CMD_CH_CAMERA_CONFIG_CURRENT_GET);
	args->chan = chan;
	return CISP_SEND_OUT(isp, args);
}

int isp_cmd_ch_camera_config_select(struct apple_isp *isp, u32 chan, u32 preset)
{
	struct cmd_ch_camera_config_select args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_CAMERA_CONFIG_SELECT),
		.chan = chan,
		.preset = preset,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_buffer_return(struct apple_isp *isp, u32 chan)
{
	struct cmd_ch_buffer_return args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_BUFFER_RETURN),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_set_file_load(struct apple_isp *isp, u32 chan, u64 addr,
			     u32 size)
{
	if (isp->hw->gen >= ISP_GEN_T8112) {
		struct cmd_ch_set_file_load64 args = {
			.opcode = CISP_OPCODE(CISP_CMD_CH_SET_FILE_LOAD),
			.chan = chan,
			.addr = addr,
			.size = size,
		};
		return CISP_SEND_IN(isp, args);
	} else {
		struct cmd_ch_set_file_load args = {
			.opcode = CISP_OPCODE(CISP_CMD_CH_SET_FILE_LOAD),
			.chan = chan,
			.addr = addr,
			.size = size,
		};
		return CISP_SEND_IN(isp, args);
	}
}

int isp_cmd_ch_sbs_enable(struct apple_isp *isp, u32 chan, u32 enable)
{
	struct cmd_ch_sbs_enable args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_SBS_ENABLE),
		.chan = chan,
		.enable = enable,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_crop_set(struct apple_isp *isp, u32 chan, u32 x1, u32 y1, u32 x2,
			u32 y2)
{
	struct cmd_ch_crop_set args = {
		.opcode = CISP_OPCODE(isp->hw->scl1 ? CISP_CMD_CH_CROP_SCL1_SET
				      : CISP_CMD_CH_CROP_SET),
		.chan = chan,
		.x1 = x1,
		.y1 = y1,
		.x2 = x2,
		.y2 = y2,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_output_config_set(struct apple_isp *isp, u32 chan, u32 width,
				 u32 height, u32 strides[3], u32 colorspace, u32 format)
{
	struct cmd_ch_output_config_set args = {
		.opcode = CISP_OPCODE(isp->hw->scl1 ? CISP_CMD_CH_OUTPUT_CONFIG_SCL1_SET
				      : CISP_CMD_CH_OUTPUT_CONFIG_SET),
		.chan = chan,
		.width = width,
		.height = height,
		.colorspace = colorspace,
		.format = format,
		.padding_rows = 0,
		.unk_h0 = height,
		.compress = 0,
		.unk_w2 = width,
	};
	memcpy(args.strides, strides, sizeof(args.strides));
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_preview_stream_set(struct apple_isp *isp, u32 chan, u32 stream)
{
	struct cmd_ch_preview_stream_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_PREVIEW_STREAM_SET),
		.chan = chan,
		.stream = stream,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_als_disable(struct apple_isp *isp, u32 chan)
{
	struct cmd_ch_als_disable args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_ALS_DISABLE),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_cnr_start(struct apple_isp *isp, u32 chan)
{
	struct cmd_ch_cnr_start args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_CNR_START),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_mbnr_enable(struct apple_isp *isp, u32 chan, u32 use_case,
			   u32 mode, u32 enable_chroma)
{
	struct cmd_ch_mbnr_enable args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_MBNR_ENABLE),
		.chan = chan,
		.use_case = use_case,
		.mode = mode,
		.enable_chroma = enable_chroma,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_sif_pixel_format_set(struct apple_isp *isp, u32 chan)
{
	struct cmd_ch_sif_pixel_format_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_SIF_PIXEL_FORMAT_SET),
		.chan = chan,
		.format = 3,
		.type = 1,
		.compress = 0,
		.unk_10 = 0,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_buffer_recycle_mode_set(struct apple_isp *isp, u32 chan,
				       u32 mode)
{
	struct cmd_ch_buffer_recycle_mode_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_BUFFER_RECYCLE_MODE_SET),
		.chan = chan,
		.mode = mode,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_buffer_recycle_start(struct apple_isp *isp, u32 chan)
{
	struct cmd_ch_buffer_recycle_start args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_BUFFER_RECYCLE_START),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_buffer_pool_config_set(struct apple_isp *isp, u32 chan, u16 type)
{
	struct cmd_ch_buffer_pool_config_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_BUFFER_POOL_CONFIG_SET),
		.chan = chan,
		.type = type,
		.count = ISP_MAX_BUFFERS,
		.meta_size0 = isp->hw->meta_size,
		.meta_size1 = isp->hw->meta_size,
		.unk0 = 0,
		.unk1 = 0,
		.unk2 = 0,
		.data_blocks = 1,
		.compress = 0,
	};
	return CISP_SEND_INOUT(isp, args);
}

int isp_cmd_ch_buffer_pool_return(struct apple_isp *isp, u32 chan)
{
	struct cmd_ch_buffer_pool_return args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_BUFFER_POOL_RETURN),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_apple_ch_temporal_filter_start(struct apple_isp *isp, u32 chan)
{
	struct cmd_apple_ch_temporal_filter_start args = {
		.opcode = CISP_OPCODE(CISP_CMD_APPLE_CH_TEMPORAL_FILTER_START),
		.chan = chan,
		.unk_c = 1,
		.unk_10 = 0,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_apple_ch_temporal_filter_stop(struct apple_isp *isp, u32 chan)
{
	struct cmd_apple_ch_temporal_filter_stop args = {
		.opcode = CISP_OPCODE(CISP_CMD_APPLE_CH_TEMPORAL_FILTER_STOP),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_apple_ch_motion_history_start(struct apple_isp *isp, u32 chan)
{
	struct cmd_apple_ch_motion_history_start args = {
		.opcode = CISP_OPCODE(CISP_CMD_APPLE_CH_MOTION_HISTORY_START),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_apple_ch_motion_history_stop(struct apple_isp *isp, u32 chan)
{
	struct cmd_apple_ch_motion_history_stop args = {
		.opcode = CISP_OPCODE(CISP_CMD_APPLE_CH_MOTION_HISTORY_STOP),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_apple_ch_temporal_filter_enable(struct apple_isp *isp, u32 chan)
{
	struct cmd_apple_ch_temporal_filter_enable args = {
		.opcode = CISP_OPCODE(CISP_CMD_APPLE_CH_TEMPORAL_FILTER_ENABLE),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_apple_ch_temporal_filter_disable(struct apple_isp *isp, u32 chan)
{
	struct cmd_apple_ch_temporal_filter_disable args = {
		.opcode =
			CISP_OPCODE(CISP_CMD_APPLE_CH_TEMPORAL_FILTER_DISABLE),
		.chan = chan,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_ae_stability_set(struct apple_isp *isp, u32 chan, u32 stability)
{
	struct cmd_ch_ae_stability_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_AE_STABILITY_SET),
		.chan = chan,
		.stability = stability,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_ae_stability_to_stable_set(struct apple_isp *isp, u32 chan,
					  u32 stability)
{
	struct cmd_ch_ae_stability_to_stable_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_AE_STABILITY_TO_STABLE_SET),
		.chan = chan,
		.stability = stability,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_ae_frame_rate_max_get(struct apple_isp *isp, u32 chan,
				     struct cmd_ch_ae_frame_rate_max_get *args)
{
	args->opcode = CISP_OPCODE(CISP_CMD_CH_AE_FRAME_RATE_MAX_GET);
	args->chan = chan;
	return CISP_SEND_OUT(isp, args);
}

int isp_cmd_ch_ae_frame_rate_max_set(struct apple_isp *isp, u32 chan,
				     u32 framerate)
{
	struct cmd_ch_ae_frame_rate_max_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_AE_FRAME_RATE_MAX_SET),
		.chan = chan,
		.framerate = framerate,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_ae_frame_rate_min_set(struct apple_isp *isp, u32 chan,
				     u32 framerate)
{
	struct cmd_ch_ae_frame_rate_min_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_AE_FRAME_RATE_MIN_SET),
		.chan = chan,
		.framerate = framerate,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_apple_ch_ae_fd_scene_metering_config_set(struct apple_isp *isp,
						     u32 chan)
{
	struct cmd_apple_ch_ae_fd_scene_metering_config_set args = {
		.opcode = CISP_OPCODE(
			CISP_CMD_APPLE_CH_AE_FD_SCENE_METERING_CONFIG_SET),
		.chan = chan,
		.unk_c = 0xb8,
		.unk_10 = 0x2000200,
		.unk_14 = 0x280800,
		.unk_18 = 0xe10028,
		.unk_1c = 0xa0399,
		.unk_20 = 0x3cc02cc,
	};
	return CISP_SEND_INOUT(isp, args);
}

int isp_cmd_apple_ch_ae_metering_mode_set(struct apple_isp *isp, u32 chan,
					  u32 mode)
{
	struct cmd_apple_ch_ae_metering_mode_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_APPLE_CH_AE_METERING_MODE_SET),
		.chan = chan,
		.mode = mode,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_apple_ch_ae_flicker_freq_update_current_set(struct apple_isp *isp,
							u32 chan, u32 freq)
{
	struct cmd_apple_ch_ae_flicker_freq_update_current_set args = {
		.opcode = CISP_OPCODE(
			CISP_CMD_APPLE_CH_AE_FLICKER_FREQ_UPDATE_CURRENT_SET),
		.chan = chan,
		.freq = freq,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_semantic_video_enable(struct apple_isp *isp, u32 chan,
				     u32 enable)
{
	struct cmd_ch_semantic_video_enable args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_SEMANTIC_VIDEO_ENABLE),
		.chan = chan,
		.enable = enable,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_semantic_awb_enable(struct apple_isp *isp, u32 chan, u32 enable)
{
	struct cmd_ch_semantic_awb_enable args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_SEMANTIC_AWB_ENABLE),
		.chan = chan,
		.enable = enable,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_lpdp_hs_receiver_tuning_set(struct apple_isp *isp, u32 chan, u32 unk1, u32 unk2)
{
	struct cmd_ch_lpdp_hs_receiver_tuning_set args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_LPDP_HS_RECEIVER_TUNING_SET),
		.chan = chan,
		.unk1 = unk1,
		.unk2 = unk2,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_property_write(struct apple_isp *isp, u32 chan, u32 prop, u32 val)
{
	struct cmd_ch_property_write args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_PROPERTY_WRITE),
		.chan = chan,
		.prop = prop,
		.val = val,
	};
	return CISP_SEND_IN(isp, args);
}

int isp_cmd_ch_property_read(struct apple_isp *isp, u32 chan, u32 prop, u32 *val)
{
	struct cmd_ch_property_write args = {
		.opcode = CISP_OPCODE(CISP_CMD_CH_PROPERTY_READ),
		.chan = chan,
		.prop = prop,
		.val = 0xdeadbeef,
	};
	int ret = CISP_SEND_OUT(isp, &args);

	*val = args.val;

	return ret;
}
