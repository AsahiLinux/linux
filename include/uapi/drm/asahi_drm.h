/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on asahi_drm.h which is
 *
 * Copyright © 2014-2018 Broadcom
 * Copyright © 2019 Collabora ltd.
 */
#ifndef _ASAHI_DRM_H_
#define _ASAHI_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define DRM_ASAHI_UNSTABLE_UABI_VERSION		10011

#define DRM_ASAHI_GET_PARAMS			0x00
#define DRM_ASAHI_VM_CREATE			0x01
#define DRM_ASAHI_VM_DESTROY			0x02
#define DRM_ASAHI_GEM_CREATE			0x03
#define DRM_ASAHI_GEM_MMAP_OFFSET		0x04
#define DRM_ASAHI_GEM_BIND			0x05
#define DRM_ASAHI_QUEUE_CREATE			0x06
#define DRM_ASAHI_QUEUE_DESTROY			0x07
#define DRM_ASAHI_SUBMIT			0x08
#define DRM_ASAHI_GET_TIME			0x09

#define DRM_ASAHI_MAX_CLUSTERS	32

struct drm_asahi_params_global {
	__u32 unstable_uabi_version;
	__u32 pad0;

	__u64 feat_compat;
	__u64 feat_incompat;

	__u32 gpu_generation;
	__u32 gpu_variant;
	__u32 gpu_revision;
	__u32 chip_id;

	__u32 num_dies;
	__u32 num_clusters_total;
	__u32 num_cores_per_cluster;
	__u32 num_frags_per_cluster;
	__u32 num_gps_per_cluster;
	__u32 num_cores_total_active;
	__u64 core_masks[DRM_ASAHI_MAX_CLUSTERS];

	__u32 vm_page_size;
	__u32 pad1;
	__u64 vm_user_start;
	__u64 vm_user_end;
	__u64 vm_usc_start;
	__u64 vm_usc_end;
	__u64 vm_kernel_min_size;

	__u32 max_syncs_per_submission;
	__u32 max_commands_per_submission;
	__u32 max_commands_in_flight;
	__u32 max_attachments;

	__u32 timer_frequency_hz;
	__u32 min_frequency_khz;
	__u32 max_frequency_khz;
	__u32 max_power_mw;

	__u32 result_render_size;
	__u32 result_compute_size;

	__u32 firmware_version[4];
};

enum drm_asahi_feat_compat {
	DRM_ASAHI_FEAT_SOFT_FAULTS = (1UL) << 0,
};

enum drm_asahi_feat_incompat {
	DRM_ASAHI_FEAT_MANDATORY_ZS_COMPRESSION = (1UL) << 0,
};

struct drm_asahi_get_params {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @param: Parameter group to fetch (MBZ) */
	__u32 param_group;

	/** @pad: MBZ */
	__u32 pad;

	/** @value: User pointer to write parameter struct */
	__u64 pointer;

	/** @value: Size of user buffer, max size supported on return */
	__u64 size;
};

struct drm_asahi_vm_create {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @kernel_start: Start of the kernel-reserved address range */
	__u64 kernel_start;

	/** @kernel_end: End of the kernel-reserved address range */
	__u64 kernel_end;

	/** @value: Returned VM ID */
	__u32 vm_id;

	/** @pad: MBZ */
	__u32 pad;
};

struct drm_asahi_vm_destroy {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @value: VM ID to be destroyed */
	__u32 vm_id;

	/** @pad: MBZ */
	__u32 pad;
};

#define ASAHI_GEM_WRITEBACK	(1L << 0)
#define ASAHI_GEM_VM_PRIVATE	(1L << 1)

struct drm_asahi_gem_create {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @size: Size of the BO */
	__u64 size;

	/** @flags: BO creation flags */
	__u32 flags;

	/** @handle: VM ID to assign to the BO, if ASAHI_GEM_VM_PRIVATE is set. */
	__u32 vm_id;

	/** @handle: Returned GEM handle for the BO */
	__u32 handle;
};

struct drm_asahi_gem_mmap_offset {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @handle: Handle for the object being mapped. */
	__u32 handle;

	/** @flags: Must be zero */
	__u32 flags;

	/** @offset: The fake offset to use for subsequent mmap call */
	__u64 offset;
};

enum drm_asahi_bind_op {
	ASAHI_BIND_OP_BIND = 0,
	ASAHI_BIND_OP_UNBIND = 1,
	ASAHI_BIND_OP_UNBIND_ALL = 2,
};

#define ASAHI_BIND_READ		(1L << 0)
#define ASAHI_BIND_WRITE	(1L << 1)

struct drm_asahi_gem_bind {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @obj: Bind operation */
	__u32 op;

	/** @flags: One or more of ASAHI_BIND_* */
	__u32 flags;

	/** @obj: GEM object to bind */
	__u32 handle;

	/** @vm_id: The ID of the VM to bind to */
	__u32 vm_id;

	/** @offset: Offset into the object */
	__u64 offset;

	/** @range: Number of bytes from the object to bind to addr */
	__u64 range;

	/** @addr: Address to bind to */
	__u64 addr;
};

enum drm_asahi_cmd_type {
	DRM_ASAHI_CMD_RENDER = 0,
	DRM_ASAHI_CMD_BLIT = 1,
	DRM_ASAHI_CMD_COMPUTE = 2,
};

/* Note: this is an enum so that it can be resolved by Rust bindgen. */
enum drm_asahi_queue_cap {
	DRM_ASAHI_QUEUE_CAP_RENDER	= (1UL << DRM_ASAHI_CMD_RENDER),
	DRM_ASAHI_QUEUE_CAP_BLIT	= (1UL << DRM_ASAHI_CMD_BLIT),
	DRM_ASAHI_QUEUE_CAP_COMPUTE	= (1UL << DRM_ASAHI_CMD_COMPUTE),
};

struct drm_asahi_queue_create {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @flags: MBZ */
	__u32 flags;

	/** @vm_id: The ID of the VM this queue is bound to */
	__u32 vm_id;

	/** @type: Bitmask of DRM_ASAHI_QUEUE_CAP_* */
	__u32 queue_caps;

	/** @priority: Queue priority, 0-3 */
	__u32 priority;

	/** @queue_id: The returned queue ID */
	__u32 queue_id;
};

struct drm_asahi_queue_destroy {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @queue_id: The queue ID to be destroyed */
	__u32 queue_id;
};

enum drm_asahi_sync_type {
	DRM_ASAHI_SYNC_SYNCOBJ = 0,
	DRM_ASAHI_SYNC_TIMELINE_SYNCOBJ = 1,
};

struct drm_asahi_sync {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @sync_type: One of drm_asahi_sync_type */
	__u32 sync_type;

	/** @handle: The sync object handle */
	__u32 handle;

	/** @timeline_value: Timeline value for timeline sync objects */
	__u64 timeline_value;
};

enum drm_asahi_subqueue {
	DRM_ASAHI_SUBQUEUE_RENDER = 0, /* Also blit */
	DRM_ASAHI_SUBQUEUE_COMPUTE = 1,
	DRM_ASAHI_SUBQUEUE_COUNT = 2,
};

#define DRM_ASAHI_BARRIER_NONE ~(0U)

struct drm_asahi_command {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @type: One of drm_asahi_cmd_type */
	__u32 cmd_type;

	/** @flags: Flags for command submission */
	__u32 flags;

	/** @cmdbuf: Pointer to the appropriate command buffer structure */
	__u64 cmd_buffer;

	/** @cmdbuf: Size of the command buffer structure */
	__u64 cmd_buffer_size;

	/** @cmdbuf: Offset into the result BO to return information about this command */
	__u64 result_offset;

	/** @cmdbuf: Size of the result data structure */
	__u64 result_size;

	/** @barriers: Array of command indices per subqueue to wait on */
	__u32 barriers[DRM_ASAHI_SUBQUEUE_COUNT];
};

struct drm_asahi_submit {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @in_syncs: An optional array of drm_asahi_sync to wait on before starting this job. */
	__u64 in_syncs;

	/** @in_syncs: An optional array of drm_asahi_sync objects to signal upon completion. */
	__u64 out_syncs;

	/** @commands: Pointer to the drm_asahi_command array of commands to submit. */
	__u64 commands;

	/** @flags: Flags for command submission (MBZ) */
	__u32 flags;

	/** @queue_id: The queue ID to be submitted to */
	__u32 queue_id;

	/** @result_handle: An optional BO handle to place result data in */
	__u32 result_handle;

	/** @in_sync_count: Number of sync objects to wait on before starting this job. */
	__u32 in_sync_count;

	/** @in_sync_count: Number of sync objects to signal upon completion of this job. */
	__u32 out_sync_count;

	/** @pad: Number of commands to be submitted */
	__u32 command_count;
};

struct drm_asahi_attachment {
	/** @pointer: Base address of the attachment */
	__u64 pointer;
	/** @size: Size of the attachment in bytes */
	__u64 size;
	/** @order: Power of 2 exponent related to attachment size (?) */
	__u32 order;
	/** @flags: MBZ */
	__u32 flags;
};

#define ASAHI_RENDER_NO_CLEAR_PIPELINE_TEXTURES (1UL << 0)
#define ASAHI_RENDER_SET_WHEN_RELOADING_Z_OR_S (1UL << 1)
#define ASAHI_RENDER_VERTEX_SPILLS (1UL << 2)
#define ASAHI_RENDER_PROCESS_EMPTY_TILES (1UL << 3)
#define ASAHI_RENDER_NO_VERTEX_CLUSTERING (1UL << 4)
#define ASAHI_RENDER_MSAA_ZS (1UL << 5)
/* XXX check */
#define ASAHI_RENDER_NO_PREEMPTION (1UL << 6)

struct drm_asahi_cmd_render {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	__u64 flags;

	__u64 encoder_ptr;
	__u64 vertex_usc_base;
	__u64 fragment_usc_base;

	__u64 vertex_attachments;
	__u64 fragment_attachments;
	__u32 vertex_attachment_count;
	__u32 fragment_attachment_count;

	__u32 vertex_helper_program;
	__u32 fragment_helper_program;
	__u32 vertex_helper_cfg;
	__u32 fragment_helper_cfg;
	__u64 vertex_helper_arg;
	__u64 fragment_helper_arg;

	__u64 depth_buffer_load;
	__u64 depth_buffer_load_stride;
	__u64 depth_buffer_store;
	__u64 depth_buffer_store_stride;
	__u64 depth_buffer_partial;
	__u64 depth_buffer_partial_stride;
	__u64 depth_meta_buffer_load;
	__u64 depth_meta_buffer_load_stride;
	__u64 depth_meta_buffer_store;
	__u64 depth_meta_buffer_store_stride;
	__u64 depth_meta_buffer_partial;
	__u64 depth_meta_buffer_partial_stride;

	__u64 stencil_buffer_load;
	__u64 stencil_buffer_load_stride;
	__u64 stencil_buffer_store;
	__u64 stencil_buffer_store_stride;
	__u64 stencil_buffer_partial;
	__u64 stencil_buffer_partial_stride;
	__u64 stencil_meta_buffer_load;
	__u64 stencil_meta_buffer_load_stride;
	__u64 stencil_meta_buffer_store;
	__u64 stencil_meta_buffer_store_stride;
	__u64 stencil_meta_buffer_partial;
	__u64 stencil_meta_buffer_partial_stride;

	__u64 scissor_array;
	__u64 depth_bias_array;
	__u64 visibility_result_buffer;

	__u64 vertex_sampler_array;
	__u32 vertex_sampler_count;
	__u32 vertex_sampler_max;

	__u64 fragment_sampler_array;
	__u32 fragment_sampler_count;
	__u32 fragment_sampler_max;

	__u64 zls_ctrl;
	__u64 ppp_multisamplectl;
	__u32 ppp_ctrl;

	__u32 fb_width;
	__u32 fb_height;

	__u32 utile_width;
	__u32 utile_height;

	__u32 samples;
	__u32 layers;

	__u32 encoder_id;
	__u32 cmd_ta_id;
	__u32 cmd_3d_id;

	__u32 sample_size;
	__u32 tib_blocks;
	__u32 iogpu_unk_214;

	__u32 merge_upper_x;
	__u32 merge_upper_y;

	__u32 load_pipeline;
	__u32 load_pipeline_bind;

	__u32 store_pipeline;
	__u32 store_pipeline_bind;

	__u32 partial_reload_pipeline;
	__u32 partial_reload_pipeline_bind;

	__u32 partial_store_pipeline;
	__u32 partial_store_pipeline_bind;

	__u32 depth_dimensions;
	__u32 isp_bgobjdepth;
	__u32 isp_bgobjvals;
};

#define ASAHI_RENDER_UNK_UNK1			(1UL << 0)
#define ASAHI_RENDER_UNK_SET_TILE_CONFIG	(1UL << 1)
#define ASAHI_RENDER_UNK_SET_UTILE_CONFIG	(1UL << 2)
#define ASAHI_RENDER_UNK_SET_AUX_FB_UNK		(1UL << 3)
#define ASAHI_RENDER_UNK_SET_G14_UNK		(1UL << 4)

#define ASAHI_RENDER_UNK_SET_FRG_UNK_140	(1UL << 20)
#define ASAHI_RENDER_UNK_SET_FRG_UNK_158	(1UL << 21)
#define ASAHI_RENDER_UNK_SET_FRG_TILECFG	(1UL << 22)
#define ASAHI_RENDER_UNK_SET_LOAD_BGOBJVALS	(1UL << 23)
#define ASAHI_RENDER_UNK_SET_FRG_UNK_38		(1UL << 24)
#define ASAHI_RENDER_UNK_SET_FRG_UNK_3C		(1UL << 25)

#define ASAHI_RENDER_UNK_SET_RELOAD_ZLSCTRL	(1UL << 27)
#define ASAHI_RENDER_UNK_SET_UNK_BUF_10		(1UL << 28)
#define ASAHI_RENDER_UNK_SET_FRG_UNK_MASK	(1UL << 29)

#define ASAHI_RENDER_UNK_SET_IOGPU_UNK54	(1UL << 40)
#define ASAHI_RENDER_UNK_SET_IOGPU_UNK56	(1UL << 41)
#define ASAHI_RENDER_UNK_SET_TILING_CONTROL	(1UL << 42)
#define ASAHI_RENDER_UNK_SET_TILING_CONTROL_2	(1UL << 43)
#define ASAHI_RENDER_UNK_SET_VTX_UNK_F0		(1UL << 44)
#define ASAHI_RENDER_UNK_SET_VTX_UNK_F8		(1UL << 45)
#define ASAHI_RENDER_UNK_SET_VTX_UNK_118	(1UL << 46)
#define ASAHI_RENDER_UNK_SET_VTX_UNK_MASK	(1UL << 47)

#define ASAHI_RENDER_EXT_UNKNOWNS	0xff00

/* XXX: Do not upstream this struct */
struct drm_asahi_cmd_render_unknowns {
	/** @type: Type ID of this extension */
	__u32 type;
	__u32 pad;
	/** @next: Pointer to the next extension struct, if any */
	__u64 next;

	__u64 flags;

	__u64 tile_config;
	__u64 utile_config;

	__u64 aux_fb_unk;
	__u64 g14_unk;
	__u64 frg_unk_140;
	__u64 frg_unk_158;
	__u64 frg_tilecfg;
	__u64 load_bgobjvals;
	__u64 frg_unk_38;
	__u64 frg_unk_3c;
	__u64 reload_zlsctrl;
	__u64 unk_buf_10;
	__u64 frg_unk_mask;

	__u64 iogpu_unk54;
	__u64 iogpu_unk56;
	__u64 tiling_control;
	__u64 tiling_control_2;
	__u64 vtx_unk_f0;
	__u64 vtx_unk_f8;
	__u64 vtx_unk_118;
	__u64 vtx_unk_mask;
};

/* XXX check */
#define ASAHI_COMPUTE_NO_PREEMPTION (1UL << 0)

struct drm_asahi_cmd_compute {
	__u64 flags;

	__u64 encoder_ptr;
	__u64 encoder_end;
	__u64 usc_base;

	__u64 attachments;
	__u32 attachment_count;
	__u32 pad;

	__u32 helper_program;
	__u32 helper_cfg;
	__u64 helper_arg;

	__u32 encoder_id;
	__u32 cmd_id;

	__u64 sampler_array;
	__u32 sampler_count;
	__u32 sampler_max;

	__u32 iogpu_unk_40;
	__u32 unk_mask;
};

enum drm_asahi_status {
	DRM_ASAHI_STATUS_PENDING = 0,
	DRM_ASAHI_STATUS_COMPLETE,
	DRM_ASAHI_STATUS_UNKNOWN_ERROR,
	DRM_ASAHI_STATUS_TIMEOUT,
	DRM_ASAHI_STATUS_FAULT,
	DRM_ASAHI_STATUS_KILLED,
	DRM_ASAHI_STATUS_NO_DEVICE,
	DRM_ASAHI_STATUS_CHANNEL_ERROR,
};

enum drm_asahi_fault {
	DRM_ASAHI_FAULT_NONE = 0,
	DRM_ASAHI_FAULT_UNKNOWN,
	DRM_ASAHI_FAULT_UNMAPPED,
	DRM_ASAHI_FAULT_AF_FAULT,
	DRM_ASAHI_FAULT_WRITE_ONLY,
	DRM_ASAHI_FAULT_READ_ONLY,
	DRM_ASAHI_FAULT_NO_ACCESS,
};

struct drm_asahi_result_info {
	/** @status: One of enum drm_asahi_status */
	__u32 status;

	/** @reason: One of drm_asahi_fault_type */
	__u32 fault_type;

	/** @unit: Unit number, hardware dependent */
	__u32 unit;

	/** @sideband: Sideband information, hardware dependent */
	__u32 sideband;

	/** @level: Page table level at which the fault occurred, hardware dependent */
	__u8 level;

	/** @read: Fault was a read */
	__u8 is_read;

	/** @pad: MBZ */
	__u16 pad;

	/** @unk_5: Extra bits, hardware dependent */
	__u32 extra;

	/** @address: Fault address, cache line aligned */
	__u64 address;
};

#define DRM_ASAHI_RESULT_RENDER_TVB_GROW_OVF (1UL << 0)
#define DRM_ASAHI_RESULT_RENDER_TVB_GROW_MIN (1UL << 1)
#define DRM_ASAHI_RESULT_RENDER_TVB_OVERFLOWED (1UL << 2)

struct drm_asahi_result_render {
	/** @address: Common result information */
	struct drm_asahi_result_info info;

	/** @flags: Zero or more of of DRM_ASAHI_RESULT_RENDER_* */
	__u64 flags;

	/** @vertex_ts_start: Timestamp of the start of vertex processing */
	__u64 vertex_ts_start;

	/** @vertex_ts_end: Timestamp of the end of vertex processing */
	__u64 vertex_ts_end;

	/** @fragment_ts_start: Timestamp of the start of fragment processing */
	__u64 fragment_ts_start;

	/** @fragment_ts_end: Timestamp of the end of fragment processing */
	__u64 fragment_ts_end;

	/** @tvb_size_bytes: TVB size at the start of this render */
	__u64 tvb_size_bytes;

	/** @tvb_usage_bytes: Total TVB usage in bytes for this render */
	__u64 tvb_usage_bytes;

	/** @num_tvb_overflows: Number of TVB overflows that occurred for this render */
	__u32 num_tvb_overflows;
};

struct drm_asahi_result_compute {
	/** @address: Common result information */
	struct drm_asahi_result_info info;

	/** @flags: Zero or more of of DRM_ASAHI_RESULT_COMPUTE_* */
	__u64 flags;

	/** @ts_start: Timestamp of the start of this compute command */
	__u64 ts_start;

	/** @vertex_ts_end: Timestamp of the end of this compute command */
	__u64 ts_end;
};

struct drm_asahi_get_time {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @flags: MBZ. */
	__u64 flags;

	/** @tv_sec: On return, seconds part of a point in time */
	__s64 tv_sec;

	/** @tv_nsec: On return, nanoseconds part of a point in time */
	__s64 tv_nsec;

	/** @gpu_timestamp: On return, the GPU timestamp at that point in time */
	__u64 gpu_timestamp;
};

/* Note: this is an enum so that it can be resolved by Rust bindgen. */
enum {
   DRM_IOCTL_ASAHI_GET_PARAMS       = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_GET_PARAMS, struct drm_asahi_get_params),
   DRM_IOCTL_ASAHI_VM_CREATE        = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_VM_CREATE, struct drm_asahi_vm_create),
   DRM_IOCTL_ASAHI_VM_DESTROY       = DRM_IOW(DRM_COMMAND_BASE + DRM_ASAHI_VM_DESTROY, struct drm_asahi_vm_destroy),
   DRM_IOCTL_ASAHI_GEM_CREATE       = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_GEM_CREATE, struct drm_asahi_gem_create),
   DRM_IOCTL_ASAHI_GEM_MMAP_OFFSET  = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_GEM_MMAP_OFFSET, struct drm_asahi_gem_mmap_offset),
   DRM_IOCTL_ASAHI_GEM_BIND         = DRM_IOW(DRM_COMMAND_BASE + DRM_ASAHI_GEM_BIND, struct drm_asahi_gem_bind),
   DRM_IOCTL_ASAHI_QUEUE_CREATE     = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_QUEUE_CREATE, struct drm_asahi_queue_create),
   DRM_IOCTL_ASAHI_QUEUE_DESTROY    = DRM_IOW(DRM_COMMAND_BASE + DRM_ASAHI_QUEUE_DESTROY, struct drm_asahi_queue_destroy),
   DRM_IOCTL_ASAHI_SUBMIT           = DRM_IOW(DRM_COMMAND_BASE + DRM_ASAHI_SUBMIT, struct drm_asahi_submit),
   DRM_IOCTL_ASAHI_GET_TIME         = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_GET_TIME, struct drm_asahi_get_time),
};

#if defined(__cplusplus)
}
#endif

#endif /* _ASAHI_DRM_H_ */
