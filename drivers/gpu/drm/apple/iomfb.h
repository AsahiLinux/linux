// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io> */

#ifndef __APPLE_DCPEP_H__
#define __APPLE_DCPEP_H__

#include <linux/types.h>

/* Fixed size of shared memory between DCP and AP */
#define DCP_SHMEM_SIZE 0x100000

/* DCP message contexts */
enum dcp_context_id {
	/* Callback */
	DCP_CONTEXT_CB = 0,

	/* Command */
	DCP_CONTEXT_CMD = 2,

	/* Asynchronous */
	DCP_CONTEXT_ASYNC = 3,

	/* Out-of-band callback */
	DCP_CONTEXT_OOBCB = 4,

	/* Out-of-band command */
	DCP_CONTEXT_OOBCMD = 6,

	DCP_NUM_CONTEXTS
};

/* RTKit endpoint message types */
enum dcpep_type {
	/* Set shared memory */
	DCPEP_TYPE_SET_SHMEM = 0,

	/* DCP is initialized */
	DCPEP_TYPE_INITIALIZED = 1,

	/* Remote procedure call */
	DCPEP_TYPE_MESSAGE = 2,
};

/* Message */
#define DCPEP_TYPE_SHIFT (0)
#define DCPEP_TYPE_MASK GENMASK(1, 0)
#define DCPEP_ACK BIT_ULL(6)
#define DCPEP_CONTEXT_SHIFT (8)
#define DCPEP_CONTEXT_MASK GENMASK(11, 8)
#define DCPEP_OFFSET_SHIFT (16)
#define DCPEP_OFFSET_MASK GENMASK(31, 16)
#define DCPEP_LENGTH_SHIFT (32)

/* Set shmem */
#define DCPEP_DVA_SHIFT (16)
#define DCPEP_FLAG_SHIFT (4)
#define DCPEP_FLAG_VALUE (4)

struct dcp_packet_header {
	char tag[4];
	u32 in_len;
	u32 out_len;
} __packed;

#define DCP_IS_NULL(ptr) ((ptr) ? 1 : 0)
#define DCP_PACKET_ALIGNMENT (0x40)

/* Structures used in v12.0 firmware */

#define SWAP_SURFACES 4
#define MAX_PLANES 3

struct dcp_iouserclient {
	/* Handle for the IOUserClient. macOS sets this to a kernel VA. */
	u64 handle;
	u32 unk;
	u8 flag1;
	u8 flag2;
	u8 padding[2];
} __packed;

struct dcp_rect {
	u32 x;
	u32 y;
	u32 w;
	u32 h;
} __packed;

/*
 * Set in the swap_{enabled,completed} field to remove missing
 * layers. Without this flag, the DCP will assume missing layers have
 * not changed since the previous frame and will preserve their
 * content.
  */
#define DCP_REMOVE_LAYERS BIT(31)

struct dcp_swap {
	u64 ts1;
	u64 ts2;
	u64 unk_10[6];
	u64 flags1;
	u64 flags2;

	u32 swap_id;

	u32 surf_ids[SWAP_SURFACES];
	struct dcp_rect src_rect[SWAP_SURFACES];
	u32 surf_flags[SWAP_SURFACES];
	u32 surf_unk[SWAP_SURFACES];
	struct dcp_rect dst_rect[SWAP_SURFACES];
	u32 swap_enabled;
	u32 swap_completed;

	u32 unk_10c;
	u8 unk_110[0x1b8];
	u32 unk_2c8;
	u8 unk_2cc[0x14];
	u32 unk_2e0;
	u8 unk_2e4[0x3c];
} __packed;

/* Information describing a plane of a planar compressed surface */
struct dcp_plane_info {
	u32 width;
	u32 height;
	u32 base;
	u32 offset;
	u32 stride;
	u32 size;
	u16 tile_size;
	u8 tile_w;
	u8 tile_h;
	u32 unk[13];
} __packed;

struct dcp_component_types {
	u8 count;
	u8 types[7];
} __packed;

/* Information describing a surface */
struct dcp_surface {
	u8 is_tiled;
	u8 unk_1;
	u8 opaque; /** ignore alpha, also required YUV overlays */
	u32 plane_cnt;
	u32 plane_cnt2;
	u32 format; /* DCP fourcc */
	u32 unk_f;
	u8 xfer_func;
	u8 colorspace;
	u32 stride;
	u16 pix_size;
	u8 pel_w;
	u8 pel_h;
	u32 offset;
	u32 width;
	u32 height;
	u32 buf_size;
	u32 unk_2d;
	u32 unk_31;
	u32 surface_id;
	struct dcp_component_types comp_types[MAX_PLANES];
	u64 has_comp;
	struct dcp_plane_info planes[MAX_PLANES];
	u64 has_planes;
	u32 compression_info[MAX_PLANES][13];
	u64 has_compr_info;
	u64 unk_1f5;
	u8 padding[7];
} __packed;

struct dcp_rt_bandwidth {
	u64 unk1;
	u64 reg_scratch;
	u64 reg_doorbell;
	u32 unk2;
	u32 doorbell_bit;
	u32 padding[7];
} __packed;

/* Method calls */

enum dcpep_method {
	dcpep_late_init_signal,
	dcpep_setup_video_limits,
	dcpep_set_create_dfb,
	dcpep_start_signal,
	dcpep_swap_start,
	dcpep_swap_submit,
	dcpep_set_display_device,
	dcpep_set_digital_out_mode,
	dcpep_create_default_fb,
	dcpep_set_display_refresh_properties,
	dcpep_flush_supports_power,
	dcpep_set_power_state,
	dcpep_first_client_open,
	dcpep_update_notify_clients_dcp,
	dcpep_set_parameter_dcp,
	dcpep_enable_disable_video_power_savings,
	dcpep_is_main_display,
	dcpep_num_methods
};

struct dcp_method_entry {
	const char *name;
	char tag[4];
};

/* Prototypes */

struct dcp_set_digital_out_mode_req {
	u32 color_mode_id;
	u32 timing_mode_id;
} __packed;

struct dcp_map_buf_req {
	u64 buffer;
	u8 unk;
	u8 buf_null;
	u8 vaddr_null;
	u8 dva_null;
} __packed;

struct dcp_map_buf_resp {
	u64 vaddr;
	u64 dva;
	u32 ret;
} __packed;

struct dcp_unmap_buf_resp {
	u64 buffer;
	u64 vaddr;
	u64 dva;
	u8 unk;
	u8 buf_null;
} __packed;

struct dcp_allocate_buffer_req {
	u32 unk0;
	u64 size;
	u32 unk2;
	u8 paddr_null;
	u8 dva_null;
	u8 dva_size_null;
	u8 padding;
} __packed;

struct dcp_allocate_buffer_resp {
	u64 paddr;
	u64 dva;
	u64 dva_size;
	u32 mem_desc_id;
} __packed;

struct dcp_map_physical_req {
	u64 paddr;
	u64 size;
	u32 flags;
	u8 dva_null;
	u8 dva_size_null;
	u8 padding[2];
} __packed;

struct dcp_map_physical_resp {
	u64 dva;
	u64 dva_size;
	u32 mem_desc_id;
} __packed;

struct dcp_map_reg_req {
	char obj[4];
	u32 index;
	u32 flags;
	u8 addr_null;
	u8 length_null;
	u8 padding[2];
} __packed;

struct dcp_map_reg_resp {
	u64 addr;
	u64 length;
	u32 ret;
} __packed;

struct dcp_swap_start_req {
	u32 swap_id;
	struct dcp_iouserclient client;
	u8 swap_id_null;
	u8 client_null;
	u8 padding[2];
} __packed;

struct dcp_swap_start_resp {
	u32 swap_id;
	struct dcp_iouserclient client;
	u32 ret;
} __packed;

struct dcp_swap_submit_req {
	struct dcp_swap swap;
	struct dcp_surface surf[SWAP_SURFACES];
	u64 surf_iova[SWAP_SURFACES];
	u8 unkbool;
	u64 unkdouble;
	u32 clear; // or maybe switch to default fb?
	u8 swap_null;
	u8 surf_null[SWAP_SURFACES];
	u8 unkoutbool_null;
	u8 padding[1];
} __packed;

struct dcp_swap_submit_resp {
	u8 unkoutbool;
	u32 ret;
	u8 padding[3];
} __packed;

struct dc_swap_complete_resp {
	u32 swap_id;
	u8 unkbool;
	u64 swap_data;
	u8 swap_info[0x6c4];
	u32 unkint;
	u8 swap_info_null;
} __packed;

struct dcp_get_uint_prop_req {
	char obj[4];
	char key[0x40];
	u64 value;
	u8 value_null;
	u8 padding[3];
} __packed;

struct dcp_get_uint_prop_resp {
	u64 value;
	u8 ret;
	u8 padding[3];
} __packed;

struct dcp_set_power_state_req {
	u64 unklong;
	u8 unkbool;
	u8 unkint_null;
	u8 padding[2];
} __packed;

struct dcp_set_power_state_resp {
	u32 unkint;
	u32 ret;
} __packed;

struct dcp_set_dcpav_prop_chunk_req {
	char data[0x1000];
	u32 offset;
	u32 length;
} __packed;

struct dcp_set_dcpav_prop_end_req {
	char key[0x40];
} __packed;

struct dcp_update_notify_clients_dcp {
	u32 client_0;
	u32 client_1;
	u32 client_2;
	u32 client_3;
	u32 client_4;
	u32 client_5;
	u32 client_6;
	u32 client_7;
	u32 client_8;
	u32 client_9;
	u32 client_a;
	u32 client_b;
	u32 client_c;
	u32 client_d;
} __packed;

struct dcp_set_parameter_dcp {
	u32 param;
	u32 value[8];
	u32 count;
} __packed;

struct dcp_swap_complete_intent_gated {
	u32 swap_id;
	u8 unkBool;
	u32 unkInt;
	u32 width;
	u32 height;
} __packed;

struct dcp_read_edt_data_req {
	char key[0x40];
	u32 count;
	u32 value[8];
} __packed;

struct dcp_read_edt_data_resp {
	u32 value[8];
	u8 ret;
} __packed;

#endif
