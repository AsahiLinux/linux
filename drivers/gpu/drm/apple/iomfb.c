// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io> */

#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/kref.h>
#include <linux/align.h>
#include <linux/apple-mailbox.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/completion.h>

#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "dcp.h"
#include "dcp-internal.h"
#include "iomfb.h"
#include "parser.h"
#include "trace.h"

/* Register defines used in bandwidth setup structure */
#define REG_SCRATCH (0x14)
#define REG_SCRATCH_T600X (0x988)
#define REG_DOORBELL (0x0)
#define REG_DOORBELL_BIT (2)

struct dcp_wait_cookie {
	struct kref refcount;
	struct completion done;
};

static void release_wait_cookie(struct kref *ref)
{
	struct dcp_wait_cookie *cookie;
	cookie = container_of(ref, struct dcp_wait_cookie, refcount);

        kfree(cookie);
}

static int dcp_tx_offset(enum dcp_context_id id)
{
	switch (id) {
	case DCP_CONTEXT_CB:
	case DCP_CONTEXT_CMD:
		return 0x00000;
	case DCP_CONTEXT_OOBCB:
	case DCP_CONTEXT_OOBCMD:
		return 0x08000;
	default:
		return -EINVAL;
	}
}

static int dcp_channel_offset(enum dcp_context_id id)
{
	switch (id) {
	case DCP_CONTEXT_ASYNC:
		return 0x40000;
	case DCP_CONTEXT_CB:
		return 0x60000;
	case DCP_CONTEXT_OOBCB:
		return 0x68000;
	default:
		return dcp_tx_offset(id);
	}
}

static inline u64 dcpep_set_shmem(u64 dart_va)
{
	return FIELD_PREP(IOMFB_MESSAGE_TYPE, IOMFB_MESSAGE_TYPE_SET_SHMEM) |
	       FIELD_PREP(IOMFB_SHMEM_FLAG, IOMFB_SHMEM_FLAG_VALUE) |
	       FIELD_PREP(IOMFB_SHMEM_DVA, dart_va);
}

static inline u64 dcpep_msg(enum dcp_context_id id, u32 length, u16 offset)
{
	return FIELD_PREP(IOMFB_MESSAGE_TYPE, IOMFB_MESSAGE_TYPE_MSG) |
		FIELD_PREP(IOMFB_MSG_CONTEXT, id) |
		FIELD_PREP(IOMFB_MSG_OFFSET, offset) |
		FIELD_PREP(IOMFB_MSG_LENGTH, length);
}

static inline u64 dcpep_ack(enum dcp_context_id id)
{
	return dcpep_msg(id, 0, 0) | IOMFB_MSG_ACK;
}

/*
 * A channel is busy if we have sent a message that has yet to be
 * acked. The driver must not sent a message to a busy channel.
 */
static bool dcp_channel_busy(struct dcp_channel *ch)
{
	return (ch->depth != 0);
}

/*
 * Get the context ID passed to the DCP for a command we push. The rule is
 * simple: callback contexts are used when replying to the DCP, command
 * contexts are used otherwise. That corresponds to a non/zero call stack
 * depth. This rule frees the caller from tracking the call context manually.
 */
static enum dcp_context_id dcp_call_context(struct apple_dcp *dcp, bool oob)
{
	u8 depth = oob ? dcp->ch_oobcmd.depth : dcp->ch_cmd.depth;

	if (depth)
		return oob ? DCP_CONTEXT_OOBCB : DCP_CONTEXT_CB;
	else
		return oob ? DCP_CONTEXT_OOBCMD : DCP_CONTEXT_CMD;
}

/* Get a channel for a context */
static struct dcp_channel *dcp_get_channel(struct apple_dcp *dcp,
					   enum dcp_context_id context)
{
	switch (context) {
	case DCP_CONTEXT_CB:
		return &dcp->ch_cb;
	case DCP_CONTEXT_CMD:
		return &dcp->ch_cmd;
	case DCP_CONTEXT_OOBCB:
		return &dcp->ch_oobcb;
	case DCP_CONTEXT_OOBCMD:
		return &dcp->ch_oobcmd;
	case DCP_CONTEXT_ASYNC:
		return &dcp->ch_async;
	default:
		return NULL;
	}
}

/* Get the start of a packet: after the end of the previous packet */
static u16 dcp_packet_start(struct dcp_channel *ch, u8 depth)
{
	if (depth > 0)
		return ch->end[depth - 1];
	else
		return 0;
}

/* Pushes and pops the depth of the call stack with safety checks */
static u8 dcp_push_depth(u8 *depth)
{
	u8 ret = (*depth)++;

	WARN_ON(ret >= DCP_MAX_CALL_DEPTH);
	return ret;
}

static u8 dcp_pop_depth(u8 *depth)
{
	WARN_ON((*depth) == 0);

	return --(*depth);
}

#define DCP_METHOD(tag, name) [name] = { #name, tag }

const struct dcp_method_entry dcp_methods[dcpep_num_methods] = {
	DCP_METHOD("A000", dcpep_late_init_signal),
	DCP_METHOD("A029", dcpep_setup_video_limits),
	DCP_METHOD("A034", dcpep_update_notify_clients_dcp),
	DCP_METHOD("A131", iomfbep_a131_pmu_service_matched),
	DCP_METHOD("A132", iomfbep_a132_backlight_service_matched),
	DCP_METHOD("A357", dcpep_set_create_dfb),
	DCP_METHOD("A358", iomfbep_a358_vi_set_temperature_hint),
	DCP_METHOD("A401", dcpep_start_signal),
	DCP_METHOD("A407", dcpep_swap_start),
	DCP_METHOD("A408", dcpep_swap_submit),
	DCP_METHOD("A410", dcpep_set_display_device),
	DCP_METHOD("A411", dcpep_is_main_display),
	DCP_METHOD("A412", dcpep_set_digital_out_mode),
	DCP_METHOD("A426", iomfbep_get_color_remap_mode),
	DCP_METHOD("A439", dcpep_set_parameter_dcp),
	DCP_METHOD("A443", dcpep_create_default_fb),
	DCP_METHOD("A447", dcpep_enable_disable_video_power_savings),
	DCP_METHOD("A454", dcpep_first_client_open),
	DCP_METHOD("A460", dcpep_set_display_refresh_properties),
	DCP_METHOD("A463", dcpep_flush_supports_power),
	DCP_METHOD("A468", dcpep_set_power_state),
};

/* Call a DCP function given by a tag */
static void dcp_push(struct apple_dcp *dcp, bool oob, enum dcpep_method method,
		     u32 in_len, u32 out_len, void *data, dcp_callback_t cb,
		     void *cookie)
{
	enum dcp_context_id context = dcp_call_context(dcp, oob);
	struct dcp_channel *ch = dcp_get_channel(dcp, context);

	struct dcp_packet_header header = {
		.in_len = in_len,
		.out_len = out_len,

		/* Tag is reversed due to endianness of the fourcc */
		.tag[0] = dcp_methods[method].tag[3],
		.tag[1] = dcp_methods[method].tag[2],
		.tag[2] = dcp_methods[method].tag[1],
		.tag[3] = dcp_methods[method].tag[0],
	};

	u8 depth = dcp_push_depth(&ch->depth);
	u16 offset = dcp_packet_start(ch, depth);

	void *out = dcp->shmem + dcp_tx_offset(context) + offset;
	void *out_data = out + sizeof(header);
	size_t data_len = sizeof(header) + in_len + out_len;

	memcpy(out, &header, sizeof(header));

	if (in_len > 0)
		memcpy(out_data, data, in_len);

	trace_iomfb_push(dcp, &dcp_methods[method], context, offset, depth);

	ch->callbacks[depth] = cb;
	ch->cookies[depth] = cookie;
	ch->output[depth] = out + sizeof(header) + in_len;
	ch->end[depth] = offset + ALIGN(data_len, DCP_PACKET_ALIGNMENT);

	dcp_send_message(dcp, IOMFB_ENDPOINT,
			 dcpep_msg(context, data_len, offset));
}

#define DCP_THUNK_VOID(func, handle)                                         \
	static void func(struct apple_dcp *dcp, bool oob, dcp_callback_t cb, \
			 void *cookie)                                       \
	{                                                                    \
		dcp_push(dcp, oob, handle, 0, 0, NULL, cb, cookie);          \
	}

#define DCP_THUNK_OUT(func, handle, T)                                       \
	static void func(struct apple_dcp *dcp, bool oob, dcp_callback_t cb, \
			 void *cookie)                                       \
	{                                                                    \
		dcp_push(dcp, oob, handle, 0, sizeof(T), NULL, cb, cookie);  \
	}

#define DCP_THUNK_IN(func, handle, T)                                       \
	static void func(struct apple_dcp *dcp, bool oob, T *data,          \
			 dcp_callback_t cb, void *cookie)                   \
	{                                                                   \
		dcp_push(dcp, oob, handle, sizeof(T), 0, data, cb, cookie); \
	}

#define DCP_THUNK_INOUT(func, handle, T_in, T_out)                            \
	static void func(struct apple_dcp *dcp, bool oob, T_in *data,         \
			 dcp_callback_t cb, void *cookie)                     \
	{                                                                     \
		dcp_push(dcp, oob, handle, sizeof(T_in), sizeof(T_out), data, \
			 cb, cookie);                                         \
	}

#define IOMFB_THUNK_INOUT(name, T_in, T_out)                                      \
	static void iomfb_ ## name(struct apple_dcp *dcp, bool oob, T_in *data,   \
			 dcp_callback_t cb, void *cookie)                         \
	{                                                                         \
		dcp_push(dcp, oob, iomfbep_ ## name, sizeof(T_in), sizeof(T_out), \
			 data,  cb, cookie);                                      \
	}

DCP_THUNK_OUT(iomfb_a131_pmu_service_matched, iomfbep_a131_pmu_service_matched, u32);
DCP_THUNK_OUT(iomfb_a132_backlight_service_matched, iomfbep_a132_backlight_service_matched, u32);
DCP_THUNK_OUT(iomfb_a358_vi_set_temperature_hint, iomfbep_a358_vi_set_temperature_hint, u32);

IOMFB_THUNK_INOUT(get_color_remap_mode, struct iomfb_get_color_remap_mode_req,
		struct iomfb_get_color_remap_mode_resp);

DCP_THUNK_INOUT(dcp_swap_submit, dcpep_swap_submit, struct dcp_swap_submit_req,
		struct dcp_swap_submit_resp);

DCP_THUNK_INOUT(dcp_swap_start, dcpep_swap_start, struct dcp_swap_start_req,
		struct dcp_swap_start_resp);

DCP_THUNK_INOUT(dcp_set_power_state, dcpep_set_power_state,
		struct dcp_set_power_state_req,
		struct dcp_set_power_state_resp);

DCP_THUNK_INOUT(dcp_set_digital_out_mode, dcpep_set_digital_out_mode,
		struct dcp_set_digital_out_mode_req, u32);

DCP_THUNK_INOUT(dcp_set_display_device, dcpep_set_display_device, u32, u32);

DCP_THUNK_OUT(dcp_set_display_refresh_properties,
	      dcpep_set_display_refresh_properties, u32);

DCP_THUNK_OUT(dcp_late_init_signal, dcpep_late_init_signal, u32);
DCP_THUNK_IN(dcp_flush_supports_power, dcpep_flush_supports_power, u32);
DCP_THUNK_OUT(dcp_create_default_fb, dcpep_create_default_fb, u32);
DCP_THUNK_OUT(dcp_start_signal, dcpep_start_signal, u32);
DCP_THUNK_VOID(dcp_setup_video_limits, dcpep_setup_video_limits);
DCP_THUNK_VOID(dcp_set_create_dfb, dcpep_set_create_dfb);
DCP_THUNK_VOID(dcp_first_client_open, dcpep_first_client_open);

__attribute__((unused))
DCP_THUNK_IN(dcp_update_notify_clients_dcp, dcpep_update_notify_clients_dcp,
	     struct dcp_update_notify_clients_dcp);

DCP_THUNK_INOUT(dcp_set_parameter_dcp, dcpep_set_parameter_dcp,
		struct dcp_set_parameter_dcp, u32);

DCP_THUNK_INOUT(dcp_enable_disable_video_power_savings,
		dcpep_enable_disable_video_power_savings, u32, int);

DCP_THUNK_OUT(dcp_is_main_display, dcpep_is_main_display, u32);

/* Parse a callback tag "D123" into the ID 123. Returns -EINVAL on failure. */
static int dcp_parse_tag(char tag[4])
{
	u32 d[3];
	int i;

	if (tag[3] != 'D')
		return -EINVAL;

	for (i = 0; i < 3; ++i) {
		d[i] = (u32)(tag[i] - '0');

		if (d[i] > 9)
			return -EINVAL;
	}

	return d[0] + (d[1] * 10) + (d[2] * 100);
}

/* Ack a callback from the DCP */
static void dcp_ack(struct apple_dcp *dcp, enum dcp_context_id context)
{
	struct dcp_channel *ch = dcp_get_channel(dcp, context);

	dcp_pop_depth(&ch->depth);
	dcp_send_message(dcp, IOMFB_ENDPOINT,
			 dcpep_ack(context));
}

/* DCP callback handlers */
static void dcpep_cb_nop(struct apple_dcp *dcp)
{
	/* No operation */
}

static u8 dcpep_cb_true(struct apple_dcp *dcp)
{
	return true;
}

static u8 dcpep_cb_false(struct apple_dcp *dcp)
{
	return false;
}

static u32 dcpep_cb_zero(struct apple_dcp *dcp)
{
	return 0;
}

static void dcpep_cb_swap_complete(struct apple_dcp *dcp,
				   struct dc_swap_complete_resp *resp)
{
	trace_iomfb_swap_complete(dcp, resp->swap_id);

	dcp_drm_crtc_vblank(dcp->crtc);
}

/* special */
static void complete_vi_set_temperature_hint(struct apple_dcp *dcp, void *out, void *cookie)
{
	// ack D100 cb_match_pmu_service
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static bool iomfbep_cb_match_pmu_service(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);
	iomfb_a358_vi_set_temperature_hint(dcp, false,
					   complete_vi_set_temperature_hint,
					   NULL);

	// return false for deferred ACK
	return false;
}

static void complete_pmu_service_matched(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = ch->output[ch->depth - 1];

	*succ = true;

	// ack D206 cb_match_pmu_service_2
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static bool iomfbep_cb_match_pmu_service_2(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);

	iomfb_a131_pmu_service_matched(dcp, false, complete_pmu_service_matched,
				       out);

	// return false for deferred ACK
	return false;
}

static void complete_backlight_service_matched(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = ch->output[ch->depth - 1];

	*succ = true;

	// ack D206 cb_match_backlight_service
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static bool iomfbep_cb_match_backlight_service(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);

	iomfb_a132_backlight_service_matched(dcp, false, complete_backlight_service_matched, out);

	// return false for deferred ACK
	return false;
}

static void iomfb_cb_pr_publish(struct apple_dcp *dcp, struct iomfb_property *prop)
{
	switch (prop->id) {
	case IOMFB_PROPERTY_NITS:
	{
		dcp->brightness.nits = prop->value / dcp->brightness.scale;
		/* notify backlight device of the initial brightness */
		if (!dcp->brightness.bl_dev && dcp->brightness.maximum > 0)
			schedule_work(&dcp->bl_register_wq);
		trace_iomfb_brightness(dcp, prop->value);
		break;
	}
	default:
		dev_dbg(dcp->dev, "pr_publish: id: %d = %u\n", prop->id, prop->value);
	}
}

static struct dcp_get_uint_prop_resp
dcpep_cb_get_uint_prop(struct apple_dcp *dcp, struct dcp_get_uint_prop_req *req)
{
	struct dcp_get_uint_prop_resp resp = (struct dcp_get_uint_prop_resp){
	    .value = 0
	};

	if (dcp->has_mini_led &&
	    memcmp(req->obj, "SUMP", sizeof(req->obj)) == 0) { /* "PMUS */
	    if (strncmp(req->key, "Temperature", sizeof(req->key)) == 0) {
		/*
		 * TODO: value from j314c, find out if it is temperature in
		 *       centigrade C and which temperature sensor reports it
		 */
		resp.value = 3029;
		resp.ret = true;
	    }
	}

	return resp;
}

static u8 iomfbep_cb_sr_set_property_int(struct apple_dcp *dcp,
					 struct iomfb_sr_set_property_int_req *req)
{
	if (memcmp(req->obj, "FMOI", sizeof(req->obj)) == 0) { /* "IOMF */
		if (strncmp(req->key, "Brightness_Scale", sizeof(req->key)) == 0) {
			if (!req->value_null)
				dcp->brightness.scale = req->value;
		}
	}

	return 1;
}

static void iomfbep_cb_set_fx_prop(struct apple_dcp *dcp, struct iomfb_set_fx_prop_req *req)
{
    // TODO: trace this, see if there properties which needs to used later
}

/*
 * Callback to map a buffer allocated with allocate_buf for PIODMA usage.
 * PIODMA is separate from the main DCP and uses own IOVA space on a dedicated
 * stream of the display DART, rather than the expected DCP DART.
 *
 * XXX: This relies on dma_get_sgtable in concert with dma_map_sgtable, which
 * is a "fundamentally unsafe" operation according to the docs. And yet
 * everyone does it...
 */
static struct dcp_map_buf_resp dcpep_cb_map_piodma(struct apple_dcp *dcp,
						   struct dcp_map_buf_req *req)
{
	struct sg_table *map;
	int ret;

	if (req->buffer >= ARRAY_SIZE(dcp->memdesc))
		goto reject;

	map = &dcp->memdesc[req->buffer].map;

	if (!map->sgl)
		goto reject;

	/* Use PIODMA device instead of DCP to map against the right IOMMU. */
	ret = dma_map_sgtable(&dcp->piodma->dev, map, DMA_BIDIRECTIONAL, 0);

	if (ret)
		goto reject;

	return (struct dcp_map_buf_resp){ .dva = sg_dma_address(map->sgl) };

reject:
	dev_err(dcp->dev, "denying map of invalid buffer %llx for pidoma\n",
		req->buffer);
	return (struct dcp_map_buf_resp){ .ret = EINVAL };
}

static void dcpep_cb_unmap_piodma(struct apple_dcp *dcp,
				  struct dcp_unmap_buf_resp *resp)
{
	struct sg_table *map;
	dma_addr_t dma_addr;

	if (resp->buffer >= ARRAY_SIZE(dcp->memdesc)) {
		dev_warn(dcp->dev, "unmap request for out of range buffer %llu",
			 resp->buffer);
		return;
	}

	map = &dcp->memdesc[resp->buffer].map;

	if (!map->sgl) {
		dev_warn(dcp->dev,
			 "unmap for non-mapped buffer %llu iova:0x%08llx",
			 resp->buffer, resp->dva);
		return;
	}

	dma_addr = sg_dma_address(map->sgl);
	if (dma_addr != resp->dva) {
		dev_warn(dcp->dev, "unmap buffer %llu address mismatch dma_addr:%llx dva:%llx",
			 resp->buffer, dma_addr, resp->dva);
		return;
	}

	/* Use PIODMA device instead of DCP to unmap from the right IOMMU. */
	dma_unmap_sgtable(&dcp->piodma->dev, map, DMA_BIDIRECTIONAL, 0);
}

/*
 * Allocate an IOVA contiguous buffer mapped to the DCP. The buffer need not be
 * physically contigiuous, however we should save the sgtable in case the
 * buffer needs to be later mapped for PIODMA.
 */
static struct dcp_allocate_buffer_resp
dcpep_cb_allocate_buffer(struct apple_dcp *dcp,
			 struct dcp_allocate_buffer_req *req)
{
	struct dcp_allocate_buffer_resp resp = { 0 };
	struct dcp_mem_descriptor *memdesc;
	u32 id;

	resp.dva_size = ALIGN(req->size, 4096);
	resp.mem_desc_id =
		find_first_zero_bit(dcp->memdesc_map, DCP_MAX_MAPPINGS);

	if (resp.mem_desc_id >= DCP_MAX_MAPPINGS) {
		dev_warn(dcp->dev, "DCP overflowed mapping table, ignoring");
		resp.dva_size = 0;
		resp.mem_desc_id = 0;
		return resp;
	}
	id = resp.mem_desc_id;
	set_bit(id, dcp->memdesc_map);

	memdesc = &dcp->memdesc[id];

	memdesc->size = resp.dva_size;
	memdesc->buf = dma_alloc_coherent(dcp->dev, memdesc->size,
					  &memdesc->dva, GFP_KERNEL);

	dma_get_sgtable(dcp->dev, &memdesc->map, memdesc->buf, memdesc->dva,
			memdesc->size);
	resp.dva = memdesc->dva;

	return resp;
}

static u8 dcpep_cb_release_mem_desc(struct apple_dcp *dcp, u32 *mem_desc_id)
{
	struct dcp_mem_descriptor *memdesc;
	u32 id = *mem_desc_id;

	if (id >= DCP_MAX_MAPPINGS) {
		dev_warn(dcp->dev,
			 "unmap request for out of range mem_desc_id %u", id);
		return 0;
	}

	if (!test_and_clear_bit(id, dcp->memdesc_map)) {
		dev_warn(dcp->dev, "unmap request for unused mem_desc_id %u",
			 id);
		return 0;
	}

	memdesc = &dcp->memdesc[id];
	if (memdesc->buf) {
		dma_free_coherent(dcp->dev, memdesc->size, memdesc->buf,
				  memdesc->dva);

		memdesc->buf = NULL;
		memset(&memdesc->map, 0, sizeof(memdesc->map));
	} else {
		memdesc->reg = 0;
	}

	memdesc->size = 0;

	return 1;
}

/* Validate that the specified region is a display register */
static bool is_disp_register(struct apple_dcp *dcp, u64 start, u64 end)
{
	int i;

	for (i = 0; i < dcp->nr_disp_registers; ++i) {
		struct resource *r = dcp->disp_registers[i];

		if ((start >= r->start) && (end <= r->end))
			return true;
	}

	return false;
}

/*
 * Map contiguous physical memory into the DCP's address space. The firmware
 * uses this to map the display registers we advertise in
 * sr_map_device_memory_with_index, so we bounds check against that to guard
 * safe against malicious coprocessors.
 */
static struct dcp_map_physical_resp
dcpep_cb_map_physical(struct apple_dcp *dcp, struct dcp_map_physical_req *req)
{
	int size = ALIGN(req->size, 4096);
	u32 id;

	if (!is_disp_register(dcp, req->paddr, req->paddr + size - 1)) {
		dev_err(dcp->dev, "refusing to map phys address %llx size %llx",
			req->paddr, req->size);
		return (struct dcp_map_physical_resp){};
	}

	id = find_first_zero_bit(dcp->memdesc_map, DCP_MAX_MAPPINGS);
	set_bit(id, dcp->memdesc_map);
	dcp->memdesc[id].size = size;
	dcp->memdesc[id].reg = req->paddr;

	return (struct dcp_map_physical_resp){
		.dva_size = size,
		.mem_desc_id = id,
		.dva = dma_map_resource(dcp->dev, req->paddr, size,
					DMA_BIDIRECTIONAL, 0),
	};
}

static u64 dcpep_cb_get_frequency(struct apple_dcp *dcp)
{
	return clk_get_rate(dcp->clk);
}

static struct dcp_map_reg_resp dcpep_cb_map_reg(struct apple_dcp *dcp,
						struct dcp_map_reg_req *req)
{
	if (req->index >= dcp->nr_disp_registers) {
		dev_warn(dcp->dev, "attempted to read invalid reg index %u",
			 req->index);

		return (struct dcp_map_reg_resp){ .ret = 1 };
	} else {
		struct resource *rsrc = dcp->disp_registers[req->index];

		return (struct dcp_map_reg_resp){
			.addr = rsrc->start, .length = resource_size(rsrc)
		};
	}
}

static struct dcp_read_edt_data_resp
dcpep_cb_read_edt_data(struct apple_dcp *dcp, struct dcp_read_edt_data_req *req)
{
	return (struct dcp_read_edt_data_resp){
		.value[0] = req->value[0],
		.ret = 0,
	};
}

static void iomfbep_cb_enable_backlight_message_ap_gated(struct apple_dcp *dcp,
							 u8 *enabled)
{
	/*
	 * update backlight brightness on next swap, on non mini-LED displays
	 * DCP seems to set an invalid iDAC value after coming out of DPMS.
	 * syslog: "[BrightnessLCD.cpp:743][AFK]nitsToDBV: iDAC out of range"
	 */
	dcp->brightness.update = true;
}

/* Chunked data transfer for property dictionaries */
static u8 dcpep_cb_prop_start(struct apple_dcp *dcp, u32 *length)
{
	if (dcp->chunks.data != NULL) {
		dev_warn(dcp->dev, "ignoring spurious transfer start\n");
		return false;
	}

	dcp->chunks.length = *length;
	dcp->chunks.data = devm_kzalloc(dcp->dev, *length, GFP_KERNEL);

	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "failed to allocate chunks\n");
		return false;
	}

	return true;
}

static u8 dcpep_cb_prop_chunk(struct apple_dcp *dcp,
			      struct dcp_set_dcpav_prop_chunk_req *req)
{
	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "ignoring spurious chunk\n");
		return false;
	}

	if (req->offset + req->length > dcp->chunks.length) {
		dev_warn(dcp->dev, "ignoring overflowing chunk\n");
		return false;
	}

	memcpy(dcp->chunks.data + req->offset, req->data, req->length);
	return true;
}

static bool dcpep_process_chunks(struct apple_dcp *dcp,
				 struct dcp_set_dcpav_prop_end_req *req)
{
	struct dcp_parse_ctx ctx;
	int ret;

	if (!dcp->chunks.data) {
		dev_warn(dcp->dev, "ignoring spurious end\n");
		return false;
	}

	/* used just as opaque pointer for tracing */
	ctx.dcp = dcp;

	ret = parse(dcp->chunks.data, dcp->chunks.length, &ctx);

	if (ret) {
		dev_warn(dcp->dev, "bad header on dcpav props\n");
		return false;
	}

	if (!strcmp(req->key, "TimingElements")) {
		dcp->modes = enumerate_modes(&ctx, &dcp->nr_modes,
					     dcp->width_mm, dcp->height_mm,
					     dcp->notch_height);

		if (IS_ERR(dcp->modes)) {
			dev_warn(dcp->dev, "failed to parse modes\n");
			dcp->modes = NULL;
			dcp->nr_modes = 0;
			return false;
		}
	} else if (!strcmp(req->key, "DisplayAttributes")) {
		/* DisplayAttributes are empty for integrated displays, use
		 * display dimensions read from the devicetree
		 */
		if (dcp->main_display) {
			ret = parse_display_attributes(&ctx, &dcp->width_mm,
						&dcp->height_mm);

			if (ret) {
				dev_warn(dcp->dev, "failed to parse display attribs\n");
				return false;
			}
		}

		dcp_set_dimensions(dcp);
	}

	return true;
}

static u8 dcpep_cb_prop_end(struct apple_dcp *dcp,
			    struct dcp_set_dcpav_prop_end_req *req)
{
	u8 resp = dcpep_process_chunks(dcp, req);

	/* Reset for the next transfer */
	devm_kfree(dcp->dev, dcp->chunks.data);
	dcp->chunks.data = NULL;

	return resp;
}

/* Boot sequence */
static void boot_done(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_channel *ch = &dcp->ch_cb;
	u8 *succ = ch->output[ch->depth - 1];
	dev_dbg(dcp->dev, "boot done");

	*succ = true;
	dcp_ack(dcp, DCP_CONTEXT_CB);
}

static void boot_5(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_set_display_refresh_properties(dcp, false, boot_done, NULL);
}

static void boot_4(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_late_init_signal(dcp, false, boot_5, NULL);
}

static void boot_3(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 v_true = true;

	dcp_flush_supports_power(dcp, false, &v_true, boot_4, NULL);
}

static void boot_2(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_setup_video_limits(dcp, false, boot_3, NULL);
}

static void boot_1_5(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_create_default_fb(dcp, false, boot_2, NULL);
}

/* Use special function signature to defer the ACK */
static bool dcpep_cb_boot_1(struct apple_dcp *dcp, int tag, void *out, void *in)
{
	trace_iomfb_callback(dcp, tag, __func__);
	dcp_set_create_dfb(dcp, false, boot_1_5, NULL);
	return false;
}

static struct dcp_rt_bandwidth dcpep_cb_rt_bandwidth(struct apple_dcp *dcp)
{
	if (dcp->disp_registers[5] && dcp->disp_registers[6])
		return (struct dcp_rt_bandwidth){
			.reg_scratch =
				dcp->disp_registers[5]->start + REG_SCRATCH,
			.reg_doorbell =
				dcp->disp_registers[6]->start + REG_DOORBELL,
			.doorbell_bit = REG_DOORBELL_BIT,

			.padding[3] = 0x4, // XXX: required by 11.x firmware
		};
	else if (dcp->disp_registers[4])
		return (struct dcp_rt_bandwidth){
			.reg_scratch = dcp->disp_registers[4]->start +
				       REG_SCRATCH_T600X,
			.reg_doorbell = 0,
			.doorbell_bit = 0,
		};
	else
		return (struct dcp_rt_bandwidth){
			.reg_scratch = 0,
			.reg_doorbell = 0,
			.doorbell_bit = 0,
		};
}

/* Callback to get the current time as milliseconds since the UNIX epoch */
static u64 dcpep_cb_get_time(struct apple_dcp *dcp)
{
	return ktime_to_ms(ktime_get_real());
}

struct dcp_swap_cookie {
	struct kref refcount;
	struct completion done;
	u32 swap_id;
};

static void release_swap_cookie(struct kref *ref)
{
	struct dcp_swap_cookie *cookie;
	cookie = container_of(ref, struct dcp_swap_cookie, refcount);

        kfree(cookie);
}

static void dcp_swap_cleared(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct dcp_swap_submit_resp *resp = data;
	dev_dbg(dcp->dev, "%s", __func__);

	if (cookie) {
		struct dcp_swap_cookie *info = cookie;
		complete(&info->done);
		kref_put(&info->refcount, release_swap_cookie);
	}

	if (resp->ret) {
		dev_err(dcp->dev, "swap_clear failed! status %u\n", resp->ret);
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}

	while (!list_empty(&dcp->swapped_out_fbs)) {
		struct dcp_fb_reference *entry;
		entry = list_first_entry(&dcp->swapped_out_fbs,
					 struct dcp_fb_reference, head);
		if (entry->fb)
			drm_framebuffer_put(entry->fb);
		list_del(&entry->head);
		kfree(entry);
	}
}

static void dcp_swap_clear_started(struct apple_dcp *dcp, void *data,
				   void *cookie)
{
	struct dcp_swap_start_resp *resp = data;
	dev_dbg(dcp->dev, "%s swap_id: %u", __func__, resp->swap_id);
	dcp->swap.swap.swap_id = resp->swap_id;

	if (cookie) {
		struct dcp_swap_cookie *info = cookie;
		info->swap_id = resp->swap_id;
	}

	dcp_swap_submit(dcp, false, &dcp->swap, dcp_swap_cleared, cookie);
}

static void dcp_on_final(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;
	dev_dbg(dcp->dev, "%s", __func__);

	if (wait) {
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

static void dcp_on_set_power_state(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_set_power_state_req req = {
		.unklong = 1,
	};
	dev_dbg(dcp->dev, "%s", __func__);

	dcp_set_power_state(dcp, false, &req, dcp_on_final, cookie);
}

static void dcp_on_set_parameter(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct dcp_set_parameter_dcp param = {
		.param = 14,
		.value = { 0 },
		.count = 1,
	};
	dev_dbg(dcp->dev, "%s", __func__);

	dcp_set_parameter_dcp(dcp, false, &param, dcp_on_set_power_state, cookie);
}

void dcp_poweron(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct dcp_wait_cookie *cookie;
	int ret;
	u32 handle;
	dev_dbg(dcp->dev, "%s", __func__);

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;

	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	if (dcp->main_display) {
		handle = 0;
		dcp_set_display_device(dcp, false, &handle, dcp_on_set_power_state,
				       cookie);
	} else {
		handle = 2;
		dcp_set_display_device(dcp, false, &handle,
				       dcp_on_set_parameter, cookie);
	}
	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(500));

	if (ret == 0)
		dev_warn(dcp->dev, "wait for power timed out");

	kref_put(&cookie->refcount, release_wait_cookie);;

	/* Force a brightness update after poweron, to restore the brightness */
	dcp->brightness.update = true;
}
EXPORT_SYMBOL(dcp_poweron);

static void complete_set_powerstate(struct apple_dcp *dcp, void *out,
				    void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;

	if (wait) {
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

void dcp_poweroff(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret, swap_id;
	struct dcp_set_power_state_req power_req = {
		.unklong = 0,
	};
	struct dcp_swap_cookie *cookie;
	struct dcp_wait_cookie *poff_cookie;
	struct dcp_swap_start_req swap_req = { 0 };

	dev_dbg(dcp->dev, "%s", __func__);

	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return;
	init_completion(&cookie->done);
	kref_init(&cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&cookie->refcount);

	// clear surfaces
	memset(&dcp->swap, 0, sizeof(dcp->swap));

	dcp->swap.swap.swap_enabled = DCP_REMOVE_LAYERS | 0x7;
	dcp->swap.swap.swap_completed = DCP_REMOVE_LAYERS | 0x7;
	dcp->swap.swap.unk_10c = 0xFF000000;

	/*
	 * Turn off the backlight. This matters because the DCP's idea of
	 * backlight brightness gets desynced after a power change, and it
	 * needs to be told it's going to turn off so it will consider the
	 * subsequent update on poweron an actual change and restore the
	 * brightness.
	 */
	dcp->swap.swap.bl_unk = 1;
	dcp->swap.swap.bl_value = 0;
	dcp->swap.swap.bl_power = 0;

	for (int l = 0; l < SWAP_SURFACES; l++)
		dcp->swap.surf_null[l] = true;

	dcp_swap_start(dcp, false, &swap_req, dcp_swap_clear_started, cookie);

	ret = wait_for_completion_timeout(&cookie->done, msecs_to_jiffies(50));
	swap_id = cookie->swap_id;
	kref_put(&cookie->refcount, release_swap_cookie);
	if (ret <= 0) {
		dcp->crashed = true;
		return;
	}

	dev_dbg(dcp->dev, "%s: clear swap submitted: %u", __func__, swap_id);

	poff_cookie = kzalloc(sizeof(*poff_cookie), GFP_KERNEL);
	if (!poff_cookie)
		return;
	init_completion(&poff_cookie->done);
	kref_init(&poff_cookie->refcount);
	/* increase refcount to ensure the receiver has a reference */
	kref_get(&poff_cookie->refcount);

	dcp_set_power_state(dcp, false, &power_req, complete_set_powerstate,
			    poff_cookie);
	ret = wait_for_completion_timeout(&poff_cookie->done,
					  msecs_to_jiffies(1000));

	if (ret == 0)
		dev_warn(dcp->dev, "setPowerState(0) timeout %u ms", 1000);
	else if (ret > 0)
		dev_dbg(dcp->dev,
			"setPowerState(0) finished with %d ms to spare",
			jiffies_to_msecs(ret));

	kref_put(&poff_cookie->refcount, release_wait_cookie);
	dev_dbg(dcp->dev, "%s: setPowerState(0) done", __func__);
}
EXPORT_SYMBOL(dcp_poweroff);

/*
 * Helper to send a DRM hotplug event. The DCP is accessed from a single
 * (RTKit) thread. To handle hotplug callbacks, we need to call
 * drm_kms_helper_hotplug_event, which does an atomic commit (via DCP) and
 * waits for vblank (a DCP callback). That means we deadlock if we call from
 * the RTKit thread! Instead, move the call to another thread via a workqueue.
 */
void dcp_hotplug(struct work_struct *work)
{
	struct apple_connector *connector;
	struct drm_device *dev;
	struct apple_dcp *dcp;

	connector = container_of(work, struct apple_connector, hotplug_wq);
	dev = connector->base.dev;

	dcp = platform_get_drvdata(connector->dcp);
	dev_info(dcp->dev, "%s: connected: %d", __func__, connector->connected);

	/*
	 * DCP defers link training until we set a display mode. But we set
	 * display modes from atomic_flush, so userspace needs to trigger a
	 * flush, or the CRTC gets no signal.
	 */
	if (connector->base.state && !dcp->valid_mode && connector->connected) {
		drm_connector_set_link_status_property(
			&connector->base, DRM_MODE_LINK_STATUS_BAD);
	}

	if (dev && dev->registered)
		drm_kms_helper_hotplug_event(dev);
}
EXPORT_SYMBOL_GPL(dcp_hotplug);

static void dcpep_cb_hotplug(struct apple_dcp *dcp, u64 *connected)
{
	struct apple_connector *connector = dcp->connector;

	/* DCP issues hotplug_gated callbacks after SetPowerState() calls on
	 * devices with display (macbooks, imacs). This must not result in
	 * connector state changes on DRM side. Some applications won't enable
	 * a CRTC with a connector in disconnected state. Weston after DPMS off
	 * is one example. dcp_is_main_display() returns true on devices with
	 * integrated display. Ignore the hotplug_gated() callbacks there.
	 */
	if (dcp->main_display)
		return;

	/* Hotplug invalidates mode. DRM doesn't always handle this. */
	if (!(*connected)) {
		dcp->valid_mode = false;
		/* after unplug swap will not complete until the next
		 * set_digital_out_mode */
		schedule_work(&dcp->vblank_wq);
	}

	if (connector && connector->connected != !!(*connected)) {
		connector->connected = !!(*connected);
		dcp->valid_mode = false;
		schedule_work(&connector->hotplug_wq);
	}
}

static void
dcpep_cb_swap_complete_intent_gated(struct apple_dcp *dcp,
				    struct dcp_swap_complete_intent_gated *info)
{
	trace_iomfb_swap_complete_intent_gated(dcp, info->swap_id,
		info->width, info->height);
}

#define DCPEP_MAX_CB (1000)

/*
 * Define type-safe trampolines. Define typedefs to enforce type-safety on the
 * input data (so if the types don't match, gcc errors out).
 */

#define TRAMPOLINE_VOID(func, handler)                                        \
	static bool func(struct apple_dcp *dcp, int tag, void *out, void *in) \
	{                                                                     \
		trace_iomfb_callback(dcp, tag, #handler);                     \
		handler(dcp);                                                 \
		return true;                                                  \
	}

#define TRAMPOLINE_IN(func, handler, T_in)                                    \
	typedef void (*callback_##handler)(struct apple_dcp *, T_in *);       \
                                                                              \
	static bool func(struct apple_dcp *dcp, int tag, void *out, void *in) \
	{                                                                     \
		callback_##handler cb = handler;                              \
                                                                              \
		trace_iomfb_callback(dcp, tag, #handler);                     \
		cb(dcp, in);                                                  \
		return true;                                                  \
	}

#define TRAMPOLINE_INOUT(func, handler, T_in, T_out)                          \
	typedef T_out (*callback_##handler)(struct apple_dcp *, T_in *);      \
                                                                              \
	static bool func(struct apple_dcp *dcp, int tag, void *out, void *in) \
	{                                                                     \
		T_out *typed_out = out;                                       \
		callback_##handler cb = handler;                              \
                                                                              \
		trace_iomfb_callback(dcp, tag, #handler);                     \
		*typed_out = cb(dcp, in);                                     \
		return true;                                                  \
	}

#define TRAMPOLINE_OUT(func, handler, T_out)                                  \
	static bool func(struct apple_dcp *dcp, int tag, void *out, void *in) \
	{                                                                     \
		T_out *typed_out = out;                                       \
                                                                              \
		trace_iomfb_callback(dcp, tag, #handler);                     \
		*typed_out = handler(dcp);                                    \
		return true;                                                  \
	}

TRAMPOLINE_VOID(trampoline_nop, dcpep_cb_nop);
TRAMPOLINE_OUT(trampoline_true, dcpep_cb_true, u8);
TRAMPOLINE_OUT(trampoline_false, dcpep_cb_false, u8);
TRAMPOLINE_OUT(trampoline_zero, dcpep_cb_zero, u32);
TRAMPOLINE_IN(trampoline_swap_complete, dcpep_cb_swap_complete,
	      struct dc_swap_complete_resp);
TRAMPOLINE_INOUT(trampoline_get_uint_prop, dcpep_cb_get_uint_prop,
		 struct dcp_get_uint_prop_req, struct dcp_get_uint_prop_resp);
TRAMPOLINE_IN(trampoline_set_fx_prop, iomfbep_cb_set_fx_prop,
	      struct iomfb_set_fx_prop_req)
TRAMPOLINE_INOUT(trampoline_map_piodma, dcpep_cb_map_piodma,
		 struct dcp_map_buf_req, struct dcp_map_buf_resp);
TRAMPOLINE_IN(trampoline_unmap_piodma, dcpep_cb_unmap_piodma,
	      struct dcp_unmap_buf_resp);
TRAMPOLINE_INOUT(trampoline_sr_set_property_int, iomfbep_cb_sr_set_property_int,
		 struct iomfb_sr_set_property_int_req, u8);
TRAMPOLINE_INOUT(trampoline_allocate_buffer, dcpep_cb_allocate_buffer,
		 struct dcp_allocate_buffer_req,
		 struct dcp_allocate_buffer_resp);
TRAMPOLINE_INOUT(trampoline_map_physical, dcpep_cb_map_physical,
		 struct dcp_map_physical_req, struct dcp_map_physical_resp);
TRAMPOLINE_INOUT(trampoline_release_mem_desc, dcpep_cb_release_mem_desc, u32,
		 u8);
TRAMPOLINE_INOUT(trampoline_map_reg, dcpep_cb_map_reg, struct dcp_map_reg_req,
		 struct dcp_map_reg_resp);
TRAMPOLINE_INOUT(trampoline_read_edt_data, dcpep_cb_read_edt_data,
		 struct dcp_read_edt_data_req, struct dcp_read_edt_data_resp);
TRAMPOLINE_INOUT(trampoline_prop_start, dcpep_cb_prop_start, u32, u8);
TRAMPOLINE_INOUT(trampoline_prop_chunk, dcpep_cb_prop_chunk,
		 struct dcp_set_dcpav_prop_chunk_req, u8);
TRAMPOLINE_INOUT(trampoline_prop_end, dcpep_cb_prop_end,
		 struct dcp_set_dcpav_prop_end_req, u8);
TRAMPOLINE_OUT(trampoline_rt_bandwidth, dcpep_cb_rt_bandwidth,
	       struct dcp_rt_bandwidth);
TRAMPOLINE_OUT(trampoline_get_frequency, dcpep_cb_get_frequency, u64);
TRAMPOLINE_OUT(trampoline_get_time, dcpep_cb_get_time, u64);
TRAMPOLINE_IN(trampoline_hotplug, dcpep_cb_hotplug, u64);
TRAMPOLINE_IN(trampoline_swap_complete_intent_gated,
	      dcpep_cb_swap_complete_intent_gated,
	      struct dcp_swap_complete_intent_gated);
TRAMPOLINE_IN(trampoline_enable_backlight_message_ap_gated,
	      iomfbep_cb_enable_backlight_message_ap_gated, u8);
TRAMPOLINE_IN(trampoline_pr_publish, iomfb_cb_pr_publish,
	      struct iomfb_property);

bool (*const dcpep_cb_handlers[DCPEP_MAX_CB])(struct apple_dcp *, int, void *,
					      void *) = {
	[0] = trampoline_true, /* did_boot_signal */
	[1] = trampoline_true, /* did_power_on_signal */
	[2] = trampoline_nop, /* will_power_off_signal */
	[3] = trampoline_rt_bandwidth,
	[100] = iomfbep_cb_match_pmu_service,
	[101] = trampoline_zero, /* get_display_default_stride */
	[102] = trampoline_nop, /* set_number_property */
	[103] = trampoline_nop, /* set_boolean_property */
	[106] = trampoline_nop, /* remove_property */
	[107] = trampoline_true, /* create_provider_service */
	[108] = trampoline_true, /* create_product_service */
	[109] = trampoline_true, /* create_pmu_service */
	[110] = trampoline_true, /* create_iomfb_service */
	[111] = trampoline_true, /* create_backlight_service */
	[116] = dcpep_cb_boot_1,
	[117] = trampoline_false, /* is_dark_boot */
	[118] = trampoline_false, /* is_dark_boot / is_waking_from_hibernate*/
	[120] = trampoline_read_edt_data,
	[122] = trampoline_prop_start,
	[123] = trampoline_prop_chunk,
	[124] = trampoline_prop_end,
	[201] = trampoline_map_piodma,
	[202] = trampoline_unmap_piodma,
	[206] = iomfbep_cb_match_pmu_service_2,
	[207] = iomfbep_cb_match_backlight_service,
	[208] = trampoline_get_time,
	[211] = trampoline_nop, /* update_backlight_factor_prop */
	[300] = trampoline_pr_publish,
	[401] = trampoline_get_uint_prop,
	[404] = trampoline_nop, /* sr_set_uint_prop */
	[406] = trampoline_set_fx_prop,
	[408] = trampoline_get_frequency,
	[411] = trampoline_map_reg,
	[413] = trampoline_true, /* sr_set_property_dict */
	[414] = trampoline_sr_set_property_int,
	[415] = trampoline_true, /* sr_set_property_bool */
	[451] = trampoline_allocate_buffer,
	[452] = trampoline_map_physical,
	[456] = trampoline_release_mem_desc,
	[552] = trampoline_true, /* set_property_dict_0 */
	[561] = trampoline_true, /* set_property_dict */
	[563] = trampoline_true, /* set_property_int */
	[565] = trampoline_true, /* set_property_bool */
	[567] = trampoline_true, /* set_property_str */
	[574] = trampoline_zero, /* power_up_dart */
	[576] = trampoline_hotplug,
	[577] = trampoline_nop, /* powerstate_notify */
	[582] = trampoline_true, /* create_default_fb_surface */
	[589] = trampoline_swap_complete,
	[591] = trampoline_swap_complete_intent_gated,
	[593] = trampoline_enable_backlight_message_ap_gated,
	[598] = trampoline_nop, /* find_swap_function_gated */
};

static void dcpep_handle_cb(struct apple_dcp *dcp, enum dcp_context_id context,
			    void *data, u32 length, u16 offset)
{
	struct device *dev = dcp->dev;
	struct dcp_packet_header *hdr = data;
	void *in, *out;
	int tag = dcp_parse_tag(hdr->tag);
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	u8 depth;

	if (tag < 0 || tag >= DCPEP_MAX_CB || !dcpep_cb_handlers[tag]) {
		dev_warn(dev, "received unknown callback %c%c%c%c\n",
			 hdr->tag[3], hdr->tag[2], hdr->tag[1], hdr->tag[0]);
		return;
	}

	in = data + sizeof(*hdr);
	out = in + hdr->in_len;

	// TODO: verify that in_len and out_len match our prototypes
	// for now just clear the out data to have at least consistant results
	if (hdr->out_len)
		memset(out, 0, hdr->out_len);

	depth = dcp_push_depth(&ch->depth);
	ch->output[depth] = out;
	ch->end[depth] = offset + ALIGN(length, DCP_PACKET_ALIGNMENT);

	if (dcpep_cb_handlers[tag](dcp, tag, out, in))
		dcp_ack(dcp, context);
}

static void dcpep_handle_ack(struct apple_dcp *dcp, enum dcp_context_id context,
			     void *data, u32 length)
{
	struct dcp_packet_header *header = data;
	struct dcp_channel *ch = dcp_get_channel(dcp, context);
	void *cookie;
	dcp_callback_t cb;

	if (!ch) {
		dev_warn(dcp->dev, "ignoring ack on context %X\n", context);
		return;
	}

	dcp_pop_depth(&ch->depth);

	cb = ch->callbacks[ch->depth];
	cookie = ch->cookies[ch->depth];

	ch->callbacks[ch->depth] = NULL;
	ch->cookies[ch->depth] = NULL;

	if (cb)
		cb(dcp, data + sizeof(*header) + header->in_len, cookie);
}

static void dcpep_got_msg(struct apple_dcp *dcp, u64 message)
{
	enum dcp_context_id ctx_id;
	u16 offset;
	u32 length;
	int channel_offset;
	void *data;

	ctx_id = FIELD_GET(IOMFB_MSG_CONTEXT, message);
	offset = FIELD_GET(IOMFB_MSG_OFFSET, message);
	length = FIELD_GET(IOMFB_MSG_LENGTH, message);

	channel_offset = dcp_channel_offset(ctx_id);

	if (channel_offset < 0) {
		dev_warn(dcp->dev, "invalid context received %u", ctx_id);
		return;
	}

	data = dcp->shmem + channel_offset + offset;

	if (FIELD_GET(IOMFB_MSG_ACK, message))
		dcpep_handle_ack(dcp, ctx_id, data, length);
	else
		dcpep_handle_cb(dcp, ctx_id, data, length, offset);
}

/*
 * Callback for swap requests. If a swap failed, we'll never get a swap
 * complete event so we need to fake a vblank event early to avoid a hang.
 */

static void dcp_swapped(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct dcp_swap_submit_resp *resp = data;

	if (resp->ret) {
		dev_err(dcp->dev, "swap failed! status %u\n", resp->ret);
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}

	while (!list_empty(&dcp->swapped_out_fbs)) {
		struct dcp_fb_reference *entry;
		entry = list_first_entry(&dcp->swapped_out_fbs,
					 struct dcp_fb_reference, head);
		if (entry->fb)
			drm_framebuffer_put(entry->fb);
		list_del(&entry->head);
		kfree(entry);
	}
}

static void dcp_swap_started(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct dcp_swap_start_resp *resp = data;

	dcp->swap.swap.swap_id = resp->swap_id;

	trace_iomfb_swap_submit(dcp, resp->swap_id);
	dcp_swap_submit(dcp, false, &dcp->swap, dcp_swapped, NULL);
}

/*
 * DRM specifies rectangles as start and end coordinates.  DCP specifies
 * rectangles as a start coordinate and a width/height. Convert a DRM rectangle
 * to a DCP rectangle.
 */
static struct dcp_rect drm_to_dcp_rect(struct drm_rect *rect)
{
	return (struct dcp_rect){ .x = rect->x1,
				  .y = rect->y1,
				  .w = drm_rect_width(rect),
				  .h = drm_rect_height(rect) };
}

static u32 drm_format_to_dcp(u32 drm)
{
	switch (drm) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return fourcc_code('A', 'R', 'G', 'B');

	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		return fourcc_code('A', 'B', 'G', 'R');

	case DRM_FORMAT_ARGB2101010:
	case DRM_FORMAT_XRGB2101010:
		return fourcc_code('r', '0', '3', 'w');
	}

	pr_warn("DRM format %X not supported in DCP\n", drm);
	return 0;
}

static u8 drm_format_to_colorspace(u32 drm)
{
	switch (drm) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		return 1;

	case DRM_FORMAT_ARGB2101010:
	case DRM_FORMAT_XRGB2101010:
		return 12;
	}

	return 1;
}

int dcp_get_modes(struct drm_connector *connector)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);
	struct platform_device *pdev = apple_connector->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;
	int i;

	for (i = 0; i < dcp->nr_modes; ++i) {
		mode = drm_mode_duplicate(dev, &dcp->modes[i].mode);

		if (!mode) {
			dev_err(dev->dev, "Failed to duplicate display mode\n");
			return 0;
		}

		drm_mode_probed_add(connector, mode);
	}

	return dcp->nr_modes;
}
EXPORT_SYMBOL_GPL(dcp_get_modes);

/* The user may own drm_display_mode, so we need to search for our copy */
static struct dcp_display_mode *lookup_mode(struct apple_dcp *dcp,
					    const struct drm_display_mode *mode)
{
	int i;

	for (i = 0; i < dcp->nr_modes; ++i) {
		if (drm_mode_match(mode, &dcp->modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS |
					   DRM_MODE_MATCH_CLOCK))
			return &dcp->modes[i];
	}

	return NULL;
}

int dcp_mode_valid(struct drm_connector *connector,
		   struct drm_display_mode *mode)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);
	struct platform_device *pdev = apple_connector->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return lookup_mode(dcp, mode) ? MODE_OK : MODE_BAD;
}
EXPORT_SYMBOL_GPL(dcp_mode_valid);

bool dcp_crtc_mode_fixup(struct drm_crtc *crtc,
			 const struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct platform_device *pdev = apple_crtc->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	/* TODO: support synthesized modes through scaling */
	return lookup_mode(dcp, mode) != NULL;
}
EXPORT_SYMBOL(dcp_crtc_mode_fixup);

/* Helpers to modeset and swap, used to flush */
static void do_swap(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct dcp_swap_start_req start_req = { 0 };
	dev_dbg(dcp->dev, "%s", __func__);

	if (dcp->connector && dcp->connector->connected)
		dcp_swap_start(dcp, false, &start_req, dcp_swap_started, NULL);
	else
		dcp_drm_crtc_vblank(dcp->crtc);
}

static void complete_set_digital_out_mode(struct apple_dcp *dcp, void *data,
					  void *cookie)
{
	struct dcp_wait_cookie *wait = cookie;
	dev_dbg(dcp->dev, "%s", __func__);

	if (wait) {
		complete(&wait->done);
		kref_put(&wait->refcount, release_wait_cookie);
	}
}

void dcp_flush(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct drm_plane *plane;
	struct drm_plane_state *new_state, *old_state;
	struct drm_crtc_state *crtc_state;
	struct dcp_swap_submit_req *req = &dcp->swap;
	int plane_idx, l;
	int has_surface = 0;
	bool modeset;
	dev_dbg(dcp->dev, "%s", __func__);

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	modeset = drm_atomic_crtc_needs_modeset(crtc_state) || !dcp->valid_mode;

	if (dcp_channel_busy(&dcp->ch_cmd))
	{
		dev_err(dcp->dev, "unexpected busy command channel");
		/* HACK: issue a delayed vblank event to avoid timeouts in
		 * drm_atomic_helper_wait_for_vblanks().
		 */
		schedule_work(&dcp->vblank_wq);
		return;
	}

	/* Reset to defaults */
	memset(req, 0, sizeof(*req));
	for (l = 0; l < SWAP_SURFACES; l++)
		req->surf_null[l] = true;

	l = 0;
	for_each_oldnew_plane_in_state(state, plane, old_state, new_state, plane_idx) {
		struct drm_framebuffer *fb = new_state->fb;
		struct drm_gem_dma_object *obj;
		struct drm_rect src_rect;
		bool opaque = false;

		/* skip planes not for this crtc */
		if (old_state->crtc != crtc && new_state->crtc != crtc)
			continue;

		WARN_ON(l >= SWAP_SURFACES);

		req->swap.swap_enabled |= BIT(l);

		if (old_state->fb && fb != old_state->fb) {
			/*
			 * Race condition between a framebuffer unbind getting
			 * swapped out and GEM unreferencing a framebuffer. If
			 * we lose the race, the display gets IOVA faults and
			 * the DCP crashes. We need to extend the lifetime of
			 * the drm_framebuffer (and hence the GEM object) until
			 * after we get a swap complete for the swap unbinding
			 * it.
			 */
			struct dcp_fb_reference *entry =
				kzalloc(sizeof(*entry), GFP_KERNEL);
			if (entry) {
				entry->fb = old_state->fb;
				list_add_tail(&entry->head,
					      &dcp->swapped_out_fbs);
			}
			drm_framebuffer_get(old_state->fb);
		}

		if (!new_state->fb) {
			if (old_state->fb)
				req->swap.swap_enabled |= DCP_REMOVE_LAYERS;

			l += 1;
			continue;
		}
		req->surf_null[l] = false;
		has_surface = 1;

		if (!fb->format->has_alpha ||
		    new_state->plane->type == DRM_PLANE_TYPE_PRIMARY)
		    opaque = true;
		drm_rect_fp_to_int(&src_rect, &new_state->src);

		req->swap.src_rect[l] = drm_to_dcp_rect(&src_rect);
		req->swap.dst_rect[l] = drm_to_dcp_rect(&new_state->dst);

		if (dcp->notch_height > 0)
			req->swap.dst_rect[l].y += dcp->notch_height;

		/* the obvious helper call drm_fb_dma_get_gem_addr() adjusts
		 * the address for source x/y offsets. Since IOMFB has a direct
		 * support source position prefer that.
		 */
		obj = drm_fb_dma_get_gem_obj(fb, 0);
		if (obj)
			req->surf_iova[l] = obj->dma_addr + fb->offsets[0];

		req->surf[l] = (struct dcp_surface){
			.opaque = opaque,
			.format = drm_format_to_dcp(fb->format->format),
			.xfer_func = 13,
			.colorspace = drm_format_to_colorspace(fb->format->format),
			.stride = fb->pitches[0],
			.width = fb->width,
			.height = fb->height,
			.buf_size = fb->height * fb->pitches[0],
			.surface_id = req->swap.surf_ids[l],

			/* Only used for compressed or multiplanar surfaces */
			.pix_size = 1,
			.pel_w = 1,
			.pel_h = 1,
			.has_comp = 1,
			.has_planes = 1,
		};

		l += 1;
	}

	/* These fields should be set together */
	req->swap.swap_completed = req->swap.swap_enabled;

	if (modeset) {
		struct dcp_display_mode *mode;
		struct dcp_wait_cookie *cookie;
		int ret;

		mode = lookup_mode(dcp, &crtc_state->mode);
		if (!mode) {
			dev_warn(dcp->dev, "no match for " DRM_MODE_FMT,
				 DRM_MODE_ARG(&crtc_state->mode));
			schedule_work(&dcp->vblank_wq);
			return;
		}

		dev_info(dcp->dev, "set_digital_out_mode(color:%d timing:%d)",
			 mode->color_mode_id, mode->timing_mode_id);
		dcp->mode = (struct dcp_set_digital_out_mode_req){
			.color_mode_id = mode->color_mode_id,
			.timing_mode_id = mode->timing_mode_id
		};

		cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
		if (!cookie) {
			schedule_work(&dcp->vblank_wq);
			return;
		}

		init_completion(&cookie->done);
		kref_init(&cookie->refcount);
		/* increase refcount to ensure the receiver has a reference */
		kref_get(&cookie->refcount);

		dcp_set_digital_out_mode(dcp, false, &dcp->mode,
					 complete_set_digital_out_mode, cookie);

		dev_dbg(dcp->dev, "%s - wait for modeset", __func__);
		ret = wait_for_completion_timeout(&cookie->done,
						  msecs_to_jiffies(500));

		kref_put(&cookie->refcount, release_wait_cookie);

		if (ret == 0) {
			dev_dbg(dcp->dev, "set_digital_out_mode 200 ms");
			schedule_work(&dcp->vblank_wq);
			return;
		} else if (ret > 0) {
			dev_dbg(dcp->dev,
				"set_digital_out_mode finished with %d to spare",
				jiffies_to_msecs(ret));
		}

		dcp->valid_mode = true;
	}

	if (!has_surface && !crtc_state->color_mgmt_changed) {
		if (crtc_state->enable && crtc_state->active &&
		    !crtc_state->planes_changed) {
			schedule_work(&dcp->vblank_wq);
			return;
		}

		req->clear = 1;
	}

	/* update brightness if changed */
	if (dcp->brightness.update) {
		req->swap.bl_unk = 1;
		req->swap.bl_value = dcp->brightness.dac;
		req->swap.bl_power = 0x40;
		dcp->brightness.update = false;
	}

	do_swap(dcp, NULL, NULL);
}
EXPORT_SYMBOL_GPL(dcp_flush);

bool dcp_is_initialized(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->active;
}
EXPORT_SYMBOL_GPL(dcp_is_initialized);

static void res_is_main_display(struct apple_dcp *dcp, void *out, void *cookie)
{
	struct apple_connector *connector;
	int result = *(int *)out;
	dev_info(dcp->dev, "DCP is_main_display: %d\n", result);

	dcp->main_display = result != 0;

	dcp->active = true;

	connector = dcp->connector;
	if (connector) {
		connector->connected = dcp->nr_modes > 0;
		schedule_work(&connector->hotplug_wq);
	}
}

static void init_3(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_is_main_display(dcp, false, res_is_main_display, NULL);
}

static void init_2(struct apple_dcp *dcp, void *out, void *cookie)
{
	dcp_first_client_open(dcp, false, init_3, NULL);
}

static void init_1(struct apple_dcp *dcp, void *out, void *cookie)
{
	u32 val = 0;
	dcp_enable_disable_video_power_savings(dcp, false, &val, init_2, NULL);
}

static void dcp_started(struct apple_dcp *dcp, void *data, void *cookie)
{
	struct iomfb_get_color_remap_mode_req color_remap =
		(struct iomfb_get_color_remap_mode_req){
			.mode = 6,
		};

	dev_info(dcp->dev, "DCP booted\n");

	iomfb_get_color_remap_mode(dcp, false, &color_remap, init_1, cookie);
}

void iomfb_recv_msg(struct apple_dcp *dcp, u64 message)
{
	enum dcpep_type type = FIELD_GET(IOMFB_MESSAGE_TYPE, message);

	if (type == IOMFB_MESSAGE_TYPE_INITIALIZED)
		dcp_start_signal(dcp, false, dcp_started, NULL);
	else if (type == IOMFB_MESSAGE_TYPE_MSG)
		dcpep_got_msg(dcp, message);
	else
		dev_warn(dcp->dev, "Ignoring unknown message %llx\n", message);
}

int iomfb_start_rtkit(struct apple_dcp *dcp)
{
	dma_addr_t shmem_iova;
	apple_rtkit_start_ep(dcp->rtk, IOMFB_ENDPOINT);

	dcp->shmem = dma_alloc_coherent(dcp->dev, DCP_SHMEM_SIZE, &shmem_iova,
					GFP_KERNEL);

	shmem_iova |= dcp->asc_dram_mask;
	dcp_send_message(dcp, IOMFB_ENDPOINT, dcpep_set_shmem(shmem_iova));

	return 0;
}

void iomfb_shutdown(struct apple_dcp *dcp)
{
	struct dcp_set_power_state_req req = {
		/* defaults are ok */
	};

	/* We're going down */
	dcp->active = false;
	dcp->valid_mode = false;

	dcp_set_power_state(dcp, false, &req, NULL, NULL);
}
