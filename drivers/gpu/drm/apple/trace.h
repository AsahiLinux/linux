// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright (C) The Asahi Linux Contributors */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM dcp

#if !defined(_TRACE_DCP_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_DCP_H

#include "dcp-internal.h"

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#define show_dcp_endpoint(ep)                                      \
	__print_symbolic(ep, { SYSTEM_ENDPOINT, "system" },        \
			 { TEST_ENDPOINT, "test" },                \
			 { DCP_EXPERT_ENDPOINT, "dcpexpert" },     \
			 { DISP0_ENDPOINT, "disp0" },              \
			 { DPTX_ENDPOINT, "dptxport" },            \
			 { HDCP_ENDPOINT, "hdcp" },                \
			 { REMOTE_ALLOC_ENDPOINT, "remotealloc" }, \
			 { IOMFB_ENDPOINT, "iomfb" })

TRACE_EVENT(dcp_recv_msg,
	    TP_PROTO(struct apple_dcp *dcp, u8 endpoint, u64 message),
	    TP_ARGS(dcp, endpoint, message),

	    TP_STRUCT__entry(__string(devname, dev_name(dcp->dev))
			     __field(u8, endpoint)
			     __field(u64, message)),

	    TP_fast_assign(__assign_str(devname, dev_name(dcp->dev));
			   __entry->endpoint = endpoint;
			   __entry->message = message;),

	    TP_printk("%s: endpoint 0x%x (%s): received message 0x%016llx",
		      __get_str(devname), __entry->endpoint,
		      show_dcp_endpoint(__entry->endpoint), __entry->message));

TRACE_EVENT(dcp_send_msg,
	    TP_PROTO(struct apple_dcp *dcp, u8 endpoint, u64 message),
	    TP_ARGS(dcp, endpoint, message),

	    TP_STRUCT__entry(__string(devname, dev_name(dcp->dev))
			     __field(u8, endpoint)
			     __field(u64, message)),

	    TP_fast_assign(__assign_str(devname, dev_name(dcp->dev));
			   __entry->endpoint = endpoint;
			   __entry->message = message;),

	    TP_printk("%s: endpoint 0x%x (%s): will send message 0x%016llx",
		      __get_str(devname), __entry->endpoint,
		      show_dcp_endpoint(__entry->endpoint), __entry->message));

TRACE_EVENT(iomfb_callback,
	    TP_PROTO(struct apple_dcp *dcp, int tag, const char *name),
	    TP_ARGS(dcp, tag, name),

	    TP_STRUCT__entry(
				__string(devname, dev_name(dcp->dev))
				__field(int, tag)
				__field(const char *, name)
			),

	    TP_fast_assign(
				__assign_str(devname, dev_name(dcp->dev));
				__entry->tag = tag; __entry->name = name;
			),

	    TP_printk("%s: Callback D%03d %s", __get_str(devname), __entry->tag,
		      __entry->name));

TRACE_EVENT(iomfb_push,
	    TP_PROTO(struct apple_dcp *dcp,
		     const struct dcp_method_entry *method, int context,
		     int offset, int depth),
	    TP_ARGS(dcp, method, context, offset, depth),

	    TP_STRUCT__entry(
				__string(devname, dev_name(dcp->dev))
				__string(name, method->name)
				__field(int, context)
				__field(int, offset)
				__field(int, depth)),

	    TP_fast_assign(
				__assign_str(devname, dev_name(dcp->dev));
				__assign_str(name, method->name);
				__entry->context = context; __entry->offset = offset;
				__entry->depth = depth;
			),

	    TP_printk("%s: Method %s: context %u, offset %u, depth %u",
		      __get_str(devname), __get_str(name), __entry->context,
		      __entry->offset, __entry->depth));

TRACE_EVENT(iomfb_swap_submit,
	    TP_PROTO(struct apple_dcp *dcp, u32 swap_id),
	    TP_ARGS(dcp, swap_id),
	    TP_STRUCT__entry(
			     __field(u64, dcp)
			     __field(u32, swap_id)
	    ),
	    TP_fast_assign(
			   __entry->dcp = (u64)dcp;
			   __entry->swap_id = swap_id;
	    ),
	    TP_printk("dcp=%llx, swap_id=%d",
		      __entry->dcp,
		      __entry->swap_id)
);

TRACE_EVENT(iomfb_swap_complete,
	    TP_PROTO(struct apple_dcp *dcp, u32 swap_id),
	    TP_ARGS(dcp, swap_id),
	    TP_STRUCT__entry(
			     __field(u64, dcp)
			     __field(u32, swap_id)
	    ),
	    TP_fast_assign(
			   __entry->dcp = (u64)dcp;
			   __entry->swap_id = swap_id;
	    ),
	    TP_printk("dcp=%llx, swap_id=%d",
		      __entry->dcp,
		      __entry->swap_id
	    )
);

TRACE_EVENT(iomfb_swap_complete_intent_gated,
	    TP_PROTO(struct apple_dcp *dcp, u32 swap_id, u32 width, u32 height),
	    TP_ARGS(dcp, swap_id, width, height),
	    TP_STRUCT__entry(
			     __field(u64, dcp)
			     __field(u32, swap_id)
			     __field(u32, width)
			     __field(u32, height)
	    ),
	    TP_fast_assign(
			   __entry->dcp = (u64)dcp;
			   __entry->swap_id = swap_id;
			   __entry->height = height;
			   __entry->width = width;
	    ),
	    TP_printk("dcp=%llx, swap_id=%u %ux%u",
		      __entry->dcp,
		      __entry->swap_id,
		      __entry->width,
		      __entry->height
	    )
);

TRACE_EVENT(iomfb_brightness,
	    TP_PROTO(struct apple_dcp *dcp, u32 nits),
	    TP_ARGS(dcp, nits),
	    TP_STRUCT__entry(
			     __field(u64, dcp)
			     __field(u32, nits)
	    ),
	    TP_fast_assign(
			   __entry->dcp = (u64)dcp;
			   __entry->nits = nits;
	    ),
	    TP_printk("dcp=%llx, nits=%u (raw=0x%05x)",
		      __entry->dcp,
		      __entry->nits >> 16,
		      __entry->nits
	    )
);

#define show_eotf(eotf)					\
	__print_symbolic(eotf, { 0, "SDR gamma"},	\
			       { 1, "HDR gamma"},	\
			       { 2, "ST 2084 (PQ)"},	\
			       { 3, "BT.2100 (HLG)"},	\
			       { 4, "unexpected"})

#define show_encoding(enc)							\
	__print_symbolic(enc, { 0, "RGB"},					\
			      { 1, "YUV 4:2:0"},				\
			      { 3, "YUV 4:2:2"},				\
			      { 2, "YUV 4:4:4"},				\
			      { 4, "DolbyVision (native)"},			\
			      { 5, "DolbyVision (HDMI)"},			\
			      { 6, "YCbCr 4:2:2 (DP tunnel)"},			\
			      { 7, "YCbCr 4:2:2 (HDMI tunnel)"},		\
			      { 8, "DolbyVision LL YCbCr 4:2:2"},		\
			      { 9, "DolbyVision LL YCbCr 4:2:2 (DP)"},		\
			      {10, "DolbyVision LL YCbCr 4:2:2 (HDMI)"},	\
			      {11, "DolbyVision LL YCbCr 4:4:4"},		\
			      {12, "DolbyVision LL RGB 4:2:2"},			\
			      {13, "GRGB as YCbCr422 (Even line blue)"},	\
			      {14, "GRGB as YCbCr422 (Even line red)"},		\
			      {15, "unexpected"})

#define show_colorimetry(col)					\
	__print_symbolic(col, { 0, "SMPTE 170M/BT.601"},	\
			      { 1, "BT.701"},			\
			      { 2, "xvYCC601"},			\
			      { 3, "xvYCC709"},			\
			      { 4, "sYCC601"},			\
			      { 5, "AdobeYCC601"},		\
			      { 6, "BT.2020 (c)"},		\
			      { 7, "BT.2020 (nc)"},		\
			      { 8, "DolbyVision VSVDB"},	\
			      { 9, "BT.2020 (RGB)"},		\
			      {10, "sRGB"},			\
			      {11, "scRGB"},			\
			      {12, "scRGBfixed"},		\
			      {13, "AdobeRGB"},			\
			      {14, "DCI-P3 (D65)"},		\
			      {15, "DCI-P3 (Theater)"},		\
			      {16, "Default RGB"},		\
			      {17, "unexpected"})

#define show_range(range)				\
	__print_symbolic(range, { 0, "Full"},		\
				{ 1, "Limited"},	\
				{ 2, "unexpected"})

TRACE_EVENT(iomfb_color_mode,
	    TP_PROTO(struct apple_dcp *dcp, u32 id, u32 score, u32 depth,
		     u32 colorimetry, u32 eotf, u32 range, u32 pixel_enc),
	    TP_ARGS(dcp, id, score, depth, colorimetry, eotf, range, pixel_enc),
	    TP_STRUCT__entry(
			     __field(u64, dcp)
			     __field(u32, id)
			     __field(u32, score)
			     __field(u32, depth)
			     __field(u32, colorimetry)
			     __field(u32, eotf)
			     __field(u32, range)
			     __field(u32, pixel_enc)
	    ),
	    TP_fast_assign(
			   __entry->dcp = (u64)dcp;
			   __entry->id = id;
			   __entry->score = score;
			   __entry->depth = depth;
			   __entry->colorimetry = min_t(u32, colorimetry, 17U);
			   __entry->eotf = min_t(u32, eotf, 4U);
			   __entry->range = min_t(u32, range, 2U);
			   __entry->pixel_enc = min_t(u32, pixel_enc, 15U);
	    ),
	    TP_printk("dcp=%llx, id=%u, score=%u,  depth=%u, colorimetry=%s, eotf=%s, range=%s, pixel_enc=%s",
		      __entry->dcp,
		      __entry->id,
		      __entry->score,
		      __entry->depth,
		      show_colorimetry(__entry->colorimetry),
		      show_eotf(__entry->eotf),
		      show_range(__entry->range),
		      show_encoding(__entry->pixel_enc)
	    )
);

TRACE_EVENT(iomfb_timing_mode,
	    TP_PROTO(struct apple_dcp *dcp, u32 id, u32 score, u32 width,
		     u32 height, u32 clock, u32 color_mode),
	    TP_ARGS(dcp, id, score, width, height, clock, color_mode),
	    TP_STRUCT__entry(
			     __field(u64, dcp)
			     __field(u32, id)
			     __field(u32, score)
			     __field(u32, width)
			     __field(u32, height)
			     __field(u32, clock)
			     __field(u32, color_mode)
	    ),
	    TP_fast_assign(
			   __entry->dcp = (u64)dcp;
			   __entry->id = id;
			   __entry->score = score;
			   __entry->width = width;
			   __entry->height = height;
			   __entry->clock = clock;
			   __entry->color_mode = color_mode;
	    ),
	    TP_printk("dcp=%llx, id=%u, score=%u,  %ux%u@%u.%u, color_mode=%u",
		      __entry->dcp,
		      __entry->id,
		      __entry->score,
		      __entry->width,
		      __entry->height,
		      __entry->clock >> 16,
		      ((__entry->clock & 0xffff) * 1000) >> 16,
		      __entry->color_mode
	    )
);

#endif /* _TRACE_DCP_H */

/* This part must be outside protection */

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#include <trace/define_trace.h>
