#undef TRACE_SYSTEM
#define TRACE_SYSTEM apple_asc

#if !defined(__APPLE_ASC_MAILBOX_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define __APPLE_ASC_MAILBOX_TRACE_H

#include <linux/types.h>
#include <linux/tracepoint.h>

#include "apple-asc-mailbox.h"

DECLARE_EVENT_CLASS(apple_rtkit_log_msg,
	TP_PROTO(struct apple_mbox *mbox, u64 msg0, u64 msg1),
	TP_ARGS(mbox, msg0, msg1),
	TP_STRUCT__entry(
                __string(name, dev_name(mbox->dev))
		__field(u64, msg0)
		__field(u64, msg1)
	),
	TP_fast_assign(
                __assign_str(name, dev_name(mbox->dev));
		__entry->msg0 = msg0;
		__entry->msg1 = msg1;
	),
	TP_printk("%s: %016llx %016llx (ep: 0x%02x)",
                __get_str(name),
		__entry->msg0,
		__entry->msg1,
		((u8)__entry->msg1&0xff))
);

DEFINE_EVENT(apple_rtkit_log_msg, apple_mbox_hw_send,
	TP_PROTO(struct apple_mbox *mbox, u64 msg0, u64 msg1),
	TP_ARGS(mbox, msg0, msg1)
);

DEFINE_EVENT(apple_rtkit_log_msg, apple_mbox_hw_recv,
	TP_PROTO(struct apple_mbox *mbox, u64 msg0, u64 msg1),
	TP_ARGS(mbox, msg0, msg1)
);

DEFINE_EVENT(apple_rtkit_log_msg, apple_mbox_send_fifo_put,
	TP_PROTO(struct apple_mbox *mbox, u64 msg0, u64 msg1),
	TP_ARGS(mbox, msg0, msg1)
);

DECLARE_EVENT_CLASS(apple_rtkit_irq_endis,
	TP_PROTO(struct apple_mbox *mbox, bool enable),
	TP_ARGS(mbox, enable),
	TP_STRUCT__entry(
                __string(name, dev_name(mbox->dev))
		__field(bool, enable)
	),
	TP_fast_assign(
                __assign_str(name, dev_name(mbox->dev));
		__entry->enable = enable;
	),
	TP_printk("%s: %d",
                __get_str(name),
		__entry->enable)
);

DEFINE_EVENT(apple_rtkit_irq_endis, apple_mbox_can_recv_irq_enable,
	TP_PROTO(struct apple_mbox *mbox, bool enable),
	TP_ARGS(mbox, enable)
);

DEFINE_EVENT(apple_rtkit_irq_endis, apple_mbox_can_send_irq_enable,
	TP_PROTO(struct apple_mbox *mbox, bool enable),
	TP_ARGS(mbox, enable)
);

#endif

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE apple-asc-mailbox-trace

#include <trace/define_trace.h>