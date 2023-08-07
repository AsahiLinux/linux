# SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
CFLAGS_trace.o			:= -I$(src)

obj-$(CONFIG_PHY_APPLE_ATC)		+= phy-apple-atc.o
phy-apple-atc-y			:= atc.o
phy-apple-atc-$(CONFIG_TRACING)	+= trace.o
