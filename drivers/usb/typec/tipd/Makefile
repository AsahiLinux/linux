# SPDX-License-Identifier: GPL-2.0
CFLAGS_trace.o				:= -I$(src)

obj-$(CONFIG_TYPEC_TPS6598X_CORE)	+= tps6598x-core.o
tps6598x-core-y				:= core.o
tps6598x-core-$(CONFIG_TRACING)		+= trace.o

obj-$(CONFIG_TYPEC_TPS6598X)		+= tps6598x.o
tps6598x-y				:= i2c.o

obj-$(CONFIG_TYPEC_SN201202X)		+= sn201202x.o
sn201202x-y				:= spmi.o
