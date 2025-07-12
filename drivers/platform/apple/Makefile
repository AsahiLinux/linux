# SPDX-License-Identifier: GPL-2.0
#
# Makefile for linux/drivers/platform/apple
# Apple Platform-Specific Drivers
#

macsmc-y				+= smc_core.o
macsmc-rtkit-y				+= smc_rtkit.o

obj-$(CONFIG_APPLE_SMC)			+= macsmc.o
obj-$(CONFIG_APPLE_SMC_RTKIT)		+= macsmc-rtkit.o
