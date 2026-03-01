# SPDX-License-Identifier: GPL-2.0-only OR MIT

CFLAGS_trace.o = -I$(src)

appledrm-y := apple_drv.o

apple_dcp-y := dcp.o dcp_backlight.o iomfb.o parser.o
apple_dcp-$(CONFIG_TRACING) += trace.o

apple_piodma-y := dummy-piodma.o

obj-$(CONFIG_DRM_APPLE) += appledrm.o
obj-$(CONFIG_DRM_APPLE) += apple_dcp.o
obj-$(CONFIG_DRM_APPLE) += apple_piodma.o
