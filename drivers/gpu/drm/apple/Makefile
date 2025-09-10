# SPDX-License-Identifier: GPL-2.0-only OR MIT

CFLAGS_trace.o = -I$(src)

appledrm-y := apple_drv.o
appledrm-y += afk.o dcp.o dcp_backlight.o dptxep.o iomfb.o parser.o systemep.o
appledrm-$(CONFIG_DRM_APPLE_AUDIO) += audio.o
appledrm-$(CONFIG_DRM_APPLE_AUDIO) += av.o
appledrm-y += connector.o
appledrm-y += ibootep.o
appledrm-y += iomfb_v12_3.o
appledrm-y += iomfb_v13_3.o
appledrm-y += epic/dpavservep.o


appledrm-$(CONFIG_TRACING) += trace.o

obj-$(CONFIG_DRM_APPLE) += appledrm.o
