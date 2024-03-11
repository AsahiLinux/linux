# SPDX-License-Identifier: GPL-2.0-only OR MIT

CFLAGS_trace.o = -I$(src)

appledrm-y := apple_drv.o

apple_dcp-y := afk.o dcp.o dcp_backlight.o dptxep.o iomfb.o parser.o systemep.o
apple_dcp-y += ibootep.o
apple_dcp-y += iomfb_v12_3.o
apple_dcp-y += iomfb_v13_3.o
apple_dcp-$(CONFIG_TRACING) += trace.o


obj-$(CONFIG_DRM_APPLE) += appledrm.o
obj-$(CONFIG_DRM_APPLE) += apple_dcp.o

# header test

# exclude some broken headers from the test coverage
no-header-test := \

always-y += \
	$(patsubst %.h,%.hdrtest, $(filter-out $(no-header-test), \
		$(shell cd $(srctree)/$(src) && find * -name '*.h')))

quiet_cmd_hdrtest = HDRTEST $(patsubst %.hdrtest,%.h,$@)
      cmd_hdrtest = $(CC) $(filter-out $(CFLAGS_GCOV), $(c_flags)) -S -o /dev/null -x c /dev/null -include $<; touch $@

$(obj)/%.hdrtest: $(src)/%.h FORCE
	$(call if_changed_dep,hdrtest)
