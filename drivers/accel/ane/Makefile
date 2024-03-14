LD = /usr/bin/ld.lld
objs-m := ane.o ./src/ane_drv.o ./src/ane_tm.o
KERNELDIR := /lib/modules/$(shell uname -r)
PWD := $(shell pwd)

prepare:
default:
	sudo make -C $(KERNELDIR) M=$(PWD) prepare O=$(PWD)
	sudo make -C $(KERNELDIR) M=$(PWD) config O=$(PWD)
	sudo make -C $(KERNELDIR) M=$(PWD) modules O=$(PWD)

headers_install:
	cp -r src/headers/* $(KERNELDIR)/include/drm
install:
	sudo modprobe ane
	sudo chmod 666 /dev/accel/accel0

uninstall:
	sudo modprobe -r ane

clean:
	make -C $(KERNELDIR) M=$(PWD) clean LD=$(LD) 

