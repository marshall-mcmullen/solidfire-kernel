
# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.

ifneq ($(KERNELRELEASE),)

obj-m := nvme_radian.o
nvme_radian-objs := nvme_main.o nvme_rms.o

else

KVERSION = $(shell uname -r)
KERNELDIR ?= /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)

#export cflags-y += -g -gdwarf-2 -DNVME_DEBUG
#export cflags-y += -g -gdwarf-2

all:
	@echo $(KBUILD_CFLAGS)
	@echo $(EXTRA_CFLAGS)
	$(MAKE) -C $(KERNELDIR) ARCH=x86_64 M=$(PWD) modules
#	$(MAKE) -C $(KERNELDIR) ARCH=i386 M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) ARCH=x86_64 M=$(PWD) clean

endif

