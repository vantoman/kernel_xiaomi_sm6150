# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)

obj-$(CONFIG_FINGERPRINT_FS_TEE)	+= fortsense_fp.o
fortsense_fp-objs			:= sf_ctl.o sf_hw.o

else

KBUILDDIR := $(ANDROID_PRODUCT_OUT)/obj/KERNEL_OBJ
MODULEPWD := $(shell pwd)

.PHONY: all install clean

# $(MAKE) -C $(KBUILDDIR) ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-
# $(MAKE) -C $(KBUILDDIR) ARCH=arm CROSS_COMPILE=arm-eabi-

all:
	$(MAKE) -C $(KBUILDDIR) ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- \
		M=$(MODULEPWD) CONFIG_FINGERPRINT_FS_TEE=m modules

install:
	adb remount
	adb push fortsense_fp.ko /data/
	adb shell sync

clean:
	rm -rf *.ko *.o *.mod.c .*.cmd .*tmp_versions *.symvers *.order 

endif # End of KERNELRELEASE.
