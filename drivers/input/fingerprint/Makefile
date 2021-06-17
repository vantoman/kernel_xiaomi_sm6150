#
# Makefile for the fingerprint drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_FINGERPRINT_FPC_FOD)	+= fpc_fod/
obj-$(CONFIG_FINGERPRINT_GOODIX_FOD) += goodix_fod/
obj-$(CONFIG_FINGERPRINT_GOODIX_TA)	+= goodix_ta/
obj-$(CONFIG_FINGERPRINT_FS_TEE)        += fs_tee/
ifeq ($(CONFIG_MACH_XIAOMI_SWEET),n)
obj-$(CONFIG_FINGERPRINT_FPC_TEE)	+= fpc_tee/
else
obj-$(CONFIG_FINGERPRINT_FPC_TEE)	+= fpc_tee_sweet/
endif
