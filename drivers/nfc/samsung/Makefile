#
# Makefile for Samsung NRNx NFC driver
#

obj-$(CONFIG_SEC_NFC)		+= sec_nfc.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
# prize added by hanjiuping for nfc hardware info 2022/03/28
ccflags-$(CONFIG_SEC_NFC)   += -I$(srctree)/drivers/misc/mediatek/hardware_info
