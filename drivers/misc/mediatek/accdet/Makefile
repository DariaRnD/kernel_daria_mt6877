subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-$(CONFIG_MACH_MT8167)+= -Werror -I$(srctree)/drivers/misc/mediatek/accdet/$(MTK_PLATFORM)

#subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/accdet/$(MTK_PLATFORM)
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6357)+= -I$(srctree)/drivers/misc/mediatek/accdet/mt6357
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6358)+= -I$(srctree)/drivers/misc/mediatek/accdet/mt6358
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6359)+= -I$(srctree)/drivers/misc/mediatek/accdet/mt6359
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6359P)+= -I$(srctree)/drivers/misc/mediatek/accdet/mt6359
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6390)+= -I$(srctree)/drivers/misc/mediatek/accdet/mt6357

obj-$(CONFIG_MTK_ACCDET) := accdet_drv.o

obj-$(CONFIG_MTK_PMIC_CHIP_MT6357) += mt6357/
obj-$(CONFIG_MTK_PMIC_CHIP_MT6358) += mt6358/
obj-$(CONFIG_MTK_PMIC_CHIP_MT6359) += mt6359/
obj-$(CONFIG_MTK_PMIC_CHIP_MT6359P) += mt6359/
obj-$(CONFIG_MTK_PMIC_CHIP_MT6390) += mt6357/
obj-$(CONFIG_MACH_MT8167) += mt8167/
#prize added by huarui, headset support, 20190111-start
ccflags-$(CONFIG_PRIZE_TYPEC_ACCDET) += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
obj-$(CONFIG_PRIZE_TYPEC_ACCDET) += typec_accdet.o
#prize add by lipengpeng 20220505 start
obj-$(CONFIG_PRIZE_TYPEC_ACCDET) += accdet_auxadc.o
#prize add by lipengpeng 20220505 end
#prize added by huarui, headset support, 20190111-end

# prize added by hanjiuping for hl5280 support start
ccflags-$(CONFIG_TYPEC_AUDIO_FSA4480_SWITCH) += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
obj-$(CONFIG_TYPEC_AUDIO_FSA4480_SWITCH) += accdet_auxadc.o
obj-$(CONFIG_TYPEC_AUDIO_FSA4480_SWITCH) += fsa4480-i2c.o
# prize added by hanjiuping for hl5280 support end