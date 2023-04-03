/*
 * UPM6910D battery charging driver
 *
 * Copyright (C) 2018 Unisemipower
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define pr_fmt(fmt)	"[upm6910]:%s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <mt-plat/v1/charger_type.h>

//#include "mtk_charger_intf.h"
/*prize add by  zhaopengge 20201031-----start*/
#include "mtk_charger.h"
#include "charger_class.h"
/*prize add by  zhaopengge 20201031-----end*/
#include "upm6910_reg.h"
#include "upm6910.h"

#include <mt-plat/upmu_common.h>

#define POWER_DETECT_ICHG		1020
#define POWER_DETECT_ICL		2000
#define POWER_DETECT_VINDPM		4500

enum {
	PN_UPM6910D,
};

enum upm6910_part_no {
	UPM6910D = 0x02,
};

static int pn_data[] = {
	[PN_UPM6910D] = 0x02,
};

static char *pn_str[] = {
	[PN_UPM6910D] = "upm6910d",
};

struct upm6910 {
	struct device *dev;
	struct i2c_client *client;

	enum upm6910_part_no part_no;
	int revision;

	const char *chg_dev_name;
	const char *eint_name;

	bool chg_det_enable;

	int status;
	int irq;
	u32 intr_gpio;

	struct mutex i2c_rw_lock;

	bool charge_enabled;	/* Register bit status */
	bool power_good;
	bool vbus_gd;

	struct upm6910_platform_data *platform_data;
	struct charger_device *chg_dev;

	//struct power_supply *psy;
	struct power_supply *bat_psy;
	struct power_supply *usb_psy;

	struct delayed_work psy_dwork;
	struct delayed_work prob_dwork;
	int psy_usb_type;
	struct power_supply_desc psy_desc;
	struct power_supply *psy;
	struct power_supply *ext_psy;
};

static const struct charger_properties upm6910_chg_props = {
	.alias_name = "upm6910",
};

static int __upm6910_read_reg(struct upm6910 *upm, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(upm->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __upm6910_write_reg(struct upm6910 *upm, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(upm->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		return ret;
	}
	return 0;
}

static int upm6910_read_byte(struct upm6910 *upm, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&upm->i2c_rw_lock);
	ret = __upm6910_read_reg(upm, reg, data);
	mutex_unlock(&upm->i2c_rw_lock);

	return ret;
}

static int upm6910_write_byte(struct upm6910 *upm, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&upm->i2c_rw_lock);
	ret = __upm6910_write_reg(upm, reg, data);
	mutex_unlock(&upm->i2c_rw_lock);

	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}

static int upm6910_update_bits(struct upm6910 *upm, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&upm->i2c_rw_lock);
	ret = __upm6910_read_reg(upm, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __upm6910_write_reg(upm, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&upm->i2c_rw_lock);
	return ret;
}

static int upm6910_enable_otg(struct upm6910 *upm)
{
	u8 val = REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_01, REG01_OTG_CONFIG_MASK,
				   val);

}

static int upm6910_disable_otg(struct upm6910 *upm)
{
	u8 val = REG01_OTG_DISABLE << REG01_OTG_CONFIG_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_01, REG01_OTG_CONFIG_MASK,
				   val);

}

static int upm6910_enable_charger(struct upm6910 *upm)
{
	int ret;
	u8 val = REG01_CHG_ENABLE << REG01_CHG_CONFIG_SHIFT;

	ret =
	    upm6910_update_bits(upm, UPM6910_REG_01, REG01_CHG_CONFIG_MASK, val);

	return ret;
}

static int upm6910_disable_charger(struct upm6910 *upm)
{
	int ret;
	u8 val = REG01_CHG_DISABLE << REG01_CHG_CONFIG_SHIFT;

	ret =
	    upm6910_update_bits(upm, UPM6910_REG_01, REG01_CHG_CONFIG_MASK, val);
	return ret;
}

int upm6910_set_chargecurrent(struct upm6910 *upm, int curr)
{
	u8 ichg;

	if (curr < REG02_ICHG_BASE)
		curr = REG02_ICHG_BASE;

	if (curr < REG02_ICHG_MIN)
		curr = REG02_ICHG_MIN;

	ichg = (curr - REG02_ICHG_BASE) / REG02_ICHG_LSB;
	return upm6910_update_bits(upm, UPM6910_REG_02, REG02_ICHG_MASK,
				   ichg << REG02_ICHG_SHIFT);

}

int upm6910_set_term_current(struct upm6910 *upm, int curr)
{
	u8 iterm;

	if (curr < REG03_ITERM_BASE)
		curr = REG03_ITERM_BASE;

	iterm = (curr - REG03_ITERM_BASE) / REG03_ITERM_LSB;

	return upm6910_update_bits(upm, UPM6910_REG_03, REG03_ITERM_MASK,
				   iterm << REG03_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(upm6910_set_term_current);

int upm6910_set_prechg_current(struct upm6910 *upm, int curr)
{
	u8 iprechg;

	if (curr < REG03_IPRECHG_BASE)
		curr = REG03_IPRECHG_BASE;

	iprechg = (curr - REG03_IPRECHG_BASE) / REG03_IPRECHG_LSB;

	return upm6910_update_bits(upm, UPM6910_REG_03, REG03_IPRECHG_MASK,
				   iprechg << REG03_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(upm6910_set_prechg_current);

int upm6910_set_chargevolt(struct upm6910 *upm, int volt)
{
	u8 val;

	if (volt < REG04_VREG_BASE)
		volt = REG04_VREG_BASE;

	val = (volt - REG04_VREG_BASE) / REG04_VREG_LSB;
	return upm6910_update_bits(upm, UPM6910_REG_04, REG04_VREG_MASK,
				   val << REG04_VREG_SHIFT);
}

int upm6910_set_input_volt_limit(struct upm6910 *upm, int volt)
{
	u8 val;

	if (volt < REG06_VINDPM_BASE)
		volt = REG06_VINDPM_BASE;

	val = (volt - REG06_VINDPM_BASE) / REG06_VINDPM_LSB;
	return upm6910_update_bits(upm, UPM6910_REG_06, REG06_VINDPM_MASK,
				   val << REG06_VINDPM_SHIFT);
}

static int upm6910_get_input_volt_limit(struct upm6910 *upm, int *volt)
{
	u8 val;
	int ret;
	int vindpm;

	ret = upm6910_read_byte(upm, UPM6910_REG_06, &val);
	if (ret == 0){
		pr_err("Reg[%.2x] = 0x%.2x\n", UPM6910_REG_06, val);
	}

	pr_info("vindpm_reg_val:0x%X\n", val);
	vindpm = (val & REG06_VINDPM_MASK) >> REG06_VINDPM_SHIFT;
	vindpm = vindpm * REG06_VINDPM_LSB + REG06_VINDPM_BASE;
	*volt = vindpm;

	return ret;
}

int upm6910_set_input_current_limit(struct upm6910 *upm, int curr)
{
	u8 val;

	if (curr < REG00_IINLIM_BASE)
		curr = REG00_IINLIM_BASE;

	val = (curr - REG00_IINLIM_BASE) / REG00_IINLIM_LSB;
	return upm6910_update_bits(upm, UPM6910_REG_00, REG00_IINLIM_MASK,
				   val << REG00_IINLIM_SHIFT);
}

int upm6910_set_watchdog_timer(struct upm6910 *upm, u8 timeout)
{
	u8 temp;

	temp = (u8) (((timeout -
		       REG05_WDT_BASE) / REG05_WDT_LSB) << REG05_WDT_SHIFT);

	return upm6910_update_bits(upm, UPM6910_REG_05, REG05_WDT_MASK, temp);
}
EXPORT_SYMBOL_GPL(upm6910_set_watchdog_timer);

int upm6910_disable_watchdog_timer(struct upm6910 *upm)
{
	u8 val = REG05_WDT_DISABLE << REG05_WDT_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_05, REG05_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(upm6910_disable_watchdog_timer);

int upm6910_reset_watchdog_timer(struct upm6910 *upm)
{
	u8 val = REG01_WDT_RESET << REG01_WDT_RESET_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_01, REG01_WDT_RESET_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(upm6910_reset_watchdog_timer);

int upm6910_reset_chip(struct upm6910 *upm)
{
	int ret;
	u8 val = REG0B_REG_RESET << REG0B_REG_RESET_SHIFT;

	ret =
	    upm6910_update_bits(upm, UPM6910_REG_0B, REG0B_REG_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(upm6910_reset_chip);

int upm6910_enter_hiz_mode(struct upm6910 *upm)
{
	u8 val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(upm6910_enter_hiz_mode);

int upm6910_exit_hiz_mode(struct upm6910 *upm)
{

	u8 val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(upm6910_exit_hiz_mode);

static int upm6910_set_hiz_mode(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);

	if (en)
		ret = upm6910_enter_hiz_mode(upm);
	else
		ret = upm6910_exit_hiz_mode(upm);

	pr_err("%s hiz mode %s\n", en ? "enable" : "disable",
			!ret ? "successfully" : "failed");

	return ret;
}
EXPORT_SYMBOL_GPL(upm6910_set_hiz_mode);

static int upm6910_get_hiz_mode(struct charger_device *chg_dev)
{
	int ret;
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);
	u8 val;
	ret = upm6910_read_byte(upm, UPM6910_REG_00, &val);
	if (ret == 0){
		pr_err("Reg[%.2x] = 0x%.2x\n", UPM6910_REG_00, val);
	}

	ret = (val & REG00_ENHIZ_MASK) >> REG00_ENHIZ_SHIFT;
	pr_err("%s:hiz mode %s\n",__func__, ret ? "enabled" : "disabled");

	return ret;
}
EXPORT_SYMBOL_GPL(upm6910_get_hiz_mode);

static int upm6910_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
    struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);

    dev_info(upm->dev, "%s event = %d\n", __func__, event);

    switch (event) {
    case EVENT_FULL:
        charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
        break;
    case EVENT_RECHARGE:
        charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
        break;
    default:
        break;
    }
    return 0;
}

static int upm6910_enable_term(struct upm6910 *upm, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = REG05_TERM_ENABLE << REG05_EN_TERM_SHIFT;
	else
		val = REG05_TERM_DISABLE << REG05_EN_TERM_SHIFT;

	ret = upm6910_update_bits(upm, UPM6910_REG_05, REG05_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(upm6910_enable_term);

int upm6910_set_boost_current(struct upm6910 *upm, int curr)
{
	u8 val;

	val = REG02_BOOST_LIM_0P5A;
	if (curr == BOOSTI_1200)
		val = REG02_BOOST_LIM_1P2A;

	return upm6910_update_bits(upm, UPM6910_REG_02, REG02_BOOST_LIM_MASK,
				   val << REG02_BOOST_LIM_SHIFT);
}

int upm6910_set_boost_voltage(struct upm6910 *upm, int volt)
{
	u8 val;

	if (volt == BOOSTV_4850)
		val = REG06_BOOSTV_4P85V;
	else if (volt == BOOSTV_5150)
		val = REG06_BOOSTV_5P15V;
	else if (volt == BOOSTV_5300)
		val = REG06_BOOSTV_5P3V;
	else
		val = REG06_BOOSTV_5V;

	return upm6910_update_bits(upm, UPM6910_REG_06, REG06_BOOSTV_MASK,
				   val << REG06_BOOSTV_SHIFT);
}
EXPORT_SYMBOL_GPL(upm6910_set_boost_voltage);

static int upm6910_set_acovp_threshold(struct upm6910 *upm, int volt)
{
	u8 val;

	if (volt == VAC_OVP_14000)
		val = REG06_OVP_14P0V;
	else if (volt == VAC_OVP_10500)
		val = REG06_OVP_10P5V;
	else if (volt == VAC_OVP_6500)
		val = REG06_OVP_6P5V;
	else
		val = REG06_OVP_5P5V;

	return upm6910_update_bits(upm, UPM6910_REG_06, REG06_OVP_MASK,
				   val << REG06_OVP_SHIFT);
}
EXPORT_SYMBOL_GPL(upm6910_set_acovp_threshold);

static int upm6910_set_stat_ctrl(struct upm6910 *upm, int ctrl)
{
	u8 val;

	val = ctrl;

	return upm6910_update_bits(upm, UPM6910_REG_00, REG00_STAT_CTRL_MASK,
				   val << REG00_STAT_CTRL_SHIFT);
}

static int upm6910_set_int_mask(struct upm6910 *upm, int mask)
{
	u8 val;

	val = mask;

	return upm6910_update_bits(upm, UPM6910_REG_0A, REG0A_INT_MASK_MASK,
				   val << REG0A_INT_MASK_SHIFT);
}

static int upm6910_force_dpdm(struct upm6910 *upm)
{
	const u8 val = REG07_FORCE_DPDM << REG07_FORCE_DPDM_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_07, REG07_FORCE_DPDM_MASK,
				val);
}

static int upm6910_enable_batfet(struct upm6910 *upm)
{
	const u8 val = REG07_BATFET_ON << REG07_BATFET_DIS_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_07, REG07_BATFET_DIS_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(upm6910_enable_batfet);

static int upm6910_disable_batfet(struct upm6910 *upm)
{
	const u8 val = REG07_BATFET_OFF << REG07_BATFET_DIS_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_07, REG07_BATFET_DIS_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(upm6910_disable_batfet);

static int upm6910_set_batfet_delay(struct upm6910 *upm, uint8_t delay)
{
	u8 val;

	if (delay == 0)
		val = REG07_BATFET_DLY_0S;
	else
		val = REG07_BATFET_DLY_10S;

	val <<= REG07_BATFET_DLY_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_07, REG07_BATFET_DLY_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(upm6910_set_batfet_delay);

static int upm6910_enable_safety_timer(struct upm6910 *upm)
{
	const u8 val = REG05_CHG_TIMER_ENABLE << REG05_EN_TIMER_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_05, REG05_EN_TIMER_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(upm6910_enable_safety_timer);

static int upm6910_disable_safety_timer(struct upm6910 *upm)
{
	const u8 val = REG05_CHG_TIMER_DISABLE << REG05_EN_TIMER_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_05, REG05_EN_TIMER_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(upm6910_disable_safety_timer);

static int upm6910_enable_hvdcp(struct upm6910 *upm, bool en)
{
	const u8 val = en << REG0C_EN_HVDCP_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_0C, REG0C_EN_HVDCP_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(upm6910_enable_hvdcp);

static int upm6910_set_dp(struct upm6910 *upm, int dp_stat)
{
	const u8 val = dp_stat << REG0C_DP_MUX_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_0C, REG0C_DP_MUX_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(upm6910_set_dp);

static int upm6910_set_dm(struct upm6910 *upm, int dm_stat)
{
	const u8 val = dm_stat << REG0C_DM_MUX_SHIFT;

	return upm6910_update_bits(upm, UPM6910_REG_0C, REG0C_DM_MUX_MASK,
				   val);
}
EXPORT_SYMBOL_GPL(upm6910_set_dm);

static int upm6910_dpdm_set_hidden_mode(struct upm6910 *upm)
{
	int ret = 0;
	union power_supply_propval propval;

	if (!upm->bat_psy) {
		upm->bat_psy = power_supply_get_by_name("mtk-gauge");
		if (!upm->bat_psy) {
			pr_err("%s Couldn't get mtk-gauge psy\n", __func__);
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(upm->bat_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &propval);
	if (ret || propval.intval <= 3450000) {
		pr_err("%s cannot goto hidden mode\n", __func__);
		return -EINVAL;
	}

	ret = upm6910_write_byte(upm, 0xA9, 0x6E);
	if (ret) {
		pr_err("write 0xA9, 0x6E failed(%d)\n", ret);
	}
	ret = upm6910_write_byte(upm, 0xB1, 0x80);
	if (ret) {
		pr_err("write 0xB1, 0x80 failed(%d)\n", ret);
	}
	ret = upm6910_write_byte(upm, 0xB0, 0x21);
	if (ret) {
		pr_err("write 0xB0, 0x21 failed(%d)\n", ret);
	}
	ret = upm6910_write_byte(upm, 0xB1, 0x00);
	if (ret) {
		pr_err("write 0xB1, 0x00 failed(%d)\n", ret);
	}
	ret = upm6910_write_byte(upm, 0xA9, 0x00);
	if (ret) {
		pr_err("write 0xA9, 0x00 failed(%d)\n", ret);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(upm6910_dpdm_set_hidden_mode);

static int upm6910_get_ichg(struct charger_device *chg_dev, u32 *curr)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int ichg;
	int ret;

	ret = upm6910_read_byte(upm, UPM6910_REG_02, &reg_val);
	if (!ret) {
		ichg = (reg_val & REG02_ICHG_MASK) >> REG02_ICHG_SHIFT;
		ichg = ichg * REG02_ICHG_LSB + REG02_ICHG_BASE;
		*curr = ichg * 1000;
	}

	return ret;
}

static int upm6910_get_icl(struct charger_device *chg_dev, u32 *curr)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int icl;
	int ret;

	ret = upm6910_read_byte(upm, UPM6910_REG_00, &reg_val);
	if (!ret) {
		icl = (reg_val & REG00_IINLIM_MASK) >> REG00_IINLIM_SHIFT;
		icl = icl * REG00_IINLIM_LSB + REG00_IINLIM_BASE;
		*curr = icl * 1000;
	}

	return ret;
}

static struct upm6910_platform_data *upm6910_parse_dt(struct device_node *np,
						      struct upm6910 *upm)
{
	int ret;
	struct upm6910_platform_data *pdata;

	pdata = devm_kzalloc(&upm->client->dev, sizeof(struct upm6910_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (of_property_read_string(np, "charger_name", &upm->chg_dev_name) < 0) {
		upm->chg_dev_name = "primary_chg";
		pr_warn("no charger name\n");
	}

	if (of_property_read_string(np, "eint_name", &upm->eint_name) < 0) {
		upm->eint_name = "chr_stat";
		pr_warn("no eint name\n");
	}

	ret = of_get_named_gpio(np, "upm,intr_gpio", 0);
	if (ret < 0) {
		pr_err("%s no upm,intr_gpio(%d)\n",
				      __func__, ret);
	} else
		upm->intr_gpio = ret;

	pr_info("%s intr_gpio = %u\n",
			    __func__, upm->intr_gpio);

	upm->chg_det_enable =
	    of_property_read_bool(np, "upm,upm6910,charge-detect-enable");

	ret = of_property_read_u32(np, "upm,upm6910,usb-vlim", &pdata->usb.vlim);
	if (ret) {
		pdata->usb.vlim = 4500;
		pr_err("Failed to read node of upm,upm6910,usb-vlim\n");
	}

	ret = of_property_read_u32(np, "upm,upm6910,usb-ilim", &pdata->usb.ilim);
	if (ret) {
		pdata->usb.ilim = 2000;
		pr_err("Failed to read node of upm,upm6910,usb-ilim\n");
	}

	ret = of_property_read_u32(np, "upm,upm6910,usb-vreg", &pdata->usb.vreg);
	if (ret) {
		pdata->usb.vreg = 4200;
		pr_err("Failed to read node of upm,upm6910,usb-vreg\n");
	}

	ret = of_property_read_u32(np, "upm,upm6910,usb-ichg", &pdata->usb.ichg);
	if (ret) {
		pdata->usb.ichg = 2000;
		pr_err("Failed to read node of upm,upm6910,usb-ichg\n");
	}

	ret = of_property_read_u32(np, "upm,upm6910,stat-pin-ctrl",
				   &pdata->statctrl);
	if (ret) {
		pdata->statctrl = 0;
		pr_err("Failed to read node of upm,upm6910,stat-pin-ctrl\n");
	}

	ret = of_property_read_u32(np, "upm,upm6910,precharge-current",
				   &pdata->iprechg);
	if (ret) {
		pdata->iprechg = 180;
		pr_err("Failed to read node of upm,upm6910,precharge-current\n");
	}

	ret = of_property_read_u32(np, "upm,upm6910,termination-current",
				   &pdata->iterm);
	if (ret) {
		pdata->iterm = 180;
		pr_err
		    ("Failed to read node of upm,upm6910,termination-current\n");
	}

	ret =
	    of_property_read_u32(np, "upm,upm6910,boost-voltage",
				 &pdata->boostv);
	if (ret) {
		pdata->boostv = 5000;
		pr_err("Failed to read node of upm,upm6910,boost-voltage\n");
	}

	ret =
	    of_property_read_u32(np, "upm,upm6910,boost-current",
				 &pdata->boosti);
	if (ret) {
		pdata->boosti = 1200;
		pr_err("Failed to read node of upm,upm6910,boost-current\n");
	}

	ret = of_property_read_u32(np, "upm,upm6910,vac-ovp-threshold",
				   &pdata->vac_ovp);
	if (ret) {
		pdata->vac_ovp = 6500;
		pr_err("Failed to read node of upm,upm6910,vac-ovp-threshold\n");
	}

	return pdata;
}
#if 0
static bool is_hiz_mode_trigger_interrupt(struct upm6910 *upm)
{
	bool exist = false;
	int ret = 0, vbus = 0, hiz_en = 0;
	u8 reg_val = 0, vbus_stat = 0;
	union power_supply_propval propval;

	if (!upm->usb_psy) {
		upm->usb_psy = power_supply_get_by_name("mtk_charger_type");
		if (!upm->usb_psy) {
			pr_err("%s Couldn't get mtk_charger_type psy\n", __func__);
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(upm->usb_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &propval);
	if (!ret) {
		vbus = propval.intval;
	}

	ret = upm6910_read_byte(upm, UPM6910_REG_08, &reg_val);
	if (!ret) {
		vbus_stat = (reg_val & REG08_VBUS_STAT_MASK);
		vbus_stat >>= REG08_VBUS_STAT_SHIFT;
	}

	ret = upm6910_read_byte(upm, UPM6910_REG_00, &reg_val);
	if (!ret) {
		hiz_en = !!(reg_val & REG00_ENHIZ_MASK);
	}

	if (vbus > 4000 && vbus_stat != REG08_VBUS_TYPE_OTG && hiz_en) {
		exist = true;
	}

	return exist;
}

static int upm6910_get_charger_type(struct upm6910 *upm, int *type)
{
	int ret;

	u8 reg_val = 0;
	int vbus_stat = 0;
	int chg_type = CHARGER_UNKNOWN;

	ret = upm6910_read_byte(upm, UPM6910_REG_08, &reg_val);
	if (ret)
		return ret;

	vbus_stat = (reg_val & REG08_VBUS_STAT_MASK);
	vbus_stat >>= REG08_VBUS_STAT_SHIFT;

	pr_info("[%s] reg08: 0x%02x\n", __func__, reg_val);

	switch (vbus_stat) {

	case REG08_VBUS_TYPE_NONE:
		chg_type = CHARGER_UNKNOWN;
		break;
	case REG08_VBUS_TYPE_SDP:
		chg_type = STANDARD_HOST;
		break;
	case REG08_VBUS_TYPE_CDP:
		chg_type = CHARGING_HOST;
		break;
	case REG08_VBUS_TYPE_DCP:
		chg_type = STANDARD_CHARGER;
		break;
	case REG08_VBUS_TYPE_UNKNOWN:
		chg_type = NONSTANDARD_CHARGER;
		break;
	case REG08_VBUS_TYPE_NON_STD:
		chg_type = STANDARD_CHARGER;
		break;
	default:
		chg_type = NONSTANDARD_CHARGER;
		break;
	}

	if (chg_type != STANDARD_CHARGER) {
		Charger_Detect_Release();
	}

	*type = chg_type;

	return 0;
}

static void upm6910_inform_psy_dwork_handler(struct work_struct *work)
{
	int ret = 0;
	union power_supply_propval propval;
	struct upm6910 *upm = container_of(work, struct upm6910,
								psy_dwork.work);

	if (!upm->psy) {
		upm->psy = power_supply_get_by_name("mtk_charger_type");
		if (!upm->psy) {
			pr_err("%s Couldn't get psy\n", __func__);
			//mod_delayed_work(system_wq, &upm->psy_dwork,
			//		msecs_to_jiffies(2*1000));
			return;
		}
	}

	if (upm->psy_usb_type != CHARGER_UNKNOWN)
		propval.intval = 1;
	else
		propval.intval = 0;

	ret = power_supply_set_property(upm->psy, POWER_SUPPLY_PROP_ONLINE,
					&propval);
	if (ret < 0)
		pr_notice("inform power supply online failed:%d\n", ret);

	propval.intval = upm->psy_usb_type;

	ret = power_supply_set_property(upm->psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE,
					&propval);

	if (ret < 0)
		pr_notice("inform power supply charge type failed:%d\n", ret);

	return;
}
#endif
#if 0
static irqreturn_t upm6910_irq_handler(int irq, void *data)
{
	int ret;
	u8 reg_val;
	bool prev_pg;
	bool prev_vbus_gd;
	bool hiz_irq = false;

    struct upm6910 *upm = (struct upm6910 *)data;

	ret = upm6910_read_byte(upm, UPM6910_REG_0A, &reg_val);
	if (ret)
		return IRQ_HANDLED;

	prev_vbus_gd = upm->vbus_gd;

	upm->vbus_gd = !!(reg_val & REG0A_VBUS_GD_MASK);

	ret = upm6910_read_byte(upm, UPM6910_REG_08, &reg_val);
	if (ret)
		return IRQ_HANDLED;

	prev_pg = upm->power_good;

	upm->power_good = !!(reg_val & REG08_PG_STAT_MASK);

	if (!prev_vbus_gd && upm->vbus_gd) {
		pr_notice("adapter/usb inserted\n");
	} else if (prev_vbus_gd && !upm->vbus_gd) {
		hiz_irq = is_hiz_mode_trigger_interrupt(upm);
		if (hiz_irq) {
			pr_notice("hiz mode trigger interrupt\n");
			upm->vbus_gd = true;
			upm->power_good = true;
			return IRQ_HANDLED;
		}
	    upm->psy_usb_type = CHARGER_UNKNOWN;
		schedule_delayed_work(&upm->psy_dwork, 0);
		Charger_Detect_Init();
		pr_notice("adapter/usb removed\n");
		return IRQ_HANDLED;
	}

	if (!prev_pg && upm->power_good) {
		ret = upm6910_get_charger_type(upm, &upm->psy_usb_type);
		schedule_delayed_work(&upm->psy_dwork, 0);
	}

	return IRQ_HANDLED;
}

static int upm6910_register_interrupt(struct upm6910 *upm)
{
	int ret = 0;
	ret = devm_gpio_request_one(upm->dev, upm->intr_gpio, GPIOF_DIR_IN,
			devm_kasprintf(upm->dev, GFP_KERNEL,
			"upm6910_intr_gpio.%s", dev_name(upm->dev)));
	if (ret < 0) {
		pr_err("%s gpio request fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	upm->client->irq = gpio_to_irq(upm->intr_gpio);
	if (upm->client->irq < 0) {
		pr_err("%s gpio2irq fail(%d)\n",
				      __func__, upm->client->irq);
		return upm->client->irq;
	}
	pr_info("%s irq = %d\n", __func__, upm->client->irq);

	ret = devm_request_threaded_irq(upm->dev, upm->client->irq, NULL,
					upm6910_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"upm_irq", upm);
	if (ret < 0) {
		pr_err("request thread irq failed:%d\n", ret);
		return ret;
	}

	enable_irq_wake(upm->irq);

	return 0;
}
#endif
static int upm6910_init_device(struct upm6910 *upm)
{
	int ret;

	upm6910_disable_watchdog_timer(upm);

	ret = upm6910_set_stat_ctrl(upm, upm->platform_data->statctrl);
	if (ret)
		pr_err("Failed to set stat pin control mode, ret = %d\n", ret);

	ret = upm6910_set_prechg_current(upm, upm->platform_data->iprechg);
	if (ret)
		pr_err("Failed to set prechg current, ret = %d\n", ret);

	ret = upm6910_set_term_current(upm, upm->platform_data->iterm);
	if (ret)
		pr_err("Failed to set termination current, ret = %d\n", ret);

	ret = upm6910_set_boost_voltage(upm, upm->platform_data->boostv);
	if (ret)
		pr_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = upm6910_set_boost_current(upm, upm->platform_data->boosti);
	if (ret)
		pr_err("Failed to set boost current, ret = %d\n", ret);

	ret = upm6910_set_acovp_threshold(upm, upm->platform_data->vac_ovp);
	if (ret)
		pr_err("Failed to set acovp threshold, ret = %d\n", ret);

	ret = upm6910_set_int_mask(upm,
				   REG0A_IINDPM_INT_MASK |
				   REG0A_VINDPM_INT_MASK);
	if (ret)
		pr_err("Failed to set vindpm and iindpm int mask\n");

	return 0;
}
#if 0
static void upm6910_inform_prob_dwork_handler(struct work_struct *work)
{
	struct upm6910 *upm = container_of(work, struct upm6910,
								prob_dwork.work);
	Charger_Detect_Init();
	msleep(50);
	upm6910_dpdm_set_hidden_mode(upm);
	msleep(700);
	upm6910_force_dpdm(upm);
	msleep(300);
	upm6910_irq_handler(upm->irq, (void *) upm);
}
#endif
static int upm6910_detect_device(struct upm6910 *upm)
{
	int ret;
	u8 data;

	ret = upm6910_read_byte(upm, UPM6910_REG_0B, &data);
	if (!ret) {
		upm->part_no = (data & REG0B_PN_MASK) >> REG0B_PN_SHIFT;
		upm->revision =
		    (data & REG0B_DEV_REV_MASK) >> REG0B_DEV_REV_SHIFT;
	}

	return ret;
}

static void upm6910_dump_regs(struct upm6910 *upm)
{
	int addr;
	u8 val;
	int ret;

	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = upm6910_read_byte(upm, addr, &val);
		if (ret == 0)
			pr_err("Reg[%.2x] = 0x%.2x\n", addr, val);
	}
}

static ssize_t
upm6910_show_registers(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct upm6910 *upm = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "upm6910 Reg");
	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = upm6910_read_byte(upm, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				       "Reg[%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t
upm6910_store_registers(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct upm6910 *upm = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < 0x0B) {
		upm6910_write_byte(upm, (unsigned char) reg,
				   (unsigned char) val);
	}

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, upm6910_show_registers,
		   upm6910_store_registers);

static struct attribute *upm6910_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group upm6910_attr_group = {
	.attrs = upm6910_attributes,
};

static int upm6910_charging(struct charger_device *chg_dev, bool enable)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;
	//drv huangjiwu for psy start
	static bool en_last = false;
	//drv huangjiwu for psy end
	if (enable)
		ret = upm6910_enable_charger(upm);
	else
		ret = upm6910_disable_charger(upm);

	pr_err("%s charger %s\n", enable ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	ret = upm6910_read_byte(upm, UPM6910_REG_01, &val);

	if (!ret)
		upm->charge_enabled = !!(val & REG01_CHG_CONFIG_MASK);

	//drv huangjiwu for psy start	
	pr_err("gezi  en_last=%d .enable = %d..\n",en_last,enable);
	if((upm->psy) && (en_last != enable)){	
		pr_err("gezi  power_supply_changed...\n");
		power_supply_changed(upm->psy);
	}
	
	en_last = enable;
	//drv huangjiwu for psy end
	return ret;
}

static int upm6910_plug_in(struct charger_device *chg_dev)
{

	int ret;

	ret = upm6910_charging(chg_dev, true);

	if (ret)
		pr_err("Failed to enable charging:%d\n", ret);

	return ret;
}

static int upm6910_plug_out(struct charger_device *chg_dev)
{
	int ret;

	ret = upm6910_charging(chg_dev, false);

	if (ret)
		pr_err("Failed to disable charging:%d\n", ret);

	return ret;
}

static int upm6910_dump_register(struct charger_device *chg_dev)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);

	upm6910_dump_regs(upm);

	return 0;
}

static int upm6910_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);

	*en = upm->charge_enabled;

	return 0;
}

static int upm6910_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;

	ret = upm6910_read_byte(upm, UPM6910_REG_08, &val);
	if (!ret) {
		val = val & REG08_CHRG_STAT_MASK;
		val = val >> REG08_CHRG_STAT_SHIFT;
		*done = (val == REG08_CHRG_STAT_CHGDONE);
	}

	return ret;
}

static int upm6910_set_ichg(struct charger_device *chg_dev, u32 curr)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);

	pr_err("charge curr = %d\n", curr);

	return upm6910_set_chargecurrent(upm, curr / 1000);
}

static int upm6910_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	*curr = 60 * 1000;

	return 0;
}

static int upm6910_set_vchg(struct charger_device *chg_dev, u32 volt)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);

	pr_err("charge volt = %d\n", volt);

	return upm6910_set_chargevolt(upm, volt / 1000);
}

static int upm6910_get_vchg(struct charger_device *chg_dev, u32 *volt)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int vchg;
	int ret;

	ret = upm6910_read_byte(upm, UPM6910_REG_04, &reg_val);
	if (!ret) {
		vchg = (reg_val & REG04_VREG_MASK) >> REG04_VREG_SHIFT;
		vchg = vchg * REG04_VREG_LSB + REG04_VREG_BASE;
		*volt = vchg * 1000;
	}

	return ret;
}

static int upm6910_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);

	pr_err("vindpm volt = %d\n", volt);

	return upm6910_set_input_volt_limit(upm, volt / 1000);

}

static int upm6910_set_icl(struct charger_device *chg_dev, u32 curr)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);

	pr_err("indpm curr = %d\n", curr);

	return upm6910_set_input_current_limit(upm, curr / 1000);
}

static int upm6910_kick_wdt(struct charger_device *chg_dev)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);

	return upm6910_reset_watchdog_timer(upm);
}

static int upm6910_set_otg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);

	if (en)
		ret = upm6910_enable_otg(upm);
	else
		ret = upm6910_disable_otg(upm);

	pr_err("%s OTG %s\n", en ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	return ret;
}

static int upm6910_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);
	int ret;

	if (en)
		ret = upm6910_enable_safety_timer(upm);
	else
		ret = upm6910_disable_safety_timer(upm);

	return ret;
}

static int upm6910_is_safety_timer_enabled(struct charger_device *chg_dev,
					   bool *en)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = upm6910_read_byte(upm, UPM6910_REG_05, &reg_val);

	if (!ret)
		*en = !!(reg_val & REG05_EN_TIMER_MASK);

	return ret;
}

static int upm6910_set_boost_ilmt(struct charger_device *chg_dev, u32 curr)
{
	struct upm6910 *upm = dev_get_drvdata(&chg_dev->dev);
	int ret;

	pr_err("otg curr = %d\n", curr);

	ret = upm6910_set_boost_current(upm, curr / 1000);

	return ret;
}

static struct charger_ops upm6910_chg_ops = {
	/* Normal charging */
	.plug_in = upm6910_plug_in,
	.plug_out = upm6910_plug_out,
	.dump_registers = upm6910_dump_register,
	.enable = upm6910_charging,
	.is_enabled = upm6910_is_charging_enable,
	.get_charging_current = upm6910_get_ichg,
	.set_charging_current = upm6910_set_ichg,
	.get_input_current = upm6910_get_icl,
	.set_input_current = upm6910_set_icl,
	.get_constant_voltage = upm6910_get_vchg,
	.set_constant_voltage = upm6910_set_vchg,
	.kick_wdt = upm6910_kick_wdt,
	.set_mivr = upm6910_set_ivl,
	.is_charging_done = upm6910_is_charging_done,
	.get_min_charging_current = upm6910_get_min_ichg,

	/* Safety timer */
	.enable_safety_timer = upm6910_set_safety_timer,
	.is_safety_timer_enabled = upm6910_is_safety_timer_enabled,

	/* Power path */
	.enable_powerpath = NULL,
	.is_powerpath_enabled = NULL,

	/* OTG */
	.enable_otg = upm6910_set_otg,
	.set_boost_current_limit = upm6910_set_boost_ilmt,
	.enable_discharge = NULL,

	/* PE+/PE+20 */
	.send_ta_current_pattern = NULL,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
	.enable_cable_drop_comp = NULL,

   /* Event */
    .event = upm6910_do_event,

   /* HIZ */
	//.set_hiz_mode = upm6910_set_hiz_mode,
	//.get_hiz_mode = upm6910_get_hiz_mode,

	/* ADC */
	.get_tchg_adc = NULL,
};

static struct of_device_id upm6910_charger_match_table[] = {
	{
	 .compatible = "upm,upm6910d",
	 .data = &pn_data[PN_UPM6910D],
	 },
	{},
};
MODULE_DEVICE_TABLE(of, upm6910_charger_match_table);

static int upm6910_charger_remove(struct i2c_client *client);

enum upm6910_charging_status {
	UPM6910_CHG_STATUS_READY = 0,
	UPM6910_CHG_STATUS_PRE,
	UPM6910_CHG_STATUS_PROGRESS,
	UPM6910_CHG_STATUS_DONE,
	//UPM6910_CHG_STATUS_FAULT,
	UPM6910_CHG_STATUS_MAX,
};

static enum power_supply_property upm6910_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static enum power_supply_usb_type upm6910_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static int upm6910_charger_get_type(struct upm6910 *info)
{
	union power_supply_propval prop;
	int ret = 0;
	
	if (IS_ERR_OR_NULL(info->ext_psy)){
		return -EINVAL;
	}
	
	ret = power_supply_get_property(info->ext_psy,POWER_SUPPLY_PROP_TYPE,&prop);
	
	
	return prop.intval;
}

static int upm6910_charger_get_usb_type(struct upm6910 *info)
{
	union power_supply_propval prop;
	int ret = 0;
	
	
	if (IS_ERR_OR_NULL(info->ext_psy)){
		return -EINVAL;
	}
	ret = power_supply_get_property(info->ext_psy,POWER_SUPPLY_PROP_USB_TYPE,&prop);
	
	
	return prop.intval;
}

unsigned int upm6910_get_chip_status(struct upm6910 *info)
{
	unsigned char val = 0;
	upm6910_read_byte(info, UPM6910_REG_08, &val);

	return val;
}

static int upm6910_get_charging_status_t(struct upm6910 *info,enum upm6910_charging_status *chg_stat)
{

	unsigned int status = 0;
	unsigned int ret_val;
	
	ret_val = upm6910_get_chip_status(info);
	ret_val = ret_val & REG08_CHRG_STAT_MASK;
	ret_val = ret_val >> REG08_CHRG_STAT_SHIFT;
	*chg_stat = ret_val;
	

	return status;

}

static int upm6910_charger_get_online(struct upm6910 *info,bool *val)
{
	union power_supply_propval prop;
	int ret = 0;
	
	if (IS_ERR_OR_NULL(info->ext_psy)){
		return -EINVAL;
	}
	
	ret = power_supply_get_property(info->ext_psy,POWER_SUPPLY_PROP_ONLINE,&prop);

	*val = prop.intval;
		
	return 0;
}

static int upm6910_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct upm6910 *info = power_supply_get_drvdata(psy);
	
	enum upm6910_charging_status chg_stat = UPM6910_CHG_STATUS_READY;
	bool pwr_rdy = false, chg_en = false;
	int ret = 0;
/*prize LiuYong, Modify eta6937 driver boot sequence, 20220221, start*/
	if (!info->ext_psy)
		info->ext_psy = power_supply_get_by_name("mtk_charger_type");

	if (!info->ext_psy) {
		pr_err("%s: get ext_psy power supply failed\n", __func__);
		return -EINVAL;
	}
/*prize LiuYong, Modify eta6937 driver boot sequence, 20220221, end*/
	
	//dev_dbg(chg_data->dev, "%s: prop = %d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = upm6910_charger_get_online(info, &pwr_rdy);
		val->intval = pwr_rdy;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = upm6910_charger_get_online(info, &pwr_rdy);
		ret = upm6910_is_charging_enable(info->chg_dev, &chg_en);
		ret = upm6910_get_charging_status_t(info, &chg_stat);
		
		//pr_err("gezi %s pwr_rdy = %d,chg_en = %d,chg_stat = %d\n",__func__,pwr_rdy,chg_en,chg_stat);
		
		if (!pwr_rdy) {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			return ret;
		}
		switch (chg_stat) {
		case UPM6910_CHG_STATUS_READY:
			/* fallthrough */
		case UPM6910_CHG_STATUS_PROGRESS:
			if (chg_en)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case UPM6910_CHG_STATUS_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		case UPM6910_CHG_STATUS_PRE:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		default:
			ret = -ENODATA;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = upm6910_charger_get_type(info);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = upm6910_charger_get_usb_type(info);
		break;
	default:
		ret = -ENODATA;
	}
	return ret;
}

static const struct power_supply_desc upm6910_charger_desc = {
	.name 			= "upm6910",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= upm6910_charger_properties,
	.num_properties		= ARRAY_SIZE(upm6910_charger_properties),
	.get_property		= upm6910_charger_get_property,
	.usb_types		= upm6910_charger_usb_types,
	.num_usb_types		= ARRAY_SIZE(upm6910_charger_usb_types),
};

static char *upm6910_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};


static int upm6910_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct upm6910 *upm;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;


	struct power_supply_config charger_cfg = {};
	
	int ret = 0;

	upm = devm_kzalloc(&client->dev, sizeof(struct upm6910), GFP_KERNEL);
	if (!upm)
		return -ENOMEM;

/*prize LiuYong, Modify eta6937 driver boot time, 20220319, start*/
	upm->ext_psy = power_supply_get_by_name("mtk_charger_type");

	if (!upm->ext_psy) {
		pr_err("%s: get ext_psy power supply failed\n", __func__);
		return -EPROBE_DEFER;
	}
/*prize LiuYong, Modify eta6937 driver boot time, 20220319, end*/

	client->addr = 0x6B;
	upm->dev = &client->dev;
	upm->client = client;

	i2c_set_clientdata(client, upm);

	mutex_init(&upm->i2c_rw_lock);

	ret = upm6910_detect_device(upm);
	if (ret) {
		pr_err("No upm6910 device found!\n");
		return -ENODEV;
	}

	match = of_match_node(upm6910_charger_match_table, node);
	if (match == NULL) {
		pr_err("device tree match not found\n");
		return -EINVAL;
	}

	if (upm->part_no != *(int *)match->data) {
		pr_info("part no mismatch, hw:%s, devicetree:%s\n",
			pn_str[upm->part_no], pn_str[*(int *) match->data]);
        upm6910_charger_remove(client);
        return -EINVAL;
	}

	upm->platform_data = upm6910_parse_dt(node, upm);
	if (!upm->platform_data) {
		pr_err("No platform data provided.\n");
		return -EINVAL;
	}

	//INIT_DELAYED_WORK(&upm->psy_dwork, upm6910_inform_psy_dwork_handler);
	//INIT_DELAYED_WORK(&upm->prob_dwork, upm6910_inform_prob_dwork_handler);

	ret = upm6910_init_device(upm);
	if (ret) {
		pr_err("Failed to init device\n");
		return ret;
	}

	//upm6910_register_interrupt(upm);

	upm->chg_dev = charger_device_register(upm->chg_dev_name,
					      &client->dev, upm,
					      &upm6910_chg_ops,
					      &upm6910_chg_props);
	if (IS_ERR_OR_NULL(upm->chg_dev)) {
		ret = PTR_ERR(upm->chg_dev);
		return ret;
	}

	if (!upm->usb_psy) {
		upm->usb_psy = power_supply_get_by_name("mtk_charger_type");
		if (!upm->usb_psy) {
			pr_err("%s Couldn't get mtk_charger_type psy\n", __func__);
		}
	}

	ret = sysfs_create_group(&upm->dev->kobj, &upm6910_attr_group);
	if (ret)
		dev_err(upm->dev, "failed to register sysfs. err: %d\n", ret);

	//mod_delayed_work(system_wq, &upm->prob_dwork,
	//				msecs_to_jiffies(2*1000));

	/* power supply register */
	memcpy(&upm->psy_desc, &upm6910_charger_desc, sizeof(upm->psy_desc));
	
	upm->psy_desc.name = "upm6910_charger";

	i2c_set_clientdata(client, upm); /*prize LiuYong, Modify eta6937 driver boot sequence, 20220221*/
	charger_cfg.drv_data = upm;
	charger_cfg.of_node = client->dev.of_node;
	charger_cfg.supplied_to = upm6910_charger_supplied_to;
	charger_cfg.num_supplicants = ARRAY_SIZE(upm6910_charger_supplied_to);
	upm->psy = devm_power_supply_register(&client->dev,&upm->psy_desc, &charger_cfg);
	if (IS_ERR(upm->psy)) {
		dev_notice(&client->dev, "gezi : Fail to register power supply dev\n");
		ret = PTR_ERR(upm->psy);
		return -EINVAL;
	}
	pr_err("upm6910 probe successfully, Part Num:%d, Revision:%d\n!",
	       upm->part_no, upm->revision);
	return 0;
}

static int upm6910_charger_remove(struct i2c_client *client)
{
	struct upm6910 *upm = i2c_get_clientdata(client);

	mutex_destroy(&upm->i2c_rw_lock);

	sysfs_remove_group(&upm->dev->kobj, &upm6910_attr_group);

	return 0;
}

static void upm6910_charger_shutdown(struct i2c_client *client)
{

}

static struct i2c_driver upm6910_charger_driver = {
	.driver = {
		   .name = "upm6910-charger",
		   .owner = THIS_MODULE,
		   .of_match_table = upm6910_charger_match_table,
		   },

	.probe = upm6910_charger_probe,
	.remove = upm6910_charger_remove,
	.shutdown = upm6910_charger_shutdown,

};

module_i2c_driver(upm6910_charger_driver);

MODULE_DESCRIPTION("Unisemipower UPM6910D Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Unisemepower");
