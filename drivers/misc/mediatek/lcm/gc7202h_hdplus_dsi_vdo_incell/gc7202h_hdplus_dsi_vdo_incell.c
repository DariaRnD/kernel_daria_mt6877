/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(ALWAYS, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(INFO, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_DSI_CMD_MODE                0
#define FRAME_WIDTH			(720)
#define FRAME_HEIGHT			(1600)

//prize-tangcong modify LCD size-20200331-start
#define LCM_PHYSICAL_WIDTH                  				(68000)
#define LCM_PHYSICAL_HEIGHT                  				(151000)
//prize-tangcong modify LCD size-20200331-end

#define REGFLAG_PORT_SWAP               0xFFFA
#define REGFLAG_DELAY                   0xFFFC
#define REGFLAG_UDELAY                  0xFFFB
#define REGFLAG_END_OF_TABLE            0xFFFD

#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST		(GPIO45 | 0x80000000)
#endif

// ---------------------------------------------------------------------------
//  Local Variable
// ---------------------------------------------------------------------------
static struct LCM_UTIL_FUNCS lcm_util;


//#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
		lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[120];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
/* C60 GC7202H+HKC6.517_IPS_QM065HS05-1_点亮代码_20230217 */
{0xFF,3,{0x55,0xAA,0x66}},
{0xFE,3,{0x55,0xAA,0x66}},
{0xFF,1,{0xC3}},
{0xFB,1,{0x00}},
{0x0D,1,{0x00}},
{0x0E,1,{0x8D}},//
{0x13,1,{0x07}},//TA_GET=7

{0xFF,1,{0xB3}},
{0x2B,1,{0x0C}},
{0x29,1,{0x3F}},
{0x28,1,{0xC0}},
{0x2A,1,{0x03}},
{0x68,1,{0x0F}},

{0xFF,1,{0x10}}, 
{0xFB,1,{0x00}}, 
{0xFF,1,{0x20}},  
{0xFB,1,{0x00}},  
{0xFF,1,{0x21}},  
{0xFB,1,{0x00}},  
{0xFF,1,{0x22}},  
{0xFB,1,{0x00}},  
{0xFF,1,{0x23}},  
{0xFB,1,{0x00}},  
{0xFF,1,{0x24}},  
{0xFB,1,{0x00}}, 
{0xFF,1,{0x27}},  
{0xFB,1,{0x00}},  
{0xFF,1,{0x26}},  
{0xFB,1,{0x00}},   
{0xFF,1,{0x28}},  
{0xFB,1,{0x00}},  
{0xFF,1,{0xB3}},  
{0xFB,1,{0x00}}, 
//更新AFE CODE 区域

//红区
{0xFF,1,{0x28}}, 
{0x53,1,{0x44}}, 
{0x50,1,{0x4c}}, 
{0x52,1,{0x52}},
{0x4D,1,{0xA2}},
{0xFF,1,{0x20}}, 
{0xA5,1,{0x00}},
{0xA6,1,{0xFF}},
{0xA9,1,{0x00}},
{0xAA,1,{0xFF}},
{0xD3,1,{0x06}},
{0x2D,1,{0x1F}},
{0x2E,1,{0x42}},
{0x2F,1,{0x14}},

{0xFF,1,{0x22}}, 
{0x1f,1,{0x06}},
{0xFF,1,{0xB3}},
{0x3E,1,{0x03}},                          
{0x58,1,{0x84}},
{0x53,1,{0x1A}}, 
{0x82,1,{0x1A}},
{0xFF,1,{0xB3}},
{0x78,1,{0x01}},

{0x4A,1,{0x0F}},
{0x7D,1,{0x80}},
{0x5B,1,{0x4B}},
{0x48,1,{0x26}},
{0x7C,1,{0x8C}},
//黄区
{0xFF,1,{0x20}}, 
{0xA3,1,{0x45}}, 
{0xA7,1,{0x45}},
{0xFF,1,{0xB3}}, 
{0x3F,1,{0x37}},
{0x5E,1,{0x10}},

{0xFF,1,{0x22}}, 
{0xE4,1,{0x00}},
{0x01,1,{0x06}},
{0x02,1,{0x40}},
{0x25,1,{0x08}},
{0x26,1,{0x00}},
{0x2E,1,{0x78}},
{0x2F,1,{0x00}},
{0x36,1,{0x0D}},
{0x37,1,{0x00}},
{0x3F,1,{0x78}},
{0x40,1,{0x00}},

//{0xDC,1,{0x77}},//OSC-te
//{0xE0,1,{0x01}}, 


{0xFF,1,{0x28}}, 
{0x01,1,{0x23}}, 
{0x02,1,{0x23}},
{0x03,1,{0x19}},
{0x04,1,{0x18}}, 
{0x05,1,{0x1A}},
{0x06,1,{0x23}}, 
{0x07,1,{0x1B}}, 
{0x08,1,{0x09}}, 
{0x09,1,{0x0B}}, 
{0x0A,1,{0x0D}}, 
{0x0B,1,{0x0F}}, 
{0x0C,1,{0x11}}, 
{0x0D,1,{0x13}},
{0x0E,1,{0x05}}, 
{0x0F,1,{0x03}}, 
{0x10,1,{0x01}}, 
{0x11,1,{0x25}},
{0x12,1,{0x25}},
{0x13,1,{0x25}}, 
{0x14,1,{0x25}},
{0x15,1,{0x25}},
{0x16,1,{0x25}},
{0x17,1,{0x23}},
{0x18,1,{0x23}},
{0x19,1,{0x19}},
{0x1A,1,{0x18}},
{0x1B,1,{0x1A}}, 
{0x1C,1,{0x23}}, 
{0x1D,1,{0x1B}},
{0x1E,1,{0x08}},
{0x1F,1,{0x0A}},
{0x20,1,{0x0C}}, 
{0x21,1,{0x0E}}, 
{0x22,1,{0x10}}, 
{0x23,1,{0x12}},
{0x24,1,{0x04}},
{0x25,1,{0x02}}, 
{0x26,1,{0x00}}, 
{0x27,1,{0x25}}, 
{0x28,1,{0x25}}, 
{0x29,1,{0x25}},
{0x2A,1,{0x25}}, 
{0x2B,1,{0x25}}, 
{0x2D,1,{0x25}}, 

{0x30,1,{0x00}},
{0x31,1,{0x55}}, 
{0x32,1,{0x00}}, 
{0x33,1,{0x05}}, 
{0x34,1,{0x00}}, 
{0x35,1,{0x15}}, 
{0x36,1,{0x00}},
{0x37,1,{0x50}},
{0x38,1,{0x00}}, 
{0x39,1,{0x05}}, 
{0x2F,1,{0x0A}},

{0xFF,1,{0x21}}, 
{0x7E,1,{0x07}}, 
{0x7F,1,{0x23}},
{0x8B,1,{0x23}}, 
{0x97,1,{0x23}},
{0x80,1,{0x09}}, 
{0x8C,1,{0x05}},
{0x98,1,{0x01}},
{0xAF,1,{0x40}}, 
{0xB0,1,{0x40}},
{0xB1,1,{0x40}}, 
{0x83,1,{0x06}},
{0x8F,1,{0x06}},
{0x9B,1,{0x06}}, 
{0x84,1,{0x02}}, 
{0x90,1,{0x02}}, 
{0x9C,1,{0x02}}, 
{0x85,1,{0x8e}},
{0x91,1,{0x8e}}, 
{0x9D,1,{0x8e}},
{0x87,1,{0x07}}, 
{0x93,1,{0x21}},
{0x9F,1,{0x03}},
{0x82,1,{0xA0}}, 
{0x8E,1,{0xA0}}, 
{0x9A,1,{0xA0}},
{0x2B,1,{0x00}},
{0x2E,1,{0x00}}, 
{0x88,1,{0x80}},
{0x89,1,{0x00}},
{0x8A,1,{0x00}}, 
{0x94,1,{0x80}}, 
{0x95,1,{0x00}}, 
{0x96,1,{0x00}}, 
{0xA0,1,{0x80}}, 
{0xA1,1,{0x00}}, 
{0xA2,1,{0x00}},
{0x45,1,{0x3F}}, 
{0x46,1,{0xA6}},
{0x4C,1,{0xA6}}, 
{0x52,1,{0xA6}},
{0x58,1,{0xA6}},
{0x5E,1,{0xA6}},
{0x64,1,{0xA6}},
{0x47,1,{0x0c}},
{0x4D,1,{0x0b}}, 
{0x53,1,{0x0a}},
{0x59,1,{0x09}}, 
{0x5F,1,{0x08}}, 
{0x65,1,{0x07}}, 
{0x76,1,{0x40}},
{0x77,1,{0x40}},
{0x78,1,{0x40}}, 
{0x79,1,{0x40}}, 
{0x7A,1,{0x40}}, 
{0x7B,1,{0x40}}, 
{0x49,1,{0x02}},
{0x4A,1,{0x06}}, 
{0x4F,1,{0x02}},
{0x50,1,{0x06}},
{0x55,1,{0x02}}, 
{0x56,1,{0x06}},
{0x5B,1,{0x02}}, 
{0x5C,1,{0x06}},
{0x61,1,{0x02}},
{0x62,1,{0x06}}, 
{0x67,1,{0x02}}, 
{0x68,1,{0x06}},
{0xBE,1,{0x03}}, 
{0xC0,1,{0x40}}, 
{0xC1,1,{0x44}},
{0xBF,1,{0xAA}}, 
{0xC2,1,{0x8A}},
{0xC3,1,{0x8A}}, 


{0xC6,1,{0x54}}, 
{0xC7,1,{0x54}}, 
{0x29,1,{0x00}},
{0xFF,1,{0x22}}, 
{0x05,1,{0x00}}, 
{0x08,1,{0x22}}, 

{0xFF,1,{0x20}}, 	
{0xC3,1,{0x00}}, 
{0xC4,1,{0x91}}, 
{0xC5,1,{0x00}},
{0xC6,1,{0x91}},  
{0xB3,1,{0x00}},
{0xB4,1,{0x20}},
{0xB5,1,{0x01}},
{0xB6,1,{0x5E}},
//绿区

{0xFF,1,{0x28}}, 
{0x3D,1,{0x60}},
{0x3E,1,{0x60}},  
{0x4D,1,{0xA3}}, 
{0x3F,1,{0x39}}, 
{0x40,1,{0x39}}, 
{0x45,1,{0x45}},
{0x46,1,{0x45}}, 
{0x47,1,{0x43}},
{0x48,1,{0x43}}, 
{0x5A,1,{0x99}},
{0x5B,1,{0x9F}},
//{0x62,1,{0xd1}}, //VCOM
//{0x63,1,{0xd1}}, 


{0xFF,1,{0x20}}, 
{0x7E,1,{0x01}},
{0x7F,1,{0x00}},
{0x80,1,{0x64}},
{0x81,1,{0x00}},
{0x82,1,{0x00}},
{0x83,1,{0x64}},
{0x84,1,{0x64}},
{0x85,1,{0x39}},
{0x86,1,{0x37}},
{0x87,1,{0x45}},
{0x88,1,{0x7B}},
{0x8A,1,{0x0A}},
{0x8B,1,{0x0A}},


{0xFF,1,{0x23}},
{0x29,1,{0x03}},

{0x01,16,{0x00,0x00,0x00,0x2B,0x00,0x4C,0x00,0x64,0x00,0x7B,0x00,0x91,0x00,0x9F,0x00,0xAF}},
{0x02,16,{0x00,0xBD,0x00,0xE8,0x01,0x0B,0x01,0x44,0x01,0x6D,0x01,0xAF,0x01,0xEB,0x01,0xED}},
{0x03,16,{0x02,0x3C,0x02,0x78,0x02,0xAA,0x02,0xEA,0x03,0x13,0x03,0x48,0x03,0x58,0x03,0x6A}},
{0x04,12,{0x03,0x7D,0x03,0x92,0x03,0xAB,0x03,0xC8,0x03,0xE8,0x03,0xFF}},


{0x0D,16,{0x00,0x00,0x00,0x2B,0x00,0x4C,0x00,0x64,0x00,0x7B,0x00,0x91,0x00,0x9F,0x00,0xAF}},
{0x0E,16,{0x00,0xBD,0x00,0xE8,0x01,0x0B,0x01,0x44,0x01,0x6D,0x01,0xAF,0x01,0xEB,0x01,0xED}},
{0x0F,16,{0x02,0x1C,0x02,0x78,0x02,0xAA,0x02,0xEA,0x03,0x13,0x03,0x48,0x03,0x58,0x03,0x6A}},
{0x10,12,{0x03,0x7D,0x03,0x92,0x03,0xAB,0x03,0xC8,0x03,0xE8,0x03,0xFF}},


{0x2D,1,{0x65}},
{0x2E,1,{0x00}},

{0x32,1,{0x02}},
{0x33,1,{0x18}},


{0xFF,1,{0x10}},                                                                                                                                                                                                 
{0x35,1,{0x00}},
{0x53,1,{0x2c}},
{0x51,2,{0x00,0x00}},                                                                                                                                
{0x36,1,{0x08}},                                                                         
{0x69,1,{0x00}},
{0x71,2,{0x12,0x08}},//OSC
{0xFF,1,{0x24}},
{0x7D,1,{0x55}},
{0xFF,3,{0x66,0x99,0x55}},
//Wait_Key();
{0xFF,1,{0x10}}, 

{0x11,1,{0x00}},
{REGFLAG_DELAY,120,{}},
{0x29,1,{0x00}},
{0xFE,3,{0x66,0x99,0x55}},
{REGFLAG_DELAY,10,{}},

{REGFLAG_END_OF_TABLE, 0x00, {}},
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {

{0xFF,3,{0x55,0xAA,0x66}},
{0xFE,3,{0x55,0xAA,0x66}},
{0xFF,1,{0xB3}},
{0x68,1,{0x00}},
{0x2A,1,{0x00}},
{0x28,1,{0x00}},
{0x29,1,{0x00}},
{0x2B,1,{0x00}},
{0xFF,1,{0x20}},
{0x4A,1,{0x01}},
{0x48,1,{0x10}},
{0x49,1,{0x00}},
{0xFF,1,{0x28}},
{0x2F,1,{0x0D}},
{0xFF,1,{0x10}},
{0x28,1,{0x00}},
{0xFF,1,{0x22}}, 
{0xE4,1,{0x00}},  // fae modify close tp scan while lcm is off
{REGFLAG_DELAY,50,{}},


{0xFF,1,{0x10}},
{0x10,1,{0x00}},
{REGFLAG_DELAY,120,{}},
{0xFF,1,{0x26}},
{0x1D,2,{0x8A,0x80}},
{0xFF,1,{0xB3}},
{0x04,1,{0x16}},
{0xFF,1,{0xB3}},
{0x01,1,{0x90}},
{0xFF,1,{0x26}},
{0x1F,1,{0x01}},
{0xFE,3,{0x66,0x99,0x55}},
{REGFLAG_DELAY,20,{}},
{REGFLAG_END_OF_TABLE, 0x00, {}},
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	for(i = 0; i < count; i++) {
		unsigned int cmd;
		cmd = table[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY :
				MDELAY(table[i].count);
				break;
			case REGFLAG_END_OF_TABLE :
				break;
			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

//	params->virtual_width = VIRTUAL_WIDTH;
//	params->virtual_height = VIRTUAL_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode            = CMD_MODE;
#else
	params->dsi.mode            = SYNC_PULSE_VDO_MODE;
#endif

	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability */
	/* video mode timing */
	params->dsi.PS                                  = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active                = 6;
	params->dsi.vertical_backporch                  = 24; // 28;
	params->dsi.vertical_frontporch                 = 330;//140;
	params->dsi.vertical_active_line                = FRAME_HEIGHT;
	
	params->dsi.horizontal_sync_active              = 4;
	params->dsi.horizontal_backporch                = 80;
	params->dsi.horizontal_frontporch               = 80;
	params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
	params->dsi.LANE_NUM                            = LCM_FOUR_LANE;


	params->dsi.PLL_CLOCK                           = 330; // 300;//250

    //prize-tangcong modify LCD size-20200331-start
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	//prize-tangcong modify LCD size-20200331-end
	params->dsi.ssc_disable                         = 1;
	params->dsi.ssc_range                           = 4;

	params->dsi.HS_TRAIL                            = 15;
	params->dsi.noncont_clock                       = 1;
	params->dsi.noncont_clock_period                = 1;
	params->dsi.LPX             					= 5;			/* prize and fae add */

#if 1
	/* ESD check function */
	params->dsi.esd_check_enable                    = 1;
	params->dsi.customization_esd_check_enable      = 1;
	//params->dsi.clk_lp_per_line_enable              = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->full_content = 0;
	params->corner_pattern_width = 1080;
	params->corner_pattern_height = 32;
	params->corner_pattern_height_bot = 32;
#endif
}

static void lcm_init_power(void)
{
	//display_bias_enable();
}

static void lcm_suspend_power(void)
{
	//display_bias_disable();
}

static void lcm_resume_power(void)
{
	//SET_RESET_PIN(0);
	//display_bias_enable();
}
static int gcore_lcm_status=0;  /* lcm 0: normal, 1 suspend -> init, 2 reset -> tp download fw done */
int gc7202h_lcm_status(void)
{
  printk("%s gcore_lcm_status=%d",__func__,gcore_lcm_status);
  return gcore_lcm_status; /* 0 run, 1 suspend, 2 just reset */
}
EXPORT_SYMBOL(gc7202h_lcm_status);
void gc7202h_lcm_stset(int status)
{
	gcore_lcm_status = status;
    printk("%s gcore_lcm_status=%d",__func__,gcore_lcm_status);
}
EXPORT_SYMBOL(gc7202h_lcm_stset);
static void lcm_init(void)
{
    printk("%s gcore_lcm_status=%d",__func__,gcore_lcm_status);

	display_ldo18_enable(1);
	display_bias_enable_v(6000);
	MDELAY(10);
	
	gcore_lcm_status=2;
/* galaxycore FAE requires reset 4 times, this IC needs to be reset more fully start */
    mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	MDELAY(5);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 0);
	MDELAY(5);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	MDELAY(5);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 0);
	MDELAY(5);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	MDELAY(5);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 0);
	MDELAY(5);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	MDELAY(5);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 0);
	MDELAY(5);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	MDELAY(30);
/* galaxycore FAE requires reset 4 times, this IC needs to be reset more fully end */

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	//SET_RESET_PIN(1);
    MDELAY(10);
	display_bias_disable();
	MDELAY(10);
	display_ldo18_enable(0);
    printk("%s gcore_lcm_status=%d",__func__,gcore_lcm_status);
    gcore_lcm_status=1;
}

static void lcm_resume(void)
{
	lcm_init();
}


static struct LCM_setting_table switch_table[] = {

{0xFF,3,{0x55,0xAA,0x66}},
{0xFF,1,{0xC3}},
{0xFB,1,{0x00}},
{0x0D,1,{0x00}},
{0x0E,1,{0x8D}},//
{0x13,1,{0x07}},//TA_GET=7
{0xFF,1,{0x10}},

};


static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[2];
	unsigned int array[16];

    mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	//SET_RESET_PIN(1);
	MDELAY(5);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 0);
	//SET_RESET_PIN(0);
	MDELAY(5);
   mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	//SET_RESET_PIN(1);
	MDELAY(30);

	push_table(switch_table, sizeof(switch_table) / sizeof(struct LCM_setting_table), 1);



	array[0] = 0x00023700;	/* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, buffer, 2);
	id = (buffer[0] << 8) + buffer[1];     /* we only need ID */

	LCM_LOGI("%s,nl9911_id=0x%x\n", __func__, id);

	if (id == 0x9911)
		return 1;
	else
		return 0;

}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
#if 0
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];
	LCM_LOGI("ATA check size = 0x%x,0x%x,0x%x,0x%x\n", x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	
	data_array[0] = 0x0003390A; /* HS packet */
	data_array[1] = 0x00595AF0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0003390A; /* HS packet */
	data_array[1] = 0x00A6A5F1;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x0004390A; /* HS packet */
	data_array[1] = (x1_LSB << 24) | (x1_MSB << 16) | (x0_LSB << 8) | 0xB9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00033700; /* read id return two byte,version and id */
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0x04, read_buf, 3);
	printk("%s read[0x04]= %x  %x %x %x\n", __func__, read_buf[0], read_buf[1],read_buf[2],read_buf[3]);

//	MDELAY(10);
	data_array[0] = 0x00033700; /* read id return two byte,version and id */
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0xB9, read_buf, 3);
	printk("%s read[0xB9]= %x %x %x %x \n", __func__, read_buf[0],read_buf[1],read_buf[2],read_buf[3]);

	if ((read_buf[0] == x0_LSB) && (read_buf[1] == x1_MSB)
	        && (read_buf[2] == x1_LSB))
		ret = 1;
	else
		ret = 0;


	printk("%s ret= %x\n", __func__, ret);
	return ret;
#endif
    unsigned char read_buf[2];
    unsigned int array[16];

	array[0] = 0x00023700;	/* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1); 

	read_reg_v2(0xDA, read_buf, 1); 

	printk("%s,[galaxycore]get gecore_lcm_id is 0x%x.\n", __func__,read_buf[0]);

	if (read_buf[0] == 0x20)
	 return 1;
	else
	 return 0;

#else
	return 0;
#endif
}

struct LCM_DRIVER gc7202h_hdplus_dsi_vdo_incell_lcm_drv =
{
	.name		= "gc7202h_hdplus_dsi_vdo_incell",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "gc7202h",
		.vendor	= "galaxy",
		.id = "0x7202",
		.more	= "720*1600",
	},
   #endif
	.set_util_funcs	= lcm_set_util_funcs,
	.get_params	= lcm_get_params,
	.init		= lcm_init,
	.suspend	= lcm_suspend,
	.resume		= lcm_resume,
	.compare_id	= lcm_compare_id,
	.init_power	= lcm_init_power,
	.ata_check	= lcm_ata_check,
#ifndef BUILD_LK
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
#endif
};
