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
#else
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

//prize-add tp enter low power mode-pengzhipeng-20190109-start
//extern void fts_enter_low_power(void);
//int fts_reset_proc(int hdelayms);
//prize-add tp enter low power mode-pengzhipeng-20190109-end

static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)			(lcm_util.set_reset_pin((v)))
#define MDELAY(n)					(lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) \
	lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define set_gpio_lcd_enp(cmd) \
		lcm_util.set_gpio_lcd_enp_bias(cmd)
		

static const unsigned char LCD_MODULE_ID = 0x01;
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH  										 (1080)
#define FRAME_HEIGHT 										 (1920)

#define REGFLAG_DELAY             							 0xFFFA
#define REGFLAG_UDELAY             							 0xFFFB
#define REGFLAG_PORT_SWAP									 0xFFFC
#define REGFLAG_END_OF_TABLE      							 0xFFFD   // END OF REGISTERS MARKER

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

struct LCM_setting_table
{
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] =
{
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 60, {} },
	//prize-add tp enter low power mode-pengzhipeng-20190109-start
	{0x04,1,{0x5a}},
    {0x05,1,{0x5a}},
	{REGFLAG_DELAY, 10, {} },
    {REGFLAG_END_OF_TABLE, 0x00, {}}
    //prize-add tp enter low power mode-pengzhipeng-20190109-start

};

static struct LCM_setting_table lcm_initialization_setting[] =
{

	{0xFF,3,{0x78,0x07,0x1}},   	//Page1
	{0x42,1,{0x11}},   	//VGH=x4 VGL=x4
	{0x43,1,{0x94}},   	//VGH_CLP = 8.0V
	{0x44,1,{0x9E}},   	//VGL_CLP = -8.0V
	{0x45,1,{0x14}},   	//VGHO = 8.0V
	{0x46,1,{0x28}},   	//VGLO = -8.0V
	{0x4A,1,{0x15}},   	//VSPR short to VSP
	{0x4B,1,{0x15}},   	//VSNR short to VSN
	{0x50,1,{0x41}},   	//GVDDP = 4.3V
	{0x51,1,{0x41}},   	//GVDDN = - 4.3V	
	{0xA2,1,{0x1}},   	
	{0xA3,1,{0x12}},   	//VCOM1 = - 0.19V

	//GIP & SOURCE
	{0x22,1,{0x06}},   	//SS&NW(絋粄) FORSCAN
	{0x36,1,{0x00}},        // 2 to 6 
	{0x64,1,{0x08}},   	//EQT
	{0x65,1,{0x03}},        //EQT2
	{0x66,1,{0x04}},   	//PCT
	{0x6C,1,{0x45}},   	//PRC & PRCB
	{0x6D,1,{0x08}},   	//PCT2
	{0x5A,1,{0x33}},        //LVD setting  
	{0xFF,3,{0x78,0x07,0x06}},   	//Page6 GOA 
	{0x00,1,{0x42}},   
	{0x1,1,{0x05}},   
	{0x02,1,{0x00}},   
	{0x03,1,{0x00}},   
	{0x04,1,{0x00}},   
	{0x05,1,{0x00}},   
	{0x06,1,{0x00}},   
	{0x07,1,{0x00}},   
	{0x08,1,{0x81}},   
	{0x09,1,{0x02}},   
	{0x0A,1,{0x30}},   
	{0x0B,1,{0x00}},   
	{0x0C,1,{0x02}},   
	{0x0D,1,{0x02}},   
	{0x0E,1,{0x02}},   
	{0x0F,1,{0x02}},   
	{0x10,1,{0x00}},   
	{0x11,1,{0x00}},   
	{0x12,1,{0x00}},   
	{0x13,1,{0x00}}, 
	{0x14,1,{0x00}},      //CLK_A keep position1<JH change add>   
	{0x15,1,{0x00}},      //CLK_A keep position2<JH change add>   

	{0x31,1,{0x07}},   
	{0x32,1,{0x08}},   
	{0x33,1,{0x1}},   
	{0x34,1,{0x00}},   
	{0x35,1,{0x07}},   
	{0x36,1,{0x10}},   
	{0x37,1,{0x22}},   
	{0x38,1,{0x12}},   
	{0x39,1,{0x06}},   
	{0x3A,1,{0x07}},   
	{0x3B,1,{0x07}},   
	{0x3C,1,{0x07}},   
	{0x3D,1,{0x07}},   
	{0x3E,1,{0x2a}},   
	{0x3F,1,{0x29}},   
	{0x40,1,{0x28}},   
	{0x41,1,{0x07}},   
	{0x42,1,{0x09}},   
	{0x43,1,{0x1}},   
	{0x44,1,{0x00}},   
	{0x45,1,{0x07}},   
	{0x46,1,{0x11}},   
	{0x47,1,{0x22}},   
	{0x48,1,{0x13}},   
	{0x49,1,{0x06}},   
	{0x4A,1,{0x07}},   
	{0x4B,1,{0x07}},   
	{0x4C,1,{0x07}},   
	{0x4D,1,{0x07}},   
	{0x4E,1,{0x2a}},   
	{0x4F,1,{0x29}},   
	{0x50,1,{0x28}},   

	{0x61,1,{0x07}},   
	{0x62,1,{0x09}},   
	{0x63,1,{0x1}},   
	{0x64,1,{0x00}},   
	{0x65,1,{0x07}},   
	{0x66,1,{0x13}},   
	{0x67,1,{0x22}},   
	{0x68,1,{0x11}},   
	{0x69,1,{0x06}},   
	{0x6A,1,{0x07}},   
	{0x6B,1,{0x07}},   
	{0x6C,1,{0x07}},   
	{0x6D,1,{0x07}},   
	{0x6E,1,{0x2a}},   
	{0x6F,1,{0x29}},   
	{0x70,1,{0x28}},   
	{0x71,1,{0x07}},   
	{0x72,1,{0x08}},   
	{0x73,1,{0x1}},   
	{0x74,1,{0x00}},   
	{0x75,1,{0x07}},   
	{0x76,1,{0x12}},   
	{0x77,1,{0x22}},   
	{0x78,1,{0x10}},   
	{0x79,1,{0x06}},   
	{0x7A,1,{0x07}},   
	{0x7B,1,{0x07}},   
	{0x7C,1,{0x07}},   
	{0x7D,1,{0x07}},   
	{0x7E,1,{0x2a}},   
	{0x7F,1,{0x29}},   
	{0x80,1,{0x28}},   
	{0x97,1,{0x11}},      //Frame blanking setting JH check add

	{0xD0,1,{0x1}},      
	{0xD1,1,{0x00}},      // JH change add
	{0xDB,1,{0x47}},      // JH change add 2
	{0xDC,1,{0x02}},      // JH change add 2
	{0xDD,1,{0x00}},      // JH change add
	{0xE5,1,{0x03}},      // JH change add
	{0xA0,1,{0x06}},	//MUX-CLK 间距  10=880ns   08=640ns  
	{0xA1,1,{0x00}},   
	{0xA2,1,{0x0B}},	//mux-MUX 间距   
	{0xA3,1,{0x1F}},	//mux宽度  1C=1.72US    20=1.96us 
	{0xA6,1,{0x22}},   
	{0xA7,1,{0x03}},   //RGBRGB  00  	RGBBGR  03
	{0xAE,1,{0x14}},   //RGBRGB  default	RGBBGR  14

	//===CKH Modulation====
	{0xB1,1,{0x00}},   
	{0xB2,1,{0x44}},   
	{0xB3,1,{0x44}},   
	{0xB4,1,{0x00}},   
	{0xB5,1,{0x00}},   
	{0xB6,1,{0x00}},   

	//============Gamma START=============//
	{0xFF,3,{0x78,0x07,0x02}},
	{0x00,1,{0x00}},    //255
	{0x1,1,{0x00}},    //255
	{0x02,1,{0x00}},    //254
	{0x03,1,{0x0D}},    //254
	{0x04,1,{0x00}},    //252
	{0x05,1,{0x26}},    //252
	{0x06,1,{0x00}},    //250
	{0x07,1,{0x3D}},    //250
	{0x08,1,{0x00}},    //248
	{0x09,1,{0x51}},    //248
	{0x0A,1,{0x00}},    //246
	{0x0B,1,{0x63}},    //246
	{0x0C,1,{0x00}},    //244
	{0x0D,1,{0x74}},    //244
	{0x0E,1,{0x00}},    //242
	{0x0F,1,{0x83}},    //242
	{0x10,1,{0x00}},    //240
	{0x11,1,{0x92}},    //240
	{0x12,1,{0x00}},    //232
	{0x13,1,{0xC2}},    //232
	{0x14,1,{0x00}},    //224
	{0x15,1,{0xE9}},    //224
	{0x16,1,{0x1}},    //208
	{0x17,1,{0x26}},    //208
	{0x18,1,{0x1}},    //192
	{0x19,1,{0x56}},    //192
	{0x1A,1,{0x1}},    //160
	{0x1B,1,{0x9E}},    //160
	{0x1C,1,{0x1}},    //128
	{0x1D,1,{0xD9}},    //128
	{0x1E,1,{0x1}},    //127
	{0x1F,1,{0xDB}},    //127
	{0x20,1,{0x02}},    //95
	{0x21,1,{0x15}},    //95
	{0x22,1,{0x02}},    //63
	{0x23,1,{0x5A}},    //63
	{0x24,1,{0x02}},    //47
	{0x25,1,{0x88}},    //47
	{0x26,1,{0x02}},    //31
	{0x27,1,{0xC5}},    //31
	{0x28,1,{0x02}},    //23
	{0x29,1,{0xED}},    //23
	{0x2A,1,{0x03}},    //15
	{0x2B,1,{0x1E}},    //15
	{0x2C,1,{0x03}},    //13
	{0x2D,1,{0x2B}},    //13
	{0x2E,1,{0x03}},    //11
	{0x2F,1,{0x3C}},    //11
	{0x30,1,{0x03}},    //9
	{0x31,1,{0x4F}},    //9
	{0x32,1,{0x03}},    //7
	{0x33,1,{0x65}},    //7
	{0x34,1,{0x03}},    //5
	{0x35,1,{0x7E}},    //5
	{0x36,1,{0x03}},    //3
	{0x37,1,{0xA3}},    //3
	{0x38,1,{0x03}},    //1
	{0x39,1,{0xD0}},    //1
	{0x3A,1,{0x03}},    //0
	{0x3B,1,{0xE6}},    //0
	{0x3C,1,{0x00}},    //255
	{0x3D,1,{0x00}},    //255
	{0x3E,1,{0x00}},    //254
	{0x3F,1,{0x0D}},    //254
	{0x40,1,{0x00}},    //252
	{0x41,1,{0x26}},    //252
	{0x42,1,{0x00}},    //250
	{0x43,1,{0x3D}},    //250
	{0x44,1,{0x00}},    //248
	{0x45,1,{0x51}},    //248
	{0x46,1,{0x00}},    //246
	{0x47,1,{0x63}},    //246
	{0x48,1,{0x00}},    //244
	{0x49,1,{0x74}},    //244
	{0x4A,1,{0x00}},    //242
	{0x4B,1,{0x83}},    //242
	{0x4C,1,{0x00}},    //240
	{0x4D,1,{0x92}},    //240
	{0x4E,1,{0x00}},    //232
	{0x4F,1,{0xC2}},    //232
	{0x50,1,{0x00}},    //224
	{0x51,1,{0xE9}},    //224
	{0x52,1,{0x1}},    //208
	{0x53,1,{0x26}},    //208
	{0x54,1,{0x1}},    //192
	{0x55,1,{0x56}},    //192
	{0x56,1,{0x1}},    //160
	{0x57,1,{0x9E}},    //160
	{0x58,1,{0x1}},    //128
	{0x59,1,{0xD9}},    //128
	{0x5A,1,{0x1}},    //127
	{0x5B,1,{0xDB}},    //127
	{0x5C,1,{0x02}},    //95
	{0x5D,1,{0x15}},    //95
	{0x5E,1,{0x02}},    //63
	{0x5F,1,{0x5A}},    //63
	{0x60,1,{0x02}},    //47
	{0x61,1,{0x88}},    //47
	{0x62,1,{0x02}},    //31
	{0x63,1,{0xC5}},    //31
	{0x64,1,{0x02}},    //23
	{0x65,1,{0xED}},    //23
	{0x66,1,{0x03}},    //15
	{0x67,1,{0x1E}},    //15
	{0x68,1,{0x03}},    //13
	{0x69,1,{0x2B}},    //13
	{0x6A,1,{0x03}},    //11
	{0x6B,1,{0x3C}},    //11
	{0x6C,1,{0x03}},    //9
	{0x6D,1,{0x4F}},    //9
	{0x6E,1,{0x03}},    //7
	{0x6F,1,{0x65}},    //7
	{0x70,1,{0x03}},    //5
	{0x71,1,{0x7E}},    //5
	{0x72,1,{0x03}},    //3
	{0x73,1,{0xA3}},    //3
	{0x74,1,{0x03}},    //1
	{0x75,1,{0xD0}},    //1
	{0x76,1,{0x03}},    //0
	{0x77,1,{0xE6}},    //0
	{0x78,1,{0x1}},
	{0x79,1,{0x1}},
	{0xFF,3,{0x78,0x07,0x03}},
	{0x00,1,{0x00}},    //255
	{0x1,1,{0x00}},    //255
	{0x02,1,{0x00}},    //254
	{0x03,1,{0x0D}},    //254
	{0x04,1,{0x00}},    //252
	{0x05,1,{0x25}},    //252
	{0x06,1,{0x00}},    //250
	{0x07,1,{0x3B}},    //250
	{0x08,1,{0x00}},    //248
	{0x09,1,{0x4E}},    //248
	{0x0A,1,{0x00}},    //246
	{0x0B,1,{0x60}},    //246
	{0x0C,1,{0x00}},    //244
	{0x0D,1,{0x71}},    //244
	{0x0E,1,{0x00}},    //242
	{0x0F,1,{0x80}},    //242
	{0x10,1,{0x00}},    //240
	{0x11,1,{0x8E}},    //240
	{0x12,1,{0x00}},    //232
	{0x13,1,{0xBE}},    //232
	{0x14,1,{0x00}},    //224
	{0x15,1,{0xE5}},    //224
	{0x16,1,{0x1}},    //208
	{0x17,1,{0x22}},    //208
	{0x18,1,{0x1}},    //192
	{0x19,1,{0x51}},    //192
	{0x1A,1,{0x1}},    //160
	{0x1B,1,{0x99}},    //160
	{0x1C,1,{0x1}},    //128
	{0x1D,1,{0xD5}},    //128
	{0x1E,1,{0x1}},    //127
	{0x1F,1,{0xD7}},    //127
	{0x20,1,{0x02}},    //95
	{0x21,1,{0x11}},    //95
	{0x22,1,{0x02}},    //63
	{0x23,1,{0x57}},    //63
	{0x24,1,{0x02}},    //47
	{0x25,1,{0x85}},    //47
	{0x26,1,{0x02}},    //31
	{0x27,1,{0xC3}},    //31
	{0x28,1,{0x02}},    //23
	{0x29,1,{0xEB}},    //23
	{0x2A,1,{0x03}},    //15
	{0x2B,1,{0x1E}},    //15
	{0x2C,1,{0x03}},    //13
	{0x2D,1,{0x2C}},    //13
	{0x2E,1,{0x03}},    //11
	{0x2F,1,{0x3D}},    //11
	{0x30,1,{0x03}},    //9
	{0x31,1,{0x4F}},    //9
	{0x32,1,{0x03}},    //7
	{0x33,1,{0x63}},    //7
	{0x34,1,{0x03}},    //5
	{0x35,1,{0x7A}},    //5
	{0x36,1,{0x03}},    //3
	{0x37,1,{0x9F}},    //3
	{0x38,1,{0x03}},    //1
	{0x39,1,{0xCE}},    //1
	{0x3A,1,{0x03}},    //0
	{0x3B,1,{0xE6}},    //0　
	{0x3C,1,{0x00}},    //255
	{0x3D,1,{0x00}},    //255
	{0x3E,1,{0x00}},    //254
	{0x3F,1,{0x0D}},    //254
	{0x40,1,{0x00}},    //252
	{0x41,1,{0x25}},    //252
	{0x42,1,{0x00}},    //250
	{0x43,1,{0x3B}},    //250
	{0x44,1,{0x00}},    //248
	{0x45,1,{0x4E}},    //248
	{0x46,1,{0x00}},    //246
	{0x47,1,{0x60}},    //246
	{0x48,1,{0x00}},    //244
	{0x49,1,{0x71}},    //244
	{0x4A,1,{0x00}},    //242
	{0x4B,1,{0x80}},    //242
	{0x4C,1,{0x00}},    //240
	{0x4D,1,{0x8E}},    //240
	{0x4E,1,{0x00}},    //232
	{0x4F,1,{0xBE}},    //232
	{0x50,1,{0x00}},    //224
	{0x51,1,{0xE5}},    //224
	{0x52,1,{0x1}},    //208
	{0x53,1,{0x22}},    //208
	{0x54,1,{0x1}},    //192
	{0x55,1,{0x51}},    //192
	{0x56,1,{0x1}},    //160
	{0x57,1,{0x99}},    //160
	{0x58,1,{0x1}},    //128
	{0x59,1,{0xD5}},    //128
	{0x5A,1,{0x1}},    //127
	{0x5B,1,{0xD7}},    //127
	{0x5C,1,{0x02}},    //95
	{0x5D,1,{0x11}},    //95
	{0x5E,1,{0x02}},    //63
	{0x5F,1,{0x57}},    //63
	{0x60,1,{0x02}},    //47
	{0x61,1,{0x85}},    //47
	{0x62,1,{0x02}},    //31
	{0x63,1,{0xC3}},    //31
	{0x64,1,{0x02}},    //23
	{0x65,1,{0xEB}},    //23
	{0x66,1,{0x03}},    //15
	{0x67,1,{0x1E}},    //15
	{0x68,1,{0x03}},    //13
	{0x69,1,{0x2C}},    //13
	{0x6A,1,{0x03}},    //11
	{0x6B,1,{0x3D}},    //11
	{0x6C,1,{0x03}},    //9
	{0x6D,1,{0x4F}},    //9
	{0x6E,1,{0x03}},    //7
	{0x6F,1,{0x63}},    //7
	{0x70,1,{0x03}},    //5
	{0x71,1,{0x7A}},    //5
	{0x72,1,{0x03}},    //3
	{0x73,1,{0x9F}},    //3
	{0x74,1,{0x03}},    //1
	{0x75,1,{0xCE}},    //1
	{0x76,1,{0x03}},    //0
	{0x77,1,{0xE6}},    //0
	{0x78,1,{0x1}},
	{0x79,1,{0x1}},
	{0xFF,3,{0x78,07,04}},
	{0x00,1,{0x00}},    //255
	{0x1,1,{0x00}},    //255
	{0x02,1,{0x00}},    //254
	{0x03,1,{0x0E}},    //254
	{0x04,1,{0x00}},    //252
	{0x05,1,{0x29}},    //252
	{0x06,1,{0x00}},    //250
	{0x07,1,{0x41}},    //250
	{0x08,1,{0x00}},    //248
	{0x09,1,{0x54}},    //248
	{0x0A,1,{0x00}},    //246
	{0x0B,1,{0x67}},    //246
	{0x0C,1,{0x00}},    //244
	{0x0D,1,{0x78}},    //244
	{0x0E,1,{0x00}},    //242
	{0x0F,1,{0x88}},    //242
	{0x10,1,{0x00}},    //240
	{0x11,1,{0x96}},    //240
	{0x12,1,{0x00}},    //232
	{0x13,1,{0xC6}},    //232
	{0x14,1,{0x00}},    //224
	{0x15,1,{0xEC}},    //224
	{0x16,1,{0x1}},    //208
	{0x17,1,{0x28}},    //208
	{0x18,1,{0x1}},    //192
	{0x19,1,{0x56}},    //192
	{0x1A,1,{0x1}},    //160
	{0x1B,1,{0x9D}},    //160
	{0x1C,1,{0x1}},    //128
	{0x1D,1,{0xD8}},    //128
	{0x1E,1,{0x1}},    //127
	{0x1F,1,{0xD9}},    //127
	{0x20,1,{0x02}},    //95
	{0x21,1,{0x13}},    //95
	{0x22,1,{0x02}},    //63
	{0x23,1,{0x58}},    //63
	{0x24,1,{0x02}},    //47
	{0x25,1,{0x88}},    //47
	{0x26,1,{0x02}},    //31
	{0x27,1,{0xC7}},    //31
	{0x28,1,{0x02}},    //23
	{0x29,1,{0xF2}},    //23
	{0x2A,1,{0x03}},    //15
	{0x2B,1,{0x2E}},    //15
	{0x2C,1,{0x03}},    //13
	{0x2D,1,{0x43}},    //13
	{0x2E,1,{0x03}},    //11
	{0x2F,1,{0x51}},    //11
	{0x30,1,{0x03}},    //9
	{0x31,1,{0x60}},    //9
	{0x32,1,{0x03}},    //7
	{0x33,1,{0x71}},    //7
	{0x34,1,{0x03}},    //5
	{0x35,1,{0x85}},    //5
	{0x36,1,{0x03}},    //3
	{0x37,1,{0xA4}},    //3
	{0x38,1,{0x03}},    //1
	{0x39,1,{0xD0}},    //1
	{0x3A,1,{0x03}},    //0
	{0x3B,1,{0xE6}},    //0
	{0x3C,1,{0x00}},    //255
	{0x3D,1,{0x00}},    //255
	{0x3E,1,{0x00}},    //254
	{0x3F,1,{0x0E}},    //254
	{0x40,1,{0x00}},    //252
	{0x41,1,{0x29}},    //252
	{0x42,1,{0x00}},    //250
	{0x43,1,{0x41}},    //250
	{0x44,1,{0x00}},    //248
	{0x45,1,{0x54}},    //248
	{0x46,1,{0x00}},    //246
	{0x47,1,{0x67}},    //246
	{0x48,1,{0x00}},    //244
	{0x49,1,{0x78}},    //244
	{0x4A,1,{0x00}},    //242
	{0x4B,1,{0x88}},    //242
	{0x4C,1,{0x00}},    //240
	{0x4D,1,{0x96}},    //240
	{0x4E,1,{0x00}},    //232
	{0x4F,1,{0xC6}},    //232
	{0x50,1,{0x00}},    //224
	{0x51,1,{0xEC}},    //224
	{0x52,1,{0x1}},    //208
	{0x53,1,{0x28}},    //208
	{0x54,1,{0x1}},    //192
	{0x55,1,{0x56}},    //192
	{0x56,1,{0x1}},    //160
	{0x57,1,{0x9D}},    //160
	{0x58,1,{0x1}},    //128
	{0x59,1,{0xD8}},    //128
	{0x5A,1,{0x1}},    //127
	{0x5B,1,{0xD9}},    //127
	{0x5C,1,{0x02}},    //95
	{0x5D,1,{0x13}},    //95
	{0x5E,1,{0x02}},    //63
	{0x5F,1,{0x58}},    //63
	{0x60,1,{0x02}},    //47
	{0x61,1,{0x88}},    //47
	{0x62,1,{0x02}},    //31
	{0x63,1,{0xC7}},    //31
	{0x64,1,{0x02}},    //23
	{0x65,1,{0xF2}},    //23
	{0x66,1,{0x03}},    //15
	{0x67,1,{0x2E}},    //15
	{0x68,1,{0x03}},    //13
	{0x69,1,{0x43}},    //13
	{0x6A,1,{0x03}},    //11
	{0x6B,1,{0x51}},    //11
	{0x6C,1,{0x03}},    //9
	{0x6D,1,{0x60}},    //9
	{0x6E,1,{0x03}},    //7
	{0x6F,1,{0x71}},    //7
	{0x70,1,{0x03}},    //5
	{0x71,1,{0x85}},    //5
	{0x72,1,{0x03}},    //3
	{0x73,1,{0xA4}},    //3
	{0x74,1,{0x03}},    //1
	{0x75,1,{0xD0}},    //1
	{0x76,1,{0x03}},    //0
	{0x77,1,{0xE6}},    //0
	{0x78,1,{0x1}},
	{0x79,1,{0x1}},
	//============Gamma END=============
	{0xFF,3,{0x78,0x07,0x00}},	//Page0
	{REGFLAG_DELAY,10,{}},	
	{0x11,1,{0x00}},// Sleep-Out
	{REGFLAG_DELAY,100,{}},	
	{0x29,1,{0x00}},// Display On
	{REGFLAG_DELAY,10,{}},	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd)
        {
        case REGFLAG_DELAY:
            if (table[i].count <= 10)
                MDELAY(table[i].count);
            else
                MDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE:
            break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type                         = LCM_TYPE_DSI;
	params->width                        = FRAME_WIDTH;
	params->height                       = FRAME_HEIGHT;

	#ifndef BUILD_LK
	params->physical_width               = 64;
	params->physical_height              = 115;
	params->physical_width_um            = 64800;
	params->physical_height_um           = 115200;
//	params->density                      = 320;
	#endif

	// enable tearing-free
	params->dbi.te_mode                  = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity         = LCM_POLARITY_RISING;

	#if (LCM_DSI_CMD_MODE)
	params->dsi.mode                     = CMD_MODE;
	params->dsi.switch_mode              = SYNC_PULSE_VDO_MODE;
	#else
	params->dsi.mode                     = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;////
	#endif

	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM                 = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order  = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq    = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding      = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format       = LCM_DSI_FORMAT_RGB888;

	params->dsi.PS                       = LCM_PACKED_PS_24BIT_RGB888;

	#if (LCM_DSI_CMD_MODE)
	params->dsi.intermediat_buffer_num   = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
	params->dsi.word_count               = FRAME_WIDTH * 3; //DSI CMD mode need set these two bellow params, different to 6577
	#else
	params->dsi.intermediat_buffer_num   = 2;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
	#endif

	// Video mode setting
	params->dsi.packet_size              = 256;

	params->dsi.vertical_sync_active     = 8;
	params->dsi.vertical_backporch       = 8;
	params->dsi.vertical_frontporch      = 8;
	params->dsi.vertical_active_line     = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active   = 20;
	params->dsi.horizontal_backporch     = 70;//36
	params->dsi.horizontal_frontporch    = 80;//78
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;
	/* params->dsi.ssc_disable = 1; */

	params->dsi.PLL_CLOCK                = 486;
	
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
}


#define LCM_ID_ILI7807D                (0x780704)

static unsigned int lcm_compare_id(void)
 {
 
     int array[4];
     char buffer[5];
     char id_high=0;
     char id_midd=0;
     char id_low=0;
     int id=0;
     //首先进行复位操作
     SET_RESET_PIN(1);
     MDELAY(20);   
     SET_RESET_PIN(0);
     MDELAY(20);
     SET_RESET_PIN(1);
     MDELAY(120);
 
     //enable CMD2 Page6
     array[0]=0x00043902;
     array[1]=0x010778ff;
     dsi_set_cmdq(array, 2, 1);
     MDELAY(10);
 
 
     array[0]=0x00033700;
     dsi_set_cmdq(array, 1, 1);
 
     read_reg_v2(0x00, buffer,1);
     id_high = buffer[0]; ///////////////////////0x98
 
     read_reg_v2(0x01, buffer,1);
     id_midd = buffer[0]; ///////////////////////0x81
 
     read_reg_v2(0x02, buffer,1);
     id_low = buffer[0]; ////////////////////////0x0d
 
     id =(id_high << 16) | (id_midd << 8) | id_low;
 
     #if defined(BUILD_LK)
     printf("ILI7807D compare-LK:0x%02x,0x%02x,0x%02x,0x%02x\n", id_high, id_midd, id_low, id);
     #else
     printk("ILI7807D compare:0x%02x,0x%02x,0x%02x,0x%02x\n", id_high, id_midd, id_low, id);
     #endif
 
     return (id == LCM_ID_ILI7807D)?1:0;
 }

static void lcm_init(void)
{
	display_bias_enable();
	
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(10);
    
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	//fts_enter_low_power();
	MDELAY(10);
    push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);

	SET_RESET_PIN(0);
    MDELAY(10);
	display_bias_disable();
}

#ifndef BUILD_LK
extern atomic_t ESDCheck_byCPU;
#endif
static unsigned int lcm_ata_check(unsigned char *bufferr)
{
	//prize-Solve ATA testing-pengzhipeng-20181127-start
	unsigned char buffer0[2]={0};
	unsigned char buffer1[2]={0};
	unsigned char buffer2[2]={0};
	
	unsigned int data_array[6]; 
	 
	data_array[0]= 0x00023902;//LS packet 
	data_array[1]= 0x00005002; 
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x0002390a;//HS packet 
	data_array[1]= 0x0000fe27; 
	dsi_set_cmdq(data_array, 2, 1);
	

	data_array[0]= 0x0002390a;//HS packet 
	data_array[1]= 0x00005658; 
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	atomic_set(&ESDCheck_byCPU, 1);
	
	read_reg_v2(0x02, buffer0, 1);
	read_reg_v2(0x27, buffer1, 1);
	read_reg_v2(0x58, buffer2, 1);
	atomic_set(&ESDCheck_byCPU, 0);
	
	printk("%s, Kernel TDDI id = 0x%04x buffer1 0x52= 0x%04x buffer2= 0x%04x\n", __func__, buffer0[0], buffer1[0], buffer2[0]);
	return ((0x50 == buffer0[0])&&(0xfe == buffer1[0])&&(0x56 == buffer2[0]))?1:0; 
	//prize-Solve ATA testing-pengzhipeng-20181127-end
}

static void lcm_resume(void)
{
#if defined(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_TS_V22)
//fts_reset_proc(80);
#endif
    lcm_init();
}

struct LCM_DRIVER ili7807d_fhd_dsi_vdo_boe_drip_incell_lcm_drv =
{
    .name			= "ili7807d_fhd_dsi_vdo_boe_drip_incell",
#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "ili7807d",
		.vendor	= "focaltech",
		.id		= "0x82",
		.more	= "1080*1920",
	},
#endif
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
	.ata_check 		= lcm_ata_check,
};
