#if defined(BUILD_LK)
#include <debug.h>
#else
#include <linux/kernel.h>
#include <linux/string.h>
#endif

#if defined(BUILD_LK)
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif

#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (960)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static struct LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)                                        (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                               (lcm_util.udelay(n))
#define MDELAY(n)                                               (lcm_util.mdelay(n))

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_END_OF_TABLE	0xFFFD

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)           lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                          lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                      lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table
{
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
{0xD1,52, {0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x14,0x00,0x32,0x00,0x6f,0x00,0x80,0x00,0xa5,0x00,0xcf,0x01,0x25,0x01,0x61,0x01,0xb4,0x01,0xf0,0x01,0xf4,0x02,0x28,0x02,0x5e,0x02,0x7a,0x02,0xa0,0x02,0xb3,0x02,0xd2,0x02,0xe3,0x02,0xfd,0x03,0x13,0x03,0x2F,0x03,0x44,0x03,0xFF}},
{0xD2,52, {0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x14,0x00,0x32,0x00,0x6f,0x00,0x80,0x00,0xa5,0x00,0xcf,0x01,0x25,0x01,0x61,0x01,0xb4,0x01,0xf0,0x01,0xf4,0x02,0x28,0x02,0x5e,0x02,0x7a,0x02,0xa0,0x02,0xb3,0x02,0xd2,0x02,0xe3,0x02,0xfd,0x03,0x13,0x03,0x2F,0x03,0x44,0x03,0xFF}},
{0xD3,52, {0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x14,0x00,0x32,0x00,0x6f,0x00,0x80,0x00,0xa5,0x00,0xcf,0x01,0x25,0x01,0x61,0x01,0xb4,0x01,0xf0,0x01,0xf4,0x02,0x28,0x02,0x5e,0x02,0x7a,0x02,0xa0,0x02,0xb3,0x02,0xd2,0x02,0xe3,0x02,0xfd,0x03,0x13,0x03,0x2F,0x03,0x44,0x03,0xFF}},
{0xD4,52, {0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x14,0x00,0x32,0x00,0x6f,0x00,0x80,0x00,0xa5,0x00,0xcf,0x01,0x25,0x01,0x61,0x01,0xb4,0x01,0xf0,0x01,0xf4,0x02,0x28,0x02,0x5e,0x02,0x7a,0x02,0xa0,0x02,0xb3,0x02,0xd2,0x02,0xe3,0x02,0xfd,0x03,0x13,0x03,0x2F,0x03,0x44,0x03,0xFF}},
{0xD5,52, {0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x14,0x00,0x32,0x00,0x6f,0x00,0x80,0x00,0xa5,0x00,0xcf,0x01,0x25,0x01,0x61,0x01,0xb4,0x01,0xf0,0x01,0xf4,0x02,0x28,0x02,0x5e,0x02,0x7a,0x02,0xa0,0x02,0xb3,0x02,0xd2,0x02,0xe3,0x02,0xfd,0x03,0x13,0x03,0x2F,0x03,0x44,0x03,0xFF}},
{0xD6,52, {0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x14,0x00,0x32,0x00,0x6f,0x00,0x80,0x00,0xa5,0x00,0xcf,0x01,0x25,0x01,0x61,0x01,0xb4,0x01,0xf0,0x01,0xf4,0x02,0x28,0x02,0x5e,0x02,0x7a,0x02,0xa0,0x02,0xb3,0x02,0xd2,0x02,0xe3,0x02,0xfd,0x03,0x13,0x03,0x2F,0x03,0x44,0x03,0xFF}},
{0x11,0, {0x00}},
{REGFLAG_DELAY, 120, {}},
{0x29,0, {0x00}},
{REGFLAG_DELAY, 50, {}},

{REGFLAG_END_OF_TABLE, 0x00, {}},

};

static struct LCM_setting_table lcm_sleep_mode_in_setting[] =
{
	{0xF0, 5,{0x55,0xaa,0x52,0x08,0x00}},  
	{0xc1, 1,{0x3f}},  
	
	{0x6C, 1,{0x60}},
	{REGFLAG_DELAY, 20, {}},
	{0xB1, 1,{0x00}},
	{0xFA, 4,{0x7F, 0x00, 0x00, 0x00}},
	{REGFLAG_DELAY, 20, {}},
	{0x6c,1,{0x50}}, 
	{REGFLAG_DELAY, 10, {}},

	{0x28, 0,{0x00}},	
	{REGFLAG_DELAY, 50, {}},  
	{0x10, 0,{0x00}},
	{REGFLAG_DELAY, 20, {}},

	{0xF0,5,{0x55,0xaa,0x52,0x08,0x00}},
	{0xc2,1,{0xce}},
	{0xc3,1,{0xcd}},
	{0xc6,1,{0xfc}},
	{0xc5,1,{0x03}},
	{0xcd,1,{0x64}},
	{0xc4,1,{0xff}},

	{0xc9,1,{0xcd}},
	{0xF6,2,{0x5a,0x87}},
	{0xFd,3,{0xaa,0xaa, 0x0a}},
	{0xFe,2,{0x6a,0x0a}},
	{0x78,2,{0x2a,0xaa}},
	{0x92,2,{0x17,0x08}},
	{0x77,2,{0xaa,0x2a}},
	{0x76,2,{0xaa,0xaa}},

	{0x84,1,{0x00}},
	{0x78,2,{0x2b,0xba}},
	{0x89,1,{0x73}},
	{0x88,1,{0x3A}},
	{0x85,1,{0xB0}},
	{0x76,2,{0xeb,0xaa}},
	{0x94,1,{0x80}},
	{0x87,3,{0x04,0x07,0x30}},					
	{0x93,1,{0x27}},
	{0xaf,1,{0x02}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd)
        {
        case REGFLAG_DELAY :
            MDELAY(table[i].count);
            break;
        case REGFLAG_END_OF_TABLE :
            break;
        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
            MDELAY(2);
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

    params->type   = LCM_TYPE_DSI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
	
#ifndef BUILD_LK
	params->physical_width               = 62;     //LCM_PHYSICAL_WIDTH/1000;
	params->physical_height              = 124;    //LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um            = 61877;  //LCM_PHYSICAL_WIDTH; = sqrt((size*25.4)^2/(18^2+9^2))*9*1000
	params->physical_height_um           = 123754; //LCM_PHYSICAL_HEIGHT; = sqrt((size*25.4)^2/(18^2+9^2))*18*1000
#endif

    // enable tearing-free
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;
    params->dsi.mode                    = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size = 256;

    // Video mode setting
    params->dsi.PS                      = LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.word_count              = FRAME_WIDTH * 3;

    params->dsi.vertical_sync_active    = 8;
    params->dsi.vertical_backporch      = 30;
    params->dsi.vertical_frontporch     = 12;
    params->dsi.vertical_active_line    = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active  = 10;
    params->dsi.horizontal_backporch    = 20;
    params->dsi.horizontal_frontporch   = 20;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 230; //234

    //params->dsi.ssc_disable = 1;
	params->dsi.cont_clock = 1;
    
    // params->dsi.clk_lp_per_line_enable   = 1;
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	
/* prize added by chenjiaxi, round corner, 20220601-start */
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->full_content = 0;
	params->corner_pattern_width = 480;
	params->corner_pattern_height = 32;
	params->corner_pattern_height_bot = 32;
#endif
/* prize added by chenjiaxi, round corner, 20220601-end */
}

static void set_ldo_control(int level)
{
	if (level) {
		display_ldo18_enable(1);
		MDELAY(2);
		display_ldo28_enable(1);
	}else {
		display_ldo28_enable(0);
		MDELAY(2);
		display_ldo18_enable(0);		
	}
}

static void lcm_init(void)
{
	set_ldo_control(1);
	MDELAY(10);
    display_rst_enable(1);
    MDELAY(10);
    display_rst_enable(0);
    MDELAY(10);
    display_rst_enable(1);
    MDELAY(50);

#ifdef BUILD_LK
    printf("cjx:%s\n", __func__);
#else
    printk("cjx:%s\n", __func__);
#endif

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
    display_rst_enable(0);
    MDELAY(10);
	set_ldo_control(0);
	MDELAY(10);
}

static unsigned int lcm_compare_id(void)
{
    return 1;
}

static void lcm_resume(void)
{
    lcm_init();
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
	unsigned int id = 0;
	unsigned char buffer1[3];
	unsigned int array[16];
	int i = 0;
	
	for (i = 0; i < 3;i++) {
	
		array[0] = 0x00013700;
		dsi_set_cmdq(array, 1, 1);

		read_reg_v2(0xDB, buffer1, 1);
		id = buffer1[0];

		printk("%s,gc9503v_id=0x%08x\n ", __func__, id);
	
		if (id == 0x13)
			return 1;
		else
			continue;
	}
	
	return 0;
}

struct LCM_DRIVER gc9503np_fwvgap_dsi_vdo_tn_lcm_drv =
{
    .name           = "gc9503np_fwvgap_dsi_vdo_tn_lcm_drv",
#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
    .lcm_info = {
        .chip	= "gc9503np",
        .vendor	= "haifei",
        .id		= "0X13",
        .more	= "480*960",
    },
#endif
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
	.ata_check      = lcm_ata_check,
    .compare_id     = lcm_compare_id,
};
