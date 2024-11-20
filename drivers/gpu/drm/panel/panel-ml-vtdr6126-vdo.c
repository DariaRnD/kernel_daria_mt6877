/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif
#include "include/panel-ml-vtdr6126-vdo.h"
#include "include/panel-ml-vtdr6126-vdo-gamma.h"
/*PRIZE:Added by lvyuanchuan,X9-678,20221230 start*/
extern int mtk_drm_esd_check_status(void);
extern void mtk_drm_esd_set_status(int status);
/*PRIZE:Added by lvyuanchuan,X9-678,20221230 end*/

//prize add by wangfei for lcd hardware info 20210726 start
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/prize/hardware_info/hardware_info.h"
extern struct hardware_info current_lcm_info;
#endif
//prize add by wangfei for lcd hardware info 20210726 end

static struct lcm *g_ctx;

/***********************************/

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *pm_enable_gpio;

	bool prepared;
	bool enabled;

	bool hbm_en;

	unsigned int bl_level;
	atomic_t reg_level;

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

/*PRIZE:Added by lvyuanchuan,X9-678,20221230 start*/
static void lcm_pannel_reconfig_blk(struct lcm *ctx)
{
	char bl_tb0[] = {0x51,0x07,0xFF};
	unsigned int reg_level = 0;
	if (mtk_drm_esd_check_status()) {
		/*PRIZE:Added by lvyuanchuan,X9-534,20230103*/
		if(ctx->bl_level)
			reg_level = Gamma_to_level[ctx->bl_level];

		bl_tb0[1] = (reg_level>>8)&0xf;
		bl_tb0[2] = (reg_level)&0xff;
		lcm_dcs_write(ctx,bl_tb0,ARRAY_SIZE(bl_tb0));
		mtk_drm_esd_set_status(0);
	}
}
/*PRIZE:Added by lvyuanchuan,X9-678,20221230 end*/

static void lcm_panel_init(struct lcm *ctx)
{
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}

	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	// udelay(250 * 1000);
	mdelay(50);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

//R
	lcm_dcs_write_seq_static(ctx,0x03,0x01);
	lcm_dcs_write_seq_static(ctx,0x2A,0x00,0x00,0x04,0x37);
	udelay(100);
	lcm_dcs_write_seq_static(ctx,0x2B,0x02,0x63,0x07,0x25);
	udelay(100);
	lcm_dcs_write_seq_static(ctx,0x35,0x00);
//	lcm_dcs_write_seq_static(ctx,0x51,0x07,0xFF);
	lcm_dcs_write_seq_static(ctx,0x53,0x28);
	lcm_dcs_write_seq_static(ctx,0x6C,0x02); //120Hz 6C=00;90Hz 6C=01;60Hz 6C=02
	lcm_dcs_write_seq_static(ctx,0x6D,0x00);
	lcm_dcs_write_seq_static(ctx,0x6F,0x01);
	lcm_dcs_write_seq_static(ctx,0x59,0x09); //09=DMR ON,00=DMR OFF
	lcm_dcs_write_seq_static(ctx,0x70,0x11,0x00,0x00,0x89,0x30,0x80,0x09,0x84,0x04,0x38,0x00,0x0C,0x02,0x1C,0x02,0x1C,0x02,0x00,0x02,0x0E,0x00,0x20,0x01,0x1F,0x00,0x07,0x00,0x0C,0x08,0xBB,0x08,0x7A,0x18,0x00,0x10,0xF0,0x03,0x0C,0x20,0x00,0x06,0x0B,0x0B,0x33,0x0E,0x1C,0x2A,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7B,0x7D,0x7E,0x01,0x02,0x01,0x00,0x09,0x40,0x09,0xBE,0x19,0xFC,0x19,0xFA,0x19,0xF8,0x1A,0x38,0x1A,0x78,0x1A,0xB6,0x2A,0xB6,0x2A,0xF4,0x2A,0xF4,0x4B,0x34,0x63,0x74,0x00,0x00,0x00,0x00,0x00,0x00);
	udelay(100);
	lcm_dcs_write_seq_static(ctx,0xF0,0xAA,0x10);
	udelay(100);
	lcm_dcs_write_seq_static(ctx,0xB1,0x01,0xCB,0x00,0x24,0x00,0x30,0x00,0x01,0xCB,0x00,0x24,0x03,0x78,0x00,0x01,0xCB,0x00,0x24,0x00,0x30,0x00,0x01,0xCB,0x00,0x24,0x00,0x30,0x00);
	udelay(100);
	lcm_dcs_write_seq_static(ctx,0xB2,0x01,0xCB,0x00,0x24,0x00,0x30,0x03,0x01,0xCB,0x00,0x24,0x00,0x30,0x03,0x01,0xCB,0x00,0x24,0x00,0x30,0x03);
	udelay(100);
	lcm_dcs_write_seq_static(ctx,0xd0,0x84,0x06,0x80,0x14,0x3c,0x00,0x39,0x07,0x1b,0x19,0x00,0x00,0x03,0x25,0x0c,0x00,0x00,0x0b,0x06,0x66,0x1b,0x1b,0x1b);//改AVDD电压
	udelay(100);
	lcm_dcs_write_seq_static(ctx,0xFF,0x5A,0x80);
	udelay(100);
	lcm_dcs_write_seq_static(ctx,0x65,0x26);
	lcm_dcs_write_seq_static(ctx,0xF7,0x01);
	lcm_dcs_write_seq_static(ctx,0x65,0x0A);
	lcm_dcs_write_seq_static(ctx,0xF9,0xA8);
	lcm_dcs_write_seq_static(ctx,0xFF,0x5A,0x81);
	udelay(100);
	lcm_dcs_write_seq_static(ctx,0x65,0x02);
	lcm_dcs_write_seq_static(ctx,0xF4,0x27);
	lcm_dcs_write_seq_static(ctx,0x65,0x19);
	lcm_dcs_write_seq_static(ctx,0xF8,0x01);
	lcm_dcs_write_seq_static(ctx,0x65,0x03);
	lcm_dcs_write_seq_static(ctx,0xFB,0x93,0x93);
	udelay(100);
/*	lcm_dcs_write_seq_static(ctx,0xF0,0xAA,0x11);  bistmode 白屏
	lcm_dcs_write_seq_static(ctx,0xDF,0x00,0x00,0x20,0x00,0xFF,0xFF,0xFF,0xFF); 
	lcm_dcs_write_seq_static(ctx,0xDE,0xAA,0x55,0x01,0x80); */
	/*PRIZE:Added by lvyuanchuan,X9-678,20221230*/
	lcm_pannel_reconfig_blk(ctx);
	lcm_dcs_write_seq_static(ctx,0x11,0x00);
	mdelay(120);
	lcm_dcs_write_seq_static(ctx,0x29,0x00);

	printk("[panel] %s lcm send cmd\n",__func__);
	mdelay(10);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	mdelay(10);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(120);

	ctx->error = 0;
	ctx->prepared = false;

	//reset
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	udelay(3000);
	// 138   --  2.8
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(3000);

	//137  -  1.2
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);


	//prize add by wangfei for ldo 1.8 20210709 start
	ctx->pm_enable_gpio = devm_gpiod_get_index(ctx->dev,
		"pm-enable", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->pm_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get pm_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->pm_enable_gpio));
		return PTR_ERR(ctx->pm_enable_gpio);
	}
	gpiod_set_value(ctx->pm_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->pm_enable_gpio);
	//prize add by wangfei for ldo 1.8 20210709 end


	ctx->hbm_en = false;
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);

	if (ctx->prepared)
		return 0;

	//prize add by wangfei for ldo 1.8 20210709 start
	ctx->pm_enable_gpio = devm_gpiod_get_index(ctx->dev,
		"pm-enable", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->pm_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get pm_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->pm_enable_gpio));
		return PTR_ERR(ctx->pm_enable_gpio);
	}
	gpiod_set_value(ctx->pm_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->pm_enable_gpio);
	//prize add by wangfei for ldo 1.8 20210709 end
	udelay(3000);


	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(3000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}
	ctx->enabled = true;

	return 0;
}

#define VAC (2436)
#define HAC (1080)

static const struct drm_display_mode switch_mode_120 = {
	.clock = ((FRAME_WIDTH+MODE_2_HFP+HSA+HBP)*(FRAME_HEIGHT+MODE_2_VFP+VSA+VBP)*(MODE_2_FPS)/1000),
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + MODE_2_HFP,
	.hsync_end = FRAME_WIDTH + MODE_2_HFP + HSA,
	.htotal = FRAME_WIDTH + MODE_2_HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_2_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_2_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_2_VFP + VSA + VBP,
	.vrefresh = MODE_2_FPS,
};


static const struct drm_display_mode switch_mode_90 = {
	.clock = ((FRAME_WIDTH+MODE_1_HFP+HSA+HBP)*(FRAME_HEIGHT+MODE_1_VFP+VSA+VBP)*(MODE_1_FPS)/1000),
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + MODE_1_HFP,
	.hsync_end = FRAME_WIDTH + MODE_1_HFP + HSA,
	.htotal = FRAME_WIDTH + MODE_1_HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_1_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_1_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_1_VFP + VSA + VBP,
	.vrefresh = MODE_1_FPS,
};

static const struct drm_display_mode default_mode = {
	.clock = ((FRAME_WIDTH+MODE_0_HFP+HSA+HBP)*(FRAME_HEIGHT+MODE_0_VFP+VSA+VBP)*(MODE_0_FPS)/1000),
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + MODE_0_HFP,
	.hsync_end = FRAME_WIDTH + MODE_0_HFP + HSA,
	.htotal = FRAME_WIDTH + MODE_0_HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_0_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_0_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_0_VFP + VSA + VBP,
	.vrefresh = MODE_0_FPS,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	ssize_t ret;

	ret = mipi_dsi_dcs_read(dsi, 0x04, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	printk("panel_ata_check-ATA read 0x04 data %x %x %x\n", data[0], data[1], data[2]);

	if (data[1] == 0x80) {
		return 1;
	}
	
	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51,0x07,0xFF};
	char hbm_tb[] = {0x51,0x0F,0xFF};
	unsigned int reg_level = 0;

	if (level && level <= BRIGHTNESS_HALF) {
		reg_level = Gamma_to_level[level];
		atomic_set(&g_ctx->reg_level, reg_level);
	}

	g_ctx->bl_level = level;
	bl_tb0[1] = (reg_level>>8)&0xf;
	bl_tb0[2] = (reg_level)&0xff;
	pr_err("level{ %d - %d },bl_tb0[1] = %d,bl_tb0[2] = %d\n",level,reg_level,bl_tb0[1],bl_tb0[2]);
	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	return 0;
}


//prize add by gongtaitao for sensorhub get backlight 20221028 start
unsigned short led_level_disp_get(char *name)
{
    int trans_level = 0;
	trans_level = Gamma_to_level[g_ctx->bl_level];
	pr_err("[%s]: name: %s, level : %d",__func__, name, trans_level);
	return trans_level;
}
EXPORT_SYMBOL(led_level_disp_get);
//prize add by gongtaitao for sensorhub get backlight 20221028 end

unsigned int lhbm_for_gain[] = {
0, 727, 751, 767, 787, 803, 819, 839, 855, 875, 900, 924, 940, 964, 984, 1000,
1024, 1044, 1068, 1088, 1108, 1124, 1148, 1173, 1185, 1205, 1225, 1245, 1261,
1285, 1301, 1317, 1341, 1361, 1381, 1401, 1422, 1434, 1454, 1474, 1494, 1518,
1538, 1550, 1570, 1594, 1614, 1634, 1654, 1675, 1687, 1707, 1727, 1751, 1771,
1795, 1807, 1819, 1831, 1843, 1855, 1871, 1887, 1903, 1915, 1928, 1944, 1956,
1968, 1980, 1984, 1996, 2012, 2024, 2032, 2044, 2064, 2076, 2092, 2104, 2120,
2128, 2140, 2152, 2160, 2172, 2189, 2201, 2213, 2225, 2233, 2249, 2261, 2277,
2285, 2297, 2309, 2325, 2333, 2341, 2353, 2365, 2373, 2385, 2397, 2405, 2421,
2429, 2442, 2450, 2462, 2470, 2482, 2486, 2494, 2502, 2514, 2518, 2526, 2534,
2542, 2546, 2558, 2566, 2578, 2590, 2594, 2602, 2610, 2618, 2626, 2638, 2642,
2650, 2658, 2662, 2666, 2682, 2691, 2695, 2711, 2715, 2723, 2731, 2739, 2747,
2759, 2759, 2771, 2783, 2791, 2799, 2807, 2811, 2815, 2827, 2831, 2835, 2847,
2859, 2863, 2875, 2883, 2887, 2895, 2899, 2911, 2919, 2927, 2931, 2948, 2952,
2964, 2968, 2976, 2980, 2988, 2996, 3004, 3008, 3012, 3016, 3028, 3040, 3044,
3056, 3056, 3060, 3068, 3076, 3084, 3092, 3104, 3112, 3120, 3132, 3136, 3140,
3152, 3160, 3164, 3172, 3172, 3184, 3192, 3201, 3205, 3213, 3221, 3229, 3237,
3245, 3253, 3261, 3269, 3273, 3285, 3285, 3293, 3305, 3313, 3317, 3329, 3333,
3337, 3349, 3353, 3361, 3365, 3369, 3373, 3389, 3393, 3397, 3405, 3409, 3417,
3429, 3433, 3441, 3449, 3453, 3458, 3470, 3474, 3482, 3486, 3494, 3498, 3510,
3514, 3522, 3526, 3538, 3538, 3550, 3554, 3558, 3566, 3574, 3574, 3582, 3594,
3598, 3610, 3618, 3630, 3630, 3642, 3642, 3646, 3654, 3662, 3670, 3678, 3682,
3686, 3694, 3702, 3710, 3723, 3731, 3731, 3743, 3747, 3755, 3767, 3771, 3775,
3783, 3787, 3787, 3799, 3799, 3807, 3819, 3827, 3831, 3839, 3843, 3851, 3871,
3875, 3879, 3883, 3891, 3895, 3903, 3911, 3919, 3931, 3935, 3939, 3947, 3959,
3959, 3972, 3976, 3984, 3992, 3996, 3996, 4012, 4020, 4024, 4032, 4036, 4040,
4048, 4060, 4060, 4076, 4080, 4084, 4088, 4096
};

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	char hbm_tb0[] = {0x63, 0x10, 0x00, 0x07, 0xFF};
	char hbm_tb1[] = {0x62, 0x03};
	char normal_tb[] = {0x62,0x00};
	unsigned int trans_level = atomic_read(&g_ctx->reg_level);
	struct lcm *ctx = panel_to_lcm(panel);

	if (!cb)
		return -1;

	if (ctx->hbm_en == en)
		goto done;

	if (en)
	{
		printk("[panel] %s : set HBM, trans_level:%d\n",__func__,trans_level);

		if (trans_level >= 335) {
			hbm_tb0[1] = 0x10;
			hbm_tb0[2] = 0x00;
			hbm_tb0[3] = (u8)((trans_level>>8)&0xFF);
			hbm_tb0[4] = (u8)(trans_level&0xFF);
			cb(dsi, handle, hbm_tb0, ARRAY_SIZE(hbm_tb0));
			cb(dsi, handle, hbm_tb1, ARRAY_SIZE(hbm_tb1));
		} else {
			hbm_tb0[1] = (u8)((lhbm_for_gain[trans_level]>>8)&0xFF);
			hbm_tb0[2] = (u8)(lhbm_for_gain[trans_level]&0xFF);
			hbm_tb0[3] = 0x01;
			hbm_tb0[4] = 0x4F;
			cb(dsi, handle, hbm_tb0, ARRAY_SIZE(hbm_tb0));
			cb(dsi, handle, hbm_tb1, ARRAY_SIZE(hbm_tb1));
		}
	}
	else
	{
		printk("[panel] %s : out HBM mode\n",__func__);

		cb(dsi, handle, normal_tb, ARRAY_SIZE(normal_tb));
	}

	ctx->hbm_en = en;

 done:
	return 0;
}

static void panel_hbm_get_state(struct drm_panel *panel, bool *state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*state = ctx->hbm_en;
}

static int lcm_get_virtual_heigh(void)
{
	return VAC;
}

static int lcm_get_virtual_width(void)
{
	return HAC;
}

static struct mtk_panel_params ext_params_120 = {
	// .vfp_low_power = 743,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 3,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list[2] = 0x00,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9c,
	},
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
			.enable = 1,
			.bdg_dsc_enable = 0,
			.ver                   =  DSC_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},
//	.wait_sof_before_dec_vfp = 1,
//drv-Resolve the problem of shining caused by out of sync between dimlayer and hbm-pengzhipeng-20231225-start
	.hbm_en_time = 0,
	.hbm_dis_time = 0,
//drv-Resolve the problem of shining caused by out of sync between dimlayer and hbm-pengzhipeng-20231225-end
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2, {0x6C, 0x00}},
	},
	.data_rate = MODE_2_DATA_RATE,
	.bdg_ssc_disable = 1,
	.ssc_disable = 1,
	.dyn = {
		.switch_en = 1,
		.data_rate = MODE_2_DATA_RATE,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_90 = {
	// .vfp_low_power = 743,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 3,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list[2] = 0x00,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9c,
	},
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
			.enable = 1,
			.bdg_dsc_enable = 0,
			.ver                   =  DSC_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},
//	.wait_sof_before_dec_vfp = 1,
//drv-Resolve the problem of shining caused by out of sync between dimlayer and hbm-pengzhipeng-20231225-start
	.hbm_en_time = 0,
	.hbm_dis_time = 0,
//drv-Resolve the problem of shining caused by out of sync between dimlayer and hbm-pengzhipeng-20231225-end
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 2, {0x6C, 0x01}},
	},
	.data_rate = MODE_2_DATA_RATE,
	.bdg_ssc_disable = 1,
	.ssc_disable = 1,
	.dyn = {
		.switch_en = 1,
		.data_rate = MODE_1_DATA_RATE,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};


static struct mtk_panel_params ext_params = {
	// .pll_clk = 373,
	// .vfp_low_power = 743,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x66,
		.count = 3,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
		.para_list[2] = 0x00,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9c,
	},
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.dsc_params = {
			.enable = 1,
			.bdg_dsc_enable = 0,
			.ver                   =  DSC_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},
//	.wait_sof_before_dec_vfp = 1,
//drv-Resolve the problem of shining caused by out of sync between dimlayer and hbm-pengzhipeng-20231225-start
	.hbm_en_time = 0,
	.hbm_dis_time = 0,
//drv-Resolve the problem of shining caused by out of sync between dimlayer and hbm-pengzhipeng-20231225-end
	.data_rate = MODE_2_DATA_RATE,
	.bdg_ssc_disable = 1,
	.ssc_disable = 1,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 2, {0x6C, 0x02}},
	},
	.dyn = {
		.switch_en = 1,
		.data_rate = MODE_0_DATA_RATE,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};


struct drm_display_mode *get_mode_by_id(struct drm_panel *panel,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &panel->connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(panel, mode);
	printk("[panel] %s,vrefresh = %d\n",__func__,m->vrefresh);
	if (m->vrefresh == MODE_0_FPS)
		ext->params = &ext_params;
	else if (m->vrefresh == MODE_1_FPS)
		ext->params = &ext_params_90;
	else if (m->vrefresh == MODE_2_FPS)
		ext->params = &ext_params_120;
	else
		ret = 1;

	return ret;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.hbm_get_state = panel_hbm_get_state,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
	.ext_param_set = mtk_panel_ext_param_set,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode_1;
	struct drm_display_mode *mode_2;

	mode = drm_mode_duplicate(panel->drm, &switch_mode_120);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			switch_mode_120.hdisplay, switch_mode_120.vdisplay,
			switch_mode_120.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER ;
	drm_mode_probed_add(panel->connector, mode);
	printk("[panel] %s,333\n",__func__);
	
	
	mode_1 = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode_1) {
		dev_err(panel->drm->dev, "failed to add mode_1 %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode_1);
	mode_1->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode_1);
	printk("[panel] %s,111\n",__func__);

	mode_2 = drm_mode_duplicate(panel->drm, &switch_mode_90);
	if (!mode_2) {
		dev_err(panel->drm->dev, "failed to add mode_2 %ux%ux@%u\n",
			switch_mode_90.hdisplay, switch_mode_90.vdisplay,
			switch_mode_90.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode_2);
	mode_2->type = DRM_MODE_TYPE_DRIVER ;
	drm_mode_probed_add(panel->connector, mode_2);
	printk("[panel] %s,222\n",__func__);

	

	panel->connector->display_info.width_mm = 70;
	panel->connector->display_info.height_mm = 157;

	return 3;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_err("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_err("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_err("gezi ---- %s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	// dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
	// 		 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
	// 		 | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	//dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS;  //prize add by yinhanhan for hbm 20230311
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;
	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}

	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	//mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	//add by wangfei
	// lcm_panel_init(ctx);
	g_ctx = ctx;
	ctx->hbm_en = false;

#if defined(CONFIG_PRIZE_HARDWARE_INFO)
    strcpy(current_lcm_info.chip,"vtdr6126.vdo");
    strcpy(current_lcm_info.vendor,"Viewtrix");
    sprintf(current_lcm_info.id,"0x%02x",0x81);
    strcpy(current_lcm_info.more,"1080*2436");
#endif
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "Viewtrix,vtdr6126,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-ml-vtdr6126-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("vtdr6126 VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");

