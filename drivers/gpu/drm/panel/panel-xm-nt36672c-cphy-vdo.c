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

#include <linux/delay.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/fb.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_drm_graphics_base.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_panel_ext.h"
#endif
/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

#include "../../../misc/prize/lcd_bias/prize_lcd_bias.h"


#define PANEL_CLOCK 361519
#define PANEL_WIDTH  1080
#define PANEL_HEIGHT 2408

#define HSA 12
#define HBP 56
#define HFP 76

#define VSA 4
#define VBP 16
/*Parameter setting for mode 0 Start*/
#define MODE_0_FPS 60
#define MODE_0_VFP 1184
/*Parameter setting for mode 0 End*/

/*Parameter setting for mode 1 Start*/
#define MODE_1_FPS 90
#define MODE_1_VFP 54
/*Parameter setting for mode 1 End*/

#define PCLK_IN_KHZ_60HZ \
    ((PANEL_WIDTH+HFP+HSA+HBP)*(PANEL_HEIGHT+MODE_0_VFP+VSA+VBP)*(60)/1000)

#define PCLK_IN_KHZ_90HZ \
    ((PANEL_WIDTH+HFP+HSA+HBP)*(PANEL_HEIGHT+MODE_1_VFP+VSA+VBP)*(90)/1000)

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;

	bool prepared;
	bool enabled;

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
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
	//if ((int)*addr < 0xB0)
		//ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	//else
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
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void lcm_panel_init(struct lcm *ctx)
{

	pr_info("NT36672C %s\n", __func__);

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
	mdelay(20);//ili9882q at least 10ms
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	lcm_dcs_write_seq_static(ctx,0xFF,0x10);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0xB0,0x00);
	lcm_dcs_write_seq_static(ctx,0xC0,0x00);
	lcm_dcs_write_seq_static(ctx,0xC1,0x89,0x28,0x00,0x08,0x00,0xAA,0x02,0x0E,0x00,0x2B,0x00,0x07,0x0D,0xB7,0x0C,0xB7);
	lcm_dcs_write_seq_static(ctx,0xC2,0x1B,0xA0);

	lcm_dcs_write_seq_static(ctx,0xFF,0x20);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x01,0x66);
	lcm_dcs_write_seq_static(ctx,0x06,0x40);
	lcm_dcs_write_seq_static(ctx,0x07,0x38);
	lcm_dcs_write_seq_static(ctx,0x1B,0x01);
	lcm_dcs_write_seq_static(ctx,0x5C,0x90);
	lcm_dcs_write_seq_static(ctx,0x5E,0xB2);
	lcm_dcs_write_seq_static(ctx,0x69,0xD1);
	lcm_dcs_write_seq_static(ctx,0x95,0xD1);
	lcm_dcs_write_seq_static(ctx,0x96,0xD1);
	lcm_dcs_write_seq_static(ctx,0xF2,0x64);
	lcm_dcs_write_seq_static(ctx,0xF4,0x64);
	lcm_dcs_write_seq_static(ctx,0xF6,0x64);
	lcm_dcs_write_seq_static(ctx,0xF8,0x64);

	lcm_dcs_write_seq_static(ctx,0xFF,0x21);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);

	lcm_dcs_write_seq_static(ctx,0xFF,0x24);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x01,0x0F);
	lcm_dcs_write_seq_static(ctx,0x03,0x0C);
	lcm_dcs_write_seq_static(ctx,0x05,0x1D);
	lcm_dcs_write_seq_static(ctx,0x08,0x2F);
	lcm_dcs_write_seq_static(ctx,0x09,0x2E);
	lcm_dcs_write_seq_static(ctx,0x0A,0x2D);
	lcm_dcs_write_seq_static(ctx,0x0B,0x2C);
	lcm_dcs_write_seq_static(ctx,0x11,0x17);
	lcm_dcs_write_seq_static(ctx,0x12,0x13);
	lcm_dcs_write_seq_static(ctx,0x13,0x15);
	lcm_dcs_write_seq_static(ctx,0x15,0x14);
	lcm_dcs_write_seq_static(ctx,0x16,0x16);
	lcm_dcs_write_seq_static(ctx,0x17,0x18);
	lcm_dcs_write_seq_static(ctx,0x1B,0x01);
	lcm_dcs_write_seq_static(ctx,0x1D,0x1D);
	lcm_dcs_write_seq_static(ctx,0x20,0x2F);
	lcm_dcs_write_seq_static(ctx,0x21,0x2E);
	lcm_dcs_write_seq_static(ctx,0x22,0x2D);
	lcm_dcs_write_seq_static(ctx,0x23,0x2C);
	lcm_dcs_write_seq_static(ctx,0x29,0x17);
	lcm_dcs_write_seq_static(ctx,0x2A,0x13);
	lcm_dcs_write_seq_static(ctx,0x2B,0x15);
	lcm_dcs_write_seq_static(ctx,0x2F,0x14);
	lcm_dcs_write_seq_static(ctx,0x30,0x16);
	lcm_dcs_write_seq_static(ctx,0x31,0x18);
	lcm_dcs_write_seq_static(ctx,0x32,0x04);
	lcm_dcs_write_seq_static(ctx,0x34,0x10);
	lcm_dcs_write_seq_static(ctx,0x35,0x1F);
	lcm_dcs_write_seq_static(ctx,0x36,0x1F);
	lcm_dcs_write_seq_static(ctx,0x4D,0x19);
	lcm_dcs_write_seq_static(ctx,0x4E,0x45);
	lcm_dcs_write_seq_static(ctx,0x4F,0x45);
	lcm_dcs_write_seq_static(ctx,0x53,0x45);
	lcm_dcs_write_seq_static(ctx,0x71,0x30);
	lcm_dcs_write_seq_static(ctx,0x79,0x11);
	lcm_dcs_write_seq_static(ctx,0x7A,0x82);
	lcm_dcs_write_seq_static(ctx,0x7B,0x94);
	lcm_dcs_write_seq_static(ctx,0x7D,0x04);
	lcm_dcs_write_seq_static(ctx,0x80,0x04);
	lcm_dcs_write_seq_static(ctx,0x81,0x04);
	lcm_dcs_write_seq_static(ctx,0x82,0x13);
	lcm_dcs_write_seq_static(ctx,0x84,0x31);
	lcm_dcs_write_seq_static(ctx,0x85,0x00);
	lcm_dcs_write_seq_static(ctx,0x86,0x00);
	lcm_dcs_write_seq_static(ctx,0x87,0x00);
	lcm_dcs_write_seq_static(ctx,0x90,0x13);
	lcm_dcs_write_seq_static(ctx,0x92,0x31);
	lcm_dcs_write_seq_static(ctx,0x93,0x00);
	lcm_dcs_write_seq_static(ctx,0x94,0x00);
	lcm_dcs_write_seq_static(ctx,0x95,0x00);
	lcm_dcs_write_seq_static(ctx,0x9C,0xF4);
	lcm_dcs_write_seq_static(ctx,0x9D,0x01);
	lcm_dcs_write_seq_static(ctx,0xA0,0x14);
	lcm_dcs_write_seq_static(ctx,0xA2,0x14);
	lcm_dcs_write_seq_static(ctx,0xA3,0x02);
	lcm_dcs_write_seq_static(ctx,0xA4,0x04);
	lcm_dcs_write_seq_static(ctx,0xA5,0x04);
	lcm_dcs_write_seq_static(ctx,0xC6,0xC0);
	lcm_dcs_write_seq_static(ctx,0xC9,0x00);
	lcm_dcs_write_seq_static(ctx,0xD9,0x80);
	lcm_dcs_write_seq_static(ctx,0xE9,0x02);

	lcm_dcs_write_seq_static(ctx,0xFF,0x25);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x0F,0x1B);
	lcm_dcs_write_seq_static(ctx,0x19,0xE4);
	lcm_dcs_write_seq_static(ctx,0x21,0x40);
	lcm_dcs_write_seq_static(ctx,0x66,0xD8);
	lcm_dcs_write_seq_static(ctx,0x68,0x50);
	lcm_dcs_write_seq_static(ctx,0x69,0x10);
	lcm_dcs_write_seq_static(ctx,0x6B,0x00);
	lcm_dcs_write_seq_static(ctx,0x6D,0x0D);
	lcm_dcs_write_seq_static(ctx,0x6E,0x48);
	lcm_dcs_write_seq_static(ctx,0x72,0x41);
	lcm_dcs_write_seq_static(ctx,0x73,0x4A);
	lcm_dcs_write_seq_static(ctx,0x74,0xD0);
	lcm_dcs_write_seq_static(ctx,0x77,0x62);
	lcm_dcs_write_seq_static(ctx,0x7D,0x40);
	lcm_dcs_write_seq_static(ctx,0x7F,0x00);
	lcm_dcs_write_seq_static(ctx,0x80,0x04);
	lcm_dcs_write_seq_static(ctx,0x84,0x0D);
	lcm_dcs_write_seq_static(ctx,0xCF,0x80);
	lcm_dcs_write_seq_static(ctx,0xD6,0x80);
	lcm_dcs_write_seq_static(ctx,0xD7,0x80);
	lcm_dcs_write_seq_static(ctx,0xEF,0x20);
	lcm_dcs_write_seq_static(ctx,0xF0,0x84);

	lcm_dcs_write_seq_static(ctx,0xFF,0x26);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x15,0x04);
	lcm_dcs_write_seq_static(ctx,0x81,0x14);
	lcm_dcs_write_seq_static(ctx,0x83,0x02);
	lcm_dcs_write_seq_static(ctx,0x84,0x03);
	lcm_dcs_write_seq_static(ctx,0x85,0x01);
	lcm_dcs_write_seq_static(ctx,0x86,0x03);
	lcm_dcs_write_seq_static(ctx,0x87,0x01);
	lcm_dcs_write_seq_static(ctx,0x88,0x06);
	lcm_dcs_write_seq_static(ctx,0x8A,0x1A);
	lcm_dcs_write_seq_static(ctx,0x8B,0x11);
	lcm_dcs_write_seq_static(ctx,0x8C,0x24);
	lcm_dcs_write_seq_static(ctx,0x8E,0x42);
	lcm_dcs_write_seq_static(ctx,0x8F,0x11);
	lcm_dcs_write_seq_static(ctx,0x90,0x11);
	lcm_dcs_write_seq_static(ctx,0x91,0x11);
	lcm_dcs_write_seq_static(ctx,0x9A,0x81);
	lcm_dcs_write_seq_static(ctx,0x9B,0x03);
	lcm_dcs_write_seq_static(ctx,0x9C,0x00);
	lcm_dcs_write_seq_static(ctx,0x9D,0x00);
	lcm_dcs_write_seq_static(ctx,0x9E,0x00);

	lcm_dcs_write_seq_static(ctx,0xFF,0x27);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x01,0x68);
	lcm_dcs_write_seq_static(ctx,0x20,0x81);
	lcm_dcs_write_seq_static(ctx,0x21,0xE9);
	lcm_dcs_write_seq_static(ctx,0x25,0x82);
	lcm_dcs_write_seq_static(ctx,0x26,0x1E);
	lcm_dcs_write_seq_static(ctx,0x6E,0x00);
	lcm_dcs_write_seq_static(ctx,0x6F,0x00);
	lcm_dcs_write_seq_static(ctx,0x70,0x00);
	lcm_dcs_write_seq_static(ctx,0x71,0x00);
	lcm_dcs_write_seq_static(ctx,0x72,0x00);
	lcm_dcs_write_seq_static(ctx,0x75,0x00);
	lcm_dcs_write_seq_static(ctx,0x76,0x00);
	lcm_dcs_write_seq_static(ctx,0x77,0x00);
	lcm_dcs_write_seq_static(ctx,0x7D,0x09);
	lcm_dcs_write_seq_static(ctx,0x7E,0x67);
	lcm_dcs_write_seq_static(ctx,0x80,0x23);
	lcm_dcs_write_seq_static(ctx,0x82,0x09);
	lcm_dcs_write_seq_static(ctx,0x83,0x67);
	lcm_dcs_write_seq_static(ctx,0x88,0x01);
	lcm_dcs_write_seq_static(ctx,0x89,0x10);
	lcm_dcs_write_seq_static(ctx,0xA6,0x23);
	lcm_dcs_write_seq_static(ctx,0xA7,0x01);
	lcm_dcs_write_seq_static(ctx,0xB6,0x40);

	lcm_dcs_write_seq_static(ctx,0xFF,0x2A);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x00,0x91);
	lcm_dcs_write_seq_static(ctx,0x03,0x20);
	lcm_dcs_write_seq_static(ctx,0x07,0x5A);
	lcm_dcs_write_seq_static(ctx,0x0A,0x70);
	lcm_dcs_write_seq_static(ctx,0x0D,0x40);
	lcm_dcs_write_seq_static(ctx,0x0E,0x02);
	lcm_dcs_write_seq_static(ctx,0x11,0xF0);
	lcm_dcs_write_seq_static(ctx,0x15,0x0F);
	lcm_dcs_write_seq_static(ctx,0x16,0x1A);
	lcm_dcs_write_seq_static(ctx,0x19,0x0E);
	lcm_dcs_write_seq_static(ctx,0x1A,0xEE);
	lcm_dcs_write_seq_static(ctx,0x1B,0x14);
	lcm_dcs_write_seq_static(ctx,0x1D,0x36);
	lcm_dcs_write_seq_static(ctx,0x1E,0x4F);
	lcm_dcs_write_seq_static(ctx,0x1F,0x4F);
	lcm_dcs_write_seq_static(ctx,0x20,0x4F);
	lcm_dcs_write_seq_static(ctx,0x28,0xF7);
	lcm_dcs_write_seq_static(ctx,0x29,0x06);
	lcm_dcs_write_seq_static(ctx,0x2A,0x03);
	lcm_dcs_write_seq_static(ctx,0x2F,0x01);
	lcm_dcs_write_seq_static(ctx,0x30,0x45);
	lcm_dcs_write_seq_static(ctx,0x31,0xBD);
	lcm_dcs_write_seq_static(ctx,0x34,0xF9);
	lcm_dcs_write_seq_static(ctx,0x35,0x31);
	lcm_dcs_write_seq_static(ctx,0x36,0x05);
	lcm_dcs_write_seq_static(ctx,0x37,0xF4);
	lcm_dcs_write_seq_static(ctx,0x38,0x35);
	lcm_dcs_write_seq_static(ctx,0x39,0x01);
	lcm_dcs_write_seq_static(ctx,0x3A,0x45);

	lcm_dcs_write_seq_static(ctx,0xFF,0x2C);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);

	lcm_dcs_write_seq_static(ctx,0xFF,0xE0);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x35,0x82);

	lcm_dcs_write_seq_static(ctx,0xFF,0xF0);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x1C,0x01);
	lcm_dcs_write_seq_static(ctx,0x33,0x01);
	lcm_dcs_write_seq_static(ctx,0x5A,0x00);
	lcm_dcs_write_seq_static(ctx,0xD2,0x52);

	lcm_dcs_write_seq_static(ctx,0xFF,0xD0);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x53,0x22);

	lcm_dcs_write_seq_static(ctx,0x54,0x02);
	lcm_dcs_write_seq_static(ctx,0xFF,0xC0);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x9C,0x11);
	lcm_dcs_write_seq_static(ctx,0x9D,0x11);

	lcm_dcs_write_seq_static(ctx,0xFF,0x2B);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0xB7,0x1A);
	lcm_dcs_write_seq_static(ctx,0xB8,0x1D);
	lcm_dcs_write_seq_static(ctx,0xC0,0x01);

	lcm_dcs_write_seq_static(ctx,0xFF,0x10);
	lcm_dcs_write_seq_static(ctx,0x11);
	mdelay(120);
	lcm_dcs_write_seq_static(ctx,0x29);
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

	pr_info("%s\n", __func__);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(1000);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);	



	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("NT36672C %s\n", __func__);
	if (ctx->prepared)
		return 0;


	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
    udelay(5000);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

#if defined(CONFIG_PRIZE_LCD_BIAS)
	display_bias_enable_v(5800);
#endif

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	pr_info("%s-\n", __func__);
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



static const struct drm_display_mode default_mode = {

	.clock       = PCLK_IN_KHZ_60HZ,
	.hdisplay    = PANEL_WIDTH,
	.hsync_start = PANEL_WIDTH  + HFP,
	.hsync_end   = PANEL_WIDTH  + HFP + HSA,
	.htotal      = PANEL_WIDTH  + HFP + HSA + HBP,  // 1224
	.vdisplay    = PANEL_HEIGHT,
	.vsync_start = PANEL_HEIGHT + MODE_0_VFP,
	.vsync_end   = PANEL_HEIGHT + MODE_0_VFP + VSA,
	.vtotal      = PANEL_HEIGHT + MODE_0_VFP + VSA + VBP,  //3612
	.vrefresh    = MODE_0_FPS,
};
//465
static const struct drm_display_mode performance_mode = {

	.clock       = PCLK_IN_KHZ_90HZ,
	.hdisplay    = PANEL_WIDTH,
	.hsync_start = PANEL_WIDTH  + HFP,
	.hsync_end   = PANEL_WIDTH  + HFP + HSA,
	.htotal      = PANEL_WIDTH  + HFP + HSA + HBP,   // 1224
	.vdisplay    = PANEL_HEIGHT,
	.vsync_start = PANEL_HEIGHT + MODE_1_VFP,
	.vsync_end   = PANEL_HEIGHT + MODE_1_VFP + VSA,
	.vtotal      = PANEL_HEIGHT + MODE_1_VFP + VSA + VBP,   //2482
	.vrefresh    = MODE_1_FPS,
};
//479

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 548,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.is_cphy = 1,
	.data_rate = 1096,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 548,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.is_cphy = 1,
	.data_rate = 1096,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
};

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0xf, 0xff};

	if (level > 255)
		level = 255;

	level = level * 4095 / 255;
	bl_tb0[1] = ((level >> 8) & 0xf);
	bl_tb0[2] = (level & 0xff);

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;

	if (mode == 0)
		ext->params = &ext_params;
	else if (mode == 1)
		ext->params = &ext_params_90hz;
	else
		ret = 1;

	return ret;
}

static int mtk_panel_ext_param_get(struct mtk_panel_params *ext_para,
			 unsigned int mode)
{
	int ret = 0;

	if (mode == 0)
		ext_para = &ext_params;
	else if (mode == 1)
		ext_para = &ext_params_90hz;
	else
		ret = 1;

	return ret;

}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
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

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *           become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *          display the first valid frame after starting to receive
	 *          video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *           turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *             to power itself down completely
	 */
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
	struct drm_display_mode *mode2;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	mode2 = drm_mode_duplicate(panel->drm, &performance_mode);
	if (!mode2) {
		pr_notice("failed to add mode %ux%ux@%u\n",
			performance_mode.hdisplay,
			performance_mode.vdisplay,
			performance_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode2);

	panel->connector->display_info.width_mm = 70;
	panel->connector->display_info.height_mm = 152;

	return 1;
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
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	pr_info("%s+\n", __func__);
	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 3;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			 |MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

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
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	pr_info("%s-\n", __func__);

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
	{
		.compatible = "xm,nt36672c,cphy,vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {

			.name = "panel-xm-nt36672c-cphy-vdo",
			.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
		},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("tianma r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
