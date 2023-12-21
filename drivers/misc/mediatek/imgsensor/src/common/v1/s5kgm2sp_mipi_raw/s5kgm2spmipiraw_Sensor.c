/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5KGM2SPmipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6763
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5kgm2spmipiraw_Sensor.h"

/*===FEATURE SWITH===*/
 // #define FPTPDAFSUPPORT   //for pdaf switch
 // #define FANPENGTAO   //for debug log

 //#define NONCONTINUEMODE
/*===FEATURE SWITH===*/

/****************************Modify Following Strings for Debug****************************/
#define PFX "S5KGM2SP"
#define LOG_INF_NEW(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF LOG_INF_NEW
#define LOG_1 LOG_INF("S5KGM2SP,MIPI 4LANE\n")
#define SENSORDB LOG_INF
/****************************   Modify end    *******************************************/

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KGM2SP_SENSOR_ID,		//Sensor ID Value: 0x30C8//record sensor id defined in Kd_imgsensor.h

	.checksum_value = 0x52500dc0, //0x49c09f86,		//checksum value for Camera Auto Test

	.pre = {
		.pclk = 960000000,
		.linelength  = 9984,
		.framelength = 3186,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 960000000,
		.linelength  = 9984,
		.framelength = 3186,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 960000000,
		.linelength  = 9984,
		.framelength = 3186,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 646400000,
		.linelength  = 4224,
		.framelength = 1272,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 646400000,
		.linelength  = 9552,
		.framelength = 2250,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
  .custom1 = {
		.pclk = 560000000,
		.linelength  = 5088,
		.framelength = 3668,
		.startx= 0,
		.starty = 0,
		.grabwindow_width  = 2328,
		.grabwindow_height = 1752,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 264000000,
		.max_framerate = 300,
	},
  .custom2 = {
		.pclk = 560000000,
		.linelength  = 5088,
		.framelength = 3668,
		.startx= 0,
		.starty = 0,
		.grabwindow_width  = 2328,
		.grabwindow_height = 1752,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 264000000,
		.max_framerate = 300,
	},
  .custom3 = {
		.pclk = 560000000,
		.linelength  = 5088,
		.framelength = 3668,
		.startx= 0,
		.starty = 0,
		.grabwindow_width  = 2328,
		.grabwindow_height = 1752,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 264000000,
		.max_framerate = 300,
	},
  .custom4 = {
		.pclk = 560000000,
		.linelength  = 5088,
		.framelength = 3668,
		.startx= 0,
		.starty = 0,
		.grabwindow_width  = 2328,
		.grabwindow_height = 1752,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 264000000,
		.max_framerate = 300,
	},
  .custom5 = {
		.pclk = 560000000,
		.linelength  = 5088,
		.framelength = 3668,
		.startx= 0,
		.starty = 0,
		.grabwindow_width  = 2328,
		.grabwindow_height = 1752,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 264000000,
		.max_framerate = 300,
	},
	.min_gain = 73,
	.max_gain = 4096,
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 1,
	.gain_type = 0,

	.margin = 8,			//sensor framelength & shutter margin
	.min_shutter = 5,		//min shutter
	.max_frame_length = 0xFFFF,//REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num ,don't support Slow motion

	.cap_delay_frame = 3,		//enter capture delay frame num
	.pre_delay_frame = 3, 		//enter preview delay frame num
	.video_delay_frame = 3,		//enter video delay frame num
	.hs_video_delay_frame = 3,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,
    .custom4_delay_frame = 2,
    .custom5_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
  .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
  .mipi_settle_delay_mode = 1, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr, //sensor output first pixel color SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0x5a, 0xff}, //record sensor support all write id addr, only supprt 4must end with 0xff
  .i2c_speed = 400, // i2c read/write speed
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x200,					//current shutter
	.gain = 0x200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x5a,//record current sensor's i2c write id
};


/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = 
{
#if 1
 { 8000, 6000,	 0,  	0, 8000, 6000, 4000, 3000, 0,	0, 4000, 3000, 0, 0, 4000, 3000}, // Preview

#else
 { 8000, 6000,	 0,  	0, 8000, 6000, 2000, 1500, 0,	0, 2000, 1500, 0, 0, 2000, 1500}, // Preview
#endif
 { 8000, 6000,	 0,  	0, 8000, 6000, 4000, 3000, 0,	0, 4000, 3000, 0, 0, 4000, 3000}, // capture
 { 8000, 6000,	 0,  	0, 8000, 6000, 4000, 3000, 0,	0, 4000, 3000, 0, 0, 4000, 3000}, // video
 { 8000, 6000,	 160,  	840, 7680, 4320, 1280, 720, 0,	0, 1280, 720, 0, 0, 1280, 720}, //hight speed video
 { 8000, 6000,	 160,  	840, 7680, 4320, 1920, 1080, 0,	0, 1920, 1080, 0, 0, 1920, 1080}, // slim video
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[4] = {
	// Preview mode setting
	{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
	0x00, 0x2B, 0x1220, 0x0E9E, 0x01, 0x00, 0x0000, 0x0000,
	0x01, 0x30, 0x0274, 0x0BA0, 0x03, 0x00, 0x0000, 0x0000},   //0x03E8   0x05D0
	// Video mode setting
	{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
	0x00, 0x2B, 0x1220, 0x0E9E, 0x01, 0x00, 0x0000, 0x0000,
	0x01, 0x30, 0x0274, 0x0BA0, 0x03, 0x00, 0x0000, 0x0000},
	// Capture mode setting
	{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
	0x00, 0x2B, 0x1220, 0x0E9E, 0x01, 0x00, 0x0000, 0x0000,
	0x01, 0x30, 0x0274, 0x0BA0, 0x03, 0x00, 0x0000, 0x0000},
	// Custom1 for stereo
	{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
	0x00, 0x2B, 0x1220, 0x0E9E, 0x01, 0x00, 0x0000, 0x0000,
	0x01, 0x30, 0x0274, 0x0BA0, 0x03, 0x00, 0x0000, 0x0000},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX = 0,
    .i4OffsetY = 12,
    .i4PitchX = 16,
    .i4PitchY = 16,
    .i4PairNum = 16,
    .i4SubBlkW = 8,
    .i4SubBlkH = 2,
    .i4PosR = {{0,12},{8,12},{2,15},{10,15},{6,16},{14,16},{4,19},{12,19},{0,20},{8,20},{2,23},{10,23},{6,24},{14,24},{4,27},{12,27}},
    .i4PosL = {{1,12},{9,12},{3,15},{11,15},{7,16},{15,16},{5,19},{13,19},{1,20},{9,20},{3,23},{11,23},{7,24},{15,24},{5,27},{13,27}},
	.i4BlockNumX = 250,
	.i4BlockNumY = 186,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable(%d) \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, shutter & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
    reg_gain = gain/2;
	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	LOG_INF("set_gain %d \n", gain);
  //gain = 64 = 1x real gain.

	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

    reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0204, (reg_gain&0xFFFF));
	return gain;
}	/*	set_gain  */


static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror;
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor_8(0x0101, 0x00); //GR
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor_8(0x0101, 0x01); //R
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor_8(0x0101, 0x02); //B
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor_8(0x0101, 0x03); //GB
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
static void sensor_init(void)
{
  LOG_INF("E\n");
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0000, 0x0013);
  write_cmos_sensor(0x0000, 0x08D2);
  write_cmos_sensor(0x6010, 0x0001);
	mdelay(10);
  write_cmos_sensor(0x6214, 0xF9F3);
  write_cmos_sensor(0x0A02, 0x0074);
  
  write_cmos_sensor(0x0D00, 0x0101);  //VC
  write_cmos_sensor(0x0D02, 0x0101);  //VC
  write_cmos_sensor(0x0114, 0x0101);  //VC
  
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x60D0);
  write_cmos_sensor(0x6F12, 0x10B5);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x4249);
  write_cmos_sensor(0x6F12, 0x4348);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x18F9);
  write_cmos_sensor(0x6F12, 0x424C);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x4249);
  write_cmos_sensor(0x6F12, 0x6060);
  write_cmos_sensor(0x6F12, 0x4248);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x11F9);
  write_cmos_sensor(0x6F12, 0x2060);
  write_cmos_sensor(0x6F12, 0x0122);
  write_cmos_sensor(0x6F12, 0x4149);
  write_cmos_sensor(0x6F12, 0x4148);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x9BFA);
  write_cmos_sensor(0x6F12, 0x0122);
  write_cmos_sensor(0x6F12, 0x4049);
  write_cmos_sensor(0x6F12, 0x4148);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x96FA);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x4049);
  write_cmos_sensor(0x6F12, 0x4048);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x91FA);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3F49);
  write_cmos_sensor(0x6F12, 0x4048);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xFCF8);
  write_cmos_sensor(0x6F12, 0xE060);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3E49);
  write_cmos_sensor(0x6F12, 0x3F48);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xF6F8);
  write_cmos_sensor(0x6F12, 0xA060);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3D49);
  write_cmos_sensor(0x6F12, 0x3E48);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xF0F8);
  write_cmos_sensor(0x6F12, 0x2061);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3C49);
  write_cmos_sensor(0x6F12, 0x3D48);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xEAF8);
  write_cmos_sensor(0x6F12, 0x6061);
  write_cmos_sensor(0x6F12, 0x3C49);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x6F12, 0x0246);
  write_cmos_sensor(0x6F12, 0x0870);
  write_cmos_sensor(0x6F12, 0x3B49);
  write_cmos_sensor(0x6F12, 0x3B48);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x71FA);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3A49);
  write_cmos_sensor(0x6F12, 0x3B48);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x6CFA);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3A49);
  write_cmos_sensor(0x6F12, 0x3A48);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xD7F8);
  write_cmos_sensor(0x6F12, 0xA061);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3949);
  write_cmos_sensor(0x6F12, 0x3948);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xD1F8);
  write_cmos_sensor(0x6F12, 0xE061);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3849);
  write_cmos_sensor(0x6F12, 0x3848);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xCBF8);
  write_cmos_sensor(0x6F12, 0x2062);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3749);
  write_cmos_sensor(0x6F12, 0x3748);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xC5F8);
  write_cmos_sensor(0x6F12, 0xA062);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3649);
  write_cmos_sensor(0x6F12, 0x3648);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xBFF8);
  write_cmos_sensor(0x6F12, 0x6062);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3549);
  write_cmos_sensor(0x6F12, 0x3548);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xB9F8);
  write_cmos_sensor(0x6F12, 0xE062);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3449);
  write_cmos_sensor(0x6F12, 0x3448);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xB3F8);
  write_cmos_sensor(0x6F12, 0x2063);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3349);
  write_cmos_sensor(0x6F12, 0x3348);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xADF8);
  write_cmos_sensor(0x6F12, 0x6063);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3249);
  write_cmos_sensor(0x6F12, 0x3248);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xA7F8);
  write_cmos_sensor(0x6F12, 0xA063);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3149);
  write_cmos_sensor(0x6F12, 0x3148);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xA1F8);
  write_cmos_sensor(0x6F12, 0xE063);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3049);
  write_cmos_sensor(0x6F12, 0x3048);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x2BFA);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0x1040);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x61B8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5433);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xEDF5);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x6B80);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x53B7);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xE83D);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5523);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xE19F);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x557F);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xDA7F);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x6509);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x9CC3);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5865);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xF9F1);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x55B9);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x332D);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5939);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x87B5);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x599F);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x689B);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5F90);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x59C9);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x5017);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5A63);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x4F91);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5A17);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x52BD);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5B5D);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x03C1);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5C91);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xF635);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5D91);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x212D);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5D23);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xF889);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x6445);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x3AC7);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5AA9);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x41D5);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x6311);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xD15F);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5AFB);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xFE7F);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x5B2D);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x4927);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x62B3);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x5D7F);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xCF49);
  write_cmos_sensor(0x6F12, 0xCE48);
  write_cmos_sensor(0x6F12, 0xCF4A);
  write_cmos_sensor(0x6F12, 0xC1F8);
  write_cmos_sensor(0x6F12, 0xDC06);
  write_cmos_sensor(0x6F12, 0x101A);
  write_cmos_sensor(0x6F12, 0xA1F8);
  write_cmos_sensor(0x6F12, 0xE006);
  write_cmos_sensor(0x6F12, 0x7047);
  write_cmos_sensor(0x6F12, 0x2DE9);
  write_cmos_sensor(0x6F12, 0xF041);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xBEF9);
  write_cmos_sensor(0x6F12, 0xC94D);
  write_cmos_sensor(0x6F12, 0x10F0);
  write_cmos_sensor(0x6F12, 0x180F);
  write_cmos_sensor(0x6F12, 0x04D1);
  write_cmos_sensor(0x6F12, 0x95F8);
  write_cmos_sensor(0x6F12, 0x9510);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xBAF9);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xBDF9);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xC0F9);
  write_cmos_sensor(0x6F12, 0xC44E);
  write_cmos_sensor(0x6F12, 0x0124);
  write_cmos_sensor(0x6F12, 0xC44F);
  write_cmos_sensor(0x6F12, 0x05E0);
  write_cmos_sensor(0x6F12, 0x308B);
  write_cmos_sensor(0x6F12, 0x00B1);
  write_cmos_sensor(0x6F12, 0x3C80);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xBBF9);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xBEF9);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x6F12, 0xF5D0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xBFF9);
  write_cmos_sensor(0x6F12, 0xE864);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xC1F9);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xC4F9);
  write_cmos_sensor(0x6F12, 0xBC48);
  write_cmos_sensor(0x6F12, 0xBC49);
  write_cmos_sensor(0x6F12, 0x8089);
  write_cmos_sensor(0x6F12, 0xA1F8);
  write_cmos_sensor(0x6F12, 0x1802);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0xF081);
  write_cmos_sensor(0x6F12, 0xBA4B);
  write_cmos_sensor(0x6F12, 0x1847);
  write_cmos_sensor(0x6F12, 0x2DE9);
  write_cmos_sensor(0x6F12, 0xFC5F);
  write_cmos_sensor(0x6F12, 0x8346);
  write_cmos_sensor(0x6F12, 0xB948);
  write_cmos_sensor(0x6F12, 0x8A46);
  write_cmos_sensor(0x6F12, 0xDDE9);
  write_cmos_sensor(0x6F12, 0x0C56);
  write_cmos_sensor(0x6F12, 0x416B);
  write_cmos_sensor(0x6F12, 0x9146);
  write_cmos_sensor(0x6F12, 0x0C0C);
  write_cmos_sensor(0x6F12, 0x8FB2);
  write_cmos_sensor(0x6F12, 0x9846);
  write_cmos_sensor(0x6F12, 0x0022);
  write_cmos_sensor(0x6F12, 0x3946);
  write_cmos_sensor(0x6F12, 0x2046);
  write_cmos_sensor(0x6F12, 0xFFF7);
  write_cmos_sensor(0x6F12, 0x8DFD);
  write_cmos_sensor(0x6F12, 0xCDE9);
  write_cmos_sensor(0x6F12, 0x0056);
  write_cmos_sensor(0x6F12, 0x4346);
  write_cmos_sensor(0x6F12, 0x4A46);
  write_cmos_sensor(0x6F12, 0x5146);
  write_cmos_sensor(0x6F12, 0x5846);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xA7F9);
  write_cmos_sensor(0x6F12, 0x8046);
  write_cmos_sensor(0x6F12, 0x0122);
  write_cmos_sensor(0x6F12, 0x3946);
  write_cmos_sensor(0x6F12, 0x2046);
  write_cmos_sensor(0x6F12, 0xFFF7);
  write_cmos_sensor(0x6F12, 0x7FFD);
  write_cmos_sensor(0x6F12, 0xAC4E);
  write_cmos_sensor(0x6F12, 0x96F8);
  write_cmos_sensor(0x6F12, 0xE802);
  write_cmos_sensor(0x6F12, 0x18B9);
  write_cmos_sensor(0x6F12, 0x96F8);
  write_cmos_sensor(0x6F12, 0xC002);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x6F12, 0x6FD0);
  write_cmos_sensor(0x6F12, 0x96F8);
  write_cmos_sensor(0x6F12, 0x0E01);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x6F12, 0x6BD1);
  write_cmos_sensor(0x6F12, 0x0025);
  write_cmos_sensor(0x6F12, 0x4FF4);
  write_cmos_sensor(0x6F12, 0x1A47);
  write_cmos_sensor(0x6F12, 0x2C46);
  write_cmos_sensor(0x6F12, 0x3A46);
  write_cmos_sensor(0x6F12, 0x1821);
  write_cmos_sensor(0x6F12, 0xA448);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x91F9);
  write_cmos_sensor(0x6F12, 0xA248);
  write_cmos_sensor(0x6F12, 0x3A46);
  write_cmos_sensor(0x6F12, 0x7821);
  write_cmos_sensor(0x6F12, 0x4030);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x90F9);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x6F12, 0x06EB);
  write_cmos_sensor(0x6F12, 0x8001);
  write_cmos_sensor(0x6F12, 0x401C);
  write_cmos_sensor(0x6F12, 0xD1F8);
  write_cmos_sensor(0x6F12, 0x6C12);
  write_cmos_sensor(0x6F12, 0x1028);
  write_cmos_sensor(0x6F12, 0x0D44);
  write_cmos_sensor(0x6F12, 0xF7DB);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x6F12, 0x06EB);
  write_cmos_sensor(0x6F12, 0x4001);
  write_cmos_sensor(0x6F12, 0x401C);
  write_cmos_sensor(0x6F12, 0xB1F8);
  write_cmos_sensor(0x6F12, 0xAC12);
  write_cmos_sensor(0x6F12, 0x0428);
  write_cmos_sensor(0x6F12, 0x0C44);
  write_cmos_sensor(0x6F12, 0xF7DB);
  write_cmos_sensor(0x6F12, 0x04B9);
  write_cmos_sensor(0x6F12, 0x0124);
  write_cmos_sensor(0x6F12, 0xA000);
  write_cmos_sensor(0x6F12, 0x95FB);
  write_cmos_sensor(0x6F12, 0xF0F3);
  write_cmos_sensor(0x6F12, 0x9548);
  write_cmos_sensor(0x6F12, 0x954C);
  write_cmos_sensor(0x6F12, 0x018B);
  write_cmos_sensor(0x6F12, 0x4A05);
  write_cmos_sensor(0x6F12, 0x00D5);
  write_cmos_sensor(0x6F12, 0x2143);
  write_cmos_sensor(0x6F12, 0x428B);
  write_cmos_sensor(0x6F12, 0x5505);
  write_cmos_sensor(0x6F12, 0x00D5);
  write_cmos_sensor(0x6F12, 0x2243);
  write_cmos_sensor(0x6F12, 0x1944);
  write_cmos_sensor(0x6F12, 0x1A44);
  write_cmos_sensor(0x6F12, 0x96F8);
  write_cmos_sensor(0x6F12, 0x4631);
  write_cmos_sensor(0x6F12, 0xC1F3);
  write_cmos_sensor(0x6F12, 0x0A01);
  write_cmos_sensor(0x6F12, 0xC2F3);
  write_cmos_sensor(0x6F12, 0x0A02);
  write_cmos_sensor(0x6F12, 0xC3B9);
  write_cmos_sensor(0x6F12, 0x8D4B);
  write_cmos_sensor(0x6F12, 0x1980);
  write_cmos_sensor(0x6F12, 0x9B1C);
  write_cmos_sensor(0x6F12, 0x1A80);
  write_cmos_sensor(0x6F12, 0x8B4C);
  write_cmos_sensor(0x6F12, 0x838B);
  write_cmos_sensor(0x6F12, 0x241D);
  write_cmos_sensor(0x6F12, 0x2380);
  write_cmos_sensor(0x6F12, 0xA41C);
  write_cmos_sensor(0x6F12, 0xC38B);
  write_cmos_sensor(0x6F12, 0x2380);
  write_cmos_sensor(0x6F12, 0xA41C);
  write_cmos_sensor(0x6F12, 0x038C);
  write_cmos_sensor(0x6F12, 0x2380);
  write_cmos_sensor(0x6F12, 0xA41C);
  write_cmos_sensor(0x6F12, 0x438C);
  write_cmos_sensor(0x6F12, 0x2380);
  write_cmos_sensor(0x6F12, 0xA41C);
  write_cmos_sensor(0x6F12, 0x90F8);
  write_cmos_sensor(0x6F12, 0x2430);
  write_cmos_sensor(0x6F12, 0x2380);
  write_cmos_sensor(0x6F12, 0xA41C);
  write_cmos_sensor(0x6F12, 0x90F8);
  write_cmos_sensor(0x6F12, 0x2530);
  write_cmos_sensor(0x6F12, 0x2380);
  write_cmos_sensor(0x6F12, 0x96F8);
  write_cmos_sensor(0x6F12, 0x5031);
  write_cmos_sensor(0x6F12, 0xB3B9);
  write_cmos_sensor(0x6F12, 0x804B);
  write_cmos_sensor(0x6F12, 0x1980);
  write_cmos_sensor(0x6F12, 0x991C);
  write_cmos_sensor(0x6F12, 0x0A80);
  write_cmos_sensor(0x6F12, 0x1A1D);
  write_cmos_sensor(0x6F12, 0x30F8);
  write_cmos_sensor(0x6F12, 0x1C1F);
  write_cmos_sensor(0x6F12, 0x1180);
  write_cmos_sensor(0x6F12, 0x921C);
  write_cmos_sensor(0x6F12, 0x4188);
  write_cmos_sensor(0x6F12, 0x1180);
  write_cmos_sensor(0x6F12, 0x921C);
  write_cmos_sensor(0x6F12, 0x8188);
  write_cmos_sensor(0x6F12, 0x1180);
  write_cmos_sensor(0x6F12, 0x921C);
  write_cmos_sensor(0x6F12, 0xC188);
  write_cmos_sensor(0x6F12, 0x1180);
  write_cmos_sensor(0x6F12, 0x921C);
  write_cmos_sensor(0x6F12, 0x017A);
  write_cmos_sensor(0x6F12, 0x1180);
  write_cmos_sensor(0x6F12, 0x911C);
  write_cmos_sensor(0x6F12, 0x407A);
  write_cmos_sensor(0x6F12, 0x0880);
  write_cmos_sensor(0x6F12, 0x4046);
  write_cmos_sensor(0x6F12, 0xBDE8);
  write_cmos_sensor(0x6F12, 0xFC9F);
  write_cmos_sensor(0x6F12, 0x38B5);
  write_cmos_sensor(0x6F12, 0x664C);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x6F12, 0x0090);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0xD822);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0xD912);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0xDB02);
  write_cmos_sensor(0x6F12, 0x6B46);
  write_cmos_sensor(0x6F12, 0x04F5);
  write_cmos_sensor(0x6F12, 0x3674);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x26F9);
  write_cmos_sensor(0x6F12, 0xE178);
  write_cmos_sensor(0x6F12, 0x0844);
  write_cmos_sensor(0x6F12, 0xA178);
  write_cmos_sensor(0x6F12, 0xA4F5);
  write_cmos_sensor(0x6F12, 0x3674);
  write_cmos_sensor(0x6F12, 0x0D18);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x23F9);
  write_cmos_sensor(0x6F12, 0x10B1);
  write_cmos_sensor(0x6F12, 0x6848);
  write_cmos_sensor(0x6F12, 0x0078);
  write_cmos_sensor(0x6F12, 0x00B1);
  write_cmos_sensor(0x6F12, 0x0120);
  write_cmos_sensor(0x6F12, 0x6049);
  write_cmos_sensor(0x6F12, 0x81F8);
  write_cmos_sensor(0x6F12, 0xDC00);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0x7725);
  write_cmos_sensor(0x6F12, 0x02B1);
  write_cmos_sensor(0x6F12, 0x00B1);
  write_cmos_sensor(0x6F12, 0x0120);
  write_cmos_sensor(0x6F12, 0x81F8);
  write_cmos_sensor(0x6F12, 0x0E01);
  write_cmos_sensor(0x6F12, 0x6249);
  write_cmos_sensor(0x6F12, 0x0978);
  write_cmos_sensor(0x6F12, 0x0229);
  write_cmos_sensor(0x6F12, 0x11D1);
  write_cmos_sensor(0x6F12, 0x80B9);
  write_cmos_sensor(0x6F12, 0xB4F8);
  write_cmos_sensor(0x6F12, 0x0203);
  write_cmos_sensor(0x6F12, 0xB4F8);
  write_cmos_sensor(0x6F12, 0xCA20);
  write_cmos_sensor(0x6F12, 0x411E);
  write_cmos_sensor(0x6F12, 0x0A44);
  write_cmos_sensor(0x6F12, 0xB2FB);
  write_cmos_sensor(0x6F12, 0xF0F2);
  write_cmos_sensor(0x6F12, 0x84F8);
  write_cmos_sensor(0x6F12, 0x2727);
  write_cmos_sensor(0x6F12, 0xB4F8);
  write_cmos_sensor(0x6F12, 0xCC20);
  write_cmos_sensor(0x6F12, 0x1144);
  write_cmos_sensor(0x6F12, 0xB1FB);
  write_cmos_sensor(0x6F12, 0xF0F0);
  write_cmos_sensor(0x6F12, 0x84F8);
  write_cmos_sensor(0x6F12, 0x2807);
  write_cmos_sensor(0x6F12, 0xB4F8);
  write_cmos_sensor(0x6F12, 0xBE00);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0xDA12);
  write_cmos_sensor(0x6F12, 0x401A);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x6F12, 0x00DC);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x6F12, 0x84F8);
  write_cmos_sensor(0x6F12, 0xDC02);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xF7F8);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0xDC12);
  write_cmos_sensor(0x6F12, 0x0844);
  write_cmos_sensor(0x6F12, 0x5149);
  write_cmos_sensor(0x6F12, 0x91F8);
  write_cmos_sensor(0x6F12, 0x9612);
  write_cmos_sensor(0x6F12, 0x0844);
  write_cmos_sensor(0x6F12, 0x5049);
  write_cmos_sensor(0x6F12, 0xC0B2);
  write_cmos_sensor(0x6F12, 0x84F8);
  write_cmos_sensor(0x6F12, 0xDC02);
  write_cmos_sensor(0x6F12, 0x91F8);
  write_cmos_sensor(0x6F12, 0x7320);
  write_cmos_sensor(0x6F12, 0x42B1);
  write_cmos_sensor(0x6F12, 0x91F8);
  write_cmos_sensor(0x6F12, 0x7110);
  write_cmos_sensor(0x6F12, 0x0429);
  write_cmos_sensor(0x6F12, 0x04D1);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x0101);
  write_cmos_sensor(0x6F12, 0x0844);
  write_cmos_sensor(0x6F12, 0x84F8);
  write_cmos_sensor(0x6F12, 0xDC02);
  write_cmos_sensor(0x6F12, 0xC0B2);
  write_cmos_sensor(0x6F12, 0x2844);
  write_cmos_sensor(0x6F12, 0x38BD);
  write_cmos_sensor(0x6F12, 0x10B5);
  write_cmos_sensor(0x6F12, 0x3D4C);
  write_cmos_sensor(0x6F12, 0x404A);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0x0E01);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0x4611);
  write_cmos_sensor(0x6F12, 0x0143);
  write_cmos_sensor(0x6F12, 0x4E3A);
  write_cmos_sensor(0x6F12, 0x1180);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0x5011);
  write_cmos_sensor(0x6F12, 0x3C4A);
  write_cmos_sensor(0x6F12, 0x0143);
  write_cmos_sensor(0x6F12, 0x4E3A);
  write_cmos_sensor(0x6F12, 0x1180);
  write_cmos_sensor(0x6F12, 0x04F5);
  write_cmos_sensor(0x6F12, 0x8774);
  write_cmos_sensor(0x6F12, 0x0028);
  write_cmos_sensor(0x6F12, 0x53D1);
  write_cmos_sensor(0x6F12, 0x2046);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xCBF8);
  write_cmos_sensor(0x6F12, 0x2046);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0xCDF8);
  write_cmos_sensor(0x6F12, 0x3A4A);
  write_cmos_sensor(0x6F12, 0x608A);
  write_cmos_sensor(0x6F12, 0x314B);
  write_cmos_sensor(0x6F12, 0x9188);
  write_cmos_sensor(0x6F12, 0x401A);
  write_cmos_sensor(0x6F12, 0x80B2);
  write_cmos_sensor(0x6F12, 0x6082);
  write_cmos_sensor(0x6F12, 0x998D);
  write_cmos_sensor(0x6F12, 0x09B1);
  write_cmos_sensor(0x6F12, 0x0846);
  write_cmos_sensor(0x6F12, 0x04E0);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0x3610);
  write_cmos_sensor(0x6F12, 0x0229);
  write_cmos_sensor(0x6F12, 0x00D1);
  write_cmos_sensor(0x6F12, 0x4008);
  write_cmos_sensor(0x6F12, 0xE085);
  write_cmos_sensor(0x6F12, 0x188E);
  write_cmos_sensor(0x6F12, 0x58B9);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0x3600);
  write_cmos_sensor(0x6F12, 0x0228);
  write_cmos_sensor(0x6F12, 0x03D0);
  write_cmos_sensor(0x6F12, 0x0128);
  write_cmos_sensor(0x6F12, 0x608A);
  write_cmos_sensor(0x6F12, 0x03D0);
  write_cmos_sensor(0x6F12, 0x03E0);
  write_cmos_sensor(0x6F12, 0x608A);
  write_cmos_sensor(0x6F12, 0x8008);
  write_cmos_sensor(0x6F12, 0x00E0);
  write_cmos_sensor(0x6F12, 0x4008);
  write_cmos_sensor(0x6F12, 0x2086);
  write_cmos_sensor(0x6F12, 0x6088);
  write_cmos_sensor(0x6F12, 0x1188);
  write_cmos_sensor(0x6F12, 0x0844);
  write_cmos_sensor(0x6F12, 0x6080);
  write_cmos_sensor(0x6F12, 0xE088);
  write_cmos_sensor(0x6F12, 0x1188);
  write_cmos_sensor(0x6F12, 0x401A);
  write_cmos_sensor(0x6F12, 0xE080);
  write_cmos_sensor(0x6F12, 0xA08A);
  write_cmos_sensor(0x6F12, 0xD188);
  write_cmos_sensor(0x6F12, 0x401A);
  write_cmos_sensor(0x6F12, 0x80B2);
  write_cmos_sensor(0x6F12, 0xA082);
  write_cmos_sensor(0x6F12, 0x598D);
  write_cmos_sensor(0x6F12, 0x09B1);
  write_cmos_sensor(0x6F12, 0x0846);
  write_cmos_sensor(0x6F12, 0x04E0);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0x3610);
  write_cmos_sensor(0x6F12, 0x0229);
  write_cmos_sensor(0x6F12, 0x00D1);
  write_cmos_sensor(0x6F12, 0x4000);
  write_cmos_sensor(0x6F12, 0x6086);
  write_cmos_sensor(0x6F12, 0xD88D);
  write_cmos_sensor(0x6F12, 0x58B9);
  write_cmos_sensor(0x6F12, 0x94F8);
  write_cmos_sensor(0x6F12, 0x3600);
  write_cmos_sensor(0x6F12, 0x0228);
  write_cmos_sensor(0x6F12, 0x03D0);
  write_cmos_sensor(0x6F12, 0x0128);
  write_cmos_sensor(0x6F12, 0xA08A);
  write_cmos_sensor(0x6F12, 0x03D0);
  write_cmos_sensor(0x6F12, 0x03E0);
  write_cmos_sensor(0x6F12, 0xA08A);
  write_cmos_sensor(0x6F12, 0x8000);
  write_cmos_sensor(0x6F12, 0x00E0);
  write_cmos_sensor(0x6F12, 0x4000);
  write_cmos_sensor(0x6F12, 0xA086);
  write_cmos_sensor(0x6F12, 0xA088);
  write_cmos_sensor(0x6F12, 0x5188);
  write_cmos_sensor(0x6F12, 0x0844);
  write_cmos_sensor(0x6F12, 0xA080);
  write_cmos_sensor(0x6F12, 0x2089);
  write_cmos_sensor(0x6F12, 0x5188);
  write_cmos_sensor(0x6F12, 0x401A);
  write_cmos_sensor(0x6F12, 0x2081);
  write_cmos_sensor(0x6F12, 0x10BD);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x66F4);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x3CC0);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x6C00);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x6A80);
  write_cmos_sensor(0x6F12, 0x4000);
  write_cmos_sensor(0x6F12, 0x6C02);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x0DC0);
  write_cmos_sensor(0x6F12, 0x4000);
  write_cmos_sensor(0x6F12, 0x6000);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x3051);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x6B80);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x4AC0);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x4D2C);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x34C0);
  write_cmos_sensor(0x6F12, 0xFFFF);
  write_cmos_sensor(0x6F12, 0xF800);
  write_cmos_sensor(0x6F12, 0x4000);
  write_cmos_sensor(0x6F12, 0xAE4E);
  write_cmos_sensor(0x6F12, 0x4000);
  write_cmos_sensor(0x6F12, 0xAF4E);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x33B0);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x2A80);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x0E00);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x4680);
  write_cmos_sensor(0x6F12, 0x2000);
  write_cmos_sensor(0x6F12, 0x6C00);
  write_cmos_sensor(0x6F12, 0x43F2);
  write_cmos_sensor(0x6F12, 0x510C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x010C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x46F2);
  write_cmos_sensor(0x6F12, 0xDD7C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x45F2);
  write_cmos_sensor(0x6F12, 0xF57C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x45F6);
  write_cmos_sensor(0x6F12, 0x735C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x43F6);
  write_cmos_sensor(0x6F12, 0x2D0C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x44F6);
  write_cmos_sensor(0x6F12, 0x934C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x010C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x47F2);
  write_cmos_sensor(0x6F12, 0x8D4C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x47F6);
  write_cmos_sensor(0x6F12, 0xD10C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x43F6);
  write_cmos_sensor(0x6F12, 0x390C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x45F6);
  write_cmos_sensor(0x6F12, 0x555C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x4DF2);
  write_cmos_sensor(0x6F12, 0x5F1C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x4BF6);
  write_cmos_sensor(0x6F12, 0x1D2C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x4BF6);
  write_cmos_sensor(0x6F12, 0x572C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x4FF6);
  write_cmos_sensor(0x6F12, 0x335C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x47F2);
  write_cmos_sensor(0x6F12, 0x0B0C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x43F6);
  write_cmos_sensor(0x6F12, 0x8F2C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x010C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x49F6);
  write_cmos_sensor(0x6F12, 0x911C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x49F6);
  write_cmos_sensor(0x6F12, 0x613C);
  write_cmos_sensor(0x6F12, 0xC0F2);
  write_cmos_sensor(0x6F12, 0x000C);
  write_cmos_sensor(0x6F12, 0x6047);
  write_cmos_sensor(0x6F12, 0x08D2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0009);
  write_cmos_sensor(0x6F12, 0xFFFF);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1EA6);
  write_cmos_sensor(0x6F12, 0x4C09);
  write_cmos_sensor(0xF44C, 0x0A0C);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x11E6);
  write_cmos_sensor(0x6F12, 0x000D);
  write_cmos_sensor(0x6F12, 0x0009);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x11EE);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x000F);
  write_cmos_sensor(0x602A, 0x122E);
  write_cmos_sensor(0x6F12, 0x001E);
  write_cmos_sensor(0x6F12, 0x0014);
  write_cmos_sensor(0x6F12, 0x0012);
  write_cmos_sensor(0x602A, 0x12D6);
  write_cmos_sensor(0x6F12, 0xFFF9);
  write_cmos_sensor(0x602A, 0x12E6);
  write_cmos_sensor(0x6F12, 0x000F);
  write_cmos_sensor(0x602A, 0x146E);
  write_cmos_sensor(0x6F12, 0x002F);
  write_cmos_sensor(0x6F12, 0x0034);
  write_cmos_sensor(0x6F12, 0x0036);
  write_cmos_sensor(0x602A, 0x1EA4);
  write_cmos_sensor(0x6F12, 0x0018);
  write_cmos_sensor(0x602A, 0x0E38);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x602A, 0x1E92);
  write_cmos_sensor(0x6F12, 0x0907);
  write_cmos_sensor(0x6F12, 0x070B);
  write_cmos_sensor(0x602A, 0x1E9E);
  write_cmos_sensor(0x6F12, 0x0303);
  write_cmos_sensor(0x6F12, 0x0305);
  write_cmos_sensor(0x602A, 0x1E9A);
  write_cmos_sensor(0x6F12, 0x0B0B);
  write_cmos_sensor(0x6F12, 0x0715);
  write_cmos_sensor(0x602A, 0x1E96);
  write_cmos_sensor(0x6F12, 0x0F0C);
  write_cmos_sensor(0x6F12, 0x0C04);
  write_cmos_sensor(0x602A, 0x1EAE);
  write_cmos_sensor(0x6F12, 0x3D68);
  write_cmos_sensor(0x602A, 0x2440);
  write_cmos_sensor(0x6F12, 0x0080);
  write_cmos_sensor(0x602A, 0x243E);
  write_cmos_sensor(0x6F12, 0x006F);
  write_cmos_sensor(0x602A, 0x243C);
  write_cmos_sensor(0x6F12, 0x44C2);
  write_cmos_sensor(0xF412, 0x0000);
  write_cmos_sensor(0xF45A, 0x0FFE);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x10B0);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x10B4);
  write_cmos_sensor(0x6F12, 0x003F);
  write_cmos_sensor(0x602A, 0x10BA);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x602A, 0x10C4);
  write_cmos_sensor(0x6F12, 0xF480);
  write_cmos_sensor(0x602A, 0x3B20);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x3B26);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x3B22);
  write_cmos_sensor(0x6F12, 0x0080);
  write_cmos_sensor(0x9C14, 0x0000);
  write_cmos_sensor(0x9C1A, 0x0000);
  write_cmos_sensor(0x9C06, 0x0303);
  write_cmos_sensor(0x9C1C, 0x0020);
  write_cmos_sensor(0x9C1E, 0x0400);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x239A);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2398);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2396);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2394);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2392);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2386);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2384);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2382);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2380);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x237E);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x9C24, 0x0100);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x3630);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1EB2);
  write_cmos_sensor(0x6F12, 0x6A2D);
  write_cmos_sensor(0x602A, 0x117A);
  write_cmos_sensor(0x6F12, 0x0088);
  write_cmos_sensor(0x6F12, 0x0098);
  write_cmos_sensor(0x602A, 0x29F0);
  write_cmos_sensor(0x6F12, 0x0000);    //pd0 0x0001 0x0000
  write_cmos_sensor(0x602A, 0x1076);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x030E, 0x0003);
  write_cmos_sensor(0x0312, 0x0000);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x0202, 0x0020);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1170);
  write_cmos_sensor(0x6F12, 0x0005);
  write_cmos_sensor(0x602A, 0x33FA);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x33F8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x0B06, 0x0101);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1094);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x602A, 0x3020);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x0FEA, 0x0EC0);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1E80);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x2516);
  write_cmos_sensor(0x6F12, 0x01C0);
  write_cmos_sensor(0x6F12, 0x01C2);
  write_cmos_sensor(0x6F12, 0x02C8);
  write_cmos_sensor(0x6F12, 0x02CA);
  write_cmos_sensor(0x6F12, 0x02D0);
  write_cmos_sensor(0x6F12, 0x02D2);
  write_cmos_sensor(0x6F12, 0x01D8);
  write_cmos_sensor(0x6F12, 0x01DA);
  write_cmos_sensor(0x6F12, 0x01E0);
  write_cmos_sensor(0x6F12, 0x01E2);
  write_cmos_sensor(0x6F12, 0x02E8);
  write_cmos_sensor(0x6F12, 0x02EA);
  write_cmos_sensor(0x6F12, 0x02F0);
  write_cmos_sensor(0x6F12, 0x02F2);
  write_cmos_sensor(0x6F12, 0x01F8);
  write_cmos_sensor(0x6F12, 0x01FA);
  write_cmos_sensor(0x6F12, 0x01C0);
  write_cmos_sensor(0x6F12, 0x01C2);
  write_cmos_sensor(0x6F12, 0x02C8);
  write_cmos_sensor(0x6F12, 0x02CA);
  write_cmos_sensor(0x6F12, 0x02D0);
  write_cmos_sensor(0x6F12, 0x02D2);
  write_cmos_sensor(0x6F12, 0x01D8);
  write_cmos_sensor(0x6F12, 0x01DA);
  write_cmos_sensor(0x6F12, 0x01E0);
  write_cmos_sensor(0x6F12, 0x01E2);
  write_cmos_sensor(0x6F12, 0x02E8);
  write_cmos_sensor(0x6F12, 0x02EA);
  write_cmos_sensor(0x6F12, 0x02F0);
  write_cmos_sensor(0x6F12, 0x02F2);
  write_cmos_sensor(0x6F12, 0x01F8);
  write_cmos_sensor(0x6F12, 0x01FA);
  write_cmos_sensor(0x602A, 0x2596);
  write_cmos_sensor(0x6F12, 0x01C0);
  write_cmos_sensor(0x6F12, 0x01C2);
  write_cmos_sensor(0x6F12, 0x01C8);
  write_cmos_sensor(0x6F12, 0x01CA);
  write_cmos_sensor(0x6F12, 0x01D0);
  write_cmos_sensor(0x6F12, 0x01D2);
  write_cmos_sensor(0x6F12, 0x01D8);
  write_cmos_sensor(0x6F12, 0x01DA);
  write_cmos_sensor(0x6F12, 0x01E0);
  write_cmos_sensor(0x6F12, 0x01E2);
  write_cmos_sensor(0x6F12, 0x01E8);
  write_cmos_sensor(0x6F12, 0x01EA);
  write_cmos_sensor(0x6F12, 0x01F0);
  write_cmos_sensor(0x6F12, 0x01F2);
  write_cmos_sensor(0x6F12, 0x01F8);
  write_cmos_sensor(0x6F12, 0x01FA);
  write_cmos_sensor(0x6F12, 0x01C0);
  write_cmos_sensor(0x6F12, 0x01C2);
  write_cmos_sensor(0x6F12, 0x01C8);
  write_cmos_sensor(0x6F12, 0x01CA);
  write_cmos_sensor(0x6F12, 0x01D0);
  write_cmos_sensor(0x6F12, 0x01D2);
  write_cmos_sensor(0x6F12, 0x01D8);
  write_cmos_sensor(0x6F12, 0x01DA);
  write_cmos_sensor(0x6F12, 0x01E0);
  write_cmos_sensor(0x6F12, 0x01E2);
  write_cmos_sensor(0x6F12, 0x01E8);
  write_cmos_sensor(0x6F12, 0x01EA);
  write_cmos_sensor(0x6F12, 0x01F0);
  write_cmos_sensor(0x6F12, 0x01F2);
  write_cmos_sensor(0x6F12, 0x01F8);
  write_cmos_sensor(0x6F12, 0x01FA);
  write_cmos_sensor(0x602A, 0x0E7C);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x0118, 0x0004);
  write_cmos_sensor(0x010C, 0x0000);
  write_cmos_sensor(0x011A, 0x0001);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1070);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x0EFC);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x0ED4);
  write_cmos_sensor(0x6F12, 0x0003);
  write_cmos_sensor(0x602A, 0x0F00);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x6F12, 0x0800);
  write_cmos_sensor(0x602A, 0x2634);
  write_cmos_sensor(0x6F12, 0x0500);
  write_cmos_sensor(0x602A, 0x34D8);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x602A, 0x34D0);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x39A8);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x39B0);
  write_cmos_sensor(0x6F12, 0x0014);
  write_cmos_sensor(0x6F12, 0x001E);
  write_cmos_sensor(0x6F12, 0x0014);
  write_cmos_sensor(0x6F12, 0x001E);
  write_cmos_sensor(0x6F12, 0x0005);
  write_cmos_sensor(0x602A, 0x39C2);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x3992);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x602A, 0x39CC);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x37EC);
  write_cmos_sensor(0x6F12, 0x0023);
  write_cmos_sensor(0x6F12, 0x0023);
  write_cmos_sensor(0x602A, 0x37FA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x3806);
  write_cmos_sensor(0x6F12, 0x0023);
  write_cmos_sensor(0x6F12, 0x0023);
  write_cmos_sensor(0x602A, 0x3814);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x39DA);
  write_cmos_sensor(0x6F12, 0x104D);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0B00);
  write_cmos_sensor(0x6F12, 0x0B80);
  write_cmos_sensor(0x6F12, 0x2274);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0812);
  write_cmos_sensor(0x6F12, 0x1F2D);
  write_cmos_sensor(0x6F12, 0x3944);
  write_cmos_sensor(0x6F12, 0x4E56);
  write_cmos_sensor(0x6F12, 0x010A);
  write_cmos_sensor(0x6F12, 0x1521);
  write_cmos_sensor(0x6F12, 0x2E3E);
  write_cmos_sensor(0x6F12, 0x4C57);
  write_cmos_sensor(0x6F12, 0x626B);
  write_cmos_sensor(0x6F12, 0x0E18);
  write_cmos_sensor(0x6F12, 0x2330);
  write_cmos_sensor(0x6F12, 0x4050);
  write_cmos_sensor(0x6F12, 0x606D);
  write_cmos_sensor(0x6F12, 0x7882);
  write_cmos_sensor(0x6F12, 0x1E2A);
  write_cmos_sensor(0x6F12, 0x3645);
  write_cmos_sensor(0x6F12, 0x5669);
  write_cmos_sensor(0x6F12, 0x7A89);
  write_cmos_sensor(0x6F12, 0x96A1);
  write_cmos_sensor(0x6F12, 0x313E);
  write_cmos_sensor(0x6F12, 0x4C5D);
  write_cmos_sensor(0x6F12, 0x7085);
  write_cmos_sensor(0x6F12, 0x98A8);
  write_cmos_sensor(0x6F12, 0xB7C4);
  write_cmos_sensor(0x6F12, 0x4250);
  write_cmos_sensor(0x6F12, 0x5F71);
  write_cmos_sensor(0x6F12, 0x869D);
  write_cmos_sensor(0x6F12, 0xB2C4);
  write_cmos_sensor(0x6F12, 0xD4E2);
  write_cmos_sensor(0x6F12, 0x4F5E);
  write_cmos_sensor(0x6F12, 0x6F82);
  write_cmos_sensor(0x6F12, 0x98B1);
  write_cmos_sensor(0x6F12, 0xC7DA);
  write_cmos_sensor(0x6F12, 0xEBFA);
  write_cmos_sensor(0x6F12, 0x5A6A);
  write_cmos_sensor(0x6F12, 0x7B90);
  write_cmos_sensor(0x6F12, 0xA7C1);
  write_cmos_sensor(0x6F12, 0xD9ED);
  write_cmos_sensor(0x6F12, 0xFFFF);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x1F1E);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x1F32);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2036);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x204A);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x214E);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2162);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2266);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x227A);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x237E);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x2392);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x6F12, 0x00F0);
  write_cmos_sensor(0x602A, 0x3640);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x364C);
  write_cmos_sensor(0x6F12, 0x2FF0);
  write_cmos_sensor(0x6F12, 0xF44A);
  write_cmos_sensor(0x6F12, 0x0005);
  write_cmos_sensor(0x6F12, 0xF44C);
  write_cmos_sensor(0x6F12, 0x0A0C);
  write_cmos_sensor(0x6F12, 0xF412);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0xF454);
  write_cmos_sensor(0x6F12, 0x0011);
  write_cmos_sensor(0x6F12, 0xF45A);
  write_cmos_sensor(0x6F12, 0x0FFE);
  write_cmos_sensor(0x6F12, 0x9C14);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x9C1A);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x9C06);
  write_cmos_sensor(0x6F12, 0x0303);
  write_cmos_sensor(0x6F12, 0x9C1C);
  write_cmos_sensor(0x6F12, 0x0020);
  write_cmos_sensor(0x6F12, 0x9C1E);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x6F12, 0x9C02);
  write_cmos_sensor(0x6F12, 0x1FC0);
  write_cmos_sensor(0x6F12, 0x9C04);
  write_cmos_sensor(0x6F12, 0x1FC7);
  write_cmos_sensor(0x6F12, 0x9C24);
  write_cmos_sensor(0x6F12, 0x0100);
  
  
 // write_cmos_sensor(0x6028, 0x4000);

}	/*	sensor_init  */

static void capture_setting(void);
static void preview_setting(void)
{
  write_cmos_sensor_8(0x0100, 0x00);
  mdelay(33);
  write_cmos_sensor(0x6214, 0xF9F3);
  write_cmos_sensor(0x0344, 0x0000);
  write_cmos_sensor(0x0346, 0x0000);
  write_cmos_sensor(0x0348, 0x1F3F);
  write_cmos_sensor(0x034A, 0x176F);
  write_cmos_sensor(0x034C, 0x0FA0);
  write_cmos_sensor(0x034E, 0x0BB8);
  write_cmos_sensor(0x0340, 0x0C72);
  write_cmos_sensor(0x0342, 0x2700);
  write_cmos_sensor(0x0900, 0x0112);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0002);
  write_cmos_sensor(0x0386, 0x0002);
  write_cmos_sensor(0x0400, 0x2010);
  write_cmos_sensor(0x0402, 0x1010);
  write_cmos_sensor(0x0404, 0x1000);
  write_cmos_sensor(0x0350, 0x0000);
  write_cmos_sensor(0x0352, 0x0000);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x013E, 0x0000);
  write_cmos_sensor(0x0300, 0x0008);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x00F0);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x030E, 0x0003);
  write_cmos_sensor(0x0310, 0x004C);
  write_cmos_sensor(0x0312, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E36);
  write_cmos_sensor(0x6F12, 0x0005);
  write_cmos_sensor(0x602A, 0x1250);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0xF44A, 0x0007);
  write_cmos_sensor(0xF454, 0x0013);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x10C0);
  write_cmos_sensor(0x6F12, 0xBFC0);
  write_cmos_sensor(0x6F12, 0xBFC1);
  write_cmos_sensor(0x602A, 0x3B24);
  write_cmos_sensor(0x6F12, 0x000A);
  write_cmos_sensor(0x9C02, 0x1FC0);
  write_cmos_sensor(0x9C04, 0x1FC7);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E6E);
  write_cmos_sensor(0x6F12, 0x0030);
  write_cmos_sensor(0x602A, 0x3632);
  write_cmos_sensor(0x6F12, 0x0010);
  write_cmos_sensor(0x602A, 0x11B4);
  write_cmos_sensor(0x6F12, 0x0360);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0DD6);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x0DDC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1EAC);
  write_cmos_sensor(0x6F12, 0x0096);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x6C00);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0155);
  write_cmos_sensor(0x602A, 0x33B0);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x33B6);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x33FC);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x33FE);
  write_cmos_sensor(0x6F12, 0x0104);
  write_cmos_sensor(0x602A, 0x3462);
  write_cmos_sensor(0x6F12, 0x0501);
  write_cmos_sensor(0x602A, 0x347C);
  write_cmos_sensor(0x6F12, 0xEEEE);
  write_cmos_sensor(0x602A, 0x34A6);
  write_cmos_sensor(0x6F12, 0xEEEE);
  write_cmos_sensor(0x602A, 0x34E4);
  write_cmos_sensor(0x6F12, 0x0101);
  write_cmos_sensor(0x602A, 0x34F2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x0D00, 0x0101);  //PD VC
  write_cmos_sensor(0x0D02, 0x0101);  //PD VC
  write_cmos_sensor(0x0114, 0x0301);  //PD VC
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x116E);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1172);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x6C0E);
  write_cmos_sensor(0x6F12, 0x0210);
  write_cmos_sensor(0x602A, 0x2A10);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x2A80);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x6C0A);
  write_cmos_sensor(0x6F12, 0x0067);
  write_cmos_sensor(0x6F12, 0x00A8);
  write_cmos_sensor(0xB134, 0x0000);
  write_cmos_sensor(0x0BC8, 0x0102);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x39AA);
  write_cmos_sensor(0x6F12, 0x0003);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x39BA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x39C4);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E26);
  write_cmos_sensor(0x6F12, 0x1250);
  write_cmos_sensor(0x6F12, 0x0ECC);
  write_cmos_sensor(0x602A, 0x1E7E);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x602A, 0x2A02);
  write_cmos_sensor(0x6F12, 0x0100);   //pdo 0x0103   0x0100
  write_cmos_sensor(0x602A, 0x2450);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x25F8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x34D8);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x6A98);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x364E);
  write_cmos_sensor(0x6F12, 0xF44A);
  write_cmos_sensor(0x6F12, 0x0007);
  write_cmos_sensor(0x602A, 0x3676);
  write_cmos_sensor(0x6F12, 0x9C02);
  write_cmos_sensor(0x6F12, 0x1FC0);
  write_cmos_sensor(0x602A, 0x367A);
  write_cmos_sensor(0x6F12, 0x9C04);
  write_cmos_sensor(0x6F12, 0x0FE7);
  write_cmos_sensor(0x602A, 0x3682);
  write_cmos_sensor(0x6F12, 0xB134);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor_8(0x0100, 0x01);
}	/*	preview_setting  */


static void capture_setting(void)
{
	LOG_INF("E! currefps:%d\n");
  write_cmos_sensor_8(0x0100, 0x00);
  mdelay(33);
  write_cmos_sensor(0x6214, 0xF9F3);
  write_cmos_sensor(0x0344, 0x0000);
  write_cmos_sensor(0x0346, 0x0000);
  write_cmos_sensor(0x0348, 0x1F3F);
  write_cmos_sensor(0x034A, 0x176F);
  write_cmos_sensor(0x034C, 0x0FA0);
  write_cmos_sensor(0x034E, 0x0BB8);
  write_cmos_sensor(0x0340, 0x0C72);
  write_cmos_sensor(0x0342, 0x2700);
  write_cmos_sensor(0x0900, 0x0112);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0002);
  write_cmos_sensor(0x0386, 0x0002);
  write_cmos_sensor(0x0400, 0x2010);
  write_cmos_sensor(0x0402, 0x1010);
  write_cmos_sensor(0x0404, 0x1000);
  write_cmos_sensor(0x0350, 0x0000);
  write_cmos_sensor(0x0352, 0x0000);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x013E, 0x0000);
  write_cmos_sensor(0x0300, 0x0008);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x00F0);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x030E, 0x0003);
  write_cmos_sensor(0x0310, 0x004C);
  write_cmos_sensor(0x0312, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E36);
  write_cmos_sensor(0x6F12, 0x0005);
  write_cmos_sensor(0x602A, 0x1250);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0xF44A, 0x0007);
  write_cmos_sensor(0xF454, 0x0013);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x10C0);
  write_cmos_sensor(0x6F12, 0xBFC0);
  write_cmos_sensor(0x6F12, 0xBFC1);
  write_cmos_sensor(0x602A, 0x3B24);
  write_cmos_sensor(0x6F12, 0x000A);
  write_cmos_sensor(0x9C02, 0x1FC0);
  write_cmos_sensor(0x9C04, 0x1FC7);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E6E);
  write_cmos_sensor(0x6F12, 0x0030);
  write_cmos_sensor(0x602A, 0x3632);
  write_cmos_sensor(0x6F12, 0x0010);
  write_cmos_sensor(0x602A, 0x11B4);
  write_cmos_sensor(0x6F12, 0x0360);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0DD6);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x0DDC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1EAC);
  write_cmos_sensor(0x6F12, 0x0096);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x6C00);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0155);
  write_cmos_sensor(0x602A, 0x33B0);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x33B6);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x33FC);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x33FE);
  write_cmos_sensor(0x6F12, 0x0104);
  write_cmos_sensor(0x602A, 0x3462);
  write_cmos_sensor(0x6F12, 0x0501);
  write_cmos_sensor(0x602A, 0x347C);
  write_cmos_sensor(0x6F12, 0xEEEE);
  write_cmos_sensor(0x602A, 0x34A6);
  write_cmos_sensor(0x6F12, 0xEEEE);
  write_cmos_sensor(0x602A, 0x34E4);
  write_cmos_sensor(0x6F12, 0x0101);
  write_cmos_sensor(0x602A, 0x34F2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x0D00, 0x0101);  //pdo  0x0100    0x0101
  write_cmos_sensor(0x0D02, 0x0101);  //pdo  0x0001    0x0101
  write_cmos_sensor(0x0114, 0x0301);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x116E);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1172);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x6C0E);
  write_cmos_sensor(0x6F12, 0x0210);
  write_cmos_sensor(0x602A, 0x2A10);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x2A80);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x6C0A);
  write_cmos_sensor(0x6F12, 0x0067);
  write_cmos_sensor(0x6F12, 0x00A8);
  write_cmos_sensor(0xB134, 0x0000);
  write_cmos_sensor(0x0BC8, 0x0102);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x39AA);
  write_cmos_sensor(0x6F12, 0x0003);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x39BA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x39C4);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E26);
  write_cmos_sensor(0x6F12, 0x1250);
  write_cmos_sensor(0x6F12, 0x0ECC);
  write_cmos_sensor(0x602A, 0x1E7E);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x602A, 0x2A02);
  write_cmos_sensor(0x6F12, 0x0100); //pdo 0x0103   0x0100
  write_cmos_sensor(0x602A, 0x2450);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x25F8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x34D8);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x6A98);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x364E);
  write_cmos_sensor(0x6F12, 0xF44A);
  write_cmos_sensor(0x6F12, 0x0007);
  write_cmos_sensor(0x602A, 0x3676);
  write_cmos_sensor(0x6F12, 0x9C02);
  write_cmos_sensor(0x6F12, 0x1FC0);
  write_cmos_sensor(0x602A, 0x367A);
  write_cmos_sensor(0x6F12, 0x9C04);
  write_cmos_sensor(0x6F12, 0x0FE7);
  write_cmos_sensor(0x602A, 0x3682);
  write_cmos_sensor(0x6F12, 0xB134);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor_8(0x0100, 0x01);
}


static void normal_video_setting(void)
{
	LOG_INF("E! currefps:%d\n");
  write_cmos_sensor_8(0x0100, 0x00);
  mdelay(33);
  write_cmos_sensor(0x6214, 0xF9F3);
  write_cmos_sensor(0x0344, 0x0000);
  write_cmos_sensor(0x0346, 0x0000);
  write_cmos_sensor(0x0348, 0x1F3F);
  write_cmos_sensor(0x034A, 0x176F);
  write_cmos_sensor(0x034C, 0x0FA0);
  write_cmos_sensor(0x034E, 0x0BB8);
  write_cmos_sensor(0x0340, 0x0C72);
  write_cmos_sensor(0x0342, 0x2700);
  write_cmos_sensor(0x0900, 0x0112);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0002);
  write_cmos_sensor(0x0386, 0x0002);
  write_cmos_sensor(0x0400, 0x2010);
  write_cmos_sensor(0x0402, 0x1010);
  write_cmos_sensor(0x0404, 0x1000);
  write_cmos_sensor(0x0350, 0x0000);
  write_cmos_sensor(0x0352, 0x0000);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x013E, 0x0000);
  write_cmos_sensor(0x0300, 0x0008);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x00F0);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x030E, 0x0003);
  write_cmos_sensor(0x0310, 0x004C);
  write_cmos_sensor(0x0312, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E36);
  write_cmos_sensor(0x6F12, 0x0005);
  write_cmos_sensor(0x602A, 0x1250);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0xF44A, 0x0007);
  write_cmos_sensor(0xF454, 0x0013);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x10C0);
  write_cmos_sensor(0x6F12, 0xBFC0);
  write_cmos_sensor(0x6F12, 0xBFC1);
  write_cmos_sensor(0x602A, 0x3B24);
  write_cmos_sensor(0x6F12, 0x000A);
  write_cmos_sensor(0x9C02, 0x1FC0);
  write_cmos_sensor(0x9C04, 0x1FC7);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E6E);
  write_cmos_sensor(0x6F12, 0x0030);
  write_cmos_sensor(0x602A, 0x3632);
  write_cmos_sensor(0x6F12, 0x0010);
  write_cmos_sensor(0x602A, 0x11B4);
  write_cmos_sensor(0x6F12, 0x0360);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0DD6);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x0DDC);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1EAC);
  write_cmos_sensor(0x6F12, 0x0096);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x6C00);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0155);
  write_cmos_sensor(0x602A, 0x33B0);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x33B6);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x33FC);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x33FE);
  write_cmos_sensor(0x6F12, 0x0104);
  write_cmos_sensor(0x602A, 0x3462);
  write_cmos_sensor(0x6F12, 0x0501);
  write_cmos_sensor(0x602A, 0x347C);
  write_cmos_sensor(0x6F12, 0xEEEE);
  write_cmos_sensor(0x602A, 0x34A6);
  write_cmos_sensor(0x6F12, 0xEEEE);
  write_cmos_sensor(0x602A, 0x34E4);
  write_cmos_sensor(0x6F12, 0x0101);
  write_cmos_sensor(0x602A, 0x34F2);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x0D00, 0x0101);  //pdo  0x0100    0x0101
  write_cmos_sensor(0x0D02, 0x0101);  //pdo  0x0001    0x0101
  write_cmos_sensor(0x0114, 0x0301);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x116E);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1172);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x6C0E);
  write_cmos_sensor(0x6F12, 0x0210);
  write_cmos_sensor(0x602A, 0x2A10);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x2A80);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x6C0A);
  write_cmos_sensor(0x6F12, 0x0067);
  write_cmos_sensor(0x6F12, 0x00A8);
  write_cmos_sensor(0xB134, 0x0000);
  write_cmos_sensor(0x0BC8, 0x0102);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x39AA);
  write_cmos_sensor(0x6F12, 0x0003);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x39BA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x39C4);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E26);
  write_cmos_sensor(0x6F12, 0x1250);
  write_cmos_sensor(0x6F12, 0x0ECC);
  write_cmos_sensor(0x602A, 0x1E7E);
  write_cmos_sensor(0x6F12, 0x0400);
  write_cmos_sensor(0x602A, 0x2A02);
  write_cmos_sensor(0x6F12, 0x0100); //pdo 0x0103   0x0100
  write_cmos_sensor(0x602A, 0x2450);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x25F8);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x34D8);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x6A98);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x364E);
  write_cmos_sensor(0x6F12, 0xF44A);
  write_cmos_sensor(0x6F12, 0x0007);
  write_cmos_sensor(0x602A, 0x3676);
  write_cmos_sensor(0x6F12, 0x9C02);
  write_cmos_sensor(0x6F12, 0x1FC0);
  write_cmos_sensor(0x602A, 0x367A);
  write_cmos_sensor(0x6F12, 0x9C04);
  write_cmos_sensor(0x6F12, 0x0FE7);
  write_cmos_sensor(0x602A, 0x3682);
  write_cmos_sensor(0x6F12, 0xB134);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor_8(0x0100, 0x01);
}


static void hs_video_setting(void)
{
	LOG_INF("E\n");
  write_cmos_sensor_8(0x0100, 0x00);
  mdelay(33);
  #if 1   //prize hjw--for 8fps  too low
 write_cmos_sensor(0x6214, 0xF9F3);
 write_cmos_sensor(0x0344, 0x00A0);
 write_cmos_sensor(0x0346, 0x0348);
 write_cmos_sensor(0x0348, 0x1E9F);
 write_cmos_sensor(0x034A, 0x1427);
 write_cmos_sensor(0x034C, 0x0500);
 write_cmos_sensor(0x034E, 0x02D0);
 write_cmos_sensor(0x0340, 0x04F8);
 write_cmos_sensor(0x0342, 0x1080);
 write_cmos_sensor(0x0900, 0x0124);
 write_cmos_sensor(0x0380, 0x0002);
 write_cmos_sensor(0x0382, 0x0002);
 write_cmos_sensor(0x0384, 0x0002);
 write_cmos_sensor(0x0386, 0x0006);
 write_cmos_sensor(0x0400, 0x1010);
 write_cmos_sensor(0x0402, 0x1818);
 write_cmos_sensor(0x0404, 0x2000);
 write_cmos_sensor(0x0350, 0x0000);
 write_cmos_sensor(0x0352, 0x0000);
 write_cmos_sensor(0x0136, 0x1800);
 write_cmos_sensor(0x013E, 0x0000);
 write_cmos_sensor(0x0300, 0x0005);
 write_cmos_sensor(0x0302, 0x0001);
 write_cmos_sensor(0x0304, 0x0006);
 write_cmos_sensor(0x0306, 0x00CA);
 write_cmos_sensor(0x0308, 0x0008);
 write_cmos_sensor(0x030A, 0x0001);
 write_cmos_sensor(0x030C, 0x0000);
 write_cmos_sensor(0x030E, 0x0003);
 write_cmos_sensor(0x0310, 0x004B);
 write_cmos_sensor(0x0312, 0x0000);
 write_cmos_sensor(0x6028, 0x2000);
 write_cmos_sensor(0x602A, 0x0E36);
 write_cmos_sensor(0x6F12, 0x000A);
 write_cmos_sensor(0x602A, 0x1250);
 write_cmos_sensor(0x6F12, 0x0002);
 write_cmos_sensor(0xF44A, 0x0007);
 write_cmos_sensor(0xF454, 0x0011);
 write_cmos_sensor(0x6028, 0x2000);
 write_cmos_sensor(0x602A, 0x10C0);
 write_cmos_sensor(0x6F12, 0xBFC2);
 write_cmos_sensor(0x6F12, 0xBFC3);
 write_cmos_sensor(0x602A, 0x3B24);
 write_cmos_sensor(0x6F12, 0x0006);
 write_cmos_sensor(0x9C02, 0x0FE0);
 write_cmos_sensor(0x9C04, 0x0FE7);
 write_cmos_sensor(0x6028, 0x2000);
 write_cmos_sensor(0x602A, 0x0E6E);
 write_cmos_sensor(0x6F12, 0xFFFF);
 write_cmos_sensor(0x602A, 0x3632);
 write_cmos_sensor(0x6F12, 0x01E0);
 write_cmos_sensor(0x602A, 0x11B4);
 write_cmos_sensor(0x6F12, 0x0060);
 write_cmos_sensor(0x6028, 0x2000);
 write_cmos_sensor(0x602A, 0x0DD6);
 write_cmos_sensor(0x6F12, 0x000A);
 write_cmos_sensor(0x602A, 0x0DDC);
 write_cmos_sensor(0x6F12, 0x0001);
 write_cmos_sensor(0x602A, 0x1EAC);
 write_cmos_sensor(0x6F12, 0x0096);
 write_cmos_sensor(0x6028, 0x2000);
 write_cmos_sensor(0x602A, 0x6C00);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x6F12, 0x0155);
 write_cmos_sensor(0x602A, 0x33B0);
 write_cmos_sensor(0x6F12, 0x0008);
 write_cmos_sensor(0x602A, 0x33B6);
 write_cmos_sensor(0x6F12, 0x0004);
 write_cmos_sensor(0x602A, 0x33FC);
 write_cmos_sensor(0x6F12, 0x0604);
 write_cmos_sensor(0x602A, 0x33FE);
 write_cmos_sensor(0x6F12, 0x0704);
 write_cmos_sensor(0x602A, 0x3462);
 write_cmos_sensor(0x6F12, 0x7701);
 write_cmos_sensor(0x602A, 0x347C);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x602A, 0x34A6);
 write_cmos_sensor(0x6F12, 0x5555);
 write_cmos_sensor(0x602A, 0x34E4);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x602A, 0x34F2);
 write_cmos_sensor(0x6F12, 0x0001);
 write_cmos_sensor(0x0D00, 0x0100);
 write_cmos_sensor(0x0D02, 0x0001);
 write_cmos_sensor(0x0114, 0x0300);
 write_cmos_sensor(0x6028, 0x2000);
 write_cmos_sensor(0x602A, 0x116E);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x602A, 0x1172);
 write_cmos_sensor(0x6F12, 0x0100);
 write_cmos_sensor(0x602A, 0x6C0E);
 write_cmos_sensor(0x6F12, 0x0210);
 write_cmos_sensor(0x602A, 0x2A10);
 write_cmos_sensor(0x6F12, 0x0100);
 write_cmos_sensor(0x602A, 0x2A80);
 write_cmos_sensor(0x6F12, 0x0200);
 write_cmos_sensor(0x602A, 0x6C0A);
 write_cmos_sensor(0x6F12, 0x0167);
 write_cmos_sensor(0x6F12, 0x00A8);
 write_cmos_sensor(0xB134, 0x0000);
 write_cmos_sensor(0x0BC8, 0x0001);
 write_cmos_sensor(0x6028, 0x2000);
 write_cmos_sensor(0x602A, 0x39AA);
 write_cmos_sensor(0x6F12, 0x0002);
 write_cmos_sensor(0x6F12, 0x0001);
 write_cmos_sensor(0x6F12, 0x0001);
 write_cmos_sensor(0x602A, 0x39BA);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x6F12, 0x0001);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x602A, 0x39C4);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x6028, 0x2000);
 write_cmos_sensor(0x602A, 0x0E26);
 write_cmos_sensor(0x6F12, 0x0440);
 write_cmos_sensor(0x6F12, 0x0440);
 write_cmos_sensor(0x602A, 0x1E7E);
 write_cmos_sensor(0x6F12, 0x0401);
 write_cmos_sensor(0x602A, 0x2A02);
 write_cmos_sensor(0x6F12, 0x0100);//0x0103
 write_cmos_sensor(0x602A, 0x2450);
 write_cmos_sensor(0x6F12, 0x0005);
 write_cmos_sensor(0x602A, 0x25F8);
 write_cmos_sensor(0x6F12, 0xAAAA);
 write_cmos_sensor(0x602A, 0x34D8);
 write_cmos_sensor(0x6F12, 0x07C0);
 write_cmos_sensor(0x6F12, 0x07C0);
 write_cmos_sensor(0x6028, 0x2000);
 write_cmos_sensor(0x602A, 0x6A98);
 write_cmos_sensor(0x6F12, 0x0000);
 write_cmos_sensor(0x602A, 0x364E);
 write_cmos_sensor(0x6F12, 0xF44A);
 write_cmos_sensor(0x6F12, 0x0007);
 write_cmos_sensor(0x602A, 0x3676);
 write_cmos_sensor(0x6F12, 0x9C02);
 write_cmos_sensor(0x6F12, 0x0FE0);
 write_cmos_sensor(0x602A, 0x367A);
 write_cmos_sensor(0x6F12, 0x9C04);
 write_cmos_sensor(0x6F12, 0x0FE7);
 write_cmos_sensor(0x602A, 0x3682);
 write_cmos_sensor(0x6F12, 0xB134);
 write_cmos_sensor(0x6F12, 0x0000);
  #else
  write_cmos_sensor(0x6214, 0xF9F3);
  write_cmos_sensor(0x0344, 0x00A0);
  write_cmos_sensor(0x0346, 0x0348);
  write_cmos_sensor(0x0348, 0x1E9F);
  write_cmos_sensor(0x034A, 0x1427);
  write_cmos_sensor(0x034C, 0x0780);
  write_cmos_sensor(0x034E, 0x0438);
  write_cmos_sensor(0x0340, 0x04F8);
  write_cmos_sensor(0x0342, 0x1080);
  write_cmos_sensor(0x0900, 0x0124);
  write_cmos_sensor(0x0380, 0x0002);
  write_cmos_sensor(0x0382, 0x0002);
  write_cmos_sensor(0x0384, 0x0002);
  write_cmos_sensor(0x0386, 0x0006);
  write_cmos_sensor(0x0400, 0x1010);
  write_cmos_sensor(0x0402, 0x1010);
  write_cmos_sensor(0x0404, 0x2000);
  write_cmos_sensor(0x0350, 0x0000);
  write_cmos_sensor(0x0352, 0x0000);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x013E, 0x0000);
  write_cmos_sensor(0x0300, 0x0005);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x00CA);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x030E, 0x0003);
  write_cmos_sensor(0x0310, 0x0057);
  write_cmos_sensor(0x0312, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E36);
  write_cmos_sensor(0x6F12, 0x000A);
  write_cmos_sensor(0x602A, 0x1250);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0xF44A, 0x0007);
  write_cmos_sensor(0xF454, 0x0011);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x10C0);
  write_cmos_sensor(0x6F12, 0xBFC2);
  write_cmos_sensor(0x6F12, 0xBFC3);
  write_cmos_sensor(0x602A, 0x3B24);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x9C02, 0x0FE0);
  write_cmos_sensor(0x9C04, 0x0FE7);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E6E);
  write_cmos_sensor(0x6F12, 0xFFFF);
  write_cmos_sensor(0x602A, 0x3632);
  write_cmos_sensor(0x6F12, 0x0240);
  write_cmos_sensor(0x602A, 0x11B4);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0DD6);
  write_cmos_sensor(0x6F12, 0x000A);
  write_cmos_sensor(0x602A, 0x0DDC);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1EAC);
  write_cmos_sensor(0x6F12, 0x0096);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x6C00);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0155);
  write_cmos_sensor(0x602A, 0x33B0);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x33B6);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x33FC);
  write_cmos_sensor(0x6F12, 0x0604);
  write_cmos_sensor(0x602A, 0x33FE);
  write_cmos_sensor(0x6F12, 0x0704);
  write_cmos_sensor(0x602A, 0x3462);
  write_cmos_sensor(0x6F12, 0x7701);
  write_cmos_sensor(0x602A, 0x347C);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x34A6);
  write_cmos_sensor(0x6F12, 0x5555);
  write_cmos_sensor(0x602A, 0x34E4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x34F2);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x0D00, 0x0000);
  write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0114, 0x0300);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x116E);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1172);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x6C0E);
  write_cmos_sensor(0x6F12, 0x0210);
  write_cmos_sensor(0x602A, 0x2A10);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x2A80);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x6C0A);
  write_cmos_sensor(0x6F12, 0x0167);
  write_cmos_sensor(0x6F12, 0x00A8);
  write_cmos_sensor(0xB134, 0x0100);
  write_cmos_sensor(0x0BC8, 0x0001);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x39AA);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x39BA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x39C4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E26);
  write_cmos_sensor(0x6F12, 0x0440);
  write_cmos_sensor(0x6F12, 0x0440);
  write_cmos_sensor(0x602A, 0x1E7E);
  write_cmos_sensor(0x6F12, 0x0401);
  write_cmos_sensor(0x602A, 0x2A02);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x2450);
  write_cmos_sensor(0x6F12, 0x0005);
  write_cmos_sensor(0x602A, 0x25F8);
  write_cmos_sensor(0x6F12, 0xAAAA);
  write_cmos_sensor(0x602A, 0x34D8);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x6A98);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x364E);
  write_cmos_sensor(0x6F12, 0xF44A);
  write_cmos_sensor(0x6F12, 0x0007);
  write_cmos_sensor(0x602A, 0x3676);
  write_cmos_sensor(0x6F12, 0x9C02);
  write_cmos_sensor(0x6F12, 0x0FE0);
  write_cmos_sensor(0x602A, 0x367A);
  write_cmos_sensor(0x6F12, 0x9C04);
  write_cmos_sensor(0x6F12, 0x0FE7);
  write_cmos_sensor(0x602A, 0x3682);
  write_cmos_sensor(0x6F12, 0xB134);
  write_cmos_sensor(0x6F12, 0x0000);
  #endif
  write_cmos_sensor_8(0x0100, 0x01);
}


static void slim_video_setting(void)
{
	LOG_INF("E\n");
  write_cmos_sensor_8(0x0100, 0x00);
  mdelay(33);
  write_cmos_sensor(0x6214, 0xF9F3);
  write_cmos_sensor(0x0344, 0x00A0);
  write_cmos_sensor(0x0346, 0x0348);
  write_cmos_sensor(0x0348, 0x1E9F);
  write_cmos_sensor(0x034A, 0x1427);
  write_cmos_sensor(0x034C, 0x0780);
  write_cmos_sensor(0x034E, 0x0438);
  write_cmos_sensor(0x0340, 0x08CA);
  write_cmos_sensor(0x0342, 0x2550);
  write_cmos_sensor(0x0900, 0x0124);
  write_cmos_sensor(0x0380, 0x0002);
  write_cmos_sensor(0x0382, 0x0002);
  write_cmos_sensor(0x0384, 0x0002);
  write_cmos_sensor(0x0386, 0x0006);
  write_cmos_sensor(0x0400, 0x1010);
  write_cmos_sensor(0x0402, 0x1010);
  write_cmos_sensor(0x0404, 0x2000);
  write_cmos_sensor(0x0350, 0x0000);
  write_cmos_sensor(0x0352, 0x0000);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x013E, 0x0000);
  write_cmos_sensor(0x0300, 0x0005);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x00CA);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x030C, 0x0000);
  write_cmos_sensor(0x030E, 0x0003);
  write_cmos_sensor(0x0310, 0x0064);
  write_cmos_sensor(0x0312, 0x0001);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E36);
  write_cmos_sensor(0x6F12, 0x000A);
  write_cmos_sensor(0x602A, 0x1250);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0xF44A, 0x0007);
  write_cmos_sensor(0xF454, 0x0011);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x10C0);
  write_cmos_sensor(0x6F12, 0xBFC2);
  write_cmos_sensor(0x6F12, 0xBFC3);
  write_cmos_sensor(0x602A, 0x3B24);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x9C02, 0x0FE0);
  write_cmos_sensor(0x9C04, 0x0FE7);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E6E);
  write_cmos_sensor(0x6F12, 0x0030);
  write_cmos_sensor(0x602A, 0x3632);
  write_cmos_sensor(0x6F12, 0x0240);
  write_cmos_sensor(0x602A, 0x11B4);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0DD6);
  write_cmos_sensor(0x6F12, 0x000A);
  write_cmos_sensor(0x602A, 0x0DDC);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x1EAC);
  write_cmos_sensor(0x6F12, 0x0096);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x6C00);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0155);
  write_cmos_sensor(0x602A, 0x33B0);
  write_cmos_sensor(0x6F12, 0x0008);
  write_cmos_sensor(0x602A, 0x33B6);
  write_cmos_sensor(0x6F12, 0x0004);
  write_cmos_sensor(0x602A, 0x33FC);
  write_cmos_sensor(0x6F12, 0x0604);
  write_cmos_sensor(0x602A, 0x33FE);
  write_cmos_sensor(0x6F12, 0x0704);
  write_cmos_sensor(0x602A, 0x3462);
  write_cmos_sensor(0x6F12, 0x7701);
  write_cmos_sensor(0x602A, 0x347C);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x34A6);
  write_cmos_sensor(0x6F12, 0x5555);
  write_cmos_sensor(0x602A, 0x34E4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x34F2);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x0D00, 0x0000);
  write_cmos_sensor(0x0D02, 0x0001);
  write_cmos_sensor(0x0114, 0x0300);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x116E);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x1172);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x6C0E);
  write_cmos_sensor(0x6F12, 0x0210);
  write_cmos_sensor(0x602A, 0x2A10);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x2A80);
  write_cmos_sensor(0x6F12, 0x0200);
  write_cmos_sensor(0x602A, 0x6C0A);
  write_cmos_sensor(0x6F12, 0x0167);
  write_cmos_sensor(0x6F12, 0x00A8);
  write_cmos_sensor(0xB134, 0x0100);
  write_cmos_sensor(0x0BC8, 0x0001);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x39AA);
  write_cmos_sensor(0x6F12, 0x0002);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x602A, 0x39BA);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0001);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x39C4);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0E26);
  write_cmos_sensor(0x6F12, 0x0440);
  write_cmos_sensor(0x6F12, 0x0440);
  write_cmos_sensor(0x602A, 0x1E7E);
  write_cmos_sensor(0x6F12, 0x0401);
  write_cmos_sensor(0x602A, 0x2A02);
  write_cmos_sensor(0x6F12, 0x0100);
  write_cmos_sensor(0x602A, 0x2450);
  write_cmos_sensor(0x6F12, 0x0005);
  write_cmos_sensor(0x602A, 0x25F8);
  write_cmos_sensor(0x6F12, 0xAAAA);
  write_cmos_sensor(0x602A, 0x34D8);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6F12, 0x07C0);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x6A98);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor(0x602A, 0x364E);
  write_cmos_sensor(0x6F12, 0xF44A);
  write_cmos_sensor(0x6F12, 0x0007);
  write_cmos_sensor(0x602A, 0x3676);
  write_cmos_sensor(0x6F12, 0x9C02);
  write_cmos_sensor(0x6F12, 0x0FE0);
  write_cmos_sensor(0x602A, 0x367A);
  write_cmos_sensor(0x6F12, 0x9C04);
  write_cmos_sensor(0x6F12, 0x0FE7);
  write_cmos_sensor(0x602A, 0x3682);
  write_cmos_sensor(0x6F12, 0xB134);
  write_cmos_sensor(0x6F12, 0x0000);
  write_cmos_sensor_8(0x0100, 0x01);
}


static kal_uint16 read_eeprom_module_id(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, 0xA0);

	return get_byte;
}

static kal_uint32 return_sensor_id(void)
{
	kal_uint32 sensor_id = 0;
	kal_uint16 module_id = 0;
	module_id = read_eeprom_module_id(0x0F78);
	printk("s5kgm2sp otp module_id =0x%x \n",module_id);
    sensor_id = ((read_cmos_sensor_byte(0x0000) << 8) | read_cmos_sensor_byte(0x0001));
	if (0x54 == module_id) {
		sensor_id += 1;
	}
	return sensor_id;
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);
                return ERROR_NONE;
            }
			LOG_INF("Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    //const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
	LOG_1;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = KAL_FALSE;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}   /*  open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
* Custom1
*
* DESCRIPTION
*   This function start the sensor Custom1.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom4   */
static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom5   */
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;

    sensor_resolution->SensorCustom4Width  = imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height     = imgsensor_info.custom4.grabwindow_height;

    sensor_resolution->SensorCustom5Width  = imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height     = imgsensor_info.custom5.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
    sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;
	sensor_info->PDAF_Support = 2;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            Custom1(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            Custom2(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            Custom3(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            Custom4(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            Custom5(image_window, sensor_config_data); // Custom1
			break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if(framerate==300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
        case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
           // set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *framerate = imgsensor_info.custom3.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *framerate = imgsensor_info.custom4.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *framerate = imgsensor_info.custom5.max_framerate;
            break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
    write_cmos_sensor(0x3202, 0x0080);
    write_cmos_sensor(0x3204, 0x0080);
    write_cmos_sensor(0x3206, 0x0080);
    write_cmos_sensor(0x3208, 0x0080);
    write_cmos_sensor(0x3232, 0x0000);
    write_cmos_sensor(0x3234, 0x0000);
    write_cmos_sensor(0x32A0, 0x0100);
    write_cmos_sensor(0x3300, 0x0001);
    write_cmos_sensor(0x3400, 0x0001);
    write_cmos_sensor(0x3402, 0x4E00);
    write_cmos_sensor(0x3268, 0x0000);
    write_cmos_sensor(0x0600, 0x0002);		 
	} else {
    write_cmos_sensor(0x3202, 0x0081);
    write_cmos_sensor(0x3204, 0x0081);
    write_cmos_sensor(0x3206, 0x0081);
    write_cmos_sensor(0x3208, 0x0081);
    write_cmos_sensor(0x3232, 0x0040);
    write_cmos_sensor(0x3234, 0x0100);
    write_cmos_sensor(0x32A0, 0x0000);
    write_cmos_sensor(0x3300, 0x0000);
    write_cmos_sensor(0x3400, 0x0000);
    write_cmos_sensor(0x3402, 0x4E42);
    write_cmos_sensor(0x3268, 0x0100);
    write_cmos_sensor(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;


    struct SENSOR_VC_INFO_STRUCT *pvcinfo;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	
	/* SET_SENSOR_AWB_GAIN *pSetSensorAWB
	 *  = (SET_SENSOR_AWB_GAIN *)feature_para;
	 */

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
			case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:	
		switch (*feature_data) {	
            case MSDK_SCENARIO_ID_CUSTOM3:			
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;			
                break;		
            default:			
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;			
                break;		
		}		
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:		
		*(feature_data + 1) = imgsensor_info.min_gain;		
		*(feature_data + 2) = imgsensor_info.max_gain;		
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:		
		*(feature_data + 0) = imgsensor_info.min_gain_iso;		
		*(feature_data + 1) = imgsensor_info.gain_step;		
		*(feature_data + 2) = imgsensor_info.gain_type;		
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:		
		*(feature_data + 1) = imgsensor_info.min_shutter;		
		*(feature_data + 2) = imgsensor_info.exp_step;		
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
		case SENSOR_FEATURE_GET_BINNING_TYPE:		
		switch (*(feature_data + 1)) 
		{
            case MSDK_SCENARIO_ID_CUSTOM3:			
                *feature_return_para_32 = 1; /*BINNING_NONE*/			
                break;		
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:		
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:		
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:		
            case MSDK_SCENARIO_ID_SLIM_VIDEO:		
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:		
            case MSDK_SCENARIO_ID_CUSTOM4:		
            default:			
                *feature_return_para_32 = 1; /*BINNING_AVERAGED*/			
                break;
		}		
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",*feature_return_para_32);		
		*feature_para_len = 4;		
		break;
		case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 3000000;
		break;
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: 
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
			break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            //ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        /******************** PDAF START >>> *********/
		
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; 
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%llu\n", *feature_data);
			PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:

				default:
					break;
			}
			break;
	   case SENSOR_FEATURE_SET_PDAF:
			LOG_INF("PDAF mode :%d\n", *feature_data_16);
			imgsensor.pdaf_mode = *feature_data_16;
		break;
		case SENSOR_FEATURE_GET_PDAF_DATA:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			//S5KGM2SP_read_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			break;
        /******************** PDAF END   <<< *********/
		case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		#if 0
		ihdr_write_shutter((UINT16)*feature_data,
				   (UINT16)*(feature_data+1));
		#endif
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		//streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		//streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
		//memcpy(feature_return_para_32,
		//&imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		//*feature_return_para_32 = imgsensor.current_ae_effective_frame;
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
		pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[3], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
	break;
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */


static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5KGM2SP_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	S5KGM2SP_MIPI_RAW_SensorInit	*/



