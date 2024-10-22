// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <mt-plat/aee.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "mt-plat/mtk_thermal_monitor.h"
#include "mach/mtk_thermal.h"
#include <linux/uidgid.h>
#include <linux/slab.h>
#include <tmp_btscharger.h>
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
#include <linux/iio/consumer.h>
#endif

int __attribute__ ((weak))
IMM_IsAdcInitReady(void)
{
	pr_notice("E_WF: thermal_charger: %s doesn't exist\n", __func__);
	return 0;
}
int __attribute__ ((weak))
IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata)
{
	pr_notice("E_WF: thermal_charger: %s doesn't exist\n", __func__);
	return -1;
}


#define mtktsbtscharge_TEMP_CRIT (150000) /* 150.000 degree Celsius */

#define mtktsbtscharge_dprintk(fmt, args...) \
do { \
	if (mtktsbtscharge_debug_log) \
		pr_debug("[Thermal/tzcharger]" fmt, ##args); \
} while (0)

#define mtktsbtscharge_dprintk_always(fmt, args...) \
	pr_notice("[Thermal/tzcharger]" fmt, ##args)

#define mtktsbtscharge_pr_notice(fmt, args...) \
	pr_notice("[Thermal/tzcharger]" fmt, ##args)

#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
struct iio_channel *thermistor_ch4;
#endif

static kuid_t uid = KUIDT_INIT(0);
static kgid_t gid = KGIDT_INIT(1000);
static DEFINE_SEMAPHORE(sem_mutex);

static int kernelmode;
static unsigned int interval; /* seconds, 0 : no auto polling */
static int num_trip = 1;
static int trip_temp[10] = { 125000, 110000, 100000, 90000, 80000,
				70000, 65000, 60000, 55000, 50000 };

static int g_THERMAL_TRIP[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static char g_bind0[20] = "mtkbtscharge-sysrst";
static char g_bind1[20] = "";
static char g_bind2[20] = "";
static char g_bind3[20] = "";
static char g_bind4[20] = "";
static char g_bind5[20] = "";
static char g_bind6[20] = "";
static char g_bind7[20] = "";
static char g_bind8[20] = "";
static char g_bind9[20] = "";
static char *g_bind_a[10] = {&g_bind0[0], &g_bind1[0], &g_bind2[0]
	, &g_bind3[0], &g_bind4[0], &g_bind5[0], &g_bind6[0], &g_bind7[0]
	, &g_bind8[0], &g_bind9[0]};
static struct thermal_zone_device *thz_dev;

static unsigned int cl_dev_sysrst_state;
static struct thermal_cooling_device *cl_dev_sysrst;

static int mtktsbtscharge_debug_log;

/* This is to preserve last temperature readings from charger driver.
 * In case mtk_ts_charger.c fails to read temperature.
 */
/**
 * If curr_temp >= polling_trip_temp1, use interval
 * else if cur_temp >= polling_trip_temp2 && curr_temp < polling_trip_temp1,
 *	use interval*polling_factor1
 * else, use interval*polling_factor2
 */
static int polling_trip_temp1 = 40000;
static int polling_trip_temp2 = 20000;
static int polling_factor1 = 5000;
static int polling_factor2 = 10000;

struct BTSCHARGER_TEMPERATURE {
	__s32 BTSCHARGER_Temp;
	__s32 TemperatureR;
};

static int g_RAP_pull_up_R = BTSCHARGER_RAP_PULL_UP_R;
static int g_TAP_over_critical_low = BTSCHARGER_TAP_OVER_CRITICAL_LOW;
static int g_RAP_pull_up_voltage = BTSCHARGER_RAP_PULL_UP_VOLTAGE;
static int g_RAP_ntc_table = BTSCHARGER_RAP_NTC_TABLE;
static int g_RAP_ADC_channel = BTSCHARGER_RAP_ADC_CHANNEL;

static int g_btscharger_TemperatureR;
/* struct BTSCHARGER_TEMPERATURE BTSCHARGER_Temperature_Table[] = {0}; */

static struct BTSCHARGER_TEMPERATURE *BTSCHARGER_Temperature_Table;
static int ntc_tbl_size;

/* AP_NTC_BL197 */
static struct BTSCHARGER_TEMPERATURE BTSCHARGER_Temperature_Table1[] = {
	{-40, 74354},		/* FIX_ME */
	{-35, 74354},		/* FIX_ME */
	{-30, 74354},		/* FIX_ME */
	{-25, 74354},		/* FIX_ME */
	{-20, 74354},
	{-15, 57626},
	{-10, 45068},
	{-5, 35548},
	{0, 28267},
	{5, 22650},
	{10, 18280},
	{15, 14855},
	{20, 12151},
	{25, 10000},		/* 10K */
	{30, 8279},
	{35, 6892},
	{40, 5768},
	{45, 4852},
	{50, 4101},
	{55, 3483},
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970}		/* FIX_ME */
};

/* AP_NTC_TSM_1 */
static struct BTSCHARGER_TEMPERATURE BTSCHARGER_Temperature_Table2[] = {
	{-40, 70603},		/* FIX_ME */
	{-35, 70603},		/* FIX_ME */
	{-30, 70603},		/* FIX_ME */
	{-25, 70603},		/* FIX_ME */
	{-20, 70603},
	{-15, 55183},
	{-10, 43499},
	{-5, 34569},
	{0, 27680},
	{5, 22316},
	{10, 18104},
	{15, 14773},
	{20, 12122},
	{25, 10000},		/* 10K */
	{30, 8294},
	{35, 6915},
	{40, 5795},
	{45, 4882},
	{50, 4133},
	{55, 3516},
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004}		/* FIX_ME */
};

/* AP_NTC_10_SEN_1 */
static struct BTSCHARGER_TEMPERATURE BTSCHARGER_Temperature_Table3[] = {
	{-40, 74354},		/* FIX_ME */
	{-35, 74354},		/* FIX_ME */
	{-30, 74354},		/* FIX_ME */
	{-25, 74354},		/* FIX_ME */
	{-20, 74354},
	{-15, 57626},
	{-10, 45068},
	{-5, 35548},
	{0, 28267},
	{5, 22650},
	{10, 18280},
	{15, 14855},
	{20, 12151},
	{25, 10000},		/* 10K */
	{30, 8279},
	{35, 6892},
	{40, 5768},
	{45, 4852},
	{50, 4101},
	{55, 3483},
	{60, 2970},
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970}		/* FIX_ME */
};

/* AP_NTC_10(TSM0A103F34D1RZ) */
static struct BTSCHARGER_TEMPERATURE BTSCHARGER_Temperature_Table4[] = {
	{-40, 188500},
	{-35, 144290},
	{-30, 111330},
	{-25, 86560},
	{-20, 67790},
	{-15, 53460},
	{-10, 42450},
	{-5, 33930},
	{0, 27280},
	{5, 22070},
	{10, 17960},
	{15, 14700},
	{20, 12090},
	{25, 10000},		/* 10K */
	{30, 8310},
	{35, 6940},
	{40, 5830},
	{45, 4910},
	{50, 4160},
	{55, 3540},
	{60, 3020},
	{65, 2590},
	{70, 2230},
	{75, 1920},
	{80, 1670},
	{85, 1450},
	{90, 1270},
	{95, 1110},
	{100, 975},
	{105, 860},
	{110, 760},
	{115, 674},
	{120, 599},
	{125, 534}
};

/* AP_NTC_47 */
static struct BTSCHARGER_TEMPERATURE BTSCHARGER_Temperature_Table5[] = {
	{-40, 483954},		/* FIX_ME */
	{-35, 483954},		/* FIX_ME */
	{-30, 483954},		/* FIX_ME */
	{-25, 483954},		/* FIX_ME */
	{-20, 483954},
	{-15, 360850},
	{-10, 271697},
	{-5, 206463},
	{0, 158214},
	{5, 122259},
	{10, 95227},
	{15, 74730},
	{20, 59065},
	{25, 47000},		/* 47K */
	{30, 37643},
	{35, 30334},
	{40, 24591},
	{45, 20048},
	{50, 16433},
	{55, 13539},
	{60, 11210},
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210}		/* FIX_ME */
};


/* NTCG104EF104F(100K) */
/* pri modified for temperature-lookup-table update by lurongzhe at 2024/9/13 */
static struct BTSCHARGER_TEMPERATURE BTSCHARGER_Temperature_Table6[] = {
	{-40, 4251000},
	{-35, 3005000},
	{-30, 2149000},
	{-25, 1554000},
	{-20, 1135000},
	{-15, 837800},
	{-10, 624100},
	{-5, 469100},
	{0, 355600},
	{5, 271800},
	{10, 209400},
	{15, 162500},
	{20, 127000},
	{25, 100000},		/* 100K */
	{30, 79230},
	{35, 63180},
	{40, 50680},
	{45, 40900},
	{50, 33190},
	{55, 27090},
	{60, 22220},
	{65, 18320},
	{70, 15180},
	{75, 12640},
	{80, 10580},
	{85, 8887},
	{90, 7500},
	{95, 6357},
	{100, 5410},
	{105, 4623},
	{110, 3965},
	{115, 3415},
	{120, 2951},
	{125, 2560}
};

/* NCP15WF104F03RC(100K) */
static struct BTSCHARGER_TEMPERATURE BTSCHARGER_Temperature_Table7[] = {
	{-30, 2197860},
	{-29, 2056610},
	{-28, 1925280},
	{-27, 1803140},
	{-26, 1689480},
	{-25, 1583660},
	{-24, 1485110},
	{-23, 1393290},
	{-22, 1307680},
	{-21, 1227850},
	{-20, 1153370},
	{-19, 1083850},
	{-18, 1018920},
	{-17, 958280},
	{-16, 901590},
	{-15, 848600},
	{-14, 799030},
	{-13, 752640},
	{-12, 709220},
	{-11, 668560},
	{-10, 630470},
	{-9, 594760},
	{-8, 561290},
	{-7, 529890},
	{-6, 500430},
	{-5, 472780},
	{-4, 446810},
	{-3, 422420},
	{-2, 399500},
	{-1, 377960},
	{0, 357700},
	{1, 338640},
	{2, 320700},
	{3, 303810},
	{4, 287910},
	{5, 272930},
	{6, 258820},
	{7, 245510},
	{8, 232960},
	{9, 221120},
	{10, 209950},
	{11, 199410},
	{12, 189450},
	{13, 180050},
	{14, 171160},
	{15, 162770},
	{16, 154830},
	{17, 147320},
	{18, 140210},
	{19, 133490},
	{20, 127130},
	{21, 121100},
	{22, 115390},
	{23, 109980},
	{24, 104860},
	{25, 100000},		/* 100K */
	{26, 95390},
	{27, 91020},
	{28, 86870},
	{29, 82940},
	{30, 79200},
	{31, 75650},
	{32, 72280},
	{33, 69080},
	{34, 66030},
	{35, 63140},
	{36, 60390},
	{37, 57770},
	{38, 55280},
	{39, 52910},
#if defined(APPLY_PRECISE_NTC_TABLE)
	{40, 50650},
	{41, 48510},
	{42, 46460},
	{43, 44510},
	{44, 42650},
	{45, 40880},
	{46, 39200},
	{47, 37590},
	{48, 36050},
	{49, 34590},
	{50, 33190},
	{51, 31850},
	{52, 30580},
	{53, 29360},
	{54, 28200},
	{55, 27090},
	{56, 26030},
	{57, 25020},
	{58, 24050},
	{59, 23120},
	{60, 22230},
	{61, 21390},
	{62, 20570},
	{63, 19800},
	{64, 19050},
	{65, 18340},
	{66, 17660},
	{67, 17010},
	{68, 16380},
	{69, 15780},
	{70, 15210},
	{71, 14660},
	{72, 14130},
	{73, 13620},
	{74, 13130},
	{75, 12670},
	{76, 12220},
	{77, 11790},
	{78, 11380},
	{79, 10980},
	{80, 10600},
	{81, 10230},
	{82,  9880},
	{83,  9550},
	{84,  9220},
	{85,  8910},
	{86,  8610},
	{87,  8320},
	{88,  8040},
	{89,  7780},
	{90,  7520},
#else
	{40, 50650},
	{45, 40880},
	{50, 33190},
	{55, 27090},
	{60, 22230},
	{65, 18340},
	{70, 15210},
	{75, 12670},
	{80, 10600},
	{85, 8910},
	{90, 7520},
#endif
	{91, 7270},
	{92, 7040},
	{93, 6810},
	{94, 6590},
	{95, 6370},
	{96, 6170},
	{97, 5970},
	{98, 5780},
	{99, 5600},
	{100, 5420},
	{101, 5250},
	{102, 5090},
	{103, 4930},
	{104, 4780},
	{105, 4630},
	{106, 4490},
	{107, 4350},
	{108, 4220},
	{109, 4100},
	{110, 3970},
	{111, 3850},
	{112, 3740},
	{113, 3630},
	{114, 3520},
	{115, 3420},
	{116, 3320},
	{117, 3220},
	{118, 3130},
	{119, 3040},
	{120, 2950},
	{125, 2560}
};


/* convert register to temperature  */
static __s16 mtkts_btscharger_thermistor_conver_temp(__s32 Res)
{
	int i = 0;
	int asize = 0;
	__s32 RES1 = 0, RES2 = 0;
	__s32 TAP_Value = -200, TMP1 = 0, TMP2 = 0;

	asize = (ntc_tbl_size / sizeof(struct BTSCHARGER_TEMPERATURE));
	/* mtktsbtscharge_dprintk("btscharger() :
	 * asize = %d, Res = %d\n",asize,Res);
	 */
	if (Res >= BTSCHARGER_Temperature_Table[0].TemperatureR) {
		TAP_Value = -40;	/* min */
	} else if (Res <=
		BTSCHARGER_Temperature_Table[asize - 1].TemperatureR) {
		TAP_Value = 125;	/* max */
	} else {
		RES1 = BTSCHARGER_Temperature_Table[0].TemperatureR;
		TMP1 = BTSCHARGER_Temperature_Table[0].BTSCHARGER_Temp;
		/* mtktsbtscharge_dprintk("%d : RES1 = %d,TMP1 = %d\n",__LINE__,
		 * RES1,TMP1);
		 */

		for (i = 0; i < asize; i++) {
			if (Res >=
				BTSCHARGER_Temperature_Table[i].TemperatureR) {
				RES2 =
				 BTSCHARGER_Temperature_Table[i].TemperatureR;

				TMP2 =
				BTSCHARGER_Temperature_Table[i].BTSCHARGER_Temp;

				/* mtktsbtscharge_dprintk("%d :i=%d, RES2 = %d,
				 * TMP2 = %d\n",__LINE__,i,RES2,TMP2);
				 */
				break;
			}
			RES1 = BTSCHARGER_Temperature_Table[i].TemperatureR;
			TMP1 = BTSCHARGER_Temperature_Table[i].BTSCHARGER_Temp;
			/* mtktsbtscharge_dprintk("%d :i=%d, RES1 = %d,
			 * TMP1 = %d\n",__LINE__,i,RES1,TMP1);
			 */
		}

		TAP_Value = (((Res - RES2) * TMP1) + ((RES1 - Res) * TMP2))
								/ (RES1 - RES2);
	}


	return TAP_Value;
}

/* convert ADC_AP_temp_volt to register */
/*Volt to Temp formula same with 6589*/
static __s16 mtk_ts_btscharger_volt_to_temp(__u32 dwVolt)
{
	__s32 TRes;
	__u64 dwVCriAP = 0;
	__u64 dwVCriAP2 = 0;
	__s32 BTSCHARGER_TMP = -100;

	/* SW workaround-----------------------------------------------------
	 * dwVCriAP = (TAP_OVER_CRITICAL_LOW * 1800) /
	 * (TAP_OVER_CRITICAL_LOW + 39000);
	 * dwVCriAP = (TAP_OVER_CRITICAL_LOW * RAP_PULL_UP_VOLT) /
	 * (TAP_OVER_CRITICAL_LOW + RAP_PULL_UP_R);
	 */

	dwVCriAP = ((__u64)g_TAP_over_critical_low *
		(__u64)g_RAP_pull_up_voltage);
	dwVCriAP2 = (g_TAP_over_critical_low + g_RAP_pull_up_R);
	do_div(dwVCriAP, dwVCriAP2);


	if (dwVolt > ((__u32)dwVCriAP)) {
		TRes = g_TAP_over_critical_low;
	} else {
		/* TRes = (39000*dwVolt) / (1800-dwVolt);
		 * TRes = (RAP_PULL_UP_R*dwVolt) / (RAP_PULL_UP_VOLT-dwVolt);
		 */
		TRes = (g_RAP_pull_up_R * dwVolt)
				/ (g_RAP_pull_up_voltage - dwVolt);
	}
	/* ------------------------------------------------------------------ */

	g_btscharger_TemperatureR = TRes;

	/* convert register to temperature */
	BTSCHARGER_TMP = mtkts_btscharger_thermistor_conver_temp(TRes);

	return BTSCHARGER_TMP;
}

static int mtktsbtscharge_get_hw_temp(void)
{
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
	int val = 0;
	int ret = 0, output;
#else
	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0, output;
	int times = 1, Channel = g_RAP_ADC_channel; /* 6752=0(AUX_IN2_NTC) */
	static int valid_temp;
	#if defined(APPLY_AUXADC_CALI_DATA)
		int auxadc_cali_temp;
	#endif
#endif

#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
	ret = iio_read_channel_processed(thermistor_ch4, &val);
	if (ret < 0) {
		mtktsbtscharge_dprintk_always(
			"Busy/Timeout, IIO ch read failed %d\n", ret);
		return ret;
	}

	/* NOT need to do the conversion "val * 1500 / 4096" */
	/* iio_read_channel_processed can get mV immediately */
	ret = val;
#else
	if (IMM_IsAdcInitReady() == 0) {
		mtktsbtscharge_dprintk_always(
				"[thermal_auxadc_get_data]: AUXADC is not ready\n");
		return 0;
	}

	i = times;
	while (i--) {
		ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
		if (ret_value) {/* AUXADC is busy */
#if defined(APPLY_AUXADC_CALI_DATA)
			auxadc_cali_temp = valid_temp;
#else
			ret_temp = valid_temp;
#endif
		} else {
#if defined(APPLY_AUXADC_CALI_DATA)
			/*
			 * by reference mtk_auxadc.c
			 *
			 * convert to volt:
			 *      data[0] = (rawdata * 1500 / (4096 + cali_ge)) /
			 *                 1000;
			 *
			 * convert to mv, need multiply 10:
			 *      data[1] = (rawdata * 150 / (4096 + cali_ge)) %
			 *                 100;
			 *
			 * provide high precision mv:
			 *      data[2] = (rawdata * 1500 / (4096 + cali_ge)) %
			 *                 1000;
			 */
			auxadc_cali_temp = data[0]*1000+data[2];
			valid_temp = auxadc_cali_temp;
#else
			valid_temp = ret_temp;
#endif
		}

#if defined(APPLY_AUXADC_CALI_DATA)
		ret += auxadc_cali_temp;
		mtktsbtscharge_dprintk(
			"[thermal_auxadc_get_data(AUX_IN2_NTC)]: ret_temp=%d\n",
			auxadc_cali_temp);
#else
		ret += ret_temp;
		mtktsbtscharge_dprintk(
			"[thermal_auxadc_get_data(AUX_IN2_NTC)]: ret_temp=%d\n",
			ret_temp);
#endif
	}

	/* Mt_auxadc_hal.c */
	/* #define VOLTAGE_FULL_RANGE  1500 // VA voltage */
	/* #define AUXADC_PRECISE      4096 // 12 bits */
#if defined(APPLY_AUXADC_CALI_DATA)
#else
	ret = ret * 1500 / 4096;
#endif
	/* ret = ret*1800/4096;//82's ADC power */

#endif

	output = mtk_ts_btscharger_volt_to_temp(ret);
	mtktsbtscharge_dprintk_always("BTSCHARGER ret = %d, temperature = %d\n",
								ret, output);
	return output;
}

static int mtktsbtscharge_get_temp(struct thermal_zone_device *thermal, int *t)
{
	*t = mtktsbtscharge_get_hw_temp() * 1000;
	mtktsbtscharge_dprintk("%s %d\n", __func__, *t);

	if (*t >= 85000)
		mtktsbtscharge_dprintk_always("HT %d\n", *t);

	if ((int)*t >= polling_trip_temp1)
		thermal->polling_delay = interval * 1000;
	else if ((int)*t < polling_trip_temp2)
		thermal->polling_delay = interval * polling_factor2;
	else
		thermal->polling_delay = interval * polling_factor1;

	return 0;
}

static int mtktsbtscharge_bind(
struct thermal_zone_device *thermal, struct thermal_cooling_device *cdev)
{
	int table_val = 0;

	if (!strcmp(cdev->type, g_bind0))
		table_val = 0;
	else if (!strcmp(cdev->type, g_bind1))
		table_val = 1;
	else if (!strcmp(cdev->type, g_bind2))
		table_val = 2;
	else if (!strcmp(cdev->type, g_bind3))
		table_val = 3;
	else if (!strcmp(cdev->type, g_bind4))
		table_val = 4;
	else if (!strcmp(cdev->type, g_bind5))
		table_val = 5;
	else if (!strcmp(cdev->type, g_bind6))
		table_val = 6;
	else if (!strcmp(cdev->type, g_bind7))
		table_val = 7;
	else if (!strcmp(cdev->type, g_bind8))
		table_val = 8;
	else if (!strcmp(cdev->type, g_bind9))
		table_val = 9;
	else
		return 0;

	if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) {
		mtktsbtscharge_dprintk("%s error binding %s\n", __func__,
								cdev->type);
		return -EINVAL;
	}

	mtktsbtscharge_dprintk("%s binding %s at %d\n", __func__, cdev->type,
								table_val);
	return 0;
}

static int mtktsbtscharge_unbind(struct thermal_zone_device *thermal,
			    struct thermal_cooling_device *cdev)
{
	int table_val = 0;

	if (!strcmp(cdev->type, g_bind0))
		table_val = 0;
	else if (!strcmp(cdev->type, g_bind1))
		table_val = 1;
	else if (!strcmp(cdev->type, g_bind2))
		table_val = 2;
	else if (!strcmp(cdev->type, g_bind3))
		table_val = 3;
	else if (!strcmp(cdev->type, g_bind4))
		table_val = 4;
	else if (!strcmp(cdev->type, g_bind5))
		table_val = 5;
	else if (!strcmp(cdev->type, g_bind6))
		table_val = 6;
	else if (!strcmp(cdev->type, g_bind7))
		table_val = 7;
	else if (!strcmp(cdev->type, g_bind8))
		table_val = 8;
	else if (!strcmp(cdev->type, g_bind9))
		table_val = 9;
	else
		return 0;

	if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) {
		mtktsbtscharge_dprintk("%s error unbinding %s\n", __func__,
								cdev->type);
		return -EINVAL;
	}

	mtktsbtscharge_dprintk("%s unbinding OK\n", __func__);
	return 0;
}

static int mtktsbtscharge_get_mode(
struct thermal_zone_device *thermal, enum thermal_device_mode *mode)
{
	*mode = (kernelmode) ? THERMAL_DEVICE_ENABLED : THERMAL_DEVICE_DISABLED;
	return 0;
}

static int mtktsbtscharge_set_mode(
struct thermal_zone_device *thermal, enum thermal_device_mode mode)
{
	kernelmode = mode;
	return 0;
}

static int mtktsbtscharge_get_trip_type(
struct thermal_zone_device *thermal, int trip, enum thermal_trip_type *type)
{
	*type = g_THERMAL_TRIP[trip];
	return 0;
}

static int mtktsbtscharge_get_trip_temp(
struct thermal_zone_device *thermal, int trip, int *temp)
{
	*temp = trip_temp[trip];
	return 0;
}

static int mtktsbtscharge_get_crit_temp(
struct thermal_zone_device *thermal, int *temperature)
{
	*temperature = mtktsbtscharge_TEMP_CRIT;
	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops mtktsbtscharge_dev_ops = {
	.bind = mtktsbtscharge_bind,
	.unbind = mtktsbtscharge_unbind,
	.get_temp = mtktsbtscharge_get_temp,
	.get_mode = mtktsbtscharge_get_mode,
	.set_mode = mtktsbtscharge_set_mode,
	.get_trip_type = mtktsbtscharge_get_trip_type,
	.get_trip_temp = mtktsbtscharge_get_trip_temp,
	.get_crit_temp = mtktsbtscharge_get_crit_temp,
};

static int mtktsbtscharge_register_thermal(void)
{
	mtktsbtscharge_dprintk("%s\n", __func__);

	/* trips : trip 0~2 */
	thz_dev = mtk_thermal_zone_device_register("mtktsbtscharger", num_trip,
					NULL, /* name: mtktsbtscharger ??? */
					&mtktsbtscharge_dev_ops, 0, 0, 0,
					interval * 1000);

	return 0;
}

static void mtktsbtscharge_unregister_thermal(void)
{
	mtktsbtscharge_dprintk("%s\n", __func__);

	if (thz_dev) {
		mtk_thermal_zone_device_unregister(thz_dev);
		thz_dev = NULL;
	}
}

static int mtktsbtscharge_sysrst_get_max_state(
struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = 1;
	return 0;
}

static int mtktsbtscharge_sysrst_get_cur_state(
struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = cl_dev_sysrst_state;
	return 0;
}

static int mtktsbtscharge_sysrst_set_cur_state(
struct thermal_cooling_device *cdev, unsigned long state)
{
	cl_dev_sysrst_state = state;
	if (cl_dev_sysrst_state == 1) {
		pr_notice("[Thermal/mtktsbtscharge_sysrst] reset, reset, reset!!!\n");
		pr_notice("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		pr_notice("*****************************************\n");
		pr_notice("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");

		/* To trigger data abort to reset the system
		 * for thermal protection.
		 */
		BUG();
	}

	return 0;
}

static struct thermal_cooling_device_ops mtktsbtscharge_cooling_sysrst_ops = {
	.get_max_state = mtktsbtscharge_sysrst_get_max_state,
	.get_cur_state = mtktsbtscharge_sysrst_get_cur_state,
	.set_cur_state = mtktsbtscharge_sysrst_set_cur_state,
};

int mtktsbtscharge_register_cooler(void)
{
	cl_dev_sysrst = mtk_thermal_cooling_device_register(
				"mtkbtscharge-sysrst", NULL,
					&mtktsbtscharge_cooling_sysrst_ops);
	return 0;
}

void mtktsbtscharge_unregister_cooler(void)
{
	if (cl_dev_sysrst) {
		mtk_thermal_cooling_device_unregister(cl_dev_sysrst);
		cl_dev_sysrst = NULL;
	}
}
void mtkts_btscharger_prepare_table(int table_num)
{

	switch (table_num) {
	case 1:		/* AP_NTC_BL197 */
		BTSCHARGER_Temperature_Table = BTSCHARGER_Temperature_Table1;
		ntc_tbl_size = sizeof(BTSCHARGER_Temperature_Table1);
		break;
	case 2:		/* AP_NTC_TSM_1 */
		BTSCHARGER_Temperature_Table = BTSCHARGER_Temperature_Table2;
		ntc_tbl_size = sizeof(BTSCHARGER_Temperature_Table2);
		break;
	case 3:		/* AP_NTC_10_SEN_1 */
		BTSCHARGER_Temperature_Table = BTSCHARGER_Temperature_Table3;
		ntc_tbl_size = sizeof(BTSCHARGER_Temperature_Table3);
		break;
	case 4:		/* AP_NTC_10 */
		BTSCHARGER_Temperature_Table = BTSCHARGER_Temperature_Table4;
		ntc_tbl_size = sizeof(BTSCHARGER_Temperature_Table4);
		break;
	case 5:		/* AP_NTC_47 */
		BTSCHARGER_Temperature_Table = BTSCHARGER_Temperature_Table5;
		ntc_tbl_size = sizeof(BTSCHARGER_Temperature_Table5);
		break;
	case 6:		/* NTCG104EF104F */
		BTSCHARGER_Temperature_Table = BTSCHARGER_Temperature_Table6;
		ntc_tbl_size = sizeof(BTSCHARGER_Temperature_Table6);
		break;
	case 7:		/* NCP15WF104F03RC */
		BTSCHARGER_Temperature_Table = BTSCHARGER_Temperature_Table7;
		ntc_tbl_size = sizeof(BTSCHARGER_Temperature_Table7);
		break;
	default:		/* AP_NTC_10 */
		BTSCHARGER_Temperature_Table = BTSCHARGER_Temperature_Table4;
		ntc_tbl_size = sizeof(BTSCHARGER_Temperature_Table4);
		break;
	}

	pr_notice("[Thermal/TZ/BTSCHARGER] %s table_num=%d\n",
						__func__, table_num);
}
static int mtktsbtscharge_read(struct seq_file *m, void *v)
{
	seq_printf(m, "log=%d\n", mtktsbtscharge_debug_log);
	seq_printf(m, "polling delay=%d\n", interval * 1000);
	seq_printf(m, "no of trips=%d\n", num_trip);
	{
		int i = 0;

		for (; i < 10; i++)
			seq_printf(m, "%02d\t%d\t%d\t%s\n", i, trip_temp[i],
						g_THERMAL_TRIP[i], g_bind_a[i]);
	}

	return 0;
}

static ssize_t mtktsbtscharge_write(
struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	int len = 0, i;
	struct mtktsbtscharge_data {
		int trip[10];
		int t_type[10];
		char bind0[20], bind1[20], bind2[20], bind3[20], bind4[20];
		char bind5[20], bind6[20], bind7[20], bind8[20], bind9[20];
		int time_msec;
		char desc[512];
	};

	struct mtktsbtscharge_data *ptr_mtktsbtscharge_data = kmalloc(
				sizeof(*ptr_mtktsbtscharge_data), GFP_KERNEL);

	if (ptr_mtktsbtscharge_data == NULL)
		return -ENOMEM;

	len = (count < (sizeof(ptr_mtktsbtscharge_data->desc) - 1)) ?
			count : (sizeof(ptr_mtktsbtscharge_data->desc) - 1);

	if (copy_from_user(ptr_mtktsbtscharge_data->desc, buffer, len)) {
		kfree(ptr_mtktsbtscharge_data);
		return 0;
	}

	ptr_mtktsbtscharge_data->desc[len] = '\0';

	/* TODO: Add a switch of mtktsbtscharge_debug_log. */

	if (sscanf(ptr_mtktsbtscharge_data->desc,
		"%d %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d",
		&num_trip,
		&ptr_mtktsbtscharge_data->trip[0],
		&ptr_mtktsbtscharge_data->t_type[0], ptr_mtktsbtscharge_data->bind0,
		&ptr_mtktsbtscharge_data->trip[1],
		&ptr_mtktsbtscharge_data->t_type[1], ptr_mtktsbtscharge_data->bind1,
		&ptr_mtktsbtscharge_data->trip[2],
		&ptr_mtktsbtscharge_data->t_type[2], ptr_mtktsbtscharge_data->bind2,
		&ptr_mtktsbtscharge_data->trip[3],
		&ptr_mtktsbtscharge_data->t_type[3], ptr_mtktsbtscharge_data->bind3,
		&ptr_mtktsbtscharge_data->trip[4],
		&ptr_mtktsbtscharge_data->t_type[4], ptr_mtktsbtscharge_data->bind4,
		&ptr_mtktsbtscharge_data->trip[5],
		&ptr_mtktsbtscharge_data->t_type[5], ptr_mtktsbtscharge_data->bind5,
		&ptr_mtktsbtscharge_data->trip[6],
		&ptr_mtktsbtscharge_data->t_type[6], ptr_mtktsbtscharge_data->bind6,
		&ptr_mtktsbtscharge_data->trip[7],
		&ptr_mtktsbtscharge_data->t_type[7], ptr_mtktsbtscharge_data->bind7,
		&ptr_mtktsbtscharge_data->trip[8],
		&ptr_mtktsbtscharge_data->t_type[8], ptr_mtktsbtscharge_data->bind8,
		&ptr_mtktsbtscharge_data->trip[9],
		&ptr_mtktsbtscharge_data->t_type[9], ptr_mtktsbtscharge_data->bind9,
		&ptr_mtktsbtscharge_data->time_msec) == 32) {
		down(&sem_mutex);
		mtktsbtscharge_dprintk("mtktsbtscharge_unregister_thermal\n");
		mtktsbtscharge_unregister_thermal();

		if (num_trip < 0 || num_trip > 10) {
			mtktsbtscharge_dprintk_always("%s bad argument\n",
								__func__);
#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__,
					DB_OPT_DEFAULT, "%s",
					"Bad argument", __func__);
#endif
			kfree(ptr_mtktsbtscharge_data);
			up(&sem_mutex);
			return -EINVAL;
		}

		for (i = 0; i < num_trip; i++)
			g_THERMAL_TRIP[i] = ptr_mtktsbtscharge_data->t_type[i];

		g_bind0[0] = g_bind1[0] = g_bind2[0] = g_bind3[0]
			= g_bind4[0] = g_bind5[0] = g_bind6[0]
			= g_bind7[0] = g_bind8[0] = g_bind9[0] = '\0';

		for (i = 0; i < 20; i++) {
			g_bind0[i] = ptr_mtktsbtscharge_data->bind0[i];
			g_bind1[i] = ptr_mtktsbtscharge_data->bind1[i];
			g_bind2[i] = ptr_mtktsbtscharge_data->bind2[i];
			g_bind3[i] = ptr_mtktsbtscharge_data->bind3[i];
			g_bind4[i] = ptr_mtktsbtscharge_data->bind4[i];
			g_bind5[i] = ptr_mtktsbtscharge_data->bind5[i];
			g_bind6[i] = ptr_mtktsbtscharge_data->bind6[i];
			g_bind7[i] = ptr_mtktsbtscharge_data->bind7[i];
			g_bind8[i] = ptr_mtktsbtscharge_data->bind8[i];
			g_bind9[i] = ptr_mtktsbtscharge_data->bind9[i];
		}

		mtktsbtscharge_dprintk("%s g_THERMAL_TRIP_0=%d,", __func__,
			g_THERMAL_TRIP[0]);
		mtktsbtscharge_dprintk("g_THERMAL_TRIP_1=%d g_THERMAL_TRIP_2=%d ",
			g_THERMAL_TRIP[1], g_THERMAL_TRIP[2]);
		mtktsbtscharge_dprintk("g_THERMAL_TRIP_3=%d g_THERMAL_TRIP_4=%d ",
			g_THERMAL_TRIP[3], g_THERMAL_TRIP[4]);
		mtktsbtscharge_dprintk("g_THERMAL_TRIP_5=%d g_THERMAL_TRIP_6=%d ",
			g_THERMAL_TRIP[5], g_THERMAL_TRIP[6]);

		mtktsbtscharge_dprintk(
			"g_THERMAL_TRIP_7=%d g_THERMAL_TRIP_8=%d g_THERMAL_TRIP_9=%d\n",
			g_THERMAL_TRIP[7], g_THERMAL_TRIP[8],
			g_THERMAL_TRIP[9]);

		mtktsbtscharge_dprintk("cooldev0=%s cooldev1=%s cooldev2=%s ",
			g_bind0, g_bind1, g_bind2);

		mtktsbtscharge_dprintk("cooldev3=%s cooldev4=%s ",
			g_bind3, g_bind4);

		mtktsbtscharge_dprintk(
			"cooldev5=%s cooldev6=%s cooldev7=%s cooldev8=%s cooldev9=%s\n",
			g_bind5, g_bind6, g_bind7, g_bind8, g_bind9);

		for (i = 0; i < num_trip; i++)
			trip_temp[i] = ptr_mtktsbtscharge_data->trip[i];

		interval = ptr_mtktsbtscharge_data->time_msec / 1000;

		mtktsbtscharge_dprintk("%s trip_0_temp=%d trip_1_temp=%d ",
					__func__, trip_temp[0], trip_temp[1]);

		mtktsbtscharge_dprintk("trip_2_temp=%d trip_3_temp=%d ",
						trip_temp[2], trip_temp[3]);

		mtktsbtscharge_dprintk(
				"trip_4_temp=%d trip_5_temp=%d trip_6_temp=%d ",
				trip_temp[4], trip_temp[5], trip_temp[6]);

		mtktsbtscharge_dprintk("trip_7_temp=%d trip_8_temp=%d ",
						trip_temp[7], trip_temp[8]);

		mtktsbtscharge_dprintk("trip_9_temp=%d time_ms=%d\n",
						trip_temp[9], interval * 1000);

		mtktsbtscharge_dprintk("mtktsbtscharge_register_thermal\n");
		mtktsbtscharge_register_thermal();
		up(&sem_mutex);

		kfree(ptr_mtktsbtscharge_data);
		return count;
	}

	mtktsbtscharge_dprintk("%s bad argument\n", __func__);
	kfree(ptr_mtktsbtscharge_data);

	return -EINVAL;
}

static int mtktsbtscharge_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtktsbtscharge_read, NULL);
}

static ssize_t mtkts_btscharger_param_write(
struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	int len = 0;
	struct mtktsbtscharger_param_data {
		char desc[512];
		char pull_R[10], pull_V[10];
		char overcrilow[16];
		char NTC_TABLE[10];
		unsigned int valR, valV, over_cri_low, ntc_table;
		unsigned int adc_channel;
	};

	struct mtktsbtscharger_param_data *ptr_mtktsbtscharger_parm_data;

	ptr_mtktsbtscharger_parm_data = kmalloc(
				sizeof(*ptr_mtktsbtscharger_parm_data),
					GFP_KERNEL);

	if (ptr_mtktsbtscharger_parm_data == NULL)
		return -ENOMEM;

	/* external pin: 0/1/12/13/14/15, can't use pin:2/3/4/5/6/7/8/9/10/11,
	 *choose "adc_channel=11" to check if there is any param input
	 */
	ptr_mtktsbtscharger_parm_data->adc_channel = 11;

	len = (count < (sizeof(ptr_mtktsbtscharger_parm_data->desc) - 1)) ?
			count :
			(sizeof(ptr_mtktsbtscharger_parm_data->desc) - 1);

	if (copy_from_user(ptr_mtktsbtscharger_parm_data->desc, buffer, len)) {
		kfree(ptr_mtktsbtscharger_parm_data);
		return 0;
	}

	ptr_mtktsbtscharger_parm_data->desc[len] = '\0';

	mtktsbtscharge_dprintk("[%s]\n", __func__);

	if (sscanf
	    (ptr_mtktsbtscharger_parm_data->desc,
		"%9s %d %9s %d %15s %d %9s %d %d",
		ptr_mtktsbtscharger_parm_data->pull_R,
		&ptr_mtktsbtscharger_parm_data->valR,
		ptr_mtktsbtscharger_parm_data->pull_V,
		&ptr_mtktsbtscharger_parm_data->valV,
		ptr_mtktsbtscharger_parm_data->overcrilow,
		&ptr_mtktsbtscharger_parm_data->over_cri_low,
		ptr_mtktsbtscharger_parm_data->NTC_TABLE,
		&ptr_mtktsbtscharger_parm_data->ntc_table,
		&ptr_mtktsbtscharger_parm_data->adc_channel) >= 8) {

		if (!strcmp(ptr_mtktsbtscharger_parm_data->pull_R, "PUP_R")) {
			g_RAP_pull_up_R = ptr_mtktsbtscharger_parm_data->valR;
			mtktsbtscharge_dprintk("g_RAP_pull_up_R=%d\n",
							g_RAP_pull_up_R);
		} else {
			mtktsbtscharge_dprintk(
				"[%s] bad PUP_R argument\n", __func__);
			kfree(ptr_mtktsbtscharger_parm_data);
			return -EINVAL;
		}

		if (!strcmp(ptr_mtktsbtscharger_parm_data->pull_V,
			"PUP_VOLT")) {
			g_RAP_pull_up_voltage =
				ptr_mtktsbtscharger_parm_data->valV;
			mtktsbtscharge_dprintk("g_Rat_pull_up_voltage=%d\n",
							g_RAP_pull_up_voltage);
		} else {
			mtktsbtscharge_dprintk(
				"[%s] bad PUP_VOLT argument\n", __func__);
			kfree(ptr_mtktsbtscharger_parm_data);
			return -EINVAL;
		}

		if (!strcmp(ptr_mtktsbtscharger_parm_data->overcrilow,
			"OVER_CRITICAL_L")) {
			g_TAP_over_critical_low =
				ptr_mtktsbtscharger_parm_data->over_cri_low;
			mtktsbtscharge_dprintk("g_TAP_over_critical_low=%d\n",
						g_TAP_over_critical_low);
		} else {
			mtktsbtscharge_dprintk(
				"[%s] bad OVERCRIT_L argument\n", __func__);
			kfree(ptr_mtktsbtscharger_parm_data);
			return -EINVAL;
		}

		if (!strcmp(ptr_mtktsbtscharger_parm_data->NTC_TABLE,
			"NTC_TABLE")) {
			g_RAP_ntc_table =
				ptr_mtktsbtscharger_parm_data->ntc_table;
			mtktsbtscharge_dprintk("g_RAP_ntc_table=%d\n",
							g_RAP_ntc_table);
		} else {
			mtktsbtscharge_dprintk(
				"[%s] bad NTC_TABLE argument\n", __func__);
			kfree(ptr_mtktsbtscharger_parm_data);
			return -EINVAL;
		}
#ifdef CONFIG_MACH_MT6877
		/* external pin: 0/1/2/3/4/5/6/7/8,
		 * can't use pin: 9/10/11,
		 */
		if (ptr_mtktsbtscharger_parm_data->adc_channel > 8)
			/* check unsupport pin value, if unsupport,
			 * set channel as default setting.
			 */
			g_RAP_ADC_channel = BTSCHARGER_RAP_ADC_CHANNEL;
		else {
			g_RAP_ADC_channel =
				ptr_mtktsbtscharger_parm_data->adc_channel;
		}
#else
		/* external pin: 0/1/12/13/14/15,
		 * can't use pin:2/3/4/5/6/7/8/9/10/11,
		 * choose "adc_channel=11" to check if there is any param input
		 */
		if ((ptr_mtktsbtscharger_parm_data->adc_channel >= 2)
		&& (ptr_mtktsbtscharger_parm_data->adc_channel <= 11))
			/* check unsupport pin value, if unsupport,
			 * set channel = 1 as default setting.
			 */
			g_RAP_ADC_channel = AUX_IN2_NTC;
		else {
			/* check if there is any param input,
			 * if not using default g_RAP_ADC_channel:1
			 */
			if (ptr_mtktsbtscharger_parm_data->adc_channel != 11)
				g_RAP_ADC_channel =
				ptr_mtktsbtscharger_parm_data->adc_channel;
			else
				g_RAP_ADC_channel = AUX_IN2_NTC;
		}
#endif
		mtktsbtscharge_dprintk("adc_channel=%d\n",
				ptr_mtktsbtscharger_parm_data->adc_channel);
		mtktsbtscharge_dprintk("g_RAP_ADC_channel=%d\n",
						g_RAP_ADC_channel);

		mtkts_btscharger_prepare_table(g_RAP_ntc_table);

		kfree(ptr_mtktsbtscharger_parm_data);
		return count;
	}

	mtktsbtscharge_dprintk("[%s] bad argument\n", __func__);
	kfree(ptr_mtktsbtscharger_parm_data);
	return -EINVAL;
}


static int mtkts_btscharger_param_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", g_RAP_pull_up_R);
	seq_printf(m, "%d\n", g_RAP_pull_up_voltage);
	seq_printf(m, "%d\n", g_TAP_over_critical_low);
	seq_printf(m, "%d\n", g_RAP_ntc_table);
	seq_printf(m, "%d\n", g_RAP_ADC_channel);

	return 0;
}

static int mtkts_btscharger_param_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtkts_btscharger_param_read, NULL);
}


static const struct file_operations mtktsbtscharge_fops = {
	.owner = THIS_MODULE,
	.open = mtktsbtscharge_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtktsbtscharge_write,
	.release = single_release,
};

static const struct file_operations mtkts_btscharger_param_fops = {
	.owner = THIS_MODULE,
	.open = mtkts_btscharger_param_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtkts_btscharger_param_write,
	.release = single_release,
};


#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
static int mtktsbtscharge_pdrv_probe(struct platform_device *pdev)
{
	int err = 0;
	int ret = 0;
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtktsbtscharge_dir = NULL;

	mtktsbtscharge_dprintk_always("%s\n", __func__);

	if (!pdev->dev.of_node) {
		mtktsbtscharge_dprintk_always("[%s] Only DT based supported\n",
			__func__);
		return -ENODEV;
	}

	/* pri deleted for removing useless code start */
	/* thermistor_ch4 = devm_kzalloc(&pdev->dev, sizeof(*thermistor_ch4),
		GFP_KERNEL);
	if (!thermistor_ch4)
		return -ENOMEM; */
	/* pri deleted for removing useless code end */


	thermistor_ch4 = iio_channel_get(&pdev->dev, "thermistor-ch4");
	ret = IS_ERR(thermistor_ch4);
	if (ret) {
		/* pri modified for adapted charger ntc by lurongzhe at 2024/9/13 */
		ret = PTR_ERR(thermistor_ch4);
		mtktsbtscharge_dprintk_always(
			"[%s] fail to get auxadc iio ch4: %d\n",
			__func__, ret);
		return ret;
	}

	err = mtktsbtscharge_register_thermal();
	if (err)
		goto err_unreg;

	mtktsbtscharge_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtktsbtscharge_dir) {
		mtktsbtscharge_pr_notice("%s get /proc/driver/thermal failed\n",
								__func__);
	} else {
		entry = proc_create("tzbtscharger", 0664, mtktsbtscharge_dir,
							&mtktsbtscharge_fops);
		if (entry)
			proc_set_user(entry, uid, gid);

		entry = proc_create("tzbtscharger_param", 0664, mtktsbtscharge_dir,
					&mtkts_btscharger_param_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
	}

	return 0;

err_unreg:
	mtktsbtscharge_unregister_cooler();

	return 0;
}

static int mtktsbtscharge_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}


#ifdef CONFIG_OF
const struct of_device_id mt_thermistor_of_match5[2] = {
	{.compatible = "mediatek,mtboard-thermistor5",},
	{},
};
#endif

#define THERMAL_THERMISTOR_NAME    "mtboard-thermistor5"
static struct platform_driver mtktsbtscharge_driver = {
	.probe = mtktsbtscharge_pdrv_probe,
	.remove = mtktsbtscharge_pdrv_remove,
	.driver = {
		.name = THERMAL_THERMISTOR_NAME,
#ifdef CONFIG_OF
		.of_match_table = mt_thermistor_of_match5,
#endif
	},
};

#endif /*CONFIG_MEDIATEK_MT6577_AUXADC*/

static int __init mtktsbtscharge_init(void)
{
	int err = 0;
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
	/* Move this segment to probe function
	 * in case mtktsbtscharge reads temperature
	 * before mtk_charger allows it.
	 */
#else
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtktsbtscharge_dir = NULL;
#endif

	mtkts_btscharger_prepare_table(g_RAP_ntc_table);
	err = mtktsbtscharge_register_cooler();
	if (err)
		return err;

#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
	err = platform_driver_register(&mtktsbtscharge_driver);
	if (err) {
		mtktsbtscharge_dprintk("%s fail to reg driver\n", __func__);
		goto err_unreg;
	}
#else
	err = mtktsbtscharge_register_thermal();
	if (err)
		goto err_unreg;

	mtktsbtscharge_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtktsbtscharge_dir) {
		mtktsbtscharge_dprintk("%s get /proc/driver/thermal failed\n",
								__func__);
	} else {
		entry = proc_create("tzbtscharger", 0664, mtktsbtscharge_dir,
							&mtktsbtscharge_fops);
		if (entry)
			proc_set_user(entry, uid, gid);

		entry = proc_create("tzbtscharger_param", 0664, mtktsbtscharge_dir,
					&mtkts_btscharger_param_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
	}
#endif

	return 0;

err_unreg:

	mtktsbtscharge_unregister_cooler();

	return err;
}


static void __exit mtktsbtscharge_exit(void)
{
	mtktsbtscharge_dprintk("%s\n", __func__);
	mtktsbtscharge_unregister_thermal();
	mtktsbtscharge_unregister_cooler();
}

late_initcall(mtktsbtscharge_init);
module_exit(mtktsbtscharge_exit);
