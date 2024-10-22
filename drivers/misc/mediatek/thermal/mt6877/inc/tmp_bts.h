/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
*/
#ifndef __TMP_BTS_H__
#define __TMP_BTS_H__

#define APPLY_PRECISE_NTC_TABLE
#define APPLY_AUXADC_CALI_DATA
#define APPLY_PRECISE_BTS_TEMP

#define AUX_IN0_NTC (0)
#define AUX_IN1_NTC (1)
#define AUX_IN2_NTC (2)

#ifdef CONFIG_CHARGER_SPIN
#define AUX_IN5_NTC (5)
#endif

#define BTS_RAP_PULL_UP_R		100000 /* 100K, pull up resister */

#ifdef CONFIG_CHARGER_SPIN
#define BTS_TAP_OVER_CRITICAL_LOW	2197860 /* base on 100K NTC temp
						 * default value -40 deg
						 */
#else
#define BTS_TAP_OVER_CRITICAL_LOW	4397119 /* base on 100K NTC temp
						 * default value -40 deg
						 */
#endif

#define BTS_RAP_PULL_UP_VOLTAGE		1800 /* 1.8V ,pull up voltage */

#define BTS_RAP_NTC_TABLE		7 /* default is NCP15WF104F03RC(100K) */

#define BTS_RAP_ADC_CHANNEL		AUX_IN0_NTC /* default is 0 */

#define BTSMDPA_RAP_PULL_UP_R		100000 /* 100K, pull up resister */

#ifdef CONFIG_CHARGER_SPIN
#define BTSMDPA_TAP_OVER_CRITICAL_LOW	2197860 /* base on 100K NTC temp
						 * default value -40 deg
						 */
#else
#define BTSMDPA_TAP_OVER_CRITICAL_LOW	4397119 /* base on 100K NTC temp
						 * default value -40 deg
						 */
#endif

#define BTSMDPA_RAP_PULL_UP_VOLTAGE	1800 /* 1.8V ,pull up voltage */

#define BTSMDPA_RAP_NTC_TABLE		7 /* default is NCP15WF104F03RC(100K) */

#define BTSMDPA_RAP_ADC_CHANNEL		AUX_IN1_NTC /* default is 1 */


#define BTSNRPA_RAP_PULL_UP_R		100000	/* 100K,pull up resister */

#ifdef CONFIG_CHARGER_SPIN
#define BTSNRPA_TAP_OVER_CRITICAL_LOW	2197860	/* base on 100K NTC temp
						 *default value -40 deg
						 */
#else
#define BTSNRPA_TAP_OVER_CRITICAL_LOW	4397119	/* base on 100K NTC temp
						 *default value -40 deg
						 */
#endif

#define BTSNRPA_RAP_PULL_UP_VOLTAGE	1800	/* 1.8V ,pull up voltage */
#define BTSNRPA_RAP_NTC_TABLE		7

#define BTSNRPA_RAP_ADC_CHANNEL		AUX_IN2_NTC

#ifdef CONFIG_CHARGER_SPIN
#define BTSN77PA_RAP_PULL_UP_R		100000	/* 100K,pull up resister */
#define BTSN77PA_TAP_OVER_CRITICAL_LOW	2197225	/* base on 100K NTC temp
						 *default value -40 deg
						 */

#define BTSN77PA_RAP_PULL_UP_VOLTAGE	1800	/* 1.8V ,pull up voltage */
#define BTSN77PA_RAP_NTC_TABLE		7

#define BTSN77PA_RAP_ADC_CHANNEL	AUX_IN5_NTC
#endif


extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);
extern int IMM_IsAdcInitReady(void);

#endif	/* __TMP_BTS_H__ */
