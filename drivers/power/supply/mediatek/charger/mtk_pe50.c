/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
*/

#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <mt-plat/v1/prop_chgalgo_class.h>
#include "mtk_intf.h"

static struct pe50 *pe5;
/*pri add by lvyuanchuan 202400820 begin*/
#define PE50_ALGO_RERUN                   (1)
#define PE50_ALGO_DONE                    (2)
/*uA*/
#define THERMAL_INPUT_LIMIT_AT_SCREENON   (3000000)
#define THERMAL_INPUT_ITEM                (1200000)

#define HW_TEMP_LEVEL_1                   (35)
#define HW_TEMP_LEVEL_2                   (40)
#define HW_TEMP_LEVEL_3                   (45)
#define HW_TEMP_LEVEL_4                   (50)

extern int mtktspmic_get_hw_temp(void);
/*pri add by lvyuanchuan 202400820 end*/
static int __pe50_notifier_call(struct notifier_block *nb, unsigned long event,
				void *data)
{
	chr_info("%s %s\n", __func__, prop_chgalgo_notify_evt_tostring(event));
	switch (event) {
	case PCA_NOTIEVT_ALGO_STOP:
		wake_up_charger();
		break;
	default:
		break;
	}
	return 0;
}

int pe50_stop(void)
{
	if (pe5 == NULL)
		return -ENODEV;

	if (pe5->online == true) {
		chr_err("%s\n", __func__);
		pe5->online = false;
		pe5->state = PE50_INIT;
	}

	return 0;
}

bool pe50_is_ready(void)
{
	if (pe5 == NULL)
		return false;

	return prop_chgalgo_is_algo_ready(pe5->pca_algo);
}

int pe50_init(void)
{
	int ret = -EBUSY;
	struct pe50 *pe50 = NULL;

	if (pe5 == NULL) {
		pe50 = kzalloc(sizeof(struct pe50), GFP_KERNEL);
		if (pe50 == NULL)
			return -ENOMEM;

		pe5 = pe50;
		pe50->pca_algo = prop_chgalgo_dev_get_by_name("pca_algo_dv2");
		if (!pe50->pca_algo) {
			chr_err("[PE50] Get pca_algo fail\n");
			ret = -EINVAL;
		} else {
			ret = prop_chgalgo_init_algo(pe50->pca_algo);
			if (ret < 0) {
				chr_err("[PE50] Init algo fail\n");
				pe50->pca_algo = NULL;
				goto out;
			}
			pe50->nb.notifier_call = __pe50_notifier_call;
			ret = prop_chgalgo_notifier_register(pe50->pca_algo,
							     &pe50->nb);
		}
		return ret;
	}

out:
	return ret;
}
/*pri add by lvyuanchuan 202400820 begin*/
int pe50_run(void)
{
	bool running;
	int ret = 0;
	int temp = 25;
	int thr_lmt = THERMAL_INPUT_LIMIT_AT_SCREENON;
	int thr_lmt_new = THERMAL_INPUT_LIMIT_AT_SCREENON;
	struct charger_consumer *chg_consumer = NULL;
	struct charger_manager *pinfo = NULL;
	struct charger_data *dvchg_data = NULL;
	struct charger_data *chg1_data = NULL;

	temp = mtktspmic_get_hw_temp() / 1000;

	if (pe5) {
		chg_consumer = charger_manager_get_by_name(&pe5->pca_algo->dev,
				"charger_port1");
		if (chg_consumer) {
			pinfo = chg_consumer->cm;
		} else {
			chr_err("[PE50] chg_consumer is null!\n");
			return 0;
		}
	} else {
		chr_err("[PE50] pe50 is null!\n");
		return 0;
	}
	chr_info("[PE50]state:%d \n",pe5->state);
	switch (pe5->state) {
	case PE50_INIT:
		ret = mtk_pe50_start(pinfo);
		if (ret == 0) {
			pe5->online = true;
			pe5->state = PE50_RUNNING;
		}
		break;
	case PE50_RUNNING:
		running = mtk_pe50_is_running(pinfo);
		if (!running) {
			if (pinfo->finish_pe5) {
				return PE50_ALGO_DONE;
			} else {
				pe5->state = PE50_INIT;
				mtk_pe50_stop_algo(pinfo,true);
				chr_info("[PE50] rerun!!! \n");
				return PE50_ALGO_RERUN;
			}
		}
		dvchg_data = &pinfo->dvchg1_data;
		chg1_data = &pinfo->chg1_data;
		thr_lmt = chg1_data->thermal_charging_current_limit;
		if (pinfo->is_screenon) {
			/*screen on*/
			if (temp <= HW_TEMP_LEVEL_1) {
				thr_lmt_new = THERMAL_INPUT_LIMIT_AT_SCREENON;
			} else if ((temp > HW_TEMP_LEVEL_1) && (temp <= HW_TEMP_LEVEL_2)) {
				thr_lmt_new = THERMAL_INPUT_LIMIT_AT_SCREENON - 300000;
			} else if ((temp > HW_TEMP_LEVEL_2) && (temp <= HW_TEMP_LEVEL_3)) {
				thr_lmt_new = THERMAL_INPUT_LIMIT_AT_SCREENON - 500000;
			} else if (temp > HW_TEMP_LEVEL_3) {
				thr_lmt_new = THERMAL_INPUT_LIMIT_AT_SCREENON - 700000;
			}
		} else {
			/* screen off*/
			thr_lmt_new = (thr_lmt != -1) ? thr_lmt * 2 : -1;
			if ((thr_lmt_new != -1) && (thr_lmt_new < THERMAL_INPUT_ITEM * 2)) {
				thr_lmt_new = THERMAL_INPUT_ITEM * 2;
			}
		}

		dvchg_data->thermal_input_current_limit = thr_lmt_new;
		mtk_pe50_thermal_throttling(pinfo, dvchg_data->thermal_input_current_limit);
		chr_info("[PE50](ap_temp,thr_lmt):(%d %d),(is_screenon ,thr_lmt_new):(%d %d),cv:%d\n",
				temp, thr_lmt,
				pinfo->is_screenon, thr_lmt_new,
				pinfo->sw_jeita.cv);
		break;
	default:
		break;
	}
	return ret;
}
/*pri add by lvyuanchuan 202400820 begin*/