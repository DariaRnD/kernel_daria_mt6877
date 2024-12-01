/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_gestrue.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"

#ifdef CONFIG_TOUCHSCREEN_COMMON
#include <linux/input/tp_common.h>
#endif

/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define GESTURE_LEFT                            0x20
#define GESTURE_RIGHT                           0x21
#define GESTURE_UP                              0x22
#define GESTURE_DOWN                            0x23
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_O                               0x30
#define GESTURE_W                               0x31
#define GESTURE_M                               0x32
#define GESTURE_E                               0x33
#define GESTURE_L                               0x44
#define GESTURE_S                               0x46
#define GESTURE_V                               0x54
#define GESTURE_Z                               0x41
#define GESTURE_C                               0x34
#define GESTURE_FODDOWN                         0x98
#define GESTURE_FODUP                           0x99

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*
* gesture_id    - mean which gesture is recognised
* point_num     - points number of this gesture
* coordinate_x  - All gesture point x coordinate
* coordinate_y  - All gesture point y coordinate
* mode          - gesture enable/disable, need enable by host
*               - 1:enable gesture function(default)  0:disable
* active        - gesture work flag,
*                 always set 1 when suspend, set 0 when resume
*/
struct fts_gesture_st {
    u8 gesture_id;
    u8 point_num;
    u16 coordinate_x[FTS_GESTURE_POINTS_MAX];
    u16 coordinate_y[FTS_GESTURE_POINTS_MAX];
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static ssize_t fts_gesture_show(
    struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
    int count = 0;
    struct fts_ts_data *ts_data = fts_data;

    mutex_lock(&ts_data->gesture_lock);
    count = snprintf(buf, PAGE_SIZE, "%d\n", ts_data->gesture_support);
    mutex_unlock(&ts_data->gesture_lock);

    return count;
}

static ssize_t fts_gesture_store(
    struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
    struct fts_ts_data *ts_data = fts_data;

    if (ts_data->suspended) {
        FTS_INFO("In suspend,not operation gesture mode!");
        return count;
    }
    mutex_lock(&ts_data->gesture_lock);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        FTS_DEBUG("enable gesture");
        ts_data->gesture_support = ENABLE;
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        FTS_DEBUG("disable gesture");
        ts_data->gesture_support = DISABLE;
    }
    mutex_unlock(&ts_data->gesture_lock);

    return count;
}

static struct tp_common_ops tp_common_double_tap_enabled_ops = {
    .show = fts_gesture_show,
    .store = fts_gesture_store,
};

#if FTS_FOD_EN
/* fts_fod_mode node */
static ssize_t fts_fod_show(
    struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
    int count = 0;
    struct fts_ts_data *ts_data = fts_data;

    mutex_lock(&ts_data->gesture_lock);
    count = snprintf(buf, PAGE_SIZE, "%d\n", ts_data->fod_mode);
    mutex_unlock(&ts_data->gesture_lock);

    return count;
}

static ssize_t fts_fod_store(
    struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
    struct fts_ts_data *ts_data = fts_data;

    mutex_lock(&ts_data->gesture_lock);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        fts_fod_enable(ENABLE);
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        fts_fod_enable(DISABLE);
    }
    mutex_unlock(&ts_data->gesture_lock);

    return count;
}

static struct tp_common_ops tp_common_fod_enabled_ops = {
    .show = fts_fod_show,
    .store = fts_fod_store,
};
#endif

static ssize_t fts_gesture_buf_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    int i = 0;
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);
    struct fts_gesture_st *gesture = &fts_gesture_data;

    mutex_lock(&ts_data->gesture_lock);
    count = snprintf(buf, PAGE_SIZE, "Gesture ID:%d\n", gesture->gesture_id);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum:%d\n",
                      gesture->point_num);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture Points Buffer:\n");

    /* save point data,max:6 */
    for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
        count += snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i,
                          gesture->coordinate_x[i], gesture->coordinate_y[i]);
        if ((i + 1) % 4 == 0)
            count += snprintf(buf + count, PAGE_SIZE, "\n");
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n");
    mutex_unlock(&ts_data->gesture_lock);

    return count;
}

static ssize_t fts_gesture_buf_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_gesture_bm_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    mutex_lock(&ts_data->gesture_lock);
    count = snprintf(buf, PAGE_SIZE, "gesture bmode:%d\n",
                     ts_data->gesture_bmode);
    mutex_unlock(&ts_data->gesture_lock);

    return count;
}

static ssize_t fts_gesture_bm_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);
    int value = 0xFF;
    int ret = 0;

    mutex_lock(&ts_data->gesture_lock);
    ret = sscanf(buf, "%d", &value);
    if (ret == 1) {
        FTS_DEBUG("gesture bmode:%d->%d", ts_data->gesture_bmode, value);
        ts_data->gesture_bmode = value;
    }
    mutex_unlock(&ts_data->gesture_lock);

    return count;
}

static ssize_t fts_gesture_double_tap_pressed_show(
    struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
    struct fts_ts_data *ts_data = fts_data;

    return snprintf(buf, PAGE_SIZE, "%d\n",
                     ts_data->double_tap_pressed);
}

static ssize_t fts_gesture_fod_pressed_show(
    struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
    struct fts_ts_data *ts_data = fts_data;

    return snprintf(buf, PAGE_SIZE, "%d\n",
                     ts_data->fod_fp_down);
}

static struct tp_common_ops tp_common_double_tap_pressed_ops = {
    .show = fts_gesture_double_tap_pressed_show,
};

static struct tp_common_ops tp_common_fod_pressed_ops = {
    .show = fts_gesture_fod_pressed_show,
};

/*
 *   read example: cat fts_gesture_buf        --- read gesture buf
 */
static DEVICE_ATTR(fts_gesture_buf, S_IRUGO | S_IWUSR,
                   fts_gesture_buf_show, fts_gesture_buf_store);

static DEVICE_ATTR(fts_gesture_bm, S_IRUGO | S_IWUSR,
                   fts_gesture_bm_show, fts_gesture_bm_store);

static struct attribute *fts_gesture_mode_attrs[] = {
    &dev_attr_fts_gesture_buf.attr,
    &dev_attr_fts_gesture_bm.attr,
    NULL,
};

static struct attribute_group fts_gesture_group = {
    .attrs = fts_gesture_mode_attrs,
};

static int fts_create_gesture_sysfs(struct device *dev)
{
    int ret = 0;

    ret = sysfs_create_group(&dev->kobj, &fts_gesture_group);
    if (ret) {
        FTS_ERROR("gesture sys node create fail");
        sysfs_remove_group(&dev->kobj, &fts_gesture_group);
        return ret;
    }

    return 0;
}

#ifdef CONFIG_TOUCHSCREEN_COMMON
static void fts_gesture_report(struct fts_ts_data *ts_data, int gesture_id)
{
    FTS_DEBUG("gesture_id:0x%x", gesture_id);
    switch (gesture_id) {
    case GESTURE_DOUBLECLICK:
        ts_data->double_tap_pressed = true;
        sysfs_notify(touchpanel_kobj, NULL, "double_tap_pressed");
        break;
    case GESTURE_FODDOWN:
        ts_data->fod_fp_down = true;
        sysfs_notify(touchpanel_kobj, NULL, "fod_pressed");
        break;
    case GESTURE_FODUP:
        ts_data->fod_fp_down = false;
        sysfs_notify(touchpanel_kobj, NULL, "fod_pressed");
        break;
    default:
        FTS_ERROR("gesture_id:0x%x not support", gesture_id);
        break;
    }
}
#endif

/*****************************************************************************
* Name: fts_gesture_readdata
* Brief: Read information about gesture: enable flag/gesture points..., if ges-
*        ture enable, save gesture points' information, and report to OS.
*        It will be called this function every intrrupt when gesture is supported.
*
*        gesture data length: 1(enable) + 1(reserve) + 2(header) + 6 * 4
* Input: ts_data - global struct data
*        data    - gesture data buffer
* Output:
* Return: 0 - read gesture data successfully, the report data is gesture data
*         1 - tp not in suspend/gesture not enable in TP FW
*         -Exx - error
*****************************************************************************/
int fts_gesture_readdata(struct fts_ts_data *ts_data, u8 *touch_buf)
{
    int ret = 0;
    int i = 0;
    int index = 0;
    u8 buf[FTS_GESTURE_DATA_LEN] = { 0 };
    u8 gesture_en = 0xFF;
    struct fts_gesture_st *gesture = &fts_gesture_data;

    ret = fts_read_reg(FTS_REG_GESTURE_EN, &gesture_en);
    if (gesture_en != ENABLE) {
        FTS_DEBUG("gesture not enable in fw, don't process gesture");
        return 0;
    }

    if ((ts_data->gesture_bmode == GESTURE_BM_TOUCH) && touch_buf &&
        (TOUCH_DEFAULT == ((touch_buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F))) {
        memcpy(buf, touch_buf + FTS_TOUCH_DATA_LEN, FTS_GESTURE_DATA_LEN);
    } else {
        buf[2] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
        ret = fts_read(&buf[2], 1, &buf[2], FTS_GESTURE_DATA_LEN - 2);
        if (ret < 0) {
            FTS_ERROR("read gesture header data fail");
            return ret;
        }
    }

    /* init variable before read gesture point */
    memset(gesture->coordinate_x, 0, FTS_GESTURE_POINTS_MAX * sizeof(u16));
    memset(gesture->coordinate_y, 0, FTS_GESTURE_POINTS_MAX * sizeof(u16));
    gesture->gesture_id = buf[2];
    gesture->point_num = buf[3];
    FTS_DEBUG("gesture_id=%d, point_num=%d",
              gesture->gesture_id, gesture->point_num);

    /* save point data,max:6 */
    for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
        index = 4 * i + 4;
        gesture->coordinate_x[i] = (u16)(((buf[0 + index] & 0x0F) << 8)
                                         + buf[1 + index]);
        gesture->coordinate_y[i] = (u16)(((buf[2 + index] & 0x0F) << 8)
                                         + buf[3 + index]);
    }

    /* report gesture to OS */
#ifdef CONFIG_TOUCHSCREEN_COMMON
    fts_gesture_report(ts_data, gesture->gesture_id);
#endif
    return FTS_RETVAL_IGNORE_TOUCHES;
}

void fts_gesture_recovery(struct fts_ts_data *ts_data)
{
    u8 state = 0xFF;
    if (ts_data->gesture_support && ts_data->suspended) {
        fts_write_reg(0xD1, 0x3F);
        fts_write_reg(0xD2, 0xFF);
        fts_write_reg(0xD5, 0xFF);
        fts_write_reg(0xD6, 0xFF);
        fts_write_reg(0xD7, 0xFF);
        fts_write_reg(0xD8, 0xFF);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
        fts_msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state != ENABLE) {
            FTS_ERROR("set gesture mode failed");
        }
    }
}

int fts_gesture_suspend(struct fts_ts_data *ts_data)
{
    int i = 0;
    u8 state = 0xFF;
    u8 reg_value = 0;

    FTS_FUNC_ENTER();

    for (i = 0; i < FTS_MAX_RETRIES_WRITEREG; i++) {
        fts_write_reg(0xD1, 0x3F);
        fts_write_reg(0xD2, 0xFF);
        fts_write_reg(0xD5, 0xFF);
        fts_write_reg(0xD6, 0xFF);
        fts_write_reg(0xD7, 0xFF);
        fts_write_reg(0xD8, 0xFF);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
        fts_msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == ENABLE)
            break;
    }

    if (i >= FTS_MAX_RETRIES_WRITEREG)
        FTS_ERROR("make IC enter into gesture(suspend) fail,state:%x", state);
    else
        FTS_INFO("Enter into gesture(suspend) successfully");

    fts_read_reg(FTS_REG_WORKMODE, &reg_value);
    FTS_INFO("[nadal][fts] reg_value:0x%x\n", reg_value);
    FTS_FUNC_EXIT();
    return 0;
}

int fts_gesture_resume(struct fts_ts_data *ts_data)
{
    int i = 0;
    u8 state = 0xFF;

    FTS_FUNC_ENTER();
    for (i = 0; i < FTS_MAX_RETRIES_WRITEREG; i++) {
        fts_write_reg(FTS_REG_GESTURE_EN, DISABLE);
        fts_msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == DISABLE)
            break;
    }

    if (i >= FTS_MAX_RETRIES_WRITEREG)
        FTS_ERROR("make IC exit gesture(resume) fail,state:%x", state);
    else
        FTS_INFO("resume from gesture successfully");

    FTS_FUNC_EXIT();
    return 0;
}

#if FTS_FOD_EN
static void fts_fod_set_reg(int value)
{
    int i = 0;
    u8 fod_val = value ? FTS_VAL_FOD_ENABLE : DISABLE;
    u8 regval = 0xFF;

    for (i = 0; i < FTS_MAX_RETRIES_WRITEREG; i++) {
        fts_read_reg(FTS_REG_FOD_MODE_EN, &regval);
        if (regval == fod_val)
            break;
        fts_write_reg(FTS_REG_FOD_MODE_EN, fod_val);
        fts_msleep(1);
    }

    if (i >= FTS_MAX_RETRIES_WRITEREG)
        FTS_ERROR("set fod mode to %x failed,reg_val:%x", fod_val, regval);
    else if (i > 0)
        FTS_INFO("set fod mode to %x successfully", fod_val);
}

void fts_fod_enable(int enable)
{
    struct fts_ts_data *ts_data = fts_data;

    ts_data->fod_fp_down = false;
    if (enable) {
        FTS_INFO("Fod enable");
        ts_data->fod_mode = ENABLE;
        fts_fod_set_reg(FTS_VAL_FOD_ENABLE);
    } else {
        FTS_INFO("Fod disable");
        ts_data->fod_mode = DISABLE;
        fts_fod_set_reg(DISABLE);
    }
}

/*****************************************************************************
* Name: fts_fod_readdata
* Brief: read fod value from TP, check whether having FOD event or not,
*        and report the state to host if need.
*
* Input: ts_data
* Output:
* Return: return negative code if error occurs,return 0 or 1 if success.
*         return 0 if continue report finger touches.
*         return 1(FTS_RETVAL_IGNORE_TOUCHES) if you want to ingore this
*         finger reporting, As default, the following situation will report 1:
*               a.System in suspend state, now not handle gesture.
*****************************************************************************/
int fts_fod_readdata(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int fod_x = 0;
    int fod_y = 0;
    int fod_pointid = 0;
    int fod_down = 0;
    u8 fod_val[FTS_FOD_BUF_LEN] = { 0 };
    u8 fod_cmd = FTS_REG_FOD_DATA;

    ret = fts_read(&fod_cmd, 1, fod_val, FTS_FOD_BUF_LEN);
    if (ret < 0) {
        FTS_ERROR("read fod data failed,ret=%d", ret);
        return ret;
    }

    if (fod_val[1] == 0x26) {
        fod_pointid = fod_val[0];
        fod_x = (fod_val[4] << 8) + fod_val[5];
        fod_y = (fod_val[6] << 8) + fod_val[7];
        fod_down = (fod_val[8] == 0) ? 1 : 0;
        FTS_DEBUG("FOD data:%x %x %x %x[%x,%x][%x]", fod_val[0], fod_val[1],
                  fod_val[2], fod_val[3], fod_x, fod_y, fod_val[8]);
        ret = (ts_data->suspended) ? FTS_RETVAL_IGNORE_TOUCHES : 0;
    } else {
        ret = 0;
    }

#ifdef CONFIG_TOUCHSCREEN_COMMON
    fts_gesture_report(ts_data, fod_down ? GESTURE_FODDOWN : GESTURE_FODUP);
#endif
    return ret;
}

int fts_fod_recovery(struct fts_ts_data *ts_data)
{
    if (ts_data->fod_mode) {
        fts_fod_set_reg(FTS_VAL_FOD_ENABLE);
    }
    return 0;
}

/*****************************************************************************
* Name: fts_fod_checkdown
* Brief: check fod down event is triggered, it's used to reset TP or not when
*        resuming.
*
* Input: ts_data
* Output:
* Return: return 1 if having fod down event, or else return 0
*****************************************************************************/
int fts_fod_suspend(struct fts_ts_data *ts_data)
{
    ts_data->fod_fp_down = false;
    fts_fod_set_reg(FTS_VAL_FOD_ENABLE);
    return 0;
}

int fts_fod_resume(struct fts_ts_data *ts_data)
{
    if (!fts_fod_checkdown(ts_data)) fts_fod_set_reg(FTS_VAL_FOD_ENABLE);
    ts_data->fod_fp_down = false;
    return 0;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_COMMON
static int fts_gesture_tp_common_init(struct fts_ts_data *ts_data)
{
    int ret = 0;

    /* gesture pressed */
    ret = tp_common_set_double_tap_pressed_ops(&tp_common_double_tap_pressed_ops);
    if (ret) {
        FTS_ERROR("set double_tap_pressed ops fail");
        return ret;
    }
    #if FTS_FOD_EN
    ret = tp_common_set_fod_pressed_ops(&tp_common_fod_pressed_ops);
    if (ret) {
        FTS_ERROR("set fod_pressed ops fail");
        return ret;
    }
    #endif

    /* gesture enabled */
    ret = tp_common_set_double_tap_enabled_ops(&tp_common_double_tap_enabled_ops);
    if (ret) {
        FTS_ERROR("set gesture_enabled ops fail");
        return ret;
    }
    #if FTS_FOD_EN
    ret = tp_common_set_fod_enabled_ops(&tp_common_fod_enabled_ops);
    if (ret) {
        FTS_ERROR("set fod_enabled ops fail");
        return ret;
    }
    #endif

    return 0;
}
#endif

int fts_gesture_init(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();

    memset(&fts_gesture_data, 0, sizeof(struct fts_gesture_st));
    ts_data->gesture_bmode = GESTURE_BM_REG;
    ts_data->gesture_support = DISABLE;
    ts_data->fod_mode = DISABLE;

    if (ts_data->bus_type == BUS_TYPE_SPI) {
        if ((ts_data->ic_info.ids.type <= 0x25)
            || (ts_data->ic_info.ids.type == 0x87)
            || (ts_data->ic_info.ids.type == 0x88)) {
            FTS_INFO("ic type:0x%02x,GESTURE_BM_TOUCH", ts_data->ic_info.ids.type);
            ts_data->touch_size += FTS_GESTURE_DATA_LEN;
            ts_data->gesture_bmode = GESTURE_BM_TOUCH;
        }
    }

    mutex_init(&ts_data->gesture_lock);

    fts_create_gesture_sysfs(ts_data->dev);
#ifdef CONFIG_TOUCHSCREEN_COMMON
    fts_gesture_tp_common_init(ts_data);
#endif

    FTS_FUNC_EXIT();
    return 0;
}

int fts_gesture_exit(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    sysfs_remove_group(&ts_data->dev->kobj, &fts_gesture_group);
    FTS_FUNC_EXIT();
    return 0;
}
