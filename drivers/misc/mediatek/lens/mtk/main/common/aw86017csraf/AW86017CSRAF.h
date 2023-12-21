#ifndef _AW86017CSRAF_H_
#define _AW86017CSRAF_H_

#define AF_DRVNAME		"AW86017CSRAF_DRV"

/* Log Format */
#define AW_LOGI(format, ...) \
	pr_info("[%s][%04d]%s: " format "\n", AF_DRVNAME, __LINE__, __func__, \
								##__VA_ARGS__)
#define AW_LOGD(format, ...) \
	pr_debug("[%s][%04d]%s: " format "\n", AF_DRVNAME, __LINE__, __func__, \
								##__VA_ARGS__)
#define AW_LOGE(format, ...) \
	pr_err("[%s][%04d]%s: " format "\n", AF_DRVNAME, __LINE__, __func__, \
								##__VA_ARGS__)

/* i2c transfer */
#define AW_DATA_BYTE_1		(1)
#define AW_DATA_BYTE_2		(2)
#define AW_DATA_BYTE_3		(3)

#define AW_SUCCESS		(0)
#define AW_ERROR		(-1)
#define AW_EOOR_LOOP		(5)

/* reset */
#define AW_SHUTDOWN		(0x01)
#define AW_WAKEUP		(0x00)
#define AW_RESET_DELAY_MAX	(1200)
#define AW_RESET_DELAY_MIN	(1000)

#define AW_INIT_ERROR_DELAY_MAX	(1000)
#define AW_INIT_ERROR_DELAY_MIN	(500)

/* Maximum limit position */
#define AW_LIMITPOS_MAX		(1023)
#define AW_LIMITPOS_MIN		(0)

/* Register */
#define AW_REG_CHIP_ID		(0x00)
#define AW_REG_IC_VER		(0x01)
#define AW_REG_CONTROL		(0x02)
#define AW_REG_CODE_H		(0x03)
#define AW_REG_CODE_L		(0x04)
#define AW_REG_STATUS		(0x05)
#define AW_REG_ALG_MODE		(0x06)
#define AW_REG_DIV		(0x07)
#define AW_REG_VRCT		(0x08)
#define AW_REG_PRESET		(0x09)
#define AW_REG_NRC		(0x0A)
#define AW_REG_IMAX		(0x10)
#define AW_REG_SWT		(0x11)

#define AW_PD_MODE_EN		(0x01)

/* Init Position */
#define AW_BOTTOM_INIT_POS_H	(0x00)
#define AW_BOTTOM_INIT_POS_L	(0x00)

#define AW_MID_INIT_POS_H	(0x02)
#define AW_MID_INIT_POS_L	(0x00)

/* Chip id  */
/* Mid-mounted motor */
#define AW8601_CHIPID		(0x01)
#define AW86016_CHIPID		(0x15)
/* Bottom motor */
#define AW86014_CHIPID		(0x41)
#define AW86017_CHIPID		(0x03)
/* motor type */
#define AW_MID_MOUNTED_MOTOR	(0)
#define AW_BOTTOM_MOTOR		(1)

/* Algo parameter config */
#define AW8601_RING		(0x01) /* Non-direct mode */
#define AW8601_ALGO_MODE	(0x00)
#define AW8601_DIV_H		(0x00)
#define AW8601_DIV_L		(0x00)
#define AW8601_VRCT		(0x00)

#define AW86016_RING		(0x01) /* Non-direct mode */
#define AW86016_ALGO_MODE	(0x00)
#define AW86016_DIV_H		(0x00)
#define AW86016_DIV_L		(0x00)
#define AW86016_VRCT		(0x00)

#define AW86014_RING		(0x01) /* Non-direct mode */
#define AW86014_ALGO_MODE	(0x00)
#define AW86014_DIV		(0x00)
#define AW86014_VRCT		(0x00)

#define AW86017_RING		(0x01) /* Non-direct mode */
#define AW86017_ALGO_MODE	(0x00)
#define AW86017_DIV		(0x00)
#define AW86017_VRCT		(0x00)

#endif
