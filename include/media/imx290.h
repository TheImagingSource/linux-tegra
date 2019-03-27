/*
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __IMX290_H__
#define __IMX290_H__

#include <linux/ioctl.h>  		/* For IOCTL macros */
#include <media/nvc.h>
#include <media/nvc_image.h>

#define IMX290_IOCTL_SET_MODE               _IOW('o', 1, struct imx290_mode)
#define IMX290_IOCTL_SET_FRAME_LENGTH       _IOW('o', 2, __u32)
#define IMX290_IOCTL_SET_COARSE_TIME        _IOW('o', 3, __u32)
#define IMX290_IOCTL_SET_GAIN               _IOW('o', 4, __u16)
#define IMX290_IOCTL_GET_STATUS             _IOR('o', 5, __u8)
#define IMX290_IOCTL_SET_BINNING            _IOW('o', 6, __u8)
#define IMX290_IOCTL_TEST_PATTERN           _IOW('o', 7, \
						 enum imx290_test_pattern)
#define IMX290_IOCTL_SET_GROUP_HOLD         _IOW('o', 8, struct imx290_ae)
/* IOCTL to set the operating mode of camera.
 * This can be either stereo , leftOnly or rightOnly */
#define IMX290_IOCTL_SET_CAMERA_MODE        _IOW('o', 10, __u32)
#define IMX290_IOCTL_SYNC_SENSORS           _IOW('o', 11, __u32)
#define IMX290_IOCTL_GET_FUSEID             _IOR('o', 12, struct nvc_fuseid)
#define IMX290_IOCTL_SET_HDR_COARSE_TIME    _IOW('o', 13, struct imx290_hdr)
#define IMX290_IOCTL_READ_OTP_BANK          _IOWR('o', 14, \
						struct imx290_otp_bank)
#define IMX290_IOCTL_SET_CAL_DATA           _IOW('o', 15, \
						struct imx290_cal_data)
#define IMX290_IOCTL_GET_EEPROM_DATA        _IOR('o', 20, __u8 *)
#define IMX290_IOCTL_SET_EEPROM_DATA        _IOW('o', 21, __u8 *)
#define IMX290_IOCTL_GET_CAPS               _IOR('o', 22, struct nvc_imager_cap)
#define IMX290_IOCTL_SET_POWER              _IOW('o', 23, __u32)

#define IMX290_INVALID_COARSE_TIME  -1
/* The following register address need to be updated */
#define IMX290_EEPROM_ADDRESS		0x50
#define IMX290_EEPROM_SIZE		1024
#define IMX290_EEPROM_STR_SIZE		(IMX290_EEPROM_SIZE * 2)
#define IMX290_EEPROM_BLOCK_SIZE	(1 << 8)
#define IMX290_EEPROM_NUM_BLOCKS \
	(IMX290_EEPROM_SIZE / IMX290_EEPROM_BLOCK_SIZE)

/* The following register address need to be updated */
#define IMX290_OTP_LOAD_CTRL_ADDR	0x3D81
#define IMX290_OTP_BANK_SELECT_ADDR	0x3D84
#define IMX290_OTP_BANK_START_ADDR	0x3D00
#define IMX290_OTP_BANK_END_ADDR	0x3D0F
#define IMX290_OTP_NUM_BANKS		(32)
#define IMX290_OTP_BANK_SIZE \
	 (IMX290_OTP_BANK_END_ADDR - IMX290_OTP_BANK_START_ADDR + 1)
#define IMX290_OTP_SIZE \
	 (IMX290_OTP_BANK_SIZE * IMX290_OTP_NUM_BANKS)
#define IMX290_OTP_STR_SIZE (IMX290_OTP_SIZE * 2)

#define IMX290_FUSE_ID_OTP_START_ADDR	0x3D00
#define IMX290_FUSE_ID_OTP_BANK	0
#define IMX290_FUSE_ID_SIZE		8
#define IMX290_FUSE_ID_STR_SIZE	(IMX290_FUSE_ID_SIZE * 2)

#define IMX290_FRAME_LENGTH_ADDR_1			0x301A /* VMAX, MSB */
#define IMX290_FRAME_LENGTH_ADDR_2			0x3019 /* VMAX, */
#define IMX290_FRAME_LENGTH_ADDR_3			0x3018 /* VMAX , LSB */
#define IMX290_SVR_REG_MSB				0x300F /* SVR */
#define IMX290_SVR_REG_LSB				0x300E /* SVR */
#define IMX290_COARSE_TIME_ADDR_MSB			0x3021 /* SHS1 */
#define IMX290_COARSE_TIME_ADDR_LSB			0x3020 /* SHS1 */
#define IMX290_GROUP_HOLD_ADDR				0x3001 /* REG HOLD */
#define IMX290_ANALOG_GAIN_ADDR 			0x3014 /* GAIN ADDR */

#define IMX290_STANDBY_REG				0x3000

struct imx290_mode {
	int res_x;
	int res_y;
	int fps;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
	__u8 hdr_en;
};

struct imx290_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

struct imx290_fuseid {
	__u32 size;
	__u8  id[16];
};

struct imx290_hdr {
	__u32 coarse_time_long;
	__u32 coarse_time_short;
};

struct imx290_otp_bank {
	__u32 id;
	__u8  buf[16];
};

struct imx290_cal_data {
	int loaded;
	int rg_ratio;
	int bg_ratio;
	int rg_ratio_typical;
	int bg_ratio_typical;
	__u8 lenc[62];
};

/* See notes in the nvc.h file on the GPIO usage */
enum imx290_gpio_type {
	IMX290_GPIO_TYPE_PWRDN = 0,
	IMX290_GPIO_TYPE_RESET,
};

struct imx290_eeprom_data {
	struct i2c_client *i2c_client;
	struct i2c_adapter *adap;
	struct i2c_board_info brd;
	struct regmap *regmap;
};

struct imx290_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *dovdd;
};

struct imx290_regulators {
	const char *avdd;
	const char *dvdd;
	const char *dovdd;
};

struct imx290_platform_data {
	unsigned cfg;
	unsigned num;
	const char *dev_name;
	unsigned gpio_count; /* see nvc.h GPIO notes */
	struct nvc_gpio_pdata *gpio; /* see nvc.h GPIO notes */
	struct nvc_imager_static_nvc *static_info;
	bool use_vcm_vdd;
	int (*probe_clock)(unsigned long);
	int (*power_on)(struct imx290_power_rail *);
	int (*power_off)(struct imx290_power_rail *);
	const char *mclk_name;
	struct nvc_imager_cap *cap;
	struct imx290_regulators regulators;
	bool has_eeprom;
	bool use_cam_gpio;
};

#endif  /* __IMX290_H__ */
