/*
 * imx290_mode_tbls.h - imx290 sensor mode tables
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION, All Rights Reserved. * 
 * Copyright (c) 2016, Leopard Imaging, Inc, All Rights Reserved.
 * Copyright (c) 2019, The Imaging Source Europe GmbH, All Rights Reserved.
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

#ifndef __IMX290_TABLES__
#define __IMX290_TABLES__

#include <media/camera_common.h>

/*
 * define register table operation macro
 */
#define IMX290_TABLE_WAIT_MS 0
#define IMX290_TABLE_END 1
#define IMX290_MAX_RETRIES 3
#define IMX290_WAIT_MS 3

/*
 * define the imx290 reg list structure
 */
#define imx290_reg struct reg_8

struct imx290_reg_struct {
	u16 addr;
	u8  shift;
	u8 bits;
	u32 val;
};

static struct reg_8 imx290_mode_1920x1080[] = {
	{IMX290_TABLE_WAIT_MS, 10},
	/* software reset */
	{0x3003, 0x01},
	{IMX290_TABLE_WAIT_MS, 50},
	{0x3000, 0x01},
	{0x3002, 0x00},
	{0x3002, 0x01},
	{IMX290_TABLE_WAIT_MS, 50},

	/* global settings */
	/*ID2*/
	{0x3004, 0x10}, /*FIXED*/

	{0x3005, 0x01}, /* AD 12BIT, NO SHIFT*/
	{0x3006, 0x00}, /* DRIVE MODE = ALL PIXEL SCAN MODE*/
	{0x3007, 0x00}, /* NO H,V REVERSE , WIN MODE =FULL 1080p mode*/
	{0x3008, 0xA0}, /* FIXED*/
	{0x3009, 0x01}, /* Middle Speed FRAME RATE = 60 FPS*/
	{0x300A, 0xF0}, /* 12BIT BLACK LEVEL[0:7]*/
	{0x300B, 0x00}, /* 12BIT BLACK LEVEL[8]*/

	{0x300C, 0x00}, /* FIXED*/
	{0x300D, 0x00}, /* FIXED*/
	{0x300E, 0x01}, /* FIXED*/
	{0x300F, 0x00}, /* FIXED*/
	{0x3010, 0x21}, /* FIXED*/
	{0x3011, 0x00}, /* FIXED*/
	{0x3012, 0x64}, /* FIXED*/
	{0x3013, 0x00}, /* FIXED*/

	{0x3014, 0x00}, /* GAIN =  DB*/

	{0x3015, 0x00}, /* FIXED*/
	{0x3016, 0x09}, /* FIXED*/
	{0x3017, 0x00}, /* FIXED*/

	{0x3018, 0x65}, /* VMAX[0:7]*/
	{0x3019, 0x04}, /* VMAX[8:15] ==>1125*/
	{0x301A, 0x00}, /* VMAX[16]*/

	{0x301B, 0x00},
	{0x301C, 0x98}, /* HMAX[0:7]*/

	{0x301D, 0x08}, /* HMAX[8:15] =>4400*/
	{0x301E, 0xB2}, /* FIXED*/
	{0x301F, 0x01}, /* FIXED*/

	{0x3020, 0x00}, /* SUB CONTROL [0:7]*/
	{0x3021, 0x00}, /* SUB CONTROL [8:15]*/
	{0x3022, 0x00}, /* SUB CONTROL [16]*/

	{0x303A, 0x0C}, /* WINWV[0:7] , V CROPPING SIZE*/
	{0x303B, 0x00}, /* WINWV[8:10]*/
	{0x303C, 0x00}, /* WINPH[0:7] , H CROPPING POSITION*/
	{0x303D, 0x00}, /* WINPV[8:10]*/
	{0x303E, 0x48}, /* WINWH[0:7] , H CROPPING SIZE*/
	{0x303F, 0x04}, /* WINWH[8:10]*/

	{0x3040, 0x00}, /* FIXED*/
	{0x3041, 0x00}, /* FIXED*/
	{0x3042, 0x9C}, /* WINWV[0:7] , H CROPPING SIZE*/
	{0x3043, 0x07}, /* WINWV[8:10]*/

	{0x3046, 0x01}, /* CSI-2 fixed 0x1*/
	{0x3047, 0x01},

	{0x3048, 0x00},

	{0x3049, 0x08},
	{0x305C, 0x18}, /* INCK for 37.125M*/
	{0x305D, 0x03},
	{0x305E, 0x20},
	{0x305F, 0x01},

	{0x3070, 0x02},
	{0x3071, 0x11},
	{0x30a2, 0x02},
	{0x30a6, 0x20},
	{0x30a8, 0x20},
	{0x30ac, 0x20},
	{0x30b0, 0x43},

	/*ID3 */
	{0x3119, 0x9E},
	{0x311c, 0x1e},
	{0x311E, 0x08},
	{0x3128, 0x05},

	{0x313D, 0x83},
	{0x3150, 0x03},

	{0x315E, 0x1A}, /* INCK5 37.125M*/
	{0x3164, 0x1A}, /* INCK6 37.125M*/

	{0x317C, 0x00},
	{0x317e, 0x00},
	{0x317F, 0x50},

	/*ID4*/
	{0x32B8, 0x50},
	{0x32B9, 0x10},
	{0x32BA, 0x00},
	{0x32BB, 0x04},
	{0x32C8, 0x50},
	{0x32C9, 0x10},
	{0x32CA, 0x00},
	{0x32CB, 0x04},

	/*ID5*/
	{0x332C, 0xD3},
	{0x332D, 0x10},
	{0x332E, 0x0D},
	{0x3358, 0x06},
	{0x3359, 0xE1},
	{0x335A, 0x11},
	{0x3360, 0x1E},
	{0x3361, 0x61},
	{0x3362, 0x10},
	{0x33B0, 0x50},
	{0x33B1, 0x80},
	{0x33B2, 0x1A},
	{0x33B3, 0x04},

	/*ID6*/
	{0x3405, 0x00}, /* frame rate 60*/
	{0x3407, 0x01}, /* 2 Lane*/
	{0x3414, 0x0A}, /* OB size*/
	{0x3418, 0x48}, /* Y OUT SIZE*/
	{0x3419, 0x04},
	{0x3441, 0x0C}, /* RAW12*/
	{0x3442, 0x0C},
	{0x3443, 0x01}, /* CSI LANE MODE*/
	{0x3444, 0x20}, /* EXTCLK 37.125M*/
	{0x3445, 0x25},
	{0x3446, 0x77},
	{0x3448, 0x67},
	{0x344A, 0x47},
	{0x344C, 0x37},
	{0x344E, 0x3F},
	{0x3450, 0xFF},
	{0x3452, 0x3F},

	{0x3454, 0x37},
	{0x3472, 0x9C}, /* X OUT SIZE*/
	{0x3473, 0x07},

	/*MISC*/
	/* LVDS 2CH CSI2  , 1080p Mode*/
	{0x305C, 0x18}, /* INCKSEL1*/
	{0x305D, 0x03}, /* INCKSEL2*/
	{0x305E, 0x20}, /* INCKSEL3*/
	{0x305F, 0x01}, /* INCKSEL4*/
	{0x315E, 0x1A}, /* INCKSEL5*/
	{0x3164, 0x1A}, /* INCKSEL6*/
	{0x3480, 0x49}, /* INCKSEL7*/

	{0x3048, 0x30}, /* XVS PULSE WIDTH=8H  = imx290_id02,*/

	{0x3049, 0x38}, /* XHS PULSE max*/
	{0x3005, 0x01}, /* AD bit 12*/

	/* TEST MODE */
	// {0x300a, 0x00},
	// {0x300b, 0x00},
	// {0x308c, 0x23},


	/* stream on */
	{0x3000, 0x00}, /* STANDBY CANCEL*/
	{IMX290_TABLE_WAIT_MS, 30},
	{0x3002, 0x00}, /* Master mode start*/
	{0x304B, 0x0A}, /* XVS Abd XHS O/P*/
	{IMX290_TABLE_END, 0x00}
};

/*
 * imx290 first step register configuration for
 * starting stream
 */
static const imx290_reg imx290_start_1[] = {
	{IMX290_STANDBY_REG, 0x12}, /* STANDBY */
	{IMX290_TABLE_END, 0x00}
};

/*
 * imx290 second step register configuration for
 * starting stream
 */
static const imx290_reg imx290_start_2[] = {
	{0x3120, 0xF0}, /* clock settings*/
	{0x3121, 0x00}, /* clock settings*/
	{0x3122, 0x02}, /* clock settings*/
	{0x3129, 0x9C}, /* clock settings*/
	{0x312A, 0x02}, /* clock settings*/
	{0x312D, 0x02}, /* clock settings*/

	{0x310B, 0x00},

	/* PLSTMG*/
	{0x304C, 0x00}, /* PLSTMG01*/
	{0x304D, 0x03},
	{0x331C, 0x1A},
	{0x331D, 0x00},
	{0x3502, 0x02},
	{0x3529, 0x0E},
	{0x352A, 0x0E},
	{0x352B, 0x0E},
	{0x3538, 0x0E},
	{0x3539, 0x0E},
	{0x3553, 0x00},
	{0x357D, 0x05},
	{0x357F, 0x05},
	{0x3581, 0x04},
	{0x3583, 0x76},
	{0x3587, 0x01},
	{0x35BB, 0x0E},
	{0x35BC, 0x0E},
	{0x35BD, 0x0E},
	{0x35BE, 0x0E},
	{0x35BF, 0x0E},
	{0x366E, 0x00},
	{0x366F, 0x00},
	{0x3670, 0x00},
	{0x3671, 0x00},

	/* PSMIPI*/
	{0x3304, 0x32}, /* PSMIPI1*/
	{0x3305, 0x00},
	{0x3306, 0x32},
	{0x3307, 0x00},
	{0x3590, 0x32},
	{0x3591, 0x00},
	{0x3686, 0x32},
	{0x3687, 0x00},

#ifdef TEST_PATTERN
	{0x303C, 0x11},
	/*{0x303D, 0x0A},  color bar*/
	{0x303D, 0x03}, /* gray image*/
	{0x370E, 0x01},
	{0x377F, 0x01},
	{0x3781, 0x01},
	{0x370B, 0x11}, /* enable*/
#endif
	{IMX290_TABLE_END, 0x00}
};

/*
 * imx290 third step register configuration for
 * starting stream
 */
static const imx290_reg imx290_start_3[] = {
	{IMX290_STANDBY_REG, 0x00}, /* STANDBY */
	{0x303E, 0x02}, /* SYS_MODE = 2*/
	{IMX290_TABLE_END, 0x00}
};

/*
 * imx290 forth step register configuration for
 * starting stream
 */
static const imx290_reg imx290_start_4[] = {
	{0x30F4, 0x00},
	{0x3018, 0xA2}, /* XHS VHS OUTUPT*/
	{IMX290_TABLE_END, 0x00}
};

/*
 * imx290 register configuration for starting stream
 */
static const imx290_reg imx290_start[] = {
	{IMX290_STANDBY_REG, 0x00},
	{IMX290_TABLE_END, 0x00}
};

/*
 * imx290 register configuration for stoping stream
 */
static const imx290_reg imx290_stop[] = {
	{IMX290_STANDBY_REG, 0x01}, /* STANDBY */
	{IMX290_TABLE_END, 0x00}
};

/*
 * imx290 register configuration for two line config stream
 */
static const imx290_reg imx290_2_lane_conf[] = {
	{0x312E, 0x01},
	{0x3AA2, 0x01},
	{IMX290_TABLE_END, 0x00}
};

/*
 * imx290 mode1(refer to datasheet) register configuration with
 * 1948x1096 resolution, raw12 data and mipi two lane output
 */
static const imx290_reg imx290_mode1_1948X1108_raw12[] = {
	{IMX290_TABLE_END, 0x00}
};

/*
 * imx290 mode related structure
 */
enum {
	IMX290_MODE_1920X1080,
	IMX290_MODE_START_STREAM,
	IMX290_MODE_STOP_STREAM,
	IMX290_MODE_2_LANE_MIPI
};
/*
 *imx290 register mode table for stream
 */
static const imx290_reg *mode_table[] = {
	[IMX290_MODE_1920X1080]		= imx290_mode_1920x1080,
	[IMX290_MODE_START_STREAM]			= imx290_start,
	[IMX290_MODE_STOP_STREAM]			= imx290_stop,
	[IMX290_MODE_2_LANE_MIPI]			= imx290_2_lane_conf,
};

static const int imx290_60fps[] = {
	60,
};

static const struct camera_common_frmfmt imx290_frmfmt[] = {
	{{1948, 1096},	imx290_60fps, 1, 0, IMX290_MODE_1920X1080},
};


#endif  /*{ __IMX290_TABLES__ }*/
