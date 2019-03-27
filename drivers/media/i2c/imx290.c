/*
 * imx290.c - imx290 sensor driver
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/camera_common.h>
#include <media/imx290.h>
#include "imx290_mode_tbls.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

/* differ to max coarse */
#define IMX290_MAX_COARSE_DIFF		(1)
/* max gain reg value */
#define IMX290_GAIN_REG_MAX		(0x00F0)
/* min gain reg value */
#define IMX290_GAIN_REG_MIN		(0x01)
/* minimum gain value */
#define IMX290_MIN_GAIN			(1)
/* maximum gain value */
#define IMX290_MAX_GAIN			(31)
/* gain shift bits */
#define IMX290_GAIN_SHIFT		8
/* minimum frame length */
#define IMX290_MIN_FRAME_LENGTH		(1125)
/* maximum frame length */
#define IMX290_MAX_FRAME_LENGTH		(0x57FA)
/* minimum exposure coarse */
#define IMX290_MIN_EXPOSURE_COARSE	(1)
/* maximum exposure coarse */
#define IMX290_MAX_EXPOSURE_COARSE	\
	(IMX290_MAX_FRAME_LENGTH-IMX290_MAX_COARSE_DIFF)

/* default gain value */
#define IMX290_DEFAULT_GAIN		(IMX290_MIN_GAIN)
/* default frame length value */
#define IMX290_DEFAULT_FRAME_LENGTH	(1125)
/* default exposure coarse value */
#define IMX290_DEFAULT_EXPOSURE_COARSE	\
	(IMX290_DEFAULT_FRAME_LENGTH-IMX290_MAX_COARSE_DIFF)

/* default mode */
#define IMX290_DEFAULT_MODE		(IMX290_MODE_1920X1080)

/* default image output width */
#define IMX290_DEFAULT_WIDTH		(1948)
/* default image output height */
#define IMX290_DEFAULT_HEIGHT		(1096)
/* default image data format */
#define IMX290_DEFAULT_DATAFMT		(MEDIA_BUS_FMT_SRGGB12_1X12)
/* default output clk frequency for camera */
#define IMX290_DEFAULT_CLK_FREQ		(24000000)

/*
 * struct imx74 - imx290 structure
 * @power: Camera common power rail structure
 * @num_ctrls: The num of V4L2 control
 * @i2c_client: Pointer to I2C client
 * @subdev: Pointer to V4L2 subdevice structure
 * @pad: Media pad structure
 * @group_hold_prev: Group hold status
 * @group_hold_en: Enable/Disable group hold
 * @regmap: Pointer to regmap structure
 * @s_data: Pointer to camera common data structure
 * @p_data: Pointer to camera common pdata structure
 * @ctrls: Pointer to V4L2 control list
 */
struct imx290 {
	struct camera_common_power_rail	power;
	int				num_ctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;

	s32				group_hold_prev;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

/*
 * struct sensror_regmap_config - sensor regmap config structure
 * @reg_bits: Sensor register address width
 * @val_bits: Sensor register value width
 * @cache_type: Cache type
 * @use_single_rw: Indicate only read or write a single time
 */
static const struct regmap_config sensor_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.cache_type	= REGCACHE_RBTREE,
	.use_single_rw	= true,
};

/*
 * Function declaration
 */
static int imx290_s_ctrl(struct v4l2_ctrl *ctrl);
static int imx290_set_gain(struct imx290 *priv, s32 val);
static int imx290_set_frame_length(struct imx290 *priv, s32 val);
static int imx290_set_coarse_time(struct imx290 *priv, s32 val);

/*
 * imx290 V4L2 control operator
 */
static const struct v4l2_ctrl_ops imx290_ctrl_ops = {
	.s_ctrl			= imx290_s_ctrl,
};

/*
 * V4L2 control configuration list
 * the control Items includes gain, exposure,
 * frame rate, group hold and HDR
 */
static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx290_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX290_MIN_GAIN,
		.max = IMX290_MAX_GAIN,
		.def = IMX290_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &imx290_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX290_MIN_FRAME_LENGTH,
		.max = IMX290_MAX_FRAME_LENGTH,
		.def = IMX290_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &imx290_ctrl_ops,
		.id = TEGRA_CAMERA_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX290_MIN_EXPOSURE_COARSE,
		.max = IMX290_MAX_EXPOSURE_COARSE,
		.def = IMX290_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &imx290_ctrl_ops,
		.id = TEGRA_CAMERA_CID_COARSE_TIME_SHORT,
		.name = "Coarse Time Short",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX290_MIN_EXPOSURE_COARSE,
		.max = IMX290_MAX_EXPOSURE_COARSE,
		.def = IMX290_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &imx290_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GROUP_HOLD,
		.name = "Group Hold",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &imx290_ctrl_ops,
		.id = TEGRA_CAMERA_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
};

/*
 * imx290_calculate_frame_legnth_regs - Function for calculate frame length
 * register value
 * @regs: Pointer to imx290 reg structure
 * @frame_length: Frame length value
 *
 * This is used to calculate the frame length value for frame length register
 */
static inline void imx290_calculate_frame_length_regs(imx290_reg *regs,
						      u32 frame_length)
{
	regs->addr = IMX290_FRAME_LENGTH_ADDR_1;
	regs->val = (frame_length >> 16) & 0x01;
	(regs + 1)->addr = IMX290_FRAME_LENGTH_ADDR_2;
	(regs + 1)->val = (frame_length >> 8) & 0xff;
	(regs + 2)->addr = IMX290_FRAME_LENGTH_ADDR_3;
	(regs + 2)->val = (frame_length) & 0xff;
}

/*
 * imx290_calculate_coarse_time_regs - Function for calculate coarse time
 * register value
 * @regs: Pointer to imx290 reg structure
 * @coarse_time: Coarse time value
 *
 * This is used to get the coarse time value for coarse time register
 */
static inline void imx290_calculate_coarse_time_regs(imx290_reg *regs,
						     u32 coarse_time)
{
	regs->addr = IMX290_COARSE_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0x00ff;
	(regs + 1)->addr = IMX290_COARSE_TIME_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0x00ff;
}

/*
 * imx290_calculate_gain_regs - Function for calculate gain
 * register value
 * @regs: Pointer to imx290 reg structure
 * @gain: Gain value
 *
 * This is used to get the gain value for gain register
 */
static inline void imx290_calculate_gain_regs(imx290_reg *regs,
					      u16 gain)
{
	regs->addr = IMX290_ANALOG_GAIN_ADDR;
	regs->val = gain;
}

/*
 * imx290_read_reg - Function for reading register value
 * @s_data: Pointer to camera common data structure
 * @addr: Registr address
 * @val: Pointer to register value
 *
 * This function is used to read a register value for imx290
 *
 * Return: 0 on success, errors otherwise
 */
static inline int imx290_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct imx290 *priv = (struct imx290 *)s_data->priv;
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

/*
 * imx290_write_reg - Function for writing register value
 * @s_data: Pointer to camera common data structure
 * @addr: Registr address
 * @val: Register value
 *
 * This function is used to write a register value for imx290
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err = 0;
	struct imx290 *priv = (struct imx290 *)s_data->priv;
	int retries = 3;
	for (retries = 3; retries>0; retries--)
	{
		err = regmap_write(priv->regmap, addr, val);
		if (err)
		{
			pr_err("%s:i2c write failed, %x = %x\n",
				__func__, addr, val);
			usleep_range(10000, 11000);
		} else {
			retries = 0;
		}
	}

	return err;
}

/*
 * regmap_util_write_table_8 - Function for writing register table
 * @priv: Pointer to imx290 structure
 * @table: Table containing register values
 *
 * This is used to write register table into sensor's reg map.
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_write_table(struct imx290 *priv,
				const imx290_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 IMX290_TABLE_WAIT_MS,
					 IMX290_TABLE_END);
}

/*
 * imx290_power_on - Function to power on the camera
 * @s_data: Pointer to camera common data
 *
 * This is used to power on imx290 camera board
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx290 *priv = (struct imx290 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			pr_err("%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	gpio_set_value(priv->pdata->pwdn_gpio, 0);
	dev_info(&priv->i2c_client->dev, "imx290: Set pwdn to 0.");

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);

	usleep_range(10000, 20000);

	// if (pw->reset_gpio)
	// 	gpio_set_value_cansleep(pw->reset_gpio, 0);

	gpio_set_value(priv->pdata->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpio_set_value(priv->pdata->reset_gpio, 1);
	// if (pw->reset_gpio)
	// 	gpio_set_value_cansleep(pw->reset_gpio, 1);

	usleep_range(30000, 31000);

	pw->state = SWITCH_ON;
	return 0;
}

/*
 * imx290_power_off - Function to power off the camera
 * @s_data: Pointer to camera common data
 *
 * This is used to power off imx290 camera board
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx290 *priv = (struct imx290 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power off\n", __func__);

	if (priv->pdata->power_off) {
		err = priv->pdata->power_off(pw);
		if (err) {
			pr_err("%s failed.\n", __func__);
			return err;
		} else {
			goto power_off_done;
		}
	}

	gpio_set_value(priv->pdata->pwdn_gpio, 1);
	dev_info(&priv->i2c_client->dev, "imx290: Set pwdn to 1.");

	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value_cansleep(pw->reset_gpio, 0);

	usleep_range(1, 2);
	if (pw->iovdd)
		regulator_disable(pw->iovdd);

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

/*
 * imx290_power_put - Function to put power
 * @priv: Pointer to imx290 structure
 *
 * This is used to put power to tegra
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_power_put(struct imx290 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	pw->iovdd = NULL;
	/* because imx290 does not get power from Tegra */
	/* so this function doing nothing */

	return 0;
}

/*
 * imx290_power_get - Function to get power
 * @priv: Pointer to imx290 structure
 *
 * This is used to get power from tegra
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_power_get(struct imx290 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = priv->pdata->mclk_name ?
		    priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(&priv->i2c_client->dev,
			"unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = priv->pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(&priv->i2c_client->dev,
				"unable to get parent clcok %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	/* IO 1.8v */
	err |= camera_common_regulator_get(&priv->i2c_client->dev,
			&pw->iovdd, pdata->regulators.iovdd);

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;

	}

	pw->state = SWITCH_OFF;
	return err;
}

/*
 * imx290_start_stream - Function for starting stream per mode index
 * @priv: Pointer to imx290 structure
 * @mode: Mode index value
 *
 * This is used to start steam per mode index.
 * mode = 0, start stream for sensor Mode 0: 1948x1108/raw12/60fps
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_start_stream(struct imx290 *priv, int mode)
{
	int err = 0;
	err = imx290_write_table(priv, mode_table[IMX290_MODE_1920X1080]);
	if (err)
		return err;

	return 0;
}

/*
 * imx290_s_stream - It is used to start/stop the streaming.
 * @sd: V4L2 Sub device
 * @enable: Flag (True / False)
 *
 * This function controls the start or stop of streaming for the
 * imx290 sensor.
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx290 *priv = (struct imx290 *)s_data->priv;
	struct v4l2_control control;
	int err = 0;

	dev_dbg(&client->dev, "%s++\n", __func__);

	if (!enable)
		return imx290_write_table(priv,
			mode_table[IMX290_MODE_STOP_STREAM]);

	err = imx290_start_stream(priv, s_data->mode);
	if (err)
		goto exit;

	if (s_data->override_enable) {
		/* write list of override regs for the asking frame length, */
		/* coarse integration time, and gain.                       */
		control.id = V4L2_CID_GAIN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx290_set_gain(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error gain override\n", __func__);

		control.id = TEGRA_CAMERA_CID_FRAME_LENGTH;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx290_set_frame_length(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error frame length override\n", __func__);

		control.id = TEGRA_CAMERA_CID_COARSE_TIME;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx290_set_coarse_time(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error coarse time override\n", __func__);
	}

	return 0;
exit:
	dev_dbg(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

/*
 * imx290_get_fmt - Get the pad format
 * @sd: Pointer to V4L2 Sub device structure
 * @cfg: Pointer to sub device pad information structure
 * @fmt: Pointer to pad level media bus format
 *
 * This function is used to get the pad format information
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

/*
 * imx290_set_fmt - This is used to set the pad format
 * @sd: Pointer to V4L2 Sub device structure
 * @cfg: Pointer to sub device pad information structure
 * @format: Pointer to pad level media bus format
 *
 * This function is used to set the pad format
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

/*
 * imx290_g_input_status - This is used to get input status
 * @sd: Pointer to V4L2 Sub device structure
 * @status: Pointer to status
 *
 * This function is used to get input status
 *
 * Return: 0 on success
 */
static int imx290_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx290 *priv = (struct imx290 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

/*
 * Camera common sensor operations
 */
static struct camera_common_sensor_ops imx290_common_ops = {
	.power_on = imx290_power_on,
	.power_off = imx290_power_off,
	.write_reg = imx290_write_reg,
	.read_reg = imx290_read_reg,
};

/*
 * im290_set_group_hold - Function to hold the sensor register
 * @priv: Pinter to imx290 structure
 *
 * This is used to hold the imx290 sensor register
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_set_group_hold(struct imx290 *priv)
{
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		priv->group_hold_prev = 1;
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		priv->group_hold_prev = 0;
	}

	return 0;
}

 /* imx290_set_gain - Function called when setting analog gain
 * @priv: Pointer to device structure
 * @val: Value for gain
 *
 * Set the analog gain based on input value.
 * Range: [0, 72]
 *
 * Return: 0 on success, errors otherwise
*/
static int imx290_set_gain(struct imx290 *priv, s32 val)
{
	imx290_reg reg_list[1];
	int err;
	u16 gain;
	int i;
	dev_dbg(&priv->i2c_client->dev, "%s - val = %d\n", __func__, val);

	if (!priv->group_hold_prev)
		imx290_set_group_hold(priv);

	dev_dbg(&priv->i2c_client->dev, "input gain value: %d\n", val);

	if (val < IMX290_MIN_GAIN)
		val = IMX290_MIN_GAIN;
	else if (val > IMX290_MAX_GAIN)
		val = IMX290_MAX_GAIN;
	gain = val;
	imx290_calculate_gain_regs(reg_list, gain);
	for (i = 0; i < 1; i++) {
		err = imx290_write_reg(priv->s_data, reg_list[i].addr,
			(reg_list[i].val));
		if (err)
			goto fail;
	}
	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: GAIN control error\n", __func__);
	return err;
}

/*
 * imx290_set_frame_length - Function called when setting frame length
 * @priv: Pointer to device structure
 * @val: Variable for frame length (= VMAX, i.e. vertical drive period length)
 *
 * Set frame length based on input value.
 *
 * Return: 0 extra tokens at end ofon success, errors otherwise
 */
static int imx290_set_frame_length(struct imx290 *priv, s32 val)
{
	imx290_reg reg_list[3];
	int err;
	u32 frame_length = 0;
	int i;
	dev_dbg(&priv->i2c_client->dev, "%s length = %d\n", __func__, val);

	if (!priv->group_hold_prev)
		imx290_set_group_hold(priv);

	frame_length = (u32)val;

	if (frame_length < IMX290_MIN_FRAME_LENGTH)
		frame_length = IMX290_MIN_FRAME_LENGTH;

	imx290_calculate_frame_length_regs(reg_list, frame_length);

	dev_dbg(&priv->i2c_client->dev,
		"%s: val: %d\n", __func__, frame_length);

	for (i = 0; i < 3; i++) {
		err = imx290_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

/*
 * imx290_get_frame_length - Function for obtaining current frame length
 * @priv: Pointer to device structure
 * @val: Pointer to obainted value
 *
 * frame_length = vmax x (svr + 1), in unit of hmax.
 *
 * Return: 0 on success
 */
static int imx290_get_frame_length(struct imx290 *priv, s32 *val)
{
	int err;
	u32 vmax;
	u8 reg_val[3];
	/* vmax */
	err = imx290_read_reg(priv->s_data, IMX290_FRAME_LENGTH_ADDR_3,
			      &reg_val[0]);
	err |= imx290_read_reg(priv->s_data, IMX290_FRAME_LENGTH_ADDR_2,
			       &reg_val[1]);
	err |= imx290_read_reg(priv->s_data, IMX290_FRAME_LENGTH_ADDR_1,
			       &reg_val[2]);
	if (err)
		goto fail;
	vmax = ((reg_val[2] & 0x07) << 16) + (reg_val[1] << 8) + reg_val[0];
	*val = vmax;
	return 0;
fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s : Get frame_length error\n", __func__);
	return err;
}

/*
 * imx290_clamp_coarse_time - Function to clamp coarse time
 * @priv: Pointer to imx290 structure
 * @val: Pointer to coarse time value
 *
 * This is used to clamp coarse time for imx290 sensor.
 *
 * Return: 0 one success, errors otherwise
 */
static int imx290_clamp_coarse_time(struct imx290 *priv, s32 *val)
{
	int err;
	s32 frame_length;

	err = imx290_get_frame_length(priv, &frame_length);
	if (err)
		goto fail;

	if (frame_length == 0)
		frame_length = IMX290_MIN_FRAME_LENGTH;

	if (*val > frame_length - IMX290_MAX_COARSE_DIFF)
		*val = frame_length - IMX290_MAX_COARSE_DIFF;
	else if (*val < IMX290_MIN_EXPOSURE_COARSE)
		*val = IMX290_MIN_EXPOSURE_COARSE;

	*val = frame_length - *val;
	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s : EXPOSURE control error\n", __func__);
	return err;
}

/*
 * imx290_set_coarse_time - Function called when setting SHR value
 * @priv: Pointer to imx290 structure
 * @val: Value for exposure time in number of line_length, or [HMAX]
 *
 * Set SHR value based on input value.
 *
 * Return: 0 on success, errors otherwise
 */

static int imx290_set_coarse_time(struct imx290 *priv, s32 val)
{
	imx290_reg reg_list[2];
	int err;
	s32 coarse_time;
	int i;

	dev_dbg(&priv->i2c_client->dev, "%s coarse time = %d\n", __func__, val);

	if (!priv->group_hold_prev)
		imx290_set_group_hold(priv);
	coarse_time = (s32)val;

	dev_dbg(&priv->i2c_client->dev,
		"%s: input val: %d\n", __func__, coarse_time);

	err = imx290_clamp_coarse_time(priv, &coarse_time);

	imx290_calculate_coarse_time_regs(reg_list, coarse_time);
	dev_dbg(&priv->i2c_client->dev,
		"%s: set val: %d\n", __func__, coarse_time);

	for (i = 0; i < 2; i++) {
		err = imx290_write_reg(priv->s_data, reg_list[i].addr,
					reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;
fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s: COARSE_TIME control error\n", __func__);

	return err;
}
/*
 * imx290_verify_streaming - Function to verify sensor streaming
 * @priv: Pointer to imx290 structure
 *
 * This is used to verify imx290 sensor output steaming
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_verify_streaming(struct imx290 *priv)
{
	int err = 0;

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return err;

	err = imx290_s_stream(priv->subdev, true);
	if (err)
		goto error;

error:
	imx290_s_stream(priv->subdev, false);
	camera_common_s_power(priv->subdev, false);

	return err;
}

/*
 * imx290_s_ctrl - Function called for setting V4L2 control operations
 * @ctrl: Pointer to V4L2 control structure
 *
 * This is used to set V4L2 control operations
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx290 *priv =
		container_of(ctrl->handler, struct imx290, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		err = imx290_set_gain(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_FRAME_LENGTH:
		err = imx290_set_frame_length(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_COARSE_TIME:
		err = imx290_set_coarse_time(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_COARSE_TIME_SHORT:
		err = imx290_set_coarse_time(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		if (switch_ctrl_qmenu[ctrl->val] == SWITCH_ON) {
			priv->group_hold_en = true;
		} else {
			priv->group_hold_en = false;
			err = imx290_set_group_hold(priv);
		}
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

/*
 * imx290_ctrls_init - Function to initialize V4L2 controls
 * @priv: Pointer to imx290 structure
 *
 * This is used to initialize V4L2 controls for imx290 sensor
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_ctrls_init(struct imx290 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
		}
		priv->ctrls[i] = ctrl;
	}

	priv->num_ctrls = num_ctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

/*
 * imx290_open - Function to open camera device
 * @fh: Pointer to V4L2 subdevice structure
 *
 * This function does nothing
 *
 * Return: 0 on success
 */
static int imx290_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

/*
 * Media operations
 */
static const struct v4l2_subdev_video_ops imx290_subdev_video_ops = {
	.s_stream	= imx290_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status	= imx290_g_input_status,
};

static const struct v4l2_subdev_core_ops imx290_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static const struct v4l2_subdev_pad_ops imx290_subdev_pad_ops = {
	.set_fmt	= imx290_set_fmt,
	.get_fmt	= imx290_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size        = camera_common_enum_framesizes,
	.enum_frame_interval    = camera_common_enum_frameintervals,
};

static const struct v4l2_subdev_internal_ops imx290_subdev_internal_ops = {
	.open = imx290_open,
};

static const struct media_entity_operations imx290_media_ops = {
#ifdef CONFIG_MEDIA_CONTROLLER
	.link_validate = v4l2_subdev_link_validate,
#endif
};

static const struct v4l2_subdev_ops imx290_subdev_ops = {
	.core	= &imx290_subdev_core_ops,
	.video	= &imx290_subdev_video_ops,
	.pad	= &imx290_subdev_pad_ops,
};

static const struct of_device_id imx290_of_match[] = {
	{ .compatible = "nvidia,imx290" },
	{ },
};
MODULE_DEVICE_TABLE(of, imx290_of_match);

/*
 * im290_parse_dt - Function to parse device tree
 * @client: Pointer to I2C client structure
 *
 * This is used to parse imx290 device tree
 *
 * Return: Pointer to camera common pdata on success, NULL on error
 */
static struct camera_common_pdata *imx290_parse_dt(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int gpio, err;

	if (!node)
		return NULL;

	match = of_match_device(imx290_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, " Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(&client->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	err = camera_common_parse_clocks(&client->dev, board_priv_pdata);
	if (err) {
		dev_err(&client->dev, "Failed to find clocks\n");
		goto error;
	}

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(&client->dev, "mclk not in DT\n");

	 gpio = of_get_named_gpio(node,
			"reset-gpios", 0);

	board_priv_pdata->reset_gpio = (unsigned int)gpio;
	if (gpio < 0) {
		dev_err(&client->dev, "reset gpios not in DT\n");
		goto error;
	}

	gpio = of_get_named_gpio(node, "pwdn-gpios", 0);
	if (gpio < 0) {
		dev_err(&client->dev, "pwdn gpios not in DT\n");
		goto error;
	}
	board_priv_pdata->pwdn_gpio = (unsigned int)gpio;


	err = of_property_read_string(node, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);
	if (err) {
		dev_err(&client->dev, "iovdd-reg not in DT\n");
		goto error;
	}

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}

/*
 * imx290 probe - Function called for I2C driver
 * @client: Pointer to I2C client structure
 * @id: Pointer to I2C device id structure
 *
 * This is used to probe imx290 sensor
 *
 * Return: 0 on success, errors otherwise
 */
static int imx290_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct imx290 *priv;
	int err;

	dev_dbg(&client->dev, "Probing IMX290 sensor\n");
	dev_err(&client->dev, "Probing IMX290 sensor\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct imx290) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(ctrl_config_list),
			    GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	priv->pdata = imx290_parse_dt(client);
	if (!priv->pdata) {
		dev_err(&client->dev, " unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops		= &imx290_common_ops;
	common_data->dev		= &client->dev;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->frmfmt		= &imx290_frmfmt[0];
	common_data->colorfmt		= camera_common_find_datafmt(
					  IMX290_DEFAULT_DATAFMT);
	common_data->power		= &priv->power;
	common_data->ctrls		= priv->ctrls;
	common_data->priv		= (void *)priv;
	common_data->numctrls		= ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts		= ARRAY_SIZE(imx290_frmfmt);
	common_data->def_mode		= IMX290_DEFAULT_MODE;
	common_data->def_width		= IMX290_DEFAULT_WIDTH;
	common_data->def_height		= IMX290_DEFAULT_HEIGHT;
	common_data->fmt_width		= common_data->def_width;
	common_data->fmt_height		= common_data->def_height;
	common_data->def_clk_freq	= IMX290_DEFAULT_CLK_FREQ;

	priv->i2c_client		= client;
	priv->s_data			= common_data;
	priv->subdev			= &common_data->subdev;
	priv->subdev->dev		= &client->dev;

	err = imx290_power_get(priv);
	if (err)
		return err;

	err = camera_common_initialize(common_data, "imx290");
	if (err) {
		dev_err(&client->dev, "Failed to initialize imx290\n");
		return err;
	}

	v4l2_i2c_subdev_init(priv->subdev, client, &imx290_subdev_ops);

	err = imx290_ctrls_init(priv);
	if (err)
		return err;

	err = imx290_verify_streaming(priv);

	/* If the actual camera is different from the device tree, */
	/* all camera can't be used, */
	/* and you need to block it to make it different or open camera */

	priv->subdev->internal_ops = &imx290_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			       V4L2_SUBDEV_FL_HAS_EVENTS;


	gpio_set_value(priv->pdata->pwdn_gpio, 0);
	dev_info(&priv->i2c_client->dev, "imx290: Set pwdn to 0.");


#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	priv->subdev->entity.ops = &imx290_media_ops;
	err = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

	dev_info(&client->dev, "Detected IMX290 sensor\n");

	return 0;
}

/*
 * imx290_remove - Function called for I2C driver
 * @client: Pointer to I2C client structure
 *
 * This is used to remove imx290 sensor
 *
 * return: 0 one success
 */
static int imx290_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx290 *priv = (struct imx290 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	imx290_power_put(priv);
	camera_common_cleanup(s_data);

	return 0;
}

/*
 * Media related structure
 */
static const struct i2c_device_id imx290_id[] = {
	{ "imx290", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, imx290_id);

/*
* I2C related structure
*/
static struct i2c_driver imx290_i2c_driver = {
	.driver = {
		.name = "imx290",
		.owner = THIS_MODULE,
	},
	.probe = imx290_probe,
	.remove = imx290_remove,
	.id_table = imx290_id,
};

module_i2c_driver(imx290_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX290 sensor");
MODULE_AUTHOR("The Imaging Source Europe GmbH");
MODULE_LICENSE("GPL v2");