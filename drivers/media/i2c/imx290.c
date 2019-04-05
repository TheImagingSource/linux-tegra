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

#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>
#include <media/imx290.h>

#include "imx290_mode_tbls.h"

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
#define IMX290_MAX_FRAME_LENGTH		(0x2BFD)
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
#define IMX290_DEFAULT_MODE		(IMX290_MODE_1920X1080_60FPS)

/* default image output width */
#define IMX290_DEFAULT_WIDTH		(1948)
/* default image output height */
#define IMX290_DEFAULT_HEIGHT		(1096)
/* default image data format */
#define IMX290_DEFAULT_DATAFMT		(MEDIA_BUS_FMT_SRGGB12_1X12)
/* default output clk frequency for camera */
#define IMX290_DEFAULT_CLK_FREQ		(37125000)





struct imx290 {
	struct camera_common_power_rail	power;
	int				num_ctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;
	u32	frame_length;

	s32				group_hold_prev;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static int imx290_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops imx290_ctrl_ops = {
	.s_ctrl		= imx290_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx290_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX290_MIN_GAIN * 100,
		.max = IMX290_MAX_GAIN * 100,
		.def = IMX290_DEFAULT_GAIN * 100,
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

static int imx290_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err;
	struct imx290 *priv = (struct imx290 *)s_data->priv;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx290_write_table(struct imx290 *priv,
				const imx290_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 IMX290_TABLE_WAIT_MS,
					 IMX290_TABLE_END);
}

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

	if (pw->reset_gpio)
		gpio_direction_output(pw->reset_gpio, 0);
	if (pw->af_gpio)
		gpio_direction_output(pw->af_gpio, 1);
	if (pw->pwdn_gpio)
		gpio_direction_output(pw->pwdn_gpio, 0);
	usleep_range(10, 20);


/*	if (pw->dvdd)
		err = regulator_enable(pw->dvdd);
	if (err)
		goto imx290_dvdd_fail;*/

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto imx290_iovdd_fail;
/*
	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto imx290_avdd_fail;

	usleep_range(1, 2);
*/
	if (pw->reset_gpio)
		gpio_direction_output(pw->reset_gpio, 1);
	if (pw->pwdn_gpio)
		gpio_direction_output(pw->pwdn_gpio, 1);

	usleep_range(300, 310);

	pw->state = SWITCH_ON;
	return 0;

/*imx290_dvdd_fail:
	if (pw->af_gpio)
		gpio_direction_output(pw->af_gpio, 0);*/

imx290_iovdd_fail:
	regulator_disable(pw->dvdd);

/*imx290_avdd_fail:
	regulator_disable(pw->iovdd);*/

	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

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

	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_direction_output(pw->reset_gpio, 0);
	if (pw->af_gpio)
		gpio_direction_output(pw->af_gpio, 0);
	if (pw->pwdn_gpio)
		gpio_direction_output(pw->pwdn_gpio, 0);
	usleep_range(1, 2);

/*	if (pw->avdd)
		regulator_disable(pw->avdd);*/
	if (pw->iovdd)
		regulator_disable(pw->iovdd);
/*	if (pw->dvdd)
		regulator_disable(pw->dvdd);*/

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

static int imx290_power_put(struct imx290 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	if (unlikely(!pw))
		return -EFAULT;

/*	if (likely(pw->avdd))
		regulator_put(pw->avdd);*/

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

/*	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);*/

/*	pw->avdd = NULL;*/
	pw->iovdd = NULL;
/*	pw->dvdd = NULL;*/

	return 0;
}

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
#if 0
	/* ananlog 2.7v */
	err |= camera_common_regulator_get(&priv->i2c_client->dev,
			&pw->avdd, pdata->regulators.avdd);
	/* digital 1.2v */
	err |= camera_common_regulator_get(&priv->i2c_client->dev,
			&pw->dvdd, pdata->regulators.dvdd);
	/* IO 1.8v */
#endif
	err |= camera_common_regulator_get(&priv->i2c_client->dev,
			&pw->iovdd, pdata->regulators.iovdd);

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;
		pw->af_gpio = pdata->af_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}

	pw->state = SWITCH_OFF;
	return err;
}

static int imx290_set_gain(struct imx290 *priv, s32 val);
static int imx290_set_frame_length(struct imx290 *priv, s32 val);
static int imx290_set_coarse_time(struct imx290 *priv, s32 val);

static int imx290_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx290 *priv = (struct imx290 *)s_data->priv;
	struct v4l2_control control;
	int err;

	dev_dbg(&client->dev, "%s++,enable:%d\n", __func__, enable);

	imx290_write_table(priv, mode_table[IMX290_MODE_STOP_STREAM]);

	if (!enable)
		return 0;

	dev_dbg(&client->dev, "%s mode[%d]\n", __func__, s_data->mode);

	err = imx290_write_table(priv, mode_table[s_data->mode]);
	if (err)
		goto exit;


	if (s_data->override_enable) {
		/* write list of override regs for the asking frame length, */
		/* coarse integration time, and gain.                       */
		control.id = TEGRA_CAMERA_CID_GAIN;
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
#if 0
	if (test_mode) {
		err = imx290_write_table(priv,
			mode_table[IMX290_MODE_TEST_PATTERN]);
		if (err)
			goto exit;
		}
#endif
	err = imx290_write_table(priv, mode_table[IMX290_MODE_START_STREAM]);
	if (err)
		goto exit;


	return 0;
exit:
	dev_dbg(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static int imx290_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

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

static int imx290_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx290 *priv = (struct imx290 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops imx290_subdev_video_ops = {
	.s_stream	= imx290_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status	= imx290_g_input_status,
};

static struct v4l2_subdev_core_ops imx290_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static struct v4l2_subdev_pad_ops imx290_subdev_pad_ops = {
	.set_fmt	= imx290_set_fmt,
	.get_fmt	= imx290_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size        = camera_common_enum_framesizes,
	.enum_frame_interval    = camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops imx290_subdev_ops = {
	.core	= &imx290_subdev_core_ops,
	.video	= &imx290_subdev_video_ops,
	.pad	= &imx290_subdev_pad_ops,
};

static struct of_device_id imx290_of_match[] = {
	{ .compatible = "nvidia,imx290", },
	{ },
};

static struct camera_common_sensor_ops imx290_common_ops = {
	.power_on = imx290_power_on,
	.power_off = imx290_power_off,
	.write_reg = imx290_write_reg,
	.read_reg = imx290_read_reg,
};

static int imx290_set_group_hold(struct imx290 *priv)
{
	int err;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		err = imx290_write_reg(priv->s_data,
				       IMX290_GROUP_HOLD_ADDR, 0x1);
		if (err)
			goto fail;
		priv->group_hold_prev = 1;
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		err = imx290_write_reg(priv->s_data,
				       IMX290_GROUP_HOLD_ADDR, 0x0);
		if (err)
			goto fail;
		priv->group_hold_prev = 0;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: Group hold control error\n", __func__);
	return err;
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
	// int err;
	u16 gain;
	// int i;

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
	// for (i = 0; i < 1; i++) {
	// 	err = imx290_write_reg(priv->s_data, reg_list[i].addr,
	// 		(reg_list[i].val));
	// 	if (err)
	// 		goto fail;
	// }
	return 0;

// fail:
// 	dev_dbg(&priv->i2c_client->dev,
// 		 "%s: GAIN control error\n", __func__);
// 	return err;
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


	/* If the image is flashing */
	/* you need to reduce the value of ae to a certain value */
	/*and divide it by 2 */

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
static int imx290_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx290 *priv =
		container_of(ctrl->handler, struct imx290, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
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

MODULE_DEVICE_TABLE(of, imx290_of_match);

static struct camera_common_pdata *imx290_parse_dt(struct i2c_client *client,
				const struct camera_common_data *s_data)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

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

	board_priv_pdata->reset_gpio = of_get_named_gpio(node,
			"reset-gpios", 0);

/*	of_property_read_string(node, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	of_property_read_string(node, "dvdd-reg",
			&board_priv_pdata->regulators.dvdd);*/
	of_property_read_string(node, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}

static int imx290_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_dbg(&client->dev, "%s:\n", __func__);


	return 0;
}

static const struct v4l2_subdev_internal_ops imx290_subdev_internal_ops = {
	.open = imx290_open,
};

static const struct media_entity_operations imx290_media_ops = {
#ifdef CONFIG_MEDIA_CONTROLLER
	.link_validate = v4l2_subdev_link_validate,
#endif
};

static int imx290_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct imx290 *priv;
	int err;

	pr_info("[IMX290]: probing v4l2 sensor.\n");

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

	priv->pdata = imx290_parse_dt(client, common_data);
	if (!priv->pdata) {
		dev_err(&client->dev, " unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops		= &imx290_common_ops;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->dev		= &client->dev;
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
		dev_err(&client->dev, "Failed to initialize imx290.\n");
		return err;
	}

	v4l2_i2c_subdev_init(priv->subdev, client, &imx290_subdev_ops);

	err = imx290_ctrls_init(priv);
	if (err)
		return err;


	priv->subdev->internal_ops = &imx290_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			       V4L2_SUBDEV_FL_HAS_EVENTS;

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

	dev_dbg(&client->dev, "Detected IMX290 sensor\n");

	return 0;
}

static int
imx290_remove(struct i2c_client *client)
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

static const struct i2c_device_id imx290_id[] = {
	{ "imx290", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx290_id);

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

MODULE_DESCRIPTION("Media Controller driver for Sony IMX290");
MODULE_AUTHOR("Josh Kuo <joshk@nvidia.com>");
MODULE_LICENSE("GPL v2");
