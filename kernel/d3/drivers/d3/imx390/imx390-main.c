/**
 * @author Greg Rowe <growe@d3engineering.com>
 *
 * imx390 v4l2 driver for Nvidia Jetson
 *
 * Copyright (c) 2018-2019, D3 Engineering.  All rights reserved.
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
#include <linux/module.h>
#include <linux/of_device.h>

#include <media/camera_common.h>

#include <d3/d3-jetson-bsp.h>
#include <d3/ub960.h>

#include "imx390-modes.h"


/**
 * If @p expr evalutes to non-zero assign it to @p err and return @p err
 */
#define TRY(err, expr) if(((err) = (expr))) { return (err); }




/**
 * Register addresses (see data sheet/register map)
 */
enum {
	IMX390_REG_STANDBY = 0x0000,
	IMX390_REG_REG_HOLD = 0x0008,
	IMX390_REG_SHS1 = 0x000c,
	IMX390_REG_SHS2 = 0x0010,
	IMX390_REG_AGAIN_SP1H = 0x0018,
	IMX390_REG_AGAIN_SP1L = 0x001a,
	IMX390_REG_OBB_CLAMP_CTRL_SEL = 0x0083,
	IMX390_REG_REV1 = 0x3060,
	IMX390_REG_REV2 = 0x3064,
	IMX390_REG_REV3 = 0x3067,
	IMX390_REG_REAR_EMBDATA_LINE = 0x2E18,
};


/**
 * The 390 has 16 bit addresses and 8 bit values.
 */
static struct regmap_config imx390_regmap_cfg = {
	.reg_bits = 16,
	.val_bits = 8,
};


/**
 * The is the imx390 driver class. Some of the pointers are not
 * strictly needed as they can be obtained via things like
 * 'client'. These pointers exist for convenience.
 */
struct imx390 {
	struct camera_common_power_rail	power;

	struct i2c_client *client;
	struct device *dev;
	struct regmap *map;

	struct camera_common_data *s_data;
	struct camera_common_pdata *pdata;
	bool hflip;
	bool vflip;

	u32 frame_length;
	s64 last_wdr_et_val;

	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev *subdev;
	struct i2c_client *deserializer;

	/* @todo don't use this feature of C here, it's not necessary */
	struct v4l2_ctrl *ctrls[];
};


static int imx390_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int imx390_s_ctrl(struct v4l2_ctrl *ctrl);
static const struct v4l2_ctrl_ops imx390_ctrl_ops = {
	.g_volatile_ctrl = imx390_g_volatile_ctrl,
	.s_ctrl = imx390_s_ctrl,
};


static struct v4l2_ctrl_config ctrl_config_list[] = {
	{
		.ops = &imx390_ctrl_ops,
		.id = TEGRA_CAMERA_CID_SENSOR_MODE_ID,
		.name = "Sensor Mode",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = IMX390_MODE_ENDMARKER - 1,
		.def = IMX390_MODE_DEFAULT,
		.step = 1,
	},
	{			/* 0 */
		.ops = &imx390_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0 * FIXED_POINT_SCALING_FACTOR,
		.max = 30 * FIXED_POINT_SCALING_FACTOR,
		.def = 0 * FIXED_POINT_SCALING_FACTOR,
		.step = 3 * FIXED_POINT_SCALING_FACTOR / 10, /* 0.3 db */
	},
	{			/* 1 */
		.ops = &imx390_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_RATE,
		.name = "Frame Rate",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 30 * FIXED_POINT_SCALING_FACTOR,
		.max = 60 * FIXED_POINT_SCALING_FACTOR,
		.def = 60 * FIXED_POINT_SCALING_FACTOR,
		.step = 1 * FIXED_POINT_SCALING_FACTOR,
	},
	{			/* 2 */
		.ops = &imx390_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		/* 30 us  */
		.min = 30 * FIXED_POINT_SCALING_FACTOR / 1000000,
		/* 33.3 ms */
		.max = 33333 * FIXED_POINT_SCALING_FACTOR / 1000000,
		/* 30 ms */
		.def = 11000 * FIXED_POINT_SCALING_FACTOR / 1000000,
		/* Microsecond steps (should probably be in units of
		 * lines */
		.step = 1 * FIXED_POINT_SCALING_FACTOR / 1000000,
	},
	{			/* 3 */
		.ops = &imx390_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GROUP_HOLD,
		.name = "Group Hold",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{			/* 4 */
		.ops = &imx390_ctrl_ops,
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


/* Standard controls:

   V4L2_CID_HFLIP
   V4L2_CID_VFLIP

*/

#define NUM_CUSTOM_CONTROLS	ARRAY_SIZE(ctrl_config_list)
#define NUM_STD_CONTROLS	(2)
#define NUM_CONTROLS		(NUM_CUSTOM_CONTROLS + NUM_STD_CONTROLS)


/**
 * Reads a single register from the sensor and prints it.
 *
 * @param self
 * @param addr which register
 * @param out where to store result
 *
 * @return 0 on success
 */
static int imx390_reg_read(struct imx390 *self, u16 addr, u8 *out)
{
	int err = 0;
	unsigned int val = 0;
	if ((err = regmap_read(self->map, addr, &val))) {
		dev_warn(self->dev, "regmap_read returned %d", err);
		return err;
	}
	*out = val;
	dev_dbg(self->dev, "read: %#.4x=%#.2x", addr, *out);
	return 0;
}


/**
 * This is part of Nvidia camera_common framework. They create a file
 * in sysfs that you can use to read and write registers. That hooks
 * into this.
 *
 * @param s_data
 * @param addr address to read
 * @param val where to store value
 *
 * @return 0 on success
 */
static int imx390_read_reg(struct camera_common_data *s_data, u16 addr, u8 *val)
{
	int err = 0;
	struct imx390 *self = s_data->priv;

	TRY(err, imx390_reg_read(self, addr, val));
	return 0;
}


/**
 * This is part of Nvidia camera_common framework. They create a file
 * in sysfs that you can use to read and write registers. That hooks
 * into this.
 *
 * @param s_data
 * @param addr address to write
 * @param val what to write
 *
 * @return 0 on success
 */
static int imx390_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err = 0;
	struct imx390 *self = s_data->priv;

	TRY(err, regmap_write(self->map, addr, val));
	return 0;
}


static int imx390_mode_set(struct imx390 *self, enum imx390_mode mode)
{
	/* This entire function is a hack and I only allow it because
	 * it's for debugging purposes. */
	int err = 0;
	struct imx390_modes_map *pmode = NULL;

	if ((mode < 0)
	    || (mode > IMX390_MODE_ENDMARKER)) {
		dev_err(self->dev, "invalid mode %d (0-4)", mode);
		return -EINVAL;
	}
	pmode = &imx390_modes_map[mode];
	dev_info(self->dev, "setting mode: %s\n", pmode->desc);
	TRY(err, regmap_multi_reg_write(self->map, pmode->vals, *pmode->n_vals));
	return 0;
}


static int imx390_set_hflip_raw(struct imx390 *self, s32 val)
{
	int err;
	TRY(err, regmap_update_bits(self->map, 0x74, 1 << 1, (val & 1) << 1));
	TRY(err, regmap_update_bits(self->map, 0x3C0, 1 << 3, (val & 1) << 3));
	return 0;
}


static int imx390_set_vflip_raw(struct imx390 *self, s32 val)
{
	int err;
	TRY(err, regmap_update_bits(self->map, 0x74, 1 << 0, (val & 1) << 0));
	TRY(err, regmap_update_bits(self->map, 0x3C0, 1 << 2, (val & 1) << 2));
	return 0;
}


/**
 * This is called to start the stream. Currently this doesn't really
 * do anything other than log start/stop.
 *
 * @param sd sub device
 * @param enable enable/disable stream
 *
 * @return
 */
static int imx390_s_stream(struct v4l2_subdev *sd, int enable)
{
	int err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx390 *self = (struct imx390 *)s_data->priv;
	int mode_ix = self->s_data->sensor_mode_id;

	dev_dbg(self->dev, "%s", enable ? "START" : "STOP");

	if (self->deserializer) {
		ub960_s_stream(self->deserializer, self->client, enable);
	}

	if (enable) {
		TRY(err, imx390_mode_set(self, mode_ix));

		if (self->hflip)
			imx390_set_hflip_raw(self, self->hflip);
		if (self->vflip)
			imx390_set_vflip_raw(self, self->vflip);
	}

	TRY(err, regmap_write(self->map, IMX390_REG_STANDBY, enable ? 0:1));
	return 0;
}


/**
 * I am not sure what the intent of this function is.
 *
 * @param sd sub device
 * @param status
 *
 * @return 0, always
 */
static int imx390_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	return 0;
}


/**
 * Called by Linux when unloading the driver instance.
 *
 * @param client which instance
 *
 * @return 0, always
 */
static int imx390_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	dev_dbg(dev, "removed");
	return 0;
}


static const struct i2c_device_id imx390_id[] = {
	{"imx390", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, imx390_id);


static struct v4l2_subdev_video_ops imx390_subdev_video_ops = {
	.s_stream	= imx390_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status	= imx390_g_input_status,
};


static struct v4l2_subdev_core_ops imx390_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};


static int imx390_power_on(struct camera_common_data *s_data)
{
	/* int err = 0; */
	struct imx390 *priv = (struct imx390 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(priv->dev, "%s: power on\n", __func__);
	pw->state = SWITCH_ON;
	return 0;
}


static int imx390_power_off(struct camera_common_data *s_data)
{
	/* int err = 0; */
	struct imx390 *priv = (struct imx390 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(priv->dev, "%s: power off\n", __func__);
	pw->state = SWITCH_OFF;
	return 0;
}


/**
 * v4l2 set and tryset format handler
 *
 * @param sd sub dev
 * @param cfg
 * @param format format to attempt to set or to set
 *
 * @return 0 on success
 */
static int imx390_set_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_pad_config *cfg,
                          struct v4l2_subdev_format *format)
{
	/* struct camera_common_data *s_data = to_camera_common_data(sd->dev); */
	int err = 0;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		TRY(err, camera_common_try_fmt(sd, &format->format));
	}
	else {
		TRY(err, camera_common_s_fmt(sd, &format->format));
	}
	return 0;
}


/**
 * Returns the active format
 *
 * @param sd sub dev
 * @param cfg
 * @param format output - the active format
 *
 * @return
 */
static int imx390_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_pad_config *cfg,
                          struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}


static struct v4l2_subdev_pad_ops imx390_subdev_pad_ops = {
	.set_fmt	     = imx390_set_fmt,
	.get_fmt	     = imx390_get_fmt,
	.enum_mbus_code	     = camera_common_enum_mbus_code,
	.enum_frame_size     = camera_common_enum_framesizes,
	.enum_frame_interval = camera_common_enum_frameintervals,
};


static struct v4l2_subdev_ops imx390_subdev_ops = {
	.core  = &imx390_subdev_core_ops,
	.video = &imx390_subdev_video_ops,
	.pad   = &imx390_subdev_pad_ops,
};


static struct of_device_id imx390_of_match[] = {
	{ .compatible = "d3,imx390"},
	{ },
};
MODULE_DEVICE_TABLE(of, imx390_of_match);


/**
 * These are for debugging. The Nvidia camera_common code creates a
 * file in sysfs to read and write the image sensor.
 */
static struct camera_common_sensor_ops imx390_common_ops = {
	.power_on = imx390_power_on,
	.power_off = imx390_power_off,
	.write_reg = imx390_write_reg,
	.read_reg  = imx390_read_reg,
};


/**
 * Initializes v4l2 controls (taken from example code).
 */
static int imx390_ctrls_init(struct imx390 *self)
{
	struct v4l2_ctrl *ctrl;
	int err;
	int custom_index;
	int ctrl_index;

	dev_dbg(self->dev, "%s++", __func__);

	v4l2_ctrl_handler_init(&self->ctrl_handler, NUM_CONTROLS);

	ctrl_index = 0;
	ctrl = v4l2_ctrl_new_std(&self->ctrl_handler, &imx390_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (ctrl == NULL) {
		dev_err(self->dev, "Error initializing standard control\n");
		err = -EINVAL;
		goto error;
	}
	self->ctrls[ctrl_index++] = ctrl;

	ctrl = v4l2_ctrl_new_std(&self->ctrl_handler, &imx390_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (ctrl == NULL) {
		dev_err(self->dev, "Error initializing standard control\n");
		err = -EINVAL;
		goto error;
	}
	self->ctrls[ctrl_index++] = ctrl;

	for (custom_index = 0;
	     custom_index < NUM_CUSTOM_CONTROLS;
	     custom_index++) {

		ctrl = v4l2_ctrl_new_custom(&self->ctrl_handler,
					    &ctrl_config_list[custom_index],
					    NULL);
		if (ctrl == NULL)
		{
			dev_err(self->dev, "Failed to init %s ctrl",
				ctrl_config_list[custom_index].name);
			continue;
		}

		if (ctrl_config_list[custom_index].type == V4L2_CTRL_TYPE_STRING &&
		    ctrl_config_list[custom_index].flags & V4L2_CTRL_FLAG_READ_ONLY)
		{
			/* @todo should use imx390_kzalloc */
			ctrl->p_new.p_char = devm_kzalloc(
				self->dev,
				ctrl_config_list[custom_index].max + 1, GFP_KERNEL);
			if (!ctrl->p_new.p_char) {
				err = -ENOMEM;
				goto error;
			}
		}
		self->ctrls[ctrl_index++] = ctrl;
	}

	BUG_ON(ctrl_index != NUM_CONTROLS);

	self->subdev->ctrl_handler = &self->ctrl_handler;
	if (self->ctrl_handler.error) {
		dev_err(self->dev, "Error %d adding controls",
			self->ctrl_handler.error);
		err = self->ctrl_handler.error;
		goto error;
	}

	if ((err = v4l2_ctrl_handler_setup(&self->ctrl_handler))) {
		dev_err(self->dev, "Error %d setting default controls", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&self->ctrl_handler);
	return err;
}


/**
 * Called when the device is opened. For example this is called when
 * you execute `media-ctl -p` from userspace.
 *
 * @param sd sub device
 * @param fh
 *
 * @return 0, always
 */
static int imx390_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_dbg(&client->dev, "open");
	return 0;
}


static const struct v4l2_subdev_internal_ops imx390_subdev_internal_ops = {
	.open = imx390_open,
};


static const struct media_entity_operations imx390_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};


static int imx390_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx390 *self =
		container_of(ctrl->handler, struct imx390, ctrl_handler);

	switch (ctrl->id)
	{
	default:
		dev_err(self->dev, "%s: unknown ctrl id", __func__);
		return -EINVAL;
	}

	return 0;
}


/**
 * Puts the sensor in (or takes out of) register group hold. This
 * means that you can make changes to the sensor through multiple
 * registers and have all of the changes activate at the same time.
 *
 * @param self driver instance
 * @param val boolean enable or disable
 *
 * @return 0 on success
 */
static int imx390_group_hold_enable(struct imx390 *self, s32 val)
{
	/* dev_dbg(self->dev, "group hold: %d", val); */
	return regmap_write(self->map, IMX390_REG_REG_HOLD, val ? 1:0);
}


static int imx390_is_hdr(struct imx390 *self)
{
	int mode_ix = self->s_data->sensor_mode_id;
	return imx390_modes_formats[mode_ix].hdr_en;
}


/**
 * Sets the gain using a raw register value to both SPI1H and SPI1L
 *
 * @param self driver instance
 * @param gain raw register value written to SP1H and SP1L
 *
 * @return 0 on success
 */
static int imx390_gain_raw_set(struct imx390 *self, u16 gain)
{
	int err = 0;
	/* This register holds an 11 bit value */
	u16 masked = gain & 0x7ff;


	/* This should never be called in HDR mode but we'll put check
	 * in to be safe. */
	if (imx390_is_hdr(self)) {
		return 0;
	}

	dev_dbg(self->dev, "set gain: val=%#.2x", masked);

	TRY(err, imx390_group_hold_enable(self, 1));

	/* Set the analog gain registers. These are in .3 db steps. */
	TRY(err, regmap_write(self->map, IMX390_REG_AGAIN_SP1H, masked & 0xff));
	TRY(err, regmap_write(self->map, IMX390_REG_AGAIN_SP1H +1,
			      (masked >> 8) & 0xff));
	TRY(err, regmap_write(self->map, IMX390_REG_AGAIN_SP1L, masked & 0xff));
	TRY(err, regmap_write(self->map, IMX390_REG_AGAIN_SP1L + 1,
			      (masked >> 8) & 0xff));
	TRY(err, imx390_group_hold_enable(self, 0));
	return 0;
}


/**
 * Takes fixed point (Q42.22) gain value in decibels and programs the
 * image sensor.
 *
 * @param self driver instance
 * @param val gain, in decibels, in Q42.22 fixed point format
 *
 * @return 0 on success
 */
static int imx390_gain_set(struct imx390 *self, s64 val)
{
	/* Specifies the gain values from the user mode library. */
	/* It uses Q42.22 format (42 Bit integer, 22 Bit fraction). * */
	/* See imx185_set_gain function in imx185.c file. */

	/* imx390 gain is 0 to 30 in .3db steps. */
	u16 gain = 0;


	/* Gain is fixed for HDR mode. Sony recommends fixing exposure
	 * to 11ms, and having two gains: one for low light conditions
	 * and one for brighter conditions. We could add another HDR
	 * mode to accommodate that. */
	if (imx390_is_hdr(self)) {
		return 0;
	}

	gain = val * 10 / 3 / FIXED_POINT_SCALING_FACTOR;

	dev_dbg(self->dev, "gain: val=%#llx reg=%#x", val, gain);

	if (gain > 100) {
		dev_warn(self->dev, "invalid gain: reg=%#x", gain);
		gain = 100;
	}
	return imx390_gain_raw_set(self, gain);
}

static int imx390_set_hflip(struct imx390 *self, s32 val)
{
	struct camera_common_power_rail *pw = &self->power;
	self->hflip = val;
	if (pw->state == SWITCH_ON)
		return imx390_set_hflip_raw(self, val);
	return 0;
}


static int imx390_set_vflip(struct imx390 *self, s32 val)
{
	struct camera_common_power_rail *pw = &self->power;
	self->vflip = val;
	if (pw->state == SWITCH_ON)
		return imx390_set_vflip_raw(self, val);
	return 0;
}


/**
 * Sets the exposure time using a raw register value
 *
 * @param self driver instance
 * @param exp exposure time register value
 *
 * @return 0 on success
 */
static int imx390_exposure_raw_set(struct imx390 *self, u32 exp)
{
	struct reg_sequence writes[6] = {0};
	int err = 0;

	/* This should never be called in HDR mode but we'll put check
	 * in to be safe. */
	if (imx390_is_hdr(self)) {
		return 0;
	}

	dev_dbg(self->dev, "set exposure: reg=%#x", exp);
	TRY(err, imx390_group_hold_enable(self, 1));

	/* 20 bit value */
	writes[0].def = exp & 0xff;
	writes[1].def = (exp & 0xff00) >> 8;
	writes[2].def = (exp & 0xf0000) >> 16;

	/* 0xc, 0xd, 0xe */
	writes[0].reg = IMX390_REG_SHS1;
	writes[1].reg = IMX390_REG_SHS1 + 1;
	writes[2].reg = IMX390_REG_SHS1 + 2;

	/* 20 bit value */
	writes[3].def = exp & 0xff;
	writes[4].def = (exp & 0xff00) >> 8;
	writes[5].def = (exp & 0xf0000) >> 16;

	/* 0x10, 0x11, 0x12 */
	writes[3].reg = IMX390_REG_SHS2;
	writes[4].reg = IMX390_REG_SHS2 + 1;
	writes[5].reg = IMX390_REG_SHS2 + 2;


	/* True means to print the register values. This is a small
	 * table so it's OK. */
	TRY(err, regmap_multi_reg_write(self->map, writes, ARRAY_SIZE(writes)));
	TRY(err, imx390_group_hold_enable(self, 0));
	return 0;
}


/**
 * Sets the exposure time using a fixed point time setting in Q42.22
 * format.
 *
 * @param self driver instance
 * @param val exposure time in seconds (see description)
 *
 * @return 0 on success
 */
static int imx390_exposure_set(struct imx390 *self, s64 val)
{
	u32 coarse_time;
	u32 reg;
	struct camera_common_data *s_data = self->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];

	/* Exposure is fixed at 11ms for HDR. */
	if (imx390_is_hdr(self))
		return 0;

	/* This is figuring out how many lines are output for the
	 * desired exposure time. */
	/* pixel clock * TIME / line_length */
	coarse_time = mode->signal_properties.pixel_clock.val *
		val / mode->image_properties.line_length /
		FIXED_POINT_SCALING_FACTOR;

	/* The 390 is configured such that the SHS registers are the
	 * difference between VMAX and the exposure time expressed as
	 * lines.  */
	/* FRAME_LENGTH is VMAX */
	/* VMAX=1125 */
	reg =  1125 - coarse_time;
	/* The data sheet says values of 0 and 1 are prohibited...and
	 * also says that the default value is 1... */
	if (reg < 2) {
		dev_warn(self->dev, "exposure out of range, setting to 2");
		reg = 2;
	}
	else if (reg >= 0x100000) {
		dev_warn(self->dev, "exposure out of range, setting to max");
		reg = 0x100000 -1;
	}

	dev_dbg(self->dev,
		"pixel clock=%lld"
		", line-length=%d"
		", frame-length=%d"
		", coarse_time=%d"
		", val=%lld"
		", reg=%#x"
		", sf=%lld"
		,
		mode->signal_properties.pixel_clock.val,
		mode->image_properties.line_length,
		1125,
		coarse_time,
		val,
		reg,
		FIXED_POINT_SCALING_FACTOR);

	return imx390_exposure_raw_set(self, reg);
}


/**
 * v4l2 control handler
 *
 * @param ctrl the v4l2 control
 *
 * @return 0 on success
 */
static int imx390_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx390 *self =
		container_of(ctrl->handler, struct imx390, ctrl_handler);
	int err = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
		/* @todo we can't really change modes until we reset
		 * the image sensor. The only way I can find to do
		 * that is the xclr pin. While this isn't impossible
		 * in a serdes configration it isn't easy right now. */
		self->s_data->sensor_mode_id = *ctrl->p_new.p_s64;
		return 0;
	case V4L2_CID_HFLIP:
		TRY(err, imx390_set_hflip(self, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		TRY(err, imx390_set_vflip(self, ctrl->val));
		break;
	case TEGRA_CAMERA_CID_GAIN:
		TRY(err, imx390_gain_set(self, *ctrl->p_new.p_s64));
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		/* @todo do we need to implement this? */
		return 0;
		break;
	case TEGRA_CAMERA_CID_EXPOSURE:
		TRY(err, imx390_exposure_set(self, *ctrl->p_new.p_s64));
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		TRY(err, imx390_group_hold_enable(self, ctrl->val));
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		break;
	default:
		dev_err(self->dev, "unknown ctrl id=%d", ctrl->id);
		return -EINVAL;
	}

	return 0;
}


/**
 * Helper function that allocated device managed memory.
 *
 * @param dev device driver
 * @param len desired amount of memory
 * @param out where to store pointer
 *
 * @return 0 on success
 */
static int imx390_kzalloc(struct device *dev, size_t len, void *out)
{
	void **real_out = out;
	if (!(*real_out = devm_kzalloc(dev, len, GFP_KERNEL))) {
		dev_err(dev, "memory allocation failure");
		return -ENOMEM;
	}
	return 0;
}


/**
 * Allocates a device managed register map.
 *
 * @param client which i2c client to use
 * @param cfg register map configuration
 * @param out_map where to store instance
 *
 * @return 0 on success
 */
static int imx390_regmap_init(struct i2c_client *client,
                              struct regmap_config *cfg,
                              struct regmap **out_map)
{
	if (!(*out_map = devm_regmap_init_i2c(client, cfg))) {
		dev_err(&client->dev, "regmap_init failed");
		return -EINVAL;
	}
	return 0;
}


/**
 * Prints the silicon revision
 *
 * @return 0 on success
 */
static int imx390_revision_report(struct imx390 *self)
{
	int err = 0;
	/* It's time to make this a loop... */
	u8 rev1 = 0xff;
	u8 rev2 = 0xff;
	u8 rev3 = 0xff;
	int tries;

	for (tries = 50; --tries >= 0; ) {
		err = imx390_reg_read(self, IMX390_REG_REV1, &rev1);
		if (err) {
			usleep_range(10 * 1000, 10 * 1000);
			continue;
		}
		err = imx390_reg_read(self, IMX390_REG_REV2, &rev2);
		if (err) {
			usleep_range(10 * 1000, 10 * 1000);
			continue;
		}
		err = imx390_reg_read(self, IMX390_REG_REV3, &rev3);
		if (err) {
			usleep_range(10 * 1000, 10 * 1000);
			continue;
		}

		dev_info(self->dev,
			 "revision %#.2x = %#.2x, %#.2x = %#.2x %#.2x = %#.2x",
			 IMX390_REG_REV1, rev1,
			 IMX390_REG_REV2, rev2,
			 IMX390_REG_REV3, rev3);
		return 0;
	}

	return err;
}


static int imx390_deserializer_parse(struct imx390 *self,
				     struct i2c_client **out)
{
	struct device_node *node = self->client->dev.of_node;
	struct device_node *deserializer_node;
	struct i2c_client *deserializer_client;

	deserializer_node = of_parse_phandle(node, "deserializer", 0);
	if (!deserializer_node) {
		dev_dbg(self->dev, "could not find deserializer node");
		return -ENOENT;
	}

	deserializer_client = of_find_i2c_device_by_node(deserializer_node);
	of_node_put(deserializer_node);
	deserializer_node = NULL;

	if (!deserializer_client) {
		dev_dbg(self->dev, "missing deserializer client");
		return -ENOENT;
	}

	*out = deserializer_client;
	return 0;
}

/**
 * Pull parameters from device tree.
 *
 * @param s_data camera common data (for Nvidia layers)
 *
 * @return 0 on success
 */
static struct camera_common_pdata *imx390_parse_dt(
	struct imx390 *self,
	struct camera_common_data *s_data)
{
	struct device_node *node = self->client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node) {
		dev_err(self->dev, "no OF node");
		return NULL;
	}

	match = of_match_device(imx390_of_match, self->dev);
	if (!match) {
		dev_err(self->dev, "Failed to find matching dt id");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(self->dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_warn(self->dev, "mclk not in DT");

	if (imx390_deserializer_parse(self, &self->deserializer) == 0) {
		dev_dbg(self->dev, "deserializer present");
	}
	else {
		self->deserializer = NULL;
	}

	return board_priv_pdata;
}


/**
 * Initializes media controller
 *
 * @param self driver instance
 *
 * @return 0 on success
 */
static int imx390_media_init(struct imx390 *self)
{
	int err = 0;
#if defined(CONFIG_MEDIA_CONTROLLER)
	self->pad.flags = MEDIA_PAD_FL_SOURCE;
	self->subdev->entity.ops = &imx390_media_ops;
	/* The media controller code is slightly different on Tx2
	 * (kernel 4.4) vs. on Xavier (kernel 4.9) */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	self->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	err = media_entity_init(&self->subdev->entity, 1, &self->pad, 0);
#else
	err = tegra_media_entity_init(&self->subdev->entity, 1,
				      &self->pad, true, true);
#endif	/* LINUX_VERSION */
	if (err < 0) {
		dev_err(self->dev, "unable to init media entity");
		return err;
	}
#endif	/* CONFIG_MEDIA_CONTROLLER */
	return 0;
}


/**
 * This is called by the kernel when a new instance of the driver is
 * instantiated.
 *
 * @param client i2c client device
 * @param id
 *
 * @return 0 on success
 */
static int imx390_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
	int err = 0;
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct imx390 *self = NULL;

	dev_dbg(&client->dev, "probe");

	if (!IS_ENABLED(CONFIG_OF) || !node) {
		return -EINVAL;
	}

	TRY(err, imx390_kzalloc(&client->dev,
				sizeof(*common_data),
				&common_data));

	TRY(err, imx390_kzalloc(&client->dev,
				sizeof(*self) + sizeof(struct v4l2_ctrl *) *
				NUM_CONTROLS,
				&self));
	self->client = client;
	self->dev = &client->dev;

	common_data->priv = self;
	common_data->ops = &imx390_common_ops;
	common_data->ctrl_handler = &self->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = &imx390_modes_formats[0];
	common_data->colorfmt =
		camera_common_find_datafmt(MEDIA_BUS_FMT_SRGGB12_1X12);

	/* struct dentry				*debugdir; */
	/* common_data->debugdir = ; */
	common_data->power = &self->power;
	/* struct v4l2_subdev			subdev; */
	/* common_data->subdev = ; */
	common_data->ctrls = self->ctrls;
	/* struct sensor_properties		sensor_props; */
	/* common_data->sensor_props = ; */

	common_data->priv = self;
	common_data->numctrls = NUM_CONTROLS;
	/* handled by camera_common_initialize */
	/* common_data->csi_port = ; */
	/* common_data->numlanes = ; */

	/* handled in camera_common_try_fmt */
	/* common_data->mode = ; */
	/* common_data->mode_prop_idx; */

	common_data->numfmts = imx390_modes_formats_len;
	dev_dbg(self->dev, "num fmts %d", common_data->numfmts);
	common_data->def_mode = IMX390_MODE_DEFAULT;
	common_data->def_width =
		common_data->frmfmt[IMX390_MODE_DEFAULT].size.width;
	common_data->def_height =
		common_data->frmfmt[IMX390_MODE_DEFAULT].size.height;
	/* I'm assuming this is the default pixel clock */
	common_data->def_clk_freq = 1485000;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;

	self->s_data = common_data;
	self->subdev = &common_data->subdev;
	self->subdev->dev = self->dev;
	self->s_data->dev = self->dev;
	self->s_data->mode = IMX390_MODE_DEFAULT;

	self->pdata = imx390_parse_dt(self, common_data);
	if (!self->pdata)
		return -EFAULT;

	TRY(err, imx390_regmap_init(self->client,
				    &imx390_regmap_cfg,
				    &self->map));
	TRY(err, imx390_revision_report(self));

	TRY(err, camera_common_initialize(common_data, "imx390"));

	v4l2_i2c_subdev_init(self->subdev, client, &imx390_subdev_ops);
	TRY(err, imx390_ctrls_init(self));

	self->subdev->internal_ops = &imx390_subdev_internal_ops;
	self->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		V4L2_SUBDEV_FL_HAS_EVENTS;

	TRY(err, imx390_media_init(self));
	TRY(err, v4l2_async_register_subdev(self->subdev));

	TRY(err, regmap_write(self->map, IMX390_REG_STANDBY, 1));
	dev_dbg(self->dev, "probe success");
	return 0;
}


/**
 * Linux driver data
 */
static struct i2c_driver imx390_driver = {
	.driver = {
		.name = "imx390",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx390_of_match),
	},
	.probe = imx390_probe,
	.remove = imx390_remove,
	.id_table = imx390_id,
};
module_i2c_driver(imx390_driver);

MODULE_DESCRIPTION("Driver for D3's Sony IMX390 camera for Nvidia Jetson");
MODULE_VERSION(D3_JETSON_BSP_VERSION);
MODULE_AUTHOR("Greg Rowe <growe@d3engineering.com>");
MODULE_LICENSE("GPL v2");
