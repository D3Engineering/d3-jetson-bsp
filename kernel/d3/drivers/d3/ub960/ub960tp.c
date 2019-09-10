/**
 * @author Greg Rowe <growe@d3engineering.com>
 *
 * ub960 test pattern v4l2 driver for Nvidia Jetson camera platform
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

/**
 * If @p expr evalutes to non-zero assign it to @p err and return @p err
 */
#define TRY(err, expr) do {\
		err = expr; \
		if (err) { \
			return err; \
		} \
	} while (false)

static const int ub960tp_30fps[] = {
	30,
};


/**
 * Mode table as expected by Nvidia's camera_common framework
 */
static const struct camera_common_frmfmt ub960tp_formats[] = {
	/* The imager outputs 1936x1096 but includes 4 lines of SMPG
	 * data.  SMPG is Safety Mechanism Pattern Generator.  There is
	 * also 1 line of embedded data that cannot be disabled.  The
	 * embedded data line does NOT count towards the overall
	 * dimensions. */
	{
		.size = {1920, 1080},
		.framerates = ub960tp_30fps,
		.num_framerates = 1,
		.hdr_en = 0,
		.mode = 0
	},
};


/**
 * The is the ub960tp driver class. Some of the pointers are not
 * strictly needed as they can be obtained via things like
 * 'client'. These pointers exist for convenience.
 */
struct ub960tp {
	struct camera_common_power_rail	power;

	/* Nvidia's code requires that you have an i2c device. */
	struct i2c_client *client;
	struct device *dev;
	struct regmap *map;

	struct camera_common_data *s_data;
	struct camera_common_pdata *pdata;

	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev *subdev;
	struct v4l2_ctrl *ctrls[];
};


static int ub960tp_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int ub960tp_s_ctrl(struct v4l2_ctrl *ctrl);
static const struct v4l2_ctrl_ops ub960tp_ctrl_ops = {
	.g_volatile_ctrl = ub960tp_g_volatile_ctrl,
	.s_ctrl = ub960tp_s_ctrl,
};


static struct v4l2_ctrl_config ctrl_config_list[] = {
	{
		.ops = &ub960tp_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 1 * FIXED_POINT_SCALING_FACTOR,
		.max = 1 * FIXED_POINT_SCALING_FACTOR,
		.def = 1 * FIXED_POINT_SCALING_FACTOR,
		.step = 1 * FIXED_POINT_SCALING_FACTOR,
	},
	{
		.ops = &ub960tp_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_RATE,
		.name = "Frame Rate",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 30 * FIXED_POINT_SCALING_FACTOR,
		.max = 30 * FIXED_POINT_SCALING_FACTOR,
		.def = 30 * FIXED_POINT_SCALING_FACTOR,
		.step = 1 * FIXED_POINT_SCALING_FACTOR,
	},
	{
		.ops = &ub960tp_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		/* 30 us  */
		.min = 30 * FIXED_POINT_SCALING_FACTOR / 1000000,
		/* 33.3 ms */
		.max = 33333 * FIXED_POINT_SCALING_FACTOR / 1000000,
		/* 11 ms */
		.def = 11000 * FIXED_POINT_SCALING_FACTOR / 1000000,
		/* Microsecond steps */
		.step = 1 * FIXED_POINT_SCALING_FACTOR / 1000000,
	},
	{
		.ops = &ub960tp_ctrl_ops,
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
		.ops = &ub960tp_ctrl_ops,
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




/**
 * This is called to start the stream. Currently this doesn't really
 * do anything other than log start/stop.
 *
 * @param sd sub device
 * @param enable enable/disable stream
 *
 * @return
 */
static int ub960tp_s_stream(struct v4l2_subdev *sd, int enable)
{
	/* int err = 0; */
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ub960tp *self = (struct ub960tp *)s_data->priv;


	dev_dbg(self->dev, "%s", enable ? "START" : "STOP");
	/* TRY(err, ub960tp_reg_write(self, UB960TP_REG_STANDBY, enable ? 0:1)); */
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
static int ub960tp_g_input_status(struct v4l2_subdev *sd, u32 *status)
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
static int ub960tp_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	dev_info(dev, "remove");
	return 0;
}


static const struct i2c_device_id ub960tp_id[] = {
	{"ub960tp", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ub960tp_id);


static struct v4l2_subdev_video_ops ub960tp_subdev_video_ops = {
	.s_stream	= ub960tp_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status	= ub960tp_g_input_status,
};


static struct v4l2_subdev_core_ops ub960tp_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};


static int ub960tp_power_on(struct camera_common_data *s_data)
{
	/* int err = 0; */
	struct ub960tp *priv = (struct ub960tp *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(priv->dev, "%s: power on\n", __func__);
	pw->state = SWITCH_ON;
	return 0;

}

static int ub960tp_power_off(struct camera_common_data *s_data)
{
	/* int err = 0; */
	struct ub960tp *priv = (struct ub960tp *)s_data->priv;
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
static int ub960tp_set_fmt(struct v4l2_subdev *sd,
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


/**
 * Returns the active format
 *
 * @param sd sub dev
 * @param cfg
 * @param format output - the active format
 *
 * @return
 */
static int ub960tp_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}


static int ub960tp_write_reg(struct camera_common_data *s_data, u16 addr,
			     u8 val)
{
	return 0;
}


static int ub960tp_read_reg(struct camera_common_data *s_data, u16 addr,
			    u8 *val)
{
	return 0;
}



static struct v4l2_subdev_pad_ops ub960tp_subdev_pad_ops = {
	.set_fmt	     = ub960tp_set_fmt,
	.get_fmt	     = ub960tp_get_fmt,
	.enum_mbus_code	     = camera_common_enum_mbus_code,
	.enum_frame_size     = camera_common_enum_framesizes,
	.enum_frame_interval = camera_common_enum_frameintervals,
};


static struct v4l2_subdev_ops ub960tp_subdev_ops = {
	.core  = &ub960tp_subdev_core_ops,
	.video = &ub960tp_subdev_video_ops,
	.pad   = &ub960tp_subdev_pad_ops,
};


static struct of_device_id ub960tp_of_match[] = {
	{ .compatible = "d3,ub960tp"},
	{ },
};
MODULE_DEVICE_TABLE(of, ub960tp_of_match);


/**
 * These are for debugging. The Nvidia camera_common code creates a
 * file in sysfs to read and write the image sensor.
 */
static struct camera_common_sensor_ops ub960tp_common_ops = {
	.power_on = ub960tp_power_on,
	.power_off = ub960tp_power_off,
	.write_reg = ub960tp_write_reg,
	.read_reg  = ub960tp_read_reg,
};


/**
 * Initializes v4l2 controls (taken from example code).
 */
static int ub960tp_ctrls_init(struct ub960tp *self)
{
	struct v4l2_ctrl *ctrl;
	int numctrls;
	int err;
	int i;

	dev_dbg(self->dev, "%s++", __func__);

	numctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&self->ctrl_handler, numctrls);

	for (i = 0; i < numctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&self->ctrl_handler,
					    &ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(self->dev, "Failed to init %s ctrl",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
		    ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			/* @todo should use ub960tp_kzalloc */
			ctrl->p_new.p_char = devm_kzalloc(
						     self->dev,
						     ctrl_config_list[i].max + 1, GFP_KERNEL);
			if (!ctrl->p_new.p_char)
				return -ENOMEM;
		}
		self->ctrls[i] = ctrl;
	}

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
static int ub960tp_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_dbg(&client->dev, "open");
	return 0;
}


static const struct v4l2_subdev_internal_ops ub960tp_subdev_internal_ops = {
	.open = ub960tp_open,
};


static const struct media_entity_operations ub960tp_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};


static int ub960tp_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ub960tp *self =
		container_of(ctrl->handler, struct ub960tp, ctrl_handler);

	switch (ctrl->id) {
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
static int ub960tp_group_hold_enable(struct ub960tp *self, s32 val)
{
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
static int ub960tp_gain_set(struct ub960tp *self, s64 val)
{
	return 0;
}


/**
 * This currently doesn't do anything
 *
 * @param self driver instance
 * @param val desired frame rate
 *
 * @return 0 on success
 */
static int ub960tp_framerate_set(struct ub960tp *self, s64 val)
{
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
static int ub960tp_exposure_set(struct ub960tp *self, s64 val)
{
	return 0;
}


/**
 * Enables or disables HDR mode.
 *
 * @param self driver instance
 * @param val enable or disable
 *
 * @return 0 on success
 */
static int ub960tp_hdr_enable(struct ub960tp *self, s32 val)
{
	return 0;
}


/**
 * v4l2 control handler
 *
 * @param ctrl the v4l2 control
 *
 * @return 0 on success
 */
static int ub960tp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ub960tp *self =
		container_of(ctrl->handler, struct ub960tp, ctrl_handler);
	int err = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		TRY(err, ub960tp_gain_set(self, *ctrl->p_new.p_s64));
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		TRY(err, ub960tp_framerate_set(self, *ctrl->p_new.p_s64));
		break;
	case TEGRA_CAMERA_CID_EXPOSURE:
		TRY(err, ub960tp_exposure_set(self, *ctrl->p_new.p_s64));
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		TRY(err, ub960tp_group_hold_enable(self, ctrl->val));
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		TRY(err, ub960tp_hdr_enable(self, ctrl->val));
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
static int ub960tp_kzalloc(struct device *dev, size_t len, void *out)
{
	void **real_out = out;
	if (!(*real_out = devm_kzalloc(dev, len, GFP_KERNEL))) {
		dev_err(dev, "memory allocation failure");
		return -ENOMEM;
	}
	return 0;
}




/**
 * Pull parameters from device tree.
 *
 * @param s_data camera common data (for Nvidia layers)
 *
 * @return 0 on success
 */
static struct camera_common_pdata *ub960tp_parse_dt(
	struct ub960tp *self,
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

	match = of_match_device(ub960tp_of_match, self->dev);
	if (!match) {
		dev_err(self->dev, "Failed to find matching dt id");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(self->dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err) {
		/* This is not fatal */
		dev_warn(self->dev, "mclk not in DT");
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
static int ub960tp_media_init(struct ub960tp *self)
{
	int err = 0;
#if defined(CONFIG_MEDIA_CONTROLLER)
	self->pad.flags = MEDIA_PAD_FL_SOURCE;
	self->subdev->entity.ops = &ub960tp_media_ops;
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
static int ub960tp_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct ub960tp *self = NULL;

	dev_dbg(&client->dev, "ub960tp probe ");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	TRY(err, ub960tp_kzalloc(&client->dev,
				 sizeof(*common_data),
				 &common_data));

	TRY(err, ub960tp_kzalloc(&client->dev,
				 sizeof(*self) + sizeof(struct v4l2_ctrl *) *
				 ARRAY_SIZE(ctrl_config_list),
				 &self));
	self->client = client;
	self->dev = &client->dev;

	common_data->priv = self;
	common_data->ops = &ub960tp_common_ops;
	common_data->ctrl_handler = &self->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = &ub960tp_formats[0];
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
	common_data->numctrls = ARRAY_SIZE(ctrl_config_list);
	/* handled by camera_common_initialize */
	/* common_data->csi_port = ; */
	/* common_data->numlanes = ; */

	/* handled in camera_common_try_fmt */
	/* common_data->mode = ; */
	/* common_data->mode_prop_idx; */

	common_data->numfmts = ARRAY_SIZE(ub960tp_formats);
	common_data->def_mode = 0;
	common_data->def_width = common_data->frmfmt[0].size.width;
	common_data->def_height = common_data->frmfmt[0].size.height;
	common_data->def_clk_freq = 50000000;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;

	self->s_data = common_data;
	self->subdev = &common_data->subdev;
	self->subdev->dev = self->dev;
	self->s_data->dev = self->dev;

	self->pdata = ub960tp_parse_dt(self, common_data);
	if (!self->pdata)
		return -EFAULT;

	/* TRY(err, ub960tp_regmap_init(self->client, */
	/* 			    &ub960tp_regmap_cfg, */
	/* 			    &self->map)); */

	TRY(err, camera_common_initialize(common_data, "ub960tp"));

	v4l2_i2c_subdev_init(self->subdev, client, &ub960tp_subdev_ops);
	TRY(err, ub960tp_ctrls_init(self));

	self->subdev->internal_ops = &ub960tp_subdev_internal_ops;
	self->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			       V4L2_SUBDEV_FL_HAS_EVENTS;

	TRY(err, ub960tp_media_init(self));
	TRY(err, v4l2_async_register_subdev(self->subdev));

	/* If 390_mode control is removed you'll need to uncomment the
	 * write_table call below to initialize the sensor. */
	/* TRY(err, ub960tp_reg_write_table(self, */
	/* 				mode_1936x1100SP1H, */
	/* 				ARRAY_SIZE(mode_1936x1100SP1H), */
	/* 				false)); */
	dev_dbg(self->dev, "ub960tp probe success: addr=%#.2x", client->addr);
	return 0;
}

/**
 * Linux driver data
 */
static struct i2c_driver ub960tp_driver = {
	.driver =
	{
		.name = "ub960tp",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ub960tp_of_match),
	},
	.probe = ub960tp_probe,
	.remove = ub960tp_remove,
	.id_table = ub960tp_id,
};
module_i2c_driver(ub960tp_driver);

MODULE_DESCRIPTION("Test pattern camera driver for ub960 on Nvidia Jetson");
MODULE_VERSION(D3_JETSON_BSP_VERSION);
MODULE_AUTHOR("Greg Rowe <growe@d3engineering.com>");
MODULE_LICENSE("GPL v2");
