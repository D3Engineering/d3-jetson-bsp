/*
 * @author Mike Soltiz <msoltiz@d3engineering.com>
 *
 * vg6768.c - vg6768 sensor driver
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
/* Nvidia camera utility code */
#include <media/camera_common.h>

#include <d3/d3-jetson-bsp.h>

#include "vg6768_tables.h"


/**
 * If @p expr evalutes to non-zero assign it to @p err and return @p err
 */
#define TRY(err, expr) if(((err) = (expr))) { return (err); }

/**
 *  16 bit addresses and 8 bit values.
 */
static struct regmap_config vg6768_regmap_cfg =
{
	.reg_bits = 16,
	.val_bits = 8,
};

struct vg6768 {
	struct camera_common_power_rail	power;

	struct i2c_client *i2c_client;
	struct device *dev;
	struct regmap *map;

	struct camera_common_data *s_data;
	struct camera_common_pdata *pdata;

	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev *subdev;
	struct v4l2_ctrl *ctrls[];
};

static int vg6768_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int vg6768_s_ctrl(struct v4l2_ctrl *ctrl);
static const struct v4l2_ctrl_ops vg6768_ctrl_ops = {
	.g_volatile_ctrl = vg6768_g_volatile_ctrl,
	.s_ctrl = vg6768_s_ctrl,
};


static struct v4l2_ctrl_config ctrl_config_list[] = {
	{
		.ops = &vg6768_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0 * FIXED_POINT_SCALING_FACTOR,
		.max = 30 * FIXED_POINT_SCALING_FACTOR,
		.def = 0 * FIXED_POINT_SCALING_FACTOR,
		.step = 1,
	},
	{
		.ops = &vg6768_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EXPOSURE,
		.name = "Exposure (us)",
		.type = V4L2_CTRL_TYPE_INTEGER,
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
		.ops = &vg6768_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_RATE,
		.name = "Frame Rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 30,
		.max = 30,
		.def = 30,
		.step = 1,
	},
	{
		.ops = &vg6768_ctrl_ops,
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
		.ops = &vg6768_ctrl_ops,
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
 * Writes a single register value and prints debug information
 *
 * @param self
 * @param addr address
 * @param val value
 *
 * @return 0 on success
 */
static int vg6768_reg_write(struct vg6768 *self, u16 addr, u8 val)
{
	int err = 0;
	dev_dbg(self->dev, "write: %#.4x=%#.2x", addr, val);
	if((err = regmap_write(self->map, addr, val)))
	{
		dev_err(self->dev, "write error: %d", err);
		return err;
	}
	return 0;
}

/**
 * Reads a single register from the sensor and prints it.
 *
 * @param self
 * @param addr which register
 * @param out where to store result
 *
 * @return 0 on success
 */
static int vg6768_reg_read(struct vg6768 *self, u16 addr, u8 *out)
{
	int err = 0;
	unsigned int val = 0;
	if((err = regmap_read(self->map, addr, &val)))
	{
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
static int vg6768_read_reg(struct camera_common_data *s_data, u16 addr, u8 *val)
{
	int err = 0;
	struct vg6768 *self = s_data->priv;

	TRY(err, vg6768_reg_read(self, addr, val));
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
static int vg6768_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err = 0;
	struct vg6768 *self = s_data->priv;

	TRY(err, vg6768_reg_write(self, addr, val));
	return 0;
}

/**
 * Writes a sequence of registers.
 *
 * @param self driver instance
 * @param vals what to write
 * @param n_vals how many entries in @p vals
 * @param do_dbg true to pint debug information
 *
 * @return 0 on success
 */
static int vg6768_reg_write_table(struct vg6768 *self,
				  const struct reg_sequence *vals,
				  size_t n_vals,
				  bool do_dbg)
{
	int err = 0;
	size_t i = 0;
	if(do_dbg)
	{
		for(i = 0; i<n_vals; ++i)
		{
			dev_dbg(self->dev,
				"write table: %#.2x=%#.2x (delay=%dus)",
				vals[i].reg, vals[i].def, vals[i].delay_us);
		}
	}
	if((err = regmap_multi_reg_write(self->map, vals, n_vals)) <0)
	{
		dev_err(self->dev, "error writing registers: %d", err);
		return err;
	}
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
static int vg6768_s_stream(struct v4l2_subdev *sd, int enable)
{
	int err = 0;
	u8 *reg_val = &(u8){0};
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct vg6768 *self = (struct vg6768 *)s_data->priv;

	dev_dbg(&client->dev, "%s", enable ? "START" : "STOP");

	if (!enable)
	{
		msleep(50);
		TRY(err, vg6768_reg_write_table(self,
						vg6768_stop,
						ARRAY_SIZE(vg6768_stop),
						false));
		msleep(50);
	}
	else
	{
		msleep(50);
		TRY(err, vg6768_reg_write_table(self,
						vg6768_start,
						ARRAY_SIZE(vg6768_start),
						false));
		msleep(50);
	}

	TRY(err, vg6768_reg_read(self, 0x004E, reg_val));
	if (err==0)
		dev_dbg(self->dev, "vg6768 X_OUTPUT_SIZE: %d", *reg_val);

	TRY(err, vg6768_reg_read(self, 0x0050, reg_val));
	if (err==0)
		dev_dbg(self->dev, "vg6768 Y_OUTPUT_SIZE: %d", *reg_val);

	TRY(err, vg6768_reg_read(self, 0x0058, reg_val));
	if (err==0)
		dev_dbg(self->dev, "vg6768 FRAME_LENGTH: %d", *reg_val);

	TRY(err, vg6768_reg_read(self, 0x005A, reg_val));
	if (err==0)
		dev_dbg(self->dev, "vg6768 LINE_LENGTH: %d", *reg_val);

	return 0;
}

static int vg6768_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	return 0;
}


static int vg6768_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct vg6768 *priv = (struct vg6768 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	camera_common_cleanup(s_data);
	return 0;
}


static const struct i2c_device_id vg6768_id[] = {
	{"vg6768", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, vg6768_id);

static struct v4l2_subdev_video_ops vg6768_subdev_video_ops = {
	.s_stream	= vg6768_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status	= vg6768_g_input_status,
};


static struct v4l2_subdev_core_ops vg6768_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

/**
 * v4l2 set and tryset format handler
 *
 * @param sd sub dev
 * @param cfg
 * @param format format to attempt to set or to set
 *
 * @return 0 on success
 */
static int vg6768_set_fmt(struct v4l2_subdev *sd,
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
static int vg6768_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static struct v4l2_subdev_pad_ops vg6768_subdev_pad_ops = {
	.set_fmt	     = vg6768_set_fmt,
	.get_fmt	     = vg6768_get_fmt,
	.enum_mbus_code	     = camera_common_enum_mbus_code,
	.enum_frame_size     = camera_common_enum_framesizes,
	.enum_frame_interval = camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops vg6768_subdev_ops = {
	.core  = &vg6768_subdev_core_ops,
	.video = &vg6768_subdev_video_ops,
	.pad   = &vg6768_subdev_pad_ops,
};


static struct of_device_id vg6768_of_match[] = {
	{ .compatible = "d3,vg6768", },
	{ },
};
MODULE_DEVICE_TABLE(of, vg6768_of_match);

/**
 * These are for debugging. The Nvidia camera_common code creates a
 * file in sysfs to read and write the image sensor.
 */
static struct camera_common_sensor_ops vg6768_common_ops = {
	.write_reg = vg6768_write_reg,
	.read_reg  = vg6768_read_reg,
};

static int vg6768_ctrls_init(struct vg6768 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int numctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++", __func__);

	numctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, numctrls);

	for (i = 0; i < numctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
					    &ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
		    ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
							  ctrl_config_list[i].max + 1, GFP_KERNEL);
			if (!ctrl->p_new.p_char)
				return -ENOMEM;
		}
		priv->ctrls[i] = ctrl;
	}

	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

static int vg6768_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_dbg(&client->dev, "open");
	return 0;
}


static const struct v4l2_subdev_internal_ops vg6768_subdev_internal_ops = {
	.open = vg6768_open,
};


static const struct media_entity_operations vg6768_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int vg6768_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vg6768 *self =
		container_of(ctrl->handler, struct vg6768, ctrl_handler);

	switch (ctrl->id)
	{
	default:
		dev_err(self->dev, "%s: unknown ctrl id", __func__);
		return -EINVAL;
	}

	return 0;
}

static int vg6768_framerate_set(struct vg6768 *priv, s32 val)
{
	int err;

	dev_dbg(&priv->i2c_client->dev,
		"%s: frame_rate: %d\n", __func__, val);

	err = vg6768_write_reg(priv->s_data, 0x04FC, (uint8_t)val);
	err |= vg6768_write_reg(priv->s_data, 0x04FD, 0x00);
	err |= vg6768_write_reg(priv->s_data, 0x04FE, 0x00);
	err |= vg6768_write_reg(priv->s_data, 0x04FF, 0x00);
	err |= vg6768_write_reg(priv->s_data, 0x0500, 0x39);
	err |= vg6768_write_reg(priv->s_data, 0x0501, 0x00);
	err |= vg6768_write_reg(priv->s_data, 0x0502, 0x00);
	err |= vg6768_write_reg(priv->s_data, 0x0503, 0x00);
	if (err)
		goto fail;
	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: FRAME_RATE control error\n", __func__);
	return err;
}

static int vg6768_hdr_enable(struct vg6768 *priv, s32 val)
{
	int err;
	int hdr_en = switch_ctrl_qmenu[val];

	if (hdr_en == SWITCH_ON) {
		err = vg6768_write_reg(priv->s_data, 0x04FC, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x04FD, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x04FE, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x04FF, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x0500, 0x42);
		err |= vg6768_write_reg(priv->s_data, 0x0501, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x0502, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x0503, 0x00);
		if (err)
			goto fail;
	} else if (hdr_en == SWITCH_OFF) {
		err = vg6768_write_reg(priv->s_data, 0x04FC, 0x01);
		err |= vg6768_write_reg(priv->s_data, 0x04FD, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x04FE, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x04FF, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x0500, 0x42);
		err |= vg6768_write_reg(priv->s_data, 0x0501, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x0502, 0x00);
		err |= vg6768_write_reg(priv->s_data, 0x0503, 0x00);
		if (err)
			goto fail;
	}
	return 0;
fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: HDR control error\n", __func__);
	return err;
}

/**
 * v4l2 control handler
 *
 * @param ctrl the v4l2 control
 *
 * @return 0 on success
 */
static int vg6768_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vg6768 *priv =
		container_of(ctrl->handler, struct vg6768, ctrl_handler);
	int err = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		//err = vg6768_set_gain(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_EXPOSURE:
		//err = vg6768_set_exposure(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		TRY(err, vg6768_framerate_set(priv, ctrl->val));
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		//err = vg6768_set_group_hold(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		TRY(err, vg6768_hdr_enable(priv, ctrl->val));
		break;
	default:
		dev_err(priv->dev, "%s: unknown ctrl id.\n", __func__);
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
static int vg6768_kzalloc(struct device *dev, size_t len, void *out)
{
	void **real_out = out;
	if(!(*real_out = devm_kzalloc(dev, len, GFP_KERNEL)))
	{
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
static int vg6768_regmap_init(struct i2c_client *client,
			      struct regmap_config *cfg,
			      struct regmap **out_map)
{
	if(!(*out_map = devm_regmap_init_i2c(client, cfg)))
	{
		dev_err(&client->dev, "regmap_init failed");
		return -EINVAL;
	}
	return 0;
}

static int vg6768_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct vg6768 *self = NULL;

	dev_dbg(&client->dev, "vg6768 probe enter: addr=%#.2x", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	TRY(err, vg6768_kzalloc(&client->dev,
				sizeof(*common_data),
				&common_data));

	TRY(err, vg6768_kzalloc(&client->dev,
				sizeof(*self) + sizeof(struct v4l2_ctrl *) *
				ARRAY_SIZE(ctrl_config_list),
				&self));
	self->dev = &client->dev;

	common_data->priv = self;
	common_data->ops = &vg6768_common_ops;
	common_data->ctrl_handler = &self->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = &vg6768_frmfmt[0];
	common_data->colorfmt =
		camera_common_find_datafmt(MEDIA_BUS_FMT_RGB888_1X24);

	/* struct dentry				*debugdir; */
	/* common_data->debugdir = ; */
	common_data->power = &self->power;
	/* struct v4l2_subdev			subdev; */
	/* common_data->subdev = ; */
	common_data->ctrls = self->ctrls;
	/* struct sensor_properties		sensor_props; */
	/* common_data->sensor_props = ; */

	common_data->priv = (void *)self;
	common_data->numctrls = ARRAY_SIZE(ctrl_config_list);
	/* handled by camera_common_initialize */
	/* common_data->csi_port = ; */
	/* common_data->numlanes = ; */

	/* handled in camera_common_try_fmt */
	/* common_data->mode = ; */
	/* common_data->mode_prop_idx; */

	common_data->numfmts = ARRAY_SIZE(vg6768_frmfmt);
	common_data->def_mode = VG6768_DEFAULT_MODE;
	common_data->def_width =
		common_data->frmfmt[VG6768_DEFAULT_MODE].size.width;
	common_data->def_height =
		common_data->frmfmt[VG6768_DEFAULT_MODE].size.height;
	common_data->def_clk_freq = 50000000;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;

	self->i2c_client = client;
	self->s_data = common_data;
	self->subdev = &common_data->subdev;
	self->subdev->dev = &client->dev;
	self->s_data->dev = &client->dev;

	TRY(err, vg6768_regmap_init(client,
				    &vg6768_regmap_cfg,
				    &self->map));
	TRY(err, camera_common_initialize(common_data, "vg6768"));

	v4l2_i2c_subdev_init(self->subdev, client, &vg6768_subdev_ops);
	TRY(err, vg6768_ctrls_init(self));

	self->subdev->internal_ops = &vg6768_subdev_internal_ops;
	self->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	dev_dbg(&client->dev, "init media entity");
	self->pad.flags = MEDIA_PAD_FL_SOURCE;
	self->subdev->entity.ops = &vg6768_media_ops;
	/* The media controller code is slightly different on Tx2
	 * (kernel 4.4) vs. on Xavier (kernel 4.9) */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	self->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	err = media_entity_init(&self->subdev->entity, 1, &self->pad, 0);
#else
	err = tegra_media_entity_init(&self->subdev->entity, 1,
				      &self->pad, true, true);
#endif
	if (err < 0)
	{
		dev_err(&client->dev, "unable to init media entity");
		return err;
	}
#endif
/* #if defined(CONFIG_MEDIA_CONTROLLER) */
/* 	self->pad.flags = MEDIA_PAD_FL_SOURCE; */
/* 	self->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR; */
/* 	self->subdev->entity.ops = &vg6768_media_ops; */
/* 	TRY(err, media_entity_init(&self->subdev->entity, 1, &self->pad, 0)); */
/* #endif */
	TRY(err, v4l2_async_register_subdev(self->subdev));

	dev_dbg(self->dev, "vg6768 configuring imager: addr=%#.2x",
		client->addr);
	TRY(err, vg6768_reg_write_table(self,
					mode_1920X1080_30fps,
					ARRAY_SIZE(mode_1920X1080_30fps),
					false));
	dev_dbg(self->dev, "vg6768 configuring imager done");

	msleep(20);
	TRY(err, vg6768_reg_write_table(self,
					vg6768_isp_start,
					ARRAY_SIZE(vg6768_isp_start),
					false));
	msleep(100);
	msleep(50);
	TRY(err, vg6768_reg_write_table(self,
					mode_1920X1080_30fps,
					ARRAY_SIZE(mode_1920X1080_30fps),
					false));
	dev_dbg(self->dev, "vg6768 probe success: addr=%#.2x", client->addr);
	return 0;
};



/**
 * Linux driver data
 */
static struct i2c_driver vg6768_driver =
{
	.driver =
	{
		.name = "vg6768",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(vg6768_of_match),
	},
	.probe = vg6768_probe,
	.remove = vg6768_remove,
	.id_table = vg6768_id,
};

module_i2c_driver(vg6768_driver);

MODULE_DESCRIPTION("Driver for VG6768 camera on Nvidia Jetson");
MODULE_VERSION(D3_JETSON_BSP_VERSION);
MODULE_AUTHOR("Thirdy Buno <luis.buno@global-imi.com>");
MODULE_AUTHOR("Mike Soltiz <msoltiz@d3engineering.com>");
MODULE_LICENSE("GPL v2");
