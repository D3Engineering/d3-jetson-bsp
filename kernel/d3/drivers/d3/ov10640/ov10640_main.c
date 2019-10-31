/**
 * @author Josh Watts <jwatts@d3engineering.com>
 *
 * ov10640 v4l2 driver for Nvidia Jetson
 *
 * Copyright (c) 2019, D3 Engineering.  All rights reserved.
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

#include <d3/ub960.h>

#include "ov10640.h"
#include "ov10640_reg.h"
#include "ov10640_tables.h"
#include "ov10640_ctrls.h"

extern int ub953_set_frame_sync_enable(struct device *dev, bool enabled);



/**
 * Notes:

	aCSP section 1.4 lists registers for start/stop of sensor output (finishes current frame on stop)
		Remove final start from reg table


 */

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
static int ov10640_camera_read_reg(struct camera_common_data *s_data, u16 addr,
				   u8 *val)
{
	struct ov10640 *self = s_data->priv;
	unsigned int _val;
	int result;
	result = regmap_read(self->map, addr, &_val);
	*val = _val;
	return result;
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
static int ov10640_camera_write_reg(struct camera_common_data *s_data, u16 addr,
				    u8 val)
{
	struct ov10640 *self = s_data->priv;
	return regmap_write(self->map, addr, val);
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
static int ov10640_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov10640 *self = (struct ov10640 *)s_data->priv;
	struct i2c_client *parent_client;
	int mode_ix = self->s_data->sensor_mode_id;
	int is_hdr = ov10640_formats[mode_ix].hdr_en;
	parent_client = of_find_i2c_device_by_node(client->dev.of_node->parent);

	dev_dbg(self->dev, "mode: %d %s %s",
		mode_ix,
		is_hdr ? "HDR": "linear",
		enable ? "START" : "STOP");

	if (self->deserializer) {
		ub960_s_stream(self->deserializer, self->client, enable);
	}


	if (enable) {
		/* configure the sensor */
		/* regmap_write(self->map, OV10640_REG_SOFTWARE_CTRL2, */
		/* 	     OV10640_REG_SOFTWARE_CTRL2_RESET); */
		regmap_multi_reg_write(
			self->map,
			mode_table[mode_ix].reg_sequence,
			mode_table[mode_ix].size);
		ov10640_hflip_set(self, self->hflip);
		ov10640_vflip_set(self, self->vflip);
		regmap_update_bits(self->map,
				   OV10640_REG_SENSOR_CTRL,
				   OV10640_REG_SENSOR_CTRL_FSIN_EN_MASK,
				   OV10640_REG_SENSOR_CTRL_FSIN_EN(self->frame_sync_mode));

		regmap_multi_reg_write(self->map, mode_stream, mode_stream_len);
		if (self->frame_sync_mode) {
			ub953_set_frame_sync_enable(&parent_client->dev, true);
		}
	} else {
		ub953_set_frame_sync_enable(&parent_client->dev, false);
		regmap_write(self->map, OV10640_REG_SOFTWARE_CTRL1,
			     OV10640_REG_SOFTWARE_CTRL1_SW_STBY);
	}
	return 0;
}

static struct v4l2_subdev_video_ops ov10640_subdev_video_ops = {
	.s_stream	= ov10640_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
};

static struct v4l2_subdev_core_ops ov10640_subdev_core_ops = {
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
static int ov10640_set_fmt(struct v4l2_subdev *sd,
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
static int ov10640_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}


static struct v4l2_subdev_pad_ops ov10640_subdev_pad_ops = {
	.set_fmt	     = ov10640_set_fmt,
	.get_fmt	     = ov10640_get_fmt,
	.enum_mbus_code	     = camera_common_enum_mbus_code,
	.enum_frame_size     = camera_common_enum_framesizes,
	.enum_frame_interval = camera_common_enum_frameintervals,
};


static struct v4l2_subdev_ops ov10640_subdev_ops = {
	.core  = &ov10640_subdev_core_ops,
	.video = &ov10640_subdev_video_ops,
	.pad   = &ov10640_subdev_pad_ops,
};


static int ov10640_camera_power_on(struct camera_common_data *s_data)
{
	struct ov10640 *priv = (struct ov10640 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	pw->state = SWITCH_ON;
	
	return 0;

}

static int ov10640_camera_power_off(struct camera_common_data *s_data)
{
	struct ov10640 *priv = (struct ov10640 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	pw->state = SWITCH_OFF;
	//TODO: Power control?
	return 0;
}

/**
 * These are for debugging. The Nvidia camera_common code creates a
 * file in sysfs to read and write the image sensor.
 */
static struct camera_common_sensor_ops ov10640_common_ops = {
	.power_on = ov10640_camera_power_on,
	.power_off = ov10640_camera_power_off,
	.write_reg = ov10640_camera_write_reg,
	.read_reg  = ov10640_camera_read_reg,
};

static struct regmap_config ov10640_regmap_cfg = {
	.reg_bits = 16,
	.val_bits = 8,
};

static const struct v4l2_subdev_internal_ops ov10640_subdev_internal_ops = {
	// No ops needed
};


static const struct media_entity_operations ov10640_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};


static int ov10640_deserializer_parse(struct ov10640 *self,
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
static struct camera_common_pdata *ov10640_parse_dt(
	struct ov10640 *self,
	struct camera_common_data *s_data)
{
	struct device_node *node = self->client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	int err;

	if (!node) {
		dev_err(self->dev, "no OF node");
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

	if (of_property_read_s32(node,
				 "frame-sync-mode",
				 &self->frame_sync_mode) != 0)
		self->frame_sync_mode = 0;

	/* Errors warnings are reported in deserializer_parse. It is
	 * OK for this to return an error as the presence of a
	 * deserializer is optional. */
	if (ov10640_deserializer_parse(self, &self->deserializer) == 0) {
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
static int ov10640_media_init(struct ov10640 *self)
{
	int err = 0;
#if defined(CONFIG_MEDIA_CONTROLLER)
	self->pad.flags = MEDIA_PAD_FL_SOURCE;
	self->subdev->entity.ops = &ov10640_media_ops;
	/* The media controller code is slightly different on Tx2
	 * (kernel 4.4) vs. on Xavier (kernel 4.9) */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	self->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	err = media_entity_init(&self->subdev->entity, 1, &self->pad, 0);
#else
	err = tegra_media_entity_init(&self->subdev->entity, 1, &self->pad, true, true);
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
static int ov10640_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	struct camera_common_data *common_data;
	struct ov10640 *self = NULL;
	int tries;

	dev_dbg(&client->dev, "probe enter");

	TRY_MEM(common_data, devm_kzalloc(&client->dev, sizeof(*common_data),
					  GFP_KERNEL));
	TRY_MEM(self, devm_kzalloc(&client->dev, sizeof(*self), GFP_KERNEL));
	self->client = client;
	self->dev = &client->dev;

	common_data->priv = self;
	common_data->ops = &ov10640_common_ops;
	common_data->ctrl_handler = &self->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = &ov10640_formats[0];
	common_data->numfmts = ov10640_formats_len;
	common_data->colorfmt = camera_common_find_datafmt(MEDIA_BUS_FMT_SBGGR12_1X12);

	common_data->power = &self->power;
	common_data->ctrls = self->ctrls;

	common_data->priv = self;
	common_data->numctrls = ov10640_ctrls_count();

	common_data->def_mode = common_data->frmfmt[OV10640_MODE_DEFAULT].mode;
	common_data->def_width = common_data->frmfmt[OV10640_MODE_DEFAULT].size.width;
	common_data->def_height =
		common_data->frmfmt[OV10640_MODE_DEFAULT].size.height;
	/* common_data->def_clk_freq = 48000000; */
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;

	self->s_data = common_data;
	self->s_data->sensor_mode_id = OV10640_MODE_DEFAULT;
	self->s_data->mode = OV10640_MODE_DEFAULT;
	self->subdev = &common_data->subdev;
	self->subdev->dev = self->dev;
	self->s_data->dev = self->dev;

	self->pdata = ov10640_parse_dt(self, common_data);
	if (!self->pdata)
		return -EFAULT;

	TRY_MEM(self->map, devm_regmap_init_i2c(self->client, &ov10640_regmap_cfg));

	for (tries = 50; --tries >= 0; ) {
		/* regmap_write(self->map, OV10640_REG_SOFTWARE_CTRL2, */
		/* 	     OV10640_REG_SOFTWARE_CTRL2_RESET); */

		err = regmap_multi_reg_write(self->map, mode_table[OV10640_MODE_DEFAULT].reg_sequence, mode_table[OV10640_MODE_DEFAULT].size);

		if (err >= 0)
			err = regmap_write(self->map, OV10640_REG_SOFTWARE_CTRL1, OV10640_REG_SOFTWARE_CTRL1_SW_STBY);

		if (err < 0) {
			dev_dbg(self->dev, "Giving device more time to settle\n");
			usleep_range(50 * 1000, 50 * 1000);
		} else
			break;
	}
	if (err < 0) {
		dev_err(self->dev, "Failed to find device\n");
		return err;
	}

	TRY(err, camera_common_initialize(common_data, "ov10640"));

	v4l2_i2c_subdev_init(self->subdev, client, &ov10640_subdev_ops);
	TRY(err, ov10640_ctrls_init(self));

	self->subdev->internal_ops = &ov10640_subdev_internal_ops;
	self->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	TRY(err, ov10640_media_init(self));
	TRY(err, v4l2_async_register_subdev(self->subdev));

	dev_dbg(self->dev, "probe success");
	return 0;
}

static struct of_device_id ov10640_of_match[] = {
	{ .compatible = "d3,ov10640"},
	{ },
};
MODULE_DEVICE_TABLE(of, ov10640_of_match);

static const struct i2c_device_id ov10640_id[] = {
	{"ov10640", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ov10640_id);


static struct i2c_driver ov10640_driver = {
	.driver = {
		.name = "ov10640",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ov10640_of_match),
	},
	.probe = ov10640_probe,
	.id_table = ov10640_id,
};
module_i2c_driver(ov10640_driver);
MODULE_DESCRIPTION("Driver for OV10640 camera for Nvidia Jetson");
MODULE_VERSION(D3_JETSON_BSP_VERSION);
MODULE_AUTHOR("Greg Rowe <growe@d3engineering.com>");
MODULE_AUTHOR("Josh Watts <jwatts@d3engineering.com>");
MODULE_LICENSE("GPL v2");
