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
#include <linux/gpio/consumer.h>

#include <d3/d3-jetson-bsp.h>
#include <d3/common.h>

#include <d3/ub960.h>
#include <d3/ub953.h>

#include "ov10640.h"
#include "ov10640_reg.h"
#include "ov10640_tables.h"
#include "ov10640_ctrls.h"

#define OV10640_PATCH_VERSION "2"
#define OV10640_PATCH_DESC "2020-10-06 - numerous debug enhancements"

static int ov10640_combine_write(struct ov10640 *self);

#ifdef CONFIG_D3_OV10640_DEBUG
static const char *s_CHANNEL_NAMES[] = {
	"PWL",
	"LONG",
	"SHORT",
	"VERY SHORT",
	"None - not overridden"
};
#endif /* CONFIG_D3_OV10640_DEBUG */



#ifdef CONFIG_D3_OV10640_DEBUG
static const char *s_HDRMODE_NAMES[] = {
	"L-S-VS",
	"L-S",
	"L-VS",
	"L",
	"None - not overridden"
};
#endif /* CONFIG_D3_OV10640_DEBUG */


static const char *s_NORMALIZATION_NAMES[] = {
	"10",
	"12",
	"None - not overridden"
};


static const char *s_MODE_NAMES[] = {
	"1280X1080_LONG",
#ifdef CONFIG_D3_OV10640_HDR_ENABLE
	"1280X1080_HDR",
#endif /* CONFIG_D3_OV10640_HDR_ENABLE */
};


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


#ifdef CONFIG_D3_OV10640_DEBUG
static int ov10640_hdrmode_set(struct ov10640 *self,
			       enum ov10640_hdrmode hdr_mode)
{
	int err = 0;

	if (hdr_mode >= OV10640_HDRMODE_ENDMARKER) {
		dev_warn(self->dev, "invalid hdr mode: %u", hdr_mode);
		return -EINVAL;
	}


	/* The two LSBs of the interface control register select which
	 * exposures/channels to output.  */
	dev_dbg(self->dev, "setting HDR mode to: %s", s_HDRMODE_NAMES[hdr_mode]);
	TRY(err, regmap_update_bits(
		    self->map,
		    OV10640_REG_READ_MODE,
		    OV10640_REG_READ_MODE_HDRMODE_MASK,
		    OV10640_REG_READ_MODE_HDRMODE(hdr_mode)));

	return 0;
}
#endif /* CONFIG_D3_OV10640_DEBUG */



#ifdef CONFIG_D3_OV10640_DEBUG
int ov10640_channel_set(struct ov10640 *self, enum ov10640_channel chan)
{
	int err = 0;

	if (chan >= OV10640_CHANNEL_ENDMARKER) {
		dev_warn(self->dev, "invalid channel id: %u", chan);
		return -EINVAL;
	}

	/* The two LSBs of the interface control register select which
	 * exposures/channels to output.  */
	dev_dbg(self->dev, "setting channel to: %s (%#x=%#x) (mask=%#x)",
		s_CHANNEL_NAMES[chan],
		OV10640_REG_INTERFACE_CTRL,
		chan,
		OV10640_REG_INTERFACE_CTRL_DATAWIDTH_MASK);
	TRY(err, regmap_update_bits(
		    self->map,
		    OV10640_REG_INTERFACE_CTRL,
		    OV10640_REG_INTERFACE_CTRL_DATAWIDTH_MASK,
		    OV10640_REG_INTERFACE_CTRL_DATAWIDTH(chan)));
	return 0;
}
#endif /* CONFIG_D3_OV10640_DEBUG */


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
#ifdef CONFIG_D3_OV10640_DEBUG
	int err = 0;
#endif /* CONFIG_D3_OV10640_DEBUG */

	int mode_ix = self->s_data->sensor_mode_id;
	int is_hdr = ov10640_formats[mode_ix].hdr_en;

	/* Check data is valid */
	if (self->frame_sync_mode && !self->serializer) {
		dev_err(self->dev, "could not find serializer node required for frame_sync_mode");
		return -EINVAL;
	}
	if (!client) {
		pr_err("no i2c client");
		return -EINVAL;
	}
	if (!s_data) {
		dev_err(&client->dev, "no camera_common_data");
		return -EINVAL;
	}
	if (!self) {
		dev_err(&client->dev, "no private data pointer");
		return -EINVAL;
	}
	if ((mode_ix < 0)
	    || (mode_ix >= ov10640_formats_len)) {
		dev_err(self->dev, "mode_ix out of range, saw %d expected 0-%zu",
			mode_ix, ov10640_formats_len);
		return -EINVAL;
	}

	dev_dbg(self->dev, "mode: %d %s %s",
		mode_ix,
		is_hdr ? "HDR": "linear",
		enable ? "START" : "STOP");

	if (self->deserializer) {
		ub960_s_stream(self->deserializer, self->client, enable);
	}


	if (enable) {
		/* configure the sensor */
		ov10640_test_pattern_set(self, self->test_pattern_is_enabled);
		ov10640_hflip_set(self, self->hflip);
		ov10640_vflip_set(self, self->vflip);
		regmap_update_bits(self->map,
				   OV10640_REG_SENSOR_CTRL,
				   OV10640_REG_SENSOR_CTRL_FSIN_EN_MASK,
				   OV10640_REG_SENSOR_CTRL_FSIN_EN(self->frame_sync_mode));

#ifdef CONFIG_D3_OV10640_DEBUG
		TRY(err, ov10640_channel_set(self, self->channel_select));
		TRY(err, ov10640_hdrmode_set(self, self->hdr_mode));
#endif /* CONFIG_D3_OV10640_DEBUG */

		if (self->normalization_type != OV10640_NORMALIZATION_ENDMARKER) {
			dev_dbg(self->dev, "setting normalization to: %s",
				s_NORMALIZATION_NAMES[self->normalization_type]);
			regmap_update_bits(
				self->map,
				OV10640_REG_NORMALIZATION_CTRL,
				OV10640_REG_NORMALIZATION_MASK,
				OV10640_REG_NORMALIZATION(self->normalization_type));

		}

		ov10640_combine_write(self);
		regmap_multi_reg_write(self->map, mode_stream, mode_stream_len);
		if (self->frame_sync_mode) {
			ub953_set_frame_sync_enable(self->serializer, true);
		}

	} else {
		if (self->frame_sync_mode) {
			ub953_set_frame_sync_enable(self->serializer, false);
		}

		regmap_write(self->map, OV10640_REG_SOFTWARE_CTRL1,
			     OV10640_REG_SOFTWARE_CTRL1_SW_STBY);
	}
	dev_dbg(self->dev, "exit");
	return 0;
}

static struct v4l2_subdev_video_ops ov10640_subdev_video_ops = {
	.s_stream	= ov10640_s_stream,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	.g_mbus_config	= camera_common_g_mbus_config,
#endif
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
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	.get_mbus_config	= camera_common_get_mbus_config,
#endif
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
	.cache_type = REGCACHE_RBTREE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
	.use_single_rw = true,
#else
	.use_single_read = true,
	.use_single_write = true,
#endif
};

static const struct v4l2_subdev_internal_ops ov10640_subdev_internal_ops = {
	// No ops needed
};


static const struct media_entity_operations ov10640_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov10640_parse_client(struct ov10640 *self, const char *name, struct i2c_client **out)
{
	struct device_node *self_node = self->client->dev.of_node;
	struct device_node *node;
	struct i2c_client *client;

	node = of_parse_phandle(self_node, name, 0);
	if (!node) {
		dev_dbg(self->dev, "could not find %s node", name);
		return -ENOENT;
	}

	client = of_find_i2c_device_by_node(node);
	of_node_put(node);
	node = NULL;

	if (!client) {
		dev_dbg(self->dev, "missing %s client", name);
		return -ENOENT;
	}

	*out = client;
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

	if (ov10640_parse_client(self, "deserializer", &self->deserializer) == 0) {
		dev_dbg(self->dev, "deserializer present");
	}
	else {
		self->deserializer = NULL;
	}

	if (ov10640_parse_client(self, "serializer", &self->serializer) == 0) {
		dev_dbg(self->dev, "serializer present");
	}
	else {
		self->serializer = NULL;
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


static void ov10640_report(struct device *dev)
{
	dev_info(dev,
		 "ov10640 v"D3_JETSON_BSP_VERSION "." OV10640_PATCH_VERSION
		 " DRE=%d ", FIXED_EXPOSURE_RATIO * FIXED_EXPOSURE_RATIO);
	dev_info(dev,
		 "min HDR exposure should be %d in DTS",
		 17 * FIXED_EXPOSURE_RATIO);
	dev_dbg(dev, "release description: %s", OV10640_PATCH_DESC);
}


#ifdef CONFIG_D3_OV10640_DEBUG
void ov10640_combine_dump(struct ov10640 *self, struct ov10640_combine *combine)
{
	int i;
	dev_dbg(self->dev, "HDR combination dump:");

	dev_dbg(self->dev, "ctrl=%#4x", combine->ctrl);

	for (i=0; i<ARRAY_SIZE(combine->thresh); ++i) {
		dev_dbg(self->dev, "thresh[%d]=%#x", i, combine->thresh[i]);
	}

	for (i=0; i<ARRAY_SIZE(combine->weight); ++i) {
		dev_dbg(self->dev, "weight[%d]=%#x", i, combine->weight[i]);
	}
}
#endif /* CONFIG_D3_OV10640_DEBUG */


#ifdef CONFIG_D3_OV10640_DEBUG
static int ov10640_channel_select_set_from_mode(struct ov10640 *self,
						enum ov10640_mode mode)
{
	if (mode >= OV10640_MODE_ENDMARKER) {
		dev_warn(self->dev, "invalid mode: %u", mode);
		return -ENOENT;
	}

	switch(mode) {
	case OV10640_MODE_1280X1080_HDR:
		self->channel_select = OV10640_CHANNEL_PWL;
		break;
	case OV10640_MODE_1280X1080_LONG:
	default:
		self->channel_select = OV10640_CHANNEL_LONG;
		break;
	};

	dev_dbg(self->dev, "set channel %s from mode %s",
		s_CHANNEL_NAMES[self->channel_select], s_MODE_NAMES[mode]);
	return 0;
}
#endif /* CONFIG_D3_OV10640_DEBUG */


static int ov10640_sensor_mode_set(struct ov10640 *self, enum ov10640_mode mode)
{
	if (mode >= OV10640_MODE_ENDMARKER) {
		dev_warn(self->dev, "invalid sensor mode %u", mode);
		return -ENOENT;
	}
	dev_dbg(self->dev, "assigning mode %s", s_MODE_NAMES[mode]);
	self->s_data->sensor_mode_id = mode;
	self->s_data->mode = mode;
	return 0;
}


static int ov10640_combine_write(struct ov10640 *self)
{
	int i;
	int j;
	u16 addr;
	u8 val;
	u8 shift;
	u32 mask;
	int err;
	struct ov10640_combine *c = &self->combine;

	/* The current configuration seems to be causing a yellow tint
	 * in the L/S combination/threshold areas of the image. I'm
	 * doing this early return to, effectively, revert to
	 * defaults in the configuration tables. */
	dev_warn(self->dev, "modification of combination registers disabled");
	return 0;

	dev_dbg(self->dev, "combine %#x=%#x", OV10640_REG_COMBINE_CTRL, c->ctrl);
	TRY(err, regmap_write(self->map, OV10640_REG_COMBINE_CTRL, c->ctrl));

	for (i=0; i<ARRAY_SIZE(c->thresh); ++i) {
		dev_dbg(self->dev, "combine thresh %#x=%#x",
			OV10640_REG_COMBINE_THRE_0 + i,
			c->thresh[i]);
		addr = OV10640_REG_COMBINE_THRE_0 + i;
		TRY(err, regmap_write(self->map, addr, c->thresh[i]));
	}

	addr = OV10640_REG_COMBINE_WEIGHT_0;
	for (i=0; i<ARRAY_SIZE(c->weight); ++i) {
		for (j=0; j<4; ++j) {
			mask = 0xff000000 >> (j*8);
			shift = 24 - (j*8);
			val = (c->weight[i] & mask) >> shift;
			dev_dbg(self->dev, "combine weight %#x=%#x", addr, val);
			regmap_write(self->map, addr, val);
			++addr;
		}
	}
	return 0;
}

static void ov10640_combine_init(struct ov10640_combine *combine)
{
	memset(combine, 0, sizeof(*combine));
	/* Setting defaults from data sheet */
	combine->ctrl = 0x3c;

	/* These are the sensor defaults, apparently for 10 bit mode. */
	/* combine->thresh[0] = 0x95; */
	/* combine->thresh[1] = 0xa8; */
	/* combine->thresh[2] = 0xaa; */

	combine->thresh[0] = 0xb7;
	combine->thresh[1] = 0xca;
	combine->thresh[2] = 0xcc;

	combine->weight[0] = 0x80806040;
	combine->weight[1] = 0x80904020;
	combine->weight[2] = 0x80600000;
	combine->weight[3] = 0x80800000;
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

	ov10640_report(&client->dev);

	dev_dbg(&client->dev, "probe enter");

	TRY_MEM(common_data, devm_kzalloc(&client->dev, sizeof(*common_data),
					  GFP_KERNEL));
	TRY_MEM(self, devm_kzalloc(&client->dev, sizeof(*self), GFP_KERNEL));
	self->client = client;
	self->dev = &client->dev;
	self->normalization_type = OV10640_NORMALIZATION_ENDMARKER;
	ov10640_combine_init(&self->combine);

#ifdef CONFIG_D3_OV10640_DEBUG
	ov10640_combine_dump(self, &self->combine);
#endif /* CONFIG_D3_OV10640_DEBUG */

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
	ov10640_sensor_mode_set(self, OV10640_MODE_DEFAULT);
#ifdef CONFIG_D3_OV10640_DEBUG
	ov10640_channel_select_set_from_mode(self, OV10640_MODE_DEFAULT);
#endif /* CONFIG_D3_OV10640_DEBUG */

	self->subdev = &common_data->subdev;
	self->subdev->dev = self->dev;
	self->s_data->dev = self->dev;

	self->i2c_addressing_pin = gpiod_get_optional(self->dev, "i2c-addressing",
			GPIOD_OUT_HIGH);

	if (!self->i2c_addressing_pin) {
		dev_dbg(self->dev, "No i2c address pin found, not required.");
	} else {
		dev_dbg(self->dev, "i2c address pin found, setting to 0");
		gpiod_set_value_cansleep(self->i2c_addressing_pin, 0);
	}

	self->pdata = ov10640_parse_dt(self, common_data);
	if (!self->pdata)
		return -EFAULT;

	TRY_MEM(self->map, devm_regmap_init_i2c(self->client,
				&ov10640_regmap_cfg));
	// The OV10640 imager in the RCM is paired with a MCU which acts as an
	// I2C master to the imager. Multiple tries to write to the imager may
	// be required if the MCU is also attempting to write to the imager.
	for (tries = 50; --tries >= 0; ) {
		err = regmap_multi_reg_write(self->map,
				mode_table[OV10640_MODE_DEFAULT].reg_sequence,
				mode_table[OV10640_MODE_DEFAULT].size);

		if (err >= 0) {
			err = regmap_write(self->map,
					OV10640_REG_SOFTWARE_CTRL1,
					OV10640_REG_SOFTWARE_CTRL1_SW_STBY);
		}

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
