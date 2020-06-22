/*
 * ar0820.c - OnSemi AR0820 CMOS Sensor
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


// #define DEBUG 1

#include <linux/module.h>
#include <linux/module.h>
#include <media/camera_common.h>
#include <d3/d3-jetson-bsp.h>
#include <linux/of_device.h>


#include "ar0820_tables.h"

/* #define AR0820_DEFAULT_FMT MEDIA_BUS_FMT_SRGGB12_1X12 */
/* #define AR0820_DEFAULT_FMT MEDIA_BUS_FMT_SGRBG12_1X12 */
#define AR0820_DEFAULT_FMT MEDIA_BUS_FMT_SBGGR12_1X12
#define AR0820_DEFAULT_MODE AR0820_MODE_3848X2168
#define AR0820_DEFAULT_CLK_FREQ 25000000

/* Register definitions */
#define AR0820_REG_COARSE_INT_TIME 0x3012
#define AR0820_REG_GROUPED_PARAMETER_HOLD 0x3022
#define AR0820_REG_GLOBAL_GAIN 0x305E
#define AR0820_REG_ANALOG_GAIN 0X3366
#define AR0820_REG_ANALOG_GAIN_CB 0X3368
#define AR0820_REG_ANALOG_GAIN2 0X336A
#define AR0820_REG_ANALOG_GAIN2_CB 0X336C
#define AR0820_REG_ANALOG_LOW_GAIN 0X33BE



#ifdef DEBUG
/* Expose some registers in debugfs */
static const struct regmap_range ar0820_yes_ranges[] = {
	regmap_reg_range(0x2000, 0x200A), /* Frame count, and status */
	regmap_reg_range(0x205C, 0x205C), /* Data format actual */
	regmap_reg_range(0x3000, 0x3000), /* chip version */
	regmap_reg_range(0x3002, 0x3002), /* Y addr. start (0) */
	regmap_reg_range(0x3004, 0x3004), /* X addr. start (0) */
	regmap_reg_range(0x3006, 0x3006), /* Y addr. end (2167) */
	regmap_reg_range(0x3008, 0x3008), /* X addr. end (3847) */
	regmap_reg_range(0x301A, 0x301C), /* reset reg, mode, orientation */
	regmap_reg_range(0x3056, 0x305E), /* gain */
	regmap_reg_range(0x3070, 0x307A), /* test pattern */
	regmap_reg_range(0x30AA, 0x30AA), /* frame length lines cb */
	regmap_reg_range(0x31AC, 0x31B2), /* data format, frame preamble, line preamble */
	regmap_reg_range(0x31BE, 0x31BE), /* MIPI config status */
	regmap_reg_range(0x31FE, 0x31FE), /* chip revision */
	regmap_reg_range(0x3366, 0x336A),
	regmap_reg_range(0x3372, 0x3372),
	regmap_reg_range(0x340A, 0x340E), /* GPIO control */
	regmap_reg_range(0x351A, 0x351A),
};

static const struct regmap_access_table ar0820_regmap_rd_access_table = {
	.yes_ranges = ar0820_yes_ranges,
	.n_yes_ranges = ARRAY_SIZE(ar0820_yes_ranges),
};
#endif /* DEBUG */

static struct regmap_config ar0820_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
#ifdef DEBUG
	.max_register = 0x517F,
	.rd_table = &ar0820_regmap_rd_access_table,
#endif /* DEBUG */
};

struct ar0820 {
	struct camera_common_power_rail power;

	struct i2c_client *client;
	struct regmap *map;

	struct camera_common_data *s_data;
	struct camera_common_pdata *pdata;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev *subdev;
	struct media_pad pad;

	int frame_sync_mode;


	struct v4l2_ctrl *ctrls[];
};

static int ar0820_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0820 *priv =
		container_of(ctrl->handler, struct ar0820, ctrl_handler);
	int ret = 0;

	switch (ctrl->id) {
	default:
		dev_err(&priv->client->dev, "%s: unknown ctrl id.", __func__);
		return -EINVAL;
	}
	return ret;
}

static int ar0820_set_group_hold(struct ar0820 *priv, s32 val)
{
	u16 reg_val = !!val;
	int ret = 0;

	dev_dbg(&priv->client->dev, "%s: %u.", __func__, reg_val);
	ret = regmap_write(priv->map, AR0820_REG_GROUPED_PARAMETER_HOLD,
			reg_val);
	if (ret)
		dev_err(&priv->client->dev, "%s: error writing register.",
				__func__);
	return ret;
}

static int ar0820_set_gain(struct ar0820 *priv, u64 val)
{
	u16 gain = 0;
	int ret = 0;

	gain = (u16) ((val << 7) / FIXED_POINT_SCALING_FACTOR);

	dev_dbg(&priv->client->dev, "%s: gain: %u.", __func__, gain);
	ret = ar0820_set_group_hold(priv, 1);
	if (ret) {
		dev_err(&priv->client->dev, "%s: group hold err.", __func__);
		return ret;
	}
	ret = regmap_write(priv->map, AR0820_REG_GLOBAL_GAIN, gain);
	if (ret) {
		dev_err(&priv->client->dev, "%s: error writing register.",
				__func__);
	}
	ar0820_set_group_hold(priv, 0);
	return ret;
}

static int ar0820_set_exposure(struct ar0820 *priv, s64 val)
{
	u16 c_int_t; /* coarse integration time */
	int ret = 0;
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];

	c_int_t = mode->signal_properties.pixel_clock.val * val /
		mode->image_properties.line_length / FIXED_POINT_SCALING_FACTOR;
	/* CIT times specified in multiples of
	 * ROPS * (NUM_EXP_MAX + 1 ) * LINE_LENGTH_PCK_
	 * See AR0820 Register ref. guide, register 0x3012
	 */
	c_int_t /= 4;
	dev_dbg(&priv->client->dev, "%s: c_int_t: %llu -> 0x%x.", __func__,
			val, c_int_t);
	ret = regmap_write(priv->map, AR0820_REG_COARSE_INT_TIME, c_int_t);
	if (ret) {
		dev_err(&priv->client->dev, "%s: error writing register",
				__func__);
	}
	return ret;
}

static int ar0820_set_frame_rate(struct ar0820 *priv, s64 val)
{
	dev_warn(&priv->client->dev, "%s: not implemented.", __func__);
	return 0;
}

static int ar0820_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0820 *priv =
		container_of(ctrl->handler, struct ar0820, ctrl_handler);
	int ret = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		ret = ar0820_set_gain(priv, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_EXPOSURE:
		ret = ar0820_set_exposure(priv, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		ret = ar0820_set_frame_rate(priv, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		ret = ar0820_set_group_hold(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		break;
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
		break;
	default:
		dev_err(&priv->client->dev, "control %d not supported!", ctrl->id);
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops ar0820_ctrl_ops = {
	.g_volatile_ctrl = ar0820_g_volatile_ctrl,
	.s_ctrl = ar0820_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
	{
		.ops = &ar0820_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0 * FIXED_POINT_SCALING_FACTOR,
		.max = 48 * FIXED_POINT_SCALING_FACTOR,
		.def = 0 * FIXED_POINT_SCALING_FACTOR,
		.step = 3 * FIXED_POINT_SCALING_FACTOR / 10, /* 0.3 db */
	},
	{ /* Exposure times are in microseconds */
		.ops = &ar0820_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 30 * FIXED_POINT_SCALING_FACTOR / 1000000,
		.max = 1000000LL * FIXED_POINT_SCALING_FACTOR / 1000000,
		.def = 30 * FIXED_POINT_SCALING_FACTOR / 1000000,
		.step = 1 * FIXED_POINT_SCALING_FACTOR / 1000000,
	},
	{
		.ops = &ar0820_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_RATE,
		.name = "Frame Rate",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 20 * FIXED_POINT_SCALING_FACTOR,
		.max = 20 * FIXED_POINT_SCALING_FACTOR,
		.def = 20 * FIXED_POINT_SCALING_FACTOR,
		.step = 1 * FIXED_POINT_SCALING_FACTOR,
	},
	{
		.ops = &ar0820_ctrl_ops,
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
		.ops = &ar0820_ctrl_ops,
		.id = TEGRA_CAMERA_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &ar0820_ctrl_ops,
		.id = TEGRA_CAMERA_CID_SENSOR_MODE_ID,
		.name = "Sensor Mode",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 0xFF,
		.def = 0xFE,
		.step = 1,
	},

};

static const struct i2c_device_id ar0820_idtable[] = {
	{ "ar0820", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ar0820_idtable);

static const struct of_device_id ar0820_of_match[] = {
	{ .compatible = "d3,ar0820" },
	{},
};
MODULE_DEVICE_TABLE(of, ar0820_of_match);

static int ar0820_power_on(struct camera_common_data *s_data)
{
	struct ar0820 *priv = (struct ar0820*) s_data->priv;
	struct camera_common_power_rail *power = &priv->power;

	dev_dbg(&priv->client->dev, "power on.");
	power->state = SWITCH_ON;
	return 0;
}

static int ar0820_power_off(struct camera_common_data *s_data)
{
	struct ar0820 *priv = (struct ar0820*) s_data->priv;
	struct camera_common_power_rail *power = &priv->power;

	dev_dbg(&priv->client->dev, "power off.");
	power->state = SWITCH_OFF;
	return 0;
}

static int ar0820_write_reg(struct camera_common_data *s_data,
		u16 addr, u8 val)
{
	struct ar0820 *priv = (struct ar0820*) s_data->priv;
	int ret;

	dev_dbg(&priv->client->dev, "%s: [0x%04x] = 0x%04x\n",
			__func__, addr, val);
	ret = regmap_write(priv->map, addr, val);
	if (ret) {
		dev_err(&priv->client->dev,
			"%s: i2c write failed: 0x%x, 0x%x",
			__func__, addr, val);
	}
	return ret;
}

static inline int ar0820_read_reg(struct camera_common_data *s_data,
		u16 addr, u8 *val)
{
	struct ar0820 *priv = (struct ar0820*) s_data->priv;
	u32 reg_val;
	int ret;

	ret = regmap_read(priv->map, addr, &reg_val);
	if (ret) {
		dev_err(&priv->client->dev,
			"%s: i2c read failed: 0x%x, ret = %d",
			__func__, addr, ret);
	}
	else {
		*val = reg_val & 0xFF;
	}
	return ret;
}

static struct camera_common_sensor_ops ar0820_common_ops = {
	.power_on = ar0820_power_on,
	.power_off = ar0820_power_off,
	.write_reg = ar0820_write_reg, /* TODO: warning here: type of val in prototype is u8 */
	.read_reg = ar0820_read_reg, /* TODO: same as above */
};

/* v4l2 subdev */
static int ar0820_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0820 *priv = (struct ar0820*)s_data->priv;
	int ret = 0;

	if (enable) {
		dev_dbg(&client->dev, "start streaming.");
		ret = regmap_multi_reg_write(priv->map, ar0820_start, ARRAY_SIZE(ar0820_start));
	}
	else {
		dev_dbg(&client->dev, "stop streaming.");
		ret = regmap_multi_reg_write(priv->map, ar0820_stop, ARRAY_SIZE(ar0820_stop));
	}
	if (ret) {
		dev_err(&client->dev, "Can't write registers: %d.", ret);
	}
	return ret;
}

static int ar0820_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	/* TODO: return the actual status */
	*status = 1;
	return 0;
}

static int ar0820_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		ret = camera_common_try_fmt(sd, &format->format);
	}
	else {
		ret = camera_common_s_fmt(sd, &format->format);
	}
	return ret;
}

static int ar0820_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static struct v4l2_subdev_core_ops ar0820_subdev_core_ops = {
	.s_power = camera_common_s_power,
};

static struct v4l2_subdev_video_ops ar0820_subdev_video_ops = {
	.s_stream = ar0820_s_stream,
	.g_mbus_config = camera_common_g_mbus_config,
	.g_input_status = ar0820_g_input_status,
};

static struct v4l2_subdev_pad_ops ar0820_subdev_pad_ops = {
	.set_fmt = ar0820_set_fmt,
	.get_fmt = ar0820_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size = camera_common_enum_framesizes,
	.enum_frame_interval = camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops ar0820_subdev_ops = {
	.core = &ar0820_subdev_core_ops,
	.video = &ar0820_subdev_video_ops,
	.pad = &ar0820_subdev_pad_ops,
};

static int ar0820_ctrls_init(struct ar0820 *priv)
{
	struct i2c_client *client = priv->client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls = ARRAY_SIZE(ctrl_config_list);
	int ret;
	int i;

	dev_dbg(&client->dev, "%s:", __func__);

	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
				&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "failed to init %s ctrl",
					ctrl_config_list[i].name);
			continue;
		}
		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY )
		{
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
					ctrl_config_list[i].max + 1, GFP_KERNEL);
		}
		priv->ctrls[i] = ctrl;
	}

	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls",
				priv->ctrl_handler.error);
		ret = priv->ctrl_handler.error;
		goto error;
	}

	ret = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (ret) {
		dev_err(&client->dev, "Error %d setting default controls", ret);
		goto error;
	}

	return 0;
error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return ret;
}

static int ar0820_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops ar0820_subdev_internal_ops = {
	.open = ar0820_open,
};

static const struct media_entity_operations ar0820_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static struct camera_common_pdata *ar0820_parse_dt(
		struct ar0820 *self,
		struct camera_common_data *s_data)
{
	struct i2c_client *client = self->client;
	struct device_node *np = client->dev.of_node;
	struct camera_common_pdata *board_priv_data;
	const struct of_device_id *match;
	int ret;

	if (!np) {
		dev_err(&client->dev, "open firmware node is NULL!");
		return NULL;
	}

	match = of_match_device(ar0820_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "failed to find matching DT id.");
		return NULL;
	}

	board_priv_data = devm_kzalloc(&client->dev,
			sizeof(struct camera_common_data), GFP_KERNEL);
	if (!board_priv_data) {
		dev_err(&client->dev, "failed to allocate memory!");
		return NULL;
	}

	ret = of_property_read_string(np, "mclk",
			&board_priv_data->mclk_name);
	if (ret)
		dev_warn(&client->dev, "can't find mclk in DT.");

	if (of_property_read_s32(np, "frame-sync-mode", &self->frame_sync_mode) != 0) {
		self->frame_sync_mode = 0;
	}

	return board_priv_data;
}


static int ar0820_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct ar0820 *priv;
	int ret = 0;

	dev_info(&client->dev, "Probing AR0820.");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
			sizeof(struct camera_common_data), GFP_KERNEL);
	priv = devm_kzalloc(&client->dev,
			sizeof(struct ar0820) + sizeof(struct v4l2_ctrl*) *
			ARRAY_SIZE(ctrl_config_list),
			GFP_KERNEL);
	if (!common_data || !priv) {
		dev_err(&client->dev, "failed to allocate memory!");
		return -ENOMEM;
	}

	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->map = devm_regmap_init_i2c(client, &ar0820_regmap_config);
	if (IS_ERR(priv->map)) {
		dev_err(&client->dev, "failed to create regmap!");
		return PTR_ERR(priv->map);
	}

	if (client->dev.of_node)
		priv->pdata = ar0820_parse_dt(priv, common_data);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data!");
		return -EFAULT;
	}

	common_data->ops = &ar0820_common_ops;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = ar0820_frmfmt;
	common_data->colorfmt = camera_common_find_datafmt(AR0820_DEFAULT_FMT);
	common_data->power = &priv->power;
	common_data->ctrls = priv->ctrls;
	common_data->priv = priv;
	common_data->numctrls = ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts = ARRAY_SIZE(ar0820_frmfmt);
	common_data->def_mode = AR0820_DEFAULT_MODE;
	common_data->def_width = common_data->frmfmt[AR0820_DEFAULT_MODE].size.width;
	common_data->def_height = common_data->frmfmt[AR0820_DEFAULT_MODE].size.height;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;
	common_data->def_clk_freq = AR0820_DEFAULT_CLK_FREQ;

	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;

	/* get power? */

	ret = camera_common_initialize(common_data, "ar0820");
	if (ret) {
		dev_err(&client->dev, "failed to initialize ar0820.");
		return ret;
	}

	v4l2_i2c_subdev_init(priv->subdev, client, &ar0820_subdev_ops);

	ret = ar0820_ctrls_init(priv);
	if (ret)
		return ret;
	priv->subdev->internal_ops = &ar0820_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE
		| V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	//priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	priv->subdev->entity.ops = &ar0820_media_ops;
	ret = tegra_media_entity_init(&priv->subdev->entity, 1, &priv->pad,
				      true, true);
	if (ret < 0) {
		dev_err(&client->dev, "failed to initialize media entity!");
		return ret;
	}
#endif
	ret = regmap_multi_reg_write(priv->map, ar0820_mode_3848x2168, ARRAY_SIZE(ar0820_mode_3848x2168));
	if (ret) {
		dev_err(&client->dev, "failed to initialize the sensor!");
		return ret;
	}
	if (priv->frame_sync_mode == 1) {
		dev_info(&client->dev, "enabling frame sync mode.");
		ret = regmap_multi_reg_write(priv->map, ar0820_frame_sync, ARRAY_SIZE(ar0820_frame_sync));
		if (ret) {
			dev_err(&client->dev, "failed to enable frame sync!");
			return ret;
		}
	}
	ret = v4l2_async_register_subdev(priv->subdev);
	if (ret)
		return ret;
	dev_info(&client->dev, "probed AR0820 sensor.");

	return 0;
}

static int ar0820_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0820 *priv = (struct ar0820 *) s_data->priv;

	dev_info(&client->dev, "Removing AR0820.");
	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	camera_common_cleanup(s_data);
	return 0;
}


static struct i2c_driver ar0820_driver = {
	.driver = {
		.name = "ar0820",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ar0820_of_match),
	},
	.probe = ar0820_probe,
	.remove = ar0820_remove,
	.id_table = ar0820_idtable,
};

module_i2c_driver(ar0820_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Catalin Petrescu <cpetrescu@d3engineering.com>");
MODULE_DESCRIPTION("OnSemi AR0820 CMOS Sensor driver");
MODULE_VERSION(D3_JETSON_BSP_VERSION);

