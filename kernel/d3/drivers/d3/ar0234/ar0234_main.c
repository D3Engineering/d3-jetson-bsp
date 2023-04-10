/*
 * ar0234_main.c - ar0234 sensor driver
 *
 * Copyright (c) 2022, D3 Engineering. All rights reserved.
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
#include <linux/regmap.h>
#include <linux/bitfield.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

#include <d3/common.h>
#include <d3/reg_tbl.h>
#include <d3/reg_ctl.h>
#include <d3/d3-jetson-bsp.h>
#include <d3/ub960.h>

#include "ar0234.h"
#include "ar0234_ctrl.h"
#include "ar0234_tables.h"
#include "ar0234_registers.h"

static int ar0234_power_on(struct camera_common_data *s_data)
{
	struct device *dev = ((struct ar0234*)s_data->priv)->tc_dev->dev;

	dev_dbg(dev, "power on.");
	s_data->power->state = SWITCH_ON;

	return 0;
}

static int ar0234_power_off(struct camera_common_data *s_data)
{
	struct device *dev = ((struct ar0234*)s_data->priv)->tc_dev->dev;

	dev_dbg(dev, "power off.");
	s_data->power->state = SWITCH_OFF;

	return 0;
}

static int ar0234_start_stream(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct ar0234 *self = (struct ar0234*)tegracam_get_privdata(tc_dev);
	int err;
	dev_dbg(dev, "start stream.");

	if (self->deserializer) {
		ub960_s_stream(self->deserializer, self->client, 1);
	}

	TRY(err, reg_tbl_write(tc_dev->s_data->regmap, ar0234_start));
	return 0;
}

static int ar0234_stop_stream(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct ar0234 *self = (struct ar0234*)tegracam_get_privdata(tc_dev);
	int err;
	dev_dbg(dev, "stop stream.");

	if (self->deserializer) {
		ub960_s_stream(self->deserializer, self->client, 0);
	}

	TRY(err, reg_tbl_write(tc_dev->s_data->regmap, ar0234_stop));
	return 0;
}

static int ar0234_deserializer_parse(struct ar0234 *self,
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

static struct camera_common_pdata *ar0234_parse_dt(
		struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	struct gpio_desc *reset_gpio;
	int err;

	if (!np)
		return NULL;

#if 0 // Don't need anything special from device_id
	const struct of_device_id *match;
	match = of_match_device(ar0234_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}
#endif

	board_priv_pdata = devm_kzalloc(dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(reset_gpio)) {
		if (PTR_ERR(reset_gpio) != -ENOENT) {
			dev_err(dev, "Failed to get reset gpio");
			return ERR_CAST(reset_gpio);
		} else {
			dev_warn(dev, "Failed to find reset gpio");
		}
	} else {
		board_priv_pdata->reset_gpio = desc_to_gpio(reset_gpio);
	}

	return board_priv_pdata;
}

static int ar0234_power_get(struct tegracam_device *tc_dev)
{
	return 0;
}

static int ar0234_power_put(struct tegracam_device *tc_dev)
{
	return 0;
}

static const struct reg_tbl_t *ar0234_mode_tables[] = {
	[AR0234_MODE_1080P_30FPS] = ar0234_1080p_30fps,
	[AR0234_MODE_1080P_60FPS] = ar0234_1080p_60fps,
	[AR0234_MODE_1200P_60FPS] = ar0234_1200p_60fps,
	[AR0234_MODE_720P_60FPS] = ar0234_720p_60fps,
	[AR0234_MODE_720P_120FPS] = ar0234_720p_120fps,
	[AR0234_MODE_480P_120FPS] = ar0234_480p_120fps,
};


static int ar0234_set_mode(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ar0234 *priv = (struct ar0234 *)s_data->priv;
	const struct reg_tbl_t *table;
	int err;
	unsigned int mask = 0;
	unsigned int value = 0;

	dev_dbg(dev, "Set mode %d @ %d FPS", s_data->sensor_mode_id, priv->framerate);

	if (s_data->mode < ARRAY_SIZE(ar0234_mode_tables)
		&& ar0234_mode_tables[s_data->sensor_mode_id]) {
		table = ar0234_mode_tables[s_data->sensor_mode_id];
	} else {
		dev_err(dev, "Invalid mode");
		return -EINVAL;
	}

	// Write out mode settings
	TRY(err, reg_tbl_write(priv->s_data->regmap, table));

	// Check if frame sync should be enabled
	if(priv->frame_sync_enabled) {
		// This frame sync is enabled via "Surround View", in this mode
		// the AR0234 provides a continuous MIPI clock to the
		// deserializer which we require. "Pulse Trigger Mode" and
		// "Automatic Trigger Mode" do not support a continuous MIPI
		// clock.
		// 	SLAVE_SH_SYNC_MODE  FRAME_START_MODE
		mask = 	(1 << 8)          | (1 << 5);
		value = (1 << 8)          | (1 << 5);

		err = regmap_update_bits(tc_dev->s_data->regmap,
					 AR0234_REG_GRR_CONTROL1,
					 mask, value);

		if(err) {
			dev_err(dev, "Failed to put sensor into Surround View Mode: %d", err);
			return -EIO;
		}
	} else {
		// Disable "Surround View", will no longer respond to frame
		// sync signal.
		// 	SLAVE_SH_SYNC_MODE  FRAME_START_MODE
		mask = 	(1 << 8)          | (1 << 5);
		value = (0 << 8)          | (0 << 5);

		err = regmap_update_bits(tc_dev->s_data->regmap,
					 AR0234_REG_GRR_CONTROL1,
					 mask, value);

		if(err) {
			dev_err(dev, "Failed to pull sensor out of Surround View Mode: %d",
				err);
			return -EIO;
		}
	}
	return err;
}

static const int ar0234_mode_30_fps[] = {
	30, 25, 10,
};

static const int ar0234_mode_60_fps[] = {
	60, 30, 25, 10,
};

static const int ar0234_mode_120_fps[] = {
	120, 100, 90, 60, 30, 25, 10,
};

static const struct camera_common_frmfmt ar0234_frmfmt[] = {
	{
		.size = {1920, 1080},
		.framerates = ar0234_mode_30_fps,
		.num_framerates = ARRAY_SIZE(ar0234_mode_30_fps),
		.hdr_en = false,
		.mode = AR0234_MODE_1080P_30FPS,
	},
	{
		.size = {1920, 1080},
		.framerates = ar0234_mode_60_fps,
		.num_framerates = ARRAY_SIZE(ar0234_mode_60_fps),
		.hdr_en = false,
		.mode = AR0234_MODE_1080P_60FPS,
	},
	{
		.size = {1920, 1200},
		.framerates = ar0234_mode_60_fps,
		.num_framerates = ARRAY_SIZE(ar0234_mode_60_fps),
		.hdr_en = false,
		.mode = AR0234_MODE_1200P_60FPS,
	},
	{
		.size = {1280, 720},
		.framerates = ar0234_mode_60_fps,
		.num_framerates = ARRAY_SIZE(ar0234_mode_60_fps),
		.hdr_en = false,
		.mode = AR0234_MODE_720P_60FPS,
	},
	{
		.size = {1280, 720},
		.framerates = ar0234_mode_120_fps,
		.num_framerates = ARRAY_SIZE(ar0234_mode_120_fps),
		.hdr_en = false,
		.mode = AR0234_MODE_720P_120FPS,
	},
	{
		.size = {640, 480},
		.framerates = ar0234_mode_120_fps,
		.num_framerates = ARRAY_SIZE(ar0234_mode_120_fps),
		.hdr_en = false,
		.mode = AR0234_MODE_480P_120FPS,
	},
};

static struct camera_common_sensor_ops ar0234_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ar0234_frmfmt),
	.frmfmt_table = ar0234_frmfmt,
	.power_on = ar0234_power_on,
	.power_off = ar0234_power_off,
	.start_streaming = ar0234_start_stream,
	.stop_streaming = ar0234_stop_stream,

	.parse_dt = ar0234_parse_dt,
	.power_get = ar0234_power_get,
	.power_put = ar0234_power_put,
	.set_mode = ar0234_set_mode,
};

static int ar0234_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops ar0234_subdev_internal_ops = {
	// No ops needed
	.open = ar0234_open,
	// Hook to allow us to add custom controls
	.registered = ar0234_ctrls_init,
};

static const struct regmap_access_table ar0234_regmap_rw_table = {
	.yes_ranges = ar0234_regmap_yes_ranges,
	.n_yes_ranges = ARRAY_SIZE(ar0234_regmap_yes_ranges),
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.wr_table = &ar0234_regmap_rw_table,
	.rd_table = &ar0234_regmap_rw_table,
	.max_register = 0x3FEE,
};

static int ar0234_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct ar0234 *priv;
	const struct sensor_mode_properties *mode;
	int ret = 0;

	dev_info(dev, "probing ar0234");

	/* Validate if should probe */
	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	/* Allocate memory */
	dev_info(dev, "allocating mem for ar0234");
	TRY_MEM(priv, devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL));
	TRY_MEM(priv->custom_ctrl_handler, devm_kzalloc(dev, sizeof(*priv->custom_ctrl_handler), GFP_KERNEL));
	TRY_MEM(tc_dev, devm_kzalloc(dev, sizeof(*tc_dev), GFP_KERNEL));

	/* Prepare tc_dev and ar0234 structs */
	priv->client = client;
	priv->dev = dev;
	tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ar0234", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &ar0234_common_ops;
	tc_dev->v4l2sd_internal_ops = &ar0234_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ar0234_ctrl_ops;
	priv->frame_sync_enabled = false;

	/* Register tc_dev with tegracam framework */
	ret = tegracam_device_register(tc_dev);
	if (ret) {
		dev_err(dev, "tegra camera driver registration failed");
		return ret;
	}

	/* Prepare ar0234 and tc_dev structs further */
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void*)priv);

	/* Register tc_dev with v4l2 through tegracam function */
	/* This will also add our custom controls through the .registered hook */
	ret = tegracam_v4l2subdev_register(tc_dev, true);
	if (ret) {
		dev_err(dev, "tegra camera subdev registration failed");
		return ret;
	}

	mode = &priv->s_data->sensor_props.sensor_modes[priv->s_data->mode];
	priv->framerate = mode->control_properties.default_framerate;

	/* Register with reg_ctl for sysfs register access */
	priv->reg_ctl = reg_ctl_init(priv->s_data->regmap, &client->dev);

	if (ar0234_deserializer_parse(priv, &priv->deserializer) == 0) {
		dev_dbg(priv->dev, "deserializer present");
	}
	else {
		priv->deserializer = NULL;
	}

	dev_info(dev, "Probed ar0234 imager");
	return 0;
}

static int ar0234_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0234 *priv = (struct ar0234 *)s_data->priv;

	reg_ctl_free(priv->reg_ctl);
	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct of_device_id ar0234_of_match[] = {
	{.compatible = "d3,ar0234",},
	{ },
};
MODULE_DEVICE_TABLE(of, ar0234_of_match);

static const struct i2c_device_id ar0234_id[] = {
	{"ar0234", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ar0234_id);

static struct i2c_driver ar0234_i2c_driver = {
	.driver = {
		.name = "ar0234",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ar0234_of_match),
	},
	.probe = ar0234_probe,
	.remove = ar0234_remove,
	.id_table = ar0234_id,
};

module_i2c_driver(ar0234_i2c_driver);

MODULE_DESCRIPTION("Driver for AR0234 camera on NVIDIA Jetson");
MODULE_VERSION(D3_JETSON_BSP_VERSION);
MODULE_AUTHOR("Tyler Hart <thart@d3engineering.com>");
MODULE_LICENSE("GPL v2");
