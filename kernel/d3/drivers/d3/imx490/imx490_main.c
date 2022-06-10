/**
 * @author Greg Rowe <growe@d3engineering.com> @author Christopher
 * White <cwhite@d3engineering.com> @author Jacob Kiggins
 * <jkiggins@d3engineering.com>
 *
 * imx490 v4l2 driver for Nvidia Jetson
 *
 * Copyright (c) 2018-2021, D3 Engineering.  All rights reserved.
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

#include <linux/seq_file.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>

#include <media/camera_common.h>
#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

#include <d3/d3-jetson-bsp.h>
#include <d3/ub960.h>
#include <d3/fixedpt.h>
#include <d3/reg_ctl.h>

#include "imx490.h"
#include "imx490_modes.h"
#include "imx490_ctrls.h"
#include "cam_rw.h"
#include <d3/common.h>


/* Human-readable names of values of %IMX490_REG_DEVSTS, for debugging */
static const char *imx490_procsts_names[] = {
	"low power", "streaming", "temperature"
};

const u8 imx490_procsts_reg_map[] =
{
	0x04,  // IMX490_PROCSTS_LOWPOWER
	0x20,  // IMX490_PROCSTS_STREAMING
	0x08,  // IMX490_PROCSTS_TEMP_MEASURE
};

/**
 * The 390 has 16 bit addresses and 8 bit values.
 */
static struct regmap_config imx490_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

/* Index in the above table of 30 fps */
/* #define IMX490_INTERVAL_30FPS (3) */

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_HDR_EN
};

static struct tegracam_ctrl_ops imx490_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx490_set_gain,
	.set_exposure = imx490_set_exposure,
	.set_frame_rate = imx490_set_frame_rate,
	.set_group_hold = imx490_set_group_hold_op,
};


/**
 * imx490_wait_for_devsts() - wait for the sensor to be in a particular status
 *
 * Return: 0 when the device reaches the status,
 *         -ETIME if the device does not reach the status,
 *         negative != -ETIME on other errors.
 */
static int
imx490_wait_for_devsts(struct imx490 *self, enum imx490_procsts desired_status)
{
	int tries, err;
	u8  procsts;
	if (desired_status >= IMX490_NUM_PROCSTS)
		return -EINVAL;

	dev_dbg(self->dev, "Waiting for %s...", imx490_procsts_names[desired_status]);

	for (tries = 50; tries > 0; --tries) { /* 50 is arbitrary */
		self->silent_read = true;
		err		  = imx490_reg_read(self, IMX490_REG_PROCSTS, &procsts);
		self->silent_read = false;
		if (err == 0 && procsts == imx490_procsts_reg_map[desired_status]) {
			dev_dbg(self->dev, "... reached %s", imx490_procsts_names[desired_status]);
			return 0;
		}
		/* ignore errors, which may just be timeouts */
		usleep_range(60, 200); /* empirical range */
	}

	return -ETIME;
}

static int
imx490_deserializer_parse(struct tegracam_device* tc_dev, struct i2c_client **out)
{
	struct device_node* node = tc_dev->dev->of_node;
	struct device_node* deserializer_node;
	struct i2c_client* deserializer_client;

	deserializer_node = of_parse_phandle(node, "deserializer", 0);
	if (!deserializer_node) {
		dev_dbg(tc_dev->dev, "could not find deserializer node");
		return -ENOENT;
	}

	deserializer_client = of_find_i2c_device_by_node(deserializer_node);
	of_node_put(deserializer_node);
	deserializer_node = NULL;

	if (!deserializer_client) {
		dev_dbg(tc_dev->dev, "missing deserializer client");
		return -ENOENT;
	}

	*out = deserializer_client;
	return 0;
}

/**
 * imx490_change_stream() - start or stop the stream
 * @sd: sub device
 * @enable: nonzero to start streaming; zero to stop streaming
 *
 * Postcondition: if the return value is 0, the sensor is in state:
 *                active, if @enable; or
 *                standby, if not @enable.
 *
 * Return: 0 on success; -1 on failure
 */
static int
imx490_change_stream(struct tegracam_device* tc_dev, bool enable)
{
	int			   err	   = 0;
	struct imx490 *		   self	   = (struct imx490 *)tc_dev->s_data->priv;
	int			   mode_ix = self->s_data->sensor_mode_id;

	dev_info(tc_dev->dev, "changing stream");

	TRY_DEV_HERE(err, regmap_write(self->map, IMX490_REG_LOWPOWER, IMX490_LOWPOWER_ENTER),
		     self->dev);
	TRY_DEV_HERE(err, imx490_wait_for_devsts(self, IMX490_PROCSTS_LOWPOWER), tc_dev->dev);
	/* now the sensor is in Low Power */

	if (enable) {
		/* write most of the regs */
		/* TRY_DEV_HERE(err, imx490_set_mode(tc_dev), tc_dev->dev); */

		/* Disable both embedded metadata lines */
		TRY_DEV_HERE(err, regmap_write(self->map, IMX490_REG_FRONT_EMBDATA_LINE, 0),
			     self->dev);
		TRY_DEV_HERE(err, regmap_write(self->map, IMX490_REG_REAR_EMBDATA_LINE, 0),
			     self->dev);

		TRY_DEV_HERE(err, regmap_write(self->map, IMX490_REG_LOWPOWER, IMX490_STREAM_ENTER_ONE),
			     self->dev);
		TRY_DEV_HERE(err, regmap_write(self->map, IMX490_REG_LOWPOWER, IMX490_STREAM_ENTER_TWO),
			     self->dev);

		TRY_DEV_HERE(err,
			     imx490_wait_for_devsts(self, IMX490_PROCSTS_STREAMING),
			     self->dev);
	}

	if (self->deserializer) {
		ub960_s_stream(self->deserializer, self->client, enable);
	}

	dev_info(tc_dev->dev, "%s in mode %d", enable ? "STARTED" : "STOPPED", mode_ix);
	return 0;
}

static int
imx490_start_stream(struct tegracam_device* tc_dev)
{
	return imx490_change_stream(tc_dev, true);
}

static int
imx490_stop_stream(struct tegracam_device* tc_dev)
{
	return imx490_change_stream(tc_dev, false);
}

static int imx490_power_get(struct tegracam_device *tc_dev)
{
	// TODO: figure out what this function should do, and implement it
	return 0;
}

static int imx490_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}


/**
 * Called by Linux when unloading the driver instance.
 *
 * @param client which instance
 *
 * @return 0, always
 */
static int
imx490_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx490* self = (struct imx490*)s_data->priv;

	tegracam_v4l2subdev_unregister(self->tc_dev);
	tegracam_device_unregister(self->tc_dev);

	return 0;
}

static const struct i2c_device_id imx490_id[] = { { "imx490", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, imx490_id);

static struct of_device_id imx490_of_match[] = {
	{ .compatible = "d3,imx490" },
	{},
};
MODULE_DEVICE_TABLE(of, imx490_of_match);

static void
imx490_gpio_set(unsigned int gpio, int val)
{
	if (gpio_cansleep(gpio))
		gpio_set_value_cansleep(gpio, val);
	else
		gpio_set_value(gpio, val);
}

static int
imx490_power_on(struct camera_common_data *s_data)
{
	struct imx490 *			 priv = (struct imx490 *)s_data->priv;
	struct camera_common_power_rail *pw   = s_data->power;

	dev_dbg(priv->dev, "%s: power on\n", __func__);
	pw->state = SWITCH_ON;
	// Note: camera_common_set_power() is declared in camera_common.h
	// but apparently not defined anywhere.  Therefore, I am not calling
	// it here.
	return 0;
}

static int
imx490_power_off(struct camera_common_data *s_data)
{
	struct imx490 *			 priv = (struct imx490 *)s_data->priv;
	struct camera_common_power_rail *pw   = s_data->power;

	dev_dbg(priv->dev, "%s: power off\n", __func__);
	pw->state = SWITCH_OFF;
	// see note in imx490_power_on().
	return 0;
}


/**
 * Pull parameters from device tree.
 *
 * @param tc_dev tegracam device
 *
 * @return 0 on success
 */
static struct camera_common_pdata *
imx490_parse_dt(struct tegracam_device *tc_dev)
{
	struct device_node* node = tc_dev->dev->of_node;
	struct camera_common_pdata* board_priv_pdata;
	const struct of_device_id* match;
	int err = 0;
	int gpio = 0;

	dev_dbg(tc_dev->dev, "imx490_parse_dt:");

	/* Check if node exists */
	if (!node) {
		dev_err(tc_dev->dev, "no OF node");
		return NULL;
	}

	/* Look for device tree match */
	match = of_match_device(imx490_of_match, tc_dev->dev);
	if (!match) {
		dev_err(tc_dev->dev, "Failed to find matching dt id");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(tc_dev->dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	/* Check for MCLK */
	err = of_property_read_string(node, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_warn(tc_dev->dev, "mclk not in DT");

	/* Get reset gpio */
	gpio = of_get_named_gpio(node, "reset_gpio", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			board_priv_pdata = ERR_PTR(-EPROBE_DEFER);
		dev_err(tc_dev->dev, "reset_gpio not found %d\n", gpio);
	}
	else
		board_priv_pdata->reset_gpio = (unsigned int)gpio;


	return board_priv_pdata;
} // imx490_parse_dt()

/* Camera common ops */
static struct camera_common_sensor_ops imx490_common_ops = {
	.power_on  = imx490_power_on,
	.power_off = imx490_power_off,
	.write_reg = imx490_reg_write_op,
	.read_reg  = imx490_reg_read_op,
	.start_streaming = imx490_start_stream,
	.stop_streaming = imx490_stop_stream,

	.parse_dt = imx490_parse_dt,
	.power_get = imx490_power_get,
	.power_put = imx490_power_put,
	.set_mode = imx490_set_mode,
};


/**
 * Called when the device is opened. For example this is called when
 * you execute `media-ctl -p` from userspace.
 *
 * @param sd sub device
 * @param fh
 *
 * @return 0, always
 */
static int
imx490_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_dbg(&client->dev, "open");
	return 0;
}

static const struct v4l2_subdev_internal_ops imx490_subdev_internal_ops = {
	.open = imx490_open,
};

/*
 * imx490_wait_for_sensor() - Waits for the sensor to come up
 * @self: the instance
 *
 * On success, logs the silicon revision.
 *
 * Return: 0 on success
 */
static int
imx490_wait_for_sensor(struct imx490 *self)
{
	int err = 0;
	/* It's time to make this a loop... */
	u8  procsts = 0xff;
	int tries;

	for (tries = 50; tries >= 0; --tries) {
		dev_info(self->dev, "Trying...");
		err = imx490_reg_read(self, 0x5C84, &procsts);
		if (err) {
			usleep_range(10 * 1000, 10 * 1000);
			continue;
		}

		dev_info(self->dev, "procsts = %#x", procsts);
		return 0;
	}

	return err;
}

/**
 * imx490_reset_gpio_get() - Gets the reset gpio and adds it to the priv data on success
 * Return: 0 on success; other on failure.
 */
int
imx490_reset_gpio_get(struct imx490 *self)
{
	int ret = 0;
	int err = 0;
	ret	= of_get_named_gpio(self->client->dev.of_node, "reset_gpio", 0);
	if (!gpio_is_valid(ret)) {
		dev_dbg(self->dev, "No reset_gpio present, continuing without: %d", ret);
		return ret;
	}
	dev_dbg(self->dev, "Found reset_gpio");
	TRY_DEV_HERE(err,
		     gpio_direction_output(ret, 0), self->dev);
	dev_dbg(self->dev, "Finished setting up reset gpio as output");
	self->reset_gpio = ret;

	return 0;
}

/**
 * imx490_report_modes() - report information about sensor modes
 */
void
imx490_report_modes(struct imx490 *self)
{
	int			       idx, nmodes;
	struct sensor_mode_properties *sensor_modes;

	sensor_modes = self->s_data->sensor_props.sensor_modes;
	nmodes	     = self->s_data->sensor_props.num_modes;

	for (idx = 0; idx < nmodes; ++idx) {
		struct sensor_control_properties *cprop = &sensor_modes[idx].control_properties;
		dev_dbg(self->dev, "<mode %d>", idx);
		// dev_dbg(self->dev, "  gain_factor %u", cprop->gain_factor);
		// dev_dbg(self->dev, "  framerate_factor %u", cprop->framerate_factor);
		dev_dbg(self->dev, "  inherent_gain %u", cprop->inherent_gain);
		dev_dbg(self->dev, "  min_gain_val %u", cprop->min_gain_val);
		dev_dbg(self->dev, "  max_gain_val %u", cprop->max_gain_val);
		dev_dbg(self->dev, "  min_hdr_ratio %u", cprop->min_hdr_ratio);
		dev_dbg(self->dev, "  max_hdr_ratio %u", cprop->max_hdr_ratio);
		dev_dbg(self->dev, "  min_framerate %u", cprop->min_framerate);
		dev_dbg(self->dev, "  max_framerate %u", cprop->max_framerate);
		dev_dbg(self->dev, "  min_exp_time %llu", cprop->min_exp_time.val);
		dev_dbg(self->dev, "  max_exp_time %llu", cprop->max_exp_time.val);
		// dev_dbg(self->dev, "  step_gain_val %u", cprop->step_gain_val);
		dev_dbg(self->dev, "  step_framerate %u", cprop->step_framerate);
		dev_dbg(self->dev, "  exposure_factor %u", cprop->exposure_factor);
		// dev_dbg(self->dev, "  step_exp_time %llu", cprop->step_exp_time.val);
		dev_dbg(self->dev, "  default_gain %u", cprop->default_gain);
		dev_dbg(self->dev, "  default_framerate %u", cprop->default_framerate);
		dev_dbg(self->dev, "  default_exp_time %llu", cprop->default_exp_time.val);
		// dev_dbg(self->dev, "  is_interlaced %u", cprop->is_interlaced);
		// dev_dbg(self->dev, "  interlace_type %u", cprop->interlace_type);
		dev_dbg(self->dev, "</mode %d>", idx);
	}
}


int
imx490_reset(struct imx490 *self)
{
	int err = 0;
	if (self->reset_gpio) {
		dev_dbg(self->dev, "putting sensor into reset");
		imx490_gpio_set(self->reset_gpio, 0);
		usleep_range(100000, 150000);

		dev_dbg(self->dev, "taking sensor out of reset");
		imx490_gpio_set(self->reset_gpio, 1);
		usleep_range(100000, 150000);
	}

	TRY(err, imx490_wait_for_sensor(self));
	TRY_DEV_HERE(err, imx490_wait_for_devsts(self, IMX490_PROCSTS_LOWPOWER), self->dev);
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
static int
imx490_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct imx490* self = NULL;
	struct tegracam_device* tc_dev = NULL;
	/* struct reg_ctl_dev rc_dev; */

	dev_dbg(&client->dev, "imx490 probing");

	/* Check if should probe */
	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node) {
		dev_err(&client->dev, "imx490 probe aborted");
		return -EINVAL;
	}

	/* Allocate memory */
	TRY_MEM(self, devm_kzalloc(&client->dev, sizeof(*self), GFP_KERNEL));
	TRY_MEM(tc_dev, devm_kzalloc(&client->dev, sizeof(*tc_dev), GFP_KERNEL));

	/* Prepare tc_dev and imx490 structs */
	self->client = client;
	tc_dev->client = client;
	self->dev = &client->dev;
	tc_dev->dev = &client->dev;
	strncpy(tc_dev->name, "imx490", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &imx490_regmap_config;
	imx490_common_ops.frmfmt_table = imx490_modes_formats;
	imx490_common_ops.numfrmfmts = imx490_modes_formats_len;
	tc_dev->sensor_ops = &imx490_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx490_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx490_ctrl_ops;


	dev_dbg(tc_dev->dev, "before: tegracam_device_register");

	/* Register tc_dev with tegracam framework */
	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(tc_dev->dev, "tegra camera driver registration failed");
		return err;
	}

	dev_dbg(tc_dev->dev, "after: tegracam_device_register");

	/* Prepare imx490 and tc_dev structs further */
	self->tc_dev = tc_dev;
	self->s_data = tc_dev->s_data;
	self->subdev = &tc_dev->s_data->subdev;
	self->map = tc_dev->s_data->regmap;
	tegracam_set_privdata(tc_dev, (void*)self);


	// Attempt to take sensor out of reset
	self->reset_gpio = 0;
	imx490_reset_gpio_get(self);

	TRY(err, imx490_reset(self));

	TRY(err, imx490_detect_revision(self));

	/* Register tc_dev with v4l2 through tegracam function */
	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(tc_dev->dev, "tegra camera subdev registration failed");
		return err;
	}


	/* Get pointer to v4l2_ctrl_handler to add custom controls */
	self->ctrl_handler = NULL;
	self->ctrl_handler = tc_dev->s_data->ctrl_handler;
	if (self->ctrl_handler == NULL) {
		dev_err(self->dev, "can't get v4l2_ctrl_handler, pointer is NULL");
	}
	else {
		dev_err(self->dev, "got v4l2_ctrl_handler: %p", (void*)self->ctrl_handler);
		imx490_ctrls_init(self);
	}

	if (imx490_deserializer_parse(tc_dev, &self->deserializer) == 0) {
		dev_dbg(tc_dev->dev, "deserializer present");
	} else {
		self->deserializer = NULL;
	}
	/* /\* Prepare for reg_ctl *\/ */
	/* rc_dev.s_data = self->s_data; */
	/* rc_dev.reg_read = cam_rw_read_reg; */
	/* rc_dev.reg_write = cam_rw_write_reg; */
	/* /\* Register with reg_ctl for sysfs register access *\/ */
	/* reg_ctl_init(rc_dev, &client->dev); */

	dev_info(self->dev, "probe success");

	// Report debugging information
	imx490_read_sensratio(self);
	imx490_report_modes(self);

	return 0;
}

/**
 * Linux driver data
 */
static struct i2c_driver imx490_driver = {
	.driver =
	{
		.name		= "imx490",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(imx490_of_match),
	},
	.probe    = imx490_probe,
	.remove   = imx490_remove,
	.id_table = imx490_id,
};
module_i2c_driver(imx490_driver);

MODULE_DESCRIPTION("Driver for D3's Sony IMX490 camera for Nvidia Jetson");
MODULE_VERSION(D3_JETSON_BSP_VERSION);
MODULE_AUTHOR("Jacob Kiggins <jkiggins@d3engineering.com>");
MODULE_LICENSE("GPL v2");
