/*
 * ar0234_ctrl.c - ar0234 sensor controls
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
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <linux/bitfield.h>

#include "ar0234.h"
#include "ar0234_ctrl.h"

enum {
	AR0234_CID_BASE = (TEGRA_CAMERA_CID_BASE + 200),
	AR0234_CID_FRAME_SYNC,
};

// Tegracam exposed settings
static int ar0234_set_gain(struct tegracam_device *tc_dev, s64 val);
static int ar0234_set_exposure(struct tegracam_device *tc_dev, s64 val);
static int ar0234_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
static int ar0234_set_group_hold(struct tegracam_device *tc_dev, bool val);


// V4L2 custom settings
static int ar0234_set_frame_sync(struct v4l2_ctrl *ctrl);

const u64 ANALOG_GAIN_FACTOR = 100000;
// From dev guide, AND8920 September, 2020 - Rev. 2, pages 40 & 41
const struct ar0234_analog_gain_settings_t ar0234_analog_gain_table[] = {
	{0x0, 0x000D, 168421, 0x2, 0x6, 0x6},
	{0x0, 0x000E, 177778, 0x2, 0x6, 0x6},
	{0x0, 0x000F, 188235, 0x2, 0x6, 0x6},
	{0x1, 0x0010, 200000, 0x2, 0x6, 0x6},
	{0x1, 0x0012, 213333, 0x2, 0x6, 0x6},
	{0x1, 0x0014, 228572, 0x2, 0x6, 0x6},
	{0x1, 0x0016, 246154, 0x2, 0x6, 0x6},
	{0x1, 0x0018, 266667, 0x2, 0x6, 0x6},
	{0x1, 0x001A, 290909, 0x2, 0x6, 0x6},
	{0x1, 0x001C, 320000, 0x2, 0x6, 0x6},
	{0x1, 0x001E, 355556, 0x2, 0x6, 0x6},
	{0x2, 0x0020, 400000, 0x1, 0x6, 0x6},
	{0x2, 0x0021, 412903, 0x1, 0x6, 0x6},
	{0x2, 0x0022, 426667, 0x1, 0x6, 0x6},
	{0x2, 0x0023, 441379, 0x1, 0x6, 0x6},
	{0x2, 0x0024, 457144, 0x1, 0x6, 0x6},
	{0x2, 0x0025, 474074, 0x1, 0x6, 0x6},
	{0x2, 0x0026, 492308, 0x1, 0x6, 0x6},
	{0x2, 0x0027, 512000, 0x1, 0x6, 0x6},
	{0x2, 0x0028, 533333, 0x1, 0x6, 0x6},
	{0x2, 0x0029, 556522, 0x1, 0x6, 0x6},
	{0x2, 0x002A, 581818, 0x1, 0x6, 0x6},
	{0x2, 0x002B, 609524, 0x1, 0x6, 0x6},
	{0x2, 0x002C, 640000, 0x1, 0x6, 0x6},
	{0x2, 0x002D, 673684, 0x1, 0x6, 0x6},
	{0x2, 0x002E, 711111, 0x1, 0x6, 0x6},
	{0x2, 0x002F, 752941, 0x1, 0x6, 0x6},
	{0x3, 0x0030, 800000, 0x1, 0x6, 0x6},
	{0x3, 0x0032, 853333, 0x1, 0x6, 0x6},
	{0x3, 0x0034, 914286, 0x1, 0x6, 0x6},
	/* {0x3, 0x0036, 984616, 0x1, 0x0, 0x6}, */
	/* {0x3, 0x0038, 1066667, 0x1, 0x0, 0x6}, */
	/* {0x3, 0x003A, 1163636, 0x0, 0x0, 0x6}, */
	/* {0x3, 0x003C, 1290000, 0x0, 0x0, 0x6}, */
	/* {0x3, 0x003E, 1422222, 0x0, 0x0, 0x6}, */
	/* {0x4, 0x0040, 1600000, 0x0, 0x0, 0x6}, */
};

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct tegracam_ctrl_ops ar0234_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = ar0234_set_gain,
	.set_exposure = ar0234_set_exposure,
	.set_frame_rate = ar0234_set_frame_rate,
	.set_group_hold = ar0234_set_group_hold,
};

// Custom V4L2 settings
static const struct v4l2_ctrl_ops ar0234_frame_sync_ctrl_ops = {
	.s_ctrl = ar0234_set_frame_sync,
};

struct v4l2_ctrl_config ar0234_custom_controls[] = {
	{
		.ops = &ar0234_frame_sync_ctrl_ops,
		.id = AR0234_CID_FRAME_SYNC,
		.name = "Frame Sync",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 1,
		.def = 0,
		.step = 1,
	}
};
unsigned int ar0234_num_custom_controls = ARRAY_SIZE(ar0234_custom_controls);


/**
 * Tegracam doesn't officially support custom controls, so this method initializes
 * its own v4l2_ctrl_handler, adds the custom controls defined in
 * ar0234_custom_controls, and then merges it with Tegracam's internal
 * v4l2_ctrl_handler. This should not be called directly - it should be declared
 * as the .registered hook in tc_dev->v4l2sd_internal_ops.
 *
 * @see https://stackoverflow.com/a/68002497
 * @param sd V4L2 Sub-device (automagically passed)
 * @return 0 on success, anything else on failure
 */
int ar0234_ctrls_init(struct v4l2_subdev *sd) {
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);

	struct ar0234 *priv = (struct ar0234 *)s_data->priv;
	struct v4l2_ctrl *ctrl;
	int i, err;

	// Pre-flight checks
	if (priv == NULL || priv->custom_ctrl_handler == NULL
			|| sd == NULL || sd->ctrl_handler == NULL) {
		dev_err(dev, "Failed to obtain a control handler");
		return -EINVAL;
	}

	// Initialize a control handler
	v4l2_ctrl_handler_init(priv->custom_ctrl_handler, ar0234_num_custom_controls);

	// Add each custom control in ar0234_custom_controls
	for (i = 0; i < ar0234_num_custom_controls; i++) {
		dev_dbg(dev, "Registering custom control %d with v4l2", i);
		ctrl = v4l2_ctrl_new_custom(priv->custom_ctrl_handler, &ar0234_custom_controls[i], NULL);

		if (ctrl == NULL) {
			// Note: ctrl_handler->error is set only by the first failed addition
			dev_err(dev, "Failed to add custom control at index %d, error %d", i, priv->custom_ctrl_handler->error);
			return -EINVAL;
		}
	}

	// Add control handler with our custom controls to Tegracam's
	if ((err = v4l2_ctrl_add_handler(sd->ctrl_handler, priv->custom_ctrl_handler, NULL, false))) {
		dev_err(dev, "Failed to add control handler, error %d", err);
		return err;
	}

	return 0;
}

static int ar0234_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;
	struct regmap *map = tc_dev->s_data->regmap;
	struct camera_common_data *s_data = tc_dev->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	const struct ar0234_analog_gain_settings_t *analog_gain;
	u64 gain_factor = mode->control_properties.gain_factor;
	const unsigned MIN_DIGITAL_GAIN = BIT(7);
	const unsigned MAX_DIGITAL_GAIN = 0x7FF;
	u64 digital_gain;
	u16 reg_30ba;
	int i = 0;
	int ret;

	// Pick the closest total analog gain value that is lower than the requested gain
	analog_gain = &ar0234_analog_gain_table[0];
	for(i = 0; i < ARRAY_SIZE(ar0234_analog_gain_table); i++) {
		analog_gain = &ar0234_analog_gain_table[i];

		// requested <= selected, return the previously selected analog gain
		if ((val * ANALOG_GAIN_FACTOR / gain_factor) <= analog_gain->total) {
			// If i==0, return the 0-th gain
			if (i > 0)
				analog_gain = &ar0234_analog_gain_table[i-1];
			break;
		}
	}

	// From dev guide, AND8920 September, 2020 - Rev. 2, page 40:
	//   The format for digital gain setting is xxxx.yyyyyyy where
	//   0b00010000000 represents a 1x gain setting and
	//   0b00011000000 represents a 1.5x gain setting. The step size
	//   for yyyyyyy is 0.0078125 while the step size for xxxx is 1.
	//   Therefore to set a gain of 2.09375 one would set digital gain
	//   to 0b00100001100. The maximum digital gain is 15.9922x.
	//   DigitalGain = Bit[10:7] + (Bit[6:0]/128)
	digital_gain = (val << 7) / (analog_gain->total * gain_factor / ANALOG_GAIN_FACTOR);

	if (digital_gain > MAX_DIGITAL_GAIN)
		digital_gain = MAX_DIGITAL_GAIN;
	else if (digital_gain < MIN_DIGITAL_GAIN)
		digital_gain = MIN_DIGITAL_GAIN;

	if (mode->signal_properties.pixel_clock.val >= 90000000llu)
		reg_30ba = analog_gain->reg_30ba_90mhz;
	else if (mode->signal_properties.pixel_clock.val >= 45000000llu)
		reg_30ba = analog_gain->reg_30ba_45mhz;
	else // 22500000llu
		reg_30ba = analog_gain->reg_30ba_22p5mhz;

	/* This approach only works to generate a base-10 decimal output because
	 *  ANALOG_GAIN_FACTOR and gain_factor are powers of 10
	 */
	dev_dbg(dev, "gain %llu.%llu -> (%llu.%llu) * (%llu.%llu), 0x%01x RESERVED_MFR_30BA",
			val / gain_factor,
			val % gain_factor,
			analog_gain->total / ANALOG_GAIN_FACTOR,
			analog_gain->total % ANALOG_GAIN_FACTOR,
			digital_gain >> 7,
			// Extract 3 base-10 decimal bits from digital gain
			((digital_gain & GENMASK(7,0)) * 1000) >> 7,
			reg_30ba);

	ret = regmap_update_bits(map, AR0234_REG_ANALOG_GAIN,
		GENMASK(6, 4)
		| GENMASK(3, 0),
		FIELD_PREP(GENMASK(6, 4), analog_gain->coarse)
		| FIELD_PREP(GENMASK(3, 0), analog_gain->fine));
	if (ret)
		return ret;

	ret = regmap_update_bits(map, AR0234_REG_RESERVED_MFR_30BA,
		GENMASK(2, 0),
		FIELD_PREP(GENMASK(2, 0), reg_30ba));
	if (ret)
		return ret;

	ret = regmap_write(map,
					   AR0234_REG_GLOBAL_GAIN,
					   FIELD_PREP(GENMASK(15,0), digital_gain));
	if (ret)
		return ret;

	return 0;
}

static int ar0234_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;
	struct regmap *map = tc_dev->s_data->regmap;
	struct camera_common_data *s_data = tc_dev->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	u64 exposure_clks;
	u16 coarse_integration_clks;
	u16 fine_integration_clks;
	const int pixels_per_clk = 2;
	const int MIN_CIT = 2;
	const int MAX_FIT = 80;
	unsigned line_length_clks;
	int ret;

	// Get current LLPCK
	ret = regmap_read(map, AR0234_REG_LINE_LENGTH_PCK, &line_length_clks);
	if (ret)
		return ret;

	// Sanity check that it matches the current mode
	if (line_length_clks * pixels_per_clk != mode->image_properties.line_length) {
		dev_warn(dev, "line_length (%d) should be 2x LINE_LENGTH_PCK (%d)!",
			mode->image_properties.line_length,
			line_length_clks);
	}

	// From dev guide, AND8920 September, 2020 - Rev. 2, page 5:
	// eq. 2: Tclk = (CIT * LLPCK) - 162 + (3 * FIT)
	// eq. 3: T = Tclk / PCLKHZ

	// Exposure -> clks, Tclk = T * PCLKHZ
	exposure_clks = (val * mode->signal_properties.pixel_clock.val) / mode->control_properties.exposure_factor;

	// Assume FIT = 0, CIT = (exposure + 162) / LLPCK
	coarse_integration_clks = (exposure_clks + 162) / line_length_clks;

	// Minimum CIT = 2 (see page 5)
	if (coarse_integration_clks < MIN_CIT)
		coarse_integration_clks = MIN_CIT;

	// Use CIT to calculate FIT = (exposure - (CIT * LLPCK) + 162) / 3
	fine_integration_clks = (exposure_clks - (coarse_integration_clks * line_length_clks) + 162) / 3;

	// FIT must be an even number for all cases. (see page 5)
	fine_integration_clks &= ~1u;

	// FIT ≥ 0 and FIT ≤ 80 (see page 5)
	if (fine_integration_clks > MAX_FIT)
		fine_integration_clks = MAX_FIT;

	dev_dbg(dev, "exposure %lld -> 0x%03x coarse, 0x%02x fine",
		val,
		coarse_integration_clks,
		fine_integration_clks);

	ret = regmap_write(map, AR0234_REG_COARSE_INT_TIME, coarse_integration_clks);
	if (ret)
		return ret;

	ret = regmap_write(map, AR0234_REG_FINE_INT_TIME, fine_integration_clks);
	if (ret)
		return ret;

	return 0;
}

/**
 * Set the frame rate of the sensor. In frame sync mode however, the frame
 * rate should not be manually set, otherwise the frame rate will be less
 * then what was set. For example, manually setting a frame rate of 60 fps
 * with a 60 fps frame sync signal may result in frame rates as low as 30 fps.
 * To combat this issue, when frame sync is enabled, the frame rate is fixed
 * at 120 fps.
 */
static int ar0234_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;
	struct regmap *map = tc_dev->s_data->regmap;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ar0234 *priv = (struct ar0234 *)s_data->priv;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	unsigned line_length_clks;
	unsigned frame_length_lines;
	int ret;

	if(priv->frame_sync_enabled)
		val = 120;

	priv->framerate = val / mode->control_properties.framerate_factor;
	dev_dbg(dev, "framerate: %lld -> %d", val, priv->framerate);

	// Get current LLPCK
	ret = regmap_read(map, AR0234_REG_LINE_LENGTH_PCK, &line_length_clks);
	if (ret)
		return ret;

	frame_length_lines = (mode->signal_properties.pixel_clock.val * mode->control_properties.framerate_factor) / (line_length_clks * val);

	dev_dbg(dev, "(%llu * %u) / (%u * %llu) = %u",
		mode->signal_properties.pixel_clock.val,
		mode->control_properties.framerate_factor,
		line_length_clks,
		val,
		frame_length_lines);

	return regmap_write(map, AR0234_REG_FRAME_LENGTH_LINES, frame_length_lines);
}

// This control is required by 2.0 framework
static int ar0234_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct device *dev = tc_dev->dev;
	struct regmap *map = tc_dev->s_data->regmap;

	dev_dbg(dev, "Group hold %d", val);

	return regmap_update_bits(map, AR0234_REG_GROUPED_PARAMETER_HOLD,
		BIT(0), FIELD_PREP(BIT(0), (val ? 1u : 0u)));
}

static int ar0234_set_frame_sync(struct v4l2_ctrl *ctrl)
{
	struct tegracam_ctrl_handler *handler =
		container_of(ctrl->handler, struct tegracam_ctrl_handler,
			     ctrl_handler);
	struct tegracam_device *tc_dev = handler->tc_dev;
	struct ar0234 *priv = (struct ar0234 *)tc_dev->s_data->priv;

	dev_dbg(tc_dev->dev, "Setting frame sync to: %d", ctrl->val);
	priv->frame_sync_enabled = ctrl->val;
	return 0;
}
