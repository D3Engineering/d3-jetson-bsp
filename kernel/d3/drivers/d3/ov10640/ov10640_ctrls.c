/**
 * @author Josh Watts <jwatts@d3engineering.com>
 * @author Greg Rowe <growe@d3engineering.com>
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
#include <linux/kernel.h>
#include <media/camera_common.h>

#include "ov10640_tables.h"
#include "ov10640_ctrls.h"
#include "ov10640_reg.h"

/* V4l2 control ids */
enum {
	OV10640_CID_BASE = (TEGRA_CAMERA_CID_BASE + 230),
	OV10640_CID_DGAIN,
	OV10640_CID_EXPOSURE,
	OV10640_CID_FLIP_MIRROR,
	OV10640_CID_FSYNC_ENABLE,
};
#define OV10640_MAX_DGAIN	(0x3FFF)
#define OV10640_MIN_DGAIN	(0x0001)
#define OV10640_DEF_DGAIN	(256)
#define OV10640_MAX_EXPO	(33698LL)
#define OV10640_MIN_EXPO	(20LL)
#define OV10640_DEF_EXPO	(20LL)
#define FIXED_POINT_BITS	(22)
#define DGAIN_FIXED_BITS	(8)

/* Values for OV10640_CID_FLIP_MIRROR */
enum {
	OV10640_NO_FLIP_MIRROR    = 0,
	OV10640_MIRROR            = 1,
	OV10640_FLIP              = 2,
	OV10640_FLIP_AND_MIRROR   = 3,
};

/* static int ov10640_read_u16(struct ov10640 *self, unsigned reg_h, */
/* 			    unsigned int *val) */
/* { */
/* 	int err; */
/* 	uint8_t temp[2]; */

/* 	err = regmap_bulk_read(self->map, reg_h, temp, 2); */
/* 	if (err) */
/* 		return err; */

/* 	*val = ((temp[0] & 0xFF) << 8) | (temp[1] & 0xFF); */

/* 	return 0; */
/* } */

static int ov10640_write_u16(struct ov10640 *self, u16 reg_h, u16 val)
{
	uint8_t temp[2];

	temp[0] = (val >> 8) & 0xFF;
	temp[1] = val & 0xFF;

	return regmap_bulk_write(self->map, reg_h, temp, 2);
}

static int ov10640_group_reg_write(struct ov10640 *self, unsigned int reg,
				   unsigned int val)
{
	if (self->group_hold)
		reg |= (1 << 15);

	return regmap_write(self->map, reg, val);
}

static int ov10640_group_reg_write_u16(struct ov10640 *self, unsigned int reg_h,
				       unsigned int val)
{
	if (self->group_hold)
		reg_h |= (1 << 15);

	return ov10640_write_u16(self, reg_h, val);
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
static int ov10640_group_hold_enable(struct ov10640 *self, s32 val)
{
	if (self->group_hold == !!val)
		return 0;

	if (val)
		regmap_write(self->map, OV10640_REG_GROUP_CTRL,
			     OV10640_REG_GROUP_CTRL_PRE_SOF | OV10640_REG_GROUP_CTRL_1ST_GRP(0));
	else
		regmap_write(self->map, OV10640_REG_OPERATION_CTRL,
			     OV10640_REG_OPERATION_CTRL_SINGLE_START);

	self->group_hold = !!val;

	return 0;
}




static int ov10640_gain_set(struct ov10640 *self, s64 val)
{
	u8 gain_exp = 0;
	s64 dgain_fixed = 0;
	u8 cgain = 0;
	u16 raw_gain = 0;
	u16 raw_dig_gain = 0;
	u8 raw_dig_gain_l = 0, raw_dig_gain_h = 0;
	int err = 0;

	if (val >= 8 * FIXED_POINT_SCALING_FACTOR) {
		gain_exp = 3;
		/* cgain = 1; */
	} else if (val >= 4 * FIXED_POINT_SCALING_FACTOR) {
		gain_exp = 2;
		/* cgain = 1; */
	} else if (val > 2 * FIXED_POINT_SCALING_FACTOR) {
		gain_exp = 1;
		/* cgain = 1; */
	} else if (val >= 1 * FIXED_POINT_SCALING_FACTOR) {
		gain_exp = 0;
		/* cgain = 1; */
	} else {
		gain_exp = 0;
		cgain = 0;
	}

	raw_dig_gain = OV10640_DEF_DGAIN;
	
	// scale analog gain using digital gain to achieve an overall gain of "val"
	dgain_fixed = val / (1 << gain_exp);

	dev_dbg(self->dev, "Changing gain to: %lli == %lli\n", val, dgain_fixed * (1 << gain_exp));

	dgain_fixed = (dgain_fixed >> (FIXED_POINT_BITS - DGAIN_FIXED_BITS));
	raw_dig_gain = dgain_fixed & 0x3FFF;
	raw_dig_gain_l = raw_dig_gain & 0xFF;
	raw_dig_gain_h = (raw_dig_gain >> 8) & 0xFF;

	raw_gain = (cgain << 7)
		| (cgain << 6)
		| (gain_exp << 4)
		| (gain_exp << 2)
		| gain_exp;
	TRY(err, ov10640_group_reg_write(self, OV10640_CG_AGAIN, raw_gain));
	
	TRY(err, ov10640_group_reg_write(self, OV10640_DIG_L_GAIN_L, raw_dig_gain_l));
	TRY(err, ov10640_group_reg_write(self, OV10640_DIG_L_GAIN_H, raw_dig_gain_h));
	TRY(err, ov10640_group_reg_write(self, OV10640_DIG_S_GAIN_L, raw_dig_gain_l));
	TRY(err, ov10640_group_reg_write(self, OV10640_DIG_S_GAIN_H, raw_dig_gain_h));
	TRY(err, ov10640_group_reg_write(self, OV10640_DIG_VS_GAIN_L, raw_dig_gain_l));
	TRY(err, ov10640_group_reg_write(self, OV10640_DIG_VS_GAIN_H, raw_dig_gain_h));

	return 0;
}

static int ov10640_framerate_set(struct ov10640 *self, s64 val)
{
	return 0;
}


static int ov10640_flip_mirror_set(struct ov10640 *self, s64 val)
{
	int ret = 0;

	if (val < OV10640_NO_FLIP_MIRROR || val > OV10640_FLIP_AND_MIRROR)
		return -EINVAL;

	ret |= regmap_update_bits(self->map,
							 OV10640_REG_READ_MODE,
							 OV10640_REG_READ_MODE_FLIP_MIRROR_MASK,
							 OV10640_REG_READ_MODE_FLIP_MIRROR(val));
	ret |= regmap_update_bits(self->map,
							 OV10640_REG_R_ISP_CTRL_2,
							 OV10640_REG_R_ISP_CTRL_2_FLIP_MIRROR_MASK,
							 OV10640_REG_R_ISP_CTRL_2_FLIP_MIRROR(val));
	ret |= regmap_update_bits(self->map,
							 OV10640_REG_R_CTRL08,
							 OV10640_REG_R_CTRL08_FLIP_MIRROR_MASK,
							 OV10640_REG_R_CTRL08_FLIP_MIRROR(val));
	return ret;
}

static int ov10640_frame_sync_enable_set(struct ov10640 *self, s64 val)
{
	int ret = 0;

	if (val < 0 || val > 1)
		return -EINVAL;

	self->frame_sync_mode = val;
	return ret;
}


static int ov10640_exposure_set(struct ov10640 *self, s64 exp_us)
{
	int err = 0;
	struct camera_common_data *s_data = self->s_data;
	const struct sensor_mode_properties *mode =
			&s_data->sensor_props.sensor_modes[s_data->mode];
	int mode_ix = self->s_data->sensor_mode_id;
	int is_hdr = ov10640_formats[mode_ix].hdr_en;

	s64 line_length = mode->image_properties.line_length;
	s64 pixel_clock = mode->signal_properties.pixel_clock.val;
	s64 n_lines = pixel_clock * exp_us / line_length /
		FIXED_POINT_SCALING_FACTOR;
	++n_lines;

	dev_dbg_ratelimited(self->dev, "exposure vals min=%lld max=%lld exp_us=%lld",
		OV10640_MIN_EXPO, OV10640_MAX_EXPO, exp_us);
	dev_dbg_ratelimited(self->dev, "line_length=%lld pixel_clock=%lld n_lines=%lld",
		line_length, pixel_clock, n_lines);
	if (n_lines == 0) {
		dev_warn(self->dev, "invalid exposure!");
		return 0;
	}


	TRY(err, ov10640_group_reg_write_u16(self, OV10640_EXPO_L_H, n_lines));
	if (is_hdr) {
		TRY(err, ov10640_group_reg_write_u16(
			    self, OV10640_EXPO_S_H, n_lines));
		/* @todo check that n_lines for very short exposure is
		 * not 0. */
		/* the very short exposure register is an 8 bit entity */
		TRY(err, ov10640_group_reg_write(
			    self, OV10640_EXPO_VS_H, n_lines / 2));
	}

	return 0;
}


/**
 * v4l2 control handler
 *
 * @param ctrl the v4l2 control
 *
 * @return 0 on success
 */
static int ov10640_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov10640 *self =
		container_of(ctrl->handler, struct ov10640, ctrl_handler);
	int err = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
		self->s_data->sensor_mode_id = *ctrl->p_new.p_s64;
		return 0;
		break;
	case TEGRA_CAMERA_CID_GAIN:
		TRY(err, ov10640_gain_set(self, *ctrl->p_new.p_s64));
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		TRY(err, ov10640_framerate_set(self, *ctrl->p_new.p_s64));
		break;
	case TEGRA_CAMERA_CID_EXPOSURE:
		TRY(err, ov10640_exposure_set(self, *ctrl->p_new.p_s64));
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		dev_dbg(self->dev, "hdr enable called");
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		TRY(err, ov10640_group_hold_enable(self, ctrl->val));
		break;
	case OV10640_CID_FLIP_MIRROR:
		TRY(err, ov10640_flip_mirror_set(self, ctrl->val));
		break;
	case OV10640_CID_FSYNC_ENABLE:
		TRY(err, ov10640_frame_sync_enable_set(self, ctrl->val));
		break;
	default:
		dev_err(self->dev, "unknown ctrl id=%d", ctrl->id);
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ov10640_ctrl_ops = {
	.s_ctrl = ov10640_s_ctrl,
};

static const s64 ov10640_analog_gain_values[] = { 1, 2, 4, 8, };

static struct v4l2_ctrl_config ctrl_config_list[] = {
	{
		.ops = &ov10640_ctrl_ops,
		.id = TEGRA_CAMERA_CID_SENSOR_MODE_ID,
		.name = "Sensor Mode",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = OV10640_MODE_END - 1,
		.def = OV10640_MODE_DEFAULT,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 1LL * FIXED_POINT_SCALING_FACTOR,
		.max = 8LL * FIXED_POINT_SCALING_FACTOR,
		.def = 1LL * FIXED_POINT_SCALING_FACTOR,
		.step = 1LL * FIXED_POINT_SCALING_FACTOR / 1000,
	},
	{
		.ops = &ov10640_ctrl_ops,
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
		.ops = &ov10640_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		/* min is one line */
		.min = OV10640_MIN_EXPO * FIXED_POINT_SCALING_FACTOR / 1000000LL,
		.max = OV10640_MAX_EXPO * FIXED_POINT_SCALING_FACTOR / 1000000LL,
		.def = OV10640_DEF_EXPO * FIXED_POINT_SCALING_FACTOR / 1000000LL,
		/* step is one line at a time */
		.step = OV10640_MIN_EXPO * FIXED_POINT_SCALING_FACTOR /1000000LL,
	},
	{
		.ops = &ov10640_ctrl_ops,
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
		.ops = &ov10640_ctrl_ops,
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
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_FLIP_MIRROR,
		.name = "Flip and Mirror",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV10640_NO_FLIP_MIRROR,
		.max = OV10640_FLIP_AND_MIRROR,
		.def = OV10640_FLIP,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_FSYNC_ENABLE,
		.name = "Frame Sync Enable",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 1,
		.def = 0,
		.step = 1,
	},
};

int ov10640_ctrls_count(void)
{
	return ARRAY_SIZE(ctrl_config_list);
}

/**
 * Initializes v4l2 controls (taken from example code).
 */
int ov10640_ctrls_init(struct ov10640 *self)
{
	struct v4l2_ctrl *ctrl;
	int numctrls;
	int err;
	int i;

	dev_dbg(self->dev, "ENTER");

	self->numctrls = numctrls = ov10640_ctrls_count();
	self->ctrls = devm_kzalloc(self->dev, sizeof(struct v4l2_ctrl *) * numctrls,
				   GFP_KERNEL);

	v4l2_ctrl_handler_init(&self->ctrl_handler, numctrls);

	for (i = 0; i < numctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&self->ctrl_handler, &ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(self->dev, "Failed to init %s ctrl", ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
		    ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(self->dev, ctrl_config_list[i].max + 1,
							  GFP_KERNEL);
			if (!ctrl->p_new.p_char)
				return -ENOMEM;
		}
		self->ctrls[i] = ctrl;
	}

	self->subdev->ctrl_handler = &self->ctrl_handler;
	if (self->ctrl_handler.error) {
		dev_err(self->dev, "Error %d adding controls", self->ctrl_handler.error);
		err = self->ctrl_handler.error;
		goto error;
	}

	if ((err = v4l2_ctrl_handler_setup(&self->ctrl_handler))) {
		dev_err(self->dev, "Error %d setting default controls", err);
		goto error;
	}

	dev_dbg(self->dev, "EXIT");

	return 0;

error:
	v4l2_ctrl_handler_free(&self->ctrl_handler);
	return err;
}
