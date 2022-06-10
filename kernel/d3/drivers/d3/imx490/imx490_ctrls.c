/**
 * @author Greg Rowe <growe@d3engineering.com>
 * @author Christopher White <cwhite@d3engineering.com>
 * @author Jacob Kiggins <jkiggins@d3engineering.com>
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
#include "imx490.h"
#include "imx490_ctrls.h"
#include "imx490_modes.h"

#include <linux/bitfield.h>
#include <linux/types.h>

#include <d3/common.h>

/* Custom V4l2 Controls */
/* This is the way controls were handled in FW 1.0, it may not
   be the offical way to implement custom controls in FW 2.0,
   As far as I can tell there is no other "offical" way, and
   this works
*/

static int imx490_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops imx490_v4l2_ctrl_ops = {
	.s_ctrl = imx490_s_ctrl,
};

static struct v4l2_ctrl_config custom_ctrl_config[] = {
	{
		.ops  = &imx490_v4l2_ctrl_ops,
		.id   = IMX490_CID_SENSRATIO_OTP,
		.name = "Factory sensitivity ratios",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min  = 0,
		.max  = 1,
		.def  = 1,
		.step = 1,
	},
};
static const size_t NUM_CUSTOM_CTRLS = ARRAY_SIZE(custom_ctrl_config);

/**
 * Initializes v4l2 controls (taken from example code).
 */
int imx490_ctrls_init(struct imx490 *self)
{
	struct v4l2_ctrl *ctrl;
	int err = 0;
	int custom_index;
	(void)custom_index;
	(void)err;
	(void)ctrl;

	dev_dbg(self->dev, "%s++", __func__);

	ctrl = v4l2_ctrl_new_std(self->ctrl_handler, &imx490_v4l2_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (ctrl == NULL) {
		dev_err(self->dev, "Error initializing standard control\n");
		err = -EINVAL;
	}

	ctrl = v4l2_ctrl_new_std(self->ctrl_handler, &imx490_v4l2_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (ctrl == NULL) {
		dev_err(self->dev, "Error initializing standard control\n");
		return -EINVAL;
	}

	for (custom_index = 0; custom_index < NUM_CUSTOM_CTRLS; ++custom_index)
	{
		ctrl = v4l2_ctrl_new_custom(self->ctrl_handler, &custom_ctrl_config[custom_index], NULL);

		if (ctrl == NULL)
		{
			dev_err(self->dev, "Failed to init %s ctrl", custom_ctrl_config[custom_index].name);
			return -ENOMEM;
		}
	}
		

	if ((err = self->ctrl_handler->error)) {
		dev_err(self->dev, "Error %d adding controls", self->ctrl_handler->error);
		return err;
	}

	/* if ((err = v4l2_ctrl_handler_setup(self->ctrl_handler))) { */
	/* 	dev_err(self->dev, "Error %d setting default controls", err); */
	/* 	return err; */
	/* } */

	return 0;
}


/**
 * imx490_reg_read() - reads a single register from the sensor
 * @self: the instance
 * @addr: address to read
 * @out: where to store value
 *
 * Return: 0 on success
 */
int imx490_reg_read(struct imx490 *self, u16 addr, u8 *out)
{
	int	     err = 0;
	unsigned int val = 0;
	if ((err = regmap_read(self->map, addr, &val))) {
		dev_err(self->dev, "regmap_read of %#x returned error %d",
			(unsigned int)addr, err);
		return err;
	}
	
	dev_dbg(self->dev, "read: %#.4x=%#.2x", addr, val);
	
	if (out)
		*out = BYTE0OF(val);
	
	return 0;
}


/**
 * imx490_read_reg_op() - camera_common shim for reading registers
 * @s_data: the instance
 * @addr: address to read
 * @val: where to store value
 *
 * This works with NVIDIA's camera_common framework. The framework creates a
 * file in sysfs that you can use to read and write registers. That hooks
 * into this.  It delegates to imx490_reg_read().
 *
 * Return: 0 on success
 */
int imx490_reg_read_op(struct camera_common_data *s_data, u16 addr, u8 *val)
{
	struct imx490 *self = (struct imx490*)s_data->priv;
	return imx490_reg_read(self, addr, val);
}


/**
 * imx490_reg_write() - camera_common shim for writing registers
 * @s_data: the instance
 * @addr: address to write
 * @val: value to write
 *
 * This works with NVIDIA's camera_common framework. The framework creates a
 * file in sysfs that you can use to read and write registers. That hooks
 * into this.  It delegates to regmap_write().
 *
 * Return: 0 on success
 */
int imx490_reg_write(struct imx490* self, u16 addr, u8 val)
{
	int err = 0;
	
	if ((err = regmap_write(self->map, addr, val)))
	{
		dev_err(self->dev, "regmap_write of %u to %u returned error %d",
			(unsigned int)addr, (unsigned int)val, err);
		return err;
	}

	return 0;
}


/* static int */
/* imx490_frame_interval_set(struct imx490 *self) */
/* { */
/* 	int	 err = 0; */
/* 	unsigned fi  = self->fiv->reg_fr_time; */
/* 	dev_dbg(self->dev, "setting frame interval %u/%u", */
/* 		self->fiv->interval.numerator, self->fiv->interval.denominator); */
/* 	TRY(err, regmap_write(self->map, IMX490_REG_MODE_VMAX, fi & 0xff)); */
/* 	TRY(err, regmap_write(self->map, IMX490_REG_MODE_VMAX + 1, */
/* 			      (fi >> 8) & 0xff)); */
/* 	TRY(err, regmap_write(self->map, IMX490_REG_MODE_VMAX + 2, */
/* 			      (fi >> 16) & 0xff)); */
/* 	return 0; */
/* } */

/**
 * imx490_reg_write_op() - camera_common shim for writing registers
 * @s_data: the instance
 * @addr: address to write
 * @val: value to write
 *
 * This works with NVIDIA's camera_common framework. The framework creates a
 * file in sysfs that you can use to read and write registers. That hooks
 * into this.  It delegates to imx490_reg_write().
 *
 * Return: 0 on success
 */
int imx490_reg_write_op(struct camera_common_data* s_data, u16 addr, u8 val)
{
	struct imx490* self = (struct imx490*)s_data->priv;
	return imx490_reg_write(self, addr, val);
}


int imx490_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	dev_err(tc_dev->dev, "%s: not implemented", __func__);

	return 0;
}


/**
 * imx490_set_mode() - set the sensor mode
 * @self: instance
 * @mode: mode to change to
 *
 * CAUTION / TODO?: It may not be possible to change modes at runtime.
 * This function may need to also reset the sensor, which might require changes
 * to the way the reset pin is controlled.
 *
 * Return: 0 on success, other on failure
 */
int imx490_set_mode(struct tegracam_device *tc_dev)
{
	int err   = 0;
	struct imx490_modes_map *pmode = NULL;
	struct imx490* self = (struct imx490*)tegracam_get_privdata(tc_dev);
	int mode = self->s_data->sensor_mode_id;

	if ((mode < 0) || (mode >= IMX490_MODE_ENDMARKER)) {
		dev_err(self->dev, "invalid mode %d (must be on [0,%d])", mode,
			IMX490_MODE_ENDMARKER - 1);
		return -EINVAL;
	}
	pmode = &imx490_modes_map[mode];
	dev_info(self->dev, "setting mode: %s", pmode->desc);

	TRY(err, imx490_reset(self));

	if (!pmode->n_vals_extra || !*pmode->n_vals_extra || !pmode->vals_extra) {
		/* Linear mode: no PWL */
		TRY_DEV_HERE(err, regmap_multi_reg_write(self->map, pmode->vals, *pmode->n_vals),
			     self->dev);
	} else {
		/* PWL mode */
		dev_dbg(self->dev, "writing %zu vals from %p", *pmode->n_vals, pmode->vals);
		TRY_DEV_HERE(err, regmap_multi_reg_write(self->map, pmode->vals, *pmode->n_vals),
			     self->dev);
		dev_dbg(self->dev, "writing %zu vals_extra from %p", *pmode->n_vals_extra,
			pmode->vals_extra);
		TRY_DEV_HERE(err,
			     regmap_multi_reg_write(self->map, pmode->vals_extra,
						    *pmode->n_vals_extra),
			     self->dev);
		dev_dbg(self->dev, "done writing vals_extra");
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
int imx490_set_group_hold(struct imx490 *self, bool val)
{
	return regmap_write(self->map, IMX490_REG_REG_HOLD, val ? 1 : 0);
}

int imx490_set_group_hold_op(struct tegracam_device *tc_dev, bool val)
{
	struct imx490* self = (struct imx490*)tegracam_get_privdata(tc_dev);

	return imx490_set_group_hold(self, val);
}

/**
 * imx490_gain_raw_set() - Sets the gain using a raw register value to both SPI1H and SPI1L
 * @self: driver instance
 * @gain_0p3dB: raw register value written to SP1H and SP1L
 *
 * For use in linear modes only.  Since SP2's gain follows one of the SP1s'
 * gains, this function also sets SP2's gain implicitly.
 *
 * Return: 0 on success
 */
static int
imx490_gain_raw_set(struct imx490 *self, u16 gain_0p3dB)
{
	int err = 0;
	/* This register holds an 11 bit value */
	u16 masked = gain_0p3dB & 0x7ff;

	dev_dbg(self->dev, "gain: SP1H val=%u * 0.3 dB", masked);
	dev_dbg(self->dev, "gain: SP1L fixed at 4.2 dB");
	dev_dbg(self->dev, "gain: SP2H fixed at 29.4 dB");
	dev_dbg(self->dev, "gain: SP2L fixed at 4.2 dB");

	/* Set the analog gain registers. These are in .3 dB steps. */
	TRY(err, regmap_write(self->map, IMX490_REG_AGAIN_SP1H, BYTE0OF(masked)));
	TRY(err, regmap_write(self->map, IMX490_REG_AGAIN_SP1H + 1, BYTE1OF(masked)));

	/* SP1L 4.2 / .3 == 14 */
	masked = 14;
	TRY(err, regmap_write(self->map, IMX490_REG_AGAIN_SP1L, BYTE0OF(masked)));
	TRY(err, regmap_write(self->map, IMX490_REG_AGAIN_SP1L + 1, BYTE1OF(masked)));

	/* SP2H 29.4 / .3 == 98 */
	masked = 98;
	TRY(err, regmap_write(self->map, IMX490_REG_AGAIN_SP2H, BYTE0OF(masked)));
	TRY(err, regmap_write(self->map, IMX490_REG_AGAIN_SP2H + 1, BYTE1OF(masked)));

	/* SP2L 4.2 / .3 == 14 */
	masked = 14;
	TRY(err, regmap_write(self->map, IMX490_REG_AGAIN_SP2L, BYTE0OF(masked)));
	TRY(err, regmap_write(self->map, IMX490_REG_AGAIN_SP2L + 1, BYTE1OF(masked)));
	return 0;
}


/**
 * imx490_gain_set() - Takes fixed point (Q42.22) gain value in decibels and programs the
 * image sensor.
 * @self: driver instance
 * @gain_q: gain, in Q42.22 fixed point format.  Must be >= 0.
 *
 * @return 0 on success
 */
int imx490_set_gain(struct tegracam_device* tc_dev, s64 gain_x)
{
	struct imx490* self = (struct imx490*)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

	u64 gain_factor = mode->control_properties.gain_factor;

	/* imx490 gain is 0 to 30 in .3dB steps. */
	u16 gain_0p3dB = 0;


	gain_0p3dB = (gain_x * 10) / (3 * gain_factor);

	dev_dbg(tc_dev->dev, "gain: val=%lld reg=%u x 0.3dB", gain_x, gain_0p3dB);

	if (gain_0p3dB > 100) {
		dev_warn(self->dev, "gain out of range: reg=%#x > 100", gain_0p3dB);
		gain_0p3dB = 100;
	}

	return imx490_gain_raw_set(self, gain_0p3dB);
}


int imx490_set_hflip_raw(struct imx490 *self, s32 val)
{
	int err;
	TRY(err, regmap_update_bits(self->map, 0x74, 1 << 1, (val & 1) << 1));
	TRY(err, regmap_update_bits(self->map, 0x3C0, 1 << 3, (val & 1) << 3));
	return 0;
}


int imx490_set_vflip_raw(struct imx490 *self, s32 val)
{
	int err;
	TRY(err, regmap_update_bits(self->map, 0x74, 1 << 0, (val & 1) << 0));
	TRY(err, regmap_update_bits(self->map, 0x3C0, 1 << 2, (val & 1) << 2));
	return 0;
}


int imx490_set_hflip(struct imx490 *self, s32 val)
{
	/* TODO check the sensor first, then update the structure */
	struct camera_common_power_rail *pw = &self->power;
	self->hflip			    = val;
	if (pw->state == SWITCH_ON)
		return imx490_set_hflip_raw(self, val);
	return 0;
}


int imx490_set_vflip(struct imx490 *self, s32 val)
{
	/* TODO check the sensor first, then update the structure */
	struct camera_common_power_rail *pw = &self->power;
	self->vflip			    = val;
	if (pw->state == SWITCH_ON)
		return imx490_set_vflip_raw(self, val);
	return 0;
}


/**
 * imx490_exposure_raw_set() - Sets the exposure time using a raw register value
 * @self: driver instance
 * @exp: exposure time register value
 *
 * Return: 0 on success
 */
static int
imx490_exposure_raw_set(struct imx490 *self, u32 exp)
{
	struct reg_sequence writes[6] = { 0 };
	int		    err	      = 0;

	if (exp & ~IMX490_INTG_TIME_MASK) {
		dev_warn(self->dev, "exposure: clipping out-of-range exposure %#x", exp);
		exp &= IMX490_INTG_TIME_MASK;
	}

	/* IMX490 Register map, in INTG_TIME_SP1 description says:
	 * "Values must be in multiples of 2"
	 */
	exp &= ~1;

	dev_dbg(self->dev, "exposure: SP1 and SP2 reg=%#x", exp);

	/* 0xc, 0xd, 0xe */
	writes[0].reg = IMX490_REG_INTG_TIME_SP1;
	writes[1].reg = IMX490_REG_INTG_TIME_SP1 + 1;
	writes[2].reg = IMX490_REG_INTG_TIME_SP1 + 2;

	writes[0].def = BYTE0OF(exp);
	writes[1].def = BYTE1OF(exp);
	writes[2].def = BYTE2OF(exp);

	/* 0x10, 0x11, 0x12 */
	writes[3].reg = IMX490_REG_INTG_TIME_SP2;
	writes[4].reg = IMX490_REG_INTG_TIME_SP2 + 1;
	writes[5].reg = IMX490_REG_INTG_TIME_SP2 + 2;

	writes[3].def = BYTE0OF(exp);
	writes[4].def = BYTE1OF(exp);
	writes[5].def = BYTE2OF(exp);

	TRY(err, regmap_multi_reg_write(self->map, writes, ARRAY_SIZE(writes)));
	return 0;
}


/**
 * imx490_set_exposure - Sets the exposure time using a fixed point time setting
 * in Q42.22 format.
 * @self: driver instance
 * @exp_us: exposure time in microseconds
 *
 * Assumes that FMAX register 0x36D8 is 0.
 *
 * Return: 0 on success
 */
int imx490_set_exposure(struct tegracam_device* tc_dev, s64 exp_us)
{

	// buffers for fixed-point display
	char buf_line_time[64];
	char buf_us[64];
	char buf_lines[64];

	struct imx490* self = (struct imx490*)tegracam_get_privdata(tc_dev);

	u32 exp_lines, reg;

	int			 mode_ix = self->s_data->sensor_mode_id;
	struct imx490_modes_map *pmode	 = &imx490_modes_map[mode_ix];

	fixedpt exp_us_q    = exp_us << 22;
	fixedpt exp_lines_q = fixedpt_div(exp_us_q, pmode->exposure_line_time_us_q);
	exp_lines	    = fixedpt_toint_round(exp_lines_q);

	reg = exp_lines;

	dev_dbg(self->dev, "exposure: exp_us=%lld", exp_us);
	/* The data sheet says values of 0 and 1 are prohibited...and
	 * also says that the default value is 1... */
	if (reg < 4) {
		reg = 4;
		dev_warn(self->dev, "exposure: out of range, setting to min, %d", reg);
	} else if (reg >= IMX490_VMAX - 8) {
		reg = IMX490_VMAX - 8;
		dev_warn(self->dev, "exposure: out of range, setting to max, %d", reg);
	}

	fixedpt_str(pmode->exposure_line_time_us_q, buf_line_time, -1);
	fixedpt_str(exp_us_q, buf_us, -1);
	fixedpt_str(exp_lines_q, buf_lines, -1);

	dev_dbg(self->dev,
		"exposure: exp_us_q=%#llx = %s us; "
		"line time %s us => %s %u lines.  reg = %d %#x",
		exp_us_q, buf_us, buf_line_time, buf_lines, exp_lines, reg, reg);

	return imx490_exposure_raw_set(self, reg);
}


/**
 * imx490_set_sensratio_otp() - toggle use of factory (OTP) HDR sensitivity ratios
 * @self: instance
 * @factory: true for the OTP ratios, false for the dynamic ratios
 * Return: 0 on success
 */
int imx490_set_sensratio_otp(struct imx490 *self, bool factory)
{
	dev_warn(self->dev, "Sensitivity ratio is not tested");
	return 0;
}


/**
 * imx490_read_sensratio() - read and report sensitivity ratios
 */
void imx490_read_sensratio(struct imx490 *self)
{
	return;
}

/**
 * v4l2 control handler
 *
 * @param ctrl the v4l2 control
 *
 * @return 0 on success
 */
static int imx490_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tegracam_ctrl_handler* tc_ctrl_hdl = container_of(ctrl->handler,
								 struct tegracam_ctrl_handler,
								 ctrl_handler);
	struct tegracam_device* tc_dev = tc_ctrl_hdl->tc_dev;
	struct imx490 *self = (struct imx490*)tegracam_get_privdata(tc_dev);
	int	       err  = 0;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		TRY(err, imx490_set_hflip(self, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		TRY(err, imx490_set_vflip(self, ctrl->val));
		break;
	case IMX490_CID_SENSRATIO_OTP:
		TRY(err, imx490_set_sensratio_otp(self, !!(ctrl->val)));
		break;
	default:
		dev_err(self->dev, "%s: unknown ctrl id=%d", __func__, ctrl->id);
		return -EINVAL;
	}

	return 0;
}

int imx490_is_hdr(struct imx490 *self)
{
	int mode_ix = self->s_data->sensor_mode_id;
	return imx490_modes_formats[mode_ix].hdr_en;
}
