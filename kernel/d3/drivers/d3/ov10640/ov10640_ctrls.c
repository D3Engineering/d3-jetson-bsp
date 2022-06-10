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

#include <d3/fixedpt.h>
#include <d3/d3-jetson-bsp.h>
#include <d3/common.h>

#include "ov10640_tables.h"
#include "ov10640_ctrls.h"
#include "ov10640_reg.h"
#include "ov10640_util.h"

/* V4l2 control ids */
enum {
	OV10640_CID_BASE = (TEGRA_CAMERA_CID_BASE + 230),
	OV10640_CID_FSYNC_ENABLE,
#ifdef CONFIG_D3_OV10640_DEBUG
	OV10640_CID_TEST_PATTERN_ENABLE,
	OV10640_CID_CHANNEL_SELECT,
	OV10640_CID_NORMALIZATION,
	OV10640_CID_COMBINE_CTRL,
	OV10640_CID_COMBINE_THRESH_0,
	OV10640_CID_COMBINE_THRESH_1,
	OV10640_CID_COMBINE_THRESH_2,
	OV10640_CID_COMBINE_WEIGHT_0,
	OV10640_CID_COMBINE_WEIGHT_1,
	OV10640_CID_COMBINE_WEIGHT_2,
	OV10640_CID_COMBINE_WEIGHT_3,
	OV10640_CID_HDRMODE,
	OV10640_CID_EXPGAIN_DUMP,
#endif	/* CONFIG_D3_OV10640_DEBUG */
};


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
	dev_dbg(self->dev, "group hold: %s", !!val ? "enabled":"disabled");
	if (self->group_hold == !!val) {
		dev_dbg(self->dev, "group hold debounce -- no action required");
		return 0;
	}

	if (val) {
		regmap_write(self->map, OV10640_REG_GROUP_CTRL,
			     OV10640_REG_GROUP_CTRL_PRE_SOF | OV10640_REG_GROUP_CTRL_1ST_GRP(0));
	} else {
		regmap_write(self->map, OV10640_REG_OPERATION_CTRL,
			     OV10640_REG_OPERATION_CTRL_SINGLE_START);
	}
	self->group_hold = !!val;
	return 0;
}


static int ov10640_framerate_set(struct ov10640 *self, s64 val)
{
	return 0;
}


int ov10640_hflip_set(struct ov10640 *self, bool hflip)
{
	/* Mirror */
	int err = 0;
	u8 val = hflip ? 1:0;

	/* the third parameter is a mask */
	TRY(err, regmap_update_bits(self->map, OV10640_REG_READ_MODE,
				    (1 << 2), val << 2));
	TRY(err, regmap_update_bits(self->map, OV10640_REG_R_ISP_CTRL_2,
				    (1 << 0), val));
	TRY(err, regmap_update_bits(self->map, OV10640_REG_R_CTRL08,
				    (1 << 1), val << 1));
	return err;
}


int ov10640_vflip_set(struct ov10640 *self, bool vflip)
{
	/* Flip */
	int err = 0;
	u8 val = vflip ? 1:0;

	/* the third parameter is a mask */
	TRY(err, regmap_update_bits(self->map, OV10640_REG_READ_MODE,
				    (1 << 3), val << 3));
	TRY(err, regmap_update_bits(self->map, OV10640_REG_R_ISP_CTRL_2,
				    (1 << 1), val << 1));
	TRY(err, regmap_update_bits(self->map, OV10640_REG_R_CTRL08,
				    (1 << 2), val << 2));
	return err;
}


static int ov10640_frame_sync_enable_set(struct ov10640 *self, s64 val)
{
	int ret = 0;

	if (val < 0 || val > 1)
		return -EINVAL;

	self->frame_sync_mode = val;
	return ret;
}


static int ov10640_single_bit_set(struct ov10640 *self,
				  const int reg_addr,
				  const int bit,
				  bool is_set)
{
	int err = 0;
	unsigned int reg_val = 0;

	TRY(err, regmap_read(self->map, reg_addr, &reg_val));

	if(is_set) {
		reg_val |= (1 << bit);
	}
	else {
		reg_val &= ~(1 << bit);
	}

	TRY(err, regmap_write(self->map, reg_addr, reg_val));
	return 0;

}


int ov10640_test_pattern_set(struct ov10640 *self, bool is_enabled)
{
	int err = 0;

	dev_dbg(self->dev, "%s", is_enabled ? "set":"not set");
	TRY(err, ov10640_single_bit_set(self, OV10640_REG_ASP_REGA,
					ASP_REGA_ATP_STATUS, is_enabled));
	return 0;
}


#ifdef CONFIG_D3_OV10640_DEBUG
static int ov10640_test_pattern_enable_set(struct ov10640 *self, s64 val)
{
	ov10640_test_pattern_set(self, self->test_pattern_is_enabled);
	self->test_pattern_is_enabled = !!val;
	return 0;
}
#endif /* CONFIG_D3_OV10640_DEBUG */


static int ov10640_exposure_long_set(struct ov10640 *self, u16 n_lines)
{
	int err = 0;
	TRY(err, ov10640_group_reg_write_u16(self, OV10640_EXPO_L_L, n_lines));
	dev_dbg(self->dev, "exposure L=%u lines (%#x=%#x)",
		n_lines, OV10640_EXPO_L_L, n_lines);
	return 0;
}


static int ov10640_exposure_short_set(struct ov10640 *self, u16 n_lines)
{
	int err = 0;
	TRY(err, ov10640_group_reg_write_u16(self, OV10640_EXPO_S_L, n_lines));
	dev_dbg(self->dev, "exposure S=%u lines (%#x=%#x)",
		n_lines, OV10640_EXPO_S_L, n_lines);
	return 0;
}


static int ov10640_exposure_vshort_set(struct ov10640 *self, u16 n_32nd_line)
{
	int err = 0;
	TRY(err, ov10640_group_reg_write(self, OV10640_EXPO_VS, n_32nd_line));
	dev_dbg(self->dev, "exposure VS=%u 1/32s lines (%#x=%#x))",
		n_32nd_line, OV10640_EXPO_VS, n_32nd_line);
	return 0;
}


static bool ov10640_expgain_is_ready(struct ov10640 *self,
					   s64 gain_fixed_pt,
					   u16 n_lines)
{
	if (gain_fixed_pt) {
		self->gain_fixed_pt = gain_fixed_pt;
	}

	if (n_lines) {
		self->n_lines = n_lines;
	}

	return (self->n_lines && self->gain_fixed_pt);
}


static int ov10640_expgain_gain_analog_set(
	struct ov10640 *self,
	const struct ae_calculation_output *ae)
{
	int err = 0;
	u16 raw_gain = 0;

	u8 again_l = ov10640_again_to_exponent(self, ae->analogGain);
	u8 again_s = ov10640_again_to_exponent(self, ae->analogGainS);
	u8 again_vs = ov10640_again_to_exponent(self, ae->analogGainVS);

	/* There is no conversion gain mode for S, only Long and Very Short */
	raw_gain =((ae->convSensorGainFlagVS & 0x01) << 7)
		| ((ae->convSensorGainFlag & 0x1) << 6)
		| ((again_vs & 0x03) << 4)
		| ((again_s & 0x03) << 2)
		| (again_l & 0x3);
	dev_dbg(self->dev, "analog gain CG_AGAIN=%#x", raw_gain);
	TRY(err, ov10640_group_reg_write(self, OV10640_CG_AGAIN, raw_gain));
	return 0;
}


#ifdef CONFIG_D3_OV10640_DEBUG
static u32 ov10640_register_to_digital_gain(u16 reg)
{
	u8 integer = (reg & 0x3f00) >> 8;
	u8 fract = reg & 0x00ff;
	u32 val = ((u32)integer * 1000) + ((u32)fract * 1000 / (u32)256);
	return val;
}
#endif /* CONFIG_D3_OV10640_DEBUG */

static int ov10640_expgain_gain_digital_set(
	struct ov10640 *self,
	const struct ae_calculation_output *ae)
{
	int err = 0;
	u16 l = ov10640_digital_gain_to_register(ae->sensorDGain);
	u16 s = ov10640_digital_gain_to_register(ae->sensorDGainS);
	u16 vs = ov10640_digital_gain_to_register(ae->sensorDGainVS);


	dev_dbg(self->dev, "digital gain L %#x=%#x S %#x=%#x VS %#x=%#x",
		OV10640_DIG_L_GAIN_H, l,
		OV10640_DIG_S_GAIN_H, s,
		OV10640_DIG_VS_GAIN_H, vs);

	TRY(err, ov10640_group_reg_write_u16(self, OV10640_DIG_L_GAIN_H, l));
	TRY(err, ov10640_group_reg_write_u16(self, OV10640_DIG_S_GAIN_H, s));
	TRY(err, ov10640_group_reg_write_u16(self, OV10640_DIG_VS_GAIN_H, vs));
	return 0;
}


static int ov10640_expgain_gain_set(struct ov10640 *self,
				    const struct ae_calculation_output *ae)
{
	int err = 0;
	TRY(err, ov10640_expgain_gain_analog_set(self, ae));
	TRY(err, ov10640_expgain_gain_digital_set(self, ae));
	return 0;
}


static int ov10640_expgain_exposure_set(struct ov10640 *self,
					const struct ae_calculation_output *ae)
{
	int err = 0;
	TRY(err,
	    ov10640_exposure_long_set(self, (ae->exposureTime & 0xffff) / 32));
	TRY(err,
	    ov10640_exposure_short_set(self, (ae->exposureTimeS & 0xffff) / 32));
	/* Very Short exposure is programmed as 32nds of a line so no
	 * division is required. */
	TRY(err,
	    ov10640_exposure_vshort_set(self, (ae->exposureTimeVS & 0xffff)));
	return 0;
}


static int ov10640_expgain_params_set(struct ov10640 *self,
				      const struct ae_calculation_output *ae)
{
	int err = 0;
	TRY(err, ov10640_expgain_gain_set(self, ae));
	TRY(err, ov10640_expgain_exposure_set(self, ae));
	return 0;
}


#ifdef CONFIG_D3_OV10640_DEBUG
static void ov10640_expgain_params_dump(struct ov10640 *self,
					const struct ae_calculation_output *ae)
{
	dev_dbg(self->dev, "L.nExposureTime: %u (lines)", ae->exposureTime / 32);
	dev_dbg(self->dev, "L.nAnalogGain: %u", ae->analogGain);
	dev_dbg(self->dev, "L.bConvGain: %u", ae->convSensorGainFlag);
	dev_dbg(self->dev, "L.nDigitalGain: %u (x1000)", ae->sensorDGain);

	dev_dbg(self->dev, "S.nExposureTime: %u (lines)", ae->exposureTimeS / 32);
	dev_dbg(self->dev, "S.nAnalogGain: %u", ae->analogGainS);
	dev_dbg(self->dev, "S.bConvGain: FALSE");
	dev_dbg(self->dev, "S.nDigitalGain: %u (x1000)", ae->sensorDGainS);

	dev_dbg(self->dev, "VS.nExposureTime: %u (lines *32)", ae->exposureTimeVS);
	dev_dbg(self->dev, "VS.nAnalogGain: %u", ae->analogGainVS);
	dev_dbg(self->dev, "VS.bConvGain: %u", ae->convSensorGainFlagVS);
	dev_dbg(self->dev, "VS.nDigitalGain: %u (x1000)", ae->sensorDGainVS);

	dev_dbg(self->dev, "L2S ratio: %u", ae->long2Short_ratio / 256);
	dev_dbg(self->dev, "L2VSratio: %u", ae->long2Vshort_ratio / 256);
}
#endif /* CONFIG_D3_OV10640_DEBUG */


#ifdef CONFIG_D3_OV10640_DEBUG
static int ov10640_expgain_debug_read(struct ov10640 *self,
				      const u16 addr,
				      u8 *out_regs,
				      size_t regs_len)
{
	int err = 0;
	dev_dbg(self->dev, "reading %#x to %#zx", addr, addr + regs_len);
	err = regmap_bulk_read(self->map, addr, out_regs, regs_len);
	if (err) {
		dev_warn(self->dev, "could not read expgain debug registers!");
		return err;
	}
	return 0;
}
#endif /* CONFIG_D3_OV10640_DEBUG */

#ifdef CONFIG_D3_OV10640_DEBUG
static void ov10640_expgain_debug_dump_raw(struct ov10640 *self,
					   u16 addr,
					   const u8 *out_regs,
					   size_t regs_len)
{
	size_t i;

	for (i=0; i<regs_len; ++i) {
		dev_dbg(self->dev, "expgain: %#4x=%#2x", addr, out_regs[i]);
		++addr;
	}
}
#endif /* CONFIG_D3_OV10640_DEBUG */


#ifdef CONFIG_D3_OV10640_DEBUG
static void ov10640_expgain_ae_exp_from_raw(struct ov10640 *self,
					    struct ae_calculation_output *out_ae,
					    const u8 *reg)
{
	out_ae->exposureTime = (reg[0] << 8) | reg[1];
	reg += 2;
	out_ae->exposureTime *= 32;

	out_ae->exposureTimeS = (reg[0] << 8) | reg[1];
	reg += 2;
	out_ae->exposureTimeS *= 32;

	/* VS is not divided by 32 */
	out_ae->exposureTimeVS = reg[0];
	++reg;
}
#endif /* CONFIG_D3_OV10640_DEBUG */


#ifdef CONFIG_D3_OV10640_DEBUG
static void ov10640_expgain_ae_again_from_raw(
	struct ov10640 *self,
	struct ae_calculation_output *out_ae,
	const u8 cg_again)
{
	u8 again_l = 0;
	u8 again_s = 0;
	u8 again_vs = 0;
	/* raw_gain =((ae->convSensorGainFlagVS & 0x01) << 7) */
	/* | ((ae->convSensorGainFlag & 0x1) << 6) */
	/* | ((again_vs & 0x03) << 4) */
	/* | ((again_s & 0x03) << 2) */
	/* | (again_l & 0x3); */
	out_ae->convSensorGainFlagVS = (cg_again & (1 << 7)) >> 7;
	out_ae->convSensorGainFlag = (cg_again & (1 << 6)) >> 6;
	out_ae->convSensorGainFlagS = 0;
	again_l = cg_again & 0x03;
	again_s = (cg_again & (0x03 << 2)) >> 2;
	again_vs = (cg_again & (0x03 << 4)) >> 4;
	out_ae->analogGain = 1 << again_l;
	out_ae->analogGainS = 1 << again_s;
	out_ae->analogGainVS = 1 << again_vs;
}
#endif /* CONFIG_D3_OV10640_DEBUG */


#ifdef CONFIG_D3_OV10640_DEBUG
static void ov10640_expgain_ae_dgain_from_raw(
	struct ov10640 *self,
	struct ae_calculation_output *out_ae,
	const u8 *reg)
{
	out_ae->sensorDGain =
		ov10640_register_to_digital_gain((reg[0] << 8) | reg[1]);
	out_ae->sensorDGainS =
		ov10640_register_to_digital_gain((reg[2] << 8) | reg[3]);
	out_ae->sensorDGainVS =
		ov10640_register_to_digital_gain((reg[4] << 8) | reg[5]);
}
#endif /* CONFIG_D3_OV10640_DEBUG */


#ifdef CONFIG_D3_OV10640_DEBUG
static void ov10640_expgain_ae_from_raw(struct ov10640 *self,
					struct ae_calculation_output *out_ae,
					const u8 *reg)
{
	/*
	 * 0x30e6 to 0x30ea (5 registers)
	 * e6, e7 long exposure time
	 * e8, e9 short exposure time
	 * ea vshort exposure time
	 */
	ov10640_expgain_ae_exp_from_raw(self, out_ae, reg);
	/* 0x30eb, cg_again is one register. */
	ov10640_expgain_ae_again_from_raw(self, out_ae, reg[5]);
	/*
	 * 0x30ec to 0x30f1 (6 registers)
	 * ec, ed long digital gain
	 * ee, ef short digital gain
	 * f0, f1 vshort digital gain
	 *
	 */
	ov10640_expgain_ae_dgain_from_raw(self, out_ae, &reg[6]);
}
#endif /* CONFIG_D3_OV10640_DEBUG */



#ifdef CONFIG_D3_OV10640_DEBUG
static void ov10640_expgain_params_debug(struct ov10640 *self)
{
	/* #define OV10640_EXPO_L_L			0x30E6 */
	/* ... */
	/* #define OV10640_DIG_VS_GAIN_L			0x30F1 */
	/* 0x30e6 to 0x30f1 */
	u8 regs[12];
	size_t reg_len = ARRAY_SIZE(regs);
	const u16 addr_start = 0x30e6;
	struct ae_calculation_output ae_new;

	/* read back the registers we want */
	ov10640_expgain_debug_read(self, addr_start, regs, reg_len);
	/* dump the raw register addresses and values */
	ov10640_expgain_debug_dump_raw(self, addr_start, regs, reg_len);

	/* compute back the expgain values */
	memset(&ae_new, 0, sizeof(ae_new));
	ov10640_expgain_ae_from_raw(self, &ae_new, regs);
	/* dump expgain */
	ov10640_expgain_params_dump(self, &ae_new);
}
#endif /* CONFIG_D3_OV10640_DEBUG */


static int ov10640_expgain_set(struct ov10640 *self)
{
	char fp_str[64];
	int err;
	struct ae_calculation_output ae;

	fixedpt_str(self->gain_fixed_pt, fp_str, -1);

	memset(&ae, 0, sizeof(ae));
	ae.shutter = self->n_lines * OV10640_MIN_EXPO;
	ae.again = self->gain_fixed_pt * 1000  / FIXED_POINT_SCALING_FACTOR;

	dev_dbg(self->dev,
		"requested: gain %s, again %u"
		", exposure %llu us (%u lines)"
		", shutter %u",
		fp_str, ae.again,
		self->n_lines * OV10640_MIN_EXPO, self->n_lines,
		ae.shutter);

	/* Compute the optimal gain and exposure parameters for the
	 * gain * integration time product */
	Set_HDR_ExpGain_Params(&ae);
#ifdef CONFIG_D3_OV10640_DEBUG
	ov10640_expgain_params_dump(self, &ae);
#endif /* CONFIG_D3_OV10640_DEBUG */
	TRY(err, ov10640_expgain_params_set(self, &ae));
	return 0;
}


static int ov10640_expgain_lines_set(struct ov10640 *self,
					   s64 gain_fixed_pt,
					   u16 n_lines)
{
	int err = 0;
	if (!ov10640_expgain_is_ready(self, gain_fixed_pt, n_lines)) {
		dev_dbg(self->dev, "not ready to set gain/exposure");
		return 0;
	}

	TRY(err, ov10640_expgain_set(self));
	return 0;
}


static int ov10640_gain_set(struct ov10640 *self, s64 val)
{
	return ov10640_expgain_lines_set(self, val, 0);
}


static int ov10640_exposure_lines_set(struct ov10640 *self, u16 n_lines)
{
	return ov10640_expgain_lines_set(self, 0, n_lines);
}


static int ov10640_exposure_set(struct ov10640 *self, s64 exp_us)
{
	char fp_str[64];
	int err = 0;
	struct camera_common_data *s_data = self->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];

	unsigned int vts_l = 0;
	unsigned int vts_h = 0;
	u16 vts = 0;
	u16 max_lines = 0;

	s64 line_length = mode->image_properties.line_length;
	s64 pixel_clock = mode->signal_properties.pixel_clock.val;
	/* We might not want to do this, at least for HDR. I think
	 * we're losing precision here. We likely should hold off in
	 * converting to lines later in the compution. */
	s64 n_lines = pixel_clock * exp_us / line_length /
		FIXED_POINT_SCALING_FACTOR;
	++n_lines;
	if (n_lines <= 0) {
		dev_warn(self->dev, "n_lines computation wrong! computed %lld",
			 n_lines);
		n_lines = 1;
	}

	fixedpt_str(exp_us, fp_str, -1);
	dev_dbg(self->dev, "requested exposure time %s", fp_str);

	/* The datasheet says that the max exposure for L and S
	 * exposures is VTS - 6 */

	TRY(err, regmap_read(self->map, OV10640_REG_VTS_H, &vts_h));
	TRY(err, regmap_read(self->map, OV10640_REG_VTS_L, &vts_l));
	vts = (vts_h << 8) | vts_l;
	max_lines = vts - 6;
	dev_dbg_ratelimited(self->dev, "vts = %u, max_lines=%u", vts, max_lines);

	dev_dbg_ratelimited(self->dev,
			    "exposure vals min=%lld max=%lld exp_us=%lld",
			    OV10640_MIN_EXPO, OV10640_MAX_EXPO, exp_us);
	dev_dbg_ratelimited(self->dev,
			    "line_length=%lld pixel_clock=%lld n_lines=%lld",
			    line_length, pixel_clock, n_lines);

	if (n_lines > max_lines) {
		dev_warn(self->dev, "invalid exposure: n_lines=%lld > max=%u",
			 n_lines, max_lines);
		n_lines = max_lines;
	}

	ov10640_exposure_lines_set(self, n_lines);
	return 0;
}

#ifdef CONFIG_D3_OV10640_DEBUG
static void ov10640_combine_thresh_set(struct ov10640 *self,
				       size_t thresh_which,
				       u8 val)
{
	dev_dbg(self->dev, "setting combine thresh[%zu]=%#x", thresh_which, val);
	self->combine.thresh[thresh_which] = val;
	ov10640_combine_dump(self, &self->combine);
}
#endif /* CONFIG_D3_OV10640_DEBUG */


#ifdef CONFIG_D3_OV10640_DEBUG
static void ov10640_combine_weight_set(struct ov10640 *self,
				       size_t weight_which,
				       u32 val)
{
	dev_dbg(self->dev, "setting combine weight[%zu]=%#x", weight_which, val);
	self->combine.weight[weight_which] = val;
	ov10640_combine_dump(self, &self->combine);
}
#endif /* CONFIG_D3_OV10640_DEBUG */


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
	case V4L2_CID_HFLIP:
		dev_dbg(self->dev, "hflip");
		self->hflip = ctrl->val;
		TRY(err, ov10640_hflip_set(self, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(self->dev, "vflip");
		self->vflip = ctrl->val;
		TRY(err, ov10640_vflip_set(self, ctrl->val));
		break;
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
		self->s_data->sensor_mode_id = *ctrl->p_new.p_s64;
		err = regmap_multi_reg_write(
			self->map,
			mode_table[self->s_data->sensor_mode_id].reg_sequence,
			mode_table[self->s_data->sensor_mode_id].size);
		dev_dbg(self->dev, "changed sensor mode id to %u",
			self->s_data->sensor_mode_id);
		return err;
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
	case TEGRA_CAMERA_CID_EXPOSURE_SHORT:
		dev_dbg(self->dev, "short exposure ctrl requested: %lld",
			*ctrl->p_new.p_s64);
		dev_dbg(self->dev,
			"short exposure is controlled in main exposure control");
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		dev_dbg(self->dev, "hdr enable called");
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		TRY(err, ov10640_group_hold_enable(self, ctrl->val));
		break;
	case OV10640_CID_FSYNC_ENABLE:
		TRY(err, ov10640_frame_sync_enable_set(self, ctrl->val));
		break;
#ifdef CONFIG_D3_OV10640_DEBUG
	case OV10640_CID_TEST_PATTERN_ENABLE:
		TRY(err, ov10640_test_pattern_enable_set(self, ctrl->val));
		break;
	case OV10640_CID_CHANNEL_SELECT:
		self->channel_select = ctrl->val;
		ov10640_channel_set(self, self->channel_select);
		break;
	case OV10640_CID_NORMALIZATION:
		self->normalization_type = ctrl->val;
		dev_dbg(self->dev,
			"normalization set to %d for next stream",
			self->normalization_type);
		break;
	case OV10640_CID_COMBINE_CTRL:
		self->combine.ctrl = ctrl->val;
		ov10640_combine_dump(self, &self->combine);
		break;
	case OV10640_CID_COMBINE_THRESH_0:
		ov10640_combine_thresh_set(self, 0, ctrl->val);
		break;
	case OV10640_CID_COMBINE_THRESH_1:
		ov10640_combine_thresh_set(self, 1, ctrl->val);
		break;
	case OV10640_CID_COMBINE_THRESH_2:
		ov10640_combine_thresh_set(self, 2, ctrl->val);
		break;
	case OV10640_CID_COMBINE_WEIGHT_0:
		ov10640_combine_weight_set(self, 0, ctrl->val);
		break;
	case OV10640_CID_COMBINE_WEIGHT_1:
		ov10640_combine_weight_set(self, 1, ctrl->val);
		break;
	case OV10640_CID_COMBINE_WEIGHT_2:
		ov10640_combine_weight_set(self, 2, ctrl->val);
		break;
	case OV10640_CID_COMBINE_WEIGHT_3:
		ov10640_combine_weight_set(self, 3, ctrl->val);
		break;
	case OV10640_CID_HDRMODE:
		self->hdr_mode = ctrl->val;
		dev_dbg(self->dev, "setting HDR mode to: %u for next stream",
			self->hdr_mode);
		break;
	case OV10640_CID_EXPGAIN_DUMP:
		ov10640_expgain_params_debug(self);
		break;
#endif /* CONFIG_D3_OV10640_DEBUG */
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
		.max = OV10640_MODE_ENDMARKER - 1,
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
		/* The Short Exposure control is required but it is a
		 * no-op. The exposure is controlled, even in HDR
		 * mode, exclusively by the Exposure control. */
		.ops = &ov10640_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EXPOSURE_SHORT,
		.name = "Short Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 1 * FIXED_POINT_SCALING_FACTOR / 1000000LL,
		.max = 1 * FIXED_POINT_SCALING_FACTOR / 1000000LL,
		.def = 1 * FIXED_POINT_SCALING_FACTOR / 1000000LL,
		.step = 1 * FIXED_POINT_SCALING_FACTOR /1000000LL,
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
		.id = OV10640_CID_FSYNC_ENABLE,
		.name = "Frame Sync Enable",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 1,
		.def = 0,
		.step = 1,
	},
#ifdef CONFIG_D3_OV10640_DEBUG
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_TEST_PATTERN_ENABLE,
		.name = "Test Pattern Enable",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 1,
		.def = 0,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_CHANNEL_SELECT,
		.name = "Channel Select",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV10640_CHANNEL_PWL,
		.max = OV10640_CHANNEL_ENDMARKER -1,
		.def = OV10640_CHANNEL_LONG,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_NORMALIZATION,
		.name = "Normalization",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV10640_NORMALIZATION_10,
		.max = OV10640_NORMALIZATION_ENDMARKER,
		.def = OV10640_NORMALIZATION_12,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_COMBINE_CTRL,
		.name = "combine_ctrl",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 255,
		.def = 0x3c,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_COMBINE_THRESH_0,
		.name = "combine_thresh_0",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 255,
		.def = 0xb7,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_COMBINE_THRESH_1,
		.name = "combine_thresh_1",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 255,
		.def = 0xca,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_COMBINE_THRESH_2,
		.name = "combine_thresh_2",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 255,
		.def = 0xcc,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_COMBINE_WEIGHT_0,
		.name = "combine_weight_0",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 0xffffffff,
		.def = 0x80806040,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_COMBINE_WEIGHT_1,
		.name = "combine_weight_1",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 0xffffffff,
		.def = 0x80804020,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_COMBINE_WEIGHT_2,
		.name = "combine_weight_2",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 0xffffffff,
		.def = 0x80600000,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_COMBINE_WEIGHT_3,
		.name = "combine_weight_3",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 0xffffffff,
		.def = 0x80800000,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_HDRMODE,
		.name = "HDR Mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV10640_HDRMODE_3EXPOSURE,
		.max = OV10640_HDRMODE_ENDMARKER -1,
		.def = OV10640_HDRMODE_3EXPOSURE,
		.step = 1,
	},
	{
		.ops = &ov10640_ctrl_ops,
		.id = OV10640_CID_EXPGAIN_DUMP,
		.name = "Expgain Dump",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 1,
		.def = 0,
		.step = 1,
	},
#endif /* CONFIG_D3_OV10640_DEBUG */

};

#define OV10640_NUM_CUSTOM_CONTROLS	ARRAY_SIZE(ctrl_config_list)
#define OV10640_NUM_STD_CONTROLS	(2)
#define OV10640_NUM_CONTROLS		(OV10640_NUM_CUSTOM_CONTROLS + OV10640_NUM_STD_CONTROLS)

int ov10640_ctrls_count(void)
{
	return OV10640_NUM_CONTROLS;
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
	int ctrl_index = 0;

	dev_dbg(self->dev, "ENTER");

	self->numctrls = numctrls = ov10640_ctrls_count();
	self->ctrls = devm_kzalloc(self->dev, sizeof(struct v4l2_ctrl *) * numctrls,
				   GFP_KERNEL);

	v4l2_ctrl_handler_init(&self->ctrl_handler, numctrls);

	ctrl = v4l2_ctrl_new_std(&self->ctrl_handler, &ov10640_ctrl_ops,
				 V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (ctrl == NULL) {
		dev_err(self->dev, "Error initializing standard control\n");
		err = -EINVAL;
		goto error;
	}
	self->ctrls[ctrl_index++] = ctrl;

	ctrl = v4l2_ctrl_new_std(&self->ctrl_handler, &ov10640_ctrl_ops,
				 V4L2_CID_VFLIP, 0, 1, 1, 1);
	if (ctrl == NULL) {
		dev_err(self->dev, "Error initializing standard control\n");
		err = -EINVAL;
		goto error;
	}
	self->ctrls[ctrl_index++] = ctrl;


	for (i = 0; i < OV10640_NUM_CUSTOM_CONTROLS; i++) {
		ctrl = v4l2_ctrl_new_custom(&self->ctrl_handler, &ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(self->dev, "Failed to init %s ctrl",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
		    ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(self->dev, ctrl_config_list[i].max + 1,
							  GFP_KERNEL);
			if (!ctrl->p_new.p_char)
				return -ENOMEM;
		}
		self->ctrls[ctrl_index++] = ctrl;
	}
	BUG_ON(ctrl_index != OV10640_NUM_CONTROLS);

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

	dev_dbg(self->dev, "probe exit success");
	return 0;

error:
	v4l2_ctrl_handler_free(&self->ctrl_handler);
	return err;
}
