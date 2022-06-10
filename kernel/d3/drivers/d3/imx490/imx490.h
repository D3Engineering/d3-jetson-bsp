/**
 * @author Greg Rowe <growe@d3engineering.com>
 * @author Christopher White <cwhite@d3engineering.com>
 * @author Jacob Kiggins <jkiggins@d3engineering.com>
 *
 * imx490 v4l2 driver for Nvidia Jetson
 *
 * Copyright (c) 2018--2021, D3 Engineering.  All rights reserved.
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

#ifndef IMX490_H
#define IMX490_H

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
#include <d3/fixedpt.h>


/**
 * BYTENOF - return the nth byte of x
 *
 * n must be a compile-time constant.
 */
#define BYTENOF(n, x)   FIELD_GET(GENMASK((n)*8+7, (n)*8), (x))


/* Convenience invokers for the first four bytes */
#define BYTE0OF(x)      BYTENOF(0, x)
#define BYTE1OF(x)      BYTENOF(1, x)
#define BYTE2OF(x)      BYTENOF(2, x)
#define BYTE3OF(x)      BYTENOF(3, x)


/**
 * enum imx490_reg_addrs - Register addresses (see data sheet/register map)
 */
enum imx490_reg_addrs {
	// updated for imx490
	IMX490_REG_LOWPOWER  = 0x3C00,
	IMX490_REG_REG_HOLD = 0x3848,

	IMX490_REG_INTG_TIME_SP1       = 0x36C0,
	IMX490_REG_INTG_TIME_SP2       = 0x36C4,

	IMX490_REG_AGAIN_SP1H	       = 0x36C8,
	IMX490_REG_AGAIN_SP1L	       = 0x36CA,
	IMX490_REG_AGAIN_SP2H	       = 0x36CC,
	IMX490_REG_AGAIN_SP2L	       = 0x36CE,

	IMX490_REG_OBB_CLAMP_CTRL_SEL  = 0x0083,
	IMX490_REG_WDC_OUTSEL	       = 0x00f9,
	IMX490_REG_WDC_THR_FRM_SEL     = 0x00fa,
	IMX490_REG_MODE_VMAX	       = 0x0000,
	IMX490_REG_MODE_HMAX	       = 0x0004,
	IMX490_REG_AG_3ADSP1_SEL       = 0x23c0,

	// Updated for imx490
	IMX490_REG_FRONT_EMBDATA_LINE = 0x020C,
	IMX490_REG_REAR_EMBDATA_LINE   = 0x020D,

	// I don't think this functionality exists in the imx490
	IMX490_REG_OTP_SENSRATIO_FIRST = 0x3050,
	IMX490_REG_OTP_SENSRATIO       = 0x3053,

	IMX490_REG_CHIP_ID_4           = 0x7663,

	// updated for imx490
	IMX490_REG_PROCSTS	       = 0x7670,
};


#define IMX490_INTG_TIME_MASK GENMASK(17, 0)

/* VMAX, is 2000 */
#define IMX490_VMAX (0x07d0)

/* values of IMX490_REG_STANDBY */
// Updated for imx490
#define IMX490_LOWPOWER_ENTER (0xFF)
#define IMX490_STREAM_ENTER_ONE (0x5C)
#define IMX490_STREAM_ENTER_TWO (0xA3)

// The rest of these are hold-overs from the imx390
/* values of IMX490_REG_WDC_OUTSEL */
#define IMX490_WDC_OUTSEL_LINEAR (0x00)
#define IMX490_WDC_OUTSEL_HDR (0x01)

/* values of IMX490_REG_WDC_THR_FRM_SEL */
#define IMX490_WDC_THR_FRM_SEL_SP1H (0x00)
#define IMX490_WDC_THR_FRM_SEL_SP1L (0x01)
#define IMX490_WDC_THR_FRM_SEL_SP2 (0x02)

/* bits in IMX490_REG_AG_3ADSP1_SEL */
#define IMX490_AG_3ADSP1_SEL_SP2_GAIN_MASK BIT(2)
#define IMX490_AG_3ADSP1_SEL_SP2_GAIN_FROM_SP1H (0)
#define IMX490_AG_3ADSP1_SEL_SP2_GAIN_FROM_SP1L (1)

#define IMX490_AG_3ADSP1_SEL_SP2_FSYNCPOL_MASK GENMASK(19, 18)
#define IMX490_AG_3ADSP1_SEL_SP2_FSYNCPOL_ACTIVE_HIGH (0)
#define IMX490_AG_3ADSP1_SEL_SP2_FSYNCPOL_ACTIVE_LOW (1)

/* Bits in IMX490_REG_OTP_SENSRATIO */
#define IMX490_OTP_SENSRATIO_EN BIT(7)
#define IMX490_OTP_SENSRATIO_EN_FACTORY (1)
#define IMX490_OTP_SENSRATIO_EN_DYNAMIC (0)


/**
 * enum imx490_procsts - device status values
 *
 * Values of %IMX490_REG_DEVSTS
 */
enum imx490_procsts {
	IMX490_PROCSTS_LOWPOWER = 0,
	IMX490_PROCSTS_STREAMING,
	IMX490_PROCSTS_TEMP_MEASURE,
	IMX490_NUM_PROCSTS
};

extern const u8 imx490_procsts_reg_map[];


struct imx490_interval {
	u32		  reg_fr_time;
	struct v4l2_fract interval;
	/* Probably, I don't need to store the size */
	struct v4l2_frmsize_discrete size; /* Maximum rectangle for the interval
					    */
};

enum imx490_rev_t {
	IMX490_REV_UNKNOWN = 0,
	IMX490_REV_ES2,
	IMX490_REV_ES4,
};

/**
 * struct imx490 - IMX490 driver class.
 *
 * Some of the pointers are not
 * strictly needed as they can be obtained via things like
 * 'client'. These pointers exist for convenience.
 */
struct imx490 {
	struct camera_common_power_rail power;

	struct i2c_client *client;
	struct device *	   dev;
	struct regmap *	   map;

	struct camera_common_data * s_data;
	/* struct camera_common_pdata *pdata; */
	bool			    hflip;
	bool			    vflip;

	u32 frame_length;
	s64 last_wdr_et_val;

	unsigned int reset_gpio;

	struct v4l2_ctrl_handler* ctrl_handler;
	struct v4l2_subdev *	 subdev;
	struct i2c_client *	 deserializer;
	const struct imx490_interval *fiv;
	struct tegracam_device* tc_dev;

	/**
	 * @silent_read: when true, imx490_reg_read() doesn't warn about errors
	 */
	bool silent_read;

	enum imx490_rev_t rev;

	struct v4l2_ctrl *ctrls[];
};

/**
 * enum imx490_mode - sensor modes
 * @IMX490_MODE_LINEAR
 */
enum imx490_mode {
	IMX490_MODE_HDR,
	IMX490_MODE_LINEAR,

	IMX490_MODE_ENDMARKER,
	IMX490_MODE_DEFAULT = IMX490_MODE_HDR,
};

/**
 * struct imx490_modes_map - definition of a mode's sensor settings
 * @desc: human-readable description
 * @n_vals: (required) number of register writes to set this mode
 * @vals: (required) register writes
 * @n_vals_extra: (optional) number of additional register writes to set this mode
 * @vals_extra: (required) additional register writes
 * @exposure_line_time_us_q: exposure time from the datasheet, per line, in
 *                           Q42.22 fixed point.
 *
 * If @n_vals_extra is nonzero, both @vals and @vals_extra are required to set
 * the mode.  For example, @vals_extra can be used for a PWL curve in an
 * HDR mode.
 */
struct imx490_modes_map {
	const char *		   desc;
	const size_t *		   n_vals;
	const struct reg_sequence *vals;
	const size_t *n_vals_extra;
	const struct reg_sequence *vals_extra;
	const u64 exposure_line_time_us_q;
};

/* Values in usec in Q42.22.  Used for exposure time per line.  From
 * Tables 20-1 and 20-2, Chapter 20.1 List of operating modes, IMX490 datasheet.
 */

#define EXP_PER_LINE_30FPS (70044876ULL)	/* 16.7 us */
#define EXP_PER_LINE_40FPS (52428800ULL)	/* 12.5 us */


/**
 * If a reset GPIO is present this will be toggled. Then we wait for
 * the image sensor to be ready.
 */
int imx490_reset(struct imx490 *self);

#endif
