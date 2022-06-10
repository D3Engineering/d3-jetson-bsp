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

#ifndef IMX490_CTRLS_H
#define IMX490_CTRLS_H

#include "imx490.h"

/**
 * enum imx490_control_ids - custom-control IDs
 * @IMX490_CID_BASE: base.  Arbitrary.
 * @IMX490_CID_SENSRATIO_OTP: set true to use the factory-set sensitivity
 *                            ratios for HDR compositing, or false to use
 *                            the host-written ratios.
 *
 * NOTE: these are NOT in the V4L2_CID_PRIVATE_BASE range.  That range
 * is no longer supported, per <https://narkive.com/8FRrkxlr.2>.
 */
enum imx490_control_ids {
	IMX490_CID_BASE		 = (TEGRA_CAMERA_CID_BASE + 250),
	IMX490_CID_SENSRATIO_OTP = (IMX490_CID_BASE + 0),
};

int imx490_ctrls_init(struct imx490 *self);

/* Controls */
int imx490_reg_read(struct imx490 *self, u16 addr, u8 *out);
int imx490_reg_read_op(struct camera_common_data *s_data, u16 addr, u8 *val);
int imx490_reg_write(struct imx490* self, u16 addr, u8 val);
int imx490_reg_write_op(struct camera_common_data* s_data, u16 addr, u8 val);

int imx490_set_gain(struct tegracam_device* tc_dev, fixedpt gain_q);
int imx490_set_exposure(struct tegracam_device* tc_dev, s64 exp_us);

int imx490_set_hflip_raw(struct imx490 *self, s32 val);
int imx490_set_vflip_raw(struct imx490 *self, s32 val);
int imx490_set_hflip(struct imx490 *self, s32 val);
int imx490_set_vflip(struct imx490 *self, s32 val);

int imx490_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
int imx490_set_group_hold(struct imx490 *self, bool val);
int imx490_set_group_hold_op(struct tegracam_device *tc_dev, bool val);
int imx490_set_mode(struct tegracam_device *tc_dev);

int imx490_set_sensratio_otp(struct imx490 *self, bool factory);
void imx490_read_sensratio(struct imx490 *self);

int imx490_is_hdr(struct imx490 *self);

#endif
