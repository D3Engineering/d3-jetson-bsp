/*
 * ar0234.h - ar0234 constants
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

#ifndef _AR0234_H
#define _AR0234_H

struct ar0234 {
	struct i2c_client *client;
	struct device *dev;
	struct v4l2_subdev *subdev;
	u32 frame_length;
	struct camera_common_data *s_data;
	struct tegracam_device *tc_dev;
	struct v4l2_ctrl_handler *custom_ctrl_handler;

	struct i2c_client *deserializer;

	unsigned framerate;
	struct reg_ctl *reg_ctl;

	bool frame_sync_enabled;
};

enum ar0234_modes {
	AR0234_MODE_1080P_30FPS = 0,
	AR0234_MODE_1080P_60FPS,
	AR0234_MODE_1200P_60FPS,
	AR0234_MODE_720P_60FPS,
	AR0234_MODE_720P_120FPS,
	AR0234_MODE_480P_120FPS,
};

/* Register definitions */
enum AR0234_REGISTERS {
	AR0234_REG_FRAME_LENGTH_LINES       = 0x300A,
	AR0234_REG_LINE_LENGTH_PCK          = 0x300C,
	AR0234_REG_RESET_REGISTER           = 0x301A,
	AR0234_REG_REVISION_NUMBER          = 0x300E,
	AR0234_REG_COARSE_INT_TIME          = 0x3012,
	AR0234_REG_FINE_INT_TIME            = 0x3014,
	AR0234_REG_GROUPED_PARAMETER_HOLD   = 0x3022,
	AR0234_REG_READ_MODE                = 0x3040,
	AR0234_REG_GLOBAL_GAIN              = 0x305E,
	AR0234_REG_DEV_CTRL                 = 0x301A,
	AR0234_REG_CUSTOMER_REV             = 0x31FE,
	AR0234_REG_ANALOG_GAIN              = 0x3060,
	AR0234_REG_RESERVED_MFR_30BA        = 0x30BA,
	AR0234_REG_GRR_CONTROL1		    = 0x30CE,
};


#endif
