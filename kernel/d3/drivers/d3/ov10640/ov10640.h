/**
 * @author Josh Watts <jwatts@d3engineering.com>
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
#ifndef _OV10640_H
#define _OV10640_H

#include <linux/i2c.h>

/**
 * If @p expr evalutes to non-zero assign it to @p err and return @p err
 */
#define TRY(err, expr) do {\
		err = expr; \
		if (err) { \
			return err; \
		} \
	} while (false)

#define TRY_MEM(mem, expr) do {\
		mem = expr; \
		if (IS_ERR(mem)) \
			return PTR_ERR(mem); \
	} while (false)

struct ov10640
{
	struct camera_common_power_rail	power;

	struct i2c_client *client;
	struct device *dev;
	struct regmap *map;

	struct camera_common_data *s_data;
	struct camera_common_pdata *pdata;

	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev *subdev;
	struct v4l2_ctrl **ctrls;
	int numctrls;

	bool group_hold;

	int frame_sync_mode;

	struct i2c_client *deserializer;
	bool hflip;
	bool vflip;
};

#endif
