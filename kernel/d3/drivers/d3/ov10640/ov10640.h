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

#include <d3/d3-jetson-bsp.h>
#include <linux/i2c.h>

#include <media/camera_common.h>

/* Refer to the explanation in the DTS template file for these
 * values. */
#define OV10640_MAX_EXPO	(31540LL)
#define OV10640_MIN_EXPO	(19LL)
#define OV10640_DEF_EXPO	(19LL)

enum ov10640_mode {
	OV10640_MODE_1280X1080_LONG,
#ifdef CONFIG_D3_OV10640_HDR_ENABLE
	OV10640_MODE_1280X1080_HDR,
#endif /* CONFIG_D3_OV10640_HDR_ENABLE */
	OV10640_MODE_ENDMARKER,
	OV10640_MODE_DEFAULT = OV10640_MODE_1280X1080_LONG
};


enum ov10640_channel {
	OV10640_CHANNEL_PWL,
	OV10640_CHANNEL_LONG,
	OV10640_CHANNEL_SHORT,
	OV10640_CHANNEL_VERYSHORT,

	OV10640_CHANNEL_ENDMARKER
};

enum ov10640_hdrmode {
	OV10640_HDRMODE_3EXPOSURE,
	OV10640_HDRMODE_S2EXPOSURE,
	OV10640_HDRMODE_LVS2EXPOSURE,
	OV10640_HDRMODE_L1EXPOSURE,

	OV10640_HDRMODE_ENDMARKER
};

enum ov10640_normalization {
	OV10640_NORMALIZATION_10,
	OV10640_NORMALIZATION_12,

	OV10640_NORMALIZATION_ENDMARKER
};

struct ov10640_combine {
	u8 ctrl;
	u8 thresh[3];
	u32 weight[4];
};

struct ov10640 {
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
	struct i2c_client *serializer;
	bool hflip;
	bool vflip;
	bool test_pattern_is_enabled;

	enum ov10640_normalization normalization_type;
	struct ov10640_combine combine;
	struct gpio_desc *i2c_addressing_pin;

	s64 gain_fixed_pt;
	u16 n_lines;

#ifdef CONFIG_D3_OV10640_DEBUG
	enum ov10640_hdrmode hdr_mode;
	enum ov10640_channel channel_select;
#endif /* CONFIG_D3_OV10640_DEBUG */
};

#ifdef CONFIG_D3_OV10640_DEBUG
void ov10640_combine_dump(struct ov10640 *self, struct ov10640_combine *combine);
int ov10640_channel_set(struct ov10640 *self, enum ov10640_channel chan);
#endif /* CONFIG_D3_OV10640_DEBUG */

#endif
