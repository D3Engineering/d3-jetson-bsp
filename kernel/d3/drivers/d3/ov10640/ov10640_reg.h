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
#ifndef _OV10640_REG_H
#define _OV10640_REG_H

#define OV10640_REG_SOFTWARE_CTRL1		0x3012
#define OV10640_REG_SOFTWARE_CTRL1_SW_STBY	(0u << 0)
#define OV10640_REG_SOFTWARE_CTRL1_STREAMING	(1u << 0)

#define OV10640_REG_SOFTWARE_CTRL2		0x3013
#define OV10640_REG_SOFTWARE_CTRL2_RESET	0x1

#define OV10640_REG_INTERFACE_CTRL		0x3119
#define OV10640_REG_INTERFACE_CTRL_DATAWIDTH(x) ((x) << 0)
#define OV10640_REG_INTERFACE_CTRL_DATAWIDTH_MASK (0x3 << 0)

#define OV10640_REG_NORMALIZATION_CTRL		0x31be
#define OV10640_REG_NORMALIZATION(x)		((x) << 0)
#define OV10640_REG_NORMALIZATION_MASK		(0x1 << 0)


#define OV10640_REG_GROUP_CTRL			0x302c
#define OV10640_REG_GROUP_CTRL_PRE_SOF		(1u << 5)
#define OV10640_REG_GROUP_CTRL_1ST_GRP(x)	((x) << 2)
#define OV10640_REG_GROUP_CTRL_1ST_GRP_MASK	(3u << 2)
#define OV10640_REG_OPERATION_CTRL		0x302f
#define OV10640_REG_OPERATION_CTRL_SINGLE_START	(1u << 0)

#define OV10640_REG_HTS_H			0x3080
#define OV10640_REG_VTS_H			0x3082
#define OV10640_REG_VTS_L			0x3083

#define OV10640_REG_SENSOR_CTRL     0x308C
#define OV10640_REG_SENSOR_CTRL_FSIN_EN(x)      (0x90*x)
#define OV10640_REG_SENSOR_CTRL_FSIN_EN_MASK    (0x90)

/* This is the ratio between both the Long and Short and the Short and
 * Very Short exposures. In other words FIXED_EXPOSURE_RATIO^2
 * (squared) will give you the Dynamic Range Extension (DRE) required
 * for Jetson */
#define FIXED_EXPOSURE_RATIO	(16)

/* Used for test pattern when debugging is enabled */
#define OV10640_REG_ASP_REGA	(0x305E)
/* bit 5 of OV10640_REG_ASP_REGA is enable/disable analog test pattern */
#define ASP_REGA_ATP_STATUS	(5)


#define OV10640_REG_COMBINE_CTRL		0x3132
#define OV10640_REG_COMBINE_THRE_0		0x3133
#define OV10640_REG_COMBINE_WEIGHT_0		0x3136

#define OV10640_DARK_CURRENT_L_H		0x30D0
#define OV10640_ROW_AVERAGE_L_H			0x30D6
#define OV10640_EXPO_L_L			0x30E6
#define OV10640_EXPO_L_H			0x30E7
#define OV10640_EXPO_S_L			0x30E8
#define OV10640_EXPO_S_H			0x30E9
#define OV10640_EXPO_VS				0x30EA
#define OV10640_CG_AGAIN			0x30EB
#define OV10640_CG_AGAIN_L_AGAIN(x)		((x) << 0)
#define OV10640_CG_AGAIN_L_AGAIN_MASK		(3u << 0)

#define OV10640_DIG_L_GAIN_L			0x30ED
#define OV10640_DIG_L_GAIN_H			0x30EC
#define OV10640_DIG_S_GAIN_L			0x30EF
#define OV10640_DIG_S_GAIN_H			0x30EE
#define OV10640_DIG_VS_GAIN_L			0x30F1
#define OV10640_DIG_VS_GAIN_H			0x30F0

#define OV10640_REG_READ_MODE                     0x3090
#define OV10640_REG_READ_MODE_FLIP_MIRROR(x)      ((x) << 2)
#define OV10640_REG_READ_MODE_FLIP_MIRROR_MASK    (0x0c)
#define OV10640_REG_READ_MODE_HDRMODE(x) ((x) << 5)
#define OV10640_REG_READ_MODE_HDRMODE_MASK (0x07 << 5)


#define OV10640_REG_R_ISP_CTRL_2                  0x3128
#define OV10640_REG_R_ISP_CTRL_2_FLIP_MIRROR(x)   ((x) << 0)
#define OV10640_REG_R_ISP_CTRL_2_FLIP_MIRROR_MASK (0x03)
#define OV10640_REG_R_CTRL08                      0x3291
#define OV10640_REG_R_CTRL08_FLIP_MIRROR(x)       ((x) << 1)
#define OV10640_REG_R_CTRL08_FLIP_MIRROR_MASK     (0x06)

#endif
