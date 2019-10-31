/**
 * @author Greg Rowe <growe@d3engineering.com>
 *
 * imx390 v4l2 driver for Nvidia Jetson
 *
 * Copyright (c) 2018-2019, D3 Engineering.  All rights reserved.
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
#include "imx390-modes.h"

const int imx390_modes_60fps[] = {
	60,
};

const int imx390_modes_30fps[] = {
	30,
};



/**
 * Mode table as expected by Nvidia's camera_common framework
 */
const struct camera_common_frmfmt imx390_modes_formats[] = {
	/* Important: ensure that the order of entries in this table
	 * matches enum IMX390_MODE */

	/* The imager outputs 1936x1096 but includes 4 lines of SMPG
	 * data.  SMPG is Safety Mechanism Pattern Generator.  There is
	 * also 1 line of embedded data that cannot be disabled.  The
	 * embedded data line does NOT count towards the overall
	 * dimensions. */
	{
		.size = {1936, 1100},
		.framerates = imx390_modes_30fps,
		.num_framerates = 1,
		.hdr_en = 0,
		.mode = IMX390_MODE_SP1L,
	},
	{
		.size = {1936, 1100},
		.framerates = imx390_modes_30fps,
		.num_framerates = 1,
		.hdr_en = 0,
		.mode = IMX390_MODE_SP1H,
	},
	{
		.size = {1936, 1100},
		.framerates = imx390_modes_30fps,
		.num_framerates = 1,
		.hdr_en = 0,
		.mode = IMX390_MODE_SP2,
	},
#ifdef CONFIG_D3_IMX390_HDR_ENABLE
	{
		.size = {1936, 1100},
		.framerates = imx390_modes_60fps,
		.num_framerates = 1,
		.hdr_en = 1,
		.mode = IMX390_MODE_HDR,
	},
#endif	/* CONFIG_D3_IMX390_HDR_ENABLE */
};
const size_t imx390_modes_formats_len = ARRAY_SIZE(imx390_modes_formats);


struct imx390_modes_map imx390_modes_map[] = {
	/* Important: ensure that the order of entries in this table
	 * matches enum IMX390_MODE */
	{
		.desc = "SP1L Linear 30 fps",
		.n_vals = &imx390_mode_1936x1100SP1L_len,
		.vals = imx390_mode_1936x1100SP1L
	},
	{
		.desc = "SP1H Linear 30 fps",
		.n_vals = &imx390_mode_1936x1100SP1H_len,
		.vals = imx390_mode_1936x1100SP1H
	},
	{
		.desc = "SP2 Linear 30 fps",
		.n_vals = &imx390_mode_1936x1100SP2_len,
		.vals = imx390_mode_1936x1100SP2
	},
#ifdef CONFIG_D3_IMX390_HDR_ENABLE
	{
		.desc = "HDR 60 fps",
		.n_vals = &imx390_mode_1936x1100HDR_len,
		.vals = imx390_mode_1936x1100HDR
	},
#endif	/* CONFIG_D3_IMX390_HDR_ENABLE */
};
const size_t imx390_modes_map_len = ARRAY_SIZE(imx390_modes_map);
