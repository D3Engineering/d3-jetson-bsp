/**
 * @author Greg Rowe <growe@d3engineering.com> @author Christopher
 * White <cwhite@d3engineering.com> @author Jacob Kiggins
 * <jkiggins@d3engineering.com>
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
#include <linux/module.h>
#include <linux/regmap.h>
#include "imx490_modes.h"
#include "imx490-mode-2880x1860-linear-es4.h"
#include "imx490-mode-2880x1860-hdr-es4.h"


static const int imx490_framerates_30fps[] = {
	30,
};

static const struct camera_common_frmfmt _imx490_modes_formats[] = {
	{
		/* .size = {2880, 1864}, */
		.size = {2880, 1860},
		.framerates = imx490_framerates_30fps,
		.num_framerates = 1,
		.hdr_en = 0,
		.mode = IMX490_MODE_LINEAR
	},
	{
		/* .size = {2880, 1864}, */
		.size = {2880, 1860},
		.framerates = imx490_framerates_30fps,
		.num_framerates = 1,
		.hdr_en = 1,
		.mode = IMX490_MODE_HDR
	},
};


static struct imx490_modes_map imx490_modes_map_es4[] = {
	{
		.desc = "HDR 30 fps (settings from SONY, ES4)",
		.n_vals = &imx490_mode_2880x1860_hdr_es4_len,
		.vals = imx490_mode_2880x1860_hdr_es4,
		.exposure_line_time_us_q = EXP_PER_LINE_30FPS,
	},
	{
		.desc = "Linear 30 fps (settings from SONY, ES4)",
		.n_vals = &imx490_mode_2880x1860_linear_es4_len,
		.vals = imx490_mode_2880x1860_linear_es4,
		.exposure_line_time_us_q = EXP_PER_LINE_30FPS,
	},

};

const struct camera_common_frmfmt *imx490_modes_formats = _imx490_modes_formats;
const size_t imx490_modes_formats_len = ARRAY_SIZE(_imx490_modes_formats);
struct imx490_modes_map *imx490_modes_map = imx490_modes_map_es4;
size_t imx490_modes_map_len = ARRAY_SIZE(imx490_modes_map_es4);

int imx490_detect_revision(struct imx490 *self)
{
	struct device *dev = self->dev;
	struct regmap *map = self->map;
	unsigned chip_id_4;
	int ret;

	ret = regmap_read(map, IMX490_REG_CHIP_ID_4, &chip_id_4);
	if (ret)
		return -1;

	dev_dbg(dev, "chip_id: %#x=%#x", IMX490_REG_CHIP_ID_4, chip_id_4);
	if (chip_id_4 == 0x00) {
		self->rev = IMX490_REV_ES2;
		dev_warn(dev, "ES2 is not supported");
		return -1;
	} else if (chip_id_4 == 0x08) {
		self->rev = IMX490_REV_ES4;
		imx490_modes_map = imx490_modes_map_es4;
		imx490_modes_map_len = ARRAY_SIZE(imx490_modes_map_es4);
		dev_dbg(dev, "REV ES4");
	} else {
		self->rev = IMX490_REV_UNKNOWN;
		dev_warn(dev, "Unknown revision (CHIP_ID_4: %#02x)", chip_id_4);
	}

	return 0;
}
