/**
 * @author Greg Rowe <growe@d3engineering.com>
 *
 * imx490 v4l2 driver for Nvidia Jetson - default register settings
 *
 * Copyright (c) 2020-2021, D3 Engineering.  All rights reserved.
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
#ifndef _IMX490_MODES_H
#define _IMX490_MODES_H

#include <media/camera_common.h>

#include "imx490.h"

extern const struct camera_common_frmfmt *imx490_modes_formats;
extern const size_t imx490_modes_formats_len;

extern struct imx490_modes_map *imx490_modes_map;
extern size_t imx490_modes_map_len;

int imx490_detect_revision(struct imx490 *self);

#endif
