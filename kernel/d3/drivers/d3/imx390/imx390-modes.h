/**
 * @author Greg Rowe <growe@d3engineering.com>
 *
 * imx390 v4l2 driver for Nvidia Jetson - default register settings
 *
 * Copyright (c) 2018, D3 Engineering.  All rights reserved.
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
#ifndef _IMX390_MODES_H
#define _IMX390_MODES_H

#include <media/camera_common.h>
#ifdef CONFIG_D3_IMX390_HDR_ENABLE
#	include "imx390-mode-1936x1100-HDR.h"
#endif	/* CONFIG_D3_IMX390_HDR_ENABLE */
#include "imx390-mode-1936x1100-SP1L.h"
#include "imx390-mode-1936x1100-SP1H.h"
#include "imx390-mode-1936x1100-SP2.h"


enum imx390_mode {
	IMX390_MODE_SP1L,
	IMX390_MODE_SP1H,
	IMX390_MODE_SP2,
#ifdef CONFIG_D3_IMX390_HDR_ENABLE
	IMX390_MODE_HDR,
#endif	/* CONFIG_D3_IMX390_HDR_ENABLE */
	IMX390_MODE_ENDMARKER,
	IMX390_MODE_DEFAULT = IMX390_MODE_SP1L,
};

struct imx390_modes_map {
	const char *desc;
	const size_t *n_vals;
	const struct reg_sequence *vals;
};

extern struct imx390_modes_map imx390_modes_map[];
extern const size_t imx390_modes_map_len;

extern const struct camera_common_frmfmt imx390_modes_formats[];
extern const size_t imx390_modes_formats_len;

#endif
