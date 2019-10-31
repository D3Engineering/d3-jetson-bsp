/**
 * @author Greg Rowe <growe@d3engineering.com>
 *
 * ov10640 v4l2 driver for Nvidia Jetson - default register settings
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
#ifndef _OV10640_TABLES_H
#define _OV10640_TABLES_H

#include <linux/kernel.h>
#include <media/camera_common.h>

enum {
	OV10640_MODE_1280X1080_LONG,
#ifdef CONFIG_D3_OV10640_HDR_ENABLE
	OV10640_MODE_1280X1080_HDR,
#endif /* CONFIG_D3_OV10640_HDR_ENABLE */
	OV10640_MODE_END,
	OV10640_MODE_DEFAULT = OV10640_MODE_1280X1080_LONG
};

struct reg_table {
	const struct reg_sequence *reg_sequence;
	size_t size;
};


#ifdef CONFIG_D3_OV10640_HDR_ENABLE
extern const struct reg_sequence mode_1280x1080hdr[];
extern const size_t mode_1280x1080hdr_len;
#endif /* CONFIG_D3_OV10640_HDR_ENABLE */
extern const struct reg_sequence mode_1280x1080long[];
extern const size_t mode_1280x1080long_len;
extern const struct reg_sequence mode_stream[];
extern const size_t mode_stream_len;
extern const struct reg_sequence mode_60fps[];
extern const size_t mode_60fps_len;


extern const struct reg_table mode_table[];
extern const size_t mode_table_len;

extern const int mode_1280x1080_fps[];
extern const size_t mode_1280x1080_fps_len;


/**
 * Mode table as expected by Nvidia's camera_common framework
 */
extern const struct camera_common_frmfmt ov10640_formats[];
extern const size_t ov10640_formats_len;

#endif
