/*
 * @author Mike Soltiz <msoltiz@d3engineering.com>
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
#ifndef __VG6768_TABLES__
#define __VG6768_TABLES__

#define VG6768_DEFAULT_MODE VG6768_MODE_1920X1080_30FPS

enum {
	VG6768_MODE_1920X1080_30FPS = 0,
	VG6768_MODE_END
};

static const int vg6768_30fps[] = {
	30,
};


static const struct camera_common_frmfmt vg6768_frmfmt[] = {
	{
		.size = {1920, 1080},
		.framerates = vg6768_30fps,
		.num_framerates = 1,
		.hdr_en = 0,
		.mode = VG6768_MODE_1920X1080_30FPS
	},
};

static const struct reg_sequence vg6768_isp_start[] = {
	{0x04FC, 0x00},
	{0x04FD, 0x00},
	{0x04FE, 0x00},
	{0x04FF, 0x00},
	{0x0500, 0x11},
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},
};

static const struct reg_sequence mode_1920X1080_30fps[] = {
/* Set CSI2 TX Interface sequence */
	{0x04F4, 0x03}, //Lane mapping in application board
	{0x04F5, 0x02}, //Lane mapping in application board
	{0x04F6, 0x01},	//Lane mapping in application board
	{0x04F7, 0x00},	//Lane mapping in application board
	{0x04F8, 0x20},	//Data Bit Rate (800 Mbs)
	{0x04F9, 0x03}, //Data Bit Rate
	{0x04FA, 0x00}, //Data Bit Rate
	{0x04FB, 0x00}, //Data Bit Rate
	{0x04FC, 0x83}, //Lane Number + Virtual Channel (4 lanes, no Virtual Channels)
	{0x04FD, 0x00},	//Lane Number + Virtual Channel
	{0x04FE, 0x00},	//Lane Number + Virtual Channel
	{0x04FF, 0x00},	//Lane Number + Virtual Channel
	{0x0500, 0x34},	//Opcode = 0x34 Set CSI2 TX Interface
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},
	{0x0151, 0x01},//Disable ASIL lines
	{0x0153, 0x01},//Disable ASIL lines

/* Set Input Image Configuration sequence 0,0-1923,1083*/
	{0x04F8, 0x83},	//Image bottom right index
	{0x04F9, 0x07}, //Image bottom right index
	{0x04FA, 0x3B}, //Image bottom right index
	{0x04FB, 0x04}, //Image bottom right index
	{0x04FC, 0x00}, //Image top left index
	{0x04FD, 0x00},	//Image top left index
	{0x04FE, 0x00},	//Image top left index
	{0x04FF, 0x00},	//Image top left index
	{0x0500, 0x33},	//Opcode = 0x33 Set Input Image Configuration
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},

/* Set Output Image Configuration sequence rgb888, 1920x1080*/
	{0x04F8, 0x02},	//Format: 0x09-YUV422 10 bits, 0x02-RGB888, 0x08-YUV422 8 bits, 0x05-YUV420 8 bits
	{0x04F9, 0x00},
	{0x04FA, 0x00},
	{0x04FB, 0x00},
	{0x04FC, 0x80}, //Image Width
	{0x04FD, 0x07},	//Image Width
	{0x04FE, 0x38},	//Image Height
	{0x04FF, 0x04},	//Image Heigth
	{0x0500, 0x37},	//Opcode = 0x37 Set Output Image Configuration
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},

/* Set CSI RX Interface sequence */
	{0x04F4, 0x04}, //Lane mapping in application board
	{0x04F5, 0x03}, //Lane mapping in application board
	{0x04F6, 0x02},	//Lane mapping in application board
	{0x04F7, 0x01},	//Lane mapping in application board
	{0x04F8, 0x18},	//Data Bit Rate
	{0x04F9, 0x03}, //Data Bit Rate
	{0x04FA, 0x00}, //Data Bit Rate
	{0x04FB, 0x00}, //Data Bit Rate
	{0x04FC, 0x03}, //Lane Number 4
	{0x04FD, 0x00},
	{0x04FE, 0x00},
	{0x04FF, 0x00},
	{0x0500, 0x31},	//Opcode = 0x31 Set CSI RX Interface
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},

/* Set Output Image Source sequence */
	{0x04FC, 0x03}, //Source 0x03-ISP for display vision
	{0x04FD, 0x00},
	{0x04FE, 0x00},
	{0x04FF, 0x00},
	{0x0500, 0x36},	//Opcode = 0x36 Set Output Image Source
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},

/* Set Dynamic Range sequence */
	{0x04FC, 0x01}, //Standard = 0x01, High = 0x00
	{0x04FD, 0x00},
	{0x04FE, 0x00},
	{0x04FF, 0x00},
	{0x0500, 0x42},	//Opcode = 0x42 Set Dynamic Range
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},

/* Set Frame Rate sequence */
	{0x04FC, 0x1D}, //Frame Rate (29???)
	{0x04FD, 0x00},
	{0x04FE, 0x00},
	{0x04FF, 0x00},
	{0x0500, 0x39},	//Opcode = 0x39 Set Frame Rate
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},

/* Set Exposure Mode sequence */
	{0x04FC, 0x00}, //0x00: Day 60Hz, 0x01: Night 60Hz, 0x02: Day 50Hz, 0x03: Night 50Hz, 0x04: No Motion Blur
	{0x04FD, 0x00},
	{0x04FE, 0x00},
	{0x04FF, 0x00},
	{0x0500, 0x53},	//Opcode = 0x53 Set Exposure Mode
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},

};

static const struct reg_sequence vg6768_start[] = {
	{0x04FC, 0x00},
	{0x04FD, 0x00},
	{0x04FE, 0x00},
	{0x04FF, 0x00},
	{0x0500, 0x11},
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},
};

static const struct reg_sequence vg6768_stop[] = {
	{0x04FC, 0x00},
	{0x04FD, 0x00},
	{0x04FE, 0x00},
	{0x04FF, 0x00},
	{0x0500, 0x12},
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x00},
};
#endif  /* __VG6768_TABLES__ */
