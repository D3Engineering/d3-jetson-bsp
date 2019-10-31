/*
 * max9296_tables.h - Maxim MAX9296 GMSL2/GMSL1 to CSI-2 Deserializer
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

#ifndef _MAX9296_TABLES_H_
#define _MAX9296_TABLES_H_

#include "max9295.h"
#include "max9296.h"
#include "max96705.h"

/* Reverse channel setup */
static const struct reg_sequence max9296_custom_rcs[] = {
	{ 0x0006, 0x1F }, /* Enable GMSL1 mode on both links */
	{ 0x0010, 0x30, 100000 }, /* One-shot reset */
	{ 0x0B0D, 0x80 },	// I2C_LOC_ACK = 1
	{ 0x14C5, 0x8F, 5000 },	// First pulse length
	{ 0x14C4, 0xA0, 5000 },	// Rise and fall time
};

static const struct reg_sequence max9296_en_him[] = {
	{ 0x0B06, 0xEF, 5000 },	// HIM = 1
	{ 0x14C4, 0xB0, 5000 }, // Rise and fall times
	{ 0x14C5, 0xAD, 5000 }, // First pulse length
	{ 0x14C6, 0x12, 5000 },
	{ 0x0F05, 0x22, 5000 }, // I2C standard-mode speed, timeout: 2ms
	{ 0x0B05, 0x79, 5000 }, // NO_REM_MST, HVTR_MODE, EN_EQ, EQTUNE: 9.7dB
};

static const struct reg_sequence max9296_dis_locack[] = {
	{ 0x0B0D, 0x00 },	// I2C_LOC_ACK = 0
};

/* Pre-Camera Setup */
static const struct reg_sequence max9296_dis_csi[] = {
	{ 0x0313,0x00 },	// Disable CSI output
	{ 0x0F00,0x01, 16000 },	// Enable GMSL1 link A
};

static const struct reg_sequence max9296_en_dbl_hven[] = {
	{ 0x0B07, 0x84, 16000 }, // DBL, HVEN
};

static const struct reg_sequence max9296_en_csi[] = {
	{ 0x0B07, 0x8C, 16000 },	// HIBW
	{ 0x0320, 0x30 },	// MIPI port A output is 1600Mbps (800MHz)
	{ 0x044A, 0x50 },	// 2 lanes on MIPI port A
	{ 0x01DA, 0x18 },	// Copy HS to DE in pipe X
	{ 0x0314, 0x00 },	// VC = 0 (default)
	{ 0x0316, 0x2C },	// DT = 0x2C
	{ 0x0313, 0x60 },	// BPP = 12
	{ 0x031D, 0x6F },	// Enable BPP/DT/VC.
	{ 0x0B96, 0x3B },	// Enable GMSL1 to GMSL2 color mapping, set to RAW12 single
	{ 0x0BA7, 0x45 },	// Shift HVD out of the way
	{ 0x040B, 0x07 },	// Enable 3 mappings for Pipe X
	{ 0x040D, 0x2C },	// Pipe X video data source
	{ 0x040E, 0x2C },	// Pipe X video data destination
	{ 0x040F, 0x00 },	// Pipe X Frame Start Source
	{ 0x0410, 0x00 },	// Pipe X Frame Start Destination
	{ 0x0411, 0x01 },	// Pipe X Frame End Source
	{ 0x0412, 0x01 },	// Pipe X Frame End Destination
	{ 0x042D, 0x15 },	// Pipe X mappings to DPHY1 (master for port A)
	{ 0x0313, 0x62 },	// Enable CSI
};

static struct reg_sequence max9296_set_gpios[] = {
	{ .reg = 0x02b0, .def = 0x83, .delay_us = 0 }, /* GPIO0 (Frame Sync Input): Enable GPIO TX to serializer */
	{ .reg = 0x02b1, .def = 0xb8, .delay_us = 0 }, /* GPIO0 (Frame Sync Input): Set TX ID to 0x18 */
	{ .reg = 0x02b2, .def = 0x18, .delay_us = 0 }, /* GPIO0 (Frame Sync Input): Set RX ID to 0x18 */
};


/* static struct reg_sequence max9296_init_seq[] = { */
/* 	{ .reg = 0x0001, .def = 0x02, .delay_us = 0 }, /\* link speed = 6Gbps *\/ */
/* 	{ .reg = 0x0010, .def = 0x31, .delay_us = 100000 }, /\* RESET_ONE_SHOT | AUTO_LINK *\/ */
/* 	{ .reg = 0x0330, .def = 0x04, .delay_us = 0 }, /\* MIPI_PHY0 set des in 2 x 4 mode*\/ */
/* 	{ .reg = 0x0333, .def = 0x4E, .delay_us = 0 }, /\* MIPI_PHY3 Lane Mapping register 1*\/ */
/* 	{ .reg = 0x0334, .def = 0xE4, .delay_us = 0 }, /\* MIPI_PHY4 Lane Mapping register 2*\/ */
/* 	{ .reg = 0x040A, .def = 0x00, .delay_us = 0 }, /\* MIPI_TX10 PHY0 Lane Count Register not used in 2 x 4*\/ */
/* 	{ .reg = 0x044A, .def = 0x40, .delay_us = 0 }, /\* MIPI_TX10 PHY1 Lane Count Register in Port A*\/ */
/* 	{ .reg = 0x048A, .def = 0x40, .delay_us = 0 }, /\* MIPI_TX10 PHY2 Lane Count Register in Port B*\/ */
/* 	{ .reg = 0x04CA, .def = 0x00, .delay_us = 0 }, /\* MIPI_TX10 PHY3 Lane Count register unused in 2 x 4*\/ */
/* 	{ .reg = 0x031D, .def = 0x30, .delay_us = 0 }, /\* BACKTOP22 PHY0 MIPI Clock Rate not used in 2 x 4*\/ */
/* 	{ .reg = 0x0320, .def = 0x30, .delay_us = 0 }, /\* BACKTOP25 PHY1 MIPI Clock Rate Port A = 600 MHz*\/ */
/* 	{ .reg = 0x0323, .def = 0x30, .delay_us = 0 }, /\* BACKTOP28 PHY2 MIPI Clock Rate Port B*\/ */
/* 	{ .reg = 0x0326, .def = 0x30, .delay_us = 0 }, /\* BACKTOP31 PHY3 MIPI Clock Rate not used in 2 x 4*\/ */
/* 	{ .reg = 0x0050, .def = 0x00, .delay_us = 0 }, /\* RX0 Pipe X Stream Select*\/ */
/* 	{ .reg = 0x0051, .def = 0x01, .delay_us = 100 }, /\* RX0 Pipe Y Stream Select*\/ */
/* 	{ .reg = 0x0313, .def = 0x02, .delay_us = 0 }, /\* BACKTOP12 Enable MAX9296 MIPI CSI *\/ */
/* }; */

/* static struct reg_sequence max9295_init_seq[] = { */
/* 	{ .reg = 0x0001, .def = 0x08, .delay_us = 0, }, /\* link speed 6Gbps *\/ */
/* 	{ .reg = 0x0010, .def = 0x21, .delay_us = 100000, }, /\* Reset one shot *\/ */
/* 	{ .reg = 0x0330, .def = 0x00, .delay_us = 0 }, /\* MIPI_RX0 Set PHY to 1x4 mode*\/ */
/* 	{ .reg = 0x0332, .def = 0xBE, .delay_us = 0 }, /\* [E0] MIPI_RX2 Lane Mapping Register 1 Swap lanes 0 & 1 for HAS000302*\/ */
/* 	{ .reg = 0x0333, .def = 0xE1, .delay_us = 0 }, /\* [04] MIPI_RX3 Lane Mapping Register 2 Swap lanes 2 & 3 for HAS000302*\/ */
/* 	{ .reg = 0x0331, .def = 0x33, .delay_us = 0 }, /\* MIPI_RX1 Enable Port A & B as they are shared in MAX9295A*\/ */
/* 	{ .reg = 0x0308, .def = 0x6F, .delay_us = 0 }, /\* FRONTTOP_0 CSI Port selection Port B active in 1 x 4 mode*\/ */
/* 	{ .reg = 0x0311, .def = 0x20, .delay_us = 0 }, /\* FRONTTOP_9 Start video pipe Y*\/ */
/* 	{ .reg = 0x0314, .def = 0x22, .delay_us = 0 }, /\* FRONTTOP_12 Pipe X unused*\/ */
/* 	{ .reg = 0x0316, .def = 0x6C, .delay_us = 0 }, /\* FRONTTOP_14 Datatype RAW12*\/ */
/* 	{ .reg = 0x0318, .def = 0x22, .delay_us = 0 }, /\* FRONTTOP_16 Pipe Z unused*\/ */
/* 	{ .reg = 0x031A, .def = 0x22, .delay_us = 0 }, /\* FRONTTOP_18 Pipe U unused*\/ */
/* 	{ .reg = 0x0002, .def = 0xF3, .delay_us = 0 }, /\* REG2 Make sure all pipes start transmission*\/ */
/* }; */

/* static struct reg_sequence max9296_set_gmsl_speed[] = { */
/* 	{.reg = 0x01, .def = MAX9296_GMSL_RATE_DEFAULT, .delay_us = 0}, */
/* 	{.reg = 0x10, .def = 0x31, .delay_us = 100000}, */
/* }; */

/* static struct reg_sequence max9295_set_gmsl_speed[] = { */
/* 	{.reg = 0x01, .def = MAX9295_GMSL_RATE_DEFAULT, .delay_us = 0}, */
/* 	{.reg = 0x10, .def = 0x21, .delay_us = 100000}, */
/* }; */

/* Begin I2C address reassignment stuff. */
static struct reg_sequence max9296_enable_link_a_only[] = {
	{ .reg = 0x0010, .def = 0x01, .delay_us = 0 },
	{ .reg = 0x0010, .def = 0x21, .delay_us = 100000 },
};

/* static struct reg_sequence max9296_enable_link_b_only[] = { */
/* 	{ .reg = 0x0010, .def = 0x02, .delay_us = 0 }, */
/* 	{ .reg = 0x0010, .def = 0x22, .delay_us = 100000 }, */
/* }; */

static struct reg_sequence max9296_enable_link_both[] = {
	{ .reg = 0x0010, .def = 0x03, .delay_us = 0 },
	{ .reg = 0x0010, .def = 0x23, .delay_us = 100000 },
};

static struct reg_sequence max9295_assign_address_40[] = {
	{ .reg = 0x0010, .def = 0x31, .delay_us = 100000 },
	{ .reg = 0x006B, .def = 0x10 },
	{ .reg = 0x0073, .def = 0x11 },
	{ .reg = 0x007B, .def = 0x30 },
	{ .reg = 0x0083, .def = 0x30 },
	{ .reg = 0x0093, .def = 0x30 },
	{ .reg = 0x009B, .def = 0x30 },
	{ .reg = 0x00A3, .def = 0x30 },
	{ .reg = 0x00AB, .def = 0x30 },
	{ .reg = 0x008B, .def = 0x30 },
};

static struct reg_sequence max9295_assign_address_42[] = {
	{ .reg = 0x0010, .def = 0x31, .delay_us = 100000 },
	{ .reg = 0x006B, .def = 0x10 },
	{ .reg = 0x0073, .def = 0x10 },
	{ .reg = 0x007B, .def = 0x31 },
	{ .reg = 0x0083, .def = 0x31 },
	{ .reg = 0x0093, .def = 0x31 },
	{ .reg = 0x009B, .def = 0x31 },
	{ .reg = 0x00A3, .def = 0x31 },
	{ .reg = 0x00AB, .def = 0x31 },
	{ .reg = 0x008B, .def = 0x31 },
};

static struct reg_sequence max9295_assign_address_60[] = {
	{ .reg = 0x0010, .def = 0x31, .delay_us = 100000 },
	{ .reg = 0x006B, .def = 0x12 },
	{ .reg = 0x0073, .def = 0x13 },
	{ .reg = 0x007B, .def = 0x32 },
	{ .reg = 0x0083, .def = 0x32 },
	{ .reg = 0x0093, .def = 0x32 },
	{ .reg = 0x009B, .def = 0x32 },
	{ .reg = 0x00A3, .def = 0x32 },
	{ .reg = 0x00AB, .def = 0x32 },
	{ .reg = 0x008B, .def = 0x32 },
};

static struct reg_sequence max9295_assign_address_62[] = {
	{ .reg = 0x0010, .def = 0x31, .delay_us = 100000 },
	{ .reg = 0x006B, .def = 0x12 },
	{ .reg = 0x0073, .def = 0x13 },
	{ .reg = 0x007B, .def = 0x33 },
	{ .reg = 0x0083, .def = 0x33 },
	{ .reg = 0x0093, .def = 0x33 },
	{ .reg = 0x009B, .def = 0x33 },
	{ .reg = 0x00A3, .def = 0x33 },
	{ .reg = 0x00AB, .def = 0x33 },
	{ .reg = 0x008B, .def = 0x33 },
};
/* End I2C address reassignment stuff. */

#endif /* _MAX9296_TABLES_H_ */
