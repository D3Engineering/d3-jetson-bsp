/*
 * max9295_tables.h - Maxim MAX9295 CSI-2 to GMSL2 Serializer
 *
 * Copyright (c) 2018, 2019, D3 Engineering.  All rights reserved.
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

#ifndef _MAX9295_TABLES_H_
#define _MAX9295_TABLES_H_

#include "max9295.h"

static struct reg_sequence max9295_init_seq[] = {
	{ .reg = 0x0001, .def = 0x08, .delay_us = 0, }, /* link speed 6Gbps */
	{ .reg = 0x0010, .def = 0x21, .delay_us = 100000, }, /* Reset one shot */
	{ .reg = 0x0330, .def = 0x00, .delay_us = 0 }, /* MIPI_RX0 Set PHY to 1x4 mode*/
	{ .reg = 0x0332, .def = 0xBE, .delay_us = 0 }, /* [E0] MIPI_RX2 Lane Mapping Register 1 Swap lanes 0 & 1 for HAS000302*/
	{ .reg = 0x0333, .def = 0xE1, .delay_us = 0 }, /* [04] MIPI_RX3 Lane Mapping Register 2 Swap lanes 2 & 3 for HAS000302*/
	{ .reg = 0x0331, .def = 0x33, .delay_us = 0 }, /* MIPI_RX1 Enable Port A & B as they are shared in MAX9295A*/
	{ .reg = 0x0308, .def = 0x6F, .delay_us = 0 }, /* FRONTTOP_0 CSI Port selection Port B active in 1 x 4 mode*/
	{ .reg = 0x0311, .def = 0x20, .delay_us = 0 }, /* FRONTTOP_9 Start video pipe Y*/
	{ .reg = 0x0314, .def = 0x22, .delay_us = 0 }, /* FRONTTOP_12 Pipe X unused*/
	{ .reg = 0x0316, .def = 0x6C, .delay_us = 0 }, /* FRONTTOP_14 Datatype RAW12*/
	{ .reg = 0x0318, .def = 0x22, .delay_us = 0 }, /* FRONTTOP_16 Pipe Z unused*/
	{ .reg = 0x031A, .def = 0x22, .delay_us = 0 }, /* FRONTTOP_18 Pipe U unused*/
	{ .reg = 0x02d6, .def = 0x94, .delay_us = 0 }, /* GPIO8 (Frame Sync Output): Enable GPIO RX from deserializer */
	{ .reg = 0x02d7, .def = 0xb8, .delay_us = 0 }, /* GPIO8 (Frame Sync Output): Set TX ID to 0x18 */
	{ .reg = 0x02d8, .def = 0x18, .delay_us = 0 }, /* GPIO8 (Frame Sync Output): Set RX ID to 0x18 */
	{ .reg = 0x0002, .def = 0xF3, .delay_us = 0 }, /* REG2 Make sure all pipes start transmission*/
};

/* static struct reg_sequence max9295_set_gmsl_speed[] = { */
/* 	{.reg = 0x01, .def = MAX9295_GMSL_RATE_DEFAULT, .delay_us = 0}, */
/* 	{.reg = 0x10, .def = 0x21, .delay_us = 100000}, */
/* }; */

/* static struct reg_sequence max9295_assign_address_40[] = { */
/* 	{ .reg = 0x0010, .def = 0x31, .delay_us = 100000 }, */
/* 	{ .reg = 0x006B, .def = 0x10 }, */
/* 	{ .reg = 0x0073, .def = 0x11 }, */
/* 	{ .reg = 0x007B, .def = 0x30 }, */
/* 	{ .reg = 0x0083, .def = 0x30 }, */
/* 	{ .reg = 0x0093, .def = 0x30 }, */
/* 	{ .reg = 0x009B, .def = 0x30 }, */
/* 	{ .reg = 0x00A3, .def = 0x30 }, */
/* 	{ .reg = 0x00AB, .def = 0x30 }, */
/* 	{ .reg = 0x008B, .def = 0x30 }, */
/* }; */

/* static struct reg_sequence max9295_assign_address_42[] = { */
/* 	{ .reg = 0x0010, .def = 0x31, .delay_us = 100000 }, */
/* 	{ .reg = 0x006B, .def = 0x10 }, */
/* 	{ .reg = 0x0073, .def = 0x10 }, */
/* 	{ .reg = 0x007B, .def = 0x31 }, */
/* 	{ .reg = 0x0083, .def = 0x31 }, */
/* 	{ .reg = 0x0093, .def = 0x31 }, */
/* 	{ .reg = 0x009B, .def = 0x31 }, */
/* 	{ .reg = 0x00A3, .def = 0x31 }, */
/* 	{ .reg = 0x00AB, .def = 0x31 }, */
/* 	{ .reg = 0x008B, .def = 0x31 }, */
/* }; */

/* static struct reg_sequence max9295_assign_address_60[] = { */
/* 	{ .reg = 0x0010, .def = 0x31, .delay_us = 100000 }, */
/* 	{ .reg = 0x006B, .def = 0x12 }, */
/* 	{ .reg = 0x0073, .def = 0x13 }, */
/* 	{ .reg = 0x007B, .def = 0x32 }, */
/* 	{ .reg = 0x0083, .def = 0x32 }, */
/* 	{ .reg = 0x0093, .def = 0x32 }, */
/* 	{ .reg = 0x009B, .def = 0x32 }, */
/* 	{ .reg = 0x00A3, .def = 0x32 }, */
/* 	{ .reg = 0x00AB, .def = 0x32 }, */
/* 	{ .reg = 0x008B, .def = 0x32 }, */
/* }; */

/* static struct reg_sequence max9295_assign_address_62[] = { */
/* 	{ .reg = 0x0010, .def = 0x31, .delay_us = 100000 }, */
/* 	{ .reg = 0x006B, .def = 0x12 }, */
/* 	{ .reg = 0x0073, .def = 0x13 }, */
/* 	{ .reg = 0x007B, .def = 0x33 }, */
/* 	{ .reg = 0x0083, .def = 0x33 }, */
/* 	{ .reg = 0x0093, .def = 0x33 }, */
/* 	{ .reg = 0x009B, .def = 0x33 }, */
/* 	{ .reg = 0x00A3, .def = 0x33 }, */
/* 	{ .reg = 0x00AB, .def = 0x33 }, */
/* 	{ .reg = 0x008B, .def = 0x33 }, */
/* }; */

/* End I2C address reassignment stuff. */

#endif /* _MAX9295_TABLES_H_ */
