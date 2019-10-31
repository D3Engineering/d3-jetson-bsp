/*
 * max9286_tables.h - Maxim MAX9286 GMSL1 to CSI-2 Deserializer
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

#ifndef _MAX9286_TABLES_H_
#define _MAX9286_TABLES_H_

#include "max9286.h"

/* MAX9286 */
static const struct reg_sequence max9286_custom_rcs[] = {
	{0x3F,0x4F},      //Enable Custom Reverse Channel & First Pulse Length
	{0x3B,0x1E,2000}, //Reverse Channel Amplitude to mid level and transition time
};

static const struct reg_sequence max9286_disable_loc_ack[] = {
	{0x34,0x35}, // Disable local ACK
};

static const struct reg_sequence max9286_dis_csi[] = {
	{0x3B,0x19,2000}, // Reverse Channel Amplitude level
	{0x15,0x03},      // Disable CSI output
	{0x12,0x77},      // 2 CSI Lanes, CSI DBL, GMSL DBL, RAW12
	{0x01,0xE2},      // Disable frame sync
	{0x00,0x01},      // Enable GMSL Link 0
};

static const struct reg_sequence max9286_en_hibw[] = {
	{0x1C,0x06,16000} // HIBW
};

static const struct reg_sequence max9286_en_csi[] = {
	{0x15,0x0B},
};

#endif /* _MAX9286_TABLES_H_ */
