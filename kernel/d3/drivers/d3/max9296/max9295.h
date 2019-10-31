/*
 * max9295.h - Maxim 9295 registers and constants.
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

#ifndef _MAX9295_H_
#define _MAX9295_H_

enum {
	MAX9295_CH0,
	MAX9295_NUM_CHANNELS,
};

/* The maximum number of I2C address translations that the chip can do */
#define MAX9295_I2C_TRANS_MAX 2U

#define MAX9295_BACK_RATE_OFFSET 0U
#define MAX9295_FORWARD_RATE_OFFSET 2U

#define MAX9295_FORWARD_RATE_MASK (3U << MAX9295_FORWARD_RATE_OFFSET)
#define MAX9295_BACK_RATE_MASK (3U << MAX9295_BACK_RATE_OFFSET)

#define MAX9295_BACK_RATE_187_5MHZ 0U
#define MAX9295_BACK_RATE_375MHZ   1U
#define MAX9295_BACK_RATE_750MHZ   2U
#define MAX9295_BACK_RATE_1500MHZ  3U
#define MAX9295_BACK_RATE_DEFAULT MAX9295_BACK_RATE_187_5MHZ

#define MAX9295_FORWARD_RATE_1500MHZ 0U
#define MAX9295_FORWARD_RATE_3000MHZ 1U
#define MAX9295_FORWARD_RATE_6000MHZ 2U
#define MAX9295_FORWARD_RATE_DEFAULT MAX9295_FORWARD_RATE_6000MHZ

#define MAX9295_GMSL_RATE(BACK_RATE, FORWARD_RATE) \
	(((BACK_RATE) << MAX9295_BACK_RATE_OFFSET) & MAX9295_BACK_RATE_MASK) | \
	(((FORWARD_RATE) << MAX9295_FORWARD_RATE_OFFSET) & MAX9295_FORWARD_RATE_MASK)

#define MAX9295_GMSL_RATE_DEFAULT \
	MAX9295_GMSL_RATE(MAX9295_BACK_RATE_DEFAULT, MAX9295_FORWARD_RATE_DEFAULT)

#define MAX9295_FWDCCEN (1 << 0)
#define MAX9295_REVCCEN (1 << 1)

/* Registers */
#define MAX9295_REG_REG0 0x0
#define MAX9295_REG_REG1 0x1

#define MAX9295_REG_CTRL0 0x10

#define MAX9295_REG_SRC_A 0x42
#define MAX9295_REG_DST_A 0x43
#define MAX9295_REG_SRC_B 0x44
#define MAX9295_REG_DST_B 0x45

#define MAX9295_REG_BACKTOP22 0x31D
#define MAX9295_REG_BACKTOP25 0x320
#define MAX9295_REG_BACKTOP28 0x323
#define MAX9295_REG_BACKTOP31 0x326

#define MAX9295_REG_GMSL1_4_A 0x0B04
#define MAX9295_REG_GMSL1_4_B 0x0C04

#endif /* _MAX9295_H_ */

