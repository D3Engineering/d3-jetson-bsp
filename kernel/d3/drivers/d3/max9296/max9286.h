/*
 * max9286.h - Maxim MAX9286 GMSL1 Deserializer tables
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

#ifndef _MAX9286_H_
#define _MAX9286_H_

#define MAX9286_VERSION "0.3.0"
enum {
	MAX9286_CH0,
	MAX9286_CH1,
	MAX9286_CH2,
	MAX9286_CH3,
	MAX9286_NUM_CHANNELS,
};

/* The maximum number of I2C address translations that the chip can do */
#define MAX9286_I2C_TRANS_MAX 2U

/* Register 0x0A */
#define MAX9286_REVCCEN0 (1 << 0)
#define MAX9286_REVCCEN1 (1 << 1)
#define MAX9286_REVCCEN2 (1 << 2)
#define MAX9286_REVCCEN3 (1 << 3)
#define MAX9286_FWDCCEN0 (1 << 4)
#define MAX9286_FWDCCEN1 (1 << 5)
#define MAX9286_FWDCCEN2 (1 << 6)
#define MAX9286_FWDCCEN3 (1 << 7)

#endif /* _MAX9286_H_ */
