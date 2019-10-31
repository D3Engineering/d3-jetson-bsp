/*
 * max96705.h - Maxim MAX96705 Serializer tables
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

#ifndef _MAX96705_H_
#define _MAX96705_H_


enum {
	MAX96705_CH0,
	MAX96705_NUM_CHANNELS
};

#define MAX96705_I2C_TRANS_MAX 2U

#define MAX96705_REG_SERADDR 0x0
#define MAX96705_REG_SRC_A 0x9
#define MAX96705_REG_DST_A 0xA
#define MAX96705_REG_SRC_B 0xB
#define MAX96705_REG_DST_B 0xC

#define MAX96705_REG_SERADDR_DEFAULT 0x80


#endif /* _MAX96705_H_ */
