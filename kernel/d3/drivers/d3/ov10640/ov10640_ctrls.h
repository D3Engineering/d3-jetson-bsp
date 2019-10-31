/**
 * @author Josh Watts <jwatts@d3engineering.com>
 *
 * ov10640 v4l2 driver for Nvidia Jetson
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
#ifndef _OV10640_CTRLS_H
#define _OV10640_CTRLS_H

#include "ov10640.h"

int ov10640_ctrls_count(void);
int ov10640_ctrls_init(struct ov10640 *self);
int ov10640_hflip_set(struct ov10640 *self, bool hflip);
int ov10640_vflip_set(struct ov10640 *self, bool vflip);

#endif
