/*
 * reg-ctl.h - Simple sysfs register access for a camera.
 *
 * Copyright (c) 2020, D3 Engineering. All rights reserved.
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

#ifndef REG_CTL_H
#define REG_CTL_H

struct regmap;
struct device;
struct reg_ctl;

struct reg_ctl *reg_ctl_init(struct regmap *regmap, struct device *dev);
void reg_ctl_free(struct reg_ctl *rc);

#endif /* !REG_CTL_H */
