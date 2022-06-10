/*
 * reg-ctl.h - Simple sysfs register access for a camera.
 *
 * Copyright (c) 2020-2021, D3 Engineering. All rights reserved.
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


struct reg_ctl_dev {
	struct camera_common_data *s_data;
	int (*reg_read)(struct camera_common_data *s, u16 a, u16 *v);
	int (*reg_write)(struct camera_common_data *s, u16 a, u16 v);
};

struct reg_ctl {
	u16 selected_reg;
	struct kobject *kobj;
	struct kobj_attribute kobj_attr_reg;
	struct kobj_attribute kobj_attr_val;
	struct reg_ctl_dev rc_dev;
};

int reg_ctl_init(struct reg_ctl_dev rc_dev, struct device *dev);

#endif /* !REG_CTL_H */
