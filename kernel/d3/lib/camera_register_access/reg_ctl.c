/*
 * reg-ctl.c - Simple sysfs register access for a camera.
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

#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <d3/reg_ctl.h>

struct reg_ctl {
	unsigned selected_reg;
	struct kobject *kobj;
	struct kobj_attribute kobj_attr_reg;
	struct kobj_attribute kobj_attr_val;
	struct regmap *regmap;
};

static ssize_t reg_ctl_kobj_val_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct reg_ctl *rc;
	unsigned val;
	int ret = 0;

	rc = container_of(attr, struct reg_ctl, kobj_attr_val);

	ret = kstrtouint(buf, 16, &val);
	if (ret)
		return -EINVAL;

	ret = regmap_write(rc->regmap, rc->selected_reg, val);
	if (ret)
		return ret;

	return count;
}


static ssize_t reg_ctl_kobj_val_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct reg_ctl *rc;
	unsigned val;
	int ret = 0;

	rc = container_of(attr, struct reg_ctl, kobj_attr_val);

	ret = regmap_read(rc->regmap, rc->selected_reg, &val);
	if (ret)
		return ret;

	ret = sprintf(buf, "%#04x\n", val);
	return ret;
}


static ssize_t reg_ctl_kobj_reg_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{

	struct reg_ctl *rc;
	unsigned val;
	int ret = 0;

	rc = container_of(attr, struct reg_ctl, kobj_attr_reg);

	ret = kstrtouint(buf, 16, &val);
	if (ret)
		return -EINVAL;

	rc->selected_reg = val;
	return count;
}


static ssize_t reg_ctl_kobj_reg_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct reg_ctl *rc;
	int ret = 0;
	rc = container_of(attr, struct reg_ctl, kobj_attr_reg);
	ret = sprintf(buf, "%#06x\n", rc->selected_reg);
	return ret;
}

struct reg_ctl *reg_ctl_init(struct regmap *regmap, struct device *dev)
{
	struct reg_ctl *rc;
	int ret = 0;
	const char *name = dev_name(dev);

	rc = devm_kzalloc(dev, sizeof(*rc), GFP_KERNEL);
	if (!rc) {
		ret = -ENOMEM;
		goto err;
	}

	*rc = ((struct reg_ctl) {
		.kobj = NULL,
		.selected_reg = 0u,
		.kobj_attr_val =
			__ATTR(value, 0644, reg_ctl_kobj_val_show,
					reg_ctl_kobj_val_store),
		.kobj_attr_reg =
			__ATTR(register, 0644, reg_ctl_kobj_reg_show,
					reg_ctl_kobj_reg_store),
		.regmap = regmap,
	});

	/* create reg-ctl sysfs kobject */
	rc->kobj = kobject_create_and_add("reg-ctl", &dev->kobj);
	if (!rc->kobj) {
		dev_err(dev, "Failed to create kobject for %s", name);
		ret = -ENOMEM;
		goto err;
	}

	/* init val sysfs node */
	ret = sysfs_create_file(rc->kobj, &rc->kobj_attr_val.attr);
	if (WARN_ONCE(ret, "Creating sysfs file failed."))
		goto err_kobject;

	/* init reg sysfs node */
	ret = sysfs_create_file(rc->kobj, &rc->kobj_attr_reg.attr);
	if (WARN_ONCE(ret, "Creating sysfs file failed."))
		goto err_kobject;

	dev_info(dev, "Created sysfs devices for %s.", name);

	return rc;

err_kobject:
	kobject_put(rc->kobj);

err:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(reg_ctl_init);

void reg_ctl_free(struct reg_ctl *rc)
{
	kobject_put(rc->kobj);
}
EXPORT_SYMBOL(reg_ctl_free);