/*
 * cam_rw.c - generalized functions for using performing regmap reads and
 *            writes with camera_common_data.
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


#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

#include "cam_rw.h"

/*
 * Wrapper around regmap_read.
 * Expects 16 bit addressing and value.
 * Returns error value.
 */
int cam_rw_read_reg(struct camera_common_data *s_data, u16 addr, u16 *val)
{
	int ret = 0;
	u32 reg_val = 0;
	struct device *dev = s_data->dev;

	ret = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFFFF; // mask for 16 bit val
	dev_info(dev, "i2c read, 0x%x(reg), 0x%x(val)", addr, *val);
	return ret;
}

/*
 * Wrapper around regmap_write.
 * Expects 16 bit addressing and value.
 * Returns error value.
 */
int cam_rw_write_reg(struct camera_common_data *s_data, u16 addr, u16 val)
{
	int ret = 0;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "i2c write, 0x%x(reg), 0x%x(val)", addr, val);
	ret = regmap_write(s_data->regmap, addr, val);
	if (ret)
		dev_err(dev, "i2c write failure, 0x%x(reg), 0x%x(val)",
			addr, val);
	return ret;
}

/*
 * Wrapper around regmap_update_bits.
 * Expects 16 bit addressing, mask, and value.
 * Returns error value.
 */
int cam_rw_update_bits(struct camera_common_data *s_data,
				u16 addr, u16 mask, u16 val)
{
	dev_dbg(s_data->dev, "i2c update bits, "
			"0x%x(reg), 0x%x(mask), 0x%x(val)",
			addr, mask, val);
	return regmap_update_bits(s_data->regmap, addr, mask, val);
}

/* Function prototype of write_table_main for use in execute_t_action */
static int cam_rw_write_table_main(struct camera_common_data*,
			const struct cam_rw_table_t*, int);
/*
 * Helper function for write_table.
 * Performs the current action specified by the current pointer into a
 * cam_rw_table_t.
 * Returns error value.
 */
static int cam_rw_execute_t_action(struct camera_common_data *s_data,
		const struct cam_rw_table_t *t_act, int depth, int index)
{
	int ret = 0;
	struct device *dev = s_data->dev;
	switch (t_act->action) {
		case LOAD_SECTION:
			dev_dbg(dev, "write_table %d.%d: Load nested table %p",
					depth, index, t_act->section);
			ret = cam_rw_write_table_main(s_data, t_act->section,
					depth + 1);
			break;
		case WRITE_REG:
			ret = cam_rw_write_reg(s_data, t_act->addr, t_act->val);
			break;
		case SET_FIELD:
			ret = cam_rw_update_bits(s_data, t_act->addr,
					t_act->mask, t_act->val);
			break;
		case DELAY_MS:
			dev_dbg(dev, "write_table %d.%d: Write Table delay %d",
					depth, index, t_act->val);
			msleep(t_act->val);
			break;
		case END_TABLE:
			dev_dbg(dev, "write_table %d.%d: Finish table",
					depth, index);
			break;
	}
	return ret;
};

/*
 * Executes a series of writes specified by a cam_rw_table_t.
 * Counts the depth of calls.
 * Returns error value.
 */
static int cam_rw_write_table_main(struct camera_common_data *s_data,
		const struct cam_rw_table_t *table, int depth)
{
	int ret = 0;
	int index = 0;
	struct device *dev = s_data->dev;

	while (table->action != END_TABLE) {
		ret = cam_rw_execute_t_action(s_data, table, depth, index);
		if (ret < 0) {
			goto table_error;
		}
		index++;
		table++;
	}
	return ret;

table_error:
	dev_err(dev, "Fail in table write, %d(depth) %d(index)", depth, index);
	return ret;
}

/*
 * Helper for am_rw_write_table_main for external use.
 * Executes a series of writes specified by a cam_rw_table_t.
 * Returns error value.
 */
int cam_rw_write_table(struct camera_common_data *s_data,
		const struct cam_rw_table_t *table)
{
	dev_dbg(s_data->dev, "write_table start new table");
	return cam_rw_write_table_main(s_data, table, 0);
}
