/*
 * reg_tbl.c - generalized functions for using performing regmap reads and
 *            writes with camera_common_data.
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


#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include <d3/reg_tbl.h>

static int reg_tbl_exec(struct device *dev, struct regmap *regmap,
	const struct reg_tbl_t *t_act, int depth, int index);
static int reg_tbl_main(struct device *dev, struct regmap *regmap,
	const struct reg_tbl_t *table, int depth);

/*
 * Helper function for write_table.
 * Performs the current action specified by the current pointer into a
 * reg_tbl_t.
 * Returns error value.
 */
static int reg_tbl_exec(struct device *dev, struct regmap *regmap,
		const struct reg_tbl_t *t_act, int depth, int index)
{
	int ret = 0;
	switch (t_act->action) {
		case LOAD_SECTION:
			dev_dbg(dev, "write_table %d.%d: Load nested table %p",
					depth, index, t_act->section);
			ret = reg_tbl_main(dev, regmap,
				t_act->section, depth + 1);
			break;

		case WRITE_REG:
			ret = regmap_write(regmap, t_act->addr, t_act->val);
			break;

		case SET_FIELD:
			ret = regmap_update_bits(regmap, t_act->addr,
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
 * Executes a series of writes specified by a reg_tbl_t.
 * Counts the depth of calls.
 * Returns error value.
 */
static int reg_tbl_main(struct device *dev, struct regmap *regmap,
		const struct reg_tbl_t *table, int depth)
{
	int ret = 0;
	int index = 0;

	while (table->action != END_TABLE) {
		ret = reg_tbl_exec(dev, regmap, table, depth,
			index);
		if (ret < 0)
			goto table_error;

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
 * Executes a series of writes specified by a reg_tbl_t.
 * Returns error value.
 */
int reg_tbl_write(struct regmap *regmap,
		const struct reg_tbl_t *table)
{
	struct device *dev = regmap_get_device(regmap);

	dev_dbg(dev, "write_table start new table");

	return reg_tbl_main(dev, regmap, table, 0);
}
EXPORT_SYMBOL(reg_tbl_write);
