/*
 * reg_tbl.h - generalized functions for using performing regmap reads and
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

#ifndef REG_TBL_H
#define REG_TBL_H

#include <linux/types.h>
#include <linux/regmap.h>

enum reg_tbl_action_t {
	END_TABLE = 0,
	LOAD_SECTION = 1,
	WRITE_REG,
	SET_FIELD,
	DELAY_MS,
};

struct reg_tbl_t {
	enum reg_tbl_action_t action;
	const struct reg_tbl_t *section;
	u16 addr;
	u16 mask;
	u16 val;
};


int reg_tbl_write(struct regmap *regmap, const struct reg_tbl_t *table);


#endif /* !REG_TBL_H */
