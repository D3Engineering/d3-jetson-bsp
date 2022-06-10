/*
 * cam_rw.h - generalized functions for using performing regmap reads and
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

#ifndef CAM_RW_H
#define CAM_RW_H

enum cam_rw_action_t {
	END_TABLE = 0,
	LOAD_SECTION = 1,
	WRITE_REG,
	SET_FIELD,
	DELAY_MS,
};

struct cam_rw_table_t {
	enum cam_rw_action_t action;
	const struct cam_rw_table_t *section;
	u16 addr;
	u16 mask;
	u16 val;
};


int cam_rw_read_reg(struct camera_common_data *s_data, u16 addr, u16 *val);
int cam_rw_write_reg(struct camera_common_data *s_data, u16 addr, u16 val);
int cam_rw_update_bits(struct camera_common_data *s_data, u16 addr, u16 mask, u16 val);
int cam_rw_write_table(struct camera_common_data *s_data, const struct cam_rw_table_t *table);


#endif /* !CAM_RW_H */
