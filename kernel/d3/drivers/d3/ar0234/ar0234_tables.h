/*
 * ar0234_tables.h - ar0234 sequence tables
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

#ifndef AR0234AT_H_
#define AR0234AT_H_

#include <d3/reg_tbl.h>

extern const struct reg_tbl_t ar0234_start[];
extern const struct reg_tbl_t ar0234_stop[];
extern const struct reg_tbl_t ar0234_1200p_110fps[];
extern const struct reg_tbl_t ar0234_1200p_120fps[];
extern const struct reg_tbl_t ar0234_1200p_60fps[];
extern const struct reg_tbl_t ar0234_1080p_60fps[];
extern const struct reg_tbl_t ar0234_1080p_30fps[];
extern const struct reg_tbl_t ar0234_720p_60fps[];
extern const struct reg_tbl_t ar0234_480p_120fps[];
extern const struct reg_tbl_t ar0234_720p_120fps[];


#endif // AR0234AT_H_
