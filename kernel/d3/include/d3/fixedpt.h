/**
 * Include file for S42.22 fixed-point support
 *
 * Copyright (c) 2020, D3 Engineering.  All rights reserved.
 * By Christopher White <cwhite@d3engineering.com>
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
#ifndef _D3_FIXEDPT_H
#define _D3_FIXEDPT_H

/* fixedptc configuration */
#define FIXEDPT_BITS (64)
#define FIXEDPT_WBITS (42)

/* Include the interface unless FIXEDPTC_IMPLEMENTATION is defined. */
#include <d3/fixedptc.h>

#endif /* _D3_FIXEDPT_H */
