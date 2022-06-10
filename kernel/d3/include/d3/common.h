/**
 * Common definitions for D3 drivers.
 *
 * Copyright (c) 2019-2021, D3 Engineering.  All rights reserved.
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
#ifndef _D3_COMMON_H
#define _D3_COMMON_H

#include <linux/bitfield.h>
#include <linux/types.h>

/* === Error-reporting macros ============================================== */

/**
 * If @p expr evalutes to non-zero assign it to @p err and return @p err
 */
#define TRY(err, expr) do {\
		err = expr; \
		if (err) { \
			return err; \
		} \
	} while (false)

/**
 * If @p expr evalutes to non-zero assign it to @p err, call dev_err, and return @p err
 */
#define TRY_DEV_MSG(err, expr, dev) do {\
		err = expr; \
		if (err) { \
			dev_err((dev), "%s failure (%d)", #expr, (err)); \
			return (err); \
		} \
	} while (false)

/**
 * If @p expr evalutes to non-zero assign it to @p err, call dev_err, and return @p err.
 * Takes a @p name for use in the error message.
 */
#define TRY_DEV_MSG2(err, expr, dev, name) do {\
		err = expr; \
		if (err) { \
			dev_err((dev), "%s failure (%d) in %s", \
				(name), (err), #expr); \
			return (err); \
		} \
	} while (false)

/**
 * If @p expr evalutes to non-zero assign it to @p err, call dev_err, and return @p err.
 * Takes a @p name for use in the error message.
 */
#define TRY_DEV_MSG2(err, expr, dev, name) do {\
		err = expr; \
		if (err) { \
			dev_err((dev), "%s failure (%d) in %s", \
				(name), (err), #expr); \
			return (err); \
		} \
	} while (false)

/**
 * If @p expr evalutes to non-zero assign it to @p err, call dev_err, and return @p err.
 * If @p expr evaluates to 0, print a success message.
 * Takes a file/line/func for use in the error message.
 */
#define TRY_DEV_MSG3(err, expr, dev, filename, lineno, funcname) \
	do { \
		err = expr; \
		if (err) { \
			dev_err((dev), \
				"%s failed (%d) in %s() at %s:%d", #expr, \
				(err), (funcname), (filename), (lineno)); \
			return (err); \
		} \
	} while (false)

/**
 * If @p expr evalutes to non-zero, assign it to @p err, call dev_err, an0d
 * return @p err.  Prints the name and line number of the calling function.
 */
#define TRY_DEV_HERE(err, expr, dev) \
	TRY_DEV_MSG3((err), (expr), (dev), __FILE__, __LINE__, __func__)
/**
 * As TRY, but for an @p expr that returns a pointer.
 */
#define TRY_MEM(mem, expr) do {\
		mem = expr; \
		if (IS_ERR(mem)) { \
			return PTR_ERR(mem); \
		} \
	} while (false)


/**
 * As TRY_MEM, but with dev_err
 */
#define TRY_MEM_DEV_MSG(mem, expr, dev, name) do {\
		mem = expr; \
		if (IS_ERR(mem)) { \
			dev_err((dev), "%s failure (%ld)", (name), PTR_ERR(mem)); \
			return PTR_ERR(mem); \
		} \
	} while (false)


/**
 * As TRY, but unlocking @p mutex before returning on error.
 */
#define TRY_MUTEX(mutex, err, expr) do {\
		err = expr; \
		if (err) { \
			mutex_unlock(mutex); \
			return err; \
		} \
	} while (false)

/**
 * struct d3_reg_table - a sized array of reg_sequence
 * @reg_sequence: the registers.  A NULL terminator is not required.
 * @size: The number of registers in @reg_sequence.
 *
 * The values are held in a &struct reg_sequence.
 */
struct d3_reg_table {
	const struct reg_sequence *reg_sequence; /* addr, val, delay_us */
	size_t size;
};

/**
 * WRITE_REG_TABLE() - write a d3_reg_table to a device
 * @map: The register map
 * @table: The &struct d3_reg_table
 *
 * Uses regmap_multi_reg_write().
 */
#define WRITE_REG_TABLE(map, table) \
	regmap_multi_reg_write((map), (table).reg_sequence, (table).size)

/**
 * STRUCT_REG_TABLE() - declare and initialize a d3_reg_table
 * @table_name: the name of the table to declare
 * @reg_sequence: a &struct reg_sequence of the values
 *
 * If you want ``static`` or ``const``, specify them outside the
 * STRUCT_REG_TABLE() call.  E.g.,
 *
 * ``static const STRUCT_REG_TABLE(...);``
 */
#define STRUCT_REG_TABLE(table_name, reg_sequence) \
	struct d3_reg_table table_name = { \
		(reg_sequence), ARRAY_SIZE((reg_sequence)) \
	}

/* === Bitfield macros ===================================================== */

/**
 * BYTENOF - return the nth byte of x
 *
 * n must be a compile-time constant.
 */
#define BYTENOF(n, x)	FIELD_GET(GENMASK((n)*8+7, (n)*8), (x))

/* Convenience invokers for the first four bytes */
#define BYTE0OF(x)	BYTENOF(0, x)
#define BYTE1OF(x)	BYTENOF(1, x)
#define BYTE2OF(x)	BYTENOF(2, x)
#define BYTE3OF(x)	BYTENOF(3, x)

#endif /* _D3_COMMON_H */
