/*
 * omelette-decode.h: Interface to the D3 EEPROM Omelette data decoding driver.
 *
 * This header file does not rely on, or reference, any EEPROM driver
 * or EEPROM specifics.
 *
 * Copyright (c) 2020, D3 Engineering.  All rights reserved.
 * Author: Robert Kurdziel <rkurdziel@d3engineering.com>
 * Author: Christopher White <cwhite@d3engineering.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#ifndef _D3_OMELETTE_DECODE_H
#define _D3_OMELETTE_DECODE_H

/* ------------------------------------------------------------------------*/
/* Permit use with the test code in the D3 BSP */
#ifdef __KERNEL__

#include <linux/list.h>
#include <linux/nvmem-consumer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/bug.h>
#include <linux/crc32.h>

#else

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "_kernel.h"
#include "_list.h"

/**
 * struct nvmem_device - shim for use by userspace test code.
 * @nbytes: number of bytes
 * @data: Data
 *
 * This fakes out an nvmem device using a fixed buffer.
 */
struct nvmem_device {
	size_t nbytes;
	uint8_t data[];
};

#endif

/* ------------------------------------------------------------------------*/
/* Common definitions */

#define OMELETTE_TAG_BYTES (3)

/**
 * struct omelette_record - Data from one Omelette record
 * @list: Pointers to next/previous records in a platter, if any
 * @tag: three-byte tag
 * @data_length: number of valid data bytes
 * @payload: @data_length bytes of data
 *
 * This structure is independent of the layout of the Omelette records
 * in EEPROM.  This is to reduce the chance that client code will rely
 * on details in the implementation of the reader.
 *
 * The memory allocation for an omelette_record may include more bytes
 * than required to hold @data_length bytes of data.  Clients may not make
 * any assumptions about whether or not that is the case, and may not use
 * any extra allocated bytes.
 */
struct omelette_record {
	struct list_head list;
	uint8_t		 type_tag[OMELETTE_TAG_BYTES];
	uint32_t		 data_length;
	uint8_t		 payload[];
};

/**
 * struct omelette_platter - an Omelette platter as read from NVMEM
 * @records: linked list of &struct omelette_record
 * @total_bytes: total bytes occupied on the storage medium by this platter
 * @nrecords: number of records in @records
 * @complete: true if decode_omelette_eeprom_data() read a complete platter
 *            without error.  If this is false, there may be additional records
 *            on the EEPROM.
 *
 * This structure holds a linked list of records read from EEPROM.
 */
struct omelette_platter {
	struct list_head records;
	uint32_t		 total_bytes;
	uint32_t		 nrecords;
	bool		 complete;
};

/**
 * decode_omelette_eeprom_data() - Reads/decodes eeprom data in omelette format.
 * @nvmem: nvmem device to read
 * @max_bytes_to_read: maximum number of bytes to read from @nvmem
 *
 * Per the Omelette specification, the last byte of a platter is the byte before
 * the first byte that cannot be interpreted as the beginning of a record.
 * This function stops reading bytes when:
 * (1) the platter is at its end per the spec; or
 * (2) an invalid record is read
 *
 * Most error conditions will still return non-NULL, but will leave the
 * complete field false in the result.  For example, if reading a record would
 * require reading past @max_bytes_to_read, that record is omitted from the
 * returned record list and complete is set false.
 *
 * If @nvmem can be read, but the data are not a valid Omelette platter, the
 * function will succeed but return a list with no records.
 *
 * Return: vmalloc()ed platter struct, or NULL on hard failure
 *         (e.g., out of memory).
 */
struct omelette_platter *
decode_omelette_eeprom_data(struct nvmem_device *nvmem,
			    const uint32_t		 max_bytes_to_read);

#endif
