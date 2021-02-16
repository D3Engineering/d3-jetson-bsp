/*
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

#include "omelette-decode.h"
#include "omelette-decode-internal.h"

/* the alignment of an Omelette record in EEPROM.  Is a power of 2, so we can
 * use round_up(). */
#define OMELETTE_ALIGN (4)

/* Required flag byte */
#define OMELETTE_FLAG ((uint8_t)(0xc1))

/* Number of bytes in a header */
#define OMELETTE_HDR_NBYTES (12)

/* Number of bytes in the CRC field */
#define OMELETTE_CRC_NBYTES (4)

/* Maximum number of data bytes in a record. */
#define OMELETTE_MAX_DATA_LEN ((uint32_t)0xffff0000)

/* Magic number for a valid CRC32 */
#define OMELETTE_CRC_OK (0x2144df1c)
/**
 * struct omelette_record_raw - Omelette record as read from EEPROM
 * @type_flag: 0xc1
 * @type_tag: three-byte tag
 * @crc_n: CRC32 (NETWORK byte order)
 * @data_length_n: number of valid data bytes (NETWORK byte order)
 * @payload: @data_length_n bytes of data, padded with \x00 to a 32-bit boundary
 */
struct omelette_record_raw {
	uint8_t  type_flag;
	uint8_t  type_tag[3];
	uint32_t crc_n;
	uint32_t data_length_n;
	uint8_t  payload[]; // Will contain data read from EEPROM record
} __attribute__((packed, aligned(OMELETTE_ALIGN)));

/* Dummy function to hold compile-time checks */
static void __maybe_unused
	    omelette_compile_time_checks(void)
{
	/* Check for correct structure layout */
	BUILD_BUG_ON(sizeof(struct omelette_record_raw) != OMELETTE_HDR_NBYTES);

	/* read_record() assumes size_t has at least 64 bits, so can represent
	 * the full range of uint32_t+uint32_t. */
	BUILD_BUG_ON(sizeof(size_t) < 2 * sizeof(uint32_t));

	/* Make sure a record+trailing CRC can fit in 32 bits */
	BUILD_BUG_ON(((__s64)OMELETTE_HDR_NBYTES + OMELETTE_MAX_DATA_LEN +
		      OMELETTE_CRC_NBYTES) > (__s64)((uint32_t)-1));
}

/**
 * read_record() - Read a single record, if possible.
 * @nvmem: as decode_omelette_eeprom_data()
 * @max_bytes_to_read: as decode_omelette_eeprom_data()
 * @starting_byte_offset: where in the NVMEM to start looking
 * @complete: Is set false if an error prevents determining whether a
 *            platter is complete.
 * @nbytes_consumed: is set to the number of bytes consumed during this call
 *                   to the function.  Always written; set to 0 on failure.
 *
 * This function never sets @complete true, so the same @complete value can be
 * passed to multiple consecutive calls to this function.
 *
 * Precondition: ( max_bytes_to_read - starting_byte_offset
 *                   - OMELETTE_HDR_NBYTES ) >= 0
 *
 * Return: the record, on success, or NULL, if there was no record to read
 *         (e.g., the platter is over).
 */
struct omelette_record_raw *
read_record(struct nvmem_device *nvmem, const uint32_t max_bytes_to_read,
	    const uint32_t starting_byte_offset, bool *complete,
	    uint32_t *nbytes_consumed)
{
	struct omelette_record_raw *retval   = NULL;
	struct omelette_record_raw *curr_rec = NULL;
	struct omelette_record_raw  header;
	uint32_t record_nbytes; /* header through padding, inclusive */
	uint32_t data_nbytes; /* length field in the record */
	__s64 bytes_rcvd;

	do { // once

		*nbytes_consumed = 0;

		/* Read and check the header */
		bytes_rcvd = nvmem_device_read(nvmem, starting_byte_offset,
					       OMELETTE_HDR_NBYTES, &header);
		if (bytes_rcvd < OMELETTE_HDR_NBYTES) {
			pr_notice("Could not read record header: error %lld\n",
				  bytes_rcvd);
			*complete = false;
			break;
		}

		if (header.type_flag != OMELETTE_FLAG) {
			/* Not Omelette format, so the platter is done */
			break;
		}

		/* Sanity checks: if the full record takes us off the end
		 * of the EEPROM, we can't read it. */
		data_nbytes = ntohl(header.data_length_n);

		if (data_nbytes > OMELETTE_MAX_DATA_LEN) {
			/* Not a valid Omelette record */
			break;
		}

		/* Check for bogus values before doing any math on
		 * data_nbytes.  The subtraction is so we don't have to worry
		 * about wraparound (see Precondition) */
		if (data_nbytes > (max_bytes_to_read - starting_byte_offset -
				   OMELETTE_HDR_NBYTES)) {
			/* The record might be valid, but we can't know without
			 * reading past the end of the EEPROM.  Therefore,
			 * signal incomplete. */
			*complete = false;
			break;
		}

		/* The max-length check above means this won't wrap around. */
		record_nbytes = OMELETTE_HDR_NBYTES +
				round_up((size_t)data_nbytes, OMELETTE_ALIGN);

		/* Check for out-of-range adjusted values.  We know that
		 * (record_nbytes + starting_byte_offset) will fit within a
		 * size_t since each is uint32_t and we checked the size of
		 * size_t at compile time. */
		if (((size_t)record_nbytes + starting_byte_offset) >
		    (size_t)max_bytes_to_read) {
			/* It's incomplete because we can't know if we finished
			 * a platter. */
			*complete = false;
			break;
		}

		/* Read the full record.  Re-read the header for simplicity.
		 * The max-length check above means that record_nbytes +
		 * OMELETTE_CRC_NBYTES is < 2^32-1. */
		curr_rec = vmalloc(record_nbytes + OMELETTE_CRC_NBYTES);
		if (!curr_rec) {
			pr_notice("Could not allocate memory for record");
			*complete = false;
			break;
		}

		bytes_rcvd = nvmem_device_read(nvmem, starting_byte_offset,
					       record_nbytes, curr_rec);

		if (bytes_rcvd < (__s64)record_nbytes) {
			pr_notice(
				"Could not read %u bytes of data data: error %lld\n",
				record_nbytes, bytes_rcvd);
			*complete = false;
			break;
		}

		/* Take CRC out of record and move it to the end of the buffer
		 */
		*(uint32_t *)((uint8_t *)curr_rec + record_nbytes) =
			cpu_to_le32(be32_to_cpu(curr_rec->crc_n));
		curr_rec->crc_n = 0;

		/* make sure the calculated CRC matches stored CRC */
		if ((crc32_le(~0, (unsigned char const *)curr_rec,
			      record_nbytes + OMELETTE_CRC_NBYTES) ^
		     ~(uint32_t)0) != OMELETTE_CRC_OK) {
			pr_warn("Invalid record in EEPROM at offset %u",
				   starting_byte_offset);
			/* The platter is over, since this record's CRC was
			 * invalid.  However, this is not an error, since
			 * a platter by definition stops before a record
			 * with an invalid CRC.  After all, this might
			 * be a non-record that happens to start with 0xc1.
			 * Therefore, do not mark as incomplete. */
			break;
		}

		/* Report success */
		retval		 = curr_rec;
		curr_rec	 = NULL;
		*nbytes_consumed = record_nbytes;

	} while (0);

	/* Cleanup after a failure */
	if (curr_rec) {
		vfree(curr_rec);
		curr_rec = NULL;
	}

	return retval;
} // read_record

struct omelette_platter *
decode_omelette_eeprom_data(struct nvmem_device *nvmem,
			    const uint32_t		 max_bytes_to_read)
{
	/* The values to fill into the returned structure */
	struct omelette_platter *   retval   = NULL;
	struct omelette_record_raw *curr_rec = NULL;
	struct omelette_record *    output   = NULL;
	uint32_t bytes_read = 0; /* == starting offset of the next record */
	uint32_t datalen, alloclen;
	uint32_t nbytes_consumed;

	if(!nvmem) {
		pr_warn("No nvmem structure");
		return NULL;
	}

	if(max_bytes_to_read == 0) {
		pr_warn("Cannot read from 0-byte structure");
		return NULL;
	}

	/* Initialize the return structure */
	retval = vzalloc(sizeof(*retval));
	if (!retval) {
		pr_warn("Could not allocate platter header");
		return NULL;
	}
	INIT_LIST_HEAD(&retval->records);
	retval->complete = true;

	/* Main loop: look for records until a record would take us past
	 * the length limit. */
	while ((bytes_read + OMELETTE_HDR_NBYTES) <= max_bytes_to_read) {
		curr_rec = read_record(nvmem, max_bytes_to_read, bytes_read,
				       &retval->complete, &nbytes_consumed);
		if (!curr_rec) {
			/* Couldn't read another record, so we're done */
			break;
		}
		datalen = ntohl(curr_rec->data_length_n);

		/* build a struct omelette_record */
		alloclen = sizeof(*output) + datalen;
		output   = vzalloc(alloclen);
		if (!output) {
			retval->complete = false;
			break;
		}
		memcpy(output->type_tag, curr_rec->type_tag,
		       OMELETTE_TAG_BYTES);
		output->data_length = datalen;
		memcpy(output->payload, curr_rec->payload, datalen);

		/* Add it to the list */
		INIT_LIST_HEAD(&output->list);
		list_add_tail(&output->list, &retval->records);

		/* Record that we successfully read a record (!) */
		bytes_read += nbytes_consumed;
		++retval->nrecords;
		output = NULL;
	}

	/* Cleanup after failure */
	if (curr_rec) {
		vfree(curr_rec);
		curr_rec = NULL;
	}

	if (output) {
		vfree(output);
		output = NULL;
	}

	/* Report */
	retval->total_bytes = bytes_read;
	return retval;
}
EXPORT_SYMBOL_GPL(decode_omelette_eeprom_data);
