/* omelette-decode.c: Sanity checks for omelette-decode.c
 *
 * Copyright (c) 2020, D3 Engineering.  All rights reserved.
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

#include <UnityTest/unity.h>
#include <errno.h>
#include <string.h>

#ifdef __KERNEL__
#error "This is a usermode test case"
#endif

#include "omelette-decode.h"
#include "_list.h"

void test_decode_invalid_calls(void)
{
	struct nvmem_device dev;
	TEST_ASSERT_EQUAL_HEX(0, decode_omelette_eeprom_data(NULL, 42) );
	TEST_ASSERT_EQUAL_HEX(0, decode_omelette_eeprom_data(&dev, 0) );
}

void test_read_record_valid(void)
{
	struct nvmem_device dev;
	bool complete = false;
	uint32_t nbytes_consumed = 0;

	dev.nbytes = 16;
	dev.data[16];
	// Test record, contains A A A for tag, precalculated CRC,
	// 4 for the data length, and 3 zero bytes and a 1 for the data
	// no padding since it is already 4 byte aligned.
	memcpy(dev.data, (uint8_t[]) {0xc1, 0x41, 0x41, 0x41,
				      0x7d, 0x7a, 0xa9, 0xa0,
				      0x00, 0x00, 0x00, 0x04,
				      0x00, 0x00, 0x00, 0x01}, 16);

	struct omelette_platter* test_platter =
		decode_omelette_eeprom_data(&dev, 16);

	// Verify that all parts of the read platter are correct
	TEST_ASSERT_EQUAL_INT(16, test_platter->total_bytes);
	TEST_ASSERT_EQUAL_INT(1, test_platter->nrecords);
	TEST_ASSERT_TRUE(test_platter->complete);

	// Verify that the record has the correct data
	struct list_head *test_list;
	struct omelette_record *test_record =
			list_entry(test_platter->records.next, struct omelette_record, list);

	TEST_ASSERT_EQUAL_HEX(0x41, test_record->type_tag[0]);
	TEST_ASSERT_EQUAL_HEX(0x41, test_record->type_tag[1]);
	TEST_ASSERT_EQUAL_HEX(0x41, test_record->type_tag[2]);

	TEST_ASSERT_EQUAL_HEX(0x00000004, test_record->data_length);

	TEST_ASSERT_EQUAL_HEX(0x00, test_record->payload[0]);
	TEST_ASSERT_EQUAL_HEX(0x00, test_record->payload[1]);
	TEST_ASSERT_EQUAL_HEX(0x00, test_record->payload[2]);
	TEST_ASSERT_EQUAL_HEX(0x01, test_record->payload[3]);
}

void test_read_record_valid_and_invalid(void)
{
	struct nvmem_device dev;
	bool complete = false;
	uint32_t nbytes_consumed = 0;

	dev.nbytes = 32;
	dev.data[32];
	// Test record, contains A A A for tag, precalculated CRC,
	// 4 for the data length, and 3 zero bytes and a 1 for the data
	// no padding since it is already 4 byte aligned.
	// Then an Omelette record with an invalid CRC
	memcpy(dev.data, (uint8_t[]) {0xc1, 0x41, 0x41, 0x41,
				      0x7d, 0x7a, 0xa9, 0xa0,
				      0x00, 0x00, 0x00, 0x04,
				      0x00, 0x00, 0x00, 0x01,
				      0xc1, 0xe7, 0xc7, 0x11,
				      0x89, 0x27, 0x30, 0x12,
				      0x00, 0x00, 0x00, 0x04,
				      0x50, 0xff, 0x8c, 0x14}, 32);

	struct omelette_platter* test_platter =
		decode_omelette_eeprom_data(&dev, 32);

	struct omelette_record *test_record =
			list_entry(test_platter->records.next, struct omelette_record, list);

	// Make sure only the valid record was kept, and was read correctly
	TEST_ASSERT_TRUE(list_is_singular(&test_platter->records));

	TEST_ASSERT_EQUAL_HEX(0x41, test_record->type_tag[0]);
	TEST_ASSERT_EQUAL_HEX(0x41, test_record->type_tag[1]);
	TEST_ASSERT_EQUAL_HEX(0x41, test_record->type_tag[2]);

	TEST_ASSERT_EQUAL_HEX(0x00000004, test_record->data_length);

	TEST_ASSERT_EQUAL_HEX(0x00, test_record->payload[0]);
	TEST_ASSERT_EQUAL_HEX(0x00, test_record->payload[1]);
	TEST_ASSERT_EQUAL_HEX(0x00, test_record->payload[2]);
	TEST_ASSERT_EQUAL_HEX(0x01, test_record->payload[3]);

}

void setUp(void) {}
void tearDown(void) {}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_decode_invalid_calls);
	RUN_TEST(test_read_record_valid);
	RUN_TEST(test_read_record_valid_and_invalid);
	return UNITY_END();
}
