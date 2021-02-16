/* sanity.c: Sanity checks for config-eeprom
 *
 * Copyright (c) 2020, D3 Engineering.  All rights reserved.
 * Author: Christopher White <cwhite@d3engineering.com>
 * Author: Robert Kurdziel <rkurdziel@d3engineering.com>
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

#ifdef __KERNEL__
#error "This is a usermode test case"
#endif

#include "omelette-decode.h"
#include "omelette-decode-internal.h"

void test_nvmem_device_read_invalid(void)
{
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		nvmem_device_read(NULL, 0, 0, NULL)
	);
}

void test_nvmem_device_read_segfault(void)
{
	struct nvmem_device dev;
	dev.nbytes = 24;
	dev.data[24];
	uint8_t buf[28];

	TEST_ASSERT_EQUAL_INT(-EFAULT,
		nvmem_device_read(&dev, 24, 4, buf)
	);
	TEST_ASSERT_EQUAL_INT(-EFAULT,
		nvmem_device_read(&dev, -1, 0, buf)
	);
}

void test_nvmem_device_read_success(void)
{
	// Need to allocate memory for arrays
	struct nvmem_device* dev = malloc(sizeof(struct nvmem_device) + 24);
	dev->nbytes = 24;
	dev->data[24];
	uint8_t* buf = malloc(24);

	memset(dev->data, 1, 24);
	memset(buf, 2, 24);
	dev->data[23] = 9;

	nvmem_device_read(dev, 0, 1, buf);

	TEST_ASSERT_EQUAL_INT(dev->data[0], buf[0]);

	// Make sure only 1 byte was read
	TEST_ASSERT_EQUAL_INT(2, buf[1]);

	nvmem_device_read(dev, 23, 1, buf);

	TEST_ASSERT_EQUAL_INT(dev->data[23], buf[0]);

	// Make sure the rest of buf is unchanged
	TEST_ASSERT_EQUAL_INT(2, buf[1]);
	TEST_ASSERT_EQUAL_INT(2, buf[23]);
	free(dev);
	free(buf);
}

void test_crc32_le_valid(void)
{
	// Test record, contains A A A for tag, zero bytes for CRC,
	// 4 for the data length, and 3 zero bytes and a 1 for the data
	// no padding since it is already 4 byte aligned.
	unsigned char test_p[16] = {0xc1, 0x41, 0x41, 0x41,
				    0x00, 0x00, 0x00, 0x00,
				    0x00, 0x00, 0x00, 0x04,
				    0x00, 0x00, 0x00, 0x01};
	size_t test_len = 16;

	// This is the output of an online calculator for the record above
	uint32_t expect_crc = 0x7D7AA9A0;

	TEST_ASSERT_EQUAL_HEX(expect_crc,
		crc32_le(~0, test_p, test_len) ^ ~(uint32_t)0
	);
}

void test_crc32_le_verify(void)
{
	// Test record, contains A A A for tag, zero bytes for CRC,
	// 4 for the data length, and 3 zero bytes and a 1 for the data
	// + the calculated CRC from above, in reverse byte order.
	unsigned char test_p[20] = {0xc1, 0x41, 0x41, 0x41,
				    0x00, 0x00, 0x00, 0x00,
				    0x00, 0x00, 0x00, 0x04,
				    0x00, 0x00, 0x00, 0x01,
				    0xa0, 0xa9, 0x7a, 0x7d};
	size_t test_len = 20;

	// Magic number from Omelette spec
	uint32_t expect_crc = 0x2144df1c;

	TEST_ASSERT_EQUAL_HEX(expect_crc,
		crc32_le(~0, test_p, test_len) ^ ~(uint32_t)0
	);
}

void setUp(void) {}
void tearDown(void) {}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_nvmem_device_read_invalid);
	RUN_TEST(test_nvmem_device_read_segfault);
	RUN_TEST(test_nvmem_device_read_success);
	RUN_TEST(test_crc32_le_valid);
	RUN_TEST(test_crc32_le_verify);
	return UNITY_END();
}
