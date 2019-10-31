/*
 * Copyright (c) 2019, D3 Engineering  All rights reserved.
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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#include <linux/nvmem-consumer.h>
#include <linux/of.h>

#include <d3/config-eeprom.h>

#define MODULE_NAME "config-eeprom"

/* Define to see a debugging dump of the first 16 bytes of the EEPROM
 * at bootup. */
//#define DEBUG_DUMP

/* The device-tree node holding the phandle of the EEPROM we want.
 * Defined in hardware/d3/d3-config-eeprom.dtsi. */
#define CONFIG_NODE_PATH ("/chosen/d3,config-eeprom@0")

// EEPROMs I know of:
// RSP: ST M24512, 64KByte
// CIC: 24AA1026, 128KByte

static struct config_eeprom_data *curr_data = NULL;

#define DEBUG_BUFSIZE (512)

/**
 * config_eeprom_load_from_eeprom() - Read the EEPROM.
 *
 * This is an internal function.
 *
 * Return: A pointer to the data, or NULL on failure.
 */

static struct config_eeprom_data *config_eeprom_load_from_eeprom(void)
{
	/// The device node holding the nvmem-names and nvmem properties
	struct config_eeprom_data *retval = NULL;
	struct device_node *config = NULL;
	struct nvmem_device *nvmem = NULL;

	do {	/* once - cleanup follows */

#ifdef DEBUG_DUMP
		unsigned char d[16];
		char str[DEBUG_BUFSIZE];
#endif //DEBUG_DUMP

		int len;

		config = of_find_node_by_path(CONFIG_NODE_PATH);
		if(!config)
			break;

		pr_info(MODULE_NAME ": Found config node %s (%s)\n",
				config->name, config->full_name);

		nvmem = of_nvmem_device_get(config, "overlay");
		if(IS_ERR(nvmem)) {
			pr_notice(MODULE_NAME ": Could not open NVMEM device: %lld\n", (long long)nvmem);
			nvmem = NULL;
			break;
		}

		pr_info(MODULE_NAME ": Found NVMEM %p\n", (void *)nvmem);

#ifdef DEBUG_DUMP
		/* Dump the first 16 bytes of the EEPROM.
		 * This is a debugging aid. */
		len = nvmem_device_read(nvmem, 0, 16, d);
		if(len<0) {
			pr_notice(MODULE_NAME ": Could not read data: error %d\n", len);
			break;
		}

		/* Brute force!  Made with Vim. */
		snprintf(str, DEBUG_BUFSIZE, "Got EEPROM data: "
				"%02x %02x %02x %02x "
				"%02x %02x %02x %02x  "
				"%02x %02x %02x %02x "
				"%02x %02x %02x %02x "
				"%c%c%c%c%c%c%c%c "
				"%c%c%c%c%c%c%c%c\n",
				d[0], d[1], d[2], d[3],
				d[4], d[5], d[6], d[7],
				d[8], d[9], d[10], d[11],
				d[12], d[13], d[14], d[15],
				(d[0] < 32 || d[0] >= 127) ? '.' : d[0],
				(d[1] < 32 || d[1] >= 127) ? '.' : d[1],
				(d[2] < 32 || d[2] >= 127) ? '.' : d[2],
				(d[3] < 32 || d[3] >= 127) ? '.' : d[3],
				(d[4] < 32 || d[4] >= 127) ? '.' : d[4],
				(d[5] < 32 || d[5] >= 127) ? '.' : d[5],
				(d[6] < 32 || d[6] >= 127) ? '.' : d[6],
				(d[7] < 32 || d[7] >= 127) ? '.' : d[7],
				(d[8] < 32 || d[8] >= 127) ? '.' : d[8],
				(d[9] < 32 || d[9] >= 127) ? '.' : d[9],
				(d[10] < 32 || d[10] >= 127) ? '.' : d[10],
				(d[11] < 32 || d[11] >= 127) ? '.' : d[11],
				(d[12] < 32 || d[12] >= 127) ? '.' : d[12],
				(d[13] < 32 || d[13] >= 127) ? '.' : d[13],
				(d[14] < 32 || d[14] >= 127) ? '.' : d[14],
				(d[15] < 32 || d[15] >= 127) ? '.' : d[15]
		);

		pr_info(MODULE_NAME ": %s\n", str);

#endif //DEBUG_DUMP

		retval = vmalloc(sizeof(struct config_eeprom_data));
		if(!retval)
			break;

		len = nvmem_device_read(nvmem, 0, CONFIG_EEPROM_DATA_SIZE,
					retval);
		if(len<0) {
			pr_notice("Could not read data: error %d\n", len);
			vfree(retval);
			retval = NULL;
			break;
		}

	} while(0);

	if(nvmem) {
		nvmem_device_put(nvmem);	// free nvmem
		nvmem = NULL;
	} else {
		pr_notice(MODULE_NAME ": could not find nvmem\n");
	}

	if(config) {
		of_node_put(config);
		config = NULL;
	} else {
		pr_notice(MODULE_NAME ": could not find config device-tree node\n");
	}

	return retval;
} //config_eeprom_load_from_eeprom

/**
 * config_eeprom_get_data() - retrieve the data loaded from the EEPROM.
 *
 * This function is the main interface to users of this module.  Any driver
 * initialized after this one can retrieve the data pointer using this
 * function.  Interpretation of the data is left to the driver user.
 *
 * Return: A pointer to the data, or NULL on failure.
 */

struct config_eeprom_data *config_eeprom_get_data(void)
{
	return curr_data;
}
EXPORT_SYMBOL_GPL(config_eeprom_get_data);

/**
 * config_eeprom_init() - startup routine - load the data
 *
 * Return: always 0
 */

static int __init config_eeprom_init(void)
{
	// __LINE__: Poor-man's double-check that we have the right file
	pr_info(MODULE_NAME ": Initializing " __stringify(__LINE__) "\n");
	curr_data = config_eeprom_load_from_eeprom();

	return 0;
}

/**
 * config_eeprom_exit() - shutdown routine - free the data
 *
 * Return: A pointer to the data, or NULL on failure.
 */

static void __exit config_eeprom_exit(void)
{
	pr_info(MODULE_NAME ": Terminating " __stringify(__LINE__) "\n");
	if(curr_data) {
		vfree(curr_data);
		curr_data = NULL;
	}
}

/* Initialize at level 5 (fs).  This is after subsys (#4), so nvmem is
 * available, and is before device (#6), when devices are initialized based on
 * the DTB state. */

fs_initcall(config_eeprom_init);
module_exit(config_eeprom_exit);

MODULE_DESCRIPTION("Driver that accesses a configuration EEPROM.");
MODULE_AUTHOR("Christopher White <cwhite@d3engineering.com>");
MODULE_LICENSE("GPL v2");
