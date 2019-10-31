/*
 * config-eeprom.h: Interface to the D3 configuration-eeprom-reading driver.
 *
 * This header file does not rely on, or reference, any EEPROM driver
 * or EEPROM specifics.
 *
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

#ifndef _D3_CONFIG_EEPROM_H
#define _D3_CONFIG_EEPROM_H

#include <linux/types.h>

/// The size of the data.  TODO replace this with a more sophisticated approach.
#define CONFIG_EEPROM_DATA_SIZE (128)

/// The data read from the EEPROM.
struct config_eeprom_data {
	/// Parameters for drivers/d3/overlay-loader.
	char overlay_loader_params[CONFIG_EEPROM_DATA_SIZE];
} __attribute__ ((packed));

extern struct config_eeprom_data *config_eeprom_get_data(void);

#endif /* _D3_CONFIG_EEPROM_H */
