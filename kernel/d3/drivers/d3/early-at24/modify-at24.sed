# modify-at24.sed: sed script to modify at24.c to produce early-at24.c.
# Run with `sed -E`.
#
# Copyright (c) 2019, D3 Engineering  All rights reserved.
# Author: Christopher White <cwhite@d3engineering.com>
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.

# Identifiers: change prefix at24_
s|\bat24_|early_at24_|g

# Exceptions
s|\bearly_at24_platform_data\b|at24_platform_data|g

# Kernel messages
s|\bat24:|early_at24:|g

# Kernel command-line parameters
s|\bio_limit\b|early_at24_io_limit|g
s|\bwrite_timeout\b|early_at24_write_timeout|g

# The I2C device IDs.  Not a global search - only replace the first " on
# each line.
/i2c_device_id.*\[\]/,/^\s*\};/s|"|"early|

# I2C driver name
s|\.name\s*=\s*"at24"|.name = "early_at24"|

# Cause the early_at24 driver to be loaded early at level 5 (fs).
# This is after subsys (#4), so nvmem is available, and is before device (#6),
# when devices are initialized based on the DTB state. */

s|^\s*module_init\(|fs_initcall\(|

# Change the module description
s|^\s*MODULE_DESCRIPTION\([^\)]+\)|MODULE_DESCRIPTION("AT24 driver, but loaded at fs_initcall time")|

