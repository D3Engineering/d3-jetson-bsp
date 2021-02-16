/**
 * @author Greg Rowe <growe@d3engineering.com>
 *
 * Copyright (c) 2019, D3 Engineering.  All rights reserved.
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
#ifndef _UB960_H
#define _UB960_H

#include <linux/i2c.h>

/**
 * DOC: Overview
 *
 * This was implemented as a quick kludge to overcome an issue on
 * Xavier where only the first time a camera streamed would be
 * successful. It was found that a digital reset of the deserializer
 * would work around the problem. Resetting the deserializer should
 * not be required but it does workaround the problem.
 *
 * The image sensor drivers, imx390 and ov10640, directly call this
 * function when they start and stop streams. They are linked via a
 * phandle in the device tree.
 *
 * Having an event mechanism between the devices in a serdes chain is
 * desire. There was not enough time to implement such a mechanism
 * properly in the time allowed. Such an implementation will be
 * generic to the serdes technology and will not tightly couple the
 * serdes components. In other words an image sensor driver would not
 * _require_ that a deserializer be present but would pass messages if
 * one were present.
 */

int ub960_s_stream(struct i2c_client *self,
		   struct i2c_client *src,
		   int enable);

#endif
