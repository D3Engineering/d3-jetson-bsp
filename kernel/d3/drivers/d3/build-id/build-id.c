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
#include <linux/module.h>

#include <linux/of.h>
#include <d3/d3-jetson-bsp.h>

#define MODULE_NAME "build-id"
MODULE_DESCRIPTION("Unique build id");
MODULE_VERSION(D3_JETSON_BSP_BUILDID);
MODULE_AUTHOR("Greg Rowe <growe@d3engineering.com>");
MODULE_LICENSE("GPL v2");


static void buildid_check_dtb_match(void) {
	static const char *NAME = "d3,jetson-bsp-build-id";
	const char *build_id = NULL;

	int err = of_property_read_string(of_root, NAME, &build_id);
	if (err != 0) {
		printk(KERN_WARNING
		       MODULE_NAME
		       ": could not read device tree %s \n", NAME);
		return;
	}
	if (strcasecmp(D3_JETSON_BSP_BUILDID, build_id) != 0) {
		printk(KERN_WARNING
		       MODULE_NAME
		       ": kernel build id does not match device tree: "
		       "kernel=\"%s\", device-tree=\"%s\" \n",
		       D3_JETSON_BSP_BUILDID, build_id);
		return;
	}
}


static int __init buildid_init(void) {

	printk(KERN_INFO
	       "D3 Jetson BSP v"
	       D3_JETSON_BSP_VERSION
	       " build id:"
	       D3_JETSON_BSP_BUILDID
	       "\n");

	buildid_check_dtb_match();
	return 0;
}
module_init(buildid_init);


