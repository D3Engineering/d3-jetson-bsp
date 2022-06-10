/**
 * @author Greg Rowe <growe@d3engineering.com>
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
#include <linux/module.h>

#include <linux/of.h>
#include <linux/sysfs.h>
#include <d3/d3-jetson-bsp.h>

#define MODULE_NAME "build-id"

static char *s_BUILD_ID = D3_JETSON_BSP_BUILDID;
static char *s_COMMIT_HASH = D3_JETSON_BSP_COMMIT_HASH;

static int buildid_of_get(const char *key, const char **out_value)
{
	int err = 0;

	err = of_property_read_string(of_root, key, out_value);
	if (err != 0) {
		printk(KERN_WARNING
		       MODULE_NAME
		       ": could not read device tree %s \n", key);
	}
	return err;
}


/* Do not make this function static - we check for it in System.map
 * to confirm that at least this D3 driver has been compiled into
 * the kernel.
 */
void buildid_check_dtb_match(void)
{
	int err = 0;
	const char *build_id = NULL;
	const char *commit_hash = NULL;

	err |= buildid_of_get("d3,jetson-bsp-build-id", &build_id);
	err |= buildid_of_get("d3,jetson-bsp-commit-hash", &commit_hash);
	if (err != 0) {
		return;
	}

	if ((strcasecmp(s_BUILD_ID, build_id) != 0)
	    || (strcasecmp(s_COMMIT_HASH, commit_hash) != 0)) {
		printk(KERN_WARNING
		       MODULE_NAME
		       ": failure, kernel does not match device tree:"
		       " kernel build-id=\"%s\", device-tree build-id=\"%s\""
		       " kernel commit-hash=\"%s\", device-tree commit-hash=\"%s\""
		       "\n",
		       s_BUILD_ID, build_id,
		       s_COMMIT_HASH, commit_hash);
		return;
	}

	printk(KERN_INFO
	       MODULE_NAME
	       ": success, kernel ids match device tree:"
	       " build-id=\"%s\""
	       " commit-hash=\"%s\""
	       "\n",
	       s_BUILD_ID, s_COMMIT_HASH);
}


static int __init buildid_init(void)
{
	printk(KERN_INFO
	       MODULE_NAME
	       ": D3 Jetson BSP v"
	       D3_JETSON_BSP_VERSION_NAME
	       " build-id:"
	       D3_JETSON_BSP_BUILDID
	       " commit-hash:"
	       D3_JETSON_BSP_COMMIT_HASH
	       "\n");

	buildid_check_dtb_match();
	return 0;
}

module_init(buildid_init);



MODULE_DESCRIPTION("Unique build id");
MODULE_AUTHOR("Greg Rowe <growe@d3engineering.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(D3_JETSON_BSP_VERSION);
module_param_named(commit_hash, s_COMMIT_HASH, charp, S_IRUSR | S_IRGRP | S_IROTH);
module_param_named(build_id, s_BUILD_ID, charp, S_IRUSR | S_IRGRP | S_IROTH);






