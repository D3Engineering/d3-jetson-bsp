/*
 * max96705.c - Maxim MAX96705 GMSL1 Serializer
 *
 * Copyright (c) 2018,2019 D3 Engineering.  All rights reserved.
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

#define MAX_DEBUG 1
#define MAX_DEBUG_GMSL_BACKCHANNEL 0

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/slab.h>

#include <d3/d3-jetson-bsp.h>

#include "max96705.h"
#include "max96705_tables.h"
#include "serdes.h"

enum {
	CHIP_MAX96705 = 0,
};

static struct i2c_device_id max96705_idtable[] = {
	{"max96705", CHIP_MAX96705},
	{},
};
MODULE_DEVICE_TABLE(i2c, max96705_idtable);

static struct of_device_id max96705_of_match[] = {
	{ .compatible = "maxim,max96705"},
	{},
};
MODULE_DEVICE_TABLE(of, max96705_of_match);

/* I2C address translation related structures */
struct max96705_addr_map_list {
	u32 src;
	u32 dst;
	struct list_head list;
};

const static struct ser_i2c_map_regs max96705_i2c_map_regs[] = {
	{MAX96705_REG_SRC_A, MAX96705_REG_DST_A},
	{MAX96705_REG_SRC_B, MAX96705_REG_DST_B}
};

#ifdef MAX_DEBUG
/* Let some registers be read via debugfs */
static const struct regmap_range max96705_yes_ranges[] = {
	regmap_reg_range(0, 0x9a),
};

static const struct regmap_access_table max96705_regmap_rd_access_table = {
	.yes_ranges = max96705_yes_ranges,
	.n_yes_ranges = ARRAY_SIZE(max96705_yes_ranges),
};
#endif /* MAX_DEBUG */

static const struct regmap_config max96705_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
#ifdef MAX_DEBUG
	.max_register = 0xff,
	.rd_table = &max96705_regmap_rd_access_table,
#endif /* MAX_DEBUG */
};

/*
 * Wrapper around regmap_read that prints errors.
 */
/* static int max96705_regmap_read(struct serializer *priv, u32 reg, u32 *val) */
/* { */
/* 	struct device *dev = &priv->client->dev; */
/* 	struct regmap *map = priv->map; */
/* 	int ret = 0; */

/* 	reg &= 0xFFFF; */
/* 	ret = regmap_read(map, reg, val); */
/* 	if (ret) */
/* 		dev_err(dev, "Failed reading register 0x%x\n", reg); */
/* 	return ret; */
/* } */

/*
 * Wrapper around regmap_write that prints errors.
 */

static int max96705_regmap_write(struct serdes *serdes, u32 reg, u32 val)
{
	struct device *dev = &serdes->client->dev;
	struct regmap *map = serdes->map;
	int ret = 0;

	reg &= 0xFFFF;
	val &= 0xFF;
	ret = regmap_write(map, reg, val);
	if (ret)
		dev_err(dev, "Failed writing register 0x%x\n", reg);
	return ret;
}

/* One-shot reset; resets the GMSL link but keeps the register settings */
/* static int max96705_one_shot_reset(struct serializer *priv) */
/* { */
/* 	const struct reg_sequence one_shot_reset[] = { */
/* 		{ 0x0010, 0x30, 100000 }, */
/* 	}; */

/* 	return regmap_multi_reg_write(priv->map, one_shot_reset, */
/* 			ARRAY_SIZE(one_shot_reset)); */
/* } */

/* Search the device tree for i2c address translations */
int max96705_get_child_addr(struct device_node *node,
		struct list_head *list)
{
	struct device_node *link = NULL;
	struct device_node *child = NULL;
	unsigned reg = 0;
	unsigned addr =  0;
	struct max96705_addr_map_list *tmp;
	int ret = 0;
	int found = false;

	for_each_available_child_of_node(node, link) {
		if (strcmp(link->name, "link"))
			continue;
		for_each_available_child_of_node(link, child) {
			ret = of_property_read_u32(child, "physical-addr",
					&addr);
			if (ret != 0)
				continue;
			ret = of_property_read_u32(child, "reg", &reg);
			if (ret != 0)
				continue;
			tmp = (struct max96705_addr_map_list *)
				kzalloc(sizeof(struct max96705_addr_map_list), GFP_KERNEL);
			tmp->src = reg;
			tmp->dst = addr;
			list_add(&tmp->list, list);
			found = true;
		}
	}
	return found;
}

/*
 * Set up an I2C address translation by setting the SRC_[AB], DST_[AB]registers
 * MAX96705 has two pairs of registers, therefore it can do up to 2 translations
 * @priv: max96705 private data
 * @src: the address the client will appear to have after translation
 * @dst: the actual address of the client
 *
 * Return: 0 if the translation was set up successfully, -1 if otherwise.
 */
static int max96705_add_i2c_translation(struct serdes *serdes, u32 src, u32 dst)
{
	int i = serdes->num_i2c_maps;

	if (i < MAX96705_I2C_TRANS_MAX) {
		max96705_regmap_write(serdes, max96705_i2c_map_regs[i].src,
				src << 1);
		max96705_regmap_write(serdes, max96705_i2c_map_regs[i].dst,
				dst << 1);
		serdes->num_i2c_maps++;
		return 0;
	}
	return -1;
}

/* static int max96705_of_address_translations(struct device_node *node, */
/* 		u32 *src, u32 *dst) */
/* { */
/* 	u32 reg; */
/* 	u32 phys_addr; */

/* 	of_property_read_u32(node, "reg", &reg); */
/* 	of_property_read_u32(node, "physical-addr", &phys_addr); */

/* 	if (!reg || !phys_addr) */
/* 		return -ENXIO; */
/* 	if (reg == phys_addr) */
/* 		return -EINVAL; */

/* 	*src = reg; */
/* 	*dst = phys_addr; */
/* 	return 0; */
/* } */

static int max96705_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct serializer *priv = NULL;
	int ret = 0;

	dev_dbg(&client->dev, "Probing MAX96705.\n");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(&client->dev, sizeof(struct serializer), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "failed to allocate memory!\n");
		return -ENOMEM;
	}

	priv->serdes.client = client;
	i2c_set_clientdata(client, priv);

	priv->serdes.map = devm_regmap_init_i2c(client, &max96705_regmap_config);
	if (IS_ERR(priv->serdes.map)) {
		dev_err(&client->dev, "Failed to create regmap.\n");
		return PTR_ERR(priv->serdes.map);
	}

	ret = serdes_mux_init(&priv->serdes, SER_MAX_CHANNELS);
	if (ret != 0) {
		return ret;
	}


#if 0
	ret = regmap_multi_reg_write(priv->map, max96705_init_seq, ARRAY_SIZE(max96705_init_seq));
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write registers.\n");
		return ret;
	}
#endif
	dev_dbg(&client->dev, "Scanning for children nodes in DT..\n");
	/* Scan the device tree for i2c slave buses */
	return serdes_of_scan_links(&priv->serdes,
				    MAX96705_NUM_CHANNELS,
				    max96705_add_i2c_translation);
}

static int max96705_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "Removing MAX96705.");
	return 0;
}

static struct i2c_driver max96705_driver = {
	.driver = {
		.name = "max96705",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max96705_of_match),
	},
	.probe = max96705_probe,
	.remove = max96705_remove,
	.id_table = max96705_idtable,
};

module_i2c_driver(max96705_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Catalin Petrescu <cpetrescu@d3engineering.com>");
MODULE_DESCRIPTION("Maxim MAX96705 GMSL1 Serializer driver");
MODULE_VERSION(D3_JETSON_BSP_VERSION);

