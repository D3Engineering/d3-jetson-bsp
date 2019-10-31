/*
 * max9295.c - Maxim MAX9295 GMSL2 to CSI-2 Serializer
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

#define MAX_DEBUG 1
#define MAX_DEBUG_GMSL_BACKCHANNEL 0

#include "max9295.h"
#include "max9295_tables.h"
#include "serdes.h"

enum {
	CHIP_MAX9295 = 0,
};

static struct i2c_device_id max9295_idtable[] = {
	{"max9295", CHIP_MAX9295},
	{},
};
MODULE_DEVICE_TABLE(i2c, max9295_idtable);

static struct of_device_id max9295_of_match[] = {
	{ .compatible = "maxim,max9295"},
	{},
};
MODULE_DEVICE_TABLE(of, max9295_of_match);

/* I2C address translation related structures */
struct max9295_addr_map_list {
	u32 src;
	u32 dst;
	struct list_head list;
};

const static struct ser_i2c_map_regs max9295_i2c_map_regs[] = {
	{MAX9295_REG_SRC_A, MAX9295_REG_DST_A},
	{MAX9295_REG_SRC_B, MAX9295_REG_DST_B}
};

#ifdef MAX_DEBUG
/* Let some registers be read via debugfs */
static const struct regmap_range max9295_yes_ranges[] = {
	regmap_reg_range(0, 6),
	regmap_reg_range(0xD, 0x13),
	regmap_reg_range(0x18, 0x25),
	regmap_reg_range(0x40, 0x47), /* I2C */
	regmap_reg_range(0x1D9, 0x1DA),
	regmap_reg_range(0x2D6, 0x2D8), /* GPIOs */
	regmap_reg_range(0x308, 0x328),
	regmap_reg_range(0x314, 0x314),
	regmap_reg_range(0x316, 0x316),
	regmap_reg_range(0x330, 0x334),
	regmap_reg_range(0x40A, 0x40A),
	regmap_reg_range(0x44A, 0x44A),
	regmap_reg_range(0x48A, 0x48A),
	regmap_reg_range(0x4CA, 0x4CA),
	regmap_reg_range(0x540, 0x541), /* CFG0, CFG1 */
	regmap_reg_range(0xB04, 0xB08), /* HIM, DBL, HVEN */
	regmap_reg_range(0xB0D, 0xB1D), /* I2C_LOC_ACK */
	regmap_reg_range(0xF03, 0xF04), /* GMSL1 ERR */
	regmap_reg_range(0x14C4, 0x14C4),
	regmap_reg_range(0x14C5, 0x14C5),
};

static const struct regmap_access_table max9295_regmap_rd_access_table = {
	.yes_ranges = max9295_yes_ranges,
	.n_yes_ranges = ARRAY_SIZE(max9295_yes_ranges),
};
#endif /* MAX_DEBUG */

static const struct regmap_config max9295_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
#ifdef MAX_DEBUG
	.max_register = 0x1f17,
	.rd_table = &max9295_regmap_rd_access_table,
#endif  /* MAX_DEBUG */
};

/*
 * Wrapper around regmap_read that prints errors.
 */
/* static int max9295_regmap_read(struct serializer *priv, u32 reg, u32 *val) */
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

static int max9295_regmap_write(struct serdes *serdes, u32 reg, u32 val)
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
/* static int max9295_one_shot_reset(struct serializer *priv) */
/* { */
/* 	const struct reg_sequence one_shot_reset[] = { */
/* 		{ 0x0010, 0x30, 100000 }, */
/* 	}; */

/* 	return regmap_multi_reg_write(priv->map, one_shot_reset, */
/* 			ARRAY_SIZE(one_shot_reset)); */
/* } */

/* static int max9295_set_lane_count(struct serializer *priv) */
/* { */
/* 	struct device *dev = &priv->client->dev; */
/* 	unsigned reg_val = 0; */
/* 	unsigned regs[] = { 0x40A, 0x44A, 0x48A, 0x4CA }; /\* MIPI_TX10 *\/ */
/* 	int i = 0; */

/* 	/\* The MAX9296 register documentation is poorly written. Until it gets */
/* 	 * better, the values that go in the MIPI_TX10 are a sort of best guess. */
/* 	 *\/ */
/* 	switch (priv->csi_config.num_lanes) { */
/* 	case 1: */
/* 		reg_val = 0; */
/* 		break; */
/* 	case 2: */
/* 		reg_val = 0x40; */
/* 		break; */
/* 	case 4: */
/* 		reg_val = 0xC0; */
/* 		break; */
/* 	default: */
/* 		dev_err(dev, "Invalid number of CSI lanes: %d", */
/* 				priv->csi_config.num_lanes); */
/* 		return -EINVAL; */
/* 	} */

/* 	for (i = 0; i < ARRAY_SIZE(regs); i++) */
/* 		max9295_regmap_write(priv, regs[i], reg_val); */
/* 	return 0; */
/* } */

/* static int max9295_set_csi_freq(struct serializer *priv) */
/* { */
/* 	unsigned f = 16; /\* frequency as multiple of 100MHz *\/ */
/* 	unsigned reg_val = 0; */
/* 	unsigned regs[] = { */
/* 		MAX9295_REG_BACKTOP22, /\* phy0 *\/ */
/* 		MAX9295_REG_BACKTOP25, /\* phy1 *\/ */
/* 		MAX9295_REG_BACKTOP28, /\* phy2 *\/ */
/* 		MAX9295_REG_BACKTOP31, /\* phy3 *\/ */
/* 	}; */
/* 	int i = 0; */
/* 	int ret = 0; */

/* 	if (priv->csi_config.freq_mhz != 0) { */
/* 		f = (priv->csi_config.freq_mhz / 100 ) & 0x1F; */
/* 		for (i = 0; i < ARRAY_SIZE(regs); i++) { */
/* 			ret = max9295_regmap_read(priv, regs[i], &reg_val); */
/* 			if (ret) */
/* 				continue; */
/* 			reg_val &= 0xE0; */
/* 			reg_val |= f; */
/* 			max9295_regmap_write(priv, regs[i], reg_val); */
/* 		} */
/* 	} */
/* 	return 0; */
/* } */

/* static int max9295_set_csi_parameters(struct serializer *priv, struct device_node *node) */
/* { */
/* 	int ret; */

/* 	ret = of_property_read_u32(node, "csi-lane-count", */
/* 			&priv->csi_config.num_lanes); */
/* 	if (ret == 0) */
/* 		max9295_set_lane_count(priv); */
/* 	ret = of_property_read_u32(node, "csi-tx-speed-mbps", */
/* 			&priv->csi_config.freq_mhz); */
/* 	if (ret == 0) */
/* 		max9295_set_csi_freq(priv); */
/* 	return 0; */
/* } */

/* TODO: set_gmsl_bit_rates needs to be refactored so it handles both
 * max9296 and max9295.
 */
static int max9295_set_gmsl_bit_rates(struct serdes *serdes)
{
	struct device *dev = &serdes->client->dev;
	struct reg_sequence set_gmsl_speed[] = {
		{.reg = 0x01, .def = 0, .delay_us = 0},
		{.reg = 0x10, .def = 0x31, .delay_us = 100000},
	};
	u8 enc_forward = 0; /* encoded rate */
	u8 enc_back = 0;
	int ret = 0;

	dev_dbg(dev, "Setting bit rates.");
	switch(serdes->serial_config.back_bit_rate) {
	case (187):
		enc_forward = MAX9295_BACK_RATE_187_5MHZ;
		break;
	case (375):
		enc_forward = MAX9295_BACK_RATE_375MHZ;
		break;
	case (750):
		enc_forward = MAX9295_BACK_RATE_750MHZ;
		break;
	case (1500):
		enc_forward = MAX9295_BACK_RATE_1500MHZ;
		break;
	default:
		dev_err(dev, "Invalid GMSL TX rate: %d",
				serdes->serial_config.back_bit_rate);
		return -1;

	}
	switch(serdes->serial_config.forward_bit_rate) {
	case (1500):
		enc_back = MAX9295_FORWARD_RATE_1500MHZ;
		break;
	case (3000):
		enc_back = MAX9295_FORWARD_RATE_3000MHZ;
		break;
	case (6000):
		enc_back = MAX9295_FORWARD_RATE_6000MHZ;
		break;
	default:
		dev_err(dev, "Invalid GMSL RX rate: %d",
				serdes->serial_config.forward_bit_rate);
		return -1;

	}
	set_gmsl_speed[0].def = MAX9295_GMSL_RATE(enc_forward, enc_back);
	dev_dbg(dev, "Setting GMSL rate to: 0x%x",
			set_gmsl_speed[0].def);
	ret = regmap_multi_reg_write(serdes->map, set_gmsl_speed,
			ARRAY_SIZE(set_gmsl_speed));
	return ret;
}

/* Search the device tree for i2c address translations */
int max9295_get_child_addr(struct device_node *node,
		struct list_head *list)
{
	struct device_node *link = NULL;
	struct device_node *child = NULL;
	unsigned reg = 0;
	unsigned addr =  0;
	struct max9295_addr_map_list *tmp;
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
			tmp = (struct max9295_addr_map_list *)
				kzalloc(sizeof(struct max9295_addr_map_list), GFP_KERNEL);
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
 * MAX9295 has two pairs of registers, therefore it can do up to 2 translations
 * @priv: max9295 private data
 * @src: the address the client will appear to have after translation
 * @dst: the actual address of the client
 *
 * Return: 0 if the translation was set up successfully, -1 if otherwise.
 */
static int max9295_add_i2c_translation(struct serdes *serdes, u32 src, u32 dst)
{
	int i = serdes->num_i2c_maps;

	if (i < MAX9295_I2C_TRANS_MAX) {
		max9295_regmap_write(serdes, max9295_i2c_map_regs[i].src,
				     src << 1);
		max9295_regmap_write(serdes, max9295_i2c_map_regs[i].dst,
				     dst << 1);
		serdes->num_i2c_maps++;
		return 0;
	}
	return -1;
}

int max9295_gmsl2_init(struct serdes *serdes, struct device_node *node)
{
	unsigned gmsl_bit_rates[2] = {0, 0};
	int ret;

	ret = of_property_read_u32_array(node, "gmsl-bit-rates",
			gmsl_bit_rates, 2);
	if (ret == 0) {
		serdes->serial_config.forward_bit_rate = gmsl_bit_rates[0];
		serdes->serial_config.back_bit_rate = gmsl_bit_rates[1];
		return max9295_set_gmsl_bit_rates(serdes);
	}
	return 0;
}

/* static int max9295_of_address_translations(struct device_node *node, */
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

static int max9295_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
	struct serializer *priv = NULL;
	int ret = 0;

	dev_dbg(&client->dev, "Probing %s.\n",
			max9295_idtable[id->driver_data].name);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(&client->dev, sizeof(struct serializer), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "failed to allocate memory!\n");
		return -ENOMEM;
	}

	priv->serdes.client = client;
	i2c_set_clientdata(client, priv);

	priv->serdes.map = devm_regmap_init_i2c(client, &max9295_regmap_config);
	if (IS_ERR(priv->serdes.map)) {
		dev_err(&client->dev, "Failed to create regmap.\n");
		return PTR_ERR(priv->serdes.map);
	}

	ret = serdes_mux_init(&priv->serdes, SER_MAX_CHANNELS);
	if (ret != 0) {
		return ret;
	}

	ret = regmap_multi_reg_write(priv->serdes.map,
				     max9295_init_seq,
				     ARRAY_SIZE(max9295_init_seq));
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write registers.\n");
		return ret;
	}

	// No priv, no flags (look for i2c-mux in dts)
	priv->serdes.mux_core = i2c_mux_alloc(adap,
					      &client->dev,
					      SER_MAX_CHANNELS,
					      0, 0,
					      serdes_select_i2c_chan,
					      serdes_deselect_i2c_chan);

	if (!priv->serdes.mux_core) {
		dev_err(&client->dev, "failed to allocate mux core");
		return -ENOMEM;
	}


	dev_dbg(&client->dev, "Scanning for children nodes in DT..\n");

	/* Scan the device tree for i2c slave buses */
	return serdes_of_scan_links(&priv->serdes,
				    MAX9295_NUM_CHANNELS,
				    max9295_add_i2c_translation);

}
/* static int max9295_chip_9295_init(struct serializer *priv) */
/* { */
/* 	struct device *dev = &priv->client->dev; */
/* 	struct device_node *node = dev->of_node; */
/* 	LIST_HEAD(addr_list); */
/* 	struct max9295_addr_map_list *p; */
/* 	int ret; */

/* 	ret = max9295_get_child_addr(node, &addr_list); */
/* 	list_for_each_entry(p, &addr_list, list) { */
/* 		dev_dbg(dev, "Test: 0x%x -> 0x%x.\n", p->src, p->dst); */
/* 		max9295_add_i2c_translation(priv, p->src, p->dst); */
/* 		/\* TODO: detelte the list elements and free the memory *\/ */
/* #if 0 */
/* 		list_del(&p->list); */
/* 		kfree(p); */
/* #endif */
/* 	} */
/* 	return 0; */
/* } */

static int max9295_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "Removing MAX9295.");
	return 0;
}

static struct i2c_driver max9295_driver = {
	.driver = {
		.name = "max9295",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max9295_of_match),
	},
	.probe = max9295_probe,
	.remove = max9295_remove,
	.id_table = max9295_idtable,
};

module_i2c_driver(max9295_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Catalin Petrescu <cpetrescu@d3engineering.com>");
MODULE_DESCRIPTION("Maxim MAX9295A CSI-2/parallel to GMSL2 Serializer driver");
MODULE_VERSION(D3_JETSON_BSP_VERSION);

