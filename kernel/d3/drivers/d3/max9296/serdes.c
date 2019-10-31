/*
 * serdes.c
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c-mux.h>
#include <linux/list.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <d3/d3-jetson-bsp.h>

#include "serdes.h"

int serdes_get_reset_gpio(struct deserializer *des)
{
	struct device *dev = &des->serdes.client->dev;

	des->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(des->reset_gpio)) {
		dev_dbg(dev, "%s failed....", of_node_full_name(dev->of_node));
		return PTR_ERR(des->reset_gpio);
	}
	return 0;
}
EXPORT_SYMBOL(serdes_get_reset_gpio);

static void serdes_set_gpio(struct gpio_desc *gpio, int state)
{
	if (gpiod_cansleep(gpio))
		gpiod_set_value_cansleep(gpio, state);
	else
		gpiod_set_value(gpio, state);
}
EXPORT_SYMBOL(serdes_set_gpio);

void serdes_des_reset(struct deserializer *priv)
{
	serdes_set_gpio(priv->reset_gpio, 1);
	usleep_range(1000, 2000);
	serdes_set_gpio(priv->reset_gpio, 0);
}
EXPORT_SYMBOL(serdes_des_reset);

int serdes_get_reset_gpio_and_reset(struct deserializer *des)
{
	int ret;
	ret = serdes_get_reset_gpio(des);
	if (ret < 0)
		return ret;
	serdes_des_reset(des);
	usleep_range(1000, 50*1000);
	return 0;
}
EXPORT_SYMBOL(serdes_get_reset_gpio_and_reset);

int serdes_enable_regulator(struct device *dev, const char *reg_name,
		struct regulator **preg)
{
	int ret = 0;
	struct regulator *r = NULL;

	dev_dbg(dev, "Enabling regulator: %s.\n", reg_name);
	r = devm_regulator_get(dev, reg_name);
	if (IS_ERR(r)) {
		return PTR_ERR(r);
	}
	else {
		*preg = r;
		ret = regulator_enable(r);
		if (ret != 0) {
			dev_err(dev, "Error: Can't enable regulator: %s.\n",
				reg_name);
			return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL(serdes_enable_regulator);


int serdes_enable_regulators(struct deserializer *priv,
		int (*disable_link)(struct deserializer*, unsigned), unsigned link)
{
	struct device *dev = &priv->serdes.client->dev;
	int ret = 0;

	if (priv->regulator_enabled == false) {
		ret = serdes_enable_regulator(dev, DES_VDD_REGULATOR_NAME,
				&priv->vdd_regulator);

		/* While probing the deserializers, we need to wait
		 * for the GMSL regulators to come up. It may seem
		 * like the deserializers can be initialized before
		 * the GMSL buses are powered, but if we need to
		 * reassign the addresses of the serializers, we need
		 * to be able to talk to them.
		 */
		if (ret == -EPROBE_DEFER) {
			dev_info(dev, "Vdd regulator is not ready. Deferring.");
			return ret;
		}
		ret = serdes_get_reset_gpio_and_reset(priv);
		if (ret < 0) {
			dev_err(dev, "Can't reset deserializer. %d\n", ret);
			return ret;
		}
		dev_dbg(dev, "Vdd regulator ready.\n");
		/* verify that the chip is present */
		ret = i2c_smbus_write_byte(priv->serdes.client, 0);
		if (ret < 0) {
			dev_err(dev, "failed to write to device.\n");
			return -EPROBE_DEFER;
		}

		/* Prevent spurious i2c commmunication */
		disable_link(priv, link);
		priv->regulator_enabled = true;
	}
	ret = serdes_enable_regulator(dev, DES_POC_REGULATOR_NAME,
			&priv->poc_regulator);
	if (ret == -EPROBE_DEFER) {
		dev_info(dev, "PoC regulator is not ready. Deferring.");
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(serdes_enable_regulators);

int serdes_disable_regulators(struct deserializer *priv)
{
	struct device *dev = &priv->serdes.client->dev;
	int ret = 0;

	dev_dbg(dev, "Disabling PoC regulator.\n");
	ret = regulator_disable(priv->poc_regulator);
	if (ret != 0) {
		dev_err(dev, "Can't disable PoC regulator.\n");
		return ret;
	}
	dev_dbg(dev, "Disabling Vdd regulator.\n");
	ret = regulator_disable(priv->vdd_regulator);
	if (ret != 0) {
		dev_err(dev, "Can't disable Vdd regulator.\n");
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL(serdes_disable_regulators);

/* Do-nothing select function for the virtual i2c adapters */
int serdes_select_i2c_chan(struct i2c_mux_core *core, u32 chan)
{
	return 0;
}
EXPORT_SYMBOL(serdes_select_i2c_chan);

/* Do-nothing deselect function for the virtual i2c adapters */
int serdes_deselect_i2c_chan(struct i2c_mux_core *core, u32 chan)

{
	return 0;
}
EXPORT_SYMBOL(serdes_deselect_i2c_chan);

/* Create dummy i2c device and a regmap */
int  serdes_create_dummy(struct i2c_adapter *adapter, unsigned addr,
		const struct regmap_config *regmap_config,
		struct i2c_client **serp, struct regmap **mapp)
{
	struct i2c_client *client = NULL;
	struct regmap *map = NULL;

	client = i2c_new_dummy(adapter, addr);
	if (IS_ERR(client))
		return PTR_ERR(client);
	map = regmap_init_i2c(client, regmap_config);
	if (IS_ERR(map))
		return PTR_ERR(map);
	*serp = client;
	*mapp = map;
	return 0;
}
EXPORT_SYMBOL(serdes_create_dummy);

int serdes_of_scan_link_clients(struct serdes *serdes,
				struct device_node *link,
				i2c_translation_func_t add_i2c_translation)
{
	struct device *dev = &serdes->client->dev;
	struct device_node *child = NULL;
	int ret = 0;

	if (!link) {
		dev_err(dev, "link is NULL.\n");
		return -EINVAL;
	}

	dev_dbg(dev, "%s.\n", __func__);
	for_each_available_child_of_node(link, child) {
		u32 reg = 0;
		u32 phys_addr = 0;

		dev_dbg(dev, "Found: %s\n",of_node_full_name(child));
		of_property_read_u32(child, "reg", &reg);
		of_property_read_u32(child, "physical-addr", &phys_addr);

		if (!reg || !phys_addr) {
			dev_warn(dev, "Register or physical address can't be zero");
			continue;
		}
		if (reg == phys_addr) {
			dev_warn(dev, "Ignoring translation, reg and phys_addr have the same value!");
			continue;
		}
		/* Call the chip-specific translation function */
		ret = add_i2c_translation(serdes, reg, phys_addr);
		if (ret == 0) {
			dev_dbg(dev, "Reassigning I2C address: 0x%x => 0x%x\n",
				phys_addr, reg);
		}
		else {
			dev_warn(dev, "I2C address translation failed.\n");
		}
	}
	return ret;
}
EXPORT_SYMBOL(serdes_of_scan_link_clients);

/*
 * Scan the children node in the device for client buses. Create an I2C adapter
 * for each link found.
 *
 * @priv: serializer/deserializer private data
 * @max_links: number of GMSL links that the chip has
 * @add_i2c_translation: callback that does the chip-specific i2c addr. translation
 */
int serdes_of_scan_links(struct serdes *serdes,
			 unsigned max_links,
			 i2c_translation_func_t add_i2c_translation)
{
	struct device *dev = &serdes->client->dev;
	struct device_node *node = dev->of_node;
	struct device_node *link = NULL;
	int force_nr = 1; /* I2C virtual adapter number */
	int chan_id = 0;
	int ret = 0;

	if (!node) {
		dev_err(dev, "of_node is NULL.\n");
		return -EINVAL;
	}

	for_each_available_child_of_node(node, link) {
		if (strcmp(link->name, "link")) {
			dev_dbg(dev, "Ignoring node %s\n",
				of_node_full_name(link));
			continue;
		}

		/* Get bus number */
		force_nr = 0;
		ret = of_property_read_u32(link, "bus-number", &force_nr);

		dev_dbg(dev, "%s: bus number: %u",
			of_node_full_name(link), force_nr);

		serdes_of_scan_link_clients(serdes, link, add_i2c_translation);

		if (chan_id < max_links) {
			ret = i2c_mux_add_adapter(serdes->mux_core,
						  force_nr, 0, 0);
			if (ret) {
				dev_err(dev,
					"failed to register multiplexed"
					" adapter %d bus-num %d\n",
					chan_id, force_nr);
				return ret;
			}
			chan_id++;
		}
		else
			dev_warn(dev, "Too many link nodes!");
	}
	return 0;
}
EXPORT_SYMBOL(serdes_of_scan_links);


int serdes_mux_init(struct serdes *serdes, unsigned max_channels)
{
	struct device *dev = &serdes->client->dev;
	struct i2c_adapter *adap = to_i2c_adapter(dev->parent);
	// No priv, no flags (look for i2c-mux in dts)
	serdes->mux_core = i2c_mux_alloc(adap,
					 dev,
					 max_channels,
					 0, 0,
					 serdes_select_i2c_chan,
					 serdes_deselect_i2c_chan);

	if (!serdes->mux_core) {
		dev_err(dev, "failed to allocate mux core");
		return -ENOMEM;
	}

	return 0;
}
EXPORT_SYMBOL(serdes_mux_init);

#if 0
int serdes_get_client_nodes(struct device_node *node, struct serdes_node_list *list)
{
	struct device_node *link = NULL;
	struct device_node *client = NULL;
	struct serdes_node_list *tmp;
	int clients_found = 0;

	for_each_available_child_of_node(node, link) {
		if (strcmp(link->name, "link"))
			continue;
		for_each_available_child_of_node(link, client) {
			tmp = (struct serdes_node_list *)
				kzalloc(sizeof(struct serdes_node_list),
						GFP_KERNEL);
			list_add(&tmp->list, &list->list);
			clients_found += 1;
		}
	}
	return clients_found;
}
EXPORT_SYMBOL(serdes_get_client_nodes);

int serdes_serializer_is_gmsl1(struct serdes_node_list *list)
{
	int is_gmsl1 = 0;
	struct serdes_node_list *s;
	struct serdes_node_list *tmp;
	LIST_HEAD(h);

	list_for_each_entry_safe(s, tmp, &h, list){
		const char **p;
		int ret;
		pr_info("Node name %s.\n", s->node->name);
		ret = of_property_read_string(s->node, "compatible", p);
		if (!ret) {
			ret = strcmp(*p, "maxim,max96705");
			if (ret == 0)
				is_gmsl1 = 1;
		}
		list_del(&s->list);
		kfree(s);
	}
	return is_gmsl1;
}
#endif

#if 0
int serdes_get_addrs(struct device_node *node, u32 *out_src, u32 *out_dst)
{
	u32 addr;
	u32 reg;
	int ret = of_property_read_u32(node, "physical-addr", &addr);
	if (ret < 0)
		return ret;
	ret = of_property_read_u32(node, "reg", &reg);
	if (ret < 0)
		return ret;
	*out_src = reg;
	*out_dst = addr;
	return ret;
}
#endif

#if 0
/* Search the device tree for i2c address translations */
int max9296_get_child_addr(struct device_node *node,
		struct list_head *list)
{
	struct device_node *link = NULL;
	struct device_node *child = NULL;
	unsigned reg = 0;
	unsigned addr =  0;
	struct max9286_addr_map_list *tmp;
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
			tmp = (struct max9286_addr_map_list *)
				kzalloc(sizeof(struct max9286_addr_map_list), GFP_KERNEL);
			tmp->src = reg;
			tmp->dst = addr;
			list_add(&tmp->list, list);
			found = true;
		}
	}
	return found;
}
#endif

static int __init serdes_init(void)
{
	pr_info("Serdes module loaded.");
	return 0;
}

static void __exit serdes_exit(void)
{
	return;
}

module_init(serdes_init);
module_exit(serdes_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Catalin Petrescu <cpetrescu@d3engineering.com>");
MODULE_DESCRIPTION("Common serializer / deserializer code");
MODULE_VERSION(D3_JETSON_BSP_VERSION);
