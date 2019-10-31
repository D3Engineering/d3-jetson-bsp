/*
 * max9286.c - Maxim MAX9286 GMSL1 to CSI-2 Deserializer
 *
 * Copyright (c) 2018-2019, D3 Engineering.  All rights reserved.
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
#include <linux/gpio/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>

#include <d3/d3-jetson-bsp.h>

#include "max9286.h"
#include "max96705.h"
#include "max9286_tables.h"
#include "max96705_tables.h"
#include "serdes.h"

#define MAX_DEBUG 1
#define MAX_DEBUG_GMSL_BACKCHANNEL 0



enum {
	CHIP_MAX9286 = 0,
};

static struct i2c_device_id max9286_idtable[] = {
	{"max9286", CHIP_MAX9286},
	{},
};
MODULE_DEVICE_TABLE(i2c, max9286_idtable);

static struct of_device_id max9286_of_match[] = {
	{ .compatible = "maxim,max9286"},
	{},
};
MODULE_DEVICE_TABLE(of, max9286_of_match);

/* TODO: This is shared by 2 functions. refactor. */
static struct reg_sequence max96705_seraddr[] = {
	{ MAX96705_REG_SERADDR, MAX96705_REG_SERADDR_DEFAULT },
};

/* TODO: This is shared by 2 functions. refactor. */
static struct reg_sequence max96705_imgaddr[] = {
	{ MAX96705_REG_SRC_A, 0 },
	{ MAX96705_REG_DST_A, 0 },
};

/* MAX9286-MAX96705 initialization playlist */
static const struct serdes_playlist max9286_max96705[] = {
	/* reverse channel setup */
	PLAYLIST_INIT('d', max9286_custom_rcs),
	PLAYLIST_INIT('s', max96705_en_cfg),
	PLAYLIST_INIT('d', max9286_disable_loc_ack),
	PLAYLIST_INIT('s', max96705_ser_input),
	PLAYLIST_INIT('d', max9286_dis_csi),
	PLAYLIST_INIT('s', max96705_en_dbl_hven_hibw),
	PLAYLIST_INIT('d', max9286_en_hibw),
	PLAYLIST_INIT('s', max96705_retime_vs_hs),
	PLAYLIST_INIT('s', max96705_set_xbar),
	/* sensor setup */
	/* post camera setup */
	PLAYLIST_INIT('s', max96705_en_gmsl),
	PLAYLIST_INIT('d', max9286_en_csi),
	/* i2c address remapping */
	PLAYLIST_INIT('s', max96705_imgaddr),
	PLAYLIST_INIT('s', max96705_seraddr),
};

#ifdef MAX_DEBUG
/* Let some registers be read via debugfs */
static const struct regmap_range max9286_yes_ranges[] = {
	regmap_reg_range(0, 0x34),
	regmap_reg_range(0x41, 0x41),
	regmap_reg_range(0x49, 0x49),
	regmap_reg_range(0x5b, 0x64),
	regmap_reg_range(0x68, 0x71),
};

static const struct regmap_access_table max9286_regmap_rd_access_table = {
	.yes_ranges = max9286_yes_ranges,
	.n_yes_ranges = ARRAY_SIZE(max9286_yes_ranges),
};
#endif /* MAX_DEBUG */

static const struct regmap_config max9286_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
#ifdef MAX_DEBUG
	.max_register = 0x71,
	.rd_table = &max9286_regmap_rd_access_table,
#endif	/* MAX_DEBUG */
};

static const struct regmap_config max96705_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct regmap_config regmap_config_16bit = {
	.reg_bits = 16,
	.val_bits = 16,
};

/*
 * Wrapper around regmap_read that prints errors.
 */
static int max9286_regmap_read(struct deserializer *priv, u32 reg, u32 *val)
{
	struct device *dev = &priv->serdes.client->dev;
	struct regmap *map = priv->serdes.map;
	int ret = 0;

	reg &= 0xFF;
	ret = regmap_read(map, reg, val);
	if (ret)
		dev_err(dev, "Failed reading register 0x%x\n", reg);
	return ret;
}

/*
 * Wrapper around regmap_write that prints errors.
 */

static int max9286_regmap_write(struct deserializer *priv, u32 reg, u32 val)
{
	struct device *dev = &priv->serdes.client->dev;
	struct regmap *map = priv->serdes.map;
	int ret = 0;

	reg &= 0xFF;
	val &= 0xFF;
	ret = regmap_write(map, reg, val);
	if (ret)
		dev_err(dev, "Failed writing register 0x%x\n", reg);
	return ret;
}

/* Set GMSL link state for 9286 */
static int max9286_set_link_state(struct deserializer *priv, unsigned link,
		int state)
{
	int ret;
	u32 reg_val;
	u8 mask;

	if (link >= MAX9286_NUM_CHANNELS)
		return -EINVAL;
	ret = max9286_regmap_read(priv, 0x0A, &reg_val);
	if (ret != 0)
		return ret;
	mask = (MAX9286_REVCCEN0 | MAX9286_FWDCCEN0) << link;
	reg_val = state ? reg_val | mask : reg_val & ~mask;
	return  max9286_regmap_write(priv, 0x0A, reg_val);
}

static inline int max9286_disable_link(struct deserializer *priv, unsigned link)
{
	return max9286_set_link_state(priv, link, false);
}

static inline int max9286_enable_link(struct deserializer *priv, unsigned link)
{
	return max9286_set_link_state(priv, link, true);
}

static int max9286_set_lane_count(struct deserializer *priv)
{
	struct device *dev = &priv->serdes.client->dev;
	unsigned reg_val = 0;
	unsigned char mask = 0;
	const unsigned char reg_addr = 0x12;
	int ret;

	if (priv->serdes.csi_config.num_lanes > 4
		|| priv->serdes.csi_config.num_lanes == 0) {
		dev_err(dev, "Invalid number of CSI lanes: %d",
				priv->serdes.csi_config.num_lanes);
		return -EINVAL;
	}
	mask = (priv->serdes.csi_config.num_lanes - 1) << 6;
	ret = max9286_regmap_read(priv, reg_addr, &reg_val);
	if (ret < 0)
		return ret;
	reg_val = (reg_val & 0x3F) | mask;
	return max9286_regmap_write(priv, reg_addr, reg_val);
}

static int max9286_set_csi_parameters(struct deserializer *priv, struct device_node *node)
{
	int ret = 0;

	ret = of_property_read_u32(node, "csi-lane-count",
			&priv->serdes.csi_config.num_lanes);
	if (ret == 0)
		ret = max9286_set_lane_count(priv);
	return ret;
}

#define IS_WORTH_RETRYING(ret) ( ret == -ETIMEDOUT \
		|| ret == -EAGAIN \
		|| ret == -EREMOTEIO \
		)

static int max9286_play_init_sequence(struct device *dev,
		const struct serdes_playlist *pl, unsigned len,
		struct regmap *des_map, struct regmap *ser_map,
		struct regmap *sen_map)
{
	int ret = 0;
	int i = 0;
	const int max_retries = 3;

	for (i = 0; i < len; i++) {
		unsigned try_cnt = 0;
		struct regmap *p;
		p = pl[i].target == 'd' ? des_map
		  : pl[i].target == 's' ? ser_map
		  : sen_map;
#if MAX_DEBUG_GMSL_BACKCHANNEL
		/* back channel debugging */
		if (i == 5 || i == 6) {
			int j;
			unsigned testval;
			int testret;
			int errcount = 0;
			dev_dbg(dev, "Starting backchannel testing.");
			for (j = 0; j < 100; j++) {
				testret = regmap_read(ser_map, 0, &testval);
				if (testret < 0)
					errcount++;
			}
			dev_dbg(dev, "Backchannel error count: %u", errcount);
		}
		/* end back channel debugging */
#endif		/* MAX_DEBUG_GMSL_BACKCHANNEL */
		do {
			ret = regmap_multi_reg_write(p, pl[i].seq, pl[i].len);
			try_cnt += 1;
			if (ret < 0) {
				dev_dbg(dev, "Step: %u, ret: %d, try: %d.\n",
						i, ret, try_cnt);
				udelay(200 * try_cnt);
			}
		} while (IS_WORTH_RETRYING(ret) && try_cnt < max_retries);
		if (ret < 0) {
			dev_err(dev, "Step %d failed. ret: %d\n", i, ret);
			return ret;
		}
	}
	return ret;
}

int max9286_gmsl1_init(struct deserializer *priv, struct device_node *node,
		const struct serdes_playlist *pl, unsigned pl_len)
{
	struct device *dev = &priv->serdes.client->dev;
	struct i2c_client *client = priv->serdes.client;
	struct i2c_adapter *i2c_parent = to_i2c_adapter(client->dev.parent);
	struct i2c_client *ser = NULL;
	struct regmap *ser_map = NULL;
	struct i2c_client *sen = NULL;
	struct regmap *sen_map = NULL;
	unsigned ser_addr = 0x40;
	unsigned sen_addr = 0x10;
	const int max_retries = 5;
	int try_cnt = 0;
	int ret = 0;
	u32 ser_addr_map[2] = {0, 0}; /* remapped addr, physical addr */
	u32 sen_addr_map[2] = {0, 0}; /* remapped addr, physical addr */

	dev_dbg(dev, "GMSL-1 init.\n");
	if (!i2c_parent) {
		dev_err(dev, "Oops! the i2c parent is NULL!\n");
		return 0;
	}

	/* Serializer i2c address translation first */
	ret = of_property_read_u32_array(node, "ser-addr", ser_addr_map, 2);
	if (ret == 0) {
		max96705_seraddr[0].def = ser_addr_map[0] << 1;
		ser_addr = ser_addr_map[1];
		dev_dbg(dev, "0x%x => 0x%x.\n",
				ser_addr_map[0], ser_addr_map[1]);
	}
	ret = of_property_read_u32_array(node, "img-addr", sen_addr_map, 2);
	if (ret == 0) {
		max96705_imgaddr[0].def = sen_addr_map[0] << 1;
		max96705_imgaddr[1].def = sen_addr_map[1] << 1;
		sen_addr = sen_addr_map[1];
		dev_dbg(dev, "0x%x => 0x%x.\n",
				sen_addr_map[0], sen_addr_map[1]);
	}

	ret = serdes_create_dummy(i2c_parent, ser_addr,
			&max96705_regmap_config,
			&ser, &ser_map);
	if (ret != 0) {
		dev_err(dev, "Error creating serializer dummy");
		return ret;
	}
	ret = serdes_create_dummy(i2c_parent, sen_addr,
			&regmap_config_16bit,
			&sen, &sen_map);
	if (ret != 0) {
		dev_err(dev, "Error creating sensor dummy");
		return ret;
	}

	do {
		ret = max9286_play_init_sequence(dev, pl, pl_len,
				priv->serdes.map, ser_map, sen_map);
		try_cnt += 1;
		if (ret < 0) {
			dev_warn(dev, "Retrying link initialization");
			//priv->reset(priv);
			// TODO:
			// serdes_des_reset(priv);
		}
	} while (ret != 0 && try_cnt < max_retries);
	if (ret < 0) {
		dev_err(dev, "Link initialization failed.");
		/* Disable the link so the SER doesn't interfere with
		 * the other SERs being probed
		 */
		max9286_disable_link(priv, MAX9286_CH0);
	} else {
		dev_dbg(dev, "Link is up");
	}

	regmap_exit(sen_map);
	regmap_exit(ser_map);
	i2c_unregister_device(ser);
	i2c_unregister_device(sen);
	return ret;
}

/* static int max9286_of_address_translations(struct device_node *node, */
/* 					   u32 *src, u32 *dst) */
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

/* The serializer address is changed by the initial configuration sequence */
static int max9286_change_device_addr_noop(struct serdes *serdes,
					   u32 src, u32 dst)
{
	return 0;
}

static int max9286_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct deserializer *priv = NULL;
	int ret = 0;

	dev_dbg(&client->dev, "Probing %s.\n",
			max9286_idtable[id->driver_data].name);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "Failed to allocate memory.\n");
		return -ENOMEM;
	}

	priv->serdes.client = client;
	i2c_set_clientdata(client, priv);

	/* TODO: CHANGE THE RESET FUNCTION.
	 * 9286 apparently can't be reset by setting a bit in a register
	 */
	/* TODO Reset it via gpio */
	//priv->reset = max9286_soft_reset;
	priv->serdes.map = devm_regmap_init_i2c(client, &max9286_regmap_config);
	if (IS_ERR(priv->serdes.map)) {
		dev_err(&client->dev, "Failed to create regmap.\n");
		return PTR_ERR(priv->serdes.map);
	}

	ret = serdes_mux_init(&priv->serdes, DES_MAX_CHANNELS);
	if (ret != 0)
		return ret;

	ret = serdes_enable_regulators(priv, max9286_disable_link, MAX9286_CH0);
	if (ret != 0)
		return ret;
	max9286_set_csi_parameters(priv, client->dev.of_node);
	max9286_enable_link(priv, MAX9286_CH0);

	ret = max9286_gmsl1_init(priv, client->dev.of_node,
			max9286_max96705, ARRAY_SIZE(max9286_max96705));
	if (ret != 0)
		return ret;

	dev_dbg(&client->dev, "Scanning for children nodes in DT..\n");
	/* Scan the device tree for i2c slave buses */
	ret = serdes_of_scan_links(&priv->serdes,
			MAX9286_NUM_CHANNELS, max9286_change_device_addr_noop);
	if (ret == 0) {
		dev_info(&client->dev, "probed");
	}
	return ret;
}

static int max9286_remove(struct i2c_client *client)
{
	struct deserializer *priv = NULL;

	dev_info(&client->dev, "Removing MAX929x.");

	priv = i2c_get_clientdata(client);
	if (priv->reset_gpio)
		gpiod_put(priv->reset_gpio);

	serdes_disable_regulators(priv);

	/* TODO: Also, remove all the serializers */
	return 0;
}

static struct i2c_driver max9286_driver = {
	.driver = {
		.name = "max9286",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max9286_of_match),
	},
	.probe = max9286_probe,
	.remove = max9286_remove,
	.id_table = max9286_idtable,
};

module_i2c_driver(max9286_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Catalin Petrescu <cpetrescu@d3engineering.com>");
MODULE_DESCRIPTION("Maxim MAX9286/MAX96705 GMSL1 Deserializer/Serializer driver");
MODULE_VERSION(D3_JETSON_BSP_VERSION);

