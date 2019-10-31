/*
 * max9296.c - Maxim MAX9296 GMSL2/GMSL1 to CSI-2 Deserializer
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
#include <linux/gpio/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>

#include <d3/d3-jetson-bsp.h>

#include "max9296.h"
#include "max96705.h"
#include "max9296_tables.h"
#include "max96705_tables.h"

#include "serdes.h"

enum {
	CHIP_MAX9296 = 0,
};

static struct i2c_device_id max9296_idtable[] = {
	{"max9296", CHIP_MAX9296},
	{},
};
MODULE_DEVICE_TABLE(i2c, max9296_idtable);

static struct of_device_id max9296_of_match[] = {
	{ .compatible = "maxim,max9296"},
	{},
};
MODULE_DEVICE_TABLE(of, max9296_of_match);

/* I2C address translation related structures */
struct max929x_addr_map_list {
	u32 src;
	u32 dst;
	struct list_head list;
};

struct max929x_i2c_map_regs {
	unsigned int src;
	unsigned int dst;
};

/* TODO: This is shared by 2 functions. refactor. */
static struct reg_sequence max96705_seraddr[] = {
	{ MAX96705_REG_SERADDR, MAX96705_REG_SERADDR_DEFAULT },
};

/* TODO: This is shared by 2 functions. refactor. */
static struct reg_sequence max96705_imgaddr[] = {
	{ MAX96705_REG_SRC_A, 0 },
	{ MAX96705_REG_DST_A, 0 },
};

/* MAX9296-MAX96705 initialization playlist */
static const struct serdes_playlist max9296_max96705[] = {
	/* reverse channel setup */
	PLAYLIST_INIT('d', max9296_custom_rcs),
	PLAYLIST_INIT('s', max96705_en_him),
	PLAYLIST_INIT('d', max9296_en_him),
	PLAYLIST_INIT('s', max96705_en_cfg),
	PLAYLIST_INIT('d', max9296_dis_locack),
	/* pre camera setup */
	PLAYLIST_INIT('d', max9296_dis_csi),
	PLAYLIST_INIT('s', max96705_en_dbl_hven),
	PLAYLIST_INIT('d', max9296_en_dbl_hven),
	/* sensor setup */
	/* post camera setup */
	PLAYLIST_INIT('s', max96705_en_link),
	PLAYLIST_INIT('d', max9296_en_csi),
	/* i2c address remapping */
	PLAYLIST_INIT('s', max96705_imgaddr),
	PLAYLIST_INIT('s', max96705_seraddr),
};

#ifdef MAX_DEBUG
/* Let some registers be read via debugfs */
static const struct regmap_range max9296_yes_ranges[] = {
	regmap_reg_range(0, 6),
	regmap_reg_range(0xD, 0x13),
	regmap_reg_range(0x18, 0x25),
	regmap_reg_range(0x40, 0x47), /* I2C */
	regmap_reg_range(0x1D9, 0x1DA),
	regmap_reg_range(0x2B0, 0x2B2), /* GPIOs */
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

static const struct regmap_access_table max9296_regmap_rd_access_table = {
	.yes_ranges = max9296_yes_ranges,
	.n_yes_ranges = ARRAY_SIZE(max9296_yes_ranges),
};
#endif /* MAX_DEBUG */

static const struct regmap_config max9296_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
#ifdef MAX_DEBUG
	.max_register = 0x1f17,
	.rd_table = &max9296_regmap_rd_access_table,
#endif
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
static int max9296_regmap_read(struct deserializer *priv, u32 reg, u32 *val)
{
	struct device *dev = &priv->serdes.client->dev;
	struct regmap *map = priv->serdes.map;
	int ret = 0;

	reg &= 0xFFFF;
	ret = regmap_read(map, reg, val);
	if (ret)
		dev_err(dev, "Failed reading register 0x%x\n", reg);
	return ret;
}

/*
 * Wrapper around regmap_write that prints errors.
 */

static int max9296_regmap_write(struct deserializer *priv, u32 reg, u32 val)
{
	struct device *dev = &priv->serdes.client->dev;
	struct regmap *map = priv->serdes.map;
	int ret = 0;

	reg &= 0xFFFF;
	val &= 0xFF;
	ret = regmap_write(map, reg, val);
	if (ret)
		dev_err(dev, "Failed writing register 0x%x\n", reg);
	return ret;
}

/* Set GMSL link state for 9296 */
static int max9296_set_link_state(struct deserializer* priv, unsigned link,
		int state)
{
	int ret;
	u32 reg_val;
	const u16 regs[] = {MAX9296_REG_GMSL1_4_A, MAX9296_REG_GMSL1_4_B};
	u8 mask;

	if (link >= MAX9296_NUM_CHANNELS)
		return -EINVAL;
	ret = max9296_regmap_read(priv, regs[link], &reg_val);
	if (ret != 0)
		return ret;
	mask = MAX9296_REVCCEN | MAX9296_FWDCCEN;
	reg_val = state? reg_val | mask : reg_val & ~mask;
	return max9296_regmap_write(priv, regs[link], reg_val);
}

static inline int max9296_disable_link(struct deserializer *priv, unsigned link)
{
	return max9296_set_link_state(priv, link, false);
}

static inline int max9296_enable_link(struct deserializer *priv, unsigned link)
{
	return max9296_set_link_state(priv, link, true);
}

/* Reset all */
static int max9296_soft_reset(struct deserializer *priv) {
	const struct reg_sequence soft_reset[] = {
		{ 0x0010, 0x80, 200000 },
	};
	return regmap_multi_reg_write(priv->serdes.map, soft_reset,
			ARRAY_SIZE(soft_reset));
}

/* One-shot reset; resets the GMSL link but keeps the register settings */
/* static int max9296_one_shot_reset(struct deserializer *priv) */
/* { */
/* 	const struct reg_sequence one_shot_reset[] = { */
/* 		{ 0x0010, 0x30, 100000 }, */
/* 	}; */

/* 	return regmap_multi_reg_write(priv->map, one_shot_reset, */
/* 			ARRAY_SIZE(one_shot_reset)); */
/* } */

static int max9296_set_lane_count(struct deserializer *priv)
{
	struct device *dev = &priv->serdes.client->dev;
	unsigned reg_val = 0;
	unsigned regs[] = { 0x40A, 0x44A, 0x48A, 0x4CA }; /* MIPI_TX10 */
	int i = 0;

	/* The MAX9296 register documentation is poorly written. Until it gets
	 * better, the values that go in the MIPI_TX10 are a sort of best guess.
	 */
	switch (priv->serdes.csi_config.num_lanes) {
	case 1:
		reg_val = 0;
		break;
	case 2:
		reg_val = 0x40;
		break;
	case 4:
		reg_val = 0xC0;
		break;
	default:
		dev_err(dev, "Invalid number of CSI lanes: %d",
				priv->serdes.csi_config.num_lanes);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(regs); i++)
		max9296_regmap_write(priv, regs[i], reg_val);
	return 0;
}

static int max9296_set_csi_freq(struct deserializer *priv)
{
	unsigned f = 16; /* frequency as multiple of 100MHz */
	unsigned reg_val = 0;
	unsigned regs[] = {
		MAX9296_REG_BACKTOP22, /* phy0 */
		MAX9296_REG_BACKTOP25, /* phy1 */
		MAX9296_REG_BACKTOP28, /* phy2 */
		MAX9296_REG_BACKTOP31, /* phy3 */
	};
	int i = 0;
	int ret = 0;

	if (priv->serdes.csi_config.freq_mhz != 0) {
		f = (priv->serdes.csi_config.freq_mhz / 100 ) & 0x1F;
		for (i = 0; i < ARRAY_SIZE(regs); i++) {
			ret = max9296_regmap_read(priv, regs[i], &reg_val);
			if (ret)
				continue;
			reg_val &= 0xE0;
			reg_val |= f;
			max9296_regmap_write(priv, regs[i], reg_val);
		}
	}
	return 0;
}

static int max9296_set_csi_parameters(struct deserializer *priv, struct device_node *node)
{
	int ret;

	ret = of_property_read_u32(node, "csi-lane-count",
				   &priv->serdes.csi_config.num_lanes);
	if (ret == 0)
		max9296_set_lane_count(priv);
	ret = of_property_read_u32(node, "csi-tx-speed-mbps",
			&priv->serdes.csi_config.freq_mhz);
	if (ret == 0)
		max9296_set_csi_freq(priv);
	return 0;
}

/* TODO: set_gmsl_bit_rates needs to be refactored so it handles both
 * max9296 and max9295.
 */
static int max9296_set_gmsl_bit_rates(struct deserializer *priv)
{
	struct device *dev = &priv->serdes.client->dev;
	struct reg_sequence set_gmsl_speed[] = {
		{.reg = 0x01, .def = 0, .delay_us = 0},
		{.reg = 0x10, .def = 0x31, .delay_us = 100000},
	};
	u8 enc_back = 0; /* encoded rate */
	u8 enc_forward = 0;
	int ret = 0;

	dev_dbg(dev, "Setting bit rates.");
	switch(priv->serdes.serial_config.back_bit_rate) {
	case (187):
		enc_back = MAX9296_BACK_RATE_187_5MHZ;
		break;
	case (375):
		enc_back = MAX9296_BACK_RATE_375MHZ;
		break;
	case (750):
		enc_back = MAX9296_BACK_RATE_750MHZ;
		break;
	case (1500):
		enc_back = MAX9296_BACK_RATE_1500MHZ;
		break;
	default:
		dev_err(dev, "Invalid GMSL TX rate: %d",
				priv->serdes.serial_config.back_bit_rate);
		return -1;

	}
	switch(priv->serdes.serial_config.forward_bit_rate) {
	case (1500):
		enc_forward = MAX9296_FORWARD_RATE_1500MHZ;
		break;
	case (3000):
		enc_forward = MAX9296_FORWARD_RATE_3000MHZ;
		break;
	case (6000):
		enc_forward = MAX9296_FORWARD_RATE_6000MHZ;
		break;
	default:
		dev_err(dev, "Invalid GMSL RX rate: %d",
				priv->serdes.serial_config.forward_bit_rate);
		return -1;

	}
	set_gmsl_speed[0].def = MAX9296_GMSL_RATE(enc_back, enc_forward);
	dev_dbg(dev, "Setting GMSL rate to: 0x%x",
			set_gmsl_speed[0].def);
	ret = regmap_multi_reg_write(priv->serdes.map, set_gmsl_speed,
			ARRAY_SIZE(set_gmsl_speed));
	return ret;
}

/* Search the device tree for i2c address translations */
int max9296_get_child_addr(struct device_node *node,
		struct list_head *list)
{
	struct device_node *link = NULL;
	struct device_node *child = NULL;
	unsigned reg = 0;
	unsigned addr =  0;
	struct max929x_addr_map_list *tmp;
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
			tmp = (struct max929x_addr_map_list *)
				kzalloc(sizeof(struct max929x_addr_map_list), GFP_KERNEL);
			tmp->src = reg;
			tmp->dst = addr;
			list_add(&tmp->list, list);
			found = true;
		}
	}
	return found;
}

/*
 * Change the I2C address of a MAX9295 serializer.
 * When the deserializer can't perform the address translation (and MAX9296
 * seems to be unable to), an alternative is to set change the I2C address by
 * writing the DEV_ADDR register (reg 0) of the serializer.
 * It is a multi step process, so use it only when the other method doesn't
 * work.
 * @priv: max9296 private data
 * @src: the address the client will appear to have after translation
 * @dst: the actual address of the client
 *
 * Return: 0 if the translation was set up successfully, -1 if otherwise.
 */
static int max9296_change_device_addr(struct deserializer *priv, u32 src, u32 dst)
{
	struct device *dev = &priv->serdes.client->dev;
	struct i2c_adapter *adapter =
		to_i2c_adapter(priv->serdes.client->dev.parent);
	struct i2c_client *r_client = NULL;
	struct i2c_client *v_client = NULL;
	struct regmap *r_map = NULL;
	struct regmap *v_map = NULL;
	struct reg_sequence *seq = NULL;
	unsigned seq_len = 0;
	int ret = 0;

	dev_dbg(dev, "%s.\n", __func__);
	if (priv->link_type == LINK_TYPE_GMSL1) {
		dev_dbg(dev, "Don't rewrite addresses for GMSL1!");
		return 0;
	}
	ret = serdes_create_dummy(adapter, dst,
			&max9296_regmap_config,
			&r_client, &r_map);
	if (ret != 0) {
		dev_err(dev, "Error creating dummy device: 0x%x.\n", dst);
		/*TODO: What if the dummy device is created, then regmap
		 * initialization fails?
		 */
		return ret;
	}

	ret = serdes_create_dummy(adapter, src,
			&max9296_regmap_config,
			&v_client, &v_map);
	if (ret != 0) {
		dev_err(dev, "Error creating dummy device: 0x%0x.\n", src);
		goto dev_addr_exit2;
	}
	/* steps 1,2 */
	ret = regmap_multi_reg_write(priv->serdes.map,
				     max9296_enable_link_a_only,
				     ARRAY_SIZE(max9296_enable_link_a_only));
	if(ret < 0) {
		dev_err(dev, "Step 1 failed %d\n", ret);
		goto dev_addr_exit1;
	}

	/* Write DEV_ADDR on MAX9295 client */
	ret = regmap_write(r_map, MAX9296_REG_REG0, src << 1);
	if (ret < 0) {
		dev_err(dev, "Failed writing DEV_ADDR: %d\n", ret);
		goto dev_addr_exit1;
	}
	/* write magic */
	switch(src) {
	case 0x40:
		seq = max9295_assign_address_40;
		seq_len = ARRAY_SIZE(max9295_assign_address_40);
		break;
	case 0x42:
		seq = max9295_assign_address_42;
		seq_len = ARRAY_SIZE(max9295_assign_address_42);
		break;
	case 0x60:
		seq = max9295_assign_address_60;
		seq_len = ARRAY_SIZE(max9295_assign_address_60);
		break;
	case 0x62:
		seq = max9295_assign_address_62;
		seq_len = ARRAY_SIZE(max9295_assign_address_62);
		break;
	default:
		dev_err(dev, "Invalid address: 0x%x\n", src);
		goto dev_addr_exit1;
	}

	ret = regmap_multi_reg_write(v_map, seq, seq_len);
	if(ret < 0) {
		dev_err(dev, "Step 7 failed %d\n", ret);
		goto dev_addr_exit1;
	}

	ret = regmap_multi_reg_write(priv->serdes.map, max9296_enable_link_both,
				     ARRAY_SIZE(max9296_enable_link_both));
	if(ret < 0) {
		dev_err(dev, "Step 9 failed %d\n", ret);
		goto dev_addr_exit1;
	}

dev_addr_exit1:
	if (v_map)
		regmap_exit(v_map);
	i2c_unregister_device(v_client);
dev_addr_exit2:
	if (r_map)
		regmap_exit(r_map);
	i2c_unregister_device(r_client);
	return ret;
}

#define IS_WORTH_RETRYING(ret) ( ret == -ETIMEDOUT \
		|| ret == -EAGAIN \
		|| ret == -EREMOTEIO \
		)

static int max9296_play_init_sequence(struct device *dev,
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
#endif
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

int max9296_gmsl1_init(struct deserializer *priv, struct device_node *node,
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
		ret = max9296_play_init_sequence(dev, pl, pl_len,
						 priv->serdes.map,
						 ser_map, sen_map);
		try_cnt += 1;
		if (ret < 0) {
			dev_warn(dev, "Retrying link initialization");
			max9296_soft_reset(priv);
		}
	} while (ret != 0 && try_cnt < max_retries);
	if (ret < 0) {
		dev_err(dev, "Link initialization failed.");
		/* Disable the link so the SER doesn't interfere with
		 * the other SERs being probed
		 */
		max9296_disable_link(priv, MAX9296_CH0);
	} else {
		dev_dbg(dev, "Link is up");
	}

	regmap_exit(sen_map);
	regmap_exit(ser_map);
	i2c_unregister_device(ser);
	i2c_unregister_device(sen);
	return ret;
}

int max9296_gmsl2_init(struct deserializer *priv, struct device_node *node)
{
	unsigned gmsl_bit_rates[2] = {0, 0};
	int ret;

	dev_dbg(&priv->serdes.client->dev, "%s.\n", __func__);
	ret = of_property_read_u32_array(node, "gmsl-bit-rates",
			gmsl_bit_rates, 2);
	if (ret == 0) {
		priv->serdes.serial_config.forward_bit_rate = gmsl_bit_rates[0];
		priv->serdes.serial_config.back_bit_rate = gmsl_bit_rates[1];
		ret = max9296_set_gmsl_bit_rates(priv);
		if (ret != 0) {
			return ret;
		}
	}

	ret = regmap_multi_reg_write(priv->serdes.map, max9296_set_gpios,
				     ARRAY_SIZE(max9296_set_gpios));
	return ret;
}

/* static int max9296_of_address_translations(struct device_node *node, */
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

/*
 * Scan the children node in the device for client buses. Create an I2C adapter
 * for each link found.
 *
 * @priv: max9296 private data
 */

static int max9296_serializer_is_gmsl1(struct device_node *node)
{
	struct device_node *link = NULL;
	struct device_node *client = NULL;
	const char *compatible;
	int is_gmsl1 = 0;
	int ret;

	for_each_available_child_of_node(node, link) {
		if (strcmp(link->name, "link"))
			continue;
		for_each_available_child_of_node(link, client) {
			ret = of_property_read_string(client, "compatible",
					&compatible);
			if (!ret) {
				ret = strcmp(compatible, "maxim,max96705");
				if (ret == 0)
					is_gmsl1 = 1;
			}
		}
	}
	return is_gmsl1;
}

static int max9296_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct deserializer *priv = NULL;
	int ret = 0;

	dev_dbg(&client->dev, "Probing %s.\n",
			max9296_idtable[id->driver_data].name);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "Failed to allocate memory.\n");
		return -ENOMEM;
	}

	priv->serdes.client = client;
	i2c_set_clientdata(client, priv);

	/* Do chip specific initialization */
	//pr_info("dev: %p, parent: %p\n", dev, dev->parent);
	/* @todo make a serdes common init routine */
	priv->serdes.map = devm_regmap_init_i2c(client, &max9296_regmap_config);
	if (IS_ERR(priv->serdes.map)) {
		dev_err(&client->dev, "Failed to create regmap.\n");
		return PTR_ERR(priv->serdes.map);
	}

	// No priv, no flags (look for i2c-mux in dts)
	ret = serdes_mux_init(&priv->serdes, DES_MAX_CHANNELS);
	if (ret != 0) {
		return ret;
	}


	ret = serdes_enable_regulators(priv, max9296_disable_link, MAX9296_CH0);
	if (ret != 0)
		return ret;

	max9296_set_csi_parameters(priv, client->dev.of_node);
	max9296_enable_link(priv, MAX9296_CH0);

	priv->link_type = max9296_serializer_is_gmsl1(client->dev.of_node)
		? LINK_TYPE_GMSL1 : LINK_TYPE_GMSL2;
	if (priv->link_type == LINK_TYPE_GMSL1)
		ret = max9296_gmsl1_init(priv, client->dev.of_node,
				max9296_max96705, ARRAY_SIZE(max9296_max96705));
	else
		ret = max9296_gmsl2_init(priv, client->dev.of_node);
	if (ret != 0)
		return ret;

	dev_dbg(&client->dev, "Scanning for children nodes in DT..\n");
	/* Scan the device tree for i2c slave buses */
	if ((ret = serdes_of_scan_links(
		     &priv->serdes,
		     MAX9296_NUM_CHANNELS,
		     (i2c_translation_func_t)max9296_change_device_addr)) == 0)

		dev_info(&client->dev, "probed");

	return ret;
}

static int max9296_remove(struct i2c_client *client)
{
	struct deserializer *priv = NULL;

	dev_info(&client->dev, "Removing MAX929x.");

	priv = i2c_get_clientdata(client);

	serdes_disable_regulators(priv);
	i2c_mux_del_adapters(priv->serdes.mux_core);

	/* TODO: Also, remove all the serializers */
	return 0;
}

static struct i2c_driver max9296_driver = {
	.driver = {
		.name = "max9296",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max9296_of_match),
	},
	.probe = max9296_probe,
	.remove = max9296_remove,
	.id_table = max9296_idtable,
};

module_i2c_driver(max9296_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Catalin Petrescu <cpetrescu@d3engineering.com>");
MODULE_DESCRIPTION("Maxim MAX9296A Dual GMSL2/GMSL1 to CSI-2 Deserializer driver");
MODULE_VERSION(D3_JETSON_BSP_VERSION);

