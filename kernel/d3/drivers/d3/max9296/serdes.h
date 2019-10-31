/*
 * serdes.h
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

#ifndef _SERDES_H_
#define MAX_DEBUG 1
#define _SERDES_H_

#include <linux/types.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>

#define DES_MAX_CHANNELS 4
#define SER_MAX_CHANNELS 1

#define DES_VDD_REGULATOR_NAME "vdd"
#define DES_POC_REGULATOR_NAME "gmsl"


enum link_type {
	LINK_TYPE_GMSL1 = 0,
	LINK_TYPE_GMSL2,
	LINK_TYPE_FPDLINK,
};

struct csi_config {
	unsigned num_lanes;
	unsigned freq_mhz;
};

struct serial_config {
	unsigned forward_bit_rate; /* forward channel bit rate */
	unsigned back_bit_rate; /* back channel bit rate */
};

struct serdes {
	struct i2c_client *client;
	struct regmap *map;
	struct i2c_mux_core *mux_core;
	unsigned num_i2c_maps; /* Number of active i2c address translations */
	struct csi_config csi_config; /* CSI configuration */
	struct serial_config serial_config; /* GMSL configuration */
};

struct deserializer {
	struct serdes serdes;

	struct gpio_desc *reset_gpio;
	struct regulator *vdd_regulator; /* feeds the deserializer */
	struct regulator *poc_regulator; /* feeds the serializer, imager */
	int irq;
	bool regulator_enabled;
	enum link_type link_type;
	unsigned fsync_mode;
};

struct serializer {
	struct serdes serdes;
};

/* The deserializer register pairs that set the i2c address translations */
struct ser_i2c_map_regs {
	unsigned src;
	unsigned dst;
};

#if 0
struct serdes_node_list {
	struct device_node *node;
	struct list_head list;
};
#endif

/* Structure that allows me to send a configuration sequence to
 * deserializer, serializer and imager.
 */
struct serdes_playlist {
	const char target; /* 'd': deserializer. 's': serializer, 'i': imager */
	const struct reg_sequence *seq;
	unsigned len;
};

#define PLAYLIST_INIT(to, seq_name) {.target = (to), .seq = (seq_name), \
	.len = ARRAY_SIZE(seq_name)}

typedef int (*i2c_translation_func_t)(struct serdes *serdes, u32 src, u32 dst);


extern int serdes_get_reset_gpio_and_reset(struct deserializer *serdes);
extern int serdes_enable_regulator(struct device *dev, const char *reg_name,
		struct regulator **preg);
extern int serdes_enable_regulators(struct deserializer *priv,
		int (*disable_link)(struct deserializer*, unsigned), unsigned link);
extern int serdes_disable_regulators(struct deserializer *priv);

extern int serdes_select_i2c_chan(struct i2c_mux_core *core, u32 chan);
extern int serdes_deselect_i2c_chan(struct i2c_mux_core *core, u32 chan);

extern int serdes_create_dummy(struct i2c_adapter *adapter, unsigned addr,
		const struct regmap_config *regmap_config,
		struct i2c_client **serp, struct regmap **mapp);

extern int serdes_of_scan_links(struct serdes *serdes,
                                unsigned max_links,
                                i2c_translation_func_t add_i2c_translation);

extern int serdes_mux_init(struct serdes *serdes, unsigned max_channels);
#endif /* _SERDES_H_ */
