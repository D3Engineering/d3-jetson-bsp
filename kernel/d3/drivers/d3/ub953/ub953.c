/**
 * @author Greg Rowe <growe@d3engineering.com>
 * 
 * ub953 FPDLINK-III serializer driver
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
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include <d3/d3-jetson-bsp.h>

/**
 * If @p expr evalutes to non-zero assign it to @p err and return @p err
 */
#define TRY(err, expr) do {\
		err = expr; \
		if (err) { \
			return err; \
		} \
	} while (false)

#define TRY_MUTEX(mutex, err, expr) do {\
		err = expr; \
		if (err) { \
			mutex_unlock(mutex); \
			return err; \
		} \
	} while (false)

#define TRY_MEM(mem, expr) do {\
		mem = expr; \
		if (IS_ERR(mem)) \
			return PTR_ERR(mem); \
	} while (false)

/**
 * Symbolic register names (see datasheet)
 */
enum {
	UB953_REG_RESET_CTL       = 0x01,
	UB953_GENERAL_CFG         = 0x02,
	UB953_REG_CLKOUT_CTRL0    = 0x06,
	UB953_REG_CLKOUT_CTRL1    = 0x07,
	UB953_REG_LOCAL_GPIO_DATA = 0x0d,
	UB953_REG_GPIO_INPUT_CTRL = 0x0e,
	UB953_REG_BCC_CONFIG      = 0x32,
	UB953_REG_REV_MASK_ID     = 0x50,
	UB953_REG_IND_ACC_CTL     = 0xB0,
	UB953_REG_IND_ACC_ADDR    = 0xB1,
	UB953_REG_IND_ACC_DATA    = 0xB2,
	UB953_REG_FPD3_TX_ID0     = 0xf0,
};

/**
 * These values are used for programming CSI_CTL1 register
 */
enum ub953_csi_lane_count {
	UB953_CSI_LANE_COUNT_4 = 3,
	UB953_CSI_LANE_COUNT_2 = 1,
	UB953_CSI_LANE_COUNT_1 = 0,

	CSI_LANE_COUNT_ENDMARKER
};

/**
 * Device driver information from device tree which is used to
 * instantiate a sensor driver.
 */
struct ub953_devinfo
{
	/**
	 * compatible string name
	 */
	char type[I2C_NAME_SIZE];

	/**
	 * alias address
	 */
	u32 addr;

	/**
	 * physical device address 
	 */
	u32 physical_addr;

	/**
	 * device tree node
	 */
	struct device_node *of_node;
};


struct ub953_cfg
{
	int n_lanes;
	int continuous_clock;
	int div_m_val;
	int hs_clk_div;
	int div_n_val;
	int gpio_rmten;
	int gpio_out_src;
	int gpio_out_en;
	int gpio_in_en;
	int i2c_voltage_sel;
	int fsync_gpio;
	int wait_for_self_configure;
};



/**
 * The device has 8 bit addresses and 8 bit values
 */
static struct regmap_config ub953_regmap_cfg =
{
	.reg_bits = 8,
	.val_bits = 8,
};


/**
 * ub953 driver class
 */
struct ub953
{
	struct i2c_client *client;
	struct device *dev;
	struct regmap *map;
	struct mutex indirect_access_lock;

	/**
	 * sensor instance loaded by this driver
	 */
	struct i2c_client *sensor;

	struct ub953_cfg cfg;
};



/* static int ub953_reg_read(struct ub953 *self, u8 addr, u8 *out) */
/* { */
/* 	int err = 0; */
/* 	unsigned int val = 0; */
/* 	if((err = regmap_read(self->map, addr, &val))) */
/* 	{ */
/* 		dev_warn(self->dev, "regmap_read returned %d", err); */
/* 		return err; */
/* 	} */
/* 	*out = val; */
/* 	dev_dbg(self->dev, "read: %#.2x=%#.2x", addr, *out); */
/* 	return 0; */
/* } */


/* static int ub953_reg_write(struct ub953 *self, u8 addr, u8 val) */
/* { */
/* 	int err = 0; */
/* 	dev_dbg(self->dev, "write: %#.2x=%#.2x", addr, val); */
/* 	if((err = regmap_write(self->map, addr, val))) */
/* 	{ */
/* 		dev_err(self->dev, "write error: %d", err); */
/* 		return err; */
/* 	} */
/* 	return 0; */
/* } */


/** 
 * Allocates device managed memory
 *
 * @param dev device to allocate from
 * @param len desired amount of memory
 * @param out where to store pointer
 *
 * @return 0 on success
 */
static int ub953_kzalloc(struct device *dev, size_t len, void *out)
{
	void **real_out = out;
	if(!(*real_out = devm_kzalloc(dev, len, GFP_KERNEL)))
	{
		dev_err(dev, "memory allocation failure");
		return -ENOMEM;
	}
	return 0;
}


/** 
 * Creates a device managed register map instance
 *
 * @param client managing device
 * @param cfg register map configuration
 * @param out_map output
 *
 * @return 0 on success
 */
static int ub953_regmap_init(struct i2c_client *client,
			     struct regmap_config *cfg,
			     struct regmap **out_map)
{
	if(!(*out_map = devm_regmap_init_i2c(client, cfg)))
	{
		dev_err(&client->dev, "regmap_init failed");
		return -EINVAL;
	}
	return 0;
}

/** 
 * Loads device information from a device tree node
 *
 * @param self driver instance
 * @param node device tree node
 * @param out where to store results
 *
 * @return 0 on success
 */
static int ub953_devinfo_load(struct ub953 *self,
			      struct device_node *node,
			      struct ub953_devinfo *out)
{
	int err = 0;
	const char *type = NULL;

	TRY(err, of_property_read_u32(node, "reg", &out->addr));
	TRY(err, of_property_read_u32(node, "physical-addr",
				      &out->physical_addr));
	TRY(err, of_property_read_string(node, "compatible", &type));
	/* TRY(err, of_property_read_string(node, "type", &type)); */
	strncpy(out->type, type, sizeof(out->type));

	out->of_node = node;
	dev_dbg(self->dev, "reg=%#.2x, physical-addr=%#.2x, type=(%s)",
		out->addr, out->physical_addr, out->type);
	return 0;
}


/** 
 * Loads sensor settings from device tree
 *
 * @param self driver instance
 * @param out_info where to write values to
 *
 * @return 0 on success
 */
static int ub953_sensor_load(struct ub953 *self, struct ub953_devinfo *out_info)
{
	struct device_node *node = self->dev->of_node;
	struct device_node *child = NULL;
	int count = 0;
	int err = 0;
	
	for_each_available_child_of_node(node, child)
	{
		if(count > 1)
		{
			dev_warn(self->dev,
				 "%s: expecting 1 node",
				 child->name);
			return 0;
		}
		++count;
		TRY(err, ub953_devinfo_load(self, child, out_info));
	}
	if(count != 1)
	{
		dev_warn(self->dev, "%s: expecting one sensor but saw %d!",
			 node->name, count);
		return -EINVAL;
	}	
	return 0;
}


/** 
 * Instantiates an i2c_client for the sensor
 *
 * @param self driver instance
 * @param devinfo device driver information (for loading sensor)
 *
 * @return 0 on success
 */
static int ub953_sensor_create(struct ub953 *self,
			       const struct ub953_devinfo *devinfo)
{
	struct i2c_adapter *adap = to_i2c_adapter(self->client->dev.parent);
	struct i2c_board_info i2c_info = {0};

	strncpy(i2c_info.type, devinfo->type, sizeof(i2c_info.type));
	i2c_info.addr = devinfo->addr;
	i2c_info.of_node = devinfo->of_node;

	self->sensor = i2c_new_device(adap, &i2c_info);
	if(!self->sensor)
	{
		dev_err(self->dev,
			"could not create i2c client for sensor"
			" adapter=%s type=%s addr=%#.2x",
			adap->name, i2c_info.type, i2c_info.addr);
		return -ENOMEM;
	}
	
	return 0;
}

static void ub953_cfg_dump(struct ub953 *self, const struct ub953_cfg *cfg)
{
	dev_dbg(self->dev,
		"div-m-val=%d"
		" hs-clk-div=%d"
		" div-n-val=%d"
		" gpio-rmten=%d"
		" gpio-out-src=%d"
		" gpio-out-en=%d"
		" gpio-in-en=%d"
		" i2c-voltage-sel=%d",
		cfg->div_m_val,
		cfg->hs_clk_div,
		cfg->div_n_val,
		cfg->gpio_rmten,
		cfg->gpio_out_src,
		cfg->gpio_out_en,
		cfg->gpio_in_en,
		cfg->i2c_voltage_sel);

}

static void ub953_of_read_optional(struct device_node *node, const char *name, int *val, int default_val)
{
	if (of_property_read_s32(node, name, val) != 0)
		*val = default_val;
}

static int ub953_wait_for_data(struct ub953 *self)
{
	int err;
	unsigned int val = 0;
	unsigned int tries = 40;  /* 2 seconds */

	do {
		err = regmap_read(self->map, 0x61, &val);
		dev_dbg(self->dev, "Waiting for data... (VC=%x, Data ID=%x)\n", ((val >> 6) & 0x3), (val & 0x3F));
		if (val == 0) {
			usleep_range(50*1000,50*1000);
		}
		/* check for timeout */
		if (--tries == 0) {
			dev_dbg(self->dev, "Timeout waiting for CSI data.");
			break;
		}
	} while ((err == 0 || err == -ETIMEDOUT) && val == 0);

	return err;
}

static int ub953_reset(struct ub953 *self, const struct ub953_cfg *cfg)
{
	int err;
	int is_known;
	int i;
	int tries;
	static const char *VALID_IDS[] = {
		"_UB953",
	};

	char fpd3_tx_id[6];

	if (cfg->wait_for_self_configure)
		TRY(err, ub953_wait_for_data(self));

	/* No reset gpio, try digital reset */
	TRY(err, regmap_write(self->map, UB953_REG_RESET_CTL, 0x01));
	usleep_range(4 * 1000, 5 * 1000);

	is_known = false;
	for (tries = 50; --tries > 0; ) {
		err = regmap_bulk_read(self->map, UB953_REG_FPD3_TX_ID0, fpd3_tx_id,
					  ARRAY_SIZE(fpd3_tx_id));

		if (err == 0)
			break;

		usleep_range(10 * 1000, 10 * 1000);
	}

	if (err < 0)
		return err;

	for(i=0; i<ARRAY_SIZE(VALID_IDS); ++i) {
		if (memcmp(fpd3_tx_id, VALID_IDS[i], sizeof(fpd3_tx_id)) == 0) {
			is_known = true;
			break;
		}
	}

	if(!is_known) {
		dev_warn(self->dev, "Unexpected FPD3_TX_ID: %.*s\n",
			 (int)ARRAY_SIZE(fpd3_tx_id), fpd3_tx_id);
	}

	return 0;
}

static int ub953_indirect_write(struct ub953 *self, unsigned int page, unsigned int addr, unsigned int data)
{
	int err;
	struct mutex *lock = &self->indirect_access_lock;

	mutex_lock(lock);
	TRY_MUTEX(lock, err, regmap_write(self->map, UB953_REG_IND_ACC_CTL, (page & 0x7) << 2));
	TRY_MUTEX(lock, err, regmap_write(self->map, UB953_REG_IND_ACC_ADDR, addr));
	TRY_MUTEX(lock, err, regmap_write(self->map, UB953_REG_IND_ACC_DATA, data));
	mutex_unlock(lock);

	return 0;
}

static int ub953_init(struct ub953 *self, const struct ub953_cfg *cfg)
{
	int err;
	enum ub953_csi_lane_count lane_cnt;

	TRY(err, ub953_reset(self, cfg));

	ub953_cfg_dump(self, cfg);

	switch (cfg->n_lanes) {
		case 1:
			lane_cnt = UB953_CSI_LANE_COUNT_1;
			break;
		case 2:
			lane_cnt = UB953_CSI_LANE_COUNT_2;
			break;
		case 4:
			lane_cnt = UB953_CSI_LANE_COUNT_4;
			break;
		default:
			dev_err(self->dev, "n_lanes invalid value (%u)", cfg->n_lanes);
			return -EINVAL;
	}

	TRY(err, regmap_update_bits(self->map,
		UB953_GENERAL_CFG,
		(0x1 << 6) |
			(0x3 << 4) |
			0x1,
		((cfg->continuous_clock & 0x1) << 6) |
			((lane_cnt & 0x3) << 4) |
			(cfg->i2c_voltage_sel & 0x01)));

	if (cfg->hs_clk_div >= 0 && cfg->div_m_val >= 0 && cfg->div_n_val >= 0) {
		TRY(err, regmap_write(self->map,
			UB953_REG_CLKOUT_CTRL0,
			(cfg->hs_clk_div & 0x07) << 5 |
				(cfg->div_m_val & 0x1F)));
		TRY(err, regmap_write(self->map,
			UB953_REG_CLKOUT_CTRL1,
			(cfg->div_n_val & 0xFF)));
	}

	if (cfg->gpio_rmten >= 0 && cfg->gpio_out_src >= 0) {
		TRY(err, regmap_write(self->map,
			UB953_REG_LOCAL_GPIO_DATA,
			((cfg->gpio_rmten & 0x0F) << 4) |
				(cfg->gpio_out_src & 0x0F)));
	}

	if (cfg->gpio_out_en >= 0 && cfg->gpio_in_en >= 0) {
		TRY(err, regmap_write(self->map,
			UB953_REG_GPIO_INPUT_CTRL,
			((cfg->gpio_out_en & 0x0F) << 4) |
				(cfg->gpio_in_en & 0x0F)));
	}

	// No documentation on what this is or why it is needed?
	TRY(err, ub953_indirect_write(self, 1, 0x08, 0x07));

	return 0;
}

static int ub953_cfg_load(struct ub953 *self, struct ub953_cfg *out)
{
	struct device_node *node = self->client->dev.of_node;

	ub953_of_read_optional(node, "csi-lane-count", &out->n_lanes, -1);
	if (out->n_lanes < 1 || out->n_lanes > 4) {
		dev_err(self->dev, "invalid or missing csi-lane-count");
		return -EINVAL;
	}

	ub953_of_read_optional(node, "csi-continuous-clock", &out->continuous_clock, -1);
	if (out->continuous_clock < 0 || out->continuous_clock > 1) {
		dev_err(self->dev, "invalid or missing csi-continuous-clock");
		return -EINVAL;
	}

	out->wait_for_self_configure = of_property_read_bool(node, "wait-for-self-configure");

	ub953_of_read_optional(node, "div-m-val", &out->div_m_val, -1);
	ub953_of_read_optional(node, "hs-clk-div", &out->hs_clk_div, -1);
	ub953_of_read_optional(node, "div-n-val", &out->div_n_val, -1);
	ub953_of_read_optional(node, "gpio-rmten", &out->gpio_rmten, -1);
	ub953_of_read_optional(node, "gpio-out-src", &out->gpio_out_src, -1);
	ub953_of_read_optional(node, "gpio-out-en", &out->gpio_out_en, -1);
	ub953_of_read_optional(node, "gpio-in-en", &out->gpio_in_en, -1);
	ub953_of_read_optional(node, "i2c-voltage-sel", &out->i2c_voltage_sel, -1);
	ub953_of_read_optional(node, "fsync-gpio", &out->fsync_gpio, -1);

	return 0;
}


/** 
 * Called by Linux when a driver instance is loaded/created.
 *
 * @param client 
 * @param id 
 *
 * @return 0 on success
 */
static int ub953_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int err = 0;
	struct ub953 *self = NULL;
	struct device *dev = &client->dev;
	struct device_node *node = client->dev.of_node;
	struct ub953_devinfo sensor_info;
	
	dev_dbg(dev, "probe");

	if(!IS_ENABLED(CONFIG_OF) || !node)
	{
		dev_err(dev, "of not enabled node=%p", node);
		return -EINVAL;
	}

	/* allocate memory for 'self' data */
	TRY(err, ub953_kzalloc(dev, sizeof(*self), &self));
	i2c_set_clientdata(client, self);
	self->client = client;
	self->dev = &client->dev;
	self->dev->platform_data = self;

	mutex_init(&self->indirect_access_lock);

	memset(&sensor_info, 0, sizeof(sensor_info));
	TRY(err, ub953_sensor_load(self, &sensor_info));

	TRY(err, ub953_regmap_init(client, &ub953_regmap_cfg, &self->map));
	memset(&self->cfg, 0, sizeof(self->cfg));
	TRY(err, ub953_cfg_load(self, &self->cfg));
	TRY(err, ub953_init(self, &self->cfg));

	TRY(err, ub953_sensor_create(self, &sensor_info));

	dev_info(dev, "probe success");
	return 0;
}


/** 
 * Called when Linux unloaded an instance
 *
 * @param client 
 *
 * @return 0 on success
 */
static int ub953_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ub953 *self = i2c_get_clientdata(client);
	
	dev_info(dev, "remove");

	if(self->sensor)
	{
		i2c_unregister_device(self->sensor);
	}
	
	return 0;
}

/* On some imagers (e.g. OV10640), the frame sync input cannot be started until
 * after the imager has been taken out of standby. So, imagers can call this
 * function to start frame sync */
int ub953_set_frame_sync_enable(struct device *dev, bool enabled)
{
	int err = 0;
	unsigned int val;

	struct ub953 *self = (struct ub953*)(dev->platform_data);
	val = ((self->cfg.gpio_rmten & 0x0F) << 4) |
		(self->cfg.gpio_out_src & 0x0F);

	if (self->cfg.fsync_gpio >= 0) {
		if (enabled) {
			val |= ((1 << self->cfg.fsync_gpio) << 4);
		} else {
			val &= ~((1 << self->cfg.fsync_gpio) << 4);
		}
		TRY(err, regmap_write(self->map, UB953_REG_LOCAL_GPIO_DATA, val));
	}
	return err;
}
EXPORT_SYMBOL(ub953_set_frame_sync_enable);

static const struct i2c_device_id ub953_id[] =
{
	{"ub953", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ub953_id);


static struct of_device_id ub953_of_match[] =
{
	{ .compatible = "d3,ub953"},
	{ },
};
MODULE_DEVICE_TABLE(of, ub953_of_match);


static struct i2c_driver ub953_driver =
{
	.driver =
	{
		.name = "ub953",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ub953_of_match),
	},
	.probe = ub953_probe,
	.remove = ub953_remove,
	.id_table = ub953_id,
};
module_i2c_driver(ub953_driver);
MODULE_DESCRIPTION("Driver for TI UB953 FPDLINK-III serializer");
MODULE_VERSION(D3_JETSON_BSP_VERSION);
MODULE_AUTHOR("Greg Rowe <growe@d3engineering.com>");
MODULE_LICENSE("GPL v2");
