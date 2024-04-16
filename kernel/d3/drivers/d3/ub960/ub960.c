/**
 * @author Greg Rowe <growe@d3engineering.com>
 * @author Josh Watts <jwatts@d3engineering.com>
 *
 * ub960 FPDLINK-III deserializer driver
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
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>

#include <d3/d3-jetson-bsp.h>

#include <d3/ub960.h>
#include <d3/common.h>

/*

des -> link@0 -> ser, sensor, eeprom, etc
       link@n -> ser, sensor, eeprom, etc

+-----+
| des |
+-----+
   |
   |  +--------+
   +--+ link@0 |
   |  +--------+
   |         |    +-----+
   |         +----+ ser |
   |         |    +-----+
   |         |
   |         |    +--------+
   |         +----+ sensor |
   |         |    +--------+
   |         |
   |         |    +--------+
   |         +----+ eeprom |
   |              +--------+
   |
   |
   |
   |
   |
   |  +--------+
   +--+ link@n |
      +--------+
             |    +-----+
             +----+ ser |
             |    +-----+
             |
             |    +--------+
             +----+ sensor |
             |    +--------+
             |
             |    +--------+
             +----+ eeprom |
                  +--------+
*/

/*
 * Default frame sync timings used if not specified in device tree
 */
#define FRAME_SYNC_DEFAULT_HIGH_TIME_US   (15000)
#define FRAME_SYNC_DEFAULT_LOW_TIME_US    (35000)

/**
 * Symbolic register names (see datasheet)
 */
enum {
	UB960_REG_RESET_CTL	= 0x01,
	UB960_REG_REV_MASK_ID	= 0x03,
	UB960_REG_RX_PORT_CTL	= 0x0c,
	UB960_REG_IO_CTL	= 0x0d,
	UB960_REG_INTERRUPT_CTL	= 0x23,
	UB960_REG_INTERRUPT_STS	= 0x24,
	UB960_REG_FPD3_PORT_SEL = 0x4c,
	UB960_REG_RX_PORT_STS1	= 0x4d,
	UB960_REG_RX_PORT_STS2	= 0x4e,
	UB960_REG_BCC_CONFIG	= 0x58,
	UB960_REG_CSI_VC_MAP	= 0x72,
	UB960_REG_CSI_RX_STS	= 0x7a,
	UB960_REG_CSI_PORT_SEL	= 0x32,
	UB960_REG_CSI_CTL1	= 0x33,
	UB960_REG_CSI_CTL2	= 0x34,
	UB960_REG_FWD_CTL1	= 0x20,
	UB960_REG_GPIO0_PIN_CTL = 0x10,
	UB960_REG_GPIO1_PIN_CTL = 0x11,
	UB960_REG_GPIO2_PIN_CTL = 0x12,
	UB960_REG_GPIO3_PIN_CTL = 0x13,
	UB960_REG_GPIO4_PIN_CTL = 0x14,
	UB960_REG_GPIO5_PIN_CTL = 0x15,
	UB960_REG_GPIO6_PIN_CTL = 0x16,
	UB960_REG_GPIO7_PIN_CTL = 0x17,
	UB960_REG_FS_CTL        = 0x18,
	UB960_REG_FS_HIGH_TIME_1 = 0x19,
	UB960_REG_FS_HIGH_TIME_0 = 0x1a,
	UB960_REG_FS_LOW_TIME_1 = 0x1b,
	UB960_REG_FS_LOW_TIME_0 = 0x1c,
	UB960_REG_SFILTER_CFG   = 0x41,
	UB960_REG_AEQ_CTL       = 0x42,
	UB960_REG_FV_MIN_TIME	= 0xbc,
	UB960_REG_SER_ID        = 0x5b,
	UB960_REG_SER_ALIAS_ID  = 0x5c,
	UB960_REG_SLAVE_ID0	= 0x5d,
	UB960_REG_SLAVE_ALIAS0	= 0x65,
	UB960_REG_PORT_CONFIG	= 0x6d,
	UB960_REG_BC_GPIO_CTL0  = 0x6e,
	UB960_REG_PORT_CONFIG2	= 0x7c,
	UB960_REG_AEQ_MIN_MAX	= 0xd5,
	UB960_REG_PORT_ICR_HI	= 0xd8,
	UB960_REG_PORT_ICR_LO	= 0xd9,
	UB960_REG_CSI_PLL_CTL	= 0x1f,
	UB960_REG_IND_ACC_CTL	= 0xb0,
	UB960_REG_IND_ACC_ADDR	= 0xb1,
	UB960_REG_IND_ACC_DATA	= 0xb2,
	UB960_REG_LINK_ERROR_CNT = 0xb9,
	UB960_REG_FPD3_RX_ID0	= 0xf0,
};

enum ub960_register_bank {
	UB960_BANK_PGEN = 0,
	UB960_BANK_CSI0_TIMING = 0,
	UB960_BANK_CSI1_TIMING = 0,
	UB960_BANK_FPD3_0_RESERVED,
	UB960_BANK_FPD3_1_RESERVED,
	UB960_BANK_FPD3_2_RESERVED,
	UB960_BANK_FPD3_3_RESERVED,
	UB960_BANK_FPD3_SHARE_RESERVED,
	UB960_BANK_FPD3_ALL_RESERVED,
	UB960_BANK_CSI2_TX_RESERVED,
};

enum {
	UB960_PGEN_RESERVED	= 0x00,
	UB960_PGEN_CTL		= 0x01,
	UB960_PGEN_CFG		= 0x02,
	UB960_PGEN_CSI_DI	= 0x03,
	UB960_PGEN_LINE_SIZE1	= 0x04,
	UB960_PGEN_LINE_SIZE0	= 0x05,
	UB960_PGEN_BAR_SIZE1	= 0x06,
	UB960_PGEN_BAR_SIZE0	= 0x07,
	UB960_PGEN_ACT_LPF1	= 0x08,
	UB960_PGEN_ACT_LPF0	= 0x09,
	UB960_PGEN_TOT_LPF1	= 0x0a,
	UB960_PGEN_TOT_LPF0	= 0x0b,
	UB960_PGEN_TOT_PD1	= 0x0c,
	UB960_PGEN_TOT_PD0	= 0x0d,
	UB960_PGEN_VBP		= 0x0e,	/* vertical back porch */
	UB960_PGEN_VFP		= 0x0f,	/* vertical front porch */
	UB960_PGEN_COLOR0	= 0x10,
};

enum {
	UB960_INT_EN_OFFSET		= 7u,
	UB960_INT_EN_MASK		= 1u << UB960_INT_EN_OFFSET,
	UB960_LOCK_STS_CHG_OFFSET	= 0u,
	UB960_LOCK_STS_CHG_MASK		= 1u << UB960_LOCK_STS_CHG_OFFSET,
	UB960_LOCK_STS_OFFSET		= 0u,
	UB960_LOCK_STS_MASK		= 1u << UB960_LOCK_STS_OFFSET,
};


/**
 * These values are used for programming CSI_PLL_CTL register
 */
enum ub960_csi_tx_speed {
	UB960_CSI_TX_SPEED_MAX  = 0,
	UB960_CSI_TX_SPEED_1664 = 0,
	UB960_CSI_TX_SPEED_1600 = 0,
	UB960_CSI_TX_SPEED_1472 = 0,
	UB960_CSI_TX_SPEED_1200 = 1,
	UB960_CSI_TX_SPEED_800  = 2,
	UB960_CSI_TX_SPEED_400  = 3,

	UB960_CSI_TX_SPEED_ENDMARKER
};

/**
 * These values are used for programming CSI_CTL1 register
 */
enum ub960_csi_lane_count {
	UB960_CSI_LANE_COUNT_4 = 0,
	UB960_CSI_LANE_COUNT_3 = 1,
	UB960_CSI_LANE_COUNT_2 = 2,
	UB960_CSI_LANE_COUNT_1 = 3,

	CSI_LANE_COUNT_ENDMARKER
};


/**
 * Identifies fpdlink ports
 */
enum {
	UB960_PORT_0 = (1 << 0),
	UB960_PORT_1 = (1 << 1),
	UB960_PORT_2 = (1 << 2),
	UB960_PORT_3 = (1 << 3),
};


/**
 * This device has 8 bit register addresses and 8 bit values.
 */
static struct regmap_config ub960_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
};

/**
 * Section 7.4.19 of the datasheet states that timing must be set
 * manually when operating at 400 Mbps.
 *
 * # Set CSI-2 Timing parameters
 * WriteI2C(0xB0,0x2) # set auto-increment, page 0
 * WriteI2C(0xB1,0x40) # CSI-2 Port 0
 * WriteI2C(0xB2,0x83) # TCK Prep
 * WriteI2C(0xB2,0x8D) # TCK Zero
 * WriteI2C(0xB2,0x87) # TCK Trail
 * WriteI2C(0xB2,0x87) # TCK Post
 * WriteI2C(0xB2,0x83) # THS Prep
 * WriteI2C(0xB2,0x86) # THS Zero
 * WriteI2C(0xB2,0x84) # THS Trail
 * WriteI2C(0xB2,0x86) # THS Exit
 * WriteI2C(0xB2,0x84) # TLPX
 * # Set CSI-2 Timing parameters
 * WriteI2C(0xB0,0x2) # set auto-increment, page 0
 * WriteI2C(0xB1,0x60) # CSI-2 Port 1
 * WriteI2C(0xB2,0x83) # TCK Prep
 * WriteI2C(0xB2,0x8D) # TCK Zero
 * WriteI2C(0xB2,0x87) # TCK Trail
 * WriteI2C(0xB2,0x87) # TCK Post
 * WriteI2C(0xB2,0x83) # THS Prep
 * WriteI2C(0xB2,0x86) # THS Zero
 * WriteI2C(0xB2,0x84) # THS Trail
 * WriteI2C(0xB2,0x86) # THS Exit
 * WriteI2C(0xB2,0x84) # TLPX
 */
static const struct reg_sequence UB960_400_TIMING[] = {
/* # Set CSI-2 Timing parameters */
/* WriteI2C(0xB0,0x2) # set auto-increment, page 0 */
	{.reg = 0xB0, .def = 0x2},
/* WriteI2C(0xB1,0x40) # CSI-2 Port 0 */
	{.reg = 0xB1, .def = 0x40},
/* WriteI2C(0xB2,0x83) # TCK Prep */
	{.reg = 0xB2, .def = 0x83},
/* WriteI2C(0xB2,0x8D) # TCK Zero */
	{.reg = 0xB2, .def = 0x8d},
/* WriteI2C(0xB2,0x87) # TCK Trail */
	{.reg = 0xB2, .def = 0x87},
/* WriteI2C(0xB2,0x87) # TCK Post */
	{.reg = 0xB2, .def = 0x87},
/* WriteI2C(0xB2,0x83) # THS Prep */
	{.reg = 0xB2, .def = 0x83},
/* WriteI2C(0xB2,0x86) # THS Zero */
	{.reg = 0xB2, .def = 0x86},
/* WriteI2C(0xB2,0x84) # THS Trail */
	{.reg = 0xB2, .def = 0x84},
/* WriteI2C(0xB2,0x86) # THS Exit */
	{.reg = 0xB2, .def = 0x86},
/* WriteI2C(0xB2,0x84) # TLPX */
	{.reg = 0xB2, .def = 0x84},
 /* Set CSI-2 Timing parameters */
/* WriteI2C(0xB0,0x2) # set auto-increment, page 0 */
	{.reg = 0xB0, .def = 0x02},
/* WriteI2C(0xB1,0x60) # CSI-2 Port 1 */
	{.reg = 0xB1, .def = 0x60},
/* WriteI2C(0xB2,0x83) # TCK Prep */
	{.reg = 0xB2, .def = 0x83},
/* WriteI2C(0xB2,0x8D) # TCK Zero */
	{.reg = 0xB2, .def = 0x8d},
/* WriteI2C(0xB2,0x87) # TCK Trail */
	{.reg = 0xB2, .def = 0x87},
/* WriteI2C(0xB2,0x87) # TCK Post */
	{.reg = 0xB2, .def = 0x87},
/* WriteI2C(0xB2,0x83) # THS Prep */
	{.reg = 0xB2, .def = 0x83},
/* WriteI2C(0xB2,0x86) # THS Zero */
	{.reg = 0xB2, .def = 0x86},
/* WriteI2C(0xB2,0x84) # THS Trail */
	{.reg = 0xB2, .def = 0x84},
/* WriteI2C(0xB2,0x86) # THS Exit */
	{.reg = 0xB2, .def = 0x86},
/* WriteI2C(0xB2,0x84) # TLPX */
	{.reg = 0xB2, .def = 0x84},
};

/* This table is taken from an errata document from TI for silicon rev
 * 4. It is recommended defaults (their recomendations changed so the
 * power on defaults are incorrect in that rev). For now this is
 * commented out because it is not correcting any problems I'm
 * experiencing. */
/* static const struct reg_sequence UB960_DEFAULTS[] = */
/* { */
/* 	{.reg = 0xb0, .def = 0x14, .delay_us=0}, */
/* 	{.reg = 0xb1, .def = 0x03, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x04, .delay_us=0}, */
/* 	{.reg = 0xb1, .def = 0x04, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x04, .delay_us=0}, */

/* 	{.reg = 0xb0, .def = 0x1e, .delay_us=0}, */
/* 	{.reg = 0xb1, .def = 0x1e, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x15, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x15, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x15, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x15, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x15, .delay_us=0}, */
/* 	{.reg = 0xb0, .def = 0x1e, .delay_us=0}, */
/* 	{.reg = 0xb1, .def = 0x30, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x15, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x15, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x15, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x15, .delay_us=0}, */
/* 	{.reg = 0xb0, .def = 0x1c, .delay_us=0}, */
/* 	{.reg = 0xb1, .def = 0x2a, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x15, .delay_us=0}, */
/* 	{.reg = 0xb0, .def = 0x1e, .delay_us=0}, */
/* 	{.reg = 0xb1, .def = 0x25, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb0, .def = 0x1e, .delay_us=0}, */
/* 	{.reg = 0xb1, .def = 0x36, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb0, .def = 0x18, .delay_us=0}, */
/* 	{.reg = 0xb1, .def = 0x01, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0xec, .delay_us=0}, */
/* 	{.reg = 0xb0, .def = 0x1e, .delay_us=0}, */
/* 	{.reg = 0xb1, .def = 0x10, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb1, .def = 0x2b, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0xb2, .def = 0x16, .delay_us=0}, */
/* 	{.reg = 0x4c, .def = 0x0f, .delay_us=0}, */
/* 	{.reg = 0x41, .def = 0xa9, .delay_us=0}, */
/* 	{.reg = 0x05, .def = 0x00, .delay_us=0}, */
/* 	{.reg = 0x06, .def = 0x01, .delay_us=0}, */
/* }; */


/**
 * I do not have documentation for these settings. These set values in
 * the csi2 reserved register set.
 */
static const struct reg_sequence UB960_CSI2_RESERVED[] = {
	{.reg = 0xB0, .def = 0x1C, .delay_us = 100000},
	{.reg = 0xB1, .def = 0x13, .delay_us = 16000},
	{.reg = 0xB2, .def = 0x1f, .delay_us = 16000},
};

struct ub960; /* Forward reference for ub960_port */

#define UB960_NUM_ALIASES 8

/**
 * Class for ports
 */
struct ub960_port {
	int index;
	struct device_node *of_node;
	bool configured;
	bool enabled;
	bool started;
	bool no_mux;
	struct delayed_work lock_work;
	bool locked;
	struct ub960 *self;
	unsigned ser_id;
	struct i2c_client *ser_client;
	struct i2c_client *alias_clients[UB960_NUM_ALIASES];
	int num_aliases;
};

/**
 * State information about test pattern. Most of this data is
 * collected from the device tree.
 */
struct ub960_test_pattern_info {
	struct device_node *node;

	char compat[I2C_NAME_SIZE];
	u32 reg;

	u32 enabled;
	u32 line_period;
	u32 width_bytes;
	u32 height_lines;
	u32 vc_id;
	u32 data_type;
	u32 color0_value;
};

/**
 * Holds csi configuration data. The 960 has two CSI ports but they
 * must be configured identically.
 */
struct ub960_csi_info {
	enum ub960_csi_tx_speed speed;
	u8 n_lanes;
	bool continuous_clock;
};


/**
 * driver class
 */
struct ub960 {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *map;
	struct mutex indirect_access_lock;
	struct delayed_work status_work;
	struct i2c_mux_core *muxc;
	unsigned long lock_settle_time;
	unsigned long lock_timeout;
	unsigned long status_period;
	unsigned long status_count;
	unsigned int active_streams;

	struct regulator *avdd_reg;
	struct regulator *iovdd_reg;

	unsigned int pdb_gpio;
	unsigned int irq;

	/* fpd link3 ports */
	struct ub960_port ports[4];
	struct ub960_csi_info csi;

	struct ub960_test_pattern_info test_pattern;
	struct i2c_client *tp_client;
};


static void ub960_port_check_lock_sts(struct work_struct *work);
static int ub960_remove(struct i2c_client *client);


int ub960_s_stream(struct i2c_client *self_in,
		   struct i2c_client *src,
		   int enable)
{
	int err = 0;
	struct ub960 *self = i2c_get_clientdata(self_in);
	struct mutex *lock = &self->indirect_access_lock;


	/* increment active number of streams when starting a new stream */
	if (enable) {
		mutex_lock(lock);
		++(self->active_streams);
		if (self->active_streams > 1 && self->csi.speed == UB960_CSI_TX_SPEED_1600) {
			dev_dbg(self->dev, "%s re-issue deskew", src->dev.driver->name);
			// Initiate single deskew
			TRY_MUTEX(lock, err, regmap_update_bits(self->map, UB960_REG_CSI_CTL2, 1u << 1, 1u << 1));
		} else if (self->active_streams == 1) {
			TRY_MUTEX(lock, err, regmap_write(self->map, UB960_REG_RESET_CTL, 0x01));
			usleep_range(3 * 1000, 4 * 1000);
		}
		dev_dbg(self->dev, "%s started a new stream; "
				"%d active streams", src->dev.driver->name,
				self->active_streams);
		mutex_unlock(lock);
		return 0;
	}
	/* decrement number of streams on disable */
	else {
		mutex_lock(lock);
		--(self->active_streams);
		dev_dbg(self->dev, "%s stopped a stream; "
				"%d active streams", src->dev.driver->name,
				self->active_streams);
		/* only reset if there are no active streams running */
		if (self->active_streams == 0) {
			dev_dbg(self->dev, "%s notified end-of-stream,"
				" performing digital reset", src->dev.driver->name);
			TRY_MUTEX(lock, err, regmap_write(self->map, UB960_REG_RESET_CTL, 0x01));
			usleep_range(3 * 1000, 4 * 1000);
		}
		mutex_unlock(lock);
	}
	return err;
}
EXPORT_SYMBOL_GPL(ub960_s_stream);


/**
 * Helper for setting gpio value (lifted from example code)
 *
 * @param gpio gpio id
 * @param val on or off
 */
static void ub960_gpio_set(unsigned int gpio, int val)
{
	if (gpio_cansleep(gpio))
		gpio_set_value_cansleep(gpio, val);
	else
		gpio_set_value(gpio, val);
}


/**
 * Gets a device managed regulator
 *
 * @param self driver instance
 * @param name name of regulator
 * @param out where to store regulator
 *
 * @return 0 on success
 */
static int ub960_regulator_get(struct ub960 *self, const char *name,
			       struct regulator **out)
{
	*out = devm_regulator_get(self->dev, name);
	if (IS_ERR(*out)) {
		dev_warn(self->dev, "could not find regulator: %s", name);
		*out = NULL;
		return -ENOENT;
	}
	return 0;
}


/**
 * Enables a regulator
 *
 * @param self driver instance
 * @param regulator the regulator to enable
 * @param name name of the regulator (for debug)
 *
 * @return 0 on success
 */
static int ub960_regulator_enable(struct ub960 *self,
				  struct regulator *regulator, const char *name)
{
	int err = 0;
	if (!regulator)
		return 0;

	err = regulator_enable(regulator);
	if (err) {
		dev_err(self->dev, "could not enable regulator: %s", name);
		return err;
	}
	return 0;
}


/**
 * Acquires the regulators for the device(s)
 *
 * @param self driver instance
 *
 * @return 0 on success
 */
static int ub960_regulators_get(struct ub960 *self)
{
	/* It's OK if the regulators aren't present */
	ub960_regulator_get(self, "avdd", &self->avdd_reg);
	ub960_regulator_get(self, "iovdd", &self->iovdd_reg);
	ub960_regulator_enable(self, self->avdd_reg, "avdd");
	ub960_regulator_enable(self, self->iovdd_reg, "iovdd");
	return 0;
}


/**
 * Gets a gpio from the device tree by name.
 *
 * @param self driver instance
 * @param name gpio name (for debugging)
 * @param out gpio id
 *
 * @return 0 on success
 */
static int ub960_gpio_get(struct ub960 *self, const char *name,
			  unsigned int *out)
{
	int err = 0;

	err = of_get_named_gpio(self->client->dev.of_node, name, 0);
	if (!gpio_is_valid(err)) {
		dev_warn(self->dev, "gpio not present: %s: %d", name, err);
		return err;
	}
	*out = err;
	return 0;
}


/**
 * Gets devices managed gpio request
 *
 * @param self driver instance
 * @param id gpio id
 * @param name gpio name for debugging
 *
 * @return 0 on success
 */
static int ub960_gpio_request(struct ub960 *self, unsigned int id,
			      const char *name)
{
	int err = 0;

	err = devm_gpio_request(self->dev, id, name);
	if (err) {
		dev_err(self->dev, "gpio request failed %s: %d", name, err);
		return err;
	}
	return 0;
}


/**
 * Exports a gpio
 *
 * @param self driver instance
 * @param id gpio id
 * @param name gpio name for debug
 *
 * @return 0 on success
 */
static int ub960_gpio_export(struct ub960 *self, unsigned int id,
			     const char *name)
{
	int err = 0;

	err = gpio_export(id, false);
	if (err) {
		dev_err(self->dev, "%s gpio export failed: %d", name, err);
		return err;
	}
	return 0;
}


/**
 * Sets gpio direction to output
 *
 * @param self driver instance
 * @param id gpio id
 * @param name gpio name for debugging
 * @param value initial output value
 *
 * @return 0 on success
 */
static int ub960_gpio_direction_output(struct ub960 *self, unsigned int id,
				       const char *name, int value)
{
	int err = 0;

	err = gpio_direction_output(id, value);
	if (err) {
		dev_err(self->dev, "gpio %s direction error: %d", name, err);
		return err;
	}
	return 0;
}


/**
 * Gets all gpios for ub960
 *
 * @param self driver instance
 *
 * @return 0 on success
 */
static int ub960_gpios_get(struct ub960 *self)
{
	int err = 0;

	ub960_gpio_get(self, "pdb-gpios", &self->pdb_gpio);

	if (self->pdb_gpio) {
		TRY(err, ub960_gpio_request(self, self->pdb_gpio,
					    "cam_pdb_gpio"));
		TRY(err, ub960_gpio_export(self, self->pdb_gpio,
					   "cam_pdb_gpio"));
		TRY(err, ub960_gpio_direction_output(self, self->pdb_gpio,
						     "pdb_gpio", 0));
	}

	err = of_irq_get(self->dev->of_node, 0);
	if (err > 0)
		self->irq = err;
	else if (err < 0) {
		if (err == -EPROBE_DEFER) {
			dev_warn(self->dev, "deferred because of gpio error");
			return -EPROBE_DEFER;
		}
	}

	return 0;
}


static int ub960_reset(struct ub960 *self)
{
	int err = 0;
	int is_known;
	int i;
	int tries;
	static const char VALID_IDS[2][6] = {
		"_UB960",
		"_UB954"
	};

	char fpd3_rx_id[6];

	if (self->pdb_gpio) {
		// active low, set low to power down
		dev_dbg(self->dev, "using pdb_gpio for reset");
		ub960_gpio_set(self->pdb_gpio, false);
		usleep_range(3 * 1000, 4 * 1000);
		ub960_gpio_set(self->pdb_gpio, true);
		usleep_range(3 * 1000, 4 * 1000);
	} else {
		/* No reset gpio, try digital reset */
		dev_dbg(self->dev, "no pdb_gpio, trying digital reset");
		TRY(err, regmap_write(self->map, UB960_REG_RESET_CTL, 0x02));
		dev_dbg(self->dev, "err = %i after regmap_write(UB960_REG_RESET_CTL)", err);
		usleep_range(3 * 1000, 4 * 1000);
	}

	is_known = false;
	for (tries = 5; --tries > 0; ) {
		err = regmap_bulk_read(self->map, UB960_REG_FPD3_RX_ID0, fpd3_rx_id,
					  ARRAY_SIZE(fpd3_rx_id));
		dev_dbg(self->dev, "err = %i after regmap_bulk_read(UB960_REG_FPD3_RX_ID0)", err);

		if (err == 0)
			break;

		usleep_range(50 * 1000, 50 * 1000);
	}

	if (err < 0)
		return err;

	for(i=0; i<ARRAY_SIZE(VALID_IDS); ++i) {
		if (memcmp(fpd3_rx_id, VALID_IDS[i], sizeof(fpd3_rx_id)) == 0) {
			is_known = true;
			break;
		}
	}

	if(!is_known) {
		dev_warn(self->dev, "Unexpected FPD3_RX_ID: %.*s\n",
			 (int)ARRAY_SIZE(fpd3_rx_id), fpd3_rx_id);
	}
	return 0;
}

/**
 * Checks to see if the device is supported
 *
 * @param self driver instance
 * @param out_is_supported result of test
 *
 * @return 0 on sucess
 */
static int ub960_is_device_supported(struct ub960 *self, bool *out_is_supported)
{
	int err = 0;
	unsigned int id = 0;

	*out_is_supported = true;
	TRY(err, regmap_read(self->map, UB960_REG_REV_MASK_ID, &id));
	id = (id >> 4) & 0xF;
	dev_info(self->dev, "revision mask id: %#.2x", id);

	switch (id) {
	/* PDS90UB960-Q1 A0 */
	case 0x2:
		dev_info(self->dev, "detected: PDS90UB960-Q1 A0");
		break;
	/* PDS90UB960-Q1 A1 */
	case 0x3:
		dev_info(self->dev, "detected: PDS90UB960-Q1 A1");
		break;
	/* DS90UB960-Q1 A0 */
	case 0x4:
		dev_info(self->dev, "detected: DS90UB960-Q1 A0");
		break;
	case 0:
	case 1:
	default:
		*out_is_supported = false;
		break;
	}
	return 0;
}

/**
 * Selects the fpd link port to program
 *
 * @param self driver instance
 * @param read_port which port to read from (0-3)
 * @param write_ports_mask which ports to write to
 *
 * @return 0 on success
 */
static int ub960_fdp3_port_select(struct ub960 *self, u8 read_port,
				  u8 write_ports_mask)
{
	int err = 0;
	/* bits 5 and 4 RX_READ_PORT 0-3 (direct port mapping)*/
	/* bit 0, port1, bit1, port 2, */
	TRY(err, regmap_write(self->map, UB960_REG_FPD3_PORT_SEL,
			      (read_port << 4) | write_ports_mask));
	return 0;
}

/**
 * Starts the ub960
 *
 * @param self driver instance
 *
 * @return 0 on success
 */
static int ub960_csi_configure(struct ub960 *self)
{
	int err = 0;

	TRY(err, ub960_reset(self));

	/* TRY(err, regmap_multi_reg_write(self->map, */
	/* 			       UB960_DEFAULTS, */
	/* 			       ARRAY_SIZE(UB960_DEFAULTS))); */

	if (self->csi.speed == UB960_CSI_TX_SPEED_400) {
		// REF_CLK_MODE: Must be set to 0=200MHz if CSI_TX_SPEED is 400Mbps
		TRY(err, regmap_write(self->map, UB960_REG_CSI_PLL_CTL, (0u << 2)|(self->csi.speed & 0x3)));

		dev_warn(self->dev,
			 "writing 400 mbps timing settings (untested!)");
		TRY(err, regmap_multi_reg_write(self->map,
						UB960_400_TIMING,
						ARRAY_SIZE(UB960_400_TIMING)));
	} else {
		TRY(err, regmap_write(self->map, UB960_REG_CSI_PLL_CTL, (1u << 2)|(self->csi.speed & 0x3)));
	}

	TRY(err, regmap_multi_reg_write(self->map, UB960_CSI2_RESERVED,
					ARRAY_SIZE(UB960_CSI2_RESERVED)));

	/* @todo move settings to device tree */

	/* IO_CTL: bit 7: select 3.3v io supply
	 * bit 6: 0 ... use detected voltage (and ignore bit 7!)
	 * bit 5-4: IO supply mode (0b01 isn't valid!)*/
	/* {0x0D, 0x90}, /\*I/O to 3V3 - Options not valid with datashee*\/ */
	TRY(err, regmap_write(self->map, UB960_REG_IO_CTL, 0x90));

	/* RX_PORT_CTL disable ports */
	/* {0x0C, 0x0F}, /\*Disable all ports*\/ */
	TRY(err, regmap_write(self->map, UB960_REG_RX_PORT_CTL, 0x00));

	/* {0x32, 0x01}, /\*Enable TX port 0*\/ */
	TRY(err, regmap_write(self->map, UB960_REG_CSI_PORT_SEL, 0x1));

	/* map all rx-ports to tx-port 0 */
	TRY(err, regmap_update_bits(self->map, UB960_REG_FWD_CTL1,
		0xF,
		0x0));

	/* enable interrupt pin */
	if (self->irq)
		TRY(err, regmap_update_bits(self->map, UB960_REG_INTERRUPT_CTL,
					    UB960_INT_EN_MASK, UB960_INT_EN_MASK));

	return 0;
}


/**
 * Reads a u32 from a device tree node and prints a warning on error.
 *
 * @param self driver instance, for warnings
 * @param node node to read from
 * @param name property name
 * @param out where value is stored
 *
 * @return 0 on success
 */
static int ub960_of_read_u32(struct ub960 *self, struct device_node *node,
			     const char *name, u32 *out)
{
	int err = 0;

	err = of_property_read_u32(node, name, out);
	if (err)
		dev_warn(self->dev, "%s: missing property: %s", node->name, name);
	return err;
}

/**
 * Configures FrameSync signal to either be generated internally or taken from
 * a GPIO pin based on device tree options.
 *
 * @param self driver instance
 *
 * @return 0 on success
 */
static int ub960_fsync_configure(struct ub960 *self, struct device_node *node)
{
	int err = 0;
	u32 mode = 0;
	u32 high_time = 0;
	u32 low_time = 0;
	u32 reg_val;

	err = of_property_read_u32(node, "frame-sync-mode", &mode);
	if (err)
	{
		mode = 0;
		dev_warn(self->dev, "using default frame-sync-mode: %d", mode);
	}

	err = of_property_read_u32(node, "frame-sync-high-time-us", &high_time);
	if (err)
	{
		high_time = FRAME_SYNC_DEFAULT_HIGH_TIME_US;
		dev_warn(self->dev, "using default frame-sync-high-time-us: %d", high_time);
	}

	err = of_property_read_u32(node, "frame-sync-low-time-us", &low_time);
	if (err)
	{
		low_time = FRAME_SYNC_DEFAULT_LOW_TIME_US;
		dev_warn(self->dev, "using default frame-sync-low-time-us: %d", low_time);
	}

	/* Convert high/low times from microseconds to number of clock periods
	 * NOTE: These calculations assume a 50 Mbps backchannel frequency
	 *       (BC_FREQ_SELECT=6). This makes FS_CLK_PD = 600 ns
	 */
	high_time = (high_time * 1000) / 600;
	low_time = (low_time * 1000) / 600;
	if (high_time > 0xffff) {
		dev_warn(self->dev, "Frame sync high time too long. Reducing to 0xffff.\n");
		high_time = 0xffff;
	}
	if (low_time > 0xffff) {
		dev_warn(self->dev, "Frame sync low time too long. Reducing to 0xffff.\n");
		low_time = 0xffff;
	}

	/* Configure high time and low time registers */
	TRY(err, regmap_write(self->map, UB960_REG_FS_HIGH_TIME_1,
		(high_time >> 8) & 0xff));
	TRY(err, regmap_write(self->map, UB960_REG_FS_HIGH_TIME_0,
		(high_time & 0xff)));
	TRY(err, regmap_write(self->map, UB960_REG_FS_LOW_TIME_1,
		(low_time >> 8) & 0xff));
	TRY(err, regmap_write(self->map, UB960_REG_FS_LOW_TIME_0,
		(low_time & 0xff)));

	/* Configure FS_CTL register */
	reg_val = (mode << 4);
	if (mode == 0) {
		reg_val |= 1;  // FS_GEN_ENABLE
	}
	TRY(err, regmap_write(self->map, UB960_REG_FS_CTL, reg_val));

	return 0;
}

static int ub960_port_pass_through_all(struct ub960 *self, struct ub960_port *port, bool enable)
{
	int err = 0;
	struct mutex *lock = &self->indirect_access_lock;

	if (unlikely(!port->configured))
		return -EINVAL;

	mutex_lock(lock);
	TRY_MUTEX(lock, err, ub960_fdp3_port_select(self, port->index, (1 << port->index)));
	TRY_MUTEX(lock, err, regmap_update_bits(self->map, UB960_REG_BCC_CONFIG, 1u<<7, (enable ? 1u : 0u)<<7));
	mutex_unlock(lock);

	return 0;
}

static int ub960_select_i2c_chan(struct i2c_mux_core *muxc, u32 chan_id)
{
	int err = 0;
	struct ub960 *self = i2c_mux_priv(muxc);

	if (unlikely(chan_id > ARRAY_SIZE(self->ports)))
		return -EINVAL;

	TRY(err, ub960_port_pass_through_all(self, &self->ports[chan_id], true));

	return 0;
}

static int ub960_deselect_i2c_chan(struct i2c_mux_core *muxc, u32 chan_id)
{
	int err = 0;
	struct ub960 *self = i2c_mux_priv(muxc);

	if (unlikely(chan_id > ARRAY_SIZE(self->ports)))
		return -EINVAL;

	TRY(err, ub960_port_pass_through_all(self, &self->ports[chan_id], false));

	return 0;
}

static int ub960_set_port_enabled(struct ub960 *self, struct ub960_port *port,
				  bool enable)
{
	int err;
	const unsigned int port_en = 1 << port->index;

	port->enabled = enable;
	TRY(err, regmap_update_bits(self->map, UB960_REG_RX_PORT_CTL, port_en,
				    (enable ? port_en : 0)));

	return 0;
}

/**
 * Loads a single port based on id.
 *
 * @param self driver instance
 * @param node device tree node
 * @param id port/channel id
 *
 * @return 0
 */
static int ub960_port_load(struct ub960 *self, struct device_node *node,
			   struct ub960_port *port)
{
	dev_dbg(self->dev, "Configure port %d", port->index);

	port->self = self;
	port->of_node = node;
	port->locked = false;
	port->configured = true;

	if (of_property_read_bool(node, "no-mux"))
		port->no_mux = true;

	INIT_DELAYED_WORK(&port->lock_work, ub960_port_check_lock_sts);

	return 0;
}


/**
 * Loads all ports from the device tree
 *
 * @param self driver instance
 *
 * @return 0 on success
 */
static int ub960_ports_load(struct ub960 *self)
{
	/* int err = 0; */
	struct device_node *child = NULL;
	u32 reg = 0;
	int err = 0;

	for_each_available_child_of_node(self->dev->of_node, child) {
		err = of_property_read_u32(child, "reg", &reg);
		if (err)
			continue;

		if (reg > ARRAY_SIZE(self->ports) - 1) {
			dev_warn(self->dev,
				 "ignoring port reg=%d, ub960 has %lu ports", reg, ARRAY_SIZE(self->ports));
			continue;
		}
		self->ports[reg].index = reg;

		TRY(err, ub960_port_load(self, child, &self->ports[reg]));
	}
	return 0;
}

static int ub960_port_configure(struct ub960 *self, struct ub960_port *port)
{
	int err = 0;
	struct mutex *lock = &self->indirect_access_lock;
	unsigned chan_id = port->index;
	unsigned val;
	struct reg_sequence seq[] = {
		{UB960_REG_AEQ_CTL,         0x71},
		{UB960_REG_SFILTER_CFG,     0xA9},
		{UB960_REG_LINK_ERROR_CNT,  0x33},
	};

	mutex_lock(lock);

	TRY_MUTEX(lock, err, ub960_fdp3_port_select(self, chan_id, 1u << chan_id));

	// Enable I2C_PASSTHROUGH
	TRY_MUTEX(lock, err, regmap_update_bits(self->map, UB960_REG_BCC_CONFIG, 1u << 6, 1u << 6));

	// Route FrameSync to BC_GPIO0
	TRY_MUTEX(lock, err, regmap_update_bits(self->map, UB960_REG_BC_GPIO_CTL0, 0xF << 0, 0xa << 0));

	TRY(err, regmap_multi_reg_write(self->map, seq, ARRAY_SIZE(seq)));

	// Enable TX port 0
	TRY_MUTEX(lock, err, regmap_write(self->map, UB960_REG_CSI_PORT_SEL, 0x1));

	//NOTE:Fowarding & CSI are disabled here prior to lock detection.
	//     See ub960_port_check_lock_sts() for when it will be enabled.

	// FWD_PORTn_DIS=1
	TRY_MUTEX(lock, err, regmap_update_bits(self->map,
		UB960_REG_FWD_CTL1,
		1u << (chan_id+4),
		1u << (chan_id+4)));

	// CSI_CTL1
	val = 0;
	switch (self->csi.n_lanes) {
		case 1:
			val |= (UB960_CSI_LANE_COUNT_1 << 4);
			break;
		case 2:
			val |= (UB960_CSI_LANE_COUNT_2 << 4);
			break;
		case 3:
			val |= (UB960_CSI_LANE_COUNT_3 << 4);
			break;
		case 4:
			val |= (UB960_CSI_LANE_COUNT_4 << 4);
			break;
		default:
			dev_err(self->dev, "n_lanes invalid value (%u)", self->csi.n_lanes);
			return -EINVAL;
	}

	// Enabled calibration for high speed
	if (self->csi.speed == UB960_CSI_TX_SPEED_1600)
		val |= (1u << 6);

	if (self->csi.continuous_clock)
		val |= (1u << 1);

	TRY_MUTEX(lock, err, regmap_write(self->map, UB960_REG_CSI_CTL1, val));

	// Enable periodic deskew (only for 1600Mbps)
	TRY_MUTEX(lock, err, regmap_update_bits(self->map, UB960_REG_CSI_CTL2,
		1u << 0,
		(self->csi.speed == UB960_CSI_TX_SPEED_1600 ? 1u : 0u) << 0));

	// VC Map - All to chan_id
	TRY_MUTEX(lock, err, regmap_update_bits(self->map,
		UB960_REG_CSI_VC_MAP,
		3u << 6|3u << 4|3u << 2|3u << 0,
		(chan_id << 6) | (chan_id << 4) | (chan_id << 2) | (chan_id << 0)));

	// DISCARD_ON_PAR_ERR=0|LV_POLARITY=HIGH|FV_POLARITY=HIGH
	TRY_MUTEX(lock, err, regmap_update_bits(self->map,
		UB960_REG_PORT_CONFIG2,
		1u << 5|1u << 1|1u << 0,
		0|0|0));

	// IE_RXn=1
	TRY_MUTEX(lock, err, regmap_update_bits(self->map,
		UB960_REG_INTERRUPT_CTL,
		1u << chan_id,
		1u << chan_id));

	TRY_MUTEX(lock, err, regmap_update_bits(self->map,
		UB960_REG_PORT_ICR_LO,
		UB960_LOCK_STS_MASK,
		UB960_LOCK_STS_MASK));

	mutex_unlock(lock);

	return 0;
}

int ub960_port_create_child(struct ub960 *self, struct ub960_port *port, struct device_node *child, unsigned addr, unsigned physical_addr)
{
	int err;
	struct mutex *lock = &self->indirect_access_lock;
	struct i2c_adapter *adapter = self->client->adapter;
	struct i2c_board_info i2c_info = {0};
	unsigned chan_id = port->index;
	const char *compatible;

	TRY(err, of_property_read_string(child, "compatible", &compatible));
	strncpy(i2c_info.type, compatible, sizeof(i2c_info.type));
	i2c_info.addr = addr;
	i2c_info.of_node = child;

	mutex_lock(lock);

	TRY_MUTEX(lock, err, ub960_fdp3_port_select(self, chan_id, 1u << chan_id));

	dev_dbg(self->dev, "[%d] Create client for %s", chan_id, of_node_full_name(child));

	if (physical_addr == port->ser_id) {
		dev_dbg(self->dev, "[%d] Alias serializer from %02x to %02x", chan_id, physical_addr, addr);

		TRY_MUTEX(lock, err, regmap_write(self->map, UB960_REG_SER_ALIAS_ID, addr<<1));

		port->ser_client = i2c_new_client_device(adapter, &i2c_info);
		if (IS_ERR(port->ser_client)) {
			err = PTR_ERR(port->ser_client);
			goto fail;
		} else if (!port->ser_client) {
			err = -ENOMEM;
			goto fail;
		}
	} else {
		unsigned index = port->num_aliases;
		unsigned slave_id_reg = UB960_REG_SLAVE_ID0 + index;
		unsigned slave_alias_id_reg = UB960_REG_SLAVE_ALIAS0 + index;

		if (index == ARRAY_SIZE(port->alias_clients)) {
			dev_err(self->dev, "[%d] Exceeded num_aliases (%d)", chan_id, index);
			err = -ENOMEM;
			goto fail;
		}

		dev_dbg(self->dev, "[%d] Alias #%d from %02x to %02x", chan_id, index, physical_addr, addr);

		TRY_MUTEX(lock, err, regmap_write(self->map, slave_id_reg, physical_addr<<1));

		TRY_MUTEX(lock, err, regmap_write(self->map, slave_alias_id_reg, addr<<1));

		port->alias_clients[index] = i2c_new_client_device(adapter, &i2c_info);
		if (IS_ERR(port->alias_clients[index])) {
			err = PTR_ERR(port->alias_clients[index]);
			goto fail;
		} else if (!port->alias_clients[index]) {
			err = -ENOMEM;
			goto fail;
		}
		port->num_aliases++;
	}

fail:
	mutex_unlock(lock);

	return err;

}

int ub960_port_scan_children(struct ub960 *self, struct ub960_port *port)
{
	int err;
	unsigned chan_id = port->index;
	struct device_node *child;

	TRY(err, regmap_read(self->map, UB960_REG_SER_ID, &port->ser_id));
	port->ser_id >>= 1;

	port->num_aliases = 0;

	for_each_available_child_of_node(port->of_node, child) {
		u32 reg, physical_addr;

		dev_info(self->dev, "[%d] Found child node: %s", chan_id, of_node_full_name(child));

		err = of_property_read_u32(child, "reg", &reg);
		if (err)
			continue;

		err = of_property_read_u32(child, "physical-addr", &physical_addr);
		if (err)
			physical_addr = reg;

		err = ub960_port_create_child(self, port, child, reg, physical_addr);
		if (err) {
			dev_err(self->dev, "[%d] Failed to create i2c device for %s: %d", chan_id, of_node_full_name(child), err);
			continue;
		}
	}

	return 0;
}

/**
 * Writes the channel/port settings to the ub960
 *
 * @param self driver instance
 *
 * @return 0 on success
 */
static int ub960_ports_configure(struct ub960 *self)
{
	int err = 0;
	int i = 0;

	self->muxc = i2c_mux_alloc(
		self->client->adapter,
		self->dev,
		ARRAY_SIZE(self->ports),
		0,
		I2C_MUX_LOCKED,
		ub960_select_i2c_chan,
		ub960_deselect_i2c_chan);
	if (IS_ERR(self->muxc)) {
		dev_err(self->dev, "Failed to allocate i2c-mux-core");
		return PTR_ERR(self->muxc);
	} else if (!self->muxc) {
		return -ENOMEM;
	}
	self->muxc->priv = self;

	for (i = 0; i < ARRAY_SIZE(self->ports); ++i) {
		struct ub960_port *port = &self->ports[i];

		if (!port->configured)
			continue;

		TRY(err, ub960_port_configure(self, port));

		TRY(err, ub960_set_port_enabled(self, port, true));
	}

	return 0;
}

static int ub960_port_start(struct ub960 *self, struct ub960_port *port)
{
	int err;
	struct mutex *lock = &self->indirect_access_lock;
	unsigned chan_id = port->index;

	if (port->started)
		return 0;

	dev_dbg(self->dev, "[%d] start port", chan_id);

	mutex_lock(lock);

	TRY_MUTEX(lock, err, ub960_fdp3_port_select(self, chan_id, 1u << chan_id));

	TRY_MUTEX(lock, err, regmap_update_bits(self->map,
		UB960_REG_FWD_CTL1,
		1u << (chan_id+4),
		0u << (chan_id+4)));

	TRY_MUTEX(lock, err, regmap_write(self->map, UB960_REG_CSI_PORT_SEL, 1u << 0));

	TRY_MUTEX(lock, err, regmap_update_bits(self->map,
		UB960_REG_CSI_CTL1,
		1u << 0,
		1u << 0));

	mutex_unlock(lock);

	if (port->no_mux) {
		dev_dbg(self->dev, "[%d] scan for child i2c nodes", chan_id);
		TRY(err, ub960_port_scan_children(self, port));
	} else {
		dev_dbg(self->dev, "[%d] start muxed i2c adapter", chan_id);
		TRY(err, i2c_mux_add_adapter(self->muxc, 0, chan_id, 0));
	}

	port->started = true;

	return 0;
}


/**
 * Get the csi transmit speed from the device tree.
 *
 * @param self driver instance
 * @param node of node to read from
 *
 * @return csi tx speed
 */
static enum ub960_csi_tx_speed ub960_load_csi_tx_speed(struct ub960 *self,
		struct device_node *node)
{
	/* default to 1600 */
	const char *name = "csi-tx-speed-mbps";
	u32 val = 1600;
	enum ub960_csi_tx_speed speed = val;

	if (of_property_read_u32(node, name, &val) != 0) {
		dev_warn(self->dev,
			 "property %s not present,"
			 " defaulting to %u Mbps",
			 name, val);
		return val;
	}
	switch (val) {
	case 1664:
	case 1600:
	case 1472:
		speed = UB960_CSI_TX_SPEED_1600;
		break;
	case 1200:
		speed = UB960_CSI_TX_SPEED_1200;
		break;
	case 800:
		speed = UB960_CSI_TX_SPEED_800;
		break;
	case 400:
		/* these are valid */
		speed = UB960_CSI_TX_SPEED_400;
		break;
	default:
		dev_warn(self->dev, "%s invalid value (%u)", name, val);
	}
	return speed;
}


/**
 * Maps csi tx speed enum to human readable string
 *
 * @param speed
 *
 * @return human readable string
 */
static const char *ub960_csi_tx_speed_to_string(enum ub960_csi_tx_speed speed)
{
	const char *MAP[] = {
		"1472-1664",
		"1200",
		"800",
		"400"
	};

	if (speed >= UB960_CSI_TX_SPEED_ENDMARKER)
		return "invalid";

	return MAP[speed];
}


/**
 * Loads CSI parameters from device tree
 *
 * @param self driver instance
 * @param node where to read from
 *
 * @return 0 on success
 */
static int ub960_load_csi(struct ub960 *self, struct device_node *node)
{

	int err = 0;
	u32 val = 0;
	const char *key;

	self->csi.speed = ub960_load_csi_tx_speed(self, node);


	key = "csi-lane-count";
	TRY(err, ub960_of_read_u32(self, node, key, &val));
	if ((val < 1)
	    || (val > 4)) {
		dev_err(self->dev, "invalid %s:%u (1-4 valid)", key, val);
		return -EINVAL;
	}
	self->csi.n_lanes = val;

	key = "csi-continuous-clock";
	TRY(err, ub960_of_read_u32(self, node, key, &val));
	if (val > 1) {
		dev_err(self->dev, "invalid %s:%u (0 or 1 valid)", key, val);
		return -EINVAL;
	}
	self->csi.continuous_clock = val;

	dev_info(self->dev, "csi speed:%s lanes:%u continuous-clock:%s",
		 ub960_csi_tx_speed_to_string(self->csi.speed),
		 self->csi.n_lanes,
		 self->csi.continuous_clock ? "yes" : "no");
	return 0;
}

/**
 * Writes to an indirect register
 *
 * @param self driver instance
 * @param bank register bank
 * @param addr indirect address
 * @param val value to write
 *
 * @return 0 on success
 */
static int ub960_reg_indirect_write(struct ub960 *self,
				    enum ub960_register_bank bank,
				    u8 addr, u8 val)
{
	int err = 0;
	struct reg_sequence to_write[] = {
		{.reg = UB960_REG_IND_ACC_CTL, .def = bank, .delay_us = 0},
		{.reg = UB960_REG_IND_ACC_ADDR, .def = addr, .delay_us = 0},
		{.reg = UB960_REG_IND_ACC_DATA, .def = val, .delay_us = 0},
	};
	TRY(err, regmap_multi_reg_write(self->map, to_write, ARRAY_SIZE(to_write)));
	return 0;
}

/**
 * This configures and turns on the test pattern
 *
 * @param self driver instance
 *
 * @return 0 on success
 */
static int ub960_test_pattern_enable(struct ub960 *self)
{
	int err = 0;
	u8 val = 0;
	/* Prior to enabling test pattern */
	/* disable video forwarding (FWD_CTL1 set register 0x20 to 0x30 */
	TRY(err, regmap_write(self->map, UB960_REG_FWD_CTL1, 0xF0));
	/* configure CSI-2 speed (already done)*/
	/* enabled csi-2 transmitter for port0 using csi_ctl */


	/* fixed color, block size "controls the size of the fixed
	 * color field in bytes"*/
	val = (1 << 7) | 1;
	TRY(err, ub960_reg_indirect_write(self, UB960_BANK_PGEN,
					  UB960_PGEN_CFG, val));

	/* vc id and data type */
	val = (self->test_pattern.vc_id << 6) | self->test_pattern.data_type;
	TRY(err, ub960_reg_indirect_write(self, UB960_BANK_PGEN,
					  UB960_PGEN_CSI_DI, val));
	/* line width in bytes */
	val = (self->test_pattern.width_bytes & 0x0000ff00) >> 8;
	TRY(err, ub960_reg_indirect_write(self, UB960_BANK_PGEN,
					  UB960_PGEN_LINE_SIZE1, val));
	val = (self->test_pattern.width_bytes & 0x000000ff);
	TRY(err, ub960_reg_indirect_write(self, UB960_BANK_PGEN,
					  UB960_PGEN_LINE_SIZE0, val));


	/* active lines per frame */
	val = (self->test_pattern.height_lines & 0x0000ff00) >> 8;
	TRY(err, ub960_reg_indirect_write(self, UB960_BANK_PGEN,
					  UB960_PGEN_ACT_LPF1, val));
	val = (self->test_pattern.height_lines & 0x000000ff);
	TRY(err, ub960_reg_indirect_write(self, UB960_BANK_PGEN,
					  UB960_PGEN_ACT_LPF0, val));

	/* total lines per frame including blanking */
	/* line period (time period depends on CSI speed)*/
	/* vertical back porch */
	/* vertical front porch */

	/* color0 - For the time being we're just going to use a
	 * single byte for the color pattern. This is easy enough to
	 * verify with a hex editor. */
	val = self->test_pattern.color0_value & 0xff;
	TRY(err, ub960_reg_indirect_write(self, UB960_BANK_PGEN,
					  UB960_PGEN_COLOR0, val));

	/* total line period (effectively controls frame rate) */
	val = (self->test_pattern.line_period & 0xff00) >> 8;
	TRY(err, ub960_reg_indirect_write(self, UB960_BANK_PGEN,
					  UB960_PGEN_TOT_PD1, val));

	val = self->test_pattern.line_period & 0xff;
	TRY(err, ub960_reg_indirect_write(self, UB960_BANK_PGEN,
					  UB960_PGEN_TOT_PD0, val));

	/* Enable the test pattern */
	TRY(err, ub960_reg_indirect_write(self, UB960_BANK_PGEN,
					  UB960_PGEN_CTL, 1));

	return 0;
}

/**
 * Loads parameters from device tree
 *
 * @param self driver instance
 * @param node device tree node
 *
 * @return 0 on success
 */
static int ub960_configuration_load(struct ub960 *self,
				    struct device_node *node)
{
	/* struct device_node *tp_node = NULL; */
	int err = 0;

	/* Load test pattern values if it is present */
	/* tp_node = of_find_node_by_name(node, "test-pattern"); */
	/* if (tp_node != NULL) { */
	/* 	if (of_device_is_available(tp_node)) { */
	/* 		dev_dbg(self->dev, "loading test-pattern parameters"); */
	/* 		err = ub960_test_pattern_load(self, tp_node); */
	/* 		if (err) { */
	/* 			of_node_put(tp_node); */
	/* 			return err; */
	/* 		} */
	/* 	} */
	/* 	of_node_put(tp_node); */
	/* } */

	/* load parameters for csi ports */
	dev_dbg(self->dev, "loading csi parameters");
	TRY(err, ub960_load_csi(self, node));

	/* Read port data from device tree. Do this first, avoid
	 * touching the hardware in case there's a device tree
	 * misconfiguration. */
	dev_dbg(self->dev, "loading fpdlink port parameters");
	TRY(err, ub960_ports_load(self));

	return 0;
}

/**
 * This creates an instance of the test pattern driver. I'm not very
 * fond of how this is done but it is functional...so there's that...
 *
 * @param self driver instance
 *
 * @return 0 on success
 */
static int ub960_test_pattern_driver_start(struct ub960 *self)
{
	struct i2c_adapter *adap = to_i2c_adapter(self->client->dev.parent);
	struct i2c_board_info i2c_info = {0, };

	strncpy(i2c_info.type, self->test_pattern.compat, sizeof(i2c_info.type));
	i2c_info.addr = self->test_pattern.reg;;
	i2c_info.of_node = self->test_pattern.node;

	self->tp_client = i2c_new_client_device(adap, &i2c_info);
	if (!self->tp_client) {
		dev_err(self->dev,
			"could not create i2c client for test pattern adapter=%s type=%s addr=%#.2x",
			adap->name, i2c_info.type, i2c_info.addr);
		return -ENOMEM;
	}

	dev_info(self->dev, "created test pattern driver");

	return 0;
}


struct bitfield {
	const char *name;
	uint offset;
	uint mask;
	uint value;
};

#ifdef VERBOSE_STATUS

#define BITFIELD(NAME, OFFSET, MASK, VALUE) \
	{ #NAME, OFFSET, MASK, VALUE }
#define BITFIELD_SINGLE(NAME, OFFSET) \
	BITFIELD(NAME, OFFSET, 1u, 1u)

static inline uint get_bitfield_value(struct bitfield *b, uint value)
{
	return ((value >> b->offset) & b->mask);
}
static inline bool is_bitfield_set(struct bitfield *b, uint value)
{
	return (get_bitfield_value(b, value) == b->mask);
}
static inline void strncat_bitfields(char *buf, int buf_len, struct bitfield *b, uint value)
{
	bool first = true;
	if (!b)
		return;
	buf[0] = '\0';
	while (b->name != NULL) {
		if (is_bitfield_set(b, value)) {
			if (!first)
				strncat(buf, "|", buf_len);
			else
				first = false;
			strncat(buf, b->name, buf_len);
		}
		b++;
	}
}

struct bitfield RX_PORT_STS1_FIELDS[] = {
	BITFIELD(RX_PORT_NUM_0, 6, 3, 0),
	BITFIELD(RX_PORT_NUM_1, 6, 3, 1),
	BITFIELD(RX_PORT_NUM_2, 6, 3, 2),
	BITFIELD(RX_PORT_NUM_3, 6, 3, 3),
	BITFIELD_SINGLE(BCC_CRC_ERROR, 5),
	BITFIELD_SINGLE(LOCK_STS_CHG, 4),
	BITFIELD_SINGLE(BCC_SEQ_ERROR, 3),
	BITFIELD_SINGLE(PARITY_ERROR, 2),
	BITFIELD_SINGLE(PORT_PASS, 1),
	BITFIELD_SINGLE(LOCK_STS, 0),
	{0, }
};

struct bitfield RX_PORT_STS2_FIELDS[] = {
	BITFIELD_SINGLE(LINE_LEN_UNSTABLE, 7),
	BITFIELD_SINGLE(LINE_LEN_CHG, 6),
	BITFIELD_SINGLE(FPD3_ENCODE_ERROR, 5),
	BITFIELD_SINGLE(BUFFER_ERROR, 4),
	BITFIELD_SINGLE(CSI_ERROR, 3),
	BITFIELD_SINGLE(FREQ_STABLE, 2),
	BITFIELD_SINGLE(NO_FPD3_CLK, 1),
	BITFIELD_SINGLE(LINE_CNT_CHG, 0),
	{0, }
};

struct bitfield CSI_RX_STS_FIELDS[] = {
	BITFIELD_SINGLE(LENGTH_ERR, 3),
	BITFIELD_SINGLE(CKSUM_ERR, 2),
	BITFIELD_SINGLE(ECC2_ERR, 1),
	BITFIELD_SINGLE(ECC1_ERR, 0),
	{0, }
};

#endif // VERBOSE_STATUS

static inline int ub960_get_port_status(struct ub960_port *port,
					  unsigned int *rx_port_sts1,
					  unsigned int *rx_port_sts2,
					  unsigned int *csi_rx_sts)
{
	int err;
#ifdef VERBOSE_STATUS
	char buf[128] = {0, };
#endif
	struct ub960 *self = port->self;
	struct mutex *lock = &self->indirect_access_lock;

	mutex_lock(lock);
	TRY_MUTEX(lock, err, ub960_fdp3_port_select(self, port->index, (1 << port->index)));
	TRY_MUTEX(lock, err, regmap_read(self->map, UB960_REG_RX_PORT_STS1, rx_port_sts1));
	TRY_MUTEX(lock, err, regmap_read(self->map, UB960_REG_RX_PORT_STS2, rx_port_sts2));
	TRY_MUTEX(lock, err, regmap_read(self->map, UB960_REG_CSI_RX_STS, csi_rx_sts));
	mutex_unlock(lock);

#ifdef VERBOSE_STATUS
	strncat_bitfields(buf, sizeof(buf), RX_PORT_STS1_FIELDS, rx_port_sts1);
	dev_dbg_ratelimited(self->dev, "[%d] RX_PORT_STS1: %02x %s\n", port->index,
			    *rx_port_sts1 & 0xff, buf);

	strncat_bitfields(buf, sizeof(buf), RX_PORT_STS2_FIELDS, rx_port_sts2);
	dev_dbg_ratelimited(self->dev, "[%d] RX_PORT_STS2: %02x %s\n", port->index,
			    *rx_port_sts2 & 0xff, buf);

	strncat_bitfields(buf, sizeof(buf), CSI_RX_STS_FIELDS, csi_rx_sts);
	dev_dbg_ratelimited(self->dev, "[%d] CSI_RX_STS: %02x %s\n", port->index,
			    *csi_rx_sts & 0xff, buf);
#else // VERBOSE_STATUS
	dev_dbg_ratelimited(self->dev, "[%d] RX_PORT_STS1: %02x\n", port->index,
			    *rx_port_sts1 & 0xff);

	dev_dbg_ratelimited(self->dev, "[%d] RX_PORT_STS2: %02x\n", port->index,
			    *rx_port_sts2 & 0xff);

	dev_dbg_ratelimited(self->dev, "[%d] CSI_RX_STS: %02x\n", port->index,
			    *csi_rx_sts & 0xff);
#endif // VERBOSE_STATUS

	return 0;
}


static void ub960_port_check_lock_sts(struct work_struct *work)
{
	int err;
	struct ub960_port *port = container_of(to_delayed_work(work),
					       struct ub960_port,
					       lock_work);
	struct ub960 *self = port->self;
	unsigned int rx_port_sts1;
	unsigned int rx_port_sts2;
	unsigned int csi_rx_sts;
	bool locked;
	unsigned chan_id = port->index;

	dev_dbg(self->dev, "[%d] %s enter", chan_id, __func__);

	if (!port->configured)
		return;

	err = ub960_get_port_status(port, &rx_port_sts1, &rx_port_sts2, &csi_rx_sts);
	if (err < 0)
		return;

	locked = ((rx_port_sts1 >> UB960_LOCK_STS_OFFSET) & UB960_LOCK_STS_MASK) == 1;

	dev_dbg_ratelimited(self->dev,
		"[%d] CHECK LOCK: locked=%d, port->locked=%d, port->enabled=%d, port->configured=%d\n",
		chan_id, locked, port->locked, port->enabled, port->configured);

	if (locked != port->locked) {
		port->locked = locked;
		if (port->configured && port->enabled && locked) {
			ub960_port_start(self, port);
		}
	}

	dev_info(self->dev, "[%d] probe success", port->index);
}

static int ub960_handle_port_irq(struct ub960 *self, struct ub960_port *port)
{
	int err;
	unsigned int rx_port_sts1;
	unsigned int rx_port_sts2;
	unsigned int csi_rx_sts;

	dev_dbg_ratelimited(self->dev, "HANDLE PORT %d", port->index);

	TRY(err, ub960_get_port_status(port, &rx_port_sts1, &rx_port_sts2, &csi_rx_sts));

	if (rx_port_sts1 & UB960_LOCK_STS_CHG_MASK) {
		cancel_delayed_work_sync(&port->lock_work);
		dev_dbg(self->dev, "port->lock_work scheduled");
		schedule_delayed_work(&port->lock_work, self->lock_settle_time);
	}

	return 0;
}

/**
  * @param self -- Instance data
  * @return The number of ports on which no interrupts were detected, or a
  * negative errno on failure.
  */
static int ub960_check_status(struct ub960 *self)
{
	int err;
	unsigned int status;
	struct ub960_port *port;
	int id;
	int remaining_ports;

	remaining_ports = ARRAY_SIZE(self->ports);
	TRY(err, regmap_read(self->map, UB960_REG_INTERRUPT_STS, &status));

	if (!status)
		return remaining_ports;

	dev_dbg_ratelimited(self->dev, "STATUS: %02x\n", status & 0xff);

	for (id = 0; id < ARRAY_SIZE(self->ports); id++) {
		port = &self->ports[id];

		if (!port->configured) {
			remaining_ports--;
			continue;
		}

		if (!port->enabled) {
			remaining_ports--;
			dev_warn(self->dev, "WARNING: port %i is configured "
			"but is not enabled, ensure this is expected", port->index);
			continue;
		}

		/* Ignore errors, check all ports */
		if (status & (1 << port->index)) {
			if (!ub960_handle_port_irq(self, port)) {
				remaining_ports--;
			}
		} else {
			dev_dbg(self->dev, "Port %i is configured "
			"and enabled, but no interrupt was detected yet", port->index);
		}
	}

	if (remaining_ports) {
		dev_dbg(self->dev, "No activity on %i port(s)", remaining_ports);
	}

	return remaining_ports;
};

static irqreturn_t ub960_handle_irq(int irq, void *arg)
{
	struct ub960 *self = (struct ub960 *)arg;

	ub960_check_status(self);

	return IRQ_HANDLED;
}

static void ub960_cancel_port_lock_work(struct ub960 *self, struct ub960_port *port)
{
	if (!port->configured)
		return;

	dev_dbg(self->dev, "port->lock_work canceled");
	cancel_delayed_work_sync(&port->lock_work);
}

static unsigned int ub960_ports_detected(struct ub960 *self)
{
	int i;
	int num_ports = 0;
	struct ub960_port *port;

	for (i = 0; i < ARRAY_SIZE(self->ports); ++i) {
		port = &(self->ports[i]);
		if (self->ports[i].locked || delayed_work_pending(&(port->lock_work))) {
			num_ports++;
		}
	}

	return num_ports;
}

static void ub960_poll_status(struct work_struct *work)
{
	struct ub960 *self = container_of(to_delayed_work(work), struct ub960,
					  status_work);

	int err = ub960_check_status(self);
	int i;
	bool saw_all_ports = (err == 0);
	bool all_ports_detected =
		(ub960_ports_detected(self) == ARRAY_SIZE(self->ports));

	if (saw_all_ports || all_ports_detected) {
		// nothing more to do, stop polling
		return;
	}

	if (self->status_count > 0) {
		// try again
		schedule_delayed_work(&self->status_work, self->status_period);
		self->status_count--;
		return;
	}

	// ran out of tries;
	if ((i = ub960_ports_detected(self))) {
		dev_warn(self->dev, "WARNING: Only detected activity on %d ports. "
				"Are all sensors connected?", i);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(self->ports); ++i) {
		ub960_cancel_port_lock_work(self, &self->ports[i]);
	}

	dev_err(self->dev, "poll_status failed: saw_all_ports=%d, all_ports_detected=%d, num_ports_detected=%d",
		saw_all_ports,
		all_ports_detected,
		ub960_ports_detected(self));
}

static int ub960_setup_irq(struct ub960 *self)
{
	int err;

	if (self->irq) {
		char *name = devm_kzalloc(self->dev, 48, GFP_KERNEL);

		strcpy(name, dev_name(self->dev));
		strcat(name, "-irq");
		TRY(err, devm_request_threaded_irq(self->dev, self->irq, NULL,
						   ub960_handle_irq, IRQ_TYPE_EDGE_FALLING, name, self));
	} else {
		INIT_DELAYED_WORK(&self->status_work, ub960_poll_status);
		schedule_delayed_work(&self->status_work, self->status_period);
	}

	return 0;
}

/**
 * Called by Linux when loading an instance of the ub960
 *
 * @param client
 * @param id
 *
 * @return 0 on success
 */
static int ub960_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = client->dev.of_node;
	struct ub960 *self = NULL;
	int err = 0;
	bool is_supported = false;

	dev_dbg(dev, "probe enter");

	if (!IS_ENABLED(CONFIG_OF) || !node) {
		dev_err(dev, "of not enabled");
		return -EINVAL;
	}

	TRY_MEM(self, devm_kzalloc(dev, sizeof(*self), GFP_KERNEL));

	i2c_set_clientdata(client, self);
	self->client = client;
	self->dev = &client->dev;
	self->lock_settle_time = HZ / 4;
	self->lock_timeout = 4 * HZ;
	self->status_period = HZ;
	self->status_count = 10;
	self->active_streams = 0;

	TRY(err, ub960_configuration_load(self, node));

	TRY(err, ub960_regulators_get(self));

	TRY(err, ub960_gpios_get(self));

	mutex_init(&self->indirect_access_lock);

	TRY_MEM(self->map, devm_regmap_init_i2c(client, &ub960_regmap_cfg));

	//NOTE: After this point, failure must invoke ub960_remove() to clean up

	err = ub960_reset(self);
	if (err == -EREMOTEIO || err == -ETIMEDOUT) {
		err = -EPROBE_DEFER;
		goto fail;
	}

	err = ub960_is_device_supported(self, &is_supported);
	if (err == -EREMOTEIO || err == -ETIMEDOUT) {
		err = -EPROBE_DEFER;
		goto fail;
	}
	if (!is_supported) {
		dev_err(dev, "unsupported device");
		err = -EINVAL;
		goto fail;
	}

	err = ub960_csi_configure(self);
	if (err)
		goto fail;

	err = ub960_fsync_configure(self, node);
	if (err)
		goto fail;

	err = ub960_ports_configure(self);
	if (err)
		goto fail;

	err = ub960_setup_irq(self);
	if (err)
		goto fail;

	if (self->test_pattern.enabled) {
		err = ub960_test_pattern_enable(self);
		if (err)
			goto fail;
		err = ub960_test_pattern_driver_start(self);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_err(dev, "Probe failure: %i", err);
	ub960_remove(client);
	return err;
}

/**
 * Called by Linux when unloading an instance
 *
 * @param client
 *
 * @return 0 on success
 */
static int ub960_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ub960 *self = i2c_get_clientdata(client);
	int i = 0;

	dev_info(dev, "remove");

	cancel_delayed_work_sync(&self->status_work);

	for (i = 0; i < ARRAY_SIZE(self->ports); ++i) {
		struct ub960_port *port = &self->ports[i];

		ub960_cancel_port_lock_work(self, port);

		if (!port->configured)
			continue;

		while (port->num_aliases > 0) {
			if (!IS_ERR_OR_NULL(port->alias_clients[port->num_aliases]))
				i2c_unregister_device(port->alias_clients[port->num_aliases]);
			port->num_aliases--;
		}

		if (!IS_ERR_OR_NULL(port->ser_client))
			i2c_unregister_device(port->ser_client);
	}

	if (self->muxc)
		i2c_mux_del_adapters(self->muxc);

	if (regulator_is_enabled(self->avdd_reg))
		regulator_disable(self->avdd_reg);
	if (regulator_is_enabled(self->iovdd_reg))
		regulator_disable(self->iovdd_reg);

	return 0;
}


static const struct i2c_device_id ub960_id[] = {
	{"ub960", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ub960_id);


static const struct of_device_id ub960_of_match[] = {
	{ .compatible = "d3,ub960"},
	{ },
};
MODULE_DEVICE_TABLE(of, ub960_of_match);

static struct i2c_driver ub960_driver = {
	.driver = {
		.name = "ub960",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ub960_of_match),
	},
	.probe = ub960_probe,
	.remove = ub960_remove,
	.id_table = ub960_id,
};
module_i2c_driver(ub960_driver);

MODULE_DESCRIPTION("Driver for TI UB960 FPDLINK-III deserializer");
MODULE_VERSION(D3_JETSON_BSP_VERSION);
MODULE_AUTHOR("Greg Rowe <growe@d3engineering.com>");
MODULE_AUTHOR("Josh Watts <jwatts@d3engineering.com>");
MODULE_LICENSE("GPL v2");
