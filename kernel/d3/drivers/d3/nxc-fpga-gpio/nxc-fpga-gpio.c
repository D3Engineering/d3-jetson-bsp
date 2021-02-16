/*
 * Copyright (c) 2020, D3 Engineering  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/init.h>
#include <linux/gpio/driver.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include "nxc-fpga-gpio.h"

static struct i2c_device_id nxc_fg_idtable[] = {
	{"nxc-fpga-gpio", 0},
	{},
}
MODULE_DEVICE_TABLE(i2c, nxc_fg_idtable);

static struct of_device_id nxc_fg_of_match[] = {
	{ .compatible = "d3,nxc-fpga-gpio"},
	{},
};
MODULE_DEVICE_TABLE(of, nxc_fg_of_match);

static const struct regmap_config nxc_fg_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int nxc_fg_regmap_read(struct regmap *map, u32 reg, u32 *val,
		const char *func, unsigned offset, struct device *dev)
{
	int ret = 0;
	ret = regmap_read(map, reg, val);
	if (ret < 0) {
		dev_err(dev, "%s GPIO %d Read: Error 0x%x(reg)",
				func, offset, reg);
		return ret;
	}
	dev_dbg(dev, "%s GPIO %d Read: 0x%x(reg) 0x%x(val)",
			func, offset, reg, *val);
	return 0;
}

static int nxc_fg_regmap_write(struct regmap *map, u32 reg, u32 val,
		const char *func, unsigned offset, struct device *dev)
{
	int ret = 0;
	ret = regmap_write(map, reg, val);
	if (ret < 0) {
		dev_err(dev, "%s GPIO %d Write: Error 0x%x(reg)",
				func, offset, reg);
		return ret;
	}
	dev_dbg(dev, "%s GPIO %d Write: 0x%x(reg) 0x%x(val)",
			func, offset, reg, val);
	return 0;
}

static int nxc_fg_get_gpio_struct(struct nxc_fg *priv,
		struct nxc_gpio **nxc_gpio, unsigned offset)
{
	if (offset < priv->chip.ngpio) {
		*nxc_gpio = priv->gpio_table + offset;
		return 0;
	}
	return -EINVAL;
}

static int nxc_fg_get_direction(struct gpio_chip *chip, unsigned offset)
{
	int ret;
	u32 val = 0;
	u32 reg = 0;
	struct nxc_fg *priv;
	struct nxc_gpio *nxc_gpio;
	priv = container_of(chip, struct nxc_fg, chip);
	ret = nxc_fg_get_gpio_struct(priv, &nxc_gpio, offset);
	if (ret < 0) {
		return ret;
	}
	reg = NXC_REG_OFFSET_OUTPUT_EN(nxc_gpio->reg_bank_start);
	ret = nxc_fg_regmap_read(priv->map, reg, &val,
			__func__, offset, chip->parent);
	if (ret)
		return ret;
	return (val & (1 << nxc_gpio->bit_offset));
}

static int nxc_fg_direction_input(struct gpio_chip *chip, unsigned offset)
{
	int ret;
	u32 val = 0;
	u32 reg = 0;
	struct nxc_fg *priv;
	struct nxc_gpio *nxc_gpio;
	priv = container_of(chip, struct nxc_fg, chip);
	ret = nxc_fg_get_gpio_struct(priv, &nxc_gpio, offset);
	if (ret < 0) {
		return ret;
	}
	reg = NXC_REG_OFFSET_OUTPUT_EN(nxc_gpio->reg_bank_start);
	ret = nxc_fg_regmap_read(priv->map, reg, &val,
			__func__, offset, chip->parent);
	if (ret)
		return ret;
	/* Calculate value to write */
	val = (val & ~( 1 << nxc_gpio->bit_offset));
	ret = nxc_fg_regmap_write(priv->map, reg, val,
			__func__, offset, chip->parent);
	if (ret)
		return ret;
	return 0;
}

static void nxc_fg_gpio_set(struct gpio_chip *chip, unsigned offset,
		int bitvalue)
{
	int ret;
	u32 val = 0;
	u32 reg = 0;
	struct nxc_fg *priv;
	struct nxc_gpio *nxc_gpio;
	priv = container_of(chip, struct nxc_fg, chip);
	ret = nxc_fg_get_gpio_struct(priv, &nxc_gpio, offset);
	if (ret < 0) {
		dev_err(chip->parent, "Error getting gpio_struct GPIO %d",
				offset);
		return;
	}
	reg = NXC_REG_OFFSET_OUTPUT(nxc_gpio->reg_bank_start);
	ret = nxc_fg_regmap_read(priv->map, reg, &val,
			__func__, offset, chip->parent);
	if (ret)
		return;
	/* Calculate value to write */
	if (bitvalue)
		val = (val | ( 1 << nxc_gpio->bit_offset));
	else
		val = (val & ~( 1 << nxc_gpio->bit_offset));
	/* Write value */
	ret = nxc_fg_regmap_write(priv->map, reg, val,
			__func__, offset, chip->parent);
	if (ret)
		return;
}

static int nxc_fg_direction_output(struct gpio_chip *chip, unsigned offset,
		int bitvalue)
{
	int ret;
	u32 val = 0;
	u32 reg = 0;
	struct nxc_fg *priv;
	struct nxc_gpio *nxc_gpio;
	priv = container_of(chip, struct nxc_fg, chip);
	ret = nxc_fg_get_gpio_struct(priv, &nxc_gpio, offset);
	if (ret < 0) {
		return ret;
	}
	reg = NXC_REG_OFFSET_OUTPUT_EN(nxc_gpio->reg_bank_start);
	ret = nxc_fg_regmap_read(priv->map, reg, &val,
			__func__, offset, chip->parent);
	if (ret)
		return ret;
	/* Set output value */
	nxc_fg_gpio_set(chip, offset, bitvalue);
 	/* Calculate value to write */
	val = (val | ( 1 << nxc_gpio->bit_offset));
	ret = nxc_fg_regmap_write(priv->map, reg, val,
			__func__, offset, chip->parent);
	if (ret)
		return ret;
	return 0;
}


static int nxc_fg_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	int ret;
	u32 val = 0;
	u32 reg = 0;
	struct nxc_fg *priv;
	struct nxc_gpio *nxc_gpio;
	priv = container_of(chip, struct nxc_fg, chip);
	ret = nxc_fg_get_gpio_struct(priv, &nxc_gpio, offset);
	if (ret < 0) {
		return ret;
	}
	/* Determine gpio pin mode */
	reg = NXC_REG_OFFSET_OUTPUT_EN(nxc_gpio->reg_bank_start);
	ret = nxc_fg_regmap_read(priv->map, reg, &val,
			__func__, offset, chip->parent);
	if (ret)
		return ret;
	if (val & (1 << nxc_gpio->bit_offset))
		reg = NXC_REG_OFFSET_OUTPUT(nxc_gpio->reg_bank_start);
	else
		reg = NXC_REG_OFFSET_INPUT(nxc_gpio->reg_bank_start);
	/* Read corresponding register */
	ret = nxc_fg_regmap_read(priv->map, reg, &val,
			__func__, offset, chip->parent);
	if (ret)
		return ret;
	return (val & (1 << nxc_gpio->bit_offset));
}

static struct gpio_chip nxc_gpio_chip = {
	.label = "nxc-fpga-gpio",
	.owner = THIS_MODULE,
	.get_direction = nxc_fg_get_direction,
	.direction_input = nxc_fg_direction_input,
	.direction_output = nxc_fg_direction_output,
	.set = nxc_fg_gpio_set,
	.get = nxc_fg_gpio_get,
	.can_sleep = false,
	.base = -1,
	.ngpio = 0, /* Change at probe */
};

static int nxc_fg_get_reset_gpio(struct nxc_fg *priv)
{
	int ret = 0;
	priv->reset_gpio = 0;
	ret = of_get_named_gpio(priv->client->dev.of_node, "reset-gpio", 0);
	if (!gpio_is_valid(ret)) {
		dev_dbg(priv->chip.parent,
			"No reset-gpio found, continuing: %d", ret);
		return ret;
	}
	dev_dbg(priv->chip.parent, "Found reset-gpio");
	gpio_direction_output(ret, 0);
	dev_dbg(priv->chip.parent,
		"Finished setting up reset gpio as output");
	priv->reset_gpio = ret;
	return 0;
}

void nxc_fg_set_gpio(unsigned gpio, int state)
{
	if (gpio_cansleep(gpio))
		gpio_set_value_cansleep(gpio, state);
	else
		gpio_set_value(gpio, state);
}

static int nxc_fg_reset(struct nxc_fg *priv)
{
	dev_dbg(priv->chip.parent, "Resetting fpga");
	if (!priv->reset_gpio) {
		dev_dbg(priv->chip.parent,
			"Unable to reset fpga without reset-gpio!");
		return -EINVAL;
	}
	nxc_fg_set_gpio(priv->reset_gpio, 1);
	usleep_range(20000, 100000);
	nxc_fg_set_gpio(priv->reset_gpio, 0);
	usleep_range(20000, 100000);
	return 0;
}

static int nxc_fg_board_specific_config(struct nxc_fg *priv)
{
	int ret = 0;
	int i = 0;
	int pca_val[NXC_BOARD_PCA_REG_COUNT] = {};
	int fw_rev = 0;
	/* read firmware revision */
	ret = regmap_read(priv->map, NXC_FPGA_FW_REV_REG, &fw_rev);
	if (ret) {
		dev_err(priv->chip.parent,
			"Error reading firmware revision on FPGA!");
		return ret;
	}
	/* read PCA board identifier */
	for (i = 0; i < NXC_BOARD_PCA_REG_COUNT; i++) {
		ret = regmap_read(priv->map,
				NXC_BOARD_PCA_REG_START + i, pca_val + i);
		if (ret) {
			dev_err(priv->chip.parent,
				"Error reading PCA number on FPGA!");
			return ret;
		}
	}
	/* prepare configuration */
	if (!memcmp(nxc_pca_ref_table_00c091001, pca_val,
			sizeof(*nxc_pca_ref_table_00c091001) *
			NXC_BOARD_PCA_REG_COUNT)) {
		if (fw_rev >= NXC_00C091001_FPGA_MIN_VERSION &&
				fw_rev <= NXC_00C091001_FPGA_MAX_VERSION) {
			dev_info(priv->chip.parent,
				"Using GPIOs for PCA 00c091001");
			priv->gpio_table = nxc_00c091001_gpio_table;
			priv->chip.ngpio = ARRAY_SIZE(nxc_00c091001_gpio_table);
		}
		else {
			dev_err(priv->chip.parent,
				"Outdated FPGA version %x for PCA 00c091001",
				fw_rev);
			dev_err(priv->chip.parent,
				"Minimum FPGA version is %x",
				NXC_00C091001_FPGA_MIN_VERSION);
			return -EINVAL;
		}

	}
	else if (!memcmp(nxc_pca_ref_table_276003001, pca_val,
			sizeof(*nxc_pca_ref_table_276003001) *
			NXC_BOARD_PCA_REG_COUNT)) {
		if (fw_rev >= NXC_276003001_FPGA_MIN_VERSION &&
				fw_rev <= NXC_276003001_FPGA_MAX_VERSION ) {
			dev_info(priv->chip.parent,
				"Using GPIOs for PCA 276003001");
			priv->gpio_table = nxc_276003001_gpio_table;
			priv->chip.ngpio = ARRAY_SIZE(nxc_276003001_gpio_table);
		}
		else {
			dev_err(priv->chip.parent,
				"Outdated FPGA version %x for PCA 276003001",
				fw_rev);
			dev_err(priv->chip.parent,
				"Minimum FPGA version is %x",
				NXC_276003001_FPGA_MIN_VERSION);
			return -EINVAL;
		}
	}
	else {
		dev_err(priv->chip.parent,
			"Error, did recognize PCA reported by FPGA");
		return -ENODEV;
	}
	return 0;
}

static int nxc_fg_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	struct nxc_fg *priv = NULL;

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "Failed to allocate memory");
		return -ENOMEM;
	}

	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->map = devm_regmap_init_i2c(client, &nxc_fg_regmap_config);
	if (IS_ERR(priv->map)) {
		dev_err(&client->dev, "Failed to create regmap");
		return PTR_ERR(priv->map);
	}

	priv->chip = nxc_gpio_chip;
	priv->chip.parent = &client->dev;

	priv->reset_gpio = 0;
	nxc_fg_get_reset_gpio(priv);

	nxc_fg_reset(priv);
	ret = nxc_fg_board_specific_config(priv);
	if (ret) {
		dev_err(&client->dev, "Failed to find a valid configuration");
		return ret;
	}
	ret = gpiochip_add_data(&priv->chip, priv);
	if (ret) {
		dev_err(&client->dev, "Unable to register gpiochip");
		return ret;
	}

	return ret;
}

static int nxc_fg_remove(struct i2c_client *client)
{
	int ret = 0;
	struct nxc_fg *priv = i2c_get_clientdata(client);
	dev_dbg(priv->chip.parent, "Removing driver");
	gpiochip_remove(&priv->chip);
	devm_kfree(&client->dev, priv);
	return ret;
}

static void nxc_fg_shutdown(struct i2c_client *client)
{
	struct nxc_fg *priv = i2c_get_clientdata(client);
	dev_dbg(priv->chip.parent, "Shutdown");
	nxc_fg_reset(priv);
}

static struct i2c_driver nxc_fg_driver = {
	.driver = {
		.name = "nxc-fg",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(nxc_fg_of_match),
	},
	.probe = nxc_fg_probe,
	.remove = nxc_fg_remove,
	.shutdown = nxc_fg_shutdown,
	.id_table = nxc_fg_idtable,
};

static int __init nxc_fg_init(void)
{
	return i2c_add_driver(&nxc_fg_driver);
}

static void __exit nxc_fg_exit(void)
{
	i2c_del_driver(&nxc_fg_driver);
}

postcore_initcall(nxc_fg_init);
module_exit(nxc_fg_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Tyler Hart <thart@d3engineering.com>");
MODULE_DESCRIPTION("D3 Xavier NX Carrier FPGA GPIO Controller driver");
MODULE_VERSION("1.0");
