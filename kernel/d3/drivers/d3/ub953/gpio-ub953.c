/**
 * @author Collin Bolles <cbolles@d3engineering.com>
 *
 * UB953 GPIO interface driver. This driver exposes the 4 GPIOs that are
 * available on the UB953 serializer. The UB953 is accessible over I2C, so
 * this driver makes use of a series of regmap calls to control the GPIO pins.
 *
 * Copyright (c) 2021-2021, D3 Engineering. All rights reserved.
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <d3/d3-jetson-bsp.h>
#include <d3/common.h>

/// The UB953 supportes 4 GPIOs GPIO0-GPIO3
#define UB953_NUM_GPIOS 4
/// Note, these are defined in a later version of the Linux kernel
#define GPIO_LINE_DIRECTION_OUT 0
#define GPIO_LINE_DIRECTION_IN 1

/**
 * Represents the internal components required interact with the GPIOs on the
 * UB953.
 *
 * @regmap The regmap interface for communicating with the UB953
 * @dev The device being represented
 * @gpio_chip The gpio representation
 * @fsync_gpio The gpio number for the optional frame sync signal
 */
struct ub953_gpio {
	struct regmap *regmap;
	struct device *dev;
	struct gpio_chip gpio;
	u32 fsync_gpio;
};

/**
 * Stores the register values from the UB953 which are used for interacting
 * with the GPIOs. For more information, refer to the UB953 reference.
 */
enum UB953_GPIO_REG {
	UB953_GPIO_INPUT_CTRL = 0x0E,
	UB953_GPIO_LOCAL_GPIO_DATA = 0x0D,
	UB953_GPIO_PIN_STS = 0x53
};


/**
 * Allocates device managed memory
 *
 * @param dev device to allocate from
 * @param len desired amount of memory
 * @param out where to store pointer
 *
 * @return 0 on success
 */
static int ub953_gpio_kzalloc(struct device *dev, size_t len, void *out)
{
	void **real_out = out;
	if(!(*real_out = devm_kzalloc(dev, len, GFP_KERNEL))) {
		dev_err(dev, "memory allocation failure");
		return -ENOMEM;
	}
	return 0;
}

/**
 * Checks to make sure the gpio_num is within the max number of GPIOs, otherwise
 * prints debug information and returns an error.
 */
static int ub953_check_gpio_num(struct ub953_gpio *self, unsigned int gpio_num)
{
	if(gpio_num >= UB953_NUM_GPIOS) {
		dev_dbg(self->dev, "GPIO not supported, gpio_num: %d", gpio_num);
		return -EINVAL;
	}
	return 0;
}

/**
 * Check to see if a given pin is current enabled as a remote GPIO.
 */
static int ub953_gpio_is_remote_enabled(struct ub953_gpio *self, unsigned int gpio_num) {
	int err;
	u32 gpio_data;

	TRY(err, regmap_read(self->regmap, UB953_GPIO_LOCAL_GPIO_DATA, &gpio_data));

	return (gpio_data >> (gpio_num + 4)) & 0x01;

}


/**
 * Determine the direction of the given GPIO pin. If the GPIO pin identified
 * by the gpio_num is not supported, an error is returned.
 *
 * The GPIO directions are stored in a byte. The top 4 bits represent GPIO
 * pins that are output, the lower 4 bits represent input pins.
 *
 * ex) 0110 1001 => GPIO1, GPIO2 are output and GPIO0, GPIO3 are input.
 *
 * 1000 1111 represents an error since GPIO3 is configured as an input and
 * output.
 */
static int ub953_gpio_get_direction(struct gpio_chip *chip, unsigned int gpio_num)
{
	struct ub953_gpio *self = gpiochip_get_data(chip);
	int err = 0;
	unsigned int reg_value;
	unsigned int direction_input_bit;
	unsigned int direction_output_bit;

	TRY(err, ub953_check_gpio_num(self, gpio_num));

	TRY(err, regmap_read(self->regmap, UB953_GPIO_INPUT_CTRL, &reg_value));

	direction_input_bit = (reg_value >> gpio_num) & 0x01;
	direction_output_bit = (reg_value >> (gpio_num + 4)) & 0x01;

	if(direction_input_bit == direction_output_bit &&
			direction_input_bit == 1) {
		dev_dbg(self->dev, "Direction cannot be both input and output");
		return -EIO;
	}

	return direction_input_bit ? GPIO_LINE_DIRECTION_IN :
		GPIO_LINE_DIRECTION_OUT;
}

static int ub953_gpio_get(struct gpio_chip *chip, unsigned int gpio_num)
{
	struct ub953_gpio *self = gpiochip_get_data(chip);
	int reg_val;
	int err = 0;

	TRY(err, ub953_check_gpio_num(self, gpio_num));

	TRY(err, regmap_read(self->regmap, UB953_GPIO_PIN_STS, &reg_val));

	return ((reg_val >> gpio_num) & 1u);
}

/**
 * Write out the state of the GPIO without impacting the other GPIO states.
 */
static void ub953_gpio_set(struct gpio_chip *chip, unsigned int gpio_num,
	int value)
{
	struct ub953_gpio *self = gpiochip_get_data(chip);
	int err = 0;
	unsigned int mask;
	unsigned int bit_value;

	if(value > 1)
		value = 1;

	err = ub953_check_gpio_num(self, gpio_num);
	if(err) {
		dev_dbg(self->dev, "Failed to set GPIO, invalid gpio_num: %d", gpio_num);
		return;
	}

	// Special case for the frame sync pin which normally is a remote pin.
	// In cases where a sensor has I2C address assignment on GPIO pins,
	// the frame sync pin may need to be manually written to. This work
	// around lets the frame sync be brought out of remote mode, written
	// to, then on the next call to set the frame sync pin, re-enabled
	// remote mode.
	if(gpio_num == self->fsync_gpio) {
		// First time, disable remote and write out value
		if(ub953_gpio_is_remote_enabled(self, self->fsync_gpio) > 0) {
			dev_dbg(self->dev, "Disabling frame sync remote");

			mask = 1 << (gpio_num + 4);
			bit_value = 0;

			err = regmap_update_bits(self->regmap,
						    UB953_GPIO_LOCAL_GPIO_DATA,
						    mask, bit_value);
			if(err) {
				dev_dbg(self->dev, "Failed to disable frame sync remote");
			}
		}
		// Second time, re-enable remote and do not write value
		else {
			dev_dbg(self->dev, "Re-enabling frame sync remote");

			mask = 1 << (gpio_num + 4);
			bit_value = 1 << (gpio_num + 4);

			err = regmap_update_bits(self->regmap,
						    UB953_GPIO_LOCAL_GPIO_DATA,
						    mask, bit_value);
			if(err) {
				dev_dbg(self->dev, "Failed to re-enable frame sync remote");
			}
			return;
		}
	}

	mask = 1 << gpio_num;
	bit_value = value << gpio_num;

	err = regmap_update_bits(self->regmap, UB953_GPIO_LOCAL_GPIO_DATA,
		mask, bit_value);

	if(err) {
		dev_dbg(self->dev,
			"Failed to write state to GPIO, gpio_num: %d, value: %d",
			gpio_num, value);
	}
}

/**
 * Sets the given pin as an input. Needs to clear the output bit and set the
 * input bit.
 */
static int ub953_gpio_direction_input(struct gpio_chip *chip,
	unsigned int gpio_num)
{
	struct ub953_gpio *self = gpiochip_get_data(chip);
	int err = 0;
	unsigned int mask;
	unsigned int bit_value;

	TRY(err, ub953_check_gpio_num(self, gpio_num));

	// Bits of interest are GPIO(gpio_num)'s input and output bits which are
	// 4 bits apart.
	mask = (1 << (gpio_num + 4)) | (1 << gpio_num);
	bit_value = 1 << gpio_num;

	TRY(err, regmap_update_bits(self->regmap, UB953_GPIO_INPUT_CTRL,
		mask, bit_value));

	return err;
}

/**
 * Sets the given pin as output. Needs to clear the input bit and set the
 * output bit.
 */
static int ub953_gpio_direction_output(struct gpio_chip *chip,
	unsigned int gpio_num, int value)
{
	struct ub953_gpio *self = gpiochip_get_data(chip);
	int err = 0;
	unsigned int mask;
	unsigned int bit_value;

	TRY(err, ub953_check_gpio_num(self, gpio_num));

	// Write out the intial value of the GPIO
	ub953_gpio_set(chip, gpio_num, value);

	// Bits of interest are GPIO(gpio_num)'s input and output bits which are
	// 4 bits apart.
	mask = (1 << (gpio_num + 4)) | (1 << gpio_num);
	bit_value = 1 << (gpio_num + 4);

	TRY(err, regmap_update_bits(self->regmap, UB953_GPIO_INPUT_CTRL,
		mask, bit_value));

	return err;
}

/**
 * Enable frame sync pass through on the UB953. If the frame sync gpio is
 * defined, then that pin will be established as a remote output of the UB953.
 * This will allow for a frame sync to be passed through the SerDes link to
 * the imager.
 *
 * If a frame sync pin is not defined, then this function will do nothing.
 *
 * @param self The GPIO device
 */
static int ub953_gpio_enable_fsync(struct ub953_gpio *self)
{
	int err;
	u32 mask;
	u32 bit_value;

	if(self->fsync_gpio >= 0) {
		dev_dbg(self->dev, "Enabling frame sync pass through");

		// Enable pin as output
		mask = (1 << (self->fsync_gpio + 4)) | (1 << self->fsync_gpio);
		bit_value = 1 << (self->fsync_gpio + 4);
		TRY(err, regmap_update_bits(self->regmap,
					    UB953_GPIO_INPUT_CTRL,
					    mask, bit_value));

		// Set pin for remote usage
		mask = 1 << (self->fsync_gpio + 4);
		bit_value = 1 << (self->fsync_gpio + 4);
		TRY(err, regmap_update_bits(self->regmap,
					    UB953_GPIO_LOCAL_GPIO_DATA,
					    mask, bit_value));
	}

	return 0;
}

/**
 * Get the of node from the parent that represents this GPIO controller.
 * This is part of the work around associated with the serializer loading the
 * deserializer which then loads the imager and this gpio controller.
 *
 * Once the work around is fixed this function will no longer be needed.
 *
 * @param self The GPIO device
 * @param parent The parent device node to search through for the gpio
 *	controller of node.
 *
 * @return 0 on success
 */
static int ub953_gpio_get_node(struct device *self, struct device *parent)
{
	struct device_node *parent_node = parent->of_node;
	struct device_node *gpio_node = NULL;

	if(!IS_ENABLED(CONFIG_OF) || !parent_node) {
		dev_warn(self, "Parent node does not have of node");
		return -EINVAL;
	}

	for_each_available_child_of_node(parent_node, gpio_node) {
		if(strstr(gpio_node->name, "gpio")) {
			goto gpio_found;
		}
	}

	dev_warn(self, "GPIO child node not found");
	return -EINVAL;

gpio_found:
	self->of_node = gpio_node;
	return 0;
}

int ub953_gpio_probe(struct platform_device *pdev)
{
	int err = 0;
	struct ub953_gpio *self = NULL;
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "gpio probe start");

	TRY(err, ub953_gpio_get_node(dev, dev->parent));

	TRY(err, ub953_gpio_kzalloc(dev, sizeof(*self), &self));

	self->dev = dev;

	self->regmap = dev_get_regmap(dev->parent, NULL);
	if(!self->regmap) {
		dev_err(dev, "Failed to get regmap from parent, no regmap");
		devm_kfree(dev, self);
		return -ENODEV;
	}

	// Clear initial state
	TRY(err, regmap_write(self->regmap, UB953_GPIO_INPUT_CTRL, 0x0F));
	TRY(err, regmap_write(self->regmap, UB953_GPIO_LOCAL_GPIO_DATA, 0x00));

	// GPIO instance setup
	self->gpio.parent = dev->parent;
	self->gpio.label = "ub953-gpio";
	self->gpio.owner = THIS_MODULE;
	self->gpio.can_sleep = true;
	self->gpio.ngpio = UB953_NUM_GPIOS;
	self->gpio.base = -1;
	self->gpio.of_node = dev->of_node;

	// GPIO related function pointers
	self->gpio.get_direction = ub953_gpio_get_direction;
	self->gpio.direction_input = ub953_gpio_direction_input;
	self->gpio.direction_output = ub953_gpio_direction_output;
	self->gpio.get = ub953_gpio_get;
	self->gpio.set = ub953_gpio_set;

	// Check for frame sync pin
	if (of_property_read_u32(self->gpio.of_node,
				 "fsync-gpio", &self->fsync_gpio)) {
		dev_dbg(self->dev, "No frame sync GPIO defined");
		self->fsync_gpio = -1;
	}

	TRY(err, ub953_gpio_enable_fsync(self));

	err = devm_gpiochip_add_data(dev, &self->gpio, self);
	if(err) {
		dev_err(dev, "Failed to populate data of gpio chip");
		devm_kfree(dev, self);
	}
	return err;
}

static struct of_device_id ub953_gpio_of_match[] =
{
	{ .compatible = "d3,ub953-gpio"},
	{ },
};
MODULE_DEVICE_TABLE(of, ub953_gpio_of_match);

struct platform_driver ub953_gpio_driver =
{
	.driver =
	{
		.name = "ub953-gpio",
		.of_match_table = ub953_gpio_of_match,
	},
	.probe = ub953_gpio_probe,
};

module_platform_driver(ub953_gpio_driver);

MODULE_DESCRIPTION("Driver for TI UB953 FPDLINK-III serializer GPIO");
MODULE_AUTHOR("Collin Bolles <cbolles@d3engineering.com>");
MODULE_LICENSE("GPL v2");
