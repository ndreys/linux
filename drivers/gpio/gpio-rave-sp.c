// SPDX-License-Identifier: GPL-2.0+
/*
 * GPIO driver for RAVE SP
 *
 * Copyright (C) 2018 Zodiac Inflight Innovations
 *
 * Author: Andrey Smirnov <andrew.smirnov@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/mfd/rave-sp.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>

#define RAVE_SP_GPIO_PER_BANK	16

struct rave_sp_gpio_port {
	struct gpio_chip gc;
	struct rave_sp *sp;
	u32 bank;
};

enum rave_sp_gpio_actions {
	RAVE_SP_GPIO_ACTION_SET,
	RAVE_SP_GPIO_ACTION_GET,
	RAVE_SP_GPIO_ACTION_GET_DIRECTION,
	RAVE_SP_GPIO_ACTION_GET_COUNT,
};

struct rave_sp_gpio_control {
	u8 id;
	u8 ack_id;
	u8 action;
	u8 bank;
	__le16 mask;
	__le16 value;
} __packed;

static int rave_sp_gpio_get_multiple(struct gpio_chip *gc,
				     unsigned long *mask,
				     unsigned long *bits)
{
	struct rave_sp_gpio_port *port = gpiochip_get_data(gc);
	struct rave_sp_gpio_control cmd = {
		.id     = RAVE_SP_CMD_GET_GPIO_CTRL,
		.action = RAVE_SP_GPIO_ACTION_GET,
		.bank   = port->bank,
		.mask   = cpu_to_le16(*mask),
	};
	__le16 value;
	int ret;

	ret = rave_sp_exec(port->sp, &cmd, sizeof(cmd), &value, sizeof(value));
	if (ret)
		return ret;

	*bits = le16_to_cpu(value);

	return 0;
}

static void rave_sp_gpio_set_multiple(struct gpio_chip *gc,
				      unsigned long *mask,
				      unsigned long *bits)
{
	struct rave_sp_gpio_port *port = gpiochip_get_data(gc);
	struct rave_sp_gpio_control cmd = {
		.id     = RAVE_SP_CMD_GET_GPIO_CTRL,
		.action = RAVE_SP_GPIO_ACTION_SET,
		.bank   = port->bank,
		.mask   = cpu_to_le16(*mask),
		.value  = cpu_to_le16(*bits),
	};

	if (rave_sp_exec(port->sp, &cmd, sizeof(cmd), NULL, 0))
		dev_err(gc->parent, "Failed to set GPIO(s)\n");
}

static void rave_sp_gpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	unsigned long mask = BIT(gpio);
	unsigned long bits = val ? BIT(gpio) : 0;

	rave_sp_gpio_set_multiple(gc, &mask, &bits);
}

static int rave_sp_gpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	unsigned long mask = BIT(gpio);
	unsigned long bits;
	int ret;

	ret = rave_sp_gpio_get_multiple(gc, &mask, &bits);
	if (ret)
		return ret;

	return !!bits;
}

static int rave_sp_gpio_verify_direction(struct gpio_chip *gc,
					 unsigned int gpio,
					 unsigned int expected)
{
	struct rave_sp_gpio_port *port = gpiochip_get_data(gc);
	const u16 mask = BIT(gpio);
	struct rave_sp_gpio_control cmd = {
		.id     = RAVE_SP_CMD_GET_GPIO_CTRL,
		.action = RAVE_SP_GPIO_ACTION_GET_DIRECTION,
		.bank   = port->bank,
		.mask   = cpu_to_le16(mask),
	};
	__le16 direction;
	int ret;
	/*
	 * Result is a 16 bit mask. Pins that are configured as inputs
	 * have their corresponding bits set to 1.
	 */
	ret = rave_sp_exec(port->sp, &cmd, sizeof(cmd), &direction,
			   sizeof(direction));
	if (ret)
		return ret;

	if ((le16_to_cpu(direction) & mask) != expected)
		return -EINVAL;

	return 0;
}

static int rave_sp_gpio_direction_input(struct gpio_chip *gc,
					unsigned int gpio)
{
	return rave_sp_gpio_verify_direction(gc, gpio, BIT(gpio));
}

static int rave_sp_gpio_direction_output(struct gpio_chip *gc, unsigned gpio,
					 int value)
{
	int ret;

	ret = rave_sp_gpio_verify_direction(gc, gpio, 0);
	if (ret)
		return ret;

	rave_sp_gpio_set(gc, gpio, value);

	return 0;
}

static int rave_sp_gpio_get_count(struct rave_sp *sp)
{
	struct rave_sp_gpio_control cmd = {
		.id     = RAVE_SP_CMD_GET_GPIO_CTRL,
		.action = RAVE_SP_GPIO_ACTION_GET_COUNT,
	};
	__le16 value;
	int ret;

	ret = rave_sp_exec(sp, &cmd, sizeof(cmd), &value, sizeof(value));
	if (ret)
		return ret;

	return le16_to_cpu(value);
}

static int rave_sp_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rave_sp *sp = dev_get_drvdata(dev->parent);
	struct device_node *np = dev->of_node;
	struct device_node *child;
	struct rave_sp_gpio_port *port;
	struct gpio_chip *gc;
	int count, i, ret;

	count = rave_sp_gpio_get_count(sp);
	if (count < 0) {
		dev_err(dev, "Failed to discover supported GPIO count\n");
		return count;
	} else {
		dev_info(dev, "Firmware reported %d GPIOs\n", count);
	}

	for_each_child_of_node(np, child) {
		u32 ngpios;

		port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
		if (!port)
			return -ENOMEM;

		port->sp = sp;

		ret = of_property_read_u32(child, "reg", &port->bank);
		if (ret)
			return ret;

		gc = &port->gc;

		ret = of_property_read_u32(child, "ngpios", &ngpios);
		if (ret)
			return ret;

		gc->base    = -1;
		gc->ngpio   = ngpios;
		gc->of_node = child;
		gc->parent  = dev;
		gc->label   = devm_kasprintf(dev, GFP_KERNEL, "%s-bank%d",
					     dev_name(dev), port->bank);

		gc->direction_input	= rave_sp_gpio_direction_input;
		gc->get			= rave_sp_gpio_get;
		gc->direction_output	= rave_sp_gpio_direction_output;
		gc->set			= rave_sp_gpio_set;

		ret = devm_gpiochip_add_data(dev, gc, port);
		if (ret)
			return ret;

		count -= gc->ngpio;
	}

	if (count)
		dev_warn(dev, "%d GPIO(s) were left unclaimed\n", count);

	return 0;
}

static const struct of_device_id rave_sp_gpio_dt_ids[] = {
	{ .compatible = "zii,rave-sp-gpio", },
	{ /* sentinel */ }
};

static struct platform_driver rave_sp_gpio_driver = {
	.probe		= rave_sp_gpio_probe,
	.driver		= {
		.name = KBUILD_MODNAME,
		.of_match_table = rave_sp_gpio_dt_ids,
	},
};
module_platform_driver(rave_sp_gpio_driver);

MODULE_DEVICE_TABLE(of, rave_sp_gpio_dt_ids);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Smirnov <andrew.smirnov@gmail.com>");
MODULE_DESCRIPTION("RAVE SP GPIO driver");
