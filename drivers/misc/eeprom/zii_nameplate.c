/*
 * Export Nameplate information in /sys
 *
 * Copyright (C) 2015 Andrew Lunn
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct attrib {
	struct list_head list;
	struct bin_attribute attr;
};

static LIST_HEAD(attribs);

static ssize_t bin_attr_zii_np_read_string(struct file *filp,
					   struct kobject *kobj,
					   struct bin_attribute *attr,
					   char *buf, loff_t pos, size_t count)
{
	struct nvmem_cell *cell = attr->private;
	void *string;
	size_t len;
	int ret = 0;

	string = nvmem_cell_read(cell, &len);
	if (IS_ERR(string))
		return PTR_ERR(string);

	if (pos >= len) {
		ret = 0;
		goto out;
	}

	if (pos + count > len)
		count = len - pos;

	memcpy(buf, string + pos, count);
	ret = count;

out:
	kfree(string);
	return ret;
}

static const struct bin_attribute bin_attr_string = {
	.attr	= {
		.mode	= S_IRUGO,
	},
	.read	= bin_attr_zii_np_read_string,
};

static ssize_t bin_attr_zii_np_read_integer(struct file *filp,
					    struct kobject *kobj,
					    struct bin_attribute *attr,
					    char *buf, loff_t pos,
					    size_t count)
{
	struct nvmem_cell *cell = attr->private;
	char string[128];
	int ret = 0;
	void *value;
	size_t cell_len;
	size_t len;
	u8 u8;
	u32 u32;

	value = nvmem_cell_read(cell, &cell_len);
	if (IS_ERR(value))
		return PTR_ERR(value);

	switch (cell_len) {
	case 1:
		memcpy(&u8, value, 1);
		len = sprintf(string, "0x%02X", u8);
		break;
	case 4:
		memcpy(&u32, value, 4);
		len = sprintf(string, "0x%08X", u32);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	if (pos >= len) {
		ret = 0;
		goto out;
	}

	if (pos + count > len)
		count = len - pos;

	memcpy(buf, string + pos, count);
	ret = count;

out:
	kfree(value);
	return ret;
}

static const struct bin_attribute bin_attr_integer = {
	.attr	= {
		.mode	= S_IRUGO,
	},
	.read	= bin_attr_zii_np_read_integer,
};

static int zii_np_remove(struct platform_device *pdev)
{
	struct attrib *attrib;

	list_for_each_entry(attrib, &attribs, list)
		device_remove_bin_file(&pdev->dev, &attrib->attr);

	return 0;
}


static int zii_np_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct nvmem_cell *cell;
	struct attrib *attrib;
	const char *name;
	int i = 0;
	int ret;

	for (;;) {
		ret = of_property_read_string_index(np, "strings", i, &name);
		if (ret)
			break;
		cell = devm_nvmem_cell_get(dev, name);
		if (IS_ERR(cell)) {
			dev_warn(dev, "Unable to get nvmem cell %s\n", name);
			ret = PTR_ERR(cell);
			goto out;
		}
		attrib = devm_kzalloc(dev, sizeof(*attrib), GFP_KERNEL);
		if (!attrib) {
			ret = -ENOMEM;
			goto out;
		}

		INIT_LIST_HEAD(&attrib->list);
		attrib->attr = bin_attr_string;
		attrib->attr.attr.name = name;
		attrib->attr.private = cell;

		ret = device_create_bin_file(dev, &attrib->attr);
		if (ret)
			goto out;

		list_add(&attrib->list, &attribs);
		i++;
	}

	i = 0;

	for (;;) {
		ret = of_property_read_string_index(np, "integers", i, &name);
		if (ret)
			break;
		cell = devm_nvmem_cell_get(dev, name);
		if (IS_ERR(cell)) {
			dev_warn(dev, "Unable to get nvmem cell %s\n", name);
			ret = PTR_ERR(cell);
			goto out;
		}
		attrib = devm_kzalloc(dev, sizeof(*attrib), GFP_KERNEL);
		if (!attrib) {
			ret = -ENOMEM;
			goto out;
		}

		INIT_LIST_HEAD(&attrib->list);
		attrib->attr = bin_attr_integer;
		attrib->attr.attr.name = name;
		attrib->attr.private = cell;

		ret = device_create_bin_file(dev, &attrib->attr);
		if (ret)
			goto out;

		list_add(&attrib->list, &attribs);
		i++;
	}

	return 0;
out:
	zii_np_remove(pdev);

	return ret;
}

static const struct of_device_id zii_np_of_match_table[] = {
	{
		.compatible = "zii,nameplate"
	},
	{}
};
MODULE_DEVICE_TABLE(of, orion_spi_of_match_table);

static struct platform_driver zii_np_driver = {
	.driver = {
		.name	= "zii-nameplate",
		.of_match_table = zii_np_of_match_table,
	},
	.probe		= zii_np_probe,
	.remove		= zii_np_remove,
};

module_platform_driver(zii_np_driver);

MODULE_DESCRIPTION("ZII Nameplate driver");
MODULE_AUTHOR("Andrew Lunn <andrew@lunn.ch>");
MODULE_LICENSE("GPL");
