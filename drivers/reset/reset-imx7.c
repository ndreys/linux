/*
 * Copyright (c) 2017, Impinj, Inc.
 *
 * i.MX7 System Reset Controller (SRC) driver
 *
 * Author: Andrey Smirnov <andrew.smirnov@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <dt-bindings/reset/imx7-reset.h>

struct imx7_src {
	struct reset_controller_dev rcdev;
	struct device *dev;
	struct regmap *regmap;
};

enum imx7_src_registers {
	SRC_PCIEPHY_RCR = 0x002c,
	SRC_PCIEPHY_RCR_PCIEPHY_G_RST     = BIT(1),
	SRC_PCIEPHY_RCR_PCIEPHY_BTN       = BIT(2),
	SRC_PCIEPHY_RCR_PCIE_CTRL_APPS_EN = BIT(6),
};

static struct imx7_src *to_imx7_src(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct imx7_src, rcdev);
}

static int imx7_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct imx7_src *imx7src = to_imx7_src(rcdev);

	switch (id) {
	case IMX7_RESET_PCIE_CTRL_APPS:
		regmap_update_bits(imx7src->regmap,
				   SRC_PCIEPHY_RCR,
				   SRC_PCIEPHY_RCR_PCIE_CTRL_APPS_EN, 0);
		return 0;

	case IMX7_RESET_PCIEPHY:
		regmap_update_bits(imx7src->regmap,
				   SRC_PCIEPHY_RCR,
				   SRC_PCIEPHY_RCR_PCIEPHY_G_RST,
				   SRC_PCIEPHY_RCR_PCIEPHY_G_RST);
		regmap_update_bits(imx7src->regmap,
				   SRC_PCIEPHY_RCR,
				   SRC_PCIEPHY_RCR_PCIEPHY_BTN,
				   SRC_PCIEPHY_RCR_PCIEPHY_BTN);
		return 0;
	default:
		dev_err(imx7src->dev, "Unknown reset ID %lu\n", id);
		break;
	}

	return -EINVAL;
}

static int imx7_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct imx7_src *imx7src = to_imx7_src(rcdev);

	switch (id) {
	case IMX7_RESET_PCIE_CTRL_APPS:
		regmap_update_bits(imx7src->regmap,
				   SRC_PCIEPHY_RCR,
				   SRC_PCIEPHY_RCR_PCIE_CTRL_APPS_EN,
				   SRC_PCIEPHY_RCR_PCIE_CTRL_APPS_EN);
		return 0;

	case IMX7_RESET_PCIEPHY:
		/* wait for more than 10us to release phy g_rst and btnrst */
		udelay(10);
		regmap_update_bits(imx7src->regmap,
				   SRC_PCIEPHY_RCR,
				   SRC_PCIEPHY_RCR_PCIEPHY_G_RST, 0);
		regmap_update_bits(imx7src->regmap,
				   SRC_PCIEPHY_RCR,
				   SRC_PCIEPHY_RCR_PCIEPHY_BTN, 0);
		return 0;
	default:
		dev_err(imx7src->dev, "Unknown reset ID %lu\n", id);
		break;
	};

	return -EINVAL;
}

static const struct reset_control_ops imx7_reset_ops = {
	.assert		= imx7_reset_assert,
	.deassert	= imx7_reset_deassert,
};

static int imx7_reset_probe(struct platform_device *pdev)
{
	struct imx7_src *imx7src;
	struct device *dev = &pdev->dev;

	imx7src = devm_kzalloc(dev, sizeof(*imx7src), GFP_KERNEL);
	if (!imx7src)
		return -ENOMEM;

	imx7src->dev = dev;
	imx7src->regmap = syscon_node_to_regmap(dev->of_node);
	if (IS_ERR(imx7src->regmap)) {
		dev_err(dev, "Unable to get imx7-src regmap");
		return PTR_ERR(imx7src->regmap);
	}

	imx7src->rcdev.owner     = THIS_MODULE;
	imx7src->rcdev.nr_resets = IMX7_RESET_NUM;
	imx7src->rcdev.ops       = &imx7_reset_ops;
	imx7src->rcdev.of_node   = dev->of_node;

	return devm_reset_controller_register(dev, &imx7src->rcdev);
}

static const struct of_device_id imx7_reset_dt_ids[] = {
	{ .compatible = "fsl,imx7d-src", },
	{ /* sentinel */ },
};

static struct platform_driver imx7_reset_driver = {
	.probe	= imx7_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= imx7_reset_dt_ids,
	},
};
builtin_platform_driver(imx7_reset_driver);
