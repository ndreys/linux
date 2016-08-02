/* Xilinx GMII2RGMII Converter driver
 *
 * Copyright (C) 2016 Xilinx, Inc.
 * Copyright (C) 2016 Andrew Lunn <andrew@lunn.ch>
 *
 * Author: Kedareswara rao Appana <appanad@xilinx.com>
 *
 * Description:
 * This driver is developed for Xilinx GMII2RGMII Converter
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/kernel.h>
#include <linux/mdio.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/types.h>

#define XILINX_GMII2RGMII_FULLDPLX	BMCR_FULLDPLX
#define XILINX_GMII2RGMII_SPEED1000	BMCR_SPEED1000
#define XILINX_GMII2RGMII_SPEED100	BMCR_SPEED100
#define XILINX_GMII2RGMII_REG_NUM	0x10

struct gmii2rgmii {
	struct phy_device *phy_dev;
	struct phy_driver phy_drv;
	struct phy_driver *phy_drv_orig;
};

static int xgmiitorgmii_read_status(struct phy_device *phydev)
{
	printk("inside read_status\n");
	return 0xffff;
}

static int xgmiitorgmii_config_init(struct phy_device *phydev)
{
	printk("Inside gmiitorgmii config_init\n");

	return 0;
}

struct phy_driver gmiitorgmii_drv = {
	.name = "xilinx,gmiitorgmii",
	.features = PHY_GBIT_FEATURES,
	.config_init = xgmiitorgmii_config_init,
	.config_aneg = genphy_config_aneg,
	.read_status = xgmiitorgmii_read_status,
};

int xgmiitorgmii_probe(struct mdio_device *mdiodev)
{
	struct device *dev = &mdiodev->dev;
	struct device_node *np = dev->of_node, *phy_node;
	struct gmii2rgmii *priv;
	int ret = 0;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phy_node = of_parse_phandle(np, "phy-handle", 0);
	if (!phy_node) {
		dev_err(dev, "Couldn't parse phy-handle\n");
		ret = -ENODEV;
		goto out;
	}

	priv->phy_dev = of_phy_find_device(phy_node);
	if (!priv->phy_dev) {
		ret = -EPROBE_DEFER;
		dev_info(dev, "Couldn't find phydev\n");
		goto out;
	}

	priv->phy_drv_orig = priv->phy_dev->drv;
	memcpy(&priv->phy_drv, priv->phy_dev->drv, sizeof(struct phy_driver));
	priv->phy_drv.read_status = xgmiitorgmii_read_status;
	priv->phy_drv.config_init = xgmiitorgmii_config_init;
	priv->phy_dev->drv = &priv->phy_drv;

out:
	return ret;
}

static void xgmiitorgmii_remove(struct mdio_device *mdiodev)
{
	printk("inside remove\n");
}

static const struct of_device_id xgmiitorgmii_of_match[] = {
	{ .compatible = "xilinx,gmiitorgmii" },
};
MODULE_DEVICE_TABLE(of, xgmiitorgmii_of_match);

static struct mdio_driver xgmiitorgmii_driver = {
	.probe	= xgmiitorgmii_probe,
	.remove = xgmiitorgmii_remove,
	.mdiodrv.driver = {
		.name = "xgmiitorgmii",
		.of_match_table = xgmiitorgmii_of_match,
	},
};

static int __init xgmiitorgmii_init(void)
{
	return mdio_driver_register(&xgmiitorgmii_driver);
}
module_init(xgmiitorgmii_init);

static void __exit xgmiitorgmii_cleanup(void)
{
	mdio_driver_unregister(&xgmiitorgmii_driver);
}
module_exit(xgmiitorgmii_cleanup);

MODULE_DEVICE_TABLE(mdio, xilinx_tbl);
MODULE_DESCRIPTION("Xilinx GMII2RGMII converter driver");
MODULE_LICENSE("GPL");
