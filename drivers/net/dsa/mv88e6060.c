/*
 * net/dsa/mv88e6060.c - Driver for Marvell 88e6060 switch chips
 * Copyright (c) 2008-2009 Marvell Semiconductor
 * Copywrite (c) 2015 Andrew Lunn <andrew@lunn.ch>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/component.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <net/dsa.h>
#include "mv88e6060.h"

static int reg_read(struct dsa_switch *ds, int addr, int reg)
{
	struct mv88e6060_priv *priv = ds_to_priv(ds);

	return mdiobus_read_nested(priv->bus, priv->sw_addr + addr, reg);
}

#define REG_READ(addr, reg)					\
	({							\
		int __ret;					\
								\
		__ret = reg_read(ds, addr, reg);		\
		if (__ret < 0)					\
			return __ret;				\
		__ret;						\
	})


static int reg_write(struct dsa_switch *ds, int addr, int reg, u16 val)
{
	struct mv88e6060_priv *priv = ds_to_priv(ds);

	return mdiobus_write_nested(priv->bus, priv->sw_addr + addr, reg, val);
}

#define REG_WRITE(addr, reg, val)				\
	({							\
		int __ret;					\
								\
		__ret = reg_write(ds, addr, reg, val);		\
		if (__ret < 0)					\
			return __ret;				\
	})

static char *mv88e6060_name(struct mii_bus *bus, int sw_addr)
{
	int ret;

	ret = mdiobus_read(bus, sw_addr + REG_PORT(0), PORT_SWITCH_ID);
	if (ret >= 0) {
		if (ret == PORT_SWITCH_ID_6060)
			return "Marvell 88E6060 (A0)";
		if (ret == PORT_SWITCH_ID_6060_R1 ||
		    ret == PORT_SWITCH_ID_6060_R2)
			return "Marvell 88E6060 (B0)";
		if ((ret & PORT_SWITCH_ID_6060_MASK) == PORT_SWITCH_ID_6060)
			return "Marvell 88E6060";
	}

	return NULL;
}

static char *mv88e6060_drv_probe(struct device *host_dev, int sw_addr)
{
	struct mii_bus *bus = dsa_host_dev_to_mii_bus(host_dev);

	if (!bus)
		return NULL;

	return mv88e6060_name(bus, sw_addr);
}

static int mv88e6060_switch_reset(struct dsa_switch *ds)
{
	int i;
	int ret;
	unsigned long timeout;

	/* Set all ports to the disabled state. */
	for (i = 0; i < MV88E6060_PORTS; i++) {
		ret = REG_READ(REG_PORT(i), PORT_CONTROL);
		REG_WRITE(REG_PORT(i), PORT_CONTROL,
			  ret & ~PORT_CONTROL_STATE_MASK);
	}

	/* Wait for transmit queues to drain. */
	usleep_range(2000, 4000);

	/* Reset the switch. */
	REG_WRITE(REG_GLOBAL, GLOBAL_ATU_CONTROL,
		  GLOBAL_ATU_CONTROL_SWRESET |
		  GLOBAL_ATU_CONTROL_ATUSIZE_1024 |
		  GLOBAL_ATU_CONTROL_ATE_AGE_5MIN);

	/* Wait up to one second for reset to complete. */
	timeout = jiffies + 1 * HZ;
	while (time_before(jiffies, timeout)) {
		ret = REG_READ(REG_GLOBAL, GLOBAL_STATUS);
		if (ret & GLOBAL_STATUS_INIT_READY)
			break;

		usleep_range(1000, 2000);
	}
	if (time_after(jiffies, timeout))
		return -ETIMEDOUT;

	return 0;
}

static int mv88e6060_setup_global(struct dsa_switch *ds)
{
	/* Disable discarding of frames with excessive collisions,
	 * set the maximum frame size to 1536 bytes, and mask all
	 * interrupt sources.
	 */
	REG_WRITE(REG_GLOBAL, GLOBAL_CONTROL, GLOBAL_CONTROL_MAX_FRAME_1536);

	/* Enable automatic address learning, set the address
	 * database size to 1024 entries, and set the default aging
	 * time to 5 minutes.
	 */
	REG_WRITE(REG_GLOBAL, GLOBAL_ATU_CONTROL,
		  GLOBAL_ATU_CONTROL_ATUSIZE_1024 |
		  GLOBAL_ATU_CONTROL_ATE_AGE_5MIN);

	return 0;
}

static int mv88e6060_setup_port(struct dsa_switch *ds, int p)
{
	int addr = REG_PORT(p);

	/* Do not force flow control, disable Ingress and Egress
	 * Header tagging, disable VLAN tunneling, and set the port
	 * state to Forwarding.  Additionally, if this is the CPU
	 * port, enable Ingress and Egress Trailer tagging mode.
	 */
	REG_WRITE(addr, PORT_CONTROL,
		  dsa_is_cpu_port(ds, p) ?
			PORT_CONTROL_TRAILER |
			PORT_CONTROL_INGRESS_MODE |
			PORT_CONTROL_STATE_FORWARDING :
			PORT_CONTROL_STATE_FORWARDING);

	/* Port based VLAN map: give each port its own address
	 * database, allow the CPU port to talk to each of the 'real'
	 * ports, and allow each of the 'real' ports to only talk to
	 * the CPU port.
	 */
	REG_WRITE(addr, PORT_VLAN_MAP,
		  ((p & 0xf) << PORT_VLAN_MAP_DBNUM_SHIFT) |
		   (dsa_is_cpu_port(ds, p) ?
			ds->phys_port_mask :
			BIT(ds->dst->cpu_port)));

	/* Port Association Vector: when learning source addresses
	 * of packets, add the address to the address database using
	 * a port bitmap that has only the bit for this port set and
	 * the other bits clear.
	 */
	REG_WRITE(addr, PORT_ASSOC_VECTOR, BIT(p));

	return 0;
}

static int mv88e6060_setup(struct dsa_switch *ds, struct device *dev)
{
	int i;
	int ret;
	struct mv88e6060_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ds->priv = priv;
	priv->bus = dsa_host_dev_to_mii_bus(ds->master_dev);
	priv->sw_addr = ds->pd->sw_addr;

	ret = mv88e6060_switch_reset(ds);
	if (ret < 0)
		return ret;

	/* @@@ initialise atu */

	ret = mv88e6060_setup_global(ds);
	if (ret < 0)
		return ret;

	for (i = 0; i < MV88E6060_PORTS; i++) {
		ret = mv88e6060_setup_port(ds, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int mv88e6060_set_addr(struct dsa_switch *ds, u8 *addr)
{
	/* Use the same MAC Address as FD Pause frames for all ports */
	REG_WRITE(REG_GLOBAL, GLOBAL_MAC_01, (addr[0] << 9) | addr[1]);
	REG_WRITE(REG_GLOBAL, GLOBAL_MAC_23, (addr[2] << 8) | addr[3]);
	REG_WRITE(REG_GLOBAL, GLOBAL_MAC_45, (addr[4] << 8) | addr[5]);

	return 0;
}

static int mv88e6060_port_to_phy_addr(int port)
{
	if (port >= 0 && port < MV88E6060_PORTS)
		return port;
	return -1;
}

static int mv88e6060_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	int addr;

	addr = mv88e6060_port_to_phy_addr(port);
	if (addr == -1)
		return 0xffff;

	return reg_read(ds, addr, regnum);
}

static int
mv88e6060_phy_write(struct dsa_switch *ds, int port, int regnum, u16 val)
{
	int addr;

	addr = mv88e6060_port_to_phy_addr(port);
	if (addr == -1)
		return 0xffff;

	return reg_write(ds, addr, regnum, val);
}

static struct dsa_switch_driver mv88e6060_switch_driver = {
	.tag_protocol	= DSA_TAG_PROTO_TRAILER,
	.probe		= mv88e6060_drv_probe,
	.setup		= mv88e6060_setup,
	.set_addr	= mv88e6060_set_addr,
	.phy_read	= mv88e6060_phy_read,
	.phy_write	= mv88e6060_phy_write,
};

static int mv88e6060_bind(struct device *dev,
			  struct device *master, void *data)
{
	struct dsa_switch_tree *dst = data;
	struct mv88e6060_priv *priv;
	struct device_node *np = dev->of_node;
	struct dsa_switch *ds;
	const char *name;
	int ret = 0;

	ds = devm_kzalloc(dev, sizeof(*ds) + sizeof(*priv), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;

	priv = (struct mv88e6060_priv *)(ds + 1);
	ds->priv = priv;

	ret = of_mdio_parse_bus_and_addr(dev, np, &priv->bus, &priv->sw_addr);
	if (ret)
		return ret;

	get_device(&priv->bus->dev);

	ds->drv = &mv88e6060_switch_driver;

	name = mv88e6060_name(priv->bus, priv->sw_addr);
	if (!name) {
		dev_err(dev, "Failed to find switch");
		return -ENODEV;
	}

	dev_set_drvdata(dev, ds);
	dsa_switch_register(dst, ds, np, name);

	return 0;
}

void mv88e6060_unbind(struct device *dev, struct device *master, void *data)
{
	struct dsa_switch *ds = dev_get_drvdata(dev);
	struct mv88e6060_priv *priv = ds_to_priv(ds);

	dsa_switch_unregister(ds);
	put_device(&priv->bus->dev);
}

static const struct component_ops mv88e6060_component_ops = {
	.bind = mv88e6060_bind,
	.unbind = mv88e6060_unbind,
};

static int mv88e6060_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mv88e6060_component_ops);

	return 0;
}

static int mv88e6060_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &mv88e6060_component_ops);
}

static const struct of_device_id mv88e6060_of_match[] = {
	{ .compatible = "marvell,mv88e6060" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mv88e6060_of_match);

static struct platform_driver mv88e6060_driver = {
	.probe  = mv88e6060_probe,
	.remove = mv88e6060_remove,
	.driver = {
		.name = "mv88e6060",
		.of_match_table = mv88e6060_of_match,
	},
};

static int __init mv88e6060_init(void)
{
	register_switch_driver(&mv88e6060_switch_driver);

	return platform_driver_register(&mv88e6060_driver);
}
module_init(mv88e6060_init);

static void __exit mv88e6060_cleanup(void)
{
	platform_driver_unregister(&mv88e6060_driver);

	unregister_switch_driver(&mv88e6060_switch_driver);
}
module_exit(mv88e6060_cleanup);

MODULE_AUTHOR("Lennert Buytenhek <buytenh@wantstofly.org>");
MODULE_DESCRIPTION("Driver for Marvell 88E6060 ethernet switch chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mv88e6060");
