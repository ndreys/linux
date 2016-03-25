/*
 * net/dsa/mv88e6123_61_65.c - Marvell 88e6123/6161/6165 switch chip support
 * Copyright (c) 2008-2009 Marvell Semiconductor
 * Copyright (c) 2015 Andrew Lunn <andrew@lunn.ch>
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
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <net/dsa.h>
#include "mv88e6xxx.h"

static const struct mv88e6xxx_switch_id mv88e6123_table[] = {
	{ PORT_SWITCH_ID_6123,		"Marvell 88E6123",		3 },
	{ PORT_SWITCH_ID_6123_A1,	"Marvell 88E6123 (A1)",		3 },
	{ PORT_SWITCH_ID_6123_A2,	"Marvell 88E6123 (A2)",		3 },
	{ PORT_SWITCH_ID_6161,		"Marvell 88E6161",		6 },
	{ PORT_SWITCH_ID_6161_A1,	"Marvell 88E6161 (A1)",		6 },
	{ PORT_SWITCH_ID_6161_A2,	"Marvell 88E6161 (A2)",		6 },
	{ PORT_SWITCH_ID_6165,		"Marvell 88E6165",		6 },
	{ PORT_SWITCH_ID_6165_A1,	"Marvell 88E6165 (A1)",		6 },
	{ PORT_SWITCH_ID_6165_A2,	"Marvell 88e6165 (A2)",		6 },
};

static char *mv88e6123_drv_probe(struct device *dsa_dev,
				 struct device *host_dev,
				 int sw_addr, void **priv)
{
	return mv88e6xxx_drv_probe(dsa_dev, host_dev, sw_addr, priv,
				   mv88e6123_table,
				   ARRAY_SIZE(mv88e6123_table));
}

static int mv88e6123_setup_global(struct dsa_switch *ds)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	u32 upstream_port = dsa_upstream_port(ds);
	int ret;
	u32 reg;

	ret = mv88e6xxx_setup_global(ds);
	if (ret)
		return ret;

	/* Disable the PHY polling unit (since there won't be any
	 * external PHYs to poll), don't discard packets with
	 * excessive collisions, and mask all interrupt sources.
	 */
	ret = mv88e6xxx_reg_write(ps, REG_GLOBAL, GLOBAL_CONTROL, 0x0000);
	if (ret)
		return ret;

	/* Configure the upstream port, and configure the upstream
	 * port as the port to which ingress and egress monitor frames
	 * are to be sent.
	 */
	reg = upstream_port << GLOBAL_MONITOR_CONTROL_INGRESS_SHIFT |
		upstream_port << GLOBAL_MONITOR_CONTROL_EGRESS_SHIFT |
		upstream_port << GLOBAL_MONITOR_CONTROL_ARP_SHIFT;
	ret = mv88e6xxx_reg_write(ps, REG_GLOBAL, GLOBAL_MONITOR_CONTROL, reg);
	if (ret)
		return ret;

	/* Disable remote management for now, and set the switch's
	 * DSA device number.
	 */
	return mv88e6xxx_reg_write(ps, REG_GLOBAL, GLOBAL_CONTROL_2,
				   ds->index & 0x1f);
}

static int mv88e6123_setup(struct dsa_switch *ds)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int ret;

	ds->slave_mii_bus = ps->mdio_bus;
	ps->dev = ds->dev;
	ps->ds = ds;

	ret = mv88e6xxx_switch_reset(ps, false);
	if (ret < 0)
		return ret;

	ret = mv88e6123_setup_global(ds);
	if (ret < 0)
		return ret;

	return mv88e6xxx_setup_ports(ds);
}

struct dsa_switch_driver mv88e6123_switch_driver = {
	.tag_protocol		= DSA_TAG_PROTO_EDSA,
	.probe			= mv88e6123_drv_probe,
	.setup			= mv88e6123_setup,
	.set_addr		= mv88e6xxx_set_addr_indirect,
	.get_strings		= mv88e6xxx_get_strings,
	.get_ethtool_stats	= mv88e6xxx_get_ethtool_stats,
	.get_sset_count		= mv88e6xxx_get_sset_count,
	.adjust_link		= mv88e6xxx_adjust_link,
#ifdef CONFIG_NET_DSA_HWMON
	.get_temp		= mv88e6xxx_get_temp,
#endif
	.get_regs_len		= mv88e6xxx_get_regs_len,
	.get_regs		= mv88e6xxx_get_regs,
};

static int mv88e6123_probe(struct mdio_device *mdiodev)
{
	return mv88e6xxx_probe(mdiodev, &mv88e6123_switch_driver,
			       mv88e6123_table, ARRAY_SIZE(mv88e6123_table));
}

static const struct of_device_id mv88e6123_of_match[] = {
	{ .compatible = "marvell,mv88e6123" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mv88e6123_of_match);

static struct mdio_driver mv88e6123_driver = {
	.probe  = mv88e6123_probe,
	.remove = mv88e6xxx_remove,
	.mdiodrv.driver = {
		.name = "mv88e6123",
		.of_match_table = mv88e6123_of_match,
	},
};

mv88e6xxx_module_driver(mv88e6123_driver, mv88e6123_switch_driver);

MODULE_DESCRIPTION("Driver for Marvell 6123 family ethernet switch chips");
MODULE_LICENSE("GPL");
