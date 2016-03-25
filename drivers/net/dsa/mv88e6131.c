/*
 * net/dsa/mv88e6131.c - Marvell 88e6095/6095f/6131 switch chip support
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

static const struct mv88e6xxx_switch_id mv88e6131_table[] = {
	{ PORT_SWITCH_ID_6085,		"Marvell 88E6085",		10 },
	{ PORT_SWITCH_ID_6095,		"Marvell 88E6095/88E6095F",	11 },
	{ PORT_SWITCH_ID_6131,		"Marvell 88E6131",		8 },
	{ PORT_SWITCH_ID_6131_B2,	"Marvell 88E6131 (B2)",		8 },
	{ PORT_SWITCH_ID_6185,		"Marvell 88E6185",		10 },
};

static char *mv88e6131_drv_probe(struct device *dsa_dev,
				 struct device *host_dev,
				 int sw_addr, void **priv)
{
	return mv88e6xxx_drv_probe(dsa_dev, host_dev, sw_addr, priv,
				   mv88e6131_table,
				   ARRAY_SIZE(mv88e6131_table));
}

static int mv88e6131_setup_global(struct dsa_switch *ds)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	u32 upstream_port = dsa_upstream_port(ds);
	int ret;
	u32 reg;

	ret = mv88e6xxx_setup_global(ds);
	if (ret)
		return ret;

	/* Enable the PHY polling unit, don't discard packets with
	 * excessive collisions, use a weighted fair queueing scheme
	 * to arbitrate between packet queues, set the maximum frame
	 * size to 1632, and mask all interrupt sources.
	 */
	ret = mv88e6xxx_reg_write(ps, REG_GLOBAL, GLOBAL_CONTROL,
				  GLOBAL_CONTROL_PPU_ENABLE |
				  GLOBAL_CONTROL_MAX_FRAME_1632);
	if (ret)
		return ret;

	/* Set the VLAN ethertype to 0x8100. */
	ret = mv88e6xxx_reg_write(ps, REG_GLOBAL, GLOBAL_CORE_TAG_TYPE, 0x8100);
	if (ret)
		return ret;

	/* Disable ARP mirroring, and configure the upstream port as
	 * the port to which ingress and egress monitor frames are to
	 * be sent.
	 */
	reg = upstream_port << GLOBAL_MONITOR_CONTROL_INGRESS_SHIFT |
		upstream_port << GLOBAL_MONITOR_CONTROL_EGRESS_SHIFT |
		GLOBAL_MONITOR_CONTROL_ARP_DISABLED;
	ret = mv88e6xxx_reg_write(ps, REG_GLOBAL, GLOBAL_MONITOR_CONTROL, reg);
	if (ret)
		return ret;

	/* Set the switch's DSA device number and enable the use of
	 * the routing table.
	 */
	ret = mv88e6xxx_reg_write(ps, REG_GLOBAL, GLOBAL_CONTROL_2,
				  GLOBAL_CONTROL_2_MULTIPLE_CASCADE |
				  (ds->index & 0x1f));
	if (ret)
		return ret;

	/* Force the priority of IGMP/MLD snoop frames and ARP frames
	 * to the highest setting.
	 */
	return mv88e6xxx_reg_write(ps, REG_GLOBAL2, GLOBAL2_PRIO_OVERRIDE,
				   GLOBAL2_PRIO_OVERRIDE_FORCE_SNOOP |
				   7 << GLOBAL2_PRIO_OVERRIDE_SNOOP_SHIFT |
				   GLOBAL2_PRIO_OVERRIDE_FORCE_ARP |
				   7 << GLOBAL2_PRIO_OVERRIDE_ARP_SHIFT);
}

static int mv88e6131_setup(struct dsa_switch *ds)
{
	struct mv88e6xxx_priv_state *ps = ds_to_priv(ds);
	int ret;

	ds->slave_mii_bus = ps->mdio_bus;
	ps->dev = ds->dev;
	ps->ds = ds;

	ret = mv88e6xxx_switch_reset(ps, false);
	if (ret < 0)
		return ret;

	ret = mv88e6131_setup_global(ds);
	if (ret < 0)
		return ret;

	return mv88e6xxx_setup_ports(ds);
}

struct dsa_switch_driver mv88e6131_switch_driver = {
	.tag_protocol		= DSA_TAG_PROTO_DSA,
	.probe			= mv88e6131_drv_probe,
	.setup			= mv88e6131_setup,
	.set_addr		= mv88e6xxx_set_addr_direct,
	.get_strings		= mv88e6xxx_get_strings,
	.get_ethtool_stats	= mv88e6xxx_get_ethtool_stats,
	.get_sset_count		= mv88e6xxx_get_sset_count,
	.adjust_link		= mv88e6xxx_adjust_link,
};

static int mv88e6131_probe(struct mdio_device *mdiodev)
{
	return mv88e6xxx_probe(mdiodev, &mv88e6131_switch_driver,
			       mv88e6131_table, ARRAY_SIZE(mv88e6131_table));
}

static const struct of_device_id mv88e6131_of_match[] = {
	{ .compatible = "marvell,mv88e6131" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mv88e6131_of_match);

static struct mdio_driver mv88e6131_driver = {
	.probe	= mv88e6131_probe,
	.remove = mv88e6xxx_remove,
	.mdiodrv.driver = {
		.name = "mv88e6131",
		.of_match_table = mv88e6131_of_match,
	},
};

mv88e6xxx_module_driver(mv88e6131_driver, mv88e6131_switch_driver);

MODULE_DESCRIPTION("Driver for Marvell 6131 family ethernet switch chips");
MODULE_LICENSE("GPL");
