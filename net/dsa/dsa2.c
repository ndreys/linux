/*
 * net/dsa/dsa2.c - Hardware switch handling, binding version 2
 * Copyright (c) 2008-2009 Marvell Semiconductor
 * Copyright (c) 2013 Florian Fainelli <florian@openwrt.org>
 * Copyright (c) 2016 Andrew Lunn <andrew@lunn.ch>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/rtnetlink.h>
#include <net/dsa.h>
#include <linux/of.h>
#include "dsa_priv.h"

static LIST_HEAD(dsa_switch_trees);

static struct dsa_switch_tree *dsa_get_dst(u32 tree)
{
	struct dsa_switch_tree *dst;

	list_for_each_entry(dst, &dsa_switch_trees, list)
		if (dst->tree == tree)
			return dst;
	return NULL;
}

static void dsa_free_dst(struct kref *ref)
{
	struct dsa_switch_tree *dst = container_of(ref, struct dsa_switch_tree,
						   refcount);

	list_del(&dst->list);
	kfree(dst);
}

static void dsa_put_dst(struct dsa_switch_tree *dst)
{
	kref_put(&dst->refcount, dsa_free_dst);
}

static struct dsa_switch_tree *dsa_add_dst(u32 tree)
{
	struct dsa_switch_tree *dst;

	dst = kzalloc(sizeof(*dst), GFP_KERNEL);
	if (!dst)
		return NULL;
	dst->tree = tree;
	INIT_LIST_HEAD(&dst->list);
	list_add_tail(&dsa_switch_trees, &dst->list);
	kref_init(&dst->refcount);

	return dst;
}

static void dsa_dst_add_ds(struct dsa_switch_tree *dst,
			   struct dsa_switch *ds, u32 index)
{
	kref_get(&dst->refcount);
	dst->ds[index] = ds;
}

static void dsa_dst_del_ds(struct dsa_switch_tree *dst,
			   struct dsa_switch *ds, u32 index)
{
	dst->ds[index] = NULL;
	kref_put(&dst->refcount, dsa_free_dst);
}

static bool dsa_port_is_dsa(struct device_node *port)
{
	const char *name;

	name = of_get_property(port, "label", NULL);
	if (!name)
		return false;

	if (!strcmp(name, "dsa"))
		return true;

	return false;
}

static bool dsa_ds_find_port(struct dsa_switch *ds,
			     struct device_node *port)
{
	u32 index;

	for (index = 0; index < DSA_MAX_PORTS; index++)
		if (ds->ports[index].dn == port)
			return true;
	return false;
}

static struct dsa_switch *dsa_dst_find_port(struct dsa_switch_tree *dst,
					    struct device_node *port)
{
	struct dsa_switch *ds;
	u32 index;

	for (index = 0; index < DSA_MAX_SWITCHES; index++) {
		ds = dst->ds[index];
		if (!ds)
			continue;

		if (dsa_ds_find_port(ds, port))
			return ds;
	}

	return NULL;
}

static int dsa_port_complete(struct dsa_switch_tree *dst,
			     struct dsa_switch *src_ds,
			     struct device_node *port,
			     u32 src_port)
{
	struct device_node *link;
	int index;
	struct dsa_switch *dst_ds;

	for (index = 0;; index++) {
		link = of_parse_phandle(port, "link", index);
		if (!link)
			break;

		dst_ds = dsa_dst_find_port(dst, link);
		of_node_put(link);

		if (!dst_ds)
			return 1;

		src_ds->rtable[dst_ds->index] = src_port;

		pr_debug("DSA: port %s to %s complete\n",
			 port->full_name, link->full_name);
	}

	pr_debug("DSA: port %s complete\n", port->full_name);

	return 0;
}

/* A switch is complete if all the DSA ports phandles point to ports
 * known in the tree. A return value of 1 means the tree is not
 * complete. This is not an error condition. A value of 0 is
 * success.
 */
static int dsa_ds_complete(struct dsa_switch_tree *dst, struct dsa_switch *ds)
{
	struct device_node *port;
	u32 index;
	int err;

	for (index = 0; index < DSA_MAX_PORTS; index++) {
		port = ds->ports[index].dn;
		if (!port)
			continue;

		if (!dsa_port_is_dsa(port))
			continue;

		err = dsa_port_complete(dst, ds, port, index);
		if (err != 0)
			return err;
	}

	pr_debug("DSA: switch %d %d complete\n", dst->tree, ds->index);

	return 0;
}

/* A tree is complete if all the DSA ports phandles point to ports
 * known in the tree. A return value of 1 means the tree is not
 * complete. This is not an error condition. A value of 0 is
 * success.
 */
static int dsa_dst_complete(struct dsa_switch_tree *dst)
{
	struct dsa_switch *ds;
	u32 index;
	int err;

	for (index = 0; index < DSA_MAX_SWITCHES; index++) {
		ds = dst->ds[index];
		if (!ds)
			continue;

		err = dsa_ds_complete(dst, ds);
		if (err != 0)
			return err;
	}

	pr_debug("DSA: tree %d complete\n", dst->tree);

	return 0;
}

static int dsa_parse_ports_dn(struct device_node *ports, struct dsa_switch *ds)
{
	struct device_node *port;
	int err;
	u32 reg;

	for_each_available_child_of_node(ports, port) {
		err = of_property_read_u32(port, "reg", &reg);
		if (err)
			return err;

		if (reg >= DSA_MAX_PORTS)
			return -EINVAL;

		ds->ports[reg].dn = port;
	}

	return 0;
}

static int dsa_parse_member(struct device_node *np, u32 *tree, u32 *index)
{
	int err;

	err = of_property_read_u32_index(np, "dsa,member", 0, tree);
	if (err)
		return err;

	err = of_property_read_u32_index(np, "dsa,member", 1, index);
	if (err)
		return err;

	if (*index >= DSA_MAX_SWITCHES)
		return -EINVAL;

	return 0;
}

static struct device_node *dsa_get_ports(struct dsa_switch *ds,
					 struct device_node *np)
{
	struct device_node *ports;

	ports = of_get_child_by_name(np, "ports");
        if (!ports) {
                dev_err(ds->dev, "no ports child node found\n");
                return ERR_PTR(-EINVAL);
        }

	return ports;
}

static int _dsa_register_switch(struct dsa_switch *ds, struct device_node *np)
{
	struct device_node *ports = dsa_get_ports(ds, np);
	struct dsa_switch_tree *dst;
	u32 tree, index;
	int err;

	err = dsa_parse_member(np, &tree, &index);
	if (err)
		return err;

	if (IS_ERR(ports))
		return PTR_ERR(ports);

	err = dsa_parse_ports_dn(ports, ds);
	if (err)
		return err;

	dst = dsa_get_dst(tree);
	if (!dst) {
		dst = dsa_add_dst(tree);
		if (!dst)
			return -ENOMEM;
	}

	if (dst->ds[index]) {
		err = -EBUSY;
		goto out;
	}

	ds->dst = dst;
	ds->index = index;
	dsa_dst_add_ds(dst, ds, index);

	err = dsa_dst_complete(dst);
	if (err < 0)
		goto out;
	err = 0;

out:
	dsa_put_dst(dst);

	return err;
}

int dsa_register_switch(struct dsa_switch *ds, struct device_node *np)
{
	int err;

	rtnl_lock();
	err = _dsa_register_switch(ds, np);
	rtnl_unlock();

	return err;
}

void _dsa_unregister_switch(struct dsa_switch *ds)
{
	dsa_dst_del_ds(ds->dst, ds, ds->index);
}

void dsa_unregister_switch(struct dsa_switch *ds)
{
	rtnl_lock();
	_dsa_unregister_switch(ds);
	rtnl_unlock();
}

