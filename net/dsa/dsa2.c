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

static int _dsa_register_switch(struct dsa_switch *ds, struct device_node *np)
{
	struct dsa_switch_tree *dst;
	u32 tree, index;
	int err;

	err = dsa_parse_member(np, &tree, &index);
	if (err)
		return err;

	dst = dsa_get_dst(tree);
	if (!dst) {
		dst = dsa_add_dst(tree);
		if (!dst)
			return -ENOMEM;
	}

	if (dst->ds[index]) {
		dsa_put_dst(dst);
		return -EBUSY;
	}

	ds->dst = dst;
	ds->index = index;
	dsa_dst_add_ds(dst, ds, index);

	dsa_put_dst(dst);

	return 0;
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

