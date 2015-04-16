/*
 * Copyright(2) 2015 Andrew Lunn <andrew@lunn.ch>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <net/bonding.h>

struct bond_priv {
	struct net_device *bond_dev;
};

static int bond_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct net_device *slave_dev;
	struct device_node *slave;
	struct bond_priv *priv;
	int i = 0;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->bond_dev = bond_create(&init_net, NULL);
	if (IS_ERR(priv->bond_dev))
		return PTR_ERR(priv->bond_dev);

	priv->bond_dev->dev.of_node = np;

	rtnl_lock();

	slave = of_parse_phandle(np, "slaves", i);
	while ((slave = of_parse_phandle(np, "slaves", i))) {
		slave_dev = of_find_net_device_by_node(slave);
		of_node_put(slave);
		if (!slave_dev)
			goto defer;
		ret = bond_enslave(priv->bond_dev, slave_dev);
		if (ret)
			goto out;
		i++;
	}
	rtnl_unlock();

	platform_set_drvdata(pdev, priv);

	return 0;

defer:
	ret = -EPROBE_DEFER;
out:
	unregister_netdevice(priv->bond_dev);
	rtnl_unlock();

	return ret;
}

static int bond_remove(struct platform_device *pdev)
{
	struct bond_priv *priv = platform_get_drvdata(pdev);

	unregister_netdevice(priv->bond_dev);
	return 0;
}

static const struct of_device_id bond_match[] = {
	{ .compatible = "linux,bond", },
	{ },
};

MODULE_DEVICE_TABLE(of, bond_match);

static struct platform_driver bind_driver = {
	.probe = bond_probe,
	.remove = bond_remove,
	.driver = {
		.name = "bond",
		.of_match_table = bond_match,
	},
};

module_platform_driver(bind_driver);

MODULE_DESCRIPTION("Device tree binding for bonding");
MODULE_AUTHOR("Andrew Lunn <andrew@lunn.ch>");
MODULE_LICENSE("GPL");
