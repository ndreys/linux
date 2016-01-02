#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mdio.h>

static int mdio_nop_probe(struct mdio_device *mdiodev)
{
	dev_info(&mdiodev->dev, "mdio_nop_probe for address %d\n",
		 mdiodev->addr);

	return 0;
}

static void mdio_nop_remove(struct mdio_device *mdiodev)
{
	dev_info(&mdiodev->dev, "mdio_nop_remove for address %d\n",
		 mdiodev->addr);
}

static const struct of_device_id mdio_nop_ids[] = {
	{ .compatible = "linux,mdio-nop" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mdio_nop_ids);

struct mdio_driver mdio_nop_drv = {
	.probe = mdio_nop_probe,
	.remove = mdio_nop_remove,
	.mdiodrv = {
		.driver	= {
			.name = "mdio-nop",
			.of_match_table = mdio_nop_ids,
		},
	},
};

mdio_module_driver(mdio_nop_drv);
