/*
 *  zii-pic-wdt.c - Watchdog driver on Zodiac Inflight Innovation PIC
 *
 * Copyright (C) 2017 Nikita Yushchenko <nikita.yoush@cogentembedded.com>
 *
 * based on work by Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

/* #define DEBUG */

/* FIXME: will have to communicate with watchdog if/when porting fw update */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/zii-pic.h>
#include <linux/watchdog.h>
#include <linux/nvmem-consumer.h>
#include <linux/slab.h>

#define DEFAULT_TIMEOUT		60

struct zii_pic_wdt;

struct zii_pic_wdt_variant {
	unsigned int max_timeout;
	unsigned int min_timeout;

	int (*configure) (struct watchdog_device *);
};

struct zii_pic_wdt {
	struct watchdog_device	wdt;
	struct zii_pic		*zp;
	const struct zii_pic_wdt_variant *variant;
};

static struct zii_pic_wdt *to_zii_pic_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct zii_pic_wdt, wdt);
}

static int zii_pic_wdt_exec(struct watchdog_device *wdd,
			    void *data,  size_t data_size)
{
	struct zii_pic_wdt *pic_wdd = to_zii_pic_wdt(wdd);
	return zii_pic_exec(pic_wdd->zp, data, data_size, NULL, 0);
}

static int zii_pic_wdt_legacy_configure(struct watchdog_device *wdd)
{
	const bool enable = watchdog_hw_running(wdd);
	u8 cmd[] = {
		[0] = ZII_PIC_CMD_SW_WDT,
		[1] = 0,
		[2] = 0,
		[3] = !!enable,
		[4] = enable ? wdd->timeout : 0,
	};
	return zii_pic_wdt_exec(wdd, cmd, sizeof(cmd));
}

static int zii_pic_wdt_rdu_configure(struct watchdog_device *wdd)
{
	u8 cmd[] = {
		[0] = ZII_PIC_CMD_SW_WDT,
		[1] = 0,
		[2] = watchdog_hw_running(wdd),
		[3] = (u8) wdd->timeout,
		[4] = (u8) (wdd->timeout >> 8),
	};
	return zii_pic_wdt_exec(wdd, cmd, sizeof(cmd));
}

static int zii_pic_wdt_configure(struct watchdog_device *wdd)
{
	return to_zii_pic_wdt(wdd)->variant->configure(wdd);
}

static int zii_pic_wdt_start(struct watchdog_device *wdd)
{
	set_bit(WDOG_HW_RUNNING, &wdd->status);
	return zii_pic_wdt_configure(wdd);
}

static int zii_pic_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int timeout)
{
	wdd->timeout = timeout;
	return zii_pic_wdt_configure(wdd);
}

static int zii_pic_wdt_ping(struct watchdog_device *wdt)
{
	u8 cmd[] = {
		[0] = ZII_PIC_CMD_PET_WDT,
		[1] = 0,
	};

	return zii_pic_wdt_exec(wdt, cmd, sizeof(cmd));
}

static const struct watchdog_info zii_pic_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "ZII PIC Watchdog",
};

static const struct watchdog_ops zii_pic_wdt_ops = {
	.owner = THIS_MODULE,
	.start = zii_pic_wdt_start,
	.stop = zii_pic_wdt_configure,
	.ping = zii_pic_wdt_ping,
	.set_timeout = zii_pic_wdt_set_timeout,
};

static const struct of_device_id zii_pic_wdt_of_match[] = {
	{ .compatible = "zii,pic-watchdog" },
	{}
};

const static struct zii_pic_wdt_variant zii_pic_wdt_legacy = {
	.max_timeout = 255,
	.min_timeout = 1,
	.configure = zii_pic_wdt_legacy_configure,
};

const static struct zii_pic_wdt_variant zii_pic_wdt_rdu = {
	.max_timeout = 180,
	.min_timeout = 60,
	.configure = zii_pic_wdt_rdu_configure,
};

const static struct of_device_id zii_pic_wdt_variants[] = {
	{ .compatible = COMPATIBLE_ZII_PIC_NIU,  .data = &zii_pic_wdt_legacy },
	{ .compatible = COMPATIBLE_ZII_PIC_MEZZ, .data = &zii_pic_wdt_legacy },
	{ .compatible = COMPATIBLE_ZII_PIC_ESB,	 .data = &zii_pic_wdt_legacy },
	{ .compatible = COMPATIBLE_ZII_PIC_RDU1, .data = &zii_pic_wdt_rdu    },
	{ .compatible = COMPATIBLE_ZII_PIC_RDU2, .data = &zii_pic_wdt_rdu    },
	{ /* sentinel */ }
};

static int zii_pic_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zii_pic *zp = zii_pic_parent(dev);
	struct zii_pic_wdt *zpw;
	struct nvmem_cell *cell;
	const struct of_device_id *id;
	__le16 timeout = 0;

	if (!zp)
		return -EINVAL;

	zpw = devm_kzalloc(dev, sizeof(*zpw), GFP_KERNEL);
	if (!zpw)
		return -ENOMEM;

	zpw->zp         = zp;
	zpw->wdt.parent = dev;
	zpw->wdt.info   = &zii_pic_wdt_info;
	zpw->wdt.ops    = &zii_pic_wdt_ops;

	id = of_match_device(zii_pic_wdt_variants, dev->parent);
	if (WARN_ON(!id))
		return -ENODEV;

	zpw->variant         = id->data;
	zpw->wdt.min_timeout = zpw->variant->min_timeout;
	zpw->wdt.max_timeout = zpw->variant->max_timeout;
	zpw->wdt.status      = WATCHDOG_NOWAYOUT_INIT_STATUS;
	zpw->wdt.timeout     = DEFAULT_TIMEOUT;

	cell = nvmem_cell_get(dev, "wdt_timeout");
	if (!IS_ERR(cell)) {
		size_t len;
		void *value = nvmem_cell_read(cell, &len);

		if (!IS_ERR(value)) {
			memcpy(&timeout, value, min(len, sizeof(timeout)));
			kfree(value);
		}
		nvmem_cell_put(cell);
	}
	watchdog_init_timeout(&zpw->wdt, le16_to_cpu(timeout), dev);

	/* We don't know if watchdog is running now. To be sure, let's start
	 * it and depend on watchdog core to ping it */
	zpw->wdt.max_hw_heartbeat_ms = zpw->wdt.max_timeout * 1000;
	zii_pic_wdt_start(&zpw->wdt);

	return devm_watchdog_register_device(dev, &zpw->wdt);
}

static struct platform_driver zii_pic_wdt_driver = {
	.probe = zii_pic_wdt_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = zii_pic_wdt_of_match,
	},
};

module_platform_driver(zii_pic_wdt_driver);

MODULE_DEVICE_TABLE(of, zii_pic_wdt_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC Watchdog driver");
MODULE_ALIAS("platform:zii-pic-watchdog");
