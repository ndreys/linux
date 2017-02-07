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
#include <linux/platform_device.h>
#include <linux/zii-pic.h>
#include <linux/watchdog.h>
#include <linux/nvmem-consumer.h>
#include <linux/slab.h>

#define DEFAULT_TIMEOUT		60

struct zii_pic_wdt {
	struct watchdog_device	wdt;
	struct zii_pic		*zp;
};

#define CMD_SET_WDT	0xA1
#define RSP_SET_WDT	0xC1

#define OCMD_SET_WDT	0x1C
#define ORSP_SET_WDT	0x5C

static int zii_pic_wdt_set(struct zii_pic_wdt *zpw, bool enable)
{
	if (zpw->zp->hw_id >= ZII_PIC_HW_ID_RDU1) {
		u8 cmd[2];
		cmd[0] = enable ? 1 : 0;
		cmd[1] = enable ? zpw->wdt.timeout : 0;
		return zii_pic_exec(zpw->zp, CMD_SET_WDT, cmd, sizeof(cmd),
				RSP_SET_WDT, NULL, 0);
	} else {
		u8 cmd[3];
		cmd[0] = 0;
		cmd[1] = enable ? 1 : 0;
		cmd[2] = enable ? zpw->wdt.timeout : 0;
		return zii_pic_exec(zpw->zp, OCMD_SET_WDT, cmd, sizeof(cmd),
				ORSP_SET_WDT, NULL, 0);
	}
}

#define CMD_PET_WDT	0xA2
#define RSP_PET_WDT	0xC2

#define OCMD_PET_WDT	0x1D
#define ORSP_PET_WDT	0x5D

static int zii_pic_wdt_pet(struct zii_pic_wdt *zpw)
{
	return zii_pic_exec(zpw->zp,
			zii_pic_code(zpw->zp, CMD_PET_WDT, OCMD_PET_WDT),
			NULL, 0,
			zii_pic_code(zpw->zp, RSP_PET_WDT, ORSP_PET_WDT),
			NULL, 0);
}

static int zii_pic_wdt_start(struct watchdog_device *wdt)
{
	struct zii_pic_wdt *zpw = container_of(wdt, struct zii_pic_wdt, wdt);
	int ret;

	ret = zii_pic_wdt_set(zpw, true);
	if (!ret)
		set_bit(WDOG_HW_RUNNING, &wdt->status);
	else
		dev_warn(wdt->parent, "start op failed, status unknown!\n");

	return ret ? -EIO : 0;
}

static int zii_pic_wdt_stop(struct watchdog_device *wdt)
{
	struct zii_pic_wdt *zpw = container_of(wdt, struct zii_pic_wdt, wdt);
	int ret;

	ret = zii_pic_wdt_set(zpw, false);
	if (!ret)
		clear_bit(WDOG_HW_RUNNING, &wdt->status);
	else
		dev_warn(wdt->parent, "stop op failed, status unknown!\n");

	return ret ? -EIO : 0;
}

static int zii_pic_wdt_ping(struct watchdog_device *wdt)
{
	struct zii_pic_wdt *zpw = container_of(wdt, struct zii_pic_wdt, wdt);

	return zii_pic_wdt_pet(zpw);
}

static int zii_pic_wdt_set_timeout(struct watchdog_device *wdt,
				   unsigned int timeout)
{
	struct zii_pic_wdt *zpw = container_of(wdt, struct zii_pic_wdt, wdt);

	zpw->wdt.timeout = timeout;

	if (test_bit(WDOG_HW_RUNNING, &wdt->status))
		return zii_pic_wdt_set(zpw, true);
	else
		return 0;
}

static const struct watchdog_info zii_pic_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "ZII PIC Watchdog",
};

static const struct watchdog_ops zii_pic_wdt_ops = {
	.owner = THIS_MODULE,
	.start = zii_pic_wdt_start,
	.stop = zii_pic_wdt_stop,
	.ping = zii_pic_wdt_ping,
	.set_timeout = zii_pic_wdt_set_timeout,
};

static const struct of_device_id zii_pic_wdt_of_match[] = {
	{ .compatible = "zii,pic-watchdog" },
	{}
};

static int zii_pic_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zii_pic *zp = zii_pic_parent(dev);
	struct zii_pic_wdt *zpw;
	struct nvmem_cell *cell;

	if (!zp)
		return -EINVAL;

	zpw = devm_kzalloc(dev, sizeof(*zpw), GFP_KERNEL);
	if (!zpw)
		return -ENOMEM;

	zpw->zp = zp;
	dev_set_drvdata(dev, zpw);

	zpw->wdt.parent = dev;
	zpw->wdt.info = &zii_pic_wdt_info;
	zpw->wdt.ops = &zii_pic_wdt_ops;
	if (zp->hw_id >= ZII_PIC_HW_ID_RDU1) {
		zpw->wdt.min_timeout = 60;
		zpw->wdt.max_timeout = 180;
	} else {
		zpw->wdt.min_timeout = 1;
		zpw->wdt.max_timeout = 255;
	}
	zpw->wdt.status = WATCHDOG_NOWAYOUT_INIT_STATUS;

	cell = nvmem_cell_get(dev, "wdt_timeout");
	if (!IS_ERR(cell)) {
		void *value;
		size_t len;

		value = nvmem_cell_read(cell, &len);
		if (!IS_ERR(value)) {
			if (len == 1)
				zpw->wdt.timeout = *(u8*)value;
			else if (len == 2)
				zpw->wdt.timeout = *(u16*)value;
			kfree(value);
		}
		nvmem_cell_put(cell);
	}
	if (zpw->wdt.timeout < zpw->wdt.min_timeout ||
	    zpw->wdt.timeout > zpw->wdt.max_timeout)
		zpw->wdt.timeout = DEFAULT_TIMEOUT;

	/* We don't know if watchdog is running now. To be sure, let's start
	 * it and depend on watchdog core to ping it */
	zpw->wdt.max_hw_heartbeat_ms = zpw->wdt.max_timeout * 1000;
	zii_pic_wdt_start(&zpw->wdt);

	return watchdog_register_device(&zpw->wdt);
}

static int zii_pic_wdt_remove(struct platform_device *pdev)
{
	struct zii_pic_wdt *zpw = dev_get_drvdata(&pdev->dev);

	watchdog_unregister_device(&zpw->wdt);

	return 0;
}

static struct platform_driver zii_pic_wdt_driver = {
	.probe = zii_pic_wdt_probe,
	.remove = zii_pic_wdt_remove,
	.driver = {
		.name = ZII_PIC_NAME_WATCHDOG,
		.of_match_table = zii_pic_wdt_of_match,
	},
};

module_platform_driver(zii_pic_wdt_driver);

MODULE_DEVICE_TABLE(of, zii_pic_wdt_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC Watchdog driver");
MODULE_ALIAS("platform:zii-pic-watchdog");
