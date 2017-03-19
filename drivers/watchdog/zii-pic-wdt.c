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

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/zii-pic.h>
#include <linux/watchdog.h>
#include <linux/nvmem-consumer.h>
#include <linux/reboot.h>
#include <linux/slab.h>

enum {
	ZII_PIC_RESET_BYTE = 1,
	ZII_PIC_RESET_REASON_NORMAL = 0,
	ZII_PIC_RESET_DELAY_MS = 500,
};

struct zii_pic_wdt_variant {
	unsigned int max_timeout;
	unsigned int min_timeout;

	int (*configure) (struct watchdog_device *);
	int (*restart)   (struct watchdog_device *);
};

struct zii_pic_wdt {
	struct watchdog_device wdd;
	struct zii_pic *pic;
	const struct zii_pic_wdt_variant *variant;
	struct notifier_block reboot_notifier;
};

static struct zii_pic_wdt *to_zii_pic_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct zii_pic_wdt, wdd);
}

static int zii_pic_wdt_exec(struct watchdog_device *wdd, void *data,
			    size_t data_size)
{
	return zii_pic_exec(to_zii_pic_wdt(wdd)->pic, data, data_size, NULL, 0);
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

static int zii_pic_wdt_legacy_restart(struct watchdog_device *wdd)
{
	u8 cmd[] = {
		[0] = ZII_PIC_CMD_RESET,
		[1] = 0,
		[2] = ZII_PIC_RESET_BYTE
	};
	return zii_pic_wdt_exec(wdd, cmd, sizeof(cmd));
}

static int zii_pic_wdt_rdu_restart(struct watchdog_device *wdd)
{
	u8 cmd[] = {
		[0] = ZII_PIC_CMD_RESET,
		[1] = 0,
		[2] = ZII_PIC_RESET_BYTE,
		[3] = ZII_PIC_RESET_REASON_NORMAL
	};
	return zii_pic_wdt_exec(wdd, cmd, sizeof(cmd));
}

static int zii_pic_wdt_reboot_notifier(struct notifier_block *nb,
				       unsigned long action, void *data)
{
	/*
	 * Restart handler is called in atomic context which means we
	 * can't commuicate to PIC via UART. Luckily for use PIC will
	 * wait 500ms before actually reseting us, so we ask it to do
	 * so here and let the rest of the system go on wrapping
	 * things up.
	 */
	if (action == SYS_DOWN || action == SYS_HALT) {
		struct zii_pic_wdt *pic_wd =
			container_of(nb, struct zii_pic_wdt, reboot_notifier);

		const int ret = pic_wd->variant->restart(&pic_wd->wdd);

		if (ret < 0)
			dev_err(pic_wd->wdd.parent,
				"Failed to issue restart command (%d)", ret);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int zii_pic_wdt_restart(struct watchdog_device *wdd,
			       unsigned long action, void *data)
{
	/*
	 * The actual work was done by reboot notifier above. PIC
	 * firmware waits 500 ms before issuing reset, so let's hang
	 * here for one second and hopefuly we'd never reach that
	 * return statement
	 */
	mdelay(ZII_PIC_RESET_DELAY_MS);
	return -EIO;
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

static int zii_pic_wdt_ping(struct watchdog_device *wdd)
{
	u8 cmd[] = {
		[0] = ZII_PIC_CMD_PET_WDT,
		[1] = 0,
	};

	return zii_pic_wdt_exec(wdd, cmd, sizeof(cmd));
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
	.restart = zii_pic_wdt_restart,
};

static const struct of_device_id zii_pic_wdt_of_match[] = {
	{ .compatible = "zii,pic-watchdog" },
	{}
};

const static struct zii_pic_wdt_variant zii_pic_wdt_legacy = {
	.max_timeout = 255,
	.min_timeout = 1,
	.configure = zii_pic_wdt_legacy_configure,
	.restart   = zii_pic_wdt_legacy_restart,
};

const static struct zii_pic_wdt_variant zii_pic_wdt_rdu = {
	.max_timeout = 180,
	.min_timeout = 60,
	.configure = zii_pic_wdt_rdu_configure,
	.restart   = zii_pic_wdt_rdu_restart,
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
	const struct of_device_id *id;
	struct zii_pic_wdt *pic_wd;
	struct nvmem_cell *cell;
	__le16 timeout = 0;
	int ret;

	pic_wd = devm_kzalloc(dev, sizeof(*pic_wd), GFP_KERNEL);
	if (!pic_wd)
		return -ENOMEM;

	pic_wd->pic        = dev_get_drvdata(dev->parent);
	pic_wd->wdd.parent = dev;
	pic_wd->wdd.info   = &zii_pic_wdt_info;
	pic_wd->wdd.ops    = &zii_pic_wdt_ops;

	if (WARN_ON(!pic_wd->pic))
		return -ENODEV;

	id = of_match_device(zii_pic_wdt_variants, dev->parent);
	if (WARN_ON(!id))
		return -ENODEV;

	pic_wd->variant         = id->data;
	pic_wd->wdd.min_timeout = pic_wd->variant->min_timeout;
	pic_wd->wdd.max_timeout = pic_wd->variant->max_timeout;
	pic_wd->wdd.status      = WATCHDOG_NOWAYOUT_INIT_STATUS;
	pic_wd->wdd.timeout     = 60;

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
	watchdog_init_timeout(&pic_wd->wdd, le16_to_cpu(timeout), dev);
	watchdog_set_restart_priority(&pic_wd->wdd, 255);

	pic_wd->reboot_notifier.notifier_call = zii_pic_wdt_reboot_notifier;
	ret = devm_register_reboot_notifier(dev, &pic_wd->reboot_notifier);
	if (ret) {
		dev_err(dev, "Failed to register reboot notifier\n");
		return ret;
	}

	/* We don't know if watchdog is running now. To be sure, let's start
	 * it and depend on watchdog core to ping it */
	pic_wd->wdd.max_hw_heartbeat_ms = pic_wd->wdd.max_timeout * 1000;
	zii_pic_wdt_start(&pic_wd->wdd);

	return devm_watchdog_register_device(dev, &pic_wd->wdd);
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
