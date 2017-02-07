/*
 *  zii-pic-pwrbutton.c - driver for power button on Zodiac Inflight Innovation
 *  PIC.
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

#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/zii-pic.h>

#define EVT_BUTTON_PRESS	0xE0
#define RSP_BUTTON_PRESS	0xE1

static void zii_pic_pwrbutton_event(void *context,
		u8 code, const u8 *data, u8 data_size)
{
	struct input_dev *idev = context;

	if (data_size < 1)
		return;

	input_report_key(idev, KEY_POWER, data[0]);
	input_sync(idev);
}

static const struct of_device_id zii_pic_pwrbutton_of_match[] = {
	{ .compatible = "zii,pic-pwrbutton" },
	{}
};

static int zii_pic_pwrbutton_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zii_pic *zp = zii_pic_parent(dev);
	struct input_dev *idev;
	int ret;

	if (!zp)
		return -EINVAL;
	if (zp->hw_id < ZII_PIC_HW_ID_RDU1)
		return -ENODEV;

	idev = devm_input_allocate_device(&pdev->dev);
	if (!idev)
		return -ENOMEM;

	idev->name = ZII_PIC_NAME_PWRBUTTON;
	idev->dev.parent = dev;

	input_set_capability(idev, EV_KEY, KEY_POWER);

	ret = input_register_device(idev);
	if (ret)
		return ret;

	return zii_pic_set_event_handler(zp, EVT_BUTTON_PRESS, RSP_BUTTON_PRESS,
			zii_pic_pwrbutton_event, idev);
}

static int zii_pic_pwrbutton_remove(struct platform_device *pdev)
{
	struct zii_pic *zp = zii_pic_parent(&pdev->dev);

	zii_pic_cleanup_event_handler(zp, EVT_BUTTON_PRESS);
	return 0;
}

static struct platform_driver zii_pic_pwrbutton_driver = {
	.probe		= zii_pic_pwrbutton_probe,
	.remove		= zii_pic_pwrbutton_remove,
	.driver		= {
		.name	= ZII_PIC_NAME_PWRBUTTON,
		.of_match_table = zii_pic_pwrbutton_of_match,
	},
};
module_platform_driver(zii_pic_pwrbutton_driver);

MODULE_DEVICE_TABLE(of, zii_pic_pwrbutton_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC Power Button driver");
MODULE_ALIAS("platform:zii-pic-pwrbutton");
