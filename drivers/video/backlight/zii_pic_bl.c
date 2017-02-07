/*
 * zii_pic_bl.c - LCD Backlight driver for Zodiac Inflight Innovations
 * PIC MCU that is connected via dedicated UART port
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/zii-pic.h>

#define CMD_SET_BACKLIGHT	0xA6
#define RSP_SET_BACKLIGHT	0xC6

static int zii_pic_bl_update_status(struct backlight_device *bd)
{
	struct zii_pic *zp = dev_get_drvdata(&bd->dev);
	int intensity = bd->props.brightness;
	u8 cmd[3];

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;

	cmd[0] = intensity ? 0x80 | intensity : 0;
	cmd[1] = 0;
	cmd[2] = 0;

	return zii_pic_exec(zp, CMD_SET_BACKLIGHT, cmd, sizeof(cmd),
			RSP_SET_BACKLIGHT, NULL, 0);
}

static const struct backlight_ops zii_pic_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status  = zii_pic_bl_update_status,
};

static struct backlight_properties zii_pic_bl_props = {
	.type = BACKLIGHT_FIRMWARE,
	.max_brightness = 100,
	.brightness = 50,
};

static const struct of_device_id zii_pic_bl_of_match[] = {
	{ .compatible = "zii,pic-backlight" },
	{}
};

static int zii_pic_backlight_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zii_pic *zp = zii_pic_parent(dev);
	struct backlight_device *bd;

	if (!zp)
		return -EINVAL;

	bd = devm_backlight_device_register(dev, ZII_PIC_NAME_BACKLIGHT,
					    dev, zp, &zii_pic_bl_ops,
					    &zii_pic_bl_props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);
	backlight_update_status(bd);

	return 0;
}

static struct platform_driver zii_pic_backlight_driver = {
	.probe		= zii_pic_backlight_probe,
	.driver		= {
		.name	= ZII_PIC_NAME_BACKLIGHT,
		.of_match_table = zii_pic_bl_of_match,
	},
};
module_platform_driver(zii_pic_backlight_driver);

MODULE_DEVICE_TABLE(of, zii_pic_bl_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC LCD Backlight driver");
MODULE_ALIAS("platform:zii-pic-backlight");
