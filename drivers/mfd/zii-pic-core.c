/*
 * zii-pic.c - Multifunction core driver for Zodiac Inflight Innovations
 * PIC MCU that is connected via dedicated UART port
 *
 * Copyright (C) 2017 Nikita Yushchenko <nikita.yoush@cogentembedded.com>
 *
 * Partially based on work by
 *   Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
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
#include <linux/serdev.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mfd/core.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/zii-pic.h>

#define ZII_PIC_DEFAULT_BAUD_RATE	57600

#define CMD_GET_FW_VERSION	0x20
#define OCMD_GET_FW_VERSION	0x11

static void zii_pic_get_fw_version(struct zii_pic *zp)
{
	u8 cmd[2] = { zii_pic_code(zp,
				   CMD_GET_FW_VERSION, OCMD_GET_FW_VERSION), 0};
	int ret;

	ret = zii_pic_exec(zp, cmd, sizeof(cmd),
			   &zp->fw_version, sizeof(zp->fw_version));
	if (ret) {
		dev_warn(&zp->serdev->dev, "failed to get fw version\n");
		memset(&zp->fw_version, 0, sizeof(zp->fw_version));
	}
}

#define CMD_GET_BL_VERSION	0x21
#define OCMD_GET_BL_VERSION	0x12

static void zii_pic_get_bl_version(struct zii_pic *zp)
{
	u8 cmd[2] = { zii_pic_code(zp,
				   CMD_GET_BL_VERSION, OCMD_GET_BL_VERSION), 0 };
	int ret;

	ret = zii_pic_exec(zp, cmd, sizeof(cmd),
			   &zp->bl_version, sizeof(zp->bl_version));
	if (ret) {
		dev_warn(&zp->serdev->dev,
				"failed to get bl version\n");
		memset(&zp->bl_version, 0, sizeof(zp->bl_version));
	}
}

#define CMD_GET_RESET_REASON	0xA8
#define OCMD_GET_RESET_REASON	0x1F

static void zii_pic_get_reset_reason(struct zii_pic *zp)
{
	u8 cmd[2] = { zii_pic_code(zp,
				   CMD_GET_RESET_REASON, OCMD_GET_RESET_REASON), 0 };
	int ret;

	ret = zii_pic_exec(zp, cmd, sizeof(cmd), &zp->reset_reason, 1);
	if (ret) {
		dev_warn(&zp->serdev->dev, "failed to get reset reason\n");
		zp->reset_reason = 0xFF;
	}
}

#define CMD_BOOT_SOURCE		0x26
#define OCMD_BOOT_SOURCE	0x14

static void zii_pic_get_boot_source(struct zii_pic *zp)
{
	u8 code = zii_pic_code(zp, CMD_BOOT_SOURCE, OCMD_BOOT_SOURCE);
	u8 cmd[] = {code, 0, 0, 0};
	int ret;

	ret = zii_pic_exec(zp, cmd, sizeof(cmd),
			   &zp->boot_source, 1);
	if (ret) {
		dev_warn(&zp->serdev->dev, "failed to get boot source\n");
		zp->boot_source = 0xFF;
	}
}

static int zii_pic_set_boot_source(struct zii_pic *zp, u8 boot_src)
{
	u8 code = zii_pic_code(zp, CMD_BOOT_SOURCE, OCMD_BOOT_SOURCE);
	u8 cmd[] = {code, 0, 1, boot_src};
	int ret;

	ret = zii_pic_exec(zp, cmd, sizeof(cmd), NULL, 0);
	if (ret)
		return ret;

	zii_pic_get_boot_source(zp);
	if (boot_src != zp->boot_source)
		return -EIO;

	return 0;
}


#define CMD_GET_STATUS		0xA0

static void zii_pic_get_status_rdu1(struct zii_pic *zp)
{
	struct {
		u8 bl[6];
		u8 fw[6];
		u8 _pad[23];
		u8 gs;
	} __packed reply;

	int ret;
	u8 cmd[] = {CMD_GET_STATUS, 0};

	ret = zii_pic_exec(zp, cmd, sizeof(cmd),
			   &reply, sizeof(reply));
	if (ret) {
		dev_warn(&zp->serdev->dev, "failed to read RDU1 status\n");
		memset(&zp->fw_version, 0, sizeof(zp->fw_version));
		memset(&zp->bl_version, 0, sizeof(zp->bl_version));
		zp->boot_source = 0xFF;
	} else {
		memcpy(&zp->fw_version, reply.fw, sizeof(reply.fw));
		memcpy(&zp->bl_version, reply.bl, sizeof(reply.bl));
		zp->boot_source = (reply.gs >> 2) & 0x03;
	}
}

static int zii_pic_set_boot_source_rdu1(struct zii_pic *zp, u8 boot_src)
{
	/* TODO: port old eeprom access code */
	return -EIO;
}

static int zii_pic_reboot_notifier(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct zii_pic *zp = container_of(nb, struct zii_pic, reboot_nb);

	if (action == SYS_RESTART) {
		zii_pic_prepare_for_reset(zp);
		return NOTIFY_OK;
	} else
		return NOTIFY_DONE;

}

#define CMD_PIC_RESET	0xA7
#define OCMD_PIC_RESET	0x1E

static int zii_pic_reset_handler(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct zii_pic *zp = container_of(nb, struct zii_pic, reset_nb);

	while (1) {

		if (zp->hw_id >= ZII_PIC_HW_ID_RDU1) {
			u8 cmd[] = { CMD_PIC_RESET, 1, 0 };
			zii_pic_exec_reset(zp, cmd, sizeof(cmd));
		} else {
			u8 cmd[] = { OCMD_PIC_RESET, 1 };
			zii_pic_exec_reset(zp, cmd, sizeof(cmd));
		}

		msleep(550);	/* PIC firmware waits 500 ms before reset */
		dev_emerg(&zp->serdev->dev, "rewset timed out, retrying\n");
	}

	return NOTIFY_OK;	/* unreachable but stops warning */
}

static void zii_pic_setup_reboot(struct zii_pic *zp)
{
	int ret;

	zp->reboot_nb.notifier_call = zii_pic_reboot_notifier;
	ret = register_reboot_notifier(&zp->reboot_nb);
	if (ret) {
		dev_warn(&zp->serdev->dev,
				"could not register reboot notifier\n");
		return;
	}

	zp->reset_nb.notifier_call = zii_pic_reset_handler;
	zp->reset_nb.priority = 255;
	ret = register_restart_handler(&zp->reset_nb);
	if (ret) {
		dev_warn(&zp->serdev->dev,
				"could not register restart handler\n");
		unregister_reboot_notifier(&zp->reboot_nb);
	}
}

static void zii_pic_cleanup_reboot(struct zii_pic *zp)
{
	unregister_restart_handler(&zp->reset_nb);
	unregister_reboot_notifier(&zp->reboot_nb);
}

static ssize_t zii_pic_show_version(char *buf, struct zii_pic_version *ver)
{
	return sprintf(buf, "%d.%d.%d.%c%c\n", ver->hw, ver->major, ver->minor,
			ver->letter_1 >= 0x20 && ver->letter_1 < 0x7f ?
				ver->letter_1 : ' ',
			ver->letter_2 >= 0x20 && ver->letter_2 < 0x7f ?
				ver->letter_2 : ' ');
}

static ssize_t zii_pic_show_fw_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct zii_pic *zp = dev_get_drvdata(dev);

	return zii_pic_show_version(buf, &zp->fw_version);
}

static ssize_t zii_pic_show_bl_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct zii_pic *zp = dev_get_drvdata(dev);

	return zii_pic_show_version(buf, &zp->bl_version);
}

static DEVICE_ATTR(part_number_firmware, S_IRUSR | S_IRGRP | S_IROTH,
		zii_pic_show_fw_version, NULL);
static DEVICE_ATTR(part_number_bootloader, S_IRUSR | S_IRGRP | S_IROTH,
		zii_pic_show_bl_version, NULL);

static ssize_t zii_pic_show_reset_reason(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct zii_pic *zp = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", zp->reset_reason);
}

static DEVICE_ATTR(reset_reason, S_IRUSR | S_IRGRP | S_IROTH,
		zii_pic_show_reset_reason, NULL);

static ssize_t zii_pic_show_boot_source(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct zii_pic *zp = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", zp->boot_source);
}

static ssize_t zii_pic_store_boot_source(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct zii_pic *zp = dev_get_drvdata(dev);
	u8 boot_src;
	int ret;

	ret = kstrtou8(buf, 0, &boot_src);
	if (ret)
		return ret;

	if (boot_src > 2)
		return -EINVAL;

	if (zp->hw_id == ZII_PIC_HW_ID_RDU1)
		ret = zii_pic_set_boot_source_rdu1(zp, boot_src);
	else
		ret = zii_pic_set_boot_source(zp, boot_src);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR(boot_source, S_IRUSR | S_IWUSR | S_IRGRP,
		zii_pic_show_boot_source, zii_pic_store_boot_source);

static struct attribute *zii_pic_dev_attrs[] = {
	&dev_attr_part_number_firmware.attr,
	&dev_attr_part_number_bootloader.attr,
	&dev_attr_reset_reason.attr,
	&dev_attr_boot_source.attr,
	NULL
};

static const struct attribute_group zii_pic_attr_group = {
	.attrs = zii_pic_dev_attrs,
};

#define CMD_COPPER_REV_RDU1	0x28

#define CMD_COPPER_REV_RDU2	0x2B

static ssize_t zii_pic_show_copper_rev(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct zii_pic *zp = dev_get_drvdata(dev);
	int ret;
	u8 cmd[2], res;

	if (zp->hw_id == ZII_PIC_HW_ID_RDU2) {
		cmd[0] = CMD_COPPER_REV_RDU2;
	} else if (zp->hw_id == ZII_PIC_HW_ID_RDU1) {
		cmd[0] = CMD_COPPER_REV_RDU1;
	} else
		return -ENOTSUPP;

	ret = zii_pic_exec(zp, cmd, sizeof(cmd), &res, sizeof(res));
	if (ret)
		return ret;

	return sprintf(buf, "%02x\n", res);
}

static DEVICE_ATTR(copper_rev, S_IRUSR | S_IRGRP | S_IROTH,
		zii_pic_show_copper_rev, NULL);


const static struct of_device_id zii_pic_dt_ids[] = {
	{ .compatible = "zii,pic-niu",
		.data = (const void *)ZII_PIC_HW_ID_NIU},
	{ .compatible = "zii,pic-mezz",
		.data = (const void *)ZII_PIC_HW_ID_MEZZ},
	{ .compatible = "zii,pic-esb",
		.data = (const void *)ZII_PIC_HW_ID_ESB},
	{ .compatible = "zii,pic-rdu1",
		.data = (const void *)ZII_PIC_HW_ID_RDU1},
	{ .compatible = "zii,pic-rdu2",
		.data = (const void *)ZII_PIC_HW_ID_RDU2},
	{}
};

static int zii_pic_probe(struct serdev_device *serdev)
{
	struct zii_pic *zp;
	const struct of_device_id *id;
	struct device *dev = &serdev->dev;
	u32 baud = ZII_PIC_DEFAULT_BAUD_RATE;
	int ret;

	id = of_match_device(zii_pic_dt_ids, dev);
	if (!id)
		return -ENODEV;

	zp = devm_kzalloc(dev, sizeof(*zp), GFP_KERNEL);
	if (!zp)
		return -ENOMEM;

	zp->serdev = serdev;
	zp->hw_id = (enum zii_pic_hw_id)id->data;
	dev_set_drvdata(dev, zp);

	of_property_read_u32(dev->of_node, "current-speed", &baud);

	ret = zii_pic_open(zp, baud);
	if (ret)
		return ret;

	if (zp->hw_id == ZII_PIC_HW_ID_RDU1)
		zii_pic_get_status_rdu1(zp);
	else {
		zii_pic_get_fw_version(zp);
		zii_pic_get_bl_version(zp);
		zii_pic_get_reset_reason(zp);
	}

	zii_pic_get_boot_source(zp);

	ret = sysfs_create_group(&dev->kobj, &zii_pic_attr_group);
	if (ret)
		goto err_create_group;

	if (zp->hw_id >= ZII_PIC_HW_ID_RDU1) {
		ret = device_create_file(dev, &dev_attr_copper_rev);
		if (ret)
			goto err_create_copper_attr;
	}

	zii_pic_setup_reboot(zp);

	return of_platform_default_populate(dev->of_node, NULL, dev);

err_create_copper_attr:
	sysfs_remove_group(&dev->kobj, &zii_pic_attr_group);
err_create_group:
	serdev_device_close(zp->serdev);
	return ret;
}

static void zii_pic_remove(struct serdev_device *serdev)
{
	struct zii_pic *zp = dev_get_drvdata(&serdev->dev);

	of_platform_depopulate(&serdev->dev);

	zii_pic_cleanup_reboot(zp);
	if (zp->hw_id >= ZII_PIC_HW_ID_RDU1)
		device_remove_file(&serdev->dev, &dev_attr_copper_rev);
	sysfs_remove_group(&serdev->dev.kobj, &zii_pic_attr_group);
	serdev_device_close(zp->serdev);
}

static struct serdev_device_driver zii_pic_drv = {
	.probe			= zii_pic_probe,
	.remove			= zii_pic_remove,
	.driver = {
		.name		= "zii-pic",
		.owner		= THIS_MODULE,
		.of_match_table	= zii_pic_dt_ids,
	},
};

module_serdev_device_driver(zii_pic_drv);

MODULE_DEVICE_TABLE(of, zii_pic_dt_ids);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC MCU core driver");
