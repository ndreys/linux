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

int zii_pic_tracing;
module_param_named(tracing, zii_pic_tracing, int, 0644);
MODULE_PARM_DESC(tracing, "enable protocol tracing");

static const struct mfd_cell zii_pic_devices[] = {
	{
		.of_compatible = "zii,pic-main-eeprom",
		.name = ZII_PIC_NAME_MAIN_EEPROM,
	},
	{
		.of_compatible = "zii,pic-dds-eeprom",
		.name = ZII_PIC_NAME_DDS_EEPROM,
	},
	{
		.of_compatible = "zii,pic-watchdog",
		.name = ZII_PIC_NAME_WATCHDOG,
	},
	{
		.of_compatible = "zii,pic-hwmon",
		.name = ZII_PIC_NAME_HWMON,
	},
	{
		.of_compatible = "zii,pic-pwrbutton",
		.name = ZII_PIC_NAME_PWRBUTTON,
	},
	{
		.of_compatible = "zii,pic-backlight",
		.name = ZII_PIC_NAME_BACKLIGHT,
	},
	{
		.of_compatible = "zii,pic-leds",
		.name = ZII_PIC_NAME_LEDS,
	},
};

#define CMD_GET_FW_VERSION	0x20
#define RSP_GET_FW_VERSION	0x60

#define OCMD_GET_FW_VERSION	0x11
#define ORSP_GET_FW_VERSION	0x51

static void zii_pic_get_fw_version(struct zii_pic *zp)
{
	u8 code = zii_pic_code(zp,
			CMD_GET_FW_VERSION, OCMD_GET_FW_VERSION);
	u8 reply_code = zii_pic_code(zp,
			RSP_GET_FW_VERSION, ORSP_GET_FW_VERSION);
	int ret;

	ret = zii_pic_exec(zp, code, NULL, 0,	reply_code,
			(u8 *) &zp->fw_version, sizeof(zp->fw_version));
	if (ret) {
		dev_warn(&zp->sdev->dev, "failed to get fw version\n");
		memset(&zp->fw_version, 0, sizeof(zp->fw_version));
	}
}

#define CMD_GET_BL_VERSION	0x21
#define RSP_GET_BL_VERSION	0x61

#define OCMD_GET_BL_VERSION	0x12
#define ORSP_GET_BL_VERSION	0x52

static void zii_pic_get_bl_version(struct zii_pic *zp)
{
	u8 code = zii_pic_code(zp,
			CMD_GET_BL_VERSION, OCMD_GET_BL_VERSION);
	u8 reply_code = zii_pic_code(zp,
			RSP_GET_BL_VERSION, ORSP_GET_BL_VERSION);
	int ret;

	ret = zii_pic_exec(zp, code, NULL, 0,	reply_code,
			(u8 *) &zp->bl_version, sizeof(zp->bl_version));
	if (ret) {
		dev_warn(&zp->sdev->dev,
				"failed to get bl version\n");
		memset(&zp->bl_version, 0, sizeof(zp->bl_version));
	}
}

#define CMD_GET_RESET_REASON	0xA8
#define RSP_GET_RESET_REASON	0xC8

#define OCMD_GET_RESET_REASON	0x1F
#define ORSP_GET_RESET_REASON	0x5F

static void zii_pic_get_reset_reason(struct zii_pic *zp)
{
	u8 code = zii_pic_code(zp,
			CMD_GET_RESET_REASON, OCMD_GET_RESET_REASON);
	u8 reply_code = zii_pic_code(zp,
			RSP_GET_RESET_REASON, ORSP_GET_RESET_REASON);
	int ret;

	ret = zii_pic_exec(zp, code, NULL, 0, reply_code, &zp->reset_reason, 1);
	if (ret) {
		dev_warn(&zp->sdev->dev, "failed to get reset reason\n");
		zp->reset_reason = 0xFF;
	}
}

#define CMD_BOOT_SOURCE		0x26
#define RSP_BOOT_SOURCE		0x66

#define OCMD_BOOT_SOURCE	0x14
#define ORSP_BOOT_SOURCE	0x54

static void zii_pic_get_boot_source(struct zii_pic *zp)
{
	u8 code = zii_pic_code(zp, CMD_BOOT_SOURCE, OCMD_BOOT_SOURCE);
	u8 reply_code = zii_pic_code(zp, RSP_BOOT_SOURCE, ORSP_BOOT_SOURCE);
	u8 cmd[] = {0, 0};
	int ret;

	ret = zii_pic_exec(zp, code, cmd, sizeof(cmd),
			reply_code, &zp->boot_source, 1);
	if (ret) {
		dev_warn(&zp->sdev->dev, "failed to get boot source\n");
		zp->boot_source = 0xFF;
	}
}

static int zii_pic_set_boot_source(struct zii_pic *zp, u8 boot_src)
{
	u8 code = zii_pic_code(zp, CMD_BOOT_SOURCE, OCMD_BOOT_SOURCE);
	u8 reply_code = zii_pic_code(zp, RSP_BOOT_SOURCE, ORSP_BOOT_SOURCE);
	u8 cmd[] = {1, boot_src};
	int ret;

	ret = zii_pic_exec(zp, code, cmd, sizeof(cmd), reply_code, NULL, 0);
	if (ret)
		return ret;

	zii_pic_get_boot_source(zp);
	if (boot_src != zp->boot_source)
		return -EIO;

	return 0;
}


#define CMD_GET_STATUS		0xA0
#define RSP_GET_STATUS		0xC0

static void zii_pic_get_status_rdu1(struct zii_pic *zp)
{
	struct {
		u8 bl[6];
		u8 fw[6];
		u8 _pad[23];
		u8 gs;
	} __packed reply;

	int ret;

	ret = zii_pic_exec(zp, CMD_GET_STATUS, NULL, 0,
			RSP_GET_STATUS, (u8 *)&reply, sizeof(reply));
	if (ret) {
		dev_warn(&zp->sdev->dev, "failed to read RDU1 status\n");
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
			u8 cmd[] = { 1, 0 };
			zii_pic_exec_reset(zp, CMD_PIC_RESET,
					cmd, sizeof(cmd));
		} else {
			u8 cmd[] = { 1 };
			zii_pic_exec_reset(zp, OCMD_PIC_RESET,
					cmd, sizeof(cmd));
		}

		msleep(550);	/* PIC firmware waits 500 ms before reset */
		dev_emerg(&zp->sdev->dev, "rewset timed out, retrying\n");
	}

	return NOTIFY_OK;	/* unreachable but stops warning */
}

static void zii_pic_setup_reboot(struct zii_pic *zp)
{
	int ret;

	zp->reboot_nb.notifier_call = zii_pic_reboot_notifier;
	ret = unregister_reboot_notifier(&zp->reboot_nb);
	if (ret) {
		dev_warn(&zp->sdev->dev,
				"could not register reboot notifier\n");
		return;
	}

	zp->reset_nb.notifier_call = zii_pic_reset_handler;
	zp->reset_nb.priority = 255;
	ret = unregister_restart_handler(&zp->reset_nb);
	if (ret) {
		dev_warn(&zp->sdev->dev,
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
#define RSP_COPPER_REV_RDU1	0x68

#define CMD_COPPER_REV_RDU2	0x2B
#define RSP_COPPER_REV_RDU2	0x6B

static ssize_t zii_pic_show_copper_rev(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct zii_pic *zp = dev_get_drvdata(dev);
	int ret;
	u8 cmd, rsp, res;

	if (zp->hw_id == ZII_PIC_HW_ID_RDU2) {
		cmd = CMD_COPPER_REV_RDU2;
		rsp = RSP_COPPER_REV_RDU2;
	} else if (zp->hw_id == ZII_PIC_HW_ID_RDU1) {
		cmd = CMD_COPPER_REV_RDU1;
		rsp = RSP_COPPER_REV_RDU1;
	} else
		return -ENOTSUPP;

	ret = zii_pic_exec(zp, cmd, NULL, 0, rsp, &res, sizeof(res));
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

static int zii_pic_probe(struct serdev_device *sdev)
{
	struct zii_pic *zp;
	const struct of_device_id *id;
	u32 baud;
	struct device_node *np;
	int i, ret;

	pr_debug("%s: enter\n", __func__);

	id = of_match_device(zii_pic_dt_ids, &sdev->dev);
	if (!id)
		return -ENODEV;

	zp = devm_kzalloc(&sdev->dev, sizeof(*zp), GFP_KERNEL);
	if (!zp)
		return -ENOMEM;

	zp->sdev = sdev;
	zp->hw_id = (enum zii_pic_hw_id)id->data;
	dev_set_drvdata(&sdev->dev, zp);

	if (of_property_read_u32(sdev->dev.of_node, "current-speed", &baud))
		zp->baud = ZII_PIC_DEFAULT_BAUD_RATE;
	else
		zp->baud = baud;

	ret = zii_pic_comm_init(zp);
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

	/* register cells for devices defined in DT */
	for_each_child_of_node(sdev->dev.of_node, np) {
		for (i = 0; i < ARRAY_SIZE(zii_pic_devices); i++) {
			const struct mfd_cell *cell = &zii_pic_devices[i];
			if (of_device_is_compatible(np, cell->of_compatible)) {
				ret = mfd_add_devices(&sdev->dev,
					PLATFORM_DEVID_NONE, cell, 1,
					NULL, 0, NULL);
				if (ret)
					return ret;
				break;
			}
		}
	}

	ret = sysfs_create_group(&sdev->dev.kobj, &zii_pic_attr_group);
	if (ret)
		goto err_create_group;

	if (zp->hw_id >= ZII_PIC_HW_ID_RDU1) {
		ret = device_create_file(&sdev->dev, &dev_attr_copper_rev);
		if (ret)
			goto err_create_copper_attr;
	}

	zii_pic_setup_reboot(zp);

	return 0;

err_create_copper_attr:
	sysfs_remove_group(&sdev->dev.kobj, &zii_pic_attr_group);
err_create_group:
	zii_pic_comm_cleanup(zp);
	return ret;
}

static void zii_pic_remove(struct serdev_device *sdev)
{
	struct zii_pic *zp = dev_get_drvdata(&sdev->dev);

	zii_pic_cleanup_reboot(zp);
	if (zp->hw_id >= ZII_PIC_HW_ID_RDU1)
		device_remove_file(&sdev->dev, &dev_attr_copper_rev);
	sysfs_remove_group(&sdev->dev.kobj, &zii_pic_attr_group);
	zii_pic_comm_cleanup(zp);
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
