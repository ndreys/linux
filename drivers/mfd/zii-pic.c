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

/*
 * UART protocol using following entities:
 *  - message to MCU => ACK response
 *  - event from MCU => event ACK
 *
 * Frame structure:
 * <STX> <DATA> <CHECKSUM> <ETX>
 * Where:
 * - STX - is start of transmission character
 * - ETX - end of transmission
 * - DATA - payload
 * - CHECKSUM - checksum calculated on <DATA>
 *
 * If <DATA> or <CHECKSUM> contain one of control characters, then it is
 * escaped using <DLE> control code. Added <DLE> does not participate in
 * checksum calculation.
 */


/* #define DEBUG */

#include <linux/atomic.h>
#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/sched.h>
#include <linux/serdev.h>
#include <linux/zii-pic.h>

/* #include <linux/mfd/core.h> */

#define ZII_PIC_DEFAULT_BAUD_RATE	57600

static void zii_pic_get_fw_version(struct zii_pic *zp)
{
	u8 cmd[2] = { ZII_PIC_CMD_GET_FIRMWARE_VERSION };
	int ret;

	ret = zii_pic_exec(zp, cmd, sizeof(cmd),
			   &zp->fw_version, sizeof(zp->fw_version));
	if (ret) {
		dev_warn(&zp->serdev->dev, "failed to get fw version\n");
		memset(&zp->fw_version, 0, sizeof(zp->fw_version));
	}
}

static void zii_pic_get_bl_version(struct zii_pic *zp)
{
	u8 cmd[2] = { ZII_PIC_CMD_GET_BOOTLOADER_VERSION };
	int ret;

	ret = zii_pic_exec(zp, cmd, sizeof(cmd),
			   &zp->bl_version, sizeof(zp->bl_version));
	if (ret) {
		dev_warn(&zp->serdev->dev,
				"failed to get bl version\n");
		memset(&zp->bl_version, 0, sizeof(zp->bl_version));
	}
}


static void zii_pic_get_boot_source(struct zii_pic *zp)
{
	u8 cmd[] = { ZII_PIC_CMD_BOOT_SOURCE, 0, 0, 0};
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
	u8 cmd[] = { ZII_PIC_CMD_BOOT_SOURCE, 0, 1, boot_src };
	int ret;

	ret = zii_pic_exec(zp, cmd, sizeof(cmd), NULL, 0);
	if (ret)
		return ret;

	zii_pic_get_boot_source(zp);
	if (boot_src != zp->boot_source)
		return -EIO;

	return 0;
}

static void zii_pic_rdu1_read_status(struct zii_pic *zp)
{
	struct {
		u8 bl[6];
		u8 fw[6];
		u8 _pad[23];
		u8 gs;
	} __packed reply;

	int ret;
	u8 cmd[] = { ZII_PIC_CMD_STATUS, 0};

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
	&dev_attr_boot_source.attr,
	NULL
};

static const struct attribute_group zii_pic_attr_group = {
	.attrs = zii_pic_dev_attrs,
};

static ssize_t zii_pic_show_copper_rev(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct zii_pic *zp = dev_get_drvdata(dev);
	int ret;
	u8 cmd[2], res;

	cmd[0] = ZII_PIC_CMD_GET_BOARD_COPPER_REV;

	ret = zii_pic_exec(zp, cmd, sizeof(cmd), &res, sizeof(res));
	if (ret)
		return ret;

	return sprintf(buf, "%02x\n", res);
}

static DEVICE_ATTR(copper_rev, S_IRUSR | S_IRGRP | S_IROTH,
		zii_pic_show_copper_rev, NULL);

/* #define DEBUG */

#define STX			0x02
#define ETX			0x03
#define DLE			0x10

static void csum_8b2c(const u8 *buf, size_t size, u8 *crc)
{
	*crc = *buf++;
	size--;

	while (size--)
		*crc += *buf++;

	*crc = 1 + ~(*crc);
}

static void csum_ccitt(const u8 *buf, size_t size, u8 *crc)
{

	const u16 calculated = crc_ccitt_false(0xffff, buf, size);

	*(__le16 *)crc = cpu_to_le16(calculated);
}

static void *stuff(unsigned char *dest, const unsigned char *src, size_t n)
{
	size_t i;
	for (i = 0; i < n; i++) {
		const unsigned char byte = *src++;

		switch (byte) {
		case STX:
		case ETX:
		case DLE:
			*dest++ = DLE;
		default:
			*dest++ = byte;
		}
	}

	return dest;
}

static int zii_pic_write(struct zii_pic *pic, const u8 *data, u8 data_size)
{
	size_t length;
	const size_t checksum_length = pic->variant->checksum->length;
	unsigned char crc[checksum_length];
	unsigned char frame[ZII_PIC_TX_BUF_SIZE];
	unsigned char *dest = frame;

	BUG_ON(data_size > ZII_PIC_TX_BUF_SIZE);

	pic->variant->checksum->subroutine(data, data_size, crc);

	*dest++ = STX;
	dest = stuff(dest, data, data_size);
	dest = stuff(dest, crc, checksum_length);
	*dest++ = ETX;

	length = dest - frame;

	print_hex_dump(KERN_CRIT, "zii_pic tx: ", DUMP_PREFIX_NONE,
		       16, 1, frame, length, false);

	return serdev_device_write(pic->serdev, frame, length);
}

static u8 zii_pic_reply_code(u8 command)
{
	if (0xA0 <= command && command <= 0xBE)
		return command + 0x20;
	else
		return command + 0x40;
}

int zii_pic_exec(struct zii_pic *pic,
		 void *__data,  size_t data_size,
		 void *reply_data, size_t reply_data_size)
{
	int ret = 0;
	unsigned char *data = __data;
	const u8 ackid = (u8)atomic_inc_return(&pic->ackid);
	struct zii_pic_reply reply = {
		.code     = zii_pic_reply_code(data[0]),
		.ackid    = ackid,
		.data     = reply_data,
		.length   = reply_data_size,
		.received = COMPLETION_INITIALIZER_ONSTACK(reply.received),
	};
	const int command = pic->variant->cmd.translate(data[0]);

	if (command < 0)
		return command;

	serdev_device_bus_lock(pic->serdev);

	mutex_lock(&pic->reply_lock);
	pic->reply = &reply;
	mutex_unlock(&pic->reply_lock);

	data[0] = (u8)command;
	data[1] = ackid;

	zii_pic_write(pic, data, data_size);

	if (!wait_for_completion_timeout(&reply.received, HZ)) {
		dev_err(&pic->serdev->dev, "command timeout\n");
		ret = -ETIMEDOUT;

		mutex_lock(&pic->reply_lock);
		pic->reply = NULL;
		mutex_unlock(&pic->reply_lock);
	}

	serdev_device_bus_unlock(pic->serdev);
	return ret;
}
EXPORT_SYMBOL(zii_pic_exec);

static void zii_pic_receive_event(struct zii_pic *zp,
				  const unsigned char *data, size_t length)
{
	u8 cmd[2];
	u8 code = data[0];

	/*
	  TODO: Is that a universal rule, that event's ack id is event id | 0x01 ?
	 */
	cmd[0] = code | 0x01;
	cmd[1] = data[1];

	zii_pic_write(zp, cmd, sizeof(cmd));

	blocking_notifier_call_chain(&zp->event_notifier_list,
				     data[2] << 8 | code, NULL);
}

static void zii_pic_receive_reply(struct zii_pic *zp,
				  const unsigned char *data, size_t length)
{
	mutex_lock(&zp->reply_lock);
	{
		struct zii_pic_reply *reply = zp->reply;

		if (reply) {
			if (reply->code  == data[0] &&
			    reply->ackid == data[1]) {
				if (reply->length)
					memcpy(reply->data, &data[2],
					       min(reply->length, length - 2));
				complete(&reply->received);
				zp->reply = NULL;
			} else {
				dev_warn(&zp->serdev->dev,
					 "unexpected reply: need code=%02x ackid=%02x "
					 "size>=%d, got code=%02x ackid=%02x size=%d\n",
					 reply->code, reply->ackid,
					 reply->length,
					 data[0], data[1], length - 2);
			}
		} else {
			dev_warn(&zp->serdev->dev,
				 "got reply frame when not expecting one");
		}
	}
	mutex_unlock(&zp->reply_lock);
}

static void zii_pic_receive_frame(struct zii_pic *zp,
				  const unsigned char *data,
				  size_t length)
{
	const size_t checksum_length = zp->variant->checksum->length;
	const size_t payload_length  = length - checksum_length;
	const u8 *crc_reported       = &data[payload_length];
	u8 crc_calculated[checksum_length];

	print_hex_dump(KERN_CRIT, "zii-pic rx: ", DUMP_PREFIX_NONE,
		       16, 1, data, length, false);

	if (unlikely(length <= checksum_length)) {
		dev_warn(&zp->serdev->dev, "dropping short frame\n");
		return;
	}

	zp->variant->checksum->subroutine(data, payload_length, crc_calculated);

	if (memcmp(crc_calculated, crc_reported, checksum_length)) {
		dev_warn(&zp->serdev->dev, "dropping bad frame\n");
		return;
	}

	if (data[0] >= ZII_PIC_EVENT_CODE_MIN &&
	    data[0] <= ZII_PIC_EVENT_CODE_MAX)
		zii_pic_receive_event(zp, data, length);
	else
		zii_pic_receive_reply(zp, data, length);
}

static int zii_pic_receive_buf(struct serdev_device *serdev,
			       const unsigned char *buf, size_t size)
{
	struct zii_pic *pic = dev_get_drvdata(&serdev->dev);
	struct zii_pic_deframer *deframer = &pic->deframer;
	const unsigned char *src = buf;
	const unsigned char *end = buf + size;

	while (src < end) {
		const unsigned char byte = *src++;

		switch (deframer->state) {

		case ZII_PIC_EXPECT_SOF:
			if (byte == STX)
				deframer->state = ZII_PIC_EXPECT_DATA;
			continue;

		case ZII_PIC_EXPECT_DATA:
			switch (byte) {
			case ETX:
				zii_pic_receive_frame(pic,
						      deframer->data,
						      deframer->length);
			case STX: /* FALLTHROUGH */
				if (unlikely(byte == STX))
					dev_warn(&serdev->dev,
						 "Frame has second STX "
						 "before ETX. Dropping it\n");
				goto reset_framer;
			case DLE:
				deframer->state = ZII_PIC_EXPECT_ESCAPED_DATA;
				continue;
			}

		case ZII_PIC_EXPECT_ESCAPED_DATA: /* FALLTHROUGH */
			deframer->data[deframer->length++] = byte;

			if (deframer->length == sizeof(deframer->data)) {
				dev_warn(&serdev->dev, "Frame too long. Dropping it\n");
				goto reset_framer;
			}

			deframer->state = ZII_PIC_EXPECT_DATA;
			break;
		}
	}

	return src - buf;

reset_framer:
	deframer->state  = ZII_PIC_EXPECT_SOF;
	deframer->length = 0;
	return src - buf;
}

static const struct serdev_device_ops zii_pic_serdev_device_ops = {
	.receive_buf = zii_pic_receive_buf,
};

static int zii_pic_open(struct zii_pic *zp, unsigned int speed)
{
	int ret;

	mutex_init(&zp->reply_lock);
	BLOCKING_INIT_NOTIFIER_HEAD(&zp->event_notifier_list);

	serdev_device_set_client_ops(zp->serdev, &zii_pic_serdev_device_ops);
	ret = serdev_device_open(zp->serdev);
	if (ret)
		return ret;

	serdev_device_set_baudrate(zp->serdev, speed);
	return 0;
}


static int zii_pic_rdu1_cmd_translate(enum zii_pic_command command)
{
	/* FIXME: This command is not in ICD */
#define CMD_COPPER_REV_RDU1	0x28

	if (0xA0 <= command && command <= 0xBB)
		return command;
	else
		return -EINVAL;
}

static int zii_pic_rdu2_cmd_translate(enum zii_pic_command command)
{
	if (0x20 <= command && command <= 0x2F)
		return command;
	else
		return zii_pic_rdu1_cmd_translate(command);
}

static int zii_pic_default_cmd_translate(enum zii_pic_command command)
{
	switch (command) {
	case ZII_PIC_CMD_GET_FIRMWARE_VERSION:
		return 0x11;
	case ZII_PIC_CMD_GET_BOOTLOADER_VERSION:
		return 0x12;
	case ZII_PIC_CMD_RESET_REASON:
		return 0x1F;
	case ZII_PIC_CMD_BOOT_SOURCE:
		return 0x14;
	case ZII_PIC_CMD_RESET:
		return 0x1E;
	case ZII_PIC_CMD_SW_WDT:
		return 0x1C;
	default:
		return -EINVAL;
	}
}

static void zii_pic_default_read_status(struct zii_pic *pic)
{
	zii_pic_get_fw_version(pic);
	zii_pic_get_bl_version(pic);
}

const static struct zii_pic_checksum zii_pic_checksum_8b2c = {
	.length     = 1,
	.subroutine = csum_8b2c,
};

const static struct zii_pic_checksum zii_pic_checksum_ccitt = {
	.length     = 2,
	.subroutine = csum_ccitt,
};

const static struct zii_pic_variant zii_pic_legacy = {
	.checksum = &zii_pic_checksum_8b2c,
	.cmd = {
		.translate = zii_pic_default_cmd_translate,
	},
	.read_status = zii_pic_default_read_status,
};

const static struct zii_pic_variant zii_pic_rdu1 = {
	.checksum = &zii_pic_checksum_8b2c,
	.cmd = {
		.translate   = zii_pic_rdu1_cmd_translate,
	},
	.read_status = zii_pic_rdu1_read_status,
};

const static struct zii_pic_variant zii_pic_rdu2 = {
	.checksum = &zii_pic_checksum_ccitt,
	.cmd  = {
		.translate   = zii_pic_rdu2_cmd_translate,
	},
	.read_status = zii_pic_default_read_status,
};

const static struct of_device_id zii_pic_dt_ids[] = {
	{ .compatible = COMPATIBLE_ZII_PIC_NIU,  .data = &zii_pic_legacy },
	{ .compatible = COMPATIBLE_ZII_PIC_MEZZ, .data = &zii_pic_legacy },
	{ .compatible = COMPATIBLE_ZII_PIC_ESB,	 .data = &zii_pic_legacy },
	{ .compatible = COMPATIBLE_ZII_PIC_RDU1, .data = &zii_pic_rdu1   },
	{ .compatible = COMPATIBLE_ZII_PIC_RDU2, .data = &zii_pic_rdu2   },
	{ /* sentinel */ }
};

static int zii_pic_probe(struct serdev_device *serdev)
{
	struct zii_pic *zp;
	const struct of_device_id *id;
	struct device *dev = &serdev->dev;
	u32 baud = ZII_PIC_DEFAULT_BAUD_RATE;
	int ret;

	zp = devm_kzalloc(dev, sizeof(*zp), GFP_KERNEL);
	if (!zp)
		return -ENOMEM;

	zp->serdev = serdev;
	dev_set_drvdata(dev, zp);

	zp->variant = of_device_get_match_data(dev);
	if (!zp->variant)
		return -ENODEV;

	of_property_read_u32(dev->of_node, "current-speed", &baud);
	ret = zii_pic_open(zp, baud);
	if (ret)
		return ret;

	zp->variant->read_status(zp);

	zii_pic_get_boot_source(zp);

	ret = sysfs_create_group(&dev->kobj, &zii_pic_attr_group);
	if (ret)
		goto err_create_group;
#if 0
	if (zp->hw_id >= ZII_PIC_HW_ID_RDU1) {
		ret = device_create_file(dev, &dev_attr_copper_rev);
		if (ret)
			goto err_create_copper_attr;
	}
#endif

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

#if 0
	if (zp->hw_id >= ZII_PIC_HW_ID_RDU1)
		device_remove_file(&serdev->dev, &dev_attr_copper_rev);
#endif
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
