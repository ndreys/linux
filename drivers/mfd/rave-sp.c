/*
 * rave-sp.c - Multifunction core driver for Zodiac Inflight Innovations
 * SP MCU that is connected via dedicated UART port
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
#include <linux/rave-sp.h>

enum {
	RAVE_SP_BOOT_SOURCE_GET = 0,
	RAVE_SP_BOOT_SOURCE_SET = 1,

	RAVE_SP_MAX_DATA_SIZE   = 64,
	RAVE_SP_CHECKSUM_SIZE   = 2, /* Worst case scenariou on RDU2 */
	/* We don't store STX, ETX and unescaped bytes, so Rx is only
	 * DATA + CSUM */
	RAVE_SP_RX_BUFFER_SIZE  = RAVE_SP_MAX_DATA_SIZE + RAVE_SP_CHECKSUM_SIZE,
	RAVE_SP_STX_ETX_SIZE    = 2,
	/* For Tx we have to have space for everything, STX, EXT and
	 * potentially stuffed DATA + CSUM data + csum */
	RAVE_SP_TX_BUFFER_SIZE  = RAVE_SP_STX_ETX_SIZE +
				  2 * RAVE_SP_RX_BUFFER_SIZE,
};

enum rave_sp_deframer_state {
	RAVE_SP_EXPECT_SOF,
	RAVE_SP_EXPECT_DATA,
	RAVE_SP_EXPECT_ESCAPED_DATA,
};

struct rave_sp_deframer {
	enum rave_sp_deframer_state state;
	unsigned char data[RAVE_SP_RX_BUFFER_SIZE];
	size_t length;
};

struct rave_sp_reply {
	size_t length;
	void  *data;
	u8     code;
	u8     ackid;
	struct completion received;
};

struct rave_sp_checksum {
	size_t length;
	void (*subroutine) (const u8 *, size_t, u8 *);
};

enum rave_sp_boot_source {
	RAVE_SP_BOOT_SOURCE_SD		= 0,
	RAVE_SP_BOOT_SOURCE_EMMC	= 1,
	RAVE_SP_BOOT_SOURCE_NOR		= 2,
};

struct rave_sp_variant {
	const struct rave_sp_checksum *checksum;

	struct {
		int  (*translate)   (enum rave_sp_command);
		int (*get_boot_source) (struct rave_sp *);
		int (*set_boot_source) (struct rave_sp *,
					enum rave_sp_boot_source);
	} cmd;

	void (*init) (struct rave_sp *);

	struct attribute_group group;
};

struct rave_sp {
	struct serdev_device *serdev;

	struct rave_sp_deframer deframer;
	atomic_t ackid;

	struct mutex bus_lock;
	struct mutex reply_lock;
	struct rave_sp_reply *reply;

	const char *part_number_firmware;
	const char *part_number_bootloader;
	const char *copper_rev;

	u8 reset_reason;

	const struct rave_sp_variant *variant;

	struct blocking_notifier_head event_notifier_list;

	struct attribute_group *group;
};

struct rave_sp_rsp_status {
	u8 bl_bytes[6];
	u8 fw_bytes[6];
	u8 gs_format;
} __packed;

static bool rave_sp_id_is_event(u8 code)
{
	return (code & 0xF0) == RAVE_SP_EVNT_BASE;
}

static void devm_rave_sp_unregister_event_notifier(struct device *dev, void *res)
{
	struct rave_sp *sp = dev_get_drvdata(dev->parent);
	struct notifier_block *nb = *(struct notifier_block **)res;
	struct blocking_notifier_head *bnh = &sp->event_notifier_list;

	WARN_ON(blocking_notifier_chain_unregister(bnh, nb));
}

int devm_rave_sp_register_event_notifier(struct device *dev,
					 struct notifier_block *nb)
{
	struct rave_sp *sp = dev_get_drvdata(dev->parent);
	struct notifier_block **rcnb;
	int ret;

	rcnb = devres_alloc(devm_rave_sp_unregister_event_notifier,
			    sizeof(*rcnb), GFP_KERNEL);
	if (!rcnb)
		return -ENOMEM;

	ret = blocking_notifier_chain_register(&sp->event_notifier_list, nb);
	if (!ret) {
		*rcnb = nb;
		devres_add(dev, rcnb);
	} else {
		devres_free(rcnb);
	}

	return ret;

}
EXPORT_SYMBOL_GPL(devm_rave_sp_register_event_notifier);

static const char *devm_rave_sp_version(struct device *dev, const char *buf)
{
	return devm_kasprintf(dev, GFP_KERNEL, "%d.%d.%d.%c%c",
			      buf[0], le16_to_cpup((const __le16 *)&buf[1]),
			      buf[3], buf[4], buf[5]);
}

static int rave_sp_get_status(struct rave_sp *sp,
			      struct rave_sp_rsp_status *status)
{
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_STATUS,
		[1] = 0
	};
	return rave_sp_exec(sp, cmd, sizeof(cmd), &status, sizeof(status));
}

static ssize_t
rave_sp_show_part_number(char *buf, const char *version, size_t version_length)
{
	memcpy(buf, version, version_length + 1);
	return version_length;
}

#define RAVE_SP_ATTR_RO_STRING(name)					\
	static ssize_t							\
	name##_show(struct device *dev,					\
		    struct device_attribute *attr,			\
		    char *buf)						\
	{								\
		struct rave_sp *sp = dev_get_drvdata(dev);		\
		return rave_sp_show_part_number(buf, sp->name,		\
						strlen(sp->name));	\
	}								\
	static DEVICE_ATTR_RO(name)

RAVE_SP_ATTR_RO_STRING(part_number_firmware);
RAVE_SP_ATTR_RO_STRING(part_number_bootloader);
RAVE_SP_ATTR_RO_STRING(copper_rev);

static int rave_sp_rdu1_get_boot_source(struct rave_sp *sp)
{
	struct rave_sp_rsp_status status;
	const int ret = rave_sp_get_status(sp, &status);

	return (ret < 0) ? ret : (status.gs_format >> 2) & 0x03;
}

static int rave_sp_rdu1_set_boot_source(struct rave_sp *sp,
					enum rave_sp_boot_source boot_source)
{
	return -ENOTSUPP;
}

static int rave_sp_common_set_boot_source(struct rave_sp *sp,
					  enum rave_sp_boot_source boot_source)
{
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_BOOT_SOURCE,
		[1] = 0,
		[2] = RAVE_SP_BOOT_SOURCE_GET,
		[3] = (u8)boot_source,
	};
	return rave_sp_exec(sp, cmd, sizeof(cmd), NULL, 0);
}

static int rave_sp_common_get_boot_source(struct rave_sp *sp)
{
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_BOOT_SOURCE,
		[1] = 0,
		[2] = RAVE_SP_BOOT_SOURCE_SET,
		[3] = 0,
	};
	u8 boot_source;
	const int ret = rave_sp_exec(sp, cmd, sizeof(cmd),
				     &boot_source, sizeof(boot_source));

	return (ret < 0) ? ret : boot_source;
}

static ssize_t rave_sp_show_boot_source(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rave_sp *sp = dev_get_drvdata(dev);
	const int ret = sp->variant->cmd.get_boot_source(sp);

	return (ret < 0) ? ret : sprintf(buf, "%d\n", ret);
}

static ssize_t rave_sp_store_boot_source(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rave_sp *sp = dev_get_drvdata(dev);
	enum rave_sp_boot_source boot_source;
	int ret;
	u8 input;

	ret = kstrtou8(buf, 0, &input);
	if (ret)
		return ret;

	boot_source = input;

	if (boot_source != RAVE_SP_BOOT_SOURCE_SD &&
	    boot_source != RAVE_SP_BOOT_SOURCE_EMMC &&
	    boot_source != RAVE_SP_BOOT_SOURCE_NOR)
		return -EINVAL;

	ret = sp->variant->cmd.set_boot_source(sp, boot_source);

	return (ret < 0) ? ret : count;
}

static DEVICE_ATTR(boot_source, S_IRUSR | S_IWUSR | S_IRGRP,
		   rave_sp_show_boot_source, rave_sp_store_boot_source);


static void devm_rave_sp_sysfs_group_release(struct device *dev, void *res)
{
	struct rave_sp *sp = *(struct rave_sp **)res;
	const struct attribute_group *group = &sp->variant->group;
	struct kobject *root = &sp->serdev->dev.kobj;

	sysfs_remove_group(root, group);
}

static int devm_zii_sysfs_create_group(struct rave_sp *sp)
{
	struct rave_sp **rcsp;
	struct device *dev = &sp->serdev->dev;
	const struct attribute_group *group = &sp->variant->group;
	struct kobject *root = &dev->kobj;
	int ret;

	rcsp = devres_alloc(devm_rave_sp_sysfs_group_release,
			     sizeof(*rcsp), GFP_KERNEL);
	if (!rcsp)
		return -ENOMEM;

	ret = sysfs_create_group(root, group);
	if (!ret) {
		*rcsp = sp;
		devres_add(dev, rcsp);
	} else {
		devres_free(rcsp);
	}

	return ret;
}

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

	*(__le16 *)crc = cpu_to_be16(calculated);
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

static int rave_sp_write(struct rave_sp *sp, const u8 *data, u8 data_size)
{
	size_t length;
	const size_t checksum_length = sp->variant->checksum->length;
	unsigned char crc[checksum_length];
	unsigned char frame[RAVE_SP_TX_BUFFER_SIZE];
	unsigned char *dest = frame;

	if (WARN_ON(data_size > sizeof(frame)))
		return -ENOMEM;

	sp->variant->checksum->subroutine(data, data_size, crc);

	*dest++ = STX;
	dest = stuff(dest, data, data_size);
	dest = stuff(dest, crc, checksum_length);
	*dest++ = ETX;

	length = dest - frame;

	print_hex_dump(KERN_CRIT, "rave-sp tx: ", DUMP_PREFIX_NONE,
		       16, 1, frame, length, false);

	return serdev_device_write(sp->serdev, frame, length, HZ);
}

static u8 rave_sp_reply_code(u8 command)
{
	switch (command) {
	case 0xA0 ... 0xBE:
		return command + 0x20;
	case 0xE0 ... 0xEF:
		return command | 0x01;
	default:
		return command + 0x40;
	}
}

int rave_sp_exec(struct rave_sp *sp,
		 void *__data,  size_t data_size,
		 void *reply_data, size_t reply_data_size)
{
	int ret = 0;
	unsigned char *data = __data;
	const u8 ackid = (u8)atomic_inc_return(&sp->ackid);
	struct rave_sp_reply reply = {
		.code     = rave_sp_reply_code(data[0]),
		.ackid    = ackid,
		.data     = reply_data,
		.length   = reply_data_size,
		.received = COMPLETION_INITIALIZER_ONSTACK(reply.received),
	};
	const int command = sp->variant->cmd.translate(data[0]);

	if (command < 0)
		return command;

	mutex_lock(&sp->bus_lock);

	mutex_lock(&sp->reply_lock);
	sp->reply = &reply;
	mutex_unlock(&sp->reply_lock);

	data[0] = (u8)command;
	data[1] = ackid;

	rave_sp_write(sp, data, data_size);

	if (!wait_for_completion_timeout(&reply.received, HZ)) {
		dev_err(&sp->serdev->dev, "command timeout\n");
		ret = -ETIMEDOUT;

		mutex_lock(&sp->reply_lock);
		sp->reply = NULL;
		mutex_unlock(&sp->reply_lock);
	}

	mutex_unlock(&sp->bus_lock);
	return ret;
}
EXPORT_SYMBOL(rave_sp_exec);

static void rave_sp_receive_event(struct rave_sp *sp,
				  const unsigned char *data, size_t length)
{
	u8 cmd[] = {
		[0] = rave_sp_reply_code(data[0]),
		[1] = data[1],
	};

	rave_sp_write(sp, cmd, sizeof(cmd));

	blocking_notifier_call_chain(&sp->event_notifier_list,
				     rave_sp_action(data[0], data[2]),
				     NULL);
}

static void rave_sp_receive_reply(struct rave_sp *sp,
				  const unsigned char *data, size_t length)
{
	struct device *dev = &sp->serdev->dev;
	mutex_lock(&sp->reply_lock);
	{
		struct rave_sp_reply *reply = sp->reply;

		if (reply) {
			if (reply->code  == data[0] &&
			    reply->ackid == data[1]) {
				if (reply->length)
					memcpy(reply->data, &data[2],
					       min(reply->length, length - 2));
				complete(&reply->received);
				sp->reply = NULL;
			} else {
				dev_warn(dev,
					 "unexpected reply: need code=%02x ackid=%02x "
					 "size>=%d, got code=%02x ackid=%02x size=%d\n",
					 reply->code, reply->ackid,
					 reply->length,
					 data[0], data[1], length - 2);
			}
		} else {
			dev_warn(dev, "got reply frame when not expecting one");
		}
	}
	mutex_unlock(&sp->reply_lock);
}

static void rave_sp_receive_frame(struct rave_sp *sp,
				  const unsigned char *data,
				  size_t length)
{
	const size_t checksum_length = sp->variant->checksum->length;
	const size_t payload_length  = length - checksum_length;
	const u8 *crc_reported       = &data[payload_length];
	struct device *dev           = &sp->serdev->dev;
	u8 crc_calculated[checksum_length];

	print_hex_dump(KERN_CRIT, "rave-sp rx: ", DUMP_PREFIX_NONE,
		       16, 1, data, length, false);

	if (unlikely(length <= checksum_length)) {
		dev_warn(dev, "dropping short frame\n");
		return;
	}

	sp->variant->checksum->subroutine(data, payload_length, crc_calculated);

	if (memcmp(crc_calculated, crc_reported, checksum_length)) {
		dev_warn(dev, "dropping bad frame\n");
		return;
	}

	if (rave_sp_id_is_event(data[0]))
		rave_sp_receive_event(sp, data, length);
	else
		rave_sp_receive_reply(sp, data, length);
}

static int rave_sp_receive_buf(struct serdev_device *serdev,
			       const unsigned char *buf, size_t size)
{
	struct device *dev  = &serdev->dev;
	struct rave_sp *sp = dev_get_drvdata(dev);
	struct rave_sp_deframer *deframer = &sp->deframer;
	const unsigned char *src = buf;
	const unsigned char *end = buf + size;

	while (src < end) {
		const unsigned char byte = *src++;

		switch (deframer->state) {

		case RAVE_SP_EXPECT_SOF:
			if (byte == STX)
				deframer->state = RAVE_SP_EXPECT_DATA;
			continue;

		case RAVE_SP_EXPECT_DATA:
			switch (byte) {
			case ETX:
				rave_sp_receive_frame(sp,
						      deframer->data,
						      deframer->length);
			case STX: /* FALLTHROUGH */
				if (unlikely(byte == STX))
					dev_warn(dev,
						 "Frame has second STX "
						 "before ETX. Dropping it\n");
				goto reset_framer;
			case DLE:
				deframer->state = RAVE_SP_EXPECT_ESCAPED_DATA;
				continue;
			}

		case RAVE_SP_EXPECT_ESCAPED_DATA: /* FALLTHROUGH */
			deframer->data[deframer->length++] = byte;

			if (deframer->length == sizeof(deframer->data)) {
				dev_warn(dev, "Frame too long. Dropping it\n");
				goto reset_framer;
			}

			deframer->state = RAVE_SP_EXPECT_DATA;
			break;
		}
	}

	return src - buf;

reset_framer:
	deframer->state  = RAVE_SP_EXPECT_SOF;
	deframer->length = 0;
	return src - buf;
}

static int rave_sp_rdu1_cmd_translate(enum rave_sp_command command)
{
	if (0xA0 <= command && command <= 0xBB)
		return command;
	else
		return -EINVAL;
}

static int rave_sp_rdu2_cmd_translate(enum rave_sp_command command)
{
	switch (command) {
	case 0x20 ... 0x2F:
		return command;
	case RAVE_SP_CMD_REQ_COPPER_REV:
		return -EINVAL;
	default:
		return rave_sp_rdu1_cmd_translate(command);
	}
}

static int rave_sp_default_cmd_translate(enum rave_sp_command command)
{
	switch (command) {
	case RAVE_SP_CMD_GET_FIRMWARE_VERSION:
		return 0x11;
	case RAVE_SP_CMD_GET_BOOTLOADER_VERSION:
		return 0x12;
	case RAVE_SP_CMD_RESET_REASON:
		return 0x1F;
	case RAVE_SP_CMD_BOOT_SOURCE:
		return 0x14;
	case RAVE_SP_CMD_RESET:
		return 0x1E;
	case RAVE_SP_CMD_SW_WDT:
		return 0x1C;
	default:
		return -EINVAL;
	}
}

static void rave_sp_rdu1_init(struct rave_sp *sp)
{
	struct device *dev = &sp->serdev->dev;
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_STATUS,
		[1] = 0
	};
	struct rave_sp_rsp_status status;
	u8 revision = 0;
	int ret;

	ret = rave_sp_get_status(sp, &status);
	if (!ret) {
		sp->part_number_firmware =
			devm_rave_sp_version(dev, status.fw_bytes);
		sp->part_number_bootloader =
			devm_rave_sp_version(dev, status.bl_bytes);
	} else {
		dev_err(dev, "CMD_STATUS failed %d\n", ret);
	}

	cmd[0] = RAVE_SP_CMD_REQ_COPPER_REV;

	ret = rave_sp_exec(sp, cmd, sizeof(cmd), &revision, sizeof(revision));
	if (!ret)
		sp->copper_rev =
			devm_kasprintf(dev, GFP_KERNEL, "%02x", revision);
	else
		dev_err(dev, "CMD_REQ_COPPER_REV failed %d\n", ret);
}

static void rave_sp_common_init(struct rave_sp *sp)
{
	struct device *dev = &sp->serdev->dev;
	u8 version[6];
	u8 cmd[2];
	int ret;

	cmd[0] = RAVE_SP_CMD_GET_FIRMWARE_VERSION;
	ret = rave_sp_exec(sp, cmd, sizeof(cmd), version, sizeof(version));
	if (!ret)
		sp->part_number_firmware =
			devm_rave_sp_version(dev, version);
	else
		dev_warn(dev, "CMD_GET_FIRMWARE_VERSION failed %d\n", ret);

	cmd[0] = RAVE_SP_CMD_GET_BOOTLOADER_VERSION;
	ret = rave_sp_exec(sp, cmd, sizeof(cmd), version, sizeof(version));
	if (!ret)
		sp->part_number_bootloader =
			devm_rave_sp_version(dev, version);
	else
		dev_warn(dev, "CMD_GET_BOOTLOADER_VERSION failed %d\n", ret);
}

const static struct rave_sp_checksum rave_sp_checksum_8b2c = {
	.length     = 1,
	.subroutine = csum_8b2c,
};

const static struct rave_sp_checksum rave_sp_checksum_ccitt = {
	.length     = 2,
	.subroutine = csum_ccitt,
};

static struct attribute *rave_sp_common_attrs[] = {
	&dev_attr_part_number_firmware.attr,
	&dev_attr_part_number_bootloader.attr,
	&dev_attr_boot_source.attr,
	NULL
};

static struct attribute *rave_sp_rdu1_attrs[] = {
	&dev_attr_part_number_firmware.attr,
	&dev_attr_part_number_bootloader.attr,
	&dev_attr_boot_source.attr,
	&dev_attr_copper_rev.attr,
	NULL
};

const static struct rave_sp_variant rave_sp_legacy = {
	.checksum = &rave_sp_checksum_8b2c,
	.cmd = {
		.translate = rave_sp_default_cmd_translate,
		.get_boot_source = rave_sp_common_get_boot_source,
		.set_boot_source = rave_sp_common_set_boot_source,
	},
	.group = {
		.attrs = rave_sp_common_attrs,
	},
	.init = rave_sp_common_init,
};

const static struct rave_sp_variant rave_sp_rdu1 = {
	.checksum = &rave_sp_checksum_8b2c,
	.cmd = {
		.translate = rave_sp_rdu1_cmd_translate,
		.get_boot_source = rave_sp_rdu1_get_boot_source,
		.set_boot_source = rave_sp_rdu1_set_boot_source,
	},
	.group = {
		.attrs = rave_sp_rdu1_attrs,
	},
	.init = rave_sp_rdu1_init,
};

const static struct rave_sp_variant rave_sp_rdu2 = {
	.checksum = &rave_sp_checksum_ccitt,
	.cmd = {
		.translate = rave_sp_rdu2_cmd_translate,
		.get_boot_source = rave_sp_common_get_boot_source,
		.set_boot_source = rave_sp_common_set_boot_source,
	},
	.group = {
		.attrs = rave_sp_common_attrs,
	},
	.init = rave_sp_common_init,
};

const static struct of_device_id rave_sp_dt_ids[] = {
	{ .compatible = COMPATIBLE_RAVE_SP_NIU,  .data = &rave_sp_legacy },
	{ .compatible = COMPATIBLE_RAVE_SP_MEZZ, .data = &rave_sp_legacy },
	{ .compatible = COMPATIBLE_RAVE_SP_ESB,	 .data = &rave_sp_legacy },
	{ .compatible = COMPATIBLE_RAVE_SP_RDU1, .data = &rave_sp_rdu1   },
	{ .compatible = COMPATIBLE_RAVE_SP_RDU2, .data = &rave_sp_rdu2   },
	{ /* sentinel */ }
};

static int rave_sp_probe(struct serdev_device *serdev)
{
	static const struct serdev_device_ops serdev_device_ops = {
		.receive_buf  = rave_sp_receive_buf,
		.write_wakeup = serdev_device_write_wakeup,
	};
	struct rave_sp *sp;
	struct device *dev = &serdev->dev;
	const char *unknown = "unknown";
	u32 baud;
	int ret;

	if (of_property_read_u32(dev->of_node, "current-speed", &baud)) {
		dev_err(dev,
			"'current-speed' is not specified in device node\n");
		return -EINVAL;
	}

	sp = devm_kzalloc(dev, sizeof(*sp), GFP_KERNEL);
	if (!sp)
		return -ENOMEM;

	sp->serdev = serdev;
	dev_set_drvdata(dev, sp);

	sp->variant = of_device_get_match_data(dev);
	if (!sp->variant)
		return -ENODEV;

	mutex_init(&sp->bus_lock);
	mutex_init(&sp->reply_lock);
	BLOCKING_INIT_NOTIFIER_HEAD(&sp->event_notifier_list);

	serdev_device_set_client_ops(serdev, &serdev_device_ops);
	ret = serdev_device_open(serdev);
	if (ret)
		return ret;

	serdev_device_set_baudrate(serdev, baud);

	sp->copper_rev			= unknown;
	sp->part_number_firmware	= unknown;
	sp->part_number_bootloader	= unknown;

	sp->variant->init(sp);

	ret = devm_zii_sysfs_create_group(sp);
	if (ret) {
		serdev_device_close(serdev);
		return ret;
	}

	return of_platform_default_populate(dev->of_node, NULL, dev);
}

static void rave_sp_remove(struct serdev_device *serdev)
{
	of_platform_depopulate(&serdev->dev);
	serdev_device_close(serdev);
}

static struct serdev_device_driver rave_sp_drv = {
	.probe			= rave_sp_probe,
	.remove			= rave_sp_remove,
	.driver = {
		.name		= KBUILD_MODNAME,
		.owner		= THIS_MODULE,
		.of_match_table	= rave_sp_dt_ids,
	},
};

module_serdev_device_driver(rave_sp_drv);

MODULE_DEVICE_TABLE(of, rave_sp_dt_ids);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("Rave SP core driver");
