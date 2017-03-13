/*
 * zii-pic-serdev.c - microcontroller unit communication on top of serdev bus
 *
 * Copyright (C) 2017 Nikita Yushchenko <nikita.yoush@cogentembedded.com>
 *
 * Based on:
 *  n_mcu.c - microcontroller unit communication line discipline
 *  Copyright (C) 2015 Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
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
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/zii-pic.h>
#include <linux/serdev.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/crc-ccitt.h>

#define STX			0x02
#define ETX			0x03
#define DLE			0x10

static void csum_8b2c(const u8 *buf, size_t size, u8 *crc)
{
	u8 i;
	u8 sum = 0;

	for (i = 0; i < size; i++)
		sum += buf[i];

	*crc = 1 + ~sum;
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
	const size_t csum_size = pic->csum_size;
	unsigned char crc[csum_size];
	unsigned char frame[ZII_PIC_TX_BUF_SIZE];
	unsigned char *dest = frame;

	BUG_ON(data_size > ZII_PIC_TX_BUF_SIZE);

	pic->csum(data, data_size, crc);

	*dest++ = STX;
	dest = stuff(dest, data, data_size);
	dest = stuff(dest, crc, csum_size);
	*dest++ = ETX;

	length = dest - frame;

	print_hex_dump(KERN_CRIT, "zii_pic tx: ", DUMP_PREFIX_NONE,
		       16, 1, frame, length, false);

	return serdev_device_write(pic->sdev, frame, length);
}

int zii_pic_exec(struct zii_pic *zp,
		 u8 *data, u8 data_size,
		 u8 reply_code, u8 *reply_data, u8 reply_data_size)
{
	int ret = 0;
	const u8 ackid = (u8)atomic_inc_return(&zp->ackid);
	struct zii_pic_reply reply = {
		.code     = reply_code,
		.ackid    = ackid,
		.data     = reply_data,
		.length   = reply_data_size,
		.received = COMPLETION_INITIALIZER_ONSTACK(reply.received),
	};

	serdev_device_bus_lock(zp->sdev);

	if (reply_code) {
		mutex_lock(&zp->reply_lock);
		zp->reply = &reply;
		mutex_unlock(&zp->reply_lock);
	}

	data[1] = ackid;

	zii_pic_write(zp, data, data_size);

	if (reply_code &&
	    !wait_for_completion_timeout(&reply.received, HZ)) {
		dev_err(&zp->sdev->dev, "command timeout\n");
		ret = -ETIMEDOUT;

		mutex_lock(&zp->reply_lock);
		zp->reply = NULL;
		mutex_unlock(&zp->reply_lock);
	}

	serdev_device_bus_unlock(zp->sdev);
	return ret;
}
EXPORT_SYMBOL(zii_pic_exec);

void zii_pic_prepare_for_reset(struct zii_pic *zp)
{
	serdev_device_bus_lock(zp->sdev);
	serdev_device_write_flush(zp->sdev);
}
EXPORT_SYMBOL(zii_pic_prepare_for_reset);

void zii_pic_exec_reset(struct zii_pic *zp,
			const u8 *data, u8 data_size)
{
	zii_pic_write(zp, data, data_size);
}

int zii_pic_set_event_handler(struct zii_pic *zp,
		u8 event_code, u8 reply_code,
		zii_pic_eh handler, void *context)
{
	struct zii_pic_eh_data *ehd;
	int ret;

	if (event_code < ZII_PIC_EVENT_CODE_MIN ||
	    event_code > ZII_PIC_EVENT_CODE_MAX)
		return -EINVAL;
	ehd = &zp->eh[event_code - ZII_PIC_EVENT_CODE_MIN];

	if (ehd->handler)
		ret = -EBUSY;
	else {
		ehd->handler = handler;
		ehd->context = context;
		ehd->reply_code = reply_code;
		ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL(zii_pic_set_event_handler);

void zii_pic_cleanup_event_handler(struct zii_pic *zp, u8 event_code)
{
	struct zii_pic_eh_data *ehd;

	if (event_code < ZII_PIC_EVENT_CODE_MIN ||
	    event_code > ZII_PIC_EVENT_CODE_MAX)
		return;
	ehd = &zp->eh[event_code - ZII_PIC_EVENT_CODE_MIN];

	ehd->handler = NULL;
}
EXPORT_SYMBOL(zii_pic_cleanup_event_handler);

static void zii_pic_receive_event(struct zii_pic *zp,
				 const unsigned char *data, size_t length)
{
	u8 cmd[2];
	u8 code = data[0];
	struct zii_pic_eh_data *ehd = &zp->eh[code - ZII_PIC_EVENT_CODE_MIN];


	if (!ehd->handler) {
		dev_warn(&zp->sdev->dev,
			"dropping unhandled event %02x\n", code);
		return;
	}

	ehd->handler(ehd->context, code, &data[2], length - 2);

	cmd[0] = ehd->reply_code;
	cmd[1] = data[1];

	zii_pic_write(zp, cmd, sizeof(cmd));
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
				dev_warn(&zp->sdev->dev,
					 "unexpected reply: need code=%02x ackid=%02x "
					 "size>=%d, got code=%02x ackid=%02x size=%d\n",
					 reply->code, reply->ackid,
					 reply->length,
					 data[0], data[1], length - 2);
			}
		} else {
			dev_warn(&zp->sdev->dev,
				 "got reply frame when not expecting one");
		}
	}
	mutex_unlock(&zp->reply_lock);
}

static void zii_pic_receive_frame(struct zii_pic *zp,
				  const unsigned char *data,
				  size_t length)
{
	const size_t csum_size = zp->csum_size;
	u8 crc_calculated[csum_size];
	const u8 *crc_reported = &data[length - zp->csum_size];

	print_hex_dump(KERN_CRIT, "zii-pic rx: ", DUMP_PREFIX_NONE,
		       16, 1, data, length, false);

	if (unlikely(length < 3)) {
		dev_warn(&zp->sdev->dev, "dropping short frame\n");
		return;
	}

	zp->csum(data, length - csum_size, crc_calculated);

	if (memcmp(crc_calculated, crc_reported, csum_size)) {
		dev_warn(&zp->sdev->dev, "dropping bad frame\n");
		return;
	}

	length -= csum_size;

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

int zii_pic_comm_init(struct zii_pic *zp)
{
	int ret;

	mutex_init(&zp->reply_lock);

	if (zp->hw_id == ZII_PIC_HW_ID_RDU1) {
		zp->csum_size  = 1;
		zp->csum = csum_8b2c;
	} else {
		zp->csum_size  = 2;
		zp->csum = csum_ccitt;
	}

	serdev_device_set_client_ops(zp->sdev, &zii_pic_serdev_device_ops);
	ret = serdev_device_open(zp->sdev);
	if (ret)
		return ret;

	serdev_device_set_baudrate(zp->sdev, zp->baud);
	return 0;
}

void zii_pic_comm_cleanup(struct zii_pic *zp)
{
	serdev_device_close(zp->sdev);
}
