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

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/zii-pic.h>
#include <linux/serdev.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/crc-ccitt.h>

extern int zii_pic_tracing;

#define STX			0x02
#define ETX			0x03
#define DLE			0x10

static u8 checksum_8b2c(const u8 *buf, u8 size)
{
	u8 i;
	u8 sum = 0;

	for (i = 0; i < size; i++)
		sum += buf[i];

	return 1 + ~sum;
}

static void zii_pic_make_frame(struct zii_pic *zp,
		u8 code, u8 ackid, const u8 *data, u8 data_size)
{
	u8 p, q, c, i;

	BUG_ON(data_size > ZII_PIC_TX_BUF_SIZE);

	p = 0;
	zp->tx_buf[p++] = code;
	zp->tx_buf[p++] = ackid;
	while (data_size--)
		zp->tx_buf[p++] = *data++;

	if (zp->hw_id == ZII_PIC_HW_ID_RDU1) {
		u8 csum = checksum_8b2c(zp->tx_buf, p);
		zp->tx_buf[p++] = csum;
	} else {
		u16 csum = crc_ccitt_false(0xffff, zp->tx_buf, p);
		zp->tx_buf[p++] = csum >> 8;
		zp->tx_buf[p++] = csum;
	}

	for (i = 0, q = 2; i < p; i++, q++) {
		c = zp->tx_buf[i];
		if (c == STX || c == ETX || c == DLE)
			q++;
	}

	zp->tx_size = q;

	zp->tx_buf[--q] = ETX;
	while (q > 0) {
		c = zp->tx_buf[--p];
		zp->tx_buf[--q] = c;
		if (c == STX || c == ETX || c == DLE)
			zp->tx_buf[--q] = DLE;
	}
	zp->tx_buf[0] = STX;

	if (zii_pic_tracing)
		print_hex_dump(KERN_INFO, "zii_pic tx: ", DUMP_PREFIX_NONE,
				16, 1, zp->tx_buf, zp->tx_size, false);
}

int zii_pic_exec(struct zii_pic *zp,
		u8 code, const u8 *data, u8 data_size,
		u8 reply_code, u8 *reply_data, u8 reply_data_size)
{
	int ret;

	mutex_lock(&zp->cmd_mutex);

	mutex_lock(&zp->tx_mutex);

	zp->ackid++;
	zii_pic_make_frame(zp, code, zp->ackid, data, data_size);

	if (reply_code) {
		spin_lock(&zp->reply_lock);
		zp->reply_code = reply_code;
		zp->reply_ackid = zp->ackid;
		zp->reply_data_size = reply_data_size;
		zp->reply_data = reply_data;
		spin_unlock(&zp->reply_lock);
	}

	serdev_device_write(zp->sdev, zp->tx_buf, zp->tx_size);

	mutex_unlock(&zp->tx_mutex);

	if (reply_code && wait_event_timeout(zp->reply_wait,
					!zp->reply_code, HZ) == 0) {
		dev_err(&zp->sdev->dev, "command timeout\n");
		ret = -ETIMEDOUT;
	} else
		ret = 0;

	mutex_unlock(&zp->cmd_mutex);
	return ret;
}
EXPORT_SYMBOL(zii_pic_exec);

void zii_pic_prepare_for_reset(struct zii_pic *zp)
{
	mutex_lock(&zp->tx_mutex);
	serdev_device_write_flush(zp->sdev);
}
EXPORT_SYMBOL(zii_pic_prepare_for_reset);

void zii_pic_exec_reset(struct zii_pic *zp,
		u8 code, const u8 *data, u8 data_size)
{
	zp->ackid++;
	zii_pic_make_frame(zp, code, zp->ackid, data, data_size);
	WARN_ON(serdev_device_write_room(zp->sdev) < zp->tx_size);
	serdev_device_write(zp->sdev, zp->tx_buf, zp->tx_size);
}


static bool
zii_pic_frame_valid_csum_8b2c(const unsigned char *data,
			      size_t length)
{
	const u8 expected   = data[length - 1];
	const u8 calculated = checksum_8b2c(data, length - 1);

	return expected == calculated;
}

static bool
zii_pic_frame_valid_csum_ccitt_false(const unsigned char *data,
				     size_t length)
{
	const u16 expected   =
		le16_to_cpup((const __le16 *)&data[length - 2]);
	const u16 calculated = crc_ccitt_false(0xffff,
					       data, length - 2);

	return expected == calculated;
}

static void zii_pic_send_event_reply(struct work_struct *work)
{
	struct zii_pic *zp = container_of(work, struct zii_pic, er_work);

	mutex_lock(&zp->tx_mutex);
	zii_pic_make_frame(zp, zp->er_code, zp->er_ackid, NULL, 0);
	zp->er_pending = 0;
	serdev_device_write(zp->sdev, zp->tx_buf, zp->tx_size);
	mutex_unlock(&zp->tx_mutex);
}

static void zii_pic_handle_event(struct zii_pic *zp,
				 const unsigned char *data, size_t length)
{
	u8 code = data[0];
	struct zii_pic_eh_data *ehd = &zp->eh[code - ZII_PIC_EVENT_CODE_MIN];

	mutex_lock(&zp->eh_mutex);

	if (unlikely(zp->er_pending)) {
		dev_warn(&zp->sdev->dev,
			"dropping event received before reply to previous one\n");
		goto out;
	}

	if (!ehd->handler) {
		dev_warn(&zp->sdev->dev,
			"dropping unhandled event %02x\n", code);
		goto out;
	}

	ehd->handler(ehd->context, code, &data[2], length - 2);

	zp->er_code = ehd->reply_code;
	zp->er_ackid = data[1];
	schedule_work(&zp->er_work);
out:
	mutex_unlock(&zp->eh_mutex);
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

	mutex_lock(&zp->eh_mutex);

	if (ehd->handler)
		ret = -EBUSY;
	else {
		ehd->handler = handler;
		ehd->context = context;
		ehd->reply_code = reply_code;
		ret = 0;
	}

	mutex_unlock(&zp->eh_mutex);
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

	mutex_lock(&zp->eh_mutex);
	ehd->handler = NULL;
	mutex_unlock(&zp->eh_mutex);
}
EXPORT_SYMBOL(zii_pic_cleanup_event_handler);

static void zii_pic_handle_reply(struct zii_pic *zp,
				 const unsigned char *data, size_t length)
{
	spin_lock(&zp->reply_lock);

	if (!zp->reply_code) {
		spin_unlock(&zp->reply_lock);
		dev_warn(&zp->sdev->dev,
			"got reply frame when not expecting one");
		return;
	}

	if (data[0] != zp->reply_code ||
	    data[1] != zp->reply_ackid ||
	    length - 2 < zp->reply_data_size) {
		spin_unlock(&zp->reply_lock);
		dev_warn(&zp->sdev->dev,
			"unexpected reply: need code=%02x ackid=%02x size>=%d, got code=%02x ackid=%02x size=%d\n",
			zp->reply_code, zp->reply_ackid,
			zp->reply_data_size,
			data[0], data[1], length - 2);
		return;
	}

	if (zp->reply_data_size)
		memcpy(zp->reply_data, &data[2],
					zp->reply_data_size);

	zp->reply_code = 0;
	wake_up(&zp->reply_wait);

	spin_unlock(&zp->reply_lock);
}

static void zii_pic_receive_frame(struct zii_pic *zp,
				  const unsigned char *data,
				  size_t length)
{
	if (zii_pic_tracing)
		print_hex_dump(KERN_INFO, "zii-pic rx: ", DUMP_PREFIX_NONE,
				16, 1, data, length, false);

	if (unlikely(length < 3)) {
		dev_warn(&zp->sdev->dev, "dropping short frame\n");
		return;
	}

	if (!zp->valid_csum(data, length)) {
		dev_warn(&zp->sdev->dev, "dropping bad frame\n");
		return;
	}

	length -= zp->csum_size;

	if (data[0] >= ZII_PIC_EVENT_CODE_MIN &&
	    data[0] <= ZII_PIC_EVENT_CODE_MAX)
		zii_pic_handle_event(zp, data, length);
	else
		zii_pic_handle_reply(zp, data, length);
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

	mutex_init(&zp->tx_mutex);
	init_waitqueue_head(&zp->tx_wait);

	mutex_init(&zp->cmd_mutex);
	init_waitqueue_head(&zp->reply_wait);

	mutex_init(&zp->eh_mutex);
	INIT_WORK(&zp->er_work, zii_pic_send_event_reply);

	if (zp->hw_id == ZII_PIC_HW_ID_RDU1) {
		zp->csum_size  = 1;
		zp->valid_csum = zii_pic_frame_valid_csum_8b2c;
	} else {
		zp->csum_size  = 2;
		zp->valid_csum = zii_pic_frame_valid_csum_ccitt_false;
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
