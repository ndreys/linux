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

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "zii_pic tx: ", DUMP_PREFIX_NONE,
			16, 1, zp->tx_buf, zp->tx_size, false);
#endif
}

static void zii_pic_send_frame(struct zii_pic *zp)
{
	u8 *buf = zp->tx_buf;
	u8 size = zp->tx_size;
	int ret;

	while (1) {

		zp->tx_flag = 0;	/* BEFORE s_d_write_buf() */

		ret = serdev_device_write_buf(zp->sdev, buf, size);
		if (ret < 0) {
			dev_err_ratelimited(&zp->sdev->dev,
					"error %d writing to serdev\n", ret);
			return;
		}

		if (ret == size)
			return;

		buf += ret;
		size -= ret;

		wait_event(zp->tx_wait, zp->tx_flag == 1);
	}
}

static void zii_pic_write_wakeup(struct serdev_device *sdev)
{
	struct zii_pic *zp = dev_get_drvdata(&sdev->dev);

	zp->tx_flag = 1;
	wake_up(&zp->tx_wait);
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

	zii_pic_send_frame(zp);

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
	zii_pic_send_frame(zp);
}


static int zii_pic_check_remove_csum_8b2c(struct zii_pic *zp)
{
	u8 csum_exp, csum_act;

	if (unlikely(zp->rx_size < 2))
		return 0;

	csum_exp = zp->rx_buf[--zp->rx_size];
	csum_act = checksum_8b2c(zp->rx_buf, zp->rx_size);

	return csum_exp == csum_act;
}

static int zii_pic_check_remove_csum_ccitt_false(struct zii_pic *zp)
{
	u16 csum_exp, csum_act;

	if (unlikely(zp->rx_size < 3))
		return 0;

	csum_exp = (zp->rx_buf[zp->rx_size - 2] << 8) |
		   zp->rx_buf[zp->rx_size - 1];
	zp->rx_size -= 2;
	csum_act = crc_ccitt_false(0xffff, zp->rx_buf, zp->rx_size);

	return csum_exp == csum_act;
}

static void zii_pic_send_event_reply(struct work_struct *work)
{
	struct zii_pic *zp = container_of(work, struct zii_pic, er_work);

	mutex_lock(&zp->tx_mutex);
	zii_pic_make_frame(zp, zp->er_code, zp->er_ackid, NULL, 0);
	zp->er_pending = 0;
	zii_pic_send_frame(zp);
	mutex_unlock(&zp->tx_mutex);
}

static void zii_pic_handle_event(struct zii_pic *zp)
{
	u8 code = zp->rx_buf[0];
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

	ehd->handler(ehd->context, code, &zp->rx_buf[2], zp->rx_size - 2);

	zp->er_code = ehd->reply_code;
	zp->er_ackid = zp->rx_buf[1];
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

static void zii_pic_handle_reply(struct zii_pic *zp)
{
	spin_lock(&zp->reply_lock);

	if (!zp->reply_code) {
		spin_unlock(&zp->reply_lock);
		dev_warn(&zp->sdev->dev,
			"got reply frame when not expecting one");
		return;
	}

	if (zp->rx_buf[0] != zp->reply_code ||
	    zp->rx_buf[1] != zp->reply_ackid ||
	    zp->rx_size - 2 < zp->reply_data_size) {
		spin_unlock(&zp->reply_lock);
		dev_warn(&zp->sdev->dev,
			"unexpected reply: need code=%02x ackid=%02x size>=%d, got code=%02x ackid=%02x size=%d\n",
			zp->reply_code, zp->reply_ackid,
			zp->reply_data_size,
			zp->rx_buf[0], zp->rx_buf[1], zp->rx_size - 2);
		return;
	}

	if (zp->reply_data_size)
		memcpy(zp->reply_data, &zp->rx_buf[2],
					zp->reply_data_size);

	zp->reply_code = 0;
	wake_up(&zp->reply_wait);

	spin_unlock(&zp->reply_lock);
}

static void zii_pic_handle_rx_frame(struct zii_pic *zp)
{
	int ret;

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "zii-pic rx: ", DUMP_PREFIX_NONE,
			16, 1, zp->rx_buf, zp->rx_size, false);
#endif

	if (zp->hw_id == ZII_PIC_HW_ID_RDU1)
		ret = zii_pic_check_remove_csum_8b2c(zp);
	else
		ret = zii_pic_check_remove_csum_ccitt_false(zp);
	if (!ret) {
		dev_warn(&zp->sdev->dev, "dropping bad frame\n");
		return;
	}

	if (unlikely(zp->rx_size < 2)) {
		dev_warn(&zp->sdev->dev, "dropping short frame\n");
		return;
	}

	if (zp->rx_buf[0] >= ZII_PIC_EVENT_CODE_MIN &&
	    zp->rx_buf[0] <= ZII_PIC_EVENT_CODE_MAX)
		zii_pic_handle_event(zp);
	else
		zii_pic_handle_reply(zp);
}

static int zii_pic_receive_buf(struct serdev_device *sdev,
		const unsigned char *buf, size_t size)
{
	struct zii_pic *zp = dev_get_drvdata(&sdev->dev);
	u8 c;
	size_t i;

	for (i = 0; i < size; i++) {

		c = buf[i];

		switch (zp->rx_state) {

		case ZII_PIC_EXPECT_SOF:

			if (c == STX)
				zp->rx_state = ZII_PIC_EXPECT_DATA;
			break;

		case ZII_PIC_EXPECT_DATA:

			if (unlikely(c == STX)) {
				dev_warn(&zp->sdev->dev,
						"dropping incomplete frame\n");
				zp->rx_size = 0;
				break;
			}

			if (c == ETX) {
				zii_pic_handle_rx_frame(zp);
				zp->rx_size = 0;
				zp->rx_state = ZII_PIC_EXPECT_SOF;
				break;
			}

			if (zp->rx_size == sizeof(zp->rx_buf)) {
				dev_warn(&zp->sdev->dev,
						"dropping too long frame\n");
				zp->rx_size = 0;
				zp->rx_state = ZII_PIC_EXPECT_SOF;
				break;
			}

			if (c == DLE)
				zp->rx_state = ZII_PIC_EXPECT_ESCAPED_DATA;
			else
				zp->rx_buf[zp->rx_size++] = c;
			break;

		case ZII_PIC_EXPECT_ESCAPED_DATA:

			zp->rx_buf[zp->rx_size++] = c;
			zp->rx_state = ZII_PIC_EXPECT_DATA;
			break;

		default:
			BUG();
		}
	}

	return size;
}

static const struct serdev_device_ops zii_pic_serdev_device_ops = {
	.receive_buf = zii_pic_receive_buf,
	.write_wakeup = zii_pic_write_wakeup,
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
