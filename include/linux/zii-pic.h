#ifndef _LINUX_ZII_PIC_H_
#define _LINUX_ZII_PIC_H_

#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/device.h>

struct zii_pic;

#define ZII_PIC_MAX_DATA_SIZE	64

/* For Tx, <STX DATA CSUM ETX> is stored, and all but STX/ETX can be escaped */
#define ZII_PIC_TX_BUF_SIZE (1 + 2*ZII_PIC_MAX_DATA_SIZE + 2*2 + 1)

/* For Rx, only frame data and csum is stored */
#define ZII_PIC_RX_BUF_SIZE (ZII_PIC_MAX_DATA_SIZE + 2)

struct serdev_device;

enum zii_pic_hw_id {
	ZII_PIC_HW_ID_NIU,
	ZII_PIC_HW_ID_MEZZ,
	ZII_PIC_HW_ID_ESB,
	ZII_PIC_HW_ID_RDU1,
	ZII_PIC_HW_ID_RDU2,
};

enum zii_pic_rx_state {
	ZII_PIC_EXPECT_SOF = 0,
	ZII_PIC_EXPECT_DATA,
	ZII_PIC_EXPECT_ESCAPED_DATA,
};

struct zii_pic_version {
	u8	hw;
	u16	major;
	u8	minor;
	u8	letter_1;
	u8	letter_2;
} __packed;

#define ZII_PIC_EVENT_CODE_MIN		0xE0
#define ZII_PIC_EVENT_CODE_MAX		0xEF

#define ZII_PIC_EH_NR	(ZII_PIC_EVENT_CODE_MAX - ZII_PIC_EVENT_CODE_MIN + 1)

/* For now, assume no data in event reply */

typedef void (*zii_pic_eh)(void *context,
		u8 event_code, const u8 *data, u8 data_size);

struct zii_pic_eh_data {
	zii_pic_eh	handler;
	void		*context;
	u8		reply_code;
};


struct zii_pic {
	struct serdev_device		*sdev;
	enum zii_pic_hw_id		hw_id;
	u32				baud;

	u8				tx_buf[ZII_PIC_TX_BUF_SIZE];
	u8				tx_size;
	u8				tx_flag;
	struct mutex			tx_mutex;
	wait_queue_head_t		tx_wait;

	u8				rx_buf[ZII_PIC_RX_BUF_SIZE];
	u8				rx_size;
	enum zii_pic_rx_state		rx_state;

	struct mutex			cmd_mutex;
	u8				ackid;

	u8				reply_code;
	u8				reply_ackid;
	u8				reply_data_size;
	u8				*reply_data;
	spinlock_t			reply_lock;
	wait_queue_head_t		reply_wait;

	struct zii_pic_eh_data		eh[ZII_PIC_EH_NR];
	struct mutex			eh_mutex;

	u8				er_pending;
	u8				er_code;
	u8				er_ackid;
	struct work_struct		er_work;

	struct zii_pic_version		fw_version;
	struct zii_pic_version		bl_version;
	u8				reset_reason;
	u8				boot_source;

	struct notifier_block		reboot_nb,
					reset_nb;
};

static inline struct zii_pic *zii_pic_parent(struct device *dev)
{
	if (dev->parent)
		return dev_get_drvdata(dev->parent);
	else
		return NULL;
}

static inline u8 zii_pic_code(struct zii_pic *zp, u8 rdu, u8 old)
{
	return zp->hw_id >= ZII_PIC_HW_ID_RDU1 ? rdu : old;
}

int zii_pic_comm_init(struct zii_pic *zp);
void zii_pic_comm_cleanup(struct zii_pic *zp);

/* reply_code = 0 means don't expect reply */
int zii_pic_exec(struct zii_pic *zp,
		u8 code, const u8 *data, u8 data_size,
		u8 reply_code, u8 *reply, u8 reply_size);

int zii_pic_set_event_handler(struct zii_pic *zp,
		u8 event_code, u8 reply_code,
		zii_pic_eh handler, void *context);

void zii_pic_cleanup_event_handler(struct zii_pic *zp, u8 event_code);

/* Special handling for sending reset command:
 * - call zii_pic_prepare_for_reset() when scheduling is still available,
 * - call zii_pic_do_reset() when already in machine_restart()
 */
void zii_pic_prepare_for_reset(struct zii_pic *zp);
void zii_pic_exec_reset(struct zii_pic *zp,
		u8 code, const u8 *data, u8 data_size);

#define ZII_PIC_NAME_WATCHDOG		"pic-watchdog"
#define ZII_PIC_NAME_HWMON		"pic-hwmon"
#define ZII_PIC_NAME_EEPROM		"pic-eeprom"
#define ZII_PIC_NAME_MAIN_EEPROM	"pic-main-eeprom"
#define ZII_PIC_NAME_DDS_EEPROM		"pic-dds-eeprom"
#define ZII_PIC_NAME_PWRBUTTON		"pic-pwrbutton"
#define ZII_PIC_NAME_BACKLIGHT		"pic-backlight"
#define ZII_PIC_NAME_LEDS		"pic-leds"

#endif /* _LINUX_ZII_PIC_H_ */
