#ifndef _LINUX_ZII_PIC_H_
#define _LINUX_ZII_PIC_H_

enum zii_pic_command {
	ZII_PIC_CMD_GET_FIRMWARE_VERSION	= 0x20,
	ZII_PIC_CMD_GET_BOOTLOADER_VERSION	= 0x21,
	ZII_PIC_CMD_BOOT_SOURCE			= 0x26,
	ZII_PIC_CMD_GET_BOARD_COPPER_REV	= 0x2B,
	ZII_PIC_CMD_STATUS			= 0xA0,
	ZII_PIC_CMD_SW_WDT			= 0xA1,
	ZII_PIC_CMD_PET_WDT			= 0xA2,
	ZII_PIC_CMD_RESET			= 0xA7,
	ZII_PIC_CMD_RESET_REASON		= 0xA8,

	ZII_PIC_CMD_REQ_COPPER_REV		= 0xB6,

	ZII_PIC_EVNT_BASE			= 0xE0,
};

struct zii_pic;

static inline unsigned long zii_pic_action(u8 event, u8 value)
{
	return ((unsigned long)value << 8) | event;
}

static inline u8 zii_pic_action_get_event(unsigned long action)
{
	return action & 0xff;
}

static inline u8 zii_pic_action_get_value(unsigned long action)
{
	return (action >> 8) & 0xff;
}

extern int zii_pic_exec(struct zii_pic *, void *, size_t, void *, size_t);
extern int devm_zii_pic_register_event_notifier(struct device *, struct notifier_block *);

#define COMPATIBLE_ZII_PIC_NIU		"zii,pic-niu"
#define COMPATIBLE_ZII_PIC_MEZZ		"zii,pic-mezz"
#define COMPATIBLE_ZII_PIC_ESB		"zii,pic-esb"
#define COMPATIBLE_ZII_PIC_RDU1		"zii,pic-rdu1"
#define COMPATIBLE_ZII_PIC_RDU2		"zii,pic-rdu2"

#endif /* _LINUX_ZII_PIC_H_ */
