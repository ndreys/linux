// SPDX-License-Identifier: GPL-2.0+

/*
 * Driver for watchdog aspect of for Zodiac Inflight Innovations RAVE
 * Supervisory Processor(SP) MCU
 *
 * Copyright (C) 2017 Zodiac Inflight Innovation
 *
 */

#include <linux/delay.h>
#include <linux/ihex.h>
#include <linux/kernel.h>
#include <linux/mfd/rave-sp.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/watchdog.h>

enum {
	RAVE_SP_RESET_BYTE = 1,
	RAVE_SP_RESET_REASON_NORMAL = 0,
	RAVE_SP_RESET_DELAY_MS = 500,
};

/**
 * struct rave_sp_wdt_variant - RAVE SP watchdog variant
 *
 * @max_timeout:	Largest possible watchdog timeout setting
 * @min_timeout:	Smallest possible watchdog timeout setting
 *
 * @configure:		Function to send configuration command
 * @restart:		Function to send "restart" command
 */
struct rave_sp_wdt_variant {
	unsigned int max_timeout;
	unsigned int min_timeout;

	int (*configure)(struct watchdog_device *, bool);
	int (*restart)(struct watchdog_device *);
};

/**
 * struct rave_sp_wdt - RAVE SP watchdog
 *
 * @wdd:		Underlying watchdog device
 * @sp:			Pointer to parent RAVE SP device
 * @variant:		Device specific variant information
 * @reboot_notifier:	Reboot notifier implementing machine reset
 */
struct rave_sp_wdt {
	struct watchdog_device wdd;
	struct rave_sp *sp;
	const struct rave_sp_wdt_variant *variant;
	struct notifier_block reboot_notifier;

	struct {
		u32 start, end;

		size_t size;
		size_t progress;
	} update;
};

struct rave_sp_booloader_query_device_response {
	u8  command;
	u8  device_type;
	struct rave_sp_version bootloader;
	u8  is_app_valid;
	u16 app_crc;
	u32 app_start_addr;
	u32 config_words_start;
	/*
	 * The rest of payload is not used, but send over the wire
	 * regardless so the total size of the payload is 62 byte and
	 * the size of all the fields above is 19, hence the size of
	 * our padding.
	 */
	u8  __padding[64 - 19];
} __packed;

static struct rave_sp_wdt *to_rave_sp_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct rave_sp_wdt, wdd);
}

static int rave_sp_wdt_exec(struct watchdog_device *wdd, void *data,
			    size_t data_size)
{
	return rave_sp_exec(to_rave_sp_wdt(wdd)->sp,
			    data, data_size, NULL, 0);
}

static int rave_sp_wdt_legacy_configure(struct watchdog_device *wdd, bool on)
{
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_SW_WDT,
		[1] = 0,
		[2] = 0,
		[3] = on,
		[4] = on ? wdd->timeout : 0,
	};

	return rave_sp_wdt_exec(wdd, cmd, sizeof(cmd));
}

static int rave_sp_wdt_rdu_configure(struct watchdog_device *wdd, bool on)
{
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_SW_WDT,
		[1] = 0,
		[2] = on,
		[3] = (u8)wdd->timeout,
		[4] = (u8)(wdd->timeout >> 8),
	};

	return rave_sp_wdt_exec(wdd, cmd, sizeof(cmd));
}

/**
 * rave_sp_wdt_configure - Configure watchdog device
 *
 * @wdd:	Device to configure
 * @on:		Desired state of the watchdog timer (ON/OFF)
 *
 * This function configures two aspects of the watchdog timer:
 *
 *  - Wheither it is ON or OFF
 *  - Its timeout duration
 *
 * with first aspect specified via function argument and second via
 * the value of 'wdd->timeout'.
 */
static int rave_sp_wdt_configure(struct watchdog_device *wdd, bool on)
{
	return to_rave_sp_wdt(wdd)->variant->configure(wdd, on);
}

static int rave_sp_wdt_legacy_restart(struct watchdog_device *wdd)
{
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_RESET,
		[1] = 0,
		[2] = RAVE_SP_RESET_BYTE
	};

	return rave_sp_wdt_exec(wdd, cmd, sizeof(cmd));
}

static int rave_sp_wdt_rdu_restart(struct watchdog_device *wdd)
{
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_RESET,
		[1] = 0,
		[2] = RAVE_SP_RESET_BYTE,
		[3] = RAVE_SP_RESET_REASON_NORMAL
	};

	return rave_sp_wdt_exec(wdd, cmd, sizeof(cmd));
}

static int rave_sp_wdt_reboot_notifier(struct notifier_block *nb,
				       unsigned long action, void *data)
{
	/*
	 * Restart handler is called in atomic context which means we
	 * can't communicate to SP via UART. Luckily for use SP will
	 * wait 500ms before actually resetting us, so we ask it to do
	 * so here and let the rest of the system go on wrapping
	 * things up.
	 */
	if (action == SYS_DOWN || action == SYS_HALT) {
		struct rave_sp_wdt *sp_wd =
			container_of(nb, struct rave_sp_wdt, reboot_notifier);

		const int ret = sp_wd->variant->restart(&sp_wd->wdd);

		if (ret < 0)
			dev_err(sp_wd->wdd.parent,
				"Failed to issue restart command (%d)", ret);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int rave_sp_wdt_restart(struct watchdog_device *wdd,
			       unsigned long action, void *data)
{
	/*
	 * The actual work was done by reboot notifier above. SP
	 * firmware waits 500 ms before issuing reset, so let's hang
	 * here for twice that delay and hopefuly we'd never reach
	 * the return statement.
	 */
	mdelay(2 * RAVE_SP_RESET_DELAY_MS);

	return -EIO;
}

static int rave_sp_wdt_start(struct watchdog_device *wdd)
{
	int ret;

	ret = rave_sp_wdt_configure(wdd, true);
	if (!ret)
		set_bit(WDOG_HW_RUNNING, &wdd->status);

	return ret;
}

static int rave_sp_wdt_stop(struct watchdog_device *wdd)
{
	return rave_sp_wdt_configure(wdd, false);
}

static int rave_sp_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int timeout)
{
	wdd->timeout = timeout;

	return rave_sp_wdt_configure(wdd, watchdog_active(wdd));
}

static int rave_sp_wdt_ping(struct watchdog_device *wdd)
{
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_PET_WDT,
		[1] = 0,
	};

	return rave_sp_wdt_exec(wdd, cmd, sizeof(cmd));
}

static const struct watchdog_info rave_sp_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "RAVE SP Watchdog",
};

static const struct watchdog_ops rave_sp_wdt_ops = {
	.owner = THIS_MODULE,
	.start = rave_sp_wdt_start,
	.stop = rave_sp_wdt_stop,
	.ping = rave_sp_wdt_ping,
	.set_timeout = rave_sp_wdt_set_timeout,
	.restart = rave_sp_wdt_restart,
};

static const struct rave_sp_wdt_variant rave_sp_wdt_legacy = {
	.max_timeout = 255,
	.min_timeout = 1,
	.configure = rave_sp_wdt_legacy_configure,
	.restart   = rave_sp_wdt_legacy_restart,
};

static const struct rave_sp_wdt_variant rave_sp_wdt_rdu = {
	.max_timeout = 180,
	.min_timeout = 60,
	.configure = rave_sp_wdt_rdu_configure,
	.restart   = rave_sp_wdt_rdu_restart,
};

static const struct of_device_id rave_sp_wdt_of_match[] = {
	{
		.compatible = "zii,rave-sp-watchdog-legacy",
		.data = &rave_sp_wdt_legacy,
	},
	{
		.compatible = "zii,rave-sp-watchdog",
		.data = &rave_sp_wdt_rdu,
	},
	{ /* sentinel */ }
};

int rave_sp_wdt_query_device(struct rave_sp_wdt *sp_wd)
{
	struct rave_sp_booloader_query_device_response r;
	struct device *dev = sp_wd->wdd.parent;
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_BOOTLOADER,
		[1] = 0,
		[2] = RAVE_SP_BOOTLOADER_CMD_QUERY_DEVICE,
	};
	int ret;

	ret = rave_sp_exec(sp_wd->sp, cmd, sizeof(cmd), &r, sizeof(r));
	if (ret) {
		return ret;
	}

	sp_wd->update.start = r.app_start_addr;
	sp_wd->update.end   = r.config_words_start;

	dev_dbg(dev, "device type:         %02x\n", r.device_type);
	dev_dbg(dev, "bootloader hardware: %02d\n", r.bootloader.hardware);
	dev_dbg(dev, "bootloader major:    %02d\n", r.bootloader.major);
	dev_dbg(dev, "bootloader minor:    %02d\n", r.bootloader.minor);
	dev_dbg(dev, "bootloader letters:  %c%c\n", r.bootloader.letter[0],
		r.bootloader.letter[1]);
	dev_dbg(dev, "is app valid:        %02x\n", r.is_app_valid);
	dev_dbg(dev, "app crc:             %04x\n", r.app_crc);
	dev_dbg(dev, "app start addr:      %08x\n", r.app_start_addr);
	dev_dbg(dev, "config words start:  %08x\n", r.config_words_start);

	return 0;
}

#define RAVE_SP_WDT_FIRMWARE_DATA_PAYLOAD_SIZE	56

struct rave_sp_wdt_firmware_data {
	u8 command;
	u32 address;
	u8 size;
	u8 payload[RAVE_SP_WDT_FIRMWARE_DATA_PAYLOAD_SIZE];
} __packed;

static int rave_sp_wdt_program_complete(struct rave_sp_wdt *sp_wd)
{
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_BOOTLOADER,
		[1] = 0,
		[2] = RAVE_SP_BOOTLOADER_CMD_PROGRAM_COMPLETE,
	};
	int ret;

	ret = rave_sp_exec(sp_wd->sp, cmd, sizeof(cmd), NULL, 0);
	if (ret)
		return ret;

	rave_sp_set_update_fw_status(sp_wd->sp, 100);
	return 0;
}

static int
rave_sp_wdt_program_ihex_record(struct rave_sp_wdt *sp_wd,
				const struct ihex_binrec *rec)
{
	const u32 firmware_start = sp_wd->update.start;
	const u32 firmware_end   = sp_wd->update.end;

	u8 cmd[2 + sizeof(struct rave_sp_wdt_firmware_data)] = {
		[0] = RAVE_SP_CMD_BOOTLOADER,
		[1] = 0,
	};
	struct rave_sp_wdt_firmware_data *fwdata = (void *)&cmd[2];
	size_t residue = be16_to_cpu(rec->len);
	u32 address    = be32_to_cpu(rec->addr);
	const u8 *data = rec->data;
	size_t chunk;
	int ret;

	do {
		size_t status;
		/*
		 * First, program firmware data
		 */
		chunk = min(residue, sizeof(fwdata->payload));

		fwdata->command = RAVE_SP_BOOTLOADER_CMD_PROGRAM_DEVICE;
		/*
		 * Not really an address, but a word index
		 */
		fwdata->address = address / 2;
		fwdata->size    = chunk;

		memcpy(fwdata->payload, data, chunk);

		print_hex_dump_debug("programmed chunk: ", DUMP_PREFIX_NONE,
				     16, 1, data, chunk, false);

		ret = rave_sp_exec(sp_wd->sp, cmd, sizeof(cmd), NULL, 0);
		if (ret)
			return ret;

		if (firmware_start <= address && address <= firmware_end) {
			/*
			 * Second, read it back and verify contents
			 */
			fwdata->command = RAVE_SP_BOOTLOADER_CMD_READ_APP;

			ret = rave_sp_exec(sp_wd->sp, cmd,
					   sizeof(cmd) -
					   sizeof(fwdata->payload),
					   fwdata,
					   sizeof(fwdata->command) +
					   sizeof(fwdata->size) +
					   sizeof(fwdata->address) +
					   chunk);
			if (ret)
				return ret;

			print_hex_dump_debug("read chunk: ", DUMP_PREFIX_NONE,
					     16, 1, fwdata->payload,
					     fwdata->size, false);

			if (memcmp(data, fwdata->payload, chunk)) {
				dev_err(sp_wd->wdd.parent,
					"Programmed data doesn't match\n");
				return -EIO;
			}
		}

		residue -= chunk;
		address += chunk;
		data    += chunk;

		sp_wd->update.progress += chunk;
		status = (sp_wd->update.progress * 100) / sp_wd->update.size;
		/*
		 * We only go up to 99 here and reserve 100 to
		 * indicate completion of the last step of firmware
		 * update process done by
		 * rave_sp_wdt_program_complete()
		 */
		status = min(status, 99U);

		rave_sp_set_update_fw_status(sp_wd->sp, status);
	} while (residue);

	return 0;
}

static int rave_sp_wdt_program_app(struct rave_sp_wdt *sp_wd,
				   const struct firmware *firmware)
{
	const struct ihex_binrec *rec;

	sp_wd->update.size = 0;
	sp_wd->update.progress = 0;

	rave_sp_set_update_fw_status(sp_wd->sp, 0);

	for (rec = (void *)firmware->data; rec; rec = ihex_next_binrec(rec))
		sp_wd->update.size += be16_to_cpu(rec->len);

	for (rec = (void *)firmware->data; rec; rec = ihex_next_binrec(rec)) {
		const int ret = rave_sp_wdt_program_ihex_record(sp_wd, rec);
		if (ret)
			return ret;
	}

	return rave_sp_wdt_program_complete(sp_wd);
}

static int rave_sp_wdt_erase_app(struct rave_sp_wdt *sp_wd)
{
	u8 cmd[] = {
		[0] = RAVE_SP_CMD_BOOTLOADER,
		[1] = 0,
		[2] = RAVE_SP_BOOTLOADER_CMD_ERASE_APP,
	};

	return rave_sp_exec(sp_wd->sp, cmd, sizeof(cmd), NULL, 0);
}

static int rave_sp_wdt_update_firmware(struct rave_sp_wdt *sp_wd,
				       const struct firmware *firmware)
{
	struct device *dev = sp_wd->wdd.parent;
	int ret;

	dev_dbg(dev, "Starting firmware update\n");

	dev_dbg(dev, "Stopping watchdog\n");
	ret = rave_sp_wdt_stop(&sp_wd->wdd);
	if (ret) {
		dev_err(dev, "Failed to stop watchdog\n");
		return ret;
	}

	ret = rave_sp_wdt_query_device(sp_wd);
	if (ret) {
		dev_err(dev, "Failed to query device\n");
		return ret;
	}

	dev_dbg(dev, "Erasing device\n");
	ret = rave_sp_wdt_erase_app(sp_wd);
	if (ret) {
		dev_err(dev, "Failed to erase device\n");
		goto release_fw;
	}

	dev_dbg(dev, "Programming device\n");
	ret = rave_sp_wdt_program_app(sp_wd, firmware);
	if (ret)
		dev_err(dev, "Failed to program device\n");

release_fw:
	rave_sp_release_firmware(sp_wd->sp);
	dev_dbg(dev, "Firmware update finished\n");
	return ret;
}

static int rave_sp_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct watchdog_device *wdd;
	struct rave_sp_wdt *sp_wd;
	struct nvmem_cell *cell;
	__le16 timeout = 0;
	int ret;

	sp_wd = devm_kzalloc(dev, sizeof(*sp_wd), GFP_KERNEL);
	if (!sp_wd)
		return -ENOMEM;

	sp_wd->variant = of_device_get_match_data(dev);
	sp_wd->sp      = dev_get_drvdata(dev->parent);

	wdd              = &sp_wd->wdd;
	wdd->parent      = dev;
	wdd->info        = &rave_sp_wdt_info;
	wdd->ops         = &rave_sp_wdt_ops;
	wdd->min_timeout = sp_wd->variant->min_timeout;
	wdd->max_timeout = sp_wd->variant->max_timeout;
	wdd->status      = WATCHDOG_NOWAYOUT_INIT_STATUS;
	wdd->timeout     = 60;

	dev_set_drvdata(dev, sp_wd);

	if (rave_sp_is_in_bootloader_mode(sp_wd->sp)) {
		const struct firmware *firmware;

		firmware = rave_sp_get_firmware(sp_wd->sp);
		if (firmware) {
			ret = rave_sp_wdt_update_firmware(sp_wd, firmware);
			if (ret)
				return ret;
		} else {
			dev_warn(dev, "Bootloader mode, but no "
				 "firmware file is given\n");
		}
	}

	cell = nvmem_cell_get(dev, "wdt-timeout");
	if (!IS_ERR(cell)) {
		size_t len;
		void *value = nvmem_cell_read(cell, &len);

		if (!IS_ERR(value)) {
			memcpy(&timeout, value, min(len, sizeof(timeout)));
			kfree(value);
		}
		nvmem_cell_put(cell);
	}
	watchdog_init_timeout(wdd, le16_to_cpu(timeout), dev);
	watchdog_set_restart_priority(wdd, 255);
	watchdog_stop_on_unregister(wdd);

	sp_wd->reboot_notifier.notifier_call = rave_sp_wdt_reboot_notifier;
	ret = devm_register_reboot_notifier(dev, &sp_wd->reboot_notifier);
	if (ret) {
		dev_err(dev, "Failed to register reboot notifier\n");
		return ret;
	}

	/*
	 * We don't know if watchdog is running now. To be sure, let's
	 * start it and depend on watchdog core to ping it
	 */
	wdd->max_hw_heartbeat_ms = wdd->max_timeout * 1000;
	ret = rave_sp_wdt_start(wdd);
	if (ret) {
		dev_err(dev, "Watchdog didn't start\n");
		return ret;
	}

	ret = devm_watchdog_register_device(dev, wdd);
	if (ret) {
		dev_err(dev, "Failed to register watchdog device\n");
		rave_sp_wdt_stop(wdd);
		return ret;
	}

	return 0;
}

static struct platform_driver rave_sp_wdt_driver = {
	.probe = rave_sp_wdt_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = rave_sp_wdt_of_match,
	},
};

module_platform_driver(rave_sp_wdt_driver);

MODULE_DEVICE_TABLE(of, rave_sp_wdt_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_AUTHOR("Andrey Smirnov <andrew.smirnov@gmail.com>");
MODULE_DESCRIPTION("RAVE SP Watchdog driver");
MODULE_ALIAS("platform:rave-sp-watchdog");
