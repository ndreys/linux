/*
 * rave-sp-hwmon.c - HWMON driver for sensors attached to Zodiac Inflight
 * Innovations RAVE Supervisory Processor
 *
 * Copyright (C) 2017-2018 Nikita Yushchenko <nikita.yoush@cogentembedded.com>
 *
 * based on work by Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/rave-sp.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

static const char *rdu1_in[] = {
	"28V"
};
static const char *rdu1_temp[] = {
	"TEMPERATURE",
	"TEMPERATURE_2"
};
static const char *rdu1_curr[] = {
	"BACKLIGHT_CURRENT"
};

static const char *rdu2_in[] = {
	"RMB_3V3_PMIC",
	"RMB_3V3_MCU",
	"RMB_5V_MAIN",
	"RMB_12V_MAIN",
	"RMB_28V_FIL",
	"RMB_28V_HOTSWAP",
	"DEB_1V8",
	"DEB_3V3",
	"DEB_28V_DEB",
	"DEB_28V_RDU"
};
static const char *rdu2_temp[] = {
	"TEMPERATURE_RMB",
	"TEMPERATURE_DEB"
};
static const char *rdu2_curr[] = {
	"RMB_28V_CURRENT"
};

#define chan_config(_name, _arr, _val)			\
	static const u32 _name[] = {			\
		[0 ... ARRAY_SIZE(_arr) - 1] = (_val),	\
		[ARRAY_SIZE(_arr)] = 0			\
	}

#define chan_info(_name, _type, _config)		\
	static const struct hwmon_channel_info _name = {\
		.type = _type,				\
		.config = _config,			\
	}

#define chan_info_list(_name, arg...)			\
	static const struct hwmon_channel_info *_name[] = { \
		arg,					\
		NULL					\
	}

chan_config(rdu1_in_config, rdu1_in, HWMON_I_INPUT | HWMON_I_LABEL);
chan_config(rdu1_temp_config, rdu1_temp, HWMON_T_INPUT | HWMON_T_LABEL);
chan_config(rdu1_curr_config, rdu1_curr, HWMON_C_INPUT | HWMON_C_LABEL);

chan_info(rdu1_in_info, hwmon_in, rdu1_in_config);
chan_info(rdu1_temp_info, hwmon_temp, rdu1_temp_config);
chan_info(rdu1_curr_info, hwmon_curr, rdu1_curr_config);

chan_info_list(rdu1_chan_info, &rdu1_in_info, &rdu1_temp_info, &rdu1_curr_info);

chan_config(rdu2_in_config, rdu2_in, HWMON_I_INPUT | HWMON_I_LABEL);
chan_config(rdu2_temp_config, rdu2_temp, HWMON_T_INPUT | HWMON_T_LABEL);
chan_config(rdu2_curr_config, rdu2_curr, HWMON_C_INPUT | HWMON_C_LABEL);

chan_info(rdu2_in_info, hwmon_in, rdu2_in_config);
chan_info(rdu2_temp_info, hwmon_temp, rdu2_temp_config);
chan_info(rdu2_curr_info, hwmon_curr, rdu2_curr_config);

chan_info_list(rdu2_chan_info, &rdu2_in_info, &rdu2_temp_info, &rdu2_curr_info);


static inline unsigned int le_f88_to_milli(__le16 le_f88)
{
	return ((unsigned int)le16_to_cpu(le_f88)) * 1000 / 256;
}

static inline int le_halfdg_to_mdg(__le16 le_halfdg)
{
	return ((int)(s16)le16_to_cpu(le_halfdg)) * 500;
}

static umode_t rave_sp_hwmon_is_visible(const void *data,
		enum hwmon_sensor_types type, u32 attr, int channel)
{
	return 0444;
}

static int rdu1_read(struct device *dev, enum hwmon_sensor_types type,
		u32 attr, int channel, long *val)
{
	struct rave_sp *sp = dev_get_drvdata(dev);
	struct rave_sp_status status;
	int ret;

	ret = rave_sp_get_status(sp, &status);
	if (ret)
		return ret;

	switch (type) {
	case hwmon_in:
		*val = le_f88_to_milli(status.voltage_28);
		return 0;
	case hwmon_temp:
		*val = le_halfdg_to_mdg(status.temp[channel]);
		return 0;
	case hwmon_curr:
		*val = le16_to_cpu(status.backlight_current);
		return 0;
	default:
		return -EINVAL;
	}
}

static int rdu1_read_string(struct device *dev, enum hwmon_sensor_types type,
		u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_in:
		*str = rdu1_in[channel];
		return 0;
	case hwmon_temp:
		*str = rdu1_temp[channel];
		return 0;
	case hwmon_curr:
		*str = rdu1_curr[channel];
		return 0;
	default:
		return -EINVAL;
	}
}

static int rdu2_read(struct device *dev, enum hwmon_sensor_types type,
		u32 attr, int channel, long *val)
{
	struct rave_sp *sp = dev_get_drvdata(dev);
	u8 cmd[3];
	__le16 rsp;
	int ret;

	switch (type) {
	case hwmon_in:
		cmd[0] = RAVE_SP_CMD_GET_VOLTAGE;
		break;
	case hwmon_temp:
		cmd[0] = RAVE_SP_CMD_GET_TEMPERATURE;
		break;
	case hwmon_curr:
		cmd[0] = RAVE_SP_CMD_GET_CURRENT;
		break;
	default:
		return -EINVAL;
	}

	cmd[2] = channel;

	ret = rave_sp_exec(sp, cmd, sizeof(cmd), &rsp, sizeof(rsp));
	if (ret)
		return ret;

	if (type == hwmon_temp)
		*val = le_halfdg_to_mdg(rsp);
	else
		*val = le_f88_to_milli(rsp);

	return 0;
}

static int rdu2_read_string(struct device *dev, enum hwmon_sensor_types type,
		u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_in:
		*str = rdu2_in[channel];
		return 0;
	case hwmon_temp:
		*str = rdu2_temp[channel];
		return 0;
	case hwmon_curr:
		*str = rdu2_curr[channel];
		return 0;
	default:
		return -EINVAL;
	}
}

static struct hwmon_ops rdu1_hwmon_ops = {
	.is_visible = rave_sp_hwmon_is_visible,
	.read = rdu1_read,
	.read_string = rdu1_read_string,
};


static struct hwmon_ops rdu2_hwmon_ops = {
	.is_visible = rave_sp_hwmon_is_visible,
	.read = rdu2_read,
	.read_string = rdu2_read_string,
};

static struct hwmon_chip_info rdu1_hwmon_chip_info = {
	.ops = &rdu1_hwmon_ops,
	.info = rdu1_chan_info,
};

static struct hwmon_chip_info rdu2_hwmon_chip_info = {
	.ops = &rdu2_hwmon_ops,
	.info = rdu2_chan_info,
};

static const struct of_device_id rave_sp_hwmon_of_match[] = {
	{
		.compatible = "zii,rave-sp-hwmon-rdu1",
		.data = &rdu1_hwmon_chip_info,
	},
	{
		.compatible = "zii,rave-sp-hwmon-rdu2",
		.data = &rdu2_hwmon_chip_info,
	},
	{ /* sentinel */ }
};

static int rave_sp_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev, *hwmon_dev;
	const struct hwmon_chip_info *info = of_device_get_match_data(dev);
	struct rave_sp *sp = dev_get_drvdata(dev->parent);

	hwmon_dev = devm_hwmon_device_register_with_info(dev, KBUILD_MODNAME,
			sp, info, NULL);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static struct platform_driver rave_sp_hwmon_driver = {
	.probe = rave_sp_hwmon_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = rave_sp_hwmon_of_match,
	},
};

module_platform_driver(rave_sp_hwmon_driver);

MODULE_DEVICE_TABLE(of, rave_sp_hwmon_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("HWMON driver for RAVE SP connected sensors");
MODULE_ALIAS("platform:rave-sp-hwmon");
