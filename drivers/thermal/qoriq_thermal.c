// SPDX-License-Identifier: GPL-2.0
//
// Copyright 2016 Freescale Semiconductor, Inc.

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#include "thermal_core.h"
#include "thermal_hwmon.h"

#define SITES_MAX	16

/*
 * QorIQ TMU Registers
 */

#define REGS_TMR	0x000	/* Mode Register */
#define TMR_DISABLE	0x0
#define TMR_ME		0x80000000
#define TMR_ALPF	0x0c000000
#define TMR_MSITE_ALL	GENMASK(15, 0)

#define REGS_TMTMIR	0x008	/* Temperature measurement interval Register */
#define TMTMIR_DEFAULT	0x0000000f

#define REGS_TIER	0x020	/* Interrupt Enable Register */
#define TIER_DISABLE	0x0

#define REGS_TTCFGR	0x080	/* Temperature Configuration Register */
#define REGS_TSCFGR	0x084	/* Sensor Configuration Register */

#define REGS_TRITSR(n)	(0x100 + 16 * (n)) /* Immediate Temperature
					    * Site Register
					    */
#define TRITSR_V	BIT(31)
#define REGS_TTRnCR(n)	(0xf10 + 4 * (n)) /* Temperature Range n
					   * Control Register
					   */
/*
 * Thermal zone data
 */
struct qoriq_sensor {
	int				id;
};

struct qoriq_tmu_data {
	struct regmap *regmap;
	struct qoriq_sensor	sensor[SITES_MAX];
};

static struct qoriq_tmu_data *qoriq_sensor_to_data(struct qoriq_sensor *s)
{
	return container_of(s, struct qoriq_tmu_data, sensor[s->id]);
}

static int tmu_get_temp(void *p, int *temp)
{
	struct qoriq_sensor *qsensor = p;
	struct qoriq_tmu_data *qdata = qoriq_sensor_to_data(qsensor);
	u32 val;
	int ret;

	ret = regmap_read_poll_timeout(qdata->regmap,
				       REGS_TRITSR(qsensor->id),
				       val,
				       val & TRITSR_V,
				       USEC_PER_MSEC,
				       10 * USEC_PER_MSEC);
	if (ret)
		return ret;

	*temp = (val & 0xff) * 1000;
	return 0;
}

static const struct thermal_zone_of_device_ops tmu_tz_ops = {
	.get_temp = tmu_get_temp,
};

static int qoriq_tmu_register_tmu_zone(struct device *dev,
				       struct qoriq_tmu_data *qdata)
{
	int id, ret;

	regmap_write(qdata->regmap, REGS_TMR,
		     TMR_MSITE_ALL | TMR_ME | TMR_ALPF);

	for (id = 0; id < SITES_MAX; id++) {
		struct thermal_zone_device *tzd;
		struct qoriq_sensor *s = &qdata->sensor[id];

		s->id = id;

		tzd = devm_thermal_zone_of_sensor_register(dev, id,
							   s, &tmu_tz_ops);
		ret = PTR_ERR_OR_ZERO(tzd);
		switch (ret) {
		case -ENODEV:
			continue;
		case 0:
			ret = devm_thermal_add_hwmon_sysfs(tzd);
			if (!ret)
				break;
			/* fallthrough */
		default:
			regmap_write(qdata->regmap, REGS_TMR, TMR_DISABLE);
			return ret;
		}
	}

	return 0;
}

static int qoriq_tmu_calibration(struct device *dev,
				 struct qoriq_tmu_data *data)
{
	int i, val, len;
	u32 range[4];
	const u32 *calibration;
	struct device_node *np = dev->of_node;

	if (of_property_read_u32_array(np, "fsl,tmu-range", range, 4)) {
		dev_err(dev, "missing calibration range.\n");
		return -ENODEV;
	}

	/* Init temperature range registers */
	for (i = 0; i < ARRAY_SIZE(range); i++)
		regmap_write(data->regmap, REGS_TTRnCR(i), range[i]);

	calibration = of_get_property(np, "fsl,tmu-calibration", &len);
	if (calibration == NULL || len % 8) {
		dev_err(dev, "invalid calibration data.\n");
		return -ENODEV;
	}

	for (i = 0; i < len; i += 8, calibration += 2) {
		val = of_read_number(calibration, 1);
		regmap_write(data->regmap, REGS_TTCFGR, val);
		val = of_read_number(calibration + 1, 1);
		regmap_write(data->regmap, REGS_TSCFGR, val);
	}

	return 0;
}

static void qoriq_tmu_init_device(struct qoriq_tmu_data *data)
{
	/* Disable interrupt, using polling instead */
	regmap_write(data->regmap, REGS_TIER, TIER_DISABLE);

	/* Set update_interval */
	regmap_write(data->regmap, REGS_TMTMIR, TMTMIR_DEFAULT);

	/* Disable monitoring */
	regmap_write(data->regmap, REGS_TMR, TMR_DISABLE);
}

static const struct regmap_range qiriq_yes_ranges[] = {
	regmap_reg_range(REGS_TMR, REGS_TSCFGR),
	regmap_reg_range(REGS_TTRnCR(0), REGS_TTRnCR(3)),
	/* Read only registers below */
	regmap_reg_range(REGS_TRITSR(0), REGS_TRITSR(15)),
};

static const struct regmap_access_table qiriq_wr_table = {
	.yes_ranges	= qiriq_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(qiriq_yes_ranges) - 1,
};

static const struct regmap_access_table qiriq_rd_table = {
	.yes_ranges	= qiriq_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(qiriq_yes_ranges),
};

static int qoriq_tmu_probe(struct platform_device *pdev)
{
	int ret;
	struct qoriq_tmu_data *data;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct resource *io;
	const bool little_endian = of_property_read_bool(np, "little-endian");
	const enum regmap_endian format_endian =
		little_endian ? REGMAP_ENDIAN_LITTLE : REGMAP_ENDIAN_BIG;
	const struct regmap_config regmap_config = {
		.reg_bits		= 32,
		.val_bits		= 32,
		.reg_stride		= 4,
		.rd_table		= &qiriq_rd_table,
		.wr_table		= &qiriq_wr_table,
		.val_format_endian	= format_endian,
		.max_register		= SZ_4K,
	};
	void __iomem *base;

	data = devm_kzalloc(dev, sizeof(struct qoriq_tmu_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io) {
		dev_err(dev, "Failed to get memory region\n");
		return -ENODEV;
	}

	base = devm_ioremap(dev, io->start, resource_size(io));
	if (!base) {
		dev_err(dev, "Failed to get memory region\n");
		return -ENODEV;
	}

	data->regmap = devm_regmap_init_mmio(dev, base, &regmap_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(dev, "Failed to init regmap (%d)\n", ret);
		return ret;
	}

	qoriq_tmu_init_device(data);	/* TMU initialization */

	ret = qoriq_tmu_calibration(dev, data);	/* TMU calibration */
	if (ret < 0)
		return ret;

	ret = qoriq_tmu_register_tmu_zone(dev, data);
	if (ret < 0) {
		dev_err(dev, "Failed to register sensors\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, data);

	return 0;
}

static int qoriq_tmu_remove(struct platform_device *pdev)
{
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev);

	/* Disable monitoring */
	regmap_write(data->regmap, REGS_TMR, TMR_DISABLE);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int qoriq_tmu_suspend_resume(struct device *dev, unsigned int val)
{
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);

	return regmap_update_bits(data->regmap, REGS_TMR, TMR_ME, val);
}

static int qoriq_tmu_suspend(struct device *dev)
{
	return qoriq_tmu_suspend_resume(dev, 0);
}

static int qoriq_tmu_resume(struct device *dev)
{
	return qoriq_tmu_suspend_resume(dev, TMR_ME);
}
#endif

static SIMPLE_DEV_PM_OPS(qoriq_tmu_pm_ops,
			 qoriq_tmu_suspend, qoriq_tmu_resume);

static const struct of_device_id qoriq_tmu_match[] = {
	{ .compatible = "fsl,qoriq-tmu", },
	{ .compatible = "fsl,imx8mq-tmu", },
	{},
};
MODULE_DEVICE_TABLE(of, qoriq_tmu_match);

static struct platform_driver qoriq_tmu = {
	.driver	= {
		.name		= "qoriq_thermal",
		.pm		= &qoriq_tmu_pm_ops,
		.of_match_table	= qoriq_tmu_match,
	},
	.probe	= qoriq_tmu_probe,
	.remove	= qoriq_tmu_remove,
};
module_platform_driver(qoriq_tmu);

MODULE_AUTHOR("Jia Hongtao <hongtao.jia@nxp.com>");
MODULE_DESCRIPTION("QorIQ Thermal Monitoring Unit driver");
MODULE_LICENSE("GPL v2");
