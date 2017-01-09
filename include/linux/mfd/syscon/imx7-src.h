/*
 * Copyright (C) 2017 Impinj
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_IMX7_SRC_H
#define __LINUX_IMX7_SRC_H

#define SRC_PCIEPHY_RCR		0x2c

#define IMX7D_SRC_PCIEPHY_RCR_PCIEPHY_G_RST	BIT(1)
#define IMX7D_SRC_PCIEPHY_RCR_PCIEPHY_BTN	BIT(2)
#define IMX7D_SRC_PCIEPHY_RCR_PCIE_CTRL_APPS_EN	BIT(6)

#endif /* __LINUX_IMX7_IOMUXC_GPR_H */
