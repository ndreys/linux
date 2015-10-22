/*
 * Marvell 88e6xxx common definitions
 *
 * Copyright (c) 2008 Marvell Semiconductor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MV88E6XXX_H
#define __MV88E6XXX_H

#include <linux/if_vlan.h>
#include <linux/gpio/consumer.h>

#ifndef UINT64_MAX
#define UINT64_MAX		(u64)(~((u64)0))
#endif

#define SMI_CMD			0x00
#define SMI_CMD_BUSY		BIT(15)
#define SMI_CMD_CLAUSE_22	BIT(12)
#define SMI_CMD_OP_22_WRITE	((1 << 10) | SMI_CMD_BUSY | SMI_CMD_CLAUSE_22)
#define SMI_CMD_OP_22_READ	((2 << 10) | SMI_CMD_BUSY | SMI_CMD_CLAUSE_22)
#define SMI_CMD_OP_45_WRITE_ADDR	((0 << 10) | SMI_CMD_BUSY)
#define SMI_CMD_OP_45_WRITE_DATA	((1 << 10) | SMI_CMD_BUSY)
#define SMI_CMD_OP_45_READ_DATA		((2 << 10) | SMI_CMD_BUSY)
#define SMI_CMD_OP_45_READ_DATA_INC	((3 << 10) | SMI_CMD_BUSY)
#define SMI_DATA		0x01

/* PHY Registers */
#define PHY_PAGE		0x16
#define PHY_PAGE_COPPER		0x00

#define ADDR_SERDES		0x0f
#define SERDES_PAGE_FIBER	0x01

#define REG_PORT(p)		(0x10 + (p))
#define PORT_STATUS		0x00
#define PORT_STATUS_PAUSE_EN	BIT(15)
#define PORT_STATUS_MY_PAUSE	BIT(14)
#define PORT_STATUS_HD_FLOW	BIT(13)
#define PORT_STATUS_PHY_DETECT	BIT(12)
#define PORT_STATUS_LINK	BIT(11)
#define PORT_STATUS_DUPLEX	BIT(10)
#define PORT_STATUS_SPEED_MASK	0x0300
#define PORT_STATUS_SPEED_10	0x0000
#define PORT_STATUS_SPEED_100	0x0100
#define PORT_STATUS_SPEED_1000	0x0200
#define PORT_STATUS_EEE		BIT(6) /* 6352 */
#define PORT_STATUS_AM_DIS	BIT(6) /* 6165 */
#define PORT_STATUS_MGMII	BIT(6) /* 6185 */
#define PORT_STATUS_TX_PAUSED	BIT(5)
#define PORT_STATUS_FLOW_CTRL	BIT(4)
#define PORT_STATUS_CMODE_MASK	0x0f
#define PORT_STATUS_CMODE_100BASE_X	0x8
#define PORT_STATUS_CMODE_1000BASE_X	0x9
#define PORT_STATUS_CMODE_SGMII		0xa
#define PORT_PCS_CTRL		0x01
#define PORT_PCS_CTRL_RGMII_DELAY_RXCLK	BIT(15)
#define PORT_PCS_CTRL_RGMII_DELAY_TXCLK	BIT(14)
#define PORT_PCS_CTRL_FC		BIT(7)
#define PORT_PCS_CTRL_FORCE_FC		BIT(6)
#define PORT_PCS_CTRL_LINK_UP		BIT(5)
#define PORT_PCS_CTRL_FORCE_LINK	BIT(4)
#define PORT_PCS_CTRL_DUPLEX_FULL	BIT(3)
#define PORT_PCS_CTRL_FORCE_DUPLEX	BIT(2)
#define PORT_PCS_CTRL_10		0x00
#define PORT_PCS_CTRL_100		0x01
#define PORT_PCS_CTRL_1000		0x02
#define PORT_PCS_CTRL_UNFORCED		0x03
#define PORT_PAUSE_CTRL		0x02
#define PORT_SWITCH_ID		0x03
#define PORT_SWITCH_ID_PROD_NUM_6085	0x04a
#define PORT_SWITCH_ID_PROD_NUM_6095	0x095
#define PORT_SWITCH_ID_PROD_NUM_6131	0x106
#define PORT_SWITCH_ID_PROD_NUM_6320	0x115
#define PORT_SWITCH_ID_PROD_NUM_6123	0x121
#define PORT_SWITCH_ID_PROD_NUM_6161	0x161
#define PORT_SWITCH_ID_PROD_NUM_6165	0x165
#define PORT_SWITCH_ID_PROD_NUM_6171	0x171
#define PORT_SWITCH_ID_PROD_NUM_6172	0x172
#define PORT_SWITCH_ID_PROD_NUM_6175	0x175
#define PORT_SWITCH_ID_PROD_NUM_6176	0x176
#define PORT_SWITCH_ID_PROD_NUM_6185	0x1a7
#define PORT_SWITCH_ID_PROD_NUM_6240	0x240
#define PORT_SWITCH_ID_PROD_NUM_6321	0x310
#define PORT_SWITCH_ID_PROD_NUM_6352	0x352
#define PORT_SWITCH_ID_PROD_NUM_6350	0x371
#define PORT_SWITCH_ID_PROD_NUM_6351	0x375
#define PORT_CONTROL		0x04
#define PORT_CONTROL_USE_CORE_TAG	BIT(15)
#define PORT_CONTROL_DROP_ON_LOCK	BIT(14)
#define PORT_CONTROL_EGRESS_UNMODIFIED	(0x0 << 12)
#define PORT_CONTROL_EGRESS_UNTAGGED	(0x1 << 12)
#define PORT_CONTROL_EGRESS_TAGGED	(0x2 << 12)
#define PORT_CONTROL_EGRESS_ADD_TAG	(0x3 << 12)
#define PORT_CONTROL_HEADER		BIT(11)
#define PORT_CONTROL_IGMP_MLD_SNOOP	BIT(10)
#define PORT_CONTROL_DOUBLE_TAG		BIT(9)
#define PORT_CONTROL_FRAME_MODE_NORMAL		(0x0 << 8)
#define PORT_CONTROL_FRAME_MODE_DSA		(0x1 << 8)
#define PORT_CONTROL_FRAME_MODE_PROVIDER	(0x2 << 8)
#define PORT_CONTROL_FRAME_ETHER_TYPE_DSA	(0x3 << 8)
#define PORT_CONTROL_DSA_TAG		BIT(8)
#define PORT_CONTROL_VLAN_TUNNEL	BIT(7)
#define PORT_CONTROL_TAG_IF_BOTH	BIT(6)
#define PORT_CONTROL_USE_IP		BIT(5)
#define PORT_CONTROL_USE_TAG		BIT(4)
#define PORT_CONTROL_FORWARD_UNKNOWN_MC	BIT(3)
#define PORT_CONTROL_FORWARD_UNKNOWN	BIT(2)
#define PORT_CONTROL_STATE_MASK		0x03
#define PORT_CONTROL_STATE_DISABLED	0x00
#define PORT_CONTROL_STATE_BLOCKING	0x01
#define PORT_CONTROL_STATE_LEARNING	0x02
#define PORT_CONTROL_STATE_FORWARDING	0x03
#define PORT_CONTROL_1		0x05
#define PORT_CONTROL_1_FID_11_4_MASK	(0xff << 0)
#define PORT_BASE_VLAN		0x06
#define PORT_BASE_VLAN_FID_3_0_MASK	(0xf << 12)
#define PORT_DEFAULT_VLAN	0x07
#define PORT_DEFAULT_VLAN_MASK	0xfff
#define PORT_CONTROL_2		0x08
#define PORT_CONTROL_2_IGNORE_FCS	BIT(15)
#define PORT_CONTROL_2_VTU_PRI_OVERRIDE	BIT(14)
#define PORT_CONTROL_2_SA_PRIO_OVERRIDE	BIT(13)
#define PORT_CONTROL_2_DA_PRIO_OVERRIDE	BIT(12)
#define PORT_CONTROL_2_JUMBO_1522	(0x00 << 12)
#define PORT_CONTROL_2_JUMBO_2048	(0x01 << 12)
#define PORT_CONTROL_2_JUMBO_10240	(0x02 << 12)
#define PORT_CONTROL_2_8021Q_MASK	(0x03 << 10)
#define PORT_CONTROL_2_8021Q_DISABLED	(0x00 << 10)
#define PORT_CONTROL_2_8021Q_FALLBACK	(0x01 << 10)
#define PORT_CONTROL_2_8021Q_CHECK	(0x02 << 10)
#define PORT_CONTROL_2_8021Q_SECURE	(0x03 << 10)
#define PORT_CONTROL_2_DISCARD_TAGGED	BIT(9)
#define PORT_CONTROL_2_DISCARD_UNTAGGED	BIT(8)
#define PORT_CONTROL_2_MAP_DA		BIT(7)
#define PORT_CONTROL_2_DEFAULT_FORWARD	BIT(6)
#define PORT_CONTROL_2_FORWARD_UNKNOWN	BIT(6)
#define PORT_CONTROL_2_EGRESS_MONITOR	BIT(5)
#define PORT_CONTROL_2_INGRESS_MONITOR	BIT(4)
#define PORT_RATE_CONTROL	0x09
#define PORT_RATE_CONTROL_2	0x0a
#define PORT_ASSOC_VECTOR	0x0b
#define PORT_ASSOC_VECTOR_HOLD_AT_1		BIT(15)
#define PORT_ASSOC_VECTOR_INT_AGE_OUT		BIT(14)
#define PORT_ASSOC_VECTOR_LOCKED_PORT		BIT(13)
#define PORT_ASSOC_VECTOR_IGNORE_WRONG		BIT(12)
#define PORT_ASSOC_VECTOR_REFRESH_LOCKED	BIT(11)
#define PORT_ATU_CONTROL	0x0c
#define PORT_PRI_OVERRIDE	0x0d
#define PORT_PRI_OVERRIDE_TRAP_TCAM_MISS	BIT(4)
#define PORT_PRI_OVERRIDE_TCAM_DISABLE		(0x0 << 0)
#define PORT_PRI_OVERRIDE_TCAM_48_BYTES		(0x1 << 0)
#define PORT_PRI_OVERRIDE_TCAM_96_BYTES		(0x2 << 0)
#define PORT_PRI_OVERRIDE_TCAM_MASK		(0x3 << 0)
#define PORT_ETH_TYPE		0x0f
#define PORT_IN_DISCARD_LO	0x10
#define PORT_IN_DISCARD_HI	0x11
#define PORT_IN_FILTERED	0x12
#define PORT_OUT_FILTERED	0x13
#define PORT_TAG_REGMAP_0123	0x18
#define PORT_TAG_REGMAP_4567	0x19

#define REG_GLOBAL		0x1b
#define GLOBAL_STATUS		0x00
#define GLOBAL_STATUS_PPU_STATE BIT(15) /* 6351 and 6171 */
/* Two bits for 6165, 6185 etc */
#define GLOBAL_STATUS_PPU_MASK		(0x3 << 14)
#define GLOBAL_STATUS_PPU_DISABLED_RST	(0x0 << 14)
#define GLOBAL_STATUS_PPU_INITIALIZING	(0x1 << 14)
#define GLOBAL_STATUS_PPU_DISABLED	(0x2 << 14)
#define GLOBAL_STATUS_PPU_POLLING	(0x3 << 14)
#define GLOBAL_MAC_01		0x01
#define GLOBAL_MAC_23		0x02
#define GLOBAL_MAC_45		0x03
#define GLOBAL_ATU_FID		0x01	/* 6097 6165 6351 6352 */
#define GLOBAL_VTU_FID		0x02	/* 6097 6165 6351 6352 */
#define GLOBAL_VTU_FID_MASK	0xfff
#define GLOBAL_VTU_SID		0x03	/* 6097 6165 6351 6352 */
#define GLOBAL_VTU_SID_MASK	0x3f
#define GLOBAL_CONTROL		0x04
#define GLOBAL_CONTROL_SW_RESET		BIT(15)
#define GLOBAL_CONTROL_PPU_ENABLE	BIT(14)
#define GLOBAL_CONTROL_DISCARD_EXCESS	BIT(13) /* 6352 */
#define GLOBAL_CONTROL_SCHED_PRIO	BIT(11) /* 6152 */
#define GLOBAL_CONTROL_MAX_FRAME_1632	BIT(10) /* 6152 */
#define GLOBAL_CONTROL_RELOAD_EEPROM	BIT(9)	/* 6152 */
#define GLOBAL_CONTROL_DEVICE_EN	BIT(7)
#define GLOBAL_CONTROL_STATS_DONE_EN	BIT(6)
#define GLOBAL_CONTROL_VTU_PROBLEM_EN	BIT(5)
#define GLOBAL_CONTROL_VTU_DONE_EN	BIT(4)
#define GLOBAL_CONTROL_ATU_PROBLEM_EN	BIT(3)
#define GLOBAL_CONTROL_ATU_DONE_EN	BIT(2)
#define GLOBAL_CONTROL_TCAM_EN		BIT(1)
#define GLOBAL_CONTROL_EEPROM_DONE_EN	BIT(0)
#define GLOBAL_VTU_OP		0x05
#define GLOBAL_VTU_OP_BUSY	BIT(15)
#define GLOBAL_VTU_OP_FLUSH_ALL		((0x01 << 12) | GLOBAL_VTU_OP_BUSY)
#define GLOBAL_VTU_OP_VTU_LOAD_PURGE	((0x03 << 12) | GLOBAL_VTU_OP_BUSY)
#define GLOBAL_VTU_OP_VTU_GET_NEXT	((0x04 << 12) | GLOBAL_VTU_OP_BUSY)
#define GLOBAL_VTU_OP_STU_LOAD_PURGE	((0x05 << 12) | GLOBAL_VTU_OP_BUSY)
#define GLOBAL_VTU_OP_STU_GET_NEXT	((0x06 << 12) | GLOBAL_VTU_OP_BUSY)
#define GLOBAL_VTU_VID		0x06
#define GLOBAL_VTU_VID_MASK	0xfff
#define GLOBAL_VTU_VID_VALID	BIT(12)
#define GLOBAL_VTU_DATA_0_3	0x07
#define GLOBAL_VTU_DATA_4_7	0x08
#define GLOBAL_VTU_DATA_8_11	0x09
#define GLOBAL_VTU_STU_DATA_MASK		0x03
#define GLOBAL_VTU_DATA_MEMBER_TAG_UNMODIFIED	0x00
#define GLOBAL_VTU_DATA_MEMBER_TAG_UNTAGGED	0x01
#define GLOBAL_VTU_DATA_MEMBER_TAG_TAGGED	0x02
#define GLOBAL_VTU_DATA_MEMBER_TAG_NON_MEMBER	0x03
#define GLOBAL_STU_DATA_PORT_STATE_DISABLED	0x00
#define GLOBAL_STU_DATA_PORT_STATE_BLOCKING	0x01
#define GLOBAL_STU_DATA_PORT_STATE_LEARNING	0x02
#define GLOBAL_STU_DATA_PORT_STATE_FORWARDING	0x03
#define GLOBAL_ATU_CONTROL	0x0a
#define GLOBAL_ATU_CONTROL_LEARN2ALL	BIT(3)
#define GLOBAL_ATU_OP		0x0b
#define GLOBAL_ATU_OP_BUSY	BIT(15)
#define GLOBAL_ATU_OP_NOP		(0 << 12)
#define GLOBAL_ATU_OP_FLUSH_MOVE_ALL		((1 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_FLUSH_MOVE_NON_STATIC	((2 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_LOAD_DB		((3 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_GET_NEXT_DB	((4 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_FLUSH_MOVE_ALL_DB		((5 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_FLUSH_MOVE_NON_STATIC_DB ((6 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_OP_GET_CLR_VIOLATION	  ((7 << 12) | GLOBAL_ATU_OP_BUSY)
#define GLOBAL_ATU_DATA		0x0c
#define GLOBAL_ATU_DATA_TRUNK			BIT(15)
#define GLOBAL_ATU_DATA_TRUNK_ID_MASK		0x00f0
#define GLOBAL_ATU_DATA_TRUNK_ID_SHIFT		4
#define GLOBAL_ATU_DATA_PORT_VECTOR_MASK	0x3ff0
#define GLOBAL_ATU_DATA_PORT_VECTOR_SHIFT	4
#define GLOBAL_ATU_DATA_STATE_MASK		0x0f
#define GLOBAL_ATU_DATA_STATE_UNUSED		0x00
#define GLOBAL_ATU_DATA_STATE_UC_MGMT		0x0d
#define GLOBAL_ATU_DATA_STATE_UC_STATIC		0x0e
#define GLOBAL_ATU_DATA_STATE_UC_PRIO_OVER	0x0f
#define GLOBAL_ATU_DATA_STATE_MC_NONE_RATE	0x05
#define GLOBAL_ATU_DATA_STATE_MC_STATIC		0x07
#define GLOBAL_ATU_DATA_STATE_MC_MGMT		0x0e
#define GLOBAL_ATU_DATA_STATE_MC_PRIO_OVER	0x0f
#define GLOBAL_ATU_MAC_01	0x0d
#define GLOBAL_ATU_MAC_23	0x0e
#define GLOBAL_ATU_MAC_45	0x0f
#define GLOBAL_IP_PRI_0		0x10
#define GLOBAL_IP_PRI_1		0x11
#define GLOBAL_IP_PRI_2		0x12
#define GLOBAL_IP_PRI_3		0x13
#define GLOBAL_IP_PRI_4		0x14
#define GLOBAL_IP_PRI_5		0x15
#define GLOBAL_IP_PRI_6		0x16
#define GLOBAL_IP_PRI_7		0x17
#define GLOBAL_IEEE_PRI		0x18
#define GLOBAL_CORE_TAG_TYPE	0x19
#define GLOBAL_MONITOR_CONTROL	0x1a
#define GLOBAL_MONITOR_CONTROL_INGRESS_SHIFT	12
#define GLOBAL_MONITOR_CONTROL_EGRESS_SHIFT	8
#define GLOBAL_MONITOR_CONTROL_ARP_SHIFT	4
#define GLOBAL_MONITOR_CONTROL_MIRROR_SHIFT	0
#define GLOBAL_MONITOR_CONTROL_ARP_DISABLED	(0xf0)
#define GLOBAL_CONTROL_2	0x1c
#define GLOBAL_CONTROL_2_NO_CASCADE		0xe000
#define GLOBAL_CONTROL_2_MULTIPLE_CASCADE	0xf000

#define GLOBAL_STATS_OP		0x1d
#define GLOBAL_STATS_OP_BUSY	BIT(15)
#define GLOBAL_STATS_OP_NOP		(0 << 12)
#define GLOBAL_STATS_OP_FLUSH_ALL	((1 << 12) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_FLUSH_PORT	((2 << 12) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_READ_CAPTURED	((4 << 12) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_CAPTURE_PORT	((5 << 12) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_HIST_RX		((1 << 10) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_HIST_TX		((2 << 10) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_HIST_RX_TX	((3 << 10) | GLOBAL_STATS_OP_BUSY)
#define GLOBAL_STATS_OP_BANK_1	BIT(9)
#define GLOBAL_STATS_COUNTER_32	0x1e
#define GLOBAL_STATS_COUNTER_01	0x1f

#define REG_GLOBAL2		0x1c
#define GLOBAL2_INT_SOURCE	0x00
#define GLOBAL2_INT_MASK	0x01
#define GLOBAL2_MGMT_EN_2X	0x02
#define GLOBAL2_MGMT_EN_0X	0x03
#define GLOBAL2_FLOW_CONTROL	0x04
#define GLOBAL2_SWITCH_MGMT	0x05
#define GLOBAL2_SWITCH_MGMT_USE_DOUBLE_TAG_DATA	BIT(15)
#define GLOBAL2_SWITCH_MGMT_PREVENT_LOOPS	BIT(14)
#define GLOBAL2_SWITCH_MGMT_FLOW_CONTROL_MSG	BIT(13)
#define GLOBAL2_SWITCH_MGMT_FORCE_FLOW_CTRL_PRI	BIT(7)
#define GLOBAL2_SWITCH_MGMT_RSVD2CPU		BIT(3)
#define GLOBAL2_DEVICE_MAPPING	0x06
#define GLOBAL2_DEVICE_MAPPING_UPDATE		BIT(15)
#define GLOBAL2_DEVICE_MAPPING_TARGET_SHIFT	8
#define GLOBAL2_DEVICE_MAPPING_PORT_MASK	0x0f
#define GLOBAL2_TRUNK_MASK	0x07
#define GLOBAL2_TRUNK_MASK_UPDATE		BIT(15)
#define GLOBAL2_TRUNK_MASK_NUM_SHIFT		12
#define GLOBAL2_TRUNK_MASK_HASK			BIT(11)
#define GLOBAL2_TRUNK_MAPPING	0x08
#define GLOBAL2_TRUNK_MAPPING_UPDATE		BIT(15)
#define GLOBAL2_TRUNK_MAPPING_ID_SHIFT		11
#define GLOBAL2_IRL_CMD		0x09
#define GLOBAL2_IRL_CMD_BUSY	BIT(15)
#define GLOBAL2_IRL_CMD_OP_INIT_ALL	((0x001 << 12) | GLOBAL2_IRL_CMD_BUSY)
#define GLOBAL2_IRL_CMD_OP_INIT_SEL	((0x010 << 12) | GLOBAL2_IRL_CMD_BUSY)
#define GLOBAL2_IRL_CMD_OP_WRITE_SEL	((0x011 << 12) | GLOBAL2_IRL_CMD_BUSY)
#define GLOBAL2_IRL_CMD_OP_READ_SEL	((0x100 << 12) | GLOBAL2_IRL_CMD_BUSY)
#define GLOBAL2_IRL_DATA	0x0a
#define GLOBAL2_PVT_ADDR	0x0b
#define GLOBAL2_PVT_ADDR_BUSY	BIT(15)
#define GLOBAL2_PVT_ADDR_OP_INIT_ONES	((0x01 << 12) | GLOBAL2_PVT_ADDR_BUSY)
#define GLOBAL2_PVT_ADDR_OP_WRITE_PVLAN	((0x03 << 12) | GLOBAL2_PVT_ADDR_BUSY)
#define GLOBAL2_PVT_ADDR_OP_READ	((0x04 << 12) | GLOBAL2_PVT_ADDR_BUSY)
#define GLOBAL2_PVT_DATA	0x0c
#define GLOBAL2_SWITCH_MAC	0x0d
#define GLOBAL2_ATU_STATS	0x0e
#define GLOBAL2_PRIO_OVERRIDE	0x0f
#define GLOBAL2_PRIO_OVERRIDE_FORCE_SNOOP	BIT(7)
#define GLOBAL2_PRIO_OVERRIDE_SNOOP_SHIFT	4
#define GLOBAL2_PRIO_OVERRIDE_FORCE_ARP		BIT(3)
#define GLOBAL2_PRIO_OVERRIDE_ARP_SHIFT		0
#define GLOBAL2_EEPROM_CMD		0x14
#define GLOBAL2_EEPROM_CMD_BUSY		BIT(15)
#define GLOBAL2_EEPROM_CMD_OP_WRITE	((0x3 << 12) | GLOBAL2_EEPROM_CMD_BUSY)
#define GLOBAL2_EEPROM_CMD_OP_READ	((0x4 << 12) | GLOBAL2_EEPROM_CMD_BUSY)
#define GLOBAL2_EEPROM_CMD_OP_LOAD	((0x6 << 12) | GLOBAL2_EEPROM_CMD_BUSY)
#define GLOBAL2_EEPROM_CMD_RUNNING	BIT(11)
#define GLOBAL2_EEPROM_CMD_WRITE_EN	BIT(10)
#define GLOBAL2_EEPROM_CMD_ADDR_MASK	0xff
#define GLOBAL2_EEPROM_DATA	0x15
#define GLOBAL2_PTP_AVB_OP	0x16
#define GLOBAL2_PTP_AVB_DATA	0x17
#define GLOBAL2_SMI_PHY_CMD			0x18
#define GLOBAL2_SMI_PHY_CMD_BUSY		BIT(15)
#define GLOBAL2_SMI_PHY_CMD_MODE_22		BIT(12)
#define GLOBAL2_SMI_PHY_CMD_OP_22_WRITE_DATA	((0x1 << 10) | \
						 GLOBAL2_SMI_PHY_CMD_MODE_22 | \
						 GLOBAL2_SMI_PHY_CMD_BUSY)
#define GLOBAL2_SMI_PHY_CMD_OP_22_READ_DATA	((0x2 << 10) | \
						 GLOBAL2_SMI_PHY_CMD_MODE_22 | \
						 GLOBAL2_SMI_PHY_CMD_BUSY)
#define GLOBAL2_SMI_PHY_DATA			0x19
#define GLOBAL2_SCRATCH_MISC	0x1a
#define GLOBAL2_SCRATCH_BUSY		BIT(15)
#define GLOBAL2_SCRATCH_REGISTER_SHIFT	8
#define GLOBAL2_SCRATCH_VALUE_MASK	0xff
#define GLOBAL2_WDOG_CONTROL	0x1b
#define GLOBAL2_QOS_WEIGHT	0x1c
#define GLOBAL2_MISC		0x1d

#define REG_GLOBAL3		0x1d
#define GLOBAL3_TCAM_OP			0x00
#define GLOBAL3_TCAM_OP_BUSY		BIT(15)
#define GLOBAL3_TCAM_OP_FLUSH_ALL	((1 << 12) | GLOBAL3_TCAM_OP_BUSY)
#define GLOBAL3_TCAM_OP_FLUSH_ENTRY	((2 << 12) | GLOBAL3_TCAM_OP_BUSY)
#define GLOBAL3_TCAM_OP_LOAD		((3 << 12) | GLOBAL3_TCAM_OP_BUSY)
#define GLOBAL3_TCAM_OP_GET_NEXT	((4 << 12) | GLOBAL3_TCAM_OP_BUSY)
#define GLOBAL3_TCAM_OP_READ		((5 << 12) | GLOBAL3_TCAM_OP_BUSY)
#define GLOBAL3_TCAM_OP_PAGE2		(0x2 << 10)
#define GLOBAL3_TCAM_OP_PAGE1		(0x1 << 10)
#define GLOBAL3_TCAM_OP_PAGE0		(0x0 << 10)
#define GLOBAL3_P0_KEY1		0x02
#define GLOBAL3_P0_KEY1_FRAME_TYPE_MASK		(3 << 14)
#define GLOBAL3_P0_KEY1_FRAME_TYPE_NORNAL	(0x0 << 6)
#define GLOBAL3_P0_KEY1_FRAME_TYPE_DSA		(0x1 << 6)
#define GLOBAL3_P0_KEY1_FRAME_TYPE_PROVIDER	(0x2 << 6)
#define GLOBAL3_P0_KEY2		0x03
#define GLOBAL3_P0_KEY2_SRC_PORT_VECTOR_MASK		(0x7f << 8)
#define GLOBAL3_P0_KEY2_SRC_PORT_VECTOR_MASK_SHIFT	8
#define GLOBAL3_P0_KEY2_SRC_PORT_VECTOR_VALUE_MASK	(0x7f)
#define GLOBAL3_P0_KEY3		0x04
#define GLOBAL3_P0_KEY3_PPRI_MASK			(0xf << 12)
#define GLOBAL3_P0_KEY3_PVID_MASK			(0xf << 8)
#define GLOBAL3_P0_KEY4		0x05
#define GLOBAL3_P0_KEY4_PVID_MASK			(0xff << 8)
#define GLOBAL3_P2_ACTION1	0x02
#define GLOBAL3_P2_ACTION1_CONTINUE		BIT(15)
#define GLOBAL3_P2_ACTION1_INTERRUPT		BIT(14)
#define GLOBAL3_P2_ACTION1_INC_TCAM_COUNTER	BIT(13)
#define GLOBAL3_P2_ACTION1_VID_OVERRIDE		BIT(12)
#define GLOBAL3_P2_ACTION1_VID_MASK		0x7ff
#define GLOBAL3_P2_ACTION2	0x3
#define GLOBAL3_P2_ACTION2_FLOW_ID_MASK		0xff00
#define GLOBAL3_P2_ACTION2_FLOW_ID_0		(0x0 << 8)
#define GLOBAL3_P2_ACTION2_FLOW_ID_1		(0x1 << 8)
#define GLOBAL3_P2_ACTION2_FLOW_ID_2		(0x2 << 8)
#define GLOBAL3_P2_ACTION2_FLOW_ID_3		(0x3 << 8)
#define GLOBAL3_P2_ACTION2_QPRI_OVERRIDE	BIT(7)
#define GLOBAL3_P2_ACTION2_QPRI_MASK		0xf0
#define GLOBAL3_P2_ACTION2_QPRI_0		(0x0 << 4)
#define GLOBAL3_P2_ACTION2_QPRI_1		(0x1 << 4)
#define GLOBAL3_P2_ACTION2_QPRI_2		(0x2 << 4)
#define GLOBAL3_P2_ACTION2_QPRI_3		(0x3 << 4)
#define GLOBAL3_P2_ACTION2_FPRI_OVERRIDE	BIT(3)
#define GLOBAL3_P2_ACTION2_FPRI_MASK		0x7
#define GLOBAL3_P2_ACTION2_FPRI_0		(0x0 << 0)
#define GLOBAL3_P2_ACTION2_FPRI_1		(0x1 << 0)
#define GLOBAL3_P2_ACTION2_FPRI_2		(0x2 << 0)
#define GLOBAL3_P2_ACTION2_FPRI_3		(0x3 << 0)
#define GLOBAL3_P2_ACTION3		0x04
#define GLOBAL3_P2_ACTION3_DPV_OVERRIDE		BIT(11)
#define GLOBAL3_P2_ACTION4		0x05
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_OVERRIDE	BIT(15)
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_SRC_IS_TAGGED	BIT(14)
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_PVID		BIT(13)
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_MGMT		BIT(12)
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_ARP		BIT(11)
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_SNOOP		BIT(10)
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_POLICY_MIRROR	BIT(9)
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_POLICY_TRAP	BIT(8)
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_SANRL		BIT(7)
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_DANRL		BIT(6)
#define GLOBAL3_P2_ACTION4_FRAME_ACTION_MASK		0xfff0
#define GLOBAL3_P2_ACTION4_LOAD_BALANCE_OVERRIDE	BIT(3)
#define GLOBAL3_P2_ACTION4_LOAD_BALANCE_MASK		0x7
#define GLOBAL3_P2_DEBUG28		0x1c
#define GLOBAL3_P2_DEBUG31		0x1f

#define MV88E6XXX_N_FID		4096

/* List of supported models */
enum mv88e6xxx_model {
	MV88E6085,
	MV88E6095,
	MV88E6123,
	MV88E6131,
	MV88E6161,
	MV88E6165,
	MV88E6171,
	MV88E6172,
	MV88E6175,
	MV88E6176,
	MV88E6185,
	MV88E6240,
	MV88E6320,
	MV88E6321,
	MV88E6350,
	MV88E6351,
	MV88E6352,
};

enum mv88e6xxx_family {
	MV88E6XXX_FAMILY_NONE,
	MV88E6XXX_FAMILY_6065,	/* 6031 6035 6061 6065 */
	MV88E6XXX_FAMILY_6095,	/* 6092 6095 */
	MV88E6XXX_FAMILY_6097,	/* 6046 6085 6096 6097 */
	MV88E6XXX_FAMILY_6165,	/* 6123 6161 6165 */
	MV88E6XXX_FAMILY_6185,	/* 6108 6121 6122 6131 6152 6155 6182 6185 */
	MV88E6XXX_FAMILY_6320,	/* 6320 6321 */
	MV88E6XXX_FAMILY_6351,	/* 6171 6175 6350 6351 */
	MV88E6XXX_FAMILY_6352,	/* 6172 6176 6240 6352 */
};

enum mv88e6xxx_cap {
	/* Two different tag protocols can be used by the driver. All
	 * switches support DSA, but only later generations support
	 * EDSA.
	 */
	MV88E6XXX_CAP_EDSA,

	/* Energy Efficient Ethernet.
	 */
	MV88E6XXX_CAP_EEE,

	/* Multi-chip Addressing Mode.
	 * Some chips respond to only 2 registers of its own SMI device address
	 * when it is non-zero, and use indirect access to internal registers.
	 */
	MV88E6XXX_CAP_SMI_CMD,		/* (0x00) SMI Command */
	MV88E6XXX_CAP_SMI_DATA,		/* (0x01) SMI Data */

	/* PHY Registers.
	 */
	MV88E6XXX_CAP_PHY_PAGE,		/* (0x16) Page Register */

	/* Fiber/SERDES Registers (SMI address F).
	 */
	MV88E6XXX_CAP_SERDES,

	/* Switch Global 2 Registers.
	 * The device contains a second set of global 16-bit registers.
	 */
	MV88E6XXX_CAP_GLOBAL2,
	MV88E6XXX_CAP_G2_MGMT_EN_2X,	/* (0x02) MGMT Enable Register 2x */
	MV88E6XXX_CAP_G2_MGMT_EN_0X,	/* (0x03) MGMT Enable Register 0x */
	MV88E6XXX_CAP_G2_IRL_CMD,	/* (0x09) Ingress Rate Command */
	MV88E6XXX_CAP_G2_IRL_DATA,	/* (0x0a) Ingress Rate Data */
	MV88E6XXX_CAP_G2_PVT_ADDR,	/* (0x0b) Cross Chip Port VLAN Addr */
	MV88E6XXX_CAP_G2_PVT_DATA,	/* (0x0c) Cross Chip Port VLAN Data */
	MV88E6XXX_CAP_G2_SWITCH_MAC,	/* (0x0d) Switch MAC/WoL/WoF */
	MV88E6XXX_CAP_G2_POT,		/* (0x0f) Priority Override Table */
	MV88E6XXX_CAP_G2_EEPROM_CMD,	/* (0x14) EEPROM Command */
	MV88E6XXX_CAP_G2_EEPROM_DATA,	/* (0x15) EEPROM Data */
	MV88E6XXX_CAP_G2_SMI_PHY_CMD,	/* (0x18) SMI PHY Command */
	MV88E6XXX_CAP_G2_SMI_PHY_DATA,	/* (0x19) SMI PHY Data */

	/* PHY Polling Unit.
	 * See GLOBAL_CONTROL_PPU_ENABLE and GLOBAL_STATUS_PPU_POLLING.
	 */
	MV88E6XXX_CAP_PPU,
	MV88E6XXX_CAP_PPU_ACTIVE,

	/* Per VLAN Spanning Tree Unit (STU).
	 * The Port State database, if present, is accessed through VTU
	 * operations and dedicated SID registers. See GLOBAL_VTU_SID.
	 */
	MV88E6XXX_CAP_STU,

	/* TCAM implementation */
	MV88E6XXX_CAP_TCAM,
	MV88E6XXX_CAP_TCAM_1,

	/* Internal temperature sensor.
	 * Available from any enabled port's PHY register 26, page 6.
	 */
	MV88E6XXX_CAP_TEMP,
	MV88E6XXX_CAP_TEMP_LIMIT,

	/* VLAN Table Unit.
	 * The VTU is used to program 802.1Q VLANs. See GLOBAL_VTU_OP.
	 */
	MV88E6XXX_CAP_VTU,
};

/* Bitmask of capabilities */
#define MV88E6XXX_FLAG_EDSA		BIT(MV88E6XXX_CAP_EDSA)
#define MV88E6XXX_FLAG_EEE		BIT(MV88E6XXX_CAP_EEE)

#define MV88E6XXX_FLAG_SMI_CMD		BIT(MV88E6XXX_CAP_SMI_CMD)
#define MV88E6XXX_FLAG_SMI_DATA		BIT(MV88E6XXX_CAP_SMI_DATA)

#define MV88E6XXX_FLAG_PHY_PAGE		BIT(MV88E6XXX_CAP_PHY_PAGE)

#define MV88E6XXX_FLAG_SERDES		BIT(MV88E6XXX_CAP_SERDES)

#define MV88E6XXX_FLAG_GLOBAL2		BIT(MV88E6XXX_CAP_GLOBAL2)
#define MV88E6XXX_FLAG_G2_MGMT_EN_2X	BIT(MV88E6XXX_CAP_G2_MGMT_EN_2X)
#define MV88E6XXX_FLAG_G2_MGMT_EN_0X	BIT(MV88E6XXX_CAP_G2_MGMT_EN_0X)
#define MV88E6XXX_FLAG_G2_IRL_CMD	BIT(MV88E6XXX_CAP_G2_IRL_CMD)
#define MV88E6XXX_FLAG_G2_IRL_DATA	BIT(MV88E6XXX_CAP_G2_IRL_DATA)
#define MV88E6XXX_FLAG_G2_PVT_ADDR	BIT(MV88E6XXX_CAP_G2_PVT_ADDR)
#define MV88E6XXX_FLAG_G2_PVT_DATA	BIT(MV88E6XXX_CAP_G2_PVT_DATA)
#define MV88E6XXX_FLAG_G2_SWITCH_MAC	BIT(MV88E6XXX_CAP_G2_SWITCH_MAC)
#define MV88E6XXX_FLAG_G2_POT		BIT(MV88E6XXX_CAP_G2_POT)
#define MV88E6XXX_FLAG_G2_EEPROM_CMD	BIT(MV88E6XXX_CAP_G2_EEPROM_CMD)
#define MV88E6XXX_FLAG_G2_EEPROM_DATA	BIT(MV88E6XXX_CAP_G2_EEPROM_DATA)
#define MV88E6XXX_FLAG_G2_SMI_PHY_CMD	BIT(MV88E6XXX_CAP_G2_SMI_PHY_CMD)
#define MV88E6XXX_FLAG_G2_SMI_PHY_DATA	BIT(MV88E6XXX_CAP_G2_SMI_PHY_DATA)

#define MV88E6XXX_FLAG_PPU		BIT(MV88E6XXX_CAP_PPU)
#define MV88E6XXX_FLAG_PPU_ACTIVE	BIT(MV88E6XXX_CAP_PPU_ACTIVE)
#define MV88E6XXX_FLAG_STU		BIT(MV88E6XXX_CAP_STU)
#define MV88E6XXX_FLAG_TCAM		BIT(MV88E6XXX_CAP_TCAM)
#define MV88E6XXX_FLAG_TCAM_1		BIT(MV88E6XXX_CAP_TCAM_1)
#define MV88E6XXX_FLAG_TEMP		BIT(MV88E6XXX_CAP_TEMP)
#define MV88E6XXX_FLAG_TEMP_LIMIT	BIT(MV88E6XXX_CAP_TEMP_LIMIT)
#define MV88E6XXX_FLAG_VTU		BIT(MV88E6XXX_CAP_VTU)

/* EEPROM Programming via Global2 with 16-bit data */
#define MV88E6XXX_FLAGS_EEPROM16	\
	(MV88E6XXX_FLAG_G2_EEPROM_CMD |	\
	 MV88E6XXX_FLAG_G2_EEPROM_DATA)

/* Ingress Rate Limit unit */
#define MV88E6XXX_FLAGS_IRL		\
	(MV88E6XXX_FLAG_G2_IRL_CMD |	\
	 MV88E6XXX_FLAG_G2_IRL_DATA)

/* Multi-chip Addressing Mode */
#define MV88E6XXX_FLAGS_MULTI_CHIP	\
	(MV88E6XXX_FLAG_SMI_CMD |	\
	 MV88E6XXX_FLAG_SMI_DATA)

/* Cross-chip Port VLAN Table */
#define MV88E6XXX_FLAGS_PVT		\
	(MV88E6XXX_FLAG_G2_PVT_ADDR |	\
	 MV88E6XXX_FLAG_G2_PVT_DATA)

/* Fiber/SERDES Registers at SMI address F, page 1 */
#define MV88E6XXX_FLAGS_SERDES		\
	(MV88E6XXX_FLAG_PHY_PAGE |	\
	 MV88E6XXX_FLAG_SERDES)

/* Indirect PHY access via Global2 SMI PHY registers */
#define MV88E6XXX_FLAGS_SMI_PHY		\
	(MV88E6XXX_FLAG_G2_SMI_PHY_CMD |\
	 MV88E6XXX_FLAG_G2_SMI_PHY_DATA)

#define MV88E6XXX_FLAGS_FAMILY_6095	\
	(MV88E6XXX_FLAG_GLOBAL2 |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_0X |	\
	 MV88E6XXX_FLAG_PPU |		\
	 MV88E6XXX_FLAG_VTU |		\
	 MV88E6XXX_FLAGS_MULTI_CHIP)

#define MV88E6XXX_FLAGS_FAMILY_6097	\
	(MV88E6XXX_FLAG_GLOBAL2 |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_2X |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_0X |	\
	 MV88E6XXX_FLAG_G2_POT |	\
	 MV88E6XXX_FLAG_PPU |		\
	 MV88E6XXX_FLAG_STU |		\
	 MV88E6XXX_FLAG_VTU |		\
	 MV88E6XXX_FLAGS_IRL |		\
	 MV88E6XXX_FLAGS_MULTI_CHIP |	\
	 MV88E6XXX_FLAGS_PVT)

#define MV88E6XXX_FLAGS_FAMILY_6165	\
	(MV88E6XXX_FLAG_GLOBAL2 |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_2X |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_0X |	\
	 MV88E6XXX_FLAG_G2_SWITCH_MAC |	\
	 MV88E6XXX_FLAG_G2_POT |	\
	 MV88E6XXX_FLAG_STU |		\
	 MV88E6XXX_FLAG_TEMP |		\
	 MV88E6XXX_FLAG_VTU |		\
	 MV88E6XXX_FLAGS_IRL |		\
	 MV88E6XXX_FLAGS_MULTI_CHIP |	\
	 MV88E6XXX_FLAGS_PVT)

#define MV88E6XXX_FLAGS_FAMILY_6185	\
	(MV88E6XXX_FLAG_GLOBAL2 |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_0X |	\
	 MV88E6XXX_FLAGS_MULTI_CHIP |	\
	 MV88E6XXX_FLAG_PPU |		\
	 MV88E6XXX_FLAG_VTU)

#define MV88E6XXX_FLAGS_FAMILY_6320	\
	(MV88E6XXX_FLAG_EDSA |		\
	 MV88E6XXX_FLAG_EEE |		\
	 MV88E6XXX_FLAG_GLOBAL2 |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_2X |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_0X |	\
	 MV88E6XXX_FLAG_G2_SWITCH_MAC |	\
	 MV88E6XXX_FLAG_G2_POT |	\
	 MV88E6XXX_FLAG_PPU_ACTIVE |	\
	 MV88E6XXX_FLAG_TEMP |		\
	 MV88E6XXX_FLAG_TEMP_LIMIT |	\
	 MV88E6XXX_FLAG_VTU |		\
	 MV88E6XXX_FLAGS_EEPROM16 |	\
	 MV88E6XXX_FLAGS_IRL |		\
	 MV88E6XXX_FLAGS_MULTI_CHIP |	\
	 MV88E6XXX_FLAGS_PVT |		\
	 MV88E6XXX_FLAGS_SMI_PHY)

#define MV88E6XXX_FLAGS_FAMILY_6351	\
	(MV88E6XXX_FLAG_EDSA |		\
	 MV88E6XXX_FLAG_GLOBAL2 |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_2X |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_0X |	\
	 MV88E6XXX_FLAG_G2_SWITCH_MAC |	\
	 MV88E6XXX_FLAG_G2_POT |	\
	 MV88E6XXX_FLAG_PPU_ACTIVE |	\
	 MV88E6XXX_FLAG_STU |		\
	 MV88E6XXX_FLAG_TCAM |		\
	 MV88E6XXX_FLAG_TEMP |		\
	 MV88E6XXX_FLAG_VTU |		\
	 MV88E6XXX_FLAGS_IRL |		\
	 MV88E6XXX_FLAGS_MULTI_CHIP |	\
	 MV88E6XXX_FLAGS_PVT |		\
	 MV88E6XXX_FLAGS_SMI_PHY)

#define MV88E6XXX_FLAGS_FAMILY_6352	\
	(MV88E6XXX_FLAG_EDSA |		\
	 MV88E6XXX_FLAG_EEE |		\
	 MV88E6XXX_FLAG_GLOBAL2 |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_2X |	\
	 MV88E6XXX_FLAG_G2_MGMT_EN_0X |	\
	 MV88E6XXX_FLAG_G2_SWITCH_MAC |	\
	 MV88E6XXX_FLAG_G2_POT |	\
	 MV88E6XXX_FLAG_PPU_ACTIVE |	\
	 MV88E6XXX_FLAG_STU |		\
	 MV88E6XXX_FLAG_TCAM |		\
	 MV88E6XXX_FLAG_TEMP |		\
	 MV88E6XXX_FLAG_TEMP_LIMIT |	\
	 MV88E6XXX_FLAG_VTU |		\
	 MV88E6XXX_FLAGS_EEPROM16 |	\
	 MV88E6XXX_FLAGS_IRL |		\
	 MV88E6XXX_FLAGS_MULTI_CHIP |	\
	 MV88E6XXX_FLAGS_PVT |		\
	 MV88E6XXX_FLAGS_SERDES |	\
	 MV88E6XXX_FLAGS_SMI_PHY)

struct mv88e6xxx_info {
	enum mv88e6xxx_family family;
	u16 prod_num;
	const char *name;
	unsigned int num_databases;
	unsigned int num_ports;
	unsigned int port_base_addr;
	unsigned int age_time_coeff;
	unsigned long flags;
};

struct mv88e6xxx_atu_entry {
	u16	fid;
	u8	state;
	bool	trunk;
	u16	portv_trunkid;
	u8	mac[ETH_ALEN];
};

struct mv88e6xxx_vtu_stu_entry {
	/* VTU only */
	u16	vid;
	u16	fid;

	/* VTU and STU */
	u8	sid;
	bool	valid;
	u8	data[DSA_MAX_PORTS];
};

struct	mv88e6xxx_tcam_data {
	u16 page0[32];
	u16 page1[32];
	u16 page2[32];
};

enum mv88e6xxx_tcam_param {
	MV88E6XXX_P0_KEY1_FRAME_TYPE,
	MV88E6XXX_P0_KEY2_SRC_PORT_VECTOR,
	MV88E6XXX_P0_KEY3_PPRI,
	MV88E6XXX_P0_KEY4_PVID,
	MV88E6XXX_P2_ACTION1_INTERRUPT,
	MV88E6XXX_P2_ACTION1_INC_TCAM_COUNTER,
	MV88E6XXX_P2_ACTION1_VID,
	MV88E6XXX_P2_ACTION2_FLOW_ID,
	MV88E6XXX_P2_ACTION2_QPRI,
	MV88E6XXX_P2_ACTION2_FPRI,
	MV88E6XXX_P2_ACTION3_DST_PORT_VECTOR,
	MV88E6XXX_P2_ACTION4_FRAME_ACTION,
	MV88E6XXX_P2_ACTION4_LOAD_BALANCE,
	MV88E6XXX_P2_DEBUG_PORT,
	MV88E6XXX_P2_DEBUG_HIT,
};

#define MV88E6XXX_TCAM_PARAM_DISABLED INT_MIN

struct mv88e6xxx_ops;

struct mv88e6xxx_priv_port {
	struct net_device *bridge_dev;
};

struct mv88e6xxx_chip {
	const struct mv88e6xxx_info *info;

	/* The dsa_switch this private structure is related to */
	struct dsa_switch *ds;

	/* The device this structure is associated to */
	struct device *dev;

	/* This mutex protects the access to the switch registers */
	struct mutex reg_lock;

	/* The MII bus and the address on the bus that is used to
	 * communication with the switch
	 */
	const struct mv88e6xxx_ops *smi_ops;
	struct mii_bus *bus;
	int sw_addr;

	/* Handles automatic disabling and re-enabling of the PHY
	 * polling unit.
	 */
	const struct mv88e6xxx_ops *phy_ops;
	struct mutex		ppu_mutex;
	int			ppu_disabled;
	struct work_struct	ppu_work;
	struct timer_list	ppu_timer;

	/* This mutex serialises access to the statistics unit.
	 * Hold this mutex over snapshot + dump sequences.
	 */
	struct mutex	stats_mutex;

	struct mv88e6xxx_priv_port	ports[DSA_MAX_PORTS];

	/* A switch may have a GPIO line tied to its reset pin. Parse
	 * this from the device tree, and use it before performing
	 * switch soft reset.
	 */
	struct gpio_desc *reset;

	/* set to size of eeprom if supported by the switch */
	int		eeprom_len;

	/* Device node for the MDIO bus */
	struct device_node *mdio_np;

	/* And the MDIO bus itself */
	struct mii_bus *mdio_bus;

	struct dentry *dbgfs;
};

struct mv88e6xxx_ops {
	int (*read)(struct mv88e6xxx_chip *chip, int addr, int reg, u16 *val);
	int (*write)(struct mv88e6xxx_chip *chip, int addr, int reg, u16 val);
};

enum stat_type {
	BANK0,
	BANK1,
	PORT,
};

struct mv88e6xxx_hw_stat {
	char string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int reg;
	enum stat_type type;
};

static inline bool mv88e6xxx_has(struct mv88e6xxx_chip *chip,
				 unsigned long flags)
{
	return (chip->info->flags & flags) == flags;
}

/* chip.c */
int mv88e6xxx_read(struct mv88e6xxx_chip *chip,
		   int addr, int reg, u16 *val);
int mv88e6xxx_write(struct mv88e6xxx_chip *chip,
		    int addr, int reg, u16 val);
int mv88e6xxx_port_read(struct mv88e6xxx_chip *chip, int port,
			int reg, u16 *val);
int mv88e6xxx_port_write(struct mv88e6xxx_chip *chip, int port,
			 int reg, u16 val);
int mv88e6xxx_wait(struct mv88e6xxx_chip *chip, int addr, int reg,
		   u16 mask);

/* tcam.c */
#ifdef CONFIG_NET_DSA_MV88E6XXX_TCAM
int mv88e6xxx_tcam_get_next(struct mv88e6xxx_chip *chip, u32 *entry,
			    struct mv88e6xxx_tcam_data *data);
int mv88e6xxx_tcam_read(struct mv88e6xxx_chip *chip, u32 entry,
			struct mv88e6xxx_tcam_data *data);
int mv88e6xxx_tcam_load_entry(struct mv88e6xxx_chip *chip, u32 entry,
			      struct mv88e6xxx_tcam_data *data);
int mv88e6xxx_tcam_flush_entry(struct mv88e6xxx_chip *chip, u32 entry);
int mv88e6xxx_tcam_flush_all(struct mv88e6xxx_chip *chip);
int mv88e6xxx_tcam_port_disable(struct mv88e6xxx_chip *chip, int port);
int mv88e6xxx_tcam_port_enable(struct mv88e6xxx_chip *chip, int port);
int mv88e6xxx_tcam_set(struct mv88e6xxx_chip *chip,
		       struct mv88e6xxx_tcam_data *data,
		       enum mv88e6xxx_tcam_param param, u16 value);
int mv88e6xxx_tcam_get(struct mv88e6xxx_chip *chip,
		       struct mv88e6xxx_tcam_data *data,
		       enum mv88e6xxx_tcam_param param, int *value);
int mv88e6xxx_tcam_get_match(struct mv88e6xxx_chip *chip,
			     struct mv88e6xxx_tcam_data *data,
			     unsigned int offset, u8 *octet, u8 *mask);
int mv88e6xxx_tcam_set_match(struct mv88e6xxx_chip *chip,
			     struct mv88e6xxx_tcam_data *data,
			     unsigned int offset, u8 octet, u8 mask);
#else
int mv88e6xxx_tcam_get_next(struct mv88e6xxx_chip *chip, u32 *entry,
			    struct mv88e6xxx_tcam_data *data)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_tcam_read(struct mv88e6xxx_chip *chip, u32 entry,
			struct mv88e6xxx_tcam_data *data)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_tcam_load_entry(struct mv88e6xxx_chip *chip, u32 entry,
			      struct mv88e6xxx_tcam_data *data)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_tcam_flush_entry(struct mv88e6xxx_chip *chip, u32 entry)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_tcam_flush_all(struct mv88e6xxx_chip *chip)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_tcam_port_disable(struct mv88e6xxx_chip *chip, int port)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_tcam_port_enable(struct mv88e6xxx_chip *chip, int port)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_tcam_set(struct mv88e6xxx_chip *chip,
		       struct mv88e6xxx_tcam_data *data,
		       enum mv88e6xxx_tcam_param param, u16 value)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_tcam_get(struct mv88e6xxx_chip *chip,
		       struct mv88e6xxx_tcam_data *data,
		       enum mv88e6xxx_tcam_param param, int *value)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_tcam_get_match(struct mv88e6xxx_chip *chip,
			     struct mv88e6xxx_tcam_data *data,
			     unsigned int offset, u8 *octet, u8 *mask)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_tcam_set_match(struct mv88e6xxx_chip *chip,
			     struct mv88e6xxx_tcam_data *data,
			     unsigned int offset, u8 octet, u8 mask)
{
	return -EOPNOTSUPP;
}
#endif
#endif
