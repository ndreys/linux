/*
 * Copyright (C) 2012 Thomas Petazzoni
 *
 * Thomas Petazzoni <thomas.petazzoni@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _LINUX_IRQCHIP_H
#define _LINUX_IRQCHIP_H

#include <linux/acpi.h>
#include <linux/of.h>

/*
 * This macro must be used by the different irqchip drivers to declare
 * the association between their DT compatible string and their
 * initialization function.
 *
 * @name: name that must be unique accross all IRQCHIP_DECLARE of the
 * same file.
 * @compstr: compatible string of the irqchip driver
 * @fn: initialization function
 */
#define IRQCHIP_DECLARE(name, compat, fn) OF_DECLARE_2(irqchip, name, compat, fn)

/*
 * Use this macro when you have a driver that requires two
 * initialization routines, one at IRQCHIP_DECLARE, and one at
 * platform device probe
 */
#define IRQCHIP_DECLARE_DRIVER(name, compat, fn)			\
	static int __init						\
	name##_of_irqchip_init_driver(struct device_node *np,		\
				      struct device_node *parent)	\
	{								\
		of_node_clear_flag(np, OF_POPULATED);			\
		return fn(np, parent);					\
	}								\
	OF_DECLARE_2(irqchip, name, compat, name##_of_irqchip_init_driver)


/*
 * This macro must be used by the different irqchip drivers to declare
 * the association between their version and their initialization function.
 *
 * @name: name that must be unique accross all IRQCHIP_ACPI_DECLARE of the
 * same file.
 * @subtable: Subtable to be identified in MADT
 * @validate: Function to be called on that subtable to check its validity.
 *            Can be NULL.
 * @data: data to be checked by the validate function.
 * @fn: initialization function
 */
#define IRQCHIP_ACPI_DECLARE(name, subtable, validate, data, fn)	\
	ACPI_DECLARE_PROBE_ENTRY(irqchip, name, ACPI_SIG_MADT, 		\
				 subtable, validate, data, fn)

#ifdef CONFIG_IRQCHIP
void irqchip_init(void);
#else
static inline void irqchip_init(void) {}
#endif

#endif
