/*
 * arch/arm/mach-kirkwood/board-openrd.c
 *
 * Marvell OpenRD (Client|Ultimate) Board Setup
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/slab.h>

static int uart1 __initdata = -EINVAL;

core_param(kw_openrd_init_uart1, uart1, int, 0444);

static __init void enable_node(char *name)
{
	struct device_node *np = of_find_node_by_path(name);
	struct property *status;

	if (!np) {
		pr_err("Unable to find node: %s\n", name);
		return;
	}

	status = kzalloc(sizeof(*status), GFP_KERNEL);
	if (!status)
		return;

	status->value = kstrdup("okay", GFP_KERNEL);
	status->length = 5;
	status->name = kstrdup("status", GFP_KERNEL);
	if (!status->value || !status->name) {
		kfree(status);
		return;
	}

	of_update_property(np, status);
	of_node_put(np);

	of_platform_device_create(np, NULL, NULL);
}

static __init void enable_serial(void)
{
	if (gpio_request(34, "SD_UART1_SEL")) {
		pr_err("GPIO request 34 failed for SD/UART1 selection\n");
		return;
	}
	gpio_direction_output(34, 0);
	gpio_free(34);

	if (gpio_request(28, "RS232_RS485_SEL")) {
		pr_err("GPIO request 28 failed for RS232/RS485 selection\n");
		return;
	}
	if (uart1 == 232)
		gpio_direction_output(28, 0);
	else
		gpio_direction_output(28, 1);
	gpio_free(28);

	enable_node("/ocp@f1000000/serial@12100");
}

static __init void enable_sdio(void)
{
	if (gpio_request(34, "SD_UART1_SEL")) {
		pr_err("GPIO request 34 failed for SD/UART1 selection\n");
		return;
	}
	gpio_direction_output(34, 1);
	gpio_free(34);

	enable_node("/ocp@f1000000/mvsdio@90000");
}

/*
 * Depending on the kernel command line parameter, we need to select
 * between the RS232 and RS485 port. Or select between serial and SD.
 * This is called in device_initcall_sync so that we know the
 * gpio/pinctrl driver has loaded, and we can access the gpio pins to
 * enable the hardware.
 */
static __init int sd_rs232_rs485_init(void)
{
	if (!of_machine_is_compatible("marvell,openrd-client") &&
	    !of_machine_is_compatible("marvell,openrd-ultimate"))
		return 0;

	if (uart1 == 232 || uart1 == 485)
		enable_serial();
	else
		enable_sdio();
	return 0;
}

device_initcall_sync(sd_rs232_rs485_init);
