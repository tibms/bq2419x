/*
 *	BQ2419x Platform Data Example
 *
 *			Copyright (C) 2012 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/bq2419x.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "mux.h"
#include "board-plat-bq2419x.h"

#define BQ2419X_IRQ_GPIO		159

static void board_gpio_config(void)
{
	int ret;

	/* interrupt pin */
	omap_mux_init_gpio(BQ2419X_IRQ_GPIO, OMAP_PIN_INPUT);
	ret = gpio_request(BQ2419X_IRQ_GPIO, "bq2419x-irq");
	if (ret) {
		printk(KERN_ERR "IRQ request gpio err = %d", ret);
		return;
	}
	gpio_direction_input(BQ2419X_IRQ_GPIO);
}

static const struct bq2419x_charge_param board_usb_param = {
	.vlim = BQ2419X_VLIM_5080mV,
	.ilim = BQ2419X_ILIM_500mA,
	.ichg = BQ2419X_ICHG_512mA,
	.vreg = BQ2419X_VREG_4208mV,
};

static const struct bq2419x_charge_param board_ta_param = {
	.vlim = BQ2419X_VLIM_5080mV,
	.ilim = BQ2419X_ILIM_3000mA,
	.ichg = BQ2419X_ICHG_2048mA,
	.vreg = BQ2419X_VREG_4208mV,
};

static struct bq2419x_platform_data bq2419x_pdata = {
	.usb_param = &board_usb_param,
	.ta_param = &board_ta_param,
};

static struct i2c_board_info __initdata bq2419x_board_info[] = {
	{
		I2C_BOARD_INFO("bq24192", 0x6B),
		.platform_data = &bq2419x_pdata,
	},
};

int plat_bq2419x_init(void)
{
	bq2419x_board_info[0].irq = gpio_to_irq(BQ2419X_IRQ_GPIO);
	i2c_register_board_info(2, bq2419x_board_info,
				ARRAY_SIZE(bq2419x_board_info));

	board_gpio_config();

	return 0;
}
