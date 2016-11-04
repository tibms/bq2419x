/*
 * Copyright (C) 2013 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LINUX_BQ2429X_I2C_H
#define _LINUX_BQ2429X_I2C_H

#include <linux/power_supply.h>

enum bq2429x_input_voltage_limit {
	BQ2429X_VLIM_3880mV,
	BQ2429X_VLIM_3960mV,
	BQ2429X_VLIM_4040mV,
	BQ2429X_VLIM_4120mV,
	BQ2429X_VLIM_4200mV,
	BQ2429X_VLIM_4280mV,
	BQ2429X_VLIM_4360mV,
	BQ2429X_VLIM_4440mV,
	BQ2429X_VLIM_4520mV,
	BQ2429X_VLIM_4600mV,
	BQ2429X_VLIM_4680mV,
	BQ2429X_VLIM_4760mV,
	BQ2429X_VLIM_4840mV,
	BQ2429X_VLIM_4920mV,
	BQ2429X_VLIM_5000mV,
	BQ2429X_VLIM_5080mV,
};

enum bq2429x_input_current_limit {
	BQ2429X_ILIM_100mA,
	BQ2429X_ILIM_150mA,
	BQ2429X_ILIM_500mA,
	BQ2429X_ILIM_900mA,
	BQ2429X_ILIM_1000mA,
	BQ2429X_ILIM_1500mA,
	BQ2429X_ILIM_2000mA,
	BQ2429X_ILIM_3000mA,
};

enum bq2429x_charge_current {
	BQ2429X_ICHG_512mA,
	BQ2429X_ICHG_1024mA = 8,
	BQ2429X_ICHG_2048mA = 24,
	BQ2429X_ICHG_3008mA = 39,
//	BQ2429X_ICHG_4032mA = 55,
//	BQ2429X_ICHG_4544mA = 63,
};

enum bq2429x_charge_voltage_limit {
	BQ2429X_VREG_3504mV,
	BQ2429X_VREG_3600mV = 6,
	BQ2429X_VREG_3808mV = 19,
	BQ2429X_VREG_3904mV = 25,
	BQ2429X_VREG_4000mV = 31,
	BQ2429X_VREG_4112mV = 38,
	BQ2429X_VREG_4208mV = 44,
	BQ2429X_VREG_4304mV = 50,
	BQ2429X_VREG_4352mV = 53,
	BQ2429X_VREG_4400mV = 56,
	BQ2429X_VREG_MAX = BQ2429X_VREG_4400mV,
};

struct bq2429x_charge_param {
	enum bq2429x_input_voltage_limit vlim;
	enum bq2429x_input_current_limit ilim;
	enum bq2429x_charge_current ichg;
	enum bq2429x_charge_voltage_limit vreg;
};

struct bq2429x_platform_data {
	const struct bq2429x_charge_param *usb_param;
	const struct bq2429x_charge_param *ta_param;
};

#endif /* _LINUX_BQ2429X_I2C_H */
