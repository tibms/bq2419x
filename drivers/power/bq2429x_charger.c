/*
 * BQ2429x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/bq2429x.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

/* Register 00h */
#define BQ2429X_INPUT_SRC_CTRL		0x00
#define BQ2429X_VINDPM_MASK		0x78
#define BQ2429X_VINDPM_SHIFT		3
#define BQ2429X_IINLIM_MASK		0x07
#define BQ2429X_IINLIM_SHIFT		0

/* Register 01h */
#define BQ2429X_CONFIG			0x01
#define BQ2429X_OTG_MASK		0x20
#define	BQ2429X_OTG_SHIFT		5
#define	BQ2429X_OTG_ENABLE		0x01
#define	BQ2429X_OTG_ENABLE		0x00

/* Register 02h */
#define BQ2429X_ICHG_CTRL		0x02
#define BQ2429X_ICHG_MASK		0xFC
#define BQ2429X_ICHG_SHIFT		2

/* Register 04h */
#define BQ2429X_VREG_CTRL		0x04
#define BQ2429X_VREG_MASK		0xFC
#define BQ2429X_VREG_SHIFT		2

/* Register 05h */
#define BQ2429X_WD_CTRL			0x05
#define BQ2429X_WD_MASK			0x30
#define BQ2429X_WD_SHIFT		4
#define BQ2429X_WD_DISABLE		0x00

/* Register 08h */
#define BQ2429X_STATUS			0x08
#define BQ2429X_VBUS_STATUS_MASK	0xC0
#define BQ2429X_VBUS_STATUS_SHIFT	6
#define BQ2429X_CHG_STAT_MASK		0x30
#define BQ2429X_CHG_STAT_SHIFT		4
#define BQ2429X_NOT_CHARGING		0
#define BQ2429X_FAST_CHARGING		2

/* Register 09h */
#define BQ2429X_FAULT			0x09

/* Charging parameters */
#define BQ2429X_DEFAULT_USB_VLIM	BQ2429X_VLIM_4360mV
#define	BQ2429X_DEFAULT_USB_ILIM	BQ2429X_ILIM_500mA
#define	BQ2429X_DEFAULT_USB_ICHG	BQ2429X_ICHG_512mA
#define	BQ2429X_DEFAULT_USB_VREG	BQ2429X_VREG_4352mV

#define BQ2429X_DEFAULT_TA_VLIM		BQ2429X_VLIM_4360mV
#define BQ2429X_DEFAULT_TA_ILIM		BQ2429X_ILIM_3000mA
#define BQ2429X_DEFAULT_TA_ICHG		BQ2429X_ICHG_3008mA
#define	BQ2429X_DEFAULT_TA_VREG		BQ2429X_VREG_4352mV

enum bq2429x_vbus_type {
	BQ2429X_VBUS_NONE,
	BQ2429X_VBUS_USB_CHARGER,
	BQ2429X_VBUS_TA_CHARGER,
	BQ2429X_VBUS_OTG,
};

struct bq2429x {
	struct device *dev;
	struct i2c_client *client;
	int irq;
	struct work_struct irq_work;

	struct power_supply usb;
	struct power_supply wall;

	/* Charging parameters */
	struct bq2429x_charge_param usb_param;
	struct bq2429x_charge_param ta_param;
};

static int bq2429x_disable_watchdog_timer(struct bq2429x *bq);

static int bq2429x_read_byte(struct bq2429x *bq, u8 *data, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read 0x%.2x\n", reg);
		return ret;
	}

	*data = (u8)ret;

	return 0;
}

static int bq2429x_write_byte(struct bq2429x *bq, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(bq->client, reg, data);
}

static int bq2429x_update_bits(struct bq2429x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = bq2429x_read_byte(bq, &tmp, reg);
	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2429x_write_byte(bq, reg, tmp);
}

static enum power_supply_property bq2429x_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static enum bq2429x_vbus_type bq2429x_get_vbus_type(struct bq2429x *bq)
{
	u8 val = 0;

	bq2429x_read_byte(bq, &val, BQ2429X_STATUS);
	val &= BQ2429X_VBUS_STATUS_MASK;
	val >>= BQ2429X_VBUS_STATUS_SHIFT;

	return val;
}

static int bq2429x_charge_status(struct bq2429x *bq)
{
	u8 val = 0;

	bq2429x_read_byte(bq, &val, BQ2429X_STATUS);
	val &= BQ2429X_CHG_STAT_MASK;
	val >>= BQ2429X_CHG_STAT_SHIFT;

	switch (val) {
	case BQ2429X_FAST_CHARGING:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2429X_NOT_CHARGING:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static int bq2429x_usb_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq2429x *bq = container_of(psy, struct bq2429x, usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bq2429x_get_vbus_type(bq) == BQ2429X_VBUS_USB_CHARGER)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2429x_charge_status(bq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2429x_wall_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq2429x *bq = container_of(psy, struct bq2429x, wall);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bq2429x_get_vbus_type(bq) == BQ2429X_VBUS_TA_CHARGER)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2429x_charge_status(bq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2429x_update_charge_params(struct bq2429x *bq)
{
	enum bq2429x_vbus_type type = bq2429x_get_vbus_type(bq);
	int ret;
	u8 vlim;
	u8 ilim;
	u8 ichg;
	u8 vreg;

	switch (type) {
	case BQ2429X_VBUS_USB_CHARGER:
		vlim = bq->usb_param.vlim;
		ilim = bq->usb_param.ilim;
		ichg = bq->usb_param.ichg;
		vreg = bq->usb_param.vreg;
		break;
	case BQ2429X_VBUS_TA_CHARGER:
		vlim = bq->ta_param.vlim;
		ilim = bq->ta_param.ilim;
		ichg = bq->ta_param.ichg;
		vreg = bq->ta_param.vreg;
		break;
	default:
		return -EINVAL;
	}

	ret = bq2429x_update_bits(bq, BQ2429X_INPUT_SRC_CTRL,
			BQ2429X_VINDPM_MASK, vlim << BQ2429X_VINDPM_SHIFT);
	if (ret)
		return ret;

	ret = bq2429x_update_bits(bq, BQ2429X_INPUT_SRC_CTRL,
			BQ2429X_IINLIM_MASK, ilim << BQ2429X_IINLIM_SHIFT);
	if (ret)
		return ret;

	ret = bq2429x_update_bits(bq, BQ2429X_ICHG_CTRL,
			BQ2429X_ICHG_MASK, ichg << BQ2429X_ICHG_SHIFT);
	if (ret)
		return ret;

	return bq2429x_update_bits(bq, BQ2429X_VREG_CTRL,
			BQ2429X_VREG_MASK, vreg << BQ2429X_VREG_SHIFT);
}

static void bq2429x_irq_workfunc(struct work_struct *work)
{
	struct bq2429x *bq = container_of(work, struct bq2429x, irq_work);
	u8 status;
	u8 fault;
	int ret;

	/* Disable the watchdog timer forcedly */
	bq2429x_disable_watchdog_timer(bq);

	/* Read STATUS and FAULT registers */
	ret = bq2429x_read_byte(bq, &status, BQ2429X_STATUS);
	if (ret) {
		dev_err(bq->dev, "i2c failure:%d\n", ret);
		return;
	}

	ret = bq2429x_read_byte(bq, &fault, BQ2429X_FAULT);
	if (ret) {
		dev_err(bq->dev, "i2c failure: %d\n", ret);
		return;
	}

	dev_info(bq->dev, "IRQ status=0x%.2x, fault=0x%.2x\n", status, fault);

	/* Set charging parameters: VINDPM, IINLIM and ICHG */
	bq2429x_update_charge_params(bq);
}

static irqreturn_t bq2429x_interrupt(int irq, void *data)
{
	struct bq2429x *bq = data;

	schedule_work(&bq->irq_work);
	return IRQ_HANDLED;
}

static int bq2429x_disable_watchdog_timer(struct bq2429x *bq)
{
	u8 val = BQ2429X_WD_DISABLE << BQ2429X_WD_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_WD_CTRL, BQ2429X_WD_MASK, val);
}


static int bq2429x_enable_otg(struct bq2429x *bq)
{
    u8 val = BQ2429X_OTG_ENABLE << BQ2429X_OTG_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_CONFIG, BQ2429X_OTG_MASK, val);
	
}


static int bq2429x_disable_otg(struct bq2429x *bq)
{
    u8 val = BQ2429X_OTG_DISABLE << BQ2429x_OTG_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_CONFIG, BQ2429X_OTG_MASK, val);
	
}

static int bq2429x_set_chargecurrent(struct bq2429x *bq, enum bq2429x_charge_current current)
{

	enum bq2429x_vbus_type type = bq2429x_get_vbus_type(bq);
	u8 ichg;

	switch (type) {
	case BQ2429X_VBUS_USB_CHARGER:
		ichg = bq->usb_param.ichg = current;
		break;
	case BQ2429X_VBUS_TA_CHARGER:
		ichg = bq->ta_param.ichg = current;
		break;
	default:
		return -EINVAL;
	}
	return bq2429x_update_bits(bq, BQ2429X_ICHG_CTRL,
			BQ2429X_ICHG_MASK, ichg << BQ2429X_ICHG_SHIFT);

}


static int bq2429x_set_chargevoltage(struct bq2429x *bq, enum bq2429x_charge_voltage_limit volt)
{

	enum bq2429x_vbus_type type = bq2429x_get_vbus_type(bq);
	int ret;
	u8 vreg;

	switch (type) {
	case BQ2429X_VBUS_USB_CHARGER:
		vreg = bq->usb_param.vreg = volt;
		break;
	case BQ2429X_VBUS_TA_CHARGER:
		vreg = bq->ta_param.vreg = volt;
		break;
	default:
		return -EINVAL;
	}
	
	return bq2429x_update_bits(bq, BQ2429X_VREG_CTRL,
			BQ2429X_VREG_MASK, vreg << BQ2429X_VREG_SHIFT);
	
}


static int bq2429x_set_input_volt_limit(struct bq2429x *bq, enum bq2429x_input_voltage_limit volt)
{

	enum bq2429x_vbus_type type = bq2429x_get_vbus_type(bq);
	int ret;
	u8 vlim;

	switch (type) {
	case BQ2429X_VBUS_USB_CHARGER:
		vlim = bq->usb_param.vlim = volt;
		break;
	case BQ2429X_VBUS_TA_CHARGER:
		vlim = bq->ta_param.vlim = volt;
		break;
	default:
		return -EINVAL;
	}
	
	return 	bq2429x_update_bits(bq, BQ2429X_INPUT_SRC_CTRL,
			BQ2429X_VINDPM_MASK, vlim << BQ2429X_VINDPM_SHIFT);
	
}


static int bq2429x_set_input_current_limit(struct bq2429x *bq, enum bq2429x_input_current_limit current)
{

	enum bq2429x_vbus_type type = bq2429x_get_vbus_type(bq);
	int ret;
	u8 ilim;

	switch (type) {
	case BQ2429X_VBUS_USB_CHARGER:
		ilim = bq->usb_param.ilim = current;
		break;
	case BQ2429X_VBUS_TA_CHARGER:
		ilim = bq->ta_param.ilim = current;
		break;
	default:
		return -EINVAL;
	}
	
	return 	bq2429x_update_bits(bq, BQ2429X_INPUT_SRC_CTRL,
			BQ2429X_IINLIM_MASK, ilim << BQ2429X_IINLIM_SHIFT);
			
}





static int bq2429x_init_charge_params(struct bq2429x *bq,
				struct bq2429x_platform_data *pdata)
{
	/*
	 * Update charging parameters:
	 *   if the platform data is not configured, use the default value
	 *   if the platform data has specific values, use them
	 */
	bq->usb_param.vlim = BQ2429X_DEFAULT_USB_VLIM;
	bq->usb_param.ilim = BQ2429X_DEFAULT_USB_ILIM;
	bq->usb_param.ichg = BQ2429X_DEFAULT_USB_ICHG;
	bq->usb_param.vreg = BQ2429X_DEFAULT_USB_VREG;

	bq->ta_param.vlim = BQ2429X_DEFAULT_TA_VLIM;
	bq->ta_param.ilim = BQ2429X_DEFAULT_TA_ILIM;
	bq->ta_param.ichg = BQ2429X_DEFAULT_TA_ICHG;
	bq->ta_param.vreg = BQ2429X_DEFAULT_TA_VREG;

	if (!pdata)
		return 0;

	if (pdata->usb_param) {
		bq->usb_param.vlim = pdata->usb_param->vlim;
		bq->usb_param.ilim = pdata->usb_param->ilim;
		bq->usb_param.ichg = pdata->usb_param->ichg;
		bq->usb_param.vreg = pdata->usb_param->vreg;
	}

	if (pdata->ta_param) {
		bq->ta_param.vlim = pdata->ta_param->vlim;
		bq->ta_param.ilim = pdata->ta_param->ilim;
		bq->ta_param.ichg = pdata->ta_param->ichg;
		bq->ta_param.vreg = pdata->ta_param->vreg;
	}

	return 0;
}

static int bq2429x_init_device(struct bq2429x *bq,
			struct bq2429x_platform_data *pdata)
{
	int ret;

	/*
	 * Put here for initialization process
	 */

	ret = bq2429x_disable_watchdog_timer(bq);
	if (ret)
		return ret;

	return bq2429x_init_charge_params(bq, pdata);
}

static int bq2429x_psy_register(struct bq2429x *bq)
{
	int ret;

	bq->usb.name = "bq2429x-usb";
	bq->usb.type = POWER_SUPPLY_TYPE_USB;
	bq->usb.properties = bq2429x_charger_props;
	bq->usb.num_properties = ARRAY_SIZE(bq2429x_charger_props);
	bq->usb.get_property = bq2429x_usb_get_property;
	bq->usb.external_power_changed = NULL;

	ret = power_supply_register(bq->dev, &bq->usb);
	if (ret) {
		dev_err(bq->dev, "failed to register usb: %d\n", ret);
		return ret;
	}

	bq->wall.name = "bq2429x-wall";
	bq->wall.type = POWER_SUPPLY_TYPE_MAINS;
	bq->wall.properties = bq2429x_charger_props;
	bq->wall.num_properties = ARRAY_SIZE(bq2429x_charger_props);
	bq->wall.get_property = bq2429x_wall_get_property;
	bq->wall.external_power_changed = NULL;

	ret = power_supply_register(bq->dev, &bq->wall);
	if (ret) {
		dev_err(bq->dev, "failed to register wall: %d\n", ret);
		goto err_psy_wall;
	}

	return 0;

err_psy_wall:
	power_supply_unregister(&bq->usb);
	return ret;
}

static void bq2429x_psy_unregister(struct bq2429x *bq)
{
	power_supply_unregister(&bq->wall);
	power_supply_unregister(&bq->usb);
}

static ssize_t bq2429x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2429x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;

	for (addr = 0x0; addr <= 0x0A; addr++) {
		bq2429x_read_byte(bq, &val, addr);
		dev_info(bq->dev, "[0x%.2x] = 0x%.2x\n", addr, val);
	}

	return 0;
}

static DEVICE_ATTR(registers, S_IRUGO, bq2429x_show_registers, NULL);

static struct attribute *bq2429x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2429x_attr_group = {
	.attrs = bq2429x_attributes,
};

static int bq2429x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2429x *bq;
	struct bq2429x_platform_data *pdata = client->dev.platform_data;
	int ret;

	bq = kzalloc(sizeof(struct bq2429x), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;
	bq->irq = client->irq;

	i2c_set_clientdata(client, bq);

	ret = bq2429x_init_device(bq, pdata);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_dev;
	}

	INIT_WORK(&bq->irq_work, bq2429x_irq_workfunc);

	ret = bq2429x_psy_register(bq);
	if (ret)
		goto err_dev;

	ret = sysfs_create_group(&bq->dev->kobj, &bq2429x_attr_group);
	if (ret) {
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_dev;
	}

	ret = request_irq(bq->irq, bq2429x_interrupt,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"bq2429x_irq", bq);
	if (ret) {
		dev_err(bq->dev, "Request IRQ%d err: %d\n", bq->irq, ret);
		goto err_irq;
	}

	return 0;

err_irq:
	bq2429x_psy_unregister(bq);
	cancel_work_sync(&bq->irq_work);
err_dev:
	kfree(bq);
	return ret;
}

static int bq2429x_charger_remove(struct i2c_client *client)
{
	struct bq2429x *bq = i2c_get_clientdata(client);

	free_irq(bq->irq, bq);
	sysfs_remove_group(&bq->dev->kobj, &bq2429x_attr_group);
	bq2429x_psy_unregister(bq);
	cancel_work_sync(&bq->irq_work);
	kfree(bq);

	return 0;
}

static const struct i2c_device_id bq2429x_id[] = {
	{ "bq24296", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2429x_id);

static struct i2c_driver bq2429x_charger = {
	.probe		= bq2429x_charger_probe,
	.remove		= bq2429x_charger_remove,
	.id_table	= bq2429x_id,
	.driver		= {
		.name	= "bq2429x",
	},
};
module_i2c_driver(bq2429x_charger);

MODULE_DESCRIPTION("TI BQ2419x Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
