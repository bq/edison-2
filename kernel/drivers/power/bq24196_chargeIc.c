/*
 * BQ24196 battery driver
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <mach/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/bq24196_chargeIc.h>
#include <linux/interrupt.h>

struct bq24196_device_info *bq24196_di;
struct bq24196_platform_data *bq24196_pdata;
static int bq24196_int = 0;
int bq24196_mode = 0;
#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

/*
 * Common code for BQ24196 devices read
 */
static int bq24196_read(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	ret = i2c_master_reg8_recv(client, reg, buf, len, BQ24196_SPEED);
	return ret; 
}

static int bq24196_write(struct i2c_client *client, u8 reg, u8 const buf[], unsigned len)
{
	int ret; 
	ret = i2c_master_reg8_send(client, reg, buf, (int)len, BQ24196_SPEED);
	return ret;
}

static ssize_t bat_param_read(struct device *dev,struct device_attribute *attr, char *buf)
{
	int i;
	u8 buffer;
	struct bq24196_device_info *di=bq24196_di;

	for(i=0;i<11;i++)
	{
		bq24196_read(di->client,i,&buffer,1);
		printk("reg %d value %x\n",i,buffer);		
	}
	return 0;
}
DEVICE_ATTR(battparam, 0664, bat_param_read,NULL);

static int bq24196_update_reg(struct i2c_client *client, int reg, u8 value, u8 mask )
{
	int ret =0;
	u8 retval = 0;

	ret = bq24196_read(client, reg, &retval, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	if ((retval & mask) != value) {
		retval = ((retval & ~mask) | value) | value;
		ret = bq24196_write(client, reg, &retval, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}
	}

	return ret;
}

int get_charge_status()
{
	int flags=0,status,ret;
	u8 buf;
	struct bq24196_device_info *di=bq24196_di;

	ret = bq24196_read(di->client, SYSTEM_STATS_REGISTER, &buf, 1);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}
	flags=(buf&0x30)>>4;
	
	if(flags==BQ24196_CHG_COMPELET)
		status = POWER_SUPPLY_STATUS_FULL;
	else if(flags==BQ24196_NO_CHG)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;

	return status;
}
EXPORT_SYMBOL_GPL(get_charge_status);

int is_usb_charging()
{
	int flags=0,status,ret;
	u8 buf;
	struct bq24196_device_info *di=bq24196_di;

	ret = bq24196_read(di->client, SYSTEM_STATS_REGISTER, &buf, 1);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}
	flags=(buf&0xc0)>>6;

	if(flags==BQ24196_USB_CHG)
		status = 1;
	else 	
		status = 0;
	return status;
}
EXPORT_SYMBOL_GPL(is_usb_charging);


static int bq24196_init_registers(void)
{
	int ret = 0;

	/* reset the register */
	ret = bq24196_update_reg(bq24196_di->client,
				POWE_ON_CONFIGURATION_REGISTER,
				REGISTER_RESET_ENABLE << REGISTER_RESET_OFFSET,
				REGISTER_RESET_MASK << REGISTER_RESET_OFFSET);
	if (ret < 0) {
		dev_err(&bq24196_di->client->dev, "%s(): Failed to reset the register \n",
				__func__);
		goto final;
	}

	mdelay(5);

	/* Disable the watchdog */
	ret = bq24196_update_reg(bq24196_di->client,
				TERMINATION_TIMER_CONTROL_REGISTER,
				WATCHDOG_DISABLE << WATCHDOG_OFFSET,
				WATCHDOG_MASK << WATCHDOG_OFFSET);
	if (ret < 0) {
		dev_err(&bq24196_di->client->dev, "%s(): Failed to disable the watchdog \n",
				__func__);
		goto final;
	}

	/* Set Pre-Charge Current Limit as 128mA */
	ret = bq24196_update_reg(bq24196_di->client,
				PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REGISTER,
				PRE_CHARGE_CURRENT_LIMIT_128MA << PRE_CHARGE_CURRENT_LIMIT_OFFSET,
				PRE_CHARGE_CURRENT_LIMIT_MASK << PRE_CHARGE_CURRENT_LIMIT_OFFSET);
	if (ret < 0) {
		dev_err(&bq24196_di->client->dev, "%s(): Failed to set pre-charge limit 128mA \n",
				__func__);
		goto final;
	}

	/* Set Termination Current Limit as 128mA */
	ret = bq24196_update_reg(bq24196_di->client,
				PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REGISTER,
				TERMINATION_CURRENT_LIMIT_128MA << TERMINATION_CURRENT_LIMIT_OFFSET,
				TERMINATION_CURRENT_LIMIT_MASK << TERMINATION_CURRENT_LIMIT_OFFSET);
	if (ret < 0) {
		dev_err(&bq24196_di->client->dev, "%s(): Failed to set termination limit 128mA \n",
				__func__);
		goto final;
	}

final:
	return ret;
}

static int bq24196_update_input_current_limit(u8 value)
{
	int ret = 0;

	ret = bq24196_update_reg(bq24196_di->client,
				INPUT_SOURCE_CONTROL_REGISTER,
				((value << IINLIM_OFFSET) | (EN_HIZ_DISABLE << EN_HIZ_OFFSET)),
				((IINLIM_MASK << IINLIM_OFFSET) | (EN_HIZ_MASK << EN_HIZ_OFFSET)));
	if (ret < 0) {
		dev_err(&bq24196_di->client->dev, "%s(): Failed to set input current limit (0x%x) \n",
				__func__, value);
	}
	return ret;
}

static int bq24196_update_en_hiz_disable(void)
{
	int ret = 0;

	ret = bq24196_update_reg(bq24196_di->client,
				INPUT_SOURCE_CONTROL_REGISTER,
				EN_HIZ_DISABLE << EN_HIZ_OFFSET,
				EN_HIZ_MASK << EN_HIZ_OFFSET);
	if (ret < 0) {
		dev_err(&bq24196_di->client->dev, "%s(): Failed to set en_hiz_disable\n",
				__func__);
	}
	return ret;
}

int bq24196_set_input_current(int on)
{
	if(!bq24196_int)
		return 0;

	if(1 == on){
#ifdef CONFIG_BATTERY_RK30_USB_AND_CHARGE
		bq24196_update_input_current_limit(IINLIM_2000MA);
#else
		bq24196_update_input_current_limit(IINLIM_3000MA);
#endif
	}else{
		bq24196_update_input_current_limit(IINLIM_500MA);
	}
	DBG("bq24196_set_input_current %s\n", on ? "3000mA" : "500mA");

	return 0;
}
EXPORT_SYMBOL_GPL(bq24196_set_input_current);

static int bq24196_update_charge_mode(u8 value)
{
	int ret = 0;

	ret = bq24196_update_reg(bq24196_di->client,
				POWE_ON_CONFIGURATION_REGISTER,
				value << CHARGE_MODE_CONFIG_OFFSET,
				CHARGE_MODE_CONFIG_MASK << CHARGE_MODE_CONFIG_OFFSET);
	if (ret < 0) {
		dev_err(&bq24196_di->client->dev, "%s(): Failed to set charge mode(0x%x) \n",
				__func__, value);
	}

	return ret;
}

static int bq24196_update_otg_mode_current(u8 value)
{
	int ret = 0;

	ret = bq24196_update_reg(bq24196_di->client,
				POWE_ON_CONFIGURATION_REGISTER,
				value << OTG_MODE_CURRENT_CONFIG_OFFSET,
				OTG_MODE_CURRENT_CONFIG_MASK << OTG_MODE_CURRENT_CONFIG_OFFSET);
	if (ret < 0) {
		dev_err(&bq24196_di->client->dev, "%s(): Failed to set otg current mode(0x%x) \n",
				__func__, value);
	}
}

static int bq24196_otg_mode_current_check(void)
{
	int ret =0, flag, status;
	u8 retval = 0;

	ret = bq24196_read(bq24196_di->client, FAULT_STATS_REGISTER, &retval, 1);
	if (ret < 0) {
		dev_err(&bq24196_di->client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	flag = (retval & 0x40)>>6;
	DBG("%s: retval = %d, flag = %d\n", __func__, retval, flag);
	if(1 == flag)
		status = 1;
	else
		status = 0;

	return status;
}

static int bq24196_charge_mode_config(int on)
{
	int i = 0;

	if(!bq24196_int)
		return 0;

	if(1 == on)
	{
		bq24196_update_en_hiz_disable();
		mdelay(5);
		bq24196_update_charge_mode(CHARGE_MODE_CONFIG_OTG_OUTPUT);
		mdelay(10);
		bq24196_update_otg_mode_current(OTG_MODE_CURRENT_CONFIG_1300MA);
		gpio_direction_output(bq24196_pdata->otg_en_pin, 1);
	}else{
		gpio_direction_output(bq24196_pdata->otg_en_pin, 0);
		bq24196_update_charge_mode(CHARGE_MODE_CONFIG_CHARGE_BATTERY);
	}

	DBG("bq24196_charge_mode_config is %s\n", on ? "OTG Mode" : "Charge Mode");

	return 0;
}

static irqreturn_t otg_irq_handler(int irq, void *dev_id);
static int otg_flag = 0;
static void otg_irq_wakeup(struct work_struct *work)
{
	int ret;
	int irq = gpio_to_irq(bq24196_pdata->otg_irq_pin);
	unsigned long flags;

	if(1 == gpio_get_value(bq24196_pdata->otg_irq_pin))
	{
		bq24196_mode = 1;
		bq24196_charge_mode_config(1);
	}else{
		bq24196_charge_mode_config(0);
		msleep(500);
		bq24196_mode = 0;
	}
	DBG("%s: otg_irq_pin is %s\n", __func__,gpio_get_value(bq24196_pdata->otg_irq_pin) ? "high" : "low");

	free_irq(irq, NULL);
	flags = gpio_get_value(bq24196_pdata->otg_irq_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	ret = request_irq(irq, otg_irq_handler, flags, "otg_irq", NULL);
	if (ret < 0) {
		pr_err("%s: request_irq(%d) failed\n", __func__, irq);
	}
}

static DECLARE_DELAYED_WORK(otg_irq_work, otg_irq_wakeup);

static irqreturn_t otg_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(irq);
	schedule_delayed_work(&otg_irq_work, HZ * 1);
	return IRQ_HANDLED;
}

static irqreturn_t status_irq_handler(int irq, void *dev_id);
static void status_irq_wakeup(struct work_struct *work);
static DECLARE_DELAYED_WORK(status_irq_work, status_irq_wakeup);

static void status_irq_wakeup(struct work_struct *work)
	{
		DBG("%s\n", __func__);

		if(1 == bq24196_mode){
			if(1 == bq24196_otg_mode_current_check())
			{
				bq24196_charge_mode_config(0);
				bq24196_mode = 0;
				otg_flag = 1;
				DBG("OTG over current\n");
				schedule_delayed_work(&status_irq_work, msecs_to_jiffies(1000));
			}
		}else{
			if((1 == gpio_get_value(bq24196_pdata->otg_irq_pin)) && (otg_flag == 1))
			{
				bq24196_mode = 1;
				bq24196_charge_mode_config(1);
				otg_flag = 0;
				DBG("OTG output again\n");
			}
		}
	}
/******************status_irq_wakeup instead of otg_status_timer******
static void otg_status_timer(unsigned long _data)
{
	DBG("%s\n", __func__);

	if(1 == bq24196_mode){
		if(1 == bq24196_otg_mode_current_check())
		{
			bq24196_charge_mode_config(0);
			bq24196_mode = 0;
			otg_flag = 1;
			DBG("OTG over current\n");
			mod_timer(&bq24196_di->timer, jiffies + msecs_to_jiffies(1000));
		}
	}else{
		if((1 == gpio_get_value(bq24196_pdata->otg_irq_pin)) && (otg_flag == 1))
		{
			bq24196_mode = 1;
			bq24196_charge_mode_config(1);
			otg_flag = 0;
			DBG("OTG output again\n");
		}
	}
}
*******************/
static irqreturn_t status_irq_handler(int irq, void *dev_id)
{
	DBG("%s\n", __func__);
	//mod_timer(&bq24196_di->timer, jiffies + msecs_to_jiffies(100));
	cancel_delayed_work(&status_irq_work);
	schedule_delayed_work(&status_irq_work, 100);
	return IRQ_HANDLED;
}

static int bq24196_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq24196_device_info *di;
	u8 retval = 0;
	struct bq24196_platform_data *pdata;
	int ret;
	int irq, irq_flag;

	ret = device_create_file(&client->dev,&dev_attr_battparam);
	if(ret){
		ret = -EINVAL;
		printk(KERN_ERR "failed to create bat param file\n");
		goto batt_failed_2;
	}

	
	pdata = client->dev.platform_data;

	if(pdata->otg_en_pin)
	{
		ret = gpio_request(pdata->otg_en_pin, "otg_en");
		if (ret) {
			printk("failed to request otg_en gpio\n");
			gpio_free(pdata->otg_en_pin);
		}
		gpio_pull_updown(pdata->otg_en_pin, GPIOPullDown);
		gpio_direction_output(pdata->otg_en_pin, 0);
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->client = client;
	bq24196_di = di;
	bq24196_pdata = pdata;

	/* get the vendor id */
	ret = bq24196_read(di->client, VENDOR_STATS_REGISTER, &retval, 1);
	if (ret < 0) {
		dev_err(&di->client->dev, "%s(): Failed in reading register"
				"0x%02x\n", __func__, VENDOR_STATS_REGISTER);
		goto batt_failed_4;
	}

	bq24196_init_registers();

#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
	if(pdata->irq_init)
		pdata->irq_init();

	if (pdata->otg_irq_pin != INVALID_GPIO){
		irq = gpio_to_irq(pdata->otg_irq_pin);
		irq_flag = gpio_get_value (pdata->otg_irq_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
		ret = request_irq(irq, otg_irq_handler, irq_flag, "otg_irq", NULL);
		if (ret) {
			ret = -EINVAL;
			printk("failed to request otg_irq\n");
			goto err_otgirq_failed;
		}
	}

	if (pdata->status_irq_pin != INVALID_GPIO){
		irq = gpio_to_irq(pdata->status_irq_pin);
		ret = request_irq(irq, status_irq_handler, IRQF_TRIGGER_FALLING, "status_irq", NULL);
		if (ret) {
			ret = -EINVAL;
			printk("failed to request status_irq\n");
			goto err_statusirq_failed;
		}
	}
	//setup_timer(&di->timer, otg_status_timer, (unsigned long)di);
#endif

	bq24196_int =1;

	DBG("bq24196_battery_probe ok");
	return 0;

batt_failed_4:
	kfree(di);
batt_failed_2:
err_statusirq_failed:
	free_irq(gpio_to_irq(pdata->status_irq_pin), NULL);
err_otgirq_failed:
	free_irq(gpio_to_irq(pdata->otg_irq_pin), NULL);
	return retval;
}

static int bq24196_battery_shutdown(struct i2c_client *client)
{
	int ret = 0;

#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
	free_irq(gpio_to_irq(bq24196_pdata->status_irq_pin), NULL);
	free_irq(gpio_to_irq(bq24196_pdata->otg_irq_pin), NULL);

	if(bq24196_mode == 1)
	{
		gpio_direction_output(bq24196_pdata->otg_en_pin, 0);
		bq24196_update_charge_mode(CHARGE_MODE_CONFIG_CHARGE_BATTERY);
	}
#endif
}
static int bq24196_battery_remove(struct i2c_client *client)
{
	struct bq24196_device_info *di = i2c_get_clientdata(client);
	kfree(di);
	return 0;
}

static const struct i2c_device_id bq24196_id[] = {
	{ "bq24196", 0 },
};

static struct i2c_driver bq24196_battery_driver = {
	.driver = {
		.name = "bq24196",
	},
	.probe = bq24196_battery_probe,
	.remove = bq24196_battery_remove,
	.shutdown = bq24196_battery_shutdown,
	.id_table = bq24196_id,
};

static int __init bq24196_battery_init(void)
{
	int ret;
	
	ret = i2c_add_driver(&bq24196_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ24196 driver\n");
	
	return ret;
}
subsys_initcall(bq24196_battery_init);

static void __exit bq24196_battery_exit(void)
{
	i2c_del_driver(&bq24196_battery_driver);
}
module_exit(bq24196_battery_exit);

MODULE_AUTHOR("Rockchip");
MODULE_DESCRIPTION("BQ24196 battery monitor driver");
MODULE_LICENSE("GPL");
