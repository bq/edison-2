/*
 * BQ27510 battery driver
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
#include <mach/board.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/power/bq27541_battery.h>


#define DRIVER_VERSION			"1.1.0"
#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_FLAGS		0x0A
#define BQ27x00_REG_TTE			0x16
#define BQ27x00_REG_TTF			0x18
#define BQ27x00_REG_TTECP		0x26
#define BQ27000_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27500_REG_SOC			0x2c

#define BQ27500_FLAG_DSC		BIT(0)
#define BQ27000_FLAG_CHGS		BIT(8)
#define BQ27500_FLAG_FC			BIT(9)
#define BQ27500_FLAG_OTD		BIT(14)
#define BQ27500_FLAG_OTC		BIT(15)

#define BQ27510_SPEED 			100 * 1000

int  virtual_battery_enable = 0;
extern int dwc_vbus_status(void);
static void bq27541_set(void);
struct bq27541_platform_data *g_pdata;
extern bool is_accharging(void);
extern bool is_usbcharging(void);
extern void kernel_power_off(void);
extern void rk30_bat_unregister(void);

#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
#define DIS_CHARGING_TEMP 450
#define EN_CHARGING_TEMP 400
extern void bq24196_charge_disable(void);
extern void bq24196_charge_en(void);
extern int check_charge_ok;
static struct kobject *bq27541_kobj;
static int stop_charging;
static int temp_val = 0;
int charge_en_flags = 0;
int update_temp_ok = 0;
#endif

extern volatile bool low_usb_charge;
extern int bq27541_init = 0;
#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
extern int bq24196_mode;
#endif
#if defined(CONFIG_REBOOT_TO_POWER_OFF)
extern int charge_status_now;
#endif

struct mutex g_bq27541_mutex;

#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_MUTEX(battery_mutex);

struct bq27541_device_info {
	struct device 		*dev;
	struct power_supply	bat;
	struct power_supply	ac;
	struct delayed_work work;
	struct delayed_work wakeup_work;
	struct i2c_client	*client;
	int wake_irq;
	unsigned int interval;
	unsigned int dc_check_pin;
	unsigned int bat_check_pin;
	unsigned int bat_num;
	int power_down;
};
  
static struct bq27541_device_info *bq27541_di;
static enum power_supply_property bq27541_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property rk29_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static ssize_t battery_proc_write(struct file *file,const char __user *buffer,
			 unsigned long count,void *data)
{
	char c;
	int rc;
	printk("USER:\n");
	printk("echo x >/proc/driver/power\n");
	printk("x=1,means just print log ||x=2,means log and data ||x= other,means close log\n");

	rc = get_user(c,buffer);
	if(rc)
		return rc;
	
	//added by zwp,c='8' means check whether we need to download firmware to bq27xxx,return 0 means yes.
	if(c == '8'){
		printk("%s,bq27541 don't need to download firmware\n",__FUNCTION__);
		return -1;//bq27541 don't need to download firmware.
	}
	if(c == '1')
		virtual_battery_enable = 1;
	else if(c == '2')
		virtual_battery_enable = 2;
	else if(c == '3')
		virtual_battery_enable = 3;
	else if(c == '9'){
		printk("%s:%d>>bq27541 set\n",__FUNCTION__,__LINE__);
		bq27541_set();
	}
	else 
		virtual_battery_enable = 0;
	printk("%s,count(%d),virtual_battery_enable(%d)\n",__FUNCTION__,(int)count,virtual_battery_enable);
	return count;
}

static const struct file_operations battery_proc_fops = {
	.owner		= THIS_MODULE, 
	.write		= battery_proc_write,
}; 

/*
 * Common code for BQ27510 devices read
 */
static int bq27541_read(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	mutex_lock(&g_bq27541_mutex);
	ret = i2c_master_reg8_recv(client, reg, buf, len, BQ27510_SPEED);
	mutex_unlock(&g_bq27541_mutex);
	return ret; 
}

static int bq27541_write(struct i2c_client *client, u8 reg, u8 const buf[], unsigned len)
{
	int ret; 
	mutex_lock(&g_bq27541_mutex);
	ret = i2c_master_reg8_send(client, reg, buf, (int)len, BQ27510_SPEED);
	mutex_unlock(&g_bq27541_mutex);
	return ret;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
 
static irqreturn_t bq27541_bat_wakeup(int irq, void *dev_id)
{	
	struct bq27541_device_info *di = (struct bq27541_device_info *)dev_id;

	printk("!!!  bq27541 bat_low irq low vol !!!\n\n\n");
	disable_irq_wake(di->wake_irq);

	schedule_delayed_work(&di->wakeup_work, msecs_to_jiffies(0));	
	return IRQ_HANDLED;
}

static void bq27541_battery_wake_work(struct work_struct *work)
{
    int ret;
    struct bq27541_device_info *di = 
		(struct bq27541_device_info *)container_of(work, struct bq27541_device_info, wakeup_work.work);
		
    rk28_send_wakeup_key();
    
    free_irq(di->wake_irq, di);
	di->wake_irq = gpio_to_irq(di->bat_check_pin);
	
	ret = request_irq(di->wake_irq, bq27541_bat_wakeup, IRQF_TRIGGER_FALLING, "bq27541_battery", di);
	if (ret) {
		printk("request faild!\n");
		return;
	}
	enable_irq_wake(di->wake_irq);
}

static int bq27541_battery_temperature(struct bq27541_device_info *di)
{
	int ret;
	int temp = 0;
	u8 buf[2] ={0};

#if defined (CONFIG_NO_BATTERY_IC)
	return 258;
#endif
	if(virtual_battery_enable == 1)
		return 125/*258*/;

	ret = bq27541_read(di->client,BQ27x00_REG_TEMP,buf,2);
	if (ret<0) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}
	temp = get_unaligned_le16(buf);
	temp = temp - 2731;  //K
	DBG("Enter:%s %d--temp = %d\n",__FUNCTION__,__LINE__,temp);
#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
	if((temp >= DIS_CHARGING_TEMP) && (0 == charge_en_flags)){
		bq24196_charge_disable();
		charge_en_flags = 1;
	}else if((temp <= EN_CHARGING_TEMP) && (1 == charge_en_flags)){
		bq24196_charge_en();
		charge_en_flags = 0;
	}
#endif
//	rk29_pm_power_off();
	return temp;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */



static int bq27541_battery_voltage(struct bq27541_device_info *di)
{
	int ret;
	u8 buf[2] = {0};
	int volt = 0;

#if defined (CONFIG_NO_BATTERY_IC)
	return 4000000;
#endif
	if(virtual_battery_enable == 1)
		return 2000000/*4000000*/;

	ret = bq27541_read(di->client,BQ27x00_REG_VOLT,buf,2); 
	if (ret<0) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}
	volt = get_unaligned_le16(buf);

	//bp27510 can only measure one li-lion bat
	if(di->bat_num == 2){
		volt = volt * 1000 * 2;
	}else{
		volt = volt * 1000;
	}

	if ((volt <= 3500000)  && (volt > 0) && (1 == gpio_get_value(di->dc_check_pin))){
		printk("vol smaller then 3.5V, report to android!");
		di->power_down = 1;
	}else{
		di->power_down = 0;
	}
	
	DBG("Enter:%s %d--volt = %d\n",__FUNCTION__,__LINE__,volt);
	return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27541_battery_current(struct bq27541_device_info *di)
{
	int ret;
	int curr = 0;
	u8 buf[2] = {0};

#if defined (CONFIG_NO_BATTERY_IC)
	return 22000;
#endif
	if(virtual_battery_enable == 1)
		return 11000/*22000*/;

	ret = bq27541_read(di->client,BQ27x00_REG_AI,buf,2);
	if (ret<0) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	curr = get_unaligned_le16(buf);
	DBG("curr = %x \n",curr);
	if(curr>0x8000){
		curr = 0xFFFF^(curr-1);
	}
	curr = curr * 1000;
	DBG("Enter:%s %d--curr = %d\n",__FUNCTION__,__LINE__,curr);
	return curr;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27541_cap = 0;
static int bq27541_battery_rsoc(struct bq27541_device_info *di)
{
	int ret;
	int rsoc = 0;
	int flags = 0;
	int status = 0;
	u8 buf[2];

#if defined (CONFIG_NO_BATTERY_IC)
	return 100;
#endif
	if(virtual_battery_enable == 1)
		return 50/*100*/;

	ret = bq27541_read(di->client,BQ27500_REG_SOC,buf,2); 
	if (ret<0) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}
	rsoc = get_unaligned_le16(buf);
	DBG("Enter:%s %d--read rsoc = %d\n",__FUNCTION__,__LINE__,rsoc);

	/* covert the capacity range */
	rsoc = min(rsoc, 100);
	if ((g_pdata != NULL) && g_pdata->capacity_max && g_pdata->capacity_min) {
		rsoc = max(rsoc, g_pdata->capacity_min);
		rsoc = ((rsoc - g_pdata->capacity_min) * 100 +
			(g_pdata->capacity_max - g_pdata->capacity_min) / 2)
			/ (g_pdata->capacity_max - g_pdata->capacity_min);
	}

	bq27541_cap = rsoc;

#if 0
	/*check full flags,if not full, show 99%*/
	ret = bq27541_read(di->client,BQ27x00_REG_FLAGS, buf, 2);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}
	flags = get_unaligned_le16(buf);
	DBG("Enter:%s %d--flags = 0x%x\n",__FUNCTION__,__LINE__,flags);
	if (flags & BQ27500_FLAG_FC)
		status = POWER_SUPPLY_STATUS_FULL;

	if(status != POWER_SUPPLY_STATUS_FULL)
		rsoc = min(rsoc, 99);
#endif
	DBG("Enter:%s %d--cal rsoc = %d\n",__FUNCTION__,__LINE__,rsoc);

#if defined (CONFIG_NO_BATTERY_IC)
	rsoc = 100;
#endif

	return rsoc;
}

static int bq27541_battery_status(struct bq27541_device_info *di,
				  union power_supply_propval *val)
{
	u8 buf[2] = {0};
	int flags = 0;
	int status = 0;
	int ret = 0;

#if defined (CONFIG_NO_BATTERY_IC)
	val->intval = POWER_SUPPLY_STATUS_FULL;
	return 0;
#endif
	if(virtual_battery_enable == 1)
	{
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	ret = bq27541_read(di->client,BQ27x00_REG_FLAGS, buf, 2);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}
	flags = get_unaligned_le16(buf);
	DBG("Enter:%s %d--flags = 0x%x\n",__FUNCTION__,__LINE__,flags);

#if 0
	if (flags & BQ27500_FLAG_FC)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (flags & BQ27500_FLAG_DSC)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;
#else
	if ((g_pdata != NULL) && g_pdata->get_charging_stat)
	{
#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
		if(!(*g_pdata->get_charging_stat)() || (bq24196_mode == 1))
#else
		if(!(*g_pdata->get_charging_stat)())
#endif
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else if(1 == (*g_pdata->get_charging_stat)()){
			if ((bq27541_cap > 99) && (flags & BQ27500_FLAG_FC))
				status = POWER_SUPPLY_STATUS_FULL;
			else
				status = POWER_SUPPLY_STATUS_CHARGING;
		}
	}
#endif
#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
		if((1 == check_charge_ok) && (!strstr(saved_command_line,"charger"))){
			if(1 == charge_en_flags)
			{
				if(status != POWER_SUPPLY_STATUS_DISCHARGING){
					status = POWER_SUPPLY_STATUS_DISCHARGING;
					stop_charging = 1;
				}
			}else{
				stop_charging = 0;
			}

			if(1 == update_temp_ok){
                        	if(1 == charge_en_flags)
                                        status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                                else
                                        update_temp_ok = 0; 
			}
		}
#endif

	val->intval = status;
	DBG("Enter:%s %d--status = %x\n",__FUNCTION__,__LINE__,status);
	return 0;
}

static int bq27541_health_status(struct bq27541_device_info *di,
				  union power_supply_propval *val)
{
	u8 buf[2] = {0};
	int flags = 0;
	int status;
	int ret;
	
#if defined (CONFIG_NO_BATTERY_IC)
	val->intval = POWER_SUPPLY_HEALTH_GOOD;
	return 0;
#endif
	if(virtual_battery_enable == 1)
	{
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	}

	ret = bq27541_read(di->client,BQ27x00_REG_FLAGS, buf, 2);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}
	flags = get_unaligned_le16(buf);
	DBG("Enter:%s %d--status = %x\n",__FUNCTION__,__LINE__,flags);
	
	if ((flags & BQ27500_FLAG_OTD)||(flags & BQ27500_FLAG_OTC))
		status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		status = POWER_SUPPLY_HEALTH_GOOD;

	val->intval = status;

	return 0;
}


/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27541_battery_time(struct bq27541_device_info *di, int reg,
				union power_supply_propval *val)
{
	u8 buf[2] = {0};
	int tval = 0;
	int ret;

	ret = bq27541_read(di->client,reg,buf,2);
	if (ret<0) {
		dev_err(di->dev, "error reading register %02x\n", reg);
		return ret;
	}
	tval = get_unaligned_le16(buf);
	DBG("Enter:%s %d--tval=%d\n",__FUNCTION__,__LINE__,tval);
	if (tval == 65535)
		return -ENODATA;

	val->intval = tval * 60;
	DBG("Enter:%s %d val->intval = %d\n",__FUNCTION__,__LINE__,val->intval);
	return 0;
}

#define to_bq27541_device_info(x) container_of((x), \
				struct bq27541_device_info, bat);
   
static int bq27541_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	
	struct bq27541_device_info *di = to_bq27541_device_info(psy);
	DBG("Enter:%s %d psp= %d\n",__FUNCTION__,__LINE__,psp);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27541_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq27541_battery_voltage(di);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		#if 0
		val->intval = bq27541_battery_voltage(di);
		if (psp == POWER_SUPPLY_PROP_PRESENT){
			val->intval = val->intval <= 0 ? 0 : 1;
		}
		#else
		val->intval = 1;
		#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27541_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27541_battery_rsoc(di);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27541_battery_temperature(di);
#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
			if(1 == charge_en_flags)
				temp_val = val->intval;
#endif

		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq27541_health_status(di, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27541_battery_time(di, BQ27x00_REG_TTE, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27541_battery_time(di, BQ27x00_REG_TTECP, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27541_battery_time(di, BQ27x00_REG_TTF, val);
		break;
	
	default:
		return -EINVAL;
	}

	return ret;
}

static int rk29_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	struct bq27541_device_info *di = container_of(psy, struct bq27541_device_info, ac);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS){
			if(gpio_get_value(di->dc_check_pin))
				val->intval = 0;	/*discharging*/
			else
				val->intval = 1;	/*charging*/
		}
		DBG("%s:%d val->intval = %d\n",__FUNCTION__,__LINE__,val->intval);
		break;
		
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void bq27541_powersupply_init(struct bq27541_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27541_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27541_battery_props);
	di->bat.get_property = bq27541_battery_get_property;
	di->power_down = 0;
	
	di->ac.name = "ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = rk29_ac_props;
	di->ac.num_properties = ARRAY_SIZE(rk29_ac_props);
	di->ac.get_property = rk29_ac_get_property;
}


static void bq27541_battery_update_status(struct bq27541_device_info *di)
{
	power_supply_changed(&di->bat);
}

static void bq27541_battery_work(struct work_struct *work)
{
	struct bq27541_device_info *di = container_of(work, struct bq27541_device_info, work.work); 
	bq27541_battery_update_status(di);

#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
	if((1 == charge_en_flags) && (temp_val >= DIS_CHARGING_TEMP))
		update_temp_ok = 1;
#endif
	/* reschedule for the next time */
	schedule_delayed_work(&di->work, di->interval);
}

static void bq27541_set(void)
{
	struct bq27541_device_info *di;
	int i = 0;
	u8 buf[2];

	di = bq27541_di;

	printk("enter 0x41\n");
	buf[0] = 0x41;
	buf[1] = 0x00;
	bq27541_write(di->client,0x00,buf,2);
	msleep(1500);
		
	printk("enter 0x21\n");
	buf[0] = 0x21;
	buf[1] = 0x00;
	bq27541_write(di->client,0x00,buf,2);


	buf[0] = 0;
	buf[1] = 0;
	bq27541_read(di->client,0x00,buf,2);


	while((buf[0] & 0x04)&&(i<5))
	{
		printk("enter more 0x21 times i = %d\n",i);
		mdelay(1000);
		buf[0] = 0x21;
		buf[1] = 0x00;
		bq27541_write(di->client,0x00,buf,2);

		buf[0] = 0;
		buf[1] = 0;
		bq27541_read(di->client,0x00,buf,2);
		i++;
	}

	if(i>5)
	   	printk("write 0x21 error\n");
	else
		printk("bq27541 write 0x21 success\n");
}

/**********************
Boot threshold voltage limit is set in the ADC driver,judge here only to prevent 0% capacity but voltage is more than 3.5V
leading to shut down immediately after boot
***********************/
static void battery_capacity_check(struct bq27541_device_info *di)
{
	int rsoc;
	rsoc = bq27541_battery_rsoc(di);
	if(rsoc <= 0){
		if(is_usbcharging()){
			printk("usb charging and capacity less than 0%,continue charging\n");
			low_usb_charge = true;
			return;
		}else if(!is_accharging()){
			printk("no charging and capacity less than 0%,power off\n");
			kernel_power_off();
		}
	}
#if defined(CONFIG_REBOOT_TO_POWER_OFF)
	if ((board_boot_mode() == BOOT_MODE_REBOOT) )
		if((0 == charge_status_now) && (rsoc <= 5))
			kernel_power_off();
#endif
	return;
}

#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
static ssize_t stop_charging_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", stop_charging);
}

static DEVICE_ATTR(stopcharging, 0644, stop_charging_show, NULL);
static int bq27541_sysfs_init(void)
{
	int ret;
	bq27541_kobj = kobject_create_and_add("bq27541_bat", NULL);
	if (bq27541_kobj == NULL) {
		printk("bq27541_sysfs_init: subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}
	ret = sysfs_create_file(bq27541_kobj, &dev_attr_stopcharging.attr);
	if (ret) {
		printk("bq27541_sysfs_init: sysfs_create_file failed\n");
		goto err0;
	}
	return 0;
err0:
	kobject_del(bq27541_kobj);
err:
	return ret;
}
#endif

static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27541_device_info *di;
	int retval = 0;
	struct bq27541_platform_data *pdata;
	int val =0;
	char buf[2];
	int volt;
	DBG("**********  bq27541_battery_probe**************  ");
	pdata = client->dev.platform_data;
	g_pdata = pdata;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = "battery";
	di->client = client;
	/* 1 seconds between monotor runs interval */
	di->interval = msecs_to_jiffies(4 * 1000);
	
	di->bat_num = pdata->bat_num;
	di->dc_check_pin = pdata->dc_check_pin;
	di->bat_check_pin = pdata->bat_check_pin;
	
	if (pdata->init_dc_check_pin)
		pdata->init_dc_check_pin( );
	
	bq27541_powersupply_init(di);
	mutex_init(&g_bq27541_mutex);

	retval = bq27541_read(di->client,BQ27x00_REG_FLAGS, buf, 2);
	if (retval < 0) {
		printk("can't find bq27541\n");
		goto batt_failed_2;
	}else{
		rk30_bat_unregister();
		bq27541_init = 1;
	}
	
	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_4;
	}
	bq27541_di = di;
#if 0
	retval = power_supply_register(&client->dev, &di->ac);
	if (retval) {
		dev_err(&client->dev, "failed to register ac\n");
		goto batt_failed_4;
	}
#endif

	INIT_DELAYED_WORK(&di->work, bq27541_battery_work);
	schedule_delayed_work(&di->work, di->interval);
	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	battery_capacity_check(di);
#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
	bq27541_sysfs_init();
#endif

	return 0;

batt_failed_4:
	kfree(di);
batt_failed_2:

err_batirq_failed:
	gpio_free(pdata->bat_check_pin);

	return retval;
}

static int bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	power_supply_unregister(&di->bat);
	kfree(di->bat.name);
	kfree(di);
	return 0;
}

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541", 0 },
};

static struct i2c_driver bq27541_battery_driver = {
	.driver = {
		.name = "bq27541",
	},
	.probe = bq27541_battery_probe,
	.remove = bq27541_battery_remove,
	.id_table = bq27541_id,
};

static int __init bq27541_battery_init(void)
{
	int ret;
	
	struct proc_dir_entry * battery_proc_entry;
	
	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27541 driver\n");
	
	battery_proc_entry = proc_create("driver/power",0777,NULL,&battery_proc_fops);

	return ret;
}

//module_init(bq27541_battery_init);
//fs_initcall_sync(bq27541_battery_init);

fs_initcall(bq27541_battery_init);
//arch_initcall(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_AUTHOR("clb");
MODULE_DESCRIPTION("BQ27541 battery monitor driver");
MODULE_LICENSE("GPL");
