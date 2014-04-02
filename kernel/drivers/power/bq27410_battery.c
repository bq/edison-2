/*
 * BQ27410 battery driver
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
#include <linux/power/bq27410_battery.h>


#define DRIVER_VERSION			"1.1.0"
#define BQ27410_REG_CONTROL		0x00
#define BQ27410_REG_TEMP			0x02
#define BQ27410_REG_VOL			0x04
#define BQ27410_REG_FLAGS			0x06
#define BQ27410_REG_NOMINAL_CAP		0x08
#define BQ27410_REG_FULL_CAP			0x0A
#define BQ27410_REG_REMAINING_CAP		0x0C
#define BQ27410_REG_FULL_CHARGE_CAP	0x0E
#define BQ27410_REG_AVERAGE_CURRENT	0x10
#define BQ27410_REG_DEBUG1				0x16
#define BQ27410_REG_AVERAGE_POWER		0x18
#define BQ27410_REG_STATE_OF_CHARGER	0x1C
#define BQ27410_REG_INT_TMP				0x1E
#define BQ27410_REG_STATE_OF_HEALTH	0x20
#define BQ27410_REG_DEBUG2				0x2C
#define BQ27410_REG_DEBUG3				0x32
#define BQ27410_REG_OPCONFIG				0x3A
#define BQ27410_REG_DESIGN_CAP			0x3C

#define BQ27410_FLAG_DSC		BIT(0)
#define BQ27410_FLAG_SOCF		BIT(1) /* State-of-Charge threshold final */
#define BQ27410_FLAG_SOC1		BIT(2) /* State-of-Charge threshold 1 */
#define BQ27410_FLAG_BAT_DET		BIT(3)

#define BQ27410_FLAG_ITPOR		BIT(5)
#define BQ27410_FLAG_FC		BIT(9)
#define BQ27410_FLAG_OTC		BIT(15)

#define BQ27410_CONTROL_STATUS_SS		BIT(13)
#define BQ27410_CONTROL_STATUS_FAS		BIT(14)

#define BQ27410_SPEED 			100 * 1000
#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
extern int bq24196_mode;
#endif

/*M-MT:define bq27410 mode*/
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/reboot.h>
#include <linux/syscalls.h>

#define UPDATE_FIRMWARE_RESTART		1
#define BQ27410_NORMAL_MODE 		0		//when in normal mode ,iic addr = 0x55
#define BQ27410_ROM_MODE 			1		//when in normal mode ,iic addr = 0x0b

static const char g_filename[]="/system/vendor/firmware/bq27410.dffs";
static struct proc_dir_entry *bq27410_proc_entry;
int  g_bq27410_mode = BQ27410_NORMAL_MODE;
int g_bq27410_fw_init_flag = 0;
struct mutex g_bq27410_mutex;
static int bq27410_update_flag = 0;

/*M-MT:define bq27410 mode end*/

int  virtual_battery_enable = 0;
extern int dwc_vbus_status(void);
extern bool is_accharging(void);
extern bool is_usbcharging(void);
extern void kernel_power_off(void);
extern void rk30_bat_unregister(void);

#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
extern void bq24196_charge_disable(void);
extern void bq24196_charge_en(void);
int charge_en_flags = 0;
#endif

extern volatile bool low_usb_charge;
extern int bq27410_init = 0;

struct bq27410_platform_data *g_pdata;
struct i2c_client *g_client;

#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

/* If the system has several batteries we need a different name for each
 * of them...
 */
struct bq27410_device_info {
	struct device 		*dev;
	struct power_supply	bat;
	struct power_supply	ac;
	struct delayed_work work;
	struct delayed_work update_work;
	struct delayed_work wakeup_work;
	struct i2c_client	*client;
	int wake_irq;
	unsigned int interval;
	unsigned int dc_check_pin;
	unsigned int bat_check_pin;
	unsigned int bat_num;
	int power_down;
};
  
static struct bq27410_device_info *bq27410_di;
static enum power_supply_property bq27410_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static enum power_supply_property rk29_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

int bq27410_write_batt_insert(struct i2c_client *client);


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
		printk("%s,bq27410 don't need to download firmware\n",__FUNCTION__);
		return -1;//bq27410 don't need to download firmware.
	}
	if(c == '1')
		virtual_battery_enable = 1;
	else if(c == '2')
		virtual_battery_enable = 2;
	else if(c == '3')
		virtual_battery_enable = 3;
	else if(c == '9'){
		printk("%s:%d>>bq27410 set\n",__FUNCTION__,__LINE__);

	}
	else 
		virtual_battery_enable = 0;
	DBG("%s,count(%d),virtual_battery_enable(%d)\n",__FUNCTION__,(int)count,virtual_battery_enable);
	return count;
}

static const struct file_operations battery_proc_fops = {
	.owner		= THIS_MODULE, 
	.write		= battery_proc_write,
}; 

/*
 * Common code for BQ27410 devices read
 */
static int bq27410_read(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	ret = i2c_master_reg8_recv(client, reg, buf, len, BQ27410_SPEED);
	return ret; 
}

static int bq27410_write(struct i2c_client *client, u8 reg, u8 const buf[], unsigned len)
{
	int ret; 
	ret = i2c_master_reg8_send(client, reg, buf, (int)len, BQ27410_SPEED);
	return ret;
}

static int bq27410_write_byte(struct i2c_client *client, u8 reg, u16 const buf[], unsigned len)
{
	int ret; 
	ret = i2c_master_reg16_send(client, reg, buf, (int)len, BQ27410_SPEED);
	return ret;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
 
static irqreturn_t bq27410_bat_wakeup(int irq, void *dev_id)
{	
	struct bq27410_device_info *di = (struct bq27410_device_info *)dev_id;

	printk("!!!  bq27410 bat_low irq low vol !!!\n\n\n");
	disable_irq_wake(di->wake_irq);

	schedule_delayed_work(&di->wakeup_work, msecs_to_jiffies(0));	
	return IRQ_HANDLED;
}

static void bq27410_battery_wake_work(struct work_struct *work)
{
	int ret;
	struct bq27410_device_info *di = 
		(struct bq27410_device_info *)container_of(work, struct bq27410_device_info, wakeup_work.work);

	rk28_send_wakeup_key();

	free_irq(di->wake_irq, di);
	di->wake_irq = gpio_to_irq(di->wake_irq);
	
	ret = request_irq(di->wake_irq, bq27410_bat_wakeup, IRQF_TRIGGER_FALLING, "bq27410_battery", di);
	if (ret) {
		printk("request faild!\n");
		return;
	}
	enable_irq_wake(di->wake_irq);
}

static int bq27410_battery_temperature(struct bq27410_device_info *di)
{
	int ret;
	int temp = 0;
	u8 buf[2] ={0};

	if(virtual_battery_enable == 1)
		return 125/*258*/;

	ret = bq27410_read(di->client,BQ27410_REG_TEMP,buf,2);
	if (ret<0) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	temp = get_unaligned_le16(buf);
	//temp = 5 * temp / 2;
	temp = temp - 2731;  //K

	DBG("Enter:%s %d--temp = %d\n",__FUNCTION__,__LINE__,temp);

#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
		if((temp >= 450) && (0 == charge_en_flags)){
			bq24196_charge_disable();
			charge_en_flags = 1;
		}else if((temp <= 400) && (1 == charge_en_flags)){
			bq24196_charge_en();
			charge_en_flags = 0;
		}
#endif
	return temp;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */



static int bq27410_battery_voltage(struct bq27410_device_info *di)
{
	int ret;
	u8 buf[2] = {0};
	int volt = 0;

	if(virtual_battery_enable == 1)
		return 4000000/*4000000*/;

	ret = bq27410_read(di->client,BQ27410_REG_VOL,buf,2); 
	if (ret<0) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	volt = get_unaligned_le16(buf);

	if(di->bat_num == 2){
		volt = volt * 1000 * 2;
	}else{
		volt = volt * 1000;
	}

	DBG("Enter:%s %d--volt = %d\n",__FUNCTION__,__LINE__,volt);

	return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27410_battery_current(struct bq27410_device_info *di)
{
	int ret;
	int curr = 0;
	u8 buf[2] = {0};

	if(virtual_battery_enable == 1)
		return 11000/*22000*/;

	ret = bq27410_read(di->client,BQ27410_REG_AVERAGE_CURRENT,buf,2);
	if (ret<0) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	curr = get_unaligned_le16(buf);
	DBG("curr = %x \n",curr);

	if(curr > 0x8000){
		curr = 0xFFFF ^ (curr -1);
	}

	curr = curr * 1000;

	DBG("Enter:%s %d--curr = %d\n",__FUNCTION__,__LINE__,curr);

	return curr;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27410_cap = 0;
static int bq27410_battery_rsoc(struct bq27410_device_info *di)
{
	int ret;
	int rsoc = 0;
	int flags = 0;
	int status = 0;
	u8 buf[2];

	if(virtual_battery_enable == 1)
		return 50/*100*/;
	
	ret = bq27410_read(di->client,BQ27410_REG_STATE_OF_CHARGER,buf,2); 
	if (ret<0) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	rsoc = get_unaligned_le16(buf);
	DBG("Enter:%s %d--read reaming capacity = %d\n",__FUNCTION__,__LINE__,rsoc);

	/* covert the capacity range */
	rsoc = min(rsoc, 100);
	if ((g_pdata != NULL) && g_pdata->capacity_max && g_pdata->capacity_min) {
		rsoc = max(rsoc, g_pdata->capacity_min);
		rsoc = ((rsoc - g_pdata->capacity_min) * 100 +
			(g_pdata->capacity_max - g_pdata->capacity_min) / 2)
			/ (g_pdata->capacity_max - g_pdata->capacity_min);
	}

	bq27410_cap = rsoc;

	/*check full flags,if not full, show 99%*/
	ret = bq27410_read(di->client,BQ27410_REG_FLAGS, buf, 2);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}

	flags = get_unaligned_le16(buf);
	DBG("Enter:%s %d--flags = 0x%x\n",__FUNCTION__,__LINE__,flags);

	if (flags & BQ27410_FLAG_FC)
		status = POWER_SUPPLY_STATUS_FULL;

	if(status != POWER_SUPPLY_STATUS_FULL)
		rsoc = min(rsoc, 99);
	DBG("Enter:%s %d--cal rsoc = %d\n",__FUNCTION__,__LINE__,rsoc);
	return rsoc;
}

static int bq27410_battery_rsoc_level(struct bq27410_device_info *di,
								union power_supply_propval *val)
{
	u8 buf[2] = {0};
	int flags = 0;
	int status = 0;
	int ret = 0;

	if(virtual_battery_enable == 1)
	{
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	ret = bq27410_read(di->client,BQ27410_REG_FLAGS, buf, 2);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}

	flags = get_unaligned_le16(buf);

	DBG("Enter:%s %d--flags = 0x%x\n",__FUNCTION__,__LINE__,flags);

	if (flags & BQ27410_FLAG_FC)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (flags & BQ27410_FLAG_SOC1)
		status = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (flags & BQ27410_FLAG_SOCF)
		status = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		status = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

	val->intval = status;

	DBG("Enter:%s %d--status = %x\n",__FUNCTION__,__LINE__,status);
	return 0;

}

static int bq27410_battery_status(struct bq27410_device_info *di,
				  union power_supply_propval *val)
{
	u8 buf[2] = {0};
	int flags = 0;
	int status = 0;
	int ret = 0;

	if(virtual_battery_enable == 1)
	{
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	ret = bq27410_read(di->client,BQ27410_REG_FLAGS, buf, 2);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}

	flags = get_unaligned_le16(buf);

	DBG("Enter:%s %d--flags = 0x%x\n",__FUNCTION__,__LINE__,flags);

	if ((g_pdata != NULL) && g_pdata->get_charging_stat)
	{
#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
		if(!(*g_pdata->get_charging_stat)() || (bq24196_mode == 1))
#else
		if(!(*g_pdata->get_charging_stat)())
#endif
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else if(1 == (*g_pdata->get_charging_stat)()){
			if ((bq27410_cap > 99) && (flags & BQ27410_FLAG_FC))
				status = POWER_SUPPLY_STATUS_FULL;
			else
				status = POWER_SUPPLY_STATUS_CHARGING;
		}
	}

	val->intval = status;

	DBG("Enter:%s %d--status = %x\n",__FUNCTION__,__LINE__,status);
	return 0;
}

static int bq27410_health_status(struct bq27410_device_info *di,
				  union power_supply_propval *val)
{
	u8 buf[2] = {0};
	int flags = 0;
	int status;
	int ret;
	
	if(virtual_battery_enable == 1)
	{
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	}

	ret = bq27410_read(di->client,BQ27410_REG_FLAGS, buf, 2);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}

	flags = get_unaligned_le16(buf);
	DBG("Enter:%s %d--status = %x\n",__FUNCTION__,__LINE__,flags);
	
	if (flags & BQ27410_FLAG_SOCF)
		status = POWER_SUPPLY_HEALTH_DEAD;
	else if(flags & BQ27410_FLAG_OTC)
		status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		status = POWER_SUPPLY_HEALTH_GOOD;

	val->intval = status;

	return 0;
}


#define to_bq27410_device_info(x) container_of((x), \
				struct bq27410_device_info, bat);
   
static int bq27410_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	
	struct bq27410_device_info *di = to_bq27410_device_info(psy);
	DBG("Enter:%s %d psp= %d,virtual_battery_enable=%d\n",__FUNCTION__,__LINE__,psp, virtual_battery_enable);
	mutex_lock(&g_bq27410_mutex);
	switch (psp) {

	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27410_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq27410_battery_voltage(di);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27410_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27410_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = bq27410_battery_rsoc_level(di, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27410_battery_temperature(di);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq27410_health_status(di, val);
		break;
	
	default:
		mutex_unlock(&g_bq27410_mutex);
		return -EINVAL;
	}
	mutex_unlock(&g_bq27410_mutex);
	return ret;
}

static void bq27410_powersupply_init(struct bq27410_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27410_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27410_battery_props);
	di->bat.get_property = bq27410_battery_get_property;
}

static void bq27410_battery_update_status(struct bq27410_device_info *di)
{
	power_supply_changed(&di->bat);
}

static void bq27410_battery_work(struct work_struct *work)
{
	struct bq27410_device_info *di = container_of(work, struct bq27410_device_info, work.work); 
	int ret = 0;
	u8 buf[2];
	int battflags = 0;
	static int flag = 0;

	if(flag == 0){
#if 0
		mutex_lock(&g_bq27410_mutex);
		bq27410_write_batt_insert(g_client);
		mutex_unlock(&g_bq27410_mutex);

		virtual_battery_enable = 0;
		flag = 1;
#else
		mutex_lock(&g_bq27410_mutex);
		ret = bq27410_read(g_client,BQ27410_REG_FLAGS, buf, 2);
		if (ret < 0) {
			DBG("error reading flags\n");
			return ret;
		}
		mutex_unlock(&g_bq27410_mutex);
		battflags = get_unaligned_le16(buf);
		if ((battflags & BQ27410_FLAG_BAT_DET)){
			printk("27410: %s, battflags = 0x%x, set virtual_battery_enable = 0\n",__FUNCTION__,battflags);
			flag = 1;
			virtual_battery_enable = 0;
		}
#endif
	}

	bq27410_battery_update_status(di);
	/* reschedule for the next time */
	schedule_delayed_work(&di->work, di->interval);
}

static void bq27410_unseal_device(void)
{
	struct bq27410_device_info *di;
	u8 buf[2];

	if(g_bq27410_mode == BQ27410_NORMAL_MODE){
		di = bq27410_di;
		/*W: AA 00 14 04*/
		buf[0] = 0x14;
		buf[1] = 0x04;
		bq27410_write(di->client,0x00,buf,2);
		/*W: AA 00 72 36*/
		buf[0] = 0x72;
		buf[1] = 0x36;
		bq27410_write(di->client,0x00,buf,2);
		/*W: AA 00 FF FF*/
		buf[0] = 0xFF;
		buf[1] = 0xFF;
		bq27410_write(di->client,0x00,buf,2);
		/*W: AA 00 FF FF*/
		buf[0] = 0xFF;
		buf[1] = 0xFF;
		bq27410_write(di->client,0x00,buf,2);
		/*X: 1000*/
		msleep(1000);
	}
}

static void bq27410_update_enter_rom_mode(void)
{
	struct bq27410_device_info *di;
	u8 buf[6];
	int ret = 2;

	di = bq27410_di;
	if(g_bq27410_mode == BQ27410_NORMAL_MODE){
		/*W: AA 00 00 0F*/
		buf[0] = 0x00;
		buf[1] = 0x0F;
		ret = bq27410_write(di->client,0x00,buf,2);
	}
	/*X: 1000*/
	g_client->addr = 0x0B;//when enter rom mode,the iic addr is 0x0B
	msleep(1000);
}

static void bq27410_exit_rom_mode(void)
{
	struct bq27410_device_info *di;
	u8 buf[2];

	di = bq27410_di;
	if(g_bq27410_mode == BQ27410_NORMAL_MODE){
		/*W: AA 00 14 04*/
		buf[0] = 0x14;
		buf[1] = 0x04;
		bq27410_write(di->client,0x00,buf,2);
		/*W: AA 00 72 36*/
		buf[0] = 0x72;
		buf[1] = 0x36;
		bq27410_write(di->client,0x00,buf,2);
		/*W: AA 00 FF FF*/
		buf[0] = 0xFF;
		buf[1] = 0xFF;
		bq27410_write(di->client,0x00,buf,2);
		/*W: AA 00 FF FF*/
		buf[0] = 0xFF;
		buf[1] = 0xFF;
		bq27410_write(di->client,0x00,buf,2);
		/*X: 1000*/
		msleep(1000);
	}
}

void bq27410_soft_reset(void)
{
	u8 buf[2];

	if(g_bq27410_mode == BQ27410_NORMAL_MODE){
		buf[0] = 0x41;						//soft reset
		buf[1] = 0x00;
		bq27410_write(g_client,0x00,buf,2);
		udelay(66);
	}
}

void bq27410_sealed(void)
{
	int ret = 0;
	u8 buf[2];

	if(g_bq27410_mode == BQ27410_NORMAL_MODE){
		buf[0] = 0x20;						//seal
		buf[1] = 0x00;
		bq27410_write(g_client,0x00,buf,2);
		udelay(66);
		ret = bq27410_read(g_client,0x00,buf,2);
	}
}

/*M-MT:read control status ss bit*/
int bq27410_read_control_status(struct i2c_client *client)
{
	int ret = 0;
	int dev_type = 0;
	int control_status = 0;
	u8 buf[2];

	if(g_bq27410_mode == BQ27410_NORMAL_MODE){
		buf[0] = 0x00;						//CONTROL_STATUS
		buf[1] = 0x00;
		bq27410_write(client,0x00,buf,2);
		udelay(66);
		ret = bq27410_read(client,0x00,buf,2);
		control_status = get_unaligned_le16(buf);
		printk("control_status=0x%x \n",control_status);

		buf[0] = 0x01;						//CONTROL_STATUS
		buf[1] = 0x00;
		bq27410_write(client,0x00,buf,2);
		udelay(66);
		ret = bq27410_read(client,0x00,buf,2);
		dev_type = get_unaligned_le16(buf);
		DBG("dev_type = 0x%x \n",dev_type);

		if((control_status & BQ27410_CONTROL_STATUS_SS) && (control_status & BQ27410_CONTROL_STATUS_FAS)){
			return 1;
		}
		else{
			return 0;
		}
	}
}

/*M-MT:write batt insert command.*/
int bq27410_write_batt_insert(struct i2c_client *client)
{
	int ret = 0;
	int control_status = 0;
	u8 buf[2];
	int flags = 0;

	if(g_bq27410_mode == BQ27410_NORMAL_MODE){
		buf[0] = 0x00;	//CONTROL_STATUS
		buf[1] = 0x00;
		bq27410_write(client,0x00,buf,2);
		udelay(66);
		ret = bq27410_read(client,0x00,buf,2);
		control_status = get_unaligned_le16(buf);
		printk("control_status=0x%x \n",control_status);

		ret = bq27410_read(client,BQ27410_REG_FLAGS, buf, 2);
		if (ret < 0) {
			DBG("error reading flags\n");
			return ret;
		}
		flags = get_unaligned_le16(buf);

		printk("1 Enter:%s --flags = 0x%x\n",__FUNCTION__,flags);

		if (!(flags & BQ27410_FLAG_BAT_DET))
		{
			buf[0] = 0x0c;				//batt insert
			buf[1] = 0x00;
			bq27410_write(client,0x00,buf,2);
			udelay(66);

			ret = bq27410_read(client,BQ27410_REG_FLAGS, buf, 2);
			if (ret < 0) {
				DBG("error reading flags\n");
				return ret;
			}
			flags = get_unaligned_le16(buf);
			if (!(flags & BQ27410_FLAG_BAT_DET)){
				printk("27410: %s, flags = 0x%x, set virtual_battery_enable = 1\n",__FUNCTION__,flags);
				virtual_battery_enable = 1;
			}
		}
	}

	return 0;
}

/*M-MT:setup discharge/charge current threshold.*/
int bq27410_write_current_threshold(struct i2c_client *client)
{
	int ret = 0;
	int dsg_current_threshold = 0;
	u8 buf[16];

	// 1. unseal bq27410.
	bq27410_unseal_device();

	// 2. write BlockDataControl command to enable block data flash control.
	buf[0] = 0x00;
	bq27410_write(client,0x61,buf,1);
	udelay(66);

	// 3. write subclass id.
	buf[0] = 0x51;
	bq27410_write(client,0x3E,buf,1);
	udelay(66);

	// 4. write subclass current offset.
	buf[0] = 0x00;
	bq27410_write(client,0x3F,buf,1);
	udelay(66);

	// 5. read old dsg current.
	ret = bq27410_read(client,0x40,buf,2);
	dsg_current_threshold = get_unaligned_le16(buf);
	DBG("dsg_current_threshold = 0x%x \n",dsg_current_threshold);
#if 0
	ret = bq27410_read(client,0x40,buf,16);
	{
		int i;
		for(i =0; i< 16; i++)
		{
			DBG("0x%x ",buf[i]);
		}
		DBG(" \n");
	}
#endif

	// 6. read old checksum.
	//ret = bq27410_read(client,0x60,buf,1);
	//DBG("checksum = 0x%x \n",buf[0]);
#if 0
	// 7. write new taper current.
	buf[0] = 0x00;
	buf[1] = 0xC8;
	bq27410_write(client,0x40,buf,2);
	udelay(66);
	ret = bq27410_read(client,0x40,buf,2);
	dsg_current_threshold = get_unaligned_le16(buf);
	DBG("dsg_current_threshold =0x%x \n",dsg_current_threshold);	
	// 8. write new checksum.
#endif
	bq27410_soft_reset();
	bq27410_sealed();

	return 0;
}



static struct file * update_file_open(char * path, mm_segment_t * old_fs_p)
{
	struct file * filp = NULL;
	int errno = -1;

	filp = filp_open(path, O_RDONLY, 0644);

	if(!filp || IS_ERR(filp))
	{
		printk(KERN_ERR "The update file for Guitar open error.\n");
		return NULL;
	}
	*old_fs_p = get_fs();
	set_fs(get_ds());

	filp->f_op->llseek(filp,0,0);
	return filp ;
}

static void update_file_close(struct file * filp, mm_segment_t old_fs)
{
	set_fs(old_fs);
	if(filp)
		filp_close(filp, NULL);
}

static int update_get_flen(char * path)
{
	struct file * file_ck = NULL;
	mm_segment_t old_fs;
	int length ;
	
	file_ck = update_file_open(path, &old_fs);
	if(file_ck == NULL)
		return 0;

	length = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
	//printk("File length: %d\n", length);
	if(length < 0)
		length = 0;
	update_file_close(file_ck, old_fs);
	return length;	
}

static int bq27410_update_fw(struct file *filp,  char *buff, unsigned long len, void *data)
{
	struct file * file_data = NULL;
	mm_segment_t old_fs;
	int file_len;
	char *data_buf = NULL;
	int ret;
	char buf[128];
	char *pch = NULL;
	s32 base = 16;
	int count = 0;
	int wCount = 0;
	int xCount = 0;
	int cCount = 0;
	#define isspace(c)	((c) == ' ')

	file_len = update_get_flen(buff);

	///Open update file.
	file_data = update_file_open(buff, &old_fs);
	if(file_data == NULL)
	{
		return -1;
	}
	data_buf = kzalloc(file_len, GFP_KERNEL);

	if(file_len > 0)
	{
		ret = file_data->f_op->read(file_data, data_buf, file_len, &file_data->f_pos);					
		//printk("get file strlen(data_buf): %d.\n", strlen(data_buf));

		pch = data_buf;
		while(count < file_len)
		{
			bq27410_update_flag = 2;
			if(*pch == 'W')
			{
				wCount = 0;
				pch +=2;
				count +=2;
				while(*pch != 'W' && *pch != 'X' && *pch != 'C')
				{
					while(isspace(*pch))
					{
						pch++;
						count ++;
					}

					ret = simple_strtoul(pch, NULL, base);
					buf[wCount] = ret;
					wCount++;
					//printk("%x ", ret);
					pch +=2;
					count +=2;
					while((*pch == '\r') || (*pch == '\n'))
					{
						pch +=2;
						count +=2;
					}
				}
				//printk("\n");
				//printk("\n W count = %d , wCount=%d\n", count, wCount);
				ret = bq27410_write(g_client,buf[1],&buf[2],wCount - 2);
				//printk("W ret =%d ", ret);
				if(ret < 0)
					break;
			}
			else if(*pch == 'X')
			{
				xCount = 0;
				pch +=2;
				count +=2;
				while(*pch != 'W' && *pch != 'X' && *pch != 'C')
				{
					while(isspace(*pch))
					{
						pch++;
						count ++;
					}

					ret = simple_strtoul(pch, NULL, 10);
					//printk("%d ", ret);
					xCount++;

					if(ret < 0x10)
					{
						pch +=1;
						count +=1;
					}
					else if(ret < 0x100){
						pch +=2;
						count +=2;
					}
					else if(ret < 0x1000){
						pch +=3;
						count +=3;		
					}
					else if(ret < 0x10000){
						pch +=4;
						count +=4;		
					}

					while((*pch == '\r') || (*pch == '\n'))
					{
						pch +=2;
						count +=2;
					}

					if(ret == 4000)
					{
						count = file_len;
						break;
					}
				}
				//printk("\n X count = %d, xCount=%d \n", count, xCount);
				//printk("\n");				
				msleep(ret);
			}
			else if(*pch == 'C')
			{
				cCount = 0;
				pch +=2;
				count +=2;
				while(*pch != 'W' && *pch != 'X' && *pch != 'C')
				{
					while(isspace(*pch))
					{
						pch++;
						count ++;
					}

					ret = simple_strtoul(pch, NULL, base);
					buf[cCount] = ret;					
					//printk("%x ", ret);
					cCount++;

					pch +=2;
					count +=2;
					while((*pch == '\r') || (*pch == '\n'))
					{
						pch +=2;
						count +=2;

						break;
					}
				}
				//printk("\n");
				//printk("\n C count = %d , cCount=%d\n", count, cCount);
				ret = bq27410_read(g_client,buf[1],&buf[2],cCount - 2);
				//printk("C ret =%d ", ret);
				if(ret < 0)
					break;
			}
		}
	}

	if(data_buf){
		kfree(data_buf);
		data_buf = NULL;
	}
	///Close file
	update_file_close(file_data, old_fs);

	return ret;
}

static int bq27410_update_flash_data(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	int ret = 0;
	u8 buf[2];
	bq27410_unseal_device();

	bq27410_update_enter_rom_mode();
	ret = bq27410_read(g_client,0x66,buf,2);
	if(ret < 0)
		bq27410_update_flag = 1;
	else
		ret = bq27410_update_fw(filp, buff, len, data);

	g_bq27410_mode = BQ27410_NORMAL_MODE;
	g_client->addr = 0x55;
	bq27410_exit_rom_mode();

	bq27410_soft_reset();
	bq27410_sealed();

	return ret;
}

static int bq27410_update_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	int ret = 0;
	char update_file[100];
	memset(update_file, 0, 100);

	if(copy_from_user(&update_file, buff, len))	
	{		
		return -EFAULT;	
	}

	update_file[len-1] = 0;
	ret = update_get_flen(update_file);
	if(ret == 0){
		printk("bq27410_update_write:update_get_flen error.\n");
		return -EFAULT;
	}

	mutex_lock(&g_bq27410_mutex);

	printk("update file: %d-[%s].\n", len, update_file);
	if(bq27410_update_flash_data(filp, update_file, len, data) < 0)
		ret = bq27410_update_flash_data(filp, update_file, len, data);

	mutex_unlock(&g_bq27410_mutex);

	if(ret)
	{
		//after update firmware ,restart the system.
		printk("%s,after update firmware ,restart the system.\n", __FUNCTION__);
		sys_sync();
		msleep(200);
		bq27410_update_flag = 3;
//		kernel_restart(NULL);
	}
	else{
		printk("bq27410_update_write:update failed.\n");
		bq27410_update_flag = 4;
	}
	return len;

}

static int bq27410_update_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	return snprintf(page, count, "%d\n", bq27410_update_flag);
}

static void bq27410_battery_update_work(struct work_struct *work)
{
	struct file *filp;
	int retval;
	unsigned long len = sizeof(g_filename);
	struct bq27410_device_info *di = container_of(work, struct bq27410_device_info, work.work); 

	retval = update_get_flen(g_filename);
	if(retval == 0){
		printk("bq27410_battery_update_work:update_get_flen error.\n");
		return;
	}

	mutex_lock(&g_bq27410_mutex);
	retval = bq27410_update_flash_data(filp, g_filename, len, NULL);
	mutex_unlock(&g_bq27410_mutex);

	if(retval == 0)
	{
	#if UPDATE_FIRMWARE_RESTART
		printk("bq27410_battery_update_work ,update success,restart the system.\n");
		virtual_battery_enable = 0;
		//after update firmware ,restart the system.
		sys_sync();
		msleep(200);
		kernel_restart(NULL);

		return;
	#else
		printk("bq27410_battery_update_work ,update success.\n");
	#endif
	}
	else{
		printk("bq27410_battery_update_work:update failed.\n");
		return;
	}

	bq27410_write_batt_insert(g_client);
	msleep(2000);
	bq27410_write_batt_insert(g_client);
	
	INIT_DELAYED_WORK(&bq27410_di->work, bq27410_battery_work);
	schedule_delayed_work(&bq27410_di->work, bq27410_di->interval);
}

/**********************
Boot threshold voltage limit is set in the ADC driver,judge here only to prevent 0% capacity but voltage is more than 3.5V
leading to shut down immediately after boot
***********************/
static void battery_capacity_check(struct bq27541_device_info *di)
{
	int rsoc;
	rsoc = bq27410_battery_rsoc(di);
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
	return;
}

static int bq27410_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27410_device_info *di;
	int retval = 0;
	u8 buf[2];
	struct bq27410_platform_data *pdata;
	
	DBG("**********  bq27410_battery_probe**************  \n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		client->addr = 0x0B;
		if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
			return -ENODEV;
		}
		else{
			g_bq27410_mode = BQ27410_ROM_MODE;
		}
	}
	else{
		g_bq27410_mode = BQ27410_NORMAL_MODE;
	}
	printk("+ g_bq27410_mode=%d \n", g_bq27410_mode);
	
	pdata = client->dev.platform_data;
	g_pdata = pdata;
	g_client = client;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	mutex_init(&g_bq27410_mutex);
		

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = "battery";
	di->client = client;
	/* 4 seconds between monotor runs interval */
	di->interval = msecs_to_jiffies(4 * 1000);
	
	di->bat_num = pdata->bat_num;
	di->dc_check_pin = pdata->dc_check_pin;
	di->bat_check_pin = pdata->bat_check_pin;
	di->wake_irq = pdata->low_power_pin;
	
	if (pdata->io_init)
		pdata->io_init();

	bq27410_di = di;

	retval = bq27410_read(client,0x00,buf,2);
	if(retval < 0){
		printk("failed to find bq27410\n");
		goto batt_failed_2;
	}else{
		rk30_bat_unregister();
		bq27410_init = 1;
	}

	//command batt insert.
	bq27410_write_batt_insert(client);
	battery_capacity_check(di);
	if(g_bq27410_mode == BQ27410_NORMAL_MODE){

//		if(!bq27410_read_control_status(client))
//		{
//			virtual_battery_enable = 1;

//			bq27410_powersupply_init(di);
//			retval = power_supply_register(&client->dev, &di->bat);

//			INIT_DELAYED_WORK(&di->update_work, bq27410_battery_update_work);
//			schedule_delayed_work(&di->update_work, msecs_to_jiffies(15 * 1000));
//		}
//		else{
			printk("NOT need bq27410_update_firmware \n");
			bq27410_powersupply_init(di);
			
			retval = power_supply_register(&client->dev, &di->bat);
			if (retval) {
				dev_err(&client->dev, "failed to register battery\n");
				goto batt_failed_4;
			}
			INIT_DELAYED_WORK(&di->work, bq27410_battery_work);
			schedule_delayed_work(&di->work, di->interval);
			dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);
//		}
	}
	else
	{
		INIT_DELAYED_WORK(&di->update_work, bq27410_battery_update_work);
		schedule_delayed_work(&di->update_work, msecs_to_jiffies(15 * 1000));
	}
	printk("- g_bq27410_mode=%d \n", g_bq27410_mode);
	
	//M-MT:setup discharge/charge current threshold. 
	//bq27410_write_current_threshold(client);

	// battery low irq
	if(pdata->low_power_pin != INVALID_GPIO)
	{
		di->wake_irq = gpio_to_irq(pdata->low_power_pin);
		retval = request_irq(di->wake_irq, bq27410_bat_wakeup, IRQF_TRIGGER_FALLING, "bq27410_battery", di);
		if (retval) {
			printk("failed to request low_power_pin irq\n");
			goto err_batirq_failed;
		}
		
		INIT_DELAYED_WORK(&di->wakeup_work, bq27410_battery_wake_work);
		enable_irq_wake(di->wake_irq);
	}
	
	bq27410_proc_entry = create_proc_entry("bq27410-update", 0666, NULL);
	if(bq27410_proc_entry == NULL)
	{
		printk("Malata bq27410 Couldn't create proc entry!\n");
		return -ENOMEM;
	}
	else
	{
		printk("Malata bq27410 Create proc entry success!\n");
		bq27410_proc_entry->write_proc = bq27410_update_write;
		bq27410_proc_entry->read_proc = bq27410_update_read;
	}
	
	return 0;

batt_failed_4:
	kfree(di);
batt_failed_2:

err_batirq_failed:
	gpio_free(pdata->bat_check_pin);

	return retval;
}

static int bq27410_battery_remove(struct i2c_client *client)
{
	struct bq27410_device_info *di = i2c_get_clientdata(client);

	power_supply_unregister(&di->bat);
	kfree(di->bat.name);
	kfree(di);
	return 0;
}

static const struct i2c_device_id bq27410_id[] = {
	{ "bq27410", 0 },
};

static struct i2c_driver bq27410_battery_driver = {
	.driver = {
		.name = "bq27410",
	},
	.probe = bq27410_battery_probe,
	.remove = bq27410_battery_remove,
	.id_table = bq27410_id,
};

static int __init bq27410_battery_init(void)
{
	int ret;
	
	struct proc_dir_entry * battery_proc_entry;
	
	ret = i2c_add_driver(&bq27410_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27410 driver\n");
	
	battery_proc_entry = proc_create("driver/power",0777,NULL,&battery_proc_fops);

	return ret;
}

//module_init(bq27410_battery_init);
//fs_initcall_sync(bq27410_battery_init);

fs_initcall(bq27410_battery_init);
//arch_initcall(bq27410_battery_init);

static void __exit bq27410_battery_exit(void)
{
	i2c_del_driver(&bq27410_battery_driver);
}
module_exit(bq27410_battery_exit);

MODULE_AUTHOR("clb");
MODULE_DESCRIPTION("BQ27410 battery monitor driver");
MODULE_LICENSE("GPL");
