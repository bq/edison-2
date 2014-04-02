/* drivers/input/sensors/temperature/tmp_ms5607.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>
#include <linux/syscalls.h>
#include <linux/fs.h>


#define	TMP108_TEMP_REG			0x00
#define	TMP108_CONF_REG			0x01
/* note: these bit definitions are byte swapped */
#define		TMP108_CONF_TM		0x0400
#define		TMP108_CONF_POL		0x0080
#define		TMP108_CONF_M0		0x0100
#define		TMP108_CONF_M1		0x0200
#define		TMP108_CONF_HYS0		0x0010
#define		TMP108_CONF_HYS1		0x0020
#define		TMP108_CONF_CR0		0x2000
#define		TMP108_CONF_CR1		0x4000

#define	TMP108_TLOW_REG			0x02
#define	TMP108_THIGH_REG		0x03

#define TMP108_SPEED 			100 * 1000

#define TMP108_CONFIG  (TMP108_CONF_M1 /*| TMP108_CONF_HYS0 |TMP108_CONF_M1 | TMP108_CONF_CR1*/)

static int tmp108_temp = 0;
static struct kobject *android_tmp108_kobj;
static struct i2c_client *t_client;
#if defined (CONFIG_TMP108) && defined(CONFIG_MALATA_D8009)
int tmp108_init;
#endif


static int tmp108_read(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	ret = i2c_master_reg8_recv(client, reg, buf, len, TMP108_SPEED);
	return ret; 
}

static int tmp108_write(struct i2c_client *client, u8 reg, u8 const buf[], unsigned len)
{
	int ret; 
	ret = i2c_master_reg8_send(client, reg, buf, (int)len, TMP108_SPEED);
	return ret;
}

static int sensor_tmp108_suspend(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);
	u8 buf[2] ={0};
	int result;

	buf[0]=0x24;
	buf[1]=0x10;
	result=tmp108_write(client,TMP108_CONF_REG,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
	}
	//printk("yemk sensor_tmp108_suspend buf[0]=0x%x buf[1]=0x%x\n",buf[0],buf[1]);

	return 0;
}

static int sensor_tmp108_resume(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);
	u8 buf[2] ={0};
	int result;

	buf[0]=0x26;
	buf[1]=0x10;
	result=tmp108_write(client,TMP108_CONF_REG,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
	}
	//printk("yemk sensor_tem108_resume buf[0]=0x%x buf[1]=0x%x\n",buf[0],buf[1]);

	return 0;
}

int tmp108_reg_to_mC(void)
{
	int result;
	u16 data;
	int t_tem;
	u8 buf[2] ={0};

	if(t_client){
		result=tmp108_read(t_client,0x00,buf,2);
		if (result < 0){
		dev_err(&t_client->dev, "error reading config register\n");
		return result;
		}
	}else{
		return 4500;
	}

	if(buf[0]&0x1000){
		data=(u16)buf[0]<<4|buf[1]>>4;
		data=(int)(~(data-0x1))*625/100;
		t_tem=-data;	
	}else{
	//data=(u16)buf[0]<<4|buf[1]>>4;
	data=(u16)(buf[0]<<4) + (buf[1]>>4);
	data=(int)data;
	t_tem=(int)(data*625/100);
	//printk("yemk:data %d t_tem=%d 0x%x 0x%x\n",data,t_tem,(buf[0]<<4),(buf[1]>>4));
	}
	return t_tem;
}
EXPORT_SYMBOL(tmp108_reg_to_mC);
/*
static inline u16 tmp108_mC_to_reg(int val)
{

	//return val *10/625;
}
*/
static ssize_t tmp108_check_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", tmp108_temp);
}
static DEVICE_ATTR(tmpcheck, 0664, tmp108_check_show, NULL);

static int tmp108_sysfs_init(void)
{
	int ret ;

	android_tmp108_kobj = kobject_create_and_add("android_tmp108", NULL);
	if (android_tmp108_kobj == NULL) {
		printk(KERN_ERR
		       "TMP108 tmp108_sysfs_init:"\
		       "subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(android_tmp108_kobj, &dev_attr_tmpcheck.attr);
	if (ret) {
		printk(KERN_ERR
		       "TMP108 tmp108_sysfs_init:"\
		       "sysfs_create_group failed\n");
		goto err4;
	}
	return 0 ;
err4:
	kobject_del(android_tmp108_kobj);
err:
	return ret ;
}

/****************operate according to sensor chip:start************/

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	int result = 0;

	if(enable){
		//printk("yemk:sensor_active enable\n");
	}else{
		//printk("yemk:sensor_active disble\n");
	}
	
	return result;
}



static int sensor_init(struct i2c_client *client)
{	
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	u8 buf[2] ={0};

	result = sensor->ops->active(client,0,0);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	
	result=tmp108_read(client,TMP108_CONF_REG,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
	}
	
	buf[0]=0x26;
	buf[1]=0x10;
	result=tmp108_write(client,TMP108_CONF_REG,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
	}

	result=tmp108_read(client,TMP108_CONF_REG,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
	}
	/*
	result=tmp108_read(client,0x00,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
	}
	*/
	buf[0]=0x00;
	buf[1]=0x00;
	result=tmp108_write(client,0x02,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
}

	result=tmp108_read(client,0x02,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
	}

	buf[0]=0x19;
	buf[1]=0x00;
	result=tmp108_write(client,0x03,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
	}

	result=tmp108_read(client,0x03,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
	}

	t_client=client;
#if defined (CONFIG_TMP108) && defined(CONFIG_MALATA_D8009)
	if(t_client)
		tmp108_init = 1;
	else
		tmp108_init= 0;
#endif

	sensor->status_cur = SENSOR_OFF;

	return result;
}



static int temperature_report_value(struct input_dev *input, int data)
{
	//get temperature, high and temperature from register data
	input_report_abs(input, ABS_THROTTLE, data);
	input_sync(input);
	
	return 0;
}


static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0; 
	u8 buf[2] ={0};
	u16 data;
	int t_tem;

	tmp108_temp=tmp108_reg_to_mC();
	result=tmp108_read(client,0x01,buf,2);
	if (result < 0) {
		dev_err(&client->dev, "error reading config register\n");
		return result;
	}
	
	temperature_report_value(sensor->input_dev, tmp108_temp);


	return result;
}


struct sensor_operate temperature_tmp108_ops = {
	.name				= "tmp108",
	.type				= SENSOR_TYPE_TEMPERATURE,	//sensor type and it should be correct
	.id_i2c				= TEMPERATURE_ID_TMP108_l,	//i2c id number
	.read_reg			= SENSOR_UNKNOW_DATA,	//read data
	.read_len			= 3,			//data length
	.id_reg				= SENSOR_UNKNOW_DATA,	//read device id from this register
	.id_data 			= SENSOR_UNKNOW_DATA,	//device id
	.precision			= 24,			//8 bits
	.ctrl_reg 			= SENSOR_UNKNOW_DATA,	//enable or disable 
	.int_status_reg 		= SENSOR_UNKNOW_DATA,	//intterupt status register
	.range				= {100,65535},		//range
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,		
	.active				= sensor_active,	
	.init				= sensor_init,
	.report				= sensor_report_value,
	.suspend            = sensor_tmp108_suspend,
	.resume            = sensor_tmp108_resume,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *temperature_get_ops(void)
{
	return &temperature_tmp108_ops;
}


static int __init temperature_tmp108_init(void)
{
	struct sensor_operate *ops = temperature_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, temperature_get_ops);
	result = tmp108_sysfs_init();
	return result;
}

static void __exit temperature_tmp108_exit(void)
{
	struct sensor_operate *ops = temperature_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, temperature_get_ops);
}


module_init(temperature_tmp108_init);
module_exit(temperature_tmp108_exit);

