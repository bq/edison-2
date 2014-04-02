/* drivers/input/sensors/access/kxtik.c
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

#if 0
#define SENSOR_DEBUG_TYPE SENSOR_TYPE_LIGHT
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

#define CONVERSION_TIME_MS		500

#define ISL5151_ADDR_COM1	0x01
#define ISL5151_ADDR_DATA_MSB	0x02
#define ISL5151_ADDR_DATA_LSB	0x03

/****************operate according to sensor chip:start************/

static bool ISL5151_write_data(struct i2c_client *client, u8 reg,
	u8 val, u8 mask, u8 shift)
{
	u8 regval;
	int ret = 0;
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	

	regval = sensor_read_reg(client, reg);
	regval &= ~mask;
	regval |= val << shift;

//	sensor->client->addr = reg;		
	sensor->ops->ctrl_data = regval;
	ret = sensor_write_reg(client, reg, regval);
	DBG("%s, reg:%d, regval:%d\n", __FUNCTION__, reg, regval);
	if (ret) {
		dev_err(&client->dev, "Write to device fails status %x\n", ret);
		return false;
	}
	return true;
}

#if 0
static bool ISL5151_set_range(struct i2c_client *client, unsigned long range,
		unsigned int *new_range)
{
	unsigned long supp_ranges[] = {1000, 4000, 16000, 64000};
	int i;

	for (i = 0; i < (ARRAY_SIZE(supp_ranges) -1); ++i) {
		if (range <= supp_ranges[i])
			break;
	}
	*new_range = (unsigned int)supp_ranges[i];
	DBG("%s, i:%d\n", __FUNCTION__, i);

	return ISL5151_write_data(client, ISL5151_ADDR_COM2,
		i, COMMANDII_RANGE_MASK, COMMANDII_RANGE_SHIFT);
}

static bool ISL5151_set_resolution(struct i2c_client *client,
			unsigned long adcbit, unsigned int *conf_adc_bit)
{
	unsigned long supp_adcbit[] = {16, 12, 8, 4};
	int i;

	for (i = 0; i < (ARRAY_SIZE(supp_adcbit)); ++i) {
		if (adcbit == supp_adcbit[i])
			break;
	}
	*conf_adc_bit = (unsigned int)supp_adcbit[i];

	return ISL5151_write_data(client, ISL5151_ADDR_COM2,
		i, COMMANDII_RESOLUTION_MASK, COMMANDII_RESOLUTION_SHIFT);
}
#endif

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	int status = 0;
	
//	sensor->client->addr = sensor->ops->ctrl_reg;	
	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	
	//register setting according to chip datasheet		
	if(!enable)
	{	
		sensor->ops->ctrl_data &= (~0x80);	
	}
	else
	{
		sensor->ops->ctrl_data |= 0x80;
	}

	DBG("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n",__func__);
	
	return result;

}


static int sensor_init(struct i2c_client *client)
{	
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	int new_adc_bit;
	unsigned int new_range;
	
	result = sensor->ops->active(client,0,0);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	
	sensor->status_cur = SENSOR_OFF;
	
#if 0
	sensor->client->addr = sensor->ops->ctrl_reg;		
	sensor->ops->ctrl_data = ISL5151_COM1_VALUE;	
	result = sensor_write_reg_normal(client, sensor->ops->ctrl_data);

	sensor->client->addr = ISL5151_ADDR_COM2;	
	result = sensor_write_reg_normal(client, ISL5151_COM2_VALUE);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}

	result = ISL5151_set_range(client, ISL5151_CHIP_RANGE, &new_range);
	if (result)
		result = ISL5151_set_resolution(client, ISL5151_ADC_BIT,
			&new_adc_bit);
				
	if(!result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
#endif		
	
	return result;
}


static void light_report_value(struct input_dev *input, int data)
{
	static unsigned char index = 0;
	static int d0=0,d1=0,d2=0,d3=0,d4=0,d5=0,d6=0,d7=0;
	//printk("yemk:light_report_value data %d\n",data);
		if(data <= 79){
		d0++;
        if(d0==2){
		 d0=0;
		 index = 0;goto report;
         }
		d1=0;
		d2=0;
		d3=0;
		d4=0;
		d5=0;
		d6=0;
		d7=0;
	}
	else if(data <= 109){
		d1++;
		if(d1==2){
		d1=0;
		index = 1;goto report;
		}
		d0=0;
		d2=0;
		d3=0;
		d4=0;
		d5=0;
		d6=0;
		d7=0;
	}
	else if(data <= 139){
		d2++;
		if(d2==2){
		d2=0;
		index = 2;goto report;
		}
		d0=0;
		d1=0;
		d3=0;
		d4=0;
		d5=0;
		d6=0;
		d7=0;
	}
	else if(data <= 179){
		d3++;
		if(d3==2){
		d3=0;
		index = 3;goto report;
		}
		d0=0;
		d1=0;
		d2=0;
		d4=0;
		d5=0;
		d6=0;
		d7=0;
	}
	else if(data <= 359){
		d4++;
		if(d4==2){
		d4=0;
		index = 4;goto report;
		}
		d0=0;
		d1=0;
		d2=0;
		d3=0;
		d5=0;
		d6=0;
		d7=0;
	}
	else if(data <= 429){
		d5++;
		if(d5==2){
		d5=0;
		index = 5;goto report;
		}
		d0=0;
		d1=0;
		d2=0;
		d3=0;
		d4=0;
		d6=0;
		d7=0;
	}
	else if(data <= 510){
		d6++;
		if(d6==2){
		d6=0;
		index = 6;goto report;
		}
		d0=0;
		d1=0;
		d2=0;
		d3=0;
		d4=0;
		d5=0;
		d7=0;
	}
	else{
		d7++;
		if(d7==2){
		d7=0;
		index = 7;goto report;
		}
		d0=0;
		d1=0;
		d2=0;
		d3=0;
		d4=0;
		d5=0;
		d6=0;
	}
report:
	DBG("ISL5151 report data=%d,index = %d\n",data,index);
	input_report_abs(input, ABS_MISC, index);
	input_sync(input);
}


static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	char msb = 0, lsb = 0;
	bool status;
	static int i = 0;
	static int res_flag = 0;
	
	lsb = sensor_read_reg(sensor->client, ISL5151_ADDR_DATA_LSB);
	msb = sensor_read_reg(sensor->client, ISL5151_ADDR_DATA_MSB);
	result = ((msb << 1) | ((lsb  >> 7 ) & 0x01)) & 0xffff;

	res_flag=res_flag+result;
	i++;
	//printk("yemk res_flag=%d,i=%d,result=%d, lsb:%d, msb:%d\n",res_flag,i,result, lsb, msb);

	if(i==2){
		result=res_flag/2;
		i=0;
		res_flag=0;
		light_report_value(sensor->input_dev, result);
	}

	return result;
}


struct sensor_operate light_lsl5151_ops = {
	.name				= "ls_lsl5151",
	.type				= SENSOR_TYPE_LIGHT,	//sensor type and it should be correct
	.id_i2c				= LIGHT_ID_ISL5151_l,	//i2c id number
	.read_reg			= ISL5151_ADDR_DATA_LSB,	//read data
	.read_len			= 2,			//data length
	.id_reg				= SENSOR_UNKNOW_DATA,	//read device id from this register
	.id_data 			= SENSOR_UNKNOW_DATA,	//device id
	.precision			= 9,			//8 bits
	.ctrl_reg 			= ISL5151_ADDR_COM1,	//enable or disable 
	.int_status_reg 		= SENSOR_UNKNOW_DATA,	//intterupt status register
	.range				= {0,10},		//range
	.brightness                     ={10,255},     //brightness
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT,		
	.active				= sensor_active,	
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
struct sensor_operate *light_get_ops(void)
{
	return &light_lsl5151_ops;
}

static int __init light_isl5151_init(void)
{
	struct sensor_operate *ops = light_get_ops();
	int result = 0;
	int type = ops->type;	
	result = sensor_register_slave(type, NULL, NULL, light_get_ops);
	printk("%s, type=%d\n",__func__, type);
	return result;
}

static void __exit light_isl5151_exit(void)
{
	struct sensor_operate *ops = light_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, light_get_ops);
}


module_init(light_isl5151_init);
module_exit(light_isl5151_exit);


