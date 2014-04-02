/* drivers/input/misc/bpw16n.c
 *
 * Copyright (C) 2011 ROCHCHIP CO,.Ltd
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/adc.h>
#include <mach/board.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/circ_buf.h>
#include <linux/timer.h>

#if 1
#define D(x...) printk(x)
#else 
#define D(x...)
#endif 
#define LIGHTSENSOR_IOCTL_MAGIC 'l'
#define LIGHTSENSOR_IOCTL_GET_ENABLED	 _IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *) 
#define LIGHTSENSOR_IOCTL_ENABLE	 _IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *) 
#define LIGHTSENSOR_IOCTL_DISABLE        _IOW(LIGHTSENSOR_IOCTL_MAGIC, 3, int *)

#define SENSOR_OFF 0
#define SENSOR_ON  1

static int LighSensorValue[8]={1000,900,700,400,200,100,40,0};
static int LighSensorLevel_TAB[8]={0,1,2,3,4,5,6,7};
static int LighSensorLevel;
static int LighSensorCount;

static struct bpw16n_data {
	struct adc_client *client;
	struct input_dev *input_dev;
	struct bpw16n_platform_data *pdata;
	struct timer_list	bpw16n_timer;
	struct workqueue_struct *wq;
  struct work_struct timer_work;
  struct delayed_work later_work;
  int adc_val;
	int enabled;

} lightsensor_data;

static int misc_ls_opened = 0;
static int last_index;

static void bpw16n_report(struct input_dev *input, int data)
{
  int i;
  if(data==0||data==1)//或许是数据异常
  {
    return;
  }	
  for(i=0; i<ARRAY_SIZE(LighSensorLevel_TAB); i++)
	{
		if(data > LighSensorValue[i])		
			break;
	}
	if(LighSensorLevel == LighSensorLevel_TAB[i])
  {
		LighSensorCount++;
	}
	else
	{
		LighSensorCount = 0;
		LighSensorLevel = LighSensorLevel_TAB[i];
	}
	
	if(LighSensorCount > 5)
	{
		LighSensorCount = 0;
		input_report_abs(input, ABS_MISC, LighSensorLevel);
	  input_sync(input); 
		printk("bpw16n origin value %d report value: %d\n",data, LighSensorLevel);	
	}
}

static void bpw16n_timer(unsigned long data)
{
  struct bpw16n_data *ip = (struct bpw16n_data *)data;
	queue_work(ip->wq,&ip->timer_work);
}


static void bpw16n_callback(struct adc_client *client, void *param, int result)
{
	 lightsensor_data.adc_val=result;
	 return;
}

static void lightsensor_work_callback(struct work_struct *work)
{
  struct bpw16n_data *ip = container_of(work, struct bpw16n_data,timer_work);
	adc_async_read(ip->client);
	bpw16n_report(ip->input_dev,ip->adc_val);
	if(ip->enabled){	
		mod_timer(&ip->bpw16n_timer,jiffies +HZ/10);
	}
	return;
}

static void lightsensor_later_work_callback(struct work_struct *work)
{
  struct bpw16n_data *ip = container_of(work, struct bpw16n_data,later_work);
  lightsensor_data.enabled=SENSOR_ON;
  lightsensor_data.bpw16n_timer.expires=jiffies+HZ/10;
  add_timer(&lightsensor_data.bpw16n_timer); 
  return;
}

static void bpw16n_setup(struct bpw16n_data *ip)
{
	ip->wq = create_singlethread_workqueue("ls_wq");
	INIT_WORK(&ip->timer_work, lightsensor_work_callback);
        INIT_DELAYED_WORK(&ip->later_work, lightsensor_later_work_callback);
	setup_timer(&ip->bpw16n_timer,bpw16n_timer,(unsigned long)ip);
	ip->bpw16n_timer.expires = jiffies + 20*HZ;
	ip->enabled = SENSOR_OFF;
}

static int bpw16n_open(struct inode *inode, struct file *file)
{
	if(misc_ls_opened)
		return -EBUSY;
	misc_ls_opened = 1;
	return 0;
}

static int bpw16n_release(struct inode *inode, struct file *file)
{
	misc_ls_opened = 0;
	return 0;
}

static long bpw16n_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned long *argp = (unsigned char *)arg;
	switch(cmd){
		case LIGHTSENSOR_IOCTL_GET_ENABLED:
			*argp = lightsensor_data.enabled;
			break;
		case LIGHTSENSOR_IOCTL_ENABLE:
			if(*argp){
			   D("bpw16n_ioctl enable....\n");
			   if(lightsensor_data.enabled==SENSOR_ON)
                              return 0;
                           queue_delayed_work(lightsensor_data.wq,&lightsensor_data.later_work,msecs_to_jiffies(1000));                               	}else{
           D("bpw16n_ioctl disable....\n");
           if(lightsensor_data.enabled==SENSOR_OFF)
             return 0;
           del_timer_sync(&lightsensor_data.bpw16n_timer);
           lightsensor_data.enabled=SENSOR_OFF;			
			}
		 break;
		default:
		 break;
	}
	return 0;
}

static struct file_operations bpw16n_fops = {
	.owner = THIS_MODULE,
	.open = bpw16n_open,
	.release = bpw16n_release,
	.unlocked_ioctl = bpw16n_ioctl
};

struct miscdevice bpw16n_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &bpw16n_fops
};

static int bpw16n_probe(struct platform_device *pdev)
{
 
	int ret = -1;
	struct input_dev *input_dev;
	struct bpw16n_data *ip;
	struct bpw16n_platform_data *pdata = pdata = pdev->dev.platform_data;

	D("%s: probe\n", __func__);
	if (!pdata) {
		pr_err("%s: missing pdata!\n", __func__);
		goto done;
	}
	ip = &lightsensor_data;
  //memset((char *)&ip,0x00,sizeof(struct bpw16n_data));
  
	ip->client = adc_register(pdata->DATA_ADC_CHN,bpw16n_callback, ip);

	D("%s: allocating input device\n", __func__);
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto done;
	}
	ip->input_dev = input_dev;
	ip->pdata = pdata;
	input_set_drvdata(input_dev, ip);
	input_dev->name = "lightsensor-level";

	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_MISC, 0, 10, 0, 0);

	D("%s: registering input device\n", __func__);
	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("%s: could not register input device\n", __func__);
		goto err_free_input_device;
	}

	D("%s: registering misc device\n", __func__);
	ret = misc_register(&bpw16n_misc);
	if (ret < 0) {
		pr_err("%s: could not register misc device\n", __func__);
		goto err_unregister_input_device;
	}
  
	bpw16n_setup(ip);
	return 0;

err_unregister_input_device:
	input_unregister_device(input_dev);
err_free_input_device:
	input_free_device(input_dev);
done:
	return ret;
}

static int bpw16n_remove(struct platform_device *pdev)
{
	input_free_device(lightsensor_data.input_dev);
	input_unregister_device(lightsensor_data.input_dev);
	misc_deregister(&bpw16n_misc);
	return 0;
}

static struct platform_driver bpw16n_driver = {
	.probe		= bpw16n_probe,
	.remove   = bpw16n_remove,
	.driver		= {
		.name	= "rk29_lsensor",
		.owner	= THIS_MODULE
	},
};

static int __init bpw16n_init(void)
{
	return platform_driver_register(&bpw16n_driver);
}

static void __exit bpw16n_exit(void)
{
	platform_driver_unregister(&bpw16n_driver);
}

module_init(bpw16n_init);
module_exit(bpw16n_exit);

