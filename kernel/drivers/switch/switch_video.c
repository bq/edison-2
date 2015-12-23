/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/switch_video.h>

static int sw1_pin;
static int sw2_pin;

static ssize_t switch_video_write( struct file *filp, const char __user *buff,
                        size_t len, loff_t *data )
{
	char switch_video_data[32];  
	if (copy_from_user( &switch_video_data, buff, len )) {
		printk("switch_video_write error.\n");
		return -EFAULT;
	}

	if (switch_video_data[0] == '1') {
		gpio_set_value(sw1_pin, 0);
		gpio_set_value(sw2_pin, 0);
		printk("switch video to 3188 hdmi\n");
	} else if (switch_video_data[0] == '2') {
		gpio_set_value(sw1_pin, 0);
		gpio_set_value(sw2_pin, 1);
		printk("switch video to external hdmi\n");
	}
    
	return len;
}

static const struct file_operations switch_video_fops = { 
	.write = switch_video_write,
};

int init_proc_log(void)
{
	int ret=0;

	proc_create("switch_video", 0644, NULL, &switch_video_fops);
//	printk( "proc_create switch_video_fops\n");

	return ret;
}


static int video_switch_probe(struct platform_device *pdev)
{
	struct switch_video_data *pdata = pdev->dev.platform_data;

//	printk("video_switch_probe\n");

	if (!pdata)
		return -EBUSY;

	if(pdata->io_init)
		pdata->io_init();

	sw1_pin = pdata->source_switch_1;
	sw2_pin = pdata->source_switch_2;

	init_proc_log();

	return 0;
}

static int __devexit video_switch_remove(struct platform_device *pdev)
{
	gpio_free(sw1_pin);
	gpio_free(sw2_pin);

	return 0;
}

static struct platform_driver video_switch_driver = {
	.probe		= video_switch_probe,
	.remove	= __devexit_p(video_switch_remove),
	.driver		= {
		.name	= "switch-video",
		.owner	= THIS_MODULE,
	},
};

static int __init video_switch_init(void)
{
	return platform_driver_register(&video_switch_driver);
}

static void __exit video_switch_exit(void)
{
	platform_driver_unregister(&video_switch_driver);
}

module_init(video_switch_init);
module_exit(video_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
