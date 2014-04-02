/* drivers/video/backlight/rk29_backlight.c
 *
 * Copyright (C) 2009-2011 Rockchip Corporation.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/clk.h>

#include <linux/earlysuspend.h>
#include <asm/io.h>
#include <mach/board.h>
#ifndef CONFIG_PWM_DRIVER_NEW
#include "rk2818_backlight.h"
#else
#include <plat/pwm.h>

#define PWM_DIV              PWM_DIV2
#if  defined(CONFIG_MACH_RK30_DS1001B)
#define PWM_APB_PRE_DIV      20000
#elif defined(CONFIG_LCD_EJ101IA01G)
#define PWM_APB_PRE_DIV      20000
#else
#define PWM_APB_PRE_DIV      1000
#endif
#define BL_STEP              255
#define BL_SYS_MIN 20
#endif

/*
 * Debug
 */
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif

#ifndef CONFIG_PWM_DRIVER_NEW
#if defined(CONFIG_ARCH_RK30)
#define write_pwm_reg(id, addr, val)        __raw_writel(val, addr+(RK30_PWM01_BASE+(id>>1)*0x20000)+id*0x10)
#define read_pwm_reg(id, addr)              __raw_readl(addr+(RK30_PWM01_BASE+(id>>1)*0x20000+id*0x10))
#elif defined(CONFIG_ARCH_RK29)
#define write_pwm_reg(id, addr, val)        __raw_writel(val, addr+(RK29_PWM_BASE+id*0x10))
#define read_pwm_reg(id, addr)              __raw_readl(addr+(RK29_PWM_BASE+id*0x10))    
#endif
#else
#define read_pwm_reg(addr)              __raw_readl(pwm_base + addr)
#endif

#if defined(CONFIG_MALATA_D8005)
#define KERNEL_BL_PWM_MIN 5
#define KERNEL_BL_PWM_MAX 255
#elif defined(CONFIG_MALATA_D7008)
#define KERNEL_BL_PWM_MIN 5
#define KERNEL_BL_PWM_MAX 190
#elif defined(CONFIG_MALATA_D7007)
#define KERNEL_BL_PWM_MIN 5
#define KERNEL_BL_PWM_MAX 240
#elif defined(CONFIG_MALATA_C7022)
#define KERNEL_BL_PWM_MIN 5
#define KERNEL_BL_PWM_MAX 177
#else
#define KERNEL_BL_PWM_MIN 13
#define KERNEL_BL_PWM_MAX 255
#endif
static struct clk *pwm_clk;
#ifdef CONFIG_PWM_DRIVER_NEW
static void __iomem *pwm_base;
#endif
static struct backlight_device *rk29_bl;
static int suspend_flag = 0;
static u32 sys_bright_save = KERNEL_BL_PWM_MIN;
static int close_lcd = 0;
static int brightness_save = 0;
static int bl_init = 0;

int convertint(char s[])  
{  
    int i;  
    int n = 0;  
    for (i = 0; s[i] >= '0' && s[i] <= '9'; ++i)  
    {  
        n = 10 * n + (s[i] - '0');  
    }  
    return n;  
} 

static ssize_t backlight_write(struct device *dev,
		struct device_attribute *attr, char *buf)
{
   
	struct rk29_bl_info *rk29_bl_info = bl_get_data(rk29_bl);
	int number;

	number = convertint(buf);
	
	rk29_bl_info->min_brightness=number;
	return 0;
}


static ssize_t backlight_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct rk29_bl_info *rk29_bl_info = bl_get_data(rk29_bl);

	DBG("rk29_bl_info->min_brightness=%d\n",rk29_bl_info->min_brightness);
}
static DEVICE_ATTR(rk29backlight, 0660, backlight_read, backlight_write);

static DEFINE_MUTEX(backlight_mutex);

static int rk29_bl_update_status(struct backlight_device *bl)
{
	u32 divh,div_total;
	struct rk29_bl_info *rk29_bl_info = bl_get_data(bl);
	u32 id = rk29_bl_info->pwm_id;
	u32 ref = rk29_bl_info->bl_ref;
	int brightness = 0;
	u32 k_bl_range,sys_bl_range,k_bl;
	
	mutex_lock(&backlight_mutex);
//BL_CORE_DRIVER2 is the flag if backlight is into early_suspend.
	if (suspend_flag && (bl->props.state & BL_CORE_DRIVER2))
	    goto out;
	brightness = bl->props.brightness;
	if(rk29_bl_info->min_brightness){
	    if(brightness){
	    	if(brightness < rk29_bl_info->min_brightness)
			brightness = rk29_bl_info->min_brightness;
	    }
	    if(brightness > 255)
	    	brightness = 255;
	}

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;	

	if (bl->props.state & BL_CORE_DRIVER3)
		brightness = 0;	

	if ((bl->props.state & BL_CORE_DRIVER2) && !suspend_flag ){
		brightness = 0;
		suspend_flag = 1;
	}else if(!(bl->props.state & BL_CORE_DRIVER2) && suspend_flag ){
		suspend_flag = 0;
	}

//BL_CORE_DRIVER1 is the flag if backlight pwm is closed.
	if ((bl->props.state & BL_CORE_DRIVER1) && brightness ==0 ){  
		bl->props.state &= ~BL_CORE_DRIVER1;
		clk_disable(pwm_clk);
		if (rk29_bl_info->pwm_suspend)
			rk29_bl_info->pwm_suspend();
	}else if(!(bl->props.state & BL_CORE_DRIVER1) && brightness != 0){
		bl->props.state |= BL_CORE_DRIVER1;
		if (rk29_bl_info->pwm_resume)
			rk29_bl_info->pwm_resume();
		clk_enable(pwm_clk);
		msleep(1);
	}
	if(brightness == 0) {
		k_bl =0;
	} else if(brightness < BL_SYS_MIN) {
		sys_bright_save = brightness;
		k_bl = KERNEL_BL_PWM_MIN;
	} else {
		sys_bl_range = BL_STEP - BL_SYS_MIN;
		k_bl_range = KERNEL_BL_PWM_MAX -KERNEL_BL_PWM_MIN;
		k_bl = KERNEL_BL_PWM_MIN +  (brightness - BL_SYS_MIN) * k_bl_range / sys_bl_range;
	}
#ifdef CONFIG_PWM_DRIVER_NEW
	div_total = read_pwm_reg(PWM_REG_LRC);
#else
	div_total = read_pwm_reg(id, PWM_REG_LRC);
#endif
	if (ref) {
		divh = div_total * k_bl / BL_STEP;
	} else {
		divh = div_total * (BL_STEP - k_bl) / BL_STEP;
	}
#ifdef CONFIG_PWM_DRIVER_NEW
	rk_pwm_setup(id, PWM_DIV, divh, div_total);
#else
	write_pwm_reg(id, PWM_REG_HRC, divh);
#endif

	if(close_lcd == 0)
		brightness_save = bl->props.brightness;
	DBG("%s:line=%d,brightness = %d, div_total = %d, divh = %d state=%x \n",__FUNCTION__,__LINE__,brightness, div_total, divh,bl->props.state);
out:
	mutex_unlock(&backlight_mutex);
	return 0;
}

static int rk29_bl_get_brightness(struct backlight_device *bl)
{
	u32 divh,div_total;
	struct rk29_bl_info *rk29_bl_info = bl_get_data(bl);
#ifndef CONFIG_PWM_DRIVER_NEW
	u32 id = rk29_bl_info->pwm_id;
#endif
	u32 ref = rk29_bl_info->bl_ref;
	u32 k_bl_range,sys_bl_range,k_bl,sys_bl;

#ifdef CONFIG_PWM_DRIVER_NEW
	div_total = read_pwm_reg(PWM_REG_LRC);
	divh = read_pwm_reg(PWM_REG_HRC);
#else
	div_total = read_pwm_reg(id, PWM_REG_LRC);
	divh = read_pwm_reg(id, PWM_REG_HRC);
#endif
	sys_bl_range = BL_STEP - BL_SYS_MIN;
	k_bl_range = KERNEL_BL_PWM_MAX -KERNEL_BL_PWM_MIN;
	if (!div_total)
		return 0;
	if(ref)
		k_bl = BL_STEP * divh / div_total;
	else
		k_bl = BL_STEP - (BL_STEP * divh /div_total);
	if(k_bl > KERNEL_BL_PWM_MIN)
		sys_bl = BL_SYS_MIN + (k_bl - KERNEL_BL_PWM_MIN) * sys_bl_range / k_bl_range;
	else if (k_bl > 0)
		sys_bl = sys_bright_save;
	else
		sys_bl = 0;
	return sys_bl;
}

static struct backlight_ops rk29_bl_ops = {
	.update_status	= rk29_bl_update_status,
	.get_brightness	= rk29_bl_get_brightness,
};

static void rk29_backlight_work_func(struct work_struct *work)
{
	rk29_bl_update_status(rk29_bl);
}
static DECLARE_DELAYED_WORK(rk29_backlight_work, rk29_backlight_work_func);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void rk29_bl_suspend(struct early_suspend *h)
{
	struct rk29_bl_info *rk29_bl_info = bl_get_data(rk29_bl);
	int brightness = rk29_bl->props.brightness;

	cancel_delayed_work_sync(&rk29_backlight_work);

	rk29_bl->props.state |= BL_CORE_DRIVER2;

	if (rk29_bl->props.brightness) {
		rk29_bl->props.brightness = 0;
		rk29_bl_update_status(rk29_bl);
		rk29_bl->props.brightness = brightness;
	}

}

static void rk29_bl_resume(struct early_suspend *h)
{
	struct rk29_bl_info *rk29_bl_info = bl_get_data(rk29_bl);
	DBG("%s : %s\n", __FILE__, __FUNCTION__);
	rk29_bl->props.state &= ~BL_CORE_DRIVER2;
	
	schedule_delayed_work(&rk29_backlight_work, msecs_to_jiffies(rk29_bl_info->delay_ms));
}

static struct early_suspend bl_early_suspend = {
	.suspend = rk29_bl_suspend,
	.resume = rk29_bl_resume,
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1,
};

bool rk29_get_backlight_status()
{
	return (rk29_bl->props.state & BL_CORE_DRIVER3)?true:false;
}
EXPORT_SYMBOL(rk29_get_backlight_status);

void rk29_backlight_set(bool on)
{
	printk("%s: set %d\n", __func__, on);
	if(!bl_init)
		return;

	if(on){
		rk29_bl->props.state &= ~BL_CORE_DRIVER3;
		rk29_bl_update_status(rk29_bl);
	}else{
		rk29_bl->props.state |= BL_CORE_DRIVER3;
		rk29_bl_update_status(rk29_bl);
	}
	return;
}
EXPORT_SYMBOL(rk29_backlight_set);

void rk29_backlight_set_old(bool on)
{
	if(!on){
		rk29_bl->props.brightness = 0;
		close_lcd = 1;
	}
	else{
		rk29_bl->props.brightness = brightness_save;
		close_lcd = 0;
	}
	rk29_bl_update_status(rk29_bl);

	return;
}
EXPORT_SYMBOL(rk29_backlight_set_old);
#endif

static int rk29_backlight_probe(struct platform_device *pdev)
{		
	int ret = 0;
	struct rk29_bl_info *rk29_bl_info = pdev->dev.platform_data;
	u32 id  =  rk29_bl_info->pwm_id;
	u32 divh, div_total;
	unsigned long pwm_clk_rate;
	struct backlight_properties props;

	if (rk29_bl) {
		printk(KERN_CRIT "%s: backlight device register has existed \n",
				__func__);
		return -EEXIST;		
	}

	if (!rk29_bl_info->delay_ms)
		rk29_bl_info->delay_ms = 100;

	if (rk29_bl_info->min_brightness < 0 || rk29_bl_info->min_brightness > BL_STEP)
		rk29_bl_info->min_brightness = 52;

	if (rk29_bl_info && rk29_bl_info->io_init) {
		rk29_bl_info->io_init();
	}

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = BL_STEP;
	rk29_bl = backlight_device_register("rk28_bl", &pdev->dev, rk29_bl_info, &rk29_bl_ops, &props);
	if (!rk29_bl) {
		printk(KERN_CRIT "%s: backlight device register error\n",
				__func__);
		return -ENODEV;		
	}

#ifdef CONFIG_PWM_DRIVER_NEW
	pwm_base = rk_pwm_get_base(id);
	pwm_clk = rk_pwm_get_clk(id);
#else
#if defined(CONFIG_ARCH_RK29)
	pwm_clk = clk_get(NULL, "pwm");
#elif defined(CONFIG_ARCH_RK30)
	if (id == 0 || id == 1)
		pwm_clk = clk_get(NULL, "pwm01");
	else if (id == 2 || id == 3)
		pwm_clk = clk_get(NULL, "pwm23");
#endif
#endif

	if (IS_ERR(pwm_clk)) {
		printk(KERN_ERR "failed to get pwm clock source\n");
		return -ENODEV;
	}
	pwm_clk_rate = clk_get_rate(pwm_clk);
	div_total = pwm_clk_rate / PWM_APB_PRE_DIV;

	div_total >>= (1 + (PWM_DIV >> 9));
	div_total = (div_total) ? div_total : 1;

	if(rk29_bl_info->bl_ref) {
		divh = 0;
	} else {
		divh = div_total;
	}

	clk_enable(pwm_clk);
#ifdef CONFIG_PWM_DRIVER_NEW
	rk_pwm_setup(id, PWM_DIV, divh, div_total);
#else
	write_pwm_reg(id, PWM_REG_CTRL, PWM_DIV|PWM_RESET);
	write_pwm_reg(id, PWM_REG_LRC, div_total);
	write_pwm_reg(id, PWM_REG_HRC, divh);
	write_pwm_reg(id, PWM_REG_CNTR, 0x0);
	write_pwm_reg(id, PWM_REG_CTRL, PWM_DIV|PWM_ENABLE|PWM_TIME_EN);
#endif

	rk29_bl->props.power = FB_BLANK_UNBLANK;
	rk29_bl->props.fb_blank = FB_BLANK_UNBLANK;
	rk29_bl->props.brightness = BL_STEP / 2;
	rk29_bl->props.state = BL_CORE_DRIVER1;		

	schedule_delayed_work(&rk29_backlight_work, msecs_to_jiffies(0));
	ret = device_create_file(&pdev->dev,&dev_attr_rk29backlight);
	if(ret)
	{
		dev_err(&pdev->dev, "failed to create sysfs file\n");
	}

	register_early_suspend(&bl_early_suspend);

	bl_init = 1;

	printk("RK29 Backlight Driver Initialized.\n");
	return ret;
}

static int rk29_backlight_remove(struct platform_device *pdev)
{		
	struct rk29_bl_info *rk29_bl_info = pdev->dev.platform_data;

	if (rk29_bl) {
		backlight_device_unregister(rk29_bl);
		unregister_early_suspend(&bl_early_suspend);
		clk_disable(pwm_clk);
		clk_put(pwm_clk);
		if (rk29_bl_info && rk29_bl_info->io_deinit) {
			rk29_bl_info->io_deinit();
		}
		return 0;
	} else {
		DBG(KERN_CRIT "%s: no backlight device has registered\n",
				__func__);
		return -ENODEV;
	}
}

static void rk29_backlight_shutdown(struct platform_device *pdev)
{
	struct rk29_bl_info *rk29_bl_info = pdev->dev.platform_data;

	unregister_early_suspend(&bl_early_suspend);
/*
	rk29_bl->props.brightness >>= 1;
	rk29_bl_update_status(rk29_bl);
	mdelay(100);

	rk29_bl->props.brightness >>= 1;
	rk29_bl_update_status(rk29_bl);
	mdelay(100);
*/
	rk29_bl->props.brightness = 0;
	rk29_bl_update_status(rk29_bl);

	if (rk29_bl_info && rk29_bl_info->io_deinit)
		rk29_bl_info->io_deinit();
}

static struct platform_driver rk29_backlight_driver = {
	.probe	= rk29_backlight_probe,
	.remove = rk29_backlight_remove,
	.driver	= {
		.name	= "rk29_backlight",
		.owner	= THIS_MODULE,
	},
	.shutdown	= rk29_backlight_shutdown,
};

static int __init rk29_backlight_init(void)
{
	platform_driver_register(&rk29_backlight_driver);
	return 0;
}
fs_initcall_sync(rk29_backlight_init);
