#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/circ_buf.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <mach/iomux.h>
#include <mach/gpio.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/me906e.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>

MODULE_LICENSE("GPL");

#define DEBUG
#ifdef DEBUG
#define MODEMDBG(x...) printk(x)
#else
#define MODEMDBG(fmt,argss...)
#endif
#define SLEEP 1
#define READY 0
static struct wake_lock modem_wakelock;
#define IRQ_BB_WAKEUP_AP_TRIGGER    IRQF_TRIGGER_FALLING
//#define IRQ_BB_WAKEUP_AP_TRIGGER    IRQF_TRIGGER_RISING
#define ME906E_RESET 0x01
struct rk29_me906e_data *gpdata = NULL;
struct class *modem_class = NULL; 
static int do_wakeup_irq = 0;
static int modem_status;
int suspend_int =0;

int modem_poweron_off(int on_off);

static void ap_wakeup_bp(struct platform_device *pdev, int wake)
{
	struct rk29_me906e_data *pdata = pdev->dev.platform_data;

	gpio_set_value(pdata->ap_wakeup_bp, wake);  

}
extern void rk28_send_wakeup_key(void);

static void do_wakeup(struct work_struct *work)
{
/*
	if(suspend_int)
	{
		gpio_set_value(gpdata->gps_disable, GPIO_LOW);
		gpio_set_value(gpdata->ap_wakeup_bp, 1);
		 suspend_int = 0;
 	}
 	*/
 	rk28_send_wakeup_key();
}

static DECLARE_DELAYED_WORK(wakeup_work, do_wakeup);
static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
    if(do_wakeup_irq)
    {
        do_wakeup_irq = 0;
  //      MODEMDBG("%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);
        wake_lock_timeout(&modem_wakelock, 10 * HZ);
        schedule_delayed_work(&wakeup_work, 2*HZ);
    }
    return IRQ_HANDLED;
}
int modem_poweron_off(int on_off)
{
	struct rk29_me906e_data *pdata = gpdata;

	printk("%s, on_off=(%d) \n", __FUNCTION__, on_off);

	if(on_off == 1)//on
	{
		gpio_set_value(pdata->modem_power_en, GPIO_HIGH);

		gpio_set_value(pdata->bp_reset, GPIO_HIGH);
		msleep(150);
		gpio_set_value(pdata->bp_reset, GPIO_LOW);
		msleep(50);

		gpio_set_value(pdata->ap_wakeup_bp, GPIO_HIGH);
		gpio_set_value(pdata->gps_disable, GPIO_LOW);//GPS POWER
	}
	else if(on_off == 2)//reset
	{
		gpio_set_value(pdata->bp_reset, GPIO_HIGH);
		msleep(50);
		gpio_set_value(pdata->bp_reset, GPIO_LOW);
		msleep(50);
	}
	else if(on_off == 0)//off
	{
		//gpio_set_value(pdata->bp_reset, GPIO_HIGH);
		//gpio_set_value(pdata->modem_power_en, GPIO_LOW);
		//msleep(600);

		gpio_set_value(pdata->gps_disable, GPIO_HIGH);
		gpio_set_value(pdata->ap_wakeup_bp, 0);
	//	gpio_set_value(pdata->bp_reset, GPIO_HIGH);
	}
	return 0;
}
static int me906e_open(struct inode *inode, struct file *file)
{
	struct rk29_me906e_data *pdata = gpdata;
	device_init_wakeup(pdata->dev, 1);
	return 0;
}

static int me906e_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long me906e_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rk29_me906e_data *pdata = gpdata;

	printk("%s, c(%d), open modem \n", __FUNCTION__, cmd);
	switch(cmd)
	{
		case ME906E_RESET:				
			gpio_set_value(pdata->bp_reset, GPIO_HIGH);
			msleep(50);
			gpio_set_value(pdata->bp_reset, GPIO_LOW);
			msleep(50);
			
			gpio_set_value(pdata->ap_wakeup_bp, GPIO_HIGH);
			gpio_set_value(pdata->gps_disable, GPIO_LOW);//GPS POWER
			break;
		default:
			break;
	}
	return 0;
}

static struct file_operations me906e_fops = {
	.owner = THIS_MODULE,
	.open = me906e_open,
	.release = me906e_release,
	.unlocked_ioctl = me906e_ioctl
};

static struct miscdevice me906e_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MODEM_NAME,
	.fops = &me906e_fops
};
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static ssize_t modem_status_read(struct class *cls, struct class_attribute *attr, char *_buf)
#else
static ssize_t modem_status_read(struct class *cls, char *_buf)
#endif
{
	return sprintf(_buf, "%d\n", modem_status);
	
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static ssize_t modem_status_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
#else
static ssize_t modem_status_write(struct class *cls, const char *_buf, size_t _count)
#endif
{
    int new_state = simple_strtoul(_buf, NULL, 16);
   if(new_state == modem_status) return _count;
   if (new_state == 1){
     printk("%s, c(%d), open modem \n", __FUNCTION__, new_state);
	 modem_poweron_off(1);
   }else if(new_state == 0){
     printk("%s, c(%d), close modem \n", __FUNCTION__, new_state);
	 modem_poweron_off(0);
   }else if(new_state == 2){
     printk("%s, c(%d), reset modem \n", __FUNCTION__, new_state);
	 modem_poweron_off(2);	 
   }else{
     printk("%s, invalid parameter \n", __FUNCTION__);
   }
	modem_status = new_state;
    return _count; 
}
static CLASS_ATTR(modem_status, 0777, modem_status_read, modem_status_write);
static void rk29_early_suspend(struct early_suspend *h)
{
	gpio_set_value(gpdata->gps_disable, GPIO_HIGH);
	gpio_set_value(gpdata->ap_wakeup_bp, 0);

	//gpio_set_value(gpdata->bp_reset, GPIO_HIGH);
}
static void rk29_early_resume(struct early_suspend *h)
{
	//gpio_set_value(gpdata->bp_reset, GPIO_LOW);
	//if(suspend_int)
	{
		gpio_set_value(gpdata->gps_disable, GPIO_LOW);
		gpio_set_value(gpdata->ap_wakeup_bp, 1);
		 suspend_int = 0;
 	}
}

static struct early_suspend me906e_early_suspend = {
	         .suspend = rk29_early_suspend,
	          .resume = rk29_early_resume,
	          .level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	  };
static int me906e_probe(struct platform_device *pdev)
{
	struct rk29_me906e_data *pdata = gpdata = pdev->dev.platform_data;
	struct modem_dev *me906e_data = NULL;
	int result, irq = 0;	

	pdata->dev = &pdev->dev;
	if(pdata->io_init)
		pdata->io_init();

	gpio_set_value(pdata->bp_power, GPIO_HIGH);
	msleep(200);
/*
	gpio_set_value(pdata->bp_reset, GPIO_HIGH);
	msleep(150);
	gpio_set_value(pdata->bp_reset, GPIO_LOW);
	msleep(50);
*/
	modem_poweron_off(1);
	modem_status = 1;
	
	register_early_suspend(&me906e_early_suspend);
	me906e_data = kzalloc(sizeof(struct modem_dev), GFP_KERNEL);
	if(me906e_data == NULL)
	{
		printk("failed to request me906e_data\n");
		goto err2;
	}
	platform_set_drvdata(pdev, me906e_data);		
	result = gpio_request(pdata->ap_wakeup_bp, "me906e");
	if (result) {
		printk("failed to request AP_BP_WAKEUP gpio\n");
		goto err1;
	}	
	irq	= gpio_to_irq(pdata->bp_wakeup_ap);
	enable_irq_wake(irq);
	if(irq < 0)
	{
		gpio_free(pdata->bp_wakeup_ap);
		printk("failed to request bp_wakeup_ap\n");
	}
	result = gpio_request(pdata->bp_wakeup_ap, "bp_wakeup_ap");
	if (result < 0) {
		printk("%s: gpio_request(%d) failed\n", __func__, pdata->bp_wakeup_ap);
	}
	wake_lock_init(&modem_wakelock, WAKE_LOCK_SUSPEND, "bp_wakeup_ap");
	gpio_direction_input(pdata->bp_wakeup_ap);
	gpio_pull_updown(pdata->bp_wakeup_ap, 1);
	result = request_irq(irq, detect_irq_handler, IRQ_BB_WAKEUP_AP_TRIGGER, "bp_wakeup_ap", NULL);
	if (result < 0) {
		printk("%s: request_irq(%d) failed\n", __func__, irq);
		gpio_free(pdata->bp_wakeup_ap);
		goto err0;
	}
	enable_irq_wake(gpio_to_irq(pdata->bp_wakeup_ap)); 

	result = misc_register(&me906e_misc);
	if(result)
	{
		printk("misc_register err\n");
	}
	
	return result;
err0:
	cancel_work_sync(&me906e_data->work);
	gpio_free(pdata->bp_wakeup_ap);
err1:
	gpio_free(pdata->ap_wakeup_bp);
err2:
	kfree(me906e_data);
	return 0;
}

int me906e_suspend(struct platform_device *pdev, pm_message_t state)
{
	suspend_int = 1;
	do_wakeup_irq = 1;
/*
	ap_wakeup_bp(pdev, 0);
	gpio_set_value(gpdata->gps_disable, GPIO_HIGH);
*/
	return 0;
}

int me906e_resume(struct platform_device *pdev)
{
/*
	if(gpio_get_value(gpdata->bp_wakeup_ap))
	{
		schedule_delayed_work(&wakeup_work, 2*HZ);
	}
*/
	return 0;
}

void me906e_shutdown(struct platform_device *pdev)
{
	struct rk29_me906e_data *pdata = pdev->dev.platform_data;
	struct modem_dev *me906e_data = platform_get_drvdata(pdev);

	//modem_poweron_off(0);

	if(pdata->io_deinit)
		pdata->io_deinit();
	cancel_work_sync(&me906e_data->work);
	gpio_free(pdata->modem_power_en);
	gpio_free(pdata->bp_power);
	gpio_free(pdata->bp_reset);
	gpio_free(pdata->ap_wakeup_bp);
	gpio_free(pdata->bp_wakeup_ap);
	kfree(me906e_data);
}

static struct platform_driver me906e_driver = {
	.probe	= me906e_probe,
	.shutdown	= me906e_shutdown,
	.suspend  	= me906e_suspend,
	.resume		= me906e_resume,
	.driver	= {
		.name	= "me906e",
		.owner	= THIS_MODULE,
	},
};

static int __init me906e_init(void)
{
	int ret ;
	modem_class = class_create(THIS_MODULE, "rk291x_modem");
	ret =  class_create_file(modem_class, &class_attr_modem_status);
	if (ret)
	{
		printk("Fail to class rk291x_modem.\n");
	}
	return platform_driver_register(&me906e_driver);
}

static void __exit me906e_exit(void)
{
	platform_driver_unregister(&me906e_driver);
	class_remove_file(modem_class, &class_attr_modem_status);
}

module_init(me906e_init);

module_exit(me906e_exit);
