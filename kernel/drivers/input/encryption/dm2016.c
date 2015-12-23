#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/adc.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <mach/board.h>
#include <linux/platform_device.h>

#define DM2016_SPEED		200 * 1000

struct i2c_client *dm2016_client;
static u8 dm2016_reg;

#define write_sdmc_mode		0x12
#define read_password_mode	0x20

#define encry_reg		0x90

#define EOPEN		5

char read_reg = read_password_mode;

static int dm2016_rx_data(struct i2c_client *client, char *rxData, int length)
{
	int ret = 0;
	char reg = rxData[0];
	ret = i2c_master_reg8_recv(client, reg, rxData, length, DM2016_SPEED);
	return (ret > 0)? 0 : ret;
}

static int dm2016_tx_data(struct i2c_client *client, char *txData, int length)
{
	int ret = 0;
	char reg = txData[0];
	ret = i2c_master_reg8_send(client, reg, &txData[1], length-1, DM2016_SPEED);
	return (ret > 0)? 0 : ret;
}

static char dm2016_read_reg(struct i2c_client *client,int addr)
{
	char tmp;
	int ret = 0;

	tmp = addr;
	ret = dm2016_rx_data(client, &tmp, 1);
	return tmp;
}

static int dm2016_write_reg(struct i2c_client *client,int addr,int value)
{
	char buffer[3];
	int ret = 0;

	buffer[0] = addr;
	buffer[1] = value;
	ret = dm2016_tx_data(client, &buffer[0], 2);
	return ret;
}
#if 0
static ssize_t dm2016_writereg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long val = 0;
	val =simple_strtoll(buf, NULL, 16);

	if (val > 0xff)
		return -EINVAL;

	dm2016_reg = val;
	printk("dm2016_reg = 0x%x\n", dm2016_reg);
	return count;
}

static ssize_t dm2016_write_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long val = 0;
	int ret = 0;

	val =simple_strtoll(buf, NULL, 16);

	if (val > 0xff)
		return -EINVAL;

	ret = dm2016_write_reg(dm2016_client, dm2016_reg, val);
	if(ret != 0)
		printk("dm2016_write_reg error\n");

	return count;
}

static ssize_t dm2016_read_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 val;

	val = dm2016_read_reg(dm2016_client, dm2016_reg);
	printk("dm2016_read_reg val = 0x%x", val);

	return sprintf(buf, "%u\n", val);
}

static DEVICE_ATTR(writereg, S_IRWXUGO, NULL, dm2016_writereg_store);
static DEVICE_ATTR(write, S_IRWXUGO, NULL, dm2016_write_store);
static DEVICE_ATTR(read, S_IRWXUGO, dm2016_read_show, NULL);

static struct attribute *dm2016_attributes[] = {
	&dev_attr_writereg.attr,
	&dev_attr_write.attr,
	&dev_attr_read.attr,
	NULL
};

static struct attribute_group dm2016_attribute_group = {
	.attrs = dm2016_attributes
};
#endif
static int dm2016_open(struct inode *inode, struct file *file)
{
	int err;
	printk("%s\n",__FUNCTION__);
	err = dm2016_rx_data(dm2016_client, &read_reg, 8);
	if(err != 0)
		return -EOPEN;

	return 0;
}

static int dm2016_release(struct inode *inode, struct file *file)
{
	return 0;
}

ssize_t dm2016_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	u8 val;
	int i;

	for(i = 0; i < count; i++){
		val = dm2016_read_reg(dm2016_client, (encry_reg + i));
		*(buf+i) = val;
//		msleep(10);
//		printk("dm2016_read val = 0x%x\n", *(buf+i));
	}

//	for(i = 0; i < count; i++)
//		printk("buf[i] = 0x%x\n", *(buf+i));

	return count;
}

ssize_t dm2016_write (struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int i;

//	for(i = 0; i < count; i++)
//		printk("buf[i] = 0x%x\n", *(buf+i));

	if(write_sdmc_mode == *buf){
		for(i = 1; i < count; i++){
			ret = dm2016_write_reg(dm2016_client, (encry_reg + i -1), *(buf+i));
			if(ret != 0)
				printk("dm2016_write_reg error\n");
//			msleep(10);
		}
	}else if(read_password_mode == *buf){

	}

	return count;
}

static struct file_operations dm2016_fops = {
	.owner = THIS_MODULE,
	.open = dm2016_open,
	.release = dm2016_release,
	.write = dm2016_write,
	.read = dm2016_read,
};
static struct miscdevice dm2016_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "dm2016",
	.fops = &dm2016_fops,
};

static int dm2016_encryp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}

	dm2016_client = client;

	err = dm2016_rx_data(client, &read_reg, 8);
	if(err != 0)
		goto exit;
#if 0
	err = sysfs_create_group(&client->dev.kobj, &dm2016_attribute_group);
	if (err < 0)
		goto error_sysfs;
#endif

	misc_register(&dm2016_device);

	return 0;

error_sysfs:
exit:
	return err;
}

static int dm2016_remove(struct i2c_client *client)
{
//	sysfs_remove_group(&client->dev.kobj, &dm2016_attribute_group);

	misc_deregister(&dm2016_device);

	return 0;
}

static const struct i2c_device_id dm2016_encryp_id[] = {
	{ "dm2016_encry", 0 },
};

static struct i2c_driver dm2016_encryp_driver = {
	.probe = dm2016_encryp_probe,
	.remove = __devexit_p(dm2016_remove),
	.id_table = dm2016_encryp_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = "dm2016_encry",
	}
};

static int __init dm2016_encryp_init(void)
{
	return i2c_add_driver(&dm2016_encryp_driver);
}

static void __exit dm2016_encryp_exit(void)
{
	i2c_del_driver(&dm2016_encryp_driver);
}

module_init(dm2016_encryp_init);
module_exit(dm2016_encryp_exit);

