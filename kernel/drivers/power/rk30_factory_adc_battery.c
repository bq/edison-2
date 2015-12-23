/* drivers/power/rk30_adc_battery.c
 *
 * battery detect driver for the rk30 
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
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <linux/adc.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/wakelock.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#if defined (CONFIG_BATTERY_BQ24196)
#include <linux/bq24196_chargeIc.h>
#endif

#ifdef CONFIG_EARLYSUSPEND
/* kernel/power/earlysuspend.c */
extern suspend_state_t get_suspend_state(void);
#endif

static int rk30_battery_dbg_level = 0;
module_param_named(dbg_level, rk30_battery_dbg_level, int, 0644);
#define DBG( args...) \
	do { \
		if (rk30_battery_dbg_level) { \
			pr_info(args); \
		} \
	} while (0)

#define	TIMER_MS_COUNTS						1000	
#define	SLOPE_SECOND_COUNTS					15	
#define	DISCHARGE_MIN_SECOND				60
#define	CHARGE_MIN_SECOND					45	
#define	CHARGE_MID_SECOND					90	
#define	CHARGE_MAX_SECOND					250
#define CHARGE_FULL_DELAY_TIMES				10     
#define USBCHARGE_IDENTIFY_TIMES				6

#define	NUM_VOLTAGE_SAMPLE					((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	 
#define	NUM_DISCHARGE_MIN_SAMPLE			((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	 
#define	NUM_CHARGE_MIN_SAMPLE				((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MID_SAMPLE				((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	     
#define	NUM_CHARGE_MAX_SAMPLE				((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	  
#define NUM_CHARGE_FULL_DELAY_TIMES			((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_USBCHARGE_IDENTIFY_TIMES		((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	

#define CHARGE_IS_OK						1
#define INVALID_CHARGE_CHECK				-1

#define	BATT_USB_CHARGE_LOW_THRES		3550
#define	BATT_USB_CHARGE_LOW_CHARGER		3600

#if defined(CONFIG_ARCH_RK3066B)||defined(CONFIG_ARCH_RK3188)||defined(CONFIG_SOC_RK3168)||defined(CONFIG_SOC_RK3028)
#define  BAT_DEFINE_VALUE					1800
#elif defined(CONFIG_ARCH_RK2928)
#define  BAT_DEFINE_VALUE					3300
#else
#define  BAT_DEFINE_VALUE					2500
#endif

#define BATT_FILENAME "/data/bat_last_capacity.dat"

#define BATTERY_APK 
#ifdef  BATTERY_APK
#define BATT_NUM  11
int    battery_dbg_level = 0;
int    battery_test_flag = 0;
int    gVoltageCnt = 3500;
int    gDoubleVoltageCnt = 6800;
unsigned long gSecondsCnt = 0;
char gDischargeFlag[4] = {"on "};

#ifdef CONFIG_BATTERY_RK30_VOL3V8
#if defined (CONFIG_BATTERY_BT_C0B5H)
#define BATT_MAX_VOL_VALUE                             4179               	//Full  charge volate
#define BATT_ZERO_VOL_VALUE                            3500              	//power down voltage
#define BATT_NOMAL_VOL_VALUE                         3800
#define BATT_CHARGE_OFFSET				10
#elif defined(CONFIG_BATTERY_BT_D1004_36722128)
#define BATT_MAX_VOL_VALUE                             4194               	//Full  charge volate
#define BATT_ZERO_VOL_VALUE                            3500              	//power down voltage
#define BATT_NOMAL_VOL_VALUE                         3800
#define BATT_CHARGE_OFFSET				10
#else
#define BATT_MAX_VOL_VALUE                             4179               	//Full  charge volate
#define BATT_ZERO_VOL_VALUE                            3500              	//power down voltage
#define BATT_NOMAL_VOL_VALUE                         3800
#define BATT_CHARGE_OFFSET				10
#endif

#if defined (CONFIG_BATTERY_BT_C0B5H)
static int batt_table[2*BATT_NUM+6] =
{
       0x4B434F52, 0x7461625F, 0x79726574, 0, 200, 120,
       3500, 3586, 3626, 3661, 3687, 3711, 3758, 3810, 3865, 3944, 4045,       //discharge
       3550, 3934, 3974, 3999, 4033, 4079, 4138, 4171, 4172, 4176, 4179,       //ac charge
};
#elif defined(CONFIG_BATTERY_BT_D1004_36722128)	//D7803
static int batt_table[2*BATT_NUM+6] =
{
       0x4B434F52, 0x7461625F, 0x79726574, 0, 200, 120,
       3500, 3589, 3640, 3670, 3692, 3708, 3767, 3834, 3878, 3946, 4070,	   //discharge
       3858, 3994, 4065, 4135, 4175, 4185, 4187, 4190, 4192, 4193, 4194,	   //ac charge
};
#else
static int batt_table[2*BATT_NUM+6] =
{
       0x4B434F52, 0x7461625F, 0x79726574, 0, 200, 120,
       3510, 3571, 3583, 3595, 3647, 3676, 3730, 3784, 3847, 3918, 4130,       //discharge
       3550, 3934, 3974, 3999, 4033, 4079, 4138, 4171, 4172, 4176, 4179,       //ac charge
};
#endif

#define adc_to_voltage(adc_val) ((adc_val * BAT_DEFINE_VALUE * (batt_table[4] +batt_table[5])) / (1024 *batt_table[5]))
#else
#define BATT_MAX_VOL_VALUE					8284              	//Full charge voltage
#define BATT_ZERO_VOL_VALUE					6900            	// power down voltage 
#define BATT_NOMAL_VOL_VALUE				7600                

static int batt_table[2*BATT_NUM+6] =
{
	0x4B434F52,0x7461625F,0x79726574,1,300,100,
	6800, 7242, 7332, 7404, 7470, 7520, 7610, 7744, 7848, 8016, 8284,//discharge
	7630, 7754, 7852, 7908, 7956, 8024, 8112, 8220, 8306, 8318, 8328//charge
};
#define adc_to_voltage(adc_val) ((adc_val * BAT_DEFINE_VALUE * (batt_table[4] +batt_table[5])) / (1024 *batt_table[5]))
#endif

#endif

/********************************************************************************/

enum {
	BATTERY_STATUS			= 0,
	BATTERY_HEALTH			= 1,
	BATTERY_PRESENT			= 2,
	BATTERY_CAPACITY		= 3,
	BATTERY_AC_ONLINE		= 4,
	BATTERY_STATUS_CHANGED	= 5,
	AC_STATUS_CHANGED		= 6,
	BATTERY_INT_STATUS		= 7,
	BATTERY_INT_ENABLE		= 8,
};

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

struct rk30_adc_battery_data {
	int irq;
	
	//struct timer_list       timer;
	struct workqueue_struct		*wq;
	struct delayed_work 	    delay_work;
	struct delayed_work			check_work;
	struct work_struct			dcwakeup_work;
	struct work_struct			lowerpower_work;
	struct work_struct			load_capacity_work;
	struct work_struct			put_capacity_work;

	bool					resume;
	
	struct rk30_adc_battery_platform_data *pdata;
	
	struct adc_client       *client; 
	int                     adc_val;
	int                     adc_samples[NUM_VOLTAGE_SAMPLE+2];
	
	int                     bat_status;
	int                     bat_status_cnt;
	int                     bat_health;
	int                     bat_present;
	int                     bat_voltage;
	int                     bat_capacity;
	int                     bat_change;
	
	int                     old_charge_level;
	int						*pSamples;
	int                     gBatCapacityDisChargeCnt;
	int                     gBatCapacityChargeCnt;
	int                     gBatCapacityacChargeCnt;
	int                     gBatCapacityusbChargeCnt ;
	int						gBatCapacityusbdisChargeCnt;
	int						capacitytmp;
	int                     suspend_capacity;
	int                     gBatUsbChargeCnt;
	int                     status_lock;

	struct power_supply		bat;
	struct power_supply		usb;
	struct power_supply		ac;
	struct power_supply		bk_bat;
	
	int                     poweron_check;
	int                     usb_charging;
	int                     ac_charging;
	int						charge_check;
	int						charge_level;
	
	int                     charge_source_now;
	int                     charge_soure_old;
	int                     charge_start_capacity;
	int                     charge_start_voltage;
	int                     start_voltage_status;
	int                     charge_up_proprotion;
	int                     charge_down_proportion;
	int                     voltage_to_local;
	unsigned long			suspend_time;
	unsigned long			resume_time;
	int						adc_value;
	
	int                     full_times;
	int						charge_full_flag;
	int                     stop_check;

	struct notifier_block	battery_nb;

	int						lower_power_flag;
	int						time_to_full;
};
static struct rk30_adc_battery_data *gBatteryData;
static struct wake_lock batt_wake_lock;
static struct wake_lock batt_wake_lock_detect_lowpower;
static struct wake_lock charge_display_lock;
static struct wake_lock dc_wakelock;
static int charge_display_lock_status = 0;
static int usb_identify_flag = 0;

int system_lowerpower = 0;
bool is_ac_charge = false;
bool is_usb_charge = false;
volatile bool low_usb_charge = false;
#if defined(CONFIG_REBOOT_TO_POWER_OFF)
int charge_status_now;
#endif
#if defined(CONFIG_BATTERY_BQ27410)
extern int bq27410_init;
#endif
#if defined(CONFIG_BATTERY_BQ27541)
extern int bq27541_init;
#endif
#if defined(CONFIG_BATTERY_BQ27425)
extern int bq27425_init;
#endif
static int put_capacity_num = 0;

#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
extern int bq24196_mode;
#endif
static int charging_flag;

#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
extern int charge_en_flags;
extern int check_charge_ok;
#endif
extern int dwc_vbus_status(void);
extern int get_gadget_connect_flag(void);
extern int dwc_otg_check_dpdm(void);
static int  is_charge_ok(struct rk30_adc_battery_data *bat);
static void rk30_adc_battery_voltage_samples(struct rk30_adc_battery_data *bat);
static int rk30_adc_battery_voltage_to_capacity(struct rk30_adc_battery_data *bat, int BatVoltage);

bool is_accharging(void)
{
	return is_ac_charge;
}
bool is_usbcharging(void)
{
	return is_usb_charge;
}
bool low_usb_charging(void)
{
	return low_usb_charge;
}

#ifdef  BATTERY_APK
//#define BAT_ADC_TABLE_LEN               11
static ssize_t bat_param_read(struct device *dev,struct device_attribute *attr, char *buf)
{
	int i;
	for(i=0;i<BATT_NUM;i++)
		printk("i=%d batt_table=%d\n",i+6,batt_table[i+6]);

	for(i=0;i<BATT_NUM;i++)
		printk("i=%d batt_table=%d\n",i+17,batt_table[i+17]);
	return 0;
}
DEVICE_ATTR(batparam, 0664, bat_param_read,NULL);


static ssize_t rkbatt_show_debug_attrs(struct device *dev,
					      				      struct device_attribute *attr, char *buf) 
{				 
	return sprintf(buf, "%d\n", battery_dbg_level);
}

static ssize_t rkbatt_restore_debug_attrs(struct device *dev, 
										  struct device_attribute *attr, const char *buf, size_t size)
{
	int liTmp;
	
	sscanf(buf, "%d", &liTmp);
	
	if(liTmp != 0 && liTmp != 1)
	{
		dev_err(dev, "rk29adc_restore_debug_attrs err\n");
	}
	else
	{
		battery_dbg_level = liTmp;
	}
	return size;
}

static ssize_t rkbatt_show_state_attrs(struct device *dev,
					      					struct device_attribute *attr, char *buf) 
{
//	struct rk30_adc_battery_platform_data *pdata = gBatteryData->pdata;
	int charge_ok_value =0 ;
	charge_ok_value = is_charge_ok(gBatteryData) ;

	return 	sprintf(buf,
		"gBatVol=%d,gBatCap=%d,charge_ok=%d,%s\n",
		gBatteryData->bat_voltage,gBatteryData->bat_capacity,
		charge_ok_value,gDischargeFlag);
}

static ssize_t rkbatt_restore_state_attrs(struct device *dev, 
										struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static ssize_t rkbatt_show_value_attrs(struct device *dev,
					     				    struct device_attribute *attr, char *buf) 
{				 
	return sprintf(buf, "pull_up_res =%d,\npull_down_res=%d\n", batt_table[4],batt_table[5]);
}

static ssize_t rkbatt_restore_value_attrs(struct device *dev, 
										struct device_attribute *attr, const char *buf, size_t size)
{
	int liUp	= 0;
	int liDown	= 0;
	
	sscanf(buf, "%d,%d", &liUp,&liDown);
	
	if(liUp != 0 && liDown != 0)
	{
		batt_table[4] = liUp;
		batt_table[5] = liDown;
	}
	return size;
}

static ssize_t rkbatt_show_flag_attrs(struct device *dev,
					     				 struct device_attribute *attr, char *buf) 
{				 
	return sprintf(buf, "rk29_battery_test_flag=%d\n", battery_test_flag);
}

static ssize_t rkbatt_restore_flag_attrs(struct device *dev, 
									    struct device_attribute *attr, const char *buf, size_t size)
{
	int liFlag;
	
	sscanf(buf, "%d", &liFlag);
	
	if(liFlag != 0)
	{
		battery_test_flag = liFlag;
	}
	return size;
}

#define ADC_PROC_FILENAME   "/proc/mtd"
#define ADC_CABLIC_FILE "gsensor"

typedef struct {
    int device_index;
    unsigned int size;
    unsigned int erase_size;
    char *name;
} AdcPartition;

typedef struct {
    AdcPartition *partitions;
    int partitions_allocd;
    int partition_count;
} AdcState;

static AdcState adc_mtd_state = {
    NULL,   // partitions
    0,      // partitions_allocd
    -1      // partition_count
};

struct adc_cal_data {
	int valid;
	int cap;
	char reser[512];
};

static int adc_mtd_index = -1;
static int load_cap_value = -1;
struct adc_cal_data adc_cal[1];

int adc_scan_partitions(void)
{
    char buf[2048];
    const char *bufp;
    int fd;
    int i;
    ssize_t nbytes;

    if (adc_mtd_state.partitions == NULL) {
        const int nump = 32;
        AdcPartition *partitions = kmalloc(nump * sizeof(*partitions), GFP_KERNEL);
        if (partitions == NULL) {
            return -1;
        }
        adc_mtd_state.partitions = partitions;
        adc_mtd_state.partitions_allocd = nump;
        memset(partitions, 0, nump * sizeof(*partitions));
    }
    adc_mtd_state.partition_count = 0;

    /* Initialize all of the entries to make things easier later.
     * (Lets us handle sparsely-numbered partitions, which
     * may not even be possible.)
     */
    for (i = 0; i < adc_mtd_state.partitions_allocd; i++) {
        AdcPartition *p = &adc_mtd_state.partitions[i];
        if (p->name != NULL) {
            kfree(p->name);
            p->name = NULL;
        }
        p->device_index = -1;
    }

    /* Open and read the file contents.
     */
    fd = sys_open(ADC_PROC_FILENAME, O_RDONLY, 0);
    if (fd < 0) {
        goto bail;
    }
    nbytes = sys_read(fd, buf, sizeof(buf) - 1);
    sys_close(fd);
    if (nbytes < 0) {
        goto bail;
    }
    buf[nbytes] = '\0';

    /* Parse the contents of the file, which looks like:
     *
     *     # cat /proc/mtd
     *     dev:    size   erasesize  name
     *     mtd0: 00080000 00020000 "bootloader"
     *     mtd1: 00400000 00020000 "mfg_and_gsm"
     *     mtd2: 00400000 00020000 "0000000c"
     *     mtd3: 00200000 00020000 "0000000d"
     *     mtd4: 04000000 00020000 "system"
     *     mtd5: 03280000 00020000 "userdata"
     */
    bufp = buf;
    while (nbytes > 0) {
        int mtdnum, mtdsize, mtderasesize;
        int matches;
        char mtdname[64];
        mtdname[0] = '\0';
        mtdnum = -1;

        matches = sscanf(bufp, "mtd%d: %x %x \"%63s\"",
                &mtdnum, &mtdsize, &mtderasesize, mtdname);
        if (mtdname[strlen(mtdname) - 1] == '"') {
			mtdname[strlen(mtdname) - 1] = '\0';
        }
        /* This will fail on the first line, which just contains
         * column headers.
         */
        if (matches == 4) {
            AdcPartition *p = &adc_mtd_state.partitions[mtdnum];
            p->device_index = mtdnum;
            p->size = mtdsize;
            p->erase_size = mtderasesize;
            p->name = kmalloc(sizeof(mtdname), GFP_KERNEL);
            if (p->name == NULL) {
                goto bail;
            }
            strcpy(p->name, mtdname);
            adc_mtd_state.partition_count++;
        }

        /* Eat the line.
         */
        while (nbytes > 0 && *bufp != '\n') {
            bufp++;
            nbytes--;
        }
        if (nbytes > 0) {
            bufp++;
            nbytes--;
        }
    }

    return adc_mtd_state.partition_count;

bail:
    // keep "partitions" around so we can free the names on a rescan.
    printk("fail!!!");
    adc_mtd_state.partition_count = -1;
    return -1;
}

static int adc_find_partition_by_name(const char *name)
{
	int i;

	if (adc_mtd_state.partitions == NULL)
		adc_scan_partitions();

    if (adc_mtd_state.partitions != NULL) {
        for (i = 0; i < adc_mtd_state.partitions_allocd; i++) {
            AdcPartition *p = &adc_mtd_state.partitions[i];
            if (p->device_index >= 0 && p->name != NULL) {
                if (strcmp(p->name, name) == 0) {
                    return p->device_index;
                }
            }
        }
    }
    return -1;
}

static int rk30_adc_battery_load_capacity(void)
{
	char mtddevname[64];
	char value[4];
	int* p = (int *)value;

	DBG("%s:adc_mtd_index = %d\n", __func__, adc_mtd_index);

	if (adc_mtd_index >= 0) {
	    sprintf(mtddevname, "/dev/mtd/mtd%d", adc_mtd_index);
		long fd = sys_open(mtddevname,O_RDONLY,0);

		if(fd < 0){
			printk("rk30_adc_battery_load_capacity: open file /dev/mtd/mtd%d failed\n", adc_mtd_index);
			return -1;
		}

		adc_cal[0].valid = 0;
		adc_cal[0].cap = 0;
		sys_lseek(fd, 10 * BLOCK_SIZE, SEEK_SET);
		sys_read(fd,(char __user *)adc_cal,sizeof(adc_cal));
		sys_close(fd);

		if(adc_cal[0].valid == 12345678){
			return adc_cal[0].cap;
		}else{
			printk("adc_cal error\n");
			return -1;
		}
	} else {
		long fd = sys_open(BATT_FILENAME,O_RDONLY,0);

		if(fd < 0){
			printk("rk30_adc_battery_load_capacity: open file /data/bat_last_capacity.dat failed\n");
			return -1;
		}

		sys_read(fd,(char __user *)value,4);
		sys_close(fd);
	}

	return (*p);
}

static void rk30_adc_battery_put_capacity(int loadcapacity)
{
	char mtddevname[64];
	char value[4];
	int* p = (int *)value;
	long fd;

	DBG("%s:adc_mtd_index = %d, loadcapacity = %d\n", __func__, adc_mtd_index, loadcapacity);

	if (adc_mtd_index >= 0) {
	    sprintf(mtddevname, "/dev/mtd/mtd%d", adc_mtd_index);
		fd = sys_open(mtddevname,O_CREAT | O_RDWR | O_TRUNC,0);
		if(fd < 0){
			printk("rk30_adc_battery_put_capacity: open file /dev/mtd/mtd%d failed\n", adc_mtd_index);
		}else{
			adc_cal[0].cap = loadcapacity;
			adc_cal[0].valid = 12345678;

			sys_lseek(fd, 10 * BLOCK_SIZE, SEEK_SET);
			sys_write(fd, (char __user *)adc_cal,sizeof(adc_cal));
			sys_close(fd);
		}
	}

	fd = sys_open(BATT_FILENAME,O_CREAT | O_RDWR,0);
	if(fd < 0){
		printk("rk30_adc_battery_put_capacity: open file /data/bat_last_capacity.dat failed\n");
		return;
	}else{
		*p = loadcapacity;
		sys_write(fd, (const char __user *)value, 4);
		sys_close(fd);
	}
}

static void rk30_adc_battery_load_capacity_delaywork(struct work_struct *work)
{
	load_cap_value = rk30_adc_battery_load_capacity();
}

static void rk30_adc_battery_put_capacity_delaywork(struct work_struct *work)
{
	rk30_adc_battery_put_capacity(gBatteryData->bat_capacity);
}

static ssize_t rkbatt_show_cal_attrs(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	queue_work(gBatteryData->wq, &gBatteryData->load_capacity_work);
	msleep(100);
	return sprintf(buf, "%d\n", load_cap_value);
}
static ssize_t rkbatt_restore_cal_attrs(struct device *dev,
							struct device_attribute *attr, const char *buf, size_t size)
{
	int i;

	if((strncmp(buf, "adc_cal", strlen("adc_cal")) == 0))
	{
		adc_sync_read(gBatteryData->client);

		for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++){
			mdelay(1);
			rk30_adc_battery_voltage_samples(gBatteryData);
		}

		DBG("%s..bat->bat_voltage=%d\n",__func__,gBatteryData->bat_voltage);
		gBatteryData->bat_capacity = rk30_adc_battery_voltage_to_capacity(gBatteryData, gBatteryData->bat_voltage);

		queue_work(gBatteryData->wq, &gBatteryData->put_capacity_work);

#if defined(CONFIG_BATTERY_BQ27410)
		if(!bq27410_init)
#elif defined(CONFIG_BATTERY_BQ27541)
		if(!bq27541_init)
#elif defined(CONFIG_BATTERY_BQ27425)
		if(!bq27425_init)
#endif
		power_supply_changed(&gBatteryData->bat);
	}

	return size;
}

static ssize_t rkbatt_show_boot_mode(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", saved_command_line);
}
static ssize_t rkbatt_restore_boot_mode(struct device *dev,
							struct device_attribute *attr, const char *buf, size_t size)
{
	if((strncmp(buf, "reboot", strlen("reboot")) == 0))
		saved_command_line = original_command_line;

	printk("charge reboot change mode\n");
	return size;
}

static struct device_attribute rkbatt_attrs[] = {
	__ATTR(state, 0664, rkbatt_show_state_attrs, rkbatt_restore_state_attrs),
	__ATTR(debug, 0664, rkbatt_show_debug_attrs, rkbatt_restore_debug_attrs),
	__ATTR(value, 0555, rkbatt_show_value_attrs, rkbatt_restore_value_attrs),
	__ATTR(flag,  0555, rkbatt_show_flag_attrs,  rkbatt_restore_flag_attrs),
	__ATTR(cal,  0664, rkbatt_show_cal_attrs,  rkbatt_restore_cal_attrs),
	__ATTR(boot_mode,  0664, rkbatt_show_boot_mode,  rkbatt_restore_boot_mode),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int liTmep;
	for (liTmep = 0; liTmep < ARRAY_SIZE(rkbatt_attrs); liTmep++){
		if (device_create_file(dev, rkbatt_attrs + liTmep)){
			goto error;
		}
	}

	return 0;

error:
	for ( ; liTmep >= 0; liTmep--){
		device_remove_file(dev, rkbatt_attrs + liTmep);
	}
	
	dev_err(dev, "%s:Unable to create sysfs interface\n", __func__);
	return -1;
}

#endif

static BLOCKING_NOTIFIER_HEAD(adc_battery_chain_head);

int register_adc_battery_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&adc_battery_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_adc_battery_notifier);

int unregister_adc_battery_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&adc_battery_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_adc_battery_notifier);

int adc_battery_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&adc_battery_chain_head, val, NULL)
			== NOTIFY_BAD) ? -EINVAL : 0;
}
EXPORT_SYMBOL_GPL(adc_battery_notifier_call_chain);

static void rk_start_charge(struct rk30_adc_battery_data *bat)
{
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	if (pdata->charge_set_pin != INVALID_GPIO){
		gpio_direction_output(pdata->charge_set_pin, pdata->charge_set_level);
	}
}

static void rk_stop_charge(struct rk30_adc_battery_data *bat)
{
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	if (pdata->charge_set_pin != INVALID_GPIO){
		gpio_direction_output(pdata->charge_set_pin, 1 - pdata->charge_set_level);
	}
}

static int  get_ac_status(struct rk30_adc_battery_data *bat){
	
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
	int status = 0;
	if (pdata->dc_det_pin != INVALID_GPIO){
       	if (gpio_get_value (pdata->dc_det_pin) == pdata->dc_det_level){
			status = 1;
		}else{
			status = 0;
		}
	}else{
		if(pdata->is_dc_charging){
			status = pdata->is_dc_charging();
		}
	}
	DBG("%s: status = %d", __func__, status);
	return status;
}

#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)
// state of charge  --- charge-display
static int  get_usb_status1(struct rk30_adc_battery_data *bat){
//	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
	int status = 0;

	if( gpio_get_value(bat->pdata->usb_det_pin)== bat->pdata->usb_det_level){
		status = 1;
	}else{
		status = 0;
	}
	DBG("%s: status = %d", __func__, status);
	return status;
}

//state of charge ----running
static int  get_usb_status2(struct rk30_adc_battery_data *bat){

//	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
	int usb_status = 0; // 0--dischage ,1 ---usb charge, 2 ---ac charge
	int vbus_status =  dwc_vbus_status();
	
	if (1 == vbus_status) {
		if (0 == get_gadget_connect_flag()){ 
			if (++bat->gBatUsbChargeCnt >= NUM_USBCHARGE_IDENTIFY_TIMES){
				bat->gBatUsbChargeCnt = NUM_USBCHARGE_IDENTIFY_TIMES + 1;
				usb_status = 1; // non-standard AC charger(500mA)
				if(usb_identify_flag != 0){
					usb_identify_flag = 0;
					bat->bat_change = 1;
				}
			}else{
				usb_status = 1;	// connect to pc
				usb_identify_flag = 1;
			}
		}else{
			usb_status = 1;	// connect to pc
			if(usb_identify_flag != 0){
				usb_identify_flag = 0;
				bat->bat_change = 1;
			}
		}
	}else{
		bat->gBatUsbChargeCnt = 0;
		if (2 == vbus_status) {
			usb_status = 2; //standard AC charger
		}else{
			usb_status = 0; 
		}
	}
	DBG("%s: usb_status = %d", __func__, usb_status);
	return usb_status;
}
#endif
static int rk_battery_get_status(struct rk30_adc_battery_data *bat)
{
	int charge_on = 0;
	int ac_ac_charging = 0, usb_ac_charging = 0, status = 0;
	int i=0;

#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
	if(bq24196_mode == 1)
	{
		bat->charge_level = 0;
		bat->ac_charging = 0;
		bat->usb_charging = 0;
		return 0;
	}
#endif

#if defined (CONFIG_BATTERY_RK30_AC_CHARGE)
	ac_ac_charging = get_ac_status(bat);
	if(1 == ac_ac_charging)
		charge_on = 1;
#endif

	if (!strstr(saved_command_line,"charger"))
	{
		if(1 == charge_display_lock_status)
		{
			wake_unlock(&charge_display_lock);  //unlock
			charge_display_lock_status = 0;
		}
	}
#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)	
	if (strstr(saved_command_line,"charger")){
#if defined(CONFIG_BATTERY_BQ27410)
		if(bq27410_init && is_usbcharging())
#elif defined(CONFIG_BATTERY_BQ27541)
		if(bq27541_init && is_usbcharging())
#elif defined(CONFIG_BATTERY_BQ27425)
		if(bq27425_init && is_usbcharging())
#endif
		{
			if(!charge_display_lock_status)
			{
				wake_lock(&charge_display_lock);  //lock
				charge_display_lock_status = 1;
			}
		}

#if 0
		if( bat->pdata->usb_det_pin  != INVALID_GPIO ){
			if( gpio_get_value(bat->pdata->usb_det_pin)== bat->pdata->usb_det_level){
				if(( 1 == usb_ac_charging )||( 1 == ac_ac_charging ))
					bat->ac_charging = 1;
#if  !defined (CONFIG_BATTERY_RK30_USB_AND_CHARGE)
				if(( 1 == bat->usb_charging)||(1 == bat->ac_charging))
					charge_on =1;
#endif
				DBG("1.ac_status=%d,usb_status=%d bat->bat_change = %d\n",bat->ac_charging, bat->usb_charging ,bat->bat_change );
				//return charge_on;
			}else{
				if(( 0 == usb_ac_charging )&&( 0 == ac_ac_charging ))
					bat->ac_charging = 0;
				else
					bat->ac_charging = 1;
				
				bat->usb_charging = 0;
				if(1 == bat->ac_charging)
					charge_on=1;
				else
					charge_on = 0;
				DBG("2.ac_status=%d,usb_status=%d bat->bat_change = %d\n",bat->ac_charging, bat->usb_charging ,bat->bat_change );
				return charge_on;
			}
		}else
#endif
		{
			status = dwc_otg_check_dpdm();
			if(status == 0){
				bat->usb_charging = 0;
				usb_ac_charging = 0;
			}else if(status == 1){
				if(0 == ac_ac_charging){
					bat->usb_charging = 1;
					if(bat->pdata->control_usb_charging)
						bat->pdata->control_usb_charging(0);
				}else
					bat->usb_charging = 0;
			}else if(status == 2){
				bat->usb_charging = 0;
				usb_ac_charging = 1;
			}
			if((1 == usb_ac_charging)||(1 == ac_ac_charging)){
				bat->ac_charging = 1;
				if(bat->pdata->control_usb_charging)
					bat->pdata->control_usb_charging(1);
			}else
				bat->ac_charging = 0;

			if((1 == bat->ac_charging)||(1 == bat->usb_charging)){
				charge_on = 1;
				bat->bat_change = 1;
			}

			if((0 == bat->ac_charging)&&(0 == bat->usb_charging)){
				charge_on = 0;
				bat->bat_change = 1;
			}
			DBG("usb_ac_charging =%d, ac_ac_charging = %d, dwc_otg_check_dpdm = %d\n", usb_ac_charging, ac_ac_charging, ac_ac_charging, status);
			DBG("3.ac_status=%d,usb_status=%d bat->bat_change = %d\n",bat->ac_charging, bat->usb_charging ,bat->bat_change );

			return charge_on;
		}
	}

	if (charge_on == 0){
#if  defined (CONFIG_BATTERY_RK30_USB_AND_CHARGE)
		usb_ac_charging = get_usb_status2(bat); //0 --discharge, 1---usb charging,2----AC charging;
#else
		usb_ac_charging = get_usb_status1(bat); //0 --discharge, 1---usb charging
#endif

		if(1 == usb_ac_charging)
			bat->usb_charging = 1;
		else
			bat->usb_charging = 0;						
	}
#endif
	if((usb_ac_charging == 2)||(ac_ac_charging == 1))
		bat->ac_charging = 1;
	else
		bat->ac_charging = 0;

	if((bat->usb_charging == 1)||(bat->ac_charging ==1))
		charge_on =1;
	
	if(1 == bat->ac_charging ){
		bat->charge_source_now = 1; //ac charge
	}else if( 1 == bat->usb_charging){
		bat->charge_source_now = 2; //usb charge
	}else{
		bat->charge_soure_old = 0;
		bat->charge_source_now = 0;
	}

	if(bat->pdata->control_usb_charging)
	{
		if(bat->charge_source_now == 1)
			bat->pdata->control_usb_charging(1);
		else if(bat->charge_source_now == 2)
			bat->pdata->control_usb_charging(0);
	}

	if(bat->charge_source_now != bat->charge_soure_old){
		for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++){                //0.3 s
			msleep(1); //mdelay --- > msleep
			rk30_adc_battery_voltage_samples(bat);              //get new voltage
		}

		bat->charge_soure_old = bat->charge_source_now;
		bat->bat_change = 1;
	}

#if defined(CONFIG_REBOOT_TO_POWER_OFF)
	charge_status_now= bat->charge_source_now;
#endif
	DBG("ac_status=%d,usb_status=%d bat->bat_change = %d\n",bat->ac_charging, bat->usb_charging ,bat->bat_change );
	
	return charge_on;
}

static int  is_charge_ok(struct rk30_adc_battery_data *bat)
{
	int charge_is_ok = 0;
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	if( 1 != bat->charge_level)
		return -1;
	
	if((pdata->charge_ok_pin == INVALID_GPIO)&& ( pdata->charging_ok == NULL))
		return -1;
	
	if (pdata->charge_ok_pin != INVALID_GPIO){		
		if (gpio_get_value(pdata->charge_ok_pin) == pdata->charge_ok_level){
			charge_is_ok =1;
		}
	}else if( pdata->charging_ok){	
		charge_is_ok = pdata->charging_ok();
	}

#if defined (CONFIG_BATTERY_BQ24196)
	else if(get_charge_status() == POWER_SUPPLY_STATUS_FULL)
		charge_is_ok = 1;
	else
		charge_is_ok = 0;		
#endif

	return charge_is_ok;
}

static int rk30_adc_battery_status_samples(struct rk30_adc_battery_data *bat)
{
	int charge_level;

	charge_level = bat->charge_level;//rk_battery_get_status(bat);

	if (charge_level != bat->old_charge_level){
		bat->old_charge_level = charge_level;
		bat->bat_change  = 1;
#if 0
		if(charge_level) {            
			rk_start_charge(bat);
		}
		else{
			rk_stop_charge(bat);
		}
#endif
		bat->bat_status_cnt = 0;        
	}
	if(( 1 == charge_level )&&(1 == bat->charge_full_flag) && (bat->bat_capacity < 90)){
		rk_start_charge(bat);  //recharge
		if(bat->pdata->ctrl_charge_led != NULL)
			bat->pdata->ctrl_charge_led(0);
	}else if (charge_level) {
		rk_start_charge(bat);
	}else{
		rk_stop_charge(bat);
	}

	if(charge_level == 0){   
	//discharge
		bat->charge_full_flag = 0;
		bat->full_times = 0;
		bat->time_to_full = 0;
		bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}else{
	//charging 	    
		if( is_charge_ok(bat)  ==  INVALID_CHARGE_CHECK){
			if(bat->bat_capacity == 100){
				if(bat->bat_status != POWER_SUPPLY_STATUS_FULL){
					bat->bat_status = POWER_SUPPLY_STATUS_FULL;
					bat->bat_change  = 1;
					bat->charge_full_flag = 1;
					if(bat->pdata->ctrl_charge_led != NULL)
						bat->pdata->ctrl_charge_led(1);
					rk_stop_charge(bat);
				}
			}else{
				bat->charge_full_flag = 0;
				bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
				if((bat->bat_capacity == 99)&&(bat->time_to_full == 0)){
					bat->time_to_full = get_seconds();
				}
				else if(bat->bat_capacity == 99)
				{
					if(get_seconds() - bat->time_to_full > 3600)
						bat->bat_capacity = 100;
						bat->time_to_full = 0;
				}

				if(bat->pdata->ctrl_charge_led != NULL)
					bat->pdata->ctrl_charge_led(0);
			}
		}else{  // pin of charge_ok_pin
			if (is_charge_ok(bat) != CHARGE_IS_OK ){
				bat->full_times = 0;
				bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;

				if((bat->bat_capacity == 99)&&(bat->time_to_full == 0)){
					bat->time_to_full = get_seconds();
					DBG("%s.....battery capcity 99%% start one hour timer\n",__func__);
				}else if(bat->bat_capacity == 99){
					DBG("%s.....battery capcity 99%% timer is %d\n",__func__, get_seconds() - bat->time_to_full);
					if(get_seconds() - bat->time_to_full > 3600){
						bat->bat_capacity = 100;
						bat->time_to_full = 0;
						bat->bat_status = POWER_SUPPLY_STATUS_FULL;
						bat->charge_full_flag = 1;
						bat->bat_change  = 1;
						DBG("%s.....battery capcity 99%% one hour timer is over\n",__func__);
					}
				}
			}else{
				bat->full_times++;

				if (bat->full_times >= NUM_CHARGE_FULL_DELAY_TIMES) {
					bat->full_times = NUM_CHARGE_FULL_DELAY_TIMES + 1;
				}

				if ((bat->full_times >= NUM_CHARGE_FULL_DELAY_TIMES) && (bat->bat_capacity >= 99)){
					if (bat->bat_status != POWER_SUPPLY_STATUS_FULL){
						bat->bat_status = POWER_SUPPLY_STATUS_FULL;
						rk_stop_charge(bat);
						bat->bat_capacity = 100;
						bat->charge_full_flag = 1;
						bat->bat_change  = 1;
						DBG("%s.....battery change to full status with bat->bat_capacity =%d",__func__,bat->bat_capacity);
					}
				}
				else{
					bat->charge_full_flag = 0;
					bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
				}
			}
		}
	}

	return charge_level;
}
static int rk_adc_voltage(struct rk30_adc_battery_data *bat, int value)
{
	int voltage;
	int ref_voltage; //reference_voltage
	int pullup_res;
	int pulldown_res;

	ref_voltage = bat->pdata->reference_voltage;
	pullup_res = bat->pdata->pull_up_res;
	pulldown_res = bat->pdata->pull_down_res;

	if(ref_voltage && pullup_res && pulldown_res){
//#if defined(CONFIG_ARCH_RK2928) || defined(CONFIG_ARCH_RK3026)
		//ref_voltage = adc_get_curr_ref_volt();
//#endif	
		voltage =  ((value * ref_voltage * (pullup_res + pulldown_res)) / (1024 * pulldown_res));
		DBG("VALUE = %d,ref_voltage =%d, voltage=%d \n", value,ref_voltage,voltage);
		
	}else{
#if 0
		if(bat->capacitytmp < 5)
			ref_voltage = adc_get_curr_ref_volt();
		else
			ref_voltage = adc_get_def_ref_volt();
#endif
#if defined(CONFIG_ARCH_RK2928) || defined(CONFIG_ARCH_RK3026)
		ref_voltage = adc_get_curr_ref_volt();
		voltage = (value * ref_voltage * (batt_table[4] + batt_table[5])) / (1024 * batt_table[5]); 
#else
		voltage = adc_to_voltage(value);
#endif
	}
	
	DBG("ref_voltage =%d, voltage=%d \n", ref_voltage,voltage);

	return voltage;
}
static void rk_handle_ripple(struct rk30_adc_battery_data *bat, int status)
{
	int *p_table;

	if (bat->pdata->use_board_table){
		p_table = bat->pdata->board_batt_table;	

		if(1 == status){
			if(bat->bat_voltage >= p_table[2*BATT_NUM +5]+ 10)
				bat->bat_voltage = p_table[2*BATT_NUM +5]  + 10;
			else if(bat->bat_voltage <= p_table[BATT_NUM +6]  - 10)
				bat->bat_voltage =  p_table[BATT_NUM +6] - 10;
		}
		else{
			if(bat->bat_voltage >= p_table[BATT_NUM +5]+ 10)
				bat->bat_voltage = p_table[BATT_NUM +5]  + 10;
			else if(bat->bat_voltage <= p_table[6]  - 10)
				bat->bat_voltage =  p_table[6] - 10;
		}
	}

}

//static int *pSamples;
static void rk30_adc_battery_voltage_samples(struct rk30_adc_battery_data *bat)
{
	int value;
	int i,*pStart = bat->adc_samples, num = 0;
	int level = bat->charge_level;

	value = bat->adc_val;
	adc_async_read(bat->client);

	*(bat->pSamples++) = rk_adc_voltage(bat,value);

	bat->bat_status_cnt++;
	if (bat->bat_status_cnt > NUM_VOLTAGE_SAMPLE)
		bat->bat_status_cnt = NUM_VOLTAGE_SAMPLE + 1;

	num = bat->pSamples - pStart;
	
	if (num >= NUM_VOLTAGE_SAMPLE){
		bat->pSamples = pStart;
		num = NUM_VOLTAGE_SAMPLE;
	}

	value = 0;
	for (i = 0; i < num; i++){
		value += bat->adc_samples[i];
	}
	bat->bat_voltage = value / num;

	/*handle  ripple */
	if(battery_test_flag == 0)
	{
		if(0 == bat->pdata->use_board_table){
			if(1 == level){
				if(bat->bat_voltage >= batt_table[2*BATT_NUM +5]+ 10)
					bat->bat_voltage = batt_table[2*BATT_NUM +5]  + 10;
				else if(bat->bat_voltage <= batt_table[BATT_NUM +6]  - 10)
					bat->bat_voltage =  batt_table[BATT_NUM +6] - 10;
			}
			else{
				if(bat->bat_voltage >= batt_table[BATT_NUM +5]+ 10)
					bat->bat_voltage = batt_table[BATT_NUM +5]  + 10;
				else if(bat->bat_voltage <= batt_table[6]  - 10)
					bat->bat_voltage =  batt_table[6] - 10;
			}
		}else{
			rk_handle_ripple(bat, level);
		}

	}else if(battery_test_flag == 2){
		if(batt_table[3] == 0){
			if(bat->bat_voltage < BATT_ZERO_VOL_VALUE){
				if((get_seconds() - gSecondsCnt) > 30){
					gSecondsCnt = get_seconds();
					if((gVoltageCnt - bat->bat_voltage) > 15){
						strncpy(gDischargeFlag, "off" ,4);	
					}
					gVoltageCnt = bat->bat_voltage;
				}
			}
			
			if(bat->bat_voltage < 3400){
				bat->bat_voltage = 3400;
			}
		}
		else{
			if(bat->bat_voltage < BATT_ZERO_VOL_VALUE){
				if((get_seconds() - gSecondsCnt) > 30){
					gSecondsCnt = get_seconds();
					if((gDoubleVoltageCnt - bat->bat_voltage) > 30){
						strncpy(gDischargeFlag, "off" ,4);	
					}
					gDoubleVoltageCnt =bat->bat_voltage;
				}
			}
			if(bat->bat_voltage < 6800){
				bat->bat_voltage = 6800;
			}	
		}
	}
#ifdef CONFIG_NO_BATTERY
		bat->bat_voltage = 4200;
#endif
}

static int rk30_adc_battery_voltage_to_capacity(struct rk30_adc_battery_data *bat, int BatVoltage)
{
	int i = 0;
	int capacity = 0;
	int  *p;
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	if (bat->pdata->use_board_table)
		p = bat->pdata->board_batt_table;	
	else 
		p = batt_table;

	if (1 == bat->charge_level){  //charge
		if(1 == gpio_get_value(pdata->back_light_pin))
			BatVoltage = BatVoltage + BATT_CHARGE_OFFSET;

		if(0 == bat->start_voltage_status ){
			if(BatVoltage >= (p[2*BATT_NUM +5])){
				capacity = 100;
			}	
			else{
				if(BatVoltage <= (p[BATT_NUM +6])){
					capacity = 0;
				}
				else{
					for(i = BATT_NUM +6; i <2*BATT_NUM +5; i++){
						if(((p[i]) <= BatVoltage) && (BatVoltage < (p[i+1]))){
							capacity = (i-(BATT_NUM +6))*10 + ((BatVoltage - p[i]) * 10)/(p[i+1]- p[i]);
							break;
						}
					}
				}  
			}
		}
		else{
			DBG("start_voltage=%d,start_capacity =%d\n", bat->charge_start_voltage, bat->charge_start_capacity);
			DBG("charge_down_proportion =%d,charge_up_proprotion=%d\n",bat->charge_down_proportion,bat->charge_up_proprotion);
			for(i = BATT_NUM +6; i <2*BATT_NUM +5; i++)
				if(((p[i]) <= bat->charge_start_voltage) && (bat->charge_start_voltage < (p[i+1])))	
					bat->voltage_to_local = i;
				if(BatVoltage >= (p[2*BATT_NUM +5])){
					capacity = 100;
				}else{	
					if(BatVoltage <= (p[BATT_NUM +6])){
						capacity = 0;
					}else{
						if(BatVoltage <bat->charge_start_voltage){
							for(i = BATT_NUM +6; i <2*BATT_NUM +5; i++)
								if(((p[i]) <= BatVoltage) && (BatVoltage <  (p[i+1]))){
									if( p[i+1] < bat->charge_start_voltage ){
										capacity =bat->charge_start_capacity - ((p[i+1] -BatVoltage) * bat->charge_start_capacity/(bat->voltage_to_local -17+1))/ (p[i+1]- p[i]) - (bat->voltage_to_local- (i ))*bat->charge_start_capacity/(bat->voltage_to_local -17+1);
								 		 DBG("1<<<<<<< %d  bat->voltage_to_local =%d capacity = %d BatVoltage =%d  p[i] = %d,p[i+1] = %d  \n", i, bat->voltage_to_local,capacity,BatVoltage,p[i], p[i+1]);
							
									}
									else {
										capacity =bat->charge_start_capacity - ((bat->charge_start_voltage -BatVoltage) * bat->charge_start_capacity/(bat->voltage_to_local -17+1) )/ (bat->charge_start_voltage - p[i]);
										DBG("2<<<<<< %d   capacity = %d BatVoltage =%d  p[i] = %d,p[i+1] = %d  \n", i,capacity,BatVoltage,p[i], p[i+1]);
									}
									break;
								}
						}else{
							if(BatVoltage > bat->charge_start_voltage){
								for(i = BATT_NUM +6; i <2*BATT_NUM +5; i++)
									if(((p[i]) <= BatVoltage) && (BatVoltage <  (p[i+1]))){
										if( p[i] > bat->charge_start_voltage ){
											capacity = bat->charge_start_capacity + (i +1- (bat->voltage_to_local))*(100- bat->charge_start_capacity )/( 10 -  (bat->voltage_to_local  - 17)) + (BatVoltage - p[i]) * (100- bat->charge_start_capacity )/( 10 -  (bat->voltage_to_local -17))/ (p[i+1]- p[i]);
											DBG("3<<<<<<<< %d bat->voltage_to_local =%d  capacity = %d BatVoltage =%d  p[i] = %d,p[i+1] = %d  \n", i, bat->voltage_to_local,capacity,BatVoltage,p[i], p[i+1]);
										}
										else {
											capacity = bat->charge_start_capacity + (BatVoltage - bat->charge_start_voltage) * (100- bat->charge_start_capacity )/( 10 -  (bat->voltage_to_local-17 )) /(p[i+1] - bat->charge_start_voltage );
											DBG(" 4<<<<<<<<<%d bat->voltage_to_local =%d  capacity = %d BatVoltage =%d  p[i] = %d,p[i+1] = %d  \n", i,bat->voltage_to_local,capacity,BatVoltage,p[i], p[i+1]);
										}
										break;
									}
								}else{
									if(BatVoltage  ==  bat->charge_start_voltage)
										capacity = bat->charge_start_capacity;
								}
							}
						}
					}
			}
		if (capacity == 0)
			capacity = 1;
		if ((capacity == 100) && (is_charge_ok(bat)  ==  0))
			capacity = 99;
		}
		else{  //discharge
			if(BatVoltage >= (p[BATT_NUM +5])){
				capacity = 99;
			}	
			else{
				if(BatVoltage <= (p[6])){
					capacity = 0;
				}
				else{
					for(i = 6; i < BATT_NUM +5; i++){
						if(((p[i]) <= BatVoltage) && (BatVoltage < (p[i+1]))){
							capacity = (i-6)*10+ ((BatVoltage - p[i]) *10 )/ (p[i+1]- p[i]) ;
							break;
						}
					}
				}  
			}
		}

	DBG("real_voltage_to_capacity =%d\n" ,capacity);

    return capacity;
}

#if 0
static void rk_usb_charger(struct rk30_adc_battery_data *bat)
{

	int capacity = 0;
//	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
	int timer_of_charge_sample = NUM_CHARGE_MIN_SAMPLE;
	int timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE;

	if((1 == bat->charge_level)&&( 0 == bat->start_voltage_status)){
		bat->charge_start_voltage = bat->bat_voltage;
		bat->start_voltage_status = 1;
		bat->charge_start_capacity = bat->bat_capacity;
		if(bat->charge_start_capacity %10 != 0){
			bat->charge_up_proprotion = (100 - bat->charge_start_capacity)/10+1;
			bat->charge_down_proportion = bat->charge_start_capacity/10+1;
		}else{
			bat->charge_up_proprotion = (100 - bat->charge_start_capacity)/10;
			bat->charge_down_proportion = bat->charge_start_capacity/10;
		}
	}

	capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);

	if (capacity > bat->bat_capacity){
		if(capacity > bat->bat_capacity + 10 )
			timer_of_charge_sample = NUM_CHARGE_MIN_SAMPLE -10;  //5s
		else if(capacity > bat->bat_capacity + 7 )
			timer_of_charge_sample = NUM_CHARGE_MIN_SAMPLE -5; //10s
		else if(capacity > bat->bat_capacity + 3 )
			timer_of_charge_sample = NUM_CHARGE_MIN_SAMPLE - 2; // 13
		if (++(bat->gBatCapacityusbChargeCnt) >= timer_of_charge_sample){
			bat->gBatCapacityusbChargeCnt  = 0;
			if (bat->bat_capacity < 99){
				bat->bat_capacity++;
				bat->bat_change  = 1;
			}
		}
		bat->gBatCapacityChargeCnt = 0;
		bat->gBatCapacityusbdisChargeCnt = 0;//get_suspend_state(void)
	}else //if(( get_suspend_state() != PM_SUSPEND_MEM)&&(capacity < bat->bat_capacity)){
	// if((gpio_get_value (bat->pdata->back_light_pin) == 1)&&(capacity < bat->bat_capacity)){
	if((capacity < bat->bat_capacity)){
		DBG("USB CHARGE DOWN\n");

		if(capacity < 10){
			timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE - 40; // 13
		}else if(capacity < 20){
			if(capacity + 3 > bat->bat_capacity  )
				timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE - 5;  //5s
			else if(capacity + 7 > bat->bat_capacity )
				timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE - 10; //10s
			else if(capacity + 10> bat->bat_capacity )
				timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE - 25; // 13
			else
				timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE - 35; // 13
		}else{
			if(capacity + 3 > bat->bat_capacity  )
				timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE - 5;  //5s
			else if(capacity + 7 > bat->bat_capacity )
				timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE - 10; //10s
			else if(capacity + 10> bat->bat_capacity )
				timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE - 15; // 13
			else
				timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE - 20; // 13
		}
			
		if (++(bat->gBatCapacityusbdisChargeCnt) >= timer_of_discharge_sample){
			bat->gBatCapacityusbdisChargeCnt = 0;
			if (bat->bat_capacity > 0){
				bat->bat_capacity-- ;
				bat->bat_change  = 1;
			}
		}
			
		bat->gBatCapacityusbChargeCnt  = 0;
	}
	else //if(get_suspend_state() == PM_SUSPEND_MEM){
		//if(gpio_get_value (bat->pdata->back_light_pin) == 0){
	{
		bat->gBatCapacityusbdisChargeCnt = 0;
		// (bat->gBatCapacityusbChargeCnt)++;
		if( is_charge_ok(bat) != INVALID_CHARGE_CHECK){
			if( is_charge_ok(bat) == CHARGE_IS_OK){
				if (++bat->gBatCapacityusbChargeCnt >= timer_of_charge_sample-30){
					bat->gBatCapacityusbChargeCnt = 0;
					if (bat->bat_capacity < 99){
						bat->bat_capacity++;
						bat->bat_change  = 1;
					}
				}
			}else{
				if (capacity > bat->capacitytmp){
					bat->gBatCapacityChargeCnt = 0;
				}else{
					if ((bat->bat_capacity >= 85) &&((bat->gBatCapacityChargeCnt) > NUM_CHARGE_MAX_SAMPLE)){
						bat->gBatCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);

						if (bat->bat_capacity < 99){
							bat->bat_capacity++;
							bat->bat_change  = 1;
						}
					}
				}
			}

		}
		else{
			if (capacity > bat->capacitytmp){
				bat->gBatCapacityChargeCnt = 0;
			}else{
				if ((bat->bat_capacity >= 85) &&(bat->gBatCapacityusbChargeCnt > NUM_CHARGE_MAX_SAMPLE)){
					bat->gBatCapacityusbChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);
						if (bat->bat_capacity < 99){
							bat->bat_capacity++;
							bat->bat_change  = 1;
					}
				}
			}
		}
	}
	bat->capacitytmp = capacity;

}
#endif

static void rk_ac_charger(struct rk30_adc_battery_data *bat)
{
	int capacity = 0;
	int timer_of_charge_sample = NUM_CHARGE_MIN_SAMPLE;

	if((1 == bat->charge_level)&&( 0 == bat->start_voltage_status)){
		bat->charge_start_voltage = bat->bat_voltage;
		bat->start_voltage_status = 1;
		bat->charge_start_capacity = bat->bat_capacity;
		if(bat->charge_start_capacity%10 != 0){
			bat->charge_up_proprotion = (100 - bat->charge_start_capacity)/10+1;
			bat->charge_down_proportion = bat->charge_start_capacity/10+1;
		}else{
			bat->charge_up_proprotion = (100 - bat->charge_start_capacity)/10;
			bat->charge_down_proportion = bat->charge_start_capacity/10;
		}
	}
	capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);
	if (capacity > bat->bat_capacity){
		if(capacity > bat->bat_capacity + 10 )
			timer_of_charge_sample = NUM_CHARGE_MIN_SAMPLE -10;  //5s
		else if(capacity > bat->bat_capacity + 7 )
			timer_of_charge_sample = NUM_CHARGE_MIN_SAMPLE -5; //10s
		else if(capacity > bat->bat_capacity + 3 )
			timer_of_charge_sample = NUM_CHARGE_MIN_SAMPLE - 2; // 13
		if (++(bat->gBatCapacityacChargeCnt) >= timer_of_charge_sample){
			bat->gBatCapacityacChargeCnt  = 0;
			if (bat->bat_capacity < 99){
				bat->bat_capacity++;
				bat->bat_change  = 1;
			}
		}
		bat->gBatCapacityChargeCnt = 0;
	}
	else{  
        bat->gBatCapacityacChargeCnt = 0;
        (bat->gBatCapacityChargeCnt)++;
		if( is_charge_ok(bat) != INVALID_CHARGE_CHECK){
			if( is_charge_ok(bat) == CHARGE_IS_OK){
				if (bat->gBatCapacityChargeCnt >= timer_of_charge_sample){
					bat->gBatCapacityChargeCnt = 0;
					if (bat->bat_capacity < 99){
						bat->bat_capacity++;
						bat->bat_change  = 1;
					}
				}
			}else{
				if (capacity > bat->capacitytmp){
					bat->gBatCapacityChargeCnt = 0;
				}
				else{
					if ((bat->bat_capacity >= 85) &&((bat->gBatCapacityChargeCnt) > NUM_CHARGE_MAX_SAMPLE)){
						bat->gBatCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);

						if (bat->bat_capacity < 99){
							bat->bat_capacity++;
							bat->bat_change  = 1;
						}
					}
				}
			}
		}else{
			if (capacity > bat->capacitytmp){
				bat->gBatCapacityChargeCnt = 0;
			}
			else{
				if ((bat->bat_capacity >= 85) &&(bat->gBatCapacityChargeCnt > NUM_CHARGE_MAX_SAMPLE)){
					bat->gBatCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);

					if (bat->bat_capacity < 99){
						bat->bat_capacity++;
						bat->bat_change  = 1;
					}
				}
			}
		}            
	}
	bat->capacitytmp = capacity;
}

static void rk_battery_charger(struct rk30_adc_battery_data *bat)
{
	int capacity = 0;
	int timer_of_discharge_sample = DISCHARGE_MIN_SECOND;
	
	capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);

	if (capacity < bat->bat_capacity){
		#if 0
		if(capacity <10){
				timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE - 40; // 13
		}else 
		#endif
		if(capacity < 20){
			if(capacity + 3 > bat->bat_capacity  )
				timer_of_discharge_sample = DISCHARGE_MIN_SECOND - 5;  //5s
			else if(capacity + 7 > bat->bat_capacity )
				timer_of_discharge_sample = DISCHARGE_MIN_SECOND - 10; //10s
			else if(capacity + 10> bat->bat_capacity )
				timer_of_discharge_sample = DISCHARGE_MIN_SECOND - 25; // 13
			else
				timer_of_discharge_sample = DISCHARGE_MIN_SECOND - 35; // 13
		}else{
			if(capacity + 3 > bat->bat_capacity  )
				timer_of_discharge_sample = DISCHARGE_MIN_SECOND - 5;  //5s
			else if(capacity + 7 > bat->bat_capacity )
				timer_of_discharge_sample = DISCHARGE_MIN_SECOND - 10; //10s
			else if(capacity + 10> bat->bat_capacity )
				timer_of_discharge_sample = DISCHARGE_MIN_SECOND - 15; // 13
			else
				timer_of_discharge_sample = DISCHARGE_MIN_SECOND - 20; // 13
		}
		if (++(bat->gBatCapacityDisChargeCnt) >= timer_of_discharge_sample){
			bat->gBatCapacityDisChargeCnt = 0;
			if (bat->bat_capacity > 0){
				bat->bat_capacity--;
				bat->bat_change = 1;
			}
		}
	}
	else{
		bat->gBatCapacityDisChargeCnt = 0;
	}
	bat->gBatCapacityChargeCnt = 0;
	bat->gBatCapacityusbdisChargeCnt=0 ;
	bat->gBatCapacityusbChargeCnt =0;
	bat->gBatCapacityacChargeCnt = 0;
	bat->start_voltage_status = 0;
	
	bat->capacitytmp = capacity;
}

static void rk30_adc_battery_capacity_samples(struct rk30_adc_battery_data *bat)
{
//	int capacity = 0;
//	int timer_of_charge_sample = NUM_CHARGE_MIN_SAMPLE;
//	int timer_of_discharge_sample = NUM_CHARGE_MIN_SAMPLE;

	if (bat->bat_status_cnt < NUM_VOLTAGE_SAMPLE)  {
		bat->gBatCapacityDisChargeCnt = 0;
		bat->gBatCapacityChargeCnt    = 0;
		bat->gBatCapacityacChargeCnt = 0;
		return;
	}
	
	if(1 == bat->charge_level){
#if 0
#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)	
		//if(1 == bat->usb_charging)
			rk_usb_charger(bat);
		//else
		//	rk_ac_charger(bat);
#else
		rk_ac_charger(bat);
#endif
#else
		rk_ac_charger(bat);
#endif
	}else{
		rk_battery_charger(bat);
	}

#ifdef CONFIG_NO_BATTERY
	bat->bat_capacity = 100;
#endif

}

//static int poweron_check = 0;
static void rk30_adc_battery_poweron_capacity_check(struct rk30_adc_battery_data *bat)
{
	int new_capacity = 0, old_capacity = 0;
	int cnt = 50 ;

	adc_mtd_index = adc_find_partition_by_name(ADC_CABLIC_FILE);

	new_capacity = bat->bat_capacity;
		
	while( cnt -- ){
	    old_capacity = rk30_adc_battery_load_capacity();
	    if( old_capacity >= 0 ){
	        break ;
	    }
	    msleep(100);
	}

	DBG("%s........new_capacity=%d,old_capacity=%d\n",__func__,new_capacity,old_capacity);
	if ((old_capacity < 0) || (old_capacity > 100)){
		old_capacity = new_capacity;
	}    
#if 0
	if (bat->bat_status == POWER_SUPPLY_STATUS_FULL){
		if (new_capacity > 80){
			bat->bat_capacity = 100;
		}
	}
	else if (bat->bat_status != POWER_SUPPLY_STATUS_NOT_CHARGING){
	//chargeing state
		if( bat ->pdata->is_reboot_charging == 1)
		{
			bat->bat_capacity = ((new_capacity+20) <  old_capacity) ? new_capacity : old_capacity;
		}
	}else{
		if (bat->pdata->is_reboot_charging == 1)
		{
			bat->bat_capacity = ((new_capacity+10) <  old_capacity) ? new_capacity : old_capacity;
		}
		else
		{
			if(new_capacity > old_capacity + 25 )
				bat->bat_capacity = new_capacity;
			else
				bat->bat_capacity = (new_capacity < old_capacity) ? new_capacity : old_capacity;  //avoid the value of capacity increase
		}
	}
#endif

	bat->bat_capacity = old_capacity;
	DBG("%s...bat->bat_capacity=%d,new_capacity=%d,old_capacity=%d\n",__func__,bat->bat_capacity,new_capacity,old_capacity);
	bat->bat_change = 1;
	rk30_adc_battery_put_capacity(bat->bat_capacity);
	put_capacity_num = 1;
}

#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)
#define to_battery_usb_device_info(x) container_of((x), \
		struct rk30_adc_battery_data, usb);

static int rk30_adc_battery_get_usb_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct rk30_adc_battery_data *bat=  to_battery_usb_device_info(psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if (psy->type == POWER_SUPPLY_TYPE_USB){
				val->intval = bat->usb_charging;
				if((1 == charging_flag) && (strstr(saved_command_line,"charger")))
					val->intval = 0;
				if((1 == usb_identify_flag) && (!strstr(saved_command_line,"charger")))
					val->intval = 0;
#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
				if((1 == check_charge_ok) && (!strstr(saved_command_line,"charger")))
					if(1 == charge_en_flags)
						val->intval = 0;
#endif
				DBG("%s......bat->usb_charging=%d,charging_flag=%d,strstr=%d\n",__func__,bat->usb_charging,charging_flag,(!strstr(saved_command_line,"charger")));
			}
			break;

		default:
			return -EINVAL;
	}
	
	return 0;

}

static enum power_supply_property rk30_adc_battery_usb_props[] = {
    
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply rk30_usb_supply = 
{
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,

	.get_property   = rk30_adc_battery_get_usb_property,

	.properties     = rk30_adc_battery_usb_props,
	.num_properties = ARRAY_SIZE(rk30_adc_battery_usb_props),
};
#endif
static irqreturn_t rk30_adc_battery_dc_wakeup(int irq, void *dev_id)
{   
	disable_irq_nosync(irq);
	wake_lock_timeout(&dc_wakelock, 10 * HZ);
	queue_work(gBatteryData->wq, &gBatteryData->dcwakeup_work);
	return IRQ_HANDLED;
}

#define to_battery_ac_device_info(x) container_of((x), \
		struct rk30_adc_battery_data, ac);

static int rk30_adc_battery_get_ac_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret;

	struct rk30_adc_battery_data *bat = to_battery_ac_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS){
			val->intval = bat->ac_charging;
			if( (1 == charging_flag) && (strstr(saved_command_line,"charger")))
				val->intval = 0;
#if defined(CONFIG_CHARGER_LIMITED_BY_TEMP)
			if((1 == check_charge_ok) && (!strstr(saved_command_line,"charger")))
				if(1 == charge_en_flags)
				val->intval = 0;
#endif
			DBG("%s......bat->ac_charging=%d,charging_flag=%d,strstr=%d\n",__func__,bat->ac_charging,charging_flag,(!strstr(saved_command_line,"charger")));
		}
		break;
		
	default:
		ret = -EINVAL;
		break;
	}

	return 0;
}

static enum power_supply_property rk30_adc_battery_ac_props[] = 
{
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply rk30_ac_supply = 
{
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,

	.get_property   = rk30_adc_battery_get_ac_property,

	.properties     = rk30_adc_battery_ac_props,
	.num_properties = ARRAY_SIZE(rk30_adc_battery_ac_props),
};
static void rk30_adc_battery_dcdet_delaywork(struct work_struct *work)
{
	int ret;
	struct rk30_adc_battery_platform_data *pdata;
	int irq;
	int irq_flag;
	struct rk30_adc_battery_data  *bat = container_of((work), \
	struct rk30_adc_battery_data, dcwakeup_work);

	rk28_send_wakeup_key(); // wake up the system
	
	pdata = bat->pdata;
	irq = gpio_to_irq(pdata->dc_det_pin);
	free_irq(irq, NULL);
	msleep(10);
	irq_flag = gpio_get_value (pdata->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	ret = request_irq(irq, rk30_adc_battery_dc_wakeup, irq_flag, "ac_charge_irq", NULL);// reinitialize the DC irq 
	if (ret) {
		free_irq(irq, NULL);
	}

	rk_battery_get_status(bat);

	power_supply_changed(&bat->ac);
	power_supply_changed(&bat->usb);
	DBG("%s......yj\n",__func__);
	bat->bat_status_cnt = 0;        //the state of battery is change
}


static int rk30_adc_battery_get_status(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_status);
}

static int rk30_adc_battery_get_health(struct rk30_adc_battery_data *bat)
{
	return POWER_SUPPLY_HEALTH_GOOD;
}

static int rk30_adc_battery_get_present(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_voltage < BATT_MAX_VOL_VALUE) ? 0 : 1;
}

static int rk30_adc_battery_get_voltage(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_voltage );
}

static int rk30_adc_battery_get_capacity(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_capacity);
}

static int rk30_adc_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{		
	int ret = 0;
	int voltage;
	struct rk30_adc_battery_data  *bat = container_of((psy), \
			struct rk30_adc_battery_data, bat);
	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = rk30_adc_battery_get_status(bat);
			DBG("gBatStatus=%d\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = rk30_adc_battery_get_health(bat);
			DBG("gBatHealth=%d\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = rk30_adc_battery_get_present(bat);
			DBG("gBatPresent=%d\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = rk30_adc_battery_get_voltage(bat);
//			voltage = rk30_adc_battery_get_voltage(bat);
//			val->intval = voltage*1000;

			DBG("gBatVoltage=%d\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			if(battery_test_flag == 2)
				val->intval = 50;
			else
				val->intval = rk30_adc_battery_get_capacity(bat);

			DBG("gBatCapacity=%d%%\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = BATT_MAX_VOL_VALUE;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			val->intval = BATT_ZERO_VOL_VALUE;
			break;
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

static enum power_supply_property rk30_adc_battery_props[] = {

	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

static struct power_supply rk30_battery_supply = 
{
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,

	.get_property   = rk30_adc_battery_get_property,

	.properties     = rk30_adc_battery_props,
	.num_properties = ARRAY_SIZE(rk30_adc_battery_props),
};

#ifdef CONFIG_PM
static void rk30_adc_battery_resume_check(struct rk30_adc_battery_data *bat)
{
	int i;
	int level,oldlevel;
	int new_capacity, old_capacity;
//	struct rk30_adc_battery_data *bat = gBatteryData;

	bat->old_charge_level = -1;
	bat->pSamples = bat->adc_samples;

	adc_sync_read(bat->client);                             //start adc sample
	level = oldlevel =  rk_battery_get_status(bat);//rk30_adc_battery_status_samples(bat);//init charge status

	for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++) {               //0.3 s   
		mdelay(1);
		rk30_adc_battery_voltage_samples(bat);              //get voltage
		level = rk_battery_get_status(bat);// rk30_adc_battery_status_samples(bat);       //check charge status
		if (oldlevel != level){		
		    oldlevel = level;                               //if charge status changed, reset sample
		    i = 0;
		}        
	}
	new_capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);
	old_capacity = bat->suspend_capacity;

	//if (bat->bat_status != POWER_SUPPLY_STATUS_NOT_CHARGING){
	if( 1 == level ){
	//chargeing state
		bat->bat_capacity = (new_capacity > old_capacity) ? new_capacity : old_capacity;
	}
	else{
		bat->bat_capacity = (new_capacity < old_capacity) ? new_capacity : old_capacity;  // aviod the value of capacity increase    dicharge
	}

	if( bat->pdata->batt_low_pin != INVALID_GPIO){
		if (gpio_get_value(bat->pdata->batt_low_pin) == bat->pdata->batt_low_level)
			bat->bat_capacity = 0;
    }

	return;
}

static int rk30_adc_battery_suspend(struct platform_device *dev, pm_message_t state)
{
	int irq;
	struct rk30_adc_battery_data *data = platform_get_drvdata(dev);

	data->suspend_capacity = data->bat_capacity;
	data->suspend_time = get_seconds();
	cancel_delayed_work(&data->delay_work);
	
	if( data->pdata->batt_low_pin != INVALID_GPIO){
		irq = gpio_to_irq(data->pdata->batt_low_pin);
		enable_irq(irq);
	    enable_irq_wake(irq);
    }

	return 0;
}

static int rk30_adc_battery_resume(struct platform_device *dev)
{
	int irq;
	struct rk30_adc_battery_data *data = platform_get_drvdata(dev);

    if( data->bat_capacity < 10 ){
    	wake_lock_timeout(&batt_wake_lock_detect_lowpower,15*HZ);
    	data->bat_change  = 1;
	}

	data->resume_time = get_seconds();
	data->resume = true;
	queue_delayed_work(data->wq, &data->delay_work, msecs_to_jiffies(500));
	if( data->pdata->batt_low_pin != INVALID_GPIO){
		irq = gpio_to_irq(data->pdata->batt_low_pin);
	    disable_irq_wake(irq);
		disable_irq(irq);
    }

	return 0;
}
#else
#define rk30_adc_battery_suspend NULL
#define rk30_adc_battery_resume NULL
#endif


unsigned long AdcTestCnt = 0;
static void rk30_adc_battery_timer_work(struct work_struct *work)
{
struct rk30_adc_battery_data  *bat = container_of((work), \
		struct rk30_adc_battery_data, delay_work);

#if 0
	if (bat->resume) {
	//	if( (bat->resume_time - bat->suspend_time) >= 1800 )
			rk30_adc_battery_resume_check(bat);
		//else
			//bat->bat_capacity = bat->suspend_capacity;
		bat->resume = false;
		bat->bat_change =1;
	}
#endif
	bat->stop_check = 1;
	if( 1 == bat->lower_power_flag ){
		bat->bat_capacity = 0;
		bat->bat_change =1;
	}
	if(bat->poweron_check){   
		bat->poweron_check = 0;
		rk30_adc_battery_poweron_capacity_check(bat);
	}

	bat->charge_level = rk_battery_get_status(bat);
	DBG("bat->charge_level =%d\n", bat->charge_level);
	rk30_adc_battery_status_samples(bat);
	rk30_adc_battery_voltage_samples(bat);
	rk30_adc_battery_capacity_samples(bat);

	if( 0 == bat->pdata->charging_sleep){
		if( 1 == bat->charge_level){  // charge
			if(0 == bat->status_lock ){			
				if (!strstr(saved_command_line,"charger"))
					wake_lock(&batt_wake_lock);  //lock
				bat->status_lock = 1; 
			}
		}
		else{
			if(1 == bat->status_lock ){			
				if (!strstr(saved_command_line,"charger"))
					wake_unlock(&batt_wake_lock);  //unlock
				bat->status_lock = 0; 
			}
		}
	}
	
	/*update battery parameter after adc and capacity has been changed*/
	if(bat->bat_change){
		bat->bat_change= 0;
//		if (strstr(saved_command_line,"charger") == NULL){	 //when low charging-current,not in charging-displays 	
//			if(0 == bat->bat_capacity){
//				bat->ac_charging = 0;
//				bat->usb_charging = 0;
//			}
		if((bat->bat_capacity > 10) && (bat->bat_capacity < 95))
		{
			if(put_capacity_num >= 5)
			{
				rk30_adc_battery_put_capacity(bat->bat_capacity);
				put_capacity_num = 0;
			}else{
				put_capacity_num++;
			}
		}else{
			rk30_adc_battery_put_capacity(bat->bat_capacity);
			put_capacity_num = 0;
		}

#if defined(CONFIG_BATTERY_BQ27410)
		if(!bq27410_init)
#elif defined(CONFIG_BATTERY_BQ27541)
		if(!bq27541_init)
#elif defined(CONFIG_BATTERY_BQ27425)
		if(!bq27425_init)
#endif
		power_supply_changed(&bat->bat);
		power_supply_changed(&bat->ac);
#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)
		power_supply_changed(&bat->usb);
#endif
	}

	//if (rk30_battery_dbg_level){
		if (++AdcTestCnt >= 2)
		{
			AdcTestCnt = 0;

			DBG("Status = %d, RealAdcVal = %d, RealVol = %d,gBatVol = %d, gBatCap = %d, RealCapacity = %d, batt_dischargecnt = %d\n,  chargecnt = %d,ac_count = %d, usb_count =%d ,usb_dischargecount =%d\n", 
			bat->bat_status, bat->adc_val, rk_adc_voltage(bat, bat->adc_val), 
			bat->bat_voltage, bat->bat_capacity, bat->capacitytmp, bat->gBatCapacityDisChargeCnt, bat->gBatCapacityChargeCnt,
			bat->gBatCapacityacChargeCnt, bat->gBatCapacityusbChargeCnt, bat->gBatCapacityusbdisChargeCnt);

		}
	//}
	queue_delayed_work(bat->wq, &bat->delay_work, msecs_to_jiffies(TIMER_MS_COUNTS));
}


static int rk30_adc_battery_io_init(struct rk30_adc_battery_platform_data *pdata)
{
	int ret = 0;
	
	if (pdata->io_init) {
		pdata->io_init();
		return 0;
	}
	
	//charge control pin
	if (pdata->charge_set_pin != INVALID_GPIO){
    	ret = gpio_request(pdata->charge_set_pin, NULL);
    	if (ret) {
    		printk("failed to request dc_det gpio\n");
    		goto error;
	    }
    	gpio_direction_output(pdata->charge_set_pin, 1 - pdata->charge_set_level);
	}
	
	//dc charge detect pin
	if (pdata->dc_det_pin != INVALID_GPIO){
    	ret = gpio_request(pdata->dc_det_pin, NULL);
    	if (ret) {
    		printk("failed to request dc_det gpio\n");
    		goto error;
    	}

    	gpio_pull_updown(pdata->dc_det_pin, GPIOPullUp);//important
    	ret = gpio_direction_input(pdata->dc_det_pin);
    	if (ret) {
    		printk("failed to set gpio dc_det input\n");
    		goto error;
    	}
	}
	
	//charge ok detect
	if (pdata->charge_ok_pin != INVALID_GPIO){
 		ret = gpio_request(pdata->charge_ok_pin, NULL);
    	if (ret) {
    		printk("failed to request charge_ok gpio\n");
    		goto error;
    	}

    	gpio_pull_updown(pdata->charge_ok_pin, GPIOPullUp);//important
    	ret = gpio_direction_input(pdata->charge_ok_pin);
    	if (ret) {
    		printk("failed to set gpio charge_ok input\n");
    		goto error;
    	}
	}
	//batt low pin
	if( pdata->batt_low_pin != INVALID_GPIO){
 		ret = gpio_request(pdata->batt_low_pin, NULL);
    	if (ret) {
    		printk("failed to request batt_low_pin gpio\n");
    		goto error;
    	}

    	gpio_pull_updown(pdata->batt_low_pin, GPIOPullUp); 
    	ret = gpio_direction_input(pdata->batt_low_pin);
    	if (ret) {
    		printk("failed to set gpio batt_low_pin input\n");
    		goto error;
    	}
	}
    
	return 0;
error:
	return -1;
}

extern void kernel_power_off(void);
int get_battery_status(void)
{
	return system_lowerpower;
}
static int rk_adc_battery_poweron_status(struct rk30_adc_battery_data *bat)
{
	int status;
#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)
	int otg_status = 0;
#endif
//	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)
#if  !defined (CONFIG_BATTERY_RK30_USB_AND_CHARGE)
	if(1 == get_ac_status(bat)){
		bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
		bat->ac_charging = 1;
		bat->usb_charging = 0;
	}else if(1 == get_usb_status1(bat)){
		bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
		bat->usb_charging = 1;
	}else{
		bat->bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
		bat->ac_charging = 0;
		bat->usb_charging = 0;
	}
#else
	if (get_ac_status(bat)){
		bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
		bat->ac_charging = 1;
		if (is_charge_ok(bat)  == 1){
			bat->bat_status = POWER_SUPPLY_STATUS_FULL;
			bat->bat_capacity = 100;
		}
	}else{
		bat->bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
		bat->ac_charging = 0;
	}

	if(bat->bat_status == POWER_SUPPLY_STATUS_DISCHARGING)
	{
		otg_status = dwc_otg_check_dpdm();
		if( 0 != otg_status ){
			bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
			if (is_charge_ok(bat) == 1){
				bat->bat_status = POWER_SUPPLY_STATUS_FULL;
				bat->bat_capacity = 100;
			}
		}else{
			bat->bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
			bat->ac_charging = 0;
			bat->usb_charging = 0;
		}

		if(otg_status == 1){
			bat->usb_charging = 1;
		}else if(otg_status == 2){
			bat->usb_charging = 0;
			bat->ac_charging = 1;
		}else{
			bat->usb_charging = 0;
		}
		DBG("%s:charge_status = %d\n", __func__, otg_status);
	}
#endif
#endif
	power_supply_changed(&bat->ac);
#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)
	power_supply_changed(&bat->usb);
#endif

	if(bat->pdata->control_usb_charging){
		if(bat->ac_charging == 1)
			bat->pdata->control_usb_charging(1);
		else if(bat->usb_charging == 1)
			bat->pdata->control_usb_charging(0);
	}

	if((bat->ac_charging == 1)||(bat->usb_charging == 1)){
		bat->old_charge_level =1;
		bat->charge_level = 1;
		status =1;
	}else{
		bat->old_charge_level =0;
		bat->charge_level = 0;
		status =0;
	}

	DBG("%s:status = %d\n", __func__, status);

	return status;
}
static void rk30_adc_battery_check(struct rk30_adc_battery_data *bat)
{
	int i;
	int level,oldlevel;
//	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
	int check_data[NUM_VOLTAGE_SAMPLE];
	//bat->old_charge_level
//	pSamples = bat->adc_samples;

	adc_sync_read(bat->client);                             //start adc sample

	DBG("first_adc_value is  %d \n",bat->adc_val );
	level = oldlevel = rk_adc_battery_poweron_status(bat);// rk30_adc_battery_status_samples(bat);//init charge status

	do{
		for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++){                //0.3 s
			mdelay(1);
			rk30_adc_battery_voltage_samples(bat);              //get voltage
			//level = rk30_adc_battery_status_samples(bat);       //check charge status
			#if 0
			level = rk_adc_battery_poweron_status(bat);

			if (oldlevel != level){
				oldlevel = level;                               //if charge status changed, reset sample
				i = 0;
			}  
			#endif
		}

		for(i=0; i< NUM_VOLTAGE_SAMPLE; i++){
			mdelay(10);
			check_data[i] = bat->adc_val;
			
			adc_async_read(bat->client);
			bat->adc_value += check_data[i];
		}

		bat->adc_value  /=  NUM_VOLTAGE_SAMPLE;

		bat->bat_capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);  //init bat_capacity
		bat->capacitytmp = bat->bat_capacity;
		level = rk_adc_battery_poweron_status(bat);
		if((1 == level)&&(0 == bat->start_voltage_status )){
			bat->charge_start_voltage = bat->bat_voltage;
			bat->start_voltage_status = 1;
			bat->charge_start_capacity = bat->bat_capacity;
			bat->charge_up_proprotion = (100 - bat->charge_start_capacity)/10+1;
			bat->charge_down_proportion = bat->charge_start_capacity/10+1;
		}

		is_usb_charge = (1 == bat->usb_charging);
		is_ac_charge = (1 == bat ->ac_charging);
		DBG("%s:is_usb_charge = %d, is_ac_charge = %d\n", __func__, is_usb_charge, is_ac_charge);
	}while((1 == bat->usb_charging) && (bat->bat_voltage < BATT_USB_CHARGE_LOW_THRES));

	if((1 == bat->usb_charging) && (bat->bat_voltage < BATT_USB_CHARGE_LOW_CHARGER))
		low_usb_charge = true;
	else
		low_usb_charge = false;

#if 0	
	if (get_ac_status(bat) ){		
			bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
			bat->ac_charging = 1;		
			if (is_charge_ok(bat)  == 1){
				bat->bat_status = POWER_SUPPLY_STATUS_FULL;
				bat->bat_capacity = 100;
			}
	}
#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)
	if( 0 != dwc_otg_check_dpdm() ){
		bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
		if (is_charge_ok(bat)  == 1){
			bat->bat_status = POWER_SUPPLY_STATUS_FULL;
			bat->bat_capacity = 100;
		}
	}
	
	if(dwc_otg_check_dpdm() == 1){
		bat->usb_charging = 1;
		if(bat->pdata->control_usb_charging)
			bat->pdata->control_usb_charging(0);
	}else if(dwc_otg_check_dpdm() == 2){
		bat->usb_charging = 0;
		bat->ac_charging = 1;
		if(bat->pdata->control_usb_charging)
			bat->pdata->control_usb_charging(1);	
	}else{
		bat->usb_charging = 0;
	}
#endif
	power_supply_changed(&bat->ac);
#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)
	power_supply_changed(&bat->usb);
#endif
	if((bat->ac_charging == 1)||(bat->usb_charging == 1)){
		bat->old_charge_level =1;
		bat->charge_level = 1;
	}

#endif
#if 0
	if (bat->bat_capacity == 0) {
		bat->bat_capacity = 1;
		system_lowerpower = 1;
	}
#endif

#if 0
	if(0 !=bat->pdata->low_voltage_protection){
		if((bat->bat_voltage <=  bat->pdata->low_voltage_protection))
			system_lowerpower = 1;
		printk("protection lower power .....\n");
	}else{
		if((bat->bat_voltage <= BATT_ZERO_VOL_VALUE))
			system_lowerpower = 1;
		printk("lower power .....\n");
	}	
#endif

#ifdef CONFIG_NO_BATTERY
	bat->bat_voltage = 4200;
	bat->bat_capacity = 100;
#endif

#if 1
	if ((bat->bat_voltage <= BATT_ZERO_VOL_VALUE)&&(bat->bat_status != POWER_SUPPLY_STATUS_CHARGING)){
		kernel_power_off();
	}
#endif
}

static void rk30_adc_battery_callback(struct adc_client *client, void *param, int result)
{
	struct rk30_adc_battery_data  *bat = container_of((client), \
				struct rk30_adc_battery_data, client);

	if (result < 0){
		DBG("adc_battery_callback result < 0 , the value is %d\n", result);
		return;
	}else{
		gBatteryData->adc_val = result;
		DBG("result = %d, gBatteryData->adc_val = %d\n", result, gBatteryData->adc_val );
	}
	return;
}

static void rk30_adc_battery_lowerpower_delaywork(struct work_struct *work)
{
	int irq;

	struct rk30_adc_battery_data  *bat = container_of((work), \
			struct rk30_adc_battery_data, lowerpower_work);

	if( bat->pdata->batt_low_pin != INVALID_GPIO){
		irq = gpio_to_irq(bat->pdata->batt_low_pin);
		disable_irq(irq);
	}

	printk("lowerpower\n");
 	rk28_send_wakeup_key(); // wake up the system	
	return;
}


static irqreturn_t rk30_adc_battery_low_wakeup(int irq,void *dev_id)
{
	queue_work(gBatteryData->wq, &gBatteryData->lowerpower_work);
	return IRQ_HANDLED;
}
static void rk_lowerpower_check(struct rk30_adc_battery_data *bat)
{
	int adc_val = 0;
	int i;
	int check_data[NUM_VOLTAGE_SAMPLE];
	
	for(i=0; i< NUM_VOLTAGE_SAMPLE; i++){
		mdelay(10);
		adc_async_read(bat->client);
		check_data[i] = bat->adc_val;	

		adc_val += check_data[i];
	}
	adc_val /=NUM_VOLTAGE_SAMPLE;

	DBG(">>>>>>>>>>>one>>>%d, two<<<<<%d<<<<\n",bat->adc_value,adc_val);
	DBG(">>>>>>>>>>>firset-value>>>%d, second-value<<<<<%d<<<<\n",rk_adc_voltage(bat, bat->adc_value),rk_adc_voltage(bat, adc_val));

	if((adc_val >= bat->adc_value+5) &&(bat->bat_status == POWER_SUPPLY_STATUS_NOT_CHARGING ) )//
	{
		printk("%d,%d\n",adc_val,bat->adc_value);
		printk("lower-power shutdown");
		kernel_power_off();
	}

}
static void rk_adc_battery_check_work(struct work_struct *work)
{
	struct rk30_adc_battery_data  *bat = container_of((work), \
			struct rk30_adc_battery_data, check_work);
#if !defined(CONFIG_BATTERY_RK30_USB_CHARGE) && !defined(CONFIG_BATTERY_RK30_AC_CHARGE)

#elif !defined(CONFIG_BATTERY_RK30_AC_CHARGE)
	if(0 == get_usb_status1(bat)){
		charging_flag = 1;
		power_supply_changed(&bat->ac);
		power_supply_changed(&bat->usb);
	}
#elif defined(CONFIG_BATTERY_RK30_USB_CHARGE)&&defined(CONFIG_BATTERY_RK30_AC_CHARGE)
	if((1 == get_ac_status(bat))^(0 == get_usb_status1(bat))){
		charging_flag = 1;
		power_supply_changed(&bat->ac);
		power_supply_changed(&bat->usb);
	}
#elif !defined(CONFIG_BATTERY_RK30_USB_CHARGE)&&defined(CONFIG_BATTERY_RK30_AC_CHARGE)
	if((0 == get_ac_status(bat))){
		charging_flag = 1;
		power_supply_changed(&bat->ac);
		power_supply_changed(&bat->usb);
	}
#endif

	if(bat->stop_check != 1)
		queue_delayed_work(bat->wq, &bat->check_work, msecs_to_jiffies(TIMER_MS_COUNTS));

}

void rk30_bat_unregister(void)
{
	power_supply_unregister(&gBatteryData ->bat);
}
EXPORT_SYMBOL(rk30_bat_unregister);

static void poweron_lowerpoer_handle(struct rk30_adc_battery_data *bat)
{
#ifdef CONFIG_LOGO_LOWERPOWER_WARNING
	if((1 ==  get_battery_status())&&(bat->bat_status == POWER_SUPPLY_STATUS_NOT_CHARGING )){
		mdelay (1500);
		kernel_power_off();
	}
#endif

}
static int battery_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct rk30_adc_battery_data *bat=
		container_of(nb, struct rk30_adc_battery_data, battery_nb);

	switch (event) {
		case LCDC_ON:
			DBG("LCDC on \n");
			rk_lowerpower_check(bat);//for lower power protect
			break;

		case BACKLIGHT_ON:
			DBG("display \n");
			poweron_lowerpoer_handle(bat);
			break;
		default:
			return NOTIFY_OK;
		}
	return NOTIFY_OK;
}
static int rk30_adc_battery_probe(struct platform_device *pdev)
{
	int    ret;
	int    irq;
	int    irq_flag;
	struct adc_client                   *client;
	struct rk30_adc_battery_data          *data;
	struct rk30_adc_battery_platform_data *pdata = pdev->dev.platform_data;
#ifdef CONFIG_MACH_RK_FAC
	int i;
	for(i=0;i<BATT_NUM;i++)
	{
		batt_table[6+i]=pdata->chargeArray[i];
		batt_table[BATT_NUM+6+i]=pdata->chargeArray[i];
	}
#endif 	
	gSecondsCnt = get_seconds();
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	memset(data, 0, sizeof(struct rk30_adc_battery_data));
	gBatteryData = data;

	platform_set_drvdata(pdev, data);

   	data->pdata = pdata;
	data->status_lock = 0; 	
	data->old_charge_level = -1;
	data->capacitytmp = 0;
	data->suspend_capacity = 0;
	data->ac_charging = 0;
	data->usb_charging = 0;
	data->full_times = 0;
	data->gBatCapacityDisChargeCnt =0;
	data->gBatCapacityChargeCnt=0;
	data->gBatCapacityusbdisChargeCnt=0 ;
	data->gBatCapacityusbChargeCnt =0;
	data->gBatCapacityacChargeCnt = 0;
	data->charge_source_now = 0;
	data->charge_soure_old = 0;
	data->start_voltage_status = 0;
	data->charge_full_flag =0;
	data->pSamples = data->adc_samples;
	data->lower_power_flag = 0;
	data->capacitytmp = 0;
	data->time_to_full = 0;
	data->stop_check = 0;
	data->voltage_to_local = 0;

	data->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
 	wake_lock_init(&batt_wake_lock, WAKE_LOCK_SUSPEND, "batt_lock");	
 	wake_lock_init(&charge_display_lock, WAKE_LOCK_SUSPEND, "charge_display_lock");	//charge_display_lock
	wake_lock_init(&batt_wake_lock_detect_lowpower, WAKE_LOCK_SUSPEND, "lowpower_lock");	
	wake_lock_init(&dc_wakelock, WAKE_LOCK_SUSPEND, "dc_detect");

	ret = rk30_adc_battery_io_init(pdata);
	if (ret) {
	 	ret = -EINVAL;
	 	goto err_io_init;
	}
    
	memset(data->adc_samples, 0, sizeof(int)*(NUM_VOLTAGE_SAMPLE + 2));

	 //register adc for battery sample
	if(0 == pdata->adc_channel)
		client = adc_register(0, rk30_adc_battery_callback, NULL);  //pdata->adc_channel = ani0
	else
		client = adc_register(pdata->adc_channel, rk30_adc_battery_callback, NULL);  
	if(!client){
		ret = -EINVAL;
		goto err_adc_register_failed;
	}
	data->wq = create_singlethread_workqueue("adc_battd");
	    
	 //variable init
	data->client  = client;
	data->adc_val = adc_sync_read(client);

	data->bat = rk30_battery_supply;
	ret = power_supply_register(&pdev->dev,&data->bat);
	if (ret){
		ret = -EINVAL;
		printk(KERN_INFO "fail to battery power_supply_register\n");
		goto err_battery_failed;
	}
#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)	
 	data->usb = rk30_usb_supply;
	ret = power_supply_register(&pdev->dev, &data->usb);
	if (ret){
		ret = -EINVAL;
		printk(KERN_INFO "fail to usb power_supply_register\n");
		goto err_usb_failed;
	}
#endif
	data->ac = rk30_ac_supply;
	ret = power_supply_register(&pdev->dev, &data->ac);
	if (ret) {
		ret = -EINVAL;
		printk(KERN_INFO "fail to ac power_supply_register\n");
		goto err_ac_failed;
	}

#if  defined (CONFIG_BATTERY_RK30_AC_CHARGE)
	//init dc dectet irq & delay work
	if (pdata->dc_det_pin != INVALID_GPIO){
		INIT_WORK(&data->dcwakeup_work, rk30_adc_battery_dcdet_delaywork);
		
		irq = gpio_to_irq(pdata->dc_det_pin);	        
		irq_flag = gpio_get_value (pdata->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
    	ret = request_irq(irq, rk30_adc_battery_dc_wakeup, irq_flag, "ac_charge_irq", NULL);
    	if (ret) {
		ret = -EINVAL;
    		printk("failed to request dc det irq\n");
    		goto err_dcirq_failed;
    	}
    	enable_irq_wake(irq);    	
	}
#endif

#ifdef BATTERY_APK
	ret = device_create_file(&pdev->dev,&dev_attr_batparam);
	if(ret){
		ret = -EINVAL;
		printk(KERN_ERR "failed to create bat param file\n");
		goto err_battery_failed;
	}
		
	ret = create_sysfs_interfaces(&pdev->dev);
	if (ret < 0)
	{
		ret = -EINVAL;
		dev_err(&pdev->dev,		  
			"device rk30_adc_batterry sysfs register failed\n");
		goto err_sysfs;
	}
#endif 
	//Power on Battery detect
	rk30_adc_battery_check(data);

//	data->wq = create_singlethread_workqueue("adc_battd");
	INIT_DELAYED_WORK(&data->delay_work, rk30_adc_battery_timer_work);

	if(1 == pdata->save_capacity ){
		queue_delayed_work(data->wq, &data->delay_work, msecs_to_jiffies(TIMER_MS_COUNTS*20));
		data->poweron_check = 1;
	}else{
		queue_delayed_work(data->wq, &data->delay_work, msecs_to_jiffies(TIMER_MS_COUNTS));
		data->poweron_check = 0;
	}
	INIT_DELAYED_WORK(&data->check_work, rk_adc_battery_check_work);
	queue_delayed_work(data->wq, &data->check_work, msecs_to_jiffies(TIMER_MS_COUNTS));
	INIT_WORK(&data->load_capacity_work, rk30_adc_battery_load_capacity_delaywork);
	INIT_WORK(&data->put_capacity_work, rk30_adc_battery_put_capacity_delaywork);

	if( pdata->batt_low_pin != INVALID_GPIO){
		if (gpio_get_value(pdata->batt_low_pin) ==0){
			mdelay(20);	   
			if (gpio_get_value(pdata->batt_low_pin) ==0){
				printk("lower power\n");
				kernel_power_off(); 
			}
		}
		
		INIT_WORK(&data->lowerpower_work, rk30_adc_battery_lowerpower_delaywork);		
		irq = gpio_to_irq(pdata->batt_low_pin);		
    	ret = request_irq(irq, rk30_adc_battery_low_wakeup, IRQF_TRIGGER_LOW, "batt_low_irq", NULL);
    	if (ret) {
			ret = -EINVAL;
    		printk("failed to request batt_low_irq irq\n");
    		goto err_lowpowerirq_failed;
    	}
		disable_irq(irq);
    }
	data->battery_nb.notifier_call = battery_notifier_call;
	register_adc_battery_notifier(&data->battery_nb);

	printk(KERN_INFO "rk30_adc_battery: driver initialized\n");
	
	return 0;
err_sysfs:	
#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)
err_usb_failed:
	power_supply_unregister(&data->usb);
#endif
err_ac_failed:
	power_supply_unregister(&data->ac);

err_battery_failed:
	power_supply_unregister(&data->bat);
    
err_dcirq_failed:
	if (pdata->dc_det_pin != INVALID_GPIO){
		free_irq(gpio_to_irq(pdata->dc_det_pin), data);
	}
#if 1
 err_lowpowerirq_failed:
 	if( pdata->batt_low_pin != INVALID_GPIO){
		free_irq(gpio_to_irq(pdata->batt_low_pin), data);
 	}
#endif
err_adc_register_failed:
err_io_init:    
err_data_alloc_failed:
	kfree(data);

	printk("rk30_adc_battery: error!\n");
    
	return ret;
}

static int rk30_adc_battery_remove(struct platform_device *pdev)
{
	struct rk30_adc_battery_data *data = platform_get_drvdata(pdev);
	struct rk30_adc_battery_platform_data *pdata = pdev->dev.platform_data;

	cancel_delayed_work_sync(&data->delay_work);	
#if  defined (CONFIG_BATTERY_RK30_AC_CHARGE)
    if (pdata->dc_det_pin != INVALID_GPIO)
        cancel_work_sync(&data->dcwakeup_work);
#endif
	cancel_delayed_work_sync(&data->check_work);

    if( pdata->batt_low_pin != INVALID_GPIO)
        cancel_work_sync(&data->lowerpower_work);
#if  defined (CONFIG_BATTERY_RK30_USB_CHARGE)
	power_supply_unregister(&data->usb);
#endif
	power_supply_unregister(&data->ac);
	power_supply_unregister(&data->bat);
    if (pdata->dc_det_pin != INVALID_GPIO)
		free_irq(gpio_to_irq(pdata->dc_det_pin), data);

	kfree(data);
	
	return 0;
}

static struct platform_driver rk30_adc_battery_driver = {
	.probe		= rk30_adc_battery_probe,
	.remove		= rk30_adc_battery_remove,
	.suspend	= rk30_adc_battery_suspend,
	.resume		= rk30_adc_battery_resume,
	.driver = {
		.name = "rk30-battery",
		.owner	= THIS_MODULE,
	}
};

static int __init rk30_adc_battery_init(void)
{
	return platform_driver_register(&rk30_adc_battery_driver);
}

static void __exit rk30_adc_battery_exit(void)
{
	platform_driver_unregister(&rk30_adc_battery_driver);
}
//module_init(rk30_adc_battery_init);//module_init(rk30_adc_battery_init);//
#if defined(CONFIG_MACH_RK3026_PHONEPAD_780)
fs_initcall(rk30_adc_battery_init); 
#else
subsys_initcall(rk30_adc_battery_init);
#endif
//fs_initcall(rk30_adc_battery_init);
module_exit(rk30_adc_battery_exit);

MODULE_DESCRIPTION("Battery detect driver for the rk30");
MODULE_AUTHOR("luowei lw@rock-chips.com");
MODULE_LICENSE("GPL");
