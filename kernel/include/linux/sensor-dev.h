/* include/linux/sensor-dev.h - sensor header file
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

#include <linux/miscdevice.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#define SENSOR_ON		1
#define SENSOR_OFF		0
#define SENSOR_UNKNOW_DATA	-1

enum sensor_type {
	SENSOR_TYPE_NULL,
	SENSOR_TYPE_ACCEL,
	SENSOR_TYPE_COMPASS,	
	SENSOR_TYPE_GYROSCOPE,	
	SENSOR_TYPE_LIGHT,	
	SENSOR_TYPE_PROXIMITY,
	SENSOR_TYPE_TEMPERATURE,	
	SENSOR_TYPE_PRESSURE,
	SENSOR_NUM_TYPES
};

enum sensor_id {
	ID_INVALID_l = 0,
		
	ACCEL_ID_ALL_l,
	ACCEL_ID_LIS331_l,
	ACCEL_ID_LSM303DLX_l,
	ACCEL_ID_LIS3DH_l,
	ACCEL_ID_LIS3DE_1,
	ACCEL_ID_KXSD9_l,
	ACCEL_ID_KXTF9_l,
	ACCEL_ID_KXTIK_l,
	ACCEL_ID_KXTJ9_l,
	ACCEL_ID_BMA150_l,
	ACCEL_ID_BMA222_l,
	ACCEL_ID_BMA250_l,
	ACCEL_ID_ADXL34X_l,
	ACCEL_ID_MMA8450_l,
	ACCEL_ID_MMA845X_l,
	ACCEL_ID_MMA7660_l,
	ACCEL_ID_MPU6050_l,
	ACCEL_ID_MXC6225_l,

	COMPASS_ID_ALL_l,
	COMPASS_ID_AK8975_l,
	COMPASS_ID_AK8963_l,
	COMPASS_ID_AK09911_l,
	COMPASS_ID_AK8972_l,
	COMPASS_ID_AMI30X_l,
	COMPASS_ID_AMI306_l,
	COMPASS_ID_YAS529_l,
	COMPASS_ID_YAS530_l,
	COMPASS_ID_HMC5883_l,
	COMPASS_ID_LSM303DLH_l,
	COMPASS_ID_LSM303DLM_l,
	COMPASS_ID_MMC314X_l,
	COMPASS_ID_HSCDTD002B_l,
	COMPASS_ID_HSCDTD004A_l,

	GYRO_ID_ALL_l,
	GYRO_ID_L3G4200D_l,
    	GYRO_ID_L3G20D_l,
	GYRO_ID_K3G_l,

	LIGHT_ID_ALL_l,
	LIGHT_ID_CM3217_l,
	LIGHT_ID_CM3232_l,
	LIGHT_ID_AL3006_l,
	LIGHT_ID_STK3171_l,
	LIGHT_ID_ISL29023_l,
	LIGHT_ID_AP321XX_l,
	LIGHT_ID_ISL5151_l,
	LIGHT_ID_PHOTORESISTOR_l,	
	LIGHT_ID_US5152_l,

	PROXIMITY_ID_ALL_l,
	PROXIMITY_ID_AL3006_l,
	PROXIMITY_ID_STK3171_l,
	PROXIMITY_ID_AP321XX_l,
	
	TEMPERATURE_ID_ALL_l,
	TEMPERATURE_ID_MS5607_l,
	TEMPERATURE_ID_TMP108_l,

	PRESSURE_ID_ALL_l,
	PRESSURE_ID_BMA085_l,
	PRESSURE_ID_MS5607_l,
	SENSOR_NUM_ID_l,
};


struct sensor_axis {
	int x;
	int y;
	int z;
};

struct sensor_flag {
	atomic_t a_flag;	
	atomic_t m_flag;	
	atomic_t mv_flag;	
	atomic_t open_flag;
	atomic_t debug_flag;
	long long delay;	
	wait_queue_head_t open_wq;
};


struct sensor_operate {
	char *name;
	int type;
	int	id_i2c;
	int	range[2];
	int 	brightness[2];//backlight min_brightness max_brightness 
	int read_reg;
	int read_len;
	int id_reg;
	int id_data;
	int precision;
	int ctrl_reg;
	int ctrl_data;
	int int_ctrl_reg;
	int	int_status_reg;
	int trig;	//intterupt trigger
	int (*active)(struct i2c_client *client, int enable, int rate);	
	int (*init)(struct i2c_client *client);	
	int (*report)(struct i2c_client *client);
	int (*suspend)(struct i2c_client *client);
	int (*resume)(struct i2c_client *client);
	struct miscdevice *misc_dev;

};


/* Platform data for the sensor */
struct sensor_private_data {
	int type;
	struct i2c_client *client;	
	struct input_dev *input_dev;
	struct work_struct work;
	struct delayed_work delaywork;	/*report second event*/
	struct sensor_axis axis;
	char sensor_data[40];		//max support40 bytes data
	atomic_t data_ready;
	wait_queue_head_t data_ready_wq;		
	struct mutex data_mutex;
	struct mutex operation_mutex;	
	struct mutex sensor_mutex;
	struct mutex i2c_mutex;
	int status_cur;
	int start_count;
	int devid;
	struct sensor_flag flags;
	struct i2c_device_id *i2c_id;
	struct sensor_platform_data *pdata;
	struct sensor_operate *ops; 
	struct file_operations fops;
	struct miscdevice miscdev;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct 	early_suspend early_suspend;
#endif
};


extern int sensor_register_slave(int type,struct i2c_client *client,
			struct sensor_platform_data *slave_pdata,
			struct sensor_operate *(*get_sensor_ops)(void));


extern int sensor_unregister_slave(int type,struct i2c_client *client,
			struct sensor_platform_data *slave_pdata,
			struct sensor_operate *(*get_sensor_ops)(void));

#if 1
#define DBG(x...) if((atomic_read(&sensor->flags.debug_flag) == sensor->pdata->type) || (atomic_read(&sensor->flags.debug_flag) == SENSOR_NUM_TYPES))printk(x)
#else
#define DBG(x...)
#endif

#define GSENSOR_IOCTL_MAGIC			'a'
#define GBUFF_SIZE				12	/* Rx buffer size */

/* IOCTLs for MMA8452 library */
#define GSENSOR_IOCTL_INIT			_IO(GSENSOR_IOCTL_MAGIC, 0x01)
#define GSENSOR_IOCTL_RESET      	        _IO(GSENSOR_IOCTL_MAGIC, 0x04)
#define GSENSOR_IOCTL_CLOSE		        _IO(GSENSOR_IOCTL_MAGIC, 0x02)
#define GSENSOR_IOCTL_START		        _IO(GSENSOR_IOCTL_MAGIC, 0x03)
#define GSENSOR_IOCTL_GETDATA               	_IOR(GSENSOR_IOCTL_MAGIC, 0x08, char[GBUFF_SIZE+1])
/* IOCTLs for APPs */
#define GSENSOR_IOCTL_APP_SET_RATE		_IOW(GSENSOR_IOCTL_MAGIC, 0x10, char)


#define COMPASS_IOCTL_MAGIC                   'c'
/* IOCTLs for APPs */
#define ECS_IOCTL_APP_SET_MODE		_IOW(COMPASS_IOCTL_MAGIC, 0x10, short)
#define ECS_IOCTL_APP_SET_MFLAG		_IOW(COMPASS_IOCTL_MAGIC, 0x11, short)
#define ECS_IOCTL_APP_GET_MFLAG		_IOW(COMPASS_IOCTL_MAGIC, 0x12, short)
#define ECS_IOCTL_APP_SET_AFLAG		_IOW(COMPASS_IOCTL_MAGIC, 0x13, short)
#define ECS_IOCTL_APP_GET_AFLAG		_IOR(COMPASS_IOCTL_MAGIC, 0x14, short)
#define ECS_IOCTL_APP_SET_TFLAG		_IOR(COMPASS_IOCTL_MAGIC, 0x15, short)/* NOT use */
#define ECS_IOCTL_APP_GET_TFLAG		_IOR(COMPASS_IOCTL_MAGIC, 0x16, short)/* NOT use */
#define ECS_IOCTL_APP_RESET_PEDOMETER   _IOW(COMPASS_IOCTL_MAGIC, 0x17)	/* NOT use */
#define ECS_IOCTL_APP_SET_DELAY		_IOW(COMPASS_IOCTL_MAGIC, 0x18, short)
#define ECS_IOCTL_APP_SET_MVFLAG	_IOW(COMPASS_IOCTL_MAGIC, 0x19, short)
#define ECS_IOCTL_APP_GET_MVFLAG	_IOR(COMPASS_IOCTL_MAGIC, 0x1A, short)
#define ECS_IOCTL_APP_GET_DELAY		_IOR(COMPASS_IOCTL_MAGIC, 0x1B, short)




#define LIGHTSENSOR_IOCTL_MAGIC 'l'
#define LIGHTSENSOR_IOCTL_GET_ENABLED		_IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *) 
#define LIGHTSENSOR_IOCTL_ENABLE		_IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *) 
#define LIGHTSENSOR_IOCTL_DISABLE		_IOW(LIGHTSENSOR_IOCTL_MAGIC, 3, int *)

#define PSENSOR_IOCTL_MAGIC 'p'
#define PSENSOR_IOCTL_GET_ENABLED 		_IOR(PSENSOR_IOCTL_MAGIC, 1, int *)
#define PSENSOR_IOCTL_ENABLE 			_IOW(PSENSOR_IOCTL_MAGIC, 2, int *)
#define PSENSOR_IOCTL_DISABLE       		_IOW(PSENSOR_IOCTL_MAGIC, 3, int *)


#define PRESSURE_IOCTL_MAGIC 'r'
#define PRESSURE_IOCTL_GET_ENABLED 		_IOR(PRESSURE_IOCTL_MAGIC, 1, int *)
#define PRESSURE_IOCTL_ENABLE 			_IOW(PRESSURE_IOCTL_MAGIC, 2, int *)
#define PRESSURE_IOCTL_DISABLE       		_IOW(PRESSURE_IOCTL_MAGIC, 3, int *)
#define PRESSURE_IOCTL_SET_DELAY       		_IOW(PRESSURE_IOCTL_MAGIC, 4, int *)


#define TEMPERATURE_IOCTL_MAGIC 't'
#define TEMPERATURE_IOCTL_GET_ENABLED 		_IOR(TEMPERATURE_IOCTL_MAGIC, 1, int *)
#define TEMPERATURE_IOCTL_ENABLE 		_IOW(TEMPERATURE_IOCTL_MAGIC, 2, int *)
#define TEMPERATURE_IOCTL_DISABLE       	_IOW(TEMPERATURE_IOCTL_MAGIC, 3, int *)
#define TEMPERATURE_IOCTL_SET_DELAY       	_IOW(TEMPERATURE_IOCTL_MAGIC, 4, int *)


extern int sensor_rx_data(struct i2c_client *client, char *rxData, int length);
extern int sensor_tx_data(struct i2c_client *client, char *txData, int length);
extern int sensor_write_reg(struct i2c_client *client, int addr, int value);
extern int sensor_read_reg(struct i2c_client *client, int addr);
extern int sensor_tx_data_normal(struct i2c_client *client, char *buf, int num);
extern int sensor_rx_data_normal(struct i2c_client *client, char *buf, int num);
extern int sensor_write_reg_normal(struct i2c_client *client, char value);
extern int sensor_read_reg_normal(struct i2c_client *client);

