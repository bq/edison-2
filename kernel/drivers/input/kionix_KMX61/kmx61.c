/*
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Kevin Powell <kpowell@kionix.com>
 * Adapted from drivers by Chris Hudson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/version.h>

#include <linux/workqueue.h>

//#include "kmx61.h"
#include <linux/input/kmx61.h> 


#define NAME			"kmx61"
#define G_MAX			(2048)
#define KMX61_ID_VAL            (0x12)  // KMX61-1008 (AG-ASIC) device ID
#define KMX61G_ID_VAL            (0x17)  // KMX61G (AG-ASIC) device ID

#define KMX61_DATA_RESOLUTION   (14)    // full-scale data resolution

// Miscellaneous time values
#define RESET_DELAY_MSEC        (5)     // ram reset polling interval
#define START_DELAY_MSEC        (20)    // startup time (@200Hz sample)

// Standard registers

#define KMX61_WHO_AM_I          (0x00)
#define KMX61_INS1              (0x01)
#define KMX61_INS2              (0x02)
#define KMX61_STATUS_REG        (0x03)
#define KMX61_ACC_XOUT_L        (0x0A)
#define KMX61_ACC_XOUT_H        (0x0B)
#define KMX61_ACC_YOUT_L        (0x0C)
#define KMX61_ACC_YOUT_H        (0x0D)
#define KMX61_ACC_ZOUT_L        (0x0E)
#define KMX61_ACC_ZOUT_H        (0x0F)
#define KMX61_TEMP_OUT_L        (0x10)
#define KMX61_TEMP_OUT_H        (0x11)
#define KMX61_MAG_XOUT_L        (0x12)
#define KMX61_MAG_XOUT_H        (0x13)
#define KMX61_MAG_YOUT_L        (0x14)
#define KMX61_MAG_YOUT_H        (0x15)
#define KMX61_MAG_ZOUT_L        (0x16)
#define KMX61_MAG_ZOUT_H        (0x17)
#define KMX61_XHP_L             (0x18)
#define KMX61_XHP_H             (0x19)
#define KMX61_YHP_L             (0x1A)
#define KMX61_YHP_H             (0x1B)
#define KMX61_ZHP_L             (0x1C)
#define KMX61_ZHP_H             (0x1D)
#define KMX61_SN_1              (0x24)
#define KMX61_SN_2              (0x25)
#define KMX61_SN_3              (0x26)
#define KMX61_SN_4              (0x27)

#define KMX61_INL               (0x28)
#define KMX61_STBY_REG          (0x29)
#define KMX61_CTRL_REG1         (0x2A)
#define KMX61_CTRL_REG2         (0x2B)
#define KMX61_ODCNTL            (0x2C)
#define KMX61_INC1              (0x2D)
#define KMX61_INC2              (0x2E)
#define KMX61_STR               (0x3C)
#define KMX61_ATH               (0x3D)
#define KMX61_WUFC              (0x3E)
#define KMX61_BTH               (0x3F)
#define KMX61_BTSC              (0x40)
#define KMX61_TEMP_EN_CONTROL   (0x4C)
#define KMX61_MAGCoX_L          (0x50)
#define KMX61_MAGCoX_H          (0x51)
#define KMX61_MAGCoY_L          (0x52)
#define KMX61_MAGCoY_H          (0x53)
#define KMX61_MAGCoZ_L          (0x54)
#define KMX61_MAGCoZ_H          (0x55)
#define KMX61_BUF_THRESH_H      (0x76)
#define KMX61_BUF_THRESH_L      (0x77)
#define KMX61_BUF_CTRL1         (0x78)
#define KMX61_BUF_CTRL2         (0x79)
#define KMX61_BUF_CLEAR         (0x7A)
#define KMX61_BUF_STATUS_REG    (0x7B)
#define KMX61_BUF_STATUS_H      (0x7C)
#define KMX61_BUF_STATUS_L      (0x7D)
#define KMX61_BUF_READ          (0x7E)

// Register Definitions
/* INS1 Registers */
#define KMX61_INS1_BFI          (0x40)
#define KMX61_INS1_WMI          (0x20)
#define KMX61_INS1_DRDY         (0x10)
#define KMX61_INS1_WUFS         (0x02)
#define KMX61_INS1_BTS          (0x01)

/* INS2 Registers */
#define KMX61_INS2_XNWU         (0x20)
#define KMX61_INS2_XPWU         (0x10)
#define KMX61_INS2_YNWU         (0x08)
#define KMX61_INS2_YPWU         (0x04)
#define KMX61_INS2_ZNWU         (0x02)
#define KMX61_INS2_ZPWU         (0x01)

/* STATUS_REG */
#define KMX61_STATUS_REG_INT    (0x10)  // interrupt has occurred

#define KMX61_STBY_REG_ACT_STBY             (0x80)
#define KMX61_STBY_REG_MAG_STBY             (0x02)
#define KMX61_STBY_REG_ACCEL_STBY           (0x01)

#define KMX61_CTRL_REG1_BTSE         (0x80)
#define KMX61_CTRL_REG1_WUFE         (0x40)
#define KMX61_CTRL_REG1_DRDYE        (0x20)
#define KMX61_CTRL_REG1_RES          (0x10) /* 14 bit mode */
#define KMX61_CTRL_REG1_GSEL_MASK    (0x03)
#define KMX61_CTRL_REG1_GSEL_2G      (0x00)
#define KMX61_CTRL_REG1_GSEL_4G      (0x01)
#define KMX61_CTRL_REG1_GSEL_8G      (0x02)
#define KMX61_CTRL_REG1_GSEL_8G_14BIT (0x03)

#define KMX61_CTRL_REG2_SRST         (0x80)
#define KMX61_CTRL_REG2_STNULL       (0x40)
#define KMX61_CTRL_REG2_STPOL        (0x20)
#define KMX61_CTRL_REG2_COTC         (0x10)
#define KMX61_CTRL_REG2_OWUFA        (0x04)
#define KMX61_CTRL_REG2_OWUFB        (0x02)
#define KMX61_CTRL_REG2_OWUFC        (0x01)

#define KMX61_ODCNTL_OSM_MASK     (0xF0)
#define KMX61_ODCNTL_OSA_MASK     (0x0F)

#define KMX61_ODCNTL_OSM_3p125    (0xA0)
#define KMX61_ODCNTL_OSM_6p25     (0xB0)
#define KMX61_ODCNTL_OSM_12p5     (0x00)
#define KMX61_ODCNTL_OSM_25       (0x10)
#define KMX61_ODCNTL_OSM_50       (0x20)
#define KMX61_ODCNTL_OSM_100      (0x30)
#define KMX61_ODCNTL_OSM_200      (0x40)
#define KMX61_ODCNTL_OSM_400      (0x50)
#define KMX61_ODCNTL_OSM_800      (0x60)
#define KMX61_ODCNTL_OSM_1600     (0x70)

#define KMX61_ODCNTL_OSA_3p125    (0x0A)
#define KMX61_ODCNTL_OSA_6p25     (0x0B)
#define KMX61_ODCNTL_OSA_12p5     (0x00)
#define KMX61_ODCNTL_OSA_25       (0x01)
#define KMX61_ODCNTL_OSA_50       (0x02)
#define KMX61_ODCNTL_OSA_100      (0x03)
#define KMX61_ODCNTL_OSA_200      (0x04)
#define KMX61_ODCNTL_OSA_400      (0x05)
#define KMX61_ODCNTL_OSA_800      (0x06)
#define KMX61_ODCNTL_OSA_1600     (0x07)

/*
OSAA	OSAB	OSAC	OSAD	Output Data Rate (Hz)
1	0	1	0	3.125
1	0	1	1	6.25
0	0	0	0	12.5
0	0	0	1	25
0	0	1	0	50
0	0	1	1	100
0	1	0	0	200
0	1	0	1	400
0	1	1	0	800
0	1	1	1	1600
*/

#define KMX61_INC1_BFI1         (0x80)
#define KMX61_INC1_WMI1         (0x40)
#define KMX61_INC1_IEN1         (0x20)
#define KMX61_INC1_IEA1         (0x10)
#define KMX61_INC1_IEL1         (0x08)
#define KMX61_INC1_DRDY_A1      (0x04)
#define KMX61_INC1_DRDY_M1      (0x02)
#define KMX61_INC1_WUFS_BTS     (0x01)

#define KMX61_INCF2_BFI2        (0x80)
#define KMX61_INCF2_WMI2        (0x40)
#define KMX61_INCF2_IEN2        (0x20)
#define KMX61_INCF2_IEA2        (0x10)
#define KMX61_INCF2_IEL2        (0x08)
#define KMX61_INCF2_DRDY_A2     (0x04)
#define KMX61_INCF2_DRDY_M2     (0x02)
#define KMX61_INCF2_WUFS_BTS    (0x01)

#define KMX61_INC3_XNWUE        (0x20)
#define KMX61_INC3_XPWUE        (0x10)
#define KMX61_INC3_YNWUE        (0x08)
#define KMX61_INC3_YPWUE        (0x04)
#define KMX61_INC3_ZNWUE        (0x02)
#define KMX61_INC3_ZPWUE        (0x01)

#define KMX61_TEMP_EN_CNTL      (0x01)

#define KMX61_BUF_CTRL1_BUFE                 (0x80)
#define KMX61_BUF_CTRL1_BUF_FIE              (0x20)
#define KMX61_BUF_CTRL1_BUF_MODE_MASK        (0x03)
#define KMX61_BUF_CTRL1_BUF_MODE_FIFO        (0x00)
#define KMX61_BUF_CTRL1_BUF_MODE_STREAM      (0x01)
#define KMX61_BUF_CTRL1_BUF_MODE_TRIGGER     (0x02)
#define KMX61_BUF_CTRL1_BUF_MODE_FILO        (0x03)

#define KMX61_BUF_CTRL2_BUF_MAG              (0x80)
#define KMX61_BUF_CTRL2_BUF_ACC              (0x40)
#define KMX61_BUF_CTRL2_BUF_TEMP             (0x01)

#define BUF_STATUS_REG_BUF_TRIG              (0x80)

/* INPUT_ABS CONSTANTS */
#define FUZZ			(0)
#define FLAT			(0)


#define KMX61_AM2_REG (0x61)
#define KMX61_AM2_DAT (0x67)

#define KMX61_AM1_REG (0x65)
#define KMX61_AM1_DAT (0xB2)

#define KMX61_MCT_REG (0x7F)
#define KMX61_MCT_PG1 (0x01)
#define KMX61_MCT_PG0 (0x00)

static const uint8_t mag_cal_000[] = {
    0x27, /* 0x1E MAGXICC */
    0x27, /* 0x1F MAGYICC */
    0x27, /* 0x20 MAGZICC */
    0x44, /* 0x21 MAGXYPDC */
    0x04, /* 0x22 MAGZPDC */
    0x77, /* 0x23 MAGXOVLC */
    0x86, /* 0x24 MAGYOVLC */
    0x6C, /* 0x25 MAGZOVLC */
    0x64, /* 0x26 MAGFgX */
    0x5D, /* 0x27 MAGFgY */
    0x44, /* 0x28 MAGFgZ */
    0x00, /* 0x29 MAGFoX */
    0x80, /* 0x2A MAGFoY */
    0x81, /* 0x2B MAGFoZ */
    0x5D, /* 0x2C MAGExtraX */
    0x5D, /* 0x2D MAGExtraY */
    0x5B, /* 0x2E MAGExtraZ */
};

static const uint8_t mag_cal_001[] = {
    0x27,
    0x27,
    0x27,
    0x44,
    0x04,
    0x79,
    0x89,
    0x71,
    0x58,
    0x54,
    0x3C,
    0x00,
    0x00,
    0x00,
    0x5D,
    0x5F,
    0x5C,
};

static const uint8_t mag_cal_002[] = {
    0x27,
    0x27,
    0x27,
    0x44,
    0x04,
    0x75,
    0x88,
    0x61,
    0x57,
    0x4D,
    0x53,
    0x00,
    0x00,
    0x99,
    0x5C,
    0x5D,
    0x5F,
};

static const uint8_t mag_cal_003[] = {
    0x27,
    0x27,
    0x27,
    0x44,
    0x04,
    0x7B,
    0x87,
    0x73,
    0x55,
    0x58,
    0x35,
    0x00,
    0x00,
    0x00,
    0x5E,
    0x5C,
    0x5D,
    0xB4,
    0xA5,
    0x96,
    0x01,
    0x00,
    0x30,
};

static const uint8_t mag_cal_004[] = {
    0x27,
    0x27,
    0x27,
    0x44,
    0x04,
    0x7A,
    0x89,
    0x75,
    0x53,
    0x4E,
    0x1C,
    0x00,
    0x80,
    0x83,
    0x5D,
    0x5C,
    0x5D,
};

/* KLP -- try to make everything be contained in the module */

static int i2c_bus =    0;   /* panda=4, odroid=2, zoom=2, nexus7= */

module_param(i2c_bus, int, S_IRUGO);

struct i2c_client *kmx61_i2c_client;

struct kmx61_platform_data kmx61_data = {
	.min_interval	= 1,
	.init_interval	= 200,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,
};

static struct i2c_board_info kmx61_board_info = {
	I2C_BOARD_INFO("kmx61", 0x0F),
	.platform_data = &kmx61_data,
};

/* list of all the supported accels */
static const unsigned short kmx61_addr_list[] = {0x0e, 0x0f, I2C_CLIENT_END};

/* try to read the WHO_AM_I register; the default value is 0 */
static int __kmx61_probe_i2c(struct i2c_adapter *adapter, unsigned short addr)
{
	struct i2c_msg msgs[2];
	u8 reg_addr = 0x00, value;  //WHO_AM_I is reg 0 on kmx61
	int ret;

	msgs[0].addr = msgs[1].addr = addr;
	msgs[0].len = msgs[1].len = 1;
	msgs[0].flags = msgs[1].flags = 0;
	msgs[1].flags |= I2C_M_RD;
	msgs[0].buf = &reg_addr;
	msgs[1].buf = &value;

	ret = i2c_transfer(adapter, msgs, 2);

	pr_warn("kmx61 probing %d %d %d\n", (int)addr, (int)ret, (int)value);

	return (ret == 2 && value == 0x12);
}

int __init kmx61_init_i2c(void)
{
	struct i2c_adapter *adapter;

	adapter = i2c_get_adapter(i2c_bus);

	if (!adapter) {
		pr_warn("kmx61: i2c_get_adapter(%d) failed\n", i2c_bus);
		goto fail;
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 35)
	kmx61_i2c_client = i2c_new_probed_device(adapter,
			&kmx61_board_info,
			kmx61_addr_list,
			__kmx61_probe_i2c);
#else
	kmx61_i2c_client = i2c_new_probed_device(adapter,
			&kmx61_board_info,
			kmx61_addr_list, NULL);
	i2c_put_adapter(adapter);

#endif

	if (!kmx61_i2c_client) {
		pr_warn("kmx61: i2c_new_probed_device() failed\n");
		goto fail1;
	}

	i2c_put_adapter(adapter);

	return 0;

fail1:
	i2c_put_adapter(adapter);
fail:
	return -1;
}

void kmx61_cleanup_i2c(void)
{
	if (kmx61_i2c_client) {
		i2c_unregister_device(kmx61_i2c_client);
		kmx61_i2c_client = NULL;
	}
}
/* KLP end */

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kmx61_odr_table[] = {
	{1,   KMX61_ODCNTL_OSM_1600 | KMX61_ODCNTL_OSA_1600},
	{3,   KMX61_ODCNTL_OSM_800  | KMX61_ODCNTL_OSA_800},
	{5,   KMX61_ODCNTL_OSM_400  | KMX61_ODCNTL_OSA_400},
	{10,  KMX61_ODCNTL_OSM_200  | KMX61_ODCNTL_OSA_200},
    {20,  KMX61_ODCNTL_OSM_100  | KMX61_ODCNTL_OSA_100},
    {40,  KMX61_ODCNTL_OSM_50   | KMX61_ODCNTL_OSA_50},
    {80,  KMX61_ODCNTL_OSM_25   | KMX61_ODCNTL_OSA_25},
    {160, KMX61_ODCNTL_OSM_12p5 | KMX61_ODCNTL_OSA_12p5},
    {0,   KMX61_ODCNTL_OSM_6p25 | KMX61_ODCNTL_OSA_6p25}
};

struct kmx61_data {
	struct i2c_client *client;
	struct kmx61_platform_data pdata;
	struct input_dev *input_dev;
	struct input_dev *input_dev2;
	unsigned int last_poll_interval;
	struct delayed_work input_work;
	struct mutex lock;
	u8 shift;
	u8 ctrl_reg1;
	u8 odr_mask;
	u8 device_id;
	u8 data_ctrl;
	u8 int_ctrl;
	u8 acc_enabled;
	u8 mag_enabled;
};
#define KMX61G_SENSOR_I2C_RATE 100*1000
static int kmx61_i2c_read(struct kmx61_data *kmx, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = kmx->client->addr,
			.flags = kmx->client->flags,
			.len = 1,
			.buf = &addr,
			.scl_rate = KMX61G_SENSOR_I2C_RATE,
		},
		{
			.addr = kmx->client->addr,
			.flags = kmx->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(kmx->client->adapter, msgs, 2);
}

static void kmx61_set_mag_cal(struct kmx61_data *kmx, const uint8_t *mag_cal, int len)
{
    i2c_smbus_write_byte_data(kmx->client, KMX61_MCT_REG, KMX61_MCT_PG1);
    i2c_smbus_write_block_data(kmx->client, 0x1E, len, mag_cal);
    i2c_smbus_write_byte_data(kmx->client, KMX61_MCT_REG, KMX61_MCT_PG0);
}

static u8 kmx61_standby(struct kmx61_data *kmx)
{
    /* Get th current control register value. */
    u8 const stby_reg = i2c_smbus_read_byte_data(kmx->client, KMX61_STBY_REG);

    /* Put device in standby mode to change settings. */
    i2c_smbus_write_byte_data(kmx->client, KMX61_STBY_REG,
                   KMX61_STBY_REG_ACCEL_STBY | KMX61_STBY_REG_MAG_STBY);

    return stby_reg;
}


static void kmx61_unstandby(struct kmx61_data *kmx, u8 regval)
{
    i2c_smbus_write_byte_data(kmx->client, KMX61_STBY_REG, regval);
}

static void kmx61_report_acceleration_data(struct kmx61_data *kmx) {
    s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
    s16 x, y, z;
    int err;
    u8 drdy;

    drdy = i2c_smbus_read_byte_data(kmx->client, KMX61_INS1);
    while( (drdy & 0x10) == 0) {
        mdelay(1);
        drdy = i2c_smbus_read_byte_data(kmx->client, KMX61_INS1);
    }

    err = kmx61_i2c_read(kmx, KMX61_ACC_XOUT_L, (u8 *) acc_data, 6);
	//printk("kmx61 acc_data[0] = %d,acc_data[1] = %d,acc_data[2] = %d\n",acc_data[0],acc_data[1],acc_data[2]);
	//printk("%d,%d,%d\n",kmx->pdata.axis_map_x,kmx->pdata.axis_map_y,kmx->pdata.axis_map_z);
    if (err < 0)
        dev_err(&kmx->client->dev, "accelerometer data read failed\n");

    x = le16_to_cpu(acc_data[kmx->pdata.axis_map_x]);
    y = le16_to_cpu(acc_data[kmx->pdata.axis_map_y]);
    z = le16_to_cpu(acc_data[kmx->pdata.axis_map_z]);

    x >>= kmx->shift;
    y >>= kmx->shift;
    z >>= kmx->shift;

    /*** DEBUG OUTPUT - REMOVE ***/
    //dev_info(&kmx->client->dev, "kmx61 acc x:%d y:%d z:%d\n", x, y, z);
    /*** <end> DEBUG OUTPUT - REMOVE ***/

    input_report_abs(kmx->input_dev, ABS_X, kmx->pdata.negate_x ? -x : x);
    input_report_abs(kmx->input_dev, ABS_Y, kmx->pdata.negate_y ? -y : y);
    input_report_abs(kmx->input_dev, ABS_Z, kmx->pdata.negate_z ? -z : z);
    input_sync(kmx->input_dev);
}

static void kmx61_report_magnetometer_data(struct kmx61_data *kmx) {
    s16 mag_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
    s16 x, y, z;
    int err;
    u8 drdy;

    drdy = i2c_smbus_read_byte_data(kmx->client, KMX61_INS1);
    while( (drdy & 0x08) == 0) {
        mdelay(1);
        drdy = i2c_smbus_read_byte_data(kmx->client, KMX61_INS1);
    }

    err = kmx61_i2c_read(kmx, KMX61_MAG_XOUT_L, (u8 *) mag_data, 6);
    if (err < 0)
        dev_err(&kmx->client->dev, "magnetometer data read failed\n");

    x = le16_to_cpu(mag_data[kmx->pdata.axis_map_x]);
    y = le16_to_cpu(mag_data[kmx->pdata.axis_map_y]);
    z = le16_to_cpu(mag_data[kmx->pdata.axis_map_z]);

    x >>= 2;
    y >>= 2;
    z >>= 2;

    /*** DEBUG OUTPUT - REMOVE ***/
    //dev_info(&kmx->client->dev, "kmx61 mag x:%d y:%d z:%d\n", x, y, z);
    /*** <end> DEBUG OUTPUT - REMOVE ***/

    input_report_abs(kmx->input_dev2, ABS_X, kmx->pdata.negate_x ? -x : x);
    input_report_abs(kmx->input_dev2, ABS_Y, kmx->pdata.negate_y ? -y : y);
    input_report_abs(kmx->input_dev2, ABS_Z, kmx->pdata.negate_z ? -z : z);

    input_sync(kmx->input_dev2);
}

static int kmx61_update_odr(struct kmx61_data *kmx, unsigned int poll_interval)
{
	int regval, err;
	int i;

	if (poll_interval < kmx->pdata.min_interval)
		return -EINVAL;

	if(kmx->mag_enabled && kmx->acc_enabled) {
	    if(poll_interval > kmx->last_poll_interval)
	        return 0;  //refuse to lower the rate when both sensors enabled
	}

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kmx61_odr_table); i++) {
		kmx->odr_mask = kmx61_odr_table[i].mask;
		if (poll_interval < kmx61_odr_table[i].cutoff)
			break;
	}

	regval = kmx61_standby(kmx);

	err = i2c_smbus_write_byte_data(kmx->client, KMX61_ODCNTL, kmx->odr_mask);

	if (err < 0)
		return err;

	kmx61_unstandby(kmx, regval);

	mutex_lock(&kmx->lock);

	kmx->last_poll_interval = poll_interval;
	if (kmx->input_dev->users || kmx->input_dev2->users) {
		cancel_delayed_work_sync(&kmx->input_work);
		schedule_delayed_work(&kmx->input_work,
				msecs_to_jiffies(kmx->last_poll_interval));
	}

	mutex_unlock(&kmx->lock);

	return 0;
}

static int kmx61_device_power_on(struct kmx61_data *kmx)
{
	if (kmx->pdata.power_on)
		return kmx->pdata.power_on();

	return 0;
}

static void kmx61_device_power_off(struct kmx61_data *kmx)
{

    i2c_smbus_write_byte_data(kmx->client, KMX61_STBY_REG,
            KMX61_STBY_REG_ACCEL_STBY | KMX61_STBY_REG_MAG_STBY);

	cancel_delayed_work_sync(&kmx->input_work);

	if (kmx->pdata.power_off)
		kmx->pdata.power_off();
}

static int kmx61_enable(struct kmx61_data *kmx)
{
	int err;

	err = kmx61_device_power_on(kmx);
	if (err < 0)
		return err;

	/* turn on outputs */

	err = i2c_smbus_write_byte_data(kmx->client, KMX61_STBY_REG, 0);
	if (err < 0)
		return err;

	schedule_delayed_work(&kmx->input_work,
			msecs_to_jiffies(kmx->last_poll_interval));

	err = kmx61_update_odr(kmx, kmx->last_poll_interval);
	if (err < 0)
		return err;

	return 0;
}

static void kmx61_disable(struct kmx61_data *kmx)
{
	kmx61_device_power_off(kmx);
}

static void kmx61_input_work_func(struct work_struct *work)
{
	struct kmx61_data *kmx = container_of((struct delayed_work *)work,
						struct kmx61_data, input_work);
	//dev_info(&kmx->client->dev, "kmx61 input poll\n");
	if(kmx->acc_enabled)
	    kmx61_report_acceleration_data(kmx);
	if(kmx->mag_enabled)
	    kmx61_report_magnetometer_data(kmx);
	schedule_delayed_work(&kmx->input_work,
			      msecs_to_jiffies(kmx->last_poll_interval));
}

int kmx61_input_open(struct input_dev *dev)
{
	struct kmx61_data *kmx = input_get_drvdata(dev);

	//return kmx61_enable(kmx);
	return 0;
}

void kmx61_input_close(struct input_dev *dev)
{
	struct kmx61_data *kmx = input_get_drvdata(dev);

	if ( ! (kmx->input_dev->users || kmx->input_dev2->users) ) {
	    kmx61_disable(kmx);
	}
}

int kmx61_input2_open(struct input_dev *dev)
{
    struct kmx61_data *kmx = input_get_drvdata(dev);

    //return kmx61_enable(kmx);
    return 0;
}

void kmx61_input2_close(struct input_dev *dev)
{
    struct kmx61_data *kmx = input_get_drvdata(dev);

    if ( ! (kmx->input_dev->users || kmx->input_dev2->users) ) {
        kmx61_disable(kmx);
    }
}

static int __devinit kmx61_input_init(struct kmx61_data *kmx)
{
	int err;

	INIT_DELAYED_WORK(&kmx->input_work, kmx61_input_work_func);
	kmx->input_dev = input_allocate_device();
	if (!kmx->input_dev) {
		dev_err(&kmx->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	kmx->input_dev->open = kmx61_input_open;
	kmx->input_dev->close = kmx61_input_close;
	input_set_drvdata(kmx->input_dev, kmx);

	__set_bit(EV_ABS, kmx->input_dev->evbit);
	input_set_abs_params(kmx->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(kmx->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(kmx->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	kmx->input_dev->name = "kmx61_accel";
	kmx->input_dev->id.bustype = BUS_I2C;
	kmx->input_dev->dev.parent = &kmx->client->dev;

	err = input_register_device(kmx->input_dev);
	if (err) {
		dev_err(&kmx->client->dev,
			"unable to register input device %s\n",
			kmx->input_dev->name);
		return err;
	}

    kmx->input_dev2 = input_allocate_device();
    if (!kmx->input_dev2) {
        dev_err(&kmx->client->dev, "input device2 allocate failed\n");
        return -ENOMEM;
    }

    kmx->input_dev2->open = kmx61_input2_open;
    kmx->input_dev2->close = kmx61_input2_close;
    input_set_drvdata(kmx->input_dev2, kmx);

    __set_bit(EV_ABS, kmx->input_dev2->evbit);
    input_set_abs_params(kmx->input_dev2, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
    input_set_abs_params(kmx->input_dev2, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
    input_set_abs_params(kmx->input_dev2, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

    kmx->input_dev2->name = "kmx61_mag";
    kmx->input_dev2->id.bustype = BUS_I2C;
    kmx->input_dev2->dev.parent = &kmx->client->dev;

    err = input_register_device(kmx->input_dev2);
    if (err) {
        dev_err(&kmx->client->dev,
            "unable to register input device %s\n",
            kmx->input_dev2->name);
        return err;
    }

	return 0;
}

/* Returns currently selected poll interval (in ms) */
static ssize_t kmx61_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kmx61_data *kmx = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", kmx->last_poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kmx61_set_delay(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kmx61_data *kmx = i2c_get_clientdata(client);
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	kmx61_update_odr(kmx, interval);

	return count;
}
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR, kmx61_get_delay, kmx61_set_delay);

/* Returns currently selected poll interval (in ms) */
static ssize_t kmx61_get_delay2(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kmx61_data *kmx = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", kmx->last_poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kmx61_set_delay2(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kmx61_data *kmx = i2c_get_clientdata(client);
    unsigned int interval;
    int error;

    error = kstrtouint(buf, 10, &interval);
    if (error < 0)
        return error;

    kmx61_update_odr(kmx, interval);

    return count;
}
static DEVICE_ATTR(delay2, S_IRUGO|S_IWUSR, kmx61_get_delay2, kmx61_set_delay2);
/* Allow users to enable and disable */
static ssize_t kmx61_set_enable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kmx61_data *kmx = i2c_get_clientdata(client);
	unsigned int val;
	int error;

	error = kstrtouint(buf, 10, &val);
	if (error < 0)
		return error;

	if (val) {
	    kmx->acc_enabled  = 1;
	    if(!kmx->mag_enabled)
	        kmx61_enable(kmx);
	} else {
	    kmx->acc_enabled = 0;
	    if(!kmx->mag_enabled)
	        kmx61_disable(kmx);
	}

	return count;
}
static DEVICE_ATTR(enable, S_IWUSR, NULL, kmx61_set_enable);

static ssize_t kmx61_set_enable2(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct kmx61_data *kmx = i2c_get_clientdata(client);
    unsigned int val;
    int error;

    error = kstrtouint(buf, 10, &val);
    if (error < 0)
        return error;

    if (val) {
        kmx->mag_enabled = 1;
        if(!kmx->acc_enabled)
            kmx61_enable(kmx);
    } else {
        kmx->mag_enabled = 0;
        if(!kmx->acc_enabled)
            kmx61_disable(kmx);
    }

    return count;
}

static DEVICE_ATTR(enable2, S_IWUSR, NULL, kmx61_set_enable2);

static struct attribute *kmx61_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_delay2.attr,
	&dev_attr_enable.attr,
	&dev_attr_enable2.attr,
	NULL
};

static struct attribute_group kmx61_attribute_group = {
	.attrs = kmx61_attributes
};

static int __devinit kmx61_verify(struct kmx61_data *kmx)
{
	int retval;

	retval = kmx61_device_power_on(kmx);
	if (retval < 0)
		return retval;

	retval = i2c_smbus_read_byte_data(kmx->client, KMX61_WHO_AM_I);
	if (retval < 0) {
		dev_err(&kmx->client->dev, "read err int source\n");
		goto out;
	}

	//retval = (retval != KMX61G_ID_VAL) ? -EIO : 0;

out:
	kmx61_device_power_off(kmx);
	return retval;
}

static int __devinit kmx61_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	const struct kmx61_platform_data *pdata = client->dev.platform_data;
	struct kmx61_data *kmx;
	int err, reset_pending;
	uint8_t sn[4];
	uint32_t tmp;
	printk("[KMX61G]kmx61_probe#############\n");
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		printk("[KMX61G]client is not i2c capable\n");
		return -ENXIO;
	}
	
	printk("[KMX61G]kmx61_probe_0\n");
	if (!pdata) {
		dev_err(&client->dev, "platform data is NULL; exiting\n");
		return -EINVAL;
	}

	kmx = kzalloc(sizeof(*kmx), GFP_KERNEL);
	if (!kmx) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	kmx->client = client;
	kmx->pdata = *pdata;

	if (pdata->init) {
		err = pdata->init();
		if (err < 0)
			goto err_free_mem;
	}

	mutex_init(&kmx->lock);

	dev_info(&client->dev, "calling kmx61_verify");
	err = kmx61_verify(kmx);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_pdata_exit;
	}
	printk("[KMX61G]kmx61_probe_1\n");

	i2c_set_clientdata(client, kmx);
	printk("[KMX61G]kmx61_probe_2\n");
	kmx->ctrl_reg1 = 0;
	kmx->last_poll_interval = kmx->pdata.init_interval;
	kmx->shift = 2;
	kmx->mag_enabled = 0;
	kmx->acc_enabled = 0;

	err = kmx61_input_init(kmx);
	if (err < 0) {
		dev_err(&client->dev, "input init failed: %d\n", err);
		goto err_pdata_exit;
	}

	err = sysfs_create_group(&client->dev.kobj, &kmx61_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs create failed: %d\n", err);
		goto err_destroy_input;
	}
	printk("[KMX61G]kmx61_probe_3\n");
	/* ******* initialize the hardware *********** */
    /* Reset RAM */
	i2c_smbus_write_byte_data(client, KMX61_CTRL_REG2, KMX61_CTRL_REG2_SRST);
	mdelay(RESET_DELAY_MSEC);
    /* Wait for the reset bit to clear. */
    reset_pending = true;
    do {
        u8 reg = i2c_smbus_read_byte_data(client, KMX61_CTRL_REG2);
        if (reg & KMX61_CTRL_REG2_SRST) {
            mdelay(RESET_DELAY_MSEC);
        } else {
            reset_pending = false;
        }
    } while (reset_pending);

	printk("[KMX61G]kmx61_probe_4\n");

    kmx61_i2c_read(kmx, KMX61_SN_1, sn, sizeof(sn));
    tmp = (sn[3] << 24) | (sn[2] << 16) | (sn[1] << 8) | sn[0];

/*
    if (tmp == 1117) {
        kmx61_set_mag_cal(kmx, mag_cal_000, sizeof(mag_cal_000));
    } else if (tmp == 1118) {
        kmx61_set_mag_cal(kmx, mag_cal_001, sizeof(mag_cal_001));
    } else if (tmp == 1119) {
        kmx61_set_mag_cal(kmx, mag_cal_002, sizeof(mag_cal_002));
    } else if (tmp == 1084) {
        kmx61_set_mag_cal(kmx, mag_cal_003, sizeof(mag_cal_003));
    } else if (tmp == 1086) {
        kmx61_set_mag_cal(kmx, mag_cal_004, sizeof(mag_cal_004));
    }
*/

    dev_err(&client->dev, "hw init start");
    /* Wake up the device in standby mode. */
    i2c_smbus_write_byte_data(client, KMX61_STBY_REG,
                   KMX61_STBY_REG_ACCEL_STBY | KMX61_STBY_REG_MAG_STBY);
    /* Default mode 2G 14 bits all IRQ's off */
    i2c_smbus_write_byte_data(client, KMX61_CTRL_REG1,
                   KMX61_CTRL_REG1_RES | KMX61_CTRL_REG1_GSEL_8G_14BIT | KMX61_CTRL_REG1_DRDYE);

    /* Enable interrupt pin, active high, latching */
    /*i2c_smbus_write_byte_data(&client, KMX61_INC1,
                   KMX61_INC1_IEN1 | KMX61_INC1_IEA1 | KMX61_INC1_IEL1); */

    /* Set the default output data rate. */
    i2c_smbus_write_byte_data(client, KMX61_ODCNTL,
                   KMX61_ODCNTL_OSM_50 | KMX61_ODCNTL_OSA_50);
    /* Clear any outstanding interrupt signal. */
    (void)i2c_smbus_read_byte_data(client, KMX61_INL);

    /* Buffer Control */
    i2c_smbus_write_byte_data(client, KMX61_BUF_CTRL2,
                   KMX61_BUF_CTRL2_BUF_MAG | KMX61_BUF_CTRL2_BUF_ACC);
    i2c_smbus_write_byte_data(client, KMX61_BUF_CLEAR, 0);
    i2c_smbus_write_byte_data(client, KMX61_BUF_THRESH_H, (9 >> 2));
    i2c_smbus_write_byte_data(client, KMX61_BUF_THRESH_L, (9 & 0x03));

    /* Enable the buffer in fifo mode */
    /*i2c_smbus_write_byte_data(client, KMX61_BUF_CTRL1,
                   KMX61_BUF_CTRL1_BUFE | KMX61_BUF_CTRL1_BUF_MODE_FIFO); */
	printk("[KMX61G]kmx61_probe end ============\n");
	
    dev_err(&client->dev, "hw init end");
	return 0;

err_destroy_input:
	input_unregister_device(kmx->input_dev);
	input_unregister_device(kmx->input_dev2);
err_pdata_exit:
	if (kmx->pdata.exit)
		kmx->pdata.exit();
err_free_mem:
	kfree(kmx);
	return err;
}

static int __devexit kmx61_remove(struct i2c_client *client)
{
	struct kmx61_data *kmx = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &kmx61_attribute_group);
	input_unregister_device(kmx->input_dev);
	input_unregister_device(kmx->input_dev2);
	if (kmx->pdata.exit)
		kmx->pdata.exit();

	kfree(kmx);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int kmx61_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kmx61_data *kmx = i2c_get_clientdata(client);
	struct input_dev *input_dev = kmx->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		kmx61_disable(kmx);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int kmx61_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kmx61_data *kmx = i2c_get_clientdata(client);
	int retval = 0;

	mutex_lock(&kmx->input_dev->mutex);

	if (kmx->input_dev->users)
		kmx61_enable(kmx);

	mutex_unlock(&kmx->input_dev->mutex);
	return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(kmx61_pm_ops, kmx61_suspend, kmx61_resume);

static const struct i2c_device_id kmx61_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kmx61_id);

static struct i2c_driver kmx61_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
		.pm	= &kmx61_pm_ops,
	},
	.probe		= kmx61_probe,
	.remove		= __devexit_p(kmx61_remove),
	.id_table	= kmx61_id,
};

static int __init kmx61_init(void)
{
	printk("[KMX61G]kmx61_init\n");
	int res = i2c_add_driver(&kmx61_driver);
	//if (MODULE)
		//kmx61_init_i2c();

	return res;
}
module_init(kmx61_init);

static void __exit kmx61_exit(void)
{
	//if (MODULE)
		//kmx61_cleanup_i2c();
	
	i2c_del_driver(&kmx61_driver);
}
module_exit(kmx61_exit);

MODULE_DESCRIPTION("KMX61 Android accelerometer driver");
MODULE_AUTHOR("Kevin Powell <kpowell@kionix.com>");
MODULE_LICENSE("GPL");
