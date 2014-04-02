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
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/lis3de.h>
#if 0
//#define SENSOR_DEBUG_TYPE SENSOR_TYPE_ACCEL
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

#define LIS3DE_INT_COUNT		(0x0E)
#define LIS3DE_WHO_AM_I			(0x0F)

/* full scale setting - register & mask */
#define LIS3DE_TEMP_CFG_REG		(0x1F)
#define LIS3DE_CTRL_REG1		(0x20)
#define LIS3DE_CTRL_REG2		(0x21)
#define LIS3DE_CTRL_REG3		(0x22)
#define LIS3DE_CTRL_REG4		(0x23)
#define LIS3DE_CTRL_REG5		(0x24)
#define LIS3DE_CTRL_REG6		(0x25)
#define LIS3DE_REFERENCE		(0x26)
#define LIS3DE_STATUS_REG		(0x27)
//#define LIS3DE_OUT_X_L			(0x28)
#define LIS3DE_OUT_X_H			(0x29)
//#define LIS3DE_OUT_Y_L			(0x2a)
#define LIS3DE_OUT_Y_H			(0x2b)
//#define LIS3DE_OUT_Z_L			(0x2c)
#define LIS3DE_OUT_Z_H			(0x2d)
#define LIS3DE_FIFO_CTRL_REG		(0x2E)

#define LIS3DE_INT1_CFG			(0x30)
#define LIS3DE_INT1_SRC			(0x31)
#define LIS3DE_INT1_THS			(0x32)
#define LIS3DE_INT1_DURATION		(0x33)

#define LIS3DE_DEVID			(0x33)	//chip id
#define LIS3DE_ACC_DISABLE		(0x08)

#define LIS3DE_RANGE			2000000

/* LIS3DE */
#define LIS3DE_PRECISION		8
#define LIS3DE_BOUNDARY			(0x1 << (LIS3DE_PRECISION - 1))
#define LIS3DE_GRAVITY_STEP		(LIS3DE_RANGE / LIS3DE_BOUNDARY)

#define LIS3DE_COUNT_AVERAGE	1


#define ODR1				0x10  /* 1Hz output data rate */
#define ODR10				0x20  /* 10Hz output data rate */
#define ODR25				0x30  /* 25Hz output data rate */
#define ODR50				0x40  /* 50Hz output data rate */
#define ODR100				0x50  /* 100Hz output data rate */
#define ODR200				0x60  /* 200Hz output data rate */
#define ODR400				0x70  /* 400Hz output data rate */
#define ODR1250				0x90  /* 1250Hz output data rate */


#define G_CONST             990000
#define LIS3DE_SPEED		200 * 1000
#define NR_SAMPHISTLEN 4
#define NR_CHECK_NUM (10*NR_SAMPHISTLEN)
#define CABLIC_NUM 10
#define HEAD 0
#define END  (NR_SAMPHISTLEN)
#define SORT 0
#define CABLIC_DATA   "/data/lis3de_cablic.dat"
#define CABLIC_MAGIC_KEY 7654321

#define MTD_PROC_FILENAME   "/dev/block/mtd/by-name/gsensor"

struct cablic {
	int xoffset;
	int yoffset;
	int zoffset;
	int xoffset1;
	int yoffset1;
	int zoffset1;
	int num1x;
	int num1y;
	int num1z;
	int num2x;
	int num2y;
	int num2z;
	int valid;
};

struct sensor_reg_data {
	char reg;
	char data;
	int x_average;
	int y_average;
	int z_average;
	int count;
};

struct lis3de_axis {
	long int x;
	long int y;
	long int z;
};

struct lis3de_axis_w {
	long int x;
	int xw;
	long int y;
	int yw;
	long int z;
	int zw;
};

static int xoffset1 = 0;
static int yoffset1 = 0;
static int zoffset1 = 0;
static int num1x = 0;
static int num1y = 0;
static int num1z = 0;
static int xoffset2 = 0;
static int yoffset2 = 0;
static int zoffset2 = 0;
static int num2x = 0;
static int num2y = 0;
static int num2z = 0;

//static struct sensor_reg_data axis_average;
static struct cablic cab_arry[CABLIC_NUM];
static const char def_cab[] = {
#if defined(CONFIG_MALATA_D7022)
0x09, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xfe, 0xff, 0xff,
0xfe, 0x6b, 0xff, 0xff, 0x1e, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xb1, 0xcb, 0x74, 0x00, 0x90, 0x01, 0x00, 0x00, 0xf0, 0x19, 0x00, 0x00, 0x25, 0x26, 0x00, 0x00,
0x87, 0xc4, 0xff, 0xff, 0xe7, 0xdc, 0xff, 0xff, 0xe4, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xb1, 0xcb, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb1, 0xcb, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#else
0x41, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x23, 0x92, 0xff, 0xff, 0x02, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xb1, 0xcb, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x3d, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcd, 0x95, 0xff, 0xff,
0xab, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb1, 0xcb, 0x74, 0x00,
0xb2, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x91, 0x9f, 0xff, 0xff, 0x3a, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xb1, 0xcb, 0x74, 0x00, 0x41, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x7a, 0x8e, 0xff, 0xff, 0x02, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xb1, 0xcb, 0x74, 0x00, 0x09, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0xa5, 0xff, 0xff, 0x3a, 0x54, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb1, 0xcb, 0x74, 0x00, 0x09, 0x3d, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xa2, 0xff, 0xff,
0x02, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb1, 0xcb, 0x74, 0x00,
0x41, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x55, 0xa9, 0xff, 0xff, 0xab, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xb1, 0xcb, 0x74, 0x00, 0xb2, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xcd, 0x95, 0xff, 0xff, 0xc9, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xb1, 0xcb, 0x74, 0x00,
#endif
};
static int current_num = 0;
static int revision = -1;
static long int last_z =0, last_x = 0, last_y =0;
static int gsensor_check = 0;
static int gsensor_get = 0;
static int gsensor_check_num = 0;
static int g_filter_m = 990000*2;
static int g_filter = 990000;
static int gsensor_clear = 0;
static struct kobject *android_gsensor_kobj;

struct lis3de_axis mma_dejitter[NR_SAMPHISTLEN];
struct lis3de_axis_w mma_sort[NR_SAMPHISTLEN];
static const unsigned char weight [NR_SAMPHISTLEN ] =
{
	   6, 4, 3, 3 /* The last element is pow2(SUM(0..3)) */
};
/*
#define MTD_PROC_FILENAME   "/proc/mtd"

typedef struct {
    int device_index;
    unsigned int size;
    unsigned int erase_size;
    char *name;
} MtdPartition;

typedef struct {
    MtdPartition *partitions;
    int partitions_allocd;
    int partition_count;
} MtdState;

static MtdState g_mtd_state = {
    NULL,   // partitions
    0,      // partitions_allocd
    -1      // partition_count
};
int
mtd_scan_partitions()
{
    char buf[2048];
    const char *bufp;
    int fd;
    int i;
    ssize_t nbytes;

    if (g_mtd_state.partitions == NULL) {
        const int nump = 32;
        MtdPartition *partitions = kmalloc(nump * sizeof(*partitions), GFP_KERNEL);
        if (partitions == NULL) {
            return -1;
        }
        g_mtd_state.partitions = partitions;
        g_mtd_state.partitions_allocd = nump;
        memset(partitions, 0, nump * sizeof(*partitions));
    }
    g_mtd_state.partition_count = 0;

    // Initialize all of the entries to make things easier later.
     //(Lets us handle sparsely-numbered partitions, which
    // may not even be possible.)

    for (i = 0; i < g_mtd_state.partitions_allocd; i++) {
        MtdPartition *p = &g_mtd_state.partitions[i];
        if (p->name != NULL) {
            kfree(p->name);
            p->name = NULL;
        }
        p->device_index = -1;
    }

    //Open and read the file contents.

    fd = sys_open(MTD_PROC_FILENAME, O_RDONLY, 0);
    if (fd < 0) {
        goto bail;
    }
    nbytes = sys_read(fd, buf, sizeof(buf) - 1);
    sys_close(fd);
    if (nbytes < 0) {
        goto bail;
    }
    buf[nbytes] = '\0';

    // Parse the contents of the file, which looks like:

      //    # cat /proc/mtd
      //    dev:    size   erasesize  name
      //    mtd0: 00080000 00020000 "bootloader"
      //    mtd1: 00400000 00020000 "mfg_and_gsm"
      //   mtd2: 00400000 00020000 "0000000c"
       //   mtd3: 00200000 00020000 "0000000d"
       //   mtd4: 04000000 00020000 "system"
      //    mtd5: 03280000 00020000 "userdata"

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
        // This will fail on the first line, which just contains
         //column headers.

        if (matches == 4) {
            MtdPartition *p = &g_mtd_state.partitions[mtdnum];
            p->device_index = mtdnum;
            p->size = mtdsize;
            p->erase_size = mtderasesize;
            p->name = kmalloc(sizeof(mtdname), GFP_KERNEL);
            if (p->name == NULL) {
                goto bail;
            }
            strcpy(p->name, mtdname);
            g_mtd_state.partition_count++;
        }

        // Eat the line.

        while (nbytes > 0 && *bufp != '\n') {
            bufp++;
            nbytes--;
        }
        if (nbytes > 0) {
            bufp++;
            nbytes--;
        }
    }

    return g_mtd_state.partition_count;

bail:
    // keep "partitions" around so we can free the names on a rescan.
    printk("fial!!!");
    g_mtd_state.partition_count = -1;
    return -1;
}

int mtd_find_partition_by_name(const char *name)
{
	if (g_mtd_state.partitions == NULL)
		mtd_scan_partitions();

    if (g_mtd_state.partitions != NULL) {
        int i;
        for (i = 0; i < g_mtd_state.partitions_allocd; i++) {
            MtdPartition *p = &g_mtd_state.partitions[i];
            if (p->device_index >= 0 && p->name != NULL) {
                if (strcmp(p->name, name) == 0) {
                    return p->device_index;
                }
            }
        }
    }
    return -1;
}
*/
/* AKM HW info */
static void lis3de_average (struct lis3de_axis *samp)
{
	int liTmp;
	long int  x = 0, y = 0, z = 0, Xsum = 0,Ysum = 0, Zsum = 0,diff;

	for (liTmp = NR_SAMPHISTLEN-1; liTmp > 0; liTmp--)
	{
		mma_dejitter[liTmp].x = mma_dejitter[liTmp-1].x;
		mma_dejitter[liTmp].y = mma_dejitter[liTmp-1].y;
		mma_dejitter[liTmp].z = mma_dejitter[liTmp-1].z;
	}

	mma_dejitter[0].x = samp->x;
	mma_dejitter[0].y = samp->y;
	mma_dejitter[0].z = samp->z;

 #if SORT
// sort num
	for (liTmp = 0; liTmp < NR_SAMPHISTLEN; liTmp++)
	{
		mma_sort[liTmp].x = mma_dejitter[liTmp].x;
		mma_sort[liTmp].y = mma_dejitter[liTmp].y;
		mma_sort[liTmp].z = mma_dejitter[liTmp].z;
		mma_sort[liTmp].xw =mma_sort[liTmp].yw = mma_sort[liTmp].zw= weight[liTmp];
	}

	for(i=0;i<NR_SAMPHISTLEN-1;i++)
		for(j=0;j<NR_SAMPHISTLEN-i-1;j++)
		{
			if(mma_sort[j].x > mma_sort[j+1].x)
			{
				t.v =mma_sort[j].x;
				t.w = mma_sort[j].xw;
				mma_sort[j].x = mma_sort[j+1].x;
				mma_sort[j].xw = mma_sort[j+1].xw;
				mma_sort[j+1].x = t.v;
				mma_sort[j+1].xw = t.w;
			}

			if(mma_sort[j].y > mma_sort[j+1].y)
			{
				t.v = mma_sort[j].y;
				t.w = mma_sort[j].yw;
				mma_sort[j].y = mma_sort[j+1].y;
				mma_sort[j].yw = mma_sort[j+1].yw;
				mma_sort[j+1].y = t.v;
				mma_sort[j+1].yw = t.w;
			}

			if(mma_sort[j].z > mma_sort[j+1].z)
			{
				t.v=mma_sort[j].z;
				t.w = mma_sort[j].zw;
				mma_sort[j].z = mma_sort[j+1].z;
				mma_sort[j].zw = mma_sort[j+1].zw;
				mma_sort[j+1].z = t.v;
				mma_sort[j+1].zw = t.w;
			}
		}

	for (liTmp = HEAD; liTmp < END; liTmp++)
	{
		x += mma_sort[liTmp].x * mma_sort[liTmp].xw;
		y += mma_sort[liTmp].y * mma_sort[liTmp].yw;
		z += mma_sort[liTmp].z * mma_sort[liTmp].zw;
		Xsum += mma_sort[liTmp].xw;
		Ysum += mma_sort[liTmp].yw;
		Zsum += mma_sort[liTmp].zw;
	}

 #else

	for (liTmp = 0; liTmp < NR_SAMPHISTLEN; liTmp++)
	{
		mma_sort[liTmp].xw =mma_sort[liTmp].yw = mma_sort[liTmp].zw= weight[liTmp];
	}

	for (liTmp = HEAD; liTmp < END; liTmp++)
	{
		x += mma_dejitter[liTmp].x * mma_sort[liTmp].xw;
		y += mma_dejitter[liTmp].y * mma_sort[liTmp].yw;
		z += mma_dejitter[liTmp].z * mma_sort[liTmp].zw;
		Xsum += mma_sort[liTmp].xw;
		Ysum += mma_sort[liTmp].yw;
		Zsum += mma_sort[liTmp].zw;
	}

 #endif

	samp->x = x / Xsum;
	samp->y = y /Ysum;
	samp->z = z /Zsum;


	diff = ((samp->x - last_x) > 0)?(samp->x - last_x):-(samp->x - last_x);
#if 0
	if(diff < 46876)
	{
		samp->x  = 	last_x;
	}
	else
	{
		last_x = samp->x;
	}

	diff = ((samp->y - last_y) > 0)?(samp->y - last_y):-(samp->y - last_y);
	if(diff < 46876)
	{
		samp->y  = 	last_y;
	}
	else
	{
		last_y = samp->y;
	}
#endif
	diff = ((samp->z - last_z) > 0)?(samp->z - last_z):-(samp->z - last_z);
	if(diff < 46876)
	{
		samp->z  = 	last_z;
	}
	else
	{
		last_z = samp->z;
	}


}


static ssize_t gsensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%#x\n", revision);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, gsensor_vendor_show, NULL);

static int lis3de_load_cablic(const char *addr)
{
	int ret;
	struct cablic cab_arry1[CABLIC_NUM];
	long fd = sys_open(addr,O_RDONLY,0);

	if(fd < 0){
		printk("lis3de_load_offset: open file %s\n", addr);
		return -1;
	}
	ret = sys_read(fd,(char __user *)cab_arry1,sizeof(cab_arry1));
	sys_close(fd);
	//printk("yemk1111:cab_arry1[0].valid %d\n",cab_arry1[0].valid);
	if (cab_arry1[0].valid == CABLIC_MAGIC_KEY)
		memcpy((char *)cab_arry, (char *)cab_arry1, sizeof(cab_arry));

	return ret;
}

static int lis3de_put_cablic(const char *addr)
{
	int ret;
	long fd = sys_open(addr,O_CREAT | O_RDWR | O_TRUNC,0);
	if(fd<0){
		printk("lis3de_put_offset: open file %s\n", addr);
		return -1;
	}
	//printk("yemk:lis3de_put_cablic sys_write\n");
	ret=sys_write(fd,(const char __user *)cab_arry,sizeof(cab_arry));
	sys_close(fd);

	return ret;
}
static ssize_t gsensor_check_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, ret, len;
	char *ptr = (char *)cab_arry;
	char *temp_ptr = buf;

	len = sizeof(cab_arry);

	ret = sprintf(temp_ptr, "cab_arry:");
	temp_ptr = buf + ret;
	for (i = 0; i < len; i++) {
		if (i%16 == 0) {
			ret += sprintf(temp_ptr, "\n");
			temp_ptr = buf + ret;
		}
		ret += sprintf(temp_ptr, " 0x%02x,", ptr[i]);
		temp_ptr = buf + ret;
	}
	ret += sprintf(temp_ptr, "\n", ptr[i]);
	return ret;
}


static ssize_t gsensor_check_store(struct device *dev,
		struct device_attribute *attr, char *buf,size_t count)
{
	//char buffer[3];
	//int num=20;

	gsensor_check = 1;
	xoffset1 = 0;
	yoffset1 = 0;
	zoffset1 = 0;
	num1x = 0;
	num1y = 0;
	num1z = 0;
	xoffset2 = 0;
	yoffset2 = 0;
	zoffset2 = 0;
	num2x = 0;
	num2y = 0;
	num2z = 0;
	return count;

}

static ssize_t gsensor_clear_store(struct device *dev,
		struct device_attribute *attr, char *buf,size_t count)
{
//	char mtddevname[64];
//	int  mtd_index;
	memset((char *)cab_arry, 0x00, sizeof(cab_arry));
	gsensor_clear = 1;
	current_num = 0;

	return count;

}

static ssize_t gsensor_filter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_filter);
}

static ssize_t gsensor_filter_store(struct device *dev,
		struct device_attribute *attr, char *buf,size_t count)
{
	int error;

	error = strict_strtoul(buf, 10, &g_filter);

	if (error)
		return error;

	return count;

}

static DEVICE_ATTR(gsensorcheck, 0666, gsensor_check_show, gsensor_check_store);
static DEVICE_ATTR(gsensorclear, 0666, NULL, gsensor_clear_store);
static DEVICE_ATTR(gsensorfilter, 0664, gsensor_filter_show, gsensor_filter_store);

static int gsensor_sysfs_init(void)
{
	int ret ;

	android_gsensor_kobj = kobject_create_and_add("android_gsensor", NULL);
	if (android_gsensor_kobj == NULL) {
		DBG(KERN_ERR
		       "LIS3DE gsensor_sysfs_init:"\
		       "subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_vendor.attr);
	if (ret) {
		DBG(KERN_ERR
		       "LIS3DE gsensor_sysfs_init:"\
		       "sysfs_create_group failed\n");
		goto err4;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_gsensorcheck.attr);   // "gsensorcheck"
	if (ret) {
		DBG(KERN_ERR
		       "lis3de gsensor_sysfs_init:"\
		       "sysfs_create_group failed\n");
		goto err4;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_gsensorclear.attr);   // "gsensorclear"
	if (ret) {
		DBG(KERN_ERR
		       "lis3de gsensor_sysfs_init: gsensorclear"\
		       "sysfs_create_group failed\n");
		goto err4;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_gsensorfilter.attr);   // "gsensorfilter"
	if (ret) {
		DBG(KERN_ERR
		       "lis3de gsensor_sysfs_init: gsensorfilter"\
		       "sysfs_create_group failed\n");
		goto err4;
	}
	return 0 ;
err4:
	kobject_del(android_gsensor_kobj);
err:
	return ret ;
}

static void lis3de_get_offset_data(void){
	if (gsensor_get){
		int ret;
		gsensor_get = 0;
		/*
		char mtddevname[64];
		int  mtd_index;

		mtd_index = mtd_find_partition_by_name(CABLIC_FILE);
		if (mtd_index >= 0) {
		    sprintf(mtddevname, "/dev/mtd/mtd%d", mtd_index);
		} else {
			sprintf(mtddevname, "%s", CABLIC_DATA);
		}
		*/
		ret=lis3de_load_cablic(MTD_PROC_FILENAME);
		if(ret<0){
			lis3de_load_cablic(CABLIC_DATA);
		}
	}
	return;
}

/****************operate according to sensor chip:start************/

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	int status = 0;
		
	gsensor_get = 1;
	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);

	sensor->ops->ctrl_data |= ODR100;	//100HZ,if 0 then power down
	
	//register setting according to chip datasheet		
	if(!enable)
	{	
		DBG("yemk DISABLE\n");
		status = LIS3DE_ACC_DISABLE;	//lis3de	
		sensor->ops->ctrl_data |= status;	
	}
	else
	{
		DBG("yemk enable\n");
		status = ~LIS3DE_ACC_DISABLE;	//lis3de
		sensor->ops->ctrl_data &= status;
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
	int i;
	struct sensor_reg_data reg_data[] = 
	{			
		{LIS3DE_CTRL_REG2,0X00},			
		{LIS3DE_CTRL_REG4,0x08},	//High resolution output mode: 1, Normal mode	
		{LIS3DE_CTRL_REG6,0x40},	
		{LIS3DE_TEMP_CFG_REG,0x00},	//
		{LIS3DE_FIFO_CTRL_REG,0x00},	//	
		{LIS3DE_INT1_CFG,0xFF},		//6 direction position recognition	
		{LIS3DE_INT1_THS,0x7F},		//Interrupt 1 threshold	
		{LIS3DE_INT1_DURATION,0x7F},	//Duration value 0x00->ox7f
	};  
	
	result = sensor->ops->active(client,0,0);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}

	sensor->status_cur = SENSOR_OFF;
	
	for(i=0;i<(sizeof(reg_data)/sizeof(struct sensor_reg_data));i++)
	{
		result = sensor_write_reg(client, reg_data[i].reg, reg_data[i].data);
		if(result)
		{
			printk("%s:line=%d,i=%d,error\n",__func__,__LINE__,i);
			return result;
		}
	}

	
	if(sensor->pdata->irq_enable)
	{

		result = sensor_write_reg(client, LIS3DE_CTRL_REG3, 0x40);//I1_AOI1 =1  if motion	
		if(result)
		{
			printk("%s:line=%d,error\n",__func__,__LINE__);
			return result;
		}

		result = sensor_write_reg(client, LIS3DE_CTRL_REG5, 0x08);
		if(result)
		{
			printk("%s:line=%d,error\n",__func__,__LINE__);
			return result;
		}

	}
	
	return result;
}

static int sensor_convert_data(struct i2c_client *client, char high_byte, char low_byte)
{
	s64 result;
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	//int precision = sensor->ops->precision;
	switch (sensor->devid) {	
		case LIS3DE_DEVID:		
			result = ((int)high_byte << 8) | (int)low_byte;
			if (result < LIS3DE_BOUNDARY)
       			result = result* LIS3DE_GRAVITY_STEP;
    		else
       			result = ~( ((~result & (0x7fff>>(16-LIS3DE_PRECISION)) ) + 1) 
			   			* LIS3DE_GRAVITY_STEP) + 1;
			break;

		default:
			printk(KERN_ERR "%s: devid wasn't set correctly\n",__func__);
			return -EFAULT;
    }

    return (int)result;
}

static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);	

	/* Report acceleration sensor information */
	input_report_abs(sensor->input_dev, ABS_X, axis->x);
	input_report_abs(sensor->input_dev, ABS_Y, axis->y);
	input_report_abs(sensor->input_dev, ABS_Z, axis->z);
	input_sync(sensor->input_dev);
	DBG("Gsensor x==%d  y==%d z==%d\n",axis->x,axis->y,axis->z);

	return 0;
}

#define GSENSOR_MIN  0
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
			(struct sensor_private_data *) i2c_get_clientdata(client);	
    	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	int x,y,z;
	int i;
	struct sensor_axis axis;	
	char buffer[6] = {0};	
	char value = 0;
	
	if(sensor->ops->read_len < 6)	//sensor->ops->read_len = 6
	{
		printk("%s:lenth is error,len=%d\n",__func__,sensor->ops->read_len);
		return -1;
	}
	
	memset(buffer, 0, 6);
	
	value = sensor_read_reg(client, LIS3DE_STATUS_REG);
	//printk("xmymk value 0x%x\n",value);
	if((value & 0x0f) == 0)
	{
		DBG("yemk %s:line=%d,value=0x%x,data is not ready\n",__func__,__LINE__,value);
		return -1;
	}
	
	
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */	
	/*
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
		return ret;
	} while (0);
	*/
	
	buffer[0] = sensor_read_reg(client, LIS3DE_OUT_X_H);
	buffer[1] = sensor_read_reg(client, LIS3DE_OUT_Y_H);
	buffer[2] = sensor_read_reg(client, LIS3DE_OUT_Z_H);

	lis3de_get_offset_data();
	//this gsensor need 6 bytes buffer
	axis.x = sensor_convert_data(sensor->client, 0, buffer[0]);	//buffer[1]:high bit 
	axis.y = sensor_convert_data(sensor->client, 0, buffer[1]);
	axis.z = -sensor_convert_data(sensor->client, 0, buffer[2]);	
/*
	if ((abs(g_axis.x - axis.x) > g_filter) && (abs(g_axis.x - axis.x) < g_filter_m))
		axis.x = g_axis.x;

	if ((abs(g_axis.y - axis.y) > g_filter) && (abs(g_axis.y - axis.y) < g_filter_m))
		axis.y = g_axis.y;

	if ((abs(g_axis.z - axis.z) > g_filter) && (abs(g_axis.z - axis.z) < g_filter_m))
		axis.z = g_axis.z;

	g_axis = axis;
*/
	//lis3de_average (&axis);
	for (i = 0; i < CABLIC_NUM; i++) {
		//printk("xmymk cab_arry[i].valid %d\n",cab_arry[i].valid);
		if (cab_arry[i].valid != CABLIC_MAGIC_KEY) break;
		if (axis.x > 0)
			axis.x = axis.x - cab_arry[i].xoffset;
		else
			axis.x = axis.x - cab_arry[i].xoffset1;

		if (axis.y > 0)
			axis.y = axis.y - cab_arry[i].yoffset;
		else
			axis.y = axis.y - cab_arry[i].yoffset1;

		if (axis.z > G_CONST)
			axis.z = axis.z - cab_arry[i].zoffset;
		else
			axis.z = axis.z + cab_arry[i].zoffset1;
	}

if (gsensor_clear == 1) {
	gsensor_clear =0;
	/*
	char mtddevname[64];
	int  mtd_index;
	mtd_index = mtd_find_partition_by_name(CABLIC_FILE);
	//printk("ysl index1:%d\n", mtd_index);
	if (mtd_index >= 0) {
	    sprintf(mtddevname, "/dev/mtd/mtd%d", mtd_index);
	} else {
		sprintf(mtddevname, "%s", CABLIC_DATA);
	}
	*/
	ret=lis3de_put_cablic(MTD_PROC_FILENAME);
	if(ret<0){
		lis3de_put_cablic(CABLIC_DATA);
	}

}
	if (gsensor_check){
		if (gsensor_check_num++ < NR_CHECK_NUM) {
			if (axis.x > 0) {
				xoffset1 += axis.x;
				num1x++;
			} else {
				xoffset2 += axis.x;
				num2x++;
			}

			if (axis.y > 0) {
				yoffset1 += axis.y;
				num1y++;
			} else {
				yoffset2 += axis.y;
				num2y++;
			}

			if (axis.z > G_CONST) {
				zoffset1 += (axis.z - G_CONST);
				num1z++;
			} else {
				zoffset2 += (G_CONST - axis.z);
				num2z++;
			}
		} else {
			gsensor_check = 0;
			gsensor_check_num = 0;
			if (num1x > 0)
				xoffset1 = xoffset1/num1x;
			else
				xoffset1 = 0;

			if (num1y > 0)
				yoffset1 = yoffset1/num1y;
			else
				yoffset1 = 0;

			if (num1z > 0)
				zoffset1 = zoffset1/num1z;
			else
				zoffset1 = 0;

			if (num2x > 0)
				xoffset2 = xoffset2/num2x;
			else
				xoffset2 = 0;

			if (num2y > 0)
				yoffset2 = yoffset2/num2y;
			else
				yoffset2 = 0;

			if (num2z > 0)
				zoffset2 = zoffset2/num2z;
			else
				zoffset2 = 0;

			//printk("num1x=%d,num2x=%d,num1y=%d,num2y=%d,num1z=%d,num2z=%d\n",num1x, num2x, num1y, num2y, num1z, num2z);
			//printk("xoffset1=%d,yoffset1=%d,zoffset1=%d\n",xoffset1,yoffset1,zoffset1);
			//printk("xoffset2=%d,yoffset2=%d,zoffset2=%d\n",xoffset2,yoffset2,zoffset2);
			//printk("yemk:current_num %d\n",current_num);
			if (current_num == CABLIC_NUM) current_num = 0;
			cab_arry[current_num].xoffset  = xoffset1;
			cab_arry[current_num].xoffset1 = xoffset2;
			cab_arry[current_num].yoffset  = yoffset1;
			cab_arry[current_num].yoffset1 = yoffset2;
			cab_arry[current_num].zoffset  = zoffset1;
			cab_arry[current_num].zoffset1 = zoffset2;
			cab_arry[current_num++].valid  = CABLIC_MAGIC_KEY;
			/*
			char mtddevname[64];
			int  mtd_index;
			mtd_index = mtd_find_partition_by_name(CABLIC_FILE);
			//printk("ysl index2:%d\n", mtd_index);
			if (mtd_index >= 0) {
			    sprintf(mtddevname, "/dev/mtd/mtd%d", mtd_index);
			} else {
				sprintf(mtddevname, "%s", CABLIC_DATA);
			}
			*/
			ret=lis3de_put_cablic(MTD_PROC_FILENAME);
			if(ret<0){
				lis3de_put_cablic(CABLIC_DATA);
		    }
		}
	}

	x = axis.x;
	y = axis.y;
	z = -axis.z;
	axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z;	
	axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;
	
	//DBG( "%s: axis = %d  %d  %d \n", __func__, axis.x, axis.y, axis.z);

	//Report event  only while value is changed to save some power
	if((abs(sensor->axis.x - axis.x) >= GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) >= GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) >= GSENSOR_MIN))
	{
		gsensor_report_value(client, &axis);

		/* »¥³âµØ»º´æÊý¾Ý. */
		mutex_lock(&(sensor->data_mutex) );
		sensor->axis = axis;
		mutex_unlock(&(sensor->data_mutex) );
	}

	if((sensor->pdata->irq_enable)&& (sensor->ops->int_status_reg >= 0))	//read sensor intterupt status register
	{
		
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:sensor int status :0x%x\n",__func__,value);
	}
	
	return ret;
}

struct sensor_operate gsensor_lis3de_ops = {
	.name				= "lis3de",
	.type				= SENSOR_TYPE_ACCEL,		//sensor type and it should be correct
	.id_i2c				= ACCEL_ID_LIS3DE_1,		//i2c id number
	//.read_reg			= (LIS3DE_OUT_X_L | 0x80),	//read data
	//.read_reg			= LIS3DE_OUT_X_L,	//read data
	.read_len			= 6,				//data length
	.id_reg				= LIS3DE_WHO_AM_I,		//read device id from this register
	.id_data 			= LIS3DE_DEVID,			//device id
	.precision			= LIS3DE_PRECISION,		//8 bits
	.ctrl_reg 			= LIS3DE_CTRL_REG1,		//enable or disable 
	.int_status_reg 		= LIS3DE_INT1_SRC,		//intterupt status register
	.range				= {-LIS3DE_RANGE,LIS3DE_RANGE},	//range
	.trig				= (IRQF_TRIGGER_LOW|IRQF_ONESHOT),		
	.active				= sensor_active,	
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_lis3de_ops;
}


static int __init gsensor_lis3de_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);
	DBG("%s\n",__func__);
	memcpy((char *)cab_arry, def_cab, sizeof(cab_arry));
	result = gsensor_sysfs_init();
	return result;
}

static void __exit gsensor_lis3de_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}


module_init(gsensor_lis3de_init);
module_exit(gsensor_lis3de_exit);


