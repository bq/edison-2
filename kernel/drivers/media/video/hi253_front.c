/*
o* Driver for MT9M001 CMOS Image Sensor from Micron
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/circ_buf.h>
#include <linux/miscdevice.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <plat/rk_camera.h>
static int debug;
module_param(debug, int, S_IRUGO|S_IWUSR);

#define dprintk(level, fmt, arg...) do {			\
	if (debug >= level) 					\
	printk(KERN_WARNING fmt , ## arg); } while (0)

#define SENSOR_TR(format, ...) printk(KERN_ERR format, ## __VA_ARGS__)
#define SENSOR_DG(format, ...) dprintk(1, format, ## __VA_ARGS__)


#define _CONS(a,b) a##b
#define CONS(a,b) _CONS(a,b)

#define __STR(x) #x
#define _STR(x) __STR(x)
#define STR(x) _STR(x)

#define MIN(x,y)   ((x<y) ? x: y)
#define MAX(x,y)    ((x>y) ? x: y)

/* Sensor Driver Configuration */
#define SENSOR_NAME RK29_CAM_SENSOR_HI253_FRONT
#define SENSOR_V4L2_IDENT V4L2_IDENT_HI253
#define SENSOR_ID 0x92
#define SENSOR_MIN_WIDTH    176
#define SENSOR_MIN_HEIGHT   144
#define SENSOR_MAX_WIDTH    1600
#define SENSOR_MAX_HEIGHT   1200
#define SENSOR_INIT_WIDTH	1600			/* Sensor pixel size for sensor_init_data array */
#define SENSOR_INIT_HEIGHT  1200
#define SENSOR_INIT_WINSEQADR sensor_uxga
#define SENSOR_INIT_PIXFMT V4L2_MBUS_FMT_UYVY8_2X8

#define CONFIG_SENSOR_WhiteBalance	1
#define CONFIG_SENSOR_Brightness	0
#define CONFIG_SENSOR_Contrast      0
#define CONFIG_SENSOR_Saturation    0
#define CONFIG_SENSOR_Effect        1
#define CONFIG_SENSOR_Scene         0
#define CONFIG_SENSOR_DigitalZoom   0
#define CONFIG_SENSOR_Focus         0
#define CONFIG_SENSOR_Exposure      0
#define CONFIG_SENSOR_Flash         0
#define CONFIG_SENSOR_Mirror        0
#define CONFIG_SENSOR_Flip          0

#define CONFIG_SENSOR_I2C_SPEED     100000       /* Hz */
/* Sensor write register continues by preempt_disable/preempt_enable for current process not be scheduled */
#define CONFIG_SENSOR_I2C_NOSCHED   0
#define CONFIG_SENSOR_I2C_RDWRCHK   0

#define SENSOR_BUS_PARAM  (SOCAM_MASTER | SOCAM_PCLK_SAMPLE_RISING|\
                          SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |\
                          SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8  |SOCAM_MCLK_24MHZ)

#define COLOR_TEMPERATURE_CLOUDY_DN  6500
#define COLOR_TEMPERATURE_CLOUDY_UP    8000
#define COLOR_TEMPERATURE_CLEARDAY_DN  5000
#define COLOR_TEMPERATURE_CLEARDAY_UP    6500
#define COLOR_TEMPERATURE_OFFICE_DN     3500
#define COLOR_TEMPERATURE_OFFICE_UP     5000
#define COLOR_TEMPERATURE_HOME_DN       2500
#define COLOR_TEMPERATURE_HOME_UP       3500

#define SENSOR_NAME_STRING(a) STR(CONS(SENSOR_NAME, a))
#define SENSOR_NAME_VARFUN(a) CONS(SENSOR_NAME, a)

#define SENSOR_AF_IS_ERR    (0x00<<0)
#define SENSOR_AF_IS_OK		(0x01<<0)
#define SENSOR_INIT_IS_ERR   (0x00<<28)
#define SENSOR_INIT_IS_OK    (0x01<<28)
#define END_REG 0xff

struct reginfo
{
    u8 reg;
    u8 val;
};

//flash off in fixed time to prevent from too hot , zyc
struct  flash_timer{
    struct soc_camera_device *icd;
	struct hrtimer timer;
};
static enum hrtimer_restart flash_off_func(struct hrtimer *timer);

static struct  flash_timer flash_off_timer;
//for user defined if user want to customize the series , zyc
#ifdef CONFIG_HI253_USER_DEFINED_SERIES
#include "hi253_user_series.c"
#else
/* init SVGA preview */
static struct reginfo sensor_init_data[] =
{
//	{0x01, 0xf9}, 
	{0x08, 0x0f}, 
	{0x01, 0xf8}, 

	{0x03, 0x00}, 
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00}, 

	{0x0e, 0x00}, 

	{0x03, 0x00}, 
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00}, 

	{0x0e, 0x00}, 
	{0x01, 0xf1}, 
	{0x08, 0x00}, 
	{0x01, 0xf3},
	{0x01, 0xf1},

	{0x03, 0x20}, 
	{0x10, 0x0c}, 
	{0x03, 0x22}, 
	{0x10, 0x69}, 
	
	//Page 00
	{0x03, 0x00}, 
	{0x10, 0x00}, //lxh
	#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
	{0x11, 0x93},
	#else
	{0x11, 0x90},
	#endif
	#if defined(CONFIG_CAMERA_EMI_ENABLE)
	{0x12, 0x05},
	#else
	{0x12,0x04},
	#endif
	{0x0b, 0xaa}, 
	{0x0c, 0xaa}, 
	{0x0d, 0xaa}, 
	{0x20, 0x00}, 
	{0x21, 0x0a}, //lxh
	{0x22, 0x00}, 
	{0x23, 0x0a}, //lxh
	{0x24, 0x04}, 
	{0x25, 0xb0}, 
	{0x26, 0x06}, 
	{0x27, 0x40}, 

	{0x40, 0x01}, 
	{0x41, 0x98},//lxh 
	{0x42, 0x00}, 
	{0x43, 0x14},//lxh 

	{0x45, 0x04}, 
	{0x46, 0x18}, 
	{0x47, 0xd8}, 

	{0xe1, 0x0f},

	//BLC
	{0x80, 0x2e}, 
	{0x81, 0x7e},
	{0x82, 0x90},
	{0x83, 0x00},
	{0x84, 0x0c},
	{0x85, 0x00},
	{0x90, 0x0c}, 
	{0x91, 0x0c}, 
	{0x92, 0x78}, 
	{0x93, 0x70}, 
	{0x94, 0x75}, 
	{0x95, 0x70}, 
	{0x96, 0xdc},
	{0x97, 0xfe},
	{0x98, 0x20},

	//OutDoor BLC
	{0x99,0x42}, 
	{0x9a,0x42}, 
	{0x9b,0x42}, 
	{0x9c,0x42}, 

	//Dark BLC
	{0xa0, 0x00},
	{0xa2, 0x00},
	{0xa4, 0x00},
	{0xa6, 0x00},

	//Normal BLC
	{0xa8, 0x43},
	{0xaa, 0x43},
	{0xac, 0x43},
	{0xae, 0x43},

	//Page 02
	{0x03, 0x02}, 
	{0x12, 0x03},
	{0x13, 0x03},
	{0x16, 0x00},
	{0x17, 0x8C},
	{0x18, 0x4c}, 
	{0x19, 0x00}, 
	{0x1a, 0x39},
	{0x1c, 0x09},
	{0x1d, 0x40},
	{0x1e, 0x30},
	{0x1f, 0x10},
	{0x20, 0x77},
	{0x21, 0xde}, 
	{0x22, 0xa7},
	{0x23, 0x30},
	{0x27, 0x3c},
	{0x2b, 0x80},
	{0x2e, 0x00},
	{0x2f, 0x00},
	{0x30, 0x05},
	{0x50, 0x20},
	{0x52, 0x01},
	{0x53, 0xc1},
	{0x55, 0x1c},
	{0x56, 0x11},
	{0x5d, 0xA2},
	{0x5e, 0x5a},
	{0x60, 0x87},
	{0x61, 0x99},
	{0x62, 0x88},
	{0x63, 0x97},
	{0x64, 0x88},
	{0x65, 0x97},
	{0x67, 0x0c},
	{0x68, 0x0c},
	{0x69, 0x0c},
	{0x72, 0x89},
	{0x73, 0x96}, 
	{0x74, 0x89},
	{0x75, 0x96}, 
	{0x76, 0x89},
	{0x77, 0x96}, 
	{0x7C, 0x85},
	{0x7d, 0xaf},
	{0x80, 0x01},
	{0x81, 0x7f}, 
	{0x82, 0x13}, 
	{0x83, 0x24}, 
	{0x84, 0x7d},
	{0x85, 0x81},
	{0x86, 0x7d},
	{0x87, 0x81},
	{0x92, 0x48}, 
	{0x93, 0x54}, 
	{0x94, 0x7d},
	{0x95, 0x81},
	{0x96, 0x7d},
	{0x97, 0x81},
	{0xa0, 0x02},
	{0xa1, 0x7b},
	{0xa2, 0x02},
	{0xa3, 0x7b},
	{0xa4, 0x7b},
	{0xa5, 0x02},
	{0xa6, 0x7b},
	{0xa7, 0x02},
	{0xa8, 0x85},
	{0xa9, 0x8c},
	{0xaa, 0x85},
	{0xab, 0x8c},
	{0xac, 0x10},
	{0xad, 0x16},
	{0xae, 0x10},
	{0xaf, 0x16},
	{0xb0, 0x99},
	{0xb1, 0xa3},
	{0xb2, 0xa4},
	{0xb3, 0xae},
	{0xb4, 0x9b},
	{0xb5, 0xa2},
	{0xb6, 0xa6},
	{0xb7, 0xac},
	{0xb8, 0x9b},
	{0xb9, 0x9f},
	{0xba, 0xa6},
	{0xbb, 0xaa},
	{0xbc, 0x9b},
	{0xbd, 0x9f},
	{0xbe, 0xa6},
	{0xbf, 0xaa},
	{0xc4, 0x2c},
	{0xc5, 0x43},
	{0xc6, 0x63},
	{0xc7, 0x79},
	{0xc8, 0x2d},
	{0xc9, 0x42},
	{0xca, 0x2d},
	{0xcb, 0x42},
	{0xcc, 0x64},
	{0xcd, 0x78},
	{0xce, 0x64},
	{0xcf, 0x78},
	{0xd0, 0x0a},
	{0xd1, 0x09},
	{0xd4, 0x0c},
	{0xd5, 0x0c},
	{0xd6, 0xd8},
	{0xd7, 0xd0},//lxh
	{0xe0, 0xc4},
	{0xe1, 0xc4},
	{0xe2, 0xc4},
	{0xe3, 0xc4},
	{0xe4, 0x00},
	{0xe8, 0x80}, 
	{0xe9, 0x40},
	{0xea, 0x7f}, 
	{0xf0, 0x01}, 
	{0xf1, 0x01}, 
	{0xf2, 0x01}, 
	{0xf3, 0x01}, 
	{0xf4, 0x01}, 

	//PAGE10
	{0x03, 0x10},
	{0x10, 0x01}, //lxh
    {0x11, 0x03}, //lxh,normal	
	{0x12, 0x00},
	{0x13, 0x00},
	{0x20, 0x00}, 

	{0x40, 0x80},
	{0x41, 0x00},
	{0x48, 0x88},// 84
	{0x50, 0x90},
	{0x30, 0x00},
	{0x31, 0x00},
	{0x32, 0x00},
	{0x33, 0x00},

	{0x34, 0x30},
	{0x35, 0x00},
	{0x36, 0x00},
	{0x38, 0x00},
	{0x3e, 0x58},
	{0x3f, 0x00},

	//Saturation
	{0x60, 0x6f},
	{0x61, 0x95},// 74
	{0x62, 0x95},// 76
	{0x63, 0x30},
	{0x64, 0x41},

	{0x66, 0x33},
	{0x67, 0x00},

	{0x6a, 0x90}, 
	{0x6b, 0x80}, 
	{0x6c, 0x80}, 
	{0x6d, 0xa0}, 

	{0x76, 0x01}, 
	{0x74, 0x66},
	{0x79, 0x06},
	
	//Page 11
	{0x03, 0x11}, 
	{0x10, 0x7f},//lxh,3f 
	{0x11, 0x40},
	{0x12, 0xba},
	{0x13, 0xcb},
	{0x26, 0x20}, 
	{0x27, 0x22}, 
	{0x28, 0x0f}, 
	{0x29, 0x10}, 
	{0x2b, 0x30}, 
	{0x2c, 0x32}, 

	//Out2 D-LPF th
	{0x30, 0x70}, 
	{0x31, 0x10}, 
	{0x32, 0x65}, 
	{0x33, 0x09}, 
	{0x34, 0x06}, 
	{0x35, 0x04}, 

	//Out1 D-LPF th
	{0x36, 0x70}, 
	{0x37, 0x18}, 
	{0x38, 0x65}, 
	{0x39, 0x09}, 
	{0x3a, 0x06}, 
	{0x3b, 0x04}, 

	//Indoor D-LPF th
	{0x3c, 0x80}, 
	{0x3d, 0x18}, 
	{0x3e, 0x80}, 
	{0x3f, 0x0c}, 
	{0x40, 0x09}, 
	{0x41, 0x06}, 

	{0x42, 0x80}, 
	{0x43, 0x18}, 
	{0x44, 0x80}, 
	{0x45, 0x12}, 
	{0x46, 0x10}, 
	{0x47, 0x10}, 
	{0x48, 0x90}, 
	{0x49, 0x40}, 
	{0x4a, 0x80}, 
	{0x4b, 0x13}, 
	{0x4c, 0x10}, 
	{0x4d, 0x11}, 
	{0x4e, 0x80}, 
	{0x4f, 0x30}, 
	{0x50, 0x80}, 
	{0x51, 0x13}, 
	{0x52, 0x10}, 
	{0x53, 0x13}, 
	{0x54, 0x11},
	{0x55, 0x17},
	{0x56, 0x20},
	{0x57, 0x20},
	{0x58, 0x20},
	{0x59, 0x30},
	{0x5a, 0x18},
	{0x5b, 0x00},
	{0x5c, 0x00},
	{0x60, 0x3f},
	{0x62, 0x50},
	{0x70, 0x06},
	
	//Page 12
	{0x03, 0x12}, 
	{0x20, 0x0f},
	{0x21, 0x0f},
	{0x25, 0x30},
	{0x28, 0x00}, 
	{0x29, 0x00}, 
	{0x2a, 0x00},
	{0x30, 0x50},
	{0x31, 0x18}, 
	{0x32, 0x32}, 
	{0x33, 0x40}, 
	{0x34, 0x50}, 
	{0x35, 0x70}, 
	{0x36, 0xa0}, 

	//Out2 th
	{0x40, 0xa0}, 
	{0x41, 0x40}, 
	{0x42, 0xa0}, 
	{0x43, 0x90}, 
	{0x44, 0x90}, 
	{0x45, 0x80}, 

	//Out1 th
	{0x46, 0xb0}, 
	{0x47, 0x55}, 
	{0x48, 0xa0}, 
	{0x49, 0x90}, 
	{0x4a, 0x90}, 
	{0x4b, 0x80}, 

	//In door th
	{0x4c, 0xb0},
	{0x4d, 0x40},
	{0x4e, 0x90},
	{0x4f, 0x90},
	{0x50, 0xe6},
	{0x51, 0x80},

	//Dark1 th
	{0x52, 0xb0},
	{0x53, 0x60},
	{0x54, 0xc0},
	{0x55, 0xc0},
	{0x56, 0xc0},
	{0x57, 0x80},

	//Dark2 th
	{0x58, 0x90},
	{0x59, 0x40},
	{0x5a, 0xd0},
	{0x5b, 0xd0},
	{0x5c, 0xe0},
	{0x5d, 0x80},

	//Dark3 th
	{0x5e, 0x88},
	{0x5f, 0x40},
	{0x60, 0xe0},
	{0x61, 0xe6},
	{0x62, 0xe6},
	{0x63, 0x80},

	{0x70, 0x15},
	{0x71, 0x01},

	{0x72, 0x18},
	{0x73, 0x01},

	{0x74, 0x25},
	{0x75, 0x15},
	{0x80, 0x30},
	{0x81, 0x50},
	{0x82, 0x80},
	{0x85, 0x1a},
	{0x88, 0x00},
	{0x89, 0x00},
	{0x90, 0x5d},

	{0xc5, 0x30},
	{0xc6, 0x2a},

	{0xD0, 0x0c},
	{0xD1, 0x80},
	{0xD2, 0x67},
	{0xD3, 0x00},
	{0xD4, 0x00},
	{0xD5, 0x02},
	{0xD6, 0xff},
	{0xD7, 0x18},
	{0x3b, 0x06},
	{0x3c, 0x06},

	{0xc5, 0x30},
	{0xc6, 0x2a},

	//Page 13
	{0x03, 0x13},
	{0x10, 0xcb},
	{0x11, 0x7b},
	{0x12, 0x07},
	{0x14, 0x00},

	{0x20, 0x15},
	{0x21, 0x13},
	{0x22, 0x33},
	{0x23, 0x04},
	{0x24, 0x09},
	{0x25, 0x08},
	{0x26, 0x18},
	{0x27, 0x30},
	{0x29, 0x12},
	{0x2a, 0x50},

	//Low clip th
	{0x2b, 0x06},
	{0x2c, 0x06},
	{0x25, 0x08},
	{0x2d, 0x0c},
	{0x2e, 0x12},
	{0x2f, 0x12},

	//Out2 Edge
	{0x50, 0x10},
	{0x51, 0x14},
	{0x52, 0x10},
	{0x53, 0x0c},
	{0x54, 0x0f},
	{0x55, 0x0c},

	//Out1 Edge
	{0x56, 0x10},
	{0x57, 0x13},
	{0x58, 0x10},
	{0x59, 0x0c},
	{0x5a, 0x0f},
	{0x5b, 0x0c},

	//Indoor Edge
	{0x5c, 0x0a},
	{0x5d, 0x0b},
	{0x5e, 0x0a},
	{0x5f, 0x08},
	{0x60, 0x09},
	{0x61, 0x08},

	//Dark1 Edge
	{0x62, 0x08},
	{0x63, 0x08},
	{0x64, 0x08},
	{0x65, 0x06},
	{0x66, 0x06},
	{0x67, 0x06},

	//Dark2 Edge
	{0x68, 0x07},
	{0x69, 0x07},
	{0x6a, 0x07},
	{0x6b, 0x05},
	{0x6c, 0x05},
	{0x6d, 0x05},

	//Dark3 Edge
	{0x6e, 0x07},
	{0x6f, 0x07},
	{0x70, 0x07},
	{0x71, 0x05},
	{0x72, 0x05},
	{0x73, 0x05},

	//2DY
	{0x80, 0xfd},
	{0x81, 0x1f},
	{0x82, 0x05},
	{0x83, 0x01},

	{0x90, 0x15},
	{0x91, 0x15},
	{0x92, 0x33},
	{0x93, 0x30},
	{0x94, 0x03},
	{0x95, 0x14},
	{0x97, 0x30},
	{0x99, 0x30},

	{0xa0, 0x04},
	{0xa1, 0x05},
	{0xa2, 0x04},
	{0xa3, 0x05},
	{0xa4, 0x07},
	{0xa5, 0x08},
	{0xa6, 0x07},
	{0xa7, 0x08},
	{0xa8, 0x07},
	{0xa9, 0x08},
	{0xaa, 0x07},
	{0xab, 0x08}, 

	//Out2 
	{0xb0, 0x22},
	{0xb1, 0x2a},
	{0xb2, 0x28},
	{0xb3, 0x22},
	{0xb4, 0x2a},
	{0xb5, 0x28},

	//Out1 
	{0xb6, 0x22},
	{0xb7, 0x2a},
	{0xb8, 0x28},
	{0xb9, 0x22},
	{0xba, 0x2a},
	{0xbb, 0x28},

	{0xbc, 0x17},
	{0xbd, 0x17},
	{0xbe, 0x17},
	{0xbf, 0x17},
	{0xc0, 0x17},
	{0xc1, 0x17},

	//Dark1
	{0xc2, 0x1e},
	{0xc3, 0x12},
	{0xc4, 0x10},
	{0xc5, 0x1e},
	{0xc6, 0x12},
	{0xc7, 0x10},

	//Dark2
	{0xc8, 0x18},
	{0xc9, 0x05},
	{0xca, 0x05},
	{0xcb, 0x18},
	{0xcc, 0x05},
	{0xcd, 0x05},

	//Dark3 
	{0xce, 0x18},
	{0xcf, 0x05},
	{0xd0, 0x05},
	{0xd1, 0x18},
	{0xd2, 0x05},
	{0xd3, 0x05},
	
	//Page 14
	{0x03, 0x14},
	{0x10, 0x11},
	{0x20, 0x40},
	{0x21, 0x80},
	{0x23, 0x80},
	{0x22, 0x80},
	{0x23, 0x80},
	{0x24, 0x80},

	{0x30, 0xc8},
	{0x31, 0x2b},
	{0x32, 0x00},
	{0x33, 0x00},
	{0x34, 0x90},

	{0x40, 0x42},
	{0x50, 0x2d},
	{0x60, 0x28},
	{0x70, 0x2d},

	//Page 15
	{0x03, 0x15}, 
	{0x10, 0x0f}, 
	{0x14, 0x52}, 
	{0x15, 0x42}, 
	{0x16, 0x32}, 
	{0x17, 0x2f}, 

	//CMC
	{0x30, 0x8f}, 
	{0x31, 0x59}, 
	{0x32, 0x0a}, 
	{0x33, 0x15}, 
	{0x34, 0x5b}, 
	{0x35, 0x06}, 
	{0x36, 0x07}, 
	{0x37, 0x40}, 
	{0x38, 0x86}, 

	//CMC OFS
	{0x40, 0x95}, 
	{0x41, 0x1f}, 
	{0x42, 0x8a}, 
	{0x43, 0x86}, 
	{0x44, 0x0a}, 
	{0x45, 0x84}, 
	{0x46, 0x87}, 
	{0x47, 0x9b}, 
	{0x48, 0x23}, 

	//CMC POFS
	{0x50, 0x8c}, 
	{0x51, 0x0c}, 
	{0x52, 0x00}, 
	{0x53, 0x07}, 
	{0x54, 0x17}, 
	{0x55, 0x9d}, 
	{0x56, 0x00}, 
	{0x57, 0x0b}, 
	{0x58, 0x89}, 

	{0x80, 0x03},
	{0x85, 0x40},
	{0x87, 0x02},
	{0x88, 0x00},
	{0x89, 0x00},
	{0x8a, 0x00},

	{0x03, 0x16}, 
	{0x10, 0x31}, 
	{0x18, 0x37},
	{0x19, 0x36},
	{0x1a, 0x0e},
	{0x1b, 0x01},
	{0x1c, 0xdc},
	{0x1d, 0xfe},
// original
	{0x30, 0x00}, 
	{0x31, 0x06}, 
	{0x32, 0x1d}, 
	{0x33, 0x33}, 
	{0x34, 0x53}, 
	{0x35, 0x6c}, 
	{0x36, 0x81}, 
	{0x37, 0x94}, 
	{0x38, 0xa4}, 
	{0x39, 0xb3}, 
	{0x3a, 0xc0}, 
	{0x3b, 0xcb}, 
	{0x3c, 0xd5}, 
	{0x3d, 0xde}, 
	{0x3e, 0xe6}, 
	{0x3f, 0xee}, 
	{0x40, 0xf5}, 
	{0x41, 0xfc}, 
	{0x42, 0xff}, 

	{0x50, 0x00}, 
	{0x51, 0x03}, 
	{0x52, 0x19}, 
	{0x53, 0x34}, 
	{0x54, 0x58}, 
	{0x55, 0x75}, 
	{0x56, 0x8d}, 
	{0x57, 0xa1}, 
	{0x58, 0xb2}, 
	{0x59, 0xbe}, 
	{0x5a, 0xc9}, 
	{0x5b, 0xd2}, 
	{0x5c, 0xdb}, 
	{0x5d, 0xe3}, 
	{0x5e, 0xeb}, 
	{0x5f, 0xf0}, 
	{0x60, 0xf5}, 
	{0x61, 0xf7}, 
	{0x62, 0xf8}, 

	{0x70, 0x00}, 
	{0x71, 0x08}, 
	{0x72, 0x17}, 
	{0x73, 0x2f}, 
	{0x74, 0x53}, 
	{0x75, 0x6c}, 
	{0x76, 0x81}, 
	{0x77, 0x94}, 
	{0x78, 0xa4}, 
	{0x79, 0xb3}, 
	{0x7a, 0xc0}, 
	{0x7b, 0xcb}, 
	{0x7c, 0xd5}, 
	{0x7d, 0xde}, 
	{0x7e, 0xe6}, 
	{0x7f, 0xee}, 
	{0x80, 0xf4}, 
	{0x81, 0xfa}, 
	{0x82, 0xff}, 
  
/*
	{0x30, 0x00}, 
	{0x31, 0x08}, 
	{0x32, 0x1f}, 
	{0x33, 0x35}, 
	{0x34, 0x55}, 
	{0x35, 0x6e}, 
	{0x36, 0x83}, 
	{0x37, 0x96}, 
	{0x38, 0xa6}, 
	{0x39, 0xb5}, 
	{0x3a, 0xc2}, 
	{0x3b, 0xcd}, 
	{0x3c, 0xd7}, 
	{0x3d, 0xe0}, 
	{0x3e, 0xe8}, 
	{0x3f, 0xf0}, 
	{0x40, 0xf7}, 
	{0x41, 0xfe}, 
	{0x42, 0xff}, 

	{0x50, 0x00}, 
	{0x51, 0x05}, 
	{0x52, 0x1b}, 
	{0x53, 0x36}, 
	{0x54, 0x5a}, 
	{0x55, 0x77}, 
	{0x56, 0x8f}, 
	{0x57, 0xa3}, 
	{0x58, 0xb4}, 
	{0x59, 0xc0}, 
	{0x5a, 0xcb}, 
	{0x5b, 0xd4}, 
	{0x5c, 0xde}, 
	{0x5d, 0xe5}, 
	{0x5e, 0xed}, 
	{0x5f, 0xf2}, 
	{0x60, 0xf7}, 
	{0x61, 0xf9}, 
	{0x62, 0xfa}, 

	{0x70, 0x00}, 
	{0x71, 0x0a}, 
	{0x72, 0x19}, 
	{0x73, 0x31}, 
	{0x74, 0x55}, 
	{0x75, 0x6e}, 
	{0x76, 0x83}, 
	{0x77, 0x96}, 
	{0x78, 0xa6}, 
	{0x79, 0xb5}, 
	{0x7a, 0xc2}, 
	{0x7b, 0xcd}, 
	{0x7c, 0xd7}, 
	{0x7d, 0xe0}, 
	{0x7e, 0xe8}, 
	{0x7f, 0xf0}, 
	{0x80, 0xf6}, 
	{0x81, 0xfc}, 
	{0x82, 0xff}, 
*/
	
	{0x03, 0x17}, 
	{0xc4, 0x6e}, 
	{0xc5, 0x5c}, 

	{0x03, 0x20}, 
	{0x10, 0x1c},
	{0x18, 0x38},
	{0x20, 0x01}, 
	{0x21, 0x30},
	{0x22, 0x10},
	{0x23, 0x00},
	{0x24, 0x04},

	{0x28, 0xff},
	{0x29, 0xad},

	{0x2a, 0xf0},
	{0x2b, 0x34},
	{0x30, 0x78},
	{0x2c, 0xc3},
	{0x2d, 0x5f},
	{0x2e, 0x33},
	//{0x30, 0xf8},
	{0x32, 0x03},
	{0x33, 0x2e},
	{0x34, 0x30},
	{0x35, 0xd4},
	{0x36, 0xfe},
	{0x37, 0x32},
	{0x38, 0x04},
	{0x47, 0xf0},

	//Y_Frame TH
	{0x50, 0x45},
	{0x51, 0x88},

	{0x56, 0x10},
	{0x57, 0xb7},
	{0x58, 0x14},
	{0x59, 0x88},
	{0x5a, 0x04},

	{0x60, 0x55}, 
	{0x61, 0x55}, 
	{0x62, 0x6a}, 
	{0x63, 0xa9}, 
	{0x64, 0x6a}, 
	{0x65, 0xa9}, 
	{0x66, 0x6a}, 
	{0x67, 0xa9}, 
	{0x68, 0x6b}, 
	{0x69, 0xe9}, 
	{0x6a, 0x6a}, 
	{0x6b, 0xa9}, 
	{0x6c, 0x6a}, 
	{0x6d, 0xa9}, 
	{0x6e, 0x55}, 
	{0x6f, 0x55}, 
	{0x70, 0x3a},             //42
	{0x71, 0xBb},

	// haunting control
	{0x76, 0x21},
	{0x77, 0x02},
	{0x78, 0x22},
	{0x79, 0x2a},

	{0x78, 0x24},
	{0x79, 0x23},
	{0x7a, 0x23},
	{0x7b, 0x22},
	{0x7d, 0x23},
	{0x83, 0x01},
	{0x84, 0x5f},
	{0x85, 0x00},
	{0x86, 0x02},
	{0x87, 0x00},
	{0x88, 0x05},
	{0x89, 0x7c},
	{0x8a, 0x00},
	{0x8B, 0x75},
	{0x8C, 0x00},
	{0x8D, 0x61},
	{0x8E, 0x00},

	{0x98, 0xdc},
	{0x99, 0x45},
	{0x9a, 0x0d},
	{0x9b, 0xde},
	{0x9c, 0x0e},
	{0x9d, 0x0a},
	{0x9e, 0x02},
	{0x9f, 0x00},
    {0x10, 0x9c},
    {0x18, 0x30},
    {0x90, 0x0c},
    {0x91, 0x0c},
    {0x92, 0xd8},   
    {0x93, 0xd0},  
    
	{0x9f, 0x26}, 
	{0xa0, 0x03},
	{0xa1, 0xa9},
	{0xa2, 0x80},
	{0xb0, 0x1d},
	{0xb1, 0x1a},
	{0xb2, 0x60},
	{0xb3, 0x1a},
	{0xb4, 0x1a},
	{0xb5, 0x44},
	{0xb6, 0x2f},
	{0xb7, 0x28},
	{0xb8, 0x25},
	{0xb9, 0x22},
	{0xba, 0x21},
	{0xbb, 0x20},
	{0xbc, 0x1f},
	{0xbd, 0x1f},
	{0xc0, 0x30},
	{0xc1, 0x20},
	{0xc2, 0x20},
	{0xc3, 0x20},
	{0xc4, 0x08},
	{0xc8, 0x60},
	{0xc9, 0x40},
	
	//Page 22
	{0x03, 0x22},
	{0x10, 0x69},//lxh
	{0x11, 0x2c},
	{0x19, 0x01},
	{0x20, 0x30},
	{0x21, 0x80},
	{0x23, 0x08},
	{0x24, 0x01},

	{0x30, 0x80},
	{0x31, 0x80},
	{0x38, 0x11},
	{0x39, 0x34},
	{0x40, 0xf7},

	{0x41, 0x77},
	{0x42, 0x55},
	{0x43, 0xf0},
	{0x44, 0x43}, 
	{0x45, 0x33},
	{0x46, 0x00}, 

	{0x47, 0x94},

	{0x50, 0xb2},
	{0x51, 0x81},
	{0x52, 0x98},

	{0x80, 0x3d},//lxh 
	{0x81, 0x20}, 
	{0x82, 0x32},//lxh 

	{0x83, 0x50}, 
	{0x84, 0x20}, 
	{0x85, 0x50}, 
	{0x86, 0x20}, 

	{0x87, 0x54}, 
	{0x88, 0x20}, 
	{0x89, 0x45}, 
	{0x8a, 0x2a}, 

	{0x8b, 0x46}, 
	{0x8c, 0x3f}, 
	{0x8d, 0x34}, 
	{0x8e, 0x2c}, 

	{0x8f, 0x60}, 
	{0x90, 0x5f}, 
	{0x91, 0x5c}, 
	{0x92, 0x4C}, 
	{0x93, 0x41}, 
	{0x94, 0x3b}, 
	{0x95, 0x36}, 
	{0x96, 0x30}, 
	{0x97, 0x27}, 
	{0x98, 0x20}, 
	{0x99, 0x1C}, 
	{0x9a, 0x19}, 

	{0x9b, 0x88}, 
	{0x9c, 0x88}, 
	{0x9d, 0x48}, 
	{0x9e, 0x38}, 
	{0x9f, 0x30}, 

	{0xa0, 0x74}, 
	{0xa1, 0x35}, 
	{0xa2, 0xaf}, 
	{0xa3, 0xf7}, 

	{0xa4, 0x10}, 
	{0xa5, 0x50}, 
	{0xa6, 0xc4}, 

	{0xad, 0x40},
	{0xae, 0x4a},

	{0xaf, 0x2a},
	{0xb0, 0x29},

	{0xb1, 0x20},
	{0xb4, 0xff},
	{0xb8, 0x6b},
	{0xb9, 0x00},

	{0x03, 0x24}, 
	{0x10, 0x01}, 
	{0x18, 0x06},
	{0x30, 0x06},
	{0x31, 0x90},
	{0x32, 0x25},
	{0x33, 0xa2},
	{0x34, 0x26},
	{0x35, 0x58},
	{0x36, 0x60},
	{0x37, 0x00},
	{0x38, 0x50},
	{0x39, 0x00},

	{0x03, 0x20}, 
	{0x10, 0x9c}, 
	{0x03, 0x22}, 
	{0x10, 0xe9}, 
	
	//Page 00
	{0x03, 0x00}, 
	{0x0e, 0x03}, 
	{0x0e, 0x73}, 

	{0x03, 0x00}, 
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00},
	{0x03, 0x00}, 

	{0x03, 0x00}, 
	{0x01, 0xf8},

	{END_REG, END_REG},

};


/* 1600X1200 UXGA capture */
static struct reginfo sensor_uxga[] =
{
	{0x03, 0x00},
	//{0x11, 0xa0}, 

	{0x20, 0x00}, 
	{0x21, 0x0a}, 
	{0x22, 0x00}, 
	{0x23, 0x0a}, 

	{0x03, 0x10},
	{0x3f, 0x00},

	//Page12
	{0x03, 0x12},
	{0x20, 0x0f},
	{0x21, 0x0f},
	{0x90, 0x5d},

	//Page13
	{0x03, 0x13},
	{0x80, 0xfd},

	// 1600*1200	
	{0x03,0x00},
	{0x10,0x00},
	
	{END_REG, END_REG},
};

/* 1280X1024 SXGA */
static struct reginfo sensor_sxga[] =
{
	{END_REG, END_REG},
};
static struct reginfo sensor_xga[] =
{
	{END_REG, END_REG},
};
/* 800X600 SVGA,30fps*/
static struct reginfo sensor_svga[] =
{
#if 1
	{0x03, 0x10},
	{0x3f, 0x00},
	

	//Page12
	{0x03, 0x12}, //Function
	{0x20, 0x0f},
	{0x21, 0x0f},
	{0x90, 0x5d},  

	//Page13
	{0x03, 0x13}, //Function
	{0x80, 0xfd}, //Function

	// 800*600	
	{0x03,0x00},
	{0x10,0x91},//11
	{0x20, 0x00},
	{0x21, 0x04},
	{0x22, 0x00},
	{0x23, 0x07},
	{0x24, 0x04},
	{0x25, 0xb0},
	{0x26, 0x06},
	{0x27, 0x40},
	
	{0x03, 0x20},
	{0x8b, 0x75},
	{0x8c, 0x00},
	{0x8d, 0x61},
	{0x8e, 0x00},
	
	{0x03, 0x20},
	{0x03, 0x20}, 
#endif
	{END_REG, END_REG},
};

/* 640X480 VGA */
static struct reginfo sensor_vga[] =
{
	{END_REG, END_REG},
};

/* 352X288 CIF */
static struct reginfo sensor_cif[] =
{
	{END_REG, END_REG},
};

/* 320*240 QVGA */
static  struct reginfo sensor_qvga[] =
{
	{END_REG, END_REG},
};

/* 176X144 QCIF*/
static struct reginfo sensor_qcif[] =
{
	{END_REG, END_REG},
};

#endif
static  struct reginfo sensor_ClrFmt_YUYV[]=
{

    //{0x00, 0x00}
	{END_REG, END_REG},
};

static  struct reginfo sensor_ClrFmt_UYVY[]=
{

    //{0x00, 0x00}
	{END_REG, END_REG},
};

#if CONFIG_SENSOR_WhiteBalance
static  struct reginfo sensor_WhiteB_Auto[]=
{
	{0x03, 0x22},
	{0x10, 0x69},
	{0x80, 0x3d},
	{0x81, 0x20},
	{0x82, 0x32},
	{0x83, 0x50},
	{0x84, 0x20},
	{0x85, 0x50},
	{0x86, 0x20},
	{0x10, 0xe9},	
	{END_REG, END_REG},
};
/* Cloudy Colour Temperature : 6500K - 8000K  */
static  struct reginfo sensor_WhiteB_Cloudy[]=
{
	//Sunny
	{0x03, 0x22},
	{0x10, 0x69},
	{0x80, 0x4e},
	{0x81, 0x20}, //20
	{0x82, 0x24},  //27
	{0x83, 0x47},
	{0x84, 0x47},
	{0x85, 0x24},
	{0x86, 0x24},
	{END_REG, END_REG},
};
/* ClearDay Colour Temperature : 5000K - 6500K  */
static  struct reginfo sensor_WhiteB_ClearDay[]=
{
    //Sunny
	{0x03, 0x22},
	{0x10, 0x69},
	{0x80, 0x47},
	{0x81, 0x20}, //20
	{0x82, 0x24},  //27
	{0x83, 0x47},
	{0x84, 0x47},
	{0x85, 0x24},
	{0x86, 0x24},
	{END_REG, END_REG},
};
/* Office Colour Temperature : 3500K - 5000K  */
static  struct reginfo sensor_WhiteB_TungstenLamp1[]=
{
    //incandescense
  	{0x03, 0x22},
	{0x10, 0x69},
	{0x80, 0x1e},
	{0x81, 0x1c},
	{0x82, 0x50},
	{0x83, 0x22},
	{0x84, 0x22},
	{0x85, 0x50},
	{0x86, 0x50},
	{END_REG, END_REG},

};
/* Home Colour Temperature : 2500K - 3500K  */
static  struct reginfo sensor_WhiteB_TungstenLamp2[]=
{
    //Home
	{0x03, 0x22},
	{0x10, 0x69},
	{0x80, 0x2a},
	{0x81, 0x20},
	{0x82, 0x42},
	{0x83, 0x2a},
	{0x84, 0x2a},
	{0x85, 0x42},
	{0x86, 0x42},
	{END_REG, END_REG},
};
static struct reginfo *sensor_WhiteBalanceSeqe[] = {sensor_WhiteB_Auto, sensor_WhiteB_TungstenLamp1,sensor_WhiteB_TungstenLamp2,
    sensor_WhiteB_ClearDay, sensor_WhiteB_Cloudy,NULL,
};
#endif

#if CONFIG_SENSOR_Brightness
static  struct reginfo sensor_Brightness0[]=
{
    // Brightness -2
    {0x03, 0x10},
    {0x40, 0xa0},
	{END_REG, END_REG},
};

static  struct reginfo sensor_Brightness1[]=
{
    // Brightness -1
	{0x03, 0x10},
    {0x40, 0x90},
	{END_REG, END_REG},
};

static  struct reginfo sensor_Brightness2[]=
{
    //  Brightness 0
	{0x03, 0x10},
    {0x40, 0x00},
	{END_REG, END_REG},
};

static  struct reginfo sensor_Brightness3[]=
{
    // Brightness +1
	{0x03, 0x10},
    {0x40, 0x10},
	{END_REG, END_REG},
};

static  struct reginfo sensor_Brightness4[]=
{
    //  Brightness +2
	{0x03, 0x10},
    {0x40, 0x20},
	{END_REG, END_REG},
};

static  struct reginfo sensor_Brightness5[]=
{
    //  Brightness +3
    {0x03, 0x10},
    {0x40, 0x30},
	{END_REG, END_REG},
};
static struct reginfo *sensor_BrightnessSeqe[] = {sensor_Brightness0, sensor_Brightness1, sensor_Brightness2, sensor_Brightness3,
    sensor_Brightness4, sensor_Brightness5,NULL,
};

#endif

#if CONFIG_SENSOR_Effect
static  struct reginfo sensor_Effect_Normal[] =
{
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x00},
	{0x13, 0x00},	
	{END_REG, END_REG},
};

static  struct reginfo sensor_Effect_WandB[] =
{
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x03},
	{0x13, 0x02},
	{0x44, 0x80},
	{0x45, 0x80},
	{0x47, 0x7f},	
	{END_REG, END_REG},
};

static  struct reginfo sensor_Effect_Sepia[] =
{
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x23},
	{0x13, 0x00},
	{0x44, 0x70},
	{0x45, 0x98},
	{0x47, 0x7f},	
	{END_REG, END_REG},
};

static  struct reginfo sensor_Effect_Negative[] =
{
    //Negative
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x08},
	{0x13, 0x02},
	{0x14, 0x00},	
	{0x44, 0x80},
	{0x45, 0x80},
	{0x47, 0x7f},	
	{END_REG, END_REG},
};
static  struct reginfo sensor_Effect_Bluish[] =
{
    // Bluish
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x03},
	{0x13, 0x02},
	{0x44, 0xb0},
	{0x45, 0x40},
	{0x47, 0x7f},	
	{END_REG, END_REG},
};

static  struct reginfo sensor_Effect_Green[] =
{
    //  Greenish
    {0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x03},
	{0x13, 0x02},
	{0x44, 0x30},
	{0x45, 0x50},
	{0x47, 0x7f},	
	{END_REG, END_REG},
};
static struct reginfo *sensor_EffectSeqe[] = {sensor_Effect_Normal, sensor_Effect_WandB, sensor_Effect_Negative,sensor_Effect_Sepia,
    sensor_Effect_Bluish, sensor_Effect_Green,NULL,
};
#endif
#if CONFIG_SENSOR_Exposure
static  struct reginfo sensor_Exposure0[]=
{
    //-3
    	{0x03,0x20},
    {0x70, 0x20},
    {END_REG, END_REG},

};

static  struct reginfo sensor_Exposure1[]=
{
    //-2
{0x03,0x20},
    {0x70, 0x28},
  	{END_REG, END_REG},
};

static  struct reginfo sensor_Exposure2[]=
{
    //-0.3EV
    {0x03,0x20},
    {0x70, 0x30},
	{END_REG, END_REG},
};

static  struct reginfo sensor_Exposure3[]=
{
    //default
    {0x03,0x20},
    {0x70, 0x3a},
	{END_REG, END_REG},
};

static  struct reginfo sensor_Exposure4[]=
{
    // 1
	{0x03,0x20},
    {0x70, 0x40},
	{END_REG, END_REG},
};

static  struct reginfo sensor_Exposure5[]=
{
    // 2
    {0x03,0x20},
    {0x70, 0x48},
	{END_REG, END_REG},
};

static  struct reginfo sensor_Exposure6[]=
{
    // 3
	{0x03,0x20},
    {0x70, 0x52},
	{END_REG, END_REG},
};

static struct reginfo *sensor_ExposureSeqe[] = {sensor_Exposure0, sensor_Exposure1, sensor_Exposure2, sensor_Exposure3,
    sensor_Exposure4, sensor_Exposure5,sensor_Exposure6,NULL,
};
#endif
#if CONFIG_SENSOR_Saturation
static  struct reginfo sensor_Saturation0[]=
{

	{END_REG, END_REG},
};

static  struct reginfo sensor_Saturation1[]=
{

	{END_REG, END_REG},
};

static  struct reginfo sensor_Saturation2[]=
{
	{END_REG, END_REG},
};
static struct reginfo *sensor_SaturationSeqe[] = {sensor_Saturation0, sensor_Saturation1, sensor_Saturation2, NULL,};

#endif
#if CONFIG_SENSOR_Contrast
static  struct reginfo sensor_Contrast0[]=
{
    //Contrast -3
  
	{END_REG, END_REG},
};

static  struct reginfo sensor_Contrast1[]=
{
    //Contrast -2

	{END_REG, END_REG},
};

static  struct reginfo sensor_Contrast2[]=
{
    // Contrast -1

 	{END_REG, END_REG},
};

static  struct reginfo sensor_Contrast3[]=
{
    //Contrast 0

 	{END_REG, END_REG},
};

static  struct reginfo sensor_Contrast4[]=
{
    //Contrast +1

	{END_REG, END_REG},
};


static  struct reginfo sensor_Contrast5[]=
{
    //Contrast +2

	{END_REG, END_REG},
};

static  struct reginfo sensor_Contrast6[]=
{
    //Contrast +3

	{END_REG, END_REG},
};
static struct reginfo *sensor_ContrastSeqe[] = {sensor_Contrast0, sensor_Contrast1, sensor_Contrast2, sensor_Contrast3,
    sensor_Contrast4, sensor_Contrast5, sensor_Contrast6, NULL,
};

#endif
#if CONFIG_SENSOR_Mirror
static  struct reginfo sensor_MirrorOn[]=
{

	{END_REG, END_REG},
};

static  struct reginfo sensor_MirrorOff[]=
{

	{END_REG, END_REG},
};
static struct reginfo *sensor_MirrorSeqe[] = {sensor_MirrorOff, sensor_MirrorOn,NULL,};
#endif
#if CONFIG_SENSOR_Flip
static  struct reginfo sensor_FlipOn[]=
{

	{END_REG, END_REG},
};

static  struct reginfo sensor_FlipOff[]=
{

	{END_REG, END_REG},
};
static struct reginfo *sensor_FlipSeqe[] = {sensor_FlipOff, sensor_FlipOn,NULL,};

#endif
#if CONFIG_SENSOR_Scene
static  struct reginfo sensor_SceneAuto[] =
{
	{0x03, 0x00}, 
	{0x12,0x04},
	{END_REG, END_REG},	
};

static  struct reginfo sensor_SceneNight[] =
{
	{0x03, 0x00}, 
	{0x12,0x05},
	{END_REG, END_REG},
};
static struct reginfo *sensor_SceneSeqe[] = {sensor_SceneAuto, sensor_SceneNight,NULL,};

#endif
#if CONFIG_SENSOR_DigitalZoom
static struct reginfo sensor_Zoom0[] =
{
	{END_REG, END_REG},
};

static struct reginfo sensor_Zoom1[] =
{
	{END_REG, END_REG},
};

static struct reginfo sensor_Zoom2[] =
{
	{END_REG, END_REG},
};


static struct reginfo sensor_Zoom3[] =
{
	{END_REG, END_REG},
};
static struct reginfo *sensor_ZoomSeqe[] = {sensor_Zoom0, sensor_Zoom1, sensor_Zoom2, sensor_Zoom3, NULL,};
#endif
static const struct v4l2_querymenu sensor_menus[] =
{
	#if CONFIG_SENSOR_WhiteBalance
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 0,  .name = "auto",  .reserved = 0, }, {  .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 1, .name = "incandescent",  .reserved = 0,},
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 2,  .name = "fluorescent", .reserved = 0,}, {  .id = V4L2_CID_DO_WHITE_BALANCE, .index = 3,  .name = "daylight", .reserved = 0,},
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 4,  .name = "cloudy-daylight", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Effect
    { .id = V4L2_CID_EFFECT,  .index = 0,  .name = "none",  .reserved = 0, }, {  .id = V4L2_CID_EFFECT,  .index = 1, .name = "mono",  .reserved = 0,},
    { .id = V4L2_CID_EFFECT,  .index = 2,  .name = "negative", .reserved = 0,}, {  .id = V4L2_CID_EFFECT, .index = 3,  .name = "sepia", .reserved = 0,},
    { .id = V4L2_CID_EFFECT,  .index = 4, .name = "posterize", .reserved = 0,} ,{ .id = V4L2_CID_EFFECT,  .index = 5,  .name = "aqua", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Scene
    { .id = V4L2_CID_SCENE,  .index = 0, .name = "auto", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,  .index = 1,  .name = "night", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Flash
    { .id = V4L2_CID_FLASH,  .index = 0,  .name = "off",  .reserved = 0, }, {  .id = V4L2_CID_FLASH,  .index = 1, .name = "auto",  .reserved = 0,},
    { .id = V4L2_CID_FLASH,  .index = 2,  .name = "on", .reserved = 0,}, {  .id = V4L2_CID_FLASH, .index = 3,  .name = "torch", .reserved = 0,},
    #endif
};

static  struct v4l2_queryctrl sensor_controls[] =
{
	#if CONFIG_SENSOR_WhiteBalance
    {
        .id		= V4L2_CID_DO_WHITE_BALANCE,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "White Balance Control",
        .minimum	= 0,
        .maximum	= 4,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Brightness
	{
        .id		= V4L2_CID_BRIGHTNESS,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Brightness Control",
        .minimum	= -3,
        .maximum	= 2,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Effect
	{
        .id		= V4L2_CID_EFFECT,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Effect Control",
        .minimum	= 0,
        .maximum	= 5,
        .step		= 1,
        .default_value = 0,
    },
	#endif

	#if CONFIG_SENSOR_Exposure
	{
        .id		= V4L2_CID_EXPOSURE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Exposure Control",
        .minimum	= 0,
        .maximum	= 6,
        .step		= 1,
        .default_value = 3,
    },
	#endif

	#if CONFIG_SENSOR_Saturation
	{
        .id		= V4L2_CID_SATURATION,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Saturation Control",
        .minimum	= 0,
        .maximum	= 2,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Contrast
	{
        .id		= V4L2_CID_CONTRAST,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Contrast Control",
        .minimum	= -3,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    },
	#endif

	#if CONFIG_SENSOR_Mirror
	{
        .id		= V4L2_CID_HFLIP,
        .type		= V4L2_CTRL_TYPE_BOOLEAN,
        .name		= "Mirror Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 1,
    },
    #endif

	#if CONFIG_SENSOR_Flip
	{
        .id		= V4L2_CID_VFLIP,
        .type		= V4L2_CTRL_TYPE_BOOLEAN,
        .name		= "Flip Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 1,
    },
    #endif

	#if CONFIG_SENSOR_Scene
    {
        .id		= V4L2_CID_SCENE,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Scene Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_DigitalZoom
    {
        .id		= V4L2_CID_ZOOM_RELATIVE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "DigitalZoom Control",
        .minimum	= -1,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_ZOOM_ABSOLUTE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "DigitalZoom Control",
        .minimum	= 0,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Focus
	{
        .id		= V4L2_CID_FOCUS_RELATIVE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Focus Control",
        .minimum	= -1,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_FOCUS_ABSOLUTE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Focus Control",
        .minimum	= 0,
        .maximum	= 255,
        .step		= 1,
        .default_value = 125,
    },
    #endif

	#if CONFIG_SENSOR_Flash
	{
        .id		= V4L2_CID_FLASH,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Flash Control",
        .minimum	= 0,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    },
	#endif
};

static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *did);
static int sensor_video_probe(struct soc_camera_device *icd, struct i2c_client *client);
static int sensor_g_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
static int sensor_s_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
static int sensor_g_ext_controls(struct v4l2_subdev *sd,  struct v4l2_ext_controls *ext_ctrl);
static int sensor_s_ext_controls(struct v4l2_subdev *sd,  struct v4l2_ext_controls *ext_ctrl);
static int sensor_suspend(struct soc_camera_device *icd, pm_message_t pm_msg);
static int sensor_resume(struct soc_camera_device *icd);
static int sensor_set_bus_param(struct soc_camera_device *icd,unsigned long flags);
static unsigned long sensor_query_bus_param(struct soc_camera_device *icd);
#if CONFIG_SENSOR_Effect
static int sensor_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
#endif
#if CONFIG_SENSOR_WhiteBalance
static int sensor_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
#endif
static int sensor_deactivate(struct i2c_client *client);

static struct soc_camera_ops sensor_ops =
{
    .suspend                     = sensor_suspend,
    .resume                       = sensor_resume,
    .set_bus_param		= sensor_set_bus_param,
    .query_bus_param	= sensor_query_bus_param,
    .controls		= sensor_controls,
    .menus                         = sensor_menus,
    .num_controls		= ARRAY_SIZE(sensor_controls),
    .num_menus		= ARRAY_SIZE(sensor_menus),
};

/* only one fixed colorspace per pixelcode */
struct sensor_datafmt {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

/* Find a data format by a pixel code in an array */
static const struct sensor_datafmt *sensor_find_datafmt(
	enum v4l2_mbus_pixelcode code, const struct sensor_datafmt *fmt,
	int n)
{
	int i;
	for (i = 0; i < n; i++)
		if (fmt[i].code == code)
			return fmt + i;

	return NULL;
}

static const struct sensor_datafmt sensor_colour_fmts[] = {
    {V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
    {V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG}	
};

typedef struct sensor_info_priv_s
{
    int whiteBalance;
    int brightness;
    int contrast;
    int saturation;
    int effect;
    int scene;
    int digitalzoom;
    int focus;
    int flash;
    int exposure;
	bool snap2preview;
	bool video2preview;
    unsigned char mirror;                                        /* HFLIP */
    unsigned char flip;                                          /* VFLIP */
    unsigned int winseqe_cur_addr;
    struct sensor_datafmt fmt;
    unsigned int funmodule_state;

} sensor_info_priv_t;

struct sensor
{
    struct v4l2_subdev subdev;
    struct i2c_client *client;
    sensor_info_priv_t info_priv;
    int model;	/* V4L2_IDENT_OV* codes from v4l2-chip-ident.h */
#if CONFIG_SENSOR_I2C_NOSCHED
	atomic_t tasklock_cnt;
#endif
	struct rk29camera_platform_data *sensor_io_request;
    struct rk29camera_gpio_res *sensor_gpio_res;
};

static struct sensor* to_sensor(const struct i2c_client *client)
{
    return container_of(i2c_get_clientdata(client), struct sensor, subdev);
}

static int sensor_task_lock(struct i2c_client *client, int lock)
{
#if CONFIG_SENSOR_I2C_NOSCHED
	int cnt = 3;
    struct sensor *sensor = to_sensor(client);

	if (lock) {
		if (atomic_read(&sensor->tasklock_cnt) == 0) {
			while ((atomic_read(&client->adapter->bus_lock.count) < 1) && (cnt>0)) {
				SENSOR_TR("\n %s will obtain i2c in atomic, but i2c bus is locked! Wait...\n",SENSOR_NAME_STRING());
				msleep(35);
				cnt--;
			}
			if ((atomic_read(&client->adapter->bus_lock.count) < 1) && (cnt<=0)) {
				SENSOR_TR("\n %s obtain i2c fail in atomic!!\n",SENSOR_NAME_STRING());
				goto sensor_task_lock_err;
			}
			preempt_disable();
		}

		atomic_add(1, &sensor->tasklock_cnt);
	} else {
		if (atomic_read(&sensor->tasklock_cnt) > 0) {
			atomic_sub(1, &sensor->tasklock_cnt);

			if (atomic_read(&sensor->tasklock_cnt) == 0)
				preempt_enable();
		}
	}
	return 0;
sensor_task_lock_err:
	return -1;  
#else
    return 0;
#endif
}

static int sensor_write(struct i2c_client *client, u8 reg, u8 val)
{
    int err,cnt;
    u8 buf[2];
    struct i2c_msg msg[1];

    buf[0] = reg;
    buf[1] = val;
	
    msg->addr = client->addr;
    msg->flags = client->flags;
    msg->buf = buf;
    msg->len = sizeof(buf);
    msg->scl_rate = CONFIG_SENSOR_I2C_SPEED;         /* ddl@rock-chips.com : 100kHz */
    msg->read_type = 0;               /* fpga i2c:0==I2C_NORMAL : direct use number not enum for don't want include spi_fpga.h */

    cnt = 3;
    err = -EAGAIN;

    while ((cnt-- > 0) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */
        err = i2c_transfer(client->adapter, msg, 1);

        if (err >= 0) {
            return 0;
        } else {
        	SENSOR_TR("\n %s write reg(0x%x, val:0x%x) failed, try to write again!\n",SENSOR_NAME_STRING(),reg, val);
            udelay(10);
        }
    }

    return err;
}

/* sensor register read */
static int sensor_read(struct i2c_client *client, u8 reg, u8 *val)
{
    int err,cnt;
    u8 buf[1];
    struct i2c_msg msg[2];

    buf[0] = reg ;

    msg[0].addr = client->addr;
    msg[0].flags = client->flags;
    msg[0].buf = buf;
    msg[0].len = sizeof(buf);
    msg[0].scl_rate = CONFIG_SENSOR_I2C_SPEED;       /* ddl@rock-chips.com : 100kHz */
    msg[0].read_type = 2;   /* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

    msg[1].addr = client->addr;
    msg[1].flags = client->flags|I2C_M_RD;
    msg[1].buf = buf;
    msg[1].len = 1;
    msg[1].scl_rate = CONFIG_SENSOR_I2C_SPEED;                       /* ddl@rock-chips.com : 100kHz */
    msg[1].read_type = 2;                             /* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

    cnt = 3;
    err = -EAGAIN;
    while ((cnt-- > 0) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */
        err = i2c_transfer(client->adapter, msg, 2);

        if (err >= 0) {
            *val = buf[0];
            return 0;
        } else {
        	SENSOR_TR("\n %s read reg(0x%x val:0x%x) failed, try to read again! \n",SENSOR_NAME_STRING(),reg, *val);
            udelay(10);
        }
    }

    return err;
}


/* write a array of registers  */
static int sensor_write_array(struct i2c_client *client, struct reginfo *regarray)
{
    int err = 0, cnt;
    int i = 0;
#if CONFIG_SENSOR_I2C_RDWRCHK    
	char valchk;
#endif

	cnt = 0;
	if (sensor_task_lock(client, 1) < 0)
		goto sensor_write_array_end;
    while (regarray[i].reg != END_REG)
    {
        err = sensor_write(client, regarray[i].reg, regarray[i].val);
        if (err < 0)
        {
            if (cnt-- > 0) {
			    SENSOR_TR("%s..write failed current reg:0x%x, Write array again !\n", SENSOR_NAME_STRING(),regarray[i].reg);
				i = 0;
				continue;
            } else {
                SENSOR_TR("%s..write array failed!!!\n", SENSOR_NAME_STRING());
                err = -EPERM;
				goto sensor_write_array_end;
            }
        } else {
        #if CONFIG_SENSOR_I2C_RDWRCHK
			//mdelay(5);
			sensor_read(client, regarray[i].reg, &valchk);
			if (valchk != regarray[i].val)
				SENSOR_TR("%s Reg:0x%x write(0x%x, 0x%x) fail\n",SENSOR_NAME_STRING(), regarray[i].reg, regarray[i].val, valchk);
		#endif
        }
        i++;
    }

sensor_write_array_end:
	sensor_task_lock(client,0);
	return err;
}
#if CONFIG_SENSOR_I2C_RDWRCHK
static int sensor_readchk_array(struct i2c_client *client, struct reginfo *regarray)
{
    int cnt;
    int i = 0;
	char valchk;

	cnt = 0;
	valchk = 0;
    while (regarray[i].reg != 0)
    {
		sensor_read(client, regarray[i].reg, &valchk);
		if (valchk != regarray[i].val)
			SENSOR_TR("%s Reg:0x%x read(0x%x, 0x%x) error\n",SENSOR_NAME_STRING(), regarray[i].reg, regarray[i].val, valchk);

        i++;
    }
    return 0;
}
#endif
static int sensor_ioctrl(struct soc_camera_device *icd,enum rk29sensor_power_cmd cmd, int on)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	int ret = 0;

    SENSOR_DG("%s %s  cmd(%d) on(%d)\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd,on);
	switch (cmd)
	{
		case Sensor_PowerDown:
		{
			if (icl->powerdown) {
				ret = icl->powerdown(icd->pdev, on);
				if (ret == RK29_CAM_IO_SUCCESS) {
					if (on == 0) {
						mdelay(2);
						if (icl->reset)
							icl->reset(icd->pdev);
					}
				} else if (ret == RK29_CAM_EIO_REQUESTFAIL) {
					ret = -ENODEV;
					goto sensor_power_end;
				}
			}
			break;
		}
		case Sensor_Flash:
		{
			struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    		struct sensor *sensor = to_sensor(client);

			if (sensor->sensor_io_request && sensor->sensor_io_request->sensor_ioctrl) {
				sensor->sensor_io_request->sensor_ioctrl(icd->pdev,Cam_Flash, on);
                if(on){
                    //flash off after 2 secs
            		hrtimer_cancel(&(flash_off_timer.timer));
            		hrtimer_start(&(flash_off_timer.timer),ktime_set(0, 800*1000*1000),HRTIMER_MODE_REL);
                    }
			}
            break;
		}
		default:
		{
			SENSOR_TR("%s %s cmd(0x%x) is unknown!",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			break;
		}
	}
sensor_power_end:
	return ret;
}

static enum hrtimer_restart flash_off_func(struct hrtimer *timer){
	struct flash_timer *fps_timer = container_of(timer, struct flash_timer, timer);
    sensor_ioctrl(fps_timer->icd,Sensor_Flash,0);
	SENSOR_DG("%s %s !!!!!!",SENSOR_NAME_STRING(),__FUNCTION__);
    return 0;
    
}
static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl;
    const struct sensor_datafmt *fmt;    
    char value;
    int ret;

    SENSOR_DG("\n%s..%s.. \n",SENSOR_NAME_STRING(),__FUNCTION__);

	if (sensor_ioctrl(icd, Sensor_PowerDown, 0) < 0) {
		ret = -ENODEV;
		goto sensor_INIT_ERR;
	}

    /* soft reset */
	if (sensor_task_lock(client,1)<0)
		goto sensor_INIT_ERR;
#if 1	
    ret = sensor_write(client, 0x01, 0xF9);
    if (ret != 0)
    {
        SENSOR_TR("%s soft reset sensor failed\n",SENSOR_NAME_STRING());
        ret = -ENODEV;
		goto sensor_INIT_ERR;
    }

    mdelay(5);  //delay 5 microseconds
	/* check if it is an sensor sensor */
    ret = sensor_read(client, 0x04, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id high byte failed\n");
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }

   
    SENSOR_TR("\n %s() %s  pid = 0x%x\n",__func__, SENSOR_NAME_STRING(), value);
    if (value == SENSOR_ID) {
        sensor->model = SENSOR_V4L2_IDENT;
    } else {
        SENSOR_TR("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), value);
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }
#endif
    ret = sensor_write_array(client, sensor_init_data);
    if (ret != 0)
    {
        SENSOR_TR("error: %s initial failed\n",SENSOR_NAME_STRING());
        goto sensor_INIT_ERR;
    }
	sensor_task_lock(client,0);
    sensor->info_priv.winseqe_cur_addr  = (int)SENSOR_INIT_WINSEQADR;
    fmt = sensor_find_datafmt(SENSOR_INIT_PIXFMT,sensor_colour_fmts, ARRAY_SIZE(sensor_colour_fmts));
    if (!fmt) {
        SENSOR_TR("error: %s initial array colour fmts is not support!!",SENSOR_NAME_STRING());
        ret = -EINVAL;
        goto sensor_INIT_ERR;
    }
	sensor->info_priv.fmt = *fmt;

    /* sensor sensor information for initialization  */
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
	if (qctrl)
    	sensor->info_priv.whiteBalance = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_BRIGHTNESS);
	if (qctrl)
    	sensor->info_priv.brightness = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
	if (qctrl)
    	sensor->info_priv.effect = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EXPOSURE);
	if (qctrl)
        sensor->info_priv.exposure = qctrl->default_value;

	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SATURATION);
	if (qctrl)
        sensor->info_priv.saturation = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_CONTRAST);
	if (qctrl)
        sensor->info_priv.contrast = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_HFLIP);
	if (qctrl)
        sensor->info_priv.mirror = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_VFLIP);
	if (qctrl)
        sensor->info_priv.flip = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SCENE);
	if (qctrl)
        sensor->info_priv.scene = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ZOOM_ABSOLUTE);
	if (qctrl)
        sensor->info_priv.digitalzoom = qctrl->default_value;

    /* ddl@rock-chips.com : if sensor support auto focus and flash, programer must run focus and flash code  */
	#if CONFIG_SENSOR_Focus
    sensor_set_focus();
    qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_ABSOLUTE);
	if (qctrl)
        sensor->info_priv.focus = qctrl->default_value;
	#endif

	#if CONFIG_SENSOR_Flash
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FLASH);
	if (qctrl)
        sensor->info_priv.flash = qctrl->default_value;
    flash_off_timer.icd = icd;
	flash_off_timer.timer.function = flash_off_func;
    #endif

    SENSOR_DG("\n%s..%s.. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),((val == 0)?__FUNCTION__:"sensor_reinit"),icd->user_width,icd->user_height);

    sensor->info_priv.funmodule_state |= SENSOR_INIT_IS_OK;
    return 0;
sensor_INIT_ERR:
    sensor->info_priv.funmodule_state &= ~SENSOR_INIT_IS_OK;
	sensor_task_lock(client,0);
	sensor_deactivate(client);
    return ret;
}

static int sensor_deactivate(struct i2c_client *client)
{
	struct soc_camera_device *icd = client->dev.platform_data;
    struct sensor *sensor = to_sensor(client);
	SENSOR_DG("\n%s..%s.. Enter\n",SENSOR_NAME_STRING(),__FUNCTION__);

	/* ddl@rock-chips.com : all sensor output pin must change to input for other sensor */
	sensor_ioctrl(icd, Sensor_PowerDown, 1);

	/* ddl@rock-chips.com : sensor config init width , because next open sensor quickly(soc_camera_open -> Try to configure with default parameters) */
	icd->user_width = SENSOR_INIT_WIDTH;
    icd->user_height = SENSOR_INIT_HEIGHT;
	msleep(100);
    sensor->info_priv.funmodule_state &= ~SENSOR_INIT_IS_OK;
	return 0;
}

static  struct reginfo sensor_power_down_sequence[]=
{
    {0x00,0x00}
};
static int sensor_suspend(struct soc_camera_device *icd, pm_message_t pm_msg)
{
    int ret;
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if (pm_msg.event == PM_EVENT_SUSPEND) {
        SENSOR_DG("\n %s Enter Suspend.. \n", SENSOR_NAME_STRING());
        ret = sensor_write_array(client, sensor_power_down_sequence) ;
        if (ret != 0) {
            SENSOR_TR("\n %s..%s WriteReg Fail.. \n", SENSOR_NAME_STRING(),__FUNCTION__);
            return ret;
        } else {
            ret = sensor_ioctrl(icd, Sensor_PowerDown, 1);
            if (ret < 0) {
			    SENSOR_TR("\n %s suspend fail for turn on power!\n", SENSOR_NAME_STRING());
                return -EINVAL;
            }
        }
    } else {
        SENSOR_TR("\n %s cann't suppout Suspend..\n",SENSOR_NAME_STRING());
        return -EINVAL;
    }
    return 0;
}

static int sensor_resume(struct soc_camera_device *icd)
{
	int ret;

    ret = sensor_ioctrl(icd, Sensor_PowerDown, 0);
    if (ret < 0) {
		SENSOR_TR("\n %s resume fail for turn on power!\n", SENSOR_NAME_STRING());
        return -EINVAL;
    }

	SENSOR_DG("\n %s Enter Resume.. \n", SENSOR_NAME_STRING());

    return 0;

}

static int sensor_set_bus_param(struct soc_camera_device *icd,
                                unsigned long flags)
{

    return 0;
}

static unsigned long sensor_query_bus_param(struct soc_camera_device *icd)
{
    struct soc_camera_link *icl = to_soc_camera_link(icd);
    unsigned long flags = SENSOR_BUS_PARAM;

    return soc_camera_apply_sensor_flags(icl, flags);
}

static int sensor_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    struct sensor *sensor = to_sensor(client);

    mf->width	= icd->user_width;
	mf->height	= icd->user_height;
	mf->code	= sensor->info_priv.fmt.code;
	mf->colorspace	= sensor->info_priv.fmt.colorspace;
	mf->field	= V4L2_FIELD_NONE;

    return 0;
}
static bool sensor_fmt_capturechk(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    bool ret = false;

	if ((mf->width == 1024) && (mf->height == 768)) {
		ret = true;
	} else if ((mf->width == 1280) && (mf->height == 1024)) {
		ret = true;
	} else if ((mf->width == 1600) && (mf->height == 1200)) {
		ret = true;
	} else if ((mf->width == 2048) && (mf->height == 1536)) {
		ret = true;
	} else if ((mf->width == 2592) && (mf->height == 1944)) {
		ret = true;
	}

	if (ret == true)
		SENSOR_DG("%s %dx%d is capture format\n", __FUNCTION__, mf->width, mf->height);
	return ret;
}

static bool sensor_fmt_videochk(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    bool ret = false;

	if ((mf->width == 1280) && (mf->height == 720)) {
		ret = true;
	} else if ((mf->width == 1920) && (mf->height == 1080)) {
		ret = true;
	}

	if (ret == true)
		SENSOR_DG("%s %dx%d is video format\n", __FUNCTION__, mf->width, mf->height);
	return ret;
}
static int sensor_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct sensor_datafmt *fmt;
	const struct v4l2_queryctrl *qctrl;
	struct soc_camera_device *icd = client->dev.platform_data;
    struct reginfo *winseqe_set_addr=NULL;
    int ret=0, set_w,set_h;

	fmt = sensor_find_datafmt(mf->code, sensor_colour_fmts,
				   ARRAY_SIZE(sensor_colour_fmts));
	if (!fmt) {
        ret = -EINVAL;
        goto sensor_s_fmt_end;
    }

	if (sensor->info_priv.fmt.code != mf->code) {
		switch (mf->code)
		{
			case V4L2_MBUS_FMT_YUYV8_2X8:
			{
				winseqe_set_addr = sensor_ClrFmt_YUYV;
				break;
			}
			case V4L2_MBUS_FMT_UYVY8_2X8:
			{
				winseqe_set_addr = sensor_ClrFmt_UYVY;
				break;
			}
			default:
				break;
		}
		if (winseqe_set_addr != NULL) {
            sensor_write_array(client, winseqe_set_addr);
			sensor->info_priv.fmt.code = mf->code;
            sensor->info_priv.fmt.colorspace= mf->colorspace;            
			SENSOR_DG("%s v4l2_mbus_code:%d set success!\n", SENSOR_NAME_STRING(),mf->code);
		} else {
			SENSOR_TR("%s v4l2_mbus_code:%d is invalidate!\n", SENSOR_NAME_STRING(),mf->code);
		}
	}

    set_w = mf->width;
    set_h = mf->height;

	if (((set_w <= 176) && (set_h <= 144)) && (sensor_qcif[0].reg != END_REG))
	{
		winseqe_set_addr = sensor_qcif;
        set_w = 176;
        set_h = 144;
	}
	else if (((set_w <= 320) && (set_h <= 240)) && (sensor_qvga[0].reg != END_REG))
    {
        winseqe_set_addr = sensor_qvga;
        set_w = 320;
        set_h = 240;
    }
    else if (((set_w <= 352) && (set_h<= 288)) && (sensor_cif[0].reg != END_REG))
    {
        winseqe_set_addr = sensor_cif;
        set_w = 352;
        set_h = 288;
    }
    else if (((set_w <= 640) && (set_h <= 480)) && (sensor_vga[0].reg != END_REG))
    {
        winseqe_set_addr = sensor_vga;
        set_w = 640;
        set_h = 480;
    }
    else if (((set_w <= 800) && (set_h <= 600)) && (sensor_svga[0].reg != END_REG))
    {
        winseqe_set_addr = sensor_svga;
        set_w = 800;
        set_h = 600;
    }
	else if (((set_w <= 1024) && (set_h <= 768)) && (sensor_xga[0].reg != END_REG))
    {
        winseqe_set_addr = sensor_xga;
        set_w = 1024;
        set_h = 768;
    }
    else if (((set_w <= 1280) && (set_h <= 1024)) && (sensor_sxga[0].reg != END_REG))
    {
        winseqe_set_addr = sensor_sxga;
        set_w = 1280;
        set_h = 1024;
    }
    else if (((set_w <= 1600) && (set_h <= 1200)) && (sensor_uxga[0].reg != END_REG))
    {
        winseqe_set_addr = sensor_uxga;
        set_w = 1600;
        set_h = 1200;
    }
    else
    {
        winseqe_set_addr = SENSOR_INIT_WINSEQADR;               /* ddl@rock-chips.com : Sensor output smallest size if  isn't support app  */
        set_w = SENSOR_INIT_WIDTH;
        set_h = SENSOR_INIT_HEIGHT;

		SENSOR_TR("\n %s..%s Format is Invalidate. pix->width = %d.. pix->height = %d\n",SENSOR_NAME_STRING(),__FUNCTION__,mf->width,mf->height);
    }

    if ((int)winseqe_set_addr  != sensor->info_priv.winseqe_cur_addr) {
        #if CONFIG_SENSOR_Flash
        if (sensor_fmt_capturechk(sd,mf) == true) {      /* ddl@rock-chips.com : Capture */
            if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
                sensor_ioctrl(icd, Sensor_Flash, Flash_On);
                SENSOR_DG("%s flash on in capture!\n", SENSOR_NAME_STRING());
            }           
        } else {                                        /* ddl@rock-chips.com : Video */
            if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
                sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
                SENSOR_DG("%s flash off in preivew!\n", SENSOR_NAME_STRING());
            }
        }
        #endif
        ret |= sensor_write_array(client, winseqe_set_addr);
		msleep(150);
        if (ret != 0) {
            SENSOR_TR("%s set format capability failed\n", SENSOR_NAME_STRING());
            #if CONFIG_SENSOR_Flash
            if (sensor_fmt_capturechk(sd,f) == true) {
                if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
                    sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
                    SENSOR_TR("%s Capture format set fail, flash off !\n", SENSOR_NAME_STRING());
                }
            }
            #endif
            goto sensor_s_fmt_end;
        }

        sensor->info_priv.winseqe_cur_addr  = (int)winseqe_set_addr;

		if (sensor_fmt_capturechk(sd,mf) == true) {				    /* ddl@rock-chips.com : Capture */
        #if CONFIG_SENSOR_Effect
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
        #endif
        #if CONFIG_SENSOR_WhiteBalance
			if (sensor->info_priv.whiteBalance != 0) {
				qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
				sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
			}
        #endif
			sensor->info_priv.snap2preview = true;
		} else if (sensor_fmt_videochk(sd,mf) == true) {			/* ddl@rock-chips.com : Video */
		#if CONFIG_SENSOR_Effect
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
        #endif
        #if CONFIG_SENSOR_WhiteBalance
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
			sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
        #endif
			sensor->info_priv.video2preview = true;
		} else if ((sensor->info_priv.snap2preview == true) || (sensor->info_priv.video2preview == true)) {
		#if CONFIG_SENSOR_Effect
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
        #endif
        #if CONFIG_SENSOR_WhiteBalance
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
			sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
        #endif
			sensor->info_priv.video2preview = false;
			sensor->info_priv.snap2preview = false;
		}

        SENSOR_DG("\n%s..%s.. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),__FUNCTION__,set_w,set_h);
    }
    else
    {
        SENSOR_DG("\n %s .. Current Format is validate. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),set_w,set_h);
    }

	mf->width = set_w;
    mf->height = set_h;

sensor_s_fmt_end:
    return ret;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct sensor_datafmt *fmt;
    int ret = 0;
   
	fmt = sensor_find_datafmt(mf->code, sensor_colour_fmts,
				   ARRAY_SIZE(sensor_colour_fmts));
	if (fmt == NULL) {
		fmt = &sensor->info_priv.fmt;
        mf->code = fmt->code;
	} 

    if (mf->height > SENSOR_MAX_HEIGHT)
        mf->height = SENSOR_MAX_HEIGHT;
    else if (mf->height < SENSOR_MIN_HEIGHT)
        mf->height = SENSOR_MIN_HEIGHT;

    if (mf->width > SENSOR_MAX_WIDTH)
        mf->width = SENSOR_MAX_WIDTH;
    else if (mf->width < SENSOR_MIN_WIDTH)
        mf->width = SENSOR_MIN_WIDTH;

    mf->colorspace = fmt->colorspace;
    
    return ret;
}

 static int sensor_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *id)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
        return -EINVAL;

    if (id->match.addr != client->addr)
        return -ENODEV;

    id->ident = SENSOR_V4L2_IDENT;      /* ddl@rock-chips.com :  Return OV2655  identifier */
    id->revision = 0;

    return 0;
}
#if CONFIG_SENSOR_Brightness
static int sensor_set_brightness(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_BrightnessSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_BrightnessSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Effect
static int sensor_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_EffectSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_EffectSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Exposure
static int sensor_set_exposure(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ExposureSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_ExposureSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Saturation
static int sensor_set_saturation(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_SaturationSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_SaturationSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Contrast
static int sensor_set_contrast(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ContrastSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_ContrastSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Mirror
static int sensor_set_mirror(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_MirrorSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_MirrorSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Flip
static int sensor_set_flip(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_FlipSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_FlipSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Scene
static int sensor_set_scene(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_SceneSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_SceneSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_WhiteBalance
static int sensor_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_WhiteBalanceSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_WhiteBalanceSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_DigitalZoom
static int sensor_set_digitalzoom(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl_info;
    int digitalzoom_cur, digitalzoom_total;

	qctrl_info = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ZOOM_ABSOLUTE);
	if (qctrl_info)
		return -EINVAL;

    digitalzoom_cur = sensor->info_priv.digitalzoom;
    digitalzoom_total = qctrl_info->maximum;

    if ((value > 0) && (digitalzoom_cur >= digitalzoom_total))
    {
        SENSOR_TR("%s digitalzoom is maximum - %x\n", SENSOR_NAME_STRING(), digitalzoom_cur);
        return -EINVAL;
    }

    if  ((value < 0) && (digitalzoom_cur <= qctrl_info->minimum))
    {
        SENSOR_TR("%s digitalzoom is minimum - %x\n", SENSOR_NAME_STRING(), digitalzoom_cur);
        return -EINVAL;
    }

    if ((value > 0) && ((digitalzoom_cur + value) > digitalzoom_total))
    {
        *value = digitalzoom_total - digitalzoom_cur;
    }

    if ((value < 0) && ((digitalzoom_cur + value) < 0))
    {
        value = 0 - digitalzoom_cur;
    }

    digitalzoom_cur += value;

    if (sensor_ZoomSeqe[digitalzoom_cur] != NULL)
    {
        if (sensor_write_array(client, sensor_ZoomSeqe[digitalzoom_cur]) != 0)
        {
            SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
            return -EINVAL;
        }
        SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
        return 0;
    }

    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Flash
static int sensor_set_flash(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{    
    if ((value >= qctrl->minimum) && (value <= qctrl->maximum)) {
        if (value == 3) {       /* ddl@rock-chips.com: torch */
            sensor_ioctrl(icd, Sensor_Flash, Flash_Torch_On);   /* Flash On */
        } else {
            sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
        }
        SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
        return 0;
    }
    
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

static int sensor_g_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct v4l2_queryctrl *qctrl;

    qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
        return -EINVAL;
    }

    switch (ctrl->id)
    {
        case V4L2_CID_BRIGHTNESS:
            {
                ctrl->value = sensor->info_priv.brightness;
                break;
            }
        case V4L2_CID_SATURATION:
            {
                ctrl->value = sensor->info_priv.saturation;
                break;
            }
        case V4L2_CID_CONTRAST:
            {
                ctrl->value = sensor->info_priv.contrast;
                break;
            }
        case V4L2_CID_DO_WHITE_BALANCE:
            {
                ctrl->value = sensor->info_priv.whiteBalance;
                break;
            }
        case V4L2_CID_EXPOSURE:
            {
                ctrl->value = sensor->info_priv.exposure;
                break;
            }
        case V4L2_CID_HFLIP:
            {
                ctrl->value = sensor->info_priv.mirror;
                break;
            }
        case V4L2_CID_VFLIP:
            {
                ctrl->value = sensor->info_priv.flip;
                break;
            }
        default :
                break;
    }
    return 0;
}



static int sensor_s_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    struct soc_camera_device *icd = client->dev.platform_data;
    const struct v4l2_queryctrl *qctrl;


    qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
        return -EINVAL;
    }

    switch (ctrl->id)
    {
#if CONFIG_SENSOR_Brightness
        case V4L2_CID_BRIGHTNESS:
            {
                if (ctrl->value != sensor->info_priv.brightness)
                {
                    if (sensor_set_brightness(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.brightness = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Exposure
        case V4L2_CID_EXPOSURE:
            {
                if (ctrl->value != sensor->info_priv.exposure)
                {
                    if (sensor_set_exposure(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.exposure = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Saturation
        case V4L2_CID_SATURATION:
            {
                if (ctrl->value != sensor->info_priv.saturation)
                {
                    if (sensor_set_saturation(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.saturation = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Contrast
        case V4L2_CID_CONTRAST:
            {
                if (ctrl->value != sensor->info_priv.contrast)
                {
                    if (sensor_set_contrast(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.contrast = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_WhiteBalance
        case V4L2_CID_DO_WHITE_BALANCE:
            {
                if (ctrl->value != sensor->info_priv.whiteBalance)
                {
                    if (sensor_set_whiteBalance(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.whiteBalance = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Mirror
        case V4L2_CID_HFLIP:
            {
                if (ctrl->value != sensor->info_priv.mirror)
                {
                    if (sensor_set_mirror(icd, qctrl,ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.mirror = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Flip
        case V4L2_CID_VFLIP:
            {
                if (ctrl->value != sensor->info_priv.flip)
                {
                    if (sensor_set_flip(icd, qctrl,ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.flip = ctrl->value;
                }
                break;
            }
#endif
        default:
            break;
    }

    return 0;
}
static int sensor_g_ext_control(struct soc_camera_device *icd , struct v4l2_ext_control *ext_ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);

    qctrl = soc_camera_find_qctrl(&sensor_ops, ext_ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
        return -EINVAL;
    }

    switch (ext_ctrl->id)
    {
        case V4L2_CID_SCENE:
            {
                ext_ctrl->value = sensor->info_priv.scene;
                break;
            }
        case V4L2_CID_EFFECT:
            {
                ext_ctrl->value = sensor->info_priv.effect;
                break;
            }
        case V4L2_CID_ZOOM_ABSOLUTE:
            {
                ext_ctrl->value = sensor->info_priv.digitalzoom;
                break;
            }
        case V4L2_CID_ZOOM_RELATIVE:
            {
                return -EINVAL;
            }
        case V4L2_CID_FOCUS_ABSOLUTE:
            {
                ext_ctrl->value = sensor->info_priv.focus;
                break;
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                return -EINVAL;
            }
        case V4L2_CID_FLASH:
            {
                ext_ctrl->value = sensor->info_priv.flash;
                break;
            }
        default :
            break;
    }
    return 0;
}
static int sensor_s_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);
    int val_offset;

    qctrl = soc_camera_find_qctrl(&sensor_ops, ext_ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
        return -EINVAL;
    }

	val_offset = 0;
    switch (ext_ctrl->id)
    {
#if CONFIG_SENSOR_Scene
        case V4L2_CID_SCENE:
            {
                if (ext_ctrl->value != sensor->info_priv.scene)
                {
                    if (sensor_set_scene(icd, qctrl,ext_ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.scene = ext_ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Effect
        case V4L2_CID_EFFECT:
            {
                if (ext_ctrl->value != sensor->info_priv.effect)
                {
                    if (sensor_set_effect(icd, qctrl,ext_ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.effect= ext_ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_DigitalZoom
        case V4L2_CID_ZOOM_ABSOLUTE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

                if (ext_ctrl->value != sensor->info_priv.digitalzoom)
                {
                    val_offset = ext_ctrl->value -sensor->info_priv.digitalzoom;

                    if (sensor_set_digitalzoom(icd, qctrl,&val_offset) != 0)
                        return -EINVAL;
                    sensor->info_priv.digitalzoom += val_offset;

                    SENSOR_DG("%s digitalzoom is %x\n",SENSOR_NAME_STRING(),  sensor->info_priv.digitalzoom);
                }

                break;
            }
        case V4L2_CID_ZOOM_RELATIVE:
            {
                if (ext_ctrl->value)
                {
                    if (sensor_set_digitalzoom(icd, qctrl,&ext_ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.digitalzoom += ext_ctrl->value;

                    SENSOR_DG("%s digitalzoom is %x\n", SENSOR_NAME_STRING(), sensor->info_priv.digitalzoom);
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Focus
        case V4L2_CID_FOCUS_ABSOLUTE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

                if (ext_ctrl->value != sensor->info_priv.focus)
                {
                    val_offset = ext_ctrl->value -sensor->info_priv.focus;

                    sensor->info_priv.focus += val_offset;
                }

                break;
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                if (ext_ctrl->value)
                {
                    sensor->info_priv.focus += ext_ctrl->value;

                    SENSOR_DG("%s focus is %x\n", SENSOR_NAME_STRING(), sensor->info_priv.focus);
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Flash
        case V4L2_CID_FLASH:
            {
                if (sensor_set_flash(icd, qctrl,ext_ctrl->value) != 0)
                    return -EINVAL;
                sensor->info_priv.flash = ext_ctrl->value;

                SENSOR_DG("%s flash is %x\n",SENSOR_NAME_STRING(), sensor->info_priv.flash);
                break;
            }
#endif
        default:
            break;
    }

    return 0;
}

static int sensor_g_ext_controls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ext_ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    int i, error_cnt=0, error_idx=-1;


    for (i=0; i<ext_ctrl->count; i++) {
        if (sensor_g_ext_control(icd, &ext_ctrl->controls[i]) != 0) {
            error_cnt++;
            error_idx = i;
        }
    }

    if (error_cnt > 1)
        error_idx = ext_ctrl->count;

    if (error_idx != -1) {
        ext_ctrl->error_idx = error_idx;
        return -EINVAL;
    } else {
        return 0;
    }
}

static int sensor_s_ext_controls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ext_ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    int i, error_cnt=0, error_idx=-1;


    for (i=0; i<ext_ctrl->count; i++) {
        if (sensor_s_ext_control(icd, &ext_ctrl->controls[i]) != 0) {
            error_cnt++;
            error_idx = i;
        }
    }

    if (error_cnt > 1)
        error_idx = ext_ctrl->count;

    if (error_idx != -1) {
        ext_ctrl->error_idx = error_idx;
        return -EINVAL;
    } else {
        return 0;
    }
}

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int sensor_video_probe(struct soc_camera_device *icd,
			       struct i2c_client *client)
{
    char value;
    int ret;
    struct sensor *sensor = to_sensor(client);

    /* We must have a parent by now. And it cannot be a wrong one.
     * So this entire test is completely redundant. */
    if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

	if (sensor_ioctrl(icd, Sensor_PowerDown, 0) < 0) {
		ret = -ENODEV;
		goto sensor_video_probe_err;
	}

    /* soft reset */
    ret = sensor_write(client, 0x01, 0xf9);
    if (ret != 0)
    {
        SENSOR_TR("soft reset %s failed\n",SENSOR_NAME_STRING());
        return -ENODEV;
    }
    mdelay(5);          //delay 5 microseconds

    /* check if it is an sensor sensor */
    ret = sensor_read(client, 0x04, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id high byte failed\n");
        ret = -ENODEV;
        goto sensor_video_probe_err;
    }

    SENSOR_TR("\n %s() %s  pid = 0x%x\n",__func__, SENSOR_NAME_STRING(), value);
    if (value == SENSOR_ID) {
        sensor->model = SENSOR_V4L2_IDENT;
    } else {
        SENSOR_TR("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), value);
        ret = -ENODEV;
        goto sensor_video_probe_err;
    }
	
    return 0;

sensor_video_probe_err:

    return ret;
}
static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    struct sensor *sensor = to_sensor(client);
    int ret = 0;
#if CONFIG_SENSOR_Flash	
    int i;
#endif
    
	SENSOR_DG("\n%s..%s..cmd:%x \n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
	switch (cmd)
	{
		case RK29_CAM_SUBDEV_DEACTIVATE:
		{
			sensor_deactivate(client);
			break;
		}

		case RK29_CAM_SUBDEV_IOREQUEST:
		{
			sensor->sensor_io_request = (struct rk29camera_platform_data*)arg;           
            if (sensor->sensor_io_request != NULL) { 
                if (sensor->sensor_io_request->gpio_res[0].dev_name && 
                    (strcmp(sensor->sensor_io_request->gpio_res[0].dev_name, dev_name(icd->pdev)) == 0)) {
                    sensor->sensor_gpio_res = (struct rk29camera_gpio_res*)&sensor->sensor_io_request->gpio_res[0];
                } else if (sensor->sensor_io_request->gpio_res[1].dev_name && 
                    (strcmp(sensor->sensor_io_request->gpio_res[1].dev_name, dev_name(icd->pdev)) == 0)) {
                    sensor->sensor_gpio_res = (struct rk29camera_gpio_res*)&sensor->sensor_io_request->gpio_res[1];
                }
            } else {
                SENSOR_TR("%s %s RK29_CAM_SUBDEV_IOREQUEST fail\n",SENSOR_NAME_STRING(),__FUNCTION__);
                ret = -EINVAL;
                goto sensor_ioctl_end;
            }
            /* ddl@rock-chips.com : if gpio_flash havn't been set in board-xxx.c, sensor driver must notify is not support flash control 
               for this project */
            #if CONFIG_SENSOR_Flash	
        	if (sensor->sensor_gpio_res) { 
                if (sensor->sensor_gpio_res->gpio_flash == INVALID_GPIO) {
                    for (i = 0; i < icd->ops->num_controls; i++) {
                		if (V4L2_CID_FLASH == icd->ops->controls[i].id) {
                			//memset((char*)&icd->ops->controls[i],0x00,sizeof(struct v4l2_queryctrl));  
                              sensor_controls[i].id=0xffff;         			
                		}
                    }
                    sensor->info_priv.flash = 0xff;
                    SENSOR_DG("%s flash gpio is invalidate!\n",SENSOR_NAME_STRING());
                }else{ //two cameras are the same,need to deal diffrently ,zyc
                    for (i = 0; i < icd->ops->num_controls; i++) {
                           if(0xffff == icd->ops->controls[i].id){
                              sensor_controls[i].id=V4L2_CID_FLASH;
                           }               
                    }
                }
        	}
            #endif
			break;
		}
		default:
		{
			SENSOR_TR("%s %s cmd(0x%x) is unknown !\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			break;
		}
	}
sensor_ioctl_end:
	return ret;

}
static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(sensor_colour_fmts))
		return -EINVAL;

	*code = sensor_colour_fmts[index].code;
	return 0;
}
static struct v4l2_subdev_core_ops sensor_subdev_core_ops = {
	.init		= sensor_init,
	.g_ctrl		= sensor_g_control,
	.s_ctrl		= sensor_s_control,
	.g_ext_ctrls          = sensor_g_ext_controls,
	.s_ext_ctrls          = sensor_s_ext_controls,
	.g_chip_ident	= sensor_g_chip_ident,
	.ioctl = sensor_ioctl,
};


static struct v4l2_subdev_video_ops sensor_subdev_video_ops = {
	.s_mbus_fmt	= sensor_s_fmt,
	.g_mbus_fmt	= sensor_g_fmt,
	.try_mbus_fmt	= sensor_try_fmt,
	.enum_mbus_fmt	= sensor_enum_fmt,
};

static struct v4l2_subdev_ops sensor_subdev_ops = {
	.core	= &sensor_subdev_core_ops,
	.video = &sensor_subdev_video_ops,
};

static int sensor_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
    struct sensor *sensor;
    struct soc_camera_device *icd = client->dev.platform_data;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct soc_camera_link *icl;
    int ret;

    SENSOR_DG("\n%s..%s..%d..\n",__FUNCTION__,__FILE__,__LINE__);
    if (!icd) {
        dev_err(&client->dev, "%s: missing soc-camera data!\n",SENSOR_NAME_STRING());
        return -EINVAL;
    }

    icl = to_soc_camera_link(icd);
    if (!icl) {
        dev_err(&client->dev, "%s driver needs platform data\n", SENSOR_NAME_STRING());
        return -EINVAL;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
        dev_warn(&adapter->dev,
        	 "I2C-Adapter doesn't support I2C_FUNC_I2C\n");
        return -EIO;
    }

    sensor = kzalloc(sizeof(struct sensor), GFP_KERNEL);
    if (!sensor)
        return -ENOMEM;

    v4l2_i2c_subdev_init(&sensor->subdev, client, &sensor_subdev_ops);

    /* Second stage probe - when a capture adapter is there */
    icd->ops		= &sensor_ops;
    sensor->info_priv.fmt = sensor_colour_fmts[0];
	#if CONFIG_SENSOR_I2C_NOSCHED
	atomic_set(&sensor->tasklock_cnt,0);
	#endif

    ret = sensor_video_probe(icd, client);
    if (ret < 0) {
        icd->ops = NULL;
        i2c_set_clientdata(client, NULL);
        kfree(sensor);
		sensor = NULL;
    }
	hrtimer_init(&(flash_off_timer.timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    SENSOR_DG("\n%s..%s..%d  ret = %x \n",__FUNCTION__,__FILE__,__LINE__,ret);
    return ret;
}

static int sensor_remove(struct i2c_client *client)
{
    struct sensor *sensor = to_sensor(client);
    struct soc_camera_device *icd = client->dev.platform_data;

    icd->ops = NULL;
    i2c_set_clientdata(client, NULL);
    client->driver = NULL;
    kfree(sensor);
	sensor = NULL;
    return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME_STRING(), 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = SENSOR_NAME_STRING(),
	},
	.probe		= sensor_probe,
	.remove		= sensor_remove,
	.id_table	= sensor_id,
};

static int __init sensor_mod_init(void)
{
    SENSOR_DG("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());
    return i2c_add_driver(&sensor_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
    i2c_del_driver(&sensor_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION(SENSOR_NAME_STRING(Camera sensor driver));
MODULE_AUTHOR("lxh@wisky.com.cn");
MODULE_LICENSE("GPL");





