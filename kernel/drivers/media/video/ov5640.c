/*
 * Driver for OV5640 CMOS Image Sensor from OmniVision
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 #define CONFIG_OV5640_AUTOFOCUS 1
#define lily_debug 0
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
#include "ov5640.h"
#include<../../../arch/arm/mach-rk30/include/mach/io.h>
#include<../../../arch/arm/mach-rk30/include/mach/gpio.h>
#if lily_debug
#include <linux/proc_fs.h>
static struct i2c_client * g_client;
#endif
static int in_work = 0;
static int open_first_time = 0;
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
#define SENSOR_NAME RK29_CAM_SENSOR_OV5640
#define SENSOR_NAME_FRONT RK29_CAM_SENSOR_OV5640_FRONT
#define SENSOR_V4L2_IDENT V4L2_IDENT_OV5640
#define SENSOR_ID 0x5640
#define SENSOR_MIN_WIDTH    176
#define SENSOR_MIN_HEIGHT   144
#define SENSOR_MAX_WIDTH    2592
#define SENSOR_MAX_HEIGHT   1944
#define SENSOR_INIT_WIDTH	800			/* Sensor pixel size for sensor_init_data array */
#define SENSOR_INIT_HEIGHT  600
#define SENSOR_INIT_WINSEQADR sensor_svga
#define SENSOR_INIT_PIXFMT V4L2_MBUS_FMT_UYVY8_2X8   // honghaishen_test

#define CONFIG_SENSOR_WhiteBalance	1
#define CONFIG_SENSOR_Brightness	0
#define CONFIG_SENSOR_Contrast      0
#define CONFIG_SENSOR_Saturation    0
#define CONFIG_SENSOR_Effect        0
#define CONFIG_SENSOR_Scene         1
#define CONFIG_SENSOR_DigitalZoom   0
#define CONFIG_SENSOR_Exposure      0
#define CONFIG_SENSOR_Flash         0
#define CONFIG_SENSOR_Mirror        0
#define CONFIG_SENSOR_Flip          0
#ifdef CONFIG_OV5640_AUTOFOCUS
#define CONFIG_SENSOR_Focus         1
#include "ov5640_af_firmware.c"
#else
#define CONFIG_SENSOR_Focus         0
#endif

#define CONFIG_SENSOR_I2C_SPEED    250000       /* Hz */
/* Sensor write register continues by preempt_disable/preempt_enable for current process not be scheduled */
#define CONFIG_SENSOR_I2C_NOSCHED   0
#define CONFIG_SENSOR_I2C_RDWRCHK   0

#define CONFIG_SENSOR_WRITE_REGS  1
#define WRITE_REGS_NUM 3

#define SENSOR_BUS_PARAM  (SOCAM_MASTER | SOCAM_PCLK_SAMPLE_RISING |\
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
#define SENSOR_NAME_FRONT_STRING(a) STR(CONS(SENSOR_NAME_FRONT, a))
#define SENSOR_NAME_VARFUN(a) CONS(SENSOR_NAME, a)

#define SENSOR_AF_IS_ERR    (0x00<<0)
#define SENSOR_AF_IS_OK		(0x01<<0)
#define SENSOR_INIT_IS_ERR   (0x00<<28)
#define SENSOR_INIT_IS_OK    (0x01<<28)

#if CONFIG_SENSOR_Focus
/*#define SENSOR_AF_MODE_INFINITY    0
#define SENSOR_AF_MODE_MACRO       1
#define SENSOR_AF_MODE_FIXED       2
#define SENSOR_AF_MODE_AUTO        3
#define SENSOR_AF_MODE_CONTINUOUS  4
#define SENSOR_AF_MODE_CLOSE       5*/
#define SENSOR_AF_MODE_AUTO        0
#define SENSOR_AF_MODE_CLOSE       1
#endif

#if CONFIG_SENSOR_Focus
/* ov5640 VCM Command and Status Registers */
#define CMD_MAIN_Reg      	0x3022
//#define CMD_TAG_Reg       	0x3023
#define CMD_ACK_Reg       	0x3023
#define CMD_PARA0_Reg     	0x3024
#define CMD_PARA1_Reg   	0x3025
#define CMD_PARA2_Reg     	0x3026
#define CMD_PARA3_Reg     	0x3027
#define CMD_PARA4_Reg     	0x3028

//#define STA_ZONE_Reg      	0x3026
#define STA_FOCUS_Reg     	0x3029

/* ov5640 VCM Command  */
#define OverlayEn_Cmd     0x01
#define OverlayDis_Cmd    0x02
#define SingleFocus_Cmd   0x03
#define ConstFocus_Cmd    0x04
#define StepMode_Cmd      0x05
#define PauseFocus_Cmd    0x06
#define ReturnIdle_Cmd    0x08
#define SetZone_Cmd       0x10
#define UpdateZone_Cmd    0x12
#define SetMotor_Cmd      0x20


#define Zone_configuration_Cmd  	0x12
#define Trig_autofocus_Cmd		0x03
#define Get_focus_result_Cmd		0x07

/* ov5640 Focus State */
//#define S_FIRWRE          	0xFF		/*Firmware is downloaded and not run*/
#define S_STARTUP         	0x7e		/*Firmware is initializing*/
#define S_ERROR           	0x7f
#define S_IDLE            	0x70		/*Idle state, focus is released; lens is located at the furthest position.*/
#define S_FOCUSING        	0x00		/*Auto Focus is running.*/
#define S_FOCUSED         	0x10		/*Auto Focus is completed.*/

#define S_CAPTURE         	0x12
#define S_STEP            		0x20

/* ov5640 Zone State */
#define Zone_Is_Focused(a, zone_val)    (zone_val&(1<<(a-3)))
#define Zone_Get_ID(zone_val)           (zone_val&0x03)

#define Zone_CenterMode   0x01
#define Zone_5xMode       0x02
#define Zone_5PlusMode    0x03
#define Zone_4fMode       0x04

#define ZoneSel_Auto      0x0b
#define ZoneSel_SemiAuto  0x0c
#define ZoneSel_Manual    0x0d
#define ZoneSel_Rotate    0x0e

/* ov5640 Step Focus Commands */
#define StepFocus_Near_Tag       0x01
#define StepFocus_Far_Tag        0x02
#define StepFocus_Furthest_Tag   0x03
#define StepFocus_Nearest_Tag    0x04
#define StepFocus_Spec_Tag       0x10
#endif

//flash off in fixed time to prevent from too hot , zyc
struct  flash_timer{
    struct soc_camera_device *icd;
	struct hrtimer timer;
};
struct  focus_timer_t{
    struct soc_camera_device *icd;
	struct hrtimer timer;
};
static enum hrtimer_restart flash_off_func(struct hrtimer *timer);
static enum hrtimer_restart focus_open_func(struct hrtimer *timer);

static struct  flash_timer flash_off_timer;
static struct  focus_timer_t focus_timer;
static int flash_on_off =0;
//for user defined if user want to customize the series , zyc
#ifdef CONFIG_OV5640_USER_DEFINED_SERIES
#include "ov5640_user_series.c"
#else

/* init 800X600 SVGA */
static struct reginfo sensor_init_data[] =
{
	//{0x3103, 0x11},
	//{0x3008, 0x82},
	{0x3008, 0x42},
	{0x3103, 0x03},
	{0x3017, 0xff},
	{0x3018, 0xff},
	{0x3034, 0x1a},
	#if defined(CONFIG_CAMERA_EMI_ENABLE)
	{0x3035, 0x41},
	#else
	{0x3035, 0x21},
	#endif
	{0x3036, 0x46},
	{0x3037, 0x13},
	{0x3108, 0x01},
	{0x3630, 0x36},
	{0x3631, 0x0e},
	{0x3632, 0xe2},
	{0x3633, 0x12},
	{0x3621, 0xe0},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3901, 0x0a},
	{0x3731, 0x12},
	{0x3600, 0x08},
	{0x3601, 0x33},
	{0x302d, 0x60},
	{0x3620, 0x52},
	{0x371b, 0x20},
	{0x471c, 0x50},
	{0x3a13, 0x43},
	{0x3a18, 0x00},
	#if defined(CONFIG_SOC_CAMERA_NOISE )
	{0x3a19, 0x48},  
	#else
	{0x3a19, 0xf8},  
	#endif
	{0x3635, 0x13},
	{0x3636, 0x03},
	{0x3634, 0x40},
	{0x3622, 0x01},
	{0x3c01, 0x34},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c06, 0x00},
	{0x3c07, 0x08},
	{0x3c08, 0x00},
	{0x3c09, 0x1c},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
	{0x3820, 0x47},
	{0x3821, 0x01},
	#else
	{0x3820, 0x41},
	{0x3821, 0x07},
	#endif
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9b},
	{0x3808, 0x03},
	{0x3809, 0x20},
	{0x380a, 0x02},
	{0x380b, 0x58},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x03},
	{0x380f, 0xd8},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x3000, 0x00},
	{0x3002, 0x1c},
	{0x3004, 0xff},
	{0x3006, 0xc3},
	{0x300e, 0x58},
	{0x302e, 0x00},
	{0x4740, 0x20},
	{0x4300, 0x32},   // honghaishen_test 0x30
	{0x501f, 0x00},
	{0x4713, 0x03},
	{0x4407, 0x04},
	{0x440e, 0x00},
	{0x460b, 0x35},
	{0x460c, 0x20},
	{0x4837, 0x22},
	{0x3824, 0x02},
	{0x5000, 0xa7},
	{0x5001, 0xa3},
	#if defined(CONFIG_MALATA_D1014) || defined(CONFIG_MALATA_D8009)
	{0x5180, 0xff},
{0x5181, 0xf2},
{0x5182, 0x0 },
{0x5183, 0x14},
{0x5184, 0x25},
{0x5185, 0x24},
{0x5186, 0x3 },
{0x5187, 0x14},
{0x5188, 0x22},
{0x5189, 0x9e},
{0x518a, 0x5f},
{0x518b, 0xa2},
{0x518c, 0x8f},
{0x518d, 0x48},
{0x518e, 0x32},
{0x518f, 0x91},
{0x5190, 0x57},
{0x5191, 0xf5},
{0x5192, 0xa },
{0x5193, 0x70},
{0x5194, 0xf0},
{0x5195, 0xf0},
{0x5196, 0x3 },
{0x5197, 0x1 },
{0x5198, 0x6 },
{0x5199, 0x9c},
{0x519a, 0x4 },
{0x519b, 0x0 },
{0x519c, 0x6 },
{0x519d, 0xa6},
{0x519e, 0x38},
#else
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x09},
	{0x5187, 0x09},
	{0x5188, 0x09},
	{0x5189, 0x75},
	{0x518a, 0x54},
	{0x518b, 0xe0},
	{0x518c, 0xb2},
	{0x518d, 0x42},
	{0x518e, 0x3d},
	{0x518f, 0x56},
	{0x5190, 0x46},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0x12},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x06},
	{0x519d, 0x82},
	{0x519e, 0x38},
	#endif
	{0x5381, 0x1c}, // hhs_0504
	{0x5382, 0x5a},
	{0x5383, 0x06},
	{0x5384, 0x1a},
	{0x5385, 0x66},
	{0x5386, 0x80},
	{0x5387, 0x82},
	{0x5388, 0x80},
	{0x5389, 0x02},
	#if 0
	{0x5381, 0x1e},
	{0x5382, 0x5b},
	{0x5383, 0x08},
	{0x5384, 0x0a},
	{0x5385, 0x7e},
	{0x5386, 0x88},
	{0x5387, 0x7c},
	{0x5388, 0x6c},
	{0x5389, 0x10},
	#endif
	{0x538a, 0x01},
	{0x538b, 0x98},
	{0x5300, 0x08},
	{0x5301, 0x30},
	{0x5302, 0x10},
	{0x5303, 0x00},
	{0x5304, 0x08},
	{0x5305, 0x30},
	{0x5306, 0x08},
	{0x5307, 0x16},
	{0x5309, 0x08},
	{0x530a, 0x30},
	{0x530b, 0x04},
	{0x530c, 0x06},
	{0x5480, 0x01},
	{0x5481, 0x08},
	{0x5482, 0x14},
	{0x5483, 0x28},
	{0x5484, 0x51},
	{0x5485, 0x65},
	{0x5486, 0x71},
	{0x5487, 0x7d},
	{0x5488, 0x87},
	{0x5489, 0x91},
	{0x548a, 0x9a},
	{0x548b, 0xaa},
	{0x548c, 0xb8},
	{0x548d, 0xcd},
	{0x548e, 0xdd},
	{0x548f, 0xea},
	{0x5490, 0x1d},
	{0x5580, 0x02},
	{0x5583, 0x40},
	{0x5584, 0x10},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	// guangzhen de mozu rolloff start
	{0x5800, 0x32},
	{0x5801, 0x1b},
	{0x5802, 0x16},
	{0x5803, 0x16},
	{0x5804, 0x1f},
	{0x5805, 0x3f},
	{0x5806, 0x12},
	{0x5807, 0x0a},
	{0x5808, 0x07},
	{0x5809, 0x07},
	{0x580a, 0x0b},
	{0x580b, 0x13},
	{0x580c, 0x0a},
	{0x580d, 0x04},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x04},
	{0x5811, 0x0b},
	{0x5812, 0x0b},
	{0x5813, 0x03},
	{0x5814, 0x00},
	{0x5815, 0x00},
	{0x5816, 0x04},
	{0x5817, 0x0c},
	{0x5818, 0x10},
	{0x5819, 0x0a},
	{0x581a, 0x06},
	{0x581b, 0x06},
	{0x581c, 0x0a},
	{0x581d, 0x10},
	{0x581e, 0x32},
	{0x581f, 0x19},
	{0x5820, 0x14},
	{0x5821, 0x14},
	{0x5822, 0x1b},
	{0x5823, 0x38},
	{0x5824, 0x6c},
	{0x5825, 0x2a},
	{0x5826, 0x2c},
	{0x5827, 0x2c},
	{0x5828, 0x6a},
	{0x5829, 0x28},  // 28
	{0x582a, 0x26},  //26
	{0x582b, 0x24},  //24
	{0x582c, 0x26},
	{0x582d, 0x2c},
	{0x582e, 0x28},  //28
	{0x582f, 0x22},  // 22
	{0x5830, 0x42},  // 42
	{0x5831, 0x42},  // 42
	{0x5832, 0x18},  // 18
	{0x5833, 0x2a},  // 2a
	{0x5834, 0x36},
	{0x5835, 0x24},
	{0x5836, 0x26},
	{0x5837, 0x2c},
	{0x5838, 0x88},   // 88
	{0x5839, 0x29},
	{0x583a, 0x29},
	{0x583b, 0x29},
	{0x583c, 0x69},
	{0x583d, 0xce},
	// guangzhen de mozu end
	{0x5025, 0x00},
	{0x3a0f, 0x32},    // honghaishen_test  30
	{0x3a10, 0x2c},
	{0x3a1b, 0x34},    // honghaishen_test 30
	{0x3a1e, 0x2a},
	{0x3a11, 0x60},
	{0x3a1f, 0x14},
	// honghaishen_test
	{0x3406 ,0x00},
	{0x5192 ,0x04},
	{0x5191 ,0xf8},
	{0x5193 ,0x70},
	{0x5194 ,0xf0},
	{0x5195 ,0xf0},
	{0x518d ,0x3d},
	{0x518f ,0x54},
	{0x518e ,0x3d},
	{0x5190 ,0x54},
	{0x518b ,0xa8},
	{0x518c ,0xa8},
	{0x5187 ,0x18},
	{0x5188 ,0x18},
	{0x5189 ,0x6e},
	{0x518a ,0x68},
	{0x5186 ,0x1c},
	{0x5181 ,0x50},
	{0x5184 ,0x25},
	{0x5182 ,0x11},
	{0x5183 ,0x14},
	{0x5184 ,0x25},
	{0x5185 ,0x24},
	{0x3400 ,0x7 },// honghaishen_test
	{0x3401 ,0x48},
	{0x3402 ,0x4 },
	{0x3403 ,0x0 },
	{0x3404 ,0x5 },     // 4
	{0x3405 ,0x83 },
	// honghaishen_test 
	#if defined(CONFIG_MALATA_D1024)
	{0x4745,0x02},
	#endif
	{0x3008, 0x02},
	{SEQUENCE_END, 0x00}
};

/* 720p 15fps @ 1280x720 */

static struct reginfo sensor_720p[]=
{
	{0x3503, 0x00},
		
	{0x3c07,0x07},
	{0x3803,0xfa},
	{0x3806,0x06},////
	{0x3807,0xa9},
	{0x3808,0x05},
	{0x3809,0x00},
	{0x380a,0x02},
	{0x380b,0xd0},
	{0x380c,0x07},
	{0x380d,0x64},
	{0x380e,0x02},
	{0x380f,0xe4},
	{0x3813,0x04},
	{0x3a02,0x02},
	{0x3a03,0xe4},
	{0x3a08,0x01},///
	{0x3a09,0xbc},////
	{0x3a0a,0x01},///
	{0x3a0b,0x72},////
	{0x3a0e,0x01},
	{0x3a0d,0x02},
	{0x3a14,0x02},
	{0x3a15,0xe4},
	#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
	{0x3820, 0x47}, //ddl@rock-chips.com add start: qsxvga -> 720p isn't stream on 
	{0x3821, 0x01},
	#else
    	{0x3820, 0x41}, //ddl@rock-chips.com add start: qsxvga -> 720p isn't stream on 
	{0x3821, 0x07},
	#endif
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08 ,0x01},///
	{0x3a09, 0x27},///
	{0x3a0a, 0x00},///
	{0x3a0b, 0xf6},///
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4004, 0x02},
	{0x3002, 0x1c},////
	{0x4713, 0x03},//////ddl@rock-chips.com add end
	{0x3002,0x00},///
	{0x4713,0x02},///
	{0x4837,0x16},
	{0x3824,0x04},///
	{0x5001,0x83},
	#if defined(CONFIG_CAMERA_EMI_ENABLE)
	{0x3035, 0x41},
	#else
	{0x3035, 0x21},
	#endif
	{0x3036,0x46},
	{0x4837, 0x22},
	{0x5001, 0xa3},
	{SEQUENCE_END, 0x00}
};

/* 	1080p, 0x15fps, 0xyuv @1920x1080 */

static struct reginfo sensor_1080p[]=
{

	{SEQUENCE_END, 0x00}
};

/* 2592X1944 QSXGA */
static struct reginfo sensor_qsxga[] =
{
	{0x3503, 0x07},
	{0x3a00, 0x78},
	{0x350c, 0x00},
	{0x350d, 0x00},
	{0x3c07, 0x07},
	#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
	{0x3820, 0x46},
	{0x3821, 0x00},
	#else
	{0x3820, 0x40},
	{0x3821, 0x06},
	#endif
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3803, 0x00},
	{0x3807, 0x9f},
	{0x3808, 0x0a},
	{0x3809, 0x20},
	{0x380a, 0x07},
	{0x380b, 0x98},
	{0x380c, 0x0b},
	{0x380d, 0x1c},
	{0x380e, 0x07},
	{0x380f, 0xb0},
	{0x3813, 0x04},
	{0x3618, 0x04},
	{0x3612, 0x2b},
	{0x3709, 0x12},
	{0x370c, 0x00},
	{0x3a02, 0x07},
	{0x3a03, 0xb0},
	{0x3a0e, 0x06},
	{0x3a0d, 0x08},
	{0x3a14, 0x07},
	{0x3a15, 0xb0},
	{0x4004, 0x06},
	#if defined(CONFIG_CAMERA_EMI_ENABLE)
	{0x3035, 0x41},
	#else
	{0x3035, 0x21},
	#endif
	{0x3036, 0x46},
	{0x4837, 0x2c},
	{0x5001, 0x83},

	{SEQUENCE_END, 0x00}
};
/* 2048*1536 QXGA */
static struct reginfo sensor_qxga[] =
{
	{0x3503, 0x07},
	{0x3a00, 0x78},
	{0x350c, 0x00},
	{0x350d, 0x00},
	{0x3c07, 0x07},
	#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
	{0x3820, 0x46},
	{0x3821, 0x00},
	#else
	{0x3820, 0x40},
	{0x3821, 0x06},
	#endif
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3803, 0x00},
	{0x3807, 0x9f},
	{0x3808, 0x08},
	{0x3809, 0x00},
	{0x380a, 0x06},
	{0x380b, 0x00},
	{0x380c, 0x0b},
	{0x380d, 0x1c},
	{0x380e, 0x07},
	{0x380f, 0xb0},
	{0x3813, 0x04},
	{0x3618, 0x04},
	{0x3612, 0x2b},
	{0x3709, 0x12},
	{0x370c, 0x00},
	{0x3a02, 0x07},
	{0x3a03, 0xb0},
	{0x3a0e, 0x06},
	{0x3a0d, 0x08},
	{0x3a14, 0x07},
	{0x3a15, 0xb0},
	{0x4004, 0x06},
	#if defined(CONFIG_CAMERA_EMI_ENABLE)
	{0x3035, 0x41},
	#else
	{0x3035, 0x21},
	#endif
	{0x3036, 0x46},
	{0x4837, 0x2c},
	{0x5001, 0xa3},

	{SEQUENCE_END, 0x00}
};

/* 1600X1200 UXGA */
static struct reginfo sensor_uxga[] =
{
	{0x3503, 0x07},
	{0x3a00, 0x78},
	{0x350c, 0x00},
	{0x350d, 0x00},
	{0x3c07, 0x07},
	#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
	{0x3820, 0x46},
	{0x3821, 0x00},
	#else
	{0x3820, 0x40},
	{0x3821, 0x06},
	#endif
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3803, 0x00},
	{0x3807, 0x9f},
	{0x3808, 0x06},
	{0x3809, 0x40},
	{0x380a, 0x04},
	{0x380b, 0xb0},
	{0x380c, 0x0b},
	{0x380d, 0x1c},
	{0x380e, 0x07},
	{0x380f, 0xb0},
	{0x3813, 0x04},
	{0x3618, 0x04},
	{0x3612, 0x2b},
	{0x3709, 0x12},
	{0x370c, 0x00},
	{0x3a02, 0x07},
	{0x3a03, 0xb0},
	{0x3a0e, 0x06},
	{0x3a0d, 0x08},
	{0x3a14, 0x07},
	{0x3a15, 0xb0},
	{0x4004, 0x06},
	#if defined(CONFIG_CAMERA_EMI_ENABLE)
	{0x3035, 0x41},
	#else
	{0x3035, 0x21},
	#endif
	{0x3036, 0x46},
	{0x4837, 0x2c},
	{0x5001, 0xa3},

	{SEQUENCE_END, 0x00}
};

/* 1280X1024 SXGA */
static struct reginfo sensor_sxga[] =
{
	{0x3503, 0x07},
	{0x3a00, 0x78},
	{0x350c, 0x00},
	{0x350d, 0x00},
	{0x3c07, 0x07},
	#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
	{0x3820, 0x46},
	{0x3821, 0x00},
	#else
	{0x3820, 0x40},
	{0x3821, 0x06},
	#endif
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3803, 0x00},
	{0x3807, 0x9f},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x04},
	{0x380b, 0x00},
	{0x380c, 0x0b},
	{0x380d, 0x1c},
	{0x380e, 0x07},
	{0x380f, 0xb0},
	{0x3813, 0x04},
	{0x3618, 0x04},
	{0x3612, 0x2b},
	{0x3709, 0x12},
	{0x370c, 0x00},
	{0x3a02, 0x07},
	{0x3a03, 0xb0},
	{0x3a0e, 0x06},
	{0x3a0d, 0x08},
	{0x3a14, 0x07},
	{0x3a15, 0xb0},
	{0x4004, 0x06},
	#if defined(CONFIG_CAMERA_EMI_ENABLE)
	{0x3035, 0x41},
	#else
	{0x3035, 0x21},
	#endif
	{0x3036, 0x46},
	{0x4837, 0x2c},
	{0x5001, 0xa3},
	{SEQUENCE_END, 0x00}
};
/* 1024X768 XGA */
static struct reginfo sensor_xga[] =
{
/*
	{0x3503, 0x07},
	{0x3a00, 0x78},
	{0x350c, 0x00},
	{0x350d, 0x00},
	{0x3c07, 0x07},
	#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
	{0x3820, 0x46},
	{0x3821, 0x00},
	#else
	{0x3820, 0x40},
	{0x3821, 0x06},
	#endif
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3803, 0x00},
	{0x3807, 0x9f},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x04},
	{0x380b, 0x00},
	{0x380c, 0x0b},
	{0x380d, 0x1c},
	{0x380e, 0x07},
	{0x380f, 0xb0},
	{0x3813, 0x04},
	{0x3618, 0x04},
	{0x3612, 0x2b},
	{0x3709, 0x12},
	{0x370c, 0x00},
	{0x3a02, 0x07},
	{0x3a03, 0xb0},
	{0x3a0e, 0x06},
	{0x3a0d, 0x08},
	{0x3a14, 0x07},
	{0x3a15, 0xb0},
	{0x4004, 0x06},
	#if defined(CONFIG_CAMERA_EMI_ENABLE)
	{0x3035, 0x41},
	#else
	{0x3035, 0x21},
	#endif
	{0x3036, 0x46},
	{0x4837, 0x2c},
	{0x5001, 0xa3},
	*/
	{SEQUENCE_END, 0x00}
};
/* 800X600 SVGA*/
static struct reginfo sensor_svga[] =
{
	{0x3503, 0x00},
	{0x3c07, 0x08},
	#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)
	{0x3820, 0x47},
	{0x3821, 0x01},
	#else
	{0x3820, 0x41},
	{0x3821, 0x07},
	#endif
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3803, 0x04},
	{0x3806, 0x07},///
	{0x3807, 0x9b},
	{0x3808, 0x03},
	{0x3809, 0x20},
	{0x380a, 0x02},
	{0x380b, 0x58},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x03},
	{0x380f, 0xd8},
	{0x3813, 0x06},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08 ,0x01},///
	{0x3a09, 0x27},///
	{0x3a0a, 0x00},///
	{0x3a0b, 0xf6},///
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4004, 0x02},
	{0x3002, 0x1c},////
	{0x4713, 0x03},////
	#if defined(CONFIG_CAMERA_EMI_ENABLE)
	{0x3035, 0x41},
	#else
	{0x3035, 0x21},
	#endif
	{0x3036, 0x46},
	{0x4837, 0x22},
	{0x3824, 0x02},////
	{0x5001, 0xa3},
	{SEQUENCE_END, 0x00}
};

/* 640X480 VGA */
static struct reginfo sensor_vga[] =
{

	{SEQUENCE_END, 0x00}
};
/* 352X288 CIF */
static struct reginfo sensor_cif[] =
{

	{SEQUENCE_END, 0x00}
};

/* 320*240 QVGA */
static  struct reginfo sensor_qvga[] =
{

	{SEQUENCE_END, 0x00}
};

/* 176X144 QCIF*/
static struct reginfo sensor_qcif[] =
{

	{SEQUENCE_END, 0x00}
};
#endif
static  struct reginfo sensor_ClrFmt_YUYV[]=
{
	{0x4300,0x30},
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_ClrFmt_UYVY[]=
{
	{0x4300,0x32},
	{SEQUENCE_END, 0x00}
};


#if CONFIG_SENSOR_WhiteBalance
static  struct reginfo sensor_WhiteB_Auto[]=
{
	{0x3406 ,0x00},
	#if defined(CONFIG_MALATA_D1014) || defined(CONFIG_MALATA_D8009)
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x0 },
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x3 },
	{0x5187, 0x14},
	{0x5188, 0x22},
	{0x5189, 0x9e},
	{0x518a, 0x5f},
	{0x518b, 0xa2},
	{0x518c, 0x8f},
	{0x518d, 0x48},
	{0x518e, 0x32},
	{0x518f, 0x91},
	{0x5190, 0x57},
	{0x5191, 0xf5},
	{0x5192, 0xa },
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x3 },
	{0x5197, 0x1 },
	{0x5198, 0x6 },
	{0x5199, 0x9c},
	{0x519a, 0x4 },
	{0x519b, 0x0 },
	{0x519c, 0x6 },
	{0x519d, 0xa6},
	{0x519e, 0x38},
       #else
	{0x5192 ,0x04},
	{0x5191 ,0xf8},
	{0x5193 ,0x70},
	{0x5194 ,0xf0},
	{0x5195 ,0xf0},
	{0x518d ,0x3d},
	{0x518f ,0x54},
	{0x518e ,0x3d},
	{0x5190 ,0x54},
	{0x518b ,0xa8},
	{0x518c ,0xa8},
	{0x5187 ,0x18},
	{0x5188 ,0x18},
	{0x5189 ,0x6e},
	{0x518a ,0x68},
	{0x5186 ,0x1c},
	{0x5181 ,0x50},
	{0x5184 ,0x25},
	{0x5182 ,0x11},
	{0x5183 ,0x14},
	{0x5184 ,0x25},
	{0x5185 ,0x24},
	#endif
	{SEQUENCE_END, 0x00}
};
/* Cloudy Colour Temperature : 6500K - 8000K  */
static  struct reginfo sensor_WhiteB_Cloudy[]=
{
	{0x3406 ,0x1 },
	{0x3400 ,0x7 },
	{0x3401 ,0x70},
	{0x3402 ,0x4 },
	{0x3403 ,0x30 },
	{0x3404 ,0x4 },
	{0x3405 ,0xa0},
	{SEQUENCE_END, 0x00}
};
/* ClearDay Colour Temperature : 5000K - 6500K  */
static  struct reginfo sensor_WhiteB_ClearDay[]=
{
	{0x3406 ,0x1 },
	{0x3400 ,0x6 },
	{0x3401 ,0xd0},
	{0x3402 ,0x4 },
	{0x3403 ,0x30 },
	{0x3404 ,0x4 },
	{0x3405 ,0xa0},
	{SEQUENCE_END, 0x00}
};
/* Office Colour Temperature : 3500K - 5000K  */
static  struct reginfo sensor_WhiteB_TungstenLamp1[]=
{
	{0x3406 ,0x1 },
	{0x3400 ,0x3 },
	{0x3401 ,0xd0},
	{0x3402 ,0x4 },
	{0x3403 ,0x0 },
	{0x3404 ,0x7 },
	{0x3405 ,0xcf},
	{SEQUENCE_END, 0x00}
};
/* Home Colour Temperature : 2500K - 3500K  */
static  struct reginfo sensor_WhiteB_TungstenLamp2[]=
{
	{0x3406 ,0x1 },
	{0x3400 ,0x4 },
	{0x3401 ,0xaf},
	{0x3402 ,0x3 },
	{0x3403 ,0x98 },
	{0x3404 ,0x6 },
	{0x3405 ,0x66},
	{SEQUENCE_END, 0x00}
};
static struct reginfo *sensor_WhiteBalanceSeqe[] = {sensor_WhiteB_Auto, sensor_WhiteB_TungstenLamp1,sensor_WhiteB_TungstenLamp2,
    sensor_WhiteB_ClearDay, sensor_WhiteB_Cloudy,NULL,
};
#endif

#if CONFIG_SENSOR_Brightness
static  struct reginfo sensor_Brightness0[]=
{
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Brightness1[]=
{
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Brightness2[]=
{
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Brightness3[]=
{
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Brightness4[]=
{
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Brightness5[]=
{
	{SEQUENCE_END, 0x00}
};
static struct reginfo *sensor_BrightnessSeqe[] = {sensor_Brightness0, sensor_Brightness1, sensor_Brightness2, sensor_Brightness3,
    sensor_Brightness4, sensor_Brightness5,NULL,
};

#endif

#if CONFIG_SENSOR_Effect
static  struct reginfo sensor_Effect_Normal[] =
{
    	{0x5001, 0x7f},
	{0x5580, 0x00},
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Effect_WandB[] =
{
    	{0x5001, 0xff},
	{0x5580, 0x18},
	{0x5583, 0x80},
	{0x5584, 0x80},
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Effect_Sepia[] =
{
    	{0x5001, 0xff},
	{0x5580, 0x18},
	{0x5583, 0x40},
	{0x5584, 0xa0},
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Effect_Negative[] =
{
    //Negative
    	{0x5001, 0xff},
	{0x5580, 0x40},
	{SEQUENCE_END, 0x00}
};static  struct reginfo sensor_Effect_Bluish[] =
{
    // Bluish
    	{0x5001, 0xff},
	{0x5580, 0x18},
	{0x5583, 0xa0},
	{0x5584, 0x40},
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Effect_Green[] =
{
    //  Greenish
    	{0x5001, 0xff},
	{0x5580, 0x18},
	{0x5583, 0x60},
	{0x5584, 0x60},
	{SEQUENCE_END, 0x00}
};

static struct reginfo *sensor_EffectSeqe[] = {sensor_Effect_Normal, sensor_Effect_WandB, sensor_Effect_Negative,sensor_Effect_Sepia,
    sensor_Effect_Bluish, sensor_Effect_Green,NULL,
};
#endif
#if CONFIG_SENSOR_Exposure
static  struct reginfo sensor_Exposure0[]=
{
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Exposure1[]=
{
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Exposure2[]=
{
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Exposure3[]=
{

	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Exposure4[]=
{
	
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Exposure5[]=
{

	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Exposure6[]=
{
	{SEQUENCE_END, 0x00}
};
static struct reginfo *sensor_ExposureSeqe[] = {sensor_Exposure0, sensor_Exposure1, sensor_Exposure2, sensor_Exposure3,
    sensor_Exposure4, sensor_Exposure5,sensor_Exposure6,NULL,
};
#endif
#if CONFIG_SENSOR_Saturation
static  struct reginfo sensor_Saturation0[]=
{
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Saturation1[]=
{
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Saturation2[]=
{
	{SEQUENCE_END, 0x00}
};static struct reginfo *sensor_SaturationSeqe[] = {sensor_Saturation0, sensor_Saturation1, sensor_Saturation2, NULL,};

#endif
#if CONFIG_SENSOR_Contrast
static  struct reginfo sensor_Contrast0[]=
{
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Contrast1[]=
{
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_Contrast2[]=
{
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Contrast3[]=
{
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Contrast4[]=
{
	{SEQUENCE_END, 0x00}
};


static  struct reginfo sensor_Contrast5[]=
{
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_Contrast6[]=
{
	{SEQUENCE_END, 0x00}
};
static struct reginfo *sensor_ContrastSeqe[] = {sensor_Contrast0, sensor_Contrast1, sensor_Contrast2, sensor_Contrast3,
    sensor_Contrast4, sensor_Contrast5, sensor_Contrast6, NULL,
};

#endif
#if CONFIG_SENSOR_Mirror
static  struct reginfo sensor_MirrorOn[]=
{
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_MirrorOff[]=
{
	{SEQUENCE_END, 0x00}
};
static struct reginfo *sensor_MirrorSeqe[] = {sensor_MirrorOff, sensor_MirrorOn,NULL,};
#endif
#if CONFIG_SENSOR_Flip
static  struct reginfo sensor_FlipOn[]=
{
	{SEQUENCE_END, 0x00}
};

static  struct reginfo sensor_FlipOff[]=
{
	{SEQUENCE_END, 0x00}
};
static struct reginfo *sensor_FlipSeqe[] = {sensor_FlipOff, sensor_FlipOn,NULL,};

#endif
#if CONFIG_SENSOR_Scene
static  struct reginfo sensor_SceneAuto[] =
{
	{0x3a00 , 0x78},
	{SEQUENCE_END, 0x00}
};
static  struct reginfo sensor_SceneNight[] =
{
    //15fps ~ 3.75fps night mode for 60/50Hz light environment, 24Mhz clock input,24Mzh pclk
	{0x3a00 ,0x7c},
	{0x3a02 ,0x05},
	{0x3a03 ,0xc4},
	{0x3a14 ,0x05},
	{0x3a15 ,0xc4},
	{SEQUENCE_END, 0x00}
};
static struct reginfo *sensor_SceneSeqe[] = {sensor_SceneAuto, sensor_SceneNight,NULL,};

#endif
#if CONFIG_SENSOR_DigitalZoom
static struct reginfo sensor_Zoom0[] =
{
	{SEQUENCE_END, 0x00}
};
static struct reginfo sensor_Zoom1[] =
{
	{SEQUENCE_END, 0x00}
};

static struct reginfo sensor_Zoom2[] =
{
	{SEQUENCE_END, 0x00}
};


static struct reginfo sensor_Zoom3[] =
{
	{SEQUENCE_END, 0x00}
};
static struct reginfo *sensor_ZoomSeqe[] = {sensor_Zoom0, sensor_Zoom1, sensor_Zoom2, sensor_Zoom3, NULL};
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
    // honghaishen_test hhs_1225
     { .id = V4L2_CID_OPEN_FLASH,  .index = 0,  .name = "off", .reserved = 0,}, {  .id = V4L2_CID_OPEN_FLASH, .index = 1,  .name = "on", .reserved = 0,},
    #endif
	
	// honghaishen_test
	#if 0
	#if CONFIG_OV5640_AUTOFOCUS
	{ .id = V4L2_CID_FOCUS_CONTINUOUS,  .index = 0,  .name = "off",  .reserved = 0, }, { .id = V4L2_CID_FOCUS_CONTINUOUS,  .index = 1,  .name = "on",  .reserved = 0, }, 
	#endif
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
	/*{
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
    },*/
	{
        .id		= V4L2_CID_FOCUS_AUTO,
        .type		= V4L2_CTRL_TYPE_BOOLEAN,
        .name		= "Focus Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    },
   
    // honghaishen_test
    #if 0
    {
        .id		= V4L2_CID_FOCUS_CONTINUOUS,
        .type		= V4L2_CTRL_TYPE_BOOLEAN,
        .name		= "Focus Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    },
    #endif
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
     // honghaishen_test hhs_1225
   {
        .id		= V4L2_CID_OPEN_FLASH,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Flash status",
        .minimum	= 0,
        .maximum	= 1,
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
static int sensor_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
static int sensor_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
static int sensor_deactivate(struct i2c_client *client);
static bool sensor_fmt_capturechk(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf);
static bool sensor_fmt_videochk(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf);
#if CONFIG_SENSOR_Scene
static int sensor_set_scene(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
#endif


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
enum sensor_wq_cmd
{
    WqCmd_af_init,
    WqCmd_af_single,
    WqCmd_af_special_pos,
    WqCmd_af_far_pos,
    WqCmd_af_near_pos,
    WqCmd_af_continues,
    WqCmd_af_return_idle,
};
enum sensor_wq_result
{
    WqRet_success = 0,
    WqRet_fail = -1,
    WqRet_inval = -2
};
struct sensor_work
{
	struct i2c_client *client;
	struct delayed_work dwork;
	enum sensor_wq_cmd cmd;
    wait_queue_head_t done;
    enum sensor_wq_result result;
    bool wait;
    int var;    
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
	int auto_focus;
	int affm_reinit;
    int flash;
    int exposure;
	bool snap2preview;
	bool video2preview;
    unsigned char mirror;                                        /* HFLIP */
    unsigned char flip;                                          /* VFLIP */
    struct reginfo *winseqe_cur_addr;
	struct sensor_datafmt fmt;
	unsigned int enable;
	unsigned int funmodule_state;
} sensor_info_priv_t;



struct sensor_parameter
{
	unsigned short int preview_maxlines;
	unsigned short int preview_exposure;
	unsigned short int preview_line_width;
	unsigned short int preview_gain;
	unsigned  int preview_systemclk;
	unsigned  int capture_systemclk;
	unsigned short int capture_framerate;
	unsigned short int preview_framerate;
	char awb[6];
};

struct sensor
{
    struct v4l2_subdev subdev;
    struct i2c_client *client;
    sensor_info_priv_t info_priv;
	struct sensor_parameter parameter;
	struct workqueue_struct *sensor_wq;
	struct mutex wq_lock;
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


#if CONFIG_SENSOR_WRITE_REGS
static int sensor_write_regs(struct i2c_client *client,  u8 *reg_info, int num)
{
	int err=0,cnt;
	struct i2c_msg msg[1];

	msg->len = num;	
	msg->addr = client->addr;	
	msg->flags = client->flags;	
	msg->buf = reg_info;	
	msg->scl_rate = CONFIG_SENSOR_I2C_SPEED;         /* ddl@rock-chips.com : 100kHz */	
	msg->read_type = 0;               /* fpga i2c:0==I2C_NORMAL : direct use number not enum for don't want include spi_fpga.h */	

	
	cnt= 3;	
	err = -EAGAIN;
	
	while ((cnt-- > 0) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */		
		err = i2c_transfer(client->adapter, msg, 1);		
		if (err >= 0) {		            
			return 0;		
		} else {		            
			SENSOR_TR("\n %s write reg failed, try to write again!\n",	SENSOR_NAME_STRING());		            
			udelay(10);	
		}	
	}
	
	return err;

}

#endif




/* sensor register write */
static int sensor_write(struct i2c_client *client, u16 reg, u8 val)
{
    int err,cnt;
    u8 buf[3];
    struct i2c_msg msg[1];

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;
    buf[2] = val;

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
static int sensor_read(struct i2c_client *client, u16 reg, u8 *val)
{
    int err,cnt;
    u8 buf[2];
    struct i2c_msg msg[2];

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;

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
#if CONFIG_SENSOR_Focus
	struct sensor *sensor = to_sensor(client);
#endif
#if CONFIG_SENSOR_I2C_RDWRCHK
	char valchk;
#endif
#if CONFIG_SENSOR_WRITE_REGS	
	int j = 0, reg_num;
	u8 *ptemp, *phead;
	int reg_length;
#endif

	cnt = 0;
	if (sensor_task_lock(client, 1) < 0)
		goto sensor_write_array_end;
    while (regarray[i].reg != SEQUENCE_END) {
    	#if CONFIG_SENSOR_Focus
    	if ((regarray == sensor_af_firmware) && (sensor->info_priv.enable == 0)) {
			SENSOR_DG("%s disable, Download af firmware terminated!\n",SENSOR_NAME_STRING());
			err = -EINVAL;
			goto sensor_write_array_end;
    	}
		#endif

#if CONFIG_SENSOR_WRITE_REGS
	
		j = i;		
		reg_num = 2;	
		reg_length = 0x0001;
						
		while((regarray[i].reg + reg_length) == regarray[i+1].reg) {		
			i++;			
			reg_num++;
			if(reg_num >= WRITE_REGS_NUM)
				break;
		}
		
		if(reg_num > 2) {			
			
			int size_num;
			size_num = reg_num + 1;
			
			ptemp = phead = (u8*)kmalloc((size_num+10)*sizeof(u8),GFP_KERNEL);
			if (!phead) {
				SENSOR_DG("-------------write registers allocate memory fail!!!\n");
                i = j;
                err = sensor_write(client, regarray[i].reg, regarray[i].val); 
			} else {			
    			*phead = regarray[j].reg >> 8;			
    			*(ptemp+1) = regarray[j].reg & 0xFF;
    			ptemp += 2;				
    			for( ; reg_num > 0; reg_num --, j++) {
    				*ptemp ++ = regarray[j].val;
    			}
    			
    			ptemp = phead;			
    			err = sensor_write_regs(client, ptemp,size_num);			
    			kfree(phead);	
			}
		}else{
			err = sensor_write(client, regarray[i].reg, regarray[i].val);
		}	
#else
		err = sensor_write(client, regarray[i].reg, regarray[i].val);        
#endif
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
			sensor_read(client, regarray[i].reg, &valchk);
			if (valchk != regarray[i].val)
				SENSOR_TR("%s Reg:0x%x write(0x%x, 0x%x) fail\n",SENSOR_NAME_STRING(), regarray[i].reg, regarray[i].val, valchk);
		#endif
        }
        i++; 
    }

	#if CONFIG_SENSOR_Focus
	if (((regarray->reg == SEQUENCE_PROPERTY) && (regarray->val == SEQUENCE_INIT))
		|| (regarray == sensor_init_data)) {
		sensor->info_priv.affm_reinit = 1;
	}
	#endif

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
#if CONFIG_SENSOR_Focus
struct af_cmdinfo
{
	char cmd_tag;
	char cmd_para[4];
	char validate_bit;
};
static int sensor_af_cmdset(struct i2c_client *client, int cmd_main, struct af_cmdinfo *cmdinfo)
{
	int i;
	char read_tag=0xff,cnt;

	if (cmdinfo) {
		if (cmdinfo->validate_bit & 0x80) {
			if (sensor_write(client, CMD_ACK_Reg, cmdinfo->cmd_tag)) {
				SENSOR_TR("%s write CMD_ACK_Reg(main:0x%x tag:0x%x) error!\n",SENSOR_NAME_STRING(),cmd_main,cmdinfo->cmd_tag);
				goto sensor_af_cmdset_err;
			}
			SENSOR_DG("%s write CMD_ACK_Reg(main:0x%x tag:0x%x) success!\n",SENSOR_NAME_STRING(),cmd_main,cmdinfo->cmd_tag);
		}
		for (i=0; i<4; i++) {
			if (cmdinfo->validate_bit & (1<<i)) {
				if (sensor_write(client, CMD_PARA0_Reg-i, cmdinfo->cmd_para[i])) {
					SENSOR_TR("%s write CMD_PARA_Reg(main:0x%x para%d:0x%x) error!\n",SENSOR_NAME_STRING(),cmd_main,i,cmdinfo->cmd_para[i]);
					goto sensor_af_cmdset_err;
				}
				SENSOR_DG("%s write CMD_PARA_Reg(main:0x%x para%d:0x%x) success!\n",SENSOR_NAME_STRING(),cmd_main,i,cmdinfo->cmd_para[i]);
			}
		}
	} else {
		if (sensor_write(client, CMD_ACK_Reg, 0xff)) {
			SENSOR_TR("%s write CMD_ACK_Reg(main:0x%x no tag) error!\n",SENSOR_NAME_STRING(),cmd_main);
			goto sensor_af_cmdset_err;
		}
		SENSOR_DG("%s write CMD_ACK_Reg(main:0x%x no tag) success!\n",SENSOR_NAME_STRING(),cmd_main);
	}

	if (sensor_write(client, CMD_MAIN_Reg, cmd_main)) {
		SENSOR_TR("%s write CMD_MAIN_Reg(main:0x%x) error!\n",SENSOR_NAME_STRING(),cmd_main);
		goto sensor_af_cmdset_err;
	}

	cnt = 0;
	do
	{
		msleep(5);
		if (sensor_read(client,CMD_ACK_Reg,&read_tag)){
		   SENSOR_TR("%s[%d] read TAG failed\n",SENSOR_NAME_STRING(),__LINE__);
		   break;
		}
    } while((read_tag != 0x00)&& (cnt++<100));

	SENSOR_DG("%s write CMD_MAIN_Reg(main:0x%x read tag:0x%x) success!\n",SENSOR_NAME_STRING(),cmd_main,read_tag);
	return 0;
sensor_af_cmdset_err:
	return -1;
}

static int sensor_af_idlechk(struct i2c_client *client)
{
	int ret = 0;
	char state;	
	int cnt =0;
	struct af_cmdinfo cmdinfo;
	
	SENSOR_DG("%s , %d\n",__FUNCTION__,__LINE__);
	
	cmdinfo.cmd_tag = 0x01;
	cmdinfo.validate_bit = 0x80;
	ret = sensor_af_cmdset(client, ReturnIdle_Cmd, &cmdinfo);
	if(0 != ret) {
		SENSOR_TR("%s[%d] read focus_status failed\n",SENSOR_NAME_STRING(),__LINE__);
		ret = -1;
		goto sensor_af_idlechk_end;
	}
	

	do{
		ret = sensor_read(client, CMD_ACK_Reg, &state);
		msleep(5);
		if (ret != 0){
		   SENSOR_TR("%s[%d] read focus_status failed\n",SENSOR_NAME_STRING(),__LINE__);
		   ret = -1;
		   goto sensor_af_idlechk_end;
		}
	}while((0x00 != state)&&(cnt++<100));


sensor_af_idlechk_end:
	return ret;
}

static int sensor_af_single(struct i2c_client *client)
{
	int ret = 0;
	char state,cnt;
	struct af_cmdinfo cmdinfo;

	cmdinfo.cmd_tag = 0x01;
	cmdinfo.validate_bit = 0x80;
	ret = sensor_af_cmdset(client, SingleFocus_Cmd, &cmdinfo);
	if(0 != ret) {
		SENSOR_TR("%s single focus mode set error!\n",SENSOR_NAME_STRING());
		ret = -1;
		goto sensor_af_single_end;
	}
	
	cnt = 0;
    do
    {
    	if (cnt != 0) {
			msleep(1);
    	}
    	cnt++;
		ret = sensor_read(client, STA_FOCUS_Reg, &state);
		if (ret != 0){
		   SENSOR_TR("%s[%d] read focus_status failed\n",SENSOR_NAME_STRING(),__LINE__);
		   ret = -1;
		   goto sensor_af_single_end;
		}
    }while((state == S_FOCUSING) && (cnt<100));

	if (state != S_FOCUSED) {
        SENSOR_TR("%s[%d] focus state(0x%x) is error!\n",SENSOR_NAME_STRING(),__LINE__,state);
		ret = -1;
		goto sensor_af_single_end;
    }
	else
		{
		sensor_write(client, CMD_MAIN_Reg, PauseFocus_Cmd);
		
		}
sensor_af_single_end:
	return ret;
}

static int sensor_af_const(struct i2c_client *client)
{
	int ret = 0;
	char state,cnt;
	struct af_cmdinfo cmdinfo;

	cmdinfo.cmd_tag = 0x01;
	cmdinfo.validate_bit = 0x80;
	ret = sensor_af_cmdset(client, ConstFocus_Cmd, &cmdinfo);
	return ret;
}


static int sensor_af_init(struct i2c_client *client)
{
	int ret = 0;
	char state,cnt;

	ret = sensor_write_array(client, sensor_af_firmware);
    if (ret != 0) {
       	SENSOR_TR("%s Download firmware failed\n",SENSOR_NAME_STRING());
       	ret = -1;
	   	goto sensor_af_init_end;
    }

    cnt = 0;
    do
    {
    	if (cnt != 0) {
			msleep(1);
    	}
    	cnt++;
		ret = sensor_read(client, STA_FOCUS_Reg, &state);
		if (ret != 0){
		   SENSOR_TR("%s[%d] read focus_status failed\n",SENSOR_NAME_STRING(),__LINE__);
		   ret = -1;
		   goto sensor_af_init_end;
		}
    }while((state == S_STARTUP) && (cnt<100));	

    if (state != S_IDLE) {
    	SENSOR_TR("%s focus state(0x%x) is error!\n",SENSOR_NAME_STRING(),state);
        ret = -1;
        goto sensor_af_init_end;
    }

sensor_af_init_end:
	SENSOR_DG("%s %s ret:0x%x \n",SENSOR_NAME_STRING(),__FUNCTION__,ret);
	//return ret;    // hhs_af
	return 0;
}

static int sensor_af_downfirmware(struct i2c_client *client)
{
	struct sensor *sensor = to_sensor(client);
	int ret=0;
    struct soc_camera_device *icd = client->dev.platform_data;
    struct v4l2_mbus_framefmt mf;
		
	SENSOR_DG("%s %s Enter\n",SENSOR_NAME_STRING(), __FUNCTION__);
       in_work = 1;
	if (sensor_af_init(client)) {
		sensor->info_priv.funmodule_state &= (~SENSOR_AF_IS_OK);
		ret = -1;
	} else {
		sensor->info_priv.funmodule_state |= SENSOR_AF_IS_OK;
        
        mf.width = icd->user_width;
		mf.height = icd->user_height;
        mf.code = sensor->info_priv.fmt.code;
        mf.colorspace = sensor->info_priv.fmt.colorspace;
        mf.field	= V4L2_FIELD_NONE;
        if (1) {    /* ddl@rock-chips.com: focus mode fix const auto focus in video */
            ret = sensor_af_const(client);
        } else {
    		switch (sensor->info_priv.auto_focus)
    		{
    			case SENSOR_AF_MODE_AUTO:
    			{
    				//ret = sensor_af_single(client);   // honghaishen_test 
    				break;
    			}
    			case SENSOR_AF_MODE_CLOSE:
    			{
    				ret = 0;
    				break;
    			}
    			default:
                {
    				SENSOR_DG("%s focus mode(0x%x) is unkonwn\n",SENSOR_NAME_STRING(),sensor->info_priv.auto_focus);
                    goto sensor_af_downfirmware_end;
    			}
    		}
        }
		SENSOR_DG("%s sensor_af_downfirmware set focus mode(0x%x) ret:0x%x\n",SENSOR_NAME_STRING(), sensor->info_priv.auto_focus,ret);
	}

sensor_af_downfirmware_end:
	in_work = 0;
	return ret;
}
static void sensor_af_workqueue(struct work_struct *work)
{
	struct sensor_work *sensor_work = container_of(work, struct sensor_work, dwork.work);
	struct i2c_client *client = sensor_work->client;
    struct sensor *sensor = to_sensor(client);
    struct af_cmdinfo cmdinfo;
    
    SENSOR_DG("%s %s Enter, cmd:0x%x \n",SENSOR_NAME_STRING(), __FUNCTION__,sensor_work->cmd);
    
    mutex_lock(&sensor->wq_lock);
    
    switch (sensor_work->cmd) 
    {
        case WqCmd_af_init:
        {
        	if (sensor_af_downfirmware(client) < 0) {
        		SENSOR_TR("%s Sensor_af_init is failed in sensor_af_workqueue!\n",SENSOR_NAME_STRING());
        	}            
            break;
        }
        case WqCmd_af_single:
        {
            if (sensor_af_single(client) < 0) {
        		SENSOR_TR("%s Sensor_af_single is failed in sensor_af_workqueue!\n",SENSOR_NAME_STRING());
                sensor_work->result = WqRet_fail;
        	} else {
                sensor_work->result = WqRet_success;
        	}
            break;
        }
        case WqCmd_af_special_pos:
        {
            sensor_af_idlechk(client);

			cmdinfo.cmd_tag = StepFocus_Spec_Tag;
			cmdinfo.cmd_para[0] = sensor_work->var;
			cmdinfo.validate_bit = 0x81;
			if (sensor_af_cmdset(client, StepMode_Cmd, &cmdinfo) < 0)
               sensor_work->result = WqRet_fail;
            else 
               sensor_work->result = WqRet_success;
            break;
        }
        case WqCmd_af_near_pos:
        {            
            sensor_af_idlechk(client);
            cmdinfo.cmd_tag = StepFocus_Near_Tag;
            cmdinfo.validate_bit = 0x80;
			if (sensor_af_cmdset(client, StepMode_Cmd, &cmdinfo) < 0)
               sensor_work->result = WqRet_fail;
            else 
               sensor_work->result = WqRet_success;
            break;
        }
        case WqCmd_af_far_pos:
        {
            sensor_af_idlechk(client);
			cmdinfo.cmd_tag = StepFocus_Far_Tag;
			cmdinfo.validate_bit = 0x80;
			if (sensor_af_cmdset(client, StepMode_Cmd, &cmdinfo) < 0)
               sensor_work->result = WqRet_fail;
            else 
               sensor_work->result = WqRet_success;
            break;
        }
        case WqCmd_af_continues:
        {
            if (sensor_af_const(client) < 0)
               sensor_work->result = WqRet_fail;
            else 
               sensor_work->result = WqRet_success;
            break;
        }
        case WqCmd_af_return_idle:
        {
            if (sensor_af_idlechk(client) < 0)
               sensor_work->result = WqRet_fail;
            else 
               sensor_work->result = WqRet_success;
            break;
        }  
        default:
            SENSOR_TR("Unknow command(%d) in %s af workqueue!",sensor_work->cmd,SENSOR_NAME_STRING());
            break;
    } 
set_end:
    if (sensor_work->wait == false) {
        kfree((void*)sensor_work);
    } else {
        wake_up(&sensor_work->done); 
    }
    mutex_unlock(&sensor->wq_lock); 
    return;
}

static int sensor_af_workqueue_set(struct soc_camera_device *icd, enum sensor_wq_cmd cmd, int var, bool wait)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct sensor *sensor = to_sensor(client); 
    struct sensor_work *wk;
    int ret=0;

    if (sensor->sensor_wq == NULL) { 
        ret = -EINVAL;
        goto sensor_af_workqueue_set_end;
    }

    if ((sensor->info_priv.funmodule_state & SENSOR_AF_IS_OK) != SENSOR_AF_IS_OK) {
        if (cmd != WqCmd_af_init) {
            SENSOR_TR("%s %s cmd(%d) ingore,because af module isn't ready!",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
            ret = -1;
            goto sensor_af_workqueue_set_end;
        }
    }
    
    wk = kzalloc(sizeof(struct sensor_work), GFP_KERNEL);
    if (wk) {
	    wk->client = client;
	    INIT_WORK(&(wk->dwork.work), sensor_af_workqueue);
        wk->cmd = cmd;
        wk->result = WqRet_inval;
        wk->wait = wait;
        wk->var = var;
        init_waitqueue_head(&wk->done);
        
	    queue_delayed_work(sensor->sensor_wq,&(wk->dwork),0);
        
        /* ddl@rock-chips.com: 
        * video_lock is been locked in v4l2_ioctl function, but auto focus may slow,
        * As a result any other ioctl calls will proceed very, very slowly since each call
        * will have to wait for the AF to finish. Camera preview is pause,because VIDIOC_QBUF 
        * and VIDIOC_DQBUF is sched. so unlock video_lock here.
        */
        if (wait == true) {
            mutex_unlock(&icd->video_lock);                     
            if (wait_event_timeout(wk->done, (wk->result != WqRet_inval), msecs_to_jiffies(4000)) == 0) {
                SENSOR_TR("%s %s cmd(%d) is timeout!",SENSOR_NAME_STRING(),__FUNCTION__,cmd);                        
            }
            ret = wk->result;
            kfree((void*)wk);
            mutex_lock(&icd->video_lock);  
        }
        
    } else {
        SENSOR_TR("%s %s cmd(%d) ingore,because struct sensor_work malloc failed!",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
        ret = -1;
    }
sensor_af_workqueue_set_end:
    return ret;
}
#endif

static u32 ov5640_get_sysclk(struct i2c_client *client)     // honghaishen_test
{
	/* calculate sysclk */
	u8 reg_low, reg_high;
	int sclk_rdiv_map[] = { 1,2,4,8};
	u32 bit_div2x,sys_div,multiplier,pre_div,pll_rdiv,sclk_rdiv,vco,aec_sys_clk;
	int err = 0;
	err = sensor_read(client, 0x3034, &reg_low);
	if (err) {
		pr_err("%s read reg 0x3034 err is %d, reg_low is 0x%02x\n",
						__func__, err, reg_low);
		return err;
	}
	reg_high = reg_low & 0x0f;
	if (reg_high == 8 || reg_high == 10)
		bit_div2x = reg_high >> 1;

	err = sensor_read(client, 0x3035, &reg_high);
	if (err) {
		pr_err("%s read reg 0x3035 err is %d, reg_high is 0x%02x\n",
						__func__, err, reg_high);
		return err;
	}
	sys_div = reg_high >> 4;
	if (sys_div == 0)
		sys_div = 16;

	err = sensor_read(client, 0x3036, &reg_low);
	if (err) {
		pr_err("%s read reg 0x3036 err is %d, reg_low is 0x%02x\n",
						__func__, err, reg_low);
		return err;
	}
	   multiplier = reg_low;

	err = sensor_read(client, 0x3037, &reg_low);
	if (err) {
		pr_err("%s read reg 0x3037 err is %d, reg_low is 0x%02x\n",
						__func__, err, reg_low);
		return err;
	}
	pre_div = reg_low & 0x0f;
	pll_rdiv = ((reg_low >> 4) & 0x01) + 1;

	err = sensor_read(client, 0x3108, &reg_low);
	if (err) {
		pr_err("%s read reg 0x3108 err is %d, reg_low is 0x%02x\n",
						__func__, err, reg_low);
		return err;
	}
	reg_high = reg_low & 0x03;
	sclk_rdiv = sclk_rdiv_map[reg_high];

	vco = 2400 * (multiplier) / (pre_div);

	aec_sys_clk = (vco) / (sys_div) / (pll_rdiv) * 2 /
					(bit_div2x) / (sclk_rdiv);
	
      //  printk("honghaishen_test get system clk is %d",aec_sys_clk);
	return aec_sys_clk;
}
static int sensor_parameter_record(struct i2c_client *client)
{
	u8 ret_l,ret_m,ret_h;
	u8 tp_l,tp_m,tp_h;
	unsigned int  wb_red;   // honghaishen_test
	struct sensor *sensor = to_sensor(client);

	sensor_write(client,0x3503,0x07);	//stop AE/AG
	sensor_write(client,0x3406,0x01);   //stop AWB

	sensor_read(client,0x3500,&ret_h);
	sensor_read(client,0x3501, &ret_m);
	sensor_read(client,0x3502, &ret_l);
	tp_l = ret_l;
	tp_m = ret_m;
	tp_h = ret_h;
	SENSOR_DG(" %s Read 0x3500 = 0x%02x  0x3501 = 0x%02x 0x3502=0x%02x \n",SENSOR_NAME_STRING(), ret_h, ret_m, ret_l);
	//sensor->parameter.preview_exposure = (tp_h<<12)+(tp_m<<4)+(tp_l>>4);
	sensor->parameter.preview_exposure = ((((tp_h & 0x0f) << 8)+ tp_m) << 4) + (tp_l>>4);
	//printk("honghaishen_test preview_exposure is %d \n ",sensor->parameter.preview_exposure);
	sensor_read(client,0x380c, &ret_h);
	sensor_read(client,0x380d, &ret_l);
	sensor->parameter.preview_line_width = ret_h & 0xff;
	sensor->parameter.preview_line_width = (sensor->parameter.preview_line_width << 8) +ret_l;
	//printk("honghaishen_test preview_line_width is %d \n ",sensor->parameter.preview_line_width);
	//Read back AGC Gain for preview
	sensor_read(client,0x350a, &ret_h);
	sensor_read(client,0x350b, &tp_l);
	sensor->parameter.preview_gain = ((ret_h & 0x01) << 8)  + tp_l;
	//printk("honghaishen_test preview_gain is %d \n ",sensor->parameter.preview_gain);
	//preview_maxlines
	sensor_read(client,0x380e, &ret_h);
	sensor->parameter.preview_maxlines = ret_h;
	sensor->parameter.preview_maxlines <<= 8;
	sensor_read(client,0x380f, &tp_l);
	sensor->parameter.preview_maxlines += tp_l;
	//printk("honghaishen_test preview_maxlines is %d \n ",sensor->parameter.preview_maxlines);
	sensor->parameter.preview_systemclk = ov5640_get_sysclk(client);   // honghaishen_test
	sensor->parameter.capture_framerate = 375;
	sensor->parameter.preview_framerate = 1500;

	sensor_read(client,0x3400,&sensor->parameter.awb[0]);		//record awb value
	sensor_read(client,0x3401,&sensor->parameter.awb[1]);
	sensor_read(client,0x3402,&sensor->parameter.awb[2]);
	sensor_read(client,0x3403,&sensor->parameter.awb[3]);
	sensor_read(client,0x3404,&sensor->parameter.awb[4]);
	sensor_read(client,0x3405,&sensor->parameter.awb[5]);
	
	//printk("honghaishen_test 3400 is  %x \n",sensor->parameter.awb[0]);
	//printk("honghaishen_test 3401 is  %x \n",sensor->parameter.awb[1]);
	//printk("honghaishen_test 3402 is  %x \n",sensor->parameter.awb[2]);
	//printk("honghaishen_test 3403 is  %x \n",sensor->parameter.awb[3]);
	//printk("honghaishen_test 3404 is  %x \n",sensor->parameter.awb[4]);
	//printk("honghaishen_test 3405 is  %x \n",sensor->parameter.awb[5]);
	
	wb_red = sensor->parameter.awb[0] * 256 + sensor->parameter.awb[1];
	//printk("honghaishen_test wb_red is %d \n",wb_red);
	wb_red = wb_red * 20 / 21;
	 sensor->parameter.awb[0]  =wb_red >> 8;
	 sensor->parameter.awb[1]  =wb_red;
//printk("honghaishen_test 2 3400 is  %x \n",sensor->parameter.awb[0]);
	//printk("honghaishen_test 2 3401 is  %x \n",sensor->parameter.awb[1]);
	 
	SENSOR_DG(" %s Read 0x350c = 0x%02x  0x350d = 0x%02x 0x350b=0x%02x \n",SENSOR_NAME_STRING(), ret_h, ret_l, sensor->parameter.preview_gain);
	return 0;
}
static int sensor_ae_transfer(struct i2c_client *client)
{
	u8  ExposureLow;
	u8  ExposureMid;
	u8  ExposureHigh;
	u16 ulCapture_Exposure;
	u32 ulCapture_Exposure_Gain;
	u16  iCapture_Gain;
	u8   Lines_10ms;
	#if defined(CONFIG_CAMERA_FLICK60)
	bool m_60Hz = 1; 
	#else
	bool m_60Hz = 0;
	#endif
	u8  reg_l = 0,reg_h =0;
	u16 Preview_Maxlines;
	u8  Gain;
	u32  Capture_MaxLines,capture_systemclk,capture_line_width,capture_max_band;
	u8 ret_l,ret_m,ret_h;
	struct sensor *sensor = to_sensor(client);

	//Preview_Maxlines = sensor->parameter.preview_line_width;
	Preview_Maxlines = sensor->parameter.preview_maxlines;
	Gain = sensor->parameter.preview_gain;
	/*
	sensor_read(client,0x350c, &reg_h);
	sensor_read(client,0x350d, &reg_l);
	Capture_MaxLines = reg_h & 0xff;
	Capture_MaxLines = (Capture_MaxLines << 8) + reg_l;
	*/
	sensor_read(client,0x380c, &ret_h);
	sensor_read(client,0x380d, &ret_l);
	capture_line_width = ret_h & 0xff;
	capture_line_width = (capture_line_width << 8) +ret_l;
	//capture_maxlines
	sensor_read(client,0x380e, &reg_h);
	Capture_MaxLines = reg_h;
	Capture_MaxLines <<= 8;
	sensor_read(client,0x380f, &reg_l);
	Capture_MaxLines +=  reg_l;
	capture_systemclk = ov5640_get_sysclk(client);
	if(m_60Hz== 1) {
		Lines_10ms = (capture_systemclk * 100 / capture_line_width) * 5 / 6;
	} else {
		Lines_10ms = capture_systemclk * 100 / capture_line_width;
	}
	capture_max_band = (Capture_MaxLines - 4) / Lines_10ms;  // honghaishen_test 
	//printk("honghaishen_test lines_10ms is %d \n",Lines_10ms);
	//Lines_10ms = Lines_10ms & 0xffff;
	
	if(Preview_Maxlines == 0)
		Preview_Maxlines = 1;

	//ulCapture_Exposure =
	//	(sensor->parameter.preview_exposure*(sensor->parameter.capture_framerate)*(Capture_MaxLines))/(((Preview_Maxlines)*(sensor->parameter.preview_framerate)));
	ulCapture_Exposure =
		sensor->parameter.preview_exposure * capture_systemclk / sensor->parameter.preview_systemclk 
		*sensor->parameter.preview_line_width /capture_line_width;
	//printk("honghaishen_test ulCapture_Exposure is %d \n",ulCapture_Exposure);
	ulCapture_Exposure_Gain =(u32) (ulCapture_Exposure * Gain);
	ulCapture_Exposure_Gain = ulCapture_Exposure_Gain /10 * 9;
	//printk("honghaishen_test ulCapture_Exposure_Gain is %d \n",ulCapture_Exposure_Gain);
	if(ulCapture_Exposure_Gain < (Lines_10ms<<4)) {
		ulCapture_Exposure = ulCapture_Exposure_Gain/16;
		if (ulCapture_Exposure < 1)
			ulCapture_Exposure = 1;
		iCapture_Gain = ulCapture_Exposure_Gain / ulCapture_Exposure;
		if (iCapture_Gain < 16)
			iCapture_Gain = 16;
	} else {
	
	
	
		if (ulCapture_Exposure_Gain > ((Lines_10ms *
					capture_max_band) << 4)) {
			/* exposure reach max */
			ulCapture_Exposure = Lines_10ms *
						capture_max_band;
			iCapture_Gain = ulCapture_Exposure_Gain /
						ulCapture_Exposure;
		}
		else {
			/*
			 * 1 / 100 < capture_shutter =< max,
			 * capture_shutter = n / 100
			 */
			ulCapture_Exposure = (ulCapture_Exposure_Gain >> 4) /
						Lines_10ms *
						Lines_10ms;
			iCapture_Gain = ulCapture_Exposure_Gain /
						ulCapture_Exposure;
		}
		
	}
	
	if(ulCapture_Exposure == 0)
		ulCapture_Exposure = 1;
	
	ExposureLow = ((unsigned char)ulCapture_Exposure)<<4;
	ExposureMid = (unsigned char)(ulCapture_Exposure >> 4) & 0xff;
	ExposureHigh = (unsigned char)(ulCapture_Exposure >> 12);
//printk("honghaishen_test ulCapture_Exposure write  is %d \n",ulCapture_Exposure);
//printk("honghaishen_test Gain write  is %d \n",iCapture_Gain);
	//Gain = 0;
	// write the gain and exposure to 0x350* registers
	//m_iWrite0x350b=Gain;
	if(iCapture_Gain > 63)
		iCapture_Gain = 63;   // honghaishen_test 
	sensor_write(client,0x350b, iCapture_Gain);
	iCapture_Gain = (iCapture_Gain >> 8) & 0x03;
	sensor_write(client,0x350a, iCapture_Gain);
	//m_iWrite0x3502=ExposureLow;
	sensor_write(client,0x3502, ExposureLow);
	//m_iWrite0x3501=ExposureMid;
	sensor_write(client,0x3501, ExposureMid);
	//m_iWrite0x3500=ExposureHigh;
	sensor_write(client,0x3500, ExposureHigh);
	// SendToFile("Gain = 0x%x\r\n", Gain);
	// SendToFile("ExposureLow = 0x%x\r\n", ExposureLow);
	// SendToFile("ExposureMid = 0x%x\r\n", ExposureMid);
	// SendToFile("ExposureHigh = 0x%x\r\n", ExposureHigh);
	//�ӳ���ʱ�����ⰵ������ʱ�������ֽ�����
	//camera_timed_wait(200);
	//linzhk camera_timed_wait(500);

	sensor_write(client,0x3400,sensor->parameter.awb[0]);		// resume awb value
	sensor_write(client,0x3401,sensor->parameter.awb[1]);
	sensor_write(client,0x3402,sensor->parameter.awb[2]);
	sensor_write(client,0x3403,sensor->parameter.awb[3]);
	sensor_write(client,0x3404,sensor->parameter.awb[4]);
	sensor_write(client,0x3405,sensor->parameter.awb[5]);
	msleep(100);
	SENSOR_DG(" %s Write 0x350b = 0x%02x  0x3502 = 0x%02x 0x3501=0x%02x 0x3500 = 0x%02x\n",SENSOR_NAME_STRING(), Gain, ExposureLow, ExposureMid, ExposureHigh);
	//mdelay(100);
	return 0;
}
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
			}
			break;
		}
		default:
		{
			SENSOR_TR("%s cmd(0x%x) is unknown!",SENSOR_NAME_STRING(),cmd);
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

static enum hrtimer_restart focus_open_func(struct hrtimer *timer){
	struct focus_timer_t* fps_timer = container_of(timer, struct focus_timer_t, timer);
     sensor_af_workqueue_set(fps_timer->icd, WqCmd_af_continues, 0, false);
    return 0;
}
static enum hrtimer_restart flash_on_func(struct hrtimer *timer) {
	struct flash_timer *fps_timer = container_of(timer, struct flash_timer, timer);
	sensor_ioctrl(fps_timer->icd,Sensor_Flash,Flash_Torch);
	flash_off_timer.timer.function = flash_off_func;
	hrtimer_forward_now(&(flash_off_timer.timer), ktime_set(0, 430*1000*1300));
	return HRTIMER_RESTART;
    
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl;
    const struct sensor_datafmt *fmt;
    char value;
    int ret,pid = 0;
     open_first_time = 1;
    SENSOR_DG("\n%s..%s.. \n",SENSOR_NAME_STRING(),__FUNCTION__);

	if (sensor_ioctrl(icd, Sensor_PowerDown, 0) < 0) {
		ret = -ENODEV;
		goto sensor_INIT_ERR;
	}
          mdelay(40);      // honghaishen_test
    /* soft reset */
	if (sensor_task_lock(client,1)<0)
		goto sensor_INIT_ERR;
    ret = sensor_write(client, 0x3008, 0x80);
    if (ret != 0) {
        SENSOR_TR("%s soft reset sensor failed\n",SENSOR_NAME_STRING());
        ret = -ENODEV;
		goto sensor_INIT_ERR;
    }

    mdelay(5);  //delay 5 microseconds
	/* check if it is an sensor sensor */
    ret = sensor_read(client, 0x300a, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id high byte failed\n");
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }

    pid |= (value << 8);

    ret = sensor_read(client, 0x300b, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id low byte failed\n");
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }

    pid |= (value & 0xff);
    SENSOR_TR("\n %s() %s  pid = 0x%x\n",__func__, SENSOR_NAME_STRING(), pid);

    if (pid == SENSOR_ID) {
        sensor->model = SENSOR_V4L2_IDENT;
	#if defined(CONFIG_MALATA_D8009)
	gpio_direction_output(RK30_PIN1_PA6,0);
	#endif
    } else {
        SENSOR_TR("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }
	sensor_write(client, 0x3103, 0x11);
	sensor_write(client, 0x3008, 0x82);
	mdelay(5);
    ret = sensor_write_array(client, sensor_init_data);
    if (ret != 0) {
        SENSOR_TR("error: %s initial failed\n",SENSOR_NAME_STRING());
        goto sensor_INIT_ERR;
    }
	sensor_task_lock(client,0);
    sensor->info_priv.winseqe_cur_addr  = SENSOR_INIT_WINSEQADR;
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
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_ABSOLUTE);
	if (qctrl)
        sensor->info_priv.focus = qctrl->default_value;

	#if CONFIG_SENSOR_Flash
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FLASH);
	if (qctrl)
        sensor->info_priv.flash = qctrl->default_value;
    flash_off_timer.icd = icd;
	flash_off_timer.timer.function = flash_off_func;
    #endif
	focus_timer.icd= icd;
	focus_timer.timer.function = focus_open_func;
    SENSOR_DG("\n%s..%s.. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),((val == 0)?__FUNCTION__:"sensor_reinit"),icd->user_width,icd->user_height);

    sensor->info_priv.funmodule_state = SENSOR_INIT_IS_OK;
        
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
    int i = 0;
	SENSOR_DG("\n%s..%s.. Enter\n",SENSOR_NAME_STRING(),__FUNCTION__);

	/* ddl@rock-chips.com : all sensor output pin must change to input for other sensor */
    while(in_work!=0)
    {
        msleep(50); 
        i++;
    }
    if (sensor->info_priv.funmodule_state & SENSOR_INIT_IS_OK) {
    	sensor_task_lock(client, 1);
        sensor_write(client, 0x3017, 0x00);  // FREX,VSYNC,HREF,PCLK,D9-D6
    	sensor_write(client, 0x3018, 0x03);  // D5-D0
    	sensor_write(client,0x3019,0x00);    // STROBE,SDA
    	sensor_task_lock(client, 0);
    } 
	hrtimer_cancel(&(flash_off_timer.timer));
	hrtimer_cancel(&(focus_timer.timer));
	sensor_ioctrl(icd, Sensor_PowerDown, 1);
	sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
	flush_workqueue(sensor->sensor_wq);
	/* ddl@rock-chips.com : sensor config init width , because next open sensor quickly(soc_camera_open -> Try to configure with default parameters) */
	icd->user_width = SENSOR_INIT_WIDTH;
    icd->user_height = SENSOR_INIT_HEIGHT;
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
static int sensor_s_fmt(struct v4l2_subdev *sd,struct v4l2_mbus_framefmt *mf)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    const struct sensor_datafmt *fmt;
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl;
	struct soc_camera_device *icd = client->dev.platform_data;
    struct reginfo *winseqe_set_addr=NULL;
	struct reginfo *winseqe_set_addr_r=NULL;
    int ret = 0, set_w,set_h;

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
			mdelay(5);
			winseqe_set_addr_r= winseqe_set_addr;
            sensor_write_array(client, winseqe_set_addr);
			mdelay(5);
			sensor->info_priv.fmt.code = mf->code;
            sensor->info_priv.fmt.colorspace= mf->colorspace;            
			SENSOR_DG("%s v4l2_mbus_code:%d set success!\n", SENSOR_NAME_STRING(),mf->code);
		} else {
			SENSOR_TR("%s v4l2_mbus_code:%d is invalidate!\n", SENSOR_NAME_STRING(),mf->code);
		}
	}

    set_w = mf->width;
    set_h = mf->height;

	if (((set_w <= 176) && (set_h <= 144)) && (sensor_qcif[0].reg!=SEQUENCE_END))
	{
		winseqe_set_addr = sensor_qcif;
        set_w = 176;
        set_h = 144;
	}
	else if (((set_w <= 320) && (set_h <= 240)) && (sensor_qvga[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_qvga;
        set_w = 320;
        set_h = 240;
    }
    else if (((set_w <= 352) && (set_h<= 288)) && (sensor_cif[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_cif;
        set_w = 352;
        set_h = 288;
    }
    else if (((set_w <= 640) && (set_h <= 480)) && (sensor_vga[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_vga;
        set_w = 640;
        set_h = 480;
    }
    else if (((set_w <= 800) && (set_h <= 600)) && (sensor_svga[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_svga;
        set_w = 800;
        set_h = 600;
    }
	else if (((set_w <= 1024) && (set_h <= 768)) && (sensor_xga[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_xga;
        set_w = 1024;
        set_h = 768;
    }
	else if (((set_w <= 1280) && (set_h <= 720)) && (sensor_720p[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_720p;
        set_w = 1280;
        set_h = 720;
    }
    else if (((set_w <= 1280) && (set_h <= 1024)) && (sensor_sxga[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_sxga;
        set_w = 1280;
        set_h = 1024;
    }
    else if (((set_w <= 1600) && (set_h <= 1200)) && (sensor_uxga[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_uxga;
        set_w = 1600;
        set_h = 1200;
    }
    else if (((set_w <= 1920) && (set_h <= 1080)) && (sensor_1080p[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_1080p;
        set_w = 1920;
        set_h = 1080;
    }
	else if (((set_w <= 2048) && (set_h <= 1536)) && (sensor_qxga[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_qxga;
        set_w = 2048;
        set_h = 1536;
    }
	else if (((set_w <= 2592) && (set_h <= 1944)) && (sensor_qsxga[0].reg!=SEQUENCE_END))
    {
        winseqe_set_addr = sensor_qsxga;
        set_w = 2592;
        set_h = 1944;
    }
    else
    {
        winseqe_set_addr = SENSOR_INIT_WINSEQADR;               /* ddl@rock-chips.com : Sensor output smallest size if  isn't support app  */
        set_w = SENSOR_INIT_WIDTH;
        set_h = SENSOR_INIT_HEIGHT;
		SENSOR_TR("\n %s..%s Format is Invalidate. pix->width = %d.. pix->height = %d\n",SENSOR_NAME_STRING(),__FUNCTION__,mf->width,mf->height);
    }

    if (winseqe_set_addr  != sensor->info_priv.winseqe_cur_addr)
    {
		if (sensor_fmt_capturechk(sd,mf) == true) {					/* ddl@rock-chips.com : Capture */
			sensor_parameter_record(client);
		/*#if CONFIG_SENSOR_Focus
			sensor_af_idlechk(client);
			if (sensor->info_priv.auto_focus == SENSOR_AF_MODE_CONTINUOUS)
				sensor_af_cmdset(client, PauseFocus_Cmd, NULL);
		#endif*/
		#if CONFIG_SENSOR_Flash
            if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
			// honghaishen_test hhs_1225
              //  sensor_ioctrl(icd, Sensor_Flash, Flash_On);
                SENSOR_DG("%s flash on in capture!\n", SENSOR_NAME_STRING());
            }
        #endif
		}else {                                        /* ddl@rock-chips.com : Video */
		#if CONFIG_SENSOR_Flash 
            if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
			// honghaishen_test hhs_1225
               // sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
                SENSOR_DG("%s flash off in preivew!\n", SENSOR_NAME_STRING());
            }
        #endif
        }
		if ((sensor->info_priv.winseqe_cur_addr->reg == SEQUENCE_PROPERTY) && (sensor->info_priv.winseqe_cur_addr->val == SEQUENCE_INIT)) {
			if (((winseqe_set_addr->reg == SEQUENCE_PROPERTY) && (winseqe_set_addr->val == SEQUENCE_NORMAL))
				|| (winseqe_set_addr->reg != SEQUENCE_PROPERTY)) {
				sensor_write(client, 0x3103, 0x11);
				sensor_write(client, 0x3008, 0x82);
				mdelay(5);
				ret |= sensor_write_array(client,sensor_init_data);
				SENSOR_DG("\n%s reinit ret:0x%x \n",SENSOR_NAME_STRING(), ret);
			}
		}
        ret |= sensor_write_array(client, winseqe_set_addr);
        if (ret != 0) {
            SENSOR_TR("%s set format capability failed\n", SENSOR_NAME_STRING());
            #if CONFIG_SENSOR_Flash
            if (sensor_fmt_capturechk(sd,mf) == true) {
                if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
			// honghaishen_test hhs_1225
                   // sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
                    SENSOR_TR("%s Capture format set fail, flash off !\n", SENSOR_NAME_STRING());
                }
            }
            #endif
            goto sensor_s_fmt_end;
        }
        sensor->info_priv.winseqe_cur_addr  = winseqe_set_addr;

		if (sensor_fmt_capturechk(sd,mf) == true) {				    /* ddl@rock-chips.com : Capture */
			sensor_ae_transfer(client);
			//qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			//sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
			if (sensor->info_priv.whiteBalance != 0) {
				qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
				sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
			}
			sensor->info_priv.snap2preview = true;
		} else if (sensor_fmt_videochk(sd,mf) == true) {			/* ddl@rock-chips.com : Video */
			//qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			//sensor_set_effect(icd, qctrl,sensor->info_priv.effect);

			sensor->info_priv.video2preview = true;
		} else if ((sensor->info_priv.snap2preview == true) || (sensor->info_priv.video2preview == true)) {
			//qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			//sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
			if (sensor->info_priv.snap2preview == true) {
				qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
				sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
				#if CONFIG_SENSOR_Scene
				qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SCENE);  // hhs_0621
				sensor_set_scene(icd, qctrl,sensor->info_priv.scene);      // hhs_0621
				#endif

			}
            #if CONFIG_SENSOR_Focus
            if (sensor->info_priv.auto_focus == SENSOR_AF_MODE_AUTO) {
#if 0
                sensor_af_workqueue_set(icd,WqCmd_af_return_idle,0,true);
#endif
               // msleep(200);
            } else {
                msleep(200);
            }
            #else   
            msleep(500);
            #endif
			msleep(400);
			sensor->info_priv.video2preview = false;
			sensor->info_priv.snap2preview = false;
		}
        SENSOR_DG("\n%s..%s.. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),__FUNCTION__,set_w,set_h);
    }
    else
    {
        SENSOR_DG("\n %s .. Current Format is validate. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),set_w,set_h);
    }
	if(open_first_time)
	{
		msleep(400);
		open_first_time = 0;
    }
	if(flash_on_off == 1)
	{
		sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
		flash_on_off =0 ;
		flash_off_timer.timer.function = flash_on_func;
		hrtimer_cancel(&(flash_off_timer.timer));
		hrtimer_start(&(flash_off_timer.timer),ktime_set(0, 20*1000*1200),HRTIMER_MODE_REL);
	}
	mf->width = set_w;
	mf->height = set_h;
	if (winseqe_set_addr_r != NULL) {
			mdelay(5);
            sensor_write_array(client, winseqe_set_addr_r);
			mdelay(5);
	}
sensor_s_fmt_end:
    return ret;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct sensor_datafmt *fmt;
    int ret = 0,set_w,set_h;
   
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

    set_w = mf->width;
    set_h = mf->height;

	if (((set_w <= 176) && (set_h <= 144)) && (sensor_qcif[0].reg!=SEQUENCE_END))
	{
        set_w = 176;
        set_h = 144;
	}
	else if (((set_w <= 320) && (set_h <= 240)) && (sensor_qvga[0].reg!=SEQUENCE_END))
    {
        set_w = 320;
        set_h = 240;
    }
    else if (((set_w <= 352) && (set_h<= 288)) && (sensor_cif[0].reg!=SEQUENCE_END))
    {
        set_w = 352;
        set_h = 288;
    }
    else if (((set_w <= 640) && (set_h <= 480)) && (sensor_vga[0].reg!=SEQUENCE_END))
    {
        set_w = 640;
        set_h = 480;
    }
    else if (((set_w <= 800) && (set_h <= 600)) && (sensor_svga[0].reg!=SEQUENCE_END))
    {
        set_w = 800;
        set_h = 600;
    }
	else if (((set_w <= 1024) && (set_h <= 768)) && (sensor_xga[0].reg!=SEQUENCE_END))
    {
        set_w = 1024;
        set_h = 768;
    }
	else if (((set_w <= 1280) && (set_h <= 720)) && (sensor_720p[0].reg!=SEQUENCE_END))
    {
        set_w = 1280;
        set_h = 720;
    }
    else if (((set_w <= 1280) && (set_h <= 1024)) && (sensor_sxga[0].reg!=SEQUENCE_END))
    {
        set_w = 1280;
        set_h = 1024;
    }
    else if (((set_w <= 1600) && (set_h <= 1200)) && (sensor_uxga[0].reg!=SEQUENCE_END))
    {
        set_w = 1600;
        set_h = 1200;
    }
    else if (((set_w <= 1920) && (set_h <= 1080)) && (sensor_1080p[0].reg!=SEQUENCE_END))
    {
        set_w = 1920;
        set_h = 1080;
    }
	else if (((set_w <= 2048) && (set_h <= 1536)) && (sensor_qxga[0].reg!=SEQUENCE_END))
    {
        set_w = 2048;
        set_h = 1536;
    }
	else if (((set_w <= 2592) && (set_h <= 1944)) && (sensor_qsxga[0].reg!=SEQUENCE_END))
    {
        set_w = 2592;
        set_h = 1944;
    }
    else
    {
        set_w = SENSOR_INIT_WIDTH;
        set_h = SENSOR_INIT_HEIGHT;
    }

    mf->width = set_w;
    mf->height = set_h;
    
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
	SENSOR_TR("\n%s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
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

//printk("honghaishen_test shezhi baipingheng \n");
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
static int sensor_set_digitalzoom(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int *value)
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

    if ((*value > 0) && (digitalzoom_cur >= digitalzoom_total))
    {
        SENSOR_TR("%s digitalzoom is maximum - %x\n", SENSOR_NAME_STRING(), digitalzoom_cur);
        return -EINVAL;
    }

    if  ((*value < 0) && (digitalzoom_cur <= qctrl_info->minimum))
    {
        SENSOR_TR("%s digitalzoom is minimum - %x\n", SENSOR_NAME_STRING(), digitalzoom_cur);
        return -EINVAL;
    }

    if ((*value > 0) && ((digitalzoom_cur + *value) > digitalzoom_total))
    {
        *value = digitalzoom_total - digitalzoom_cur;
    }

    if ((*value < 0) && ((digitalzoom_cur + *value) < 0))
    {
        *value = 0 - digitalzoom_cur;
    }

    digitalzoom_cur += *value;

    if (sensor_ZoomSeqe[digitalzoom_cur] != NULL)
    {
        if (sensor_write_array(client, sensor_ZoomSeqe[digitalzoom_cur]) != 0)
        {
            SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
            return -EINVAL;
        }
        SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, *value);
        return 0;
    }

    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Focus
static int sensor_set_focus_absolute(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl_info;
	int ret = 0;

	qctrl_info = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_ABSOLUTE);
	if (!qctrl_info)
		return -EINVAL;
    
	if ((sensor->info_priv.funmodule_state & SENSOR_AF_IS_OK) && (sensor->info_priv.affm_reinit == 0)) {
		if ((value >= qctrl_info->minimum) && (value <= qctrl_info->maximum)) {
            ret = sensor_af_workqueue_set(icd, WqCmd_af_special_pos, value, true);
			SENSOR_DG("%s..%s : %d  ret:0x%x\n",SENSOR_NAME_STRING(),__FUNCTION__, value,ret);
		} else {
			ret = -EINVAL;
			SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
		}
	} else {
		ret = -EACCES;
		SENSOR_TR("\n %s..%s AF module state(0x%x, 0x%x) is error!\n",SENSOR_NAME_STRING(),__FUNCTION__,
			sensor->info_priv.funmodule_state,sensor->info_priv.affm_reinit);
	}

	return ret;
}
static int sensor_set_focus_relative(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl_info;
	int ret = 0;

	qctrl_info = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_RELATIVE);
	if (!qctrl_info)
		return -EINVAL;    

	if ((sensor->info_priv.funmodule_state & SENSOR_AF_IS_OK) && (sensor->info_priv.affm_reinit == 0)) {
		if ((value >= qctrl_info->minimum) && (value <= qctrl_info->maximum)) {            
            if (value > 0) {
                ret = sensor_af_workqueue_set(icd, WqCmd_af_near_pos, 0, true);
            } else {
                ret = sensor_af_workqueue_set(icd, WqCmd_af_far_pos, 0, true);
            }
			SENSOR_DG("%s..%s : %d  ret:0x%x\n",SENSOR_NAME_STRING(),__FUNCTION__, value,ret);
		} else {
			ret = -EINVAL;
			SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
		}
	} else {
		ret = -EACCES;
		SENSOR_TR("\n %s..%s AF module state(0x%x, 0x%x) is error!\n",SENSOR_NAME_STRING(),__FUNCTION__,
			sensor->info_priv.funmodule_state,sensor->info_priv.affm_reinit);
	}
	return ret;
}

static int sensor_set_focus_mode(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct sensor *sensor = to_sensor(client);
	int ret = 0;
    
	if ((sensor->info_priv.funmodule_state & SENSOR_AF_IS_OK)  && (sensor->info_priv.affm_reinit == 0)) {
		switch (value)
		{
			case SENSOR_AF_MODE_AUTO:
			{
				if(in_work!=0)
					return 0;
				ret = sensor_af_workqueue_set(icd, WqCmd_af_single, 0, true);
		hrtimer_cancel(&(focus_timer.timer));
		focus_timer.timer.function = focus_open_func;
		hrtimer_start(&(focus_timer.timer),ktime_set(10, 5000*1000*1200),HRTIMER_MODE_REL);
		 flush_workqueue(sensor->sensor_wq);
				break;
			}

			/*case SENSOR_AF_MODE_MACRO:
			{
				ret = sensor_set_focus_absolute(icd, qctrl, 0xff);
				break;
			}

			case SENSOR_AF_MODE_INFINITY:
			{
				ret = sensor_set_focus_absolute(icd, qctrl, 0x00);
				break;
			}
			
			case SENSOR_AF_MODE_CONTINUOUS:
			{
				ret = sensor_af_workqueue_set(icd, WqCmd_af_continues, 0, true);
				break;
			}*/
			default:
				SENSOR_TR("\n %s..%s AF value(0x%x) is error!\n",SENSOR_NAME_STRING(),__FUNCTION__,value);
				break;

		}

		SENSOR_DG("%s..%s : %d  ret:0x%x\n",SENSOR_NAME_STRING(),__FUNCTION__, value,ret);
	} else {
		ret = -EACCES;
		SENSOR_TR("\n %s..%s AF module state(0x%x, 0x%x) is error!\n",SENSOR_NAME_STRING(),__FUNCTION__,
			sensor->info_priv.funmodule_state,sensor->info_priv.affm_reinit);
	}

	return ret;
}
#endif
#if CONFIG_SENSOR_Flash
// honghaishen_test hhs_1225
static int sensor_set_flash(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{    
    if ((value >= qctrl->minimum) && (value <= qctrl->maximum)) {
        if (value == 3) 
	{       /* ddl@rock-chips.com: torch */
            sensor_ioctrl(icd, Sensor_Flash, Flash_Torch);   /* Flash On */
        }
	else if(value ==2)
        {
		sensor_ioctrl(icd, Sensor_Flash, Flash_Torch);
		flash_on_off = 1;
        }
	else if(value ==0)
	{
            sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
        }
        SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
        return 0;
    }
    
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}

#if 0
static int sensor_set_flash(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{    
    if ((value >= qctrl->minimum) && (value <= qctrl->maximum)) {
        if (value == 3) {       /* ddl@rock-chips.com: torch */
            sensor_ioctrl(icd, Sensor_Flash, Flash_Torch);   /* Flash On */
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
#endif
static int sensor_g_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct v4l2_queryctrl *qctrl;

    qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
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
        SENSOR_TR("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
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
	// honghaishen_test hhs_1225
	u8 y_value=0;

    qctrl = soc_camera_find_qctrl(&sensor_ops, ext_ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
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
                return -EINVAL;
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
	// honghaishen_test hhs_1225 start
	case V4L2_CID_OPEN_FLASH:
		{
		 sensor_read(client, 0x56A1, &y_value);
		 if(y_value > 0x1A)
		 	ext_ctrl ->value = 0;
		 else 
		 	ext_ctrl->value =1;	
		}
	// honghaishen_test hhs_1225 end
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
    int val_offset,ret;

    qctrl = soc_camera_find_qctrl(&sensor_ops, ext_ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
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

                break;
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

                sensor_set_focus_relative(icd, qctrl,ext_ctrl->value);
                break;
            }
		case V4L2_CID_FOCUS_AUTO:
			#if 0
			{ printk("honghaishen_test V4L2_CID_FOCUS_AUTO \n");
				if (ext_ctrl->value == 1) {
					if (sensor_set_focus_mode(icd, qctrl,SENSOR_AF_MODE_AUTO) != 0) {
						if(0 == (sensor->info_priv.funmodule_state & SENSOR_AF_IS_OK)) {
							sensor->info_priv.auto_focus = SENSOR_AF_MODE_AUTO;
						}
						return -EINVAL;
					}
					sensor->info_priv.auto_focus = SENSOR_AF_MODE_AUTO;
				} else if (SENSOR_AF_MODE_AUTO == sensor->info_priv.auto_focus){
					if (ext_ctrl->value == 0)
						sensor->info_priv.auto_focus = SENSOR_AF_MODE_CLOSE;
				}
				break;
			}
			#endif
			{
				if (ext_ctrl->value) {
                    if ((ext_ctrl->value==1) || (SENSOR_AF_MODE_AUTO == sensor->info_priv.auto_focus)) {
    					if (sensor_set_focus_mode(icd, qctrl,SENSOR_AF_MODE_AUTO) != 0) {
    						if(0 == (sensor->info_priv.funmodule_state & SENSOR_AF_IS_OK)) {
    							sensor->info_priv.auto_focus = SENSOR_AF_MODE_AUTO;
    						}
    						return -EINVAL;
    					}
                    }
                    if (ext_ctrl->value == 1)
					    sensor->info_priv.auto_focus = SENSOR_AF_MODE_AUTO;
				} else if (SENSOR_AF_MODE_AUTO == sensor->info_priv.auto_focus){
					if (ext_ctrl->value == 0)
						sensor->info_priv.auto_focus = SENSOR_AF_MODE_CLOSE;
				} 
				break;
			}
		case V4L2_CID_FOCUS_CONTINUOUS:
			{
				/*if (SENSOR_AF_MODE_CONTINUOUS != sensor->info_priv.auto_focus) {
					if (ext_ctrl->value == 1) {
						if (sensor_set_focus_mode(icd, qctrl,SENSOR_AF_MODE_CONTINUOUS) != 0)
							return -EINVAL;
						sensor->info_priv.auto_focus = SENSOR_AF_MODE_CONTINUOUS;
					}
				} else {
					if (ext_ctrl->value == 0)
						sensor->info_priv.auto_focus = SENSOR_AF_MODE_CLOSE;
				}*/
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

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
	//#if CONFIG_SENSOR_Focus
	struct soc_camera_device *icd = client->dev.platform_data;
	struct v4l2_mbus_framefmt mf;
	//#endif
    
	if (enable == 1) {
		sensor->info_priv.enable = 1;
		#if CONFIG_SENSOR_Focus
		mf.width	= icd->user_width;
    	mf.height	= icd->user_height;
    	mf.code	= sensor->info_priv.fmt.code;
    	mf.colorspace	= sensor->info_priv.fmt.colorspace;
    	mf.field	= V4L2_FIELD_NONE;
		/* If auto focus firmware haven't download success, must download firmware again when in video or preview stream on */
		if (sensor_fmt_capturechk(sd, &mf) == false) {
			if ((sensor->info_priv.affm_reinit == 1) || ((sensor->info_priv.funmodule_state & SENSOR_AF_IS_OK)==0)) {
                sensor_af_workqueue_set(icd, WqCmd_af_init, 0, false);
				sensor->info_priv.affm_reinit = 0;
			}
		}
		#endif
	} else if (enable == 0) {
	    sensor->info_priv.enable = 0;
		#if CONFIG_SENSOR_Focus	
              // flush_workqueue(sensor->sensor_wq);
		#endif
	}
	return 0;
}

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int sensor_video_probe(struct soc_camera_device *icd,
			       struct i2c_client *client)
{
    char value;
    int ret,pid = 0;
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
    ret = sensor_write(client, 0x3008, 0x80);
    if (ret != 0) {
        SENSOR_TR("soft reset %s failed\n",SENSOR_NAME_STRING());
        ret = -ENODEV;
		goto sensor_video_probe_err;
    }
    mdelay(5);          //delay 5 microseconds

    /* check if it is an sensor sensor */
    ret = sensor_read(client, 0x300a, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id high byte failed\n");
        ret = -ENODEV;
        goto sensor_video_probe_err;
    }

    pid |= (value << 8);

    ret = sensor_read(client, 0x300b, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id low byte failed\n");
        ret = -ENODEV;
        goto sensor_video_probe_err;
    }

    pid |= (value & 0xff);
    SENSOR_TR("\n %s() %s  pid = 0x%x\n",__func__, SENSOR_NAME_STRING(), pid);
    if (pid == SENSOR_ID) {
        sensor->model = SENSOR_V4L2_IDENT;
    } else {
        SENSOR_TR("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
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
    int ret = 0,i;

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
                sensor->sensor_gpio_res = NULL;
                for (i=0; i<RK29_CAM_SUPPORT_NUMS;i++) {
                    if (sensor->sensor_io_request->gpio_res[i].dev_name && 
                        (strcmp(sensor->sensor_io_request->gpio_res[i].dev_name, dev_name(icd->pdev)) == 0)) {
                        sensor->sensor_gpio_res = (struct rk29camera_gpio_res*)&sensor->sensor_io_request->gpio_res[i];
                    }
                }
                if (sensor->sensor_gpio_res == NULL) {
                    SENSOR_TR("%s %s obtain gpio resource failed when RK29_CAM_SUBDEV_IOREQUEST \n",SENSOR_NAME_STRING(),__FUNCTION__);
                    ret = -EINVAL;
                    goto sensor_ioctl_end;
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
              //  printk("flash io:%d\n",sensor->sensor_gpio_res->gpio_flash);
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
	.s_stream   = sensor_s_stream,
};

static struct v4l2_subdev_ops sensor_subdev_ops = {
	.core	= &sensor_subdev_core_ops,
	.video = &sensor_subdev_video_ops,
};
#if lily_debug
int ov5642_write_proc (struct file *file, const char *buffer,
                      unsigned long count, void *data)
{
	char kbuf[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		char vol[10]={0,0,0,0,0,0,0,0,0,0};
		char v[4]={0,0,0,0};
	unsigned long totime;
	unsigned short vol2=0;
	unsigned short offset = 0;
	int32_t rc = 0;
	unsigned short vol1=0;
	//u16 reg = 0;
	char  val = 0;
	struct clk *clk = NULL;
	char value;

	if (count >= 14)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	
	if ('w' == kbuf[0]) {
		memcpy(vol, kbuf+1, 4);
		offset = (unsigned short) simple_strtoul(vol, NULL, 16);
		memcpy(v,kbuf+5,2);
		value = (u8) simple_strtoul(v,NULL,16);
		sensor_write(g_client, offset, value);
		printk("honghaishen_test %x %x \n ",offset ,value);
	
	}
	else if('r'==kbuf[0])
		{
		memcpy(vol, kbuf+1, 4);
		offset = (unsigned short) simple_strtoul(vol, NULL, 16);
		sensor_read(g_client,offset,&val);
		printk("honghaishen_test read val is %x \n",val);
		}
	else if('i'==kbuf[0])
		{
		 sensor_write_array(g_client, sensor_init_data);
		}
	
	return count;
}
int ov5642_read_proc (char *buffer, char **buffer_location, off_t offset,
                            int buffer_length, int *zero, void *ptr)
{

	if(offset > 0)
		return 0;
	return offset;
}
static void create_ov5642_proc_file(void)
{
	struct proc_dir_entry *ov5642_proc_file =
		create_proc_entry("ov5642", 0666, NULL);

	if (ov5642_proc_file) {
		ov5642_proc_file->read_proc = ov5642_read_proc;
		ov5642_proc_file->write_proc = ov5642_write_proc;
	} else
		printk(KERN_NOTICE "lily:(%d)%s:proc file create failed! \n",__LINE__,__func__);
}
#endif


static int sensor_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
    struct sensor *sensor;
    struct soc_camera_device *icd = client->dev.platform_data;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct soc_camera_link *icl;
    int ret;
#if lily_debug
   g_client = client;
#endif

    SENSOR_DG("\n%s..%s..%d..\n",__FUNCTION__,__FILE__,__LINE__);
    if (!icd) {
        dev_err(&client->dev, "%s: missing soc-camera data!\n",SENSOR_NAME_STRING());
        return -EINVAL;
    }
	#if lily_debug
	create_ov5642_proc_file();
	#endif

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
    } else {
		#if CONFIG_SENSOR_Focus
		sensor->sensor_wq = create_workqueue(SENSOR_NAME_STRING(_af_workqueue));
		if (sensor->sensor_wq == NULL)
			SENSOR_TR("%s create fail!", SENSOR_NAME_STRING(_af_workqueue));
		mutex_init(&sensor->wq_lock);
		#if defined(CONFIG_MALATA_D8009)
		gpio_request(RK30_PIN1_PA6, "camera power");
		#endif
		#endif
    }
	hrtimer_init(&(flash_off_timer.timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_init(&(focus_timer.timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    SENSOR_DG("\n%s..%s..%d  ret = %x \n",__FUNCTION__,__FILE__,__LINE__,ret);
    return ret;
}

static int sensor_remove(struct i2c_client *client)
{
    struct sensor *sensor = to_sensor(client);
    struct soc_camera_device *icd = client->dev.platform_data;

	#if CONFIG_SENSOR_Focus
	if (sensor->sensor_wq) {
		destroy_workqueue(sensor->sensor_wq);
		sensor->sensor_wq = NULL;
	}
	#endif

    icd->ops = NULL;
    i2c_set_clientdata(client, NULL);
    client->driver = NULL;
    kfree(sensor);
	sensor = NULL;
    return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME_STRING(), 0 },
	{SENSOR_NAME_FRONT_STRING(), 1 },
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
MODULE_AUTHOR("ddl <kernel@rock-chips>");
MODULE_LICENSE("GPL");










