/*
 * Copyright (C) Quicklogic 2013
 *
 * Quicklogic VX5A3B RGB-to-MIPI display driver
 * Author : Sunny
 *
 * * Based on tc358768.c
 * * for Rockchip.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
/*
 * Copyright (C) 2012 ROCKCHIP, Inc.
 *
 * author: hhb@rock-chips.com
 * create date: 2012-10-26
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * ql_vx5a3b ver 1.0: lcd bridge
 * ql_vx5a3b ver 2.0: lcd bridge and vee
 */
#define QLVX_DRIVER_VER "ql_vx5a3b ver 2.0"
/* Quicklogic VX5A3B RGB-to-MIPI
	20130516 - rev 1.0 Modified from tc358768.c
*/
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/rk_fb.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/rk_screen.h>
#include <linux/ktime.h>
#include "mipi_dsi.h"
#include "ql_vx5a3b_rc.h"
//Quicklogic routines
//mipi panel command max size
#define QL_MIPI_PANEL_CMD_SIZE 256
//QL chip id at 0x4fe
#define QL_ID 0X2300
#define QL_VX_LCD_VC 0
/* dcs read/write */
#define DTYPE_DCS_WRITE		0x05	/* short write, 0 parameter */
#define DTYPE_DCS_WRITE1	0x15	/* short write, 1 parameter */
#define DTYPE_DCS_READ		0x06	/* read */
#define DTYPE_DCS_LWRITE	0x39	/* long write */
/* generic read/write */
#define DTYPE_GEN_WRITE		0x03	/* short write, 0 parameter */
#define DTYPE_GEN_WRITE1	0x13	/* short write, 1 parameter */
#define DTYPE_GEN_WRITE2	0x23	/* short write, 2 parameter */
#define DTYPE_GEN_LWRITE	0x29	/* long write */
#define DTYPE_GEN_READ		0x04	/* long read, 0 parameter */
#define DTYPE_GEN_READ1		0x14	/* long read, 1 parameter */
#define DTYPE_GEN_READ2		0x24	/* long read, 2 parameter */

#define CONTROL_BYTE_DCS       (0x08u)
#define CONTROL_BYTE_GEN       (0x09u)
#define CONTROL_BYTE_I2C_RELEASE (0x0u)

#define QL_I2C_RELEASE  {\
		CONTROL_BYTE_I2C_RELEASE, \
}

#define GEN_QL_CSR_OFFSET_LENGTH  {\
		CONTROL_BYTE_GEN, \
        0x29,  /* Data ID */\
        0x05,  /* Vendor Id 1 */\
        0x01,  /* Vendor Id 2 */\
        0x41,  /* Vendor Unique Command */\
        0x00,  /* Address LS */\
        0x00,  /* Address MS */\
        0x00,  /* Length LS */\
        0x00,  /* Length MS */\
    }

#define GEN_QL_CSR_WRITE  {\
		CONTROL_BYTE_GEN, \
        0x29,  /* Data ID */\
        0x05,  /* Vendor Id 1 */\
        0x01,  /* Vendor Id 2 */\
        0x40,  /* Vendor Unique Command */\
        0x00,  /* Address LS */\
        0x00,  /* Address MS */\
        0x00,  /* data LS */\
		0x00, \
	    0x00, \
        0x00,  /* data MS */\
    }

struct QL_VX_INIT_INFO
{
	u16 address;
	u32 data;
};

//#define QL_SYS_CLK_20M
//#define QL_SYS_CLK_12_27M
//#define QL_SYS_CLK_14_30M_
//Porting: put your init seq. here.
static struct QL_VX_INIT_INFO ql_initialization_setting[] = 
{
#if 0
    //SYS_CLK = 14.30MHz  PCLK = 96.152MHz
	0x700 , 0x2CF00040 ,
	0x704 , 0x1F00BF ,
	0x70C , 0x00004600 ,
	0x710 , 0xCD1002 ,
	0x714 , 0x0 ,
	0x718 , 0x0000010A ,
	0x71C , 0xA8002F ,
	0x720 , 0x0 ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x700 , 0x2CF00040 ,
#ifdef QL_VEE_ENABLE
	0x70C , 0x77B1 ,//0x77A1 ,//xmlq vee enable
#else
	0x70C , 0x77A1 ,//xmlq vee disable
#endif
	0x718 , 0x0000020A ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x120 , 0x5 ,
	0x124 , 0xF2D0384 ,
	0x128 , 0x301405 ,
	0x12C , 0x32 ,
	0x130 , 0x3C10 ,
	0x134 , 0x15 ,
	0x138 , 0xFF8000 ,
	0x13C , 0x0 ,
	0x140 , 0x10000 ,
#ifdef QL_VEE_ENABLE
	0x174 , 0xff , //xmlq vee
	0x418 , 0x555502aa ,
//  0x404 , 0x55550822 ,//0x55552022
#endif
	0x234 , 0xCA033E90 ,
	0x238 , 0x00001063 ,
	0x240 , 0x2861408B ,
	0x244 , 0x00130285 ,
	0x248 , 0x10630009 ,
	0x250 , 0x400B82A8 ,
	0x158 , 0x0 ,
	0x158 , 0x1 ,
	0x308 , 0xffffffff ,
	0x30C , 0x2204 ,
	0x310 , 0xffffff ,
	0x314 , 0x1ffff ,
	0x318 , 0x17 ,
	0x31C , 0xff ,
	0x320 , 0x5A00384 ,
	0x328 , 0x3 ,
	0x32C , 0x13 ,
	0x330 , 0x11 ,
	0x334 , 0x2EE ,
	0x338 , 0xA ,
	0x33C , 0x6 ,
	0x340 , 0x4 ,
	0x344 , 0x17 ,
	0x350 , 0x598 ,
	0x354 , 0x4 ,
	0x358 , 0x3 ,
	0x35C , 0x0 ,
	0x364 , 0x300011 ,
	0x368 , 0x5 ,
	0x36C , 0x09050901 ,
	0x370 , 0x08061402 ,
	0x374 , 0x1 ,
	0x378 , 0xCA033810 ,
	0x37C , 0x00000060 ,
	0x380 , 0x82E86030 ,
	0x384 , 0x28614088 ,
	0x388 , 0x00110285 ,
	0x38C , 0x10600008 ,
	0x394 , 0x400882A8 ,
	0x608 , 0x50F ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x300 , 0x1 ,
#elif 0
    //PCLK=96.152 MIPI_CLK=288MHz
	0x700 , 0x2CF00040 ,
	0x704 , 0x1F00BF ,
	0x70C , 0x00004600 ,
	0x710 , 0xCD1082 ,
	0x714 , 0x0 ,
	0x718 , 0x0000010A ,
	0x71C , 0xA8002F ,
	0x720 , 0x0 ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x700 , 0x2CF00040 ,
#ifdef QL_VEE_ENABLE
	0x70C , 0x77B1 ,//0x77A1 ,//xmlq vee enable
#else
	0x70C , 0x77A1 ,//xmlq vee disable
#endif
	0x718 , 0x0000020A ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x120 , 0x5 ,
	0x124 , 0xF2D0384 ,
	0x128 , 0x301405 ,
	0x12C , 0x32 ,
	0x130 , 0x3C10 ,
	0x134 , 0x15 ,
	0x138 , 0xFF8000 ,
	0x13C , 0x0 ,
	0x140 , 0x10000 ,
#ifdef QL_VEE_ENABLE
	0x174 , 0xff , //xmlq vee
	0x418 , 0x555502bb ,
#endif
	0x174 , 0xff ,
	0x234 , 0xCA033E90 ,
	0x238 , 0x00001063 ,
	0x240 , 0x2861408B ,
	0x244 , 0x00130285 ,
	0x248 , 0x10630009 ,
	0x250 , 0x400B82A8 ,
	0x158 , 0x0 ,
	0x158 , 0x1 ,
	0x308 , 0xffffffff ,
	0x30C , 0x2204 ,
	0x310 , 0xffffff ,
	0x314 , 0x1ffff ,
	0x318 , 0x17 ,
	0x31C , 0xff ,
	0x320 , 0x5A00384 ,
	0x328 , 0x3 ,
	0x32C , 0x13 ,
	0x330 , 0x11 ,
	0x334 , 0x38C ,
	0x338 , 0xA ,
	0x33C , 0x6 ,
	0x340 , 0x4 ,
	0x344 , 0x1B ,
	0x350 , 0x598 ,
	0x354 , 0x4 ,
	0x358 , 0x3 ,
	0x35C , 0x0 ,
	0x364 , 0x380014 ,
	0x368 , 0x6 ,
	0x36C , 0x0A060B01 ,
	0x370 , 0x0A071802 ,
	0x374 , 0x1 ,
	0x378 , 0xC6033810 ,
	0x37C , 0x00000060 ,
	0x380 , 0x82E86030 ,
	0x384 , 0x28614088 ,
	0x388 , 0x00110285 ,
	0x38C , 0x10600008 ,
	0x394 , 0x400882A8 ,
	0x608 , 0x50F ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x300 , 0x1 ,
#elif 0
	//pclk=74.42MHz MIPI_CLK=229MHz VSYNC=HVSYNC=0
	0x700 , 0x2CF00040 ,
	0x704 , 0x1F00BF ,
	0x70C , 0x00004600 ,
	0x710 , 0xCD1082 ,
	0x714 , 0x0 ,
	0x718 , 0x00000108 ,
	0x71C , 0xA8002F ,
	0x720 , 0x0 ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x700 , 0x2CF00040 ,
#ifdef QL_VEE_ENABLE
	0x70C , 0x77B1 ,//0x77A1 ,//xmlq vee enable
#else
	0x70C , 0x77A1 ,//xmlq vee disable
#endif
	0x718 , 0x00000208 ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x120 , 0x5 ,
	0x124 , 0xF2D0384 ,
	0x128 , 0x301405 ,
	0x12C , 0x32 ,
	0x130 , 0x3C10 ,
	0x134 , 0x15 ,
	0x138 , 0xFF8000 ,
	0x13C , 0x0 ,
	0x140 , 0x10000 ,
#ifdef QL_VEE_ENABLE
	0x174 , 0xff , //xmlq vee
	0x418 , 0x555502ee ,
#endif
	0x174 , 0xff ,
	0x234 , 0xCA033E90 ,
	0x238 , 0x00001063 ,
	0x240 , 0x2861408B ,
	0x244 , 0x00130285 ,
	0x248 , 0x10630009 ,
	0x250 , 0x400B82A8 ,
	0x158 , 0x0 ,
	0x158 , 0x1 ,
	0x308 , 0xffffffff ,
	0x30C , 0x2204 ,
	0x310 , 0xffffff ,
	0x314 , 0x1ffff ,
	0x318 , 0x17 ,
	0x31C , 0xff ,
	0x320 , 0x5A00384 ,
	0x328 , 0x3 ,
	0x32C , 0x13 ,
	0x330 , 0x11 ,
	0x334 , 0x309 ,
	0x338 , 0xA ,
	0x33C , 0x6 ,
	0x340 , 0x4 ,
	0x344 , 0x12 ,
	0x350 , 0x598 ,
	0x354 , 0x4 ,
	0x358 , 0x3 ,
	0x35C , 0x0 ,
	0x364 , 0x28000F ,
	0x368 , 0x4 ,
	0x36C , 0x07040800 ,
	0x370 , 0x07051101 ,
	0x374 , 0x1 ,
	0x378 , 0xC2033810 ,
	0x37C , 0x00000060 ,
	0x380 , 0x82E86030 ,
	0x384 , 0x28614088 ,
	0x388 , 0x00110285 ,
	0x38C , 0x10600008 ,
	0x394 , 0x400882A8 ,
	0x608 , 0x50F ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x300 , 0x1 ,
#elif 1 //76.69MHz
0x700 , 0x2CF00040 ,
0x704 , 0x1F00BF ,
0x70C , 0x00004600 ,
0x710 , 0xCD1082 ,
0x714 , 0x0 ,
0x718 , 0x00000108 ,
0x71C , 0xA8002F ,
0x720 , 0x0 ,
0x154 , 0x00000000 ,
0x154 , 0x80000000 ,
0x700 , 0x2CF00040 ,
0x70C , 0x77B1 ,
0x718 , 0x00000208 ,
0x154 , 0x00000000 ,
0x154 , 0x80000000 ,
0x120 , 0x5 ,
0x124 , 0xF2D0384 ,
0x128 , 0x301405 ,
0x12C , 0x32 ,
0x130 , 0x3C10 ,
0x134 , 0x15 ,
0x138 , 0xFF8000 ,
0x13C , 0x0 ,
0x140 , 0x10000 ,
0x174 , 0xff ,
0x234 , 0xCA033E90 ,
0x238 , 0x00001063 ,
0x240 , 0x2861408B ,
0x244 , 0x00130285 ,
0x248 , 0x10630009 ,
0x250 , 0x400B82A8 ,
0x158 , 0x0 ,
0x158 , 0x1 ,
0x308 , 0xffffffff ,
0x30C , 0x2204 ,
0x310 , 0xffffff ,
0x314 , 0x1ffff ,
0x318 , 0x17 ,
0x31C , 0xff ,
0x320 , 0x5A00384 ,
0x328 , 0x3 ,
0x32C , 0x13 ,
0x330 , 0x11 ,
0x334 , 0x2EF ,
0x338 , 0xA ,
0x33C , 0x6 ,
0x340 , 0x4 ,
0x344 , 0x12 ,
0x350 , 0x598 ,
0x354 , 0x4 ,
0x358 , 0x3 ,
0x35C , 0x0 ,
0x364 , 0x28000F ,
0x368 , 0x4 ,
0x36C , 0x07040800 ,
0x370 , 0x07051101 ,
0x374 , 0x1 ,
0x378 , 0xC2033810 ,
0x37C , 0x00000060 ,
0x380 , 0x82E86030 ,
0x384 , 0x28614088 ,
0x388 , 0x00110285 ,
0x38C , 0x10600008 ,
0x394 , 0x400882A8 ,
0x608 , 0x50F ,
0x154 , 0x00000000 ,
0x154 , 0x80000000 ,
	0x300 , 0x1 ,	
#endif

//Porting: You may need this to restart the host after the init.......
	{0x300,0x00000000},
	{0x300,0x00000001},
	{0x304,0xffffffff},
	{0x204,0xffffffff},
	{0x148,0xffffffff},
};

static struct QL_VX_INIT_INFO ql_reset_setting[] = 
{
	{0x708,0xc0000000},
};

//Porting: put your DPI turn on seq. here. It's used after the panel on command....
static struct QL_VX_INIT_INFO ql_dpi_on_setting[] = 
{
	{0x300, 0x0},
#ifdef QL_VEE_ENABLE
	{0x710, 0xCD1092},//0xCD1012}, //xmlq vee
#else
	{0x710, 0xCD1012}, //xmlq vee disable
#endif
	{0x154, 0x0},
	{0x154, 0x80000000},
	{0x300, 0x1},
};


//Porting: put your standby seq. here.
// Stanby is disable
/*static struct QL_VX_INIT_INFO ql_standby_setting[] = 
{
	{0x700, 0x24000040},
	{0x704, 0x9F0009},
	{0x70C, 0x600},
	{0x710, 0xC0002},
	{0x714, 0x20},
	{0x718, 0x309},
	{0x154, 0x00000000},
	{0x154, 0x80000000},
	{0x248, 0x10630009},
	{0x378, 0xCA0F3C90},
	{0x37C, 0x00000060},
	{0x38C, 0x10630009},
	{0x608, 0x50F},
};*/

//#define MALATA_I2C
#define CONFIG_QLVX5A3B_I2C_CLK     400*1000

static struct ql_vx5a3b_t *ql_vx5a3b = NULL;
static struct i2c_client *i2c_quick_client = NULL;

static struct i2c_device_id i2c_quickvx_idtable[] = {
  { "quickvx", 0 },
  { }
};

MODULE_DEVICE_TABLE(i2c, i2c_quickvx_idtable);

static int __devinit i2c_quickvx_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int i2c_quickvx_remove(struct i2c_client *client);

static struct i2c_driver i2c_quickvx_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name  = "quickvx",
  },
  .id_table = i2c_quickvx_idtable,
  .probe    = i2c_quickvx_probe,
  .remove	= i2c_quickvx_remove, //xmlsh add
};

/////////////////////////
//xmlsh add
int ql_vx5a3b_gpio_init(void *data) {
	int ret = 0;
	struct reset_t *reset = &ql_vx5a3b->reset;
	struct power_t *lcd_en = &ql_vx5a3b->lcd_en;
    struct power_t *vcc_lcd = &ql_vx5a3b->vcc_lcd;

    QL_DBG("###ql_vx5a3b_gpio_init: reset pin=0x%x;lcd_en pin=0x%x;vcc_lcd pin=0x%x.\n",
        reset->reset_pin, lcd_en->enable_pin, vcc_lcd->enable_pin);

	if(reset->reset_pin > INVALID_GPIO) {
		ret = gpio_request(reset->reset_pin, "ql_vx5a3b_reset");
		if (ret != 0) {
			printk("%s: request ql_vx5a3b_RST_PIN error\n", __func__);
		} else {
#if OLD_RK_IOMUX
			if(reset->mux_name)
				rk30_mux_api_set(reset->mux_name, 0);
#endif
			gpio_direction_output(reset->reset_pin, !reset->effect_value);
		}
	}

	if(lcd_en->enable_pin > INVALID_GPIO) {
		ret = gpio_request(lcd_en->enable_pin, "ql_vx5a3b_lcd_en");
		if (ret != 0) {
			printk("%s: request ql_vx5a3b_lcd_en_PIN error\n", __func__);
		} else {
#if OLD_RK_IOMUX
			if(lcd_en->mux_name)
				rk30_mux_api_set(lcd_en->mux_name, 0);
#endif
			gpio_direction_output(lcd_en->enable_pin, !lcd_en->effect_value);
		}
	}

	if(vcc_lcd->enable_pin > INVALID_GPIO) {
		ret = gpio_request(vcc_lcd->enable_pin, "ql_vx5a3b_vcc_lcd");
		if (ret != 0) {
			printk("%s: request ql_vx5a3b_vcc_lcd_PIN error\n", __func__);
		} else {
#if OLD_RK_IOMUX
			if(vcc_lcd->mux_name)
				rk30_mux_api_set(vcc_lcd->mux_name, 0);
#endif
			gpio_direction_output(vcc_lcd->enable_pin, !vcc_lcd->effect_value);
		}
	}

	return 0;

}

int ql_vx5a3b_gpio_deinit(void *data) {
	struct reset_t *reset = &ql_vx5a3b->reset;
	struct power_t *lcd_en = &ql_vx5a3b->lcd_en;
    struct power_t *vcc_lcd = &ql_vx5a3b->vcc_lcd;
    
    if(reset->reset_pin > INVALID_GPIO) {
        gpio_direction_output(reset->reset_pin, reset->effect_value);
    	//gpio_direction_input(reset->reset_pin);
    	gpio_free(reset->reset_pin);
    }
    
    if(lcd_en->enable_pin > INVALID_GPIO) {
		gpio_direction_output(lcd_en->enable_pin, !lcd_en->effect_value);
    	//gpio_direction_input(lcd_en->enable_pin);
    	gpio_free(lcd_en->enable_pin);
    }
    if(vcc_lcd->enable_pin > INVALID_GPIO){
    	gpio_direction_input(vcc_lcd->enable_pin);
    	gpio_free(vcc_lcd->enable_pin);
    }
	return 0;
}

int ql_vx5a3b_reset(void *data) {
	int ret = 0;
	struct reset_t *reset = &ql_vx5a3b->reset;

    printk("############### ql_vx5a3b_reset.\n");
	if(reset->reset_pin <= INVALID_GPIO){
        printk("%s: ql_vx5a3b->reset invalid.\n", __func__);
		return -1;
	}
	gpio_set_value(reset->reset_pin, !reset->effect_value);
    msleep(30);
	gpio_set_value(reset->reset_pin, reset->effect_value);
	//if(reset->time_before_reset <= 0)
		msleep(20);
	//else
		//msleep(reset->time_before_reset);

	gpio_set_value(reset->reset_pin, !reset->effect_value);
	//if(reset->time_after_reset <= 0)
		msleep(5);
	//else
		//msleep(reset->time_after_reset);
	return ret;
}


int ql_vx5a3b_lcd_en_enable(void *data) {
	int ret = 0;
	struct power_t *lcd_en = &ql_vx5a3b->lcd_en;
	if(lcd_en->enable_pin > INVALID_GPIO) {
		gpio_set_value(lcd_en->enable_pin, lcd_en->effect_value);
	} 
	return ret;
}

int ql_vx5a3b_lcd_en_disable(void *data) {
	int ret = 0;
	struct power_t *lcd_en = &ql_vx5a3b->lcd_en;

	if(lcd_en->enable_pin > INVALID_GPIO) {
		gpio_set_value(lcd_en->enable_pin, !lcd_en->effect_value);
	} 
	return ret;
}

int ql_vx5a3b_vcc_lcd_enable(void *data) {
	int ret = 0;
	struct power_t *vcc_lcd = &ql_vx5a3b->vcc_lcd;
	if(vcc_lcd->enable_pin > INVALID_GPIO) {
		gpio_set_value(vcc_lcd->enable_pin, vcc_lcd->effect_value);
	} 
	return ret;
}

int ql_vx5a3b_vcc_lcd_disable(void *data) {
	int ret = 0;
	struct power_t *vcc_lcd = &ql_vx5a3b->vcc_lcd;

	if(vcc_lcd->enable_pin > INVALID_GPIO) {
		gpio_set_value(vcc_lcd->enable_pin, !vcc_lcd->effect_value);
	} 
	return ret;
}

u32 ql_vx5a3b_i2c_write(u16 addr, u32 val, u32 data_size) 
{
	struct i2c_msg msgs;
    int write_size = 0;
	int ret = -1;
	char buf[] = GEN_QL_CSR_WRITE;
    int i = 0;
    
    QL_DBG("###ql_vx5a3b_i2c_write enter: addr=0x%x; val=0x%x; data_size =0x%x.\n",addr, val, data_size);

	buf[5] = (u8)addr;  /* Address LS */
	buf[6] = (u8)(addr >> 8);  /* Address MS */

	buf[7] = val & 0xff;
	buf[8] = (data_size > 1) ? ((val >> 8) & 0xff) : 0;
	buf[9] = (data_size > 2) ? ((val >> 16) & 0xff) : 0;
	buf[10] = (data_size > 3) ? ((val >> 24) & 0xff) : 0;

	write_size = data_size + 7;

    for (i=0; i<write_size; i++){
        QL_DBG("###ql_vx5a3b_i2c_write: buf[%d]=0x%x;\n",i, buf[i]);
    }
	msgs.addr = i2c_quick_client->addr;
	msgs.flags = i2c_quick_client->flags;
	msgs.len = write_size;
	msgs.buf = buf;
	msgs.scl_rate = CONFIG_QLVX5A3B_I2C_CLK;
	msgs.udelay = i2c_quick_client->udelay;

	ret = i2c_transfer(i2c_quick_client->adapter, &msgs, 1);
	if(ret < 0)
		printk("%s:i2c_transfer fail =%d\n",__func__, ret);
	return ret;
}

u32 ql_vx5a3b_i2c_read(u16 addr, u32 *val, u32 data_size)
{
    struct i2c_msg msgs_first;
	struct i2c_msg msgs[2];
    u32 data;
	int ret = -1;
	char buf[] = GEN_QL_CSR_OFFSET_LENGTH;
    char rx[10] = {0};
    int write_size;
    int i = 0;
    
    QL_DBG("$$$ql_vx5a3b_i2c_read enter: addr=0x%x; data_size =0x%x.\n",addr, data_size);

    buf[5] = addr & 0xff;
    buf[6] = (addr >> 8) & 0xff;
    buf[7] = data_size & 0xff;
    buf[8] = (data_size >> 8) & 0xff;
    write_size = 9;

	msgs_first.addr = i2c_quick_client->addr;
	msgs_first.flags = i2c_quick_client->flags;
	msgs_first.len = write_size;
	msgs_first.buf = buf;
	msgs_first.scl_rate = CONFIG_QLVX5A3B_I2C_CLK;
	msgs_first.udelay = i2c_quick_client->udelay;

	ret = i2c_transfer(i2c_quick_client->adapter, &msgs_first, 1);
	if(ret < 0)
		printk("%s:i2c_transfer fail =%d\n",__func__, ret);

    // generic read request 0x24 to send generic read command 
    write_size = 4;
    buf[0] = CONTROL_BYTE_GEN;
    buf[1] =    0x24;  /* Data ID */
    buf[2] =    0x05;  /* Vendor Id 1 */
    buf[3] =    0x01;  /* Vendor Id 2 */
	
	msgs[0].addr = i2c_quick_client->addr;
	msgs[0].flags = i2c_quick_client->flags;
	msgs[0].len = write_size;
	msgs[0].buf = buf;
	msgs[0].scl_rate = CONFIG_QLVX5A3B_I2C_CLK;
	msgs[0].udelay = i2c_quick_client->udelay;

	msgs[1].addr = i2c_quick_client->addr;
	msgs[1].flags = i2c_quick_client->flags | I2C_M_RD;
	msgs[1].len = data_size;
	msgs[1].buf = rx;
	msgs[1].scl_rate = CONFIG_QLVX5A3B_I2C_CLK;
	msgs[1].udelay = i2c_quick_client->udelay;

	ret = i2c_transfer(i2c_quick_client->adapter, msgs, 2);
	if(ret < 0){
		printk("%s:i2c_transfer fail =%d\n",__func__, ret);
	}else{
		data = rx[0];
		if (data_size > 1) 
			data |= (rx[1] << 8);
		if (data_size > 2)
			data |= (rx[2] << 16) | (rx[3] << 24);
        
		*val = data;

        for (i=0; i<data_size; i++){
            QL_DBG("$$$ql_vx5a3b_i2c_read: rx[%d]=0x%x;\n",i, rx[i]);
        }
    	QL_DBG("ql_vx5a3b_i2c_read:data=0x%x\n", data);	
	}
	return ret;
}


u32 ql_vx5a3b_i2c_release(void) 
{
	struct i2c_msg msgs;
	int ret = -1;
	char buf[] = QL_I2C_RELEASE;
    
    QL_DBG("###ql_vx5a3b_i2c_release enter.\n");
	msgs.addr = i2c_quick_client->addr;
	msgs.flags = i2c_quick_client->flags;
	msgs.len = 1;
	msgs.buf = buf;
	msgs.scl_rate = CONFIG_QLVX5A3B_I2C_CLK;
	msgs.udelay = i2c_quick_client->udelay;

	ret = i2c_transfer(i2c_quick_client->adapter, &msgs, 1);
	if(ret < 0)
		printk("%s:i2c_transfer fail =%d\n",__func__, ret);
	return ret;
}
/////////////////////////

static int ql_i2c_read(u32 addr, u32 *val, u32 data_size) 
{
	u32 data;
    char buf[] = GEN_QL_CSR_OFFSET_LENGTH;
	char rx[10];
	int ret = -1;
	int write_size;

  	  buf[5] = addr & 0xff;
  	  buf[6] = (addr >> 8) & 0xff;
  	  buf[7] = data_size & 0xff;
  	  buf[8] = (data_size >> 8) & 0xff;

	  write_size = 9;

    if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}


// generic read request 0x24 to send generic read command 
    write_size = 4;

    buf[0] = CONTROL_BYTE_GEN;
    buf[1] =    0x24;  /* Data ID */
    buf[2] =    0x05;  /* Vendor Id 1 */
    buf[3] =    0x01;  /* Vendor Id 2 */

    if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
        printk(KERN_ERR
          "%s: i2c_master_send failed (%d)!\n", __func__, ret);
        return -1;
    }
	//return number of bytes or error
    if ((ret = i2c_master_recv( i2c_quick_client,
                     (char*)(&rx[0]),
                     data_size )) != data_size) {
		printk(KERN_ERR
		  "%s: i2c_master_recv failed (%d)!\n", __func__, ret);
		return -1;
	}

	data = rx[0];
	if (data_size > 1) 
		data |= (rx[1] << 8);
	if (data_size > 2)
		data |= (rx[2] << 16) | (rx[3] << 24);

	*val = data;

	QL_DBG("r0x%x=0x%x\n",addr,data);

	return 0;

}

static int ql_i2c_write(long addr, long val, int data_size)
{
	  int write_size;
	  int ret = -1;
	  char buf[] = GEN_QL_CSR_WRITE;

 		QL_DBG("w0x%lx=0x%lx\n",addr,val);

	buf[5] = (u8)addr;  /* Address LS */
	buf[6] = (u8)(addr >> 8);  /* Address MS */

	buf[7] = val & 0xff;
	buf[8] = (data_size > 1) ? ((val >> 8) & 0xff) : 0;
	buf[9] = (data_size > 2) ? ((val >> 16) & 0xff) : 0;
	buf[10] = (data_size > 3) ? ((val >> 24) & 0xff) : 0;

	write_size = data_size + 7;

	if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}

	return 0;

}

static int ql_i2c_release(void)
{
  int write_size;
  int ret = -1;
  char buf[] = QL_I2C_RELEASE;

    QL_DBG("+++\n");

	write_size = 1;

	if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}

	return 0;
}

int ql_i2c_4_master_send(const struct i2c_client *client, const char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = count;
	msg.buf = (char *)buf;
	msg.scl_rate = CONFIG_QLVX5A3B_I2C_CLK;
	msg.udelay = client->udelay;

	ret = i2c_transfer(adap, &msg, 1);
	return (ret == 1) ? count : ret;
}

int ql_i2c_4_master_recv(const struct i2c_client *client, char *buf, int count)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags | I2C_M_RD;
	msg.len = count;
	msg.buf = (char *)buf;
	msg.scl_rate = CONFIG_QLVX5A3B_I2C_CLK;
	msg.udelay = client->udelay;

	ret = i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;
}

int ql_i2c_4_mipi_panel_write(int dtype, int vc, int data_size,  char *ptr)
{
    char buf[QL_MIPI_PANEL_CMD_SIZE];
    int write_size;
    int i;
    int ret = -1;
    int dtype_int;

   if (data_size > (QL_MIPI_PANEL_CMD_SIZE-10)) {
	   QL_ERR("data size (%d) too big, adjust the QL_MIPI_PANEL_CMD_SIZE!\n", data_size);
	   return ret;
   }

   dtype_int = dtype;
   switch (dtype) {
   case DTYPE_GEN_WRITE:
   case DTYPE_GEN_LWRITE:
   case DTYPE_GEN_WRITE1:
       buf[0] = CONTROL_BYTE_GEN;
        dtype_int = (data_size == 1) ? DTYPE_GEN_WRITE :
                           (data_size == 2) ? DTYPE_GEN_WRITE1 : DTYPE_GEN_LWRITE;
      break;
   case DTYPE_DCS_WRITE:
   case DTYPE_DCS_LWRITE:
   case DTYPE_DCS_WRITE1:
       buf[0] =  CONTROL_BYTE_DCS;
       dtype_int = (data_size == 1) ? DTYPE_DCS_WRITE :
                           (data_size == 2) ? DTYPE_DCS_WRITE1 : DTYPE_DCS_LWRITE;
        break;
   default:
  		QL_DBG("Error unknown data type (0x%x).\n", dtype);
       break;
   }

	buf[1] = ((vc & 0x3) << 6) | dtype_int ;  /*vc,  Data ID */

    /* Copy data */
    for(i = 0; i < data_size; i++)
    {
        buf[2+i] = *ptr++;
    }

	write_size = data_size + 2;

    if ((ret = ql_i2c_4_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}
	return 0;
}
 

int ql_i2c_4_mipi_panel_read(int dtype, int vc, int data_size,  char *ptr)
{
	char buf[QL_MIPI_PANEL_CMD_SIZE];
	char *cmd = ptr;
	int write_size, read_size;
	int i,j;
	int ret = -1;

	if (data_size > (QL_MIPI_PANEL_CMD_SIZE-10)) {
		QL_ERR("data size (%d) too big, adjust the QL_MIPI_PANEL_CMD_SIZE!\n", data_size);
		return ret;
	}

	write_size = 3;

	switch (dtype) {
	case DTYPE_GEN_READ:
	case DTYPE_GEN_READ1: //		0x14	/* long read, 1 parameter */
	case DTYPE_GEN_READ2: //		0x24	/* long read, 2 parameter */
	case DTYPE_DCS_LWRITE: //	0x39	/* long write */
		buf[0] = CONTROL_BYTE_GEN;
		break;
	case DTYPE_DCS_READ:
		buf[0] = CONTROL_BYTE_DCS;
		break;
	default:
  		QL_ERR("Error unknown data type (0x%x).\n", dtype);
		break;
	}
	buf[1] = ((vc & 0x3) << 6) | dtype ;  /*vc,  Data ID */

	switch (dtype) {
	case DTYPE_GEN_READ2: //		0x24	/* long read, 2 parameter */
		write_size = 4;
		buf[2] = *cmd;
		buf[3] = 0;
		break;
	case DTYPE_DCS_LWRITE: //	0x39	/* long write */
		write_size = 5;
		buf[2] = 1;
		buf[3] = 0;
		buf[4] = *cmd;
		break;
	default:
		buf[2] = *cmd;
		break;
	}

    if ((ret = ql_i2c_4_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed 1 (%d)!\n", __func__, ret);
		return -1;
	}

	if (dtype == DTYPE_DCS_LWRITE) {
 
        write_size = 4;
        buf[0] = CONTROL_BYTE_GEN;
		buf[1] =    0x24;  /* Data ID */
		buf[2] =    0x05;  /* Vendor Id 1 */
		buf[3] =    0x01;  /* Vendor Id 2 */

		if ((ret = ql_i2c_4_master_send( i2c_quick_client,
						 (char*)(&buf[0]),
						 write_size )) != write_size) {
			printk(KERN_ERR
			  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
			return -1;
		}
	} 

	read_size = data_size -1;

    for (j = 10; j >= 0; j--) {
		//return number of bytes or error
		if ((ret = ql_i2c_4_master_recv( i2c_quick_client,
						 (char*)(&buf[0]),
						 read_size )) != read_size) {
			//need to retry here, may be the panel not fast enough to return data....
			QL_ERR("ql_i2c_system_read failed (%d)! Retrying.....\n", ret);
			msleep(16);
		} else {
			for(i = 0; i < read_size; i++)
			{
				*++ptr = buf[i];
				QL_DBG("buf[%d]=0x%x\n",i,buf[i]);
			}
			return 0;
		}
	}

	return -1;
}


static void ql_init_table(struct QL_VX_INIT_INFO *table, unsigned int count)
{
	unsigned int i = 0;
    u16 addr = 0;
    u32 data = 0;

    for(i = 0; i < count; i++) {
        addr = table[i].address;
        data = table[i].data;
#ifdef MALATA_I2C
        ql_vx5a3b_i2c_write(addr, data, 4);
#else
        ql_i2c_write(addr, data, 4);
#endif
#if 0
        /* if you are using 32kHz sys_clk, and seeing i2c error after the first 0x154 write,
        you may try enable this, and adjust the delay below, now it's 0.5 seconds */
        if ((addr == 0x154) && (data == 0x80000000))
        msleep(500);
#endif	
	}
}

static int quickvx_Init(void)
{
    u32 val = 0x8899;
#ifdef MALATA_I2C
    ql_vx5a3b_i2c_read(0x4fe, &val, 2); 
#else
    ql_i2c_read(0x4fe, &val, 2); 
#endif
    QL_DBG("VEE ID 0x%x\n", val);

    if(val != QL_ID){
        printk(KERN_ERR
			  "%s: QL chip id error,val=0x%x(need:0x2300)!\n", __func__, val);
        return -1;
    }
    ///ql_init_table(ql_reset_setting, sizeof(ql_reset_setting) / sizeof(struct QL_VX_INIT_INFO));
    ///msleep(100);
    //init code
    ql_init_table(ql_initialization_setting, sizeof(ql_initialization_setting) / sizeof(struct QL_VX_INIT_INFO));

    msleep(20);

#ifdef MALATA_I2C
    ql_vx5a3b_i2c_release();
#else
    ql_i2c_release();
#endif
    QL_DBG("quickvx_Init ok\n");

    msleep(10);
    return 0;
}

static int ql_vx_dpi_on(void)
{
	//Porting : Turn on dpi block. 
    ql_init_table(ql_dpi_on_setting, sizeof(ql_dpi_on_setting) / sizeof(struct QL_VX_INIT_INFO));
#ifdef MALATA_I2C
    ql_vx5a3b_i2c_release();
#else
    ql_i2c_release();
#endif
    return 0;
}

int mipi_dsi_send_dcs_packet(unsigned char regs[], int n) {
	///ql_i2c_4_mipi_panel_write(DTYPE_DCS_WRITE, QL_VX_LCD_VC, n, regs);
	return 0;
}

int mipi_dsi_read_dcs_packet(unsigned char *data, int n) {
	//DCS READ 
	ql_i2c_4_mipi_panel_read(DTYPE_DCS_READ, QL_VX_LCD_VC, n, data);
	return 0;
}

int ql_vx_power_up(void);
int ql_vx_get_id(void) {
	
	int id = -1;
    /*
	ql_vx_power_up();
#ifdef MALATA_I2C
    ql_vx5a3b_i2c_read(0x4fe, &id, 2); 
#else
    ql_i2c_read(0x4fe, &id, 2); 
 #endif
 */
    id = 0X2300;//xmlsh test
	QL_DBG("###ql_vx_get_id():id=0x%x\n", id);
	return id;
}

//xmlsh add
int ql_vx_init(void *array, int n){
    QL_DBG("###ql_vx_init()\n");
    ql_vx_dpi_on();
    return 0;
}

int ql_vx_power_up(void) {

	int ret = 0;
    
    QL_DBG("###ql_vx_power_up()\n");
    ///ql_vx5a3b_gpio_init(NULL);
    ql_vx5a3b->vcc_lcd.enable(NULL);
    ql_vx5a3b->lcd_en.enable(NULL);
    ql_vx5a3b->reset.do_reset(NULL);
    
    msleep(100);//xmlsh test
	quickvx_Init();
	return ret;
}

int ql_vx_power_down(void) {

	int ret = 0;
    
    QL_DBG("###ql_vx_power_down()\n");
	//Cut off all power supplies. TODO##########################
    ql_vx5a3b->lcd_en.disable(NULL);
    ql_vx5a3b->vcc_lcd.disable(NULL);
    ///ql_vx5a3b_gpio_deinit(NULL); //disable by xmlq
	return ret;
}

/*
 * For the function of QL_VX5A3B's VEE and DPO
 * QuickLogic provide the function
 * Porting and Modify by ArthurLin,20130724
 * VEE: Visual Enhancement Engine
 * DPO: Display Power Optimizer
 */
#ifdef QL_VEE_ENABLE
#include <linux/backlight.h>

#if 0 //The code from quicklogic
enum CABC {
	CABC_OFF,
	CABC_ON,
	CABC_MAX,
};
struct Vx5b3d_backlight_value {
	const unsigned int max;
	const unsigned int mid;
	const unsigned char low;
	const unsigned char dim;
};
static struct Vx5b3d_backlight_value backlight_table[1] = {
	{
		.max = 236,
		.mid = 140,
		.low = 10,
		.dim = 10,
	}
};

static int first_cnt = 1;
static int cabc = CABC_OFF;
#endif

#if 1  //Visual Enhancement Oriented
#define VEE_BRIGHTNESS_0  0
#define VEE_BRIGHTNESS_1  30
#define VEE_BRIGHTNESS_2  50
#define VEE_BRIGHTNESS_3  75
#define VEE_BRIGHTNESS_4  100
#define VEE_BRIGHTNESS_5  125
#define VEE_BRIGHTNESS_6  150
#define VEE_BRIGHTNESS_7  150
#define VEE_BRIGHTNESS_8  175
#define VEE_BRIGHTNESS_9  200

#define VEE_STRENGT_0 0
#define VEE_STRENGT_1 8
#define VEE_STRENGT_2 8
#define VEE_STRENGT_3 9
#define VEE_STRENGT_4 10
#define VEE_STRENGT_5 10
#define VEE_STRENGT_6 11
#define VEE_STRENGT_7 11
#define VEE_STRENGT_8 12
#define VEE_STRENGT_9 12

#else //Power Saving Oriented
#define VEE_BRIGHTNESS_0  0
#define VEE_BRIGHTNESS_1  30
#define VEE_BRIGHTNESS_2  30
#define VEE_BRIGHTNESS_3  50
#define VEE_BRIGHTNESS_4  75
#define VEE_BRIGHTNESS_5  75
#define VEE_BRIGHTNESS_6  100
#define VEE_BRIGHTNESS_7  100
#define VEE_BRIGHTNESS_8  125
#define VEE_BRIGHTNESS_9  125

#define VEE_STRENGT_0 0
#define VEE_STRENGT_1 7
#define VEE_STRENGT_2 8
#define VEE_STRENGT_3 9
#define VEE_STRENGT_4 10
#define VEE_STRENGT_5 10
#define VEE_STRENGT_6 11
#define VEE_STRENGT_7 11
#define VEE_STRENGT_8 12
#define VEE_STRENGT_9 12
#endif


#define V5D3BX_VEESTRENGHT		0x00001f07
#define V5D3BX_VEEDEFAULTVAL		7
#define V5D3BX_DEFAULT_STRENGHT		10
#define V5D3BX_DEFAULT_LOW_STRENGHT	11
#define V5D3BX_DEFAULT_HIGH_STRENGHT	12
#define V5D3BX_MAX_STRENGHT		15

#define V5D3BX_CABCBRIGHTNESSRATIO	815

#define MIN_BRIGHTNESS			30
#define MAX_BRIGHTNESS_LEVEL		255
#define MID_BRIGHTNESS_LEVEL		195
#define LOW_BRIGHTNESS_LEVEL		19 //changed by zss
#define DIM_BRIGHTNESS_LEVEL		20
#define DEFAULT_BRIGHTNESS		MID_BRIGHTNESS_LEVEL

int vx5a3b_update_brightness(struct backlight_device *bl)
{
	//struct Vx5b3d_backlight_value *pwm = &backlight_table[0];

	int brightness = bl->props.brightness;
	
	int vx5a3b_brightness = 0;
	u32 vee_strenght = 0;
	u32 vee_strenght_default = 0;
	int ret = 0;
    QL_DBG("%s\n",__func__);		
	//QL_DBG("sys brightness = %d\n", brightness);
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

//	if (fbi_global->active == 0)
//		return ret;
	/*
	register 0x160
	register 0x164
			  value of 0x164
	---> duty ration = -------------
			  value of 0x160
	*/

	/* brightness tuning*/
	if (brightness > MAX_BRIGHTNESS_LEVEL)
		brightness = MAX_BRIGHTNESS_LEVEL;

	if (brightness < MIN_BRIGHTNESS)
		brightness = MIN_BRIGHTNESS;

	/*The logic is used by a070pan01 of malata*/
	switch(brightness){
		case 0:
			vx5a3b_brightness = brightness;
			vee_strenght = VEE_STRENGT_0;
		case 1 ... 30:
			vx5a3b_brightness = brightness;
            vee_strenght = VEE_STRENGT_1;
			break;
		case 31 ... 75:
			vx5a3b_brightness = VEE_BRIGHTNESS_1 + \
				(brightness - VEE_BRIGHTNESS_1) * (VEE_BRIGHTNESS_2 - VEE_BRIGHTNESS_1) / (75 - VEE_BRIGHTNESS_1);
            vee_strenght = VEE_STRENGT_2;
			break;
		case 76 ... 100:
			vx5a3b_brightness = VEE_BRIGHTNESS_2 + \
				(brightness - VEE_BRIGHTNESS_2) * (VEE_BRIGHTNESS_3 - VEE_BRIGHTNESS_2) / (100 - VEE_BRIGHTNESS_2);
            vee_strenght = VEE_STRENGT_3;
			break;
		case 101 ... 125:
			vx5a3b_brightness = VEE_BRIGHTNESS_3 + \
				(brightness - VEE_BRIGHTNESS_3) * (VEE_BRIGHTNESS_4 - VEE_BRIGHTNESS_3) / (125 - VEE_BRIGHTNESS_3);
            vee_strenght = VEE_STRENGT_4;
			break;
		case 126 ... 150:
			vx5a3b_brightness = VEE_BRIGHTNESS_4 + \
				(brightness - VEE_BRIGHTNESS_4) * (VEE_BRIGHTNESS_5 - VEE_BRIGHTNESS_4) / (150 - VEE_BRIGHTNESS_4);
            vee_strenght = VEE_STRENGT_5;
			break;
		case 151 ... 175:
			vx5a3b_brightness = VEE_BRIGHTNESS_5 + \
				(brightness - VEE_BRIGHTNESS_5) * (VEE_BRIGHTNESS_6 - VEE_BRIGHTNESS_5) / (175 - VEE_BRIGHTNESS_5);
            vee_strenght = VEE_STRENGT_6;
			break;
		case 176 ... 200:
			vx5a3b_brightness = VEE_BRIGHTNESS_6 + \
				(brightness - VEE_BRIGHTNESS_6) * (VEE_BRIGHTNESS_7 - VEE_BRIGHTNESS_6) / (200 - VEE_BRIGHTNESS_6);
            vee_strenght = VEE_STRENGT_7;
			break;
		case 201 ... 225:
			vx5a3b_brightness = VEE_BRIGHTNESS_7 + \
				(brightness - VEE_BRIGHTNESS_7) * (VEE_BRIGHTNESS_8 - VEE_BRIGHTNESS_7) / (225 - VEE_BRIGHTNESS_7);
            vee_strenght = VEE_STRENGT_8;
			break;
		case 226 ... 255:
			vx5a3b_brightness = VEE_BRIGHTNESS_8 + \
				(brightness - VEE_BRIGHTNESS_8) * (VEE_BRIGHTNESS_9 - VEE_BRIGHTNESS_8) / (255 - VEE_BRIGHTNESS_8);
            vee_strenght = VEE_STRENGT_9;
			break;
        default:
			vx5a3b_brightness = brightness;
            vee_strenght = VEE_STRENGT_1;
			break;
	}
    vee_strenght = V5D3BX_VEESTRENGHT | (vee_strenght << 27);

#if 0 //The code from quicklogic
	if (brightness >= MID_BRIGHTNESS_LEVEL) {
		vx5a3b_brightness  = (brightness - MID_BRIGHTNESS_LEVEL) *
		(pwm->max - pwm->mid) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + pwm->mid;
	} else if (brightness >= LOW_BRIGHTNESS_LEVEL) {
		vx5a3b_brightness  = (brightness - LOW_BRIGHTNESS_LEVEL) *
		(pwm->mid - pwm->low) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + pwm->low;
	} else if (brightness >= DIM_BRIGHTNESS_LEVEL) {
		vx5a3b_brightness  = (brightness - DIM_BRIGHTNESS_LEVEL) *
		(pwm->low - pwm->dim) / (LOW_BRIGHTNESS_LEVEL-DIM_BRIGHTNESS_LEVEL) + pwm->dim;
	} else if (brightness > 0)
		vx5a3b_brightness  = pwm->dim;
	else {
		vx5a3b_brightness = 0;
		QL_DBG("brightness = [%d]: vx5a3b_brightness = [%d]\n",\
			brightness,vx5a3b_brightness);	
	}

	if (cabc) {
		QL_DBG("vx5a3b cabc [%d]..autobrightness=[%d]! \n",\
			cabc,brightness);

		switch (vx5a3b_brightness) {

			case   0 ... 34:
				vee_strenght_default = V5D3BX_DEFAULT_STRENGHT;				
				break;
			case  35 ... 152:
				vee_strenght_default = V5D3BX_DEFAULT_LOW_STRENGHT;
				break;
			case 153 ... 255:
				vee_strenght_default = V5D3BX_DEFAULT_HIGH_STRENGHT;
				break;	
			default:
				vee_strenght_default = V5D3BX_DEFAULT_STRENGHT;
		}

		if (brightness >= 5)
			vee_strenght_default = brightness;

		vee_strenght = V5D3BX_VEESTRENGHT | ((vee_strenght_default) << 27);
		vx5a3b_brightness = (vx5a3b_brightness * V5D3BX_CABCBRIGHTNESSRATIO) / 1000;
	}	else {
		vee_strenght = V5D3BX_VEESTRENGHT | (V5D3BX_VEEDEFAULTVAL << 27);
	}
	
	/* brightness setting from platform is from 0 to 255 */

	//QL_DBG("first_cnt = %d, vee_strength = 0x%x \n", first_cnt, vee_strenght);
	
	if(first_cnt){
	       
	       ret |= ql_i2c_write(0x160, 0xff, 4);    
	       ret |= ql_i2c_write(0x164, vx5a3b_brightness, 4);       
	       ret |= ql_i2c_write(0x138, 0x3fff0000, 4);
	       ret |= ql_i2c_write(0x15c, 0x5, 4);
	       first_cnt = 0;
	}else{
	       ret |= ql_i2c_write(0x164, vx5a3b_brightness, 1); 
	}
#endif
	QL_DBG("  sys brightness = %d, vx brightness = %d, vee_strength = 0x%x \n", \
		brightness, vx5a3b_brightness, vee_strenght);

	ret |= ql_i2c_write(0x400, vee_strenght, 4);

	if (ret < 0)
		QL_DBG("ql_i2c_write fail [%d] ! \n",ret);

	return vx5a3b_brightness;
}
EXPORT_SYMBOL(vx5a3b_update_brightness);

static ssize_t vx5a3d_Regread_store(struct device *dev, struct
device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = 0;
	u32 ret_value = 0;	
	int rc = 0;

	//if (fbi_global->active == 0)
		//return rc;

	rc = strict_strtoul(buf, (unsigned int) 0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		ql_i2c_read(value,&ret_value,4);
		QL_DBG("vx5b3d_Regread_register[0x%x]..return=[0x%x]\n",\
			value, ret_value);

		return size;
	}
}
static DEVICE_ATTR(vx5a3d_Regread, 0664,NULL, vx5a3d_Regread_store);

static ssize_t vee_strenght_store(struct device *dev, struct
device_attribute *attr, const char *buf, size_t size)
{
    static int vee_strenght;
	int value;
	u32 vee_value = 0x00001f07;	
	int rc;
		
	rc = strict_strtoul(buf, (unsigned int) 0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		QL_DBG("vee_strenght_store[0x400] - %d, %d\n", \
			vee_strenght, value);

		if (vee_strenght!= value) {
			vee_strenght = value;
			vee_value = vee_value | (value << 27);
			ql_i2c_write(0x400, vee_value, 4);
			ql_i2c_release();
			pr_info("vee_strenght value [0x%x]\n",vee_value);			
		}

		return size;
	}
}
static DEVICE_ATTR(vee_strenght, 0664,NULL, vee_strenght_store);
#endif
/*End of function VX5A3B VEE*/

static struct mipi_dsi_ops ql_vx_ops = {
	.id = QL_ID,
	.name = "ql_vx5a3b",
	.get_id = ql_vx_get_id,
	.dsi_init = ql_vx_init,//xmlsh add
	.dsi_set_regs = NULL,
	.dsi_send_dcs_packet = mipi_dsi_send_dcs_packet,
	.dsi_read_dcs_packet = mipi_dsi_read_dcs_packet,
	.power_up = ql_vx_power_up,
	.power_down = ql_vx_power_down,
	
};

static int __devinit i2c_quickvx_probe(struct i2c_client *client,
                              const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    int ret = 0;

    QL_DBG("i2c_quickvx_probe\n");
    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
        dev_warn(&adapter->dev,
        	 "I2C-Adapter doesn't support I2C_FUNC_I2C\n");
        ///return -EIO;
    }
    QL_DBG("%s probing adapter: %s, address: 0x%x\n", QLVX_DRIVER_VER, adapter->name, client->addr);

	ql_vx5a3b = (struct ql_vx5a3b_t *)client->dev.platform_data;
    if(!ql_vx5a3b) {
    	ret = -1;
    	printk("%s:%d ql_vx5a3b is null\n", __func__, __LINE__);
    	///return ret;
    }
    
    i2c_quick_client = client;
    if(!i2c_quick_client) {
    	ret = -1;
    	printk("%s:%d i2c_quick_client is null\n", __func__, __LINE__);
    	///return ret;
    }
    
//#ifndef MALATA_I2C
    //i2c_quick_client->timing = 400;
//#endif
    if(!ql_vx5a3b->gpio_init)
    	ql_vx5a3b->gpio_init = ql_vx5a3b_gpio_init;
    if(!ql_vx5a3b->gpio_deinit)
    	ql_vx5a3b->gpio_deinit = ql_vx5a3b_gpio_deinit;

    if(!ql_vx5a3b->lcd_en.enable)
    	ql_vx5a3b->lcd_en.enable = ql_vx5a3b_lcd_en_enable; 

    if(!ql_vx5a3b->lcd_en.disable)
    	ql_vx5a3b->lcd_en.disable = ql_vx5a3b_lcd_en_disable;

    if(!ql_vx5a3b->reset.do_reset)
    	ql_vx5a3b->reset.do_reset = ql_vx5a3b_reset;

    if(!ql_vx5a3b->vcc_lcd.enable)
    	ql_vx5a3b->vcc_lcd.enable = ql_vx5a3b_vcc_lcd_enable;    
    if(!ql_vx5a3b->vcc_lcd.disable)
    	ql_vx5a3b->vcc_lcd.disable = ql_vx5a3b_vcc_lcd_disable;

/*
    if(!ql_vx5a3b->vrgb_io.enable)
    	ql_vx5a3b->vrgb_io.enable = ql_vx5a3b_vrgb_io_enable;    
    if(!ql_vx5a3b->vrgb_io.disable)
    	ql_vx5a3b->vrgb_io.disable = ql_vx5a3b_vrgb_io_disable;
    
    if(!ql_vx5a3b->vdd_log.enable)
    	ql_vx5a3b->vdd_log.enable = ql_vx5a3b_vdd_log_enable;    
    if(!ql_vx5a3b->vdd_log.disable)
    	ql_vx5a3b->vdd_log.disable = ql_vx5a3b_vdd_log_disable;
*/
    if(!ql_vx5a3b->power_up)
    	ql_vx5a3b->power_up = ql_vx_power_up;  
    if(!ql_vx5a3b->power_down)
    	ql_vx5a3b->power_down = ql_vx_power_down;  	
    if(!ql_vx5a3b->dpi_on)
    	ql_vx5a3b->dpi_on = ql_vx_dpi_on;  	

	ql_vx5a3b_gpio_init(NULL);

#ifdef QL_VEE_ENABLE
	if (device_create_file(&i2c_quick_client->dev, &dev_attr_vx5a3d_Regread) < 0)
		QL_DBG("%s:Failed to create vx5b3d_Regread\n", __func__);

	if (device_create_file(&i2c_quick_client->dev, &dev_attr_vee_strenght) < 0)
		QL_DBG("%s:Failed to create device file for vee_strenght!\n", __func__);
#endif
    return ret;
}

static int i2c_quickvx_remove(struct i2c_client *client)
{
    i2c_quick_client = NULL;
    ql_vx5a3b = NULL;
    return 0;
}

static int __init ql_vx_module_init(void)
{
    int ret = 0;
	QL_DBG("%s+++\n", QLVX_DRIVER_VER);

	ret= i2c_add_driver(&i2c_quickvx_driver);	
	QL_DBG("i2c_add_driver ret = %d\n", ret);
	if (ret){
		printk(KERN_ERR "%s: i2c_add_driver failed!\n", __func__);
	}
	//if(!ql_vx5a3b || !i2c_quick_client)
		//return -1;

	ret = register_dsi_ops(&ql_vx_ops);
	if(ret){
		printk(KERN_ERR "%s: register_dsi_ops failed!\n", __func__);
	}
	if(ql_vx5a3b->id > 0)
		ql_vx_ops.id = ql_vx5a3b->id;
    
    return 0;
}

static void __exit ql_vx_module_exit(void)
{
	del_dsi_ops(&ql_vx_ops);
	i2c_del_driver(&i2c_quickvx_driver);

}
subsys_initcall_sync(ql_vx_module_init);

module_exit(ql_vx_module_exit);
