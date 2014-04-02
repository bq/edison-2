/*
 * Copyright (C) Quicklogic 2013
 *
 * Quicklogic VX5A3B RGB-to-MIPI display driver for Panel A070PAN01
 * Author : Sunny
 *
 * * Based on lcd_tl5001_mipi.c
 * * for Rockchip.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
/*
 * Copyright (C) 2012 ROCKCHIP, Inc.
 *
 * author: hhb@rock-chips.com
 * creat date: 2012-04-19
 * route:drivers/video/display/screen/lcd_hj050na_06a.c
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _LCD_A070PAN01_MIPI_1440X900__
#define _LCD_A070PAN01_MIPI_1440X900__

//#define QLVX_DRIVER_VER "lcd_ql_a070pan01_mipi ver 1.0"
/* Quicklogic VX5A3B RGB-to-MIPI for Panel A070PAN01
	20130516 - rev 1.0 Modified from lcd_tl5001_mipi.c
*/
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/rk_fb.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/rk_screen.h>
#include <linux/ktime.h>
#include "../transmitter/ql_vx5a3b_rc.h"
#include "../transmitter/mipi_dsi.h"

//Porting: panel parameter here.
/* Base */
#define SCREEN_TYPE	    	SCREEN_RGB
#define OUT_FACE	    	OUT_P888
#define LVDS_FORMAT      	LVDS_8BIT_3


#define DCLK	         92900000//96555555// //88000000//  //  in fact it is 61384615

#define LCDC_ACLK        300000000

/* Timing */
#define H_PW			60
#define H_BP			50
#define H_VD			900
#define H_FP			50

#define V_PW			5
#define V_BP			10
#define V_VD			1440
#define V_FP			5


#define LCD_WIDTH       94//62    //uint mm the lenth of lcd active area
#define LCD_HEIGHT      151//111
/* Other */
#define VSYNC_POL		0
#define HSYNC_POL		VSYNC_POL
#define DCLK_POL		1
#define DEN_POL         0 //positive
#define SWAP_RB			0
#define SWAP_RG		0
#define SWAP_GB		0
#define CONFIG_DEEP_STANDBY_MODE 0
#define CONFIG_VX5A3B_INIT_MODE     0   //1:ARRAY  0:FUNCTION

#define RK_SCREEN_INIT  1           //this screen need to init

#define dsi_send_dcs_packet(data) 	dsi_send_dcs_packet(data, ARRAY_SIZE(data))
#define dsi_hs_start(data)			dsi_hs_start(data, ARRAY_SIZE(data))
#define dsi_read_dcs_packet(data) 	dsi_read_dcs_packet(data, ARRAY_SIZE(data))

#if defined(RK_SCREEN_INIT)
static struct rk29lcd_info *gLcd_info = NULL;
#ifndef QL_VX_INIT_EXTERNAL
//Porting: panel on command here.
static char lcm_init_cmd_0[] = {0x01}; // Software Reset	
static char lcm_init_cmd_1[] = {0x11}; // Sleep Out	
static char lcm_init_cmd_2[] = {0x29}; // display on	
static char lcm_suspend_cmd_1[1] = {0x28}; // display off
static char lcm_suspend_cmd_2[1] = {0x10}; // stand-by on	
#endif

int rk_lcd_init(void) {

    printk("xmlq===%s!!! \n", __func__);//xmlq debug
    //power on
    dsi_power_up();
		
	if(gLcd_info)
        gLcd_info->io_init();

 #ifndef QL_VX_INIT_EXTERNAL     
    //lcd init
	#if 1
    //dsi_send_dcs_packet(lcm_init_cmd_0);
    msleep(150);
    dsi_send_dcs_packet(lcm_init_cmd_1);
    msleep(1);
    dsi_send_dcs_packet(lcm_init_cmd_2);
    msleep(1);
	#endif 
    dsi_init(NULL, 0);//lcd_ql_vx5a3b->dpi_on();
#endif   
	msleep(10);

    return 0;
};



int rk_lcd_standby(u8 enable) {
    printk("xmlq===%s!!! \n", __func__);//xmlq debug

		
	if(enable) {
	    printk("suspend lcd\n");
#ifndef QL_VX_INIT_EXTERNAL
	    //Porting: panel sleep command here.
	    #if 1 //no need to sent MIPI command, power off directly
	dsi_send_dcs_packet(lcm_suspend_cmd_1);
	dsi_send_dcs_packet(lcm_suspend_cmd_2);
	    #endif
#endif
		//power down
	    if(gLcd_info)
	        gLcd_info->io_deinit();
	        
        dsi_power_off();
	
	} else {
		/*below is changeable*/
		rk_lcd_init();
		//printk("++++++++++++++++%s:%d\n", __func__, __LINE__);
	
	}

    return 0;
};
#endif
#endif  

