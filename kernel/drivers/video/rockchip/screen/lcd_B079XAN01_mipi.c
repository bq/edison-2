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
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/rk_fb.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/rk_screen.h>
#include "../transmitter/mipi_dsi.h"

/* Base */
#define SCREEN_TYPE		SCREEN_RGB
#define LVDS_FORMAT		LVDS_8BIT_1
#define OUT_FACE	    OUT_D888_P666


#define DCLK	         65*1000000//70000000    
#define LCDC_ACLK        300000000   //29 lcdc axi DMA

/* Timing */
#define H_PW			64
#define H_BP			56
#define H_VD			768
#define H_FP			60

#define V_PW			50
#define V_BP			30
#define V_VD			1024
#define V_FP			36

#define LCD_WIDTH       119    //uint mm the lenth of lcd active area
#define LCD_HEIGHT      159
/* Other */
#define DCLK_POL		1  //  1  hhs
#define DEN_POL			0
#define VSYNC_POL		0
#define HSYNC_POL		0

#define SWAP_RB			0
#define SWAP_RG			0
#define mipi_dsi_init(data) 				dsi_set_regs(data, ARRAY_SIZE(data))
#define mipi_dsi_send_dcs_packet(data) 		dsi_send_dcs_packet(data, ARRAY_SIZE(data))
#define mipi_dsi_post_init(data)			dsi_set_regs(data, ARRAY_SIZE(data))
#define SWAP_GB			0 
#define data_lane  4
static struct rk29lcd_info *gLcd_info = NULL;
int lcd_init(void);
int lcd_standby(u8 enable);


static unsigned int pre_initialize[] = {
	0x00B10000 | ((V_PW & 0Xff) << 8) | (H_PW & 0Xff),
	0x00B20000 | (((V_BP+V_PW) & 0Xff) << 8) | ((H_BP+H_PW) & 0Xff),
	//0x00B20000 | ((V_BP & 0Xff) << 8) | (H_BP & 0Xff),
	0x00B30000 | ((V_FP & 0Xff) << 8) | (H_FP & 0Xff),
	0x00B40000 | H_VD,
	0x00B50000 | V_VD,
	0x00B60000 | (VPF_18BPPL) | (VM_BM << 2),     // burst mode 24bits
	
	0x00de0000 | (data_lane -1),
	0x00d60004,
	
	0x00B90000,
	0x00bac016,   //pll    //480MHz    14
	0x00Bb0008,
	0x00B90001,	
	0x00c40001,

};

static unsigned int post_initialize[] = {
	0x00B90000,
	
//	0x00ba8008,   //pll
//	0x00Bb0002,	
//	0x00B7032b,
	0x00B7030b,
	0x00B90001,
	0x00B80000,
	0x00BC0000,
	0x00c00100,      //software reset ssd2828
};

static unsigned char dcs_exit_sleep_mode_arry[] = {0x11};
static unsigned char dcs_set_diaplay_on[] = {0x29};
static unsigned char dcs_enter_sleep_mode_arry[] = {0x10};
static unsigned char dcs_set_diaplay_off[] = {0x28};


int lcd_io_init(void)
{
	int ret = 0;
	if(!gLcd_info)
		return -1;
	
	ret = gpio_request(gLcd_info->reset_pin, NULL);
	if (ret != 0) {
		gpio_free(gLcd_info->reset_pin);
		printk("%s: request LCD_RST_PIN error\n", __func__);
		return -EIO;
	}
	
	gpio_direction_output(gLcd_info->reset_pin, !GPIO_LOW);
	
	return ret;
}

int lcd_io_deinit(void)
{

	int ret = 0;
	if(!gLcd_info)
		return -1;
	gpio_direction_input(gLcd_info->reset_pin);
	gpio_free(gLcd_info->reset_pin);
	return ret;
	
}


int lcd_reset(void) {
	
	int ret = 0;
	if(!gLcd_info)
		return -1;
	gpio_set_value(gLcd_info->reset_pin, GPIO_LOW);
	msleep(10);
	gpio_set_value(gLcd_info->reset_pin, !GPIO_LOW);
	msleep(2);
	return ret;
	
}


int lcd_init(void)
{	

	lcd_reset();	
	msleep(10);
   	mipi_dsi_init(pre_initialize);
   	
	mipi_dsi_send_dcs_packet(dcs_exit_sleep_mode_arry);
	msleep(100);
	mipi_dsi_send_dcs_packet(dcs_set_diaplay_on);
	msleep(1);
	mipi_dsi_post_init(post_initialize);   


    return 0;

}



int lcd_standby(u8 enable)
{
	if(enable) {

		printk("lcd_standby...\n");
		mipi_dsi_send_dcs_packet(dcs_set_diaplay_off);
		msleep(2);		
		mipi_dsi_send_dcs_packet(dcs_enter_sleep_mode_arry);
		msleep(150);
		dsi_power_off();
		gpio_set_value(gLcd_info->reset_pin, 0);
		msleep(200);
		
	} else {
		dsi_power_up();
		lcd_init();
	}

    return 0;
}
EXPORT_SYMBOL(lcd_standby);
void set_lcd_info(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info )
{
    /* screen type & face */
    screen->type = SCREEN_TYPE;
    screen->face = OUT_FACE;

    /* Screen size */
    screen->x_res = H_VD;
    screen->y_res = V_VD;

    screen->width = LCD_WIDTH;
    screen->height = LCD_HEIGHT;

    /* Timing */
    screen->lcdc_aclk = LCDC_ACLK;
    screen->pixclock = DCLK;
	screen->left_margin = H_BP;
	screen->right_margin = H_FP;
	screen->hsync_len = H_PW;
	screen->upper_margin = V_BP;
	screen->lower_margin = V_FP;
	screen->vsync_len = V_PW;

	/* Pin polarity */
	screen->pin_hsync = 0;
	screen->pin_vsync = 0;
	screen->pin_den = 0;
	screen->pin_dclk = DCLK_POL;
	screen->pin_den = DEN_POL;

	/* Swap rule */
    screen->swap_rb = 0;
    screen->swap_rg = 0;
    screen->swap_gb = 0;
    screen->swap_delta = 0;
    screen->swap_dumy = 0;

    /* Operation function*/
    screen->init = lcd_init;
    screen->standby = lcd_standby;

    if(lcd_info)
        gLcd_info = lcd_info;
    lcd_io_init();    
    dsi_probe_current_chip();
}

size_t get_fb_size(void)
{
	size_t size = 0;
	#if defined(CONFIG_THREE_FB_BUFFER)
		size = ((H_VD)*(V_VD)<<2)* 3; //three buffer
	#else
		size = ((H_VD)*(V_VD)<<2)<<1; //two buffer
	#endif
	return ALIGN(size,SZ_1M);
}

