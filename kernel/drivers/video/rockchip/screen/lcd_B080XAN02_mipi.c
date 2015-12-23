#ifndef __LCD_B080XAN02__
#define __LCD_B080XAN02__

#if defined(CONFIG_MIPI_DSI)
#include "../transmitter/mipi_dsi.h"
#endif
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <linux/delay.h>

#if  defined(CONFIG_RK610_LVDS) || defined(CONFIG_RK616_LVDS)
#define SCREEN_TYPE	    	SCREEN_LVDS
#else
#define SCREEN_TYPE	    	SCREEN_MIPI
#endif
#define LVDS_FORMAT         0     //mipi lcd don't need it, so 0 would be ok.
#define OUT_FACE	    	OUT_P666  //OUT_P888


#define DCLK	          	67*1000000 //150*1000000
#define LCDC_ACLK         	300000000           //29 lcdc axi DMA ?¦Ì?¨º

/* Timing */
#define H_PW			 64  //16
#define H_BP			56 //40
#define H_VD			768 //1920
#define H_FP			60 //24

#define V_PW			14//50 //6
#define V_BP			30 // 26
#define V_VD			1024 //120
#define V_FP			36 //4

#define LCD_WIDTH         119 //204 
#define LCD_HEIGHT         159//	136
/* Other */
#if defined(CONFIG_RK610_LVDS) || defined(CONFIG_RK616_LVDS) || defined(CONFIG_MIPI_DSI)
#define DCLK_POL	1
#else
#define DCLK_POL	0
#endif
#define DEN_POL		0
#define VSYNC_POL	0
#define HSYNC_POL	0

#define SWAP_RB		0
#define SWAP_RG		0
#define SWAP_GB		0

#define RK_SCREEN_INIT 	1

/* about mipi */
#define MIPI_DSI_LANE 4
#define MIPI_DSI_HS_CLK 528*1000000 //1000*1000000

#if defined(RK_SCREEN_INIT)
static struct rk29lcd_info *gLcd_info = NULL;

int gTmp = 1;
//#define MIPI_RST_PIN RK30_PIN0_PD5
int rk_lcd_init(void) {

	int ret = 0;
	u8 dcs[16] = {0};
	if(dsi_is_active() != 1)
		return -1;
	/*below is changeable*/
	if(gLcd_info)
	{
		if(gTmp == 1)
		{
			ret = gpio_request(gLcd_info->reset_pin, "mipi rest pin");
			if(ret != 0)
			{
				gpio_free(gLcd_info->reset_pin);
				printk("mipi rest pin error\n");
				return -EIO;
			}
			ret = gpio_request(gLcd_info->lcd_en_pin, "mipi lcd en pin");
			if(ret != 0)
			{
				gpio_free(gLcd_info->lcd_en_pin);
				printk("mipi lcd en pin error\n");
				return -EIO;
			}
			gTmp++;
		}
		gpio_set_value(gLcd_info->lcd_en_pin, gLcd_info->lcd_en_pin_level);
		msleep(40);
		gpio_set_value(gLcd_info->reset_pin, GPIO_HIGH);
		msleep(20);
		gpio_set_value(gLcd_info->reset_pin, GPIO_LOW);
		msleep(20);
		gpio_set_value(gLcd_info->reset_pin, GPIO_HIGH);
		msleep(40);
		printk("++++++++++++++++%s:reset\n", __func__);
	}

	dsi_enable_hs_clk(1);

	dcs[0] = LPDT;
	dcs[1] = dcs_exit_sleep_mode; 
	dsi_send_dcs_packet(dcs, 2);
	msleep(2);
	dcs[0] = LPDT;
	dcs[1] = dcs_set_display_on;
	dsi_send_dcs_packet(dcs, 2);
	msleep(20);
   	//dsi_enable_command_mode(0);
	dsi_enable_video_mode(1);
	
	printk("++++++++++++++++%s:%d\n", __func__, __LINE__);
	return 0 ;
}

int giFlag = 1;
int rk_lcd_standby(u8 enable) {

	u8 dcs[16] = {0};
	if(dsi_is_active() != 1 )
		return -1;
		
	if(enable){
		
		giFlag = giFlag - 1;
		if( giFlag < 0 )
			return -1;
		
		/*below is changeable*/
		dcs[0] = LPDT;
		dcs[1] = dcs_set_display_off; 
		dsi_send_dcs_packet(dcs, 2);
		
		msleep(1);
		
		dcs[0] = LPDT;
		dcs[1] = dcs_enter_sleep_mode; 
		dsi_send_dcs_packet(dcs, 2);
		msleep(20);
		gpio_set_value(gLcd_info->reset_pin, GPIO_LOW);
		msleep(20);
		gpio_set_value(gLcd_info->lcd_en_pin, !(gLcd_info->lcd_en_pin_level));
		printk("++++enable++++++++++++%s:%d\n", __func__, __LINE__);
	} else {
		/*below is changeable*/
		giFlag = giFlag + 1 ;
		rk_lcd_init();
		printk("++++++++++++++++%s:%d\n", __func__, __LINE__);	
	}
	return 0;
}
#endif

#endif  
