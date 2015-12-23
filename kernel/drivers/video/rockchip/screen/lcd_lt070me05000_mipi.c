#ifndef __LCD_LD089WU1__
#define __LCD_LD089WU1__
#include <mach/gpio.h>
#include <mach/iomux.h>

#if defined(CONFIG_MIPI_DSI)
#include "../transmitter/mipi_dsi.h"
#endif

#define SCREEN_TYPE	    	SCREEN_MIPI
#define LVDS_FORMAT         0     //mipi lcd don't need it, so 0 would be ok.
#define OUT_FACE	    	OUT_P888

#define DCLK	          	148.5*1000000
#define LCDC_ACLK         	300000000           //29 lcdc axi DMA

/* Timing */
#define H_PW			12//8
#define H_BP			60//40
#define H_VD			1200 
#define H_FP			200//120

#define V_PW			2//8
#define V_BP			8
#define V_VD			1920
#define V_FP			8

#define LCD_WIDTH          	94
#define LCD_HEIGHT         	151
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
#define MIPI_DSI_HS_CLK 1000*1000000

#if defined(RK_SCREEN_INIT)
static struct rk29lcd_info *gLcd_info = NULL;
/*
//DCDC_EN
#define LCD_EN_PIN             RK30_PIN1_PA4//RK30_PIN0_PD7

//Vddp
#define VCC_LCD_EN_PIN         RK30_PIN0_PB0//RK30_PIN0_PB0

//IOVcc
#define LCD_PWR_EN_PIN         RK30_PIN0_PB4//RK30_PIN0_PA7

//lcd_rst
#define LCD_RESET_PIN          RK30_PIN0_PD5//RK30_PIN3_PD7

//LED_EN
#define LCDC_BL_PIN            RK30_PIN3_PD6//RK30_PIN3_PD4
#define LCDC_BL_VALUE          GPIO_LOW

//VCC_LED
#define VCC_LED_PIN            RK30_PIN0_PB0//RK30_PIN0_PB1
*/
int gTmp = 1;

void init_lcd_hw(void)
{
	int ret = 0;
	printk("%s\n",__FUNCTION__);

	if(gTmp == 1)
	{
		ret = gpio_request(LCD_PWR_EN_PIN, NULL);
		if(ret != 0)
		{
			gpio_free(LCD_PWR_EN_PIN);
			printk("LCD_PWR_EN_PIN pin error\n");
			return -EIO;
		}
		ret = gpio_request(VCC_LCD_EN_PIN, NULL);
		if(ret != 0)
		{
			gpio_free(VCC_LCD_EN_PIN);
			printk("VCC_LCD_EN_PIN error\n");
			return -EIO;
		}
		ret = gpio_request(LCD_EN_PIN, NULL);
		if(ret != 0)
		{
			gpio_free(LCD_EN_PIN);
			printk("LCD_EN_PIN error\n");
			return -EIO;
		}

		ret = gpio_request(LCD_RESET_PIN, NULL);
		if(ret != 0)
		{
			gpio_free(LCD_RESET_PIN);
			printk("LCD_RESET_PIN error\n");
			return -EIO;
		}
		ret = gpio_request(LCDC_BL_PIN, NULL);
		if(ret != 0)
		{
			gpio_free(LCDC_BL_PIN);
			printk("LCDC_BL_PIN error\n");
			return -EIO;
		}

		gTmp++;
	}

    gpio_direction_output(LCD_RESET_PIN, GPIO_LOW);
    msleep(10);
	gpio_direction_output(LCD_PWR_EN_PIN, GPIO_HIGH);
    msleep(15);
//	gpio_direction_output(VCC_LED_PIN, GPIO_LOW);
//	msleep(5);
	gpio_direction_output(VCC_LCD_EN_PIN, GPIO_LOW);
	msleep(10);
	gpio_direction_output(LCD_EN_PIN, GPIO_HIGH);
	msleep(20);
	gpio_direction_output(LCD_RESET_PIN, GPIO_HIGH);
	msleep(10);
    gpio_direction_output(LCD_RESET_PIN, GPIO_LOW);
    msleep(10);
    gpio_direction_output(LCD_RESET_PIN, GPIO_HIGH);
    msleep(10);
}

void exit_sleepmode(void)
{
	u8 dcs[16] = {0};
	int ret;
	printk("%s\n",__FUNCTION__);

	gpio_direction_output(LCD_EN_PIN, GPIO_HIGH);
	msleep(20);

	dcs[0] = LPDT;
	dcs[1] = 0x01;
	dsi_send_dcs_packet(dcs, 2);
	msleep(5);
	
	dcs[0] = LPDT;
	dcs[1] = 0x11;
	dsi_send_dcs_packet(dcs, 2);
	msleep(150);

	dcs[0] = LPDT;
	dcs[1] = 0xB0;
	dcs[2] = 0x00;
	dsi_send_packet(dcs, 3);
	msleep(2);

	dcs[0] = LPDT;
	dcs[1] = 0xD6;
	dcs[2] = 0x01;
	dsi_send_packet(dcs, 3);
	msleep(2);

	dcs[0] = LPDT;
	dcs[1] = 0xB3;
	dcs[2] = 0x14;
	dcs[3] = 0x08;
	dcs[4] = 0x00;
	dcs[5] = 0x22;
	dcs[6] = 0x00;
	dsi_send_packet(dcs, 7);//dsi_send_dcs_packet(dcs, 7);
	msleep(2);

	dcs[0] = LPDT;
	dcs[1] = 0xB4;
	dcs[2] = 0x0C;
	dsi_send_packet(dcs, 3);
	msleep(2);

	dcs[0] = LPDT;
	dcs[1] = 0xB6;
	dcs[2] = 0x3A;
	dcs[3] = 0xC3;
	dsi_send_packet(dcs, 4);
	msleep(2);

    dcs[0] = LPDT;
    dcs[1] = 0x2A;
    dcs[2] = 0x00;
    dcs[3] = 0x00;
    dcs[4] = 0x04;
    dcs[5] = 0xAF;
    dsi_send_dcs_packet(dcs, 6);
    msleep(2);

    dcs[0] = LPDT;
    dcs[1] = 0x2B;
    dcs[2] = 0x00;
    dcs[3] = 0x00;
    dcs[4] = 0x07;
    dcs[5] = 0x7F;
    dsi_send_dcs_packet(dcs, 6);
    msleep(2);
    
	dcs[0] = LPDT;
	dcs[1] = 0x51;
	dcs[2] = 0xA6;
	dsi_send_dcs_packet(dcs, 3);
	msleep(2);

	dcs[0] = LPDT;
	dcs[1] = 0x53;
	dcs[2] = 0x2C;
	dsi_send_dcs_packet(dcs, 3);
	msleep(2);

	dcs[0] = LPDT;
	dcs[1] = 0x3A;
	dcs[2] = 0x66;
	dsi_send_dcs_packet(dcs, 3);
	msleep(2);

	dcs[0] = LPDT;
	dcs[1] = 0x29;
	dsi_send_dcs_packet(dcs, 2);
	msleep(5);

	gpio_direction_output(LCDC_BL_PIN, GPIO_HIGH);
}

int rk_lcd_init(void)
{
	u8 dcs[16] = {0};
	printk("%s\n",__FUNCTION__);
	if(dsi_is_active() != 1)
	{
	    printk("dsi is active,return\n");
		return -1;
	}

	init_lcd_hw();
	/*below is changeable*/
	dsi_enable_hs_clk(1);
	dsi_enable_video_mode(0);
	dsi_enable_command_mode(1);
	exit_sleepmode();
	dcs[0] = LPDT;
	dcs[1] = dcs_exit_sleep_mode; 
	dsi_send_dcs_packet(dcs, 2);
	msleep(1);
	dcs[0] = LPDT;
	dcs[1] = dcs_set_display_on;
	dsi_send_dcs_packet(dcs, 2);
	msleep(10);
	dsi_enable_command_mode(0);
	dsi_enable_video_mode(1);
	printk("++++++++++++++++%s:%d\n", __func__, __LINE__);
};

void set_DSTB_mode(u8 enable)
{
	u8 dcs[16] = {0};

	printk("%s, enable=%d\n",__FUNCTION__,enable);
	if(enable)
	{ 
        dcs[0] = LPDT;
        dcs[1] = 0x28;
        dsi_send_dcs_packet(dcs, 2);
        msleep(20);
        
        dcs[0] = LPDT;
        dcs[1] = 0x10;
        dsi_send_dcs_packet(dcs, 2);
        msleep(80);
        gpio_direction_output(LCD_EN_PIN, GPIO_LOW);
	    msleep(20);
	}
    else
	{
        exit_sleepmode();
	}   
}


int rk_lcd_standby(u8 enable) 
{
	u8 dcs[16] = {0};
	
	printk("%s\n",__FUNCTION__);

	if(dsi_is_active() != 1)
		return -1;
		
	if(enable) {
		dsi_enable_video_mode(0);
		dsi_enable_command_mode(1);
		/*below is changeable*/
		set_DSTB_mode(enable);
		dcs[0] = LPDT;
		dcs[1] = dcs_set_display_off; 
		dsi_send_dcs_packet(dcs, 2);
		msleep(1);
		dcs[0] = LPDT;
		dcs[1] = dcs_enter_sleep_mode; 
		dsi_send_dcs_packet(dcs, 2);
		msleep(1);
		printk("++++++++++++++++%s:%d\n", __func__, __LINE__);
	} else {
		/*below is changeable*/
		set_DSTB_mode(enable);
		rk_lcd_init();
		printk("++++++++++++++++%s:%d\n", __func__, __LINE__);
	}
}
#endif
#endif
