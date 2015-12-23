#include <mach/gpio.h>
#include <linux/delay.h>

#ifndef _LCD_TPT420H2_1920x1080__
#define _LCD_TPT420H2_1920x1080__

/* Base */
#define SCREEN_TYPE			SCREEN_RGB
#define LVDS_FORMAT		LVDS_8BIT_1

#define OUT_FACE				OUT_P888

#define DCLK					128000000
#define LCDC_ACLK			500000000

/* Timing */
#define H_PW				10
#define H_BP					20
#define H_VD				1920
#define H_FP					20

#define V_PW				2
#define V_BP					20
#define V_VD					1080
#define V_FP					4

/* Other */
#define DCLK_POL			0
#define DEN_POL			0
#define VSYNC_POL		0
#define HSYNC_POL		0

#define SWAP_RB			0
#define SWAP_RG			0
#define SWAP_GB			0

#define LCD_WIDTH		940
#define LCD_HEIGHT		533

#define RK_SCREEN_INIT	1
static struct rk29lcd_info *gLcd_info = NULL;

static int rk_lcd_init(void)
{
	if(gLcd_info && gLcd_info->io_init)
		gLcd_info->io_init();

	return 0;
}

static int rk_lcd_standby(u8 enable)
{
	int ret = 0;

	printk("rk_lcd_standby enable = %d\n", enable);
	if(!enable)
	{
		if(gLcd_info && gLcd_info->lcd_en_pin)
		{
			msleep(200);
			gpio_set_value(gLcd_info->lcd_en_pin, gLcd_info->lcd_en_pin_level);
		}

		if(gLcd_info && gLcd_info->cs_pin)
		{
			//msleep(200);
			gpio_set_value(gLcd_info->cs_pin, gLcd_info->lcd_cs_pin_level);
		}
	}
	else
	{
		if(gLcd_info && gLcd_info->cs_pin)
		{
			gpio_set_value(gLcd_info->cs_pin, !gLcd_info->lcd_cs_pin_level);
			msleep(200);
		}

		if(gLcd_info && gLcd_info->lcd_en_pin)
		{
			gpio_set_value(gLcd_info->lcd_en_pin, !gLcd_info->lcd_en_pin_level);
			msleep(200);
		}
	}

	return 0;
}

#endif
