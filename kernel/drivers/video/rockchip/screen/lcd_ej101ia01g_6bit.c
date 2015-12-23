#ifndef _LCD_EJ101H_6BIT__
#define _LCD_EJ101H_6BIT__

#ifdef CONFIG_RK610_LVDS
#include "../transmitter/rk610_lcd.h"
#endif


/* Base */
#if  defined(CONFIG_RK610_LVDS) || defined(CONFIG_RK616_LVDS)
#define SCREEN_TYPE	    SCREEN_LVDS
#else
#define SCREEN_TYPE		SCREEN_RGB
#endif

#define LVDS_FORMAT		LVDS_6BIT
#if  defined(CONFIG_RK610_LVDS) || defined(CONFIG_RK616_LVDS)
#define OUT_FACE	    OUT_P666
#else
#define OUT_FACE	    OUT_D888_P666
#endif

#define DCLK			71000000
#define LCDC_ACLK         500000000           //29 lcdc axi DMA Ƶ��

/* Timing */
#define H_PW			10
#define H_BP			160
#define H_VD			1280
#define H_FP			16

#define V_PW			3
#define V_BP			23
#define V_VD			800
#define V_FP			12

/* Other */
#define DCLK_POL	0
#define DEN_POL			0
#define VSYNC_POL		0
#define HSYNC_POL		0

#define SWAP_RB			0
#define SWAP_RG			0
#define SWAP_GB			0 
#define LCD_WIDTH          	270
#define LCD_HEIGHT         	202

#endif
