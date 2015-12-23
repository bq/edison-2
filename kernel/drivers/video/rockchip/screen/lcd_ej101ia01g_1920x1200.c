#ifndef _LCD_EJ101H_1920x1200__
#define _LCD_EJ101H_1920x1200__

#ifdef CONFIG_RK610_LVDS
#include "../transmitter/rk610_lcd.h"
#endif


/* Base */
#define SCREEN_TYPE		SCREEN_RGB
#define LVDS_FORMAT		LVDS_8BIT_1

#define OUT_FACE	    OUT_P888//OUT_D888_P666

#define DCLK	          150000000
#define LCDC_ACLK         500000000           //29 lcdc axi DMA ÆµÂÊ

/* Timing */
#define H_PW			10
#define H_BP			16
#define H_VD			1920
#define H_FP			18

#define V_PW			2
#define V_BP			6
#define V_VD			1200
#define V_FP			4

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
