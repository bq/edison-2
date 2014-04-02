/*
 * This Lcd Driver is for BYD 5' LCD BM800480-8545FTGE.
 * written by Michael Lin, 2010-06-18
 */
#ifndef __LCD_HX090G01__
#define __LCD_HX090G01__

/* Base */
#define SCREEN_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_D888_P666
#define DCLK			51200000
#define LCDC_ACLK       300000000     //29 lcdc axi DMA ÆµÂÊ//yangshl modify
#define LVDS_FORMAT      	LVDS_8BIT_2
//yangshl modify begin
/* Timing */
#define H_PW			100
#define H_BP			100
#define H_VD			1024
#define H_FP			120

#define V_PW			10
#define V_BP			10
#define V_VD			600
#define V_FP			15


/* Other */
#define DCLK_POL	0
#define DEN_POL		0
#define VSYNC_POL	0
#define HSYNC_POL	0

#define SWAP_RB		0
#define SWAP_RG		0
#define SWAP_GB		0

#define LCD_WIDTH       216    //need modify
#define LCD_HEIGHT      162
#endif

