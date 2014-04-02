/*
 * This Lcd Driver is for BYD 5' LCD BM800480-8545FTGE.
 * written by Michael Lin, 2010-06-18
 */
#ifndef __LCD_EJ101IA01G__
#define __LCD_EJ101IA01G__

/* Base */
#define SCREEN_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_D888_P666
#define DCLK			71000000
#define LCDC_ACLK       500000000     //29 lcdc axi DMA ÆµÂÊ//yangshl modify
#define LVDS_FORMAT      	LVDS_8BIT_2
//yangshl modify begin
/* Timing */
#define H_PW			10
#define H_BP			160
#define H_VD			1280//1280
#define H_FP			16

#define V_PW			3
#define V_BP			23
#define V_VD			800//800
#define V_FP			12


/* Other */
#define DCLK_POL	0
#define DEN_POL		0
#define VSYNC_POL	0
#define HSYNC_POL	0

#define SWAP_RB		0
#define SWAP_RG		0
#define SWAP_GB		0

#define LCD_WIDTH       270    //need modify
#define LCD_HEIGHT      202

//yangshl modify end
#endif


