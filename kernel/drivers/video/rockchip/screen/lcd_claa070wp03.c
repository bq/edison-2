
#ifndef _LCD_CLAA070WP03__
#define _LCD_CLAA070WP03__

/* Base */
#define SCREEN_TYPE		SCREEN_RGB
#define LVDS_FORMAT		LVDS_8BIT_1
#define OUT_FACE		OUT_D888_P666
#define DCLK			75000000
#define LCDC_ACLK       300000000     //29 lcdc axi DMA ÆµÂÊ

/* Timing */
#define H_PW			10
#define H_BP			64
#define H_VD			800
#define H_FP			16

#define V_PW			3
#define V_BP			8
#define V_VD			1280
#define V_FP			10

/* Other */
#define DCLK_POL                0
#define DEN_POL			0
#define VSYNC_POL		0
#define HSYNC_POL		0

#define SWAP_RB			0
#define SWAP_RG			0
#define SWAP_GB			0 



#define LCD_WIDTH       	153    //need modify
#define LCD_HEIGHT      	90

#endif
