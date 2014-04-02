
#ifndef _LCD_HJ080H_1024X768__
#define _LCD_HJ080H_1024X768__


/* Base */
#define SCREEN_TYPE		SCREEN_RGB
#define LVDS_FORMAT		LVDS_8BIT_1

#define OUT_FACE	    OUT_P888//OUT_D888_P666


#define DCLK	          71000000//65000000 xmxl@130424: use max dclk because 65000000 results in HDMI noise
#define LCDC_ACLK         500000000           //29 lcdc axi DMA ÆµÂÊ

/* Timing */
#define H_PW			10
#define H_BP			160
#define H_VD			1024//1280 //xmlsh
#define H_FP			150 //H_BP+H_FP = 320 //xmlsh

#define V_PW			3
#define V_BP			23
#define V_VD			768 //800 xmlsh
#define V_FP			12 //xmlsh

/* Other */
#define DCLK_POL                0
#define DEN_POL			0
#define VSYNC_POL		0
#define HSYNC_POL		0

#define SWAP_RB			0
#define SWAP_RG			0
#define SWAP_GB			0 


#define LCD_WIDTH          	174//270 //xmlsh
#define LCD_HEIGHT         	136//202 //xmlsh
#endif
