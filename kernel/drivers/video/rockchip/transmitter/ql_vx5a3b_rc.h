/*
 * Copyright (C) Quicklogic 2013
 *
 * ql_vx5a3b_rc.h
 * Quicklogic VX 5A3B driver for Rockchip.
 * Author : Sunny 0516
 *
 * 
 * License terms: GNU General Public License (GPL), version 2.
 */
#ifndef __QL_VX5A3B_RC_H
#define __QL_VX5A3B_RC_H

//i2c address
#define QL_I2C_ADDR 0x64
#define QL_I2C_BOOTLOADER_ADDR	(QL_I2C_ADDR<<1)
//Porting: i2c details here.
//i2c bus number for kernel
#define QL_I2C_REGISTER_BUSNUM 1
//i2c operation for BUILD_UBOOT.
#define QL_I2C_BOOTLOADER_INIT i2c0_v1_init
#define QL_I2C_BOOTLOADER_WRITE i2c0_v1_write
#define QL_I2C_BOOTLOADER_READ i2c0_v1_read
//enable setup VX chip and panel from Diolan board.
//#define QL_VX_INIT_EXTERNAL
//enable panel mipi set backlight routines. 
//#define QL_PANEL_HAS_SETBACKLIGHT

#define QL_VEE_ENABLE 1

//enable debug message.
#define QL_DEBUG

#ifndef QL_DEBUG
#define QL_DBG(x...) 
#define QL_DBGL(x...) 
#else
#ifdef BUILD_UBOOT
#define QL_DBG(f, x...) \
	printf("[QLVX] %s: " f, __func__,## x)
#define QL_DBGL(lvl, f, x...) do {if (lvl) printf("[QLVX] %s: " f, __func__,## x); }while(0)
#define QL_ERR(f, x...) \
	printf("[QLVX] ERROR %s: " f, __func__,## x)
#else
#define QL_DBG(f, x...) \
	printk("[QLVX] %s: " f, __func__,## x)
#define QL_DBGL(lvl, f, x...) do {if (lvl) printk("[QLVX] %s: " f, __func__,## x); }while(0)
#define QL_ERR(f, x...) \
	printk("[QLVX] ERROR %s: " f, __func__,## x)
#endif
#endif

#define QL_DBG_FUNC_ENTER QL_DBG("+++\n");
#define QL_DBG_FUNC_EXIT QL_DBG("---\n");

extern int vx5a3b_update_brightness(struct backlight_device *bl);

#endif
