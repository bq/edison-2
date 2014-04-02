/*
 *
 * Copyright (C) 2012 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/ion.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <mach/dvfs.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <linux/rk_fb.h>
#include <linux/regulator/machine.h>
#include <linux/rfkill-rk.h>
#include <linux/sensor-dev.h>
#include <linux/mfd/tps65910.h>
#include <linux/regulator/act8846.h>
#include <linux/regulator/rk29-pwm-regulator.h>
#include <linux/bq24196_chargeIc.h>

#include <linux/power/bq27541_battery.h>
#include <linux/power/bq27410_battery.h>

#include <linux/gt9xx.h>

#ifdef CONFIG_CW2015_BATTERY
#include <linux/power/cw2015_battery.h>
#endif

#if defined(CONFIG_MFD_RK610)
#include <linux/mfd/rk610_core.h>
#endif

#if defined(CONFIG_DP_ANX6345)
	#include<linux/anx6345.h>
#endif

#if defined(CONFIG_CT36X_TS)
#include <linux/ct36x.h>
#endif

#if defined(CONFIG_RK_HDMI)
	#include "../../../drivers/video/rockchip/hdmi/rk_hdmi.h"
#endif

#if defined(CONFIG_SPIM_RK29)
#include "../../../drivers/spi/rk29_spim.h"
#endif
#if defined(CONFIG_GPS_RK)
#include "../../../drivers/misc/gps/rk_gps/rk_gps.h"
#endif

#if defined(CONFIG_MU509)
#include <linux/mu509.h>
#endif
#if defined(CONFIG_ME906E)
#include <linux/me906e.h>
#endif
#if defined(CONFIG_MW100)
#include <linux/mw100.h>
#endif
#if defined(CONFIG_MT6229) || defined(CONFIG_MT6229_UNAP)
#include <linux/mt6229.h>
#endif
#if defined(CONFIG_ANDROID_TIMED_GPIO)
#include "../../../drivers/staging/android/timed_gpio.h"
#endif

#ifdef CONFIG_HAPTICS_DRV2605
#include <../../../drivers/haptics/drv2605.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5606_MALATA
#include <linux/i2c/ft5x06_ts2.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_WALTOP_EM
#include <linux/i2c/waltop_em_i2c.h>
#endif

#ifdef CONFIG_MPU_SENSORS_MPU6050B1
#include <linux/mpu.h>
#endif

#include "board-rk3168-LR097-camera.c"

#if defined(CONFIG_TOUCHSCREEN_GT8XX)
#define TOUCH_RESET_PIN  RK30_PIN0_PB6
#define TOUCH_PWR_PIN    RK30_PIN0_PC5   // need to fly line by hardware engineer
static int goodix_init_platform_hw(void)
{
	int ret;
	
	if (TOUCH_PWR_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_PWR_PIN, "goodix power pin");
		if (ret != 0) {
			gpio_free(TOUCH_PWR_PIN);
			printk("goodix power error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_PWR_PIN, 0);
		gpio_set_value(TOUCH_PWR_PIN, GPIO_LOW);
		msleep(100);
	}

	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "goodix reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("goodix gpio_request error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_RESET_PIN, 1);
                msleep(100);
		//gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
		//msleep(100);
		//gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
		//msleep(500);
	}
	return 0;
}

struct goodix_platform_data goodix_info = {
	.model = 8105,
	.irq_pin = RK30_PIN1_PB7,
	.rest_pin = TOUCH_RESET_PIN,
	.init_platform_hw = goodix_init_platform_hw,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_GT9110_MALATA
struct goodix_9110_platform_data  goodix9110_info = {
        .irq_pin = RK30_PIN1_PB7,
        .reset= RK30_PIN0_PB6,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5606_MALATA
int ft5x0x_init_platform_hw(void)
{
	struct regulator *ldo;
	int ret;
//	rk30_mux_api_set(GPIO1B7_CIFDATA11_NAME, GPIO1B_GPIO1B7);
//	rk30_mux_api_set(GPIO0B6_I2S8CHSDO2_NAME, GPIO0B_GPIO0B6);
//	pr_debug("%s:0x%x,0x%x\n",__func__,rk30_mux_api_get(GPIO4D0_SMCDATA8_TRACEDATA8_NAME),rk30_mux_api_get(GPIO4C2_SMCDATA2_TRACEDATA2_NAME));

//	if(g_pmic_type == PMIC_TYPE_TPS65910)
//		ldo = regulator_get(NULL, "vaux33");
//	else if(g_pmic_type == PMIC_TYPE_WM8326)
//		ldo = regulator_get(NULL, "ldo9");
//	regulator_set_voltage(ldo, 3300000, 3300000);
//	regulator_enable(ldo);
//	pr_debug("%s set vaux33 vcc_tp=%dmV end\n", __func__, regulator_get_voltage(ldo));
//	regulator_put(ldo);
//	msleep(100);

	return 0;
}

static struct  ft5x0x_ts_platform_data  ft5x0x_touch_info = {
	.intr_number = RK30_PIN1_PB7,
	.reset_pin =  RK30_PIN0_PB6,
	.init_platform_hw = ft5x0x_init_platform_hw,
};

#endif

#ifdef CONFIG_TOUCHSCREEN_WALTOP_EM
static struct sis_i2c_rmi_platform_data waltop_em_info = {
	.reset_pin = RK30_PIN0_PD5,
};

#endif
#if defined(CONFIG_CT36X_TS)

#define TOUCH_MODEL		363
#define TOUCH_MAX_X		1280
#define TOUCH_MAX_y		800
#define TOUCH_RESET_PIN		RK30_PIN0_PB6
#define TOUCH_INT_PIN		RK30_PIN1_PB7

static struct ct36x_platform_data ct36x_info = {
	.model   = TOUCH_MODEL,
	.x_max   = TOUCH_MAX_X,
	.y_max   = TOUCH_MAX_y,

	.rst_io = {
		.gpio = TOUCH_RESET_PIN,
		.active_low = 1,
	},
	.irq_io = {
		.gpio = TOUCH_INT_PIN,
		.active_low = 1,
	},
	.orientation = {1, 0, 0, 1},
};
#endif

static struct spi_board_info board_spi_devices[] = {
};

/***********************************************************
*	rk30  backlight
************************************************************/
#ifdef CONFIG_BACKLIGHT_RK29_BL
#if defined(CONFIG_MALATA_D1014)||defined(CONFIG_MALATA_D8009)||defined(CONFIG_MALATA_D7022)
#define PWM_ID            1
#define PWM_MODE          PWM1
#else
#define PWM_ID            3
#define PWM_MODE          PWM3
#endif
#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D8006) || defined(CONFIG_MALATA_D1012)||defined(CONFIG_MALATA_D7014)
#define PWM_EFFECT_VALUE  0
#else
#define PWM_EFFECT_VALUE  1
#endif

#ifndef CONFIG_MALATA_D1004
#define LCD_DISP_ON_PIN
#endif

//#ifdef  LCD_DISP_ON_PIN
#if defined(CONFIG_MALATA_D1012)
#define BL_EN_PIN         RK30_PIN0_PA2
#define BL_EN_VALUE       GPIO_HIGH
#elif defined(CONFIG_MALATA_D1014)||defined(CONFIG_MALATA_D8009)||defined(CONFIG_MALATA_D7022)
#define BL_EN_PIN         RK30_PIN3_PD4
#define BL_EN_VALUE       GPIO_HIGH
#elif defined(CONFIG_MALATA_D1004)
#define BL_EN_PIN         RK30_PIN3_PD6
#define BL_EN_VALUE       GPIO_HIGH
#elif defined(CONFIG_MALATA_D7803)
#define BL_EN_PIN         RK30_PIN0_PD4
#define BL_EN_VALUE       GPIO_HIGH     //GPIO_HIGH
#elif defined(CONFIG_MALATA_D8006)
#define BL_EN_PIN         RK30_PIN0_PA2
#define BL_EN_VALUE       GPIO_HIGH
#elif defined(CONFIG_MALATA_D7014)
#define BL_EN_PIN         RK30_PIN0_PA2
#define BL_EN_VALUE       GPIO_HIGH
#else
#define BL_EN_PIN         RK30_PIN0_PA2
#define BL_EN_VALUE       GPIO_HIGH
#endif

#if defined(CONFIG_MALATA_D1001) || defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7014) \
|| defined(CONFIG_MALATA_D8006) || defined(CONFIG_MALATA_C1017) ||defined(CONFIG_MALATA_D1012) || defined(CONFIG_MALATA_D1013)
#define LCD_DISP_ON_PIN

#if defined(CONFIG_MALATA_D1001) || defined(CONFIG_MALATA_C1017) || defined(CONFIG_MALATA_D1013)
#define BL_EN_PIN         RK30_PIN0_PA2
#define BL_EN_VALUE       GPIO_HIGH
#define BL_PWM_SEL        RK30_PIN0_PD6
#endif
#endif
//#endif
static int rk29_backlight_io_init(void)
{
	int ret = 0;

	iomux_set(PWM_MODE);
#ifdef  LCD_DISP_ON_PIN
	printk("honghaishen_test open bl \n");
	ret = gpio_request(BL_EN_PIN, "bl_en");
	if (ret == 0) {
		gpio_direction_output(BL_EN_PIN, BL_EN_VALUE);
	}else{
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
	}
#endif
#if defined(CONFIG_MALATA_D1001) || defined(CONFIG_MALATA_C1017)
	ret =  gpio_request(BL_PWM_SEL, "bl_pwm_sel");
	if (ret == 0) {
		gpio_direction_output(BL_PWM_SEL, 0);
		printk("rk29_backlight_io_init, Select lcd pwm from LCDC_BL.\n");
	}
#endif
	return ret;
}

static int rk29_backlight_io_deinit(void)
{
	int ret = 0, pwm_gpio;
#ifdef  LCD_DISP_ON_PIN
	gpio_free(BL_EN_PIN);
#endif
	pwm_gpio = iomux_mode_to_gpio(PWM_MODE);
	gpio_request(pwm_gpio, "bl_pwm");
	gpio_direction_output(pwm_gpio, GPIO_LOW);
	return ret;
}

static int rk29_backlight_pwm_suspend(void)
{
	int ret, pwm_gpio = iomux_mode_to_gpio(PWM_MODE);

	ret = gpio_request(pwm_gpio, "bl_pwm");

/*[A]For lcd flash a white line while suspend at first time, ArthurLin, 20130704*/
#if defined(CONFIG_MALATA_D8006) || defined(CONFIG_MALATA_D7014) || defined(CONFIG_MALATA_D1013)
    if (ret != 0)
    {
        gpio_free(pwm_gpio);
        printk("request pwm_gpio failed, gpio_free first! \n");
        ret = gpio_request(pwm_gpio, "bl_pwm");
    }
#endif
/*END ArthurLin, 20130704*/

#if defined(CONFIG_MALATA_D7803)
	/*
	if (ret) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return ret;
	} */    // hhs
	//gpio_direction_output(pwm_gpio, GPIO_LOW);

#else
	if (ret) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return ret;
	}
	gpio_direction_output(pwm_gpio, GPIO_LOW);
#endif
#ifdef  LCD_DISP_ON_PIN
	gpio_direction_output(BL_EN_PIN, !BL_EN_VALUE);
#endif
#if defined(CONFIG_MALATA_D7803)
	return 0;
#else
	return ret;
#endif
}

static int rk29_backlight_pwm_resume(void)
{
	int pwm_gpio = iomux_mode_to_gpio(PWM_MODE);

	gpio_free(pwm_gpio);
	iomux_set(PWM_MODE);
#ifdef  LCD_DISP_ON_PIN

/*[M]For lcd flash a white line sometime while rusume, ArthurLin, 20130704*/
#if defined(CONFIG_MALATA_D8006)|| defined(CONFIG_MALATA_D7014)
    msleep(100);
#else
    msleep(30);
#endif
/*END ArthurLin, 20130704*/

#ifdef  LCD_DISP_ON_PIN
	gpio_direction_output(BL_EN_PIN, BL_EN_VALUE);
#endif
#endif
	return 0;
}

static struct rk29_bl_info rk29_bl_info = {
	.pwm_id = PWM_ID,
#if defined(CONFIG_MALATA_D1001)
	.min_brightness=40,
	.max_brightness=255,
	.brightness_mode =BRIGHTNESS_MODE_LINE,
#elif defined(CONFIG_MALATA_D7803)
	.min_brightness=15,
	.max_brightness=255,
	.brightness_mode = BRIGHTNESS_MODE_LINE,
	.delay_ms = 50,
#elif defined(CONFIG_MALATA_C1017)
	.min_brightness=40,
	.max_brightness=230,
	.brightness_mode = BRIGHTNESS_MODE_LINE,
#elif defined(CONFIG_MALATA_D7005)
	.min_brightness=10,
	.max_brightness=205,
	.brightness_mode = BRIGHTNESS_MODE_LINE,
#elif defined(CONFIG_MALATA_D1013)
	.min_brightness=7,
	.max_brightness=255,
	.brightness_mode = BRIGHTNESS_MODE_LINE,
#elif defined(CONFIG_MALATA_D1012)
	.min_brightness=10,
	.max_brightness=230,
	.brightness_mode = BRIGHTNESS_MODE_LINE,
#elif defined(CONFIG_MALATA_D8009)
	.min_brightness=10,
	.max_brightness=255,
	.brightness_mode = BRIGHTNESS_MODE_LINE,
#elif defined(CONFIG_MALATA_D7014)
	.min_brightness=30,
	.max_brightness=255,
	.brightness_mode = BRIGHTNESS_MODE_LINE,
#elif defined(CONFIG_MALATA_D7022)
	.min_brightness=10,
	.max_brightness=195,
	.brightness_mode = BRIGHTNESS_MODE_LINE,
#else
	.min_brightness=10,    //  20
	.brightness_mode = BRIGHTNESS_MODE_LINE,
	.max_brightness=255,
#endif
	.pre_div = 30 * 1000,
	.bl_ref = PWM_EFFECT_VALUE,
	.io_init = rk29_backlight_io_init,
	.io_deinit = rk29_backlight_io_deinit,
	.pwm_suspend = rk29_backlight_pwm_suspend,
	.pwm_resume = rk29_backlight_pwm_resume,
};

static struct platform_device rk29_device_backlight = {
	.name	= "rk29_backlight",
	.id 	= -1,
	.dev	= {
		.platform_data  = &rk29_bl_info,
	}
};

#endif

#ifdef CONFIG_RK29_SUPPORT_MODEM

#define RK30_MODEM_POWER        RK30_PIN0_PC6
#define RK30_MODEM_POWER_IOMUX  iomux_set(GPIO0_C6)

static int rk30_modem_io_init(void)
{
    printk("%s\n", __FUNCTION__);
    RK30_MODEM_POWER_IOMUX;

	return 0;
}

static struct rk29_io_t rk30_modem_io = {
    .io_addr    = RK30_MODEM_POWER,
    .enable     = GPIO_HIGH,
    .disable    = GPIO_LOW,
    .io_init    = rk30_modem_io_init,
};

static struct platform_device rk30_device_modem = {
	.name	= "rk30_modem",
	.id 	= -1,
	.dev	= {
		.platform_data  = &rk30_modem_io,
	}
};
#endif
#if defined(CONFIG_MU509)
static int mu509_io_init(void)
{

	iomux_set(GPIO2_D5);
	iomux_set(GPIO0_C6);
	iomux_set(GPIO2_D4);
	iomux_set(GPIO0_C4);
	iomux_set(GPIO0_C5);
	return 0;
}

static int mu509_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mu509_data rk29_mu509_info = {
	.io_init = mu509_io_init,
  	.io_deinit = mu509_io_deinit,
	.modem_power_en = RK30_PIN2_PD5,   
	.bp_power = RK30_PIN0_PC6,        	
	.bp_reset = RK30_PIN2_PD4,          	
	.ap_wakeup_bp = RK30_PIN0_PC4,	
	.bp_wakeup_ap = RK30_PIN0_PC5, 	
};
struct platform_device rk29_device_mu509 = {	
        .name = "mu509",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk29_mu509_info,
	}    	
    };
#endif

#if defined(CONFIG_ME906E)
static int me906e_io_init(void)
{
	iomux_set(GPIO2_D5);
	iomux_set(GPIO0_C6);
	iomux_set(GPIO2_D4);
	iomux_set(GPIO0_C4);
	iomux_set(GPIO0_C5);
	iomux_set(GPIO0_D4);
	return 0;
}

static int me906e_io_deinit(void)
{
	return 0;
}

struct rk29_me906e_data rk29_me906e_info = {
	.io_init = me906e_io_init,
	.io_deinit = me906e_io_deinit,
	.modem_power_en = RK30_PIN2_PD5,
	.bp_power = RK30_PIN0_PC6,
	.bp_reset = RK30_PIN2_PD4,
	.ap_wakeup_bp = RK30_PIN0_PC4,
	.bp_wakeup_ap = RK30_PIN0_PC5,
	.gps_disable = RK30_PIN0_PD4,
};
struct platform_device rk29_device_me906e = {
        .name = "me906e",
	.id = -1,
	.dev		= {
		.platform_data = &rk29_me906e_info,
	}
    };
#endif

#if defined(CONFIG_MW100)
static int mw100_io_init(void)
{
	iomux_set(GPIO2_D5);
	iomux_set(GPIO0_C6);
	iomux_set(GPIO2_D4);
	iomux_set(GPIO0_C4);
	iomux_set(GPIO0_C5);
	return 0;
}

static int mw100_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mw100_data rk29_mw100_info = {
	.io_init = mw100_io_init,
  	.io_deinit = mw100_io_deinit,
	.modem_power_en = RK30_PIN2_PD5,
	.bp_power = RK30_PIN0_PC6,
	.bp_reset = RK30_PIN2_PD4,
	.ap_wakeup_bp = RK30_PIN0_PC4,
	.bp_wakeup_ap = RK30_PIN0_PC5,
};
struct platform_device rk29_device_mw100 = {	
        .name = "mw100",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk29_mw100_info,
	}    	
    };
#endif
#if defined(CONFIG_MT6229)
static int mt6229_io_init(void)
{
	#ifdef CONFIG_MALATA_D7803
	iomux_set(GPIO2_D5);
	iomux_set(GPIO0_D6);
	iomux_set(GPIO2_D4);
	iomux_set(GPIO1_C4);
	iomux_set(GPIO1_C5);
	#else
	iomux_set(GPIO2_D5);
	iomux_set(GPIO0_C6);
	iomux_set(GPIO2_D4);
	iomux_set(GPIO0_C4);
	iomux_set(GPIO0_C5);
	#endif
	return 0;
}

static int mt6229_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mt6229_data rk29_mt6229_info = {
	.io_init = mt6229_io_init,
  	.io_deinit = mt6229_io_deinit,
#ifdef CONFIG_MALATA_D7803
	.modem_power_en = RK30_PIN3_PB2, //3G_PWR
	.bp_power = RK30_PIN2_PD5,//3G_EN
	.modem_usb_en = RK30_PIN2_PD4,//3G_W_DISABLE
	.modem_uart_en = RK30_PIN0_PD6,//EINT9
	.bp_wakeup_ap = RK30_PIN1_PB4,//3G_WAKE_OUT
	.ap_ready = RK30_PIN1_PB5,//3G_WAKE_IN
#else
	.modem_power_en = RK30_PIN0_PC6,
#ifndef CONFIG_MALATA_D1003
	.bp_power = RK30_PIN2_PD5,
	.modem_usb_en = RK30_PIN2_PD4,
#else
	.bp_power = RK30_PIN2_PD4,
	.modem_usb_en = RK30_PIN2_PD5,
#endif
	.modem_uart_en = RK30_PIN0_PC0,
	.bp_wakeup_ap = RK30_PIN0_PC5,
	.ap_ready = RK30_PIN0_PC4,
#endif
};
struct platform_device rk29_device_mt6229 = {
	.name = "mt6229",
	.id = -1,
	.dev		= {
		.platform_data = &rk29_mt6229_info,
	}
	};
#endif

#if defined(CONFIG_MT6229_UNAP)
static int mt6229_unap_io_init(void)
{
	iomux_set(GPIO2_D5);
	iomux_set(GPIO0_C6);
	return 0;
}

static int mt6229_unap_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mt6229_data rk29_mt6229_unap_info = {
	.io_init = mt6229_unap_io_init,
	.io_deinit = mt6229_unap_io_deinit,
	.modem_power_en = RK30_PIN0_PC6,
	.bp_power = RK30_PIN2_PD5,
};
struct platform_device rk29_device_mt6229_unap = {
	.name = "mt6229_unap",
	.id = -1,
	.dev		= {
		.platform_data = &rk29_mt6229_unap_info,
	}
	};
#endif

/*MMA8452 gsensor*/
#if defined (CONFIG_GS_MMA8452)
#define MMA8452_INT_PIN   RK30_PIN0_PB7

static int mma8452_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data mma8452_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 0,
	.poll_delay_ms = 20,
	.init_platform_hw = mma8452_init_platform_hw,
#if defined(CONFIG_MALATA_D1012)||defined(CONFIG_MALATA_D1014)
	.orientation = { -1, 0, 0 ,0, -1, 0, 0, 0, 1},
#elif defined(CONFIG_MALATA_D1004)
	.orientation = {0, -1, 0, -1, 0, 0 , 0, 0, -1},
#elif defined(CONFIG_MALATA_D7803)
	#if defined(CONFIG_MALATA_D7806)
		.orientation = {0, 1, 0, -1, 0, 0 , 0, 0, 1},
	#else
		.orientation = {0, -1, 0, -1, 0, 0 , 0, 0, -1},
	#endif
#else
	.orientation = {0, 1, 0, -1, 0, 0 , 0, 0, 1},
#endif
};
#endif
#ifdef CONFIG_GS_MMA7660
#define MMA7660_INT_PIN   RK30_PIN0_PB7
static int mma7660_init_platform_hw(void)
{
	//rk30_mux_api_set(GPIO0B7_I2S8CHSDO3_NAME, GPIO0B_GPIO0B7);

	return 0;
}

#ifdef CONFIG_G_SENSOR_DEVICE
static struct gsensor_platform_data mma7660_info = {
	.model= 7660,
	.swap_xy = 0,
	.init_platform_hw = mma7660_init_platform_hw,
};
#endif
static struct sensor_platform_data mma7660_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 0,
	.poll_delay_ms = 20,
        .init_platform_hw = mma7660_init_platform_hw,
        .orientation = {  -1, 0, 0, 0, 1, 0,0, 0, -1},
};
#endif

#if defined (CONFIG_GS_LIS3DE)
#define LIS3DE_INT_PIN   RK30_PIN0_PB7

static int lis3de_init_platform_hw(void)
{
        return 0;
}

static struct sensor_platform_data lis3de_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 0,
	.poll_delay_ms = 20,
	.init_platform_hw = lis3de_init_platform_hw,
#if defined(CONFIG_MALATA_D1004)
	#if defined(CONFIG_MALATA_D1014)
	.orientation = {1, 0, 0, 0, -1, 0, 0, 0, -1},//land
	#elif defined(CONFIG_MALATA_D8009)
	//.orientation = {0, -1, 0, -1, 0, 0, 0, 0, -1},//Portrait
	.orientation = {1, 0, 0, 0, -1, 0, 0, 0, -1},//land
	#elif defined(CONFIG_MALATA_D1013)
	.orientation = {0, 1, 0, 1, 0, 0, 0, 0, -1},
	#elif defined(CONFIG_MALATA_D7022)
	.orientation = {1, 0, 0, 0, -1, 0, 0, 0, -1},//land
	//.orientation = {0, -1, 0, -1, 0, 0, 0, 0, -1},//Portrait
	#else
	.orientation = {0, -1, 0, -1, 0, 0, 0, 0, -1},
	#endif
#else
	.orientation = {0, -1, 0, -1, 0, 0, 0, 0, -1},
#endif
};
#endif

#if defined (CONFIG_GS_LIS3DH)
#define LIS3DH_INT_PIN   RK30_PIN0_PB7

static int lis3dh_init_platform_hw(void)
{

        return 0;
}

static struct sensor_platform_data lis3dh_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = lis3dh_init_platform_hw,
	.orientation = {-1, 0, 0, 0, 0, 1, 0, -1, 0},
};
#endif

#ifdef CONFIG_MPU_SENSORS_MPU6050B1
#define MPU6050_INT_PIN  RK30_PIN0_PB4
static struct mpu_platform_data mpu6050_data = {
	.int_config 	= 0x10,
	.level_shifter	= 0,
#if defined(CONFIG_MALATA_D7803)
	#if defined(CONFIG_MALATA_D7806)
	.orientation	= {
					  1,   0,	 0,
					   0,	 1,   0 ,
					  0,   0, 1},
	#else
	.orientation	= {
					  0,   -1,	 0,
					   -1,	 0,   0 ,
					  0,   0, -1},
	#endif
#elif defined(CONFIG_MALATA_D1005)
	.orientation 	= {
					  0,   -1,   0,
					   1,   0,   0 ,
					  0,   0,  1},
#elif defined(CONFIG_MALATA_D8006)||defined(CONFIG_MALATA_D8009)
        .orientation    = {
                                          1,   0,  0,
                                          0,  -1,  0,
                                          0,   0, -1},
#elif defined(CONFIG_MALATA_D1012)||defined(CONFIG_MALATA_D1014)
	.orientation 	= {
					   1,   0,   0 ,
					   0,   1,   0,
					  0,   0, 1},
#elif defined(CONFIG_MALATA_D7014)
	.orientation	= {
				  0,   1,	0,
				  -1,	0,	 0 ,
				  0,   0, -1},

#else
	.orientation 	= {
					  0,   1,   0,
					   1,   0,   0 ,
					  0,   0, -1},
#endif
};
static struct ext_slave_platform_data mpu_compass_data = {
#if defined(CONFIG_MALATA_D8006) || defined(CONFIG_MALATA_D1012)||defined(CONFIG_MALATA_D1014)
	.address 		= 0x0c,
#else
	.address 		= 0x0d,
#endif
	.adapt_num 	= 0,
	.bus 			= EXT_SLAVE_BUS_PRIMARY,
#if defined(CONFIG_MALATA_D8006)||defined(CONFIG_MALATA_D8009)
	.orientation 	= {
						0,   -1,  0,
						1,    0,   0,
						0,   0,   1},
#elif defined(CONFIG_MALATA_D7803)
	#if defined(CONFIG_MALATA_D7806)
		.orientation    = {
	                       -1,  0,   0,
	                       0,   -1,  0,
	                       0,   0,   1},
	#else
	    .orientation    = {
	                       -1,  0,   0,
	                       0,   1,  0,
	                       0,   0,   -1},
	#endif
#elif defined(CONFIG_MALATA_D1004)
		#if defined(CONFIG_MALATA_D7005)
			.orientation	= {
							0,  1,	0,
							-1,   0,  0,
							0,   0,	-1},
		#elif defined(CONFIG_MALATA_D1012)||defined(CONFIG_MALATA_D1014)
			.orientation	= {
							-1,   0,  0,
							0,  -1,  0,
							0,   0,   1},
		#else
			.orientation	= {
							0,  -1,	0,
							-1,   0,  0,
							0,   0,	-1},
		#endif
#else
	.orientation 	= {
					  -1,  0,   0,
					  0,   1,  0,
					  0,   0,   -1},
#endif
};
#endif

#if defined(CONFIG_CHARGER_CW2015)
struct cw2015_platform_data cw2015_info = {
		.dc_det_pin      = RK30_PIN0_PB2,
        .batt_low_pin    = RK30_PIN0_PB1,
        //.charge_ok_pin   = RK30_PIN0_PA6,
        .dc_det_level    = GPIO_LOW,
        .batt_low_level  = GPIO_LOW,
        //.charge_ok_level = GPIO_HIGH,
};
#endif
#if defined (CONFIG_COMPASS_AK8975)
static struct sensor_platform_data akm8975_info =
{
	.type = SENSOR_TYPE_COMPASS,
	.irq_enable = 0,
	.poll_delay_ms = 10,
	.m_layout =
	{
		{
			{-1, 0, 0},
			{0, 1, 0},
			{0, 0, -1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	}
};

#endif
#if defined (CONFIG_COMPASS_AK8963)
static struct sensor_platform_data akm8963_info =
{
	.type = SENSOR_TYPE_COMPASS,
	.irq_enable = 0,
	.poll_delay_ms = 20,
	.m_layout = 
	{
	#if defined(CONFIG_MALATA_D1004) && defined(CONFIG_MALATA_D1014)
		{
			{0, -1, 0},
			{-1, 0, 0},
			{0, 0, -1},
		},
	#elif defined(CONFIG_MALATA_D1004) && defined(CONFIG_MALATA_D8009)
		{
			{0, -1, 0},
			{1, 0, 0},
			{0, 0, 1},
		},
	#elif defined(CONFIG_MALATA_D1004) && defined(CONFIG_MALATA_D7022)
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	#else
		{
			{-1, 0, 0},
			{0, 1, 0},
			{0, 0, -1},
		},
	#endif
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	#if defined(CONFIG_MALATA_D1004) && defined(CONFIG_MALATA_D1014)
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, -1},
		},
	#elif defined(CONFIG_MALATA_D1004) && defined(CONFIG_MALATA_D8009)
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, -1},
		},
	#elif defined(CONFIG_MALATA_D1004) && defined(CONFIG_MALATA_D7022)
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, -1},
		},
	#else
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	#endif
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	}
};

#endif

#if defined(CONFIG_GYRO_L3G4200D)

#include <linux/l3g4200d.h>
#define L3G4200D_INT_PIN  RK30_PIN0_PB4

static int l3g4200d_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data l3g4200d_info = {
	.type = SENSOR_TYPE_GYROSCOPE,
	.irq_enable = 0,
	.poll_delay_ms = 10,
	.orientation = {0, 1, 0, 1, 0, 0, 0, 0, -1},
	.init_platform_hw = l3g4200d_init_platform_hw,
	.x_min = 5,//x_min,y_min,z_min = (0-100) according to hardware
	.y_min = 5,
	.z_min = 5,
};

#endif
#ifdef CONFIG_LS_ISL29023
	static struct sensor_platform_data lsl29023_info = {
		.type = SENSOR_TYPE_LIGHT,
		.irq_enable = 0,
		.poll_delay_ms = 500,
	};

#endif
#ifdef CONFIG_LS_ISL5151
static struct sensor_platform_data isl5151_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 0,
	.poll_delay_ms = 500,
};

#endif
#ifdef CONFIG_LS_CM3217
static struct sensor_platform_data cm3217_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 0,
	.poll_delay_ms = 500,
};

#endif
#ifdef CONFIG_TMP108
static struct sensor_platform_data tmp108_info = {
	.type = SENSOR_TYPE_TEMPERATURE,
	.irq_enable = 1,
	.poll_delay_ms = 1000,
};

#endif

#ifdef CONFIG_SSD2828_RGB2MIPI
#include "../../../drivers/video/rockchip/transmitter/mipi_dsi.h"
struct ssd2828_t ssd2828_platdata = {
	.id = 0x2828,
	.reset = {
		.reset_pin = RK30_PIN3_PD4,   //RESET PIN
		.effect_value = GPIO_LOW,    // hhs
	},
/*
RK30_PIN0_PA2
	.vddio = {                       //POWER ON
		.enable_pin = RK30_PIN0_PB0,
		.effect_value = GPIO_LOW,
	},

*/
	.vddio = {                       //POWER ON
		.enable_pin = RK30_PIN0_PB0,
		.effect_value = GPIO_LOW,
	},
	.vdd_mipi = {                     //MVDD
		.enable_pin = INVALID_GPIO,
	},
	.spi = {
		.cs =RK30_PIN1_PA7,
		.sck = RK30_PIN1_PA6,
		.miso = RK30_PIN1_PA4,
		.mosi = RK30_PIN1_PA5,
	},
};

//board
static struct platform_device device_ssd2828 = {
        .name   = "ssd2828",
        .id     = -1,
        .dev = {
                .platform_data = &ssd2828_platdata,
        },
};

#endif

#ifdef CONFIG_QLVX5A3B_RGB2MIPI
#include "../../../drivers/video/rockchip/transmitter/mipi_dsi.h"
static struct ql_vx5a3b_t qlvx5a3b_platdata = {
	.id = 0X2300,
	.lcd_en = {
		.enable_pin = RK30_PIN1_PA6,//RK30_PIN0_PB0,   //LCD EN PIN
		.effect_value = GPIO_LOW,    // hhs
	},
	.reset = {
		.reset_pin = RK30_PIN1_PA7,   //RESET PIN
		.effect_value = GPIO_LOW,    // hhs
	},
	.vcc_lcd = {                       //POWER ON
		.enable_pin = RK30_PIN0_PB0,//RK30_PIN0_PA2,
		.effect_value = GPIO_LOW,
	},
	.vrgb_io = {                     //MVDD
		.enable_pin = INVALID_GPIO,
	},
	.vdd_log = {                     //SHUT PIN
		.enable_pin = INVALID_GPIO,//RK30_PIN0_PB0,
		.effect_value = GPIO_LOW,
	}, 
};
#endif

#if defined(CONFIG_KIONIX_KMX61G_SENSOR)
#include <linux/input/kmx61.h>
#define KMX61G_INT_PIN  RK30_PIN0_PB7
static struct kmx61_platform_data kmx61_board_info = {
		.min_interval   = 5,
		.init_interval  = 200,
		.axis_map_x     = 0,
		.axis_map_y     = 1,
		.axis_map_z     = 2,
		.negate_x       = 1,
		.negate_y       = 0,
		.negate_z       = 1,
};
#endif

#ifdef CONFIG_FB_ROCKCHIP
#if defined(CONFIG_MALATA_D1014)||defined(CONFIG_MALATA_D8009)
#define LCD_CS_PIN         iomux_mode_to_gpio(PWM3)
#define LCD_CS_VALUE       GPIO_HIGH
#elif defined(CONFIG_MALATA_D7022)
#define LCD_CS_PIN         RK30_PIN3_PD6
#define LCD_CS_VALUE       GPIO_HIGH
#elif defined(CONFIG_MALATA_D1004) || defined(CONFIG_MALATA_D8006)
#define LCD_CS_PIN         RK30_PIN3_PD4
#define LCD_CS_VALUE       GPIO_HIGH
#elif defined(CONFIG_MALATA_D7014)
#define LCD_CS_PIN         INVALID_GPIO
#define LCD_CS_VALUE       GPIO_HIGH
#else
#define LCD_CS_PIN         INVALID_GPIO
#define LCD_CS_VALUE       GPIO_HIGH
#endif

#if defined(CONFIG_MALATA_D7803)||defined(CONFIG_MALATA_D7014)
#define LCD_EN_PIN       INVALID_GPIO  //RK30_PIN0_PA2
#else
#define LCD_EN_PIN         RK30_PIN0_PB0
#endif
#define LCD_EN_VALUE       GPIO_LOW

#if defined(CONFIG_MALATA_D1004) || defined(CONFIG_MALATA_D7005) ||defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D8006)
#define LCD_DRV_PIN         RK30_PIN0_PA3
#define LCD_DRV_VALUE       GPIO_HIGH
#elif defined(CONFIG_MALATA_D7014)
#define LCD_DRV_PIN         RK30_PIN0_PA3
#define LCD_DRV_VALUE       GPIO_HIGH
#else
#define LCD_DRV_PIN         INVALID_GPIO
#define LCD_DRV_VALUE       GPIO_HIGH
#endif

// Add this pin for additional LCD control sequence on C1017 v3.0, xmylm, 20130704
#if defined(CONFIG_MALATA_C1017)
#define LCD_VDD_EN          RK30_PIN1_PA6
#else
#define LCD_VDD_EN          INVALID_GPIO
#endif
#define LCD_VDD_EN_VALUE    GPIO_HIGH

static int rk_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
	int ret = 0;
   // Modified by EvanZeng for lcd EJ101H in project D1004, adjusting the power timing for resolving the "white screen"
#if defined(CONFIG_LCD_EJ101H)
   if(LCD_EN_PIN !=INVALID_GPIO)
   {
      ret = gpio_request(LCD_EN_PIN, NULL);
      if (ret != 0)
      {
         gpio_free(LCD_EN_PIN);
         printk(KERN_ERR "request lcd ens pin fail!\n");
         return -1;
      }
      else
      {
         printk("zengshanbin_test set LCD_EN_PIN enable\n");
         gpio_direction_output(LCD_EN_PIN, LCD_EN_VALUE);
      }
   }

/*	 #ifdef CONFIG_MALATA_D1013
	 msleep(200);
	 #else
   msleep(30); //[M]According to datasheet, modify 200ms->30ms to fix "white screen" when powerup && resume
   #endif*/
    #if defined(CONFIG_MALATA_D1004) && defined(CONFIG_LCD_EJ101H_D1004) || defined(CONFIG_MALATA_D1014)
    msleep(30); //[M]According to datasheet, modify 200ms->30ms to fix "white screen" when powerup && resume
    #else
    msleep(200);
    #endif

   if(LCD_CS_PIN !=INVALID_GPIO)
   {
      ret = gpio_request(LCD_CS_PIN, NULL);
      if (ret != 0)
      {
         gpio_free(LCD_CS_PIN);
         printk(KERN_ERR "request lcd cs pin fail!\n");
         return -1;
      }
      else
      {
         gpio_direction_output(LCD_CS_PIN, LCD_CS_VALUE);
         printk("zengshanbin_test set LCD_CS_PIN enable\n");
      }
   }
    #if defined(CONFIG_MALATA_D1004) && defined(CONFIG_LCD_EJ101H_D1004) || defined(CONFIG_MALATA_D1014)
    //donothing
    #else
    msleep(20);
    #endif

#else
	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_EN_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_EN_PIN);
			printk(KERN_ERR "request lcd en pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_EN_PIN, LCD_EN_VALUE);
			printk("honghaishen_test set LCD_EN_PIN able\n");
		}
	}
	mdelay(20);  //yangshl modify LCD power timing according LCD engineer

	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_CS_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_CS_PIN);
			printk(KERN_ERR "request lcd cs pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_CS_PIN, LCD_CS_VALUE);
		}
	}

#endif//Modifed by EvanZeng End

	if(LCD_DRV_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_DRV_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_DRV_PIN);
			printk(KERN_ERR "request lcd drv pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_DRV_PIN, LCD_DRV_VALUE);
		}
	}

	if (LCD_VDD_EN != INVALID_GPIO)
	{
		ret = gpio_request(LCD_VDD_EN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_VDD_EN);
			printk(KERN_ERR "request LCD_VDD_EN pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_VDD_EN, LCD_VDD_EN_VALUE);
		}
	}
	return 0;
}
static int fb_status(unsigned int on_off_get)
{
	static int status = 1;

	if (on_off_get >= FB_STATUS_GET)
	{
		return status;
	}
	else
	{
		status = on_off_get;
		return -1;
	}
}
static void fb_lcd_drv_ctl(bool on)
{
	if (LCD_DRV_PIN != INVALID_GPIO)
	{
		gpio_set_value(LCD_DRV_PIN, on ? LCD_DRV_VALUE : !LCD_DRV_VALUE);
	}
}
int rk_fb_io_disable(void)
{
#if  defined(LCD_DISP_ON_PIN) && defined(CONFIG_MALATA_D1013)
	int pwm_gpio;
	gpio_direction_output(BL_EN_PIN, !BL_EN_VALUE);
	mdelay(100);

	pwm_gpio = iomux_mode_to_gpio(PWM_MODE);
	gpio_request(pwm_gpio, "bl_pwm");
	gpio_direction_output(pwm_gpio, GPIO_LOW);
	mdelay(200);
#endif
	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, !LCD_CS_VALUE);
	}
	mdelay(20);
	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, !LCD_EN_VALUE);
		printk("honghaishen_test LCD EN PIN IS NO \n");
	}
	if (LCD_VDD_EN != INVALID_GPIO)
	{
		gpio_set_value(LCD_VDD_EN, !LCD_VDD_EN_VALUE);
	}
// Modified by EvanZeng for lcd EJ101H in project C1004, adjusting the power timing for resolving the "white screen"
#if defined(CONFIG_LCD_EJ101H)
    msleep(50);
#endif
// Modified by EvanZeng End
	if(LCD_DRV_PIN !=INVALID_GPIO)
	{
		#if (defined(CONFIG_MALATA_D1004) || defined(CONFIG_MALATA_D7803)) && defined(CONFIG_SND_SOC_RT5616) && !defined(CONFIG_MALATA_D7005)
		fb_status(FB_STATUS_OFF);
		#else
		gpio_set_value(LCD_DRV_PIN, !LCD_DRV_VALUE);
		#endif
	}

	return 0;
}
EXPORT_SYMBOL(rk_fb_io_disable);
static int rk_fb_io_enable(void)
{
// Modified by EvanZeng for lcd EJ101H in project D1004, adjusting the power timing for resolving the "white screen"
#if defined(CONFIG_LCD_EJ101H)
	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
		  if(LCD_EN_PIN == RK30_PIN0_PB0)
			printk("honghaishen_test LCD en pin is open \n");
		  printk("honghaishen_test LCD EN PIN IS OK \n");
	}
/*	 #ifdef CONFIG_MALATA_D1013
	 msleep(200);
	 #else
   msleep(30); //[M]According to datasheet, modify 200ms->30ms to fix "white screen" when powerup && resume
   #endif*/

    #if defined(CONFIG_MALATA_D1004) && defined(CONFIG_LCD_EJ101H_D1004) || defined(CONFIG_MALATA_D1014)
    msleep(30); //[M]According to datasheet, modify 200ms->30ms to fix "white screen" when powerup && resume
    #else
    msleep(200);
    #endif

   if(LCD_CS_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	}
    #if defined(CONFIG_MALATA_D1004) && defined(CONFIG_LCD_EJ101H_D1004) || defined(CONFIG_MALATA_D1014)
    //donothing
    #else
    msleep(20);
    #endif
#else
	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
		  if(LCD_EN_PIN == RK30_PIN0_PB0)
		  	printk("honghaishen_test LCD en pin is open \n");
		  printk("honghaishen_test LCD EN PIN IS OK \n");
	}
	mdelay(20); //yangshl modify LCD timing according LCD engineer 2013.09.30

	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	}

	if (LCD_VDD_EN != INVALID_GPIO)
	{
		gpio_set_value(LCD_VDD_EN, LCD_VDD_EN_VALUE);
	}
#endif// Modified by EvanZeng End
	if(LCD_DRV_PIN !=INVALID_GPIO)
	{
		#if defined(CONFIG_MALATA_D1004) && defined(CONFIG_SND_SOC_RT5616)
		fb_status(FB_STATUS_ON);
		#endif
		gpio_set_value(LCD_DRV_PIN, LCD_DRV_VALUE);
	}

	return 0;
}

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK3188)
#if defined(CONFIG_MALATA_D7803)
struct rk29lcd_info rk29lcd_infoe = {
 .reset_pin = RK30_PIN0_PD5,
};
#endif
struct rk29fb_info lcdc0_screen_info = {
	.prop	   = PRMRY,		//primary display device
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,
#if defined(CONFIG_MALATA_D7803)
	.lcd_info = &rk29lcd_infoe,
#endif
};
#endif

#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK3188)
struct rk29fb_info lcdc1_screen_info = {
	#if defined(CONFIG_RK_HDMI)
	.prop		= EXTEND,	//extend display device
	.lcd_info  = NULL,
	.set_screen_info = hdmi_init_lcdc,
	#endif
};
#endif

static struct resource resource_fb[] = {
	[0] = {
		.name  = "fb0 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "ipp buf",  //for rotate
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = "fb2 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device device_fb = {
	.name		= "rk-fb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_fb),
	.resource	= resource_fb,
};
#endif

#if defined(CONFIG_ARCH_RK3188)
static struct resource resource_mali[] = {
	[0] = {
	.name  = "ump buf",
	.start = 0,
	.end   = 0,
	.flags = IORESOURCE_MEM,
	},

};

static struct platform_device device_mali= {
	.name		= "mali400_ump",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_mali),
	.resource	= resource_mali,
};
#endif

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK3188)
static struct resource resource_lcdc0[] = {
	[0] = {
		.name  = "lcdc0 reg",
		.start = RK30_LCDC0_PHYS,
		.end   = RK30_LCDC0_PHYS + RK30_LCDC0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	
	[1] = {
		.name  = "lcdc0 irq",
		.start = IRQ_LCDC0,
		.end   = IRQ_LCDC0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc0 = {
	.name		  = "rk30-lcdc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(resource_lcdc0),
	.resource	  = resource_lcdc0,
	.dev 		= {
		.platform_data = &lcdc0_screen_info,
	},
};
#endif
#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK3188)
static struct resource resource_lcdc1[] = {
	[0] = {
		.name  = "lcdc1 reg",
		.start = RK30_LCDC1_PHYS,
		.end   = RK30_LCDC1_PHYS + RK30_LCDC1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "lcdc1 irq",
		.start = IRQ_LCDC1,
		.end   = IRQ_LCDC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc1 = {
	.name		  = "rk30-lcdc",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(resource_lcdc1),
	.resource	  = resource_lcdc1,
	.dev 		= {
		.platform_data = &lcdc1_screen_info,
	},
};
#endif

#if defined(CONFIG_MFD_RK610)
#define RK610_RST_PIN 			RK30_PIN3_PB2//RK30_PIN2_PC5
static int rk610_power_on_init(void)
{
	int ret;
	if(RK610_RST_PIN != INVALID_GPIO)
	{
//		rk30_mux_api_set(RK610_RST_PIN_MUX_NAME,RK610_RST_PIN_MUX_MODE);
		ret = gpio_request(RK610_RST_PIN, "rk610 reset");
		if (ret)
		{
			printk(KERN_ERR "rk610_control_probe request gpio fail\n");
		}
		else 
		{
			gpio_direction_output(RK610_RST_PIN, GPIO_HIGH);
			msleep(100);
			gpio_direction_output(RK610_RST_PIN, GPIO_LOW);
			msleep(100);
	    		gpio_set_value(RK610_RST_PIN, GPIO_HIGH);
		}
		printk("%s........rst ok\n",__func__);
	}

	return 0;
	
}


static struct rk610_ctl_platform_data rk610_ctl_pdata = {
	.rk610_power_on_init = rk610_power_on_init,
};
#endif

#ifdef CONFIG_SND_SOC_RK610
static int rk610_codec_io_init(void)
{
//if need iomux.
//Must not gpio_request
	return 0;
}

static struct rk610_codec_platform_data rk610_codec_pdata = {
	.spk_ctl_io = RK30_PIN2_PD7,//RK30_PIN4_PC6,
	.io_init = rk610_codec_io_init,
};
#endif

#ifdef CONFIG_SND_SOC_RT5616
static int rt5616_codec_io_init(void)
{
//if need iomux.
//Must not gpio_request
	return 0;
}

static struct rt5616_codec_platform_data rt5616_codec_pdata = {
	.spk_ctl_io = RK30_PIN2_PD7,//RK30_PIN4_PC6,
	.io_init = rt5616_codec_io_init,
	.fb_status = fb_status,
	.fb_lcd_drv_ctl = fb_lcd_drv_ctl,
};
unsigned int rt5616_codec_spk_io;
#endif
#ifdef CONFIG_RK_HDMI
#define RK_HDMI_RST_PIN 			RK30_PIN3_PB2
static int rk_hdmi_power_init(void)
{
	int ret;

	if(RK_HDMI_RST_PIN != INVALID_GPIO)
	{
		if (gpio_request(RK_HDMI_RST_PIN, NULL)) {
			printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
			return -1;
		}
		gpio_direction_output(RK_HDMI_RST_PIN, GPIO_LOW);
		gpio_set_value(RK_HDMI_RST_PIN, GPIO_LOW);
		msleep(100);
		gpio_set_value(RK_HDMI_RST_PIN, GPIO_HIGH);
		msleep(50);
	}
	return 0;
}
static struct rk_hdmi_platform_data rk_hdmi_pdata = {
	.io_init = rk_hdmi_power_init,
};
#endif

#ifdef CONFIG_ANDROID_TIMED_GPIO
static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = RK30_PIN0_PC7,
		.max_timeout = 1000,
		.active_low = 0,
#if (defined(CONFIG_MALATA_D7803)) && (!defined(CONFIG_MALATA_D7806))
		.adjust_time =0,      //adjust for diff product
#elif defined(CONFIG_MALATA_D7806)
		.adjust_time =30,
#else
		.adjust_time =20,
#endif
	},
};

static struct timed_gpio_platform_data rk29_vibrator_info = {
	.num_gpios = 1,
	.gpios = timed_gpios,
};

static struct platform_device rk29_device_vibrator = {
	.name = "timed-gpio",
	.id = -1,
	.dev = {
		.platform_data = &rk29_vibrator_info,
	},

};
#endif

#ifdef CONFIG_DP_ANX6345

	#define DVDD33_EN_PIN 		RK30_PIN0_PB0
	#define DVDD33_EN_VALUE 	GPIO_LOW
	
	#define DVDD18_EN_PIN 		RK30_PIN3_PD4//RK30_PIN3_PD4//RK30_PIN1_PB6//RK30_PIN4_PC7
	#define DVDD18_EN_VALUE 	GPIO_HIGH

	#define EDP_RST_PIN 		RK30_PIN0_PB4
	static int rk_edp_power_ctl(void)
	{
		int ret;
		ret = gpio_request(DVDD33_EN_PIN, "dvdd33_en_pin");
		if (ret != 0)
		{
			gpio_free(DVDD33_EN_PIN);
			printk(KERN_ERR "request dvdd33 en pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(DVDD33_EN_PIN, DVDD33_EN_VALUE);
		}
		msleep(5);
		
		ret = gpio_request(DVDD18_EN_PIN, "dvdd18_en_pin");
		if (ret != 0)
		{
			gpio_free(DVDD18_EN_PIN);
			printk(KERN_ERR "request dvdd18 en pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(DVDD18_EN_PIN, DVDD18_EN_VALUE);
		}
		
		ret = gpio_request(EDP_RST_PIN, "edp_rst_pin");
		if (ret != 0)
		{
			gpio_free(EDP_RST_PIN);
			printk(KERN_ERR "request rst pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(EDP_RST_PIN, GPIO_LOW);
			msleep(50);
			gpio_direction_output(EDP_RST_PIN, GPIO_HIGH);
		}
		return 0;
		
	}
	static struct anx6345_platform_data anx6345_platform_data = {
		.power_ctl 	= rk_edp_power_ctl,
		.dvdd33_en_pin 	= DVDD33_EN_PIN,
		.dvdd33_en_val 	= DVDD33_EN_VALUE,
		.dvdd18_en_pin 	= DVDD18_EN_PIN,
		.dvdd18_en_val 	= DVDD18_EN_VALUE,
		.edp_rst_pin   	= EDP_RST_PIN,
	};
#endif

#ifdef CONFIG_LEDS_GPIO_PLATFORM
static struct gpio_led rk29_leds[] = {
	{
		.name = "button-backlight",
		.gpio = RK30_PIN2_PB3,
		.default_trigger = "timer",
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data rk29_leds_pdata = {
	.leds = rk29_leds,
	.num_leds = ARRAY_SIZE(rk29_leds),
};

static struct platform_device rk29_device_gpio_leds = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data  = &rk29_leds_pdata,
	},
};
#endif

#ifdef CONFIG_RK_IRDA
#define IRDA_IRQ_PIN           RK30_PIN0_PA3

static int irda_iomux_init(void)
{
	int ret = 0;

	//irda irq pin
	ret = gpio_request(IRDA_IRQ_PIN, NULL);
	if (ret != 0) {
		gpio_free(IRDA_IRQ_PIN);
		printk(">>>>>> IRDA_IRQ_PIN gpio_request err \n ");
	}
	gpio_pull_updown(IRDA_IRQ_PIN, PullDisable);
	gpio_direction_input(IRDA_IRQ_PIN);

	return 0;
}

static int irda_iomux_deinit(void)
{
	gpio_free(IRDA_IRQ_PIN);
	return 0;
}

static struct irda_info rk29_irda_info = {
	.intr_pin = IRDA_IRQ_PIN,
	.iomux_init = irda_iomux_init,
	.iomux_deinit = irda_iomux_deinit,
	//.irda_pwr_ctl = bu92747guw_power_ctl,
};

static struct platform_device irda_device = {
#ifdef CONFIG_RK_IRDA_NET
	.name = "rk_irda",
#else
	.name = "bu92747_irda",
#endif
	.id = -1,
	.dev = {
		.platform_data = &rk29_irda_info,
	}
};
#endif

#ifdef CONFIG_ION
#define ION_RESERVE_SIZE        (80 * SZ_1M)
static struct ion_platform_data rk30_ion_pdata = {
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_NOR_HEAP_ID,
			.name = "norheap",
			.size = ION_RESERVE_SIZE,
		}
	},
};

static struct platform_device device_ion = {
	.name = "ion-rockchip",
	.id = 0,
	.dev = {
		.platform_data = &rk30_ion_pdata,
	},
};
#endif

/**************************************************************************************************
 * SDMMC devices,  include the module of SD,MMC,and sdio.noted by xbw at 2012-03-05
**************************************************************************************************/
#ifdef CONFIG_SDMMC_RK29
#include "board-LR097-sdmmc-config.c"
#include "../plat-rk/rk-sdmmc-ops.c"
#include "../plat-rk/rk-sdmmc-wifi.c"
#endif //endif ---#ifdef CONFIG_SDMMC_RK29

#ifdef CONFIG_SDMMC0_RK29
static int rk29_sdmmc0_cfg_gpio(void)
{
#ifdef CONFIG_SDMMC_RK29_OLD
	iomux_set(MMC0_CMD);
	iomux_set(MMC0_CLKOUT);
	iomux_set(MMC0_D0);
	iomux_set(MMC0_D1);
	iomux_set(MMC0_D2);
	iomux_set(MMC0_D3);

	iomux_set_gpio_mode(iomux_mode_to_gpio(MMC0_DETN));

	gpio_request(RK30_PIN3_PA7, "sdmmc-power");
	gpio_direction_output(RK30_PIN3_PA7, GPIO_LOW);

#else
	rk29_sdmmc_set_iomux(0, 0xFFFF);

    #if defined(CONFIG_SDMMC0_RK29_SDCARD_DET_FROM_GPIO)
        #if SDMMC_USE_NEW_IOMUX_API
        iomux_set_gpio_mode(iomux_gpio_to_mode(RK29SDK_SD_CARD_DETECT_N));
        #else
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO);
        #endif
    #else
        #if SDMMC_USE_NEW_IOMUX_API       
        iomux_set(MMC0_DETN);
        #else
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FMUX);
        #endif
    #endif	

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	gpio_request(SDMMC0_WRITE_PROTECT_PIN, "sdmmc-wp");
	gpio_direction_input(SDMMC0_WRITE_PROTECT_PIN);
#endif

#endif

	return 0;
}

#define CONFIG_SDMMC0_USE_DMA
struct rk29_sdmmc_platform_data default_sdmmc0_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36),
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc0_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif

	.dma_name = "sd_mmc",
#ifdef CONFIG_SDMMC0_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC) && defined(CONFIG_USE_SDMMC0_FOR_WIFI_DEVELOP_BOARD)
    .status = rk29sdk_wifi_mmc0_status,
    .register_status_notify = rk29sdk_wifi_mmc0_status_register,
#endif

#if defined(RK29SDK_SD_CARD_PWR_EN) || (INVALID_GPIO != RK29SDK_SD_CARD_PWR_EN)
    .power_en = RK29SDK_SD_CARD_PWR_EN,
    .power_en_level = RK29SDK_SD_CARD_PWR_EN_LEVEL,
#else
    .power_en = INVALID_GPIO,
    .power_en_level = GPIO_LOW,
#endif    
	.enable_sd_wakeup = 0,

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	.write_prt = SDMMC0_WRITE_PROTECT_PIN,
	.write_prt_enalbe_level = SDMMC0_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    .det_pin_info = {    
    #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N, //INVALID_GPIO,
	#if defined(CONFIG_MALATA_D7022)
		.enable         = RK29SDK_SD_CARD_INSERT_LEVEL_H,
		#else
        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
		#endif
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
    #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
    #endif    
    }, 

};
#endif // CONFIG_SDMMC0_RK29

#ifdef CONFIG_SDMMC1_RK29
#define CONFIG_SDMMC1_USE_DMA
static int rk29_sdmmc1_cfg_gpio(void)
{
#if defined(CONFIG_SDMMC_RK29_OLD)
	iomux_set(MMC1_CMD);
	iomux_set(MMC1_CLKOUT);
	iomux_set(MMC1_D0);
	iomux_set(MMC1_D1);
	iomux_set(MMC1_D2);
	iomux_set(MMC1_D3);
#else

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	gpio_request(SDMMC1_WRITE_PROTECT_PIN, "sdio-wp");
	gpio_direction_input(SDMMC1_WRITE_PROTECT_PIN);
#endif

#endif

	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc1_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34),

#if !defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
		      MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#else
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#endif

	.io_init = rk29_sdmmc1_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif

	.dma_name = "sdio",
#ifdef CONFIG_SDMMC1_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_CONTROL_FUNC) || defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
    .status = rk29sdk_wifi_status,
    .register_status_notify = rk29sdk_wifi_status_register,
#endif

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	.write_prt = SDMMC1_WRITE_PROTECT_PIN,
	    .write_prt_enalbe_level = SDMMC1_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    #if defined(CONFIG_RK29_SDIO_IRQ_FROM_GPIO)
        .sdio_INT_gpio = RK29SDK_WIFI_SDIO_CARD_INT,
    #endif

    .det_pin_info = {    
#if defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
     #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N,
     #else
         .io             = INVALID_GPIO,
     #endif   

        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
 #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
#endif
    },
   
	.enable_sd_wakeup = 0,
};
#endif //endif--#ifdef CONFIG_SDMMC1_RK29

/**************************************************************************************************
 * the end of setting for SDMMC devices
**************************************************************************************************/

#if defined (CONFIG_BATTERY_BQ24196)
#define	CHG_EN	RK30_PIN0_PC1

#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
#if defined (CONFIG_MALATA_D7803)
#define	OTG_EN	RK30_PIN0_PC4
#define	OTG_IRQ	RK30_PIN3_PB1
#define	STATU_IRQ	RK30_PIN0_PD7
#else
#define	OTG_EN	RK30_PIN0_PB1
#define	OTG_IRQ	RK30_PIN3_PB1
#define	STATU_IRQ	RK30_PIN0_PD7
#endif
static int bq24196_otg_irq_init(void)
{
	int ret = 0;

	if(OTG_IRQ != INVALID_GPIO){
		ret = gpio_request(OTG_IRQ, "otg_irq");
		if(ret)
			return ret;

		gpio_pull_updown(OTG_IRQ, GPIOPullUp);
		gpio_direction_input(OTG_IRQ);
	}

	if(STATU_IRQ != INVALID_GPIO){
		ret = gpio_request(STATU_IRQ, "status_irq");
		if(ret)
			return ret;

		gpio_pull_updown(STATU_IRQ, GPIOPullUp);
		gpio_direction_input(STATU_IRQ);
	}
	return ret;
}
#endif

struct bq24196_platform_data bq24196_info = {
	.chg_en_pin=CHG_EN,
#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
	.otg_en_pin = OTG_EN,
	.otg_irq_pin = OTG_IRQ,
	.status_irq_pin = STATU_IRQ,
	.irq_init = bq24196_otg_irq_init,
#endif
};

void bq24196_charge_en(void)
{
	int ret = 0;

	if(CHG_EN != INVALID_GPIO){
		ret = gpio_request(CHG_EN, "charge_en");
		if (ret) {
			printk("failed to request charge_en gpio\n");
			return;
		}

		gpio_pull_updown(CHG_EN, 0);
		ret = gpio_direction_output(CHG_EN, 0);
		if (ret) {
			printk("failed to set gpio charge_en output\n");
			return;
		}
		gpio_free(CHG_EN);
	}

}

void bq24196_charge_disable(void)
{
	int ret = 0;

	if(CHG_EN != INVALID_GPIO){
		ret = gpio_request(CHG_EN, "charge_en");
		if (ret) {
			printk("failed to request charge_en gpio\n");
			return;
		}

		gpio_pull_updown(CHG_EN, 0);
		ret = gpio_direction_output(CHG_EN, 1);
		if (ret) {
			printk("failed to set gpio charge_en output\n");
			return;
		}
		gpio_free(CHG_EN);
	}

}


extern int bq24196_set_input_current(int);

#endif

#ifdef CONFIG_BATTERY_RK30_ADC_FAC
#ifdef CONFIG_BATTERY_RK30_USB_AND_CHARGE
#define DC_DET_PIN		INVALID_GPIO
#else
#define DC_DET_PIN		RK30_PIN0_PB2
#endif

#ifdef CONFIG_BATTERY_RK30_USB_CHARGE
#define USB_SUPPORT	1
#else
#define USB_SUPPORT	0
#endif

#define USB_DET_PIN	RK30_PIN0_PA7
#define CHARGE_TYPE_PIN	INVALID_GPIO

void charge_current_set(int on)
{
	int ret = 0, value = 0;
	int charge_current_pin = CHARGE_TYPE_PIN;

	if(charge_current_pin != INVALID_GPIO){
		ret = gpio_request(charge_current_pin, NULL);
		if (ret) {
			printk("failed to request charge_current_pin gpio%d\n", charge_current_pin);
			return;
		}
		value = gpio_get_value(charge_current_pin);
		if(value != on){
			gpio_direction_output(charge_current_pin, on);
		//	printk("charge_current_set %s\n", on ? "2000mA" : "500mA");
		}
		gpio_free(charge_current_pin);
	}
}

static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
	.dc_det_pin      = DC_DET_PIN,//INVALID_GPIO,
	.batt_low_pin    = INVALID_GPIO,
	.charge_set_pin  = INVALID_GPIO,
	.charge_ok_pin   = RK30_PIN0_PA6,
	.dc_det_level    = GPIO_LOW,
	.charge_ok_level = GPIO_HIGH,
	.usb_det_pin     = USB_DET_PIN,
	.usb_det_level   = GPIO_LOW,
	.back_light_pin = BL_EN_PIN,

	.charging_sleep   = 0 ,
	.save_capacity   = 1 ,
	.is_reboot_charging = 1,
	.adc_channel      =0 ,
	.spport_usb_charging = USB_SUPPORT,
#if defined (CONFIG_BATTERY_BQ24196)
	.control_usb_charging = bq24196_set_input_current,
#else
	.control_usb_charging = charge_current_set,
#endif
};

static struct platform_device rk30_device_adc_battery = {
        .name   = "rk30-battery",
        .id     = -1,
        .dev = {
                .platform_data = &rk30_adc_battery_platdata,
        },
};
#endif

#ifdef CONFIG_BATTERY_RK30_ADC
static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
        .dc_det_pin      = RK30_PIN0_PB2,
        .batt_low_pin    = RK30_PIN0_PB1, 
        .charge_set_pin  = INVALID_GPIO,
        .charge_ok_pin   = RK30_PIN0_PA6,
        .dc_det_level    = GPIO_LOW,
        .charge_ok_level = GPIO_HIGH,
	 .adc_channel=0,
	 .spport_usb_charging = 1,
	 .is_usb_charging=1,

	 .pull_up_res=200,
	 .pull_down_res=120,
	 .reference_voltage=1800,
};


static struct platform_device rk30_device_adc_battery = {
        .name   = "rk30-battery",
        .id     = -1,
        .dev = {
                .platform_data = &rk30_adc_battery_platdata,
        },
};
#endif

#if defined (CONFIG_BATTERY_BQ27541)
#define BQ_DC_DET_PIN		DC_DET_PIN
#define BQ_USB_DET_PIN		USB_DET_PIN

void bq24751_dc_detect_init(void)
{
	int ret = 0;

	if(BQ_DC_DET_PIN != INVALID_GPIO){
		ret = gpio_request(BQ_DC_DET_PIN, "dc_det");
		if (ret) {
			printk("failed to request BQ_DC_DET_PIN gpio\n");
			return;
		}

		gpio_pull_updown(BQ_DC_DET_PIN, 1);
		ret = gpio_direction_input(BQ_DC_DET_PIN);
		if (ret) {
			printk("failed to set gpio BQ_DC_DET_PIN input\n");
			return;
		}
		gpio_free(BQ_DC_DET_PIN);
	}

	if(BQ_USB_DET_PIN != INVALID_GPIO){
		ret = gpio_request(BQ_USB_DET_PIN, "usb_det");
		if (ret) {
			printk("failed to request BQ_USB_DET_PIN gpio\n");
			return;
		}

		gpio_pull_updown(BQ_USB_DET_PIN, 1);
		ret = gpio_direction_input(BQ_USB_DET_PIN);
		if (ret) {
			printk("failed to set gpio BQ_USB_DET_PIN input\n");
			return;
		}
		gpio_free(BQ_USB_DET_PIN);
	}

}

static int bq27541_charging_stat(void)
{
	return (!gpio_get_value(BQ_DC_DET_PIN)) || (!gpio_get_value(BQ_USB_DET_PIN));
}

struct bq27541_platform_data bq27541_data = {
	.capacity_max = 100,
#if defined(CONFIG_BATTERY_BT_B0BF_3574108)
	.capacity_min = 4,
#elif defined(CONFIG_BATTERY_BT_B0BFS_4571107)
	.capacity_min = 5,
#elif defined(CONFIG_BATTERY_BT_B0BD)
	.capacity_min = 4,
#elif defined(CONFIG_BATTERY_BT_B0B5G)
	.capacity_min = 8,
#else
	.capacity_min = 5,
#endif
	.bat_num = 1,
	.dc_check_pin = BQ_DC_DET_PIN,
	.init_dc_check_pin = bq24751_dc_detect_init,
	.get_charging_stat = bq27541_charging_stat,
};
#endif

#if defined (CONFIG_BATTERY_BQ27410)
#define BQ27410_DC_DET_PIN		DC_DET_PIN
#define BQ27410_USB_DET_PIN		USB_DET_PIN
#define BQ27410_LOW_POWER_PIN	INVALID_GPIO

void bq27410_io_init(void)
{
	int ret = 0;

	if(BQ27410_DC_DET_PIN != INVALID_GPIO){
		ret = gpio_request(BQ27410_DC_DET_PIN, "dc_det");
		if (ret) {
			printk("failed to request BQ27410_DC_DET_PIN gpio\n");
		}

		gpio_pull_updown(BQ27410_DC_DET_PIN, 1);
		ret = gpio_direction_input(BQ27410_DC_DET_PIN);
		if (ret) {
			printk("failed to set gpio BQ27410_DC_DET_PIN input\n");
		}
		gpio_free(BQ27410_DC_DET_PIN);
	}

	if(BQ27410_USB_DET_PIN != INVALID_GPIO){
		ret = gpio_request(BQ27410_USB_DET_PIN, "usb_det");
		if (ret) {
			printk("failed to request BQ27410_USB_DET_PIN gpio\n");
		}

		gpio_pull_updown(BQ27410_USB_DET_PIN, 1);
		ret = gpio_direction_input(BQ27410_USB_DET_PIN);
		if (ret) {
			printk("failed to set gpio BQ27410_USB_DET_PIN input\n");
		}
		gpio_free(BQ27410_USB_DET_PIN);
	}

	if(BQ27410_LOW_POWER_PIN != INVALID_GPIO){
		ret = gpio_request(BQ27410_LOW_POWER_PIN, "low_power");
		if (ret) {
			printk("failed to request BQ27410_LOW_POWER_PIN gpio\n");
		}

		gpio_pull_updown(BQ27410_LOW_POWER_PIN, 1);
		ret = gpio_direction_input(BQ27410_LOW_POWER_PIN);
		if (ret) {
			printk("failed to set gpio BQ27410_LOW_POWER_PIN input\n");
		}
		gpio_free(BQ27410_LOW_POWER_PIN);
	}
}

static int bq27410_charging_stat(void)
{
	return (!gpio_get_value(BQ27410_DC_DET_PIN)) || (!gpio_get_value(BQ27410_USB_DET_PIN));
}

struct bq27410_platform_data bq27410_data = {
	.capacity_max = 100,
#if defined (CONFIG_MALATA_D7803)
	.capacity_min = 3,
#else
	.capacity_min = 5,
#endif
	.bat_num = 1,
	.dc_check_pin = BQ27410_DC_DET_PIN,
	.wake_irq = BQ27410_LOW_POWER_PIN,
	.io_init = bq27410_io_init,
	.get_charging_stat = bq27410_charging_stat,
	.low_power_pin = INVALID_GPIO,
};
#endif

#ifdef CONFIG_CW2015_BATTERY
/*
   note the follow array must set depend on the battery that you use
   you must send the battery to cellwise-semi the contact information:
   name: chen gan; tel:13416876079; E-mail: ben.chen@cellwise-semi.com
 */
static u8 config_info[SIZE_BATINFO] = {
	0x15, 0x95, 0x5D, 0x60, 0x61,
	0x5E, 0x58, 0x53, 0x4F, 0x4B,
	0x47, 0x41, 0x3E, 0x39, 0x37,
	0x2E, 0x25, 0x1D, 0x18, 0x16,
	0x1B, 0x27, 0x3D, 0x48, 0x48,
	0x44, 0x0C, 0xCD, 0x27, 0x47,
	0x54, 0x6A, 0x60, 0x5D, 0x59,
	0x5C, 0x3E, 0x19, 0x2E, 0x35,
	0x00, 0x2A, 0x52, 0x87, 0x8F,
	0x91, 0x94, 0x52, 0x82, 0x8C,
	0x92, 0x96, 0x82, 0x89, 0xC5,
	0xCB, 0x2F, 0x7D, 0x72, 0xA5,
	0xB5, 0xC1, 0x57, 0x0B

};

#ifdef CONFIG_BATTERY_AC_CHARGE
#define DC_DET_PIN		RK30_PIN0_PB2
#else
#define DC_DET_PIN		INVALID_GPIO
#endif

#ifdef CONFIG_BATTERY_USB_CHARGE
#define USB_DET_PIN		RK30_PIN0_PA7
#else
#define USB_DET_PIN		INVALID_GPIO
#endif

static struct cw_bat_platform_data cw_bat_platdata = {
	.is_dc_charge	= 1,
	.is_usb_charge	= 1,
	.dc_det_pin		= DC_DET_PIN,
    .bat_low_pin    = INVALID_GPIO,
    .usb_det_pin	= USB_DET_PIN,
    .chg_ok_pin		= RK30_PIN0_PA6,
    .chg_mode_sel_pin	= INVALID_GPIO,
    .dc_det_level	= GPIO_LOW,
    .usb_det_level	= GPIO_LOW,
    .bat_low_level  = GPIO_LOW,
    .chg_ok_level	= GPIO_HIGH,
    .capacity_max = 100,
#if defined(CONFIG_MALATA_D7806)
	.capacity_min = 3,
#else
	.capacity_min = 5,
#endif
    .cw_bat_config_info	= config_info,
    #if defined (CONFIG_BATTERY_BQ24196)
	.charge_current_set = bq24196_set_input_current,
    #endif
};
#endif

#ifdef CONFIG_RK30_PWM_REGULATOR
static int pwm_voltage_map[] = {
	800000,825000,850000, 875000,900000, 925000 ,950000, 975000,1000000, 1025000, 1050000, 1075000, 1100000, 1125000, 1150000, 1175000, 1200000, 1225000, 1250000, 1275000, 1300000, 1325000, 1350000,1375000
};
static struct regulator_consumer_supply pwm_dcdc1_consumers[] = {
	{
		.supply = "vdd_cpu",
	}
};

struct regulator_init_data pwm_regulator_init_dcdc[1] =
{
	{
		.constraints = {
			.name = "PWM_DCDC1",
			.min_uV = 600000,
			.max_uV = 1800000,	//0.6-1.8V
			.apply_uV = true,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(pwm_dcdc1_consumers),
		.consumer_supplies = pwm_dcdc1_consumers,
	},
};

static struct pwm_platform_data pwm_regulator_info[1] = {
	{
		.pwm_id = 1,
		.pwm_gpio = RK30_PIN3_PD4,
		.pwm_iomux_pwm = PWM1,
		.pwm_iomux_gpio = GPIO3_D4,
		.pwm_voltage = 1100000,
		.suspend_voltage = 1000000,
		.min_uV = 800000,
		.max_uV	= 1375000,
		.coefficient = 575,	//57.5%
		.pwm_voltage_map = pwm_voltage_map,
		.init_data	= &pwm_regulator_init_dcdc[0],
	},
};

struct platform_device pwm_regulator_device[1] = {
	{
		.name = "pwm-voltage-regulator",
		.id = 0,
		.dev		= {
			.platform_data = &pwm_regulator_info[0],
		}
	},
};
#endif

#ifdef CONFIG_RK29_VMAC
#define PHY_PWR_EN_GPIO	RK30_PIN1_PD6
#include "board-rk30-sdk-vmac.c"
#endif

#ifdef CONFIG_RFKILL_RK
// bluetooth rfkill device, its driver in net/rfkill/rfkill-rk.c
static struct rfkill_rk_platform_data rfkill_rk_platdata = {
    .type               = RFKILL_TYPE_BLUETOOTH,

    .poweron_gpio       = { // BT_REG_ON
        .io             = RK30_PIN3_PC7,
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = "bt_poweron",
            .fgpio      = GPIO3_C7,
        },
    },

    .reset_gpio         = { // BT_RST
        .io             = RK30_PIN3_PD1, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = "bt_reset",
            .fgpio      = GPIO3_C7,
       },
   }, 

    .wake_gpio          = { // BT_WAKE, use to control bt's sleep and wakeup
        .io             = RK30_PIN3_PC6, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = "bt_wake",
            .fgpio      = GPIO3_C6,
        },
    },

    .wake_host_irq      = { // BT_HOST_WAKE, for bt wakeup host when it is in deep sleep
        .gpio           = {
            .io         = RK30_PIN0_PA5, // set io to INVALID_GPIO for disable it
            .enable     = GPIO_LOW,      // set GPIO_LOW for falling, set 0 for rising
            .iomux      = {
                .name   = NULL,
            },
        },
    },

    .rts_gpio           = { // UART_RTS, enable or disable BT's data coming
        .io             = RK30_PIN1_PA3, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = "bt_rts",
            .fgpio      = GPIO1_A3,
            .fmux       = UART0_RTSN,
        },
    },
};

static struct platform_device device_rfkill_rk = {
    .name   = "rfkill_rk",
    .id     = -1,
    .dev    = {
        .platform_data = &rfkill_rk_platdata,
    },
};
#endif

#if defined(CONFIG_GPS_RK)
int rk_gps_io_init(void)
{
	printk("%s \n", __FUNCTION__);
	
	gpio_request(RK30_PIN1_PB5, NULL);
	gpio_direction_output(RK30_PIN1_PB5, GPIO_LOW);

	iomux_set(GPS_RFCLK);//GPS_CLK
	iomux_set(GPS_MAG);//GPS_MAG
	iomux_set(GPS_SIG);//GPS_SIGN

	gpio_request(RK30_PIN1_PA6, NULL);
	gpio_direction_output(RK30_PIN1_PA6, GPIO_LOW);

	gpio_request(RK30_PIN1_PA5, NULL);
	gpio_direction_output(RK30_PIN1_PA5, GPIO_LOW);	

	gpio_request(RK30_PIN1_PA7, NULL);
	gpio_direction_output(RK30_PIN1_PA7, GPIO_LOW);		
	return 0;
}
int rk_gps_power_up(void)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}

int rk_gps_power_down(void)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}

int rk_gps_reset_set(int level)
{
	return 0;
}
int rk_enable_hclk_gps(void)
{
	printk("%s \n", __FUNCTION__);
	clk_enable(clk_get(NULL, "hclk_gps"));
	return 0;
}
int rk_disable_hclk_gps(void)
{
	printk("%s \n", __FUNCTION__);
	clk_disable(clk_get(NULL, "hclk_gps"));
	return 0;
}
struct rk_gps_data rk_gps_info = {
	.io_init = rk_gps_io_init,
	.power_up = rk_gps_power_up,
	.power_down = rk_gps_power_down,
	.reset = rk_gps_reset_set,
	.enable_hclk_gps = rk_enable_hclk_gps,
	.disable_hclk_gps = rk_disable_hclk_gps,
	.GpsSign = RK30_PIN1_PB3,
	.GpsMag = RK30_PIN1_PB2,        //GPIO index
	.GpsClk = RK30_PIN1_PB4,        //GPIO index
	.GpsVCCEn = RK30_PIN1_PB5,     //GPIO index
	.GpsSpi_CSO = RK30_PIN1_PA4,    //GPIO index
	.GpsSpiClk = RK30_PIN1_PA5,     //GPIO index
	.GpsSpiMOSI = RK30_PIN1_PA7,	  //GPIO index
	.GpsIrq = IRQ_GPS,
	.GpsSpiEn = 0,
	.GpsAdcCh = 2,
	.u32GpsPhyAddr = RK30_GPS_PHYS,
	.u32GpsPhySize = RK30_GPS_SIZE,
};

struct platform_device rk_device_gps = {
	.name = "gps_hv5820b",
	.id = -1,
	.dev		= {
	.platform_data = &rk_gps_info,
		}
	};
#endif

#if defined(CONFIG_MT5931_MT6622)
static struct mt6622_platform_data mt6622_platdata = {
    .power_gpio         = { // BT_REG_ON
        .io             = RK30_PIN3_PC7, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = "mt6622_power",
			.fgpio		= GPIO3_C7,
        },
    },

    .reset_gpio         = { // BT_RST
        .io             = RK30_PIN3_PD1,
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = NULL,
        },
    },

    .irq_gpio           = {
        .io             = RK30_PIN0_PA5,
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = NULL,
        },
    },

    .rts_gpio           = { // UART_RTS
        .io             = RK30_PIN1_PA3,
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = "bt_rts",
            .fgpio      = GPIO1_A3,
            .fmux       = UART0_RTSN,
        },
    },
};

static struct platform_device device_mt6622 = {
    .name   = "mt6622",
    .id     = -1,
    .dev    = {
        .platform_data = &mt6622_platdata,
    },
};	

#endif

static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_SSD2828_RGB2MIPI
	&device_ssd2828,
#endif
#ifdef CONFIG_ION
	&device_ion,
#endif
#ifdef CONFIG_ANDROID_TIMED_GPIO
	&rk29_device_vibrator,
#endif
#ifdef CONFIG_LEDS_GPIO_PLATFORM
	&rk29_device_gpio_leds,
#endif
#ifdef CONFIG_RK_IRDA
	&irda_device,
#endif
#ifdef CONFIG_WIFI_CONTROL_FUNC
	&rk29sdk_wifi_device,
#endif
#ifdef CONFIG_RK29_SUPPORT_MODEM
	&rk30_device_modem,
#endif
#if defined(CONFIG_MU509)
	&rk29_device_mu509,
#endif
#if defined(CONFIG_ME906E)
	&rk29_device_me906e,
#endif
#if defined(CONFIG_MW100)
	&rk29_device_mw100,
#endif
#if defined(CONFIG_MT6229)
	&rk29_device_mt6229,
#endif
#if defined(CONFIG_MT6229_UNAP)
	&rk29_device_mt6229_unap,
#endif
#if defined(CONFIG_BATTERY_RK30_ADC) || defined(CONFIG_BATTERY_RK30_ADC_FAC)
 	&rk30_device_adc_battery,
#endif
#ifdef CONFIG_RFKILL_RK
	&device_rfkill_rk,
#endif
#ifdef CONFIG_GPS_RK
	&rk_device_gps,
#endif
#ifdef CONFIG_MT5931_MT6622
        &device_mt6622,
#endif
#if defined(CONFIG_ARCH_RK3188)
	&device_mali,
#endif

};
static int rk_platform_add_display_devices(void)
{
	struct platform_device *fb = NULL;  //fb
	struct platform_device *lcdc0 = NULL; //lcdc0
	struct platform_device *lcdc1 = NULL; //lcdc1
	struct platform_device *bl = NULL; //backlight
#ifdef CONFIG_FB_ROCKCHIP
	fb = &device_fb;
#endif

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK3188)
	lcdc0 = &device_lcdc0,
#endif

#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK3188)
	lcdc1 = &device_lcdc1,
#endif

#ifdef CONFIG_BACKLIGHT_RK29_BL
	bl = &rk29_device_backlight,
#endif
	__rk_platform_add_display_devices(fb,lcdc0,lcdc1,bl);

	return 0;
	
}
// i2c
#ifdef CONFIG_I2C0_RK30
static struct i2c_board_info __initdata i2c0_info[] = {
#if defined (CONFIG_GS_MMA8452)
	{
		.type	        = "gs_mma8452",
		.addr	        = 0x1d,
		.flags	        = 0,
		.irq	        = MMA8452_INT_PIN,
		.platform_data = &mma8452_info,
	},
#endif
#if defined (CONFIG_GS_MMA7660)
	{
		.type	        = "gs_mma7660",
		.addr	        = 0x4c,
		.flags	        = 0,
		.irq	        = MMA7660_INT_PIN,
		.platform_data = &mma7660_info,
	},
#endif
#if defined (CONFIG_GS_LIS3DE)
	{
		.type	        = "gs_lis3de",
		.addr	        = 0x29,   //0x29(SA0-->VCC), 0x28(SA0-->GND)
		.flags	        = 0,
		.irq	        = LIS3DE_INT_PIN,
		.platform_data = &lis3de_info,
	},
#endif
#if defined (CONFIG_GS_LIS3DH)
	{
		.type	        = "gs_lis3dh",
		.addr	        = 0x19,   //0x19(SA0-->VCC), 0x18(SA0-->GND)
		.flags	        = 0,
		.irq	        = LIS3DH_INT_PIN,
		.platform_data = &lis3dh_info,
	},
#endif
#if defined (CONFIG_COMPASS_AK8975)
	{
		.type          = "ak8975",
		#if defined(CONFIG_MALATA_D8006)
		.addr          = 0x0c,
		#else
		.addr          = 0x0d,
		#endif//#if defined(CONFIG_MALATA_D8005)
		.flags         = 0,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8975_info,
	},
#endif
#if defined (CONFIG_COMPASS_AK8963)
	{
		.type          = "ak8963",
		.addr          = 0x0d,
		.flags         = 0,
		.irq           = RK30_PIN3_PD7,
		.platform_data = &akm8963_info,
	},
#endif

#ifdef CONFIG_MPU_SENSORS_MPU6050B1
{
	.type		   = "mpu6050",
	.addr		   = 0x68,
	.flags		   = 0,
	.irq			= MPU6050_INT_PIN,
	.platform_data = &mpu6050_data,
},
#if defined(CONFIG_MALATA_D1004) || defined(CONFIG_MALATA_D7803)|| defined(CONFIG_MALATA_D8006)||defined(CONFIG_MALATA_D7014)
	{
		.type		   = "ak8963",
		#if defined(CONFIG_MALATA_D8006) || defined(CONFIG_MALATA_D1012)||defined(CONFIG_MALATA_D1014)
		.addr		   = 0x0c,
		#else
		.addr		   = 0x0d,
		#endif
		.flags		   = 0,
		.irq		   = 0,
		.platform_data = &mpu_compass_data,
	},
#else
{
	.type          = "ak8975",
	.addr          = 0x0d,
	.flags         = 0,
	.irq           = 0,
	.platform_data = &mpu_compass_data,
},
#endif
#endif
#if defined (CONFIG_GYRO_L3G4200D)
	{
		.type          = "l3g4200d_gryo",
		.addr          = 0x69,
		.flags         = 0,
		.irq           = L3G4200D_INT_PIN,
		.platform_data = &l3g4200d_info,
	},
#endif
#if defined (CONFIG_SND_SOC_RK1000)
	{
		.type          = "rk1000_i2c_codec",
		.addr          = 0x60,
		.flags         = 0,
	},
	{
		.type          = "rk1000_control",
		.addr          = 0x40,
		.flags         = 0,
	},
#endif
#if defined (CONFIG_SND_SOC_RT5631)
        {
                .type                   = "rt5631",
                .addr                   = 0x1a,
                .flags                  = 0,
        },
#endif
#if defined (CONFIG_CHARGER_CW2015)
        {
                .type                   = "cw2015",
                .addr                   = 0x62,
                .flags                  = 0,
                .platform_data = &cw2015_info,
        },
#endif
#ifdef CONFIG_ENCRYPTION_DM2016_MALATA
	{
		.type		   = "dm2016_encry",
		.addr		   = 0x50,
		.flags		   = 0,
	},
#endif

#ifdef CONFIG_KIONIX_KMX61G_SENSOR
	{
		.type             = "kmx61",
		.addr              = KMX61_I2C_ADDR,
		.flags             = 0,
		//.irq             = KMX61G_INT_PIN,
		.platform_data = &kmx61_board_info,
		},
#endif

};
#endif

int __sramdata g_pmic_type =  0;
#ifdef CONFIG_I2C1_RK30

#ifdef CONFIG_MFD_WM831X_I2C
#define PMU_POWER_SLEEP 		RK30_PIN0_PA1 

static struct pmu_info  wm8326_dcdc_info[] = {
	{
		.name          = "vdd_core",   //logic
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  950000,
	},
	{
		.name          = "vdd_cpu",    //arm
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  950000,
	},
	{
		.name          = "dcdc3",   //ddr
		.min_uv          = 1150000,
		.max_uv         = 1150000,
		.suspend_vol  =  1150000,
	},
	#ifdef CONFIG_MACH_RK3066_SDK
	{
		.name          = "dcdc4",   //vcc_io
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3000000,
	},
	#else
	{
		.name          = "dcdc4",   //vcc_io
		.min_uv          = 3000000,
		.max_uv         = 3000000,
		.suspend_vol  =  2800000,
	},
	#endif
};

static struct pmu_info  wm8326_ldo_info[] = {
	{
		.name          = "ldo1",   //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
	{
		.name          = "ldo2",    //vccio_wl
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
	{
		.name          = "ldo3",   //
		.min_uv          = 1100000,
		.max_uv         = 1100000,
		.suspend_vol  =  1100000,
	},
	{
		.name          = "ldo4",   //vdd11
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  1000000,
	},
	{
		.name          = "ldo5",   //vcc25
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
	{
		.name          = "ldo6",   //vcc33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3300000,
	},
	{
		.name          = "ldo7",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
		.suspend_vol  =  2800000,
	},
	{
		.name          = "ldo8",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3300000,
	},
	{
		.name          = "ldo9",   //vcc_tp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3300000,
	},
	{
		.name          = "ldo10",   //flash_io
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
};

#include "board-pmu-wm8326.c"
#endif

#ifdef CONFIG_MFD_TPS65910

#define TPS65910_HOST_IRQ        RK30_PIN0_PB3

#define PMU_POWER_SLEEP RK30_PIN0_PA1

static struct pmu_info  tps65910_dcdc_info[] = {
	{
		.name          = "vdd_core",   //logic
		.min_uv          = 1100000,
		.max_uv         = 1100000,
	},
	{
		.name          = "vdd2",    //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "vio",   //vcc_io
		.min_uv          = 2500000,
		.max_uv         = 2500000,
	},
	
};
static  struct pmu_info  tps65910_ldo_info[] = {
	{
		.name          = "vpll",   //vdd10
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "vdig1",    //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "vdig2",   //vdd_jetta
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "vaux1",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "vaux2",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vaux33",   //vcc_tp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vmmc",   //vcc30
		.min_uv          = 3000000,
		.max_uv         = 3000000,
	},
	{
		.name          = "vdac",   //vcc18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
 };

#include "board-pmu-tps65910.c"
#endif

#ifdef CONFIG_REGULATOR_ACT8846
#define PMU_POWER_SLEEP RK30_PIN0_PA1
#define ACT8846_HOST_IRQ                RK30_PIN0_PB3
#define PMU_VSEL RK30_PIN3_PD3

static struct pmu_info  act8846_dcdc_info[] = {
	{
		.name          = "act_dcdc1",   //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		.suspend_vol  =  1200000,
	},
	{
		.name          = "vdd_core",    //logic
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  1200000,
		#else
		.suspend_vol  =  900000,
		#endif
	},
	{
		.name          = "vdd_cpu",   //arm
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  1200000,
		#else
		.suspend_vol  =  900000,
		#endif
	},
	{
		.name          = "act_dcdc4",   //vccio
		#ifdef CONFIG_MALATA_D7022
		.min_uv          = 3150000,
		.max_uv         = 3150000,
		#else
		.min_uv          = 3000000,
		.max_uv         = 3000000,
		#endif
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  3000000,
		#else
		.suspend_vol  =  2800000,
		#endif
	},
	
};
static  struct pmu_info  act8846_ldo_info[] = {
	{
		.name          = "act_ldo1",   //vdd11
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "act_ldo2",    //vdd12
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "act_ldo3",   //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo4",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "act_ldo5",   //vcctp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "act_ldo6",   //vccio_wl
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "act_ldo7",   //vcc_18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo8",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
 };

#include "board-pmu-act8846.c"
#endif

static struct i2c_board_info __initdata i2c1_info[] = {
#if defined (CONFIG_MFD_WM831X_I2C)
	{
		.type          = "wm8326",
		.addr          = 0x34,
		.flags         = 0,
		.irq           = RK30_PIN0_PB3,
		.platform_data = &wm831x_platdata,
	},
#endif
#if defined (CONFIG_MFD_TPS65910)
	{
        .type           = "tps65910",
        .addr           = TPS65910_I2C_ID0,
        .flags          = 0,
        .irq            = TPS65910_HOST_IRQ,
    	.platform_data = &tps65910_data,
	},
#endif
#if defined (CONFIG_REGULATOR_ACT8846)
	{
		.type    		= "act8846",
		.addr           = 0x5a, 
		.flags			= 0,
	//	.irq            = ACT8846_HOST_IRQ,
		.platform_data=&act8846_data,
	},
#endif
#if defined (CONFIG_RTC_HYM8563)
    {    
        .type           = "rtc_hym8563",
        .addr           = 0x51,
        .flags          = 0, 
        .irq            = RK30_PIN0_PB5,
    },   
#endif

#if defined (CONFIG_BATTERY_BQ24196)
	{
		.type    		= "bq24196",
		.addr           = 0x6b,
		.flags			= 0,
		.platform_data  = &bq24196_info,
	},
#endif
#if defined (CONFIG_BATTERY_BQ27541)
	{
		.type		= "bq27541",
		.addr	= 0x55,
		.flags	= 0,
		.irq		= 0,
		.platform_data=&bq27541_data,
	},
#endif
#if defined (CONFIG_BATTERY_BQ27410)
	{
		.type		= "bq27410",
		.addr	= 0x55,
		.flags	= 0,
		.irq		= 0,
		.platform_data=&bq27410_data,
	},
#endif
#if defined (CONFIG_CW2015_BATTERY)
    {
        .type           = "cw201x",
        .addr           = 0x62,
        .flags          = 0,
        .platform_data  = &cw_bat_platdata,
    },
#endif

//yemk add tmp108
#if defined (CONFIG_TMP108)
	{
		.type          = "tmp108",
		.addr          = 0x48,
		.flags         = 0,
		.irq           = RK30_PIN1_PA7,
		.platform_data = &tmp108_info,
	},
#endif
//yemk add end


};
#endif

void __sramfunc board_pmu_suspend(void)
{      
#if defined (CONFIG_MFD_WM831X_I2C)
	if(pmic_is_wm8326())
	board_pmu_wm8326_suspend();
#endif
#if defined (CONFIG_MFD_TPS65910)
	if(pmic_is_tps65910())
	board_pmu_tps65910_suspend(); 
#endif   
#if defined (CONFIG_REGULATOR_ACT8846)
		if(pmic_is_act8846())
		board_pmu_act8846_suspend(); 
#endif   
}

void __sramfunc board_pmu_resume(void)
{    
#if defined (CONFIG_MFD_WM831X_I2C)
	if(pmic_is_wm8326())
	board_pmu_wm8326_resume();
#endif
#if defined (CONFIG_MFD_TPS65910)
	if(pmic_is_tps65910())
	board_pmu_tps65910_resume(); 
#endif
#if defined (CONFIG_REGULATOR_ACT8846)
	if(pmic_is_act8846())
	board_pmu_act8846_resume(); 
#endif   

}

 int __sramdata gpio3d6_iomux,gpio3d6_do,gpio3d6_dir,gpio3d6_en;

#define grf_readl(offset)	readl_relaxed(RK30_GRF_BASE + offset)
#define grf_writel(v, offset)	do { writel_relaxed(v, RK30_GRF_BASE + offset); dsb(); } while (0)
 
void __sramfunc rk30_pwm_logic_suspend_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR

//	int gpio0d7_iomux,gpio0d7_do,gpio0d7_dir,gpio0d7_en;
	sram_udelay(10000);
	gpio3d6_iomux = grf_readl(GRF_GPIO3D_IOMUX);
	gpio3d6_do = grf_readl(GRF_GPIO3H_DO);
	gpio3d6_dir = grf_readl(GRF_GPIO3H_DIR);
	gpio3d6_en = grf_readl(GRF_GPIO3H_EN);

	grf_writel((1<<28), GRF_GPIO3D_IOMUX);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_DIR);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_DO);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_EN);
#endif 
}
void __sramfunc rk30_pwm_logic_resume_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	grf_writel((1<<28)|gpio3d6_iomux, GRF_GPIO3D_IOMUX);
	grf_writel((1<<30)|gpio3d6_en, GRF_GPIO3H_EN);
	grf_writel((1<<30)|gpio3d6_dir, GRF_GPIO3H_DIR);
	grf_writel((1<<30)|gpio3d6_do, GRF_GPIO3H_DO);
	sram_udelay(10000);

#endif

}
extern void pwm_suspend_voltage(void);
extern void pwm_resume_voltage(void);
void  rk30_pwm_suspend_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_suspend_voltage();
#endif
}
void  rk30_pwm_resume_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_resume_voltage();
#endif
}

#ifdef CONFIG_HAPTICS_DRV2605
#define GPIO_VIBTONE_EN1 RK30_PIN0_PC7
static struct drv2605_platform_data  drv2605_plat_data = {	
	.GpioEnable = GPIO_VIBTONE_EN1,	
	.GpioTrigger = 0,	
	.g_effect_bank = LIBRARY_A,
};
#endif
#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {
#if defined (CONFIG_TOUCHSCREEN_GT8XX)
	{
		.type          = "Goodix-TS",
		.addr          = 0x55,
		.flags         = 0,
		.irq           = RK30_PIN1_PB7,
		.platform_data = &goodix_info,
	},
#endif
#if defined (CONFIG_LS_CM3217)
	{
		.type          = "lightsensor",
		.addr          = 0x10,
		.flags         = 0,
		.platform_data = &cm3217_info,
	},
#endif

#if defined (CONFIG_DP_ANX6345)
	{
		.type          = "anx6345",
		.addr          = 0x39,
		.flags         = 0,
		.platform_data = &anx6345_platform_data,
	},
#endif
#ifdef CONFIG_HAPTICS_DRV2605	
	{	
		I2C_BOARD_INFO(HAPTICS_DEVICE_NAME, 0x5a),		
		.platform_data = &drv2605_plat_data,				
	},
#endif


#if defined (CONFIG_LS_ISL29023)
	{
		.type          = "ls_isl29023",
		.addr          = 0x44,
		.flags         = 0,
		.platform_data = &lsl29023_info,
	},
#endif
#if defined (CONFIG_LS_ISL5151)
	{
		.type          = "ls_lsl5151",
		.addr          = 0x10,
		.flags         = 0,
		.platform_data = &isl5151_info,
	},
#endif

#if defined (CONFIG_TOUCHSCREEN_GT9110_MALATA)
	{
		.type          = "Goodix9110-TS",
		.addr          = 0x14,
		.flags         = 0,
		.irq           = RK30_PIN1_PB7,
		.platform_data = &goodix9110_info,
	},
#endif
#if defined (CONFIG_TOUCHSCREEN_WALTOP_EM)
	{
		.type          = "waltop_i2c_em",
		.addr          = 0x37,
		.flags         = 0,
		.irq           = RK30_PIN0_PD4,
		.platform_data = &waltop_em_info,
	},
#endif
#if defined (CONFIG_CT36X_TS)
	{
		.type	       = CT36X_NAME,
		.addr          = 0x01,
		.flags         = 0,
		.platform_data = &ct36x_info,
	},
#endif
#if  defined(CONFIG_TOUCHSCREEN_FT5606_MALATA)
	{
		.type          = "ft5x0x_ts",
		.addr          = 0x3E,
		.flags         = 0,
		.irq           = RK30_PIN1_PB7,
		.platform_data = &ft5x0x_touch_info,
	},
#endif
#if defined(CONFIG_HDMI_CAT66121)
	{
		.type		= "cat66121_hdmi",
		.addr		= 0x4c,
		.flags		= 0,
		.irq		= RK30_PIN2_PD6,
		.platform_data 	= &rk_hdmi_pdata,
	},
#endif

};
#endif

#ifdef CONFIG_I2C3_RK30
static struct i2c_board_info __initdata i2c3_info[] = {
};
#endif

#ifdef CONFIG_I2C4_RK30
static struct i2c_board_info __initdata i2c4_info[] = {
#ifdef CONFIG_MFD_RK610
		{
			.type			= "rk610_ctl",
			.addr			= 0x40,
			.flags			= 0,
			.platform_data		= &rk610_ctl_pdata,
		},
#ifdef CONFIG_RK610_TVOUT
		{
			.type			= "rk610_tvout",
			.addr			= 0x42,
			.flags			= 0,
		},
#endif
#ifdef CONFIG_HDMI_RK610
		{
			.type			= "rk610_hdmi",
			.addr			= 0x46,
			.flags			= 0,
			.irq			= INVALID_GPIO,
		},
#endif
#ifdef CONFIG_SND_SOC_RK610
		{//RK610_CODEC addr  from 0x60 to 0x80 (0x60~0x80)
			.type			= "rk610_i2c_codec",
			.addr			= 0x60,
			.flags			= 0,
			.platform_data          = &rk610_codec_pdata,
		},
#endif
#endif
#ifdef CONFIG_SND_SOC_RT5631
        {
                .type                   = "rt5631",
                .addr                   = 0x1a,
                .flags                  = 0,
        },
#endif
#ifdef CONFIG_SND_SOC_RT5616
        {
                .type                   = "rt5616",
                .addr                   = 0x1b,
                .flags                  = 0,
                .platform_data          = &rt5616_codec_pdata,
        },
#endif
#ifdef CONFIG_QLVX5A3B_RGB2MIPI
		{
				.type                   = "quickvx",
				.addr                   = 0x64,
				.flags                  = 0,
				.platform_data          = &qlvx5a3b_platdata,
		},
#endif
};
#endif

#ifdef CONFIG_I2C_GPIO_RK30
#define I2C_SDA_PIN     INVALID_GPIO// RK30_PIN2_PD6   //set sda_pin here
#define I2C_SCL_PIN     INVALID_GPIO//RK30_PIN2_PD7   //set scl_pin here
static int rk30_i2c_io_init(void)
{
        //set iomux (gpio) here
        //rk30_mux_api_set(GPIO2D7_I2C1SCL_NAME, GPIO2D_GPIO2D7);
        //rk30_mux_api_set(GPIO2D6_I2C1SDA_NAME, GPIO2D_GPIO2D6);

        return 0;
}
struct i2c_gpio_platform_data default_i2c_gpio_data = {
       .sda_pin = I2C_SDA_PIN,
       .scl_pin = I2C_SCL_PIN,
       .udelay = 5, // clk = 500/udelay = 100Khz
       .timeout = 100,//msecs_to_jiffies(100),
       .bus_num    = 5,
       .io_init = rk30_i2c_io_init,
};
static struct i2c_board_info __initdata i2c_gpio_info[] = {
};
#endif

static void __init rk30_i2c_register_board_info(void)
{
#ifdef CONFIG_I2C0_RK30
	i2c_register_board_info(0, i2c0_info, ARRAY_SIZE(i2c0_info));
#endif
#ifdef CONFIG_I2C1_RK30
	i2c_register_board_info(1, i2c1_info, ARRAY_SIZE(i2c1_info));
#endif
#ifdef CONFIG_I2C2_RK30
	i2c_register_board_info(2, i2c2_info, ARRAY_SIZE(i2c2_info));
#endif
#ifdef CONFIG_I2C3_RK30
	i2c_register_board_info(3, i2c3_info, ARRAY_SIZE(i2c3_info));
#endif
#ifdef CONFIG_I2C4_RK30
	i2c_register_board_info(4, i2c4_info, ARRAY_SIZE(i2c4_info));
#endif
#ifdef CONFIG_I2C_GPIO_RK30
	i2c_register_board_info(5, i2c_gpio_info, ARRAY_SIZE(i2c_gpio_info));
#endif
}
//end of i2c

// ========== Begin of rk3168 top board keypad defination ============

#include <plat/key.h>

static struct rk29_keys_button key_button[] = {
        {
                .desc   = "vol-",
                .code   = KEY_VOLUMEDOWN,
#ifdef CONFIG_MALATA_D7022
		.adc_value		= 768,
#else
		.adc_value      = 170,
#endif
                .gpio   = INVALID_GPIO,
                .active_low = PRESS_LEV_LOW,
        },
        {
                .desc   = "play",
                .code   = KEY_POWER,
                .gpio   = RK30_PIN0_PA4,
                .active_low = PRESS_LEV_LOW,
                .wakeup = 1,
        },
        {
                .desc   = "vol+",
                .code   = KEY_VOLUMEUP,
                .adc_value      = 1,
                .gpio = INVALID_GPIO,
                .active_low = PRESS_LEV_LOW,
        },
#ifdef CONFIG_HALL_KEY
		{
				.desc	= "hall",
				.code	= KEY_HALL_SLEEP,
				.code1	= KEY_HALL_WAKE,
#if defined(CONFIG_MALATA_D1013)
				.gpio	= RK30_PIN3_PA0,
#else
				.gpio	= RK30_PIN0_PC6,
#endif
				.active_low = PRESS_LEV_LOW,
				.hall_key = 1,
				.wakeup = 1,
		},
#endif

     #if 0
		{
                .desc   = "menu",
                .code   = EV_MENU,
                .adc_value      = 133,
                .gpio = INVALID_GPIO,
                .active_low = PRESS_LEV_LOW,
        },
      #endif
	  #if 0
        {
                .desc   = "home",
                .code   = KEY_HOME,
                .adc_value      = 355,
                .gpio = INVALID_GPIO,
                .active_low = PRESS_LEV_LOW,
        },
       #endif
       #if 0
        {
                .desc   = "esc",
                .code   = KEY_BACK,
                .adc_value      = 333,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	
	{
		.desc	= "camera",
		.code	= KEY_CAMERA,
		.adc_value	= 742,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	#endif
};
struct rk29_keys_platform_data rk29_keys_pdata = {
	.buttons	= key_button,
	.nbuttons	= ARRAY_SIZE(key_button),
	.chn	= 1,  //chn: 0-7, if do not use ADC,set 'chn' -1
};

// =========== End of rk3168 top board keypad defination  =============


#define POWER_ON_PIN RK30_PIN0_PA0   //power_hold
static void rk30_pm_power_off(void)
{
	printk(KERN_ERR "rk30_pm_power_off start...\n");
	#if defined(CONFIG_MFD_WM831X)
	if(pmic_is_wm8326()){
		wm831x_set_bits(Wm831x,WM831X_GPIO_LEVEL,0x0001,0x0000);  //set sys_pwr 0
		wm831x_device_shutdown(Wm831x);//wm8326 shutdown
	 }
	#endif

	#if defined(CONFIG_REGULATOR_ACT8846)
        if(pmic_is_act8846())
        {
               printk("enter dcdet:");
               //Modified by EvanZeng for adding the USB cable's detecttion @2013-0704
               if(gpio_get_value (RK30_PIN0_PB2) == GPIO_LOW || gpio_get_value (RK30_PIN0_PA7) == GPIO_LOW)
               {
                       printk("with dc:enter restart system\n");
                       arm_pm_restart(0, NULL);
               }
			   else
				{
					printk("without dc,shutdown system\n");
					//act8846_device_shutdown();
					//while(1);
			   }
        }
	#endif
	
	#if defined(CONFIG_MFD_TPS65910)	
	if(pmic_is_tps65910())
	{
		tps65910_device_shutdown();//tps65910 shutdown
	}
	#endif
/*
// Mute PA before system power down to avoid "pop" noise, xmylm, 20130704, MT-BUG:9568
#ifdef CONFIG_SND_SOC_RT5616
	if (rt5616_codec_pdata.spk_ctl_io)
		gpio_direction_output(rt5616_codec_pdata.spk_ctl_io, GPIO_LOW);
	msleep(50);
#endif
*/
	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	while (1);
}

#ifdef CONFIG_POWER_ON_CHARGER_DISPLAY
static int __boot_from_source = -1 ; // 1: power on ; 2 : dc
static int store_boot_source()
{
	int charge_pin;

#ifdef CONFIG_BATTERY_RK30_USB_CHARGE
	if(rk30_adc_battery_platdata.usb_det_pin != INVALID_GPIO)
	{
		charge_pin = rk30_adc_battery_platdata.usb_det_pin;
		gpio_request(charge_pin, "USB_DET");
		gpio_direction_input(charge_pin);

		if(gpio_get_value(charge_pin) == rk30_adc_battery_platdata.usb_det_level)
			__boot_from_source = 2;
		gpio_free(charge_pin);
	}
#endif

#ifdef CONFIG_BATTERY_RK30_AC_CHARGE
	if(rk30_adc_battery_platdata.dc_det_pin != INVALID_GPIO)
	{
		charge_pin = rk30_adc_battery_platdata.dc_det_pin;
		gpio_request(charge_pin, "USB_DET");
		gpio_direction_input(charge_pin);

		if(gpio_get_value(charge_pin) == rk30_adc_battery_platdata.dc_det_level)
			__boot_from_source = 2;
		gpio_free(charge_pin);
	}
#endif

#ifdef CONFIG_BATTERY_USB_CHARGE
	if(cw_bat_platdata.usb_det_pin != INVALID_GPIO)
	{
		charge_pin = cw_bat_platdata.usb_det_pin;
		gpio_request(charge_pin, "USB_DET");
		gpio_direction_input(charge_pin);

		if(gpio_get_value(charge_pin) == cw_bat_platdata.usb_det_level)
			__boot_from_source = 2;
		gpio_free(charge_pin);
	}
#endif

#ifdef CONFIG_BATTERY_AC_CHARGE
	if(cw_bat_platdata.dc_det_pin != INVALID_GPIO)
	{
		charge_pin = cw_bat_platdata.dc_det_pin;
		gpio_request(charge_pin, "DC_DET");
		gpio_direction_input(charge_pin);

		if(gpio_get_value(charge_pin) == cw_bat_platdata.dc_det_level)
			__boot_from_source = 2;
		gpio_free(charge_pin);
	}
#endif

	if(__boot_from_source < 0)
		__boot_from_source = 1;

	return __boot_from_source;
}

int get_boot_source()
{
	return __boot_from_source ;
}
#endif

static void __init machine_rk30_board_init(void)
{
	//avs_init();
#ifdef CONFIG_POWER_ON_CHARGER_DISPLAY
	store_boot_source();
#endif

	gpio_request(POWER_ON_PIN, "poweronpin");
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
	
	pm_power_off = rk30_pm_power_off;
	
        gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
#if defined (CONFIG_BATTERY_BQ24196)
	bq24196_charge_en();
#endif

#ifdef CONFIG_SND_SOC_RT5616
	if (rt5616_codec_pdata.spk_ctl_io)
		rt5616_codec_spk_io = rt5616_codec_pdata.spk_ctl_io;
#endif

	rk30_i2c_register_board_info();
	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	rk_platform_add_display_devices();
	board_usb_detect_init(RK30_PIN0_PA7);

#ifdef CONFIG_WIFI_CONTROL_FUNC
	rk29sdk_wifi_bt_gpio_control_init();
#endif
#if defined(CONFIG_MT5931_MT6622)
	   clk_set_rate(clk_get_sys("rk_serial.0", "uart"), 24*1000000);
#endif

#if defined(CONFIG_BACKLIGHT_RK29_BL) && (defined(CONFIG_MALATA_D1004)||defined(CONFIG_MALATA_D8006)) && !(defined(CONFIG_MALATA_D1012)||defined(CONFIG_MALATA_D1014)||defined(CONFIG_MALATA_D8009)||defined(CONFIG_MALATA_D7022))
	int pwm_gpio;

	pwm_gpio = iomux_mode_to_gpio(PWM_MODE);
	gpio_request(pwm_gpio, "bl_pwm");
	gpio_direction_output(pwm_gpio, GPIO_LOW);
#endif
}

#define HD_SCREEN_SIZE 1920UL*1200UL*4*3
static void __init rk30_reserve(void)
{
#if defined(CONFIG_ARCH_RK3188)
	/*if lcd resolution great than or equal to 1920*1200,reserve the ump memory */
	if(!(get_fb_size() < ALIGN(HD_SCREEN_SIZE,SZ_1M)))
	{
		/*M-MT:reduce gpu memory to 256M,otherwise the system has no memory.*/
		//int ump_mem_phy_size=512UL*1024UL*1024UL;
		int ump_mem_phy_size=256UL*1024UL*1024UL;
		resource_mali[0].start = board_mem_reserve_add("ump buf", ump_mem_phy_size); 
		resource_mali[0].end = resource_mali[0].start + ump_mem_phy_size -1;
	}
#endif
#ifdef CONFIG_ION
	rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ION_RESERVE_SIZE);
#endif
#ifdef CONFIG_FB_ROCKCHIP
	resource_fb[0].start = board_mem_reserve_add("fb0 buf", get_fb_size());
	resource_fb[0].end = resource_fb[0].start + get_fb_size()- 1;
#if 0
	resource_fb[1].start = board_mem_reserve_add("ipp buf", RK30_FB0_MEM_SIZE);
	resource_fb[1].end = resource_fb[1].start + RK30_FB0_MEM_SIZE - 1;
#endif

#if defined(CONFIG_FB_ROTATE) || !defined(CONFIG_THREE_FB_BUFFER)
	resource_fb[2].start = board_mem_reserve_add("fb2 buf",get_fb_size());
	resource_fb[2].end = resource_fb[2].start + get_fb_size() - 1;
#endif
#endif

#ifdef CONFIG_VIDEO_RK29
	rk30_camera_request_reserve_mem();
#endif
	
#ifdef CONFIG_GPS_RK
	//it must be more than 8MB
	rk_gps_info.u32MemoryPhyAddr = board_mem_reserve_add("gps", SZ_8M);
#endif
	board_mem_reserved();
}

/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs 
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 * @logic_volt	: logic voltage arm requests depend on frequency
 * comments	: min arm/logic voltage
 */
 #ifdef CONFIG_DDR_TYPE_LPDDR2
 static struct cpufreq_frequency_table dvfs_arm_table[] = {
        {.frequency = 312 * 1000,       .index = 875 * 1000},
        {.frequency = 504 * 1000,       .index = 925 * 1000},
        {.frequency = 816 * 1000,       .index = 975 * 1000},
        {.frequency = 1008 * 1000,      .index = 1075 * 1000},
        {.frequency = 1200 * 1000,      .index = 1150 * 1000},
        {.frequency = 1416 * 1000,      .index = 1250 * 1000},
        {.frequency = 1608 * 1000,      .index = 1350 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};


static struct cpufreq_frequency_table dvfs_gpu_table[] = {
#if defined(CONFIG_ARCH_RK3188)
        {.frequency = 133 * 1000,       .index = 975 * 1000},//the mininum rate is limited 133M for rk3188
#elif defined(CONFIG_ARCH_RK3066B)
	{.frequency = 100 * 1000, 	.index = 950 * 1000},//the minimum rate is no limit for rk3168 rk3066B
#endif

	{.frequency = 200 * 1000,       .index = 1000 * 1000},
	{.frequency = 266 * 1000,       .index = 1025 * 1000},
	{.frequency = 300 * 1000,       .index = 1050 * 1000},
	{.frequency = 400 * 1000,       .index = 1100 * 1000},
	{.frequency = 600 * 1000,       .index = 1250 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
#if defined(CONFIG_ARCH_RK3188)
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 950 * 1000},
	{.frequency = 300 * 1000 + DDR_FREQ_VIDEO,      .index = 1000 * 1000},
#endif
	{.frequency = 396 * 1000 + DDR_FREQ_NORMAL,     .index = 1100 * 1000},
	//{.frequency = 528 * 1000 + DDR_FREQ_NORMAL,     .index = 1200 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
#else
static struct cpufreq_frequency_table dvfs_arm_table[] = {
		{.frequency = 312 * 1000,       .index = 925 * 1000},
		{.frequency = 504 * 1000,       .index = 950 * 1000},
#if defined(CONFIG_MALATA_D7806)
		{.frequency = 816 * 1000,       .index = 1025 * 1000},
#else
		{.frequency = 816 * 1000,       .index = 1000 * 1000},
#endif
		{.frequency = 1008 * 1000,      .index = 1075 * 1000},
		{.frequency = 1200 * 1000,      .index = 1200 * 1000},
#if defined(CONFIG_MALATA_D1013)
		{.frequency = 1416 * 1000,      .index = 1300 * 1000},
#else
		{.frequency = 1416 * 1000,      .index = 1250 * 1000},
#endif
        {.frequency = 1608 * 1000,      .index = 1350 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};

#ifdef CONFIG_GPU_FREQ_LIMITED_BY_TEMP
struct cpufreq_frequency_table dvfs_arm_table_1000[] = {
		{.frequency = 312 * 1000,       .index = 925 * 1000},
		{.frequency = 504 * 1000,       .index = 950 * 1000},
#if defined(CONFIG_MALATA_D7806)
		{.frequency = 816 * 1000,       .index = 1025 * 1000},
#else
		{.frequency = 816 * 1000,       .index = 1000 * 1000},
#endif
		{.frequency = 1008 * 1000,      .index = 1075 * 1000},
		{.frequency = CPUFREQ_TABLE_END},
};

struct cpufreq_frequency_table dvfs_arm_table_1600[] = {
		{.frequency = 312 * 1000,       .index = 925 * 1000},
		{.frequency = 504 * 1000,       .index = 950 * 1000},
#if defined(CONFIG_MALATA_D7806)
		{.frequency = 816 * 1000,       .index = 1025 * 1000},
#else
		{.frequency = 816 * 1000,       .index = 1000 * 1000},
#endif
		{.frequency = 1008 * 1000,      .index = 1075 * 1000},
		{.frequency = 1200 * 1000,      .index = 1200 * 1000},
#if defined(CONFIG_MALATA_D1013)
		{.frequency = 1416 * 1000,      .index = 1300 * 1000},
#else
		{.frequency = 1416 * 1000,      .index = 1250 * 1000},
#endif
        {.frequency = 1608 * 1000,      .index = 1350 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};
#endif

static struct cpufreq_frequency_table dvfs_gpu_table[] = {	
#if defined(CONFIG_ARCH_RK3188)
        {.frequency = 133 * 1000,       .index = 975 * 1000},//the mininum rate is limited 133M for rk3188
#elif defined(CONFIG_ARCH_RK3066B)
	{.frequency = 100 * 1000, 	.index = 950 * 1000},//the minimum rate is no limit for rk3168 rk3066B
#endif

	{.frequency = 200 * 1000,       .index = 975 * 1000},
	{.frequency = 266 * 1000,       .index = 1000 * 1000},
	{.frequency = 300 * 1000,       .index = 1050 * 1000},
	{.frequency = 400 * 1000,       .index = 1100 * 1000},
	{.frequency = 600 * 1000,       .index = 1200 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
#ifdef CONFIG_DDR_MIRA_P3P4GF4BLF
	{.frequency = 250 * 1000 + DDR_FREQ_SUSPEND,    .index = 950 * 1000},
#else
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 950 * 1000},
#endif
	//{.frequency = 300 * 1000 + DDR_FREQ_VIDEO,      .index = 1000 * 1000},
#ifdef CONFIG_DDR_SUPPORT_504M
	{.frequency = 504 * 1000 + DDR_FREQ_NORMAL,     .index = 1200 * 1000},
#elif CONFIG_DDR_SUPPORT_480M
	{.frequency = 480 * 1000 + DDR_FREQ_NORMAL, 	.index = 1200 * 1000},
#elif CONFIG_DDR_SUPPORT_410M
	{.frequency = 410 * 1000 + DDR_FREQ_NORMAL,     .index = 1200 * 1000},
#elif CONFIG_DDR_SUPPORT_336M
	{.frequency = 336 * 1000 + DDR_FREQ_NORMAL, 	.index = 1200 * 1000},
#endif
	{.frequency = CPUFREQ_TABLE_END},
};
#endif
//#define DVFS_CPU_TABLE_SIZE	(ARRAY_SIZE(dvfs_cpu_logic_table))
//static struct cpufreq_frequency_table cpu_dvfs_table[DVFS_CPU_TABLE_SIZE];
//static struct cpufreq_frequency_table dep_cpu2core_table[DVFS_CPU_TABLE_SIZE];

void __init board_clock_init(void)
{
	rk30_clock_data_init(periph_pll_default, codec_pll_default, RK30_CLOCKS_DEFAULT_FLAGS);
	//dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);	
	dvfs_set_freq_volt_table(clk_get(NULL, "cpu"), dvfs_arm_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
}

MACHINE_START(RK30, "RK30board")
	.boot_params	= PLAT_PHYS_OFFSET + 0x800,
	.fixup		= rk30_fixup,
	.reserve	= &rk30_reserve,
	.map_io		= rk30_map_io,
	.init_irq	= rk30_init_irq,
	.timer		= &rk30_timer,
	.init_machine	= machine_rk30_board_init,
MACHINE_END
