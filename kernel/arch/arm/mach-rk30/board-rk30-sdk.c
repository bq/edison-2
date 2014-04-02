/* arch/arm/mach-rk30/board-rk30-sdk.c
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
#include <linux/regulator/machine.h>
#include <linux/rfkill-rk.h>
#include <linux/sensor-dev.h>
#include <linux/bq24196_chargeIc.h>

#if defined (CONFIG_BATTERY_BQ27541)
#include <linux/power/bq27541_battery.h>
#endif
#if defined (CONFIG_BATTERY_BQ27425)
#include <linux/power/bq27425_battery.h>
#endif
#if defined (CONFIG_BATTERY_BQ27410)
#include <linux/power/bq27410_battery.h>
#endif
#include <linux/regulator/rk29-pwm-regulator.h>

#if defined(CONFIG_MFD_RK610)
#include <linux/mfd/rk610_core.h>
#endif

#if defined(CONFIG_RK_HDMI)
	#include "../../../drivers/video/rockchip/hdmi/rk_hdmi.h"
#endif

#if defined(CONFIG_SPIM_RK29)
#include "../../../drivers/spi/rk29_spim.h"
#endif
#if defined(CONFIG_MU509)
#include <linux/mu509.h>
#endif
#if defined(CONFIG_MW100)
#include <linux/mw100.h>
#endif
#if defined(CONFIG_MT6229)
#include <linux/mt6229.h>
#endif
#if defined(CONFIG_SEW868)
#include <linux/sew868.h>
#endif
#if defined(CONFIG_ANDROID_TIMED_GPIO)
#include "../../../drivers/staging/android/timed_gpio.h"
#endif
#ifdef  CONFIG_TOUCHSCREEN_GT811_MALATA
#include <linux/gt811.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_GT8110_MALATA
#include <linux/gt8110.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_GT9110_MALATA
#include <linux/gt9xx.h>
#endif
#if defined(CONFIG_MT6620)
#include <linux/gps.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_FT5606_MALATA
#include <linux/i2c/ft5x06_ts2.h>
#endif

#if defined(CONFIG_DP501)   //for display port transmitter dp501
#include<linux/dp501.h>
#endif
#ifdef CONFIG_MPU_SENSORS_MPU6050B1
#include <linux/mpu.h>
#endif

#include "board-rk30-sdk-camera.c"

#include <plat/key.h>
static struct rk29_keys_button key_button[] = {
	{
		.desc	= "vol-",
		.code	= KEY_VOLUMEDOWN,
		.gpio	= RK30_PIN4_PC5,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "play",
		.code	= KEY_POWER,
		.gpio	= RK30_PIN6_PA2,
		.active_low = PRESS_LEV_LOW,
		//.code_long_press = EV_ENCALL,
        .hall_key = 0,
		.wakeup	= 1,
	},
#ifdef CONFIG_HALL_KEY
	{
		.desc	= "hall",
		.code	= KEY_POWER,
		.gpio	= RK30_PIN4_PD1,
		.active_low = PRESS_LEV_LOW,
		.hall_key = 1,
		.wakeup = 1,
	},
#endif
	{
		.desc	= "vol+",
		.code	= KEY_VOLUMEUP,
		.adc_value	= 1,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
#if 0
#ifndef RK3000_SDK
	{
		.desc	= "menu",
		.code	= EV_MENU,
		.adc_value	= 135,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "home",
		.code	= KEY_HOME,
		.adc_value	= 550,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "esc",
		.code	= KEY_BACK,
		.adc_value	= 334,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "camera",
		.code	= KEY_CAMERA,
		.adc_value	= 743,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
#else
	{
		.desc	= "menu",
		.code	= EV_MENU,
		.adc_value	= 155,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "home",
		.code	= KEY_HOME,
		.adc_value	= 630,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "esc",
		.code	= KEY_BACK,
		.adc_value	= 386,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "camera",
		.code	= KEY_CAMERA,
		.adc_value	= 827,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
#endif
#endif
};

struct rk29_keys_platform_data rk29_keys_pdata = {
	.buttons	= key_button,
	.nbuttons	= ARRAY_SIZE(key_button),
	.chn	= 1,  //chn: 0-7, if do not use ADC,set 'chn' -1
};

#if defined(CONFIG_TOUCHSCREEN_GT8XX)
#define TOUCH_RESET_PIN  RK30_PIN4_PD0
#define TOUCH_PWR_PIN    INVALID_GPIO
int goodix_init_platform_hw(void)
{
	int ret;
	
	rk30_mux_api_set(GPIO4D0_SMCDATA8_TRACEDATA8_NAME, GPIO4D_GPIO4D0);
	rk30_mux_api_set(GPIO4C2_SMCDATA2_TRACEDATA2_NAME, GPIO4C_GPIO4C2);
	printk("%s:0x%x,0x%x\n",__func__,rk30_mux_api_get(GPIO4D0_SMCDATA8_TRACEDATA8_NAME),rk30_mux_api_get(GPIO4C2_SMCDATA2_TRACEDATA2_NAME));

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
	.irq_pin = RK30_PIN4_PC2,
	.rest_pin = TOUCH_RESET_PIN,
	.init_platform_hw = goodix_init_platform_hw,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_GT811_MALATA
#define TOUCH_ENABLE_PIN	INVALID_GPIO
#define TOUCH_INT_PIN		RK30_PIN4_PC2
#define TOUCH_RESET_PIN		RK30_PIN4_PD0
int goodix811_init_platform_hw(void)
{
	struct regulator *ldo;
	int ret;
	rk30_mux_api_set(GPIO4D0_SMCDATA8_TRACEDATA8_NAME, GPIO4D_GPIO4D0);
	rk30_mux_api_set(GPIO4C2_SMCDATA2_TRACEDATA2_NAME, GPIO4C_GPIO4C2);
	pr_debug("%s:0x%x,0x%x\n",__func__,rk30_mux_api_get(GPIO4D0_SMCDATA8_TRACEDATA8_NAME),rk30_mux_api_get(GPIO4C2_SMCDATA2_TRACEDATA2_NAME));

	if(g_pmic_type == PMIC_TYPE_TPS65910)
		ldo = regulator_get(NULL, "vaux33");
	else if(g_pmic_type == PMIC_TYPE_WM8326)
		ldo = regulator_get(NULL, "ldo9");
	regulator_set_voltage(ldo, 3300000, 3300000);
	regulator_enable(ldo);
	pr_debug("%s set vaux33 vcc_tp=%dmV end\n", __func__, regulator_get_voltage(ldo));
	regulator_put(ldo);
	msleep(100);

	if (TOUCH_ENABLE_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_ENABLE_PIN, "goodix power pin");
		if (ret != 0) {
			gpio_free(TOUCH_ENABLE_PIN);
			printk("goodix power error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_ENABLE_PIN, 0);
		gpio_set_value(TOUCH_ENABLE_PIN, GPIO_LOW);
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
		gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
		msleep(100);
		gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
		msleep(500);
	}
	return 0;
}

struct goodix_811_platform_data  goodix_info = {
	//.model = 8105,
	//.irq_pin = RK30_PIN4_PC2,
	.reset= TOUCH_RESET_PIN,
	.init_platform_hw = goodix811_init_platform_hw,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_GT8110_MALATA) || defined(CONFIG_TOUCHSCREEN_GT811_MALATA)
#define TOUCH_ENABLE_PIN	INVALID_GPIO
#define TOUCH_INT_PIN		RK30_PIN4_PC2
#define TOUCH_RESET_PIN		RK30_PIN4_PD0

int goodix_init_platform_hw(void)
{
	int ret;
	rk30_mux_api_set(GPIO4D0_SMCDATA8_TRACEDATA8_NAME, GPIO4D_GPIO4D0);
	rk30_mux_api_set(GPIO4C2_SMCDATA2_TRACEDATA2_NAME, GPIO4C_GPIO4C2);

	if (TOUCH_ENABLE_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_ENABLE_PIN, "goodix power pin");
		if (ret != 0) {
			gpio_free(TOUCH_ENABLE_PIN);
			printk("goodix power error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_ENABLE_PIN, 0);
		gpio_set_value(TOUCH_ENABLE_PIN, GPIO_LOW);
		gpio_free(TOUCH_ENABLE_PIN);
		msleep(100);
	}

	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "goodix reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("goodix gpio_request error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_RESET_PIN, 0);
		gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
		gpio_free(TOUCH_RESET_PIN);
		msleep(100);
	}

	return 0;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_GT8110_MALATA
#define TOUCH_ENABLE_PIN	INVALID_GPIO
#define TOUCH_INT_PIN		RK30_PIN4_PC2
#define TOUCH_RESET_PIN		RK30_PIN4_PD0
int goodix8110_init_platform_hw(void)
{
	struct regulator *ldo;
	int ret;
	rk30_mux_api_set(GPIO4D0_SMCDATA8_TRACEDATA8_NAME, GPIO4D_GPIO4D0);
	rk30_mux_api_set(GPIO4C2_SMCDATA2_TRACEDATA2_NAME, GPIO4C_GPIO4C2);
	pr_debug("%s:0x%x,0x%x\n",__func__,rk30_mux_api_get(GPIO4D0_SMCDATA8_TRACEDATA8_NAME),rk30_mux_api_get(GPIO4C2_SMCDATA2_TRACEDATA2_NAME));

	if(g_pmic_type == PMIC_TYPE_TPS65910)
		ldo = regulator_get(NULL, "vaux33");
	else if(g_pmic_type == PMIC_TYPE_WM8326)
		ldo = regulator_get(NULL, "ldo9");
	regulator_set_voltage(ldo, 3300000, 3300000);
	regulator_enable(ldo);
	pr_debug("%s set vaux33 vcc_tp=%dmV end\n", __func__, regulator_get_voltage(ldo));
	regulator_put(ldo);
	msleep(100);

	if (TOUCH_ENABLE_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_ENABLE_PIN, "goodix power pin");
		if (ret != 0) {
			gpio_free(TOUCH_ENABLE_PIN);
			printk("goodix power error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_ENABLE_PIN, 0);
		gpio_set_value(TOUCH_ENABLE_PIN, GPIO_LOW);
		msleep(100);
	}

	if (TOUCH_RESET_PIN != INVALID_GPIO) {
		ret = gpio_request(TOUCH_RESET_PIN, "goodix reset pin");
		if (ret != 0) {
			gpio_free(TOUCH_RESET_PIN);
			printk("goodix gpio_request error\n");
			return -EIO;
		}
		gpio_direction_output(TOUCH_RESET_PIN, 0);
		//msleep(100);
		gpio_set_value(TOUCH_RESET_PIN, GPIO_LOW);
		msleep(100);
		//gpio_set_value(TOUCH_RESET_PIN, GPIO_HIGH);
		//msleep(500);
	}
	return 0;
}

struct goodix_8110_platform_data  goodix_info = {
	.irq_pin = TOUCH_INT_PIN,
	.reset= TOUCH_RESET_PIN,
	.power_control=RK30_PIN0_PD1,
	.init_platform_hw = goodix8110_init_platform_hw,
};
#endif
#ifdef CONFIG_TOUCHSCREEN_GT9110_MALATA
int goodix9110_init_platform_hw(void)
{
	struct regulator *ldo;
	int ret;
	rk30_mux_api_set(GPIO4D0_SMCDATA8_TRACEDATA8_NAME, GPIO4D_GPIO4D0);
	rk30_mux_api_set(GPIO4C2_SMCDATA2_TRACEDATA2_NAME, GPIO4C_GPIO4C2);
	pr_debug("%s:0x%x,0x%x\n",__func__,rk30_mux_api_get(GPIO4D0_SMCDATA8_TRACEDATA8_NAME),rk30_mux_api_get(GPIO4C2_SMCDATA2_TRACEDATA2_NAME));

	if(g_pmic_type == PMIC_TYPE_TPS65910)
		ldo = regulator_get(NULL, "vaux33");
	else if(g_pmic_type == PMIC_TYPE_WM8326)
		ldo = regulator_get(NULL, "ldo9");
	regulator_set_voltage(ldo, 3300000, 3300000);
	regulator_enable(ldo);
	pr_debug("%s set vaux33 vcc_tp=%dmV end\n", __func__, regulator_get_voltage(ldo));
	regulator_put(ldo);
	msleep(100);

	return 0;
}

struct goodix_9110_platform_data  goodix9110_info = {
	.irq_pin = RK30_PIN4_PC2,
	.reset= RK30_PIN4_PD0,
	.init_platform_hw = goodix9110_init_platform_hw,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5606_MALATA
int ft5x0x_init_platform_hw(void)
{
	struct regulator *ldo;
	int ret;
	rk30_mux_api_set(GPIO4D0_SMCDATA8_TRACEDATA8_NAME, GPIO4D_GPIO4D0);
	rk30_mux_api_set(GPIO4C2_SMCDATA2_TRACEDATA2_NAME, GPIO4C_GPIO4C2);
	pr_debug("%s:0x%x,0x%x\n",__func__,rk30_mux_api_get(GPIO4D0_SMCDATA8_TRACEDATA8_NAME),rk30_mux_api_get(GPIO4C2_SMCDATA2_TRACEDATA2_NAME));

#ifdef CONFIG_MALATA_D9001
	ret = gpio_request(RK30_PIN4_PD0, "NULL");
	if (ret == 0) {
		gpio_direction_output(RK30_PIN4_PD0, 1);
		msleep(5);
	}
#endif

	if(g_pmic_type == PMIC_TYPE_TPS65910)
		ldo = regulator_get(NULL, "vaux33");
	else if(g_pmic_type == PMIC_TYPE_WM8326)
		ldo = regulator_get(NULL, "ldo9");
	regulator_set_voltage(ldo, 3300000, 3300000);
	regulator_enable(ldo);
	pr_debug("%s set vaux33 vcc_tp=%dmV end\n", __func__, regulator_get_voltage(ldo));
	regulator_put(ldo);
	msleep(100);

	return 0;
}
static struct  ft5x0x_ts_platform_data  ft5x0x_touch_info = {
	.intr_number = RK30_PIN4_PC2,
	.reset_pin =  RK30_PIN4_PD0,
	.init_platform_hw = ft5x0x_init_platform_hw,
};

#endif
static struct spi_board_info board_spi_devices[] = {
#if defined(CONFIG_ROCKCHIP_DTV)
	{
		.modalias	= "tstv_control",
		.chip_select	= 0,
		.max_speed_hz	= 1500 * 1000,/* (max sample rate @ 3V) * (cmd + data + overhead) */
		.bus_num	= 0,
		.irq = RK30_PIN0_PA2,
		.mode = SPI_MODE_3,
	},
#endif
};

/***********************************************************
*	rk30  backlight
************************************************************/
#ifdef CONFIG_BACKLIGHT_RK29_BL
#define PWM_ID            2
#define PWM_MUX_NAME      GPIO0D6_PWM2_NAME
#define PWM_MUX_MODE      GPIO0D_PWM2
#define PWM_MUX_MODE_GPIO GPIO0D_GPIO0D6
#define PWM_GPIO 	  RK30_PIN0_PD6
#if defined(CONFIG_MALATA_C1016)||defined(CONFIG_MALATA_D7008)||defined(CONFIG_MALATA_C7022)
#define PWM_EFFECT_VALUE  1
#else
#define PWM_EFFECT_VALUE  0
#endif
#define LCD_DISP_ON_PIN

#ifdef  LCD_DISP_ON_PIN
//#define BL_EN_MUX_NAME    GPIOF34_UART3_SEL_NAME
//#define BL_EN_MUX_MODE    IOMUXB_GPIO1_B34

#define BL_EN_PIN         RK30_PIN6_PB3
#define BL_EN_VALUE       GPIO_HIGH
#endif
static int rk29_backlight_io_init(void)
{
	int ret = 0;
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE);
#ifdef  LCD_DISP_ON_PIN
	// rk30_mux_api_set(BL_EN_MUX_NAME, BL_EN_MUX_MODE);

	ret = gpio_request(BL_EN_PIN, NULL);
	if (ret != 0) {
		gpio_free(BL_EN_PIN);
	}

	gpio_direction_output(BL_EN_PIN, 0);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif
	return ret;
}

static int rk29_backlight_io_deinit(void)
{
	int ret = 0;
#ifdef  LCD_DISP_ON_PIN
	gpio_direction_output(BL_EN_PIN, 0);
	gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
	gpio_free(BL_EN_PIN);
#endif

	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	if (gpio_request(PWM_GPIO, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return -1;
	}
	gpio_direction_output(PWM_GPIO, GPIO_LOW);
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	return ret;
}

static int rk29_backlight_pwm_suspend(void)
{
	int ret = 0;
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	if (gpio_request(PWM_GPIO, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return -1;
	}
	gpio_direction_output(PWM_GPIO, GPIO_LOW);
#ifdef  LCD_DISP_ON_PIN
	gpio_direction_output(BL_EN_PIN, 0);
	gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
#endif
	return ret;
}

static int rk29_backlight_pwm_resume(void)
{
	gpio_free(PWM_GPIO);
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE);
#ifdef  LCD_DISP_ON_PIN
	msleep(150);
	gpio_direction_output(BL_EN_PIN, 1);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif
	return 0;
}

static struct rk29_bl_info rk29_bl_info = {
	.pwm_id = PWM_ID,
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

#define RK30_MODEM_POWER        RK30_PIN4_PD1
#define RK30_MODEM_POWER_IOMUX  rk29_mux_api_set(GPIO4D1_SMCDATA9_TRACEDATA9_NAME, GPIO4D_GPIO4D1)

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
#if defined(CONFIG_MALATA_C8005)
	rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
        rk30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	rk30_mux_api_set(GPIO4D1_SMCDATA9_TRACEDATA9_NAME, GPIO4D_GPIO4D1);
	rk30_mux_api_set(GPIO2B7_LCDC1DATA15_SMCADDR19_HSADCDATA7_NAME, GPIO2B_GPIO2B7);
	rk30_mux_api_set(GPIO2C0_LCDCDATA16_GPSCLK_HSADCCLKOUT_NAME, GPIO2C_GPIO2C0);
#else
      rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
        rk30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	rk30_mux_api_set(GPIO2B7_LCDC1DATA15_SMCADDR19_HSADCDATA7_NAME, GPIO2B_GPIO2B7);
	rk30_mux_api_set(GPIO2C0_LCDCDATA16_GPSCLK_HSADCCLKOUT_NAME, GPIO2C_GPIO2C0);
	return 0;
#endif
}

static int mu509_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mu509_data rk29_mu509_info = {
	.io_init = mu509_io_init,
  	.io_deinit = mu509_io_deinit,
  	#if defined(CONFIG_MALATA_C8005)
       .modem_power_en = RK30_PIN4_PD1,
	.bp_power = RK30_PIN2_PB6,//RK30_PIN4_PD1,
	.bp_reset = RK30_PIN4_PD2,
	.ap_wakeup_bp = RK30_PIN2_PB7 ,
	.bp_wakeup_ap = RK30_PIN6_PA1, 
	#else
	.modem_power_en = RK30_PIN6_PB2,//RK30_PIN4_PD1,
	.bp_power = RK30_PIN2_PB6,//RK30_PIN4_PD1,
	.bp_reset = RK30_PIN4_PD2,
	.ap_wakeup_bp = RK30_PIN2_PB7,
	.bp_wakeup_ap = RK30_PIN6_PA0, 
	#endif
};
struct platform_device rk29_device_mu509 = {	
        .name = "mu509",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk29_mu509_info,
	}    	
    };
#endif
#if defined(CONFIG_MW100)
static int mw100_io_init(void)
{
	 rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
	 rk30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	 rk30_mux_api_set(GPIO2B7_LCDC1DATA15_SMCADDR19_HSADCDATA7_NAME, GPIO2B_GPIO2B7);
	 rk30_mux_api_set(GPIO2C0_LCDCDATA16_GPSCLK_HSADCCLKOUT_NAME, GPIO2C_GPIO2C0);
	return 0;
}

static int mw100_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mw100_data rk29_mw100_info = {
	.io_init = mw100_io_init,
  	.io_deinit = mw100_io_deinit,
	.modem_power_en = RK30_PIN6_PB2,
	.bp_power = RK30_PIN2_PB6,
	.bp_reset = RK30_PIN4_PD2,
	.ap_wakeup_bp = RK30_PIN2_PB7,
	.bp_wakeup_ap = RK30_PIN6_PA0,
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
	 rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
	 rk30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	 rk30_mux_api_set(GPIO2B7_LCDC1DATA15_SMCADDR19_HSADCDATA7_NAME, GPIO2B_GPIO2B7);
	 rk30_mux_api_set(GPIO2C0_LCDCDATA16_GPSCLK_HSADCCLKOUT_NAME, GPIO2C_GPIO2C0);
	 rk30_mux_api_set(GPIO2C1_LCDC1DATA17_SMCBLSN0_HSADCDATA6_NAME, GPIO2C_GPIO2C1);
	 rk30_mux_api_set(GPIO2C1_LCDC1DATA17_SMCBLSN0_HSADCDATA6_NAME, GPIO2C_GPIO2C1);
	return 0;

	 return 0;
}

static int mt6229_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mt6229_data rk29_mt6229_info = {
	.io_init = mt6229_io_init,
  	.io_deinit = mt6229_io_deinit,
	.modem_power_en = RK30_PIN6_PB2,
	.bp_power = RK30_PIN2_PB6,
	.modem_usb_en = RK30_PIN2_PC0,
	.modem_uart_en = RK30_PIN2_PC1,
	.bp_wakeup_ap = RK30_PIN6_PA1,
	.ap_ready = RK30_PIN2_PB7,

};
struct platform_device rk29_device_mt6229 = {	
        .name = "mt6229",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk29_mt6229_info,
	}    	
    };
#endif
#if defined(CONFIG_SEW868)
static int sew868_io_init(void)
{
	rk30_mux_api_set(GPIO2B6_LCDC1DATA14_SMCADDR18_TSSYNC_NAME, GPIO2B_GPIO2B6);
    rk30_mux_api_set(GPIO4D2_SMCDATA10_TRACEDATA10_NAME, GPIO4D_GPIO4D2);
	rk30_mux_api_set(GPIO4D4_SMCDATA12_TRACEDATA12_NAME, GPIO4D_GPIO4D4);
	return 0;
}
static int sew868_io_deinit(void)
{
	return 0;
}
struct rk30_sew868_data rk30_sew868_info = {
	.io_init = sew868_io_init,
  	.io_deinit = sew868_io_deinit,
	.bp_power = RK30_PIN6_PB2, 
	.bp_power_active_low = 1,
	.bp_sys = RK30_PIN2_PB6, 
	.bp_reset = RK30_PIN4_PD2, 
	.bp_reset_active_low = 1,
	.bp_wakeup_ap = RK30_PIN4_PD4, 
	.ap_wakeup_bp = NULL,
};

struct platform_device rk30_device_sew868 = {	
        .name = "sew868",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk30_sew868_info,
	}    	
    };
#endif

/*MMA8452 gsensor*/
#if defined (CONFIG_GS_MMA8452)
#define MMA8452_INT_PIN   RK30_PIN4_PC0

static int mma8452_init_platform_hw(void)
{
	rk30_mux_api_set(GPIO4C0_SMCDATA0_TRACEDATA0_NAME, GPIO4C_GPIO4C0);

	return 0;
}

#ifdef CONFIG_MALATA_D8002
static struct sensor_platform_data mma8452_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = mma8452_init_platform_hw,
        .orientation = {-1, 0, 0, 0, 0, 1, 0, -1, 0},
};
#else
static struct sensor_platform_data mma8452_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = mma8452_init_platform_hw,
        .orientation =  {-1, 0, 0, 0, 0, -1, 0, -1, 0},
};
#endif
#endif

#ifdef CONFIG_GS_MMA7660
#define MMA7660_INT_PIN   RK30_PIN4_PC0

static int mma7660_init_platform_hw(void)
{
	rk30_mux_api_set(GPIO4C0_SMCDATA0_TRACEDATA0_NAME, GPIO4C_GPIO4C0);

	return 0;
}

#ifdef CONFIG_G_SENSOR_DEVICE
static struct gsensor_platform_data mma7660_info = {
	.model= 7660,
	.swap_xy = 0,
	.init_platform_hw = mma7660_init_platform_hw,
};
#endif
/*modify the sensor orientation:hwrotation=0,fakerotation=false xmymk 20131206 */
#if defined(CONFIG_MALATA_C1016)
static struct sensor_platform_data mma7660_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 0,
	.poll_delay_ms = 20,
        .init_platform_hw = mma7660_init_platform_hw,
        //.orientation = {-1, 0, 0, 0, 1, 0, 0, 0, -1},//270 true
        .orientation = {0, -1, 0, -1, 0, 0, 0, 0, -1},//0 false
        //.orientation = {0, -1, 0, 0, 0, -1, -1, 0, 0 },
};
/*end xmymk 20131206*/
#elif defined(CONFIG_MALATA_D1002) ||defined(CONFIG_MALATA_C7008)
static struct sensor_platform_data mma7660_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 0,
	.poll_delay_ms = 20,
        .init_platform_hw = mma7660_init_platform_hw,
        .orientation = {0, -1, 0, 0, 0, -1, -1, 0, 0 },
};

#elif defined(CONFIG_MALATA_D7007)
static struct sensor_platform_data mma7660_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 20,
        .init_platform_hw = mma7660_init_platform_hw,
        .orientation = {1, 0, 0, 0, 0, -1, 0, -1, 0},
};
#elif defined(CONFIG_MALATA_C7019B)
static struct sensor_platform_data mma7660_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 10,
        .init_platform_hw = mma7660_init_platform_hw,
        .orientation = { 1, 0, 0, 0, 0, -1, 0, -1, 0},
};
#elif defined(CONFIG_MALATA_D7006)
static struct sensor_platform_data mma7660_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 20,
        .init_platform_hw = mma7660_init_platform_hw,
        .orientation = { 1, 0, 0, 0, 0, -1, 0, -1, 0},
};
#else
static struct sensor_platform_data mma7660_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 20,
        .init_platform_hw = mma7660_init_platform_hw,
       // .orientation = { 0, 1, 0, 0, 0, -1, 1, 0, 0},
       .orientation = { 0, 1, 0, 1, 0, 0, 0, 0, -1},
    
};
#endif
#endif
#if defined (CONFIG_GS_LIS3DH)
#define LIS3DH_INT_PIN   RK30_PIN4_PC0

static int lis3dh_init_platform_hw(void)
{
        rk30_mux_api_set(GPIO4C0_SMCDATA0_TRACEDATA0_NAME, GPIO4C_GPIO4C0);

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
#if defined (CONFIG_GS_KXTIK)
#define KXTIK_INT_PIN   RK30_PIN4_PC0

static int kxtik_init_platform_hw(void)
{
	rk30_mux_api_set(GPIO4C0_SMCDATA0_TRACEDATA0_NAME, GPIO4C_GPIO4C0);

	return 0;
}

static struct sensor_platform_data kxtik_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
	.init_platform_hw = kxtik_init_platform_hw,
	.orientation = {0, 1, 0, 0, 0, -1, 1, 0, 0},
};

#endif
#if defined (CONFIG_COMPASS_AK8975)
static struct sensor_platform_data akm8975_info =
{
	.type = SENSOR_TYPE_COMPASS,
	.irq_enable = 0,
	.poll_delay_ms = 20,
	.m_layout = 
	{
#if defined(CONFIG_MALATA_C7022)
		{
			{-1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
#elif defined(CONFIG_MALATA_D8005)
		{
			{0, -1, 0},
			{1, 0, 0},
			{0, 0, -1},

		},
#elif defined(CONFIG_MALATA_C1016)
		{
			{0, -1, 0},
			{-1, 0, 0},
			{0, 0, -1},

		},
#else
		{
			{0, -1, 0},
			{-1, 0, 0},
			{0, 0, -1},
		},
#endif
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
#if defined(CONFIG_MALATA_C1016)
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
#define L3G4200D_INT_PIN  RK30_PIN4_PC3

static int l3g4200d_init_platform_hw(void)
{
	rk30_mux_api_set(GPIO4C3_SMCDATA3_TRACEDATA3_NAME, GPIO4C_GPIO4C3);
	
	return 0;
}

static struct sensor_platform_data l3g4200d_info = {
	.type = SENSOR_TYPE_GYROSCOPE,
	.irq_enable = 0,
	.poll_delay_ms = 10,
	.orientation = {1, 0, 0, 0, -1, 0, 0, 0, -1},
	.init_platform_hw = l3g4200d_init_platform_hw,
	.x_min = 5,//x_min,y_min,z_min = (0-100) according to hardware
	.y_min = 5,
	.z_min = 5,
};

#endif

#ifdef CONFIG_LS_CM3217
static struct sensor_platform_data cm3217_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 0,
	.poll_delay_ms = 500,
};

#endif

#if defined(CONFIG_PS_AL3006)
static struct sensor_platform_data proximity_al3006_info = {
	.type = SENSOR_TYPE_PROXIMITY,
	.irq_enable = 1,
	.poll_delay_ms = 200,
};
#endif

#if defined(CONFIG_PS_STK3171)
static struct sensor_platform_data proximity_stk3171_info = {
	.type = SENSOR_TYPE_PROXIMITY,
	.irq_enable = 1,
	.poll_delay_ms = 200,
};
#endif
#ifdef CONFIG_LS_LSL29023
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
#ifdef CONFIG_MPU_SENSORS_MPU6050B1
#define MPU6050_INT_PIN  RK30_PIN4_PC3
static struct mpu_platform_data mpu6050_data = {
	.int_config 	= 0x10,
	.level_shifter	= 0,
#if (defined(CONFIG_MALATA_D7008))|| (defined(CONFIG_MALATA_C7022))
	.orientation 	= {
					  0,   1,   0,
					 1,   0,   0,
					  0,   0, -1},
#elif defined(CONFIG_MALATA_D8005)
	.orientation	= {
					  -1, 0, 0,
					   0, 1, 0,
					   0, 0, -1},

#else
	.orientation 	= {
					0,   -1,   0,
					  1,   0,   0 ,
					  0,   0, 1},
#endif
};
static struct ext_slave_platform_data mpu_compass_data = {
	.address 		= 0x0d,
	.adapt_num 	= 0,
	.bus 			= EXT_SLAVE_BUS_PRIMARY,
#if defined(CONFIG_MALATA_D8002)
	.orientation 	= {
					  1,  0,  0,
					  0,  -1, 0,
					  0,  0,  -1},
#elif defined(CONFIG_MALATA_D8005)
	.orientation	= {
					  -1,  0,  0,
					  0,  -1, 0,
					  0,  0,  1},


#else
	.orientation 	= {
					  0,  -1,   0,
					  1,   0,  0,
					  0,   0,   1},
#endif
};
#endif

#if defined(CONFIG_LS_AL3006)
static struct sensor_platform_data light_al3006_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 1,
	.poll_delay_ms = 200,
};
#endif

#if defined(CONFIG_LS_STK3171)
static struct sensor_platform_data light_stk3171_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 1,
	.poll_delay_ms = 200,
};
#endif
#ifdef CONFIG_FB_ROCKCHIP

#define LCD_CS_MUX_NAME    GPIO4C7_SMCDATA7_TRACEDATA7_NAME
#define LCD_CS_PIN         RK30_PIN4_PC7
#define LCD_CS_VALUE       GPIO_HIGH

#define LCD_EN_MUX_NAME    GPIO4C7_SMCDATA7_TRACEDATA7_NAME
#define LCD_EN_PIN         RK30_PIN6_PB4
#define LCD_EN_VALUE       GPIO_LOW

#if defined(CONFIG_MALATA_D8002)
#define LCD_STANDBY_MUX_NAME    GPIO4D2_SMCDATA10_TRACEDATA10_NAME
#define LCD_STANDBY_PIN         RK30_PIN4_PD2
#define LCD_STANDBY_VALUE       GPIO_HIGH

#define LCD_RST_MUX_NAME    GPIO4D6_SMCDATA14_TRACEDATA14_NAME
#define LCD_RST_PIN         RK30_PIN4_PD6
#define LCD_RST_VALUE       GPIO_HIGH

#define BL_VCC_EN_MUX_NAME    GPIO0C6_TRACECLK_SMCADDR2_NAME
#define BL_VCC_EN_PIN         RK30_PIN0_PC6
#define BL_VCC_EN_VALUE       GPIO_HIGH
#endif
static int rk_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
	int ret = 0;
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
	}

	mdelay(50);
	rk30_mux_api_set(LCD_CS_MUX_NAME, GPIO4C_GPIO4C7);
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

#if defined(CONFIG_MALATA_D8002)
	rk30_mux_api_set(LCD_STANDBY_MUX_NAME, GPIO4D_GPIO4D2);
	ret = gpio_request(LCD_STANDBY_PIN, NULL);
	if (ret != 0)
	{
		gpio_free(LCD_STANDBY_PIN);
		printk(KERN_ERR "request lcd cs pin fail!\n");
		return -1;
	}
	else
	{
		gpio_direction_output(LCD_STANDBY_PIN, LCD_STANDBY_VALUE);
	}

	rk30_mux_api_set(LCD_RST_MUX_NAME, GPIO4D_GPIO4D1);
	ret = gpio_request(LCD_RST_PIN, NULL);
	if (ret != 0)
	{
		gpio_free(LCD_RST_PIN);
		printk(KERN_ERR "request lcd cs pin fail!\n");
		return -1;
	}
	else
	{
		gpio_direction_output(LCD_RST_PIN, LCD_RST_VALUE);
	}

	rk30_mux_api_set(BL_VCC_EN_MUX_NAME, GPIO0C_GPIO0C6);
	ret = gpio_request(BL_VCC_EN_PIN, NULL);
	if (ret != 0)
	{
		gpio_free(BL_VCC_EN_PIN);
		printk(KERN_ERR "request lcd cs pin fail!\n");
		return -1;
	}
	else
	{
		gpio_direction_output(BL_VCC_EN_PIN, BL_VCC_EN_VALUE);
	}
#endif
	return 0;
}
int rk_fb_io_disable(void)
{
#if defined(CONFIG_MALATA_D8002)
	gpio_set_value(BL_VCC_EN_PIN, BL_VCC_EN_VALUE? 0:1);
#endif
	gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE? 0:1);
	gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE? 0:1);
#if defined(CONFIG_MALATA_D8002)
	gpio_set_value(LCD_RST_PIN, LCD_RST_VALUE? 0:1);
	gpio_set_value(LCD_STANDBY_PIN, LCD_STANDBY_VALUE? 0:1);
#endif

	return 0;
}
EXPORT_SYMBOL(rk_fb_io_disable);

static int rk_fb_io_enable(void)
{
	gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
#if defined(CONFIG_MALATA_D8002)
	gpio_set_value(BL_VCC_EN_PIN, BL_VCC_EN_VALUE);
	gpio_set_value(LCD_RST_PIN, LCD_RST_VALUE);
	gpio_set_value(LCD_STANDBY_PIN, LCD_STANDBY_VALUE);
	msleep(150);	//wait for power stable
#endif

	return 0;
}

#if defined(CONFIG_LCDC0_RK30)
struct rk29fb_info lcdc0_screen_info = {
#if defined(CONFIG_RK_HDMI) && defined(CONFIG_HDMI_SOURCE_LCDC0) && defined(CONFIG_DUAL_LCDC_DUAL_DISP_IN_KERNEL)
		.prop	   = EXTEND,	//extend display device
		.io_init    = NULL,
		.io_disable = NULL,
		.io_enable = NULL,
		.set_screen_info = hdmi_init_lcdc,
#else
		.prop	   = PRMRY,		//primary display device
		.io_init   = rk_fb_io_init,
		.io_disable = rk_fb_io_disable,
		.io_enable = rk_fb_io_enable,
		.set_screen_info = set_lcd_info,
#endif

};
#endif

#if defined(CONFIG_LCDC1_RK30)
struct rk29fb_info lcdc1_screen_info = {
#if defined(CONFIG_RK_HDMI) && defined(CONFIG_HDMI_SOURCE_LCDC1) && defined(CONFIG_DUAL_LCDC_DUAL_DISP_IN_KERNEL)
		.prop	   = EXTEND,	//extend display device
		.io_init    = NULL,
		.io_disable = NULL,
		.io_enable = NULL,
		.set_screen_info = hdmi_init_lcdc,
#else
		.prop	   = PRMRY,		//primary display device
		.io_init   = rk_fb_io_init,
		.io_disable = rk_fb_io_disable,
		.io_enable = rk_fb_io_enable,
		.set_screen_info = set_lcd_info,
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

#if defined(CONFIG_LCDC0_RK30)
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
#if defined(CONFIG_LCDC1_RK30) 
extern struct rk29fb_info lcdc1_screen_info;
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

#if defined(CONFIG_DP501)
	#define DVDD33_EN_PIN 		RK30_PIN6_PB4
	#define DVDD33_EN_VALUE 	GPIO_LOW

	#define DVDD12_EN_PIN 		RK30_PIN4_PC7
	#define DVDD12_EN_VALUE 	GPIO_HIGH

	#define EDP_RST_PIN 		RK30_PIN2_PC4
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

		ret = gpio_request(DVDD12_EN_PIN, "dvdd18_en_pin");
		if (ret != 0)
		{
			gpio_free(DVDD12_EN_PIN);
			printk(KERN_ERR "request dvdd18 en pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(DVDD12_EN_PIN, DVDD12_EN_VALUE);
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
			msleep(10);
			gpio_direction_output(EDP_RST_PIN, GPIO_HIGH);
		}
		return 0;

	}
	static struct dp501_platform_data dp501_platform_data = {
		.power_ctl 	= rk_edp_power_ctl,
		.dvdd33_en_pin 	= DVDD33_EN_PIN,
		.dvdd33_en_val 	= DVDD33_EN_VALUE,
		.dvdd18_en_pin 	= DVDD12_EN_PIN,
		.dvdd18_en_val 	= DVDD12_EN_VALUE,
		.edp_rst_pin   	= EDP_RST_PIN,
	};
#endif

#if defined(CONFIG_MFD_RK610)
#define RK610_RST_PIN_MUX_NAME		GPIO0C6_TRACECLK_SMCADDR2_NAME	
#define RK610_RST_PIN_MUX_MODE		GPIO0C_GPIO0C6
#define RK610_RST_PIN 			RK30_PIN0_PC6
static int rk610_power_on_init(void)
{
	int ret;
	if(RK610_RST_PIN != INVALID_GPIO)
	{
		rk30_mux_api_set(RK610_RST_PIN_MUX_NAME,RK610_RST_PIN_MUX_MODE);
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
	.spk_ctl_io = RK30_PIN4_PC6,
	.io_init = rk610_codec_io_init,
};
#endif
#ifdef CONFIG_ANDROID_TIMED_GPIO
static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = RK30_PIN0_PA4,
		.max_timeout = 1000,
		.active_low = 0,
		.adjust_time =20,      //adjust for diff product
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

#ifdef CONFIG_LEDS_GPIO_PLATFORM
static struct gpio_led rk29_leds[] = {
	{
		.name = "button-backlight",
		.gpio = RK30_PIN4_PD7,
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
#define IRDA_IRQ_PIN           RK30_PIN6_PA1

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
 * SDMMC devices,  include the module of SD,MMC,and SDIO.noted by xbw at 2012-03-05
**************************************************************************************************/
#ifdef CONFIG_SDMMC_RK29
#include "board-rk30-sdk-sdmmc.c"
#endif

#ifdef CONFIG_SDMMC0_RK29
static int rk29_sdmmc0_cfg_gpio(void)
{
#ifdef CONFIG_SDMMC_RK29_OLD
	rk30_mux_api_set(GPIO3B1_SDMMC0CMD_NAME, GPIO3B_SDMMC0_CMD);
	rk30_mux_api_set(GPIO3B0_SDMMC0CLKOUT_NAME, GPIO3B_SDMMC0_CLKOUT);
	rk30_mux_api_set(GPIO3B2_SDMMC0DATA0_NAME, GPIO3B_SDMMC0_DATA0);
	rk30_mux_api_set(GPIO3B3_SDMMC0DATA1_NAME, GPIO3B_SDMMC0_DATA1);
	rk30_mux_api_set(GPIO3B4_SDMMC0DATA2_NAME, GPIO3B_SDMMC0_DATA2);
	rk30_mux_api_set(GPIO3B5_SDMMC0DATA3_NAME, GPIO3B_SDMMC0_DATA3);

	rk30_mux_api_set(GPIO3B6_SDMMC0DETECTN_NAME, GPIO3B_GPIO3B6);

	rk30_mux_api_set(GPIO3A7_SDMMC0PWREN_NAME, GPIO3A_GPIO3A7);
	gpio_request(RK30_PIN3_PA7, "sdmmc-power");
	gpio_direction_output(RK30_PIN3_PA7, GPIO_LOW);

#else
	    rk29_sdmmc_set_iomux(0, 0xFFFF);

    #if defined(CONFIG_SDMMC0_RK29_SDCARD_DET_FROM_GPIO)
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO);
    #else
	    rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FMUX);
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

};
#endif // CONFIG_SDMMC0_RK29

#ifdef CONFIG_SDMMC1_RK29
#define CONFIG_SDMMC1_USE_DMA
static int rk29_sdmmc1_cfg_gpio(void)
{
#if defined(CONFIG_SDMMC_RK29_OLD)
	rk30_mux_api_set(GPIO3C0_SMMC1CMD_NAME, GPIO3C_SMMC1_CMD);
	rk30_mux_api_set(GPIO3C5_SDMMC1CLKOUT_NAME, GPIO3C_SDMMC1_CLKOUT);
	rk30_mux_api_set(GPIO3C1_SDMMC1DATA0_NAME, GPIO3C_SDMMC1_DATA0);
	rk30_mux_api_set(GPIO3C2_SDMMC1DATA1_NAME, GPIO3C_SDMMC1_DATA1);
	rk30_mux_api_set(GPIO3C3_SDMMC1DATA2_NAME, GPIO3C_SDMMC1_DATA2);
	rk30_mux_api_set(GPIO3C4_SDMMC1DATA3_NAME, GPIO3C_SDMMC1_DATA3);
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
#define	CHG_EN	RK30_PIN4_PD5
#define	CHG_INT       INVALID_GPIO
#define 	DC_DET  	INVALID_GPIO
#define 	BAT_LOW  	INVALID_GPIO
#define	CHG_DET	INVALID_GPIO

#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
#define	OTG_EN	RK30_PIN6_PA0
#define	OTG_IRQ	RK30_PIN4_PD5
#define	STATU_IRQ	RK30_PIN0_PD2

static int bq24196_otg_irq_init(void)
{
	int ret = 0;

	rk30_mux_api_set(GPIO4D5_SMCDATA13_TRACEDATA13_NAME, GPIO4D_GPIO4D5);
	rk30_mux_api_set(GPIO0D2_I2S22CHLRCKRX_SMCOEN_NAME, GPIO0D_GPIO0D2);

	ret = gpio_request(OTG_IRQ, "otg_irq");
	if(ret)
		return ret;

	gpio_pull_updown(OTG_IRQ, GPIOPullUp);
	gpio_direction_input(OTG_IRQ);

	ret = gpio_request(STATU_IRQ, "status_irq");
	if(ret)
		return ret;

	gpio_pull_updown(STATU_IRQ, GPIOPullUp);
	gpio_direction_input(STATU_IRQ);

	return ret;
}
#endif

struct bq24196_platform_data bq24196_info = {
	.chg_en_pin=CHG_EN,
	.bat_low_pin=BAT_LOW,
	.dc_det_pin=DC_DET,
	.chg_int_pin=CHG_INT,
	.chg_det_pin=CHG_DET,
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

	rk29_mux_api_set(GPIO4D5_SMCDATA13_TRACEDATA13_NAME, GPIO4D_GPIO4D5);

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

extern int bq24196_set_input_current(int);

#endif

#ifdef CONFIG_BATTERY_RK30_ADC_FAC
#ifdef CONFIG_BATTERY_RK30_USB_AND_CHARGE
#define DC_DET_PIN		INVALID_GPIO
#else
#define DC_DET_PIN		RK30_PIN6_PA5
#endif

#ifdef CONFIG_BATTERY_RK30_USB_CHARGE
#define USB_SUPPORT	1
#else
#define USB_SUPPORT	0
#endif

#define USB_DET_PIN	RK30_PIN6_PA3
#ifdef CONFIG_BATTERY_RK30_USB_AND_CHARGE
#define CHARGE_TYPE_PIN	RK30_PIN6_PB2
#else
#define CHARGE_TYPE_PIN	INVALID_GPIO
#endif
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

static int charge_en_init(void)
{
	int ret = 0;

	rk30_mux_api_set(GPIO4D5_SMCDATA13_TRACEDATA13_NAME, GPIO4D_GPIO4D5);

	ret = gpio_request(RK30_PIN4_PD5, NULL);
	if (ret != 0)
	{
		gpio_free(RK30_PIN4_PD5);
		printk(KERN_ERR "request charge en pin fail!\n");
		return -1;
	}
	else
	{
		gpio_direction_output(RK30_PIN4_PD5, 0);
		gpio_set_value(RK30_PIN4_PD5,  0);
		gpio_free(RK30_PIN4_PD5);
	}

	return 0;
}

static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
	.io_init	= charge_en_init,
	.dc_det_pin      = DC_DET_PIN,//INVALID_GPIO,
	.batt_low_pin    = INVALID_GPIO,
#if !defined(CONFIG_BATTERY_BQ24196)
	.charge_set_pin  = RK30_PIN4_PD5,
#else
	.charge_set_pin  = INVALID_GPIO,
#endif
	.charge_set_level = GPIO_LOW,
	.charge_ok_pin   = RK30_PIN6_PA6,
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

#if defined(CONFIG_BATTERY_RK30_ADC)
static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
        .dc_det_pin      = RK30_PIN6_PA5,
        .batt_low_pin    = RK30_PIN6_PA0,
        .charge_set_pin  = INVALID_GPIO,
        .charge_ok_pin   = RK30_PIN6_PA6,
        .dc_det_level    = GPIO_LOW,
        .charge_ok_level = GPIO_HIGH,
};

static struct platform_device rk30_device_adc_battery = {
        .name   = "rk30-battery",
        .id     = -1,
        .dev = {
                .platform_data = &rk30_adc_battery_platdata,
        },
};
#endif

#ifdef CONFIG_RK30_PWM_REGULATOR
const static int pwm_voltage_map[] = {
	950000,975000,1000000, 1025000, 1050000, 1075000, 1100000, 1125000, 1150000, 1175000, 1200000, 1225000, 1250000, 1275000, 1300000, 1325000, 1350000, 1375000, 1400000
};

static struct regulator_consumer_supply pwm_dcdc1_consumers[] = {
	{
		.supply = "vdd_core",
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
		.pwm_id = 3,
		.pwm_gpio = RK30_PIN0_PD7,
		.pwm_iomux_name = GPIO0D7_PWM3_NAME,
		.pwm_iomux_pwm = GPIO0D_PWM3,
		.pwm_iomux_gpio = GPIO0D_GPIO0D6,
		.pwm_voltage = 1100000,
		.suspend_voltage = 1050000,
		.min_uV = 950000,
		.max_uV	= 1400000,
		.coefficient = 455,	//45.5%
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
#if defined (CONFIG_BATTERY_BT_B0B6G)
	.capacity_min = 4,
#elif defined(CONFIG_BATTERY_BT_C0B2G)
	.capacity_min = 3,
#elif defined(CONFIG_BATTERY_BT_B0BFH)
	.capacity_min = 6,
#elif defined(CONFIG_BATTERY_BT_B0BFN_3474107)
	.capacity_min = 6,
#else
	.capacity_min = 5,
#endif

	.bat_num = 1,
	.dc_check_pin = BQ_DC_DET_PIN,
	.init_dc_check_pin = bq24751_dc_detect_init,
	.get_charging_stat = bq27541_charging_stat,
};
#endif

#if defined (CONFIG_BATTERY_BQ27425)
#define BQ27425_DC_DET_PIN		DC_DET_PIN
#define BQ27425_USB_DET_PIN		USB_DET_PIN
#define BQ27425_LOW_POWER_PIN	INVALID_GPIO

void bq27425_io_init(void)
{
	int ret = 0;

	if(BQ27425_DC_DET_PIN != INVALID_GPIO){
		ret = gpio_request(BQ27425_DC_DET_PIN, "dc_det");
		if (ret) {
			printk("failed to request BQ27425_DC_DET_PIN gpio\n");
		}

		gpio_pull_updown(BQ27425_DC_DET_PIN, 1);
		ret = gpio_direction_input(BQ27425_DC_DET_PIN);
		if (ret) {
			printk("failed to set gpio BQ27425_DC_DET_PIN input\n");
		}
		gpio_free(BQ27425_DC_DET_PIN);
	}

	if(BQ27425_USB_DET_PIN != INVALID_GPIO){
		ret = gpio_request(BQ27425_USB_DET_PIN, "usb_det");
		if (ret) {
			printk("failed to request BQ27425_USB_DET_PIN gpio\n");
		}

		gpio_pull_updown(BQ27425_USB_DET_PIN, 1);
		ret = gpio_direction_input(BQ27425_USB_DET_PIN);
		if (ret) {
			printk("failed to set gpio BQ27425_USB_DET_PIN input\n");
		}
		gpio_free(BQ27425_USB_DET_PIN);
	}

	if(BQ27425_LOW_POWER_PIN != INVALID_GPIO){
		ret = gpio_request(BQ27425_LOW_POWER_PIN, "low_power");
		if (ret) {
			printk("failed to request BQ27425_LOW_POWER_PIN gpio\n");
		}

		gpio_pull_updown(BQ27425_LOW_POWER_PIN, 1);
		ret = gpio_direction_input(BQ27425_LOW_POWER_PIN);
		if (ret) {
			printk("failed to set gpio BQ27425_LOW_POWER_PIN input\n");
		}
		gpio_free(BQ27425_LOW_POWER_PIN);
	}
}

static int bq27425_charging_stat(void)
{
	return (!gpio_get_value(BQ27425_DC_DET_PIN)) || (!gpio_get_value(BQ27425_USB_DET_PIN));
}

struct bq27425_platform_data bq27425_data = {
	.capacity_max = 100,
	.capacity_min = 6,
	.bat_num = 1,
	.dc_check_pin = BQ27425_DC_DET_PIN,
	.wake_irq = BQ27425_LOW_POWER_PIN,
	.io_init = bq27425_io_init,
	.get_charging_stat = bq27425_charging_stat,
};
#endif

#if defined (CONFIG_BATTERY_BQ27410)
#define BQ27410_DC_DET_PIN		DC_DET_PIN
#define BQ27410_USB_DET_PIN		USB_DET_PIN
#define BQ27410_LOW_POWER_PIN	RK30_PIN0_PD5

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
	.capacity_min = 6,
	.bat_num = 1,
	.dc_check_pin = BQ27410_DC_DET_PIN,
	.wake_irq = BQ27410_LOW_POWER_PIN,
	.io_init = bq27410_io_init,
	.get_charging_stat = bq27410_charging_stat,
	.low_power_pin = RK30_PIN0_PD5,
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
            .name       = GPIO3C7_SDMMC1WRITEPRT_NAME,
            .fgpio      = GPIO3C_GPIO3C7,
        },
    },

    .reset_gpio         = { // BT_RST
        .io             = RK30_PIN3_PD1, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = GPIO3D1_SDMMC1BACKENDPWR_NAME,
            .fgpio      = GPIO3D_GPIO3D1,
        },
    },

    .wake_gpio          = { // BT_WAKE, use to control bt's sleep and wakeup
        .io             = RK30_PIN3_PC6, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = GPIO3C6_SDMMC1DETECTN_NAME,
            .fgpio      = GPIO3C_GPIO3C6,
        },
    },

    .wake_host_irq      = { // BT_HOST_WAKE, for bt wakeup host when it is in deep sleep
        .gpio           = {
            .io         = RK30_PIN6_PA7, // set io to INVALID_GPIO for disable it
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
            .name       = GPIO1A3_UART0RTSN_NAME,
            .fgpio      = GPIO1A_GPIO1A3,
            .fmux       = GPIO1A_UART0_RTS_N,
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

#if defined(CONFIG_MT5931_MT6622)
static struct mt6622_platform_data mt6622_platdata = {
    .power_gpio         = { // BT_REG_ON
        .io             = RK30_PIN3_PC7, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = NULL,
        },
    },

    .reset_gpio         = { // BT_RST
        .io             = RK30_PIN3_PD1,
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = NULL,
        },
    },

    .irq_gpio           = {
        .io             = RK30_PIN6_PA7,
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = NULL,
        },
    }
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
#if defined(CONFIG_WIFI_CONTROL_FUNC)||defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
	&rk29sdk_wifi_device,
#endif

#if defined(CONFIG_MT6620)
    &mt3326_device_gps,
#endif   

#ifdef CONFIG_RK29_SUPPORT_MODEM
	&rk30_device_modem,
#endif
#if defined(CONFIG_MU509)
	&rk29_device_mu509,
#endif
#if defined(CONFIG_MW100)
	&rk29_device_mw100,
#endif
#if defined(CONFIG_MT6229)
	&rk29_device_mt6229,
#endif
#if defined(CONFIG_SEW868)
	&rk30_device_sew868,
#endif
#if defined(CONFIG_BATTERY_RK30_ADC)||defined(CONFIG_BATTERY_RK30_ADC_FAC)
 	&rk30_device_adc_battery,
#endif
#ifdef CONFIG_RFKILL_RK
	&device_rfkill_rk,
#endif
#ifdef CONFIG_MT5931_MT6622
	&device_mt6622,
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

#if defined(CONFIG_LCDC0_RK30)
	lcdc0 = &device_lcdc0,
#endif

#if defined(CONFIG_LCDC1_RK30)
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
#if defined(CONFIG_MALATA_D8002)
		.addr	        = 0x1c,
#else
		.addr	        = 0x1d,
#endif
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
#if defined (CONFIG_GS_LIS3DH)
	{
		.type	        = "gs_lis3dh",
		.addr	        = 0x19,   //0x19(SA0-->VCC), 0x18(SA0-->GND)
		.flags	        = 0,
		.irq	        = LIS3DH_INT_PIN,
		.platform_data = &lis3dh_info,
	},
#endif
#if defined (CONFIG_GS_KXTIK)
	{
		.type	        = "gs_kxtik",
		.addr	        = 0x0F,
		.flags	        = 0,
		.irq	        = KXTIK_INT_PIN,
		.platform_data = &kxtik_info,
	},
#endif
#if defined (CONFIG_COMPASS_AK8975)
	{
		.type          = "ak8975",
	#if	defined(CONFIG_MALATA_D8005)
		.addr          = 0x0c,
	#else
		.addr          = 0x0d,
	#endif
		.flags         = 0,
		.irq           = RK30_PIN4_PC1,
		.platform_data = &akm8975_info,
	},
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
#if defined (CONFIG_LS_AL3006)
	{
		.type           = "light_al3006",
		.addr           = 0x1c,             //sel = 0; if sel =1, then addr = 0x1D
		.flags          = 0,
		.irq            = RK30_PIN6_PA2,	
		.platform_data = &light_al3006_info,
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
#if defined(CONFIG_MALATA_D7008)||defined(CONFIG_MALATA_D8005)
{
	.type          = "ak8963",
	.addr          = 0x0c,
	.flags         = 0,
	.irq           = 0,
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
#if defined (CONFIG_LS_STK3171)
	{
		.type           = "ls_stk3171",
		.addr           = 0x48,            
		.flags          = 0,
		.irq            = RK30_PIN6_PA2,	
		.platform_data = &light_stk3171_info,
	},
#endif


#if defined (CONFIG_PS_AL3006)
	{
		.type           = "proximity_al3006",
		.addr           = 0x1c,             //sel = 0; if sel =1, then addr = 0x1D
		.flags          = 0,
		.irq            = RK30_PIN6_PA2,	
		.platform_data = &proximity_al3006_info,
	},
#endif

#if defined (CONFIG_PS_STK3171)
	{
		.type           = "ps_stk3171",
		.addr           = 0x48,            
		.flags          = 0,
		.irq            = RK30_PIN6_PA2,	
		.platform_data = &proximity_stk3171_info,
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
			.platform_data		= &rk610_codec_pdata,					
		},
#endif
#endif
#ifdef CONFIG_ITE9133_CONTROL	
#if defined (CONFIG_ROCKCHIP_DTV) 
    {
      .type           = "tstv_control",
      .addr           = (0x38>>1),
      .flags          = 0,
      .irq            = RK29_PIN0_PA2,
    },
#endif
#endif
#ifdef CONFIG_MTV818_CONTROL	
#if defined (CONFIG_ROCKCHIP_DTV) 
    {
      .type           = "tstv_control",
      .addr           = (0x86>>1),
      .flags          = 0,
      .irq            = RK29_PIN0_PA2,
    },
#endif
#endif
#ifdef CONFIG_GX1131_CONTROL	
#if defined (CONFIG_ROCKCHIP_DTV) 
    {
      .type           = "tstv_control",
      .addr           = (0xD0>>1),
      .flags          = 0,
      .irq            = RK29_PIN0_PA2,
    },
#endif
#endif

#ifdef CONFIG_DIBCOM8096_CONTROL	
#if defined (CONFIG_ROCKCHIP_DTV) 
    {
      .type           = "tstv_control",
      .addr           = (0x86>>1),
      .flags          = 0,
      //.irq            = RK29_PIN0_PA2,
    },
#endif
#endif
};
#endif
#define PMIC_TYPE_WM8326	1
#define PMIC_TYPE_TPS65910	2
int __sramdata g_pmic_type =  0;
#ifdef CONFIG_I2C1_RK30
#ifdef CONFIG_MFD_WM831X_I2C
#include "board-rk30-sdk-wm8326.c"
#endif
#ifdef CONFIG_MFD_TPS65910
#define TPS65910_HOST_IRQ        RK30_PIN6_PA4
#include "board-rk30-sdk-tps65910.c"
#endif

static struct i2c_board_info __initdata i2c1_info[] = {
#if defined (CONFIG_MFD_WM831X_I2C)
	{
		.type          = "wm8326",
		.addr          = 0x34,
		.flags         = 0,
		.irq           = RK30_PIN6_PA4,
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
#if defined (CONFIG_BATTERY_BQ27425)
	{
		.type		= "bq27425",
		.addr	= 0x55,
		.flags	= 0,
		.irq		= 0,
		.platform_data=&bq27425_data,
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
}

#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {
#if defined (CONFIG_TOUCHSCREEN_GT8XX)
	{
		.type          = "Goodix-TS",
		.addr          = 0x55,
		.flags         = 0,
		.irq           = RK30_PIN4_PC2,
		.platform_data = &goodix_info,
	},
#endif
#if  defined(CONFIG_TOUCHSCREEN_GT811_MALATA)
	{
		.type          = "Goodix-TS",
		.addr          = 0x5d,
		.flags         = 0,
		.irq           = TOUCH_INT_PIN,
		.platform_data = &goodix_info,
	},
#endif
#if  defined(CONFIG_TOUCHSCREEN_GT9110_MALATA)
	{
		.type          = "Goodix9110-TS",
		.addr          = 0x14,
		.flags         = 0,
		.irq           = RK30_PIN4_PC2,
		.platform_data = &goodix9110_info,
	},
#endif
#if  defined(CONFIG_TOUCHSCREEN_GT8110_MALATA)
	{
		.type          = "Goodix-TS",
		.addr          = 0x5c,
		.flags         = 0,
		.irq           = TOUCH_INT_PIN,
		.platform_data = &goodix_info,
	},
#endif
#if defined (CONFIG_LS_LSL29023)
	{
		.type          = "lightsensor",
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
#if defined (CONFIG_TOUCHSCREEN_CT36X)
	{
		.type          = "ct36x_ts",	
		.addr     	   = 0x01,
		.flags         = 0,
		.irq           = RK30_PIN4_PC2,
		.platform_data = &ct36x_info,
	},
#endif
#if defined (CONFIG_LS_CM3217)
	{
		.type          = "light_cm3217",
		.addr          = 0x10,
		.flags         = 0,
		.platform_data = &cm3217_info,
	},
#endif

#if defined(CONFIG_DP501)
	{
		.type = "dp501",
		.addr = 0x30,
		.flags = 0,
		.platform_data = &dp501_platform_data,
	},
#endif
#if  defined(CONFIG_TOUCHSCREEN_FT5606_MALATA)
	{
		.type          = "ft5x0x_ts",
		.addr          = 0x3E, 
		.flags         = 0,
		.irq           = RK30_PIN4_PC2,
		.platform_data = &ft5x0x_touch_info,
	},
#endif
#ifdef CONFIG_ENCRYPTION_DM2016_MALATA
	{
		.type		   = "dm2016_encry",
		.addr		   = 0x50,
		.flags		   = 0,
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
#ifdef CONFIG_DIBCOM7090_CONTROL	
#if defined (CONFIG_ROCKCHIP_DTV) 
  {
    .type           = "tstv_control",
    .addr           = (0x86>>1),
    .flags          = 0,
    //.irq            = RK29_PIN0_PA2,
  },
#endif
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
#if   defined(CONFIG_TOUCHSCREEN_GT811_MALATA)||defined(CONFIG_TOUCHSCREEN_GT8110_MALATA)
	goodix_init_platform_hw();
#endif
}
//end of i2c

#define POWER_ON_PIN RK30_PIN6_PB0   //power_hold
static void rk30_pm_power_off(void)
{
	printk(KERN_ERR "rk30_pm_power_off start...\n");
	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	#if defined(CONFIG_MFD_WM831X)	
	if(pmic_is_wm8326())
	{
		wm831x_set_bits(Wm831x,WM831X_GPIO_LEVEL,0x0001,0x0000);  //set sys_pwr 0
		wm831x_device_shutdown(Wm831x);//wm8326 shutdown
	}
	#endif
	#if defined(CONFIG_MFD_TPS65910)
	if(pmic_is_tps65910())
	{
		tps65910_device_shutdown();//tps65910 shutdown
	}
	#endif

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
#ifdef CONFIG_POWER_ON_CHARGER_DISPLAY
	store_boot_source();
#endif
	avs_init();
	gpio_request(POWER_ON_PIN, "poweronpin");
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
	
	pm_power_off = rk30_pm_power_off;
	
#if defined (CONFIG_BATTERY_BQ24196)
	bq24196_charge_en();
#endif
	rk30_i2c_register_board_info();
	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	rk_platform_add_display_devices();
	#if defined(CONFIG_BATTERY_RK30_ADC) || defined(CONFIG_BATTERY_RK30_ADC_FAC)
	board_usb_detect_init(USB_DET_PIN);
#endif

#if defined(CONFIG_WIFI_CONTROL_FUNC)
	rk29sdk_wifi_bt_gpio_control_init();
#elif defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
    rk29sdk_wifi_combo_module_gpio_init();
#endif

#if defined(CONFIG_MT6620)
    clk_set_rate(clk_get_sys("rk_serial.0", "uart"), 48*1000000);
#endif
#ifdef CONFIG_HALL_KEY
	rk30_mux_api_set(GPIO4D1_SMCDATA9_TRACEDATA9_NAME, GPIO4D_GPIO4D1);
#endif
#if defined(CONFIG_MALATA_D8002)
	gpio_request(RK30_PIN0_PD0, "waltop_sleep");
	gpio_request(RK30_PIN0_PD1, "waltop_reset");
	gpio_request(RK30_PIN4_PC4, "waltop_int");

	rk30_mux_api_set(GPIO0D0_I2S22CHCLK_SMCCSN0_NAME, GPIO0D_GPIO0D0);
	rk30_mux_api_set(GPIO0D1_I2S22CHSCLK_SMCWEN_NAME, GPIO0D_GPIO0D1);
	rk30_mux_api_set(GPIO4D1_SMCDATA9_TRACEDATA9_NAME, GPIO4D_GPIO4D1);
	gpio_direction_output(RK30_PIN0_PD0, GPIO_LOW);
	gpio_direction_output(RK30_PIN0_PD1, GPIO_HIGH);

	rk29_mux_api_set(GPIO4C4_SMCDATA4_TRACEDATA4_NAME, GPIO4C_GPIO4C4);
	gpio_pull_updown(RK30_PIN4_PC4, PullDisable);
	gpio_direction_input(RK30_PIN4_PC4);
#elif defined(CONFIG_MALATA_D7007)
	gpio_pull_updown(RK30_PIN4_PC4, PullDisable);
	gpio_direction_input(RK30_PIN4_PC4);
#endif
}

static void __init rk30_reserve(void)
{
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
	board_mem_reserved();
}

/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs 
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 * @logic_volt	: logic voltage arm requests depend on frequency
 * comments	: min arm/logic voltage
 */
#if 0
static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
	{.frequency = 252 * 1000,	.cpu_volt = 1075 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1100 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},//1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},//1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},//1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1225 * 1000,	.logic_volt = 1200 * 1000},//1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1300 * 1000,	.logic_volt = 1200 * 1000},//1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1350 * 1000,	.logic_volt = 1250 * 1000},//1.300V/1.150V
	{.frequency = 1608 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1300 * 1000},//1.325V/1.175V
	{.frequency = CPUFREQ_TABLE_END},
};
#else
static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
    {.frequency = 252 * 1000, .cpu_volt = 1100 * 1000, .logic_volt = 1125 * 1000},//0.975V/1.000V
    {.frequency = 504 * 1000, .cpu_volt = 1100 * 1000, .logic_volt = 1125* 1000},//0.975V/1.000V
    {.frequency = 816 * 1000, .cpu_volt = 1125 * 1000, .logic_volt = 1150 * 1000},//1.000V/1.025V
    {.frequency = 1008 * 1000, .cpu_volt = 1150 * 1000, .logic_volt = 1150 * 1000},//1.025V/1.050V
    {.frequency = 1200 * 1000, .cpu_volt = 1200 * 1000, .logic_volt = 1200 * 1000},//1.100V/1.050V
    {.frequency = 1272 * 1000, .cpu_volt = 1250 * 1000, .logic_volt = 1200 * 1000},//1.150V/1.100V
    {.frequency = 1416 * 1000, .cpu_volt = 1325 * 1000, .logic_volt = 1225 * 1000},//1.225V/1.100V
    {.frequency = 1512 * 1000, .cpu_volt = 1375 * 1000, .logic_volt = 1250 * 1000},//1.300V/1.150V
    {.frequency = 1608 * 1000, .cpu_volt = 1450 * 1000, .logic_volt = 1325 * 1000},//1.325V/1.175V
    //{.frequency = 1512 * 1000, .cpu_volt = 1400 * 1000, .logic_volt = 1250 * 1000},//1.300V/1.150V
    //{.frequency = 1608 * 1000, .cpu_volt = 1450 * 1000, .logic_volt = 1300 * 1000},//1.325V/1.175V
    {.frequency = CPUFREQ_TABLE_END},};

#endif
#ifdef CONFIG_GPU_SUPPORT_266M
struct cpufreq_frequency_table dvfs_gpu_table[] = {
	{.frequency = 266 * 1000,	.index = 1050 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
int gpu_freq_max = 266;
#else
struct cpufreq_frequency_table dvfs_gpu_table[] = {
	{.frequency = 266 * 1000,	.index = 1050 * 1000},
	{.frequency = 400 * 1000,	.index = 1275 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
int gpu_freq_max = 400;
#endif

struct cpufreq_frequency_table dvfs_gpu_table_266[] = {
	{.frequency = 266 * 1000,	.index = 1050 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

struct cpufreq_frequency_table dvfs_gpu_table_400[] = {
	{.frequency = 266 * 1000,	.index = 1050 * 1000},
	{.frequency = 400 * 1000,	.index = 1275 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
static struct cpufreq_frequency_table dvfs_ddr_table[] = {
	{.frequency = 300 * 1000,	.index = 1050 * 1000},
	{.frequency = 400 * 1000,	.index = 1125 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

#define DVFS_CPU_TABLE_SIZE	(ARRAY_SIZE(dvfs_cpu_logic_table))
static struct cpufreq_frequency_table cpu_dvfs_table[DVFS_CPU_TABLE_SIZE];
static struct cpufreq_frequency_table dep_cpu2core_table[DVFS_CPU_TABLE_SIZE];

void __init board_clock_init(void)
{
	rk30_clock_data_init(periph_pll_default, codec_pll_default, RK30_CLOCKS_DEFAULT_FLAGS);
	dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);
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
