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
#include <linux/mfd/rk808.h>
#include <linux/mfd/ricoh619.h>
#include <linux/regulator/rk29-pwm-regulator.h>
#include <plat/efuse.h>
#include <plat/ddr.h>

#if defined (CONFIG_BATTERY_BQ24196)
#include <linux/bq24196_chargeIc.h>
#endif

#if defined (CONFIG_BATTERY_BQ27541)
#include <linux/power/bq27541_battery.h>
#endif

#if defined (CONFIG_BATTERY_BQ27410)
#include <linux/power/bq27410_battery.h>
#endif

#ifdef CONFIG_MFD_RT5025
#include <linux/mfd/rt5025.h>
#endif

#ifdef CONFIG_CW2015_BATTERY
#include <linux/power/cw2015_battery.h>
#endif
#if defined(CONFIG_MFD_RK610)
#include <linux/mfd/rk610_core.h>
#endif

#if defined(CONFIG_MFD_RK616)
#include <linux/mfd/rk616.h>
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
#if defined(CONFIG_MW100)
#include <linux/mw100.h>
#endif
#if defined(CONFIG_MT6229) || defined(CONFIG_MT6229_UNAP)
#include <linux/mt6229.h>
#endif
#if defined(CONFIG_ANDROID_TIMED_GPIO)
#include "../../../drivers/staging/android/timed_gpio.h"
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5606_MALATA
#include <linux/i2c/ft5x06_ts2.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_GT9110_MALATA
#include <linux/gt9xx.h>
#endif

#ifdef CONFIG_RK_REMOTECTL
#include <linux/input/remotectl.h>
#endif

#ifdef CONFIG_SWITCH_GPIO_FOR_VIDEO
#include <linux/switch_video.h>
#endif

#ifdef CONFIG_MPU_SENSORS_MPU6050B1
#include <linux/mpu.h>
#endif

#ifdef CONFIG_HAPTICS_DRV2605
#include <../../../drivers/haptics/drv2605.h>
#endif

#if defined (CONFIG_RK_HEADSET_DET) || defined (CONFIG_RK_HEADSET_IRQ_HOOK_ADC_DET)
#include <../../../drivers/headset_observe/rk_headset.h>
#endif

#if defined(CONFIG_MT6620) && !defined(CONFIG_MTK_COMBO_MT66XX)
#include <linux/gps.h>
#endif

#if defined(CONFIG_MTK_COMBO_MT66XX)
#include <linux/combo_mt66xx.h>
#endif
#include "board-rk3168-tb-camera.c"

#if defined(CONFIG_TOUCHSCREEN_GT8XX)
#define TOUCH_RESET_PIN  RK30_PIN0_PB6
#define TOUCH_PWR_PIN    RK30_PIN0_PC5   // need to fly line by hardware engineer

/* Android Parameter */
static int ap_mdm = 0;
module_param(ap_mdm, int, 0644);
static int ap_has_alsa = 0;
module_param(ap_has_alsa, int, 0644);
static int ap_data_only = 2;
module_param(ap_data_only, int, 0644);
static int ap_has_earphone = 0;
module_param(ap_has_earphone, int, 0644);


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

static struct spi_board_info board_spi_devices[] = {
};

/***********************************************************
*	rk30  backlight
************************************************************/
#ifdef CONFIG_BACKLIGHT_RK29_BL
#if defined(CONFIG_MALATA_D1014)||defined(CONFIG_MALATA_D8009)||defined(CONFIG_MALATA_D7022)||defined(CONFIG_MALATA_D7803A)\
	||(defined(CONFIG_MALATA_D7803)&&defined(CONFIG_RK616_MIPI_DSI)) || defined(CONFIG_MALATA_E4201) || defined(CONFIG_MALATA_D1024)
#define PWM_ID            1
#define PWM_MODE          PWM1
#else
#define PWM_ID            3
#define PWM_MODE          PWM3
#endif
#if (defined(CONFIG_MALATA_D7803)&&!defined(CONFIG_MALATA_D7803A)&&!defined(CONFIG_RK616_MIPI_DSI))|| defined(CONFIG_MALATA_D8006)\
	|| defined(CONFIG_MALATA_D1012)||defined(CONFIG_MALATA_D7014)
#define PWM_EFFECT_VALUE  0
#else
#define PWM_EFFECT_VALUE  1
#endif

#if !(defined(CONFIG_MALATA_D1004)||defined(CONFIG_MALATA_D1024))
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
#elif defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_E4201)
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

#if (defined(CONFIG_MALATA_D7803) && !defined(CONFIG_RK616_MIPI_DSI))
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
#elif defined(CONFIG_MALATA_D7005A) || defined(CONFIG_MALATA_D7005B)
	.min_brightness=10,
	.max_brightness=180,
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
#elif defined(CONFIG_MALATA_E4201)
	.min_brightness=25,
	.max_brightness=255,
	.brightness_mode = BRIGHTNESS_MODE_LINE,
#else
	.min_brightness=10,    //  20
	.brightness_mode = BRIGHTNESS_MODE_LINE,
	.max_brightness=255,
#endif

#if defined(CONFIG_MALATA_E4201)
	.pre_div = 160,
#else
	.pre_div = 30 * 1000,
#endif
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
#if defined(CONFIG_MALATA_D1003)
	.bp_power = RK30_PIN2_PD4,
	.modem_usb_en = RK30_PIN2_PD5,
#else
	.bp_power = RK30_PIN2_PD5,
	.modem_usb_en = RK30_PIN2_PD4,
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
#if defined(CONFIG_MALATA_D1024)
	iomux_set(GPIO0_C5);
	iomux_set(GPIO0_D6);
#elif defined(CONFIG_MALATA_E4201)
	iomux_set(GPIO0_C6);
#else
	iomux_set(GPIO2_D5);
	iomux_set(GPIO0_C6);
#endif
	return 0;
}

static int mt6229_unap_io_deinit(void)
{
	
	return 0;
}
 
struct rk29_mt6229_data rk29_mt6229_unap_info = {
	.io_init = mt6229_unap_io_init,
	.io_deinit = mt6229_unap_io_deinit,
#if defined(CONFIG_MALATA_D1024)
	.modem_power_en = RK30_PIN0_PD6,
	.bp_power = RK30_PIN0_PC5,
#elif defined(CONFIG_MALATA_E4201)
	.modem_power_en = RK30_PIN0_PC6,
	.bp_power = RK30_PIN0_PB4,
#else
	.modem_power_en = RK30_PIN0_PC6,
	.bp_power = RK30_PIN2_PD5,
#endif
};
struct platform_device rk29_device_mt6229_unap = {
	.name = "mt6229_unap",
	.id = -1,
	.dev		= {
		.platform_data = &rk29_mt6229_unap_info,
	}
	};
#endif

#if defined (CONFIG_RK_HEADSET_DET) || defined (CONFIG_RK_HEADSET_IRQ_HOOK_ADC_DET)
#if defined(CONFIG_MALATA_D7005A) || defined(CONFIG_MALATA_D1012B) || defined(CONFIG_MALATA_D1024)
#define	HEADSET_DET_PIN	RK30_PIN3_PA0
#define	HEADSET_DET_PIN_IOMUX	MMC0_RSTNOUT
#elif defined(CONFIG_MALATA_D1025)
#define	HEADSET_DET_PIN	RK30_PIN3_PB1
#define	HEADSET_DET_PIN_IOMUX	MMC0_WRPRT
#else
#define	HEADSET_DET_PIN	RK30_PIN3_PB1
#define	HEADSET_DET_PIN_IOMUX	MMC0_WRPRT
#endif
static int rk_headset_io_init(void)
{
	int ret;
	ret = gpio_request(HEADSET_DET_PIN, NULL);
	if(ret)
		return ret;

	iomux_set_gpio_mode(iomux_mode_to_gpio(HEADSET_DET_PIN_IOMUX));

	gpio_pull_updown(HEADSET_DET_PIN, PullDisable);
	gpio_direction_input(HEADSET_DET_PIN);
	return 0;
};

struct rk_headset_pdata rk_headset_info = {
	.Headset_gpio		= HEADSET_DET_PIN,
#if defined(CONFIG_MALATA_D7803) && defined(CONFIG_RK616_MIPI_DSI)
	.headset_in_type = HEADSET_IN_LOW,
#else
	.headset_in_type = HEADSET_IN_HIGH,
#endif
	.Hook_adc_chn = 2,
	.hook_key_code = KEY_MEDIA,
	.headset_io_init = rk_headset_io_init,
};

struct platform_device rk_device_headset = {
		.name	= "rk_headsetdet",
		.id 	= 0,
		.dev    = {
		    .platform_data = &rk_headset_info,
		}
};
#endif


/*kxtik gsensor*/
#if defined (CONFIG_GS_KXTIK)
#define KXTIK_INT_PIN   RK30_PIN0_PB7

static int kxtik_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data kxtik_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 0,
	.poll_delay_ms = 20,
	.init_platform_hw = kxtik_init_platform_hw,
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
        .orientation = {  0, -1, 0, -1, 0, 0,0, 0, -1},
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
	.orientation = {-1, 0, 0, 0, 1, 0, 0, 0, -1},
	#elif defined(CONFIG_MALATA_D7022)
	.orientation = {1, 0, 0, 0, -1, 0, 0, 0, -1},//land
	//.orientation = {0, -1, 0, -1, 0, 0, 0, 0, -1},//Portrait
	#else
	.orientation = {0, -1, 0, -1, 0, 0, 0, 0, -1},
	#endif
#elif defined(CONFIG_MALATA_D1025)
	.orientation = {0, 1, 0, 1, 0, 0, 0, 0, -1},
#elif defined(CONFIG_MALATA_D7803) && defined(CONFIG_RK616_MIPI_DSI)
	.orientation = {0, 1, 0, 1, 0, 0, 0, 0, -1},
#else
	.orientation = {1, 0, 0, 0,-1, 0, 0, 0, -1},
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
#if defined(CONFIG_MALATA_D7803) && defined(CONFIG_RK616_MIPI_DSI)
#define MPU6050_INT_PIN  RK30_PIN1_PA6
#else
#define MPU6050_INT_PIN  RK30_PIN0_PB4
#endif
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
			{-1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
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
	#elif defined(CONFIG_MALATA_D1024)
		{
			{-1, 0, 0},
			{0, 1, 0},
			{0, 0, -1},
		},
	#elif defined(CONFIG_MALATA_D7803) && defined(CONFIG_RK616_MIPI_DSI)
		{
			{-1, 0, 0},
			{0, 1, 0},
			{0, 0, -1},
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
			{0,-1, 0},
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
			{0, -1, 0},
			{0, 0, -1},
		},
	#elif defined(CONFIG_MALATA_D1024)
	{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, -1},
	},
	#elif defined(CONFIG_MALATA_D7803) && defined(CONFIG_RK616_MIPI_DSI)
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
#if defined(CONFIG_MALATA_D7803A)
		.reset_pin = RK30_PIN3_PD6,   //RESET PIN
#else
		.reset_pin = RK30_PIN3_PD4,   //RESET PIN
#endif
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
#elif defined(CONFIG_MALATA_E4201)
#define LCD_CS_PIN         RK30_PIN0_PD6
#define LCD_CS_VALUE       GPIO_HIGH
#else
#define LCD_CS_PIN         INVALID_GPIO
#define LCD_CS_VALUE       GPIO_HIGH
#endif

#if defined(CONFIG_MALATA_D7803)||defined(CONFIG_MALATA_D7014)
#define LCD_EN_PIN       INVALID_GPIO
#elif defined(CONFIG_MALATA_E4201)
#define LCD_EN_PIN         RK30_PIN0_PA2
#else
#define LCD_EN_PIN         RK30_PIN0_PB0
#endif

#if defined(CONFIG_MALATA_D1024)||defined(CONFIG_MALATA_E4201)
#define LCD_EN_VALUE       GPIO_HIGH
#else
#define LCD_EN_VALUE       GPIO_LOW
#endif

#if defined(CONFIG_MALATA_D1004) || defined(CONFIG_MALATA_D7005) ||(defined(CONFIG_MALATA_D7803) && !defined(CONFIG_RK616_MIPI_DSI))\
	|| defined(CONFIG_MALATA_D8006)
#define LCD_DRV_PIN         RK30_PIN0_PA3
#define LCD_DRV_VALUE       GPIO_HIGH
#else
#define LCD_DRV_PIN         INVALID_GPIO
#define LCD_DRV_VALUE       GPIO_HIGH
#endif

// Add this pin for additional LCD control sequence on C1017 v3.0, xmylm, 20130704
#if defined(CONFIG_MALATA_C1017)
#define LCD_VDD_EN          RK30_PIN1_PA6
#elif defined(CONFIG_MALATA_E4201)
#define LCD_VDD_EN          RK30_PIN0_PA1
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
		}else{
			 printk("zengshanbin_test set LCD_EN_PIN enable\n");
			 gpio_direction_output(LCD_EN_PIN, LCD_EN_VALUE);
		}
	}

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
		}else{
			 gpio_direction_output(LCD_CS_PIN, LCD_CS_VALUE);
			 printk("zengshanbin_test set LCD_CS_PIN enable\n");
		}
	}
#if defined(CONFIG_MALATA_D1004) && defined(CONFIG_LCD_EJ101H_D1004) || defined(CONFIG_MALATA_D1014)
    ;
#else
    msleep(20);
#endif
#elif defined(CONFIG_LCD_TPT420H2_1920x1080)
	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_CS_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_CS_PIN);
			printk(KERN_ERR "request lcd cs pin fail!\n");
		}

		gpio_direction_output(LCD_CS_PIN, !LCD_CS_VALUE);
	}

#if defined(CONFIG_MALATA_E4201)
	struct regulator *ldo_18;
	ldo_18 = regulator_get(NULL, "act_ldo3");
	if (ldo_18 == NULL || IS_ERR(ldo_18))
		printk("get lcd ldo failed!\n");

	regulator_set_voltage(ldo_18, 1800000, 1800000);
	regulator_enable(ldo_18);
	regulator_put(ldo_18);
#endif

	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_EN_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_EN_PIN);
			printk(KERN_ERR "request lcd en pin fail!\n");
		}

		gpio_direction_output(LCD_EN_PIN, LCD_EN_VALUE);
	}

	msleep(200);
	if (LCD_VDD_EN != INVALID_GPIO)
	{
		ret = gpio_request(LCD_VDD_EN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_VDD_EN);
			printk(KERN_ERR "request LCD_VDD_EN pin fail!\n");
		}

		gpio_direction_output(LCD_VDD_EN, LCD_VDD_EN_VALUE);
	}
	msleep(200);

	return 0;

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
#if defined(CONFIG_LCD_TPT420H2_1920x1080)
#if defined(CONFIG_MALATA_E4201)
	struct regulator *ldo_18;
	ldo_18 = regulator_get(NULL, "act_ldo3");
	if (ldo_18 == NULL || IS_ERR(ldo_18))
		printk("get lcd ldo failed!\n");

	while(regulator_is_enabled(ldo_18) > 0)
		regulator_disable(ldo_18);
	regulator_put(ldo_18);
#endif

	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, !LCD_EN_VALUE);
	}
	msleep(200);

	if (LCD_VDD_EN != INVALID_GPIO)
	{
		gpio_set_value(LCD_VDD_EN, !LCD_VDD_EN_VALUE);
	}
	msleep(200);

#else
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
#endif

	return 0;
}
EXPORT_SYMBOL(rk_fb_io_disable);
static int rk_fb_io_enable(void)
{
#if defined(CONFIG_MALATA_D7803A)
	if(LCD_DRV_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_DRV_PIN, LCD_DRV_VALUE);
	}
#endif
// Modified by EvanZeng for lcd EJ101H in project D1004, adjusting the power timing for resolving the "white screen"
#if defined(CONFIG_LCD_EJ101H)
	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
	}

#if defined(CONFIG_MALATA_D1004) && defined(CONFIG_LCD_EJ101H_D1004) || defined(CONFIG_MALATA_D1014)
	msleep(30); //[M]According to datasheet, modify 200ms->30ms to fix "white screen" when powerup && resume
#else
	msleep(200);
#endif

	if(LCD_CS_PIN !=INVALID_GPIO)
		gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);

#if defined(CONFIG_MALATA_D1004) && defined(CONFIG_LCD_EJ101H_D1004) || defined(CONFIG_MALATA_D1014)
	;
#else
    msleep(20);
#endif

#elif defined(CONFIG_LCD_TPT420H2_1920x1080)

#if defined(CONFIG_MALATA_E4201)
	if (LCD_VDD_EN != INVALID_GPIO)
	{
		gpio_set_value(LCD_VDD_EN, LCD_VDD_EN_VALUE);
	}
	msleep(200);

	struct regulator *ldo_18;
	ldo_18 = regulator_get(NULL, "act_ldo3");
	if (ldo_18 == NULL || IS_ERR(ldo_18))
		printk("get lcd ldo failed!\n");

	regulator_set_voltage(ldo_18, 1800000, 1800000);
	regulator_enable(ldo_18);
	regulator_put(ldo_18);
#endif

	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
	}

	msleep(200);

#else
	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
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
#if !defined(CONFIG_MALATA_D7803A)
		gpio_set_value(LCD_DRV_PIN, LCD_DRV_VALUE);
#endif
	}

	return 0;
}

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK3188)
#if defined(CONFIG_MALATA_D7803)
struct rk29lcd_info rk29lcd_infoe = {
	.reset_pin = RK30_PIN0_PD5,
#if defined(CONFIG_RK616_MIPI_DSI)
	.lcd_en_pin = RK30_PIN0_PB0,
	.lcd_en_pin_level = GPIO_HIGH,
#else
	.lcd_en_pin = INVALID_GPIO,
#endif
};
#elif defined(CONFIG_MALATA_E4201)
struct rk29lcd_info rk29lcd_infoe = {
	.lcd_en_pin = INVALID_GPIO,
	.cs_pin = LCD_CS_PIN,
	.lcd_en_pin_level = LCD_VDD_EN_VALUE,
	.lcd_cs_pin_level = LCD_CS_VALUE,
};
#endif
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
#if defined(CONFIG_MALATA_D7803)||defined(CONFIG_MALATA_E4201)
	.lcd_info = &rk29lcd_infoe,
#endif
#endif

};
#endif

#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK3188)
struct rk29fb_info lcdc1_screen_info = {
#if defined(CONFIG_RK_HDMI)
#if defined(CONFIG_HDMI_SOURCE_LCDC0) && defined(CONFIG_DUAL_LCDC_DUAL_DISP_IN_KERNEL)
	.prop	   = PRMRY, 	//primary display device
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,
#if defined(CONFIG_MALATA_D7803)||defined(CONFIG_MALATA_E4201)
	.lcd_info = &rk29lcd_infoe,
#endif
#else
	.prop	   = EXTEND,	//extend display device
	.io_init	= NULL,
	.io_disable = NULL,
	.io_enable = NULL,
	.set_screen_info = hdmi_init_lcdc,
#endif
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
#define RK610_RST_PIN 			RK30_PIN2_PC5
static int rk610_power_on_init(void)
{
	int ret;
	if(RK610_RST_PIN != INVALID_GPIO)
	{
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

#if defined(CONFIG_MFD_RK616)
#define RK616_RST_PIN 			RK30_PIN3_PB2
#define RK616_PWREN_PIN			RK30_PIN0_PA3
#define RK616_SCL_RATE			(100*1000)   //i2c scl rate

static int rk616_power_on_init(void)
{
	int ret;

	if(RK616_PWREN_PIN != INVALID_GPIO)
	{
		ret = gpio_request(RK616_PWREN_PIN, "rk616 pwren");
		if (ret)
		{
			printk(KERN_ERR "rk616 pwren gpio request fail\n");
		}
		else
		{
			gpio_direction_output(RK616_PWREN_PIN,GPIO_HIGH);
		}
	}

	if(RK616_RST_PIN != INVALID_GPIO)
	{
		ret = gpio_request(RK616_RST_PIN, "rk616 reset");
		if (ret)
		{
			printk(KERN_ERR "rk616 reset gpio request fail\n");
		}
		else
		{
			gpio_direction_output(RK616_RST_PIN, GPIO_HIGH);
			msleep(2);
			gpio_direction_output(RK616_RST_PIN, GPIO_LOW);
			msleep(10);
			gpio_set_value(RK616_RST_PIN, GPIO_HIGH);
		}
	}

	return 0;
}


static int rk616_power_deinit(void)
{
	gpio_set_value(RK616_PWREN_PIN,GPIO_LOW);
	gpio_set_value(RK616_RST_PIN,GPIO_LOW);
	gpio_free(RK616_PWREN_PIN);
	gpio_free(RK616_RST_PIN);

	return 0;
}

#if defined(CONFIG_MALATA_D1025) && !defined(CONFIG_MALATA_D1012B)
#define SPK_CTL_PIN	RK30_PIN2_PD7
#define HP_CTL_PIN	RK30_PIN0_PA5
#define MIC_SEL_PIN	RK30_PIN0_PA1
#elif defined(CONFIG_MALATA_D1025) && defined(CONFIG_MALATA_D1012B)
#define SPK_CTL_PIN	RK30_PIN0_PA5
#define HP_CTL_PIN	RK30_PIN2_PD7
#define MIC_SEL_PIN	RK30_PIN2_PD5
#elif defined(CONFIG_MALATA_D7005A)
#define SPK_CTL_PIN	RK30_PIN0_PA5
#define HP_CTL_PIN	RK30_PIN1_PA5
#define MIC_SEL_PIN	RK30_PIN0_PA6
#elif defined(CONFIG_MALATA_D1024)
#define SPK_CTL_PIN	RK30_PIN2_PD7
#define HP_CTL_PIN	RK30_PIN1_PA5
#define MIC_SEL_PIN	RK30_PIN3_PB1
#elif defined(CONFIG_MALATA_D7803) && defined(CONFIG_RK616_MIPI_DSI)
#define SPK_CTL_PIN	RK30_PIN0_PA3
#define HP_CTL_PIN	RK30_PIN1_PA5
#define MIC_SEL_PIN	INVALID_GPIO
#else
#define SPK_CTL_PIN	RK30_PIN0_PA5
#define HP_CTL_PIN	RK30_PIN2_PD7
#define MIC_SEL_PIN	RK30_PIN0_PA1
#endif

static struct rk616_platform_data rk616_pdata = {
	.power_init = rk616_power_on_init,
	.power_deinit = rk616_power_deinit,
	.scl_rate   = RK616_SCL_RATE,
	.lcd0_func = INPUT,             //port lcd0 as input
	.lcd1_func = INPUT,             //port lcd1 as input
	.lvds_ch_nr = 1,		//the number of used lvds channel
	.hdmi_irq = RK30_PIN2_PD6,
	.spk_ctl_gpio = SPK_CTL_PIN,
	.hp_ctl_gpio = HP_CTL_PIN,
	.mic_sel_gpio = MIC_SEL_PIN,
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
#if defined(CONFIG_MALATA_E4201)
#define RK_HDMI_POWERON_PIN	RK30_PIN0_PB7
#else
#define RK_HDMI_POWERON_PIN	INVALID_GPIO
#endif

static int rk_hdmi_power_init(void)
{
	int ret;

	if(RK_HDMI_POWERON_PIN != INVALID_GPIO)
	{
		if (gpio_request(RK_HDMI_POWERON_PIN, NULL)) {
			printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
			return -1;
		}
		gpio_direction_output(RK_HDMI_POWERON_PIN, GPIO_LOW);
		gpio_set_value(RK_HDMI_POWERON_PIN, GPIO_LOW);
		msleep(100);
	}

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
#if defined(CONFIG_MALATA_D7005A) ||(defined(CONFIG_MALATA_D7803) && defined(CONFIG_RK616_MIPI_DSI))
		.gpio = RK30_PIN2_PD7,
#elif defined(CONFIG_MALATA_D1024)
		.gpio = RK30_PIN0_PD6,
#else
		.gpio = RK30_PIN0_PC7,
#endif
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
#define IRDA_IRQ_PIN           INVALID_GPIO //RK30_PIN0_PA3

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
#define ION_RESERVE_SIZE        (100 * SZ_1M)
#define ION_RESERVE_SIZE_120M   (120 * SZ_1M)
static struct ion_platform_data rk30_ion_pdata = {
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_NOR_HEAP_ID,
			.name = "norheap",
//			.size = ION_RESERVE_SIZE,
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
#include "board-rk3168-tb-sdmmc-conifg.c"
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
	#if !defined(CONFIG_USE_SDMMC0_FOR_WIFI_DEVELOP_BOARD)
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
	#else
	.host_caps = 
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ),	
	#endif
	.io_init = rk29_sdmmc0_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif
#ifdef USE_SDMMC_DATA4_DATA7	
    .emmc_is_selected = NULL,
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
#ifdef USE_SDMMC_DATA4_DATA7	
	.emmc_is_selected = NULL,
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

#ifdef CONFIG_SDMMC2_RK29
static int rk29_sdmmc2_cfg_gpio(void)
{
    ;
}

struct rk29_sdmmc_platform_data default_sdmmc2_data = {
	.host_ocr_avail =
	    (MMC_VDD_165_195|MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 | MMC_VDD_33_34),

	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA| MMC_CAP_NONREMOVABLE  |
	        MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_UHS_SDR12 |MMC_CAP_UHS_SDR25 |MMC_CAP_UHS_SDR50),

	.io_init = rk29_sdmmc2_cfg_gpio,
	.set_iomux = rk29_sdmmc_set_iomux,
	.emmc_is_selected = sdmmc_is_selected_emmc,

	//.power_en = INVALID_GPIO,
   // .power_en_level = GPIO_LOW,

	.dma_name = "emmc",
	.use_dma = 1,

};
#endif//endif--#ifdef CONFIG_SDMMC2_RK29

/**************************************************************************************************
 * the end of setting for SDMMC devices
**************************************************************************************************/

#if defined (CONFIG_BATTERY_BQ24196)
#if defined (CONFIG_MALATA_D1024)
#define CHG_EN	RK30_PIN0_PA1
#else
#define	CHG_EN	RK30_PIN0_PC1
#endif
#if defined(CONFIG_MALATA_D7806A)
#define	INPUT_EN	RK30_PIN2_PD2
#else
#define	INPUT_EN	INVALID_GPIO
#endif
#ifdef CONFIG_BATTERY_BQ24196_OTG_MODE
#if defined (CONFIG_MALATA_D7803)
#define	OTG_EN	RK30_PIN0_PC4
#define	OTG_IRQ	RK30_PIN3_PB1
#define	STATU_IRQ	RK30_PIN0_PD7
#elif defined (CONFIG_MALATA_D1024)
#define	OTG_EN	RK30_PIN1_PA4
#define	OTG_IRQ	RK30_PIN1_PA6
#define	STATU_IRQ	RK30_PIN0_PD7
#else
#define	OTG_EN		RK30_PIN0_PB1
#define	OTG_IRQ		RK30_PIN3_PB1
#define	STATU_IRQ	RK30_PIN0_PD7
#endif

static int bq24196_otg_irq_init(void)
{
	int ret = 0;

	if(OTG_IRQ != INVALID_GPIO){
		ret = gpio_request(OTG_IRQ, "otg_irq");
		if(ret){
			printk("%s:failed to request otg_irq gpio\n", __func__);
			return ret;
		}

		gpio_pull_updown(OTG_IRQ, GPIOPullUp);
		gpio_direction_input(OTG_IRQ);
	}

	if(STATU_IRQ != INVALID_GPIO){
		ret = gpio_request(STATU_IRQ, "status_irq");
		if(ret){
			printk("%s:failed to request status_irq gpio\n", __func__);
			return ret;
		}

		gpio_pull_updown(STATU_IRQ, GPIOPullUp);
		gpio_direction_input(STATU_IRQ);
	}
	return ret;
}
#endif

static int bq24196_input_en_init(void)
{
	int ret = 0;

	if(INPUT_EN != INVALID_GPIO){
		ret = gpio_request(INPUT_EN, "input_en");
		if(ret)
		{
			printk("%s:failed to request input en pin\n", __func__);
			return ret;
		}
		gpio_pull_updown(INPUT_EN, GPIOPullDown);
		gpio_direction_output(INPUT_EN, 0);
	}
	return ret;
}
struct bq24196_platform_data bq24196_info = {
	.chg_en_pin=CHG_EN,
	.input_en_pin = INPUT_EN,
	.input_en_init = bq24196_input_en_init,
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
#if !defined(CONFIG_BATTERY_RK30_AC_CHARGE) && !defined(CONFIG_BATTERY_RK30_USB_CHARGE)
#define DC_DET_PIN		INVALID_GPIO
#define USB_DET_PIN		INVALID_GPIO
#elif !defined(CONFIG_BATTERY_RK30_AC_CHARGE)
#define DC_DET_PIN		INVALID_GPIO
#define USB_DET_PIN		RK30_PIN0_PA7
#else
#define DC_DET_PIN		RK30_PIN0_PB2
#define USB_DET_PIN		RK30_PIN0_PA7
#endif

#define CHARGE_TYPE_PIN	INVALID_GPIO
#define CHARGE_OK_PIN   RK30_PIN0_PA6

static int rk30_adc_battery_io_init(void){
	//dc charge detect pin
	int ret=0;
	if (DC_DET_PIN != INVALID_GPIO){
		ret = gpio_request(DC_DET_PIN, NULL);
		if (ret) {
			printk("failed to request dc_det gpio\n");
		}

		gpio_pull_updown(DC_DET_PIN, GPIOPullUp);//important
		ret = gpio_direction_input(DC_DET_PIN);
		if (ret) {
			printk("failed to set gpio dc_det input\n");
		}
	}

	//charge ok detect
	if (CHARGE_OK_PIN != INVALID_GPIO){
		ret = gpio_request(CHARGE_OK_PIN, NULL);
		if (ret) {
			printk("failed to request charge_ok gpio\n");
		}

		gpio_pull_updown(CHARGE_OK_PIN, GPIOPullUp);//important
		ret = gpio_direction_input(CHARGE_OK_PIN);
		if (ret) {
			printk("failed to set gpio charge_ok input\n");
		}
	}
}

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
		.dc_det_pin      = DC_DET_PIN,
		.batt_low_pin    = INVALID_GPIO,
		.charge_set_pin  = INVALID_GPIO,
		.charge_ok_pin   = RK30_PIN0_PA6,
		.usb_det_pin     = USB_DET_PIN,
		.dc_det_level    = GPIO_LOW,
		.charge_ok_level = GPIO_HIGH,

		.reference_voltage = 1800, // the rK2928 and rk3026 are 3300;RK3066 and rk29 are 2500;rk3066B and rk3188 are 1800;
		.pull_up_res = 200,     //divider resistance ,  pull-up resistor
		.pull_down_res = 120, //divider resistance , pull-down resistor

		.io_init = rk30_adc_battery_io_init,
		.is_reboot_charging = 1,
		.save_capacity   = 1 ,
		.low_voltage_protection = 3500,
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

 static int charge_status(void)
{
	int charge_pin;
	int ret = 0;

#ifdef CONFIG_BATTERY_RK30_USB_CHARGE
	if(rk30_adc_battery_platdata.usb_det_pin != INVALID_GPIO)
	{
		charge_pin = rk30_adc_battery_platdata.usb_det_pin;
		if(gpio_get_value(charge_pin) == rk30_adc_battery_platdata.usb_det_level)
			ret = 1;
	}
#endif

#ifdef CONFIG_BATTERY_RK30_AC_CHARGE
	if(rk30_adc_battery_platdata.dc_det_pin != INVALID_GPIO)
	{
		charge_pin = rk30_adc_battery_platdata.dc_det_pin;

		if(gpio_get_value(charge_pin) == rk30_adc_battery_platdata.dc_det_level)
			ret = 1;
	}
#endif

	return ret;
}

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
	.capacity_min = 1,
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

static int charge_status(void)
{
	int charge_pin;
	int ret = 0;

#ifdef CONFIG_BATTERY_USB_CHARGE
	if(cw_bat_platdata.usb_det_pin != INVALID_GPIO)
	{
		if(gpio_get_value(charge_pin) == cw_bat_platdata.usb_det_level)
			ret = 1;
	}
#endif

#ifdef CONFIG_BATTERY_AC_CHARGE
	if(cw_bat_platdata.dc_det_pin != INVALID_GPIO)
	{
		charge_pin = cw_bat_platdata.dc_det_pin;

		if(gpio_get_value(charge_pin) == cw_bat_platdata.dc_det_level)
			ret = 1;
	}
#endif

	return ret;
}

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

#ifdef CONFIG_BK3515A_COMBO
    .poweron_gpio       = { // BT_REG_ON
        .io             = RK30_PIN3_PD1,
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = "bt_poweron",
            .fgpio      = GPIO3_D1,
        },
    },

	.reset_gpio 		= { // BT_RST
		.io 			= INVALID_GPIO,
		.enable 		= GPIO_HIGH,
		.iomux			= {
			.name		= NULL,
		},
	},

    .wake_gpio          = { // BT_WAKE, 車?HOST_UART_TX?∩車?
        .io             = RK30_PIN1_PA1, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = "gpio1a1_uart0sout_name",
            .fgpio      = GPIO1_A1,
            .fmux       = UART0_SOUT,
        },
    },

	.wake_host_irq		= { // BT_HOST_WAKE, 車?HOST_UART_RX?∩車?
		.gpio			= {
			.io 		= RK30_PIN1_PA0, // set io to INVALID_GPIO for disable it
			.enable 	= GPIO_LOW, 	 // set GPIO_LOW for falling, set 0 for rising
			.iomux		= {
				.name		= "uart0_rx",
				.fgpio		= GPIO1_A0,
				.fmux		= UART0_SIN,
			},
		},
	},

#elif defined(CONFIG_RTL8723BS) 
	 .poweron_gpio		 = { // BT_REG_ON
		 .io			 = INVALID_GPIO,
		 .enable		 = GPIO_HIGH,
		 .iomux 		 = {
			 .name		 = "bt_poweron",
			 .fgpio 	 = INVALID_GPIO,
		 },
	 },

	 .reset_gpio		 = { // BT_RST
		 .io			 = RK30_PIN3_PD1, // set io to INVALID_GPIO for disable it
		 .enable		 = GPIO_LOW,
		 .iomux 		 = {
			 .name		 = "bt_reset",
			 .fgpio 	 = GPIO3_D1,
		},
	}, 

	 .wake_gpio 		 = { // BT_WAKE, use to control bt's sleep and wakeup
		 .io			 = RK30_PIN3_PC6, // set io to INVALID_GPIO for disable it
		 .enable		 = GPIO_HIGH,
		 .iomux 		 = {
			 .name		 = "bt_wake",
			 .fgpio 	 = GPIO3_C6,
		 },
	 },

	 .wake_host_irq 	 = { // BT_HOST_WAKE, for bt wakeup host when it is in deep sleep
		 .gpio			 = {
			 .io		 = RK30_PIN3_PC7, // set io to INVALID_GPIO for disable it
			 .enable	 = GPIO_LOW,	  // set GPIO_LOW for falling, set 0 for rising
			 .iomux 	 = {
				 .name	 = NULL,
			 },
		 },
	 },
#else
	.poweron_gpio		= { // BT_REG_ON
		.io 			= RK30_PIN3_PC7,
		.enable 		= GPIO_HIGH,
		.iomux			= {
			.name		= "bt_poweron",
			.fgpio		= GPIO3_C7,
		},
	},

    .reset_gpio         = { // BT_RST
        .io             = RK30_PIN3_PD1, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = "bt_reset",
            .fgpio      = GPIO3_D1,
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
#endif
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
	struct clk *gps_aclk = NULL;
	gps_aclk = clk_get(NULL, "aclk_gps");
	if(gps_aclk) {
		clk_enable(gps_aclk);
		clk_put(gps_aclk);
		printk("%s \n", __FUNCTION__);
	}
	else
		printk("get gps aclk fail\n");
	return 0;
}
int rk_disable_hclk_gps(void)
{
	struct clk *gps_aclk = NULL;
	gps_aclk = clk_get(NULL, "aclk_gps");
	if(gps_aclk) {
		//TO wait long enough until GPS ISR is finished.
		msleep(5);
		clk_disable(gps_aclk);
		clk_put(gps_aclk);
		printk("%s \n", __FUNCTION__);
	}	
	else
		printk("get gps aclk fail\n");
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
		    	.io             = RK30_PIN3_PD5, // set io to INVALID_GPIO for disable it
			    .enable         = GPIO_HIGH,
			    .iomux          = {
				    .name       = NULL,
				},
		    },

		    .reset_gpio         = { // BT_RST
		        .io             = RK30_PIN0_PD7,
		        .enable         = GPIO_HIGH,
		        .iomux          = {
		            .name       = NULL,
		        },
		    },

		    .irq_gpio           = {
			    .io             = RK30_PIN3_PD2,
			    .enable         = GPIO_HIGH,
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

#ifdef CONFIG_RK_REMOTECTL
void rk30_remotectl_iomux(void)
{
	;
}

struct RKxx_remotectl_platform_data rk30_remotectl_pdata = 
{
	.gpio	=   RK30_PIN0_PB2, 
	.wakeup	= 1,
	.rep    = 0,
	.set_iomux = rk30_remotectl_iomux,    
};

static struct platform_device rk30_device_remotectl = {
	.name	= "rkxx-remotectl",
	.id		= -1,
	.dev	= {
		.platform_data	= &rk30_remotectl_pdata,
	},
};
#endif

#ifdef CONFIG_SWITCH_GPIO_FOR_VIDEO
#define SWITCH_VIDEO_STANDBY_PIN	RK30_PIN3_PA0
#define SWITCH_VIDEO_SW1_PIN			RK30_PIN0_PC1
#define SWITCH_VIDEO_SW2_PIN			RK30_PIN3_PD6

void rk30_switch_iomux(void)
{
	int ret = 0;

	iomux_set(GPIO3_A0);
	iomux_set(GPIO0_C0);
	iomux_set(GPIO3_D6);

	printk("rk30_switch_iomux\n");
	if(SWITCH_VIDEO_STANDBY_PIN != INVALID_GPIO)
	{
		ret = gpio_request(SWITCH_VIDEO_STANDBY_PIN, "switch_standby");
		if (ret) {
			printk("failed to request SWITCH_VIDEO_STANDBY_PIN gpio\n");
			return;
		}

		ret = gpio_direction_output(SWITCH_VIDEO_STANDBY_PIN, 0);
		if (ret) {
			printk("failed to set gpio SWITCH_VIDEO_STANDBY_PIN output\n");
			return;
		}
	}

	if(SWITCH_VIDEO_SW1_PIN != INVALID_GPIO)
	{
		ret = gpio_request(SWITCH_VIDEO_SW1_PIN, "sw1");
		if (ret) {
			printk("failed to request SWITCH_VIDEO_SW1_PIN gpio\n");
			return;
		}

		ret = gpio_direction_output(SWITCH_VIDEO_SW1_PIN, 0);
		if (ret) {
			printk("failed to set gpio SWITCH_VIDEO_SW1_PIN output\n");
			return;
		}
	}

	if(SWITCH_VIDEO_SW2_PIN != INVALID_GPIO)
	{
		ret = gpio_request(SWITCH_VIDEO_SW2_PIN, "sw2");
		if (ret) {
			printk("failed to request SWITCH_VIDEO_SW2_PIN gpio\n");
			return;
		}

		ret = gpio_direction_output(SWITCH_VIDEO_SW2_PIN, 0);
		if (ret) {
			printk("failed to set gpio SWITCH_VIDEO_SW2_PIN output\n");
			return;
		}
	}
}

struct switch_video_data rk30_switch_video_pdata = 
{
	.io_init					= rk30_switch_iomux, 
	.standy_pin			= SWITCH_VIDEO_STANDBY_PIN,
	.source_switch_1		= SWITCH_VIDEO_SW1_PIN,
	.source_switch_2		= SWITCH_VIDEO_SW2_PIN,    
};

static struct platform_device rk30_device_switch = {
	.name	= "switch-video",
	.id		= -1,
	.dev	= {
		.platform_data	= &rk30_switch_video_pdata,
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
#if defined(CONFIG_WIFI_CONTROL_FUNC)||defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
	&rk29sdk_wifi_device,
#endif

#if defined(CONFIG_MT6620) && !defined(CONFIG_MTK_COMBO_MT66XX)
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
#if defined(CONFIG_MT6229_UNAP)
	&rk29_device_mt6229_unap,
#endif
#ifdef CONFIG_BATTERY_RK30_ADC_FAC
 	&rk30_device_adc_battery,
#endif
#ifdef CONFIG_RFKILL_RK
	&device_rfkill_rk,
#endif
#ifdef CONFIG_GPS_RK
	&rk_device_gps,
#endif
#if defined(CONFIG_ARCH_RK3188)
	&device_mali,
#endif
#ifdef CONFIG_MT5931_MT6622
	&device_mt6622,
#endif
#if defined (CONFIG_RK_HEADSET_DET) ||  defined (CONFIG_RK_HEADSET_IRQ_HOOK_ADC_DET)
    &rk_device_headset,
#endif
#ifdef CONFIG_RK_REMOTECTL	
    &rk30_device_remotectl,
#endif

#ifdef CONFIG_SWITCH_GPIO_FOR_VIDEO
	&rk30_device_switch,
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
#if defined(CONFIG_GS_KXTIK)
		{
			.type			= "gs_kxtik",
			.addr			= 0x0f,
			.flags			= 0,
			.irq			= KXTIK_INT_PIN,
			.platform_data = &kxtik_info,
		},
#endif
#if defined (CONFIG_GS_MMA8452)
		{
			.type			= "gs_mma8452",
			.addr			= 0x1d,
			.flags			= 0,
			.irq			= MMA8452_INT_PIN,
			.platform_data = &mma8452_info,
		},
#endif
#if defined (CONFIG_GS_MMA7660)
		{
			.type			= "gs_mma7660",
			.addr			= 0x4c,
			.flags			= 0,
			.irq			= MMA7660_INT_PIN,
			.platform_data = &mma7660_info,
		},
#endif
#if defined (CONFIG_GS_LIS3DE)
		{
			.type			= "gs_lis3de",
			.addr			= 0x29,   //0x29(SA0-->VCC), 0x28(SA0-->GND)
			.flags			= 0,
			.irq			= LIS3DE_INT_PIN,
			.platform_data = &lis3de_info,
		},
#endif
#if defined (CONFIG_GS_LIS3DH)
		{
			.type			= "gs_lis3dh",
			.addr			= 0x19,   //0x19(SA0-->VCC), 0x18(SA0-->GND)
			.flags			= 0,
			.irq			= LIS3DH_INT_PIN,
			.platform_data = &lis3dh_info,
		},
#endif
#if defined (CONFIG_COMPASS_AK8975)
		{
			.type		   = "ak8975",
		#if defined(CONFIG_MALATA_D8006)
			.addr		   = 0x0c,
		#else
			.addr		   = 0x0d,
		#endif//#if defined(CONFIG_MALATA_D8005)
			.flags		   = 0,
			.irq		   = RK30_PIN3_PD7, 
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
		.min_uv          = 1200000,
		.max_uv         = 1200000,
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
#define PMU_POWER_SLEEP INVALID_GPIO
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
#if defined (CONFIG_GB86302I)
	{
		.name		   = "act_ldo6",   //vccio_wl
		.min_uv 		 = 1800000,
		.max_uv 		= 1800000,
	},
#elif defined(CONFIG_RTL8723BS)
	{
		.name		   = "act_ldo6",   //vccio_wl
		.min_uv 		 = 3300000,
		.max_uv 		= 3300000,
	},
#else
	{
		.name		   = "act_ldo6",   //vccio_wl
		.min_uv 		 = 2800000,
		.max_uv 		= 2800000,
	},
#endif
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

#ifdef CONFIG_MFD_RK808
#define PMU_POWER_SLEEP RK30_PIN0_PA1
#define RK808_HOST_IRQ        RK30_PIN0_PB3

static struct pmu_info  rk808_dcdc_info[] = {
	{
		.name          = "vdd_cpu",   //arm
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  900000,
	},
	{
		.name          = "vdd_core",    //logic
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  900000,
	},
	{
		.name          = "rk_dcdc3",   //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		.suspend_vol  =  1200000,
	},
	{
		.name          = "rk_dcdc4",   //vccio
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3000000,
	},
	
};
static  struct pmu_info  rk808_ldo_info[] = {
	{
		.name          = "rk_ldo1",   //vcc33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol   = 3300000,
	},
	{
		.name          = "rk_ldo2",    //vcctp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		 .suspend_vol   = 3300000,

	},
	{
		.name          = "rk_ldo3",   //vdd10
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		 .suspend_vol   = 1000000,
	},
	{
		.name          = "rk_ldo4",   //vcc18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		 .suspend_vol   = 1800000,
	},
	{
		.name          = "rk_ldo5",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
		 .suspend_vol   = 2800000,
	},
	{
		.name          = "rk_ldo6",   //vdd12
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		 .suspend_vol   = 1200000,
	},
	{
		.name          = "rk_ldo7",   //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		 .suspend_vol   = 1800000,
	},
	{
		.name          = "rk_ldo8",   //vcca_33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		 .suspend_vol   = 3300000,
	},
 };

#include "board-pmu-rk808.c"
#endif
#ifdef CONFIG_MFD_RICOH619
#define PMU_POWER_SLEEP RK30_PIN0_PA1
#define RICOH619_HOST_IRQ        RK30_PIN0_PB3

static struct pmu_info  ricoh619_dcdc_info[] = {
	{
		.name          = "vdd_cpu",   //arm
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  900000,
	},
	{
		.name          = "vdd_core",    //logic
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		.suspend_vol  =  900000,
	},
	
	{
		.name          = "ricoh_dc3",   //vcc18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.suspend_vol  =  1800000,
	},
	
	{
		.name          = "ricoh_dc4",   //vccio
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.suspend_vol  =  3300000,
	},

	{
		.name          = "ricoh_dc5",   //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		.suspend_vol  =  1200000,
	},
	
};
static  struct pmu_info  ricoh619_ldo_info[] = {
	{
		.name          = "ricoh_ldo1",   //vcc30
		.min_uv          = 3000000,
		.max_uv         = 3000000,
	},
	{
		.name          = "ricoh_ldo2",    //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "ricoh_ldo3",   //vcctp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "ricoh_ldo4",   //vccsd
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "ricoh_ldo5",   //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "ricoh_ldo6",   //vdd12
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "ricoh_ldo7",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "ricoh_ldo8",   //vcc25
		.min_uv          = 2500000,
		.max_uv         = 2500000,
	},
	{
		.name          = "ricoh_ldo9",   //vdd10
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "ricoh_ldo10",   //vcca18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},	
	
 };

#include "board-pmu-ricoh619.c"
#endif

#ifdef CONFIG_MFD_RT5025
#define RT5025_HOST_IRQ        RK30_PIN0_PB3

static struct pmu_info  rt5025_dcdc_info[] = {
	{
		.name          = "vdd_cpu",   //arm
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "vdd_core",    //logic
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	
	{
		.name          = "rt5025-dcdc3",   //vccio
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	
	{
		.name          = "rt5025-dcdc4",   //vccio
		.min_uv         = 5200000,
		.max_uv         = 5200000,
	},
	
};
static  struct pmu_info  rt5025_ldo_info[] = {
	{
		.name          = "rt5025-ldo1",   //vcc18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "rt5025-ldo2",    //vddjetta
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "rt5025-ldo3",   //vdd10
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "rt5025-ldo4",   //vccjetta
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "rt5025-ldo5",   //vccio_wl
		.min_uv          = 3000000,
		.max_uv         = 3000000,
	},
	{
		.name          = "rt5025-ldo6",   //vcc_tp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},	
	
 };

#include "board-pmu-rt5025.c"
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
#if defined (CONFIG_MFD_RK808)
	{
		.type    		= "rk808",
		.addr           = 0x1b, 
		.flags			= 0,
	//	.irq            = ACT8846_HOST_IRQ,
		.platform_data=&rk808_data,
	},
#endif

#if defined (CONFIG_MFD_RICOH619)
	{
		.type                   = "ricoh619",
		.addr           = 0x32,
		.flags                  = 0,
	       .irq            = RICOH619_HOST_IRQ,
	       .platform_data=&ricoh619_data,
	},
#endif

#if defined (CONFIG_MFD_RT5025)
	{
		.type                   = "RT5025",
		.addr           = 0x35,
		.flags                  = 0,
	       .irq            = RT5025_HOST_IRQ,
	       .platform_data=&rt5025_data,
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

#if defined (CONFIG_RTC_HYM8563)
	{
		.type                   = "rtc_hym8563",
		.addr           = 0x51,
		.flags                  = 0,
		.irq            = RK30_PIN0_PB5,
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
#if defined (CONFIG_MFD_RK808)
	if(pmic_is_rk808())
	board_pmu_rk808_suspend();
#endif
#if defined (CONFIG_MFD_RICOH619)
	if(pmic_is_ricoh619())
	board_pmu_ricoh619_suspend(); 
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
#if defined (CONFIG_MFD_RK808)
	if(pmic_is_rk808())
	board_pmu_rk808_resume();
#endif
#if defined (CONFIG_MFD_RICOH619)
	if(pmic_is_ricoh619())
	board_pmu_ricoh619_resume(); 
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
#if defined (CONFIG_TOUCHSCREEN_GT9110_MALATA)
	{
		.type          = "Goodix9110-TS",
		.addr          = 0x14,
		.flags         = 0,
		.irq           = RK30_PIN1_PB7,
		.platform_data = &goodix9110_info,
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
#if defined(CONFIG_HDMI_CAT66121)
	{
		.type		= "cat66121_hdmi",
		.addr		= 0x4c,
		.flags		= 0,
		.irq		= RK30_PIN2_PD6,
		.platform_data 	= &rk_hdmi_pdata,
	},
#endif
#ifdef CONFIG_HAPTICS_DRV2605	
	{	
		I2C_BOARD_INFO(HAPTICS_DEVICE_NAME, 0x5a),		
		.platform_data = &drv2605_plat_data,				
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
#if defined (CONFIG_MFD_RK616)
	{
		.type	       = "rk616",
		.addr	       = 0x50,
		.flags	       = 0,
		.platform_data = &rk616_pdata,
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
#ifdef CONFIG_HALL_KEY
#if defined(CONFIG_MALATA_D1013)||defined(CONFIG_MALATA_D1025)
#define HALL_INT_PIN	RK30_PIN3_PA0
#define HALL_INT_PIN_IOMUX	MMC0_RSTNOUT
#elif defined(CONFIG_MALATA_D7005A)|| defined(CONFIG_MALATA_D1024)||(defined(CONFIG_MALATA_D7803) && defined(CONFIG_RK616_MIPI_DSI))
#define HALL_INT_PIN	RK30_PIN0_PB2
#define HALL_INT_PIN_IOMUX	INVALID_GPIO
#elif defined(CONFIG_MALATA_D1012B)
#define HALL_INT_PIN	RK30_PIN1_PA7
#define HALL_INT_PIN_IOMUX	UART1_RTSN
#else
#define HALL_INT_PIN	RK30_PIN0_PC6
#define HALL_INT_PIN_IOMUX	INVALID_GPIO
#endif

static void HALL_KEY_GPIO_INIT(void)
{
	if(HALL_INT_PIN_IOMUX != INVALID_GPIO)
		iomux_set_gpio_mode(iomux_mode_to_gpio(HALL_INT_PIN_IOMUX));
}
#endif

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
#elif defined(CONFIG_MALATA_D7005A)
				.gpio	= RK30_PIN0_PB2,
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
#ifdef CONFIG_BATTERY_RK30_ADC_FAC
			if(charge_status() == 1)
			{
				printk("with dc:enter restart system\n");
				arm_pm_restart(0, "charge");
			}
			else
			{
				printk("without dc,shutdown system\n");
				//act8846_device_shutdown();
			}
#endif
        }
	#endif
	
	#if defined(CONFIG_MFD_TPS65910)	
	if(pmic_is_tps65910())
	{
		tps65910_device_shutdown();//tps65910 shutdown
	}
	#endif
	
	 #if defined(CONFIG_MFD_RK808)        
        if(pmic_is_rk808())
       {
                rk808_device_shutdown();//rk808 shutdown
        }
        #endif
	 #if defined(CONFIG_MFD_RICOH619) 
	 if(pmic_is_ricoh619()){
	ricoh619_power_off();    //ricoh619 shutdown
	}
	#endif
	/*
#ifdef CONFIG_SND_SOC_RT5616
	if (rt5616_codec_pdata.spk_ctl_io)
		gpio_direction_output(rt5616_codec_pdata.spk_ctl_io, GPIO_LOW);
	msleep(50);
#endif
*/

	#if defined(CONFIG_MFD_RT5025) 
	if(pmic_is_rt5025()){
		if (rt5025_cable_exist())
        		arm_pm_restart(0, "charge");
		else
			rt5025_power_off(); //rt5025 shutdown
	}
	#endif

	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	while (1);
}

#if defined(CONFIG_ESP8089) || defined(CONFIG_RTL8723BS) 
#if defined(CONFIG_MALATA_D7005A) || defined(CONFIG_MALATA_E4201)
#define WIFI_PWR_EN	RK30_PIN0_PB1
#elif defined(CONFIG_MALATA_D1024)
#define WIFI_PWR_EN	RK30_PIN0_PA5
#else
#define WIFI_PWR_EN	RK30_PIN0_PD6
#endif

void wifi_power_enable(void)
{
	int ret = 0;

	if(WIFI_PWR_EN != INVALID_GPIO)
	{
		ret = gpio_request(WIFI_PWR_EN, "wifi_en");
		if (ret) {
			printk("failed to request wifi_en gpio\n");
			return;
		}

		gpio_pull_updown(WIFI_PWR_EN, 0);
		ret = gpio_direction_output(WIFI_PWR_EN, 0);
		if (ret) {
			printk("failed to set gpio wifi_en output\n");
			return;
		}
		gpio_free(WIFI_PWR_EN);
	}
}
#endif

#if defined(CONFIG_MALATA_E4201)
#define HUB_POWER_ON_PIN		RK30_PIN1_PA4
#define HUB_RESET_PIN				RK30_PIN0_PB6

static void usb_hub_power_on(void)
{
	int ret = 0;

	if(HUB_POWER_ON_PIN != INVALID_GPIO)
	{
		iomux_set(GPIO1_A4);

		ret = gpio_request(HUB_POWER_ON_PIN, "hub_power");
		if (ret) {
			printk("failed to request hub_power gpio\n");
			return;
		}

		gpio_pull_updown(HUB_POWER_ON_PIN, 0);
		ret = gpio_direction_output(HUB_POWER_ON_PIN, 0);
		if (ret) {
			printk("failed to set gpio hub_power output\n");
			return;
		}
		gpio_free(HUB_POWER_ON_PIN);
	}

	if(HUB_RESET_PIN != INVALID_GPIO)
	{
		ret = gpio_request(HUB_RESET_PIN, "hub_reset");
		if (ret) {
			printk("failed to request hub_reset gpio\n");
			return;
		}

		ret = gpio_direction_output(HUB_RESET_PIN, 1);
		if (ret) {
			printk("failed to set gpio hub_reset output\n");
			return;
		}
		gpio_free(HUB_RESET_PIN);
	}

}
#endif

#ifdef CONFIG_MSTAR_DISPLAY_CONTROLER
#define MSTAR_POWER_ON_PIN		RK30_PIN0_PD5
#define MSTAR_RESET_PIN				RK30_PIN3_PD7
static void mstar_power_on(void)
{
	int ret = 0;

	if(MSTAR_POWER_ON_PIN != INVALID_GPIO)
	{
		iomux_set(GPIO0_D5);

		ret = gpio_request(MSTAR_POWER_ON_PIN, "mstar_power");
		if (ret) {
			printk("failed to request mstar_power gpio\n");
			return;
		}

		gpio_pull_updown(MSTAR_POWER_ON_PIN, 1);
		ret = gpio_direction_output(MSTAR_POWER_ON_PIN, 1);
		if (ret) {
			printk("failed to set gpio mstar_power output\n");
			return;
		}
		gpio_free(MSTAR_POWER_ON_PIN);
	}

	if(MSTAR_RESET_PIN != INVALID_GPIO)
	{
		ret = gpio_request(MSTAR_RESET_PIN, "mstar_reset");
		if (ret) {
			printk("failed to request mstar_reset gpio\n");
			return;
		}

		gpio_direction_output(MSTAR_RESET_PIN, 1);
		msleep(100);
		gpio_direction_output(MSTAR_RESET_PIN, 0);

		gpio_free(MSTAR_RESET_PIN);
	}
}
#endif

static void __init machine_rk30_board_init(void)
{
	avs_init();
	gpio_request(POWER_ON_PIN, "poweronpin");
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
	
	pm_power_off = rk30_pm_power_off;
	
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);

#if defined(CONFIG_MALATA_E4201)
	usb_hub_power_on();
#endif

#ifdef CONFIG_MSTAR_DISPLAY_CONTROLER
	mstar_power_on();
#endif

#if defined(CONFIG_ESP8089) || defined(CONFIG_RTL8723BS) 
	wifi_power_enable();
#endif

#ifdef CONFIG_HALL_KEY
	HALL_KEY_GPIO_INIT();
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

#if defined(CONFIG_WIFI_CONTROL_FUNC)
	rk29sdk_wifi_bt_gpio_control_init();
#elif defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
    rk29sdk_wifi_combo_module_gpio_init();
#endif

#if defined(CONFIG_MT6620)
    clk_set_rate(clk_get_sys("rk_serial.0", "uart"), 48*1000000);
#endif

#if defined(CONFIG_MT5931_MT6622)
	clk_set_rate(clk_get_sys("rk_serial.0", "uart"), 24*1000000);
#elif defined(CONFIG_BK3515A_COMBO)
	clk_set_rate(clk_get_sys("rk_serial.0", "uart"), 32*1000000);
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
	int size, ion_reserve_size;
#if defined(CONFIG_ARCH_RK3188)
	/*if lcd resolution great than or equal to 1920*1200,reserve the ump memory */
	if(!(get_fb_size() < ALIGN(HD_SCREEN_SIZE,SZ_1M)))
	{
		int ump_mem_phy_size=512UL*1024UL*1024UL; 
		resource_mali[0].start = board_mem_reserve_add("ump buf", ump_mem_phy_size); 
		resource_mali[0].end = resource_mali[0].start + ump_mem_phy_size -1;
	}
#endif
#ifdef CONFIG_ION
	size = ddr_get_cap() >> 20;
	if(size >= 1024) { // DDR >= 1G, set ion to 120M
		rk30_ion_pdata.heaps[0].size = ION_RESERVE_SIZE_120M;
		ion_reserve_size = ION_RESERVE_SIZE_120M;
	}
	else {
		rk30_ion_pdata.heaps[0].size = ION_RESERVE_SIZE;
		ion_reserve_size = ION_RESERVE_SIZE;
	}
	printk("ddr size = %d M, set ion_reserve_size size to %d\n", size, ion_reserve_size);
	//rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ION_RESERVE_SIZE);
	rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ion_reserve_size);
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
/******************************** arm dvfs frequency volt table **********************************/
/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs 
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 */

#if defined(CONFIG_ARCH_RK3188)
//sdk
static struct cpufreq_frequency_table dvfs_arm_table_volt_level0[] = {
	{.frequency = 312 * 1000,       .index = 850 * 1000},
	{.frequency = 504 * 1000,       .index = 900 * 1000},
	{.frequency = 816 * 1000,       .index = 950 * 1000},
	{.frequency = 1008 * 1000,      .index = 1025 * 1000},
	{.frequency = 1200 * 1000,      .index = 1100 * 1000},
	{.frequency = 1416 * 1000,      .index = 1200 * 1000},
	{.frequency = 1608 * 1000,      .index = 1300 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
//default
static struct cpufreq_frequency_table dvfs_arm_table_volt_level1[] = {
	{.frequency = 312 * 1000,       .index = 875 * 1000},
	{.frequency = 504 * 1000,       .index = 925 * 1000},
	{.frequency = 816 * 1000,       .index = 975 * 1000},
	{.frequency = 1008 * 1000,      .index = 1075 * 1000},
	{.frequency = 1200 * 1000,      .index = 1150 * 1000},
	{.frequency = 1416 * 1000,      .index = 1250 * 1000},
	{.frequency = 1608 * 1000,      .index = 1350 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
// cube 10'
static struct cpufreq_frequency_table dvfs_arm_table_volt_level2[] = {
	{.frequency = 312 * 1000,       .index = 900 * 1000},
	{.frequency = 504 * 1000,       .index = 925 * 1000},
	{.frequency = 816 * 1000,       .index = 1000 * 1000},
	{.frequency = 1008 * 1000,      .index = 1075 * 1000},
	{.frequency = 1200 * 1000,      .index = 1200 * 1000},
	{.frequency = 1416 * 1000,      .index = 1250 * 1000},
	{.frequency = 1608 * 1000,      .index = 1350 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_arm_table_volt_malata[] = {
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

/******************************** gpu dvfs frequency volt table **********************************/
//sdk
static struct cpufreq_frequency_table dvfs_gpu_table_volt_level0[] = {	
	{.frequency = 133 * 1000,       .index = 975 * 1000},//the mininum rate is limited 133M for rk3188
	{.frequency = 200 * 1000,       .index = 975 * 1000},
	{.frequency = 266 * 1000,       .index = 1000 * 1000},
	{.frequency = 300 * 1000,       .index = 1050 * 1000},
	{.frequency = 400 * 1000,       .index = 1100 * 1000},
	{.frequency = 600 * 1000,       .index = 1200 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
//cube 10'
static struct cpufreq_frequency_table dvfs_gpu_table_volt_level1[] = {	
	{.frequency = 133 * 1000,       .index = 975 * 1000},//the mininum rate is limited 133M for rk3188
	{.frequency = 200 * 1000,       .index = 1000 * 1000},
	{.frequency = 266 * 1000,       .index = 1025 * 1000},
	{.frequency = 300 * 1000,       .index = 1050 * 1000},
	{.frequency = 400 * 1000,       .index = 1100 * 1000},
	{.frequency = 600 * 1000,       .index = 1250 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

/******************************** ddr dvfs frequency volt table **********************************/
static struct cpufreq_frequency_table dvfs_ddr_table_volt_level0[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 950 * 1000},
	{.frequency = 300 * 1000 + DDR_FREQ_VIDEO,      .index = 1000 * 1000},
	{.frequency = 396 * 1000 + DDR_FREQ_NORMAL,     .index = 1100 * 1000},
        {.frequency = 460 * 1000 + DDR_FREQ_DUALVIEW,     .index = 1150 * 1000},
	//{.frequency = 528 * 1000 + DDR_FREQ_NORMAL,     .index = 1200 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table_t[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 950 * 1000},
	{.frequency = 460 * 1000 + DDR_FREQ_NORMAL,     .index = 1150 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table_malata[] = {
#if defined(CONFIG_DDR_MIRA_P3P4GF4BLF)||defined(CONFIG_DDR_KS_D2516EC4BXGGB)||defined(CONFIG_DDR_SC_HXB15H4G160BF_15H)	\
	||defined(CONFIG_MALATA_D1014)||defined(CONFIG_MALATA_D8009)||defined(CONFIG_MALATA_D7022)

#else
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 950 * 1000},
#if !defined(CONFIG_MALATA_E4201)
	{.frequency = 300 * 1000 + DDR_FREQ_VIDEO,      .index = 1000 * 1000},
#endif
#endif

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

static struct cpufreq_frequency_table dvfs_ddr_table_t_malata[] = {
#if defined(CONFIG_DDR_MIRA_P3P4GF4BLF)||defined(CONFIG_DDR_KS_D2516EC4BXGGB)||defined(CONFIG_DDR_SC_HXB15H4G160BF_15H)
	{.frequency = 440 * 1000 + DDR_FREQ_NORMAL, 	.index = 1200 * 1000},
#else
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,	.index = 950 * 1000},
	{.frequency = 460 * 1000 + DDR_FREQ_NORMAL,     .index = 1150 * 1000},
#endif
	{.frequency = CPUFREQ_TABLE_END},
};

//if you board is good for volt quality,select dvfs_arm_table_volt_level0
#define dvfs_arm_table dvfs_arm_table_volt_malata
#define dvfs_gpu_table dvfs_gpu_table_volt_level0
#define dvfs_ddr_table dvfs_ddr_table_malata
#define dvfs_ddr_table_t dvfs_ddr_table_t_malata

#else
//for RK3168 && RK3066B
static struct cpufreq_frequency_table dvfs_arm_table[] = {
	{.frequency = 312 * 1000,       .index = 950 * 1000},
	{.frequency = 504 * 1000,       .index = 1000 * 1000},
	{.frequency = 816 * 1000,       .index = 1050 * 1000},
	{.frequency = 1008 * 1000,      .index = 1125 * 1000},
	{.frequency = 1200 * 1000,      .index = 1200 * 1000},
	//{.frequency = 1416 * 1000,      .index = 1250 * 1000},
	//{.frequency = 1608 * 1000,      .index = 1300 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_gpu_table[] = {
	{.frequency = 100 * 1000,       .index = 1000 * 1000},
	{.frequency = 200 * 1000,       .index = 1000 * 1000},
	{.frequency = 266 * 1000,       .index = 1050 * 1000},
	//{.frequency = 300 * 1000,       .index = 1050 * 1000},
	{.frequency = 400 * 1000,       .index = 1125 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 1000 * 1000},
	{.frequency = 300 * 1000 + DDR_FREQ_VIDEO,      .index = 1050 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL,     .index = 1100 * 1000},
	{.frequency = 450 * 1000,                       .index = 1150 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
#endif
/******************************** arm dvfs frequency volt table end **********************************/
//#define DVFS_CPU_TABLE_SIZE	(ARRAY_SIZE(dvfs_cpu_logic_table))
//static struct cpufreq_frequency_table cpu_dvfs_table[DVFS_CPU_TABLE_SIZE];
//static struct cpufreq_frequency_table dep_cpu2core_table[DVFS_CPU_TABLE_SIZE];
int get_max_freq(struct cpufreq_frequency_table *table)
{
	int i,temp=0;
	
	for(i=0;table[i].frequency!= CPUFREQ_TABLE_END;i++)
	{
		if(temp<table[i].frequency)
			temp=table[i].frequency;
	}	
	printk("get_max_freq=%d\n",temp);
	return temp;
}

void __init board_clock_init(void)
{
	u32 flags=RK30_CLOCKS_DEFAULT_FLAGS;
#if !defined(CONFIG_ARCH_RK3188)
	if(get_max_freq(dvfs_gpu_table)<=(400*1000))
	{	
		flags=RK30_CLOCKS_DEFAULT_FLAGS|CLK_GPU_GPLL;
	}
	else
		flags=RK30_CLOCKS_DEFAULT_FLAGS|CLK_GPU_CPLL;
#endif	
	rk30_clock_data_init(periph_pll_default, codec_pll_default, flags);
	//dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);	
	dvfs_set_freq_volt_table(clk_get(NULL, "cpu"), dvfs_arm_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);
#if defined(CONFIG_ARCH_RK3188)
	if (rk_pll_flag() == 0)
		dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
	else
		dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table_t);
#else
	dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
#endif
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
