#ifdef CONFIG_VIDEO_RK29
#include <plat/rk_camera.h>
/* Notes:

Simple camera device registration:

       new_camera_device(sensor_name,\       // sensor name, it is equal to CONFIG_SENSOR_X
                          face,\              // sensor face information, it can be back or front
                          pwdn_io,\           // power down gpio configuration, it is equal to CONFIG_SENSOR_POWERDN_PIN_XX
                          flash_attach,\      // sensor is attach flash or not
                          mir,\               // sensor image mirror and flip control information
                          i2c_chl,\           // i2c channel which the sensor attached in hardware, it is equal to CONFIG_SENSOR_IIC_ADAPTER_ID_X
                          cif_chl)  \         // cif channel which the sensor attached in hardware, it is equal to CONFIG_SENSOR_CIF_INDEX_X

Comprehensive camera device registration:

      new_camera_device_ex(sensor_name,\
                             face,\
                             ori,\            // sensor orientation, it is equal to CONFIG_SENSOR_ORIENTATION_X
                             pwr_io,\         // sensor power gpio configuration, it is equal to CONFIG_SENSOR_POWER_PIN_XX
                             pwr_active,\     // sensor power active level, is equal to CONFIG_SENSOR_RESETACTIVE_LEVEL_X
                             rst_io,\         // sensor reset gpio configuration, it is equal to CONFIG_SENSOR_RESET_PIN_XX
                             rst_active,\     // sensor reset active level, is equal to CONFIG_SENSOR_RESETACTIVE_LEVEL_X
                             pwdn_io,\
                             pwdn_active,\    // sensor power down active level, is equal to CONFIG_SENSOR_POWERDNACTIVE_LEVEL_X
                             flash_attach,\
                             res,\            // sensor resolution, this is real resolution or resoltuion after interpolate
                             mir,\
                             i2c_chl,\
                             i2c_spd,\        // i2c speed , 100000 = 100KHz
                             i2c_addr,\       // the i2c slave device address for sensor
                             cif_chl,\
                             mclk)\           // sensor input clock rate, 24 or 48
                          
*/
#if defined(CONFIG_MALATA_D7803) || defined(CONFIG_MALATA_D7005)|| defined(CONFIG_MALATA_D7005A)
#define ori_back_camera 180
#define ori_front_camera 180
#elif defined(CONFIG_MALATA_D7022)
#define ori_back_camera 0
#define ori_front_camera 0
#elif defined(CONFIG_MALATA_D8006)
#define ori_back_camera 270
#define ori_front_camera 90
#else
#define ori_back_camera 90
#define ori_front_camera 270
#endif

static struct rkcamera_platform_data new_camera[] = { 
    new_camera_device(RK29_CAM_SENSOR_GC2035,
                        back,
                        RK30_PIN3_PB5,
                        0,
                        0,
                        3,
                        0,
                        ori_back_camera),
                            new_camera_device(RK29_CAM_SENSOR_GT2005,
                        back,
                        RK30_PIN3_PB5,
                        0,
                        0,
                        3,
                        0,
                        ori_back_camera),
                          new_camera_device(RK29_CAM_SENSOR_GC2155,
                        back,
                        RK30_PIN3_PB5,
                        0,
                        0,
                        3,
                        0,
                        ori_back_camera),
                            new_camera_device(RK29_CAM_SENSOR_OV5640,
                        back,
                        RK30_PIN3_PB5,
                        0,
                        0,
                        3,
                        0,
                        ori_back_camera),
                          new_camera_device(RK29_CAM_SENSOR_SID130B_BACK,
                        back,
                        RK30_PIN3_PB5,
                        0,
                        0,
                        3,
                        0,
                        ori_back_camera),
                           new_camera_device(RK29_CAM_SENSOR_OV2655,
                        back,
                        RK30_PIN3_PB5,
                        0,
                        0,
                        3,
                        0,
                        ori_back_camera),
                         new_camera_device(RK29_CAM_SENSOR_GC2035_FRONT,
                        front,
                        RK30_PIN3_PB4,
                        0,
                        0,
                        3,
                        0,
                        ori_front_camera),
                        new_camera_device(RK29_CAM_SENSOR_GC0308,
                        front,
                        RK30_PIN3_PB4,
                        0,
                        0,
                        3,
                        0,
                        ori_front_camera),
                        new_camera_device(RK29_CAM_SENSOR_HI253_FRONT,
                        front,
                        RK30_PIN3_PB4,
                        0,
                        0,
                        3,
                        0,
                        ori_front_camera),
                        new_camera_device(RK29_CAM_SENSOR_GT2005_FRONT,
                        front,
                        RK30_PIN3_PB4,
                        0,
                        0,
                        3,
                        0,
                        ori_front_camera),
                        new_camera_device(RK29_CAM_SENSOR_SID130B,
                        front,
                        RK30_PIN3_PB4,
                        0,
                        0,
                        3,
                        0,
                        ori_front_camera),
                        new_camera_device(RK29_CAM_SENSOR_GC0329,
                        front,
                        RK30_PIN3_PB4,
                        0,
                        0,
                        3,
                        0,
                        ori_front_camera),
                        new_camera_device(RK29_CAM_SENSOR_OV2659,
                        front,
                        RK30_PIN3_PB4,
                        0,
                        0,
                        3,
                        0,
                        ori_front_camera),
                        new_camera_device(RK29_CAM_SENSOR_SP2518,
                        front,
                        RK30_PIN3_PB4,
                        0,
                        0,
                        3,
                        0,
                        ori_front_camera), 
                           
    new_camera_device_end  
};
#endif  //#ifdef CONFIG_VIDEO_RK29
/*---------------- Camera Sensor Configuration Macro End------------------------*/
#include "../../../drivers/media/video/rk30_camera.c"
/*---------------- Camera Sensor Macro Define End  ---------*/

#define PMEM_CAM_SIZE PMEM_CAM_NECESSARY
/*****************************************************************************************
 * camera  devices
 * author: ddl@rock-chips.com
 *****************************************************************************************/
#ifdef CONFIG_VIDEO_RK29
#define CONFIG_SENSOR_POWER_IOCTL_USR	   1 //define this refer to your board layout
#define CONFIG_SENSOR_RESET_IOCTL_USR	   0
#define CONFIG_SENSOR_POWERDOWN_IOCTL_USR	   0
#define CONFIG_SENSOR_FLASH_IOCTL_USR	   0
#define CONFIG_SENSOR_AF_IOCTL_USR	   0
#if defined(CONFIG_MALATA_D7005A)||(defined(CONFIG_MALATA_D7803)&&defined(CONFIG_MFD_RT5025))
#if defined(CONFIG_RK616_MIPI_DSI)
#define CAMERA_POWER_UP_PIN		RK30_PIN0_PA1
#else
#define CAMERA_POWER_UP_PIN		RK30_PIN0_PB4
#endif
#elif defined(CONFIG_MALATA_E4201)
#define CAMERA_POWER_UP_PIN		INVALID_GPIO
#endif

static void rk_cif_all_power(int on)
{
#if defined(CONFIG_MALATA_D7005A)||(defined(CONFIG_MALATA_D7803)&&defined(CONFIG_MFD_RT5025))||defined(CONFIG_MALATA_E4201)
	int ret = 0;
	if(CAMERA_POWER_UP_PIN != INVALID_GPIO)
	{
		ret = gpio_request(CAMERA_POWER_UP_PIN, "camera_power_up");
		if(ret < 0)
		{
			printk("%s:failed to request gpio camera_power_up\n", __func__);
			return;
		}
	}
#else
	struct regulator *ldo_18,*ldo_28;
		ldo_28 = regulator_get(NULL, "act_ldo8");	// vcc28_cif
	ldo_18 = regulator_get(NULL, "act_ldo3");	// vcc18_cif
	if (ldo_28 == NULL || IS_ERR(ldo_28) || ldo_18 == NULL || IS_ERR(ldo_18)){
		printk("get cif ldo failed!\n");
		return;
		}
#endif
	if(on == 0){
#if defined(CONFIG_MALATA_D7005A)||(defined(CONFIG_MALATA_D7803)&&defined(CONFIG_MFD_RT5025))||defined(CONFIG_MALATA_E4201)
		if(CAMERA_POWER_UP_PIN != INVALID_GPIO)
		{
			gpio_direction_output(CAMERA_POWER_UP_PIN, 0);
			gpio_free(CAMERA_POWER_UP_PIN);
		}
#else
		while(regulator_is_enabled(ldo_28)>0)	
			regulator_disable(ldo_28);
		regulator_put(ldo_28);
		while(regulator_is_enabled(ldo_18)>0)
			regulator_disable(ldo_18);
		regulator_put(ldo_18);
#endif
		mdelay(10);
		}
	else{
#if defined(CONFIG_MALATA_D7005A)||(defined(CONFIG_MALATA_D7803)&&defined(CONFIG_MFD_RT5025))||defined(CONFIG_MALATA_E4201)
		if(CAMERA_POWER_UP_PIN != INVALID_GPIO)
		{
			gpio_direction_output(CAMERA_POWER_UP_PIN, 1);
			gpio_free(CAMERA_POWER_UP_PIN);
		}
#else
		regulator_set_voltage(ldo_28, 2800000, 2800000);
		regulator_enable(ldo_28);
   //	printk("%s set ldo7 vcc28_cif=%dmV end\n", __func__, regulator_get_voltage(ldo_28));
		regulator_put(ldo_28);

		regulator_set_voltage(ldo_18, 1800000, 1800000);
	//	regulator_set_suspend_voltage(ldo, 1800000);
		regulator_enable(ldo_18);
	//	printk("%s set ldo1 vcc18_cif=%dmV end\n", __func__, regulator_get_voltage(ldo_18));
		regulator_put(ldo_18);
#endif
	}
}

static void rk_cif_power(struct rk29camera_gpio_res *res,int on)
{
	struct regulator *ldo_18,*ldo_28;
	int camera_power = res->gpio_power;
	  int camera_ioflag = res->gpio_flag;
	  int camera_io_init = res->gpio_init;

#if defined(CONFIG_MALATA_D7005A)||(defined(CONFIG_MALATA_D7803)&&defined(CONFIG_MFD_RT5025))||defined(CONFIG_MALATA_E4201)
	int ret = 0;

	if(CAMERA_POWER_UP_PIN != INVALID_GPIO)
	{
		ret = gpio_request(CAMERA_POWER_UP_PIN, "camera_power_up");
		if(ret < 0)
		{
			printk("%s:failed to request gpio camera_power_up\n", __func__);
			return;
		}
	}
#else
	//ldo_28 = regulator_get(NULL, "ldo7");	// vcc28_cif
	//ldo_18 = regulator_get(NULL, "ldo1");	// vcc18_cif
		ldo_28 = regulator_get(NULL, "act_ldo8");	// vcc28_cif
	ldo_18 = regulator_get(NULL, "act_ldo3");	// vcc18_cif
	if (ldo_28 == NULL || IS_ERR(ldo_28) || ldo_18 == NULL || IS_ERR(ldo_18)){
		printk("get cif ldo failed!\n");
		return;
		}
#endif
	if(on == 0){
#if defined(CONFIG_MALATA_D7005A)||(defined(CONFIG_MALATA_D7803)&&defined(CONFIG_MFD_RT5025))||defined(CONFIG_MALATA_E4201)
		if(CAMERA_POWER_UP_PIN != INVALID_GPIO)
		{
			gpio_direction_output(CAMERA_POWER_UP_PIN, 0);
			gpio_free(CAMERA_POWER_UP_PIN);
		}
#else
		while(regulator_is_enabled(ldo_28)>0)	
			regulator_disable(ldo_28);
		regulator_put(ldo_28);
		while(regulator_is_enabled(ldo_18)>0)
			regulator_disable(ldo_18);
		regulator_put(ldo_18);
#endif
		mdelay(10);
	if (camera_power != INVALID_GPIO)  {
		  if (camera_io_init & RK29_CAM_POWERACTIVE_MASK) {
			  gpio_set_value(camera_power, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
			//	dprintk("%s..%s..PowerPin=%d ..PinLevel = %x	 \n",__FUNCTION__,res->dev_name, camera_power, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
			}
		}
		}
	else{
#if defined(CONFIG_MALATA_D7005A)||(defined(CONFIG_MALATA_D7803)&&defined(CONFIG_MFD_RT5025))||defined(CONFIG_MALATA_E4201)
		if(CAMERA_POWER_UP_PIN != INVALID_GPIO)
		{
			gpio_direction_output(CAMERA_POWER_UP_PIN, 1);
			gpio_free(CAMERA_POWER_UP_PIN);
		}
#else
		regulator_set_voltage(ldo_28, 2800000, 2800000);
		regulator_enable(ldo_28);
   //	printk("%s set ldo7 vcc28_cif=%dmV end\n", __func__, regulator_get_voltage(ldo_28));
		regulator_put(ldo_28);

		regulator_set_voltage(ldo_18, 1800000, 1800000);
	//	regulator_set_suspend_voltage(ldo, 1800000);
		regulator_enable(ldo_18);
	//	printk("%s set ldo1 vcc18_cif=%dmV end\n", __func__, regulator_get_voltage(ldo_18));
		regulator_put(ldo_18);
#endif
	if (camera_power != INVALID_GPIO)  {
		  if (camera_io_init & RK29_CAM_POWERACTIVE_MASK) {
			gpio_set_value(camera_power, ((camera_ioflag&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
			//dprintk("%s..%s..PowerPin=%d ..PinLevel = %x	 \n",__FUNCTION__,res->dev_name, camera_power, ((camera_ioflag&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
			mdelay(10);
			}
	}

	}
}

#if CONFIG_SENSOR_POWER_IOCTL_USR
static int sensor_power_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	//#error "CONFIG_SENSOR_POWER_IOCTL_USR is 1, sensor_power_usr_cb function must be writed!!";
	rk_cif_power(res,on);
	return 0;
}
#endif

#if CONFIG_SENSOR_FLASH_IOCTL_USR
static int sensor_flash_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	#error "CONFIG_SENSOR_FLASH_IOCTL_USR is 1, sensor_flash_usr_cb function must be writed!!";
}
#endif

#if CONFIG_SENSOR_AF_IOCTL_USR
static int sensor_af_usr_cb (struct rk29camera_gpio_res *res,int on)
{
	#error "CONFIG_SENSOR_AF_IOCTL_USR is 1, sensor_af_usr_cb function must be writed!!";
}
#endif


static struct rk29camera_platform_ioctl_cb	sensor_ioctl_cb = {
	#if CONFIG_SENSOR_POWER_IOCTL_USR
	.sensor_power_cb = sensor_power_usr_cb,
	#else
	.sensor_power_cb = NULL,
	#endif

	#if CONFIG_SENSOR_RESET_IOCTL_USR
	.sensor_reset_cb = sensor_reset_usr_cb,
	#else
	.sensor_reset_cb = NULL,
	#endif

	#if CONFIG_SENSOR_POWERDOWN_IOCTL_USR
	.sensor_powerdown_cb = sensor_powerdown_usr_cb,
	#else
	.sensor_powerdown_cb = NULL,
	#endif

	#if CONFIG_SENSOR_FLASH_IOCTL_USR
	.sensor_flash_cb = sensor_flash_usr_cb,
	#else
	.sensor_flash_cb = NULL,
	#endif

	#if CONFIG_SENSOR_AF_IOCTL_USR
	.sensor_af_cb = sensor_af_usr_cb,
	#else
	.sensor_af_cb = NULL,
	#endif
};

#if CONFIG_SENSOR_IIC_ADDR_0
static struct reginfo_t rk_init_data_sensor_reg_0[] =
{
		{0x0000, 0x00,0,0}
	};
static struct reginfo_t rk_init_data_sensor_winseqreg_0[] ={
	{0x0000, 0x00,0,0}
	};
#endif

#if CONFIG_SENSOR_IIC_ADDR_1
static struct reginfo_t rk_init_data_sensor_reg_1[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_1[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_01
static struct reginfo_t rk_init_data_sensor_reg_01[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_01[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_02
static struct reginfo_t rk_init_data_sensor_reg_02[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_02[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_11
static struct reginfo_t rk_init_data_sensor_reg_11[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_11[] =
{
       {0x0000, 0x00,0,0}
};
#endif
#if CONFIG_SENSOR_IIC_ADDR_12
static struct reginfo_t rk_init_data_sensor_reg_12[] =
{
    {0x0000, 0x00,0,0}
};
static struct reginfo_t rk_init_data_sensor_winseqreg_12[] =
{
       {0x0000, 0x00,0,0}
};
#endif
static rk_sensor_user_init_data_s rk_init_data_sensor[RK_CAM_NUM] = 
{
    #if CONFIG_SENSOR_IIC_ADDR_0
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_0,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_0,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_0) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_0) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_1
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_1,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_1,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_1) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_1) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_01
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_01,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_01,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_01) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_01) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_02
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_02,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_02,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_02) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_02) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_11
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_11,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_11,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_11) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_11) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif
    #if CONFIG_SENSOR_IIC_ADDR_12
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = rk_init_data_sensor_reg_12,
       .rk_sensor_init_winseq = rk_init_data_sensor_winseqreg_12,
       .rk_sensor_winseq_size = sizeof(rk_init_data_sensor_winseqreg_12) / sizeof(struct reginfo_t),
       .rk_sensor_init_data_size = sizeof(rk_init_data_sensor_reg_12) / sizeof(struct reginfo_t),
    },
    #else
    {
       .rk_sensor_init_width = INVALID_VALUE,
       .rk_sensor_init_height = INVALID_VALUE,
       .rk_sensor_init_bus_param = INVALID_VALUE,
       .rk_sensor_init_pixelcode = INVALID_VALUE,
       .rk_sensor_init_data = NULL,
       .rk_sensor_init_winseq = NULL,
       .rk_sensor_winseq_size = 0,
       .rk_sensor_init_data_size = 0,
    },
    #endif

 };
#include "../../../drivers/media/video/rk30_camera.c"

#endif /* CONFIG_VIDEO_RK29 */
