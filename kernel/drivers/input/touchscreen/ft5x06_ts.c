/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 * VERSION      	DATE			AUTHOR
 *    1.0		  2010-01-05			WenFS
 *
 * note: only support mulititouch	Wenfs 2010-10-01
 */



#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/i2c/ft5x06_ts2.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <mach/iomux.h>
#include <mach/gpio.h>
#include <mach/irqs.h>


#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/completion.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#define IRQ_DETECT  RK30_PIN4_PC2
#define TS_INT gpio_to_irq(IRQ_DETECT)
#define CONFIG_TOUCH_I2C_SPEED    400000       /* Hz */

static struct i2c_client *this_client;
static struct ft5x0x_ts_platform_data *pdata;

//#define SCREEN_MAX_X    1280//1280
//#define SCREEN_MAX_Y    768
#define PRESS_MAX       255

static  int ft5x0x_reset_pin;
static  int ft5x0x_int_number;

#define CONFIG_FT5X0X_MULTITOUCH 1

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	x6;
	u16	y6;
	u16	x7;
	u16	y7;
	u16	x8;
	u16	y8;
	u16	x9;
	u16	y9;
	u16	x10;
	u16	y10;
	u16	pressure;
	u16     point_state[10];
	u16     point_last_state[10];
	u16     point_track_id[10]; 
    u8  touch_point;
};

struct ft5x0x_ts_data {
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
};

/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata

Input	:	*rxdata
                     *length

Output	:	ret

function	:

***********************************************************************************************/
static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
			.scl_rate = CONFIG_TOUCH_I2C_SPEED,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
			.scl_rate = CONFIG_TOUCH_I2C_SPEED,
		},
	};

    //msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{

	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
			.scl_rate = 200000,
		},
	};


	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;

}
/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:

function	:	write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    return 0;
}


/***********************************************************************************************
Name	:	ft5x0x_read_reg

Input	:	addr
                     pdata

Output	:

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2] = {0};

	buf[0] = addr;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
			.scl_rate = CONFIG_TOUCH_I2C_SPEED,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf,
			.scl_rate = CONFIG_TOUCH_I2C_SPEED,
		},
	};

    //msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
}


/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void

Output	:	 firmware version

function	:	 read TP firmware version

***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	int ret;
	ret = ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
	if (  ret < 0)
	   ver  =  0xff ;
	return(ver);
}


#define CONFIG_SUPPORT_FTS_CTP_UPG


#ifdef CONFIG_SUPPORT_FTS_CTP_UPG

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70 //no reference!


void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
    	/* For rk3066 cpu  , 1.5G freq , 855 just 1 ms*/
        for (j = 0; j < 855; j++)
        {
            udelay(1);
        }
    }
}


/*
[function]:
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;

    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]:
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]:
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;
    btPara2[in]    :parameter 2;
    btPara3[in]    :parameter 3;
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]:
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]:
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]:
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[]=
{
  #include "ft_app-ba.h"
};

static unsigned char CTPM_FW1[]=
{
  #include "ft_app-91.h"
};

static unsigned char CTPM_FW2[]=
{
  #include "ft_app-08.h"
};

static unsigned char CTPM_FW3[]=
{
  #include "ft_app-19.h"
};

static unsigned char CTPM_FW4[]=
{
  #include "ft_app-59.h"
};

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;
    
    #if  (defined(CONFIG_MALATA_D8009)||defined(CONFIG_MALATA_D7022))
    unsigned char au_delay_timings[11] = {30, 33, 36, 39, 42, 45, 27, 24,21,18,15};
    #else
    unsigned char au_delay_timings[11] = {10, 11, 12, 13, 14, 15, 9, 8, 7,6,5};
    #endif
    j = 0;
    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    // ft5x0x_write_reg(0xfc,0xaa);
    //delay_qt_ms(50);
     /*write 0x55 to register 0xfc*/
    //ft5x0x_write_reg(0xfc,0x55);
     //delay_qt_ms(50); 
     // delay_qt_ms(50); 
     
 UPGR_START:
      
    printk("[TSP] Step 1: Reset CTPM test\n");
    gpio_direction_output(ft5x0x_reset_pin, GPIO_LOW);
    delay_qt_ms(50);
    gpio_direction_output(ft5x0x_reset_pin, GPIO_HIGH);
   // delay_qt_ms(45); 
    delay_qt_ms(au_delay_timings[j]); 
     // delay_qt_ms(40);

    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/
    delay_qt_ms(100);  
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    /* if IC is ft5216 , id is 0x79 0x07 ; if ic is ft5606 ,ic is 0x79 0x06*/
    /* if IC if ft5406 , id is 0x79 0x03*/
  #if  (defined(CONFIG_MALATA_D8009)||defined(CONFIG_MALATA_D7022))
    if (reg_val[0] == 0x79 && reg_val[1] == 0x03)
  #else
    if (reg_val[0] == 0x79 && reg_val[1] == 0x06)
  #endif  
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
    	if (j < 10)
	{
	     j ++;
	     //msleep(200);
	     printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	     printk("[FTS]goto UPGR_START!\n");
	     goto UPGR_START; 
	}
	else
	{	
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    	gpio_direction_output(ft5x0x_reset_pin, GPIO_LOW);
        delay_qt_ms(50);
        gpio_direction_output(ft5x0x_reset_pin, GPIO_HIGH);
        delay_qt_ms(40); 
        return ERR_READID;
	}
        //i_is_new_protocol = 1;
    }

     /*********Step 4:erase app*******************************/
    cmd_write(0x61,0x00,0x00,0x00,1);

    delay_qt_ms(1500);
    printk("[TSP] Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[TSP] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);
        delay_qt_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i];
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);
        delay_qt_ms(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        gpio_direction_output(ft5x0x_reset_pin, GPIO_LOW);
        delay_qt_ms(50);
        gpio_direction_output(ft5x0x_reset_pin, GPIO_HIGH);
        delay_qt_ms(40);      
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    //cmd_write(0x07,0x00,0x00,0x00,1);
    gpio_direction_output(ft5x0x_reset_pin, GPIO_LOW);
    delay_qt_ms(50);
    gpio_direction_output(ft5x0x_reset_pin, GPIO_HIGH);
    /*make sure CTP startup normally */
    msleep(300);  
    return ERR_OK;
}

#define FTS_FACTORYMODE_VALUE		0x40
#define FTS_WORKMODE_VALUE		0x00

int fts_ctpm_auto_clb(void)
{
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	printk("start auto clb.\n");
	/*start auto CLB */
	msleep(200);

	ft5x0x_write_reg(0, FTS_FACTORYMODE_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	ft5x0x_write_reg(2, 0x4);
	msleep(300);
	for (i = 0; i < 100; i++) {
		ft5x0x_read_reg(0, &uc_temp);
		/*return to normal mode, calibration finish */
		if (0x0 == ((uc_temp & 0x70) >> 4))
			break;
	}

	//msleep(200);
	/*calibration OK */
	msleep(300);
	ft5x0x_write_reg(0, FTS_FACTORYMODE_VALUE);	/*goto factory mode for store */
	msleep(100);	/*make sure already enter factory mode */
	ft5x0x_write_reg(2, 0x5);	/*store CLB result */
	msleep(300);
	ft5x0x_write_reg(0, FTS_WORKMODE_VALUE);	/*return to normal mode */
	msleep(300);

	printk("end auto clb.\n");
	/*store CLB result OK */
	return 0;
}


int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;

    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
       printk("TP upgrade fail.\n");
   }
   else
   {
	fts_ctpm_auto_clb();
   }

   return i_ret;
}


int fts_ctpm_fw_upgrade_with_i_file1(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;

    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW1;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW1));
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
       printk("TP upgrade fail.\n");
   }
   else
   {
	fts_ctpm_auto_clb();
   }

   return i_ret;
}

int fts_ctpm_fw_upgrade_with_i_file2(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;

    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW2;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW2));
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
       printk("TP upgrade fail.\n");
   }
   else
   {
	fts_ctpm_auto_clb();
   }

   return i_ret;
}


int fts_ctpm_fw_upgrade_with_i_file3(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;

    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW3;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW3));
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
       printk("TP upgrade fail.\n");
   }
   else
   {
	fts_ctpm_auto_clb();
   }

   return i_ret;
}

int fts_ctpm_fw_upgrade_with_i_file4(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;

    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW4;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW4));
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
       printk("TP upgrade fail.\n");
   }
   else
   {
	fts_ctpm_auto_clb();
   }

   return i_ret;
}


unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}

#endif


/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_release(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
#ifdef CONFIG_FT5X0X_MULTITOUCH
       input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(data->input_dev);
}

static int ft5x0x_read_data(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

	u8 buf[62] = {0};
	int ret = -1;
	unsigned char threshold_value;

#ifdef CONFIG_FT5X0X_MULTITOUCH
  
        ret = ft5x0x_i2c_rxdata(buf, 7);
        if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	
	if( (buf[2] & 0x0f)>1) 
        {
		ret = ft5x0x_i2c_rxdata(buf, 61);
	}
#else
    ret = ft5x0x_i2c_rxdata(buf, 7);
#endif
    if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x0f;// 000 0111

    if (event->touch_point == 0) {
        ft5x0x_ts_release();
        return 1;
    }

#ifdef CONFIG_FT5X0X_MULTITOUCH
    switch (event->touch_point) {
		case 10:
			event->y10 =  (s16)(buf[0x39] & 0x0F)<<8 | (s16)buf[0x3a];
			event->x10 = (s16)(buf[0x3b] & 0x0F)<<8 | (s16)buf[0x3c];

			event->x10 = SCREEN_MAX_X - event->x10;
			event->point_state[9] =  buf[0x39]>>6;
			event->point_track_id[9] =  buf[0x3b]>>4;

		case 9:
			event->y9 = (s16)(buf[0x33] & 0x0F)<<8 | (s16)buf[0x34];
			event->x9 = (s16)(buf[0x35] & 0x0F)<<8 | (s16)buf[0x36];

			event->x9 = SCREEN_MAX_X - event->x9;
			event->point_state[8] =  buf[0x33]>>6;
			event->point_track_id[8] =  buf[0x35]>>4;

		case 8:
			event->y8 = (s16)(buf[0x2d] & 0x0F)<<8 | (s16)buf[0x2e];
			event->x8 = (s16)(buf[0x2f] & 0x0F)<<8 | (s16)buf[0x30];

			event->x8 = SCREEN_MAX_X - event->x8;
			event->point_state[7] =  buf[0x2d]>>6;
			event->point_track_id[7] =  buf[0x2f]>>4;
		case 7:
			event->y7 = (s16)(buf[0x27] & 0x0F)<<8 | (s16)buf[0x28];
			event->x7 = (s16)(buf[0x29] & 0x0F)<<8 | (s16)buf[0x2a];

			event->x7 = SCREEN_MAX_X - event->x7;
			event->point_state[6] =  buf[0x27]>>6;
			event->point_track_id[6] =  buf[0x29]>>4;

		case 6:
			event->y6 =(s16)(buf[0x21] & 0x0F)<<8 | (s16)buf[0x22];
			event->x6 = (s16)(buf[0x23] & 0x0F)<<8 | (s16)buf[0x24];

			event->x6 = SCREEN_MAX_X - event->x6;
			event->point_state[5] =  buf[0x21]>>6;
			event->point_track_id[5] =  buf[0x23]>>4;
		case 5:
			event->y5 =  (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			event->x5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];

			event->x5 = SCREEN_MAX_X - event->x5;
			event->point_state[4] =  buf[0x1b]>>6;
			event->point_track_id[4] =  buf[0x1d]>>4;
		case 4:
			event->y4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			event->x4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];

			event->x4 = SCREEN_MAX_X - event->x4;
			event->point_state[3] =  buf[0x15]>>6;
			event->point_track_id[3] =  buf[0x17]>>4;
		case 3:
			event->y3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			event->x3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];

			event->x3 = SCREEN_MAX_X - event->x3;
			event->point_state[2] =  buf[0x0f]>>6;
			event->point_track_id[2] =  buf[0x11]>>4;
		case 2:
			event->y2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
			event->x2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];

			event->x2 = SCREEN_MAX_X - event->x2;
			event->point_state[1] =  buf[9]>>6;
			event->point_track_id[1] =  buf[11]>>4;

		case 1:
			event->y1 =(s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
			event->x1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];

			event->x1 = SCREEN_MAX_X - event->x1;
			event->point_state[0] =  buf[3]>>6;
			event->point_track_id[0] =  buf[5]>>4;

                break;
		default:
		    return -1;
	}
#else
    if (event->touch_point == 1) {
	event->y1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
	event->x1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
    }
#endif
    event->pressure = 50;

	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);
	//printk("%d (%d, %d), (%d, %d)\n", event->touch_point, event->x1, event->y1, event->x2, event->y2);

#if  defined(CONFIG_MALATA_D7022)
     ret = ft5x0x_read_reg(FT5X0X_REG_THGROUP, &threshold_value);
     if ( ret < 0)
     {
     	  return 0;
     }
     else
     {
     	  /* DC IN status */
          if(0 == gpio_get_value(RK30_PIN0_PA7))
          {
          	/* change sensitivity , threshold value is 40 x 4 , peak value is 120*/
          	if(threshold_value != 40)
          	{
          	   ft5x0x_write_reg(FT5X0X_REG_THGROUP,40);
          	   ft5x0x_write_reg(FT5X0X_REG_THPEAK,120); 	
        	}		
          	
          }  
          else  /*Not DC in */   
          {
          	/* change sensitivity , threshold value is 22 x 4 , peak value is 70 */
                if(threshold_value != 22)
          	{
          	    ft5x0x_write_reg(FT5X0X_REG_THGROUP,22);	
          	    ft5x0x_write_reg(FT5X0X_REG_THPEAK,70); 
        	}		
          }			
     }	
#endif

    return 0;
}
/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 uVersion;

//		printk("==ft5x0x_report_value =\n");
#ifdef CONFIG_FT5X0X_MULTITOUCH
#if (defined(CONFIG_MALATA_C7022) ||defined(CONFIG_MALATA_C7018)||defined(CONFIG_MALATA_D7022) )
	switch(event->touch_point) {
		case 10:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			#if defined(CONFIG_MALATA_D7022)
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x10);
			#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_X -event->x10);
			#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y10);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[9]);
			input_mt_sync(data->input_dev);

		case 9:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			#if defined(CONFIG_MALATA_D7022)
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x9);
			#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_X -event->x9);
			#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y9);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[8]);
			input_mt_sync(data->input_dev);

		case 8:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			#if defined(CONFIG_MALATA_D7022)
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x8);
			#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_X -event->x8);
			#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y8);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[7]);
			input_mt_sync(data->input_dev);

		case 7:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			#if defined(CONFIG_MALATA_D7022)
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x7);
			#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_X -event->x7);
			#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y7);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[6]);
			input_mt_sync(data->input_dev);

		case 6:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			#if defined(CONFIG_MALATA_D7022)
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x6);
			#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_X -event->x6);
			#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y6);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[5]);
			input_mt_sync(data->input_dev);
		case 5:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			#if defined(CONFIG_MALATA_D7022)
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x5);
			#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_X -event->x5);
			#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y5);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[4]);
			input_mt_sync(data->input_dev);

		case 4:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			#if defined(CONFIG_MALATA_D7022)
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x4);
			#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_X -event->x4);
			#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y4);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[3]);
			input_mt_sync(data->input_dev);

		case 3:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			#if defined(CONFIG_MALATA_D7022)
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x3);
			#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_X -event->x3);
			#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y3);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[2]);
			input_mt_sync(data->input_dev);

		case 2:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			#if defined(CONFIG_MALATA_D7022)
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x2);
			#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_X -event->x2);
			#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y2);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[1]);
			input_mt_sync(data->input_dev);

		case 1:
		    input_report_key(data->input_dev, BTN_TOUCH, 1);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			#if defined(CONFIG_MALATA_D7022)
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->x1);
			#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_X -event->x1);
			#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->y1);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[0]);
			input_mt_sync(data->input_dev);


		default:

			break;
	}
#else
	switch(event->touch_point) {
		case 10:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x10);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y10);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[9]);
			input_mt_sync(data->input_dev);

		case 9:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x9);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y9);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[8]);
			input_mt_sync(data->input_dev);

		case 8:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x8);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y8);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[7]);
			input_mt_sync(data->input_dev);

		case 7:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x7);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y7);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[6]);
			input_mt_sync(data->input_dev);

		case 6:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x6);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y6);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[5]);
			input_mt_sync(data->input_dev);
		case 5:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x5);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y5);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[4]);
			input_mt_sync(data->input_dev);

		case 4:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x4);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y4);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[3]);
			input_mt_sync(data->input_dev);

		case 3:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x3);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y3);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[2]);
			input_mt_sync(data->input_dev);

		case 2:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[1]);
			input_mt_sync(data->input_dev);

		case 1:
		        input_report_key(data->input_dev, BTN_TOUCH, 1);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point_track_id[0]);
			input_mt_sync(data->input_dev);


		default:

			break;
	}
#endif
#else	/* CONFIG_FT5X0X_MULTITOUCH*/
	if (event->touch_point == 1) {
#if (defined(CONFIG_MALATA_C7022) ||defined(CONFIG_MALATA_C7018)||defined(CONFIG_MALATA_D7022) )
		input_report_abs(data->input_dev, ABS_Y, SCREEN_MAX_X -event->x1);
		input_report_abs(data->input_dev, ABS_X, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
#else
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
#endif
	}
	input_report_key(data->input_dev, BTN_TOUCH, 1);
#endif	/* CONFIG_FT5X0X_MULTITOUCH*/
	input_sync(data->input_dev);

	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);
}	/*end ft5x0x_report_value*/

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	ret = ft5x0x_read_data();
	if (ret == 0) {
		ft5x0x_report_value();
	}
	enable_irq(ft5x0x_int_number);
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
	disable_irq_nosync(irq);
	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ts;
	int result = 0;
	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);
	printk("==ft5x0x_ts_suspend=\n");
	gpio_request(ft5x0x_reset_pin, "NULL");
        gpio_direction_output(ft5x0x_reset_pin, 0);
        msleep(20);
        gpio_direction_output(ft5x0x_reset_pin, 1);
        msleep(200);
        /* For ft5606 , 0xa5 is power mode register , set 0x03 to the register ,ic will enter standy mode*/
	result =  ft5x0x_write_reg(0xa5,0x03);
	if( result >= 0 )
	{
		printk("Touchsreen ft5406 Send  suspend command sucessfully\n");
	}
	else
	{
		printk("Touchsreen ft5406 Send  suspend command failed\n");
	}
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	printk("==ft5x0x_ts_resume=\n");
        gpio_direction_output(ft5x0x_reset_pin, 0);
        msleep(20);
        gpio_direction_output(ft5x0x_reset_pin, 1);
        msleep(20);

}
#endif  //CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static int
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;

	printk("==ft5x0x_ts_probe=\n");

        struct ft5x0x_ts_platform_data *pdata;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	printk("==kzalloc=\n");
	ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}


        this_client = client;
	i2c_set_clientdata(client, ft5x0x_ts);

        pdata =  client->dev.platform_data;
	ft5x0x_reset_pin = pdata->reset_pin ;
        ft5x0x_int_number = gpio_to_irq(pdata->intr_number) ;
        printk("ft5x0x_reset_pin = %d ft5x0x_int_number %d\n ", ft5x0x_reset_pin, ft5x0x_int_number);

	if(pdata->init_platform_hw)
		pdata->init_platform_hw();

	gpio_request(ft5x0x_reset_pin, "NULL");
	gpio_direction_output(ft5x0x_reset_pin, 1);
	msleep(20);
	gpio_direction_output(ft5x0x_reset_pin, 0);
	msleep(20);
	gpio_direction_output(ft5x0x_reset_pin, 1);
	msleep(200);

	uc_reg_value = ft5x0x_read_fw_ver();
	printk("[FST] Firmware version = 0x%x\n", uc_reg_value);
	if( uc_reg_value == 0xff)   /* unvalid version */
	{
		#if  defined(CONFIG_MALATA_D8009)
		/* change I2C address to 0x3e*/
		client->addr = 0x38;
		uc_reg_value = ft5x0x_read_fw_ver();
	        printk("[FST] Firmware version = 0x%x\n", uc_reg_value);
	        #endif
		
		if( uc_reg_value == 0xff) 
		{
			gpio_free(ft5x0x_reset_pin);
			goto  exit_check_functionality_failed;
		}
	}

        if( uc_reg_value != 0xff )
	{
          /* compare to last version, if not equal,do update!*/  
          #if  defined(CONFIG_MALATA_D1004)
               /* for xiamen tianzhiyu TP , the firmare is 0xax to 0xbx */
 		if(( uc_reg_value < 0xba) && ( uc_reg_value > 0xa0))
		    fts_ctpm_fw_upgrade_with_i_file();

                /* for beitai TP, the firmware is 0x9x */ 
		if(( uc_reg_value < 0x91) && ( uc_reg_value >= 0x90))
			fts_ctpm_fw_upgrade_with_i_file1();    

          #endif
          #if  defined(CONFIG_MALATA_D8009)
              if(( uc_reg_value < 0x08) && ( uc_reg_value > 0x00))
              {
        	  fts_ctpm_fw_upgrade_with_i_file2();
              }	
              
              if(( uc_reg_value < 0x19) && ( uc_reg_value > 0x10))
              {
        	  fts_ctpm_fw_upgrade_with_i_file3();
              }	 
              
              
              if( uc_reg_value == 0xa6 )
              {
                   fts_ctpm_fw_upgrade_with_i_file3();	
              }	
              
          #endif
          
           #if  defined(CONFIG_MALATA_D7022)
              if(( uc_reg_value < 0x59) && ( uc_reg_value > 0x50))
              {
        	  fts_ctpm_fw_upgrade_with_i_file4();
              }	
              if( uc_reg_value == 0xa6 )
              {
                   fts_ctpm_fw_upgrade_with_i_file4();	
              }	 
              
           #endif
	    
	uc_reg_value = ft5x0x_read_fw_ver();
	printk("[FST] Firmware version = 0x%x\n", uc_reg_value);
		    
	}

	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	err = gpio_request(pdata->intr_number, "ft5x06 irq");
        if(err!= 0){
               gpio_free(pdata->intr_number);
               printk("ft5x06 gpio_request error\n");
               goto exit_create_singlethread;
       }

	err = request_irq(ft5x0x_int_number, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}


	disable_irq_nosync(ft5x0x_int_number);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_TRACKING_ID, input_dev->absbit);

	input_set_abs_params(input_dev,ABS_MT_TRACKING_ID, 0, 10, 0, 0);
#if (defined(CONFIG_MALATA_C7022) ||defined(CONFIG_MALATA_C7018)||defined(CONFIG_MALATA_D7022) )
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_X, 0, 0);

#else
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
#endif
	input_set_abs_params(input_dev,
                 ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

        set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_dev->name		= FT5X0X_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

    msleep(50);
    //get some register information

   //fts_ctpm_fw_upgrade_with_i_file();

    enable_irq(ft5x0x_int_number);

	printk("==probe over =\n");
    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(ft5x0x_int_number, ft5x0x_ts);
exit_irq_request_failed:
	 gpio_free(pdata->intr_number);
exit_platform_data_null:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	printk("==ft5x0x_ts_remove=\n");
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
//	free_irq(client->irq, ft5x0x_ts);
	free_irq(ft5x0x_int_number, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
};

/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static int __init ft5x0x_ts_init(void)
{
	int ret;
	printk("==ft5x0x_ts_init==\n");
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	printk("ret=%d\n",ret);
	return ret;
//	return i2c_add_driver(&ft5x0x_ts_driver);
}

/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static void __exit ft5x0x_ts_exit(void)
{
	printk("==ft5x0x_ts_exit==\n");
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");

