/* drivers/input/touchscreen/goodix_tool.c
 *
 * 2010 - 2012 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version:1.2
 *        V1.0:2012/05/01,create file.
 *        V1.2:2012/06/08,modify some warning.
 *
 */

#include <linux/gt811.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/delay.h>
//#include <linux/string.h>
//#include <linux/completion.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>

//#define IC_TYPE_NAME        "GT813" //Default
#define DATA_LENGTH_UINT    512
#define CMD_HEAD_LENGTH     (sizeof(st_cmd_head) - sizeof(u8*))
#define GOODIX_ENTRY_NAME   "goodix_tool"

#define GTP_ADDR_LENGTH       2
#define FAIL    0
#define SUCCESS  1
//#define UPDATE_FUNCTIONS

#ifdef UPDATE_FUNCTIONS
extern s32 gup_enter_update_mode(struct i2c_client *client);
extern void gup_leave_update_mode(void);
extern s32 gup_update_proc(void *dir);
#endif

//extern void gt811_irq_disable(struct goodix_ts_data *);
//extern void gt811_irq_enable(struct goodix_ts_data *);





#define GTP_DRIVER_VERSION    "V1.2<2012/06/08>"
#define GTP_DEBUG_ARRAY_ON    0

#define GTP_DEBUG_ARRAY(array, num)    do{\
                                         s32 i;\
                                         u8* a = array;\
                                         if(GTP_DEBUG_ARRAY_ON)\
                                         {\
                                            printk("<<-GTP-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printk("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printk("\n");\
                                                }\
                                            }\
                                            printk("\n");\
                                        }\
                                       }while(0)

#pragma pack(1)
typedef struct{
    u8  wr;         //write read flag, 0:R  1:W  2:PID 3:
    u8  flag;       //0:no need flag/int 1: need flag  2:need int
    u8 flag_addr[2];  //flag address
    u8  flag_val;   //flag val
    u8  flag_relation;  //flag_val:flag 0:not equal 1:equal 2:> 3:<
    u16 circle;     //polling cycle
    u8  times;      //plling times
    u8  retry;      //I2C retry times
    u16 delay;      //delay befor read or after write
    u16 data_len;   //data length
    u8  addr_len;   //address length
    u8  addr[2];    //address
    u8  res[3];     //reserved
    u8* data;       //data pointer
}st_cmd_head;
#pragma pack()
st_cmd_head cmd_head;

static struct i2c_client *gt_client = NULL;

static struct proc_dir_entry *goodix_proc_entry;

static s32 goodix_tool_write(struct file *filp, const char __user *buff, unsigned long len, void *data);
static s32 goodix_tool_read( char *page, char **start, off_t off, int count, int *eof, void *data );
static s32 (*tool_i2c_read)(u8 *, u16);
static s32 (*tool_i2c_write)(u8 *, u16);

s32 DATA_LENGTH = 0;
s8 IC_TYPE[16] = {0};



static void gt_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable)
	{
		enable_irq(ts->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

static void gt_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable)
	{
		disable_irq_nosync(ts->irq);
		ts->irq_is_disable = 1;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}



static s32 tool_i2c_read_no_extra(u8* buf, u16 len)
{
    s32 ret = -1;
    s32 i = 0;
    struct i2c_msg msgs[2];

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = gt_client->addr;
    msgs[0].len   = cmd_head.addr_len;
    msgs[0].buf   = &buf[0];

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = gt_client->addr;
    msgs[1].len   = len;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];

    for (i = 0; i < cmd_head.retry; i++)
    {
        ret=i2c_transfer(gt_client->adapter, msgs, 2);
        if (ret > 0)
        {
            break;
        }
    }
    return ret;
}

static s32 tool_i2c_write_no_extra(u8* buf, u16 len)
{
    s32 ret = -1;
    s32 i = 0;
    struct i2c_msg msg;

    msg.flags = !I2C_M_RD;
    msg.addr  = gt_client->addr;
    msg.len   = len;
    msg.buf   = buf;

    for (i = 0; i < cmd_head.retry; i++)
    {
        ret=i2c_transfer(gt_client->adapter, &msg, 1);
        if (ret > 0)
        {
            break;
        }
    }
    return ret;
}

static s32 tool_i2c_read_with_extra(u8* buf, u16 len)
{
    s32 ret = -1;
    u8 pre[2] = {0x0f, 0xff};
    u8 end[2] = {0x80, 0x00};

    tool_i2c_write_no_extra(pre, 2);
    ret = tool_i2c_read_no_extra(buf, len);
    tool_i2c_write_no_extra(end, 2);

    return ret;
}

static s32 tool_i2c_write_with_extra(u8* buf, u16 len)
{
    s32 ret = -1;
    u8 pre[2] = {0x0f, 0xff};
    u8 end[2] = {0x80, 0x00};

    tool_i2c_write_no_extra(pre, 2);
    ret = tool_i2c_write_no_extra(buf, len);
    tool_i2c_write_no_extra(end, 2);

    return ret;
}

static void register_i2c_func(void)
{
//    if (!strncmp(IC_TYPE, "GT818", 5) || !strncmp(IC_TYPE, "GT816", 5)
//        || !strncmp(IC_TYPE, "GT811", 5) || !strncmp(IC_TYPE, "GT818F", 6)
//        || !strncmp(IC_TYPE, "GT827", 5) || !strncmp(IC_TYPE,"GT828", 5)
//        || !strncmp(IC_TYPE, "GT813", 5))
    if (strncmp(IC_TYPE, "GT8110", 6) && strncmp(IC_TYPE, "GT8105", 6)
        && strncmp(IC_TYPE, "GT801", 5) && strncmp(IC_TYPE, "GT800", 5)
        && strncmp(IC_TYPE, "GT801PLUS", 9) && strncmp(IC_TYPE, "GT811", 5)
        && strncmp(IC_TYPE, "GTxxx", 5))
    {
        tool_i2c_read = tool_i2c_read_with_extra;
        tool_i2c_write = tool_i2c_write_with_extra;
        //printk("I2C function: with pre and end cmd!");
    }
    else
    {
        tool_i2c_read = tool_i2c_read_no_extra;
        tool_i2c_write = tool_i2c_write_no_extra;
        //printk("I2C function: without pre and end cmd!");
    }
}

static void unregister_i2c_func(void)
{
    tool_i2c_read = NULL;
    tool_i2c_write = NULL;
    //printk("I2C function: unregister i2c transfer function!");
}


s32 init_wr_node(struct i2c_client *client)
{
    s32 i;

    gt_client = client;
    memset(&cmd_head, 0, sizeof(cmd_head));
    cmd_head.data = NULL;

    i = 5;
    while ((!cmd_head.data) && i)
    {
        cmd_head.data = kzalloc(i * DATA_LENGTH_UINT, GFP_KERNEL);
        if (NULL != cmd_head.data)
        {
            break;
        }
        i--;
    }
    if (i)
    {
        DATA_LENGTH = i * DATA_LENGTH_UINT + GTP_ADDR_LENGTH;
        //printk("Applied memory size:%d.", DATA_LENGTH);
    }
    else
    {
        //printk("Apply for memory failed.");
        return FAIL;
    }

    cmd_head.addr_len = 2;
    cmd_head.retry = 5;

    register_i2c_func();

    goodix_proc_entry = create_proc_entry(GOODIX_ENTRY_NAME, 0666, NULL);
    if (goodix_proc_entry == NULL)
    {
        //printk("Couldn't create proc entry!");
        return FAIL;
    }
    else
    {
        //printk("Create proc entry success!");
        goodix_proc_entry->write_proc = goodix_tool_write;
        goodix_proc_entry->read_proc = goodix_tool_read;
    }

    return SUCCESS;
}

void uninit_wr_node(void)
{
    kfree(cmd_head.data);
    cmd_head.data = NULL;
    unregister_i2c_func();
    remove_proc_entry(GOODIX_ENTRY_NAME, NULL);
}

static u8 relation(u8 src, u8 dst, u8 rlt)
{
    u8 ret = 0;

    switch (rlt)
    {
    case 0:
        ret = (src != dst) ? true : false;
        break;

    case 1:
        ret = (src == dst) ? true : false;
        //printk("equal:src:0x%02x   dst:0x%02x   ret:%d\n", src, dst, (s32)ret);
        break;

    case 2:
        ret = (src > dst) ? true : false;
        break;

    case 3:
        ret = (src < dst) ? true : false;
        break;

    case 4:
        ret = (src & dst) ? true : false;
        break;

    case 5:
        ret = (!(src | dst)) ? true : false;
        break;

    default:
        ret = false;
        break;
    }

    return ret;
}

/*******************************************************
Function:
	Comfirm function.
Input:
  None.
Output:
	Return write length.
********************************************************/
static u8 comfirm(void)
{
    s32 i = 0;
    u8 buf[32];

//    memcpy(&buf[GTP_ADDR_LENGTH - cmd_head.addr_len], &cmd_head.flag_addr, cmd_head.addr_len);
//    memcpy(buf, &cmd_head.flag_addr, cmd_head.addr_len);//Modified by Scott, 2012-02-17
    memcpy(buf, cmd_head.flag_addr, cmd_head.addr_len);

    for (i = 0; i < cmd_head.times; i++)
    {
        if (tool_i2c_read(buf, 1) <= 0)
        {
            //printk("Read flag data failed!");
            return FAIL;
        }
        if (true == relation(buf[GTP_ADDR_LENGTH], cmd_head.flag_val, cmd_head.flag_relation))
        {
            //printk("value at flag addr:0x%02x\n", buf[GTP_ADDR_LENGTH]);
            //printk("flag value:0x%02x\n", cmd_head.flag_val);
            break;
        }

        msleep(cmd_head.circle);
    }

    if (i >= cmd_head.times)
    {
        //printk("Didn't get the flag to continue!");
        return FAIL;
    }

    return SUCCESS;
}

/*******************************************************
Function:
	Goodix tool write function.
Input:
  standard proc write function param.
Output:
	Return write length.
********************************************************/
static s32 goodix_tool_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
    u64 ret = 0;
    ////printk();
    GTP_DEBUG_ARRAY((u8*)buff, len);

    ret = copy_from_user(&cmd_head, buff, CMD_HEAD_LENGTH);
    if(!ret)
    {
        //printk("copy_from_user failed.");
    }

    //printk("wr  :0x%02x\n", cmd_head.wr);
    //printk("flag:0x%02x\n", cmd_head.flag);
    //printk("flag addr:0x%02x%02x\n", cmd_head.flag_addr[0], cmd_head.flag_addr[1]);
    //printk("flag val:0x%02x\n", cmd_head.flag_val);
    //printk("flag rel:0x%02x\n", cmd_head.flag_relation);
    //printk("circle  :%d\n", (s32)cmd_head.circle);
    //printk("times   :%d\n", (s32)cmd_head.times);
    //printk("retry   :%d\n", (s32)cmd_head.retry);
    //printk("delay   :%d\n", (s32)cmd_head.delay);
    //printk("data len:%d\n", (s32)cmd_head.data_len);
    //printk("addr len:%d\n", (s32)cmd_head.addr_len);
    //printk("addr:0x%02x%02x\n", cmd_head.addr[0], cmd_head.addr[1]);
    //printk("len:%d\n", (s32)len);
    //printk("buf[20]:0x%02x\n", buff[CMD_HEAD_LENGTH]);

    if (1 == cmd_head.wr)
    {
      //  copy_from_user(&cmd_head.data[cmd_head.addr_len], &buff[CMD_HEAD_LENGTH], cmd_head.data_len);
        ret = copy_from_user(&cmd_head.data[GTP_ADDR_LENGTH], &buff[CMD_HEAD_LENGTH], cmd_head.data_len);
        if(!ret)
        {
            //printk("copy_from_user failed.");
        }
        memcpy(&cmd_head.data[GTP_ADDR_LENGTH - cmd_head.addr_len], cmd_head.addr, cmd_head.addr_len);

        GTP_DEBUG_ARRAY(cmd_head.data, cmd_head.data_len + cmd_head.addr_len);
        GTP_DEBUG_ARRAY((u8*)&buff[CMD_HEAD_LENGTH], cmd_head.data_len);

        if (1 == cmd_head.flag)
        {
            if (FAIL == comfirm())
            {
                //printk("[WRITE]Comfirm fail!");
                return FAIL;
            }
        }
        else if (2 == cmd_head.flag)
        {
            //Need interrupt!
        }
        if (tool_i2c_write(&cmd_head.data[GTP_ADDR_LENGTH - cmd_head.addr_len],
            cmd_head.data_len + cmd_head.addr_len) <= 0)
        {
            //printk("[WRITE]Write data failed!");
            return FAIL;
        }

        GTP_DEBUG_ARRAY(&cmd_head.data[GTP_ADDR_LENGTH - cmd_head.addr_len],cmd_head.data_len + cmd_head.addr_len);
        if (cmd_head.delay)
        {
            msleep(cmd_head.delay);
        }

        return cmd_head.data_len + CMD_HEAD_LENGTH;
    }
    else if (3 == cmd_head.wr)  //Write ic type
    {
        memcpy(IC_TYPE, cmd_head.data, cmd_head.data_len);
        register_i2c_func();

        return cmd_head.data_len + CMD_HEAD_LENGTH;
    }
    else if (5 == cmd_head.wr)
    {
        //memcpy(IC_TYPE, cmd_head.data, cmd_head.data_len);

        return cmd_head.data_len + CMD_HEAD_LENGTH;
    }
    else if (7 == cmd_head.wr)//disable irq!
    {
         gt_irq_disable(i2c_get_clientdata(gt_client));

        return CMD_HEAD_LENGTH;
    }
    else if (9 == cmd_head.wr) //enable irq!
    {
        gt_irq_enable(i2c_get_clientdata(gt_client));

        return CMD_HEAD_LENGTH;
    }
#ifdef UPDATE_FUNCTIONS
    else if (11 == cmd_head.wr)//Enter update mode!
    {
        if (FAIL == gup_enter_update_mode(gt_client))
        {
            return FAIL;
        }
    }
    else if (13 == cmd_head.wr)//Leave update mode!
    {
        gup_leave_update_mode();
    }
    else if (15 == cmd_head.wr) //Update firmware!
    {
        memset(cmd_head.data, 0, cmd_head.data_len + 1);
        memcpy(cmd_head.data, &buff[CMD_HEAD_LENGTH], cmd_head.data_len);

        if (FAIL == gup_update_proc((void*)cmd_head.data))
        {
            return FAIL;
        }
    }
#endif

    return CMD_HEAD_LENGTH;
}

/*******************************************************
Function:
	Goodix tool read function.
Input:
  standard proc read function param.
Output:
	Return read length.
********************************************************/
static s32 goodix_tool_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
   // //printk();

    if (cmd_head.wr % 2)
    {
        return FAIL;
    }
    else if (!cmd_head.wr)
    {
        u16 len = 0;
        s16 data_len = 0;
        u16 loc = 0;

        if (1 == cmd_head.flag)
        {
            if (FAIL == comfirm())
            {
                //printk("[READ]Comfirm fail!");
                return FAIL;
            }
        }
        else if (2 == cmd_head.flag)
        {
            //Need interrupt!
        }

        memcpy(cmd_head.data, cmd_head.addr, cmd_head.addr_len);

        //printk("[CMD HEAD DATA] ADDR:0x%02x%02x\n", cmd_head.data[0], cmd_head.data[1]);
        //printk("[CMD HEAD ADDR] ADDR:0x%02x%02x\n", cmd_head.addr[0], cmd_head.addr[1]);

        if (cmd_head.delay)
        {
            msleep(cmd_head.delay);
        }

        data_len = cmd_head.data_len;
        while(data_len > 0)
        {
            if (data_len > DATA_LENGTH)
            {
                len = DATA_LENGTH;
            }
            else
            {
                len = data_len;
            }
            data_len -= DATA_LENGTH;

            if (tool_i2c_read(cmd_head.data, len) <= 0)
            {
                //printk("[READ]Read data failed!");
                return FAIL;
            }
            memcpy(&page[loc], &cmd_head.data[GTP_ADDR_LENGTH], len);
            loc += len;

            //GTP_DEBUG_ARRAY(&cmd_head.data[GTP_ADDR_LENGTH], len);
            GTP_DEBUG_ARRAY(page, len);
        }
    }
    else if (2 == cmd_head.wr)
    {
    //    memcpy(page, "gt8", cmd_head.data_len);
       // memcpy(page, "GT818", 5);
      //  page[5] = 0;

        //printk("Return ic type:%s len:%d\n", page, (s32)cmd_head.data_len);
        return cmd_head.data_len;
        //return sizeof(IC_TYPE_NAME);
    }
    else if (4 == cmd_head.wr)
    {
        page[0] = 100 >> 8;
        page[1] = 100 & 0xff;
        page[2] = 100 >> 8;
        page[3] = 100 & 0xff;

        return cmd_head.data_len;
    }
    else if (6 == cmd_head.wr)
    {
        //Read error code!
    }
    else if (8 == cmd_head.wr)  //Read driver version
    {
       // memcpy(page, GTP_DRIVER_VERSION, strlen(GTP_DRIVER_VERSION));
       s32 tmp_len;
       tmp_len = strlen(GTP_DRIVER_VERSION);
       memcpy(page, GTP_DRIVER_VERSION, tmp_len);
       page[tmp_len] = 0;
    }

    return cmd_head.data_len;
}
