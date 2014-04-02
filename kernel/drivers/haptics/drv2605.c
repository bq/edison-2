/*
** =============================================================================
** Copyright (c) 2012  Immersion Corporation.
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** File:
**     drv2605.c
**
** Description:
**     DRV2605 chip driver
**
** =============================================================================
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/slab.h>
#include <linux/types.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>

#include <linux/syscalls.h>
#include <asm/uaccess.h>

#include <linux/gpio.h>
#include <mach/gpio.h>

#include <linux/sched.h>

#include "drv2605.h"

#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>

#include <linux/workqueue.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/wakelock.h>

/*  Current code version: 182 */
MODULE_AUTHOR("Immersion Corp.");
MODULE_DESCRIPTION("Driver for "HAPTICS_DEVICE_NAME);

#if SKIP_LRA_AUTOCAL == 1
static const unsigned char LRA_init_sequence[] = {
    MODE_REG,                       MODE_INTERNAL_TRIGGER,
    REAL_TIME_PLAYBACK_REG,         REAL_TIME_PLAYBACK_STRENGTH,
    LIBRARY_SELECTION_REG,          LIBRARY_F,
    WAVEFORM_SEQUENCER_REG,         WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG2,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG3,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG4,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG5,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG6,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG7,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG8,        WAVEFORM_SEQUENCER_DEFAULT,
    GO_REG,                         STOP,
    OVERDRIVE_TIME_OFFSET_REG,      0x00,
    SUSTAIN_TIME_OFFSET_POS_REG,    0x00,
    SUSTAIN_TIME_OFFSET_NEG_REG,    0x00,
    BRAKE_TIME_OFFSET_REG,          0x00,
    AUDIO_HAPTICS_CONTROL_REG,      AUDIO_HAPTICS_RECT_20MS | AUDIO_HAPTICS_FILTER_125HZ,
    AUDIO_HAPTICS_MIN_INPUT_REG,    AUDIO_HAPTICS_MIN_INPUT_VOLTAGE,
    AUDIO_HAPTICS_MAX_INPUT_REG,    AUDIO_HAPTICS_MAX_INPUT_VOLTAGE,
    AUDIO_HAPTICS_MIN_OUTPUT_REG,   AUDIO_HAPTICS_MIN_OUTPUT_VOLTAGE,
    AUDIO_HAPTICS_MAX_OUTPUT_REG,   AUDIO_HAPTICS_MAX_OUTPUT_VOLTAGE,
    RATED_VOLTAGE_REG,              LRA_RATED_VOLTAGE,
    OVERDRIVE_CLAMP_VOLTAGE_REG,    LRA_OVERDRIVE_CLAMP_VOLTAGE,
    AUTO_CALI_RESULT_REG,           DEFAULT_LRA_AUTOCAL_COMPENSATION,
    AUTO_CALI_BACK_EMF_RESULT_REG,  DEFAULT_LRA_AUTOCAL_BACKEMF,
    FEEDBACK_CONTROL_REG,           FEEDBACK_CONTROL_MODE_LRA | FB_BRAKE_FACTOR_4X | LOOP_RESPONSE_MEDIUM | FEEDBACK_CONTROL_BEMF_LRA_GAIN2,
    Control1_REG,                   STARTUP_BOOST_ENABLED | AC_COUPLE_ENABLED | AUDIOHAPTIC_DRIVE_TIME,
    Control2_REG,                   BIDIRECT_INPUT | AUTO_RES_GAIN_HIGH | BLANKING_TIME_SHORT | IDISS_TIME_SHORT | BRAKE_STABLIZER,
    Control3_REG,                   NG_Thresh_2 | INPUT_ANALOG,
    AUTOCAL_MEM_INTERFACE_REG,      AUTOCAL_TIME_500MS,
};
#endif

static const unsigned char ERM_autocal_sequence[] = {
    MODE_REG,                       AUTO_CALIBRATION,
    REAL_TIME_PLAYBACK_REG,         REAL_TIME_PLAYBACK_STRENGTH,
    LIBRARY_SELECTION_REG,          EFFECT_LIBRARY,
    WAVEFORM_SEQUENCER_REG,         WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG2,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG3,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG4,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG5,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG6,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG7,        WAVEFORM_SEQUENCER_DEFAULT,
    WAVEFORM_SEQUENCER_REG8,        WAVEFORM_SEQUENCER_DEFAULT,
    OVERDRIVE_TIME_OFFSET_REG,      0x00,
    SUSTAIN_TIME_OFFSET_POS_REG,    0x00,
    SUSTAIN_TIME_OFFSET_NEG_REG,    0x00,
    BRAKE_TIME_OFFSET_REG,          0x00,
    AUDIO_HAPTICS_CONTROL_REG,      AUDIO_HAPTICS_RECT_20MS | AUDIO_HAPTICS_FILTER_125HZ,
    AUDIO_HAPTICS_MIN_INPUT_REG,    AUDIO_HAPTICS_MIN_INPUT_VOLTAGE,
    AUDIO_HAPTICS_MAX_INPUT_REG,    AUDIO_HAPTICS_MAX_INPUT_VOLTAGE,
    AUDIO_HAPTICS_MIN_OUTPUT_REG,   AUDIO_HAPTICS_MIN_OUTPUT_VOLTAGE,
    AUDIO_HAPTICS_MAX_OUTPUT_REG,   AUDIO_HAPTICS_MAX_OUTPUT_VOLTAGE,
    RATED_VOLTAGE_REG,              ERM_RATED_VOLTAGE,
    OVERDRIVE_CLAMP_VOLTAGE_REG,    ERM_OVERDRIVE_CLAMP_VOLTAGE,
    AUTO_CALI_RESULT_REG,           DEFAULT_ERM_AUTOCAL_COMPENSATION,
    AUTO_CALI_BACK_EMF_RESULT_REG,  DEFAULT_ERM_AUTOCAL_BACKEMF,
    FEEDBACK_CONTROL_REG,           FB_BRAKE_FACTOR_4X | LOOP_RESPONSE_MEDIUM | FEEDBACK_CONTROL_BEMF_ERM_GAIN2,
    Control1_REG,                   STARTUP_BOOST_ENABLED | DEFAULT_DRIVE_TIME,
    Control2_REG,                   BIDIRECT_INPUT | BRAKE_STABLIZER |AUTO_RES_GAIN_HIGH| BLANKING_TIME_SHORT | IDISS_TIME_SHORT,
    Control3_REG,                   ERM_OpenLoop_Enabled | NG_Thresh_2,
    AUTOCAL_MEM_INTERFACE_REG,      AUTOCAL_TIME_500MS,
    GO_REG,                         GO,
};

static const unsigned char LRA_autocal_sequence[] = {
    MODE_REG,                       AUTO_CALIBRATION,
    RATED_VOLTAGE_REG,              LRA_RATED_VOLTAGE,
    OVERDRIVE_CLAMP_VOLTAGE_REG,    LRA_OVERDRIVE_CLAMP_VOLTAGE,
    FEEDBACK_CONTROL_REG,           FEEDBACK_CONTROL_MODE_LRA | FB_BRAKE_FACTOR_4X | LOOP_RESPONSE_FAST,
    Control3_REG,                   NG_Thresh_2,
    GO_REG,                         GO,
};


static int drv260x_write_reg_val(struct i2c_client *client,const unsigned char* data, unsigned int size)
{
    int i = 0;
	int err = 0;

    if (size % 2 != 0)
        return -EINVAL;

    while (i < size)
    {
        err = i2c_smbus_write_byte_data(client, data[i], data[i+1]);
		if(err < 0){
	        printk(KERN_ERR"%s, err=%d\n", __FUNCTION__, err);
			break;
		}	
        i+=2;
    }

	return err;
}

static void drv260x_set_go_bit(struct i2c_client *client,char val)
{
    char go[] =
    {
        GO_REG, val
    };
    drv260x_write_reg_val(client, go, sizeof(go));
}

static unsigned char drv260x_read_reg(struct i2c_client *client, unsigned char reg)
{
    return i2c_smbus_read_byte_data(client, reg);
}


static unsigned char drv260x_setbit_reg(struct i2c_client *client, unsigned char reg, unsigned char mask, unsigned char value)
{
	unsigned char temp = 0;
	unsigned char regval = drv260x_read_reg(client,reg);
	unsigned char buff[2];

	temp = regval & ~mask;
	temp |= value & mask;

	if(temp != regval){
		buff[0] = reg;
		buff[1] = temp;

		return drv260x_write_reg_val(client, buff, 2);
	}else
	    return 2;
}

static void drv2605_poll_go_bit(struct i2c_client *client)
{
    while (drv260x_read_reg(client, GO_REG) == GO)
      schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));
}

static void drv2605_select_library(struct i2c_client *client, char lib)
{
    char library[] =
    {
        LIBRARY_SELECTION_REG, lib
    };
    drv260x_write_reg_val(client, library, sizeof(library));
}

static void drv260x_set_rtp_val(struct i2c_client *client, char value)
{
    char rtp_val[] =
    {
        REAL_TIME_PLAYBACK_REG, value
    };
    drv260x_write_reg_val(client, rtp_val, sizeof(rtp_val));
}

static void drv2605_set_waveform_sequence(struct i2c_client *client, unsigned char* seq, unsigned int size)
{
    unsigned char data[WAVEFORM_SEQUENCER_MAX + 1];

    if (size > WAVEFORM_SEQUENCER_MAX)
        return;

    memset(data, 0, sizeof(data));
    memcpy(&data[1], seq, size);
    data[0] = WAVEFORM_SEQUENCER_REG;

    i2c_master_send(client, data, sizeof(data));
}

static void drv260x_change_mode(struct i2c_client *client, char mode)
{
    unsigned char tmp[] =
    {
        MODE_REG, mode
    };
    drv260x_write_reg_val(client, tmp, sizeof(tmp));
}

/* --------------------------------------------------------------------------------- */
#define YES 1
#define NO  0

static void setAudioHapticsEnabled(struct i2c_client *client, int enable);

static struct Haptics {
    struct wake_lock wklock;
    struct pwm_device *pwm_dev;
    struct hrtimer timer;
    struct mutex lock;
    struct work_struct work;
    struct work_struct work_play_eff;
    unsigned char sequence[8];
    volatile int should_stop;
	struct timed_output_dev to_dev;
	int testdata;
} vibdata;

static struct i2c_client *this_client;

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct Haptics	*pvibdata = container_of(dev, struct Haptics, to_dev);

    if (hrtimer_active(&vibdata.timer)) {
        ktime_t r = hrtimer_get_remaining(&vibdata.timer);
        return ktime_to_ms(r);
    }

    return 0;
}

static void vibrator_off(struct i2c_client *client)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	
    if (pDrv2605data->vibrator_is_playing) {
        pDrv2605data->vibrator_is_playing = NO;
        if (pDrv2605data->audio_haptics_enabled)
        {
            if ((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) != MODE_AUDIOHAPTIC)
                setAudioHapticsEnabled(client, YES);
        } else
        {
            drv260x_change_mode(client, MODE_STANDBY);
        }
    }

    wake_unlock(&vibdata.wklock);
}

static void vibrator_enable( struct timed_output_dev *dev, int value)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	struct Haptics	*pvibdata = container_of(dev, struct Haptics, to_dev);
    char mode;
		
    mutex_lock(&vibdata.lock);
    hrtimer_cancel(&vibdata.timer);
    cancel_work_sync(&vibdata.work);

    if (value) {
        wake_lock(&vibdata.wklock);

        mode = drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK;
        /* Only change the mode if not already in RTP mode; RTP input already set at init */
        if (mode != MODE_REAL_TIME_PLAYBACK)
        {
            if (pDrv2605data->audio_haptics_enabled && mode == MODE_AUDIOHAPTIC)
                setAudioHapticsEnabled(client, NO);

            drv260x_set_rtp_val(client, REAL_TIME_PLAYBACK_STRENGTH);
            drv260x_change_mode(client, MODE_REAL_TIME_PLAYBACK);
            pDrv2605data->vibrator_is_playing = YES;
        }

        if (value > 0) {
#if defined(CONFIG_MALATA_D1012)
	 value=value+120;
#endif
            if (value > MAX_TIMEOUT)
                value = MAX_TIMEOUT;
            hrtimer_start(&vibdata.timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
        }
    }
    else
        vibrator_off(client);

    mutex_unlock(&vibdata.lock);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
    schedule_work(&vibdata.work);
    return HRTIMER_NORESTART;
}

static void vibrator_work(struct work_struct *work)
{
    vibrator_off(this_client);
}

/* ----------------------------------------------------------------------------- */

static void play_effect(struct work_struct *work)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

    if (pDrv2605data->audio_haptics_enabled &&
        ((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC))
        setAudioHapticsEnabled(client, NO);

    drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
    drv2605_set_waveform_sequence(client, vibdata.sequence, sizeof(vibdata.sequence));
    drv260x_set_go_bit(client, GO);

    while(drv260x_read_reg(client, GO_REG) == GO && !vibdata.should_stop)
        schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));

    wake_unlock(&vibdata.wklock);
    if (pDrv2605data->audio_haptics_enabled)
    {
        setAudioHapticsEnabled(client, YES);
    } else
    {
        drv260x_change_mode(client, MODE_STANDBY);
    }
}

static void setAudioHapticsEnabled(struct i2c_client *client, int enable)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	
    if (enable)
    {
		if(pDrv2605data->PlatData.g_effect_bank != LIBRARY_F){
			char audiohaptic_settings[] =
			{
				Control1_REG, STARTUP_BOOST_ENABLED | AC_COUPLE_ENABLED | AUDIOHAPTIC_DRIVE_TIME,
				Control3_REG, NG_Thresh_2 | INPUT_ANALOG
			};
			// Chip needs to be brought out of standby to change the registers
			drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
			schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));
			drv260x_write_reg_val(client, audiohaptic_settings, sizeof(audiohaptic_settings));
		}
        drv260x_change_mode(client, MODE_AUDIOHAPTIC);
    } else
    {
        drv260x_change_mode(client, MODE_STANDBY); // Disable audio-to-haptics
        schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));
        // Chip needs to be brought out of standby to change the registers
        drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
		if(pDrv2605data->PlatData.g_effect_bank != LIBRARY_F){
	        char default_settings[] =
			{
				Control1_REG, STARTUP_BOOST_ENABLED | DEFAULT_DRIVE_TIME,
				Control3_REG, NG_Thresh_2 | ERM_OpenLoop_Enabled
			};
			
			schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

			drv260x_write_reg_val(client, default_settings, sizeof(default_settings));
		}
    }
}

static ssize_t drv260x_read(struct file* filp, char* buff, size_t length, loff_t* offset)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	int ret = 0, i;

    

	if(pDrv2605data->pReadValue != NULL){

		ret = copy_to_user(buff,pDrv2605data->pReadValue, pDrv2605data->ReadLen);
		if (ret != 0){
			printk("%s, copy_to_user err=%d \n", __FUNCTION__, ret);
		}else{
			ret = pDrv2605data->ReadLen;
		}
		pDrv2605data->ReadLen = 0;
		kfree(pDrv2605data->pReadValue);
		pDrv2605data->pReadValue = NULL;
		
	}else{

		buff[0] = pDrv2605data->read_val;	
		ret = 1;
	}
	
    return ret;
}

static ssize_t drv260x_write(struct file* filp, const char* buff, size_t len, loff_t* off)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	
    mutex_lock(&vibdata.lock);
    hrtimer_cancel(&vibdata.timer);

    vibdata.should_stop = YES;
    cancel_work_sync(&vibdata.work_play_eff);
    cancel_work_sync(&vibdata.work);

    if (pDrv2605data->vibrator_is_playing)
    {
        pDrv2605data->vibrator_is_playing = NO;
        drv260x_change_mode(client, MODE_STANDBY);
    }

    switch(buff[0])
    {
        case HAPTIC_CMDID_PLAY_SINGLE_EFFECT:
        case HAPTIC_CMDID_PLAY_EFFECT_SEQUENCE:
        {
            memset(&vibdata.sequence, 0, sizeof(vibdata.sequence));
            if (!copy_from_user(&vibdata.sequence, &buff[1], len - 1))
            {
                vibdata.should_stop = NO;
                wake_lock(&vibdata.wklock);
                schedule_work(&vibdata.work_play_eff);
            }
            break;
        }
        case HAPTIC_CMDID_PLAY_TIMED_EFFECT:
        {
            unsigned int value = 0;
			char mode;

            value = buff[2];
            value <<= 8;
            value |= buff[1];

            if (value)
            {
                wake_lock(&vibdata.wklock);

                mode = drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK;
                if (mode != MODE_REAL_TIME_PLAYBACK)
                {
                    if (pDrv2605data->audio_haptics_enabled && mode == MODE_AUDIOHAPTIC)
                        setAudioHapticsEnabled(client, NO);

                    drv260x_set_rtp_val(client, REAL_TIME_PLAYBACK_STRENGTH);
                    drv260x_change_mode(client, MODE_REAL_TIME_PLAYBACK);
                    pDrv2605data->vibrator_is_playing = YES;
                }

                if (value > 0)
                {
                    if (value > MAX_TIMEOUT)
                        value = MAX_TIMEOUT;
                    hrtimer_start(&vibdata.timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
                }
            }
            break;
        }
        case HAPTIC_CMDID_STOP:
        {
            if (pDrv2605data->vibrator_is_playing)
            {
                pDrv2605data->vibrator_is_playing = NO;
                if (pDrv2605data->audio_haptics_enabled)
                {
                    setAudioHapticsEnabled(client, YES);
                } else
                {
                    drv260x_change_mode(client, MODE_STANDBY);
                }
            }
            vibdata.should_stop = YES;
            break;
        }
        case HAPTIC_CMDID_GET_DEV_ID:
        {
            /* Dev ID includes 2 parts, upper word for device id, lower word for chip revision */
            int revision = (drv260x_read_reg(client, SILICON_REVISION_REG) & SILICON_REVISION_MASK);
            pDrv2605data->read_val = (pDrv2605data->device_id >> 1) | revision;
            break;
        }
        case HAPTIC_CMDID_RUN_DIAG:
        {
            char diag_seq[] =
            {
                MODE_REG, MODE_DIAGNOSTICS,
                GO_REG,   GO
            };

            if (pDrv2605data->audio_haptics_enabled &&
                ((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC))
                setAudioHapticsEnabled(client, NO);
			
            drv260x_write_reg_val(client, diag_seq, sizeof(diag_seq));
            drv2605_poll_go_bit(client);
            pDrv2605data->read_val = (drv260x_read_reg(client, STATUS_REG) & DIAG_RESULT_MASK) >> 3;
            break;
        }
        case HAPTIC_CMDID_AUDIOHAPTIC_ENABLE:
        {
            if ((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) != MODE_AUDIOHAPTIC)
            {
                setAudioHapticsEnabled(client, YES);
                pDrv2605data->audio_haptics_enabled = YES;
            }
            break;
        }
        case HAPTIC_CMDID_AUDIOHAPTIC_DISABLE:
        {
            if (pDrv2605data->audio_haptics_enabled)
            {
                if ((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC)
                    setAudioHapticsEnabled(client, NO);
                pDrv2605data->audio_haptics_enabled = NO;
                drv260x_change_mode(client, MODE_STANDBY);
            }
            break;
        }
        case HAPTIC_CMDID_AUDIOHAPTIC_GETSTATUS:
        {
            if ((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC)
            {
                pDrv2605data->read_val = 1;
            }
            else
            {
                pDrv2605data->read_val = 0;
            }
            break;
        }
		case HAPTIC_CMDID_REG_READ:
		{
			int i=1;
			if(pDrv2605data->pReadValue != NULL){
				printk("%s, ERROR, pReadValue should be NULL\n",__FUNCTION__);
			}else{
				pDrv2605data->pReadValue = (char *)kzalloc(len-1, GFP_KERNEL);
				if(pDrv2605data->pReadValue == NULL){
					printk("%s, ERROR, pReadValue alloc fail\n",__FUNCTION__);					
				}else{
					pDrv2605data->ReadLen = len -1;
					
					for(i=0;i<(len-1);i++){
						pDrv2605data->pReadValue[i] = drv260x_read_reg(client, buff[i+1]);	
					}
				}
			}

			break;
		}
		case HAPTIC_CMDID_REG_WRITE:
		{
			drv260x_write_reg_val(client, &buff[1], len-1);	
			
			break;
		}
		case HAPTIC_CMDID_REG_SETBIT:
		{
			int i=1;			
			for(i=1; i< len; ){
				drv260x_setbit_reg(client, buff[i], buff[i+1], buff[i+2]);
				i += 3;
			}
			break;
		}		
    default:
		printk("%s, unknown HAPTIC cmd\n", __FUNCTION__);
      break;
    }

    mutex_unlock(&vibdata.lock);

    return len;
}


static struct file_operations fops =
{
    .read = drv260x_read,
    .write = drv260x_write
};

static int Haptics_init(struct drv2605_data *pDrv2605Data)
{
    int reval = -ENOMEM;

   
    pDrv2605Data->version = MKDEV(0,0);
    reval = alloc_chrdev_region(&pDrv2605Data->version, 0, 1, HAPTICS_DEVICE_NAME);
    if (reval < 0)
    {
        printk(KERN_ALERT"drv260x: error getting major number %d\n", reval);
        goto fail0;
    }

    pDrv2605Data->class = class_create(THIS_MODULE, HAPTICS_DEVICE_NAME);
    if (!pDrv2605Data->class)
    {
        printk(KERN_ALERT"drv260x: error creating class\n");
        goto fail1;
    }

    pDrv2605Data->device = device_create(pDrv2605Data->class, NULL, pDrv2605Data->version, NULL, HAPTICS_DEVICE_NAME);
    if (!pDrv2605Data->device)
    {
        printk(KERN_ALERT"drv260x: error creating device 2605\n");
        goto fail2;
    }

    cdev_init(&pDrv2605Data->cdev, &fops);
    pDrv2605Data->cdev.owner = THIS_MODULE;
    pDrv2605Data->cdev.ops = &fops;
    reval = cdev_add(&pDrv2605Data->cdev, pDrv2605Data->version, 1);

    if (reval)
    {
        printk(KERN_ALERT"drv260x: fail to add cdev\n");
        goto fail3;
    }

	vibdata.to_dev.name = "vibrator";
	vibdata.to_dev.get_time = vibrator_get_time;
	vibdata.to_dev.enable = vibrator_enable;
	vibdata.testdata = 0x12345678;

    if (timed_output_dev_register(&(vibdata.to_dev)) < 0)
    {
        printk(KERN_ALERT"drv260x: fail to create timed output dev\n");
        goto fail3;
    }

    hrtimer_init(&vibdata.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    vibdata.timer.function = vibrator_timer_func;
    INIT_WORK(&vibdata.work, vibrator_work);
    INIT_WORK(&vibdata.work_play_eff, play_effect);

    wake_lock_init(&vibdata.wklock, WAKE_LOCK_SUSPEND, "vibrator");
    mutex_init(&vibdata.lock);

    printk(KERN_ALERT"drv260x: initialized\n");
    return 0;

fail3:
	device_destroy(pDrv2605Data->class, pDrv2605Data->version);
fail2:
    class_destroy(pDrv2605Data->class);	
fail1:
    unregister_chrdev_region(pDrv2605Data->version, 1);	
fail0:
    return reval;
}

static int drv260x_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	struct drv2605_data *pDrv2605data;
	struct drv2605_platform_data *pDrv2605Platdata = client->dev.platform_data;
	
	int err = 0;
	int status = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR"%s:I2C check failed\n", __FUNCTION__);
		return -ENODEV;
	}

	pDrv2605data = kzalloc(sizeof(struct drv2605_data),GFP_KERNEL);
	if(!pDrv2605data){
		err = -ENOMEM;
		printk(KERN_ERR"%s: -ENOMEM error\n", __FUNCTION__);		
		goto exit_alloc_data_failed;
	}

	pDrv2605data->client = client;
	this_client = client;
	
	memcpy(&pDrv2605data->PlatData, pDrv2605Platdata, sizeof(struct drv2605_platform_data));
	i2c_set_clientdata(client,pDrv2605data);

	if(pDrv2605data->PlatData.GpioTrigger){
		err = gpio_request(pDrv2605data->PlatData.GpioEnable,HAPTICS_DEVICE_NAME"Trigger");
		if(err < 0){
			printk(KERN_ERR"%s: GPIO request Trigger error\n", __FUNCTION__);				
			goto exit_gpio_request_failed1;
		}
	}

	if(pDrv2605data->PlatData.GpioEnable){
		err = gpio_request(pDrv2605data->PlatData.GpioEnable,HAPTICS_DEVICE_NAME"Enable");
		if(err < 0){
			printk(KERN_ERR"%s: GPIO request enable error\n", __FUNCTION__);					
			goto exit_gpio_request_failed2;
		}

	    /* Enable power to the chip */
	    gpio_direction_output(pDrv2605data->PlatData.GpioEnable, GPIO_HIGH);

	    /* Wait 30 us */
	    udelay(30);
	}

#if SKIP_LRA_AUTOCAL == 1
	if(pDrv2605data->PlatData.g_effect_bank != LIBRARY_F)
		err = drv260x_write_reg_val(pDrv2605data->client, ERM_autocal_sequence, sizeof(ERM_autocal_sequence));
	else
		err = drv260x_write_reg_val(pDrv2605data->client, LRA_init_sequence, sizeof(LRA_init_sequence));
#else
	if(pDrv2605data->PlatData.g_effect_bank == LIBRARY_F)
		err = drv260x_write_reg_val(pDrv2605data->client, LRA_autocal_sequence, sizeof(LRA_autocal_sequence));
	else
		err = drv260x_write_reg_val(pDrv2605data->client, ERM_autocal_sequence, sizeof(ERM_autocal_sequence));
#endif

	if(err < 0){
		printk(KERN_ERR"%s: I2C access error\n", __FUNCTION__);			
		goto exit_gpio_request_failed2;
	}

    /* Wait until the procedure is done */
    drv2605_poll_go_bit(pDrv2605data->client);

    /* Read status */
    status = drv260x_read_reg(pDrv2605data->client, STATUS_REG);

#if SKIP_LRA_AUTOCAL == 0
    /* Check result */
    if ((status & DIAG_RESULT_MASK) == AUTO_CAL_FAILED)
    {
        printk(KERN_ALERT"drv260x auto-cal failed.\n");
        if (pDrv2605data->PlatData.g_effect_bank == LIBRARY_F)
            drv260x_write_reg_val(pDrv2605data->client, LRA_autocal_sequence, sizeof(LRA_autocal_sequence));
        else
            drv260x_write_reg_val(pDrv2605data->client, ERM_autocal_sequence, sizeof(ERM_autocal_sequence));
		
        drv2605_poll_go_bit(pDrv2605data->client);
        status = drv260x_read_reg(pDrv2605data->client, STATUS_REG);
        if ((status & DIAG_RESULT_MASK) == AUTO_CAL_FAILED)
        {
            printk(KERN_ALERT"drv260x auto-cal retry failed.\n");
            // return -ENODEV;
        }
    }
#endif

    /* Read calibration results */
    drv260x_read_reg(pDrv2605data->client, AUTO_CALI_RESULT_REG);
    drv260x_read_reg(pDrv2605data->client, AUTO_CALI_BACK_EMF_RESULT_REG);
    drv260x_read_reg(pDrv2605data->client, FEEDBACK_CONTROL_REG);

    /* Read device ID */
    pDrv2605data->device_id = (status & DEV_ID_MASK);
    switch (pDrv2605data->device_id)
    {
    case DRV2605:
        printk("drv260x driver found: drv2605.\n");
        break;
    case DRV2604:
        printk(KERN_ALERT"drv260x driver found: drv2604.\n");
        break;
    default:
        printk(KERN_ERR"drv260x driver found: unknown.\n");
        break;
    }

    /* Choose default effect library */
    drv2605_select_library(pDrv2605data->client, pDrv2605data->PlatData.g_effect_bank);

    /* Put hardware in standby */
    drv260x_change_mode(pDrv2605data->client, MODE_STANDBY);

    Haptics_init(pDrv2605data);
	
    printk("drv260x probe succeeded\n");

    return 0;

exit_gpio_request_failed2:
	if(pDrv2605data->PlatData.GpioTrigger){
		gpio_free(pDrv2605data->PlatData.GpioTrigger);
	}

exit_gpio_request_failed1:
	if(pDrv2605data){
		kfree(pDrv2605data);
	}
exit_alloc_data_failed:
    printk(KERN_ERR"%s failed, err=%d\n",__FUNCTION__, err);
	return err;
}

static int drv260x_remove(struct i2c_client* client)
{
	struct drv2605_data *pDrv2605Data = i2c_get_clientdata(client);

    device_destroy(pDrv2605Data->class, pDrv2605Data->version);
    class_destroy(pDrv2605Data->class);
    unregister_chrdev_region(pDrv2605Data->version, 1);

	if(pDrv2605Data->PlatData.GpioTrigger)
		gpio_free(pDrv2605Data->PlatData.GpioTrigger);

	if(pDrv2605Data->PlatData.GpioEnable)
		gpio_free(pDrv2605Data->PlatData.GpioEnable);

	kfree(pDrv2605Data);

	i2c_set_clientdata(client,NULL);
	
    printk(KERN_ALERT"drv260x remove");
	
    return 0;
}


static struct i2c_device_id drv260x_id_table[] =
{
    { HAPTICS_DEVICE_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, drv260x_id_table);

static struct i2c_driver drv260x_driver =
{
    .driver = {
        .name = HAPTICS_DEVICE_NAME,
    },
    .id_table = drv260x_id_table,
    .probe = drv260x_probe,
    .remove = drv260x_remove
};

static int __init drv260x_init(void)
{
	return i2c_add_driver(&drv260x_driver);
}

static void __exit drv260x_exit(void)
{
	i2c_del_driver(&drv260x_driver);
}


subsys_initcall_sync(drv260x_init);
module_exit(drv260x_exit);
