/*
 * rt5616_ioctl.h  --  RT5616 ALSA SoC audio driver IO control
 *
 * Copyright 2012 Realtek Microelectronics
 * Author: Bard <bardliao@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/spi/spi.h>
#include <sound/soc.h>
#include "rt_codec_ioctl.h"
#include "rt5616_ioctl.h"
#include "rt5616.h"


hweq_t hweq_param[] = {
	{/* NORMAL */
		{0},
		{0},
		0x0000,
	},
	{/* LOUT */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2,	0xb3,	0xb4},
		{0x1cb5,0xf5ae,	0xc9a4,	0x1bbc,	0x0000,	0xd8cb,	0x1bbc,	0x0000,	0xef01,	0x1bbc,	0x0000, 0xef01,	0x1bbc,	0x0000,	0x0257,	0x0000,	0x1cc9,	0x02eb,	0x1cee,	0x0402,	0x0ff6},
		0x0001,
	},
	{/* HP */
		{0xa0,	0xa1,	0xa2,	0xa3,	0xa4,	0xa5,	0xa6,	0xa7,	0xa8,	0xa9,	0xaa,	0xab,	0xac,	0xad,	0xae,	0xaf,	0xb0,	0xb1,	0xb2},
		{0x1c10,0x01f4,	0xc5e9,	0x1a98,	0x1d2c,	0xc882,	0x1c10,	0x01f4,	0xe904,	0x1c10,	0x01f4, 0xe904,	0x1c10,	0x01f4,	0x1c10,	0x01f4,	0x2000,	0x0000,	0x2000},
		0x0000,
	},
};
#define RT5616_HWEQ_LEN ARRAY_SIZE(hweq_param)

int rt5616_update_eqmode(
	struct snd_soc_codec *codec, int mode)
{
	struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops();
	int i;
	static int  eq_mode=0;

	if(codec == NULL ||  mode >= RT5616_HWEQ_LEN)
		return -EINVAL;

	dev_dbg(codec->dev, "%s(): mode=%d\n", __func__, mode);
	if(mode == eq_mode)
		return 0;
	for(i = 0; i <= EQ_REG_NUM; i++) {
		if(hweq_param[mode].reg[i])
			ioctl_ops->index_write(codec, hweq_param[mode].reg[i],
					hweq_param[mode].value[i]);
		else
			break;
	}
	snd_soc_update_bits(codec, RT5616_EQ_CTRL2, RT5616_EQ_CTRL_MASK,
					hweq_param[mode].ctrl);
	snd_soc_update_bits(codec, RT5616_EQ_CTRL1,
		RT5616_EQ_UPD, RT5616_EQ_UPD);
	snd_soc_update_bits(codec, RT5616_EQ_CTRL1, RT5616_EQ_UPD, 0);
	eq_mode = mode;
	return 0;
}

void set_drc_agc_enable(struct snd_soc_codec *codec, int enable, int path)
{
	snd_soc_update_bits(codec, RT5616_DRC_AGC_1, RT5616_DRC_AGC_P_MASK |
		RT5616_DRC_AGC_MASK | RT5616_DRC_AGC_UPD,
		enable << RT5616_DRC_AGC_SFT | path << RT5616_DRC_AGC_P_SFT |
		1 << RT5616_DRC_AGC_UPD_BIT);
}

void set_drc_agc_parameters(struct snd_soc_codec *codec, int attack_rate,
			int sample_rate, int recovery_rate, int limit_level)
{
	snd_soc_update_bits(codec, RT5616_DRC_AGC_3, RT5616_DRC_AGC_TAR_MASK,
				limit_level << RT5616_DRC_AGC_TAR_SFT);
	snd_soc_update_bits(codec, RT5616_DRC_AGC_1, RT5616_DRC_AGC_AR_MASK |
		RT5616_DRC_AGC_R_MASK | RT5616_DRC_AGC_UPD |
		RT5616_DRC_AGC_RC_MASK, attack_rate << RT5616_DRC_AGC_AR_SFT |
		sample_rate << RT5616_DRC_AGC_R_SFT |
		recovery_rate << RT5616_DRC_AGC_RC_SFT |
		0x1 << RT5616_DRC_AGC_UPD_BIT);
}

void set_digital_boost_gain(struct snd_soc_codec *codec,
			int post_gain, int pre_gain)
{
	snd_soc_update_bits(codec, RT5616_DRC_AGC_2,
		RT5616_DRC_AGC_POB_MASK | RT5616_DRC_AGC_PRB_MASK,
		post_gain << RT5616_DRC_AGC_POB_SFT |
		pre_gain << RT5616_DRC_AGC_PRB_SFT);
	snd_soc_update_bits(codec, RT5616_DRC_AGC_1, 
		RT5616_DRC_AGC_UPD, 1 << RT5616_DRC_AGC_UPD_BIT);
}

void set_noise_gate(struct snd_soc_codec *codec, int noise_gate_en,
	int noise_gate_hold_en, int compression_gain, int noise_gate_th)
{
	snd_soc_update_bits(codec, RT5616_DRC_AGC_3,
		RT5616_DRC_AGC_NGB_MASK | RT5616_DRC_AGC_NG_MASK |
		RT5616_DRC_AGC_NGH_MASK | RT5616_DRC_AGC_NGT_MASK,
		noise_gate_en << RT5616_DRC_AGC_NG_SFT |
		noise_gate_hold_en << RT5616_DRC_AGC_NGH_SFT |
		compression_gain << RT5616_DRC_AGC_NGB_SFT |
		noise_gate_th << RT5616_DRC_AGC_NGT_SFT);
	snd_soc_update_bits(codec, RT5616_DRC_AGC_1,
		RT5616_DRC_AGC_UPD, 1 << RT5616_DRC_AGC_UPD_BIT);
}

void set_drc_agc_compression(struct snd_soc_codec *codec,
		int compression_en, int compression_ratio)
{
	snd_soc_update_bits(codec, RT5616_DRC_AGC_2,
		RT5616_DRC_AGC_CP_MASK | RT5616_DRC_AGC_CPR_MASK,
		compression_en << RT5616_DRC_AGC_CP_SFT |
		compression_ratio << RT5616_DRC_AGC_CPR_SFT);
	snd_soc_update_bits(codec, RT5616_DRC_AGC_1,
		RT5616_DRC_AGC_UPD, 1 << RT5616_DRC_AGC_UPD_BIT);
}

void get_drc_agc_enable(struct snd_soc_codec *codec, int *enable, int *path)
{
	unsigned int reg = snd_soc_read(codec, RT5616_DRC_AGC_1);

	*enable = (reg & RT5616_DRC_AGC_MASK) >> RT5616_DRC_AGC_SFT;
	*path = (reg & RT5616_DRC_AGC_P_MASK) >> RT5616_DRC_AGC_P_SFT;
}

void get_drc_agc_parameters(struct snd_soc_codec *codec, int *attack_rate,
		int *sample_rate, int *recovery_rate, int *limit_level)
{
	unsigned int reg = snd_soc_read(codec, RT5616_DRC_AGC_3);

	*limit_level = (reg & RT5616_DRC_AGC_TAR_MASK) >>
			RT5616_DRC_AGC_TAR_SFT;
	reg = snd_soc_read(codec, RT5616_DRC_AGC_1);
	*attack_rate = (reg & RT5616_DRC_AGC_AR_MASK) >> RT5616_DRC_AGC_AR_SFT;
	*sample_rate = (reg & RT5616_DRC_AGC_R_MASK) >> RT5616_DRC_AGC_R_SFT;
	*recovery_rate = (reg & RT5616_DRC_AGC_RC_MASK) >>
				RT5616_DRC_AGC_RC_SFT;
}

void get_digital_boost_gain(struct snd_soc_codec *codec,
			int *post_gain, int *pre_gain)
{
	unsigned int reg = snd_soc_read(codec, RT5616_DRC_AGC_2);

	*post_gain = (reg & RT5616_DRC_AGC_POB_MASK) >> RT5616_DRC_AGC_POB_SFT;
	*pre_gain = (reg & RT5616_DRC_AGC_PRB_MASK) >> RT5616_DRC_AGC_PRB_SFT;
}

void get_noise_gate(struct snd_soc_codec *codec, int *noise_gate_en,
	int *noise_gate_hold_en, int *compression_gain, int *noise_gate_th)
{
	unsigned int reg = snd_soc_read(codec, RT5616_DRC_AGC_3);

	printk("get_noise_gate reg=0x%04x\n",reg);
	*noise_gate_en = (reg & RT5616_DRC_AGC_NG_MASK) >>
				RT5616_DRC_AGC_NG_SFT;
	*noise_gate_hold_en = (reg & RT5616_DRC_AGC_NGH_MASK) >>
				RT5616_DRC_AGC_NGH_SFT;
	*compression_gain = (reg & RT5616_DRC_AGC_NGB_MASK) >>
				RT5616_DRC_AGC_NGB_SFT;
	*noise_gate_th = (reg & RT5616_DRC_AGC_NGT_MASK) >>
				RT5616_DRC_AGC_NGT_SFT;
}

void get_drc_agc_compression(struct snd_soc_codec *codec,
		int *compression_en, int *compression_ratio)
{
	unsigned int reg = snd_soc_read(codec, RT5616_DRC_AGC_2);

	*compression_en = (reg & RT5616_DRC_AGC_CP_MASK) >>
				RT5616_DRC_AGC_CP_SFT;
	*compression_ratio = (reg & RT5616_DRC_AGC_CPR_MASK) >>
				RT5616_DRC_AGC_CPR_SFT;
}

int rt5616_ioctl_common(struct snd_hwdep *hw, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct snd_soc_codec *codec = hw->private_data;
	struct rt_codec_cmd __user *_rt_codec = (struct rt_codec_cmd *)arg;
	struct rt_codec_cmd rt_codec;
	struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops();
	int *buf, mask1 = 0, mask2 = 0;
	static int eq_mode;

	if (copy_from_user(&rt_codec, _rt_codec, sizeof(rt_codec))) {
		dev_err(codec->dev,"copy_from_user faild\n");
		return -EFAULT;
	}
	dev_dbg(codec->dev, "%s(): rt_codec.number=%d, cmd=%d\n",
			__func__, rt_codec.number, cmd);
	buf = kmalloc(sizeof(*buf) * rt_codec.number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;
	if (copy_from_user(buf, rt_codec.buf, sizeof(*buf) * rt_codec.number)) {
		goto err;
	}
	
	switch (cmd) {
	case RT_SET_CODEC_HWEQ_IOCTL:
		if (eq_mode == *buf)
			break;
		eq_mode = *buf;
		rt5616_update_eqmode(codec, eq_mode);
		break;

	case RT_GET_CODEC_ID:
		*buf = snd_soc_read(codec, RT5616_DEVICE_ID);
		if (copy_to_user(rt_codec.buf, buf, sizeof(*buf) * rt_codec.number))
			goto err;
		break;

	case RT_SET_CODEC_MIC_GAIN_IOCTL:
		if(*(buf) <= 0x8) {
			snd_soc_update_bits(codec, RT5616_IN1_IN2,
				RT5616_BST_MASK1, *(buf) << RT5616_BST_SFT1);
		}
		break;

	case RT_SET_CODEC_3D_SPK_IOCTL:
		if(rt_codec.number < 4)
			break;
		if (NULL == ioctl_ops->index_update_bits)
			break;

		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5616_3D_SPK_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5616_3D_SPK_M_MASK;
		if(*(buf + 2) != -1)
			mask1 |= RT5616_3D_SPK_CG_MASK;
		if(*(buf + 3) != -1)
			mask1 |= RT5616_3D_SPK_SG_MASK;
		ioctl_ops->index_update_bits(codec, RT5616_3D_SPK, mask1,
			*(buf) << RT5616_3D_SPK_SFT |
			*(buf + 1) << RT5616_3D_SPK_M_SFT |
			*(buf + 2) << RT5616_3D_SPK_CG_SFT |
			*(buf + 3) << RT5616_3D_SPK_SG_SFT);
		break;

	case RT_SET_CODEC_MP3PLUS_IOCTL:
		if(rt_codec.number < 5)
			break;
		mask1 = mask2 = 0;
		if(*buf != -1)
			mask1 |= RT5616_M_MP3_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5616_EG_MP3_MASK;
		if(*(buf + 2) != -1)
			mask2 |= RT5616_OG_MP3_MASK;
		if(*(buf + 3) != -1)
			mask2 |= RT5616_HG_MP3_MASK;
		if(*(buf + 4) != -1)
			mask2 |= RT5616_MP3_WT_MASK;
		
		snd_soc_update_bits(codec, RT5616_MP3_PLUS1, mask1,
			*(buf) << RT5616_M_MP3_SFT |
			*(buf + 1) << RT5616_EG_MP3_SFT);
		snd_soc_update_bits(codec, RT5616_MP3_PLUS2, mask2,
			*(buf + 2) << RT5616_OG_MP3_SFT |
			*(buf + 3) << RT5616_HG_MP3_SFT |
			*(buf + 4) << RT5616_MP3_WT_SFT);
		break;

	case RT_SET_CODEC_BASS_BACK_IOCTL:
		if(rt_codec.number < 3)
			break;
		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5616_BB_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5616_BB_CT_MASK;
		if(*(buf + 2) != -1)
			mask1 |= RT5616_G_BB_BST_MASK;
		
		snd_soc_update_bits(codec, RT5616_BASE_BACK, mask1,
			*(buf) << RT5616_BB_SFT |
			*(buf + 1) << RT5616_BB_CT_SFT |
			*(buf + 2) << RT5616_G_BB_BST_SFT);
		break;

	case RT_SET_CODEC_DIPOLE_SPK_IOCTL:
		if(rt_codec.number < 2)
			break;
		if (NULL == ioctl_ops->index_update_bits)
			break;

		mask1 = 0;
		if(*buf != -1)
			mask1 |= RT5616_DP_SPK_MASK;
		if(*(buf + 1) != -1)
			mask1 |= RT5616_DP_ATT_MASK;
		
		ioctl_ops->index_update_bits(codec, RT5616_DIP_SPK_INF,
			mask1, *(buf) << RT5616_DP_SPK_SFT |
			*(buf + 1) << RT5616_DP_ATT_SFT );
		break;

	case RT_SET_CODEC_DRC_AGC_ENABLE_IOCTL:
		if(rt_codec.number < 2)
			break;
		set_drc_agc_enable(codec, *(buf), *(buf + 1));
		break;

	case RT_SET_CODEC_DRC_AGC_PAR_IOCTL:
		if(rt_codec.number < 4)
			break;
		set_drc_agc_parameters(codec, *(buf), *(buf + 1),
				*(buf + 2), *(buf + 3));
		break;

	case RT_SET_CODEC_DIGI_BOOST_GAIN_IOCTL:
		if(rt_codec.number < 2)
			break;
		set_digital_boost_gain(codec, *(buf), *(buf + 1));
		break;

	case RT_SET_CODEC_NOISE_GATE_IOCTL:
		if(rt_codec.number < 4)
			break;
		set_noise_gate(codec, *(buf), *(buf + 1),
				*(buf + 2), *(buf + 3));
		break;

	case RT_SET_CODEC_DRC_AGC_COMP_IOCTL:
		if(rt_codec.number < 2)
			break;
		set_drc_agc_compression(codec, *(buf), *(buf + 1));
		break;

	case RT_SET_CODEC_WNR_ENABLE_IOCTL:
		if (NULL == ioctl_ops->index_update_bits)
			break;

		ioctl_ops->index_update_bits(codec, RT5616_WND_1,
			RT5616_WND_MASK, *(buf) << RT5616_WND_SFT );
		break;

	case RT_GET_CODEC_DRC_AGC_ENABLE_IOCTL:
		if(rt_codec.number < 2)
			break;
		get_drc_agc_enable(codec, (buf), (buf + 1));
		if (copy_to_user(rt_codec.buf, buf, sizeof(*buf) * rt_codec.number))
			goto err;
		break;

	case RT_GET_CODEC_DRC_AGC_PAR_IOCTL:
		if(rt_codec.number < 4)
			break;
		get_drc_agc_parameters(codec, (buf), (buf + 1),
				(buf + 2), (buf + 3));
		if (copy_to_user(rt_codec.buf, buf,
			sizeof(*buf) * rt_codec.number))
			goto err;
		break;

	case RT_GET_CODEC_DIGI_BOOST_GAIN_IOCTL:
		if(rt_codec.number < 2)
			break;
		get_digital_boost_gain(codec, (buf), (buf + 1));
		if (copy_to_user(rt_codec.buf, buf,
			sizeof(*buf) * rt_codec.number))
			goto err;
		break;

	case RT_GET_CODEC_NOISE_GATE_IOCTL:
		if(rt_codec.number < 4)
			break;
		get_noise_gate(codec, (buf), (buf + 1), (buf + 2), (buf + 3));
		if (copy_to_user(rt_codec.buf, buf,
			sizeof(*buf) * rt_codec.number))
			goto err;
		break;

	case RT_GET_CODEC_DRC_AGC_COMP_IOCTL:
		if(rt_codec.number < 2)
			break;
		get_drc_agc_compression(codec, (buf), (buf + 1));
		if (copy_to_user(rt_codec.buf, buf,
			sizeof(*buf) * rt_codec.number))
			goto err;
		break;

	case RT_GET_CODEC_MIC_GAIN_IOCTL:
		*buf = (snd_soc_read(codec, RT5616_IN1_IN2) & RT5616_BST_MASK1)
			>> RT5616_BST_SFT1;
		if (copy_to_user(rt_codec.buf, buf, sizeof(*buf) * rt_codec.number))
			goto err;
		break;

	case RT_GET_CODEC_HWEQ_IOCTL:
	case RT_GET_CODEC_3D_SPK_IOCTL:
	case RT_GET_CODEC_MP3PLUS_IOCTL:
	case RT_GET_CODEC_3D_HEADPHONE_IOCTL:
	case RT_GET_CODEC_BASS_BACK_IOCTL:
	case RT_GET_CODEC_DIPOLE_SPK_IOCTL:
	default:
		break;
	}

	kfree(buf);
	return 0;

err:
	kfree(buf);
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(rt5616_ioctl_common);
