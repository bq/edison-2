/*
 *  Switch class driver
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#ifndef __LINUX_SWITCH_VIDEO_H__
#define __LINUX_SWITCH_VIDEO_H__

struct switch_video_data {
	int (*io_init)(void);
	int standy_pin;
	int source_switch_1;
	int source_switch_2;
};

#endif /* __LINUX_SWITCH_VIDEO_H__ */
