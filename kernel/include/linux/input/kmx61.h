/*
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __KIONIX_ANDROID_H__
#define __KIONIX_ANDROID_H__

#define KMX61_I2C_ADDR		(0x0F)

struct kmx61_platform_data {
	unsigned int min_interval;	/* minimum poll interval (in milli-seconds) */
	unsigned int init_interval;	/* initial poll interval (in milli-seconds) */

	/*
	 * By default, x is axis 0, y is axis 1, z is axis 2; these can be
	 * changed to account for sensor orientation within the host device.
	 */
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	/*
	 * Each axis can be negated to account for sensor orientation within
	 * the host device.
	 */
	bool negate_x;
	bool negate_y;
	bool negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};
#endif  /* __KIONIX_ANDROID_H__ */
