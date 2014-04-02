/*
 * kernel/include/power/bq27541_battery.h
 */
#ifndef __BQ27541_BATTERY_H_INCLUDED__
#define __BQ27541_BATTERY_H_INCLUDED__

struct bq27541_platform_data {
	int wake_irq;
	unsigned int interval;
	unsigned int dc_check_pin;
	unsigned int bat_check_pin;
	unsigned int chgok_check_pin;
	unsigned int bat_num;
	int power_down;
	void (*init_dc_check_pin)(void);
	int capacity_max;
	int capacity_min;
	int (*get_charging_stat)(void);
};

#endif

