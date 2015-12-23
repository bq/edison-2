/*
 * kernel/include/power/bq27410_battery.h
 */
#ifndef __BQ27410_BATTERY_H_INCLUDED__
#define __BQ27410_BATTERY_H_INCLUDED__

struct bq27410_platform_data {
	int wake_irq;
	unsigned int interval;
	unsigned int dc_check_pin;
	unsigned int usb_check_pin;
	unsigned int bat_check_pin;
	unsigned int chgok_check_pin;
	unsigned int low_power_pin;
	unsigned int bat_num;
	int power_down;
	void (*io_init)(void);
	int capacity_max;
	int capacity_min;
	int (*get_charging_stat)(void);
};

#endif

