#ifndef __RK29_KEYS_H__
#define __RK29_KEYS_H__
#include <linux/input.h>

/*buttion "+" or "-" jitter
	In order to solve the problem of key jitter,the report the polling time(DEFAULT_DEBOUNCE_INTERVAL)
	is greater than the adc read  polling time.
*/
#define ADC_SAMPLE_TIME				30
#define DEBONCE_TIME_DELAY    20
#define DEFAULT_DEBOUNCE_INTERVAL	(ADC_SAMPLE_TIME + DEBONCE_TIME_DELAY)

#define LONG_PRESS_COUNT			100 //100 * 10 = 1000ms
#define ONE_SEC_COUNT				(1000/DEFAULT_DEBOUNCE_INTERVAL)

#define EV_ENCALL			KEY_F4
#define EV_MENU				KEY_F1

#define PRESS_LEV_LOW			1
#define PRESS_LEV_HIGH			0

struct rk29_keys_button {
	int code;		
	int code1;
	int code_long_press;
	int gpio;
	int adc_value;
	int adc_state;
	int adc_oldstate;
	int active_low;
	char *desc;
	int wakeup;	
	int hall_key;
};


struct rk29_keys_platform_data {
	struct rk29_keys_button *buttons;
	int nbuttons;
	int chn;
	int rep;
};

#endif
