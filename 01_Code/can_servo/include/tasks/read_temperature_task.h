#ifndef __READ_TEMPERATURE_TASK_H__
#define __READ_TEMPERATURE_TASK_H__
#include "../all_includes.h"


#define K_TO_C 273.15f
#define THERMISTOR_RESISTOR_DIVIDER 10000.0f
#define THERM_A 0.001129148f
#define THERM_B 0.000234125f
#define THERM_C 0.0000000876741f
#define READ_TEMPERATURE_TASK_DELAY 1000



extern adc_oneshot_unit_handle_t adc_handle;

extern SemaphoreHandle_t temperature_mutex;
extern float last_motor_temperature;
extern float max_motor_temperature;
extern float motor_temperature_limit;


extern SemaphoreHandle_t motor_status_mutex;
extern uint8_t motor_status;
extern uint8_t motor_mode; // Default to position control

void read_temperature(void *pv);
#endif /*__READ_TEMPERATURE_TASK_H__*/