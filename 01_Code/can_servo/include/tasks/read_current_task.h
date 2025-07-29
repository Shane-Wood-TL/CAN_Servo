#ifndef __READ_CURRENT_TASK_H__
#define __READ_CURRENT_TASK_H__


#include "../all_includes.h"

#include "../supporting/running_average.h"
#include "../drivers/mp6550.h"
#define READ_CURRENT_TASK_DELAY 30


extern adc_oneshot_unit_handle_t adc_handle;
extern mp6550 motor;

extern SemaphoreHandle_t current_mutex;
extern float last_current_draw0;
extern float max_current_draw;
extern float current_limit_value;


void read_current(void *pv);


#endif /*__READ_CURRENT_TASK_H__*/