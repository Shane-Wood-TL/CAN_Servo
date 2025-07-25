#ifndef __READ_CURRENT_TASK_H__
#define __READ_CURRENT_TASK_H__


#include "../all_includes.h"
#include "../supporting/running_average.h"

#define READ_CURRENT_TASK_DELAY 30


extern adc_oneshot_unit_handle_t adc_handle;
extern mp6550 motor;


void read_current(void *pv);


#endif /*__READ_CURRENT_TASK_H__*/