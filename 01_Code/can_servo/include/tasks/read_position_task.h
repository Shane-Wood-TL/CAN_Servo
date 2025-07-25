#ifndef __READ_POSITION_TASK_H__
#define __READ_POSITION_TASK_H__

#include "../all_includes.h"

#define MAX_RAW_POSITION 3297
#define MIN_RAW_POSITION 0
#define MAX_ANGLE 360.0f
#define READ_POSITION_TASK_DELAY 10

extern SemaphoreHandle_t current_angle_velocity_mutex;
extern adc_oneshot_unit_handle_t adc_handle;
extern float current_angle;

void read_position(void *pv);
#endif /*__READ_POSITION_TASK_H__*/