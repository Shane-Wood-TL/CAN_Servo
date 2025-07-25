#ifndef __DRIVE_MOTOR_TASK_H__
#define __DRIVE_MOTOR_TASK_H__

#include "../all_includes.h"
#include "../supporting/get_shortest_angle.h"


#define DRIVE_MOTOR_TASK_DELAY 10
#define INTEGRAL_ON_LARGE_CHANGE_RESET 10.0f
#define INTEGRAL_LIMIT 100.0f
#define ERROR_THRESHOLD 1.0f
#define M_to_S 1000.0f
#define INTEGRAL_LIMIT 100.0f
#define LOW_PASS_FILTER_COEFFICIENT 0.5f


extern mp6550 motor;
void drive_motor(void *pv);
#endif /*__DRIVE_MOTOR_TASK_H__*/