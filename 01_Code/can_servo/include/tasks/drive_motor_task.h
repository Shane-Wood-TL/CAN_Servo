#ifndef __DRIVE_MOTOR_TASK_H__
#define __DRIVE_MOTOR_TASK_H__

#include "../all_includes.h"
#include "../supporting/get_shortest_angle.h"
#include "../drivers/mp6550.h"
#include "../interpolators/interpolation.h"
#include "../interpolators/stepped_interpolator.h"

#define DRIVE_MOTOR_TASK_DELAY 10
#define INTEGRAL_ON_LARGE_CHANGE_RESET 50.0f
#define INTEGRAL_LIMIT 500.0f
#define ERROR_THRESHOLD 1.0f //angle
#define VELOCITY_ERROR_THRESHOLD 0.5 
#define M_to_S 1000.0f
#define LOW_PASS_FILTER_COEFFICIENT 0.5f
#define MAX_SPEED 4095

extern mp6550 motor;

extern SemaphoreHandle_t motor_status_mutex;
extern uint8_t motor_status;
extern uint8_t motor_mode; // Default to position control

extern SemaphoreHandle_t target_angle_velocity_mutex;
extern float target_angle;
extern float target_velocity;

extern SemaphoreHandle_t current_angle_velocity_mutex;
extern float current_angle;
extern float current_velocity;


// Motor Position Variables
extern SemaphoreHandle_t motor_offset_mutex;
extern float motor_offset_value;


extern SemaphoreHandle_t PID_values_mutex;
extern float pid_P;
extern float pid_I;
extern float pid_D;
void drive_motor(void *pv);
#endif /*__DRIVE_MOTOR_TASK_H__*/