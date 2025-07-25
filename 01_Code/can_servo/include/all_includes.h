#ifndef __all_includes__
#define __all_includes__

#include <string>
#include <string.h>
#include <stdio.h>
#include <math.h>

enum motor_status{OVERTEMP, OVERCURRENT, AWAKE, SLEEP, ERROR};
enum motor_modes{POSITION_CONTROL, VELOCITY_CONTROL};
enum pid_message_types{PID_MESSAGE_P, PID_MESSAGE_I, PID_MESSAGE_D};



#define VCC 3.3f
#define COMMAND_ID_MASK 0x1F
#define ID_OFFSET 5
#define BYTES_IN_FLOAT 4
#define MAX_ADC_FLOAT 4095.0f
#define MAX_ADC_INT 4095

#define BYTE_0 0
#define BYTE_1 1
#define BYTE_2 2
#define BYTE_3 3
#define BYTE_4 4
#define BYTE_5 5
#define BYTE_6 6
#define BYTE_7 7
#define BYTE_8 8


#define READ_TEMPERATURE_STACK_SIZE 2048
#define READ_TEMPERATURE_PRIORITY 5

#define READ_CURRENT_STACK_SIZE 2048
#define READ_CURRENT_PRIORITY 5

#define READ_POSITION_STACK_SIZE 2048
#define READ_POSITION_PRIORITY 5

#define DRIVE_MOTOR_STACK_SIZE 2048
#define DRIVE_MOTOR_PRIORITY 5

#define CAN_BUS_STACK_SIZE 10000
#define CAN_BUS_PRIORITY 5

#define LED_STRIP_DRIVER_TASK_STACK_SIZE 2048
#define LED_STRIP_DRIVER_TASK_PRIORITY 10

#define MAIN_TASK_DELAY 1000



#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "pinout.h"

#include "servo_info.h"
#include "drivers/mp6550.h"
#include "drivers/can_servo.h"




/**
 * @brief map function to map a value from one range to another
 * 
 * @param a : numerical value to change the range of
 * @param b : the lower bound of the current range
 * @param c : the upper bound of the current range
 * @param w : the lower bound of the new range
 * @param q : the upper bound of the new range
 * 
 * @note No checking is done on the input or output side, if a is outside of [b,c] the results
 * will not be as expected, given that the input is also not an expected value
 */
#define map(a,b,c,w,q) ((w)+(((a) - (b)) * ((q) - (w)))/((c)-(b)))
#endif
