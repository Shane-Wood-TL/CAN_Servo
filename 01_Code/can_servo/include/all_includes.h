#ifndef __all_includes__
#define __all_includes__

#include <string>
#include <string.h>
#include <stdio.h>
#include <atomic>


enum motor_status{OVERTEMP, OVERCURRENT, AWAKE, SLEEP, ERROR};
enum motor_modes{POSITION_CONTROL, VELOCITY_CONTROL};
enum pid_message_types{PID_MESSAGE_P, PID_MESSAGE_I, PID_MESSAGE_D};
#define bytes_in_float 4

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
#include "mp6550.h"
#include "can_servo.h"




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
