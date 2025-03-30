#ifndef __all_includes__
#define __all_includes__

#include <string>
#include <string.h>
#include <stdio.h>


enum direction{FORWARD, BACKWARD};
enum status{OVERTEMP, OVERCURRENT, MOVE, HOLD, SLEEP};

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
#endif