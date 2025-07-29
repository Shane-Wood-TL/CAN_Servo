#ifndef __LED_STRIP_DRIVER_TASK_H__
#define __LED_STRIP_DRIVER_TASK_H__

#include "../all_includes.h"

#include "../drivers/led_strip_driver.h"



extern SemaphoreHandle_t LED_RGB_values_mutex;
extern float led_r;
extern float led_g;
extern float led_b;

void led_strip_driver_task(void *pv);
#endif /*__LED_STRIP_DRIVER_TASK_H__*/