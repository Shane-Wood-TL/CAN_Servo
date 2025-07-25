#include "../../include/tasks/led_strip_driver_task.h"

void led_strip_driver_task(void *pv){
    led_strip_driver led_strip(ONBOARD_RGB_LED, 1);
    xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
    uint8_t old_motor_status = motor_status;
    xSemaphoreGive(motor_status_mutex);
    for(;;){

        xSemaphoreTake(motor_status_mutex, portMAX_DELAY);

        if(motor_status == OVERCURRENT and old_motor_status != OVERCURRENT){
            xSemaphoreGive(motor_status_mutex);
            led_strip.set_color(255,0,255, 0, 0);
            old_motor_status = OVERCURRENT;
        }else if (motor_status == OVERTEMP and old_motor_status != OVERTEMP){
            xSemaphoreGive(motor_status_mutex);
            led_strip.set_color(255,0,0, 0, 0);
            old_motor_status = OVERTEMP;
        }else if (motor_status == SLEEP and old_motor_status != SLEEP){
            xSemaphoreGive(motor_status_mutex);
            led_strip.set_color(255,0,0, 0, 0);
            old_motor_status = SLEEP;
        }else{
            xSemaphoreGive(motor_status_mutex);

            xSemaphoreTake(LED_RGB_values_mutex, portMAX_DELAY);
            led_strip.set_color(led_r,led_g,led_b, 0, 0);
            xSemaphoreGive(LED_RGB_values_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

