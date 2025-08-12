#include "../../include/tasks/led_strip_driver_task.h"

#define max_led_brightness 128
void led_strip_driver_task(void *pv){
    led_strip_driver led_strip(ONBOARD_RGB_LED, 1);

    for(;;){

        xSemaphoreTake(motor_status_mutex, portMAX_DELAY);


        if(motor_status == OVERCURRENT or motor_status == ERROR){
            xSemaphoreGive(motor_status_mutex);
            led_strip.set_color(max_led_brightness,max_led_brightness,0, 0, 0);
        }else if (motor_status == OVERTEMP){
            xSemaphoreGive(motor_status_mutex);
            led_strip.set_color(max_led_brightness,0,0, 0, 0);
        }else if (motor_status == SLEEP){
            xSemaphoreGive(motor_status_mutex);
            led_strip.set_color(max_led_brightness,0,max_led_brightness, 0, 0);
        }else{
            xSemaphoreGive(motor_status_mutex);
            xSemaphoreTake(LED_RGB_values_mutex, portMAX_DELAY);
            led_strip.set_color(map(led_r,0,255,0,max_led_brightness),map(led_g,0,255,0,max_led_brightness),map(led_b,0,255,0,max_led_brightness), 0, 0);
            xSemaphoreGive(LED_RGB_values_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

