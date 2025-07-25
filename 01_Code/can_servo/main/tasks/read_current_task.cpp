#include "../../include/tasks/read_current_task.h"


void read_current(void *pv){
    running_average current_average;
    for(;;){
        xSemaphoreTake(current_mutex, portMAX_DELAY);
        current_average.add_value(motor.read_current());
        if(current_average.average() > current_limit_value){
            xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
            motor_status = OVERCURRENT;
            xSemaphoreGive(motor_status_mutex);
        }
        xSemaphoreGive(current_mutex);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}




