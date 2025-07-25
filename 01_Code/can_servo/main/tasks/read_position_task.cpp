#include "../../include/tasks/read_position_task.h"


void read_position(void *pv){
    int raw_position_reading;
    for(;;){
        xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
        adc_oneshot_read(adc_handle, position_channel, &raw_position_reading);
        current_angle = raw_position_reading/(MAX_RAW_POSITION/MAX_ANGLE);
        //printf("Angle: %f\n", current_angle);
        //read position
        xSemaphoreGive(current_angle_velocity_mutex);
        vTaskDelay(pdMS_TO_TICKS(READ_POSITION_TASK_DELAY));
    }
}



