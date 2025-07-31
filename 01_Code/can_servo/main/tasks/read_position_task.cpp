#include "../../include/tasks/read_position_task.h"


void read_position(void *pv){
    uint16_t MAX_RAW_POSITION = 3297;
    uint16_t MIN_RAW_POSITION = 0;
    int raw_position_reading;
    for(;;){
        xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
        adc_oneshot_read(adc_handle, position_channel, &raw_position_reading);
        printf("%d\n", raw_position_reading);
        if(raw_position_reading > MAX_RAW_POSITION){
            MAX_RAW_POSITION = raw_position_reading; //just to ensure correct behavoir across different setups
        }
        current_angle = raw_position_reading/(MAX_RAW_POSITION/MAX_ANGLE);
        //printf("Angle: %f\n", current_angle);
        //read position
        xSemaphoreGive(current_angle_velocity_mutex);
        vTaskDelay(pdMS_TO_TICKS(READ_POSITION_TASK_DELAY));
    }
}



