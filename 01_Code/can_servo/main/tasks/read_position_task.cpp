#include "../../include/tasks/read_position_task.h"
#include "../../include/supporting/running_average.h"

void read_position(void *pv){
    uint16_t MAX_RAW_POSITION = 3297;
    uint16_t MIN_RAW_POSITION = 0;
    int raw_position_reading;

    static float last_angle = 0.0f;
    static int64_t last_time_us = 0;

    running_average velocity_average;
    for(;;){
        xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
        adc_oneshot_read(adc_handle, position_channel, &raw_position_reading);
        //printf("%d\n", raw_position_reading);
        if(raw_position_reading > MAX_RAW_POSITION){
            MAX_RAW_POSITION = raw_position_reading; //just to ensure correct behavoir across different setups
        }
        current_angle = raw_position_reading/(MAX_RAW_POSITION/MAX_ANGLE);
        printf("Angle: %f\n", current_angle);
        //read position

        int64_t now = esp_timer_get_time(); // microseconds
        if (last_time_us > 0) {
            float dt = (now - last_time_us) / 1e6f; // convert to seconds
            if (dt > 0) {
                current_velocity = (current_angle - last_angle) / dt;
            }
            velocity_average.add_value(current_velocity);
            current_velocity = velocity_average.average();
            //printf("Angle: %f, Delta: %f, Velocity: %f, dt: %f\n", current_angle, current_angle - last_angle, current_velocity, dt);

        }
        
        last_angle = current_angle;
        last_time_us = now;


        xSemaphoreGive(current_angle_velocity_mutex);
        vTaskDelay(pdMS_TO_TICKS(READ_POSITION_TASK_DELAY));
    }
}



