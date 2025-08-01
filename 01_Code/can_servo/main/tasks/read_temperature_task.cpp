#include "../../include/tasks/read_temperature_task.h"


void read_temperature(void *pv){
    temperature_sensor_handle_t temp_handle = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));

    temperature_sensor_enable(temp_handle);
    int raw_temp_reading;
    for(;;){
       
        adc_oneshot_read(adc_handle, temp_sense_channel, &raw_temp_reading);
        float temp_resistance = THERMISTOR_RESISTOR_DIVIDER * ((MAX_ADC_FLOAT / (float)raw_temp_reading) - 1.0f);

        float logR2 = log(temp_resistance);
        float T =  1.0f / ( THERM_A + (THERM_B * logR2 ) + ( THERM_C * logR2 * logR2 * logR2) ) ;
        T = T - K_TO_C;

        xSemaphoreTake(temperature_mutex, portMAX_DELAY);
        last_motor_temperature = T;

        if(last_motor_temperature > max_motor_temperature){
            max_motor_temperature = last_motor_temperature;
        }
        //printf("Temperature: %f\n", T);

        if(last_motor_temperature > motor_temperature_limit){
            xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
            motor_status = OVERTEMP;
            xSemaphoreGive(motor_status_mutex);
        }
        

        float tsens_out;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
        esp_core_temperature = tsens_out;
        //printf("Temperature in %f °C\n", tsens_out);


        xSemaphoreGive(temperature_mutex);

        vTaskDelay(pdMS_TO_TICKS(READ_TEMPERATURE_TASK_DELAY));
    }
}
