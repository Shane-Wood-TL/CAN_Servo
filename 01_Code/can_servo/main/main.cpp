
#include "../include/all_includes.h"
#include <math.h>

void read_temperature(void *pv);
void read_position(void *pv);
void drive_motor(void *pv);
void can_bus(void *pv);

SemaphoreHandle_t current_current_mutex;
float last_current_draw = 0;

SemaphoreHandle_t current_temperature_mutex;
float last_motor_temperature = 0;

SemaphoreHandle_t current_position_mutex;
uint16_t last_positon_reading = 0;

SemaphoreHandle_t time_to_move_mutex;
float time_to_move_value = 0;

SemaphoreHandle_t direction_mutex;
uint8_t direction = FORWARD;

SemaphoreHandle_t temperature_limit_mutex;
float motor_temperature_limit = 100;

SemaphoreHandle_t current_limit_mutex;
float current_limit_value = 2.0f;

SemaphoreHandle_t motor_offset_mutex;
int32_t motor_offset_value=0;


SemaphoreHandle_t motor_status_mutex;
uint8_t motor_status = SLEEP;

SemaphoreHandle_t target_position_mutex;
uint16_t target_position;


adc_oneshot_unit_handle_t adc_handle = NULL;

adc_oneshot_unit_init_cfg_t adc1_init_config = {
.unit_id = ADC_UNIT_1,
.ulp_mode = ADC_ULP_MODE_DISABLE,
};



adc_oneshot_chan_cfg_t adc_config = {
.atten = ADC_ATTEN_DB_12,
.bitwidth = ADC_BITWIDTH_DEFAULT
};




extern "C" void app_main(void)
{
    current_current_mutex = xSemaphoreCreateMutex();
    current_temperature_mutex = xSemaphoreCreateMutex();
    current_position_mutex = xSemaphoreCreateMutex();
    time_to_move_mutex = xSemaphoreCreateMutex();
    direction_mutex = xSemaphoreCreateMutex();
    temperature_limit_mutex = xSemaphoreCreateMutex();
    current_limit_mutex = xSemaphoreCreateMutex();
    motor_offset_mutex = xSemaphoreCreateMutex();
    motor_status_mutex = xSemaphoreCreateMutex();
    target_position_mutex = xSemaphoreCreateMutex();



    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_init_config, &adc_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, current_sense_channel, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, temp_sense_channel, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, position_channel, &adc_config));



    xTaskCreate(read_temperature, "read_temperature", 2048, NULL, 5, NULL);
    xTaskCreate(read_position, "read_position", 2048, NULL, 5, NULL);
    //xTaskCreate(drive_motor, "drive_motor", 2048, NULL, 5, NULL);
    //xTaskCreate(can_bus, "can_bus", 2048, NULL, 5, NULL);

    int adc_read0, adc_read1;
    int mv_output;

    
    for(;;){
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_read0));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_4, &adc_read1));
        //printf("Adc channel-0 raw read result %d \n", adc_read0);
        //printf("Adc channel-1 raw read result %d \n", adc_read1);
        //printf("\n\n");
        //adc_cali_raw_to_voltage(cali_handle, adc_read1, &mv_output);
        //printf("ADC milivold output %d \n", mv_output);

        
        vTaskDelay(100);
    }
}

void read_temperature(void *pv){
    int raw_temp_reading;
    for(;;){
        xSemaphoreTake(current_temperature_mutex, portMAX_DELAY);
        adc_oneshot_read(adc_handle, temp_sense_channel, &raw_temp_reading);
        float temp_resistance = 10000.0f * ((4095.0f / (float)raw_temp_reading) - 1.0f);

        float logR2 = log(temp_resistance);
        float T =  1.0f / ( 0.001129148 + ( 0.000234125 * logR2 ) + ( 0.0000000876741 * logR2 * logR2 * logR2) ) ;
        T = T - 273.15;

        //printf("Temperature: %f\n", T);
        xSemaphoreGive(current_temperature_mutex);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void read_position(void *pv){
    int raw_position_reading;
    for(;;){
        xSemaphoreTake(current_position_mutex, portMAX_DELAY);
        adc_oneshot_read(adc_handle, position_channel, &raw_position_reading);
        printf("Position: %d\n", raw_position_reading);
        //read position
        xSemaphoreGive(current_position_mutex);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void drive_motor(void *pv){
    ledc_channel_config_t in1_channel = {
        .gpio_num = in1_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ledc_timer_config_t in1_timer_config = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT,
    .timer_num  = LEDC_TIMER_0,
    .freq_hz    = 100,              
    };

    ledc_timer_config(&in1_timer_config);
    ledc_channel_config(&in1_channel);
    //mp6550(gpio_num_t, gpio_num_t, gpio_num_t, gpio_num_t, adc_channel_t, ledc_channel_t, ledc_channel_t);
    mp6550 (in1_pin, in2_pin, sleep_pin, current_sense_pin, current_sense_channel, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
    
    
    for(;;){       
        //drive motor
        //motor.update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void can_bus(void *pv){
    can_servo can_bus_driver(node_id);
    for(;;){
        can_bus_driver.receiveMessage();
        
    }
}