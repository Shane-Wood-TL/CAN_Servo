#include "../include/all_includes.h"

#include "../include/tasks/can_bus_task.h"
#include "../include/tasks/read_temperature_task.h"
#include "../include/tasks/read_current_task.h"
#include "../include/tasks/read_position_task.h"
#include "../include/tasks/drive_motor_task.h"
#include "../include/tasks/led_strip_driver_task.h"

#define CAN_BUS_TRANSMIT_TASK_STACK_SIZE 8096
#define CAN_BUS_TRANSMIT_TASK_PRIORITY 1

void can_bus_transmit_task(void *pv);
void self_test_task(void *pv);

SemaphoreHandle_t motor_speed_mutex;
int16_t motor_speed = 0;



SemaphoreHandle_t motor_status_mutex;
uint8_t motor_status = SLEEP;
uint8_t motor_mode = POSITION_CONTROL; // Default to position control


//Motor Measurement variables
SemaphoreHandle_t current_mutex;
float last_current_draw = 0;
float max_current_draw = 0;
float current_limit_value = 2.0f;


SemaphoreHandle_t temperature_mutex;
float last_motor_temperature = 0;
float max_motor_temperature = 0;
float motor_temperature_limit = 100;
float esp_core_temperature = 0;


// Motor Position Variables
SemaphoreHandle_t motor_offset_mutex;
float motor_offset_value=0;

SemaphoreHandle_t target_angle_velocity_mutex;
float target_angle = 180.0f;
float target_velocity = 0.0f;

SemaphoreHandle_t current_angle_velocity_mutex;
float current_angle = 0;
float current_velocity = 0;


SemaphoreHandle_t PID_values_mutex;
float pid_P = 50.0f;
float pid_I = 5.0f;
float pid_D = 3.0f;

SemaphoreHandle_t LED_RGB_values_mutex;
float led_r = 0.0f;
float led_g = 0.0f;
float led_b = 255.0f;


mp6550 motor(in1_pin, in2_pin, sleep_pin, current_sense_pin, current_sense_channel, LEDC_CHANNEL_0, LEDC_CHANNEL_1);


QueueHandle_t transmit_queue = NULL;

adc_oneshot_unit_handle_t adc_handle = NULL;

adc_oneshot_unit_init_cfg_t adc1_init_config = {
.unit_id = ADC_UNIT_1,
.clk_src = ADC_DIGI_CLK_SRC_XTAL,
.ulp_mode = ADC_ULP_MODE_DISABLE,
};



adc_oneshot_chan_cfg_t adc_config = {
.atten = ADC_ATTEN_DB_12,
.bitwidth = ADC_BITWIDTH_DEFAULT
};


SemaphoreHandle_t TWAI_mutex = NULL;

#define MOTOR_FREQUENCY 500

extern "C" void app_main(void)
{
    gpio_reset_pin(can_TX_pin);
    gpio_reset_pin(can_RX_pin);
    //Motor Status variables
    motor_speed_mutex                 = xSemaphoreCreateMutex();
    motor_status_mutex                = xSemaphoreCreateMutex();
    current_mutex                     = xSemaphoreCreateMutex();
    temperature_mutex                 = xSemaphoreCreateMutex();
    motor_offset_mutex                = xSemaphoreCreateMutex();
    target_angle_velocity_mutex       = xSemaphoreCreateMutex();
    current_angle_velocity_mutex      = xSemaphoreCreateMutex();
    PID_values_mutex                  = xSemaphoreCreateMutex();
    LED_RGB_values_mutex              = xSemaphoreCreateMutex();

    TWAI_mutex = xSemaphoreCreateMutex();
    transmit_queue = xQueueCreate(10, sizeof(twai_message_t));

    gpio_set_direction(in1_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(in2_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(sleep_pin, GPIO_MODE_OUTPUT);


    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_init_config, &adc_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, current_sense_channel, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, temp_sense_channel, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, position_channel, &adc_config));





    gpio_reset_pin(in1_pin);
    gpio_reset_pin(in2_pin);
    gpio_reset_pin(sleep_pin);

    
    gpio_set_direction(sleep_pin, GPIO_MODE_OUTPUT);

    ledc_channel_config_t in1_channel = {
        .gpio_num = in1_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = 0
    };

    ledc_timer_config_t in1_2_timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num  = LEDC_TIMER_0,
        .freq_hz    = MOTOR_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,   
    };

    ledc_channel_config_t in2_channel = {
        .gpio_num = in2_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = 0
    };

    ledc_timer_config(&in1_2_timer_config);
    ledc_channel_config(&in1_channel);
    ledc_channel_config(&in2_channel);



    xTaskCreate(read_temperature, "read_temperature", READ_TEMPERATURE_STACK_SIZE, NULL, READ_TEMPERATURE_PRIORITY, NULL);
    xTaskCreate(read_current, "read_current", READ_CURRENT_STACK_SIZE, NULL, READ_CURRENT_PRIORITY, NULL);
    xTaskCreate(read_position, "read_position", READ_POSITION_STACK_SIZE, NULL, READ_POSITION_PRIORITY, NULL);
    xTaskCreate(drive_motor, "drive_motor", DRIVE_MOTOR_STACK_SIZE, NULL, DRIVE_MOTOR_PRIORITY, NULL);
    xTaskCreate(can_bus, "can_bus", CAN_BUS_STACK_SIZE, NULL, CAN_BUS_PRIORITY, NULL);
    xTaskCreate(led_strip_driver_task, "led_strip_driver_task", LED_STRIP_DRIVER_TASK_STACK_SIZE, NULL, LED_STRIP_DRIVER_TASK_PRIORITY, NULL);
    //xTaskCreate(can_bus_transmit_task, "can_bus_transmit_task", CAN_BUS_TRANSMIT_TASK_STACK_SIZE, NULL, CAN_BUS_TRANSMIT_TASK_PRIORITY, NULL);

    xTaskCreate(self_test_task, "self_test_task", CAN_BUS_TRANSMIT_TASK_STACK_SIZE, NULL, CAN_BUS_TRANSMIT_TASK_PRIORITY, NULL);
    for(;;){
        vTaskDelay(MAIN_TASK_DELAY);
    }
}

void can_bus_transmit_task(void *pv){
    can_servo can_bus_driver(node_id);
    for(;;){
        // Transmit the current state of the servo
        twai_message_t received_message;
        if (xQueueReceive(transmit_queue, &received_message, pdMS_TO_TICKS(500))) {
            xSemaphoreTake(TWAI_mutex, portMAX_DELAY);
            esp_err_t result = twai_transmit(&received_message, pdMS_TO_TICKS(MAX_TWAI_TIMEOUT));
            xSemaphoreGive(TWAI_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}






void self_test_task(void *pv){
    
    vTaskDelay(pdMS_TO_TICKS(10000)); // Wait for system to stabilize
    twai_message_t received_message;
    received_message.identifier = (node_id << ID_OFFSET) | SET_MODE_COMMAND_ID;
    received_message.data_length_code = 2;
    received_message.data[0] = POSITION_CONTROL;
    received_message.data[1] = AWAKE;
    received_message.self = 1;
    printf("Sending wakeup and position control mode\n");

    xSemaphoreTake(TWAI_mutex, portMAX_DELAY);
    esp_err_t res = twai_transmit(&received_message, pdMS_TO_TICKS(MAX_TWAI_TIMEOUT));
    xSemaphoreGive(TWAI_mutex);
    if (res != ESP_OK) {
        printf("Transmit failed: %s\n", esp_err_to_name(res));
    }



    received_message.identifier = (node_id << ID_OFFSET) | SET_GOAL_POSITION_VELOCITY_COMMAND_ID;
    received_message.data_length_code = 4;
    union{
        float a;
        uint8_t bytes[BYTES_IN_FLOAT];     
    } temp_union;

    


    for(;;){
        temp_union.a = 260.0f;
        memcpy(&received_message.data[0], temp_union.bytes, BYTES_IN_FLOAT);
        xSemaphoreTake(TWAI_mutex, portMAX_DELAY);
        twai_transmit(&received_message, pdMS_TO_TICKS(MAX_TWAI_TIMEOUT));
        xSemaphoreGive(TWAI_mutex);
        printf("Sent position 260\n");
        vTaskDelay(pdMS_TO_TICKS(500));

        temp_union.a = 30.0f;
        memcpy(&received_message.data[0], temp_union.bytes, BYTES_IN_FLOAT);
        xSemaphoreTake(TWAI_mutex, portMAX_DELAY);
        twai_transmit(&received_message, pdMS_TO_TICKS(MAX_TWAI_TIMEOUT));
        xSemaphoreGive(TWAI_mutex);
        printf("Sent position 30\n");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}