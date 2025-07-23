
#include "../include/all_includes.h"
#include "../include/led_strip_driver.h"
#include <math.h>

void read_temperature(void *pv);
void read_position(void *pv);
void drive_motor(void *pv);
void can_bus(void *pv);
void read_current(void *pv);

float get_shortest_angle(float target, float current);




led_strip_driver* led_strip = nullptr;


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



// Motor Position Variables
SemaphoreHandle_t motor_offset_mutex;
float motor_offset_value=0;

SemaphoreHandle_t target_angle_velocity_mutex;
float target_angle = 180.0f;
float target_velocity = 0.0f;

SemaphoreHandle_t current_angle_velocity_mutex;
float current_angle = 0;
float current_velocity = 0;

std::atomic<bool> is_over_current = false;
std::atomic<bool> is_over_temp = false;

SemaphoreHandle_t PID_values_mutex;
float pid_P = 85.0f;
float pid_I = 7.8f;
float pid_D = 30.0f;

mp6550 motor(in1_pin, in2_pin, sleep_pin, current_sense_pin, current_sense_channel, LEDC_CHANNEL_0, LEDC_CHANNEL_1);




adc_oneshot_unit_handle_t adc_handle = NULL;

adc_oneshot_unit_init_cfg_t adc1_init_config = {
.unit_id = ADC_UNIT_1,
.ulp_mode = ADC_ULP_MODE_DISABLE,
};



adc_oneshot_chan_cfg_t adc_config = {
.atten = ADC_ATTEN_DB_12,
.bitwidth = ADC_BITWIDTH_DEFAULT
};



#define max_raw_position 3297
#define min_raw_position 0

extern "C" void app_main(void)
{
    //Motor Status variables
    motor_speed_mutex        = xSemaphoreCreateMutex();
    motor_status_mutex       = xSemaphoreCreateMutex();
    current_mutex            = xSemaphoreCreateMutex();
    temperature_mutex        = xSemaphoreCreateMutex();
    motor_offset_mutex       = xSemaphoreCreateMutex();
    target_angle_velocity_mutex       = xSemaphoreCreateMutex();
    current_angle_velocity_mutex      = xSemaphoreCreateMutex();


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

    //gpio_reset_pin(can_RX_pin);
    //gpio_reset_pin(can_TX_pin);
    
    gpio_set_direction(sleep_pin, GPIO_MODE_OUTPUT);

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
    .freq_hz    = 5000,              
    };

    ledc_channel_config_t in2_channel = {
        .gpio_num = in2_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ledc_timer_config(&in1_timer_config);
    ledc_channel_config(&in1_channel);
    ledc_channel_config(&in2_channel);






    //xTaskCreate(read_temperature, "read_temperature", 2048, NULL, 5, NULL);
    xTaskCreate(read_current, "read_current", 2048, NULL, 5, NULL);
    xTaskCreate(read_position, "read_position", 2048, NULL, 5, NULL);
    xTaskCreate(drive_motor, "drive_motor", 2048, NULL, 5, NULL);
    xTaskCreate(can_bus, "can_bus", 10000, NULL, 5, NULL);

    int adc_read0, adc_read1;
    int mv_output;

    

    led_strip = new led_strip_driver(ONBOARD_RGB_LED, 1);

    for(;;){
        //ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_read0));
        //ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_4, &adc_read1));
        //printf("Adc channel-0 raw read result %d \n", adc_read0);
        //printf("Adc channel-1 raw read result %d \n", adc_read1);
        //printf("\n\n");
        //adc_cali_raw_to_voltage(cali_handle, adc_read1, &mv_output);
        //printf("ADC milivold output %d \n", mv_output);
        //led_strip->set_color(255,255,255, 0, 0);
        //R = G
        //G = R
        //B = B
        vTaspid_Delay(100);
    }
}

#define MAX_ADC_FLOAT 4095.0f
#define K_TO_C 273.15f

void read_temperature(void *pv){
    int raw_temp_reading;
    for(;;){
        xSemaphoreTake(temperature_mutex, portMAX_DELAY);
        adc_oneshot_read(adc_handle, temp_sense_channel, &raw_temp_reading);
        float temp_resistance = 10000.0f * ((MAX_ADC_FLOAT / (float)raw_temp_reading) - 1.0f);

        float logR2 = log(temp_resistance);
        float T =  1.0f / ( 0.001129148 + ( 0.000234125 * logR2 ) + ( 0.0000000876741 * logR2 * logR2 * logR2) ) ;
        T = T - K_TO_C;

        //printf("Temperature: %f\n", T);
        xSemaphoreGive(temperature_mutex);
        vTaspid_Delay(pdMS_TO_TICKS(1000));
    }
}


void read_position(void *pv){
    int raw_position_reading;
    for(;;){
        xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
        adc_oneshot_read(adc_handle, position_channel, &raw_position_reading);
        current_angle = raw_position_reading/(max_raw_position/360.0f);
        //printf("Angle: %f\n", current_angle);
        //read position
        xSemaphoreGive(current_angle_velocity_mutex);
        vTaspid_Delay(pdMS_TO_TICKS(10));
    }
}

class running_average{
    private:
        const uint8_t window_size = 10;
        float buffer[window_size] = {0};
        uint8_t index = 0;
        uint8_t count = 0;
        float sum = 0;
    public:
        void add_value(float new_value){
            if (count < window_size){
                buffer[index] = new_value;
                sum += new_value;
                count++;
            } else {
                  sum -=  buffer[index];
                  buffer[index] = value;
                  sum += value;   
            }
            index += 1;
            if (index >= window_size){
                index = 0;
            }
        }

        float average() {
            return (count == 0) ? 0.0f : sum / count;
        }
   
}

void read_current(void *pv){
    running_average current_average;
    for(;;){
        xSemaphoreTake(current_mutex, portMAX_DELAY);
        current_average.add_value(motor.read_current());
        if(current_average.average() > current_limit_value){
            is_over_current = true;
            xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
            motor_status = OVERCURRENT;
            xSemaphoreGive(motor_status_mutex);
        }
        xSemaphoreGive(current_mutex);
        vTaspid_Delay(pdMS_TO_TICKS(30));
    }
}
float get_shortest_angle(float target, float current) {
    return target - current;
}




void drive_motor(void *pv){
    // PID state
    static float integral = 0.0f;
    static float last_error = 0.0f;

    uint8_t last_motor_status = AWAKE;
    motor.wake();
    int16_t last_motor_speed = 0;
    float last_target = 0.0f;
    for(;;){          
        //drive motor


        // if(is_over_temp){
        //     xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
        //     last_motor_status = SLEEP;
        //     motor.sleep();
        //     xSemaphoreGive(motor_status_mutex);
        // }else if(is_over_current){
        //     xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
        //     motor_status = OVERCURRENT;
        //     last_motor_status = SLEEP;
        //     motor.sleep();
        //     xSemaphoreGive(motor_status_mutex);
        // }else{
        //     xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
        //     if(motor_status == SLEEP and last_motor_status != SLEEP){
        //         motor.sleep();
        //         last_motor_status = SLEEP;

        //         xSemaphoreGive(motor_status_mutex);
        //     }else if(motor_status == AWAKE and last_motor_status != AWAKE){
        //         motor.wake();
        //         last_motor_status = AWAKE;

        //         xSemaphoreGive(motor_status_mutex);
        //     }else{
        //         xSemaphoreGive(motor_status_mutex);
        //     }
        // }
        

        if (last_motor_status == AWAKE) {
        xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
            if(motor_mode == VELOCITY_CONTROL){
                xSemaphoreGive(motor_status_mutex);
                xSemaphoreTake(motor_speed_mutex, portMAX_DELAY);
                last_motor_speed = motor_speed;
                xSemaphoreGive(motor_speed_mutex);



            }else if(motor_mode == POSITION_CONTROL){
                xSemaphoreGive(motor_status_mutex);
                xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
                xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                xSemaphoreTake(motor_offset_mutex, portMAX_DELAY);
                //printf("Current Angle: %f\n", current_angle);
                printf("Target Angle: %f\n", target_angle);
                //printf("Offset Value: %d\n", motor_offset_value);
                //float error = target_angle - current_angle;
                float error = get_shortest_angle(target_angle, current_angle);
                float dt = 0.01f;  // time step in seconds
                // Integral with anti-windup
                integral += error * dt;
                float integral_limit = 100.0f;  // limit integral
                if (integral > integral_limit) integral = integral_limit;
                else if (integral < -integral_limit) integral = -integral_limit;

                // Derivative
                float derivative = (error - last_error) / dt;
                float alpha = 0.5f; // Low-pass filter coefficient
                static float last_filtered = 0.0f; // Last filtered value for derivative
                float filtered_derivative = alpha * derivative + (1 - alpha) * last_filtered;

                last_error = error;

                
                if (fabsf(last_target - target_angle) > 10.0f) {
                    integral = 0;
                }
                last_target = target_angle;

                float output = pid_P * error + pid_I * integral + pid_D * filtered_derivative;

                if(output > 4095){
                    last_motor_speed = 4095;
                }else if(output < -4095){
                    last_motor_speed = -4095;
                }else{
                    last_motor_speed = (int16_t)output;
                }
                // Stop near target
                if (fabsf(error) < 1.0f) {
                    last_motor_speed = 0;
                    motor.break_motor();
                }

                xSemaphoreGive(motor_offset_mutex);
                xSemaphoreGive(target_angle_velocity_mutex);
                xSemaphoreGive(current_angle_velocity_mutex);
                //printf("angle: %f, target: %f, error: %f, output: %f\n", current_angle, target_angle, error, output);  
            }
            motor.driveMotor(last_motor_speed);
            
        }
        vTaspid_Delay(pdMS_TO_TICKS(10));        
    }
}


void can_bus(void *pv){


    static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


    // static const twai_filter_config_t f_config = {
    //     .acceptance_code = (node_id << 5),     // Match the node ID bits
    //     .acceptance_mask = (uint32_t)(~0x1F),            // Mask: only match top 6+ bits, ignore lower 5
    //     .single_filter = true
    // };


    static const twai_general_config_t g_config = {
        .controller_id = 0,
        .mode = TWAI_MODE_NORMAL,
        .tx_io = can_TX_pin,
        .rx_io = can_RX_pin,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 1,
        .rx_queue_len = 32,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0,
        .intr_flags = 0
    };

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        assert(false);
    }

    while (twai_start() != ESP_OK)
    {
        assert(false);
    }

    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
    twai_reconfigure_alerts(alerts_to_enable, NULL);

    can_servo can_bus_driver(node_id);
    for(;;){
        can_bus_driver.receive_message();
        vTaspid_Delay(pdMS_TO_TICKS(50));
    }
}
