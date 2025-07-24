
#include "../include/all_includes.h"
#include "../include/led_strip_driver.h"
#include <math.h>

void read_temperature(void *pv);
void read_position(void *pv);
void drive_motor(void *pv);
void can_bus(void *pv);
void read_current(void *pv);
void led_strip_driver_task(void *pv);

float get_shortest_angle(float target, float current);




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


SemaphoreHandle_t PID_values_mutex;
float pid_P = 85.0f;
float pid_I = 7.8f;
float pid_D = 30.0f;

SemaphoreHandle_t LED_RGB_values_mutex;
float led_r = 255.0f;
float led_g = 0.0f;
float led_b = 0.0f;


mp6550 motor(in1_pin, in2_pin, sleep_pin, current_sense_pin, current_sense_channel, LEDC_CHANNEL_0, LEDC_CHANNEL_1);




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




#define MOTOR_FREQUENCY 5000

extern "C" void app_main(void)
{
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
        


    for(;;){
        vTaskDelay(MAIN_TASK_DELAY);
    }
}







#define K_TO_C 273.15f
#define THERMISTOR_RESISTOR_DIVIDER 10000.0f
#define THERM_A 0.001129148f
#define THERM_B 0.000234125f
#define THERM_C 0.0000000876741f
#define READ_TEMPERATURE_TASK_DELAY 1000
void read_temperature(void *pv){
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
        
        xSemaphoreGive(temperature_mutex);

        vTaskDelay(pdMS_TO_TICKS(READ_TEMPERATURE_TASK_DELAY));
    }
}










#define MAX_RAW_POSITION 3297
#define MIN_RAW_POSITION 0
#define MAX_ANGLE 360.0f
#define READ_POSITION_TASK_DELAY 10

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







#define WINDOW_SIZE 10
class running_average{
    private:
        float buffer[WINDOW_SIZE] = {0};
        uint8_t index = 0;
        uint8_t count = 0;
        float sum = 0;
    public:
        void add_value(float new_value){
            if (count < WINDOW_SIZE){
                buffer[index] = new_value;
                sum += new_value;
                count++;
            } else {
                  sum -=  buffer[index];
                  buffer[index] = new_value;
                  sum += new_value;   
            }
            index += 1;
            if (index >= WINDOW_SIZE){
                index = 0;
            }
        }

        float average() {
            return (count == 0) ? 0.0f : sum / count;
        }
};









#define READ_CURRENT_TASK_DELAY 30
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
float get_shortest_angle(float target, float current) {
    return target - current;
}










#define INTEGRAL_LIMIT 100.0f
#define LOW_PASS_FILTER_COEFFICIENT 0.5f

#define MAX_ADC_INT 4095
#define DRIVE_MOTOR_TASK_DELAY 10
#define INTEGRAL_ON_LARGE_CHANGE_RESET 10.0f
#define INTEGRAL_LIMIT 100.0f
#define ERROR_THRESHOLD 1.0f
#define M_to_S 1000.0f

void drive_motor(void *pv){
    // PID state
    static float integral = 0.0f;
    static float last_error = 0.0f;

    int16_t last_motor_speed = 0;
    
    float last_target = 0.0f;

    bool is_sleeping = false;
    for(;;){          
        //drive motor
        xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
        uint8_t current_motor_status = motor_status;
        uint8_t current_motor_mode = motor_mode;
        xSemaphoreGive(motor_status_mutex);

        

        if(current_motor_status == OVERTEMP){
            motor.sleep();
            is_sleeping = true;
        }else if(current_motor_status == OVERCURRENT){
            motor.sleep();
            is_sleeping = true;
        }else if(current_motor_status == AWAKE){
            if(is_sleeping){
                motor.wake();
                is_sleeping = false;
            }
        }else if(current_motor_status == SLEEP){
            if(!is_sleeping){
                motor.sleep();
                is_sleeping = true;
            }
        }else if(current_motor_status == ERROR){
            motor.sleep();
            is_sleeping = true;
        }

        
                
        
        if (current_motor_status == AWAKE){
            if(current_motor_mode == VELOCITY_CONTROL){
                xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                last_motor_speed = (int16_t)map(target_velocity, -100.0f, 100.0f, -4095.0f, 4095.0f);
                xSemaphoreGive(target_angle_velocity_mutex);
            }else if(current_motor_mode == POSITION_CONTROL){

                xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
                xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                xSemaphoreTake(motor_offset_mutex, portMAX_DELAY);
                


                
                //printf("Current Angle: %f\n", current_angle);
                printf("Target Angle: %f\n", target_angle);
                //printf("Offset Value: %d\n", motor_offset_value);
                //float error = target_angle - current_angle;
                float error = get_shortest_angle(target_angle, current_angle);
                float dt = DRIVE_MOTOR_TASK_DELAY / M_to_S;  // time step in seconds
                // Integral with anti-windup
                integral += error * dt;
                if (integral > integral_limit) integral = INTEGRAL_LIMIT;
                else if (integral < -integral_limit) integral = -INTEGRAL_LIMIT;

                // Derivative
                float derivative = (error - last_error) / dt;
                static float last_filtered = 0.0f; // Last filtered value for derivative
                float filtered_derivative = LOW_PASS_FILTER_COEFFICIENT * derivative + (1 - LOW_PASS_FILTER_COEFFICIENT) * last_filtered;

                last_error = error;

                
                if (fabsf(last_target - target_angle) > INTEGRAL_ON_LARGE_CHANGE_RESET) {
                    integral = 0;
                }
                last_target = target_angle;

                xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                float output = pid_P * error + pid_I * integral + pid_D * filtered_derivative;
                xSemaphoreGive(PID_values_mutex);

                if(output > MAX_ADC_INT){
                    last_motor_speed = MAX_ADC_INT;
                }else if(output < -MAX_ADC_INT){
                    last_motor_speed = -MAX_ADC_INT;
                }else{
                    last_motor_speed = (int16_t)output;
                }
                // Stop near target
                if (fabsf(error) < ERROR_THRESHOLD) {
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

        vTaskDelay(pdMS_TO_TICKS(DRIVE_MOTOR_TASK_DELAY));        
    }
}

















#define TWAI_RX_QUEUE_SIZE 32
#define TWAI_RX_QUEUE_SIZE 1
#define CAN_BUS_TASK_DELAY 50

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
        .tx_queue_len = TWAI_TX_QUEUE_SIZE,
        .rx_queue_len = TWAI_RX_QUEUE_SIZE,
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
        vTaskDelay(pdMS_TO_TICKS(CAN_BUS_TASK_DELAY));
    }
}























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

