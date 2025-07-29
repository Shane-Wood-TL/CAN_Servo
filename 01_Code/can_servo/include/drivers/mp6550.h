#ifndef __mp6550__
#define __mp6550__
#include "../all_includes.h"
#include "driver/gpio.h"

#define MP6550_CURRENT_SENSITIVITY 0.2f

// Motor Status variables
extern SemaphoreHandle_t motor_status_mutex;
extern uint8_t motor_status;
extern uint8_t motor_mode; // Default to position control


extern SemaphoreHandle_t motor_speed_mutex;
extern int16_t motor_speed;



// Motor Measurement variables
extern SemaphoreHandle_t current_mutex;
extern float last_current_draw;
extern float current_limit_value;

extern SemaphoreHandle_t temperature_mutex;
extern float last_motor_temperature;
extern float motor_temperature_limit;


// Motor Position Variables
extern SemaphoreHandle_t motor_offset_mutex;
extern float motor_offset_value;

extern SemaphoreHandle_t target_angle_velocity_mutex;
extern float target_angle;

extern SemaphoreHandle_t current_angle_velocity_mutex;
extern float current_angle;
extern float current_velocity;



class mp6550{
   private:
    gpio_num_t in1_p;
    gpio_num_t in2_p;
    gpio_num_t sleep_p;
    gpio_num_t current_sense_p;
    adc_channel_t current_sense_channel_c;
    ledc_channel_t in1_ledc_channel;
    ledc_channel_t in2_ledc_channel;

   public:
    mp6550(gpio_num_t in1_p, gpio_num_t in2_p, gpio_num_t sleep_p, gpio_num_t current_sense_p, adc_channel_t current_sense_channel_c, ledc_channel_t in1_ledc_channel, ledc_channel_t in2_ledc_channel);
    void driveMotor(int16_t speed);
    void sleep();
    void wake();
    float read_current();
    void break_motor();
};

#endif
