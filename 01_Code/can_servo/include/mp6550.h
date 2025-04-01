#ifndef __mp6550__
#define __mp6550__
#include "all_includes.h"


extern float current_position;
extern float goal_position;
extern float current_usage;
extern SemaphoreHandle_t current_position_mutex;
extern SemaphoreHandle_t goal_position_mutex;
extern SemaphoreHandle_t current_usage_mutex;

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
    void update();
};

#endif