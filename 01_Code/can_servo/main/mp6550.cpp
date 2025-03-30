#include "../include/mp6550.h"
mp6550::mp6550(gpio_num_t in1_p, gpio_num_t in2_p, gpio_num_t sleep_p, gpio_num_t current_sense_p, adc_channel_t current_sense_channel_c, ledc_channel_t in1_ledc_channel, ledc_channel_t in2_ledc_channel){
    this->in1_p = in1_p;
    this->in2_p = in2_p;
    this->sleep_p = sleep_p;
    this->current_sense_p = current_sense_p;
    this->current_sense_channel_c = current_sense_channel_c;
    this->in1_ledc_channel = in1_ledc_channel;
    this->in2_ledc_channel = in2_ledc_channel;
    wake();
}




void mp6550::driveMotor(int16_t speed){
    if(speed > 0){
        //forward
        ledc_set_duty(LEDC_LOW_SPEED_MODE, in1_ledc_channel, abs(speed));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, in1_ledc_channel);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, in2_ledc_channel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, in2_ledc_channel);
    }else if(speed < 0){
        //reverse
        ledc_set_duty(LEDC_LOW_SPEED_MODE, in2_ledc_channel, abs(speed));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, in2_ledc_channel);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, in1_ledc_channel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, in1_ledc_channel);
    }else{
        //stop
        ledc_set_duty(LEDC_LOW_SPEED_MODE, in2_ledc_channel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, in2_ledc_channel);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, in1_ledc_channel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, in1_ledc_channel);
    }
}

void mp6550::sleep(){
    gpio_set_level(this->sleep_p, 1);
}
void mp6550::wake(){
    gpio_set_level(this->sleep_p, 0);
}
float mp6550::read_current(){
    uint16_t raw_reading = 0;//adc1_get_raw(this->current_sense_channel_c);
    float vout = raw_reading * 3.3 / 4095;
    vout = vout / 0.2; //200mV/A sensitivity
    return vout;
}

void mp6550::update(){
    xSemaphoreTake(current_position_mutex, portMAX_DELAY);
    xSemaphoreTake(current_usage_mutex, portMAX_DELAY);
    xSemaphoreTake(goal_position_mutex, portMAX_DELAY);
    current_usage = read_current();
    xSemaphoreGive(current_usage_mutex);

    if(goal_position == current_position){
        break_motor();
    }else if(goal_position > current_position){
        driveMotor(goal_position-current_position);
    }else if(goal_position < current_position){
        driveMotor(current_position-goal_position);
    }

    xSemaphoreGive(current_position_mutex);
    xSemaphoreGive(goal_position_mutex);
}

void mp6550::break_motor(){
    driveMotor(0);
}