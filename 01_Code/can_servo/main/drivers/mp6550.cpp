#include "../../include/drivers/mp6550.h"

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
    //printf("Speed: %d\n", speed);
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
    gpio_set_level(this->sleep_p, false);
}
void mp6550::wake(){
    gpio_set_level(this->sleep_p, true);
}
float mp6550::read_current(){
    int raw_current_reading;
    adc_oneshot_read(adc_handle, current_sense_channel_c, &raw_current_reading);
    //uint16_t raw_reading = adc_get_raw(this->current_sense_channel_c);
    float vout = raw_current_reading * VCC / MAX_ADC_INT;
    vout = vout / MP6550_CURRENT_SENSITIVITY; //200mV/A sensitivity
    return vout;
}


void mp6550::break_motor(){
    driveMotor(0);
}
