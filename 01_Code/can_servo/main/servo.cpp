#include "../include/servo.h"

servo::servo(uint16_t target_position, uint8_t *motor_status, int16_t *motor_offset, float *current_limit, float *temperature_limit, bool *direction, float *time_to_move, float current_position, float current_temperature){
    this->target_position = target_position;
    this->motor_status = motor_status;
    this->motor_offset = motor_offset;
    this->current_limit = current_limit;
    this->temperature_limit = temperature_limit;
    this->direction = direction;
    this->time_to_move = time_to_move;
    this->current_position = current_position;
    this->current_temperature = current_temperature;
}

bool servo::over_temperature(){
    return (current_temperature > *(temperature_limit));
}
void servo::drive_motor(){
    
}

