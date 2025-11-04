#include "../../include/interpolators/interpolation.h"


interpolation::interpolation(float start){
    current_position = start;
    target_position = start;
    start_position = start;
    start_time = esp_timer_get_time();
    current_duration = 0;
    lastTime = 0;
}

void interpolation::set_target(float target, float duration){
    target_position = target;
    current_duration = duration;
    start_time = esp_timer_get_time();
    lastTime = start_time;
}

float interpolation::get_position(){
    uint64_t currentTime = esp_timer_get_time();
    float time = (currentTime - start_time)/1000000.0f;
    lastTime = time;
    if(time > current_duration){
        current_position = target_position;
        start_position = current_position;
    }else if(target_position == current_position){
        start_position = current_position;
        return current_position;
    }else{
        current_position = start_position + (target_position - start_position) * time / current_duration;
    }
    return current_position;
}

bool interpolation::is_complete(){
    if(current_position == target_position){
        start_position = current_position;
        return true;
    }else{
        return false;
    }
}

void interpolation::reset(){
    current_position = 0;
    target_position = 0;
    start_position = 0;
    start_time = esp_timer_get_time();
    current_duration = 0;
    lastTime = 0;
}