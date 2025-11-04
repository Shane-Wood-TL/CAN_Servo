#include "../../include/interpolators/stepped_interpolator.h"

stepped_interpolator::stepped_interpolator(float start, float step_size){
    current_position = start;
    target_position = start;
    this->step_size = step_size;
}

void stepped_interpolator::set_target(float target){
    target_position = target;
}

float stepped_interpolator::get_position(){
    if(current_position < target_position){
        current_position += step_size;
        if(current_position > target_position){
            current_position = target_position;
        }
    }else if(current_position > target_position){
        current_position -= step_size;
        if(current_position < target_position){
            current_position = target_position;
        }
    }
    return current_position;
}

bool stepped_interpolator::is_complete(){
    return current_position == target_position;
}

void stepped_interpolator::reset(){
    current_position = 0;
    target_position = 0;
}