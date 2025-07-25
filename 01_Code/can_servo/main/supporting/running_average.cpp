#include "../../include/supporting/running_average.h"

void running_average::add_value(float new_value){
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

float running_average::average() {
    return (count == 0) ? 0.0f : sum / count;
}
