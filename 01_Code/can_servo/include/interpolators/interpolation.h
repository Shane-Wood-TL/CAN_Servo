#ifndef __interpolation__
#define __interpolation__
#include "esp_timer.h"

class interpolation{
    private:
        float current_position;
        float target_position;
        float start_position;
        float start_time;
        float current_duration;
        uint64_t lastTime;
    public:
        interpolation(float start);
        void set_target(float target, float duration);
        float get_position();
        bool is_complete();
        void reset();
};


#endif 