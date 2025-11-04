#ifndef __steppered_interpolator_h__
#define __steppered_interpolator_h__

class stepped_interpolator{
    public:
        stepped_interpolator(float start, float step_size);
        void set_target(float target);
        float get_position();
        bool is_complete();
        void reset();
    private:
        float current_position;
        float target_position;
        float step_size;
};

#endif