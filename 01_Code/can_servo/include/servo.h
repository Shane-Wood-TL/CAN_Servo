#include "all_includes.h"


class servo{
    private:
        uint16_t target_position;
        uint8_t *motor_status;
        int16_t *motor_offset;
        float *current_limit;
        float *temperature_limit;
        bool *direction;
        float *time_to_move;
        uint16_t current_position;
        float current_temperature;

        bool over_temperature();
        bool over_current();
        
    public:
        servo(uint16_t target_position, uint8_t *motor_status, int16_t *motor_offset, float *current_limit, float *temperature_limit, bool *direction, float *time_to_move, uint16_t current_position, float current_temperature);
        void drive_motor();
};