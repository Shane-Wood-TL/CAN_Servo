#ifndef __can_servo__
#define __can_servo__

#include "all_includes.h"



// Motor Status variables
extern SemaphoreHandle_t motor_speed_mutex;
extern int16_t motor_speed;

extern SemaphoreHandle_t motor_status_mutex;
extern uint8_t motor_status;
extern uint8_t motor_mode;

// Motor Measurement variables
extern SemaphoreHandle_t current_mutex;
extern float last_current_draw;
extern float max_current_draw;
extern float current_limit_value;

extern SemaphoreHandle_t temperature_mutex;
extern float last_motor_temperature;
extern float max_motor_temperature;
extern float motor_temperature_limit;


// Motor Position Variables
extern SemaphoreHandle_t motor_offset_mutex;
extern float motor_offset_value;

extern SemaphoreHandle_t target_angle_velocity_mutex;
extern float target_angle;
extern float target_velocity;


extern SemaphoreHandle_t current_angle_velocity_mutex;
extern float current_angle;
extern float current_velocity;




class can_servo{
    public:
    enum CommandID {
        GET_INFO                   = 0x000,
        SET_MODE_STATE             = 0x002,
        GET_MODE_STATE             = 0x004,
        GET_POSITION_VELOCITY      = 0x006,
        GET_CURRENT_DRAW           = 0x008,
        GET_TEMPERATURE            = 0x00A,
        SET_GOAL_POSITION_VELOCITY = 0x00C,
        SET_OFFSET                 = 0x00E,
        SET_CURRENT_LIMIT          = 0x010,
        SET_TEMPERATURE_LIMIT      = 0x012,
        SET_LED                    = 0x014,
        UNUSED_0                   = 0x016,
        UNUSED_1                   = 0x018,
        UNUSED_2                   = 0x01A,
        UNUSED_3                   = 0x01C,
        UNUSED_4                   = 0x01E,
    };
    enum data_direction {
        to_host = 0x00,
        from_host = 0x01,
        none = 0x02

    };
    struct command{
        uint8_t id;
        uint8_t data_length;
        bool data_returned;
        data_direction direction;
    };
    command commandList[16] = {
        //name, data length, data returned, data direction
        {GET_INFO, 3, true, to_host},
        {SET_MODE_STATE, 2, false, from_host},
        {GET_MODE_STATE, 2, true, to_host},
        {GET_POSITION_VELOCITY, 8, true, to_host},
        {GET_CURRENT_DRAW, 4, true, to_host},
        {GET_TEMPERATURE, 4, true, to_host},
        {SET_GOAL_POSITION_VELOCITY, 4, false, from_host},
        {SET_OFFSET, 4, true, from_host},
        {SET_CURRENT_LIMIT, 4, false, from_host},
        {SET_TEMPERATURE_LIMIT, 4, false, from_host},
        {SET_LED, 4, false, from_host},
        {UNUSED_0, 0, false, none},
        {UNUSED_1, 0, false, none},
        {UNUSED_2, 0,false,none},
        {UNUSED_3, 0, false, none},
        {UNUSED_4, 0, false, none},
    };
    uint8_t id;
    void display_message(uint8_t *data, uint8_t length, uint16_t identifier);
    public:
        can_servo(uint8_t id);
        void send_message(const command to_send, const uint8_t* message_contents);
        void receive_message();
        

};


#endif
