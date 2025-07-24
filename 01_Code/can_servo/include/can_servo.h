#ifndef __can_servo__
#define __can_servo__

#include "all_includes.h"

#define MAX_TWAI_TIMEOUT 1000
#define MAX_COMMAND_COUNT 16

#define GET_INFO_COMMAND_LENGTH 3
#define SET_MODE_SET_COMMAND_LENGTH 2
#define GET_MODE_SET_COMMAND_LENGTH 2
#define GET_POSITION_VELOCITY_COMMAND_LENGTH 8
#define GET_CURRENT_DRAW_COMMAND_LENGTH 4
#define GET_TEMPERATURE_COMMAND_LENGTH 4
#define SET_GOAL_POSITION_VELOCITY_COMMAND_LENGTH 4
#define SET_OFFSET_COMMAND_LENGTH 4
#define SET_CURRENT_LIMIT_COMMAND_LENGTH 4
#define SET_TEMPERATURE_LIMIT_COMMAND_LENGTH 4
#define SET_LED_COMMAND_LENGTH 4
#define SET_PID_COMMAND_LENGTH 5
#define UNUSED_1_COMMAND_LENGTH 0
#define UNUSED_2_COMMAND_LENGTH 0
#define UNUSED_3_COMMAND_LENGTH 0
#define UNUSED_4_COMMAND_LENGTH 0

#define GET_INFO_COMMAND_ID 0x000
#define SET_MODE_SET_COMMAND_ID 0x002
#define GET_MODE_SET_COMMAND_ID 0x004
#define GET_POSITION_VELOCITY_COMMAND_ID 0x006
#define GET_CURRENT_DRAW_COMMAND_ID 0x008
#define GET_TEMPERATURE_COMMAND_ID 0x00A
#define SET_GOAL_POSITION_VELOCITY_COMMAND_ID 0x00C
#define SET_OFFSET_COMMAND_ID 0x00E
#define SET_CURRENT_LIMIT_COMMAND_ID 0x010
#define SET_TEMPERATURE_LIMIT_COMMAND_ID 0x012
#define SET_LED_COMMAND_ID 0x014
#define SET_PID_COMMAND_ID 0x016
#define UNUSED_1_COMMAND_ID 0x018
#define UNUSED_2_COMMAND_ID 0x01A
#define UNUSED_3_COMMAND_ID 0x01C
#define UNUSED_4_COMMAND_ID 0x01E

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

extern SemaphoreHandle_t PID_values_mutex;
extern float pid_P;
extern float pid_I;
extern float pid_D;

extern SemaphoreHandle_t LED_RGB_values_mutex;
extern float led_r;
extern float led_g;
extern float led_b;
 
class can_servo{
    public:
    enum CommandID {
        GET_INFO                   = GET_INFO_COMMAND_ID,
        SET_MODE_STATE             = SET_MODE_SET_COMMAND_ID,
        GET_MODE_STATE             = GET_MODE_SET_COMMAND_ID,
        GET_POSITION_VELOCITY      = GET_POSITION_VELOCITY_COMMAND_ID,
        GET_CURRENT_DRAW           = GET_CURRENT_DRAW_COMMAND_ID,
        GET_TEMPERATURE            = GET_TEMPERATURE_COMMAND_ID,
        SET_GOAL_POSITION_VELOCITY = SET_GOAL_POSITION_VELOCITY_COMMAND_ID,
        SET_OFFSET                 = SET_OFFSET_COMMAND_ID,
        SET_CURRENT_LIMIT          = SET_CURRENT_LIMIT_COMMAND_ID,
        SET_TEMPERATURE_LIMIT      = SET_TEMPERATURE_LIMIT_COMMAND_ID,
        SET_LED                    = SET_LED_COMMAND_ID,
        SET_PID                    = SET_PID_COMMAND_ID,
        UNUSED_1                   = UNUSED_1_COMMAND_ID,
        UNUSED_2                   = UNUSED_2_COMMAND_ID,
        UNUSED_3                   = UNUSED_3_COMMAND_ID,
        UNUSED_4                   = UNUSED_4_COMMAND_ID,
    };
    enum data_direction {to_host = 0x00, from_host, none};

    struct command{
        uint8_t id;
        uint8_t data_length;
        bool data_returned;
        data_direction direction;
    };
    command commandList[MAX_COMMAND_COUNT] = {
        //name, data length, data returned, data direction
        {GET_INFO, GET_INFO_COMMAND_LENGTH, true, to_host},
        {SET_MODE_STATE, SET_MODE_SET_COMMAND_LENGTH, false, from_host},
        {GET_MODE_STATE, GET_MODE_SET_COMMAND_LENGTH, true, to_host},
        {GET_POSITION_VELOCITY, GET_POSITION_VELOCITY_COMMAND_LENGTH, true, to_host},
        {GET_CURRENT_DRAW, GET_CURRENT_DRAW_COMMAND_LENGTH, true, to_host},
        {GET_TEMPERATURE, GET_TEMPERATURE_COMMAND_LENGTH, true, to_host},
        {SET_GOAL_POSITION_VELOCITY, SET_GOAL_POSITION_VELOCITY_COMMAND_LENGTH, false, from_host},
        {SET_OFFSET, SET_OFFSET_COMMAND_LENGTH, true, from_host},
        {SET_CURRENT_LIMIT, SET_CURRENT_LIMIT_COMMAND_LENGTH, false, from_host},
        {SET_TEMPERATURE_LIMIT, SET_TEMPERATURE_LIMIT_COMMAND_LENGTH, false, from_host},
        {SET_LED, SET_LED_COMMAND_LENGTH, false, from_host},
        {SET_PID, SET_PID_COMMAND_LENGTH, false, from_host},
        {UNUSED_1, UNUSED_1_COMMAND_LENGTH, false, none},
        {UNUSED_2, UNUSED_2_COMMAND_LENGTH,false,none},
        {UNUSED_3, UNUSED_3_COMMAND_LENGTH, false, none},
        {UNUSED_4, UNUSED_4_COMMAND_LENGTH , false, none},
    };
    uint8_t id;
    void display_message(uint8_t *data, uint8_t length, uint16_t identifier);
    public:
        can_servo(uint8_t id);
        void send_message(const command to_send, const uint8_t* message_contents);
        void receive_message();
        

};


#endif
