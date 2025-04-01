#ifndef __can_servo__
#define __can_servo__

#include "all_includes.h"

//values to transmit
extern SemaphoreHandle_t current_current_mutex;
extern float last_current_draw;

extern SemaphoreHandle_t current_temperature_mutex;
extern float last_motor_temperature;

extern SemaphoreHandle_t current_position_mutex;
extern float last_positon_reading;


//values to receive
extern SemaphoreHandle_t time_to_move_mutex;
extern float time_to_move_value;

extern SemaphoreHandle_t direction_mutex;
extern uint8_t direction;

extern SemaphoreHandle_t temperature_limit_mutex;
extern float motor_temperature_limit;

extern SemaphoreHandle_t current_limit_mutex;
extern float current_limit_value;

extern SemaphoreHandle_t motor_offset_mutex;
extern int32_t motor_offset;


extern SemaphoreHandle_t motor_status_mutex;
extern uint8_t motor_status;

extern SemaphoreHandle_t target_position_mutex;
extern uint16_t target_position;

class can_servo{
    private:
    enum CommandID {
        GET_INFO = 0x000,
        HEARTBEAT = 0x001,
        ESTOP = 0x002,
        GET_STATE = 0x003,
        GET_POSITION = 0x004,
        GET_CURRENT_DRAW = 0x005,
        GET_TEMPERATURE = 0x006,
        GET_GOAL_POSITION = 0x007,
        SET_OFFSET = 0x008,
        SET_CURRENT_LIMIT = 0x009,
        SET_TEMPERATURE_LIMIT = 0x010,
        SET_DIRECTION = 0x011,
        SET_GOAL_ANGLE = 0x012,
        SET_MOVEMENT = 0x013,
        BRAKE = 0x014,
        SLEEP = 0x015,
        RELEASE = 0x016,
        AWAKE = 0x017
    };
    enum State {
        IDLE = 0x00,
        MOVING = 0x01,
        BRAKING = 0x02,
        ERROR = 0x03
    };
    enum Direction {
        CLOCKWISE = 0x00,
        COUNTERCLOCKWISE = 0x01
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
    command commandList[18] = {
        {GET_INFO, 3, true, to_host},
        {HEARTBEAT, 0, false, none},
        {ESTOP, 0, false, none},
        {GET_STATE, 1, true, to_host},
        {GET_POSITION, 2, true, to_host},
        {GET_CURRENT_DRAW, 4, true, to_host},
        {GET_TEMPERATURE, 4, true, to_host},
        {GET_GOAL_POSITION, 2, true, to_host},
        {SET_OFFSET, 2, false, from_host},
        {SET_CURRENT_LIMIT, 4, false, from_host},
        {SET_TEMPERATURE_LIMIT, 4, false, from_host},
        {SET_DIRECTION, 1, false, from_host},
        {SET_GOAL_ANGLE, 2, false, from_host},
        {SET_MOVEMENT, 6, false, from_host},
        {BRAKE, 0, false, none},
        {SLEEP, 0, false, none},
        {RELEASE, 0,false,none},
        {AWAKE, 0, false, none}
    };
    uint8_t id;
    void displayMessage(uint8_t *data, uint8_t length, uint16_t identifier);
    public:
        can_servo(uint8_t id);
        void sendmessage(const command to_send, const uint8_t* message_contents);
        void receiveMessage();
        

};


#endif
