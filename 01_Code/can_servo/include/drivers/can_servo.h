#ifndef __can_servo__
#define __can_servo__

#include "../all_includes.h"
#include "led_strip_driver.h"
#include "../servo_info.h"
#include "../CAN_endpoints.h"
#include "esp_mac.h"

#define MAX_TWAI_TIMEOUT 1000
#define MAX_COMMAND_COUNT 32

#define RXGEN_OFFSET 1
#define RXGEN_OFFSET 1

#define GET_INFO_COMMAND_LENGTH 4
#define RESTART_COMMAND_LENGTH 0
#define SET_MODE_SET_COMMAND_LENGTH 2
#define UNUSED_1_COMMAND_LENGTH 0
#define GET_MODE_SET_COMMAND_LENGTH 2
#define UNUSED_2_COMMAND_LENGTH 0
#define GET_POSITION_VELOCITY_COMMAND_LENGTH 8
#define UNUSED_3_COMMAND_LENGTH 0
#define GET_CURRENT_DRAW_COMMAND_LENGTH 4
#define UNUSED_4_COMMAND_LENGTH 0
#define GET_TEMPERATURE_COMMAND_LENGTH 4
#define UNUSED_5_COMMAND_LENGTH 0
#define SET_GOAL_POSITION_VELOCITY_COMMAND_LENGTH 4
#define UNUSED_6_COMMAND_LENGTH 0
#define SET_OFFSET_COMMAND_LENGTH 4
#define UNUSED_7_COMMAND_LENGTH 0
#define SET_CURRENT_LIMIT_COMMAND_LENGTH 4
#define UNUSED_8_COMMAND_LENGTH 0
#define SET_TEMPERATURE_LIMIT_COMMAND_LENGTH 4
#define UNUSED_9_COMMAND_LENGTH 0
#define SET_LED_COMMAND_LENGTH 4
#define UNUSED_10_COMMAND_LENGTH 0
#define SET_PID_COMMAND_LENGTH 5
#define UNUSED_11_COMMAND_LENGTH 0
#define RXSDO_COMMAND_LENGTH 9
#define UNUSED_13_COMMAND_LENGTH 0
#define TXSDO_COMMAND_LENGTH 9


#define GET_INFO_COMMAND_ID 0x000
#define REBOOT_COMMAND_ID 0x001
#define SET_MODE_COMMAND_ID 0x002
#define UNUSED_1_COMMAND_ID 0x003
#define GET_MODE_COMMAND_ID 0x004
#define UNUSED_2_COMMAND_ID 0x005
#define GET_POSITION_VELOCITY_COMMAND_ID 0x006
#define UNUSED_3_COMMAND_ID 0x007
#define GET_CURRENT_DRAW_COMMAND_ID 0x008
#define UNUSED_4_COMMAND_ID 0x009
#define GET_TEMPERATURE_COMMAND_ID 0x00A
#define UNUSED_5_COMMAND_ID 0x00B
#define SET_GOAL_POSITION_VELOCITY_COMMAND_ID 0x00C
#define UNUSED_6_COMMAND_ID 0x00D
#define SET_OFFSET_COMMAND_ID 0x00E
#define UNUSED_7_COMMAND_ID 0x00F
#define SET_CURRENT_LIMIT_COMMAND_ID 0x010
#define UNUSED_8_COMMAND_ID 0x011
#define SET_TEMPERATURE_LIMIT_COMMAND_ID 0x012
#define UNUSED_9_COMMAND_ID 0x013
#define SET_LED_COMMAND_ID 0x014
#define UNUSED_10_COMMAND_ID 0x015
#define SET_PID_COMMAND_ID 0x016
#define UNUSED_11_COMMAND_ID 0x017
#define RXGEN_COMMAND_ID 0x018
#define UNUSED_13_COMMAND_ID 0x01C
#define TXGEN_COMMAND_ID 0x01E


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
extern float esp_core_temperature;

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
        REBOOT_                     = REBOOT_COMMAND_ID,
        SET_MODE_STATE             = SET_MODE_COMMAND_ID,
        UNUSED_1                   = UNUSED_1_COMMAND_ID,
        GET_MODE_STATE             = GET_MODE_COMMAND_ID,
        UNUSED_2                   = UNUSED_2_COMMAND_ID,
        GET_POSITION_VELOCITY      = GET_POSITION_VELOCITY_COMMAND_ID,
        UNUSED_3                   = UNUSED_3_COMMAND_ID,
        GET_CURRENT_DRAW           = GET_CURRENT_DRAW_COMMAND_ID,
        UNUSED_4                   = UNUSED_4_COMMAND_ID,
        GET_TEMPERATURE            = GET_TEMPERATURE_COMMAND_ID,
        UNUSED_5                   = UNUSED_5_COMMAND_ID,
        SET_GOAL_POSITION_VELOCITY = SET_GOAL_POSITION_VELOCITY_COMMAND_ID,
        UNUSED_6                   = UNUSED_6_COMMAND_ID,
        SET_OFFSET                 = SET_OFFSET_COMMAND_ID,
        UNUSED_7                   = UNUSED_7_COMMAND_ID,
        SET_CURRENT_LIMIT          = SET_CURRENT_LIMIT_COMMAND_ID,
        UNUSED_8                   = UNUSED_8_COMMAND_ID,
        SET_TEMPERATURE_LIMIT      = SET_TEMPERATURE_LIMIT_COMMAND_ID,
        UNUSED_9                   = UNUSED_9_COMMAND_ID,
        SET_LED                    = SET_LED_COMMAND_ID,
        UNUSED_10                  = UNUSED_10_COMMAND_ID,
        SET_PID                    = SET_PID_COMMAND_ID,
        UNUSED_11                  = UNUSED_11_COMMAND_ID,
        RXGEN                   = RXGEN_COMMAND_ID,
        UNUSED_13                  = UNUSED_13_COMMAND_ID,
        TXGEN                  = TXGEN_COMMAND_ID,
    };
    enum data_direction {to_host, from_host, none};

    typedef struct command{
        uint8_t id;
        uint8_t data_length;
        bool data_returned;
        data_direction direction;
    } command;

    command commandList[MAX_COMMAND_COUNT];
        //name, data length, data returned, data direction

    uint8_t id;
    void display_message(uint8_t *data, uint8_t length, uint16_t identifier);
    void handle_TXSDO(twai_message_t rxMessage);
    void handle_RXSDO(twai_message_t rxMessage);
    can_servo(uint8_t id);
    void send_message(const command to_send, const uint8_t* message_contents, uint8_t length = 0);
    void receive_message();
};


#endif
