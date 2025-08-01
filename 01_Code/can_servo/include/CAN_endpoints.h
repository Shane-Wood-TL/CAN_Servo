#include "all_includes.h"


#ifndef  __CAN_ENDPOINTS__
#define __CAN_ENDPOINTS__
//      endpoint id                 data length (bytes)     read/write

#define RXSDO_OFFSET 1
#define TXSDO_OFFSET 1


#define NODE_ID_ENDPOINT 0          //1                      r
#define NODE_ID_ENDPOINT_LENGTH 1

#define RECEIVE_ALL_ENDPOINT 1
#define RECEIVE_ALL_ENDPOINT_LENGTH 1

#define VERSION_MAJOR_ENDPOINT 2    //read only
#define VERSION_MAJOR_ENDPOINT_LENGTH 1

#define VERSION_MINOR_ENDPOINT 3    //read only
#define VERSION_MINOR_ENDPOINT_LENGTH 1

#define MOTOR_STATUS_ENDPOINT 4
#define MOTOR_STATUS_ENDPOINT_LENGTH 1

#define MOTOR_MODE_ENDPOINT 5
#define MOTOR_MODE_ENDPOINT_LENGTH 1

#define LAST_CURRENT_DRAW_ENDPOINT 6 //last_current_draw, 4
#define LAST_CURRENT_DRAW_ENDPOINT_LENGTH 4

#define MAX_CURRENT_DRAW_ENDPOINT 7 //max_current_draw, 4
#define MAX_CURRENT_DRAW_ENDPOINT_LENGTH 4

#define CURRENT_LIMIT_VALUE_ENDPOINT 8 //current_limit_value, 4
#define CURRENT_LIMIT_VALUE_ENDPOINT_LENGTH 4

#define LAST_MOTOR_TEMPERATURE_ENDPOINT 9 //last_motor_temperature, 4
#define LAST_MOTOR_TEMPERATURE_ENDPOINT_LENGTH 4

#define MAX_MOTOR_TEMPERATURE_ENDPOINT 10 //max_motor_temperature, 4
#define MAX_MOTOR_TEMPERATURE_ENDPOINT_LENGTH 4

#define MOTOR_TEMPERATURE_LIMIT_ENDPOINT 11 //motor_temperature_limit, 4
#define MOTOR_TEMPERATURE_LIMIT_ENDPOINT_LENGTH 4

#define MOTOR_OFFSET_VALUE_ENDPOINT 12 //motor_offset_value, 4
#define MOTOR_OFFSET_VALUE_ENDPOINT_LENGTH 4

#define TARGET_ANGLE_ENDPOINT 13 //target_angle, 4
#define TARGET_ANGLE_ENDPOINT_LENGTH 4

#define TARGET_VELOCITY_ENDPOINT 14 //target_velocity, 4
#define TARGET_VELOCITY_ENDPOINT_LENGTH 4

#define CURRENT_ANGLE_ENDPOINT 15 //current_angle, 4
#define CURRENT_ANGLE_ENDPOINT_LENGTH 4

#define CURRENT_VELOCITY_ENDPOINT 16 //current_velocity, 4
#define CURRENT_VELOCITY_ENDPOINT_LENGTH 4

#define PID_P_ENDPOINT 17 //pid_P, 4
#define PID_P_ENDPOINT_LENGTH 4

#define PID_I_ENDPOINT 18 //pid_I, 4
#define PID_I_ENDPOINT_LENGTH 4

#define PID_D_ENDPOINT 19 //pid_D, 4
#define PID_D_ENDPOINT_LENGTH 4

#define LED_R_ENDPOINT 20 //led_r, 4
#define LED_R_ENDPOINT_LENGTH 1

#define LED_G_ENDPOINT 21 //led_g, 4
#define LED_G_ENDPOINT_LENGTH 1

#define LED_B_ENDPOINT 22 //led_b, 4
#define LED_B_ENDPOINT_LENGTH 1

#define CORE_TEMP_ENDPOINT 23
#define CORE_TEMP_ENDPOINT_LENGTH 4

#define MAC_ADDRESS_ENDPOINT 24
#define MAC_ADDRESS_ENDPOINT_LENGTH 6

enum endpoint_rw{READ, WRITE, READ_WRITE};
typedef struct endpoint_structure{
    uint8_t id;
    uint8_t data_length;
    endpoint_rw READ_WRITE;
} endpoint_structure;

#define ENDPOINT_COUNT 25
extern endpoint_structure endpoint_list[ENDPOINT_COUNT];
#endif
