#include "all_includes.h"


/*
//      endpoint id                 data length (bytes)     read/write


#define NODE_ID_ENDPOINT 0          //1                      r
#define NODE_ID_ENDPOINT_LENGTH 1

#define receive_all_id_enpoint 1   //1                      r
#define version_major_endpoint 2    //read only
#define version_minor_endpoint 3    //read only


motor_status, 1
motor_mode, 1

last_current_draw, 4
max_current_draw, 4
current_limit_value, 4

last_motor_temperature, 4
max_motor_temperature, 4
motor_temperature_limit, 4

motor_offset_value, 4
target_angle, 4
target_velocity, 4

current_angle, 4
current_velocity, 4

pid_P, 4
pid_I, 4
pid_D, 4

led_r, 4
led_g, 4
led_b, 4

*/

enum endpoint_rw{READ, WRITE, READ_WRITE};
struct endpoint_structure{
    uint8_t id;
    uint8_t data_length;
    endpoint_rw read_write;
}

#define ENDPOINT_COUNT 100
endpoint_structure endpoint_list[ENDPOINT_COUNT] = {
    {NODE_ID_ENDPOINT, NODE_ID_ENDPOINT_LENGTH, READ},
}