#ifndef __pinout__
#define __pinout__

#include "driver/gpio.h"

#define in1_pin GPIO_NUM_18
#define in2_pin GPIO_NUM_15
#define sleep_pin GPIO_NUM_14
#define current_sense_pin GPIO_NUM_1

#define temp_sense_pin GPIO_NUM_0
#define position_pin GPIO_NUM_2

#define can_TX_pin GPIO_NUM_20
#define can_RX_pin GPIO_NUM_19

#define current_sense_channel ADC_CHANNEL_1
#define temp_sense_channel ADC_CHANNEL_0
#define position_channel ADC_CHANNEL_2


#define ONBOARD_RGB_LED GPIO_NUM_8
#endif