#ifndef __CAN_BUS_TASK_H__
#define __CAN_BUS_TASK_H__

#include "../all_includes.h"
#include "../drivers/can_servo.h"

#define TWAI_RX_QUEUE_SIZE 32
#define TWAI_TX_QUEUE_SIZE 1
#define CAN_BUS_TASK_DELAY 50



void can_bus(void *pv);
#endif /*__CAN_BUS_TASK_H__*/