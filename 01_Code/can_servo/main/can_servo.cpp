#include "../include/all_includes.h"
#include "../include/can_servo.h"




can_servo::can_servo(uint8_t id){
    this->id = id;
}

void can_servo::receiveMessage(){
    twai_message_t rxMessage;
    esp_err_t result = twai_receive(&rxMessage, pdMS_TO_TICKS(200));
    if (rxMessage.identifier == ((id << 5) | GET_INFO)) {
        //need to call write message
        displayMessage(rxMessage.data, 0, rxMessage.identifier);
        uint8_t info[3] = {version_major, version_minor, node_id};
        sendmessage(commandList[GET_INFO], info);
    } else if (rxMessage.identifier == ((id << 5) | HEARTBEAT)) {
        //should never occur, as heartbeat is not sent to the servo, it is
        //sent to the host
        displayMessage(rxMessage.data, 0, rxMessage.identifier);
    } else if (rxMessage.identifier == ((id << 5) | ESTOP)) {
        displayMessage(rxMessage.data, 0, rxMessage.identifier);
        //estop = true;
        //do not need to call write message
    } else if (rxMessage.identifier == ((id << 5) | GET_STATE)) {
        displayMessage(rxMessage.data, 0, rxMessage.identifier);
        //need to call write message
    } else if (rxMessage.identifier == ((id << 5) | GET_POSITION)) {
        displayMessage(rxMessage.data, 0, rxMessage.identifier);
        //need to call write message
    } else if (rxMessage.identifier == ((id << 5) | GET_CURRENT_DRAW)) {
        displayMessage(rxMessage.data, 0, rxMessage.identifier);
        //need to call write message
    } else if (rxMessage.identifier == ((id << 5) | GET_TEMPERATURE)) {
        displayMessage(rxMessage.data, 0, rxMessage.identifier);
        //need to call write message

    } else if (rxMessage.identifier == ((id << 5) | GET_GOAL_POSITION)) {
        displayMessage(rxMessage.data, 0, rxMessage.identifier);
        //need to call write message
    } else if (rxMessage.identifier == ((id << 5) | SET_OFFSET)) {
        displayMessage(rxMessage.data, 2, rxMessage.identifier);
        //do not need to call write message
    } else if (rxMessage.identifier == ((id << 5) | SET_CURRENT_LIMIT)) {
        displayMessage(rxMessage.data, 4, rxMessage.identifier);
        //do not need to call write message
    } else if (rxMessage.identifier == ((id << 5) | SET_TEMPERATURE_LIMIT)) {
        displayMessage(rxMessage.data, 4, rxMessage.identifier);
        //do not need to call write message
    } else if (rxMessage.identifier == ((id << 5) | SET_DIRECTION)) {
        displayMessage(rxMessage.data, 1, rxMessage.identifier);
        //do not need to call write message
    } else if (rxMessage.identifier == ((id << 5) | SET_GOAL_ANGLE)) {
        displayMessage(rxMessage.data, 2, rxMessage.identifier);
        //do not need to call write message
    } else if (rxMessage.identifier == ((id << 5) | SET_MOVEMENT)) {
        displayMessage(rxMessage.data, 6, rxMessage.identifier);
        //do not need to call write message
    } else if (rxMessage.identifier == ((id << 5) | BRAKE)) {
        displayMessage(rxMessage.data, 0, rxMessage.identifier);
        //estop=true;
        //do not need to call write message
    } else if (rxMessage.identifier == ((id << 5) | SLEEP)) {
        //sleep_state = true;
        //do not need to call write message
    }else if (rxMessage.identifier == ((id << 5) | RELEASE)) {
        //estop = false;
        //do not need to call write message
    }else if (rxMessage.identifier == ((id << 5) | AWAKE)) {
        //sleep_state = false;
        //do not need to call write message
    }else {
        // do nothing, likely not a message for this servo
    }
}

void can_servo::displayMessage(uint8_t *data, uint8_t length, uint16_t identifier){
    switch (length){
        case(0):{
            ESP_LOGI("can_servo", "Received data for 0x%03X", (unsigned int)identifier);
            break;
        }
        case(1):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X",
            (unsigned int)identifier,
            (unsigned int)data[0]);
            break;
        }case(2):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X %02X",
            (unsigned int)identifier,
            (unsigned int)data[0], 
            (unsigned int)data[1]);
            break;
        }case(3):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X %02X %02X",
            (unsigned int)identifier,
            (unsigned int)data[0], 
            (unsigned int)data[1], 
            (unsigned int)data[2]);
            break;
        }case(4):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X %02X %02X %02X",
            (unsigned int)identifier,
            (unsigned int)data[0], 
            (unsigned int)data[1], 
            (unsigned int)data[2], 
            (unsigned int)data[3]);
            break;
        }case(5):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X %02X %02X %02X %02X",
            (unsigned int)identifier,
            (unsigned int)data[0], 
            (unsigned int)data[1], 
            (unsigned int)data[2], 
            (unsigned int)data[3], 
            (unsigned int)data[4]);
            break;
        }case(6):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X %02X %02X %02X %02X %02X",
            (unsigned int)identifier,
            (unsigned int)data[0], 
            (unsigned int)data[1], 
            (unsigned int)data[2], 
            (unsigned int)data[3], 
            (unsigned int)data[4], 
            (unsigned int)data[5]);
            break;
        }
        default:{
            ESP_LOGW("can_servo", "Unexpected data length: %d", (unsigned int)length);
            break;
        }
    }  
}