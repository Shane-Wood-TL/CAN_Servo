#include "../include/all_includes.h"
#include "../include/can_servo.h"
#include "../include/led_strip_driver.h"
#include "../include/servo_info.h"

extern led_strip_driver* led_strip;


can_servo::can_servo(uint8_t id){
    this->id = id;
}


void can_servo::receive_message() {
    union{
        float a;
        uint8_t bytes[4];     
      } temp_union;
    
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1000));

    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
        twai_message_t rxMessage;

        uint8_t command_id = rxMessage.identifier & 0x1F;
        //printf("Command ID: 0x%02X\n", command_id);
        // Receive all available messages
        while (twai_receive(&rxMessage, 0) == ESP_OK) {

            if (rxMessage.identifier == ((id << 5) | GET_INFO)) {
                printf("Received message with ID: 0x%03lX, Data Length: %d\n",
                   rxMessage.identifier, rxMessage.data_length_code);
                display_message(rxMessage.data, 0, rxMessage.identifier);
                uint8_t info[3] = {version_major, version_minor, node_id};
                send_message(commandList[GET_INFO], info);


            } else if (rxMessage.identifier == ((id << 5) | SET_MODE_STATE)) {
                if (rxMessage.data_length_code == 2) {
                    xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
                    motor_mode = rxMessage.data[0];
                    motor_status = rxMessage.data[1];
                    xSemaphoreGive(motor_status_mutex);
                }


            } else if (rxMessage.identifier == ((id << 5) | GET_MODE_STATE)) {
                uint8_t mode_state[2] = {0, 0};
                xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
                mode_state[0] = motor_mode;
                mode_state[1] = motor_status;
                xSemaphoreGive(motor_status_mutex);                
                send_message(commandList[GET_MODE_STATE], mode_state);


            }else if (rxMessage.identifier == ((id << 5) | GET_POSITION_VELOCITY)) {
                uint8_t temp_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
                xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
                temp_union.a = current_angle;
                temp_data[0] = temp_union.bytes[0];
                temp_data[1] = temp_union.bytes[1];
                temp_data[2] = temp_union.bytes[2];
                temp_data[3] = temp_union.bytes[3];

                temp_union.a = current_velocity;
                temp_data[4] = temp_union.bytes[0];
                temp_data[5] = temp_union.bytes[1];
                temp_data[6] = temp_union.bytes[2];
                temp_data[7] = temp_union.bytes[3];

                xSemaphoreGive(current_angle_velocity_mutex);
                send_message(commandList[GET_POSITION_VELOCITY], temp_data);


            }else if (rxMessage.identifier == ((id << 5) | GET_CURRENT_DRAW)) {
                uint8_t temp_data[4] = {0, 0, 0, 0};
                xSemaphoreTake(current_mutex, portMAX_DELAY);
                temp_union.a = last_current_draw;
                temp_data[0] = temp_union.bytes[0];
                temp_data[1] = temp_union.bytes[1];
                temp_data[2] = temp_union.bytes[2];
                temp_data[3] = temp_union.bytes[3];
                xSemaphoreGive(current_mutex);
                send_message(commandList[GET_CURRENT_DRAW], temp_data);


            }else if (rxMessage.identifier == ((id << 5) | GET_TEMPERATURE)) {
                uint8_t temp_data[4] = {0, 0, 0, 0};
                xSemaphoreTake(temperature_mutex, portMAX_DELAY);
                temp_union.a = last_motor_temperature;
                temp_data[0] = temp_union.bytes[0];
                temp_data[1] = temp_union.bytes[1];
                temp_data[2] = temp_union.bytes[2];
                temp_data[3] = temp_union.bytes[3];
                xSemaphoreGive(temperature_mutex);
                send_message(commandList[GET_TEMPERATURE], temp_data);


            }else if (rxMessage.identifier == ((id << 5) | SET_GOAL_POSITION_VELOCITY)) {
                temp_union.bytes[0] = rxMessage.data[0];
                temp_union.bytes[1] = rxMessage.data[1];
                temp_union.bytes[2] = rxMessage.data[2];
                temp_union.bytes[3] = rxMessage.data[3];
                xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                if(motor_mode == POSITION_CONTROL){
                    target_angle = temp_union.a;
                }else if(motor_mode == VELOCITY_CONTROL){
                    target_velocity = temp_union.a;
                }
                xSemaphoreGive(target_angle_velocity_mutex);

            }else if (rxMessage.identifier == ((id << 5) | SET_OFFSET)) {
                if (rxMessage.data_length_code == 4) {
                    xSemaphoreTake(motor_offset_mutex, portMAX_DELAY);
                    temp_union.bytes[0] = rxMessage.data[0];
                    temp_union.bytes[1] = rxMessage.data[1];
                    temp_union.bytes[2] = rxMessage.data[2];
                    temp_union.bytes[3] = rxMessage.data[3];
                    motor_offset_value = temp_union.a;
                    xSemaphoreGive(motor_offset_mutex);
                }
            }else if (rxMessage.identifier == ((id << 5) | SET_CURRENT_LIMIT)) { 
                if (rxMessage.data_length_code == 4) {
                    xSemaphoreTake(current_mutex, portMAX_DELAY);
                    temp_union.bytes[0] = rxMessage.data[0];
                    temp_union.bytes[1] = rxMessage.data[1];
                    temp_union.bytes[2] = rxMessage.data[2];
                    temp_union.bytes[3] = rxMessage.data[3];
                    current_limit_value = temp_union.a;
                    xSemaphoreGive(current_mutex);
                }   
            }else if (rxMessage.identifier == ((id << 5) | SET_TEMPERATURE_LIMIT)) {
                if (rxMessage.data_length_code == 4) {
                    xSemaphoreTake(temperature_mutex, portMAX_DELAY);
                    temp_union.bytes[0] = rxMessage.data[0];
                    temp_union.bytes[1] = rxMessage.data[1];
                    temp_union.bytes[2] = rxMessage.data[2];
                    temp_union.bytes[3] = rxMessage.data[3];
                    motor_temperature_limit = temp_union.a;
                    xSemaphoreGive(temperature_mutex);
                }
            }else if (rxMessage.identifier == ((id << 5) | SET_LED)) {
                xSemaphoreTake(LED_RGB_values_mutex, portMAX_DELAY);
                led_r = rxMessage.data[0];
                led_g = rxMessage.data[1];
                led_b = rxMessage.data[2];
                xSemaphoreGive(LED_RGB_values_mutex);
            }else if (rxMessage.identifier == ((id << 5) | SET_PID)) {
                temp_union.bytes[0] = rxMessage.data[1];
                temp_union.bytes[1] = rxMessage.data[2];
                temp_union.bytes[2] = rxMessage.data[3];
                temp_union.bytes[3] = rxMessage.data[4];
                
                xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                if(rxMessage.data[0] == PID_MESSAGE_P){
                    pid_P = temp_union.a;
                }else if (rxMessage.data[0] == PID_MESSAGE_I){
                    pid_I = temp_union.a;
                }else if(rxMessage.data[0] == PID_MESSAGE_D){
                    pid_D = temp_union.a;
                }
                xSemaphoreGive(PID_values_mutex);
                
            }else {
                // Unhandled message ID
            }
        }
    }
}






void can_servo::display_message(uint8_t *data, uint8_t length, uint16_t identifier){
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



void can_servo::send_message(const command to_send, const uint8_t* message_contents){
    if(to_send.data_length == 0){
        return;
    }

    union {
        float a;
        uint8_t bytes[bytes_in_float];
    } temp_union;

    twai_message_t tx_message = {};
    tx_message.extd = 0;
    tx_message.rtr=0;
    tx_message.identifier = (node_id << 5) | to_send.id;
    tx_message.data_length_code = to_send.data_length;


    
    if (to_send.id == GET_INFO){
        tx_message.data[0] = version_major;
        tx_message.data[1] = version_minor;
        tx_message.data[2] = node_id;


    }else if (to_send.id == GET_MODE_STATE){
        tx_message.data[0] = message_contents[0]; // motor_mode
        tx_message.data[1] = message_contents[1]; // motor_status


    }else if (to_send.id == GET_POSITION_VELOCITY){
        tx_message.data[0] = message_contents[0];
        tx_message.data[1] = message_contents[1];
        tx_message.data[2] = message_contents[2];
        tx_message.data[3] = message_contents[3];

        tx_message.data[4] = message_contents[0];
        tx_message.data[5] = message_contents[1];
        tx_message.data[6] = message_contents[2];
        tx_message.data[7] = message_contents[3];


    }else if (to_send.id == GET_CURRENT_DRAW){
        tx_message.data[0] = message_contents[0];
        tx_message.data[1] = message_contents[1];
        tx_message.data[2] = message_contents[2];
        tx_message.data[3] = message_contents[3];

    }else if (to_send.id == GET_TEMPERATURE){
        tx_message.data[0] = message_contents[0];
        tx_message.data[1] = message_contents[1];
        tx_message.data[2] = message_contents[2];
        tx_message.data[3] = message_contents[3];
    }else{
        return;
    }
    twai_message_t rx_message;
    while(twai_receive(&rx_message,0)== ESP_OK){
    
    }
    esp_err_t result = twai_transmit(&tx_message, pdMS_TO_TICKS(200));
    
}
