#include "../../include/all_includes.h"
#include "../../include/drivers/can_servo.h"



can_servo::can_servo(uint8_t id){
    this->id = id;
}


void can_servo::receive_message() {
    union{
        float a;
        uint8_t bytes[BYTES_IN_FLOAT];     
    } temp_union;
    
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(MAX_TWAI_TIMEOUT));

    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
        twai_message_t rxMessage;

        // Receive all available messages
        while (twai_receive(&rxMessage, 0) == ESP_OK) {

            if (rxMessage.identifier == ((id << ID_OFFSET) | GET_INFO) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | GET_INFO)) {
                printf("Received message with ID: 0x%03lX, Data Length: %d\n",
                   rxMessage.identifier, rxMessage.data_length_code);
                display_message(rxMessage.data, 0, rxMessage.identifier);
                uint8_t info[commandList[GET_INFO].data_length] = {version_major, version_minor, node_id};
                send_message(commandList[GET_INFO], info);
                
            } else if (rxMessage.identifier == ((id << ID_OFFSET) | REBOOT_COMMAND_ID) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | REBOOT_COMMAND_ID)) {
                esp_restart();
                
            } else if (rxMessage.identifier == ((id << ID_OFFSET) | SET_MODE_STATE) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | SET_MODE_STATE)) {
                if (rxMessage.data_length_code == commandList[SET_MODE_STATE].data_length) {
                    xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
                    motor_mode = rxMessage.data[BYTE_0];
                    motor_status = rxMessage.data[BYTE_1];
                    xSemaphoreGive(motor_status_mutex);
                }


            } else if (rxMessage.identifier == ((id << ID_OFFSET) | GET_MODE_STATE) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | GET_MODE_STATE)) {
                uint8_t mode_state[commandList[GET_MODE_STATE].data_length] = {0, 0};
                xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
                mode_state[BYTE_0] = motor_mode;
                mode_state[BYTE_1] = motor_status;
                xSemaphoreGive(motor_status_mutex);                
                send_message(commandList[GET_MODE_STATE], mode_state);


            }else if (rxMessage.identifier == ((id << ID_OFFSET) | GET_POSITION_VELOCITY) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | GET_POSITION_VELOCITY)) {
                uint8_t temp_data[commandList[GET_POSITION_VELOCITY].data_length] = {0, 0, 0, 0, 0, 0, 0, 0};
                xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
                temp_union.a = current_angle;
                temp_data[BYTE_0] = temp_union.bytes[BYTE_0];
                temp_data[BYTE_1] = temp_union.bytes[BYTE_1];
                temp_data[BYTE_2] = temp_union.bytes[BYTE_2];
                temp_data[BYTE_3] = temp_union.bytes[BYTE_3];

                temp_union.a = current_velocity;
                temp_data[BYTE_4] = temp_union.bytes[BYTE_0];
                temp_data[BYTE_5] = temp_union.bytes[BYTE_1];
                temp_data[BYTE_6] = temp_union.bytes[BYTE_2];
                temp_data[BYTE_7] = temp_union.bytes[BYTE_3];

                xSemaphoreGive(current_angle_velocity_mutex);
                send_message(commandList[GET_POSITION_VELOCITY], temp_data);


            }else if (rxMessage.identifier == ((id << ID_OFFSET) | GET_CURRENT_DRAW) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | GET_CURRENT_DRAW)) {
                uint8_t temp_data[commandList[GET_CURRENT_DRAW].data_length] = {0, 0, 0, 0};
                xSemaphoreTake(current_mutex, portMAX_DELAY);
                temp_union.a = last_current_draw;
                temp_data[BYTE_0] = temp_union.bytes[BYTE_0];
                temp_data[BYTE_1] = temp_union.bytes[BYTE_1];
                temp_data[BYTE_2] = temp_union.bytes[BYTE_2];
                temp_data[BYTE_3] = temp_union.bytes[BYTE_3];
                xSemaphoreGive(current_mutex);
                send_message(commandList[GET_CURRENT_DRAW], temp_data);


            }else if (rxMessage.identifier == ((id << ID_OFFSET) | GET_TEMPERATURE) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | GET_TEMPERATURE)) {
                uint8_t temp_data[commandList[GET_TEMPERATURE].data_length] = {0, 0, 0, 0};
                xSemaphoreTake(temperature_mutex, portMAX_DELAY);
                temp_union.a = last_motor_temperature;
                temp_data[BYTE_0] = temp_union.bytes[BYTE_0];
                temp_data[BYTE_1] = temp_union.bytes[BYTE_1];
                temp_data[BYTE_2] = temp_union.bytes[BYTE_2];
                temp_data[BYTE_3] = temp_union.bytes[BYTE_3];
                xSemaphoreGive(temperature_mutex);
                send_message(commandList[GET_TEMPERATURE], temp_data);


            }else if (rxMessage.identifier == ((id << ID_OFFSET) | SET_GOAL_POSITION_VELOCITY) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | SET_GOAL_POSITION_VELOCITY)) {
                temp_union.bytes[BYTE_0] = rxMessage.data[BYTE_0];
                temp_union.bytes[BYTE_1] = rxMessage.data[BYTE_1];
                temp_union.bytes[BYTE_2] = rxMessage.data[BYTE_2];
                temp_union.bytes[BYTE_3] = rxMessage.data[BYTE_3];
                xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                if(motor_mode == POSITION_CONTROL){
                    target_angle = temp_union.a;
                }else if(motor_mode == VELOCITY_CONTROL){
                    target_velocity = temp_union.a;
                }
                xSemaphoreGive(target_angle_velocity_mutex);

            }else if (rxMessage.identifier == ((id << ID_OFFSET) | SET_OFFSET) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | SET_OFFSET)) {
                if (rxMessage.data_length_code == commandList[SET_OFFSET].data_length) {
                    xSemaphoreTake(motor_offset_mutex, portMAX_DELAY);
                    temp_union.bytes[BYTE_0] = rxMessage.data[BYTE_0];
                    temp_union.bytes[BYTE_1] = rxMessage.data[BYTE_1];
                    temp_union.bytes[BYTE_2] = rxMessage.data[BYTE_2];
                    temp_union.bytes[BYTE_3] = rxMessage.data[BYTE_3];
                    motor_offset_value = temp_union.a;
                    xSemaphoreGive(motor_offset_mutex);
                }
            }else if (rxMessage.identifier == ((id << ID_OFFSET) | SET_CURRENT_LIMIT) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | SET_CURRENT_LIMIT)) { 
                if (rxMessage.data_length_code == commandList[SET_CURRENT_LIMIT].data_length) {
                    xSemaphoreTake(current_mutex, portMAX_DELAY);
                    temp_union.bytes[BYTE_0] = rxMessage.data[BYTE_0];
                    temp_union.bytes[BYTE_1] = rxMessage.data[BYTE_1];
                    temp_union.bytes[BYTE_2] = rxMessage.data[BYTE_2];
                    temp_union.bytes[BYTE_3] = rxMessage.data[BYTE_3];
                    current_limit_value = temp_union.a;
                    xSemaphoreGive(current_mutex);
                }   
            }else if (rxMessage.identifier == ((id << ID_OFFSET) | SET_TEMPERATURE_LIMIT) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | SET_TEMPERATURE_LIMIT)) {
                if (rxMessage.data_length_code == commandList[SET_TEMPERATURE_LIMIT].data_length) {
                    xSemaphoreTake(temperature_mutex, portMAX_DELAY);
                    temp_union.bytes[BYTE_0] = rxMessage.data[BYTE_0];
                    temp_union.bytes[BYTE_1] = rxMessage.data[BYTE_1];
                    temp_union.bytes[BYTE_2] = rxMessage.data[BYTE_2];
                    temp_union.bytes[BYTE_3] = rxMessage.data[BYTE_3];
                    motor_temperature_limit = temp_union.a;
                    xSemaphoreGive(temperature_mutex);
                }
            }else if (rxMessage.identifier == ((id << ID_OFFSET) | SET_LED) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | SET_LED)) {
                xSemaphoreTake(LED_RGB_values_mutex, portMAX_DELAY);
                led_r = rxMessage.data[BYTE_0];
                led_g = rxMessage.data[BYTE_1];
                led_b = rxMessage.data[BYTE_2];
                xSemaphoreGive(LED_RGB_values_mutex);
            }else if (rxMessage.identifier == ((id << ID_OFFSET) | SET_PID) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | SET_PID)) {
                temp_union.bytes[BYTE_0] = rxMessage.data[BYTE_1];
                temp_union.bytes[BYTE_1] = rxMessage.data[BYTE_2];
                temp_union.bytes[BYTE_2] = rxMessage.data[BYTE_3];
                temp_union.bytes[BYTE_3] = rxMessage.data[BYTE_4];
                
                xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                if(rxMessage.data[BYTE_0] == PID_MESSAGE_P){
                    pid_P = temp_union.a;
                }else if (rxMessage.data[BYTE_0] == PID_MESSAGE_I){
                    pid_I = temp_union.a;
                }else if(rxMessage.data[BYTE_0] == PID_MESSAGE_D){
                    pid_D = temp_union.a;
                }
                xSemaphoreGive(PID_values_mutex);
            }else if (rxMessage.identifier == ((id << ID_OFFSET) | RXSDO_COMMAND_ID) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | RXSDO_COMMAND_ID)){
                handle_RXSDO(rxMessage);
            }else if (rxMessage.identifier == ((id << ID_OFFSET) | TXSDO_COMMAND_ID) or rxMessage.identifier == ((receive_all_id << ID_OFFSET) | TXSDO_COMMAND_ID)){
                handle_TXSDO(rxMessage);
            }else {
                // Unhandled message ID
            }
        }
    }
}

void can_servo::handle_RXSDO(twai_message_t rxMessage){
    union{
        float a;
        uint8_t bytes[BYTES_IN_FLOAT];     
    } temp_union;

    if(endpoint_structure[rxMessage.data[BYTE_0]].read_write == WRITE or endpoint_structure[rxMessage.data[BYTE_0]].read_write == READ_WRITE){
        switch(rxMessage.data[BYTE_0]){
            case(‎MOTOR_STATUS_ENDPOINT):{
                if(rxMessage.data_length_code == MOTOR_STATUS_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
                    motor_status = rxMessage.data[RXSDO_OFFSET];
                    xSemaphoreGive(motor_status_mutex);
                }
                break;
            
            }case(‎MOTOR_MODE_ENDPOINT):{
                if(rxMessage.data_length_code == MOTOR_MODE_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
                    motor_mode = rxMessage.data[RXSDO_OFFSET];
                    xSemaphoreGive(motor_status_mutex);
                }
                break;
                
            }case(‎MAX_CURRENT_DRAW_ENDPOINT):{
                if(rxMessage.data_length_code == MAX_CURRENT_DRAW_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    memcpy(temp_union.bytes, rxMessage.data[RXSDO_OFFSET], BYTES_IN_FLOAT);
                    xSemaphoreTake(current_mutex, portMAX_DELAY);
                    max_current_draw = temp_union.a;
                    xSemaphoreGive(current_mutex);
                }
                break;
                
            }case(‎CURRENT_LIMIT_VALUE_ENDPOINT):{
                if(rxMessage.data_length_code == CURRENT_LIMIT_VALUE_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    memcpy(temp_union.bytes, rxMessage.data[RXSDO_OFFSET], BYTES_IN_FLOAT);
                    xSemaphoreTake(current_mutex, portMAX_DELAY);
                    current_limit_value = temp_union.a;
                    xSemaphoreGive(current_mutex);
                }
                break;
                
            }case(‎MAX_MOTOR_TEMPERATURE_ENDPOINT):{
                if(rxMessage.data_length_code == MAX_MOTOR_TEMPERATURE_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    memcpy(temp_union.bytes, rxMessage.data[RXSDO_OFFSET], BYTES_IN_FLOAT);
                    xSemaphoreTake(temperature_mutex, portMAX_DELAY);
                    max_motor_temperature = temp_union.a;
                    xSemaphoreGive(temperature_mutex);
                }
                break;
                
            }case(‎MOTOR_TEMPERATURE_LIMIT_ENDPOINT):{
                if(rxMessage.data_length_code == MOTOR_TEMPERATURE_LIMIT_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    memcpy(temp_union.bytes, rxMessage.data[RXSDO_OFFSET], BYTES_IN_FLOAT);
                    xSemaphoreTake(temperature_mutex, portMAX_DELAY);
                    motor_temperature_limit = temp_union.a;
                    xSemaphoreGive(temperature_mutex);
                }
                break;
                
            }case(‎MOTOR_OFFSET_VALUE_ENDPOINT):{
                if(rxMessage.data_length_code == MOTOR_OFFSET_VALUE_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    memcpy(temp_union.bytes, rxMessage.data[RXSDO_OFFSET], BYTES_IN_FLOAT);
                    xSemaphoreTake(motor_offset_mutex, portMAX_DELAY);
                    motor_offset_value = temp_union.a;
                    xSemaphoreGive(motor_offset_mutex);
                }
                break;
                
            }case(‎TARGET_ANGLE_ENDPOINT):{
                if(rxMessage.data_length_code == TARGET_ANGLE_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    memcpy(temp_union.bytes, rxMessage.data[RXSDO_OFFSET], BYTES_IN_FLOAT);
                    xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                    target_angle = temp_union.a;
                    xSemaphoreGive(target_angle_velocity_mutex);
                }
                break;
                
            }case(‎TARGET_VELOCITY_ENDPOINT):{
                if(rxMessage.data_length_code == TARGET_VELOCITY_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    memcpy(temp_union.bytes, rxMessage.data[RXSDO_OFFSET], BYTES_IN_FLOAT);
                    xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                    target_velocity = temp_union.a;
                    xSemaphoreGive(target_angle_velocity_mutex);
                }
                break;
                
            }case(‎PID_P_ENDPOINT):{
                if(rxMessage.data_length_code == PID_P_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    memcpy(temp_union.bytes, rxMessage.data[RXSDO_OFFSET], BYTES_IN_FLOAT);
                    xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                    pid_P = temp_union.a;
                    xSemaphoreGive(PID_values_mutex);
                }
                break;
                
            }case(‎PID_I_ENDPOINT):{
                if(rxMessage.data_length_code == PID_I_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    memcpy(temp_union.bytes, rxMessage.data[RXSDO_OFFSET], BYTES_IN_FLOAT);
                    xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                    pid_I = temp_union.a;
                    xSemaphoreGive(PID_values_mutex);
                }
                break;
                
            }case(‎PID_D_ENDPOINT):{
                if(rxMessage.data_length_code == PID_D_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    memcpy(temp_union.bytes, rxMessage.data[RXSDO_OFFSET], BYTES_IN_FLOAT);
                    xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                    pid_D = temp_union.a;
                    xSemaphoreGive(PID_values_mutex);
                }
                break;
                
            }case(‎LED_R_ENDPOINT):{
                if(rxMessage.data_length_code == LED_R_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    xSemaphoreTake(LED_RGB_values_mutex, portMAX_DELAY);
                    led_r = (float)rxMessage.data[RXSDO_OFFSET];
                    xSemaphoreGive(LED_RGB_values_mutex);
                }
                break;
                
            }case(‎LED_G_ENDPOINT):{
                if(rxMessage.data_length_code == LED_G_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    xSemaphoreTake(LED_RGB_values_mutex, portMAX_DELAY);
                    led_g = (float)rxMessage.data[RXSDO_OFFSET];
                    xSemaphoreGive(LED_RGB_values_mutex);
                }
                break;
            }case(‎LED_B_ENDPOINT):{
                if(rxMessage.data_length_code == LED_B_ENDPOINT_LENGTH+RXSDO_OFFSET){
                    xSemaphoreTake(LED_RGB_values_mutex, portMAX_DELAY);
                    led_b = (float)rxMessage.data[RXSDO_OFFSET];
                    xSemaphoreGive(LED_RGB_values_mutex);
                }
                break;
            }
            
        }
    else{
        return; //value is not writable
    }
}

void can_servo::handle_TXSDO(twai_message_t rxMessage){
    if(endpoint_structure[rxMessage.data[BYTE_0]].read_write == READ or endpoint_structure[rxMessage.data[BYTE_0]].read_write == READ_WRITE){
        uint8_t data_to_send[8] = {0,0,0,0,0,0,0,0};

        union{
            float a;
            uint8_t bytes[BYTES_IN_FLOAT];     
        } temp_union;
        
        switch(rxMessage.data[BYTE_0]){
            case(‎NODE_ID_ENDPOINT):{
                data_to_send[BYTE_0] = NODE_ID_ENDPOINT;
                data_to_send[TXSDO_OFFSET] = node_id;
                //send_message(const command to_send, const uint8_t* message_contents, uint8_t length = 0)
                send_message(commandList[TXSDO], data_to_send, NODE_ID_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎RECEIVE_ALL_ENDPOINT):{
                data_to_send[BYTE_0] = RECEIVE_ALL_ENDPOINT;
                data_to_send[TXSDO_OFFSET] = receive_all_id;
                //send_message(const command to_send, const uint8_t* message_contents, uint8_t length = 0)
                send_message(commandList[TXSDO], data_to_send, RECEIVE_ALL_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎VERSION_MAJOR_ENDPOINT):{
                data_to_send[BYTE_0] = VERSION_MAJOR_ENDPOINT;
                data_to_send[TXSDO_OFFSET] = version_major;
                //send_message(const command to_send, const uint8_t* message_contents, uint8_t length = 0)
                send_message(commandList[TXSDO], data_to_send, VERSION_MAJOR_ENDPOINT_LENGTH+);
                break;
                
            }case(‎VERSION_MINOR_ENDPOINT):{
                data_to_send[BYTE_0] = VERSION_MINOR_ENDPOINT;
                data_to_send[TXSDO_OFFSET] = version_minor;
                //send_message(const command to_send, const uint8_t* message_contents, uint8_t length = 0)
                send_message(commandList[TXSDO], data_to_send, VERSION_MINOR_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎MOTOR_STATUS_ENDPOINT):{
                data_to_send[BYTE_0] = MOTOR_STATUS_ENDPOINT;
                xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
                data_to_send[TXSDO_OFFSET] = motor_status;
                xSemaphoreGive(motor_status_mutex);
                send_message(commandList[TXSDO], data_to_send, MOTOR_STATUS_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎MOTOR_MODE_ENDPOINT):{
                data_to_send[BYTE_0] = MOTOR_MODE_ENDPOINT;
                xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
                data_to_send[TXSDO_OFFSET] = motor_mode;
                xSemaphoreGive(motor_status_mutex);
                send_message(commandList[TXSDO], data_to_send, MOTOR_MODE_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(LAST_CURRENT_DRAW_ENDPOINT):{
                data_to_send[BYTE_0] = LAST_CURRENT_DRAW_ENDPOINT;
                xSemaphoreTake(current_mutex, portMAX_DELAY);
                temp_union.a = last_current_draw;
                xSemaphoreGive(current_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, LAST_CURRENT_DRAW_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎MAX_CURRENT_DRAW_ENDPOINT):{
                data_to_send[BYTE_0] = MAX_CURRENT_DRAW_ENDPOINT; 
                xSemaphoreTake(current_mutex, portMAX_DELAY);
                temp_union.a = max_current_draw;
                xSemaphoreGive(current_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, MAX_CURRENT_DRAW_ENDPOINT_LENGTH+TXSDO_OFFSET); 
                break;
                
            }case(‎CURRENT_LIMIT_VALUE_ENDPOINT):{
                data_to_send[BYTE_0] = CURRENT_LIMIT_VALUE_ENDPOINT;
                xSemaphoreTake(current_mutex, portMAX_DELAY);
                temp_union.a = current_limit_value;
                xSemaphoreGive(current_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, CURRENT_LIMIT_VALUE_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎LAST_MOTOR_TEMPERATURE_ENDPOINT):{
                data_to_send[BYTE_0] = LAST_MOTOR_TEMPERATURE_ENDPOINT;
                xSemaphoreTake(temperature_mutex, portMAX_DELAY);
                temp_union.a = last_motor_temperature;
                xSemaphoreGive(temperature_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, LAST_MOTOR_TEMPERATURE_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎MAX_MOTOR_TEMPERATURE_ENDPOINT):{
                data_to_send[BYTE_0] = MAX_MOTOR_TEMPERATURE_ENDPOINT;
                xSemaphoreTake(temperature_mutex, portMAX_DELAY);
                temp_union.a = max_motor_temperature;
                xSemaphoreGive(temperature_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, MAX_MOTOR_TEMPERATURE_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎MOTOR_TEMPERATURE_LIMIT_ENDPOINT):{
                data_to_send[BYTE_0] = MOTOR_TEMPERATURE_LIMIT_ENDPOINT;
                xSemaphoreTake(temperature_mutex, portMAX_DELAY);
                temp_union.a = motor_temperature_limit;
                xSemaphoreGive(temperature_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, MOTOR_TEMPERATURE_LIMIT_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎MOTOR_OFFSET_VALUE_ENDPOINT):{
                data_to_send[BYTE_0] = MOTOR_OFFSET_VALUE_ENDPOINT;
                xSemaphoreTake(motor_offset_mutex, portMAX_DELAY);
                temp_union.a = motor_offset_value;
                xSemaphoreGive(motor_offset_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, MOTOR_OFFSET_VALUE_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎TARGET_ANGLE_ENDPOINT):{
                data_to_send[BYTE_0] = TARGET_ANGLE_ENDPOINT;
                xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                temp_union.a = target_angle;
                xSemaphoreGive(target_angle_velocity_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, TARGET_ANGLE_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎TARGET_VELOCITY_ENDPOINT):{
                data_to_send[BYTE_0] = TARGET_VELOCITY_ENDPOINT;
                xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                temp_union.a = target_velocity;
                xSemaphoreGive(target_angle_velocity_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, TARGET_VELOCITY_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎CURRENT_ANGLE_ENDPOINT):{
                data_to_send[BYTE_0] = CURRENT_ANGLE_ENDPOINT;
                xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
                temp_union.a = current_angle;
                xSemaphoreGive(current_angle_velocity_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, CURRENT_ANGLE_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎CURRENT_VELOCITY_ENDPOINT):{
                data_to_send[BYTE_0] = CURRENT_VELOCITY_ENDPOINT;
                xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
                temp_union.a = current_velocity;
                xSemaphoreGive(current_angle_velocity_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, CURRENT_VELOCITY_ENDPOINT_LENGTH+TXSDO_OFFSET);
                 break;
                
            }case(‎PID_P_ENDPOINT):{
                data_to_send[BYTE_0] = PID_P_ENDPOINT;
                xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                temp_union.a = pid_P;
                xSemaphoreGive(PID_values_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, PID_P_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎PID_I_ENDPOINT):{
                data_to_send[BYTE_0] = PID_I_ENDPOINT;
                xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                temp_union.a = pid_I;
                xSemaphoreGive(PID_values_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, PID_I_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎PID_D_ENDPOINT):{
                data_to_send[BYTE_0] = PID_D_ENDPOINT;
                xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                temp_union.a = pid_D;
                xSemaphoreGive(PID_values_mutex);
                memcpy(data_to_send[TXSDO_OFFSET], temp_union, BYTES_IN_FLOAT);
                send_message(commandList[TXSDO], data_to_send, PID_D_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎LED_R_ENDPOINT):{
                data_to_send[BYTE_0] = LED_R_ENDPOINT;
                xSemaphoreTake(LED_RGB_values_mutex, portMAX_DELAY);
                data_to_send[TXSDO_OFFSET] = (uint8_t)led_r;
                xSemaphoreGive(LED_RGB_values_mutex);
                send_message(commandList[TXSDO], data_to_send, LED_R_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎LED_G_ENDPOINT):{
                data_to_send[BYTE_0] = LED_G_ENDPOINT;
                xSemaphoreTake(LED_RGB_values_mutex, portMAX_DELAY);
                data_to_send[TXSDO_OFFSET] = (uint8_t)led_g;
                xSemaphoreGive(LED_RGB_values_mutex);
                send_message(commandList[TXSDO], data_to_send, LED_G_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
                
            }case(‎LED_B_ENDPOINT):{
                data_to_send[BYTE_0] = LED_B_ENDPOINT;
                xSemaphoreTake(LED_RGB_values_mutex, portMAX_DELAY);
                data_to_send[TXSDO_OFFSET] = (uint8_t)led_b;
                xSemaphoreGive(LED_RGB_values_mutex);
                send_message(commandList[TXSDO], data_to_send, LED_B_ENDPOINT_LENGTH+TXSDO_OFFSET);
                break;
            }default:{
                break;
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
            (unsigned int)data[BYTE_0]);
            break;
        }case(2):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X %02X",
            (unsigned int)identifier,
            (unsigned int)data[BYTE_0], 
            (unsigned int)data[BYTE_1]);
            break;
        }case(3):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X %02X %02X",
            (unsigned int)identifier,
            (unsigned int)data[BYTE_0], 
            (unsigned int)data[BYTE_1], 
            (unsigned int)data[BYTE_2]);
            break;
        }case(4):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X %02X %02X %02X",
            (unsigned int)identifier,
            (unsigned int)data[BYTE_0], 
            (unsigned int)data[BYTE_1], 
            (unsigned int)data[BYTE_2], 
            (unsigned int)data[BYTE_3]);
            break;
        }case(5):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X %02X %02X %02X %02X",
            (unsigned int)identifier,
            (unsigned int)data[BYTE_0], 
            (unsigned int)data[BYTE_1], 
            (unsigned int)data[BYTE_2], 
            (unsigned int)data[BYTE_3], 
            (unsigned int)data[BYTE_4]);
            break;
        }case(6):{
            ESP_LOGI("can_servo", "Received data for 0x%03X: %02X %02X %02X %02X %02X %02X",
            (unsigned int)identifier,
            (unsigned int)data[BYTE_0], 
            (unsigned int)data[BYTE_1], 
            (unsigned int)data[BYTE_2], 
            (unsigned int)data[BYTE_3], 
            (unsigned int)data[BYTE_4], 
            (unsigned int)data[BYTE_5]);
            break;
        }
        default:{
            ESP_LOGW("can_servo", "Unexpected data length: %d", (unsigned int)length);
            break;
        }
    }  
}



void can_servo::send_message(const command to_send, const uint8_t* message_contents, uint8_t length = 0){
    if(to_send.data_length == 0){
        return;
    }


    union {
        float a;
        uint8_t bytes[BYTES_IN_FLOAT];
    } temp_union;

    twai_message_t tx_message = {};
    tx_message.extd = 0;
    tx_message.rtr=0;
    tx_message.identifier = (node_id << ID_OFFSET) | to_send.id;
    if (length == 0){
        tx_message.data_length_code = to_send.data_length;
    }else{
        tx_message.data_length_code = length;
    }


    
    if (to_send.id == GET_INFO){
        tx_message.data[BYTE_0] = version_major;
        tx_message.data[BYTE_1] = version_minor;
        tx_message.data[BYTE_2] = node_id;


    }else if (to_send.id == GET_MODE_STATE){
        tx_message.data[BYTE_0] = message_contents[BYTE_0]; // motor_mode
        tx_message.data[BYTE_1] = message_contents[BYTE_1]; // motor_status


    }else if (to_send.id == GET_POSITION_VELOCITY){
        tx_message.data[BYTE_0] = message_contents[BYTE_0];
        tx_message.data[BYTE_1] = message_contents[BYTE_1];
        tx_message.data[BYTE_2] = message_contents[BYTE_2];
        tx_message.data[BYTE_3] = message_contents[BYTE_3];

        tx_message.data[BYTE_4] = message_contents[BYTE_0];
        tx_message.data[BYTE_5] = message_contents[BYTE_1];
        tx_message.data[BYTE_6] = message_contents[BYTE_2];
        tx_message.data[BYTE_7] = message_contents[BYTE_3];


    }else if (to_send.id == GET_CURRENT_DRAW){
        tx_message.data[BYTE_0] = message_contents[BYTE_0];
        tx_message.data[BYTE_1] = message_contents[BYTE_1];
        tx_message.data[BYTE_2] = message_contents[BYTE_2];
        tx_message.data[BYTE_3] = message_contents[BYTE_3];

    }else if (to_send.id == GET_TEMPERATURE){
        tx_message.data[BYTE_0] = message_contents[BYTE_0];
        tx_message.data[BYTE_1] = message_contents[BYTE_1];
        tx_message.data[BYTE_2] = message_contents[BYTE_2];
        tx_message.data[BYTE_3] = message_contents[BYTE_3];
    }else{
        return;
    }
    twai_message_t rx_message;
    while(twai_receive(&rx_message,0)== ESP_OK){
    
    }
    esp_err_t result = twai_transmit(&tx_message, pdMS_TO_TICKS(MAX_TWAI_TIMEOUT));
    
}
