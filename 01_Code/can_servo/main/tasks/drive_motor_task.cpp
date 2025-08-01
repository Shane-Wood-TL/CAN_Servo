#include "../../include/tasks/drive_motor_task.h"

void drive_motor(void *pv){
    // PID state
    static float integral = 0.0f;
    static float last_error = 0.0f;


    static float vel_integral = 0.0f;
    static float vel_last_error = 0.0f;
    int16_t last_motor_speed = 0;
    
    float last_target = 0.0f;

    bool is_sleeping = true;
    motor.sleep();
    for(;;){          
        //drive motor
        xSemaphoreTake(motor_status_mutex, portMAX_DELAY);
        uint8_t current_motor_status = motor_status;
        uint8_t current_motor_mode = motor_mode;
        xSemaphoreGive(motor_status_mutex);

        //printf("status: %d\n", current_motor_status);
        //printf("mode: %d\n", current_motor_mode);

        if(current_motor_status == OVERTEMP){
            printf("overtemp\n");
            motor.sleep();
            is_sleeping = true;
        }else if(current_motor_status == OVERCURRENT){
            printf("over current\n");
            motor.sleep();
            is_sleeping = true;
        }else if(current_motor_status == AWAKE){
            if(is_sleeping){
                printf("waking\n");
                motor.wake();
                is_sleeping = false;
            }
        }else if(current_motor_status == SLEEP){
            if(!is_sleeping){
                printf("sleeping\n");
                motor.sleep();
                is_sleeping = true;
            }
        }else if(current_motor_status == ERROR){
            printf("error\n");
            motor.sleep();
            is_sleeping = true;
        }

        
                
        
        if (current_motor_status == AWAKE){
            //printf("awake\n");
            if(current_motor_mode == VELOCITY_CONTROL){
                xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
                xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                float error = target_velocity;  
                float dt = DRIVE_MOTOR_TASK_DELAY / M_to_S;  // time step in seconds

                // Integral with anti-windup
                vel_integral += error * dt;
                if (vel_integral > INTEGRAL_LIMIT) vel_integral = INTEGRAL_LIMIT;
                else if (vel_integral < -INTEGRAL_LIMIT) vel_integral = -INTEGRAL_LIMIT;

                // Derivative with low-pass filtering
                float derivative = (error - vel_last_error) / dt;
                static float vel_last_filtered = 0.0f;
                float filtered_derivative = LOW_PASS_FILTER_COEFFICIENT * derivative +
                                            (1 - LOW_PASS_FILTER_COEFFICIENT) * vel_last_filtered;
                vel_last_filtered = filtered_derivative;

                vel_last_error = error;

                // PID output
                xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                float output = -(pid_P * error + pid_I * vel_integral + pid_D * filtered_derivative);
                xSemaphoreGive(PID_values_mutex);

                // Clamp output
                if (output > MAX_ADC_INT) {
                    last_motor_speed = MAX_ADC_INT;
                } else if (output < -MAX_ADC_INT) {
                    last_motor_speed = -MAX_ADC_INT;
                } else {
                    last_motor_speed = (int16_t)output;
                }

                // Stop near zero velocity
                if (fabsf(error) < VELOCITY_ERROR_THRESHOLD) {
                    last_motor_speed = 0;
                    motor.break_motor();
                }

                xSemaphoreGive(target_angle_velocity_mutex);
                xSemaphoreGive(current_angle_velocity_mutex);


            }else if(current_motor_mode == POSITION_CONTROL){
                xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
                xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                xSemaphoreTake(motor_offset_mutex, portMAX_DELAY);
                


                
                //printf("Current Angle: %f\n", current_angle);
                printf("Target Angle: %f\n", target_angle);
                //printf("Offset Value: %d\n", motor_offset_value);
                //float error = target_angle - current_angle;
                float error = get_shortest_angle(target_angle, current_angle);
                float dt = DRIVE_MOTOR_TASK_DELAY / M_to_S;  // time step in seconds
                // Integral with anti-windup
                integral += error * dt;
                if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
                else if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

                // Derivative
                float derivative = (error - last_error) / dt;
                static float last_filtered = 0.0f; // Last filtered value for derivative
                float filtered_derivative = LOW_PASS_FILTER_COEFFICIENT * derivative + (1 - LOW_PASS_FILTER_COEFFICIENT) * last_filtered;

                last_error = error;

                
                if (fabsf(last_target - target_angle) > INTEGRAL_ON_LARGE_CHANGE_RESET) {
                    integral = 0;
                }
                last_target = target_angle;

                xSemaphoreTake(PID_values_mutex, portMAX_DELAY);
                float output = pid_P * error + pid_I * integral + pid_D * filtered_derivative;
                xSemaphoreGive(PID_values_mutex);

                if(output > MAX_ADC_INT){
                    last_motor_speed = MAX_ADC_INT;
                }else if(output < -MAX_ADC_INT){
                    last_motor_speed = -MAX_ADC_INT;
                }else{
                    last_motor_speed = (int16_t)output;
                }
                // Stop near target
                if (fabsf(error) < ERROR_THRESHOLD) {
                    last_motor_speed = 0;
                    motor.break_motor();
                }

                xSemaphoreGive(motor_offset_mutex);
                xSemaphoreGive(target_angle_velocity_mutex);
                xSemaphoreGive(current_angle_velocity_mutex);
                
                //printf("angle: %f, target: %f, error: %f, output: %f\n", current_angle, target_angle, error, output);  
            }
            motor.driveMotor(last_motor_speed);
        }

        vTaskDelay(pdMS_TO_TICKS(DRIVE_MOTOR_TASK_DELAY));        
    }
}