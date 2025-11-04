#include "../../include/tasks/drive_motor_task.h"

void drive_motor(void *pv){
    static int16_t max_change = 50;
    static int16_t last_pass_motor_speed = 0;
    // PID state
    static float integral = 0.0f;
    static float last_error = 0.0f;


    static float vel_integral = 0.0f;
    static float vel_last_error = 0.0f;
    int16_t last_motor_speed = 0;
    
    float last_target = 0.0f;

    bool is_sleeping = true;
    motor.sleep();

    
    stepped_interpolator stepped_interpolator_instance(0.0, 500.0f);

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


                if (target_velocity * 500 > MAX_ADC_INT) {
                    last_motor_speed = MAX_ADC_INT;
                } else if (target_velocity * 500 < -MAX_ADC_INT) {
                    last_motor_speed = -MAX_ADC_INT;
                } else {
                    last_motor_speed = (int16_t)(target_velocity * 500);
                }
                xSemaphoreGive(target_angle_velocity_mutex);
                xSemaphoreGive(current_angle_velocity_mutex);


            }else if(current_motor_mode == POSITION_CONTROL){
                xSemaphoreTake(current_angle_velocity_mutex, portMAX_DELAY);
                xSemaphoreTake(target_angle_velocity_mutex, portMAX_DELAY);
                xSemaphoreTake(motor_offset_mutex, portMAX_DELAY);
                const float Kp = 200.0f;     // proportional gain
                const float Kd = 0.5f;

                float error = get_shortest_angle(target_angle, current_angle);
                static float last_error = 0.0f;
                float derivative = error - last_error;

                // Basic PD controller
                float control_output = (Kp * error) + (Kd * derivative);

                // Clamp output to allowable range
                if (control_output > MAX_SPEED) control_output = MAX_SPEED;
                if (control_output < -MAX_SPEED) control_output = -MAX_SPEED;

                // Convert to integer speed for the motor driver
                last_motor_speed = static_cast<int16_t>(control_output);
                printf("%f\n", control_output);
                
                //printf("Current Angle: %f\n", current_angle);
                //printf("Target Angle: %f\n", target_angle);
                //printf("Offset Value: %d\n", motor_offset_value);
                //float error = target_angle - current_angle;

                xSemaphoreGive(motor_offset_mutex);
                xSemaphoreGive(target_angle_velocity_mutex);
                xSemaphoreGive(current_angle_velocity_mutex);
                
                //printf("angle: %f, target: %f, error: %f, output: %f\n", current_angle, target_angle, error, output);  
            }

            stepped_interpolator_instance.set_target(last_motor_speed);

            motor.driveMotor(stepped_interpolator_instance.get_position());
            //printf("Motor Speed: %d\n", last_motor_speed);
        }

        vTaskDelay(pdMS_TO_TICKS(DRIVE_MOTOR_TASK_DELAY));        
    }
}