idf_component_register(SRCS "./drivers/can_servo.cpp" "./drivers/led_strip_driver.cpp" "./drivers/mp6550.cpp"
                    "./supporting/running_average.cpp" "./supporting/get_shortest_angle.cpp"
                    "./tasks/can_bus_task.cpp" "./tasks/drive_motor_task.cpp" "./tasks/led_strip_driver_task.cpp" "./tasks/read_current_task.cpp"
                    "./tasks/read_position_task.cpp" "./tasks/read_temperature_task.cpp"
                    "main.cpp"
                    INCLUDE_DIRS "." "./drivers" "./tasks" "./supporting")