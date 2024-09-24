idf_component_register(SRCS "sensor_imu.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "mpu6050_driver" "driver")