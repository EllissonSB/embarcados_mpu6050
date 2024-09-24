#ifndef IMU_TOOLS_H
#define IMU_TOOLS_H

#include <esp_err.h>

typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} IMUData;

typedef struct {
    float w, x, y, z;
} Quaternion;

typedef struct {
    float yaw, pitch, roll;
} EulerAngle;

esp_err_t get_imu_data(IMUData *data);
esp_err_t calculate_quaternion(const IMUData *data, Quaternion *quaternion);
esp_err_t quaternion_to_euler(const Quaternion *quaternion, EulerAngle *euler);
esp_err_t get_quaternion(Quaternion *quaternion);

#endif // IMU_TOOLS_H