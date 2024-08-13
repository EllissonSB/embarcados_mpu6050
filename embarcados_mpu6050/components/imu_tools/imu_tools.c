#include "imu_tools.h"
esp_err_t imu_read_data(IMUData *data){
    AccelerationData data_acel;
    GyroscopeData data_gyro;
    esp_err_t ret;
    ret=imu_get_acceleration_data(&data_acel);
    if (ret!=ESP_OK)
        return ESP_FAIL;
    ret=imu_get_gyroscope_data(&data_gyro);
    if (ret!=ESP_OK)
        return ESP_FAIL;
    data->accel_x=data_acel.accel_x;
    data->accel_y=data_acel.accel_y;
    data->accel_z=data_acel.accel_z;
    data->gyro_x=data_gyro.gyro_x;
    data->gyro_y=data_gyro.gyro_y;
    data->gyro_z=data_gyro.gyro_z;
    return ESP_OK;
}
esp_err_t imu_calculate_quaternion(const IMUData *data, Quaternion *quaternion){
    
    Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f}; // Inicialização do quaternion
    float q1 = 1.0, q2 =0.0, q3 = 0.0, q4 =0.0; // Quaternion de entrada
    float norm;
    float vx, vy, vz, wx, wy, wz, ex, ey, ez;
    float ax,ay,az,gx,gy,gz,dt=0.01;
    ax=data->accel_x;
    ay=data->accel_y;
    az=data->accel_z;
    gx=data->gyro_x;
    gy=data->gyro_y;
    gz=data->gyro_z;
    norm = sqrtf(data->accel_x * data->accel_x + data->accel_y * data->accel_y + data->accel_z * data->accel_z);
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return ESP_FAIL; // Evitar divisão por zero
    ax /= norm;
    ay /= norm;
    az /= norm;
    // Calcular os vetores de rotação e erro
    vx = 2.0f * (q2 * q4 - q1 * q3);
    vy = 2.0f * (q1 * q2 + q3 * q4);
    vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;
    
    wx = 2.0f * (q2 * q3 - q1 * q4);
    wy = 2.0f * (q1 * q3 + q2 * q4);
    wz = q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4;

    ex = (ay * vz - az * vy) - (1.0f - vx * vx - vy * vy - vz * vz);
    ey = (az * wx - ax * vz) - (1.0f - wx * wx - wy * wy - wz * wz);
    ez = (ax * vy - ay * wx) - (1.0f - vx * wx - vy * wy - vz * wz);
    // Ajustar o quaternion
    q1 += (0.5f * (-q2 * gx - q3 * gy - q4 * gz)) * dt;
    q2 += (0.5f * (q1 * gx + q3 * gz - q4 * gy + ex * MADGWICK_BETA)) * dt;
    q3 += (0.5f * (q4 * gx - q1 * gz + q2 * gy - ey * MADGWICK_BETA)) * dt;
    q4 += (0.5f * (q1 * gy - q2 * gx + q3 * gz + ez * MADGWICK_BETA)) * dt;
    // Normalizar o quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);// Calcular os vetores de rotação e erro
    vx = 2.0f * (q2 * q4 - q1 * q3);
    vy = 2.0f * (q1 * q2 + q3 * q4);
    vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;
    
    wx = 2.0f * (q2 * q3 - q1 * q4);
    wy = 2.0f * (q1 * q3 + q2 * q4);
    wz = q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4;

    ex = (ay * vz - az * vy) - (1.0f - vx * vx - vy * vy - vz * vz);
    ey = (az * wx - ax * vz) - (1.0f - wx * wx - wy * wy - wz * wz);
    ez = (ax * vy - ay * wx) - (1.0f - vx * wx - vy * wy - vz * wz);

    // Ajustar o quaternion
    q1 += (0.5f * (-q2 * gx - q3 * gy - q4 * gz)) * dt;
    q2 += (0.5f * (q1 * gx + q3 * gz - q4 * gy + ex * MADGWICK_BETA)) * dt;
    q3 += (0.5f * (q4 * gx - q1 * gz + q2 * gy - ey * MADGWICK_BETA)) * dt;
    q4 += (0.5f * (q1 * gy - q2 * gx + q3 * gz + ez * MADGWICK_BETA)) * dt;

    // Normalizar o quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    quaternion->w=q1/norm;
    quaternion->x=q2/norm;
    quaternion->y=q3/norm;
    quaternion->z=q4/norm;
    return ESP_OK;
}
esp_err_t imu_calculate_euler_angles(const Quaternion *quaternion, EulerAngle *euler){
    float w = quaternion->w;
    float x = quaternion->x;
    float y = quaternion->y;
    float z = quaternion->z;

    euler->roll = atan2f(2.0f * (y * z + w * x), w * w - x * x - y * y + z * z);
    euler->pitch = asinf(-2.0f * (x * z - w * y));
    euler->yaw = atan2f(2.0f * (x * y + w * z), w * w + x * x - y * y - z * z);
    return ESP_OK;
}