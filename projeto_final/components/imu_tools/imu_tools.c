#include "imu_tools.h"
#include "sensor_imu.h"
#include <math.h>
#include <esp_err.h>

Quaternion *calculatedQuaternion = NULL;

esp_err_t get_imu_data(IMUData *data) {
    GyroscopeData gyro;
    AccelerationData acell;

    esp_err_t statusGetGyro = get_gyroscope_data(&gyro);
    esp_err_t statusGetAcell = get_acceleration_data(&acell);

    data->gyro_x = gyro.x;
    data->gyro_y = gyro.y;
    data->gyro_z = gyro.z;

    data->accel_x = acell.x;
    data->accel_y = acell.y;
    data->accel_z = acell.z;

    if (statusGetGyro == ESP_OK && statusGetAcell == ESP_OK) return ESP_OK;
    else return ESP_FAIL;
}

esp_err_t calculate_quaternion(const IMUData *data, Quaternion *quaternion) {
    //implementando filtro de kalman estendido
    quaternion->w = 1.0;
    quaternion->x = 0.0;
    quaternion->y = 0.0;
    quaternion->z = 0.0;

    quaternion->w += 0.5 * (-quaternion->x * data->gyro_x - quaternion->y * data->gyro_y - quaternion->z * data->gyro_z);
    quaternion->x += 0.5 * (quaternion->w * data->gyro_x + quaternion->y * data->gyro_z - quaternion->z * data->gyro_y);
    quaternion->y += 0.5 * (quaternion->w * data->gyro_y - quaternion->x * data->gyro_z + quaternion->z * data->gyro_x);
    quaternion->z += 0.5 * (quaternion->w * data->gyro_z + quaternion->x * data->gyro_y - quaternion->y * data->gyro_x);

    //normalizando quaternions
    float norm = sqrt(quaternion->w * quaternion->w + quaternion->x * quaternion->x + quaternion->y * quaternion->y + quaternion->z * quaternion->z);
    quaternion->w /= norm;
    quaternion->x /= norm;
    quaternion->y /= norm;
    quaternion->z /= norm;

    //criando copia do quaternion para usar na função de get
    calculatedQuaternion = quaternion;

    return ESP_OK;
}

esp_err_t quaternion_to_euler(const Quaternion *quaternion, EulerAngle *euler) {
    float w = quaternion->w;
    float x = quaternion->x;
    float y = quaternion->y;
    float z = quaternion->z;

    euler->roll  = atan2((w * x + y * z), 0.5 - (x * x + y * y));
    euler->pitch = asin(2.0 * (w * y - x * z));
    euler->yaw   = -atan2((x * y + w * z), 0.5 - (y * y + z * z));
    
    return ESP_OK;
}

esp_err_t get_quaternion(Quaternion *quaternion) {
    if (calculatedQuaternion == NULL) return ESP_FAIL;

    quaternion->w = calculatedQuaternion->w;
    quaternion->x = calculatedQuaternion->x;
    quaternion->y = calculatedQuaternion->y;
    quaternion->z = calculatedQuaternion->z;

    return ESP_OK;
}