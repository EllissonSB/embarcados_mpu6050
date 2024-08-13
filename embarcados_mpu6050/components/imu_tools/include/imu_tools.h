#ifndef IMU_H_TOOLS
#define IMU_H_TOOLS
#include "sensor_imu.h"
#include "math.h"
typedef struct {
// Ângulos de Euler
    float roll;   // Rotação em torno do eixo X
    float pitch;  // Rotação em torno do eixo Y
    float yaw;    // Rotação em torno do eixo Z
}EulerAngle;
// Definição da struct para o MPU6050
typedef struct {
    // Leituras do acelerômetro
    float accel_x;
    float accel_y;
    float accel_z;

    // Leituras do giroscópio
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
}IMUData;
typedef struct {
    float w; // Componente escalar
    float x; // Componente x
    float y; // Componente y
    float z; // Componente z
} Quaternion;
#define MADGWICK_BETA 0.1f  // Parâmetro de ganho (ajustável)
#define ACCEL_SCALE 16384.0f  // Escala do acelerômetro para ±2g
#define GYRO_SCALE 131.0f     // Escala do giroscópio para ±250°/s
#define DEG_TO_RAD (M_PI / 180.0f)
esp_err_t imu_read_data(IMUData *data);
esp_err_t imu_calculate_quaternion(const IMUData *data, Quaternion *quaternion);
esp_err_t imu_calculate_euler_angles(const Quaternion *quaternion, EulerAngle *euler);
#endif