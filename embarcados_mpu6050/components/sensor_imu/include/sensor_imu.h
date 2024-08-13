#ifndef IMU_H
#define IMU_H
/* Includes*/
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
//#include "mpu6050.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "driver/i2c.h"
/*Protótipo das funções*/
#define mpu_6050_addr 0x68
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7
#define ACCEL_XOUT_H 0x3B        // Registrador de leitura do eixo X do acelerômetro
#define GYRO_XOUT_H 0x43
#define I2C_MASTER_SCL_IO 22        // Pino para o SCL
#define I2C_MASTER_SDA_IO 21        // Pino para o SDA
#define I2C_MASTER_FREQ_HZ 100000   // Frequência do I2C (100 kHz)
#define I2C_MASTER_NUM I2C_NUM_0    // Número da interface I2C
#define I2C_MASTER_TX_BUF_DISABLE 0 // Desabilitar buffer de transmissão
#define I2C_MASTER_RX_BUF_DISABLE 0 // Desabilitar buffer de recepção
#define I2C_MASTER_TIMEOUT_MS 1000  // Tempo limite de timeout em milissegundos
typedef struct {
    // Leituras do acelerômetro
    float accel_x;
    float accel_y;
    float accel_z;
}AccelerationData;
typedef struct {
    // Leituras do giroscópio
    float gyro_x;
    float gyro_y;
    float gyro_z;
} GyroscopeData;
esp_err_t imu_init(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin);
esp_err_t imu_get_acceleration_data(AccelerationData *data);
esp_err_t imu_get_gyroscope_data(GyroscopeData *data);
esp_err_t imu_deinit();
esp_err_t i2c_master_write_slave(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_master_read_slave(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, size_t len);
#endif