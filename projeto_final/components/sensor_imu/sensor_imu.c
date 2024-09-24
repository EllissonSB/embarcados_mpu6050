#include "driver/i2c.h"
#include "sensor_imu.h"
#include "mpu6050_driver.h"

static const char *TAG = "mpu6050 test";
static mpu6050_handle_t mpu6050 = NULL;

static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t imu_init(void) {
  i2c_bus_init();
  esp_err_t errorStatus;
  mpu6050 = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
  errorStatus = mpu6050_config(mpu6050, ACCE_FS_2G, GYRO_FS_250DPS);
  errorStatus = mpu6050_wake_up(mpu6050);

  return errorStatus;
}

esp_err_t get_acceleration_data(AccelerationData *data) {
  mpu6050_acce_value_t acce;
  esp_err_t espStatus = mpu6050_get_acce(mpu6050, &acce);
  data->x = acce.acce_x;
  data->y = acce.acce_y;
  data->z = acce.acce_z;
  return espStatus;
}

esp_err_t get_gyroscope_data(GyroscopeData *data) {
  mpu6050_gyro_value_t gyro;
  esp_err_t espStatus = mpu6050_get_gyro(mpu6050, &gyro);
  data->x = gyro.gyro_x;
  data->y = gyro.gyro_y;
  data->z = gyro.gyro_z;
  return espStatus;
}