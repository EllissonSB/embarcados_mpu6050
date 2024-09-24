#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#define I2C_MASTER_SCL_IO 22      
#define I2C_MASTER_SDA_IO 21      
#define I2C_MASTER_NUM I2C_NUM_0//porta do I2C 
#define I2C_MASTER_FREQ_HZ 400000 

typedef struct {
  float x;
  float y;
  float z;
} AccelerationData;

typedef struct {
  float x;
  float y;
  float z;
} GyroscopeData;

esp_err_t imu_init(void);
esp_err_t get_acceleration_data(AccelerationData *data);
esp_err_t get_gyroscope_data(GyroscopeData *data);

#endif // SENSOR_IMU_H 