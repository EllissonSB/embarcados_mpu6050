#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "imu_tools.h"
void app_main(void)
{
    esp_err_t ret=0;
    IMUData dados_sensor;
    Quaternion dados_quaternion;
    EulerAngle dados_eulerangle;
    ret=imu_read_data(&dados_sensor);
    printf("Accel: X=%d, Y=%d, Z=%d", dados_sensor.accel_x, dados_sensor.accel_y, dados_sensor.accel_z);
    printf("Gyro: X=%d, Y=%d, Z=%d", dados_sensor.gyro_x, dados_sensor.gyro_y, dados_sensor.gyro_z);
    printf("%d\n",ret);
    ret=imu_calculate_quaternion(&dados_sensor,&dados_quaternion);
    printf("%d\n",ret);
    ret=imu_calculate_euler_angles(&dados_quaternion,&dados_eulerangle);
    printf("%d\n",ret);
    printf("Dados do sensor\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
