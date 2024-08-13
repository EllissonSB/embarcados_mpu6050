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
    printf("%d\n",ret);
    ret=imu_calculate_quaternion(&dados_sensor,&dados_quaternion);
    printf("quaternion: q_w=%.2f q_x=%.2f q_y=%.2f q_z=%.2f\n\n",dados_quaternion.w,dados_quaternion.x,dados_quaternion.y,dados_quaternion.z);
    printf("%d\n",ret);
    ret=imu_calculate_euler_angles(&dados_quaternion,&dados_eulerangle);
    printf("%d\n",ret);
    prinrf("euler angle: roll=%.2f pitch=%.2f yaw=%.2f\n\n",dados_eulerangle.roll,dados_eulerangle.pitch,dados_eulerangle.yaw);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}