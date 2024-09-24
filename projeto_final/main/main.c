#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <esp_err.h>
#include "esp_log.h"
#include "sensor_imu.h"
#include "imu_tools.h"
#include <math.h>
#include "servo_tools.h"
static const char *TAG_SENSOR_IMU = "mpu6050 sensor_imu test";
static const char *TAG_IMU_TOOLS = "mpu6050 imu_tools test";

#define SERVO1_GPIO 5
#define SERVO2_GPIO 19
#define SERVO_MIN_PULSEWIDTH 500    // Em microsegundos
#define SERVO_MAX_PULSEWIDTH 2400   // Em microsegundos
#define SERVO_MAX_ANGLE 180         // Ângulo máximo
int mapValue(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
AccelerationData tempAccelerationMeasures = { .x = -1, .y = -1, .z = -1 };
AccelerationData accelerationMeasures;
GyroscopeData gyroscopeMeasures;
IMUData gyroAcellMeasures;

Quaternion quaternion;
Quaternion quaternionToGetTest;
EulerAngle eulerAngle;

/*Estados do sistema*/
typedef enum {
    GET_IMU_DATA = 0, CALCULATE_QUATERNIONS, CALCULATE_EULER_ANGLES, OUTPUT_VALUES
} States;

void app_main() {
   // Configurações do Servo 1 (GPIO 5)
    ServoConfig servo1_config = {
        .gpio_num = SERVO1_GPIO,
        .min_pulse_us = SERVO_MIN_PULSEWIDTH,
        .max_pulse_us = SERVO_MAX_PULSEWIDTH,
        .max_angle = SERVO_MAX_ANGLE,
        .channel = LEDC_CHANNEL_0
    };

    // Configurações do Servo 2 (GPIO 19)
    ServoConfig servo2_config = {
        .gpio_num = SERVO2_GPIO,
        .min_pulse_us = SERVO_MIN_PULSEWIDTH,
        .max_pulse_us = SERVO_MAX_PULSEWIDTH,
        .max_angle = SERVO_MAX_ANGLE,
        .channel = LEDC_CHANNEL_1
    };



     // Inicializa os dois servos
    printf("Inicializando servos...\n");
    if (servo_init(&servo1_config) != ESP_OK) {
        printf("Falha ao inicializar o Servo 1\n");
    } else {
        printf("Servo 1 inicializado\n");
    }

    if (servo_init(&servo2_config) != ESP_OK) {
        printf("Falha ao inicializar o Servo 2\n");
    } else {
        printf("Servo 2 inicializado\n");
    }

    printf("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n");



  imu_init();

  static States state = GET_IMU_DATA;
  static const int quantStates = 4; //variavel que informa a quantidade de estados
  static bool has_imu_data_change = true; //variavel que informa se os dados do sensor alteraram.
  static bool first_measure = true;

  while (true){
    for (int i = 0; i < quantStates; i++){
      switch(state) {
        case GET_IMU_DATA:
          esp_err_t status = get_acceleration_data(&accelerationMeasures);
          if(first_measure) {
            status = get_acceleration_data(&tempAccelerationMeasures); //struct temporario para construir a logica de verificar se o sensor se movimentou
            first_measure = false;
          }
          //logica para verificar se o sensor movimentou 0.03 foi o valor que decidi usar para filtrar o ruido
          if(!first_measure && (fabs((accelerationMeasures.x - tempAccelerationMeasures.x)) > 0.03 ||
            fabs((accelerationMeasures.y - tempAccelerationMeasures.y)) > 0.03 ||
            fabs((accelerationMeasures.z - tempAccelerationMeasures.z)) > 0.03
          )){
            has_imu_data_change = true;
            tempAccelerationMeasures.x = accelerationMeasures.x;
            tempAccelerationMeasures.y = accelerationMeasures.y;
            tempAccelerationMeasures.z = accelerationMeasures.z;
          }
          else has_imu_data_change = false;

          if (status == ESP_OK) status = get_gyroscope_data(&gyroscopeMeasures);
          if (status == ESP_OK) status = get_imu_data(&gyroAcellMeasures);
          if (status == ESP_OK) state = CALCULATE_QUATERNIONS;
          break;

        case CALCULATE_QUATERNIONS:
          status = calculate_quaternion(&gyroAcellMeasures, &quaternion);
          if (status == ESP_OK) status = get_quaternion(&quaternionToGetTest);
          if (status == ESP_OK) state = CALCULATE_EULER_ANGLES;
          break;
          
        case CALCULATE_EULER_ANGLES:
          status = quaternion_to_euler(&quaternion, &eulerAngle);
          //logica para imprimir ou não os dados de acordo com movimento do sensor
          if (status == ESP_OK && has_imu_data_change) state = OUTPUT_VALUES;
          else if (status == ESP_OK) state = GET_IMU_DATA;
          break;

        case OUTPUT_VALUES:
          //Saída dos dados capturados no sensor
          printf("acce_:%.2f, acce_y:%.2f, acce_z:%.2f", accelerationMeasures.x, accelerationMeasures.y, accelerationMeasures.z);
          printf("gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyroscopeMeasures.x, gyroscopeMeasures.y, gyroscopeMeasures.z);
          
          printf("acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", gyroAcellMeasures.accel_x, gyroAcellMeasures.accel_y, gyroAcellMeasures.accel_z);
          printf("gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyroAcellMeasures.gyro_x, gyroAcellMeasures.gyro_y, gyroAcellMeasures.gyro_z);
          
          //Saída dos quaternion calculados
          printf("quat_w:%.2f, quat_x:%.2f, quat_y:%.2f, quat_z:%.2f", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
          printf( "quat_get_w:%.2f, quat_get_x:%.2f, quat_get_y:%.2f, quat_get_z:%.2f", quaternionToGetTest.w, quaternionToGetTest.x, quaternionToGetTest.y, quaternionToGetTest.z);
          
          //Saída dos angulos de euler calculados
          printf( "pitch:%.2f, yaw:%.2f, roll:%.2f\n", eulerAngle.pitch, eulerAngle.yaw, eulerAngle.roll);
          
          state = GET_IMU_DATA;
          //float roll_angle_servo = ( eulerAngle.roll + 90) * 0.5;  // Mapeia Roll para 0°-180°
          //float pitch_angle_servo = (eulerAngle.pitch + 90) * 0.5; // Mapeia Pitch para 0°-180°
          float roll_angle_servo = (float)eulerAngle.roll*(180/3.14);
          float pitch_angle_servo =  (float)eulerAngle.pitch*(180/3.14);
          int roll_angle_servo_calc = mapValue(roll_angle_servo,-90,90,0,180);
          int pitch_angle_servo_calc = mapValue(pitch_angle_servo,-90,90,0,180);
          servo_set_angle(&servo1_config, roll_angle_servo_calc);
          servo_set_angle(&servo2_config, pitch_angle_servo_calc);
          printf("%d\n",roll_angle_servo_calc);
          printf("%d\n",pitch_angle_servo_calc);
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          break;
      }
    }
  }
}