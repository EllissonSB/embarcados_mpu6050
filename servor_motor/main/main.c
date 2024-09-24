#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo_tools.h"

#define SERVO1_PIN GPIO_NUM_13
#define SERVO2_PIN GPIO_NUM_12
#define LEDC_RESOLUTION LEDC_TIMER_14_BIT // Resolução de 14 bits
#define SERVO1_FREQ 50
#define SERVO2_FREQ 50
#define SERVO1_MIN_PULSE (int)(0.023 * (1 << LEDC_RESOLUTION))
#define SERVO1_MAX_PULSE (int)(0.124 * (1 << LEDC_RESOLUTION))
#define SERVO2_MIN_PULSE (int)(0.023 * (1 << LEDC_RESOLUTION))
#define SERVO2_MAX_PULSE (int)(0.124 * (1 << LEDC_RESOLUTION))

#define DELAY_MS 1000

static ServoConfig servo1_config = {
    .gpio_num = SERVO1_PIN,
    .freq_hz = SERVO1_FREQ,
    .min_pulse_us = SERVO1_MIN_PULSE,
    .max_pulse_us = SERVO1_MAX_PULSE
};

static ServoConfig servo2_config = {
    .gpio_num = SERVO2_PIN,
    .freq_hz = SERVO2_FREQ,
    .min_pulse_us = SERVO2_MIN_PULSE,
    .max_pulse_us = SERVO2_MAX_PULSE
};

void app_main(void) {
    // Inicialize os servomotores
    if (servo_init(&servo1_config) != ESP_OK) {
        printf("Erro ao inicializar o servo 1\n");
        return;
    }
    if (servo_init(&servo2_config) != ESP_OK) {
        printf("Erro ao inicializar o servo 2\n");
        return;
    }

    // Defina os ângulos iniciais
    servo_set_angle(&servo1_config, 0);
    servo_set_angle(&servo2_config, 0);

    while (1) {
        // Mova o servo 1 de 0 a 180 graus
        for (int angle = 0; angle <= 180; angle += 10) {
            servo_set_angle(&servo1_config, angle);
            vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
        }
        for (int angle = 180; angle >= 0; angle -= 10) {
            servo_set_angle(&servo1_config, angle);
            vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
        }

        // Mova o servo 2 de 0 a 180 graus
        for (int angle = 0; angle <= 180; angle += 10) {
            servo_set_angle(&servo2_config, angle);
            vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
        }
        for (int angle = 180; angle >= 0; angle -= 10) {
            servo_set_angle(&servo2_config, angle);
            vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
        }
    }
}
