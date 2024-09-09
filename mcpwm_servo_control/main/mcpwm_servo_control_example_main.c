#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define SERVO_PIN GPIO_NUM_18
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY 50 // Frequência PWM

// Função para converter o ângulo do servo em ciclo de trabalho
int ServoAngle(int angle) {
    const int minDuty = (int)(0.023 * 65536);
    const int maxDuty = (int)(0.124 * 65536);
    return minDuty + (angle * (maxDuty - minDuty) / 180);
}

// Função para mover o servo para um determinado ângulo
void moveServo(int angle) {
    int dutyCycle = ServoAngle(angle);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, dutyCycle);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Função para configurar o temporizador LEDC
void configure_ledc_timer(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
}

// Função para configurar o canal LEDC
void configure_ledc_channel(void) {
    ledc_channel_config_t ledc_channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

// Função para percorrer os 180ª do servo 
void validaServo() {

    printf("\nValidando servo motor .... \n");
    for (int angle = 0; angle <= 180; angle += 10) { 
        moveServo(angle);
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
    for (int angle = 180; angle >= 0; angle -= 10) { 
        moveServo(angle);
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
    printf("\nValidação concluida .... \n");

}

void app_main(void) {
    // Configuração do LEDC
    configure_ledc_timer();
    configure_ledc_channel();
    validaServo();
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
    while (1) {
        moveServo(0); 
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
        moveServo(180); 
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
}
