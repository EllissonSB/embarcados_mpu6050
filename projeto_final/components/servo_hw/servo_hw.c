/**
 * @file servo_hw.c
 * @brief Implementação de funções para controle de hardware de servomotores
 *
 * Este arquivo contém a implementação das funções declaradas em servo_hw.h,
 * responsáveis por inicializar, configurar e desativar o hardware necessário
 * para controlar servomotores usando o módulo LEDC do ESP-IDF.
 */

#include "servo_hw.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include <stdio.h>

/**
 * @brief Converte um ângulo em largura de pulso
 *
 * Calcula a largura de pulso correspondente a um determinado ângulo,
 * interpolando linearmente entre SERVO_MIN_PULSEWIDTH e SERVO_MAX_PULSEWIDTH.
 *
 * @param angle Ângulo do servo em graus
 * @return Largura de pulso correspondente em microssegundos
 */
static uint32_t angle_to_pulsewidth(int angle)
{
    return SERVO_MIN_PULSEWIDTH + ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle) / SERVO_MAX_DEGREE;
}

/**
 * @brief Converte largura de pulso em valor de duty cycle
 *
 * Calcula o valor de duty cycle correspondente a uma dada largura de pulso,
 * considerando a frequência do timer e a resolução de 16 bits.
 *
 * @param pulsewidth_us Largura de pulso em microssegundos
 * @param timer_freq_hz Frequência do timer em Hz
 * @return Valor de duty cycle
 */
static uint32_t pulsewidth_to_duty(uint32_t pulsewidth_us, uint32_t timer_freq_hz)
{
    uint32_t max_duty = (1 << LEDC_TIMER_16_BIT) - 1;
    uint32_t period_us = 1000000 / timer_freq_hz;
    return (pulsewidth_us * max_duty) / period_us;
}

/**
 * @brief Inicializa o hardware necessário para controlar um servo
 *
 * Configura o temporizador LEDC, o canal LEDC e realiza uma varredura inicial do servo.
 *
 * @param gpio_num Número da GPIO onde o servo está conectado
 * @param channel Canal LEDC a ser usado
 * @return ESP_OK em caso de sucesso, ESP_ERR_NOT_FOUND caso contrário
 */
esp_err_t hw_servo_init(uint8_t gpio_num, ledc_channel_t channel)
{
    // Configurar o temporizador LEDC para PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // Modo de alta velocidade para melhor precisão
        .timer_num = LEDC_TIMER_0,            // Utiliza-se o temporizador 0
        .duty_resolution = LEDC_TIMER_16_BIT, // Resolução de 16 bits para precisão do PWM
        .freq_hz = LEDC_TIMER_FREQ_HZ,        // Frequência de 50Hz padrão para controle de servos
        .clk_cfg = LEDC_AUTO_CLK              // Configura o clock automaticamente
    };

    // Aplica a configuração do temporizador
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        printf("Falha na configuração do temporizador LEDC\n");
        return ESP_ERR_NOT_FOUND; // Retorna erro se a configuração falhar
    }

    // Configurar o canal LEDC para gerar PWM
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,  // Modo de alta velocidade para melhor precisão
        .channel = channel,                  // Canal especificado como parâmetro
        .timer_sel = LEDC_TIMER_0,           // Associa ao temporizador configurado anteriormente
        .intr_type = LEDC_INTR_DISABLE,      // Desativa interrupções para este canal
        .gpio_num = gpio_num,                // GPIO onde o servo está conectado
        .duty = 0,                           // Duty cycle inicial em 0%
        .hpoint = 0                          // Não utilizado neste caso
    };

    // Aplica a configuração do canal
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        printf("Falha na configuração do canal LEDC\n");
        return ESP_ERR_NOT_FOUND; // Retorna erro se a configuração falhar
    }

    // Realiza uma varredura inicial do servo para garantir seu posicionamento correto
    // Varredura de 0° até SERVO_MAX_DEGREE
    for (int angle = 0; angle <= SERVO_MAX_DEGREE; angle++) {
        uint32_t pulse_width = angle_to_pulsewidth(angle); // Calcula largura de pulso para o ângulo atual
        hw_servo_set_pulse_width(gpio_num, channel, pulse_width); // Define a largura de pulso
        vTaskDelay(pdMS_TO_TICKS(10)); // Espera 10ms entre cada posição para movimento suave
    }

    // Varredura de SERVO_MAX_DEGREE até 0°
    for (int angle = SERVO_MAX_DEGREE; angle >= 0; angle--) {
        uint32_t pulse_width = angle_to_pulsewidth(angle); // Calcula largura de pulso para o ângulo atual
        hw_servo_set_pulse_width(gpio_num, channel, pulse_width); // Define a largura de pulso
        vTaskDelay(pdMS_TO_TICKS(10)); // Espera 10ms entre cada posição para movimento suave
    }

    return ESP_OK; // Retorna sucesso se tudo foi configurado corretamente
}


/**
 * @brief Define a largura de pulso para um servo específico
 *
 * Converte a largura de pulso em duty cycle e atualiza o hardware.
 *
 * @param gpio_num Número da GPIO utilizada para conectar o servo
 * @param channel Canal LEDC usado pelo servo
 * @param pulse_width_us Largura do pulso em microssegundos
 * @return ESP_OK em caso de sucesso, ESP_FAIL caso contrário
 */
esp_err_t hw_servo_set_pulse_width(uint8_t gpio_num, ledc_channel_t channel, uint32_t pulse_width_us)
{
    // Converter a largura de pulso para duty cycle
    uint32_t duty = pulsewidth_to_duty(pulse_width_us, LEDC_TIMER_FREQ_HZ);
    
    // Definir o duty cycle para o valor calculado
    esp_err_t ret = ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    if (ret != ESP_OK) {
        printf("Falha ao definir o duty cycle\n");
        return ESP_FAIL;
    }

    // Atualizar o duty cycle no hardware
    ret = ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
    if (ret != ESP_OK) {
        printf("Falha ao atualizar o duty cycle\n");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Desativa o hardware utilizado por um servo
 *
 * Para o PWM no canal especificado e libera os recursos utilizados pelo driver LEDC.
 *
 * @param gpio_num Número da GPIO utilizada para conectar o servo
 * @param channel Canal LEDC usado pelo servo
 * @return ESP_OK em caso de sucesso, ESP_FAIL caso contrário
 */
esp_err_t hw_servo_deinit(uint8_t gpio_num, ledc_channel_t channel)
{
    // Desabilitar o PWM no canal associado ao GPIO especificado
    esp_err_t ret = ledc_stop(LEDC_HIGH_SPEED_MODE, channel, 0); // Define o duty cycle para 0 antes de parar
    if (ret != ESP_OK) {
        printf("Falha ao desabilitar o PWM no GPIO %d\n", gpio_num);
        return ESP_FAIL;
    }

    // Desalocar os recursos utilizados pelo driver LEDC
    ret = ledc_timer_rst(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0); // Reseta o temporizador
    if (ret != ESP_OK) {
        printf("Falha ao liberar o temporizador LEDC\n");
        return ESP_FAIL;
    }

    printf("PWM no GPIO %d desabilitado com sucesso e recursos liberados\n", gpio_num);
    return ESP_OK;
}