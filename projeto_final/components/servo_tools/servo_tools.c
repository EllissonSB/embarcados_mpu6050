/**
 * @file servo_tools.c
 * @brief Implementação de ferramentas para controle de servomotores
 *
 * Este arquivo contém a implementação das funções para inicializar, configurar e controlar servomotores.
 */

#include "servo_tools.h"
#include "servo_hw.h"

// Array para armazenar o ângulo atual de cada canal
static ServoAngle current_angles[LEDC_CHANNEL_MAX] = {0};

/**
 * @brief Inicializa o hardware e configurações necessárias para um servo
 *
 * Esta função verifica os argumentos, inicializa o hardware usando hw_servo_init(),
 * e define o ângulo inicial como 0 para o canal especificado.
 *
 * @param config Ponteiro para a estrutura de configuração do servo
 * @return ESP_OK em caso de sucesso, ESP_ERR_INVALID_ARG caso contrário
 */
esp_err_t servo_init(ServoConfig *config)
{
    if (config == NULL || config->channel >= LEDC_CHANNEL_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = hw_servo_init(config->gpio_num, config->channel);
    if (ret != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }

    current_angles[config->channel] = 0; // Define o ângulo inicial como 0
    return ESP_OK;
}

/**
 * @brief Define um novo ângulo para um servo
 *
 * Esta função verifica os argumentos, calcula a largura de pulso correspondente ao ângulo,
 * atualiza o hardware usando hw_servo_set_pulse_width(), e armazena o novo ângulo.
 *
 * @param config Ponteiro para a estrutura de configuração do servo
 * @param angle Novo ângulo a ser definido
 * @return ESP_OK em caso de sucesso, ESP_FAIL caso contrário
 */
esp_err_t servo_set_angle(ServoConfig *config, ServoAngle angle)
{
    if (config == NULL || angle > config->max_angle || config->channel >= LEDC_CHANNEL_MAX) {
        return ESP_FAIL;
    }

    // Cálculo da largura de pulso baseado no ângulo e nas configurações do servo
    uint32_t pulse_width_us = config->min_pulse_us + 
        ((config->max_pulse_us - config->min_pulse_us) * angle) / config->max_angle;

    esp_err_t ret = hw_servo_set_pulse_width(config->gpio_num, config->channel, pulse_width_us);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    current_angles[config->channel] = angle; // Atualiza o ângulo atual para o canal
    return ESP_OK;
}

/**
 * @brief Obtém o ângulo atual de um servo
 *
 * Esta função verifica os argumentos e retorna o ângulo armazenado para o canal especificado.
 *
 * @param config Ponteiro constante para a estrutura de configuração do servo
 * @param angle Ponteiro onde será armazenado o ângulo atual
 * @return ESP_OK em caso de sucesso, ESP_FAIL caso contrário
 */
esp_err_t servo_get_angle(const ServoConfig *config, ServoAngle *angle)
{
    if (config == NULL || angle == NULL || config->channel >= LEDC_CHANNEL_MAX) {
        return ESP_FAIL;
    }

    *angle = current_angles[config->channel]; // Retorna o ângulo atual para o canal
    return ESP_OK;
}

/**
 * @brief Desativa o hardware utilizado por um servo
 *
 * Esta função verifica os argumentos e chama hw_servo_deinit() para desativar o hardware do servo.
 *
 * @param config Ponteiro para a estrutura de configuração do servo
 * @return ESP_OK em caso de sucesso, ESP_FAIL caso contrário
 */
esp_err_t servo_deinit(ServoConfig *config)
{
    if (config == NULL || config->channel >= LEDC_CHANNEL_MAX) {
        return ESP_FAIL;
    }

    esp_err_t ret = hw_servo_deinit(config->gpio_num, config->channel);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}