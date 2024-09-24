/**
 * @file servo_tools.h
 * @brief Ferramentas para controle de servomotores
 *
 * Este arquivo contém estruturas e protótipos de funções para facilitar o uso de servomotores
 * no ESP-IDF, encapsulando configurações e operações comuns.
 */

#ifndef SERVO_TOOLS_H
#define SERVO_TOOLS_H

#include <esp_err.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

/**
 * @struct ServoConfig
 * @brief Estrutura que representa a configuração de um servo
 *
 * Esta estrutura contém todas as informações necessárias para configurar e controlar um servo.
 */
typedef struct {
    /**
     * @var gpio_num
     * @brief Número da GPIO onde o servo está conectado
     */
    uint8_t gpio_num;
    
    /**
     * @var min_pulse_us
     * @brief Largura mínima de pulso em microsegundos (correspondente ao ângulo 0)
     */
    uint32_t min_pulse_us;
    
    /**
     * @var max_pulse_us
     * @brief Largura máxima de pulso em microsegundos (correspondente ao ângulo máximo)
     */
    uint32_t max_pulse_us;
    
    /**
     * @var max_angle
     * @brief Ângulo máximo do servo em graus (geralmente 180)
     */
    uint32_t max_angle;
    
    /**
     * @var channel
     * @brief Canal PWM utilizado pelo servo
     */
    ledc_channel_t channel;
} ServoConfig;

/**
 * @typedef ServoAngle
 * @brief Tipo para representar o ângulo do servo
 *
 * Definido como uint32_t para garantir compatibilidade com as funções de controle.
 */
typedef uint32_t ServoAngle;

/**
 * @brief Inicializa um servo com base na configuração fornecida
 *
 * @param config Ponteiro para uma estrutura ServoConfig contendo as configurações do servo
 * @return ESP_OK em caso de sucesso, ESP_ERR_INVALID_ARG caso contrário
 */
esp_err_t servo_init(ServoConfig *config);

/**
 * @brief Define o ângulo de um servo
 *
 * @param config Ponteiro para uma estrutura ServoConfig contendo as configurações do servo
 * @param angle Ângulo desejado para o servo
 * @return ESP_OK em caso de sucesso, ESP_FAIL caso contrário
 */
esp_err_t servo_set_angle(ServoConfig *config, ServoAngle angle);

/**
 * @brief Obtém o ângulo atual de um servo
 *
 * @param config Ponteiro para uma estrutura ServoConfig contendo as configurações do servo
 * @param angle Ponteiro onde será armazenado o ângulo atual do servo
 * @return ESP_OK em caso de sucesso, ESP_FAIL caso contrário
 */
esp_err_t servo_get_angle(const ServoConfig *config, ServoAngle *angle);

/**
 * @brief Desativa o hardware utilizado por um servo
 *
 * @param config Ponteiro para uma estrutura ServoConfig contendo as configurações do servo
 * @return ESP_OK em caso de sucesso, ESP_FAIL caso contrário
 */
esp_err_t servo_deinit(ServoConfig *config);

#endif /* SERVO_TOOLS_H */