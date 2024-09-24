/**
 * @file servo_hw.h
 * @brief Driver de hardware para controlar servomotores usando LEDC
 *
 * Este arquivo contém definições e protótipos de funções para controlar servomotores
 * utilizando o módulo LEDC do ESP-IDF.
 */

#ifndef SERVO_HW_H
#define SERVO_HW_H

#include <esp_err.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

/**
 * @def SERVO_MIN_PULSEWIDTH
 * @brief Pulso mínimo em microsegundos (ângulo 0)
 */
#define SERVO_MIN_PULSEWIDTH 500

/**
 * @def SERVO_MAX_PULSEWIDTH
 * @brief Pulso máximo em microsegundos (ângulo máximo)
 */
#define SERVO_MAX_PULSEWIDTH 2500

/**
 * @def SERVO_MAX_DEGREE
 * @brief Ângulo máximo do servo em graus
 */
#define SERVO_MAX_DEGREE 180

/**
 * @def LEDC_TIMER_FREQ_HZ
 * @brief Frequência de operação do timer LEDC em Hz
 *
 * Valor típico para controle de servos é 50Hz
 */
#define LEDC_TIMER_FREQ_HZ 50

/**
 * @brief Inicializa o hardware necessário para controlar um servo
 *
 * @param gpio_num Número da GPIO utilizada para conectar o servo
 * @param channel Canal LEDC a ser usado
 * @return ESP_OK em caso de sucesso, ESP_ERR_NOT_FOUND caso contrário
 */
esp_err_t hw_servo_init(uint8_t gpio_num, ledc_channel_t channel);

/**
 * @brief Configura o pulso de largura para um servo específico
 *
 * @param gpio_num Número da GPIO utilizada para conectar o servo
 * @param channel Canal LEDC usado pelo servo
 * @param pulse_width_us Largura do pulso em microsegundos
 * @return ESP_OK em caso de sucesso, ESP_FAIL caso contrário
 */
esp_err_t hw_servo_set_pulse_width(uint8_t gpio_num, ledc_channel_t channel, uint32_t pulse_width_us);

/**
 * @brief Desativa o hardware utilizado por um servo
 *
 * @param gpio_num Número da GPIO utilizada para conectar o servo
 * @param channel Canal LEDC usado pelo servo
 * @return ESP_OK em caso de sucesso, ESP_FAIL caso contrário
 */
esp_err_t hw_servo_deinit(uint8_t gpio_num, ledc_channel_t channel);

#endif /* SERVO_HW_H */