# Projeto - SE 2024.1 - Parte 2 - Controle de Servomotores

Este projeto demonstra como utilizar as bibliotecas e funções para controlar servomotores em sistemas embarcados usando o framework ESP-IDF. O documento abrange a configuração do servomotor, a inicialização, controle de ângulo e a implementação de uma máquina de estados para o loop principal.

## Componentes Utilizados

- 1x ESP32 + Fonte (3.3V)
- 2x Servomotores

## Tecnologias/Bibliotecas Utilizadas

### ESP32

| Nome da biblioteca         | Descrição                                                                                                                                                                                                                 |
|----------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| stdio.h                    | Biblioteca padrão de entrada/saída em C.                                                                                                                                                                                  |
| freertos/FreeRTOS.h        | Biblioteca de código aberto que implementa um sistema operacional de tempo real (RTOS) para microcontroladores.                                                                                                           |
| freertos/task.h            | Responsável por gerenciar as tarefas do sistema. Tarefas são unidades de código independentes que podem ser executadas simultaneamente por um sistema multitarefa como o FreeRTOS.                                        |
| driver/ledc.h              | Biblioteca ESP-IDF para controle de LED PWM Controller (LEDC), utilizada para gerar sinais PWM, necessários para controlar servomotores.                                                                                  |
| esp_err.h                  | Biblioteca para manipulação de códigos de erro no ESP-IDF. Usada para tratar e retornar erros durante a execução das funções.                                                                                             |

## Funcionamento da Biblioteca

Para controlar os servomotores, é necessário configurar os pinos GPIO e os canais do LEDC (LED PWM Controller), que geram os sinais PWM usados para definir os ângulos dos servos. Abaixo, as instruções detalham as estruturas e funções para inicializar e controlar os servos.

### Definições e Estruturas

- **ServoConfig**
  - `gpio_num_t gpio_num;` — Pino GPIO onde o servo está conectado.
  - `uint32_t pwm_freq;` — Frequência PWM (normalmente 50 Hz).
  - `uint32_t pulse_min;` — Largura mínima do pulso (em microsegundos) para o ângulo de 0°.
  - `uint32_t pulse_max;` — Largura máxima do pulso (em microsegundos) para o ângulo de 180°.

- **ServoState**
  - `ServoConfig config;` — Configuração do servomotor.
  - `float current_angle;` — Ângulo atual do servomotor (em graus).

- **ServoChannel**
  - `gpio_num_t gpio_num;` — Pino GPIO associado ao servomotor.
  - `ledc_channel_t ledc_channel;` — Canal LEDC associado ao pino GPIO.

### Funções de Configuração

- **`esp_err_t configure_ledc_timer();`**
  
  Configura o temporizador LEDC com a frequência padrão (50 Hz).
  
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`esp_err_t configure_ledc_channel(ServoChannel *servo_channel);`**
  
  Configura um canal LEDC para o pino GPIO especificado.
  
  - **Parâmetros:**
    - `servo_channel` — Ponteiro para a estrutura `ServoChannel` que armazena o pino GPIO e o canal LEDC.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

### Funções de Controle do Servomotor

- **`esp_err_t hw_servo_init(ServoState *servo_state);`**
  
  Inicializa um servomotor configurando o temporizador e canal LEDC para o pino GPIO especificado.
  
  - **Parâmetros:**
    - `servo_state` — Ponteiro para a estrutura `ServoState` que armazena o estado do servomotor.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`esp_err_t hw_servo_set_pulse_width(ServoChannel *servo_channel, uint32_t pulse_width);`**
  
  Define a largura do pulso PWM para o pino GPIO especificado.
  
  - **Parâmetros:**
    - `servo_channel` — Ponteiro para a estrutura `ServoChannel` que armazena o pino GPIO e o canal LEDC.
    - `pulse_width` — Largura do pulso em microsegundos.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`esp_err_t servo_init(ServoState *servo_state);`**
  
  Inicializa o servomotor, armazenando seu estado.
  
  - **Parâmetros:**
    - `servo_state` — Ponteiro para a estrutura `ServoState` que armazena a configuração e o estado atual do servo.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`esp_err_t servo_set_angle(ServoState *servo_state, float angle);`**
  
  Define o ângulo do servomotor convertendo-o em largura de pulso.
  
  - **Parâmetros:**
    - `servo_state` — Ponteiro para a estrutura `ServoState` que armazena a configuração e o estado atual do servo.
    - `angle` — Ângulo desejado (em graus).
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`float servo_get_angle(ServoState *servo_state);`**
  
  Obtém o ângulo atual do servomotor.
  
  - **Parâmetros:**
    - `servo_state` — Ponteiro para a estrutura `ServoState` que armazena a configuração e o estado atual do servo.
  - **Retorno:** Ângulo atual (em graus).

- **`esp_err_t hw_servo_deinit(ServoChannel *servo_channel);`**
  
  Desinicializa o servomotor, parando o canal LEDC e liberando o pino GPIO.
  
  - **Parâmetros:**
    - `servo_channel` — Ponteiro para a estrutura `ServoChannel` que armazena o pino GPIO e o canal LEDC.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

## Função Principal (app_main)

A função principal inicializa dois servomotores e define ângulos iniciais para cada um. Em um loop infinito, os servos são movidos de 0 a 180 graus e de volta a 0 graus, alternadamente.

### Máquina de Estados para o Loop Principal

A máquina de estados é uma técnica de controle que organiza o comportamento do sistema em estados distintos, cada um representando uma etapa ou condição específica do processo. No contexto deste projeto, a máquina de estados é usada para gerenciar os movimentos dos servomotores de forma sequencial e controlada.

#### Estados da Máquina

1. **INICIALIZAÇÃO:**  
   Este é o estado inicial, onde o sistema configura os servomotores e define seus ângulos iniciais. É a preparação necessária antes de iniciar qualquer movimento.

2. **MOVER_SERVO1_CRESCER:**  
   Neste estado, o servo 1 é movido de 0 a 180 graus. Este movimento gradual permite controlar o servo com precisão, aumentando o ângulo até atingir a posição máxima.

3. **MOVER_SERVO1_DIMINUIR:**  
   Após o servo 1 atingir 180 graus, este estado é responsável por mover o servo de volta para 0 graus, diminuindo o ângulo de forma controlada.

4. **MOVER_SERVO2_CRESCER:**  
   Similar ao estado *MOVER_SERVO1_CRESCER*, mas aplicado ao servo 2. Neste estado, o servo 2 é movido de 0 a 180 graus, realizando um movimento crescente.

5. **MOVER_SERVO2_DIMINUIR:**  
   Por fim, este estado move o servo 2 de 180 a 0 graus, concluindo o ciclo de movimentos do sistema.

### Comportamento Geral

A máquina de estados permite que o sistema transite de um estado para outro em resposta a condições ou eventos predefinidos. No caso dos servomotores, a transição entre os estados ocorre após o término do movimento de um servo, iniciando o movimento do próximo. Isso garante que os movimentos dos servos sejam realizados de maneira organizada e sincronizada, facilitando o controle e a manutenção do sistema.

Essa abordagem modular e estruturada facilita a compreensão do fluxo de execução do sistema e a implementação de comportamentos complexos, garantindo que cada parte do processo ocorra na ordem correta e sob as condições apropriadas.
