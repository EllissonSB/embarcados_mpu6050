# Projeto - SE 2024.1

Este projeto demonstra como utilizar as bibliotecas imu_tools e sensor_imu para integrar o sensor inercial MPU6050 em sistemas embarcados usando o framework ESP-IDF. As bibliotecas permitem a leitura dos dados do acelerômetro e giroscópio, além de realizar cálculos de orientação espacial como quaternions e ângulos de Euler.

## Componentes utilizados

- 1x ESP32 + Fonte (3.3V)
- 1x Sensor MPU6050

## Tecnologias/Bibliotecas utilizadas

### ESP32

| Nome da biblioteca                                            | Descrição                                                                                                                                                                                                                                                                                                                                                                                       |
| ------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| stdio.h                                                       | Biblioteca padrão de entrada/saída em C.                                                                                                                                                                                                                                                                                                                                                        |
| inttypes.h                                                      | Biblioteca que provém um set de inteiros de tamanho definido independente de sistemas operacionais e outras implementações.                                                                                                                                                                                                                                                                                                                                              |
| freertos/FreeRTOS.h                                           | Biblioteca de código aberto que implementa um sistema operacional de tempo real (RTOS) para microcontroladores. O RTOS é responsável por gerenciar o tempo e os recursos do microcontrolador, permitindo que vários programas sejam executados simultaneamente.                                                                                                                               |
| freertos/task.h                                               | A biblioteca FreeRTOS/task.h é uma parte específica da biblioteca FreeRTOS.h responsável por gerenciar as tarefas do sistema. Tarefas são unidades de código independentes que podem ser executadas simultaneamente por um sistema multitarefa como o FreeRTOS.                                                                                                                              |
| esp_log.h                                                     | Biblioteca para logs no ESP-IDF. Usada para gerar logs em diferentes níveis (debug, info, erro, etc.).                                                                                                                                                                                                                                                                                           |
| sdkconfig.h                                                   | Arquivo gerado pelo ESP-IDF durante a compilação. Contém configurações específicas do projeto.                                                                                                                                                                                                                                                                                              |
| esp_chip_info.h                                                 | Biblioteca com funções que trabalham com as informações do chip esp.                                                                                                                                                                                                                                                                                                        |
| esp_flash.h                                                 | Biblioteca para os chips flash da esp. É necessária quando trabalhando com quaisquer operações flash na esp.                                                                                                                                                                                                                                                                                                        |
| esp_system.h                                                 | Biblioteca com funções do sistema da esp, como: restart, free heap size, random, entre outras.                                                                                                                                                                                                                                                                                                        |
| math.h                                                 | Biblioteca padrão C com funções matemáticas.                                                                                                                                                                                                                                                                                                        |
| driver/i2c.h                                                 | Biblioteca esp que implementa o protocolo i2c.                                                                                                                                                                                                                                                                                                        |


### Funcionamento da biblioteca

Para usar as bibliotecas `imu_tools` e `sensor_imu`, é necessário ter a extensão Espressif IDF instalada no VSCode, configurada na versão 5.3 ou superior. Abaixo, as instruções para inicializar o sensor MPU6050 e realizar leituras de dados são detalhadas. 

## sensor_imu

### Arquivo: `sensor_imu.h`

#### Descrição

A biblioteca `sensor_imu` define as estruturas e funções necessárias para interagir com o sensor MPU6050 via protocolo I2C, permitindo a leitura dos dados brutos do acelerômetro e giroscópio.

#### Estruturas

- **AccelerationData**
  - `float accel_x;` — Leituras do acelerômetro no eixo X.
  - `float accel_y;` — Leituras do acelerômetro no eixo Y.
  - `float accel_z;` — Leituras do acelerômetro no eixo Z.

- **GyroscopeData**
  - `float gyro_x;` — Leituras do giroscópio no eixo X.
  - `float gyro_y;` — Leituras do giroscópio no eixo Y.
  - `float gyro_z;` — Leituras do giroscópio no eixo Z.

#### Funções

- **`esp_err_t imu_init(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin);`**
  
  Inicializa a comunicação I2C com o MPU6050, configurando os pinos SDA e SCL e acordando o sensor.
  
  - **Parâmetros:**
    - `devAddr` — Endereço I2C do MPU6050.
    - `sda_pin` — Pino de dados I2C (SDA).
    - `scl_pin` — Pino de clock I2C (SCL).
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_ERR_NOT_FOUND` em caso de erro.

- **`esp_err_t imu_get_acceleration_data(AccelerationData *data);`**
  
  Lê os dados do acelerômetro do MPU6050 e preenche a estrutura `AccelerationData`.
  
  - **Parâmetros:**
    - `data` — Ponteiro para a estrutura `AccelerationData`.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`esp_err_t imu_get_gyroscope_data(GyroscopeData *data);`**
  
  Lê os dados do giroscópio do MPU6050 e preenche a estrutura `GyroscopeData`.
  
  - **Parâmetros:**
    - `data` — Ponteiro para a estrutura `GyroscopeData`.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`esp_err_t imu_deinit();`**
  
  Desativa a interface I2C e libera os recursos associados.
  
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`esp_err_t i2c_master_write_slave(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, size_t len);`**
  
  Escreve dados em um registrador do MPU6050.
  
  - **Parâmetros:**
    - `i2c_addr` — Endereço I2C do dispositivo.
    - `reg_addr` — Endereço do registrador para escrita.
    - `data` — Dados a serem escritos.
    - `len` — Tamanho dos dados.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`esp_err_t i2c_master_read_slave(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, size_t len);`**
  
  Lê dados de um registrador do MPU6050.
  
  - **Parâmetros:**
    - `i2c_addr` — Endereço I2C do dispositivo.
    - `reg_addr` — Endereço do registrador para leitura.
    - `data` — Buffer para armazenar os dados lidos.
    - `len` — Tamanho dos dados.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.


## imu_tools

### Arquivo: `imu_tools.h`

#### Descrição

A biblioteca `imu_tools.h` define as estruturas e funções para realizar cálculos de orientação espacial, utilizando os dados obtidos do sensor MPU6050. Ela calcula quaternions e ângulos de Euler, essenciais para determinar a orientação de um dispositivo em espaço tridimensional.

#### Estruturas

- **EulerAngle**
  - `float roll;` — Ângulo de rotação em torno do eixo X.
  - `float pitch;` — Ângulo de rotação em torno do eixo Y.
  - `float yaw;` — Ângulo de rotação em torno do eixo Z.

- **IMUData**
  - `float accel_x;` — Leituras do acelerômetro no eixo X.
  - `float accel_y;` — Leituras do acelerômetro no eixo Y.
  - `float accel_z;` — Leituras do acelerômetro no eixo Z.
  - `float gyro_x;` — Leituras do giroscópio no eixo X.
  - `float gyro_y;` — Leituras do giroscópio no eixo Y.
  - `float gyro_z;` — Leituras do giroscópio no eixo Z.

- **Quaternion**
  - `float w;` — Componente escalar.
  - `float x;` — Componente x.
  - `float y;` — Componente y.
  - `float z;` — Componente z.

#### Definições

- `#define MADGWICK_BETA 0.1f` — Parâmetro de ganho do filtro de Madgwick.
- `#define ACCEL_SCALE 16384.0f` — Escala do acelerômetro para ±2g.
- `#define GYRO_SCALE 131.0f` — Escala do giroscópio para ±250°/s.
- `#define DEG_TO_RAD (M_PI / 180.0f)` — Conversão de graus para radianos.

#### Funções

- **`esp_err_t imu_read_data(IMUData *data);`**
  
  Lê os dados do acelerômetro e giroscópio utilizando a biblioteca `sensor_imu`.
  
  - **Parâmetros:**
    - `data` — Ponteiro para a estrutura `IMUData` onde os dados serão armazenados.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`esp_err_t imu_calculate_quaternion(const IMUData *data, Quaternion *quaternion);`**
  
  Calcula o quaternion a partir dos dados do acelerômetro e giroscópio.
  
  - **Parâmetros:**
    - `data` — Ponteiro para a estrutura `IMUData` contendo os dados do sensor.
    - `quaternion` — Ponteiro para a estrutura `Quaternion` onde o resultado será armazenado.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.

- **`esp_err_t imu_calculate_euler_angles(const Quaternion *quaternion, EulerAngle *euler);`**
  
  Calcula os ângulos de Euler a partir do quaternion fornecido.
  
  - **Parâmetros:**
    - `quaternion` — Ponteiro para a estrutura `Quaternion` contendo os dados do quaternion.
    - `euler` — Ponteiro para a estrutura `EulerAngle` onde os ângulos de Euler serão armazenados.
  - **Retorno:** `ESP_OK` em caso de sucesso, `ESP_FAIL` em caso de erro.
