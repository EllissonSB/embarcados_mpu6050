// mpu6050.h

#ifndef MPU6050_H
#define MPU6050_H
#define mpu_6050_addr 0x68
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7
#define ACCEL_XOUT_H 0x3B        // Registrador de leitura do eixo X do acelerômetro
#define GYRO_XOUT_H 0x43
#define I2C_MASTER_SCL_IO 22        // Pino para o SCL
#define I2C_MASTER_SDA_IO 21        // Pino para o SDA
#define I2C_MASTER_FREQ_HZ 100000   // Frequência do I2C (100 kHz)
#define I2C_MASTER_NUM I2C_NUM_0    // Número da interface I2C
#define I2C_MASTER_TX_BUF_DISABLE 0 // Desabilitar buffer de transmissão
#define I2C_MASTER_RX_BUF_DISABLE 0 // Desabilitar buffer de recepção
#define I2C_MASTER_TIMEOUT_MS 1000  // Tempo limite de timeout em milissegundos
#define MADGWICK_BETA 0.1f  // Parâmetro de ganho (ajustável)
#define ACCEL_SCALE 16384.0f  // Escala do acelerômetro para ±2g
#define GYRO_SCALE 131.0f     // Escala do giroscópio para ±250°/s
#define DEG_TO_RAD (M_PI / 180.0f)


#endif // MPU6050_H
