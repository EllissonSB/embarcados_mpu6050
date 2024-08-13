#include "sensor_imu.h"
esp_err_t imu_init(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin){
    uint8_t data;
    // Configuração dos parâmetros do I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,               // Modo mestre
        .sda_io_num = sda_pin,       // Pino SDA
        .sda_pullup_en = GPIO_PULLUP_ENABLE,   // Habilitar pull-up interno
        .scl_io_num = scl_pin,       // Pino SCL
        .scl_pullup_en = GPIO_PULLUP_ENABLE,   // Habilitar pull-up interno
        .master.clk_speed = I2C_MASTER_FREQ_HZ // Frequência do I2C
    };
    // Configurar os parâmetros de I2C
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    // Instalar o driver I2C
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    i2c_master_read_slave(devAddr, MPU6050_WHO_AM_I_REG_ADDR, &data, 1);
    if (data != 0x68) {
        return ESP_ERR_NOT_FOUND;
    }

    // Acorda o MPU6050 (escreve 0 no registrador PWR_MGMT_1)
    data = 0x00;
    i2c_master_write_slave(devAddr, MPU6050_PWR_MGMT_1_REG_ADDR, &data, 1);
    return ESP_OK;
}

esp_err_t imu_get_acceleration_data(AccelerationData *data){
    uint8_t data1[6];

    // Ler 6 bytes começando do registrador ACCEL_XOUT_H
    if (i2c_master_read_slave(mpu_6050_addr, ACCEL_XOUT_H, data1, 6)!= ESP_OK){
        return ESP_FAIL;
    }

    // Converter os dados em valores de aceleração (16 bits)
    data->accel_x = (data1[0] << 8) | data1[1];
    data->accel_y = (data1[2] << 8) | data1[3];
    data->accel_z = (data1[4] << 8) | data1[5];
    return ESP_OK;
}
esp_err_t imu_get_gyroscope_data(GyroscopeData *data){
     uint8_t data1[6];

    // Ler 6 bytes começando do registrador GYRO_XOUT_H
    if(i2c_master_read_slave(mpu_6050_addr, GYRO_XOUT_H, data1, 6)!=ESP_OK){
        return ESP_FAIL;
    }

    // Converter os dados em valores de rotação (16 bits)
    data->gyro_x = (data1[0] << 8) | data1[1];
    data->gyro_y = (data1[2] << 8) | data1[3];
    data->gyro_z = (data1[4] << 8) | data1[5];
    return ESP_OK;
}
esp_err_t imu_deinit(){
    esp_err_t ret = i2c_driver_delete(I2C_MASTER_NUM);
    if (ret == ESP_OK) {
        return ESP_OK;
    } else {
       return ESP_FAIL;
    }
}
// Função para escrever dados para o MPU6050
esp_err_t i2c_master_write_slave(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Função para ler dados do MPU6050
esp_err_t i2c_master_read_slave(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}