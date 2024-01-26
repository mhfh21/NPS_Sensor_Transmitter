// components/mpu6050/mpu6050.c
#include "mpu6050.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

static const char *TAG = "MPU6050";

esp_err_t mpu6050_read_register(uint8_t addr, uint8_t reg, uint8_t* data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading register 0x%X: %s", reg, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t mpu6050_write_register(uint8_t addr, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error writing to register 0x%X: %s", reg, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t mpu6050_read_bytes(uint8_t addr, uint8_t reg, uint8_t* data, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading bytes from register 0x%X: %s", reg, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t mpu6050_init() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_MPU6050_SDA_PIN,
        .scl_io_num = CONFIG_MPU6050_SCL_PIN,
        .master.clk_speed = CONFIG_MPU6050_I2C_FREQ_HZ,
    };

    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    // Add a delay after powering on the MPU6050
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint8_t who_am_i;
    esp_err_t ret = mpu6050_read_register(MPU6050_ADDR, MPU6050_WHO_AM_I, &who_am_i);

    if (ret != ESP_OK || who_am_i != 0x68) {
        ESP_LOGE(TAG, "MPU6050 initialization failed. WHO_AM_I register: 0x%X", who_am_i);
        return ESP_FAIL;
    }

    ret = mpu6050_write_register(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 initialization failed. Error writing to PWR_MGMT_1 register.");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialization successful");
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t* data) {
    uint8_t raw_data[6] = {0};  // Initialize raw_data to zero

    esp_err_t ret = mpu6050_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, raw_data, 6);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading accelerometer data: %s", esp_err_to_name(ret));
        return ret;
    }

    data->acceleration.x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->acceleration.y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->acceleration.z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    return ESP_OK;
}
