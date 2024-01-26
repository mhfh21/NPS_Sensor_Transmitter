#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"

// MPU6050 I2C Address
#define MPU6050_ADDR            0x68

// MPU6050 Register Addresses
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B

// Data structure for holding MPU6050 sensor data
typedef struct {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } acceleration;
} mpu6050_data_t;

// Function prototypes
esp_err_t mpu6050_init();
esp_err_t mpu6050_read(mpu6050_data_t* data);
esp_err_t mpu6050_read_register(uint8_t addr, uint8_t reg, uint8_t* data);
esp_err_t mpu6050_write_register(uint8_t addr, uint8_t reg, uint8_t data);
esp_err_t mpu6050_read_bytes(uint8_t addr, uint8_t reg, uint8_t* data, size_t size);

#endif // MPU6050_H
