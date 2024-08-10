#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "mirf.h"

#define BUFFER_SIZE 50 // اندازه بافر برای ذخیره‌سازی داده‌های 1 ثانیه

static const char *TAG = "Transmitter 1";

void sender(void *pvParameters)
{
    ESP_LOGI(TAG, "Start Transmitter 1");
    NRF24_t dev;
    Nrf24_init(&dev);
    uint8_t payload = 32;
    uint8_t channel = 90; // کانال 90 برای فرستنده 1
    Nrf24_config(&dev, channel, payload);

    esp_err_t ret = Nrf24_setTADDR(&dev, (uint8_t *)"FGHIJ");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "nrf24l01 not installed");
        while (1)
        {
            vTaskDelay(1);
        }
    }

    Nrf24_printDetails(&dev);

    uint8_t buf[32];
    mpu6050_data_t sensor_data[BUFFER_SIZE];
    int index = 0;

    while (1)
    {
        // جمع‌آوری داده‌ها برای 1 ثانیه
        for (index = 0; index < BUFFER_SIZE; ++index)
        {
            if (mpu6050_read(&sensor_data[index]) != ESP_OK)
            {
                ESP_LOGW(TAG, "Failed to read accelerometer data");
            }
            vTaskDelay(20 / portTICK_PERIOD_MS); // جمع‌آوری داده‌ها هر 20 میلی‌ثانیه
        }

        // آماده‌سازی و ارسال دسته‌ای داده‌ها
        for (index = 0; index < BUFFER_SIZE; ++index)
        {
            sprintf((char *)buf, "X=%d, Y=%d, Z=%d", sensor_data[index].acceleration.x,
                    sensor_data[index].acceleration.y, sensor_data[index].acceleration.z);

            Nrf24_send(&dev, buf);
            ESP_LOGI(TAG, "Sent data: %s", buf);

            // بررسی موفقیت ارسال
            if (!Nrf24_isSend(&dev, 1000))
            {
                ESP_LOGW(TAG, "Send fail:");
            }

            vTaskDelay(20 / portTICK_PERIOD_MS); // تأخیر برای هماهنگی با نرخ جمع‌آوری داده‌ها
        }
    }
}

void app_main(void)
{
    mpu6050_data_t sensor_data;

    if (mpu6050_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 initialization failed");
        return;
    }

    if (mpu6050_read(&sensor_data) == ESP_OK)
    {
        ESP_LOGI(TAG, "Accelerometer: X=%d, Y=%d, Z=%d",
                 sensor_data.acceleration.x,
                 sensor_data.acceleration.y,
                 sensor_data.acceleration.z);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    xTaskCreate(&sender, "SENDER", 1024 * 3, NULL, 2, NULL);
}
