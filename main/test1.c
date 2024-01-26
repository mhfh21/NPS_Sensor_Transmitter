#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "mirf.h"

static const char *TAG = "main";

void AdvancedSettings(NRF24_t *dev)
{
    ESP_LOGW(pcTaskGetName(0), "Set RF Data Ratio to 1MBps");
    Nrf24_SetSpeedDataRates(dev, 0);
}

void sender(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(0), "Start");
    NRF24_t dev;
    Nrf24_init(&dev);
    uint8_t payload = 32;
    uint8_t channel = 90;
    Nrf24_config(&dev, channel, payload);

    // Set the receiver address using 5 characters
    esp_err_t ret = Nrf24_setTADDR(&dev, (uint8_t *)"FGHIJ");
    if (ret != ESP_OK)
    {
        ESP_LOGE(pcTaskGetName(0), "nrf24l01 not installed");
        while (1)
        {
            vTaskDelay(1);
        }
    }

    AdvancedSettings(&dev);

    // Print settings
    Nrf24_printDetails(&dev);

    uint8_t buf[32];
    while (1)
    {
        TickType_t nowTick = xTaskGetTickCount();

        // Format the accelerometer data into the buf array
        mpu6050_data_t sensor_data;
        if (mpu6050_read(&sensor_data) == ESP_OK)
        {
            sprintf((char *)buf, "X=%d, Y=%d, Z=%d", sensor_data.acceleration.x,
                    sensor_data.acceleration.y, sensor_data.acceleration.z);
        }
        else
        {
            ESP_LOGW(pcTaskGetName(0), "Failed to read accelerometer data");
            continue;
        }

        Nrf24_send(&dev, buf);
        vTaskDelay(1);
        ESP_LOGI(pcTaskGetName(0), "Wait for sending.....");

        if (Nrf24_isSend(&dev, 1000))
        {
            ESP_LOGI(pcTaskGetName(0), "Send success: %s", buf);
        }
        else
        {
            ESP_LOGW(pcTaskGetName(0), "Send fail:");
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    mpu6050_data_t sensor_data;

    // Initialize MPU6050
    if (mpu6050_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 initialization failed");
        return;
    }

    // Read accelerometer data
    if (mpu6050_read(&sensor_data) == ESP_OK)
    {
        ESP_LOGI(TAG, "Accelerometer: X=%d, Y=%d, Z=%d",
                 sensor_data.acceleration.x,
                 sensor_data.acceleration.y,
                 sensor_data.acceleration.z);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for 1 second

    xTaskCreate(&sender, "SENDER", 1024 * 3, NULL, 2, NULL);
}
