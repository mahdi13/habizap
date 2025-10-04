#include "training.h"

#include "app.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "TRAINING";

#define SAMPLE_RATE_HZ 50
#define SAMPLE_DELAY_MS (1000 / SAMPLE_RATE_HZ)

#define SAMPLE_RATE_HZ   50
#define SAMPLE_DELAY_MS  (1000 / SAMPLE_RATE_HZ)

void start_data_collection(void) {
    const mpu6050_handle_t mpu = app_ctx()->mpu;
    if (mpu == NULL) {
        ESP_LOGE(TAG, "MPU6050 handle is NULL. Call init_mpu() first.");
        return;
    }

    ESP_LOGI(TAG, "Starting data collection at %d Hz", SAMPLE_RATE_HZ);

    // Print CSV header once
    printf("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ\n");
    fflush(stdout);

    uint64_t start_time_ms = esp_timer_get_time() / 1000;

    // Reuse these structs each loop instead of redeclaring
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    while (1) {
        if (mpu6050_get_acce(mpu, &acce) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read accel");
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
            continue;
        }

        if (mpu6050_get_gyro(mpu, &gyro) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read gyro");
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
            continue;
        }

        uint64_t timestamp = (esp_timer_get_time() / 1000) - start_time_ms;

        // Print in Edge Impulse CSV format
        printf("%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
               (unsigned long long) timestamp,
               acce.acce_x, acce.acce_y, acce.acce_z,
               gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
}
