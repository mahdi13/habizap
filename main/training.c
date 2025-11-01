#include "training.h"

#include "app.h"
#include "esp_log.h"
#include "motion.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "TRAINING";

#define SAMPLE_RATE_HZ   50
#define SAMPLE_DELAY_MS  (1000 / SAMPLE_RATE_HZ)

void start_data_collection(void) {
    ESP_LOGI(TAG, "Starting data collection at %d Hz", SAMPLE_RATE_HZ);

    // Print CSV header once
    printf("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ\n");
    fflush(stdout);

    uint64_t start_time_ms = esp_timer_get_time() / 1000;
    uint64_t last_warn_us = 0;           // rate-limit warnings
    const uint64_t warn_interval_us = 2000000ULL; // 2 seconds

    motion_sample_t sample;

    while (1) {
        // If motion device isn't ready, pause sampling and warn infrequently
        if (!app_ctx()->motion_ready) {
            uint64_t now_us = esp_timer_get_time();
            if (now_us - last_warn_us >= warn_interval_us) {
                last_warn_us = now_us;
                ESP_LOGW(TAG, "Motion not ready; data collection paused (will resume when sensor initializes)");
            }
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        // Attempt to read a sample; on failure, warn infrequently
        if (motion_read(&sample) != ESP_OK) {
            uint64_t now_us = esp_timer_get_time();
            if (now_us - last_warn_us >= warn_interval_us) {
                last_warn_us = now_us;
                ESP_LOGW(TAG, "Failed to read motion sample (will keep trying)");
            }
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
            continue;
        }

        uint64_t timestamp = (esp_timer_get_time() / 1000) - start_time_ms;

        // Print in Edge Impulse CSV format
        printf("%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
               (unsigned long long) timestamp,
               sample.ax, sample.ay, sample.az,
               sample.gx, sample.gy, sample.gz);

        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
}
