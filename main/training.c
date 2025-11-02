#include "training.h"

#include "app.h"
#include "esp_log.h"
#include "motion.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "TRAINING";

#define SAMPLE_RATE_HZ   50
#define SAMPLE_DELAY_MS  (1000 / SAMPLE_RATE_HZ)

void start_data_collection(const training_config_t *config) {
    ESP_LOGI(TAG, "Starting data collection at %d Hz", SAMPLE_RATE_HZ);

    if (!config) {
        ESP_LOGE(TAG, "config is null");
        return;
    }

    // Print CSV header once
    printf("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ,accel_mag,gyro_mag,jerkX,jerkY,jerkZ\n");
    fflush(stdout);

    uint64_t start_time_ms = esp_timer_get_time() / 1000;
    uint64_t last_warn_us = 0; // rate-limit warnings
    const uint64_t warn_interval_us = 2000000ULL; // 2 seconds

    motion_sample_t sample;

    // Keep previous sample to compute jerk (dA/dt)
    bool has_prev = false;
    float prev_ax = 0.0f, prev_ay = 0.0f, prev_az = 0.0f;
    uint64_t prev_ts_ms = 0;

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

        uint64_t timestamp_ms = (esp_timer_get_time() / 1000) - start_time_ms;

        // Magnitudes
        float accel_mag = sqrtf(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
        float gyro_mag = sqrtf(sample.gx * sample.gx + sample.gy * sample.gy + sample.gz * sample.gz);

        // Jerk (derivative of acceleration)
        float jerk_x = 0.0f, jerk_y = 0.0f, jerk_z = 0.0f;
        if (has_prev && timestamp_ms > prev_ts_ms) {
            float dt_s = (float) (timestamp_ms - prev_ts_ms) / 1000.0f;
            jerk_x = (sample.ax - prev_ax) / dt_s;
            jerk_y = (sample.ay - prev_ay) / dt_s;
            jerk_z = (sample.az - prev_az) / dt_s;
        }

        printf("%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
               (unsigned long long) timestamp_ms,
               sample.ax, sample.ay, sample.az,
               sample.gx, sample.gy, sample.gz);

        if (config->include_magnitudes) {
            printf(",%.6f,%.6f", accel_mag, gyro_mag);
        }

        if (config->include_jerk) {
            printf(",%.6f,%.6f,%.6f", jerk_x, jerk_y, jerk_z);
        }
        printf("\n");

        // Save current sample as previous for next iteration
        prev_ax = sample.ax;
        prev_ay = sample.ay;
        prev_az = sample.az;
        prev_ts_ms = timestamp_ms;
        has_prev = true;

        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
}
