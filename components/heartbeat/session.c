#include "heartbeat/session.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG_SESSION = "HB_SESSION";

esp_err_t heartbeat_session_run(heartbeat_ctx_t *ctx,
                                const heartbeat_session_params_t *params,
                                heartbeat_sample_t *out)
{
    if (!ctx || !out) return ESP_ERR_INVALID_ARG;
    heartbeat_session_params_t p = {
        .min_seconds = 3,
        .max_seconds = 10,
        .require_contact = true,
        .poll_interval_ms = 50,
    };
    if (params) {
        if (params->min_seconds) p.min_seconds = params->min_seconds;
        if (params->max_seconds) p.max_seconds = params->max_seconds;
        p.require_contact = params->require_contact;
        if (params->poll_interval_ms) p.poll_interval_ms = params->poll_interval_ms;
    }
    if (p.max_seconds && p.min_seconds > p.max_seconds) p.min_seconds = p.max_seconds;

    esp_err_t err = heartbeat_start(ctx);
    if (err != ESP_OK) return err;

    int64_t start_us = esp_timer_get_time();
    int64_t deadline_us = start_us + (int64_t)(p.max_seconds ? p.max_seconds : 10) * 1000000LL;
    int64_t min_ready_us = start_us + (int64_t)p.min_seconds * 1000000LL;

    esp_err_t last_err = ESP_OK;
    heartbeat_sample_t last_sample = {0};

    while (esp_timer_get_time() < deadline_us) {
        last_err = heartbeat_read(ctx, &last_sample);
        if (last_err != ESP_OK) {
            ESP_LOGW(TAG_SESSION, "heartbeat_read failed: %s", esp_err_to_name(last_err));
            vTaskDelay(pdMS_TO_TICKS(p.poll_interval_ms));
            continue;
        }
        bool enough_time = (esp_timer_get_time() >= min_ready_us);
        bool ok_contact = (!p.require_contact) || last_sample.contact;
        bool ok_metrics = last_sample.hr_valid && last_sample.spo2_valid;
        if (enough_time && ok_contact && ok_metrics) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(p.poll_interval_ms));
    }

    // stop the sensor regardless of outcome
    (void)heartbeat_stop(ctx);

    // copy out the last sample
    *out = last_sample;

    bool ok_contact = (!p.require_contact) || last_sample.contact;
    bool ok_metrics = last_sample.hr_valid && last_sample.spo2_valid;
    bool met_time = (esp_timer_get_time() >= min_ready_us);

    if (!met_time) {
        return ESP_ERR_TIMEOUT;
    }
    if (!ok_contact) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!ok_metrics) {
        return ESP_ERR_NOT_FINISHED;
    }
    return ESP_OK;
}
