#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "heartbeat/heartbeat.h"

#ifdef __cplusplus
extern "C" {
#endif

// Parameters for a one-shot measurement session.
typedef struct {
    // Minimum decimated seconds (at 25 Hz) to consider before evaluating validity.
    // Recommend 3-4 seconds for stable results.
    uint32_t min_seconds;
    // Maximum seconds to keep sampling before giving up (sensor will be stopped regardless).
    // If 0, defaults to 10 seconds.
    uint32_t max_seconds;
    // If true, require contact detection to return ESP_OK; otherwise returns ESP_ERR_INVALID_STATE
    // when time limit is reached without valid contact.
    bool require_contact;
    // Polling interval in milliseconds for reading FIFO (default 50 ms if 0).
    uint32_t poll_interval_ms;
} heartbeat_session_params_t;

// Runs a measurement session: starts the sensor, samples until HR/SpO2 are valid
// or until max_seconds expires, then stops the sensor and returns the last sample.
// Returns ESP_OK if both HR and SpO2 are valid under the chosen policy; otherwise
// returns an error code (ESP_ERR_TIMEOUT, ESP_ERR_INVALID_STATE, etc.), while still
// filling 'out' with the last available sample.
esp_err_t heartbeat_session_run(heartbeat_ctx_t *ctx,
                                const heartbeat_session_params_t *params,
                                heartbeat_sample_t *out);

#ifdef __cplusplus
}
#endif
