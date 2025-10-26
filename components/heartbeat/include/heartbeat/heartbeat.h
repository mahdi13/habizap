#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// Raw sample and derived metrics from MAX30102
typedef struct {
    uint32_t red;      // 18-bit value (0..262143)
    uint32_t ir;       // 18-bit value (0..262143)
    float hr_bpm;      // derived, not medically validated
    float spo2_pct;    // derived, not medically validated
    float temp_c;      // sensor die temperature in Celsius
    // Additional context/quality info
    uint16_t samples_in_window; // number of decimated samples contributing to metrics
    float seconds_in_window;    // seconds covered by samples_in_window
    bool contact;               // true if finger/skin detected
    bool hr_valid;              // true if hr_bpm is considered valid
    bool spo2_valid;            // true if spo2_pct is considered valid
} heartbeat_sample_t;

// Driver context (public to allow stack allocation and zero-heap design)
typedef struct heartbeat_ctx_s {
    const char *tag;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint32_t scl_speed_hz;
    // Decimated ring buffers at 25 Hz (4 s window => 100 samples)
    uint32_t ir_buf[25 * 4];
    uint32_t red_buf[25 * 4];
    int buf_count;
    int buf_index;
    // Decimation (100 Hz -> 25 Hz by averaging 4 raw samples)
    uint32_t decim_ir_acc;
    uint32_t decim_red_acc;
    int decim_count;
    uint32_t last_decim_ir;
    uint32_t last_decim_red;
    float last_hr;
    float last_spo2;
    float last_temp;
    struct { bool inited; float x; float P; } kf_hr;
    struct { bool inited; float x; float P; } kf_spo2;
    bool contact;
    uint32_t ir_dc_avg;
    uint32_t temp_next_ms;
    bool started;
} heartbeat_ctx_t;

// Create/initialize the heartbeat driver with provided I2C bus.
// The caller owns the ctx storage (no heap allocations inside the driver).
esp_err_t heartbeat_init(heartbeat_ctx_t *ctx,
                         i2c_master_bus_handle_t bus,
                         uint32_t scl_speed_hz);

// Start sensor measurements (enable mode, reset buffers). Safe to call multiple times.
esp_err_t heartbeat_start(heartbeat_ctx_t *ctx);

// Read new samples, update internal algorithms, and output latest results.
esp_err_t heartbeat_read(heartbeat_ctx_t *ctx, heartbeat_sample_t *out);

// Stop measurements and put device in low-power state.
esp_err_t heartbeat_stop(heartbeat_ctx_t *ctx);

// Remove device from I2C bus and clear context state.
void heartbeat_deinit(heartbeat_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
