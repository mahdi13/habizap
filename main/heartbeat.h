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

// Initialize MAX30102 device on provided I2C bus
esp_err_t heartbeat_init(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz);

// Read one or more new samples from FIFO, update derived metrics, and output latest
esp_err_t heartbeat_read(heartbeat_sample_t *out);

// Remove device from I2C bus
void heartbeat_deinit(void);

#ifdef __cplusplus
}
#endif
