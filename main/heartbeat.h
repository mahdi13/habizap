#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// Raw sample from MAX30102
typedef struct {
    uint32_t red;      // 18-bit value (0..262143)
    uint32_t ir;       // 18-bit value (0..262143)
    float hr_bpm;      // optional: simple placeholder, not clinically accurate
    float spo2_pct;    // optional: simple placeholder, not clinically accurate
} heartbeat_sample_t;

// Initialize MAX30102 device on provided I2C bus
esp_err_t heartbeat_init(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz);

// Read a sample (RED + IR) from FIFO and compute naive HR/SpO2 placeholders
esp_err_t heartbeat_read(heartbeat_sample_t *out);

// Remove device from I2C bus
void heartbeat_deinit(void);

#ifdef __cplusplus
}
#endif
