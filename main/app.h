#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t i2c_bus; // shared I2C bus for the app
    bool motion_ready;               // flag for MPU6050 availability
    void *motion_retry_timer;        // esp_timer_handle_t for motion retry (opaque here)
    volatile bool motion_retry_due;  // set by timer callback when retry should occur
} app_context_t;

app_context_t *app_ctx(void);

void app_run(void);

#ifdef __cplusplus
}
#endif
