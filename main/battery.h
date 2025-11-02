#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Battery monitoring subsystem
 * - Initializes ADC for a configured channel
 * - Runs a background task to periodically sample battery voltage
 * - Computes and prints percentage to log output
 */

/** Initialize the battery monitoring subsystem (ADC + calibration).
 * Must be called before battery_start().
 */
esp_err_t battery_init(void);

/** Start the background task that periodically samples and logs battery level. */
esp_err_t battery_start(void);

/** Stop the background task and free resources. */
void battery_stop(void);

/** Get the latest computed battery percentage (0-100). */
int battery_get_percent(void);

/** Get the latest measured battery voltage in millivolts (scaled to battery side). */
int battery_get_voltage_mv(void);

#ifdef __cplusplus
}
#endif
