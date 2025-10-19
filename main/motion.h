#ifndef HABIZAP_MOTION_H
#define HABIZAP_MOTION_H

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDR_7BIT 0x68

// Data container for one sensor sample
typedef struct {
    float ax, ay, az;   // acceleration in g
    float gx, gy, gz;   // angular rate in dps
    float temp_c;       // temperature in Celsius
} motion_sample_t;

// Initialize the MPU6050 device on a pre-initialized I2C master bus.
// bus must be created in app.c via i2c_new_master_bus and kept alive.
// scl_speed_hz allows overriding the default per-device SCL speed (pass 0 for default 100kHz).
esp_err_t motion_init(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz);

// Read one sample (accel, gyro, temp)
esp_err_t motion_read(motion_sample_t *out);

// Deinitialize the device and release the device handle (bus remains owned by app.c)
void motion_deinit(void);

#ifdef __cplusplus
}
#endif

#endif //HABIZAP_MOTION_H