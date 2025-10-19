#include "motion.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// MPU6050 registers
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_INT_PIN_CFG     0x37
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_INT_STATUS      0x3A
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

#define WHO_AM_I_EXPECTED           0x68

static const char *TAG = "MOTION";
static i2c_master_dev_handle_t s_mpu = NULL;
static uint8_t s_addr_in_use = MPU6050_I2C_ADDR_7BIT;

static inline esp_err_t mpu_write(uint8_t reg, const uint8_t *data, size_t len) {
    if (!s_mpu) return ESP_ERR_INVALID_STATE;
    uint8_t buf[1 + 16];
    if (len > 16) {
        // for simplicity; can be extended if needed
        ESP_LOGE(TAG, "write too long: %u", (unsigned)len);
        return ESP_ERR_INVALID_ARG;
    }
    buf[0] = reg;
    if (data && len) memcpy(&buf[1], data, len);
    esp_err_t err = i2c_master_transmit(s_mpu, buf, 1 + len, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_transmit reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "W reg 0x%02X (%uB)", reg, (unsigned)len);
    }
    return err;
}

static inline esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len) {
    if (!s_mpu) return ESP_ERR_INVALID_STATE;
    esp_err_t err = i2c_master_transmit_receive(s_mpu, &reg, 1, data, len, 1000);
    if (err == ESP_OK) {
        ESP_LOGD(TAG, "R reg 0x%02X -> ok (%uB)", reg, (unsigned)len);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "i2c_master_transmit_receive reg 0x%02X len %u failed: %s", reg, (unsigned)len, esp_err_to_name(err));
    // Fallback: try two-step (write reg pointer, then read)
    esp_err_t e1 = i2c_master_transmit(s_mpu, &reg, 1, 1000);
    if (e1 != ESP_OK) {
        ESP_LOGE(TAG, "fallback transmit reg 0x%02X failed: %s", reg, esp_err_to_name(e1));
        return err; // keep original error context
    }
    esp_err_t e2 = i2c_master_receive(s_mpu, data, len, 1000);
    if (e2 != ESP_OK) {
        ESP_LOGE(TAG, "fallback receive reg 0x%02X len %u failed: %s", reg, (unsigned)len, esp_err_to_name(e2));
        return err; // keep original error
    }
    ESP_LOGW(TAG, "R reg 0x%02X succeeded via fallback two-step (%uB)", reg, (unsigned)len);
    return ESP_OK;
}

esp_err_t motion_init(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz) {
    ESP_LOGI(TAG, "motion_init: configuring MPU6050 (preferred addr=0x%02X)", MPU6050_I2C_ADDR_7BIT);
    if (!bus) {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (scl_speed_hz == 0) scl_speed_hz = 100000; // default 100kHz

    // Try both possible 7-bit addresses (AD0=0 -> 0x68, AD0=1 -> 0x69)
    const uint8_t candidates[2] = { 0x68, 0x69 };
    esp_err_t err = ESP_FAIL;
    s_addr_in_use = 0;
    for (size_t i = 0; i < 2; ++i) {
        uint8_t addr = candidates[i];
        esp_err_t p = i2c_master_probe(bus, addr, 1000);
        ESP_LOGI(TAG, "Probe 0x%02X -> %s", addr, esp_err_to_name(p));
        if (p == ESP_OK) { s_addr_in_use = addr; break; }
    }
    if (s_addr_in_use == 0) {
        // As a fallback, still try the default address
        s_addr_in_use = MPU6050_I2C_ADDR_7BIT;
        ESP_LOGW(TAG, "No address responded; falling back to 0x%02X", s_addr_in_use);
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_addr_in_use,
        .scl_speed_hz = (int)scl_speed_hz,
        .scl_wait_us = 0,
    };

    err = i2c_master_bus_add_device(bus, &dev_cfg, &s_mpu);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPU6050 device (0x%02X) to bus: %s", s_addr_in_use, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "MPU6050 device added to I2C bus (addr=0x%02X, SCL %u Hz)", s_addr_in_use, (unsigned)scl_speed_hz);

    // Give sensor time to power up after bus attach
    vTaskDelay(pdMS_TO_TICKS(50));

    // Verify WHO_AM_I (with fallbacks)
    uint8_t who = 0;
    err = mpu_read(MPU6050_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WHO_AM_I read failed at %u Hz: %s", (unsigned)scl_speed_hz, esp_err_to_name(err));
        // Fallback 1: re-add device at 100kHz and retry
        i2c_master_bus_rm_device(s_mpu);
        s_mpu = NULL;
        i2c_device_config_t slow_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = s_addr_in_use,
            .scl_speed_hz = 100000,
            .scl_wait_us = 0,
        };
        esp_err_t eadd = i2c_master_bus_add_device(bus, &slow_cfg, &s_mpu);
        if (eadd == ESP_OK) {
            ESP_LOGW(TAG, "Re-added MPU6050 at 100kHz for retry");
            vTaskDelay(pdMS_TO_TICKS(10));
            err = mpu_read(MPU6050_REG_WHO_AM_I, &who, 1);
        } else {
            ESP_LOGE(TAG, "Failed to re-add device at 100kHz: %s", esp_err_to_name(eadd));
        }
    }
    if (err != ESP_OK) {
        // Fallback 2: try explicitly waking device then read again
        uint8_t zero = 0x00;
        ESP_LOGW(TAG, "Attempting wake (PWR_MGMT_1=0x00) before another WHO_AM_I read");
        (void)mpu_write(MPU6050_REG_PWR_MGMT_1, &zero, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        err = mpu_read(MPU6050_REG_WHO_AM_I, &who, 1);
    }
    if (err != ESP_OK) return err;
    if (who != WHO_AM_I_EXPECTED) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch at addr 0x%02X: got 0x%02X expected 0x%02X", s_addr_in_use, who, WHO_AM_I_EXPECTED);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "WHO_AM_I ok (0x%02X) at addr 0x%02X", who, s_addr_in_use);

    // Wake up: clear sleep bit in PWR_MGMT_1
    uint8_t pwr = 0;
    err = mpu_read(MPU6050_REG_PWR_MGMT_1, &pwr, 1);
    if (err != ESP_OK) return err;
    uint8_t new_pwr = (uint8_t)(pwr & ~(1 << 6));
    if (new_pwr != pwr) {
        err = mpu_write(MPU6050_REG_PWR_MGMT_1, &new_pwr, 1);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "MPU6050 woken up (PWR_MGMT_1=0x%02X->0x%02X)", pwr, new_pwr);

    // Set DLPF to 42Hz for accel/gyro (CONFIG register DLPF_CFG=3) to reduce noise
    uint8_t cfg = 0;
    err = mpu_read(MPU6050_REG_CONFIG, &cfg, 1);
    if (err != ESP_OK) return err;
    uint8_t new_cfg = (cfg & ~0x07) | 0x03;
    if (new_cfg != cfg) {
        err = mpu_write(MPU6050_REG_CONFIG, &new_cfg, 1);
        if (err != ESP_OK) return err;
    }

    // Configure accel = ±4g (ACCEL_CONFIG[4:3]=01) and gyro = ±500dps (GYRO_CONFIG[4:3]=01)
    uint8_t accel_cfg = 0, gyro_cfg = 0;
    err = mpu_read(MPU6050_REG_ACCEL_CONFIG, &accel_cfg, 1);
    if (err != ESP_OK) return err;
    err = mpu_read(MPU6050_REG_GYRO_CONFIG, &gyro_cfg, 1);
    if (err != ESP_OK) return err;

    uint8_t new_accel_cfg = (accel_cfg & ~(0x3 << 3)) | (1 << 3);
    uint8_t new_gyro_cfg  = (gyro_cfg  & ~(0x3 << 3)) | (1 << 3);

    if (new_accel_cfg != accel_cfg) {
        err = mpu_write(MPU6050_REG_ACCEL_CONFIG, &new_accel_cfg, 1);
        if (err != ESP_OK) return err;
    }
    if (new_gyro_cfg != gyro_cfg) {
        err = mpu_write(MPU6050_REG_GYRO_CONFIG, &new_gyro_cfg, 1);
        if (err != ESP_OK) return err;
    }
    ESP_LOGI(TAG, "Configured ranges: Accel=±4g, Gyro=±500 dps");

    // Set sample rate to 1kHz / (1 + SMPLRT_DIV); choose 100Hz -> divider 9
    uint8_t smpl = 9;
    err = mpu_write(MPU6050_REG_SMPLRT_DIV, &smpl, 1);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t motion_read(motion_sample_t *out) {
    if (!s_mpu) {
        ESP_LOGE(TAG, "motion_read called before motion_init");
        return ESP_ERR_INVALID_STATE;
    }
    if (!out) return ESP_ERR_INVALID_ARG;

    uint8_t buf[6];
    esp_err_t err;

    // Read accel
    err = mpu_read(MPU6050_REG_ACCEL_XOUT_H, buf, 6);
    if (err != ESP_OK) return err;
    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);

    // sensitivity for ±4g
    const float acc_sens = 8192.0f;
    out->ax = ax_raw / acc_sens;
    out->ay = ay_raw / acc_sens;
    out->az = az_raw / acc_sens;

    // Read gyro
    err = mpu_read(MPU6050_REG_GYRO_XOUT_H, buf, 6);
    if (err != ESP_OK) return err;
    int16_t gx_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t gy_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t gz_raw = (int16_t)((buf[4] << 8) | buf[5]);

    // sensitivity for ±500 dps
    const float gyro_sens = 65.5f;
    out->gx = gx_raw / gyro_sens;
    out->gy = gy_raw / gyro_sens;
    out->gz = gz_raw / gyro_sens;

    // Read temp
    err = mpu_read(MPU6050_REG_TEMP_OUT_H, buf, 2);
    if (err != ESP_OK) return err;
    int16_t t_raw = (int16_t)((buf[0] << 8) | buf[1]);
    out->temp_c = (float)t_raw / 340.0f + 36.53f;

    ESP_LOGD(TAG, "Sample: A(%.3f,%.3f,%.3f) G(%.3f,%.3f,%.3f) T=%.2fC", out->ax, out->ay, out->az, out->gx, out->gy, out->gz, out->temp_c);
    return ESP_OK;
}

void motion_deinit(void) {
    if (s_mpu) {
        ESP_LOGI(TAG, "Removing MPU6050 device from bus");
        i2c_master_bus_rm_device(s_mpu);
        s_mpu = NULL;
    }
}