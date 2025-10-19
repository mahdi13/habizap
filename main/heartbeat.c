#include "heartbeat.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX30102_I2C_ADDR_7BIT   0x57

// MAX30102 Registers
#define REG_INT_STATUS1          0x00
#define REG_INT_ENABLE1          0x02
#define REG_FIFO_WR_PTR          0x04
#define REG_OVF_COUNTER          0x05
#define REG_FIFO_RD_PTR          0x06
#define REG_FIFO_DATA            0x07
#define REG_MODE_CONFIG          0x09
#define REG_SPO2_CONFIG          0x0A
#define REG_LED1_PA              0x0C  // RED
#define REG_LED2_PA              0x0D  // IR
#define REG_MULTI_LED_CTRL1      0x11
#define REG_MULTI_LED_CTRL2      0x12
#define REG_REV_ID               0xFE
#define REG_PART_ID              0xFF

#define PART_ID_EXPECTED         0x15

static const char *TAG = "HEARTBEAT";
static i2c_master_dev_handle_t s_max = NULL;

static inline esp_err_t max_write(uint8_t reg, const uint8_t *data, size_t len) {
    if (!s_max) return ESP_ERR_INVALID_STATE;
    uint8_t buf[1 + 16];
    if (len > 16) {
        ESP_LOGE(TAG, "write too long: %u", (unsigned)len);
        return ESP_ERR_INVALID_ARG;
    }
    buf[0] = reg;
    if (data && len) memcpy(&buf[1], data, len);
    esp_err_t err = i2c_master_transmit(s_max, buf, 1 + len, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C TX reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "W reg 0x%02X (%uB)", reg, (unsigned)len);
    }
    return err;
}

static inline esp_err_t max_read(uint8_t reg, uint8_t *data, size_t len) {
    if (!s_max) return ESP_ERR_INVALID_STATE;
    esp_err_t err = i2c_master_transmit_receive(s_max, &reg, 1, data, len, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C TXRX reg 0x%02X len %u failed: %s", reg, (unsigned)len, esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "R reg 0x%02X -> ok (%uB)", reg, (unsigned)len);
    }
    return err;
}

static esp_err_t max_reset(void) {
    // Set reset bit (bit 6) in MODE_CONFIG
    uint8_t v = 0x40;
    esp_err_t err = max_write(REG_MODE_CONFIG, &v, 1);
    if (err != ESP_OK) return err;
    // Wait until reset bit clears
    const int max_tries = 20;
    for (int i = 0; i < max_tries; ++i) {
        v = 0;
        err = max_read(REG_MODE_CONFIG, &v, 1);
        if (err != ESP_OK) return err;
        if ((v & 0x40) == 0) {
            ESP_LOGI(TAG, "MAX30102 reset complete (MODE_CONFIG=0x%02X)", v);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "MAX30102 reset bit did not clear in time");
    return ESP_OK; // continue anyway
}

esp_err_t heartbeat_init(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz) {
    ESP_LOGI(TAG, "heartbeat_init: configuring MAX30102 at 0x%02X", MAX30102_I2C_ADDR_7BIT);
    if (!bus) {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (scl_speed_hz == 0) scl_speed_hz = 100000; // default 100kHz

    // Probe device
    esp_err_t err = i2c_master_probe(bus, MAX30102_I2C_ADDR_7BIT, 1000);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_probe failed for 0x%02X: %s (continuing)", MAX30102_I2C_ADDR_7BIT, esp_err_to_name(err));
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30102_I2C_ADDR_7BIT,
        .scl_speed_hz = (int)scl_speed_hz,
        .scl_wait_us = 0,
    };

    err = i2c_master_bus_add_device(bus, &dev_cfg, &s_max);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MAX30102 device to bus: %s", esp_err_to_name(err));
        return err;
    }

    // Verify PART ID
    uint8_t part = 0, rev = 0;
    if ((err = max_read(REG_PART_ID, &part, 1)) != ESP_OK) return err;
    if ((err = max_read(REG_REV_ID, &rev, 1)) != ESP_OK) return err;
    if (part != PART_ID_EXPECTED) {
        ESP_LOGW(TAG, "Unexpected PART_ID: 0x%02X (rev 0x%02X), expected 0x%02X", part, rev, PART_ID_EXPECTED);
    } else {
        ESP_LOGI(TAG, "MAX30102 detected: PART_ID=0x%02X REV=0x%02X", part, rev);
    }

    // Reset
    if ((err = max_reset()) != ESP_OK) return err;

    // Clear FIFO pointers
    uint8_t zero = 0;
    if ((err = max_write(REG_FIFO_WR_PTR, &zero, 1)) != ESP_OK) return err;
    if ((err = max_write(REG_OVF_COUNTER, &zero, 1)) != ESP_OK) return err;
    if ((err = max_write(REG_FIFO_RD_PTR, &zero, 1)) != ESP_OK) return err;

    // Enable A_FULL and PPG_RDY interrupts (optional)
    uint8_t int_en1 = 0xC0; // A_FULL (bit7) + PPG_RDY (bit6)
    if ((err = max_write(REG_INT_ENABLE1, &int_en1, 1)) != ESP_OK) return err;

    // SPO2 config: ADC range=4096nA (1), sample rate=100Hz (3), pulse width=411us/18bit (3)
    uint8_t spo2_cfg = (1u << 5) | (3u << 2) | 3u; // 0x2F
    if ((err = max_write(REG_SPO2_CONFIG, &spo2_cfg, 1)) != ESP_OK) return err;

    // LED currents (approx 6-12mA); tune as needed
    uint8_t led_red = 0x24; // ~7.6mA
    uint8_t led_ir  = 0x24;
    if ((err = max_write(REG_LED1_PA, &led_red, 1)) != ESP_OK) return err;
    if ((err = max_write(REG_LED2_PA, &led_ir, 1)) != ESP_OK) return err;

    // Mode: SpO2 mode (0x03)
    uint8_t mode = 0x03;
    if ((err = max_write(REG_MODE_CONFIG, &mode, 1)) != ESP_OK) return err;

    ESP_LOGI(TAG, "MAX30102 initialized: SPO2 mode, SR=100Hz, PW=411us, LED=0x%02X", led_red);
    return ESP_OK;
}

static inline uint32_t read_18b(const uint8_t *p) {
    uint32_t v = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
    return v & 0x3FFFFu; // 18-bit mask
}

esp_err_t heartbeat_read(heartbeat_sample_t *out) {
    if (!s_max) {
        ESP_LOGE(TAG, "heartbeat_read called before heartbeat_init");
        return ESP_ERR_INVALID_STATE;
    }
    if (!out) return ESP_ERR_INVALID_ARG;

    // Read one RED+IR sample pair (6 bytes) from FIFO
    uint8_t buf[6] = {0};
    esp_err_t err = max_read(REG_FIFO_DATA, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    uint32_t red = read_18b(&buf[0]);
    uint32_t ir  = read_18b(&buf[3]);

    out->red = red;
    out->ir = ir;

    // Naive placeholders: do not claim accuracy
    out->hr_bpm = 0.0f;
    out->spo2_pct = 0.0f;

    ESP_LOGD(TAG, "MAX30102 sample: RED=%u IR=%u", (unsigned)red, (unsigned)ir);
    return ESP_OK;
}

void heartbeat_deinit(void) {
    if (s_max) {
        ESP_LOGI(TAG, "Removing MAX30102 device from bus");
        i2c_master_bus_rm_device(s_max);
        s_max = NULL;
    }
}