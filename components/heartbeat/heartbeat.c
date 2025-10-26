#include "heartbeat/heartbeat.h"
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "heartbeat/algo_maxim.h"

#define MAX30102_I2C_ADDR_7BIT   0x57

// MAX30102 Registers
#define REG_INT_STATUS1          0x00
#define REG_INT_STATUS2          0x01
#define REG_INT_ENABLE1          0x02
#define REG_INT_ENABLE2          0x03
#define REG_FIFO_WR_PTR          0x04
#define REG_OVF_COUNTER          0x05
#define REG_FIFO_RD_PTR          0x06
#define REG_FIFO_DATA            0x07
#define REG_FIFO_CONFIG          0x08
#define REG_MODE_CONFIG          0x09
#define REG_SPO2_CONFIG          0x0A
#define REG_LED1_PA              0x0C  // RED
#define REG_LED2_PA              0x0D  // IR
#define REG_MULTI_LED_CTRL1      0x11
#define REG_MULTI_LED_CTRL2      0x12
#define REG_TEMP_INT             0x1F
#define REG_TEMP_FRAC            0x20
#define REG_TEMP_CONFIG          0x21
#define REG_REV_ID               0xFE
#define REG_PART_ID              0xFF

#define PART_ID_EXPECTED         0x15

#define HB_ALGO_FS           25
#define HB_WINDOW_S          4
#define HB_BUF_LEN           (HB_ALGO_FS * HB_WINDOW_S)  // 100 decimated samples (25 Hz * 4 s)

// Kalman structs are public inside heartbeat_ctx_t (see header)

static const char *TAG = "HEARTBEAT";

#define IR_CONTACT_ENTER_THRESH  8000u
#define IR_CONTACT_EXIT_THRESH   6000u

static inline uint32_t millis(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

static inline esp_err_t max_write(heartbeat_ctx_t *ctx, uint8_t reg, const uint8_t *data, size_t len) {
    if (!ctx || !ctx->dev) return ESP_ERR_INVALID_STATE;
    uint8_t buf[1 + 16];
    if (len > 16) return ESP_ERR_INVALID_ARG;
    buf[0] = reg;
    if (data && len) memcpy(&buf[1], data, len);
    esp_err_t err = i2c_master_transmit(ctx->dev, buf, 1 + len, 1000);
    if (err != ESP_OK) ESP_LOGE(TAG, "I2C TX reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    return err;
}

static inline esp_err_t max_read(heartbeat_ctx_t *ctx, uint8_t reg, uint8_t *data, size_t len) {
    if (!ctx || !ctx->dev) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(ctx->dev, &reg, 1, data, len, 1000);
}

static esp_err_t max_reset(heartbeat_ctx_t *ctx) {
    uint8_t v = 0x40; // reset
    esp_err_t err = max_write(ctx, REG_MODE_CONFIG, &v, 1);
    if (err != ESP_OK) return err;
    for (int i = 0; i < 20; ++i) {
        v = 0;
        err = max_read(ctx, REG_MODE_CONFIG, &v, 1);
        if (err != ESP_OK) return err;
        if ((v & 0x40) == 0) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_OK;
}

static inline uint32_t read_18b(const uint8_t *p) {
    uint32_t v = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
    return v & 0x3FFFFu;
}

static void compute_hr_spo2(const uint32_t *ir, const uint32_t *red, int len, float *hr_bpm, float *spo2_pct) {
    if (len < (HB_ALGO_FS * 3)) {
        if (hr_bpm) *hr_bpm = 0.0f;
        if (spo2_pct) *spo2_pct = 0.0f;
        return;
    }
    // Call Maxim algorithm on 25 Hz buffers (expects 4s window length but supports >=3s reasonably)
    int32_t spo2 = 0, hr = 0;
    int8_t spo2_valid = 0, hr_valid = 0;
    maxim_heart_rate_and_oxygen_saturation((uint32_t *)ir, len, (uint32_t *)red, &spo2, &spo2_valid, &hr, &hr_valid);
    if (hr_bpm) *hr_bpm = (hr_valid && hr > 0) ? (float)hr : 0.0f;
    if (spo2_pct) *spo2_pct = (spo2_valid && spo2 > 0) ? (float)spo2 : 0.0f;
}

static esp_err_t configure_running_mode(heartbeat_ctx_t *ctx) {
    // FIFO config: avg 4, rollover, A_FULL default
    uint8_t fifo_cfg = 0x40 | 0x10;
    esp_err_t err;
    err = max_write(ctx, REG_FIFO_CONFIG, &fifo_cfg, 1); if (err != ESP_OK) return err;
    uint8_t zero = 0;
    err = max_write(ctx, REG_FIFO_WR_PTR, &zero, 1); if (err != ESP_OK) return err;
    err = max_write(ctx, REG_OVF_COUNTER, &zero, 1); if (err != ESP_OK) return err;
    err = max_write(ctx, REG_FIFO_RD_PTR, &zero, 1); if (err != ESP_OK) return err;
    uint8_t int_en1 = 0xC0; // A_FULL + PPG_RDY
    err = max_write(ctx, REG_INT_ENABLE1, &int_en1, 1); if (err != ESP_OK) return err;
    uint8_t int_en2 = 0x02; // TEMP_RDY
    err = max_write(ctx, REG_INT_ENABLE2, &int_en2, 1); if (err != ESP_OK) return err;
    uint8_t spo2_cfg = (1u << 5) | (1u << 2) | 3u; // 0x27
    err = max_write(ctx, REG_SPO2_CONFIG, &spo2_cfg, 1); if (err != ESP_OK) return err;
    uint8_t led = 0x24;
    err = max_write(ctx, REG_LED1_PA, &led, 1); if (err != ESP_OK) return err;
    err = max_write(ctx, REG_LED2_PA, &led, 1); if (err != ESP_OK) return err;
    uint8_t mode = 0x03; // SpO2 mode
    err = max_write(ctx, REG_MODE_CONFIG, &mode, 1); if (err != ESP_OK) return err;
    return ESP_OK;
}

esp_err_t heartbeat_init(heartbeat_ctx_t *ctx, i2c_master_bus_handle_t bus, uint32_t scl_speed_hz) {
    if (!ctx || !bus) return ESP_ERR_INVALID_ARG;
    memset(ctx, 0, sizeof(*ctx));
    ctx->tag = TAG;
    ctx->bus = bus;
    ctx->scl_speed_hz = scl_speed_hz ? scl_speed_hz : 100000;

    // Probe device (non-fatal)
    i2c_master_probe(bus, MAX30102_I2C_ADDR_7BIT, 1000);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30102_I2C_ADDR_7BIT,
        .scl_speed_hz = (int)ctx->scl_speed_hz,
        .scl_wait_us = 0,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &ctx->dev);
    if (err != ESP_OK) return err;

    // Verify part id (non-fatal warn)
    uint8_t part = 0, rev = 0;
    if (max_read(ctx, REG_PART_ID, &part, 1) == ESP_OK && max_read(ctx, REG_REV_ID, &rev, 1) == ESP_OK) {
        if (part != PART_ID_EXPECTED) {
            ESP_LOGW(TAG, "Unexpected PART_ID: 0x%02X (rev 0x%02X)", part, rev);
        } else {
            ESP_LOGI(TAG, "MAX30102 detected: PART_ID=0x%02X REV=0x%02X", part, rev);
        }
    }

    // Reset the device, but do not start yet. Caller must call heartbeat_start() to begin sampling.
    err = max_reset(ctx);
    if (err != ESP_OK) return err;
    return ESP_OK;
}

esp_err_t heartbeat_start(heartbeat_ctx_t *ctx) {
    if (!ctx || !ctx->dev) return ESP_ERR_INVALID_STATE;
    esp_err_t err = configure_running_mode(ctx);
    if (err != ESP_OK) return err;
    ctx->buf_count = 0; ctx->buf_index = 0;
    ctx->decim_ir_acc = 0; ctx->decim_red_acc = 0; ctx->decim_count = 0;
    ctx->last_decim_ir = 0; ctx->last_decim_red = 0;
    ctx->last_hr = ctx->last_spo2 = ctx->last_temp = 0.0f;
    ctx->kf_hr.inited = false; ctx->kf_hr.x = 0.0f; ctx->kf_hr.P = 0.0f;
    ctx->kf_spo2.inited = false; ctx->kf_spo2.x = 0.0f; ctx->kf_spo2.P = 0.0f;
    ctx->contact = false;
    ctx->ir_dc_avg = 0;
    ctx->temp_next_ms = 0;
    ctx->started = true;
    return ESP_OK;
}

static esp_err_t read_fifo_and_update_buffers(heartbeat_ctx_t *ctx, uint32_t *last_red, uint32_t *last_ir) {
    if (!ctx || !ctx->dev) return ESP_ERR_INVALID_STATE;
    uint8_t st1 = 0;
    esp_err_t err = max_read(ctx, REG_INT_STATUS1, &st1, 1);
    if (err != ESP_OK) return err;
    (void)st1;
    // Read as many 6-byte samples as available in FIFO (up to 32)
    uint8_t rd_ptr=0, wr_ptr=0;
    err = max_read(ctx, REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (err != ESP_OK) return err;
    err = max_read(ctx, REG_FIFO_WR_PTR, &wr_ptr, 1);
    if (err != ESP_OK) return err;
    int n = (int)((wr_ptr - rd_ptr) & 0x1F); // modulo 32
    if (n <= 0) return ESP_OK;

    for (int i = 0; i < n; ++i) {
        uint8_t s[6];
        err = max_read(ctx, REG_FIFO_DATA, s, 6);
        if (err != ESP_OK) return err;
        uint32_t red_raw = read_18b(&s[0]);
        uint32_t ir_raw  = read_18b(&s[3]);
        *last_red = red_raw; *last_ir = ir_raw;

        // update dc average for contact detect on raw IR
        if (ctx->ir_dc_avg == 0) ctx->ir_dc_avg = ir_raw; else ctx->ir_dc_avg = (ctx->ir_dc_avg * 15 + ir_raw) / 16;
        if (!ctx->contact && ctx->ir_dc_avg > IR_CONTACT_ENTER_THRESH) ctx->contact = true;
        if (ctx->contact && ctx->ir_dc_avg < IR_CONTACT_EXIT_THRESH) ctx->contact = false;

        // decimate by 4: accumulate and every 4 samples push average to ring at 25 Hz
        ctx->decim_ir_acc += ir_raw;
        ctx->decim_red_acc += red_raw;
        ctx->decim_count++;
        if (ctx->decim_count >= 4) {
            uint32_t ir = ctx->decim_ir_acc / (uint32_t)ctx->decim_count; // usually 4
            uint32_t red = ctx->decim_red_acc / (uint32_t)ctx->decim_count;
            ctx->last_decim_ir = ir;
            ctx->last_decim_red = red;
            ctx->decim_ir_acc = 0;
            ctx->decim_red_acc = 0;
            ctx->decim_count = 0;

            // ring buffer at decimated rate
            ctx->red_buf[ctx->buf_index] = red;
            ctx->ir_buf[ctx->buf_index] = ir;
            ctx->buf_index = (ctx->buf_index + 1) % HB_BUF_LEN;
            if (ctx->buf_count < HB_BUF_LEN) ctx->buf_count++;
        }
    }
    return ESP_OK;
}

esp_err_t heartbeat_read(heartbeat_ctx_t *ctx, heartbeat_sample_t *out) {
    if (!ctx || !ctx->dev || !out) return ESP_ERR_INVALID_ARG;
    if (!ctx->started) return ESP_ERR_INVALID_STATE;

    uint32_t last_red = 0, last_ir = 0;
    {
        esp_err_t err = read_fifo_and_update_buffers(ctx, &last_red, &last_ir);
        if (err != ESP_OK) return err;
    }

    // temperature once per ~2s
    uint32_t now = millis();
    if (now >= ctx->temp_next_ms) {
        uint8_t v = 0x01;
        (void)max_write(ctx, REG_TEMP_CONFIG, &v, 1);
        uint32_t start = now;
        while (millis() - start < 100) {
            uint8_t st2 = 0;
            if (max_read(ctx, REG_INT_STATUS2, &st2, 1) == ESP_OK && (st2 & 0x02)) break;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        int8_t ti = 0; uint8_t tf = 0;
        if (max_read(ctx, REG_TEMP_INT, (uint8_t *)&ti, 1) == ESP_OK && max_read(ctx, REG_TEMP_FRAC, &tf, 1) == ESP_OK) {
            ctx->last_temp = (float)ti + ((float)tf * 0.0625f);
        }
        ctx->temp_next_ms = millis() + 2000;
    }

    // compute metrics on current window
    float hr = 0.0f, spo2 = 0.0f;
    if (ctx->buf_count >= HB_ALGO_FS * 3) {
        // prepare linear window starting at buf_index - buf_count
        uint32_t irw[HB_BUF_LEN];
        uint32_t redw[HB_BUF_LEN];
        for (int i = 0; i < ctx->buf_count; ++i) {
            int idx = (ctx->buf_index - ctx->buf_count + i + HB_BUF_LEN) % HB_BUF_LEN;
            irw[i] = ctx->ir_buf[idx];
            redw[i] = ctx->red_buf[idx];
        }
        compute_hr_spo2(irw, redw, ctx->buf_count, &hr, &spo2);
    }

    // Fill output
    // report last decimated values if available, else last raw
    out->red = ctx->last_decim_red ? ctx->last_decim_red : last_red;
    out->ir = ctx->last_decim_ir ? ctx->last_decim_ir : last_ir;
    out->hr_bpm = hr;
    out->spo2_pct = spo2;
    out->temp_c = ctx->last_temp;
    out->samples_in_window = (uint16_t)ctx->buf_count;
    out->seconds_in_window = (float)ctx->buf_count / (float)HB_ALGO_FS;
    out->contact = ctx->contact;
    // clamp and validate values
    if (out->spo2_pct < 0.0f) out->spo2_pct = 0.0f;
    if (out->spo2_pct > 100.0f) out->spo2_pct = 100.0f;
    out->hr_valid = (hr >= 40.0f && hr <= 180.0f) && ctx->contact;
    out->spo2_valid = (out->spo2_pct >= 85.0f && out->spo2_pct <= 100.0f) && ctx->contact;

    return ESP_OK;
}

esp_err_t heartbeat_stop(heartbeat_ctx_t *ctx) {
    if (!ctx || !ctx->dev) return ESP_ERR_INVALID_STATE;
    uint8_t mode = 0x00; // shutdown
    esp_err_t err = max_write(ctx, REG_MODE_CONFIG, &mode, 1);
    ctx->started = false;
    return err;
}

void heartbeat_deinit(heartbeat_ctx_t *ctx) {
    if (!ctx) return;
    if (ctx->dev) {
        (void)heartbeat_stop(ctx);
        i2c_master_bus_rm_device(ctx->dev);
        ctx->dev = NULL;
    }
    memset(ctx, 0, sizeof(*ctx));
}
