#include "heartbeat.h"
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

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

static const char *TAG = "HEARTBEAT";
static i2c_master_dev_handle_t s_max = NULL;

// Internal buffers and state for metrics
#define HB_DECIM_FACTOR      4        // 100 Hz -> 25 Hz for algorithm
#define HB_ALGO_FS           25       // samples per second expected by algorithm
#define HB_WINDOW_S          4        // 4 seconds
#define HB_BUF_LEN           (HB_ALGO_FS * HB_WINDOW_S) // 100 samples

static uint32_t s_ir_buf[HB_BUF_LEN];
static uint32_t s_red_buf[HB_BUF_LEN];
static int s_buf_count = 0;   // number of valid samples in buffers (<= HB_BUF_LEN)
static int s_buf_index = 0;   // next index to write (circular)
static int s_decim = 0;       // decimation counter
static float s_last_hr = 0.0f;
static float s_last_spo2 = 0.0f;
static float s_last_temp = 0.0f;
static uint32_t s_temp_tick = 0; // ms tick when next temp read is allowed

// Lightweight 1D Kalman filters for HR and SpO2 smoothing
typedef struct {
    bool inited;
    float x;   // state (estimate)
    float P;   // covariance
} kalman1d_t;

static kalman1d_t s_kf_hr = {0};
static kalman1d_t s_kf_spo2 = {0};

// Tuning parameters (can be adjusted as needed)
#define KF_HR_Q_CONTACT      0.05f   // process noise when contact present
#define KF_HR_R_MEAS         9.0f    // measurement noise variance (bpm^2)
#define KF_HR_Q_NOCONTACT    0.5f

#define KF_SPO2_Q_CONTACT    0.01f
#define KF_SPO2_R_MEAS       4.0f    // (%^2)
#define KF_SPO2_Q_NOCONTACT  0.2f

static inline void kf_predict(kalman1d_t *kf, float q) {
    if (!kf->inited) return;
    kf->P += q; // x is assumed constant model
}

static inline void kf_update(kalman1d_t *kf, float z, float r) {
    if (!kf->inited) {
        kf->x = z;
        kf->P = r; // initialize covariance around measurement noise
        kf->inited = true;
        return;
    }
    float S = kf->P + r;
    if (S <= 0.0f) S = r;
    float K = kf->P / S;
    kf->x = kf->x + K * (z - kf->x);
    kf->P = (1.0f - K) * kf->P;
}

// Simple contact detection based on IR DC level with hysteresis
static bool s_contact = false;
static uint32_t s_ir_dc_avg = 0; // low-pass filtered IR level (non-decimated)
#define IR_CONTACT_ENTER_THRESH  8000u
#define IR_CONTACT_EXIT_THRESH   6000u

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

static uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
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

// Read die temperature (C). Returns ESP_OK on success and updates s_last_temp
static esp_err_t max_read_temperature(void) {
    // Kick off one-shot temperature conversion
    uint8_t v = 0x01;
    esp_err_t err = max_write(REG_TEMP_CONFIG, &v, 1);
    if (err != ESP_OK) return err;
    // Wait up to 100ms for completion by polling INT_STATUS2 or TEMP_CONFIG bit0
    uint32_t start = millis();
    do {
        uint8_t st2 = 0;
        err = max_read(REG_INT_STATUS2, &st2, 1);
        if (err != ESP_OK) return err;
        if (st2 & 0x02) break; // DIE_TEMP_RDY
        vTaskDelay(pdMS_TO_TICKS(2));
    } while (millis() - start < 100);

    int8_t ti = 0; uint8_t tf = 0;
    if ((err = max_read(REG_TEMP_INT, (uint8_t *)&ti, 1)) != ESP_OK) return err;
    if ((err = max_read(REG_TEMP_FRAC, &tf, 1)) != ESP_OK) return err;
    s_last_temp = (float)ti + ((float)tf * 0.0625f);
    return ESP_OK;
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

    // FIFO config: average 4 samples, rollover enabled, almost full at 32-4=28
    uint8_t fifo_cfg = 0;
    // set sample average = 4 (bits 7:5 = 010)
    fifo_cfg |= 0x40;
    // enable rollover (bit4=1)
    fifo_cfg |= 0x10;
    // A_FULL = 0 (32 samples) leave as 0
    if ((err = max_write(REG_FIFO_CONFIG, &fifo_cfg, 1)) != ESP_OK) return err;

    // Clear FIFO pointers
    uint8_t zero = 0;
    if ((err = max_write(REG_FIFO_WR_PTR, &zero, 1)) != ESP_OK) return err;
    if ((err = max_write(REG_OVF_COUNTER, &zero, 1)) != ESP_OK) return err;
    if ((err = max_write(REG_FIFO_RD_PTR, &zero, 1)) != ESP_OK) return err;

    // Enable A_FULL and PPG_RDY interrupts (optional) and TEMP_RDY
    uint8_t int_en1 = 0xC0; // A_FULL (bit7) + PPG_RDY (bit6)
    if ((err = max_write(REG_INT_ENABLE1, &int_en1, 1)) != ESP_OK) return err;
    uint8_t int_en2 = 0x02; // DIE_TEMP_RDY
    if ((err = max_write(REG_INT_ENABLE2, &int_en2, 1)) != ESP_OK) return err;

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

    // Reset buffers/state
    s_buf_count = 0; s_buf_index = 0; s_decim = 0;
    s_last_hr = 0.0f; s_last_spo2 = 0.0f; s_last_temp = 0.0f; s_temp_tick = 0;

    ESP_LOGI(TAG, "MAX30102 initialized: SPO2 mode, SR=100Hz, PW=411us, LED=0x%02X", led_red);
    return ESP_OK;
}

static inline uint32_t read_18b(const uint8_t *p) {
    uint32_t v = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
    return v & 0x3FFFFu; // 18-bit mask
}

// --- Minimal port of Maxim SpO2/HR algorithm (expects 25 Hz, 4 sec window = 100 samples) ---
static void compute_hr_spo2(const uint32_t *ir, const uint32_t *red, int len, float *hr_bpm, float *spo2_pct) {
    // Based on spo2_algorithm.cpp (Maxim). This is a compact, C-only adaptation.
    if (len < HB_BUF_LEN) {
        if (hr_bpm) *hr_bpm = 0.0f;
        if (spo2_pct) *spo2_pct = 0.0f;
        return;
    }
    // Work arrays
    int32_t an_x[HB_BUF_LEN];
    int32_t an_y[HB_BUF_LEN];
    // Remove DC and invert for peak detection
    uint64_t sum_ir = 0;
    for (int i = 0; i < len; ++i) sum_ir += ir[i];
    uint32_t mean_ir = (uint32_t)(sum_ir / len);
    for (int i = 0; i < len; ++i) an_x[i] = -(int32_t)(ir[i] - mean_ir);
    // 4-pt moving average
    for (int i = 0; i < len - 4; ++i) an_x[i] = (an_x[i] + an_x[i+1] + an_x[i+2] + an_x[i+3]) / 4;
    // Threshold 30..60
    int32_t th = 0; for (int i = 0; i < HB_BUF_LEN; ++i) th += an_x[i]; th /= HB_BUF_LEN; if (th < 30) th = 30; if (th > 60) th = 60;
    // Find peaks (valleys in original signal)
    int32_t locs[15]; int32_t npks = 0;
    // peaks above min height
    int i = 1;
    while (i < HB_BUF_LEN - 1) {
        if (an_x[i] > th && an_x[i] > an_x[i-1]) {
            int width = 1; while (i+width < HB_BUF_LEN && an_x[i] == an_x[i+width]) width++;
            if (an_x[i] > an_x[i+width] && npks < 15) { locs[npks++] = i; i += width + 1; }
            else i += width;
        } else i++;
    }
    // Remove close peaks (<4)
    // Sort indices by peak height descending
    int32_t idx[15]; for (i=0;i<npks;i++) idx[i]=locs[i];
    // insertion sort by an_x[idx] desc
    for (int a=1;a<npks;a++){int32_t t=idx[a];int b=a;while(b>0 && an_x[t]>an_x[idx[b-1]]){idx[b]=idx[b-1];b--;}idx[b]=t;}
    int32_t new_locs[15]; int new_n=0;
    for (int a=-1; a<npks; a++){
        int old_n = npks; int tmp_n = a+1; new_n = tmp_n;
        for (int j=a+1;j<old_n;j++){
            int dist = idx[j] - (a==-1 ? -1 : idx[a]);
            if (dist > 4 || dist < -4) { new_locs[new_n++] = idx[j]; }
        }
        npks = new_n; for (int k=0;k<npks;k++) idx[k]=new_locs[k];
    }
    // sort ascending
    for (int a=1;a<npks;a++){int32_t t=idx[a];int b=a;while(b>0 && t<idx[b-1]){idx[b]=idx[b-1];b--;}idx[b]=t;}

    float hr = 0.0f;
    if (npks >= 2) {
        int32_t interval_sum = 0; for (int k=1;k<npks;k++) interval_sum += (idx[k]-idx[k-1]);
        interval_sum /= (npks - 1);
        hr = (float)(HB_ALGO_FS * 60) / (float)interval_sum; // bpm
    }

    // Prepare for SpO2
    for (i=0;i<len;i++){ an_x[i] = (int32_t)ir[i]; an_y[i] = (int32_t)red[i]; }
    int exact_n = npks;
    int32_t ratios[5]; int ratio_count=0; for (i=0;i<5;i++) ratios[i]=0;
    for (i=0;i<exact_n;i++) if (idx[i] > HB_BUF_LEN) { if (hr_bpm) *hr_bpm=hr; if (spo2_pct) *spo2_pct=0.0f; return; }
    int32_t y_dc_max, x_dc_max, y_dc_max_idx=0, x_dc_max_idx=0;
    for (int k=0; k<exact_n-1; k++){
        y_dc_max = INT32_MIN; x_dc_max = INT32_MIN;
        if (idx[k+1]-idx[k] > 3) {
            for (int j=idx[k]; j<idx[k+1]; j++){
                if (an_x[j] > x_dc_max) { x_dc_max = an_x[j]; x_dc_max_idx = j; }
                if (an_y[j] > y_dc_max) { y_dc_max = an_y[j]; y_dc_max_idx = j; }
            }
            int32_t y_ac = (an_y[idx[k+1]] - an_y[idx[k]]) * (y_dc_max_idx - idx[k]);
            y_ac = an_y[idx[k]] + y_ac / (idx[k+1] - idx[k]);
            y_ac = an_y[y_dc_max_idx] - y_ac;
            int32_t x_ac = (an_x[idx[k+1]] - an_x[idx[k]]) * (x_dc_max_idx - idx[k]);
            x_ac = an_x[idx[k]] + x_ac / (idx[k+1] - idx[k]);
            x_ac = an_x[y_dc_max_idx] - x_ac;
            int32_t nume = (y_ac * x_dc_max) >> 7;
            int32_t denom = (x_ac * y_dc_max) >> 7;
            if (denom > 0 && ratio_count < 5 && nume != 0) {
                ratios[ratio_count++] = (nume * 100) / denom;
            }
        }
    }
    // median of ratios
    // sort ascending first ratio_count entries
    for (int a=1;a<ratio_count;a++){int32_t t=ratios[a];int b=a;while(b>0 && t<ratios[b-1]){ratios[b]=ratios[b-1];b--;}ratios[b]=t;}
    int mid = ratio_count/2; int32_t ratio_avg = ratio_count>1 ? (ratios[mid-1]+ratios[mid])/2 : (ratio_count?ratios[mid]:0);
    // LUT approximation from Maxim (subset sufficient)
    static const uint8_t spo2_table[184] = {
        95,95,95,96,96,96,97,97,97,97,97,98,98,98,98,98,99,99,99,99,
        99,99,99,99,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,
        100,100,100,100,99,99,99,99,99,99,99,99,98,98,98,98,98,98,97,97,
        97,97,96,96,96,96,95,95,95,94,94,94,93,93,93,92,92,92,91,91,
        90,90,89,89,89,88,88,87,87,86,86,85,85,84,84,83,82,82,81,81,
        80,80,79,78,78,77,76,76,75,74,74,73,72,72,71,70,69,69,68,67,
        66,66,65,64,63,62,62,61,60,59,58,57,56,56,55,54,53,52,51,50,
        49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,31,30,29,
        28,27,26,25,23,22,21,20,19,17,16,15,14,12,11,10,9,7,6,5,
        3,2,1
    };
    float spo2 = 0.0f;
    if (ratio_avg > 2 && ratio_avg < 184) spo2 = (float)spo2_table[ratio_avg];
    if (hr_bpm) *hr_bpm = hr;
    if (spo2_pct) *spo2_pct = spo2;
}

static esp_err_t read_fifo_and_update_buffers(uint32_t *last_red, uint32_t *last_ir) {
    // Determine how many samples to read
    uint8_t rd=0, wr=0; esp_err_t err;
    if ((err = max_read(REG_FIFO_RD_PTR, &rd, 1)) != ESP_OK) return err;
    if ((err = max_read(REG_FIFO_WR_PTR, &wr, 1)) != ESP_OK) return err;
    int samples = (int)wr - (int)rd; if (samples < 0) samples += 32; // 32 deep FIFO
    if (samples == 0) return ESP_OK;

    for (int n = 0; n < samples; ++n) {
        uint8_t buf[6];
        if ((err = max_read(REG_FIFO_DATA, buf, sizeof(buf))) != ESP_OK) return err;
        uint32_t red = read_18b(&buf[0]);
        uint32_t ir  = read_18b(&buf[3]);
        if (last_red) *last_red = red;
        if (last_ir) *last_ir = ir;

        // Update IR DC average (IIR low-pass). alpha = 1/8
        if (s_ir_dc_avg == 0) {
            s_ir_dc_avg = ir; // initialize on first sample
        } else {
            s_ir_dc_avg -= (s_ir_dc_avg >> 3);
            s_ir_dc_avg += (ir >> 3);
        }
        bool prev_contact = s_contact;
        if (!s_contact && s_ir_dc_avg > IR_CONTACT_ENTER_THRESH) s_contact = true;
        if (s_contact && s_ir_dc_avg < IR_CONTACT_EXIT_THRESH) s_contact = false;
        if (prev_contact != s_contact) {
            ESP_LOGI(TAG, "Contact %s (IR avg=%u)", s_contact ? "ON" : "OFF", (unsigned)s_ir_dc_avg);
            // Reset buffers when contact changes to avoid mixing states
            s_buf_count = 0;
            s_buf_index = 0;
            s_last_hr = 0.0f;
            s_last_spo2 = 0.0f;
        }

        // decimation and buffering only when contact is present
        s_decim = (s_decim + 1) % HB_DECIM_FACTOR;
        if (s_decim == 0 && s_contact) {
            s_ir_buf[s_buf_index] = ir;
            s_red_buf[s_buf_index] = red;
            s_buf_index = (s_buf_index + 1) % HB_BUF_LEN;
            if (s_buf_count < HB_BUF_LEN) s_buf_count++;
        }
    }
    return ESP_OK;
}

esp_err_t heartbeat_read(heartbeat_sample_t *out) {
    if (!s_max) {
        ESP_LOGE(TAG, "heartbeat_read called before heartbeat_init");
        return ESP_ERR_INVALID_STATE;
    }
    if (!out) return ESP_ERR_INVALID_ARG;

    uint32_t last_red = 0, last_ir = 0;
    esp_err_t err = read_fifo_and_update_buffers(&last_red, &last_ir);
    if (err != ESP_OK) return err;

    bool hr_valid = false, spo2_valid = false;

    // Update metrics if buffer is full and contact is present
    if (s_contact && s_buf_count >= HB_BUF_LEN) {
        // Build linear arrays in order (oldest to newest)
        uint32_t ir[HB_BUF_LEN];
        uint32_t red[HB_BUF_LEN];
        int idx = s_buf_index; // points to next write -> oldest is idx
        for (int i = 0; i < HB_BUF_LEN; ++i) {
            int j = (idx + i) % HB_BUF_LEN;
            ir[i] = s_ir_buf[j];
            red[i] = s_red_buf[j];
        }
        float hr = 0.0f, sp = 0.0f;
        compute_hr_spo2(ir, red, HB_BUF_LEN, &hr, &sp);
        if (hr > 30.0f && hr < 220.0f) { s_last_hr = hr; hr_valid = true; }
        if (sp > 70.0f && sp <= 100.0f) { s_last_spo2 = sp; spo2_valid = true; }
    }

    // Periodically refresh temperature (every ~2s)
    uint32_t now = millis();
    if (now - s_temp_tick > 2000) {
        (void)max_read_temperature();
        s_temp_tick = now;
    }

    out->red = last_red;
    out->ir = last_ir;
    out->hr_bpm = s_last_hr;
    out->spo2_pct = s_last_spo2;
    out->temp_c = s_last_temp;
    out->samples_in_window = (uint16_t)s_buf_count;
    out->seconds_in_window = (float)s_buf_count / (float)HB_ALGO_FS;
    out->contact = s_contact;
    out->hr_valid = hr_valid;
    out->spo2_valid = spo2_valid;

    ESP_LOGD(TAG, "MAX30102 sample: RED=%u IR=%u HR=%.1f(%d) SpO2=%.1f(%d) T=%.2f contact=%d win=%u(%.1fs)",
             (unsigned)last_red, (unsigned)last_ir, s_last_hr, hr_valid, s_last_spo2, spo2_valid, s_last_temp,
             (int)s_contact, (unsigned)s_buf_count, (double)out->seconds_in_window);
    return ESP_OK;
}

void heartbeat_deinit(void) {
    if (s_max) {
        ESP_LOGI(TAG, "Removing MAX30102 device from bus");
        i2c_master_bus_rm_device(s_max);
        s_max = NULL;
    }
}