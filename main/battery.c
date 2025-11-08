#include "battery.h"

#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_rom_sys.h"

// Small portable microsecond delay helper
static inline void delay_us(uint32_t us)
{
    // esp-idf v5+: esp_rom_delay_us is available via esp_rom_sys.h
    esp_rom_delay_us(us);
}

static const char *TAG = "BAT";

// Configuration via Kconfig
#ifndef CONFIG_HABIZAP_BATTERY_ENABLE
#define CONFIG_HABIZAP_BATTERY_ENABLE 1
#endif

#ifndef CONFIG_HABIZAP_BATTERY_ADC_UNIT
#define CONFIG_HABIZAP_BATTERY_ADC_UNIT 1
#endif

#ifndef CONFIG_HABIZAP_BATTERY_ADC_CHANNEL
#define CONFIG_HABIZAP_BATTERY_ADC_CHANNEL 0
#endif

#ifndef CONFIG_HABIZAP_BATTERY_ADC_ATTEN_DB
#define CONFIG_HABIZAP_BATTERY_ADC_ATTEN_DB 11
#endif

#ifndef CONFIG_HABIZAP_BATTERY_SAMPLE_COUNT
#define CONFIG_HABIZAP_BATTERY_SAMPLE_COUNT 16
#endif

#ifndef CONFIG_HABIZAP_BATTERY_PERIOD_MS
#define CONFIG_HABIZAP_BATTERY_PERIOD_MS 5000
#endif

#ifndef CONFIG_HABIZAP_BATTERY_DIVIDER_R1_OHMS
#define CONFIG_HABIZAP_BATTERY_DIVIDER_R1_OHMS 100000
#endif
#ifndef CONFIG_HABIZAP_BATTERY_DIVIDER_R2_OHMS
#define CONFIG_HABIZAP_BATTERY_DIVIDER_R2_OHMS 100000
#endif

#ifndef CONFIG_HABIZAP_BATTERY_EMPTY_MV
#define CONFIG_HABIZAP_BATTERY_EMPTY_MV 3300
#endif
#ifndef CONFIG_HABIZAP_BATTERY_FULL_MV
#define CONFIG_HABIZAP_BATTERY_FULL_MV 4200
#endif

// Context
static TaskHandle_t s_task = NULL;
static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_cali = NULL;
static int s_last_mv = 0;
static int s_last_pct = 0;
static bool s_inited = false;

static inline adc_unit_t cfg_adc_unit(void) {
    return (CONFIG_HABIZAP_BATTERY_ADC_UNIT == 1) ? ADC_UNIT_1 : ADC_UNIT_2;
}

static inline adc_channel_t cfg_adc_channel(void) {
    // We will interpret channel number as ADC1_CHANNEL_x or ADC2_CHANNEL_x depending on unit
    int ch = CONFIG_HABIZAP_BATTERY_ADC_CHANNEL;
    if (CONFIG_HABIZAP_BATTERY_ADC_UNIT == 1) {
        switch (ch) {
            case 0: return ADC_CHANNEL_0; case 1: return ADC_CHANNEL_1; case 2: return ADC_CHANNEL_2; case 3: return ADC_CHANNEL_3;
            case 4: return ADC_CHANNEL_4; case 5: return ADC_CHANNEL_5; case 6: return ADC_CHANNEL_6; case 7: return ADC_CHANNEL_7;
#if SOC_ADC_CHANNEL_NUM(0) > 8
            case 8: return ADC_CHANNEL_8; case 9: return ADC_CHANNEL_9;
#endif
            default: return ADC_CHANNEL_0;
        }
    } else {
        switch (ch) {
            case 0: return ADC_CHANNEL_0; case 1: return ADC_CHANNEL_1; case 2: return ADC_CHANNEL_2; case 3: return ADC_CHANNEL_3;
            case 4: return ADC_CHANNEL_4; case 5: return ADC_CHANNEL_5; case 6: return ADC_CHANNEL_6; case 7: return ADC_CHANNEL_7;
            default: return ADC_CHANNEL_0;
        }
    }
}

static inline adc_atten_t cfg_adc_atten(void) {
    switch (CONFIG_HABIZAP_BATTERY_ADC_ATTEN_DB) {
        case 0: return ADC_ATTEN_DB_0;
        case 2: return ADC_ATTEN_DB_2_5;
        case 6: return ADC_ATTEN_DB_6;
        case 11: default: return ADC_ATTEN_DB_11;
    }
}

static bool init_calibration(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten) {
    (void) channel;
    adc_cali_scheme_ver_t scheme = ADC_CALI_SCHEME_VER_LINE_FITTING;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    scheme = ADC_CALI_SCHEME_VER_CURVE_FITTING;
#endif

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t curve_cfg = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&curve_cfg, &s_cali) == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration: curve fitting");
        return true;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t line_cfg = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&line_cfg, &s_cali) == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration: line fitting");
        return true;
    }
#endif
    ESP_LOGW(TAG, "ADC calibration not available; using raw to voltage approx");
    s_cali = NULL;
    return false;
}

static void deinit_calibration(void) {
    if (s_cali) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(s_cali);
#else
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_delete_scheme_line_fitting(s_cali);
#endif
#endif
        s_cali = NULL;
    }
}

static int raw_to_mv(adc_channel_t channel, int raw) {
    int mv = 0;
    if (s_cali) {
        if (adc_cali_raw_to_voltage(s_cali, raw, &mv) != ESP_OK) {
            mv = 0;
        }
    } else {
        // Fallback heuristic: map raw (0..4095) to approximate 0..Vref attenuated range.
        // This is very crude; encourage enabling calibration. Assume 1100mV Vref for 0dB; scale atten.
        int vref = 1100; // mV
        (void) channel;
        switch (cfg_adc_atten()) {
            case ADC_ATTEN_DB_0: mv = (raw * vref) / 4095; break;
            case ADC_ATTEN_DB_2_5: mv = (raw * 1500) / 4095; break;
            case ADC_ATTEN_DB_6: mv = (raw * 2200) / 4095; break;
            case ADC_ATTEN_DB_11: default: mv = (raw * 3100) / 4095; break;
        }
    }
    return mv;
}

static int board_mv_to_battery_mv(int mv_at_adc) {
    // Voltage divider: battery -> R1 -> sense node -> R2 -> GND. ADC reads sense node.
    // Battery voltage = mv_at_adc * (R1 + R2) / R2
    const double R1 = (double) CONFIG_HABIZAP_BATTERY_DIVIDER_R1_OHMS;
    const double R2 = (double) CONFIG_HABIZAP_BATTERY_DIVIDER_R2_OHMS;
    double scale = (R1 + R2) / R2;
    int batt_mv = (int) lround(mv_at_adc * scale);
    return batt_mv;
}

static int mv_to_percent(int batt_mv) {
    int empty_mv = CONFIG_HABIZAP_BATTERY_EMPTY_MV;
    int full_mv = CONFIG_HABIZAP_BATTERY_FULL_MV;
    if (batt_mv <= empty_mv) return 0;
    if (batt_mv >= full_mv) return 100;
    // Simple linear mapping. For better accuracy, replace with LUT and piecewise curve.
    int pct = (int) ((batt_mv - empty_mv) * 100 / (full_mv - empty_mv));
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    return pct;
}

static void battery_task(void *arg) {
    (void) arg;
    const int samples = CONFIG_HABIZAP_BATTERY_SAMPLE_COUNT;
    const TickType_t delay_ticks = pdMS_TO_TICKS(CONFIG_HABIZAP_BATTERY_PERIOD_MS);
    adc_channel_t channel = cfg_adc_channel();

    while (1) {
        int64_t acc = 0;
        int good = 0;
        for (int i = 0; i < samples; ++i) {
            int raw = 0;
            if (adc_oneshot_read(s_adc, channel, &raw) == ESP_OK) {
                acc += raw;
                good++;
            }
            delay_us(500); // small gap between samples
        }
        if (good > 0) {
            int avg_raw = (int)(acc / good);
            int mv_at_adc = raw_to_mv(channel, avg_raw);
            int batt_mv = board_mv_to_battery_mv(mv_at_adc);
            int pct = mv_to_percent(batt_mv);
            s_last_mv = batt_mv;
            s_last_pct = pct;
            ESP_LOGI(TAG, "Battery: %d mV (~%d%%)", batt_mv, pct);
        } else {
            ESP_LOGW(TAG, "ADC read failed for all samples");
        }
        vTaskDelay(delay_ticks);
    }
}

esp_err_t battery_init(void) {
#if !CONFIG_HABIZAP_BATTERY_ENABLE
    ESP_LOGI(TAG, "Battery subsystem disabled by Kconfig");
    return ESP_OK;
#endif
    if (s_inited) return ESP_OK;

    adc_unit_t unit = cfg_adc_unit();
    adc_channel_t channel = cfg_adc_channel();
    adc_atten_t atten = cfg_adc_atten();

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = unit,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc, channel, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    (void) init_calibration(unit, channel, atten);

    s_inited = true;
    ESP_LOGI(TAG, "Battery subsystem init OK (unit=%d channel=%d atten=%d dB)", (int)unit, (int)CONFIG_HABIZAP_BATTERY_ADC_CHANNEL, (int)CONFIG_HABIZAP_BATTERY_ADC_ATTEN_DB);
    return ESP_OK;
}

esp_err_t battery_start(void) {
#if !CONFIG_HABIZAP_BATTERY_ENABLE
    return ESP_OK;
#endif
    if (!s_inited) {
        esp_err_t e = battery_init();
        if (e != ESP_OK) return e;
    }
    if (s_task) return ESP_OK;
    BaseType_t ok = xTaskCreatePinnedToCore(battery_task, "battery", 2048, NULL, tskIDLE_PRIORITY + 1, &s_task, tskNO_AFFINITY);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create battery task");
        s_task = NULL;
        return ESP_FAIL;
    }
    return ESP_OK;
}

void battery_stop(void) {
    if (s_task) {
        TaskHandle_t t = s_task; s_task = NULL;
        vTaskDelete(t);
    }
    if (s_adc) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
    }
    deinit_calibration();
    s_inited = false;
}

int battery_get_percent(void) {
    return s_last_pct;
}

int battery_get_voltage_mv(void) {
    return s_last_mv;
}
