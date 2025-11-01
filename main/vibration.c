#include "vibration.h"


#include "esp_log.h"
#include "esp_err.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "driver/gpio.h"


#ifndef CONFIG_HABIZAP_VIBRATION_GPIO
#define CONFIG_HABIZAP_VIBRATION_GPIO 2
#endif
#ifndef CONFIG_HABIZAP_VIBRATION_ACTIVE_HIGH
#define CONFIG_HABIZAP_VIBRATION_ACTIVE_HIGH 1
#endif
#ifndef CONFIG_HABIZAP_VIBRATION_QUEUE_LEN
#define CONFIG_HABIZAP_VIBRATION_QUEUE_LEN 8
#endif


// ------- VIBRATION SUBSYSTEM -------

typedef struct {
    int *dur_ms; // heap array of durations
    size_t count; // number of entries
} vib_cmd_t;

static inline void vib_gpio_set(bool on) {
    bool level = g_ctx.vibration.active_high ? on : !on;
    gpio_set_level(g_ctx.vibration.gpio, level);
}

static void vib_process_pattern(const vib_cmd_t *cmd) {
    if (!cmd || !cmd->dur_ms || cmd->count == 0) return;
    // Start with ON duration at index 0
    bool on = true;
    for (size_t i = 0; i < cmd->count; ++i) {
        int d = cmd->dur_ms[i];
        if (d <= 0) {
            // Skip non-positive durations but still toggle state progression
            on = !on;
            continue;
        }
        vib_gpio_set(on);
        vTaskDelay(pdMS_TO_TICKS(d));
        on = !on;
    }
    // Ensure motor is OFF at end
    vib_gpio_set(false);
}

static void vib_task(void *arg) {
    (void) arg;
    vib_cmd_t cmd;
    while (1) {
        if (xQueueReceive(g_ctx.vibration.queue, &cmd, portMAX_DELAY) == pdTRUE) {
            vib_process_pattern(&cmd);
            // free resources
            if (cmd.dur_ms) free(cmd.dur_ms);
        }
    }
}

static bool vibration_gpio_init(void) {
    int gpio_num = CONFIG_HABIZAP_VIBRATION_GPIO;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t e = gpio_config(&io_conf);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "Vibration GPIO config failed: %s", esp_err_to_name(e));
        return false;
    }
    // Default OFF
    g_ctx.vibration.gpio = gpio_num;
    g_ctx.vibration.active_high = CONFIG_HABIZAP_VIBRATION_ACTIVE_HIGH;
    vib_gpio_set(false);
    return true;
}

bool vibration_init(void) {
    if (g_ctx.vibration.queue) return true; // already inited
    if (!vibration_gpio_init()) return false;

    size_t qlen = CONFIG_HABIZAP_VIBRATION_QUEUE_LEN;
    g_ctx.vibration.queue = xQueueCreate(qlen, sizeof(vib_cmd_t));
    if (!g_ctx.vibration.queue) {
        ESP_LOGE(TAG, "Failed to create vibration queue");
        return false;
    }
    g_ctx.vibration.max_items = qlen;
    g_ctx.vibration.default_on_ms = 100;
    g_ctx.vibration.default_off_ms = 100;

    BaseType_t ok = xTaskCreate(vib_task, "vib_task", 2048, NULL, tskIDLE_PRIORITY + 2, &g_ctx.vibration.task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create vibration task");
        vQueueDelete(g_ctx.vibration.queue);
        g_ctx.vibration.queue = NULL;
        return false;
    }
    ESP_LOGI(TAG, "Vibration subsystem initialized on GPIO %d (active_%s), queue=%u", g_ctx.vibration.gpio,
             g_ctx.vibration.active_high ? "high" : "low", (unsigned) qlen);
    return true;
}

static bool vib_enqueue_copy(const int *dur_ms, size_t count) {
    if (!g_ctx.vibration.queue || !dur_ms || count == 0) return false;
    vib_cmd_t cmd = {0};
    cmd.count = count;
    cmd.dur_ms = (int *) malloc(count * sizeof(int));
    if (!cmd.dur_ms) return false;
    memcpy(cmd.dur_ms, dur_ms, count * sizeof(int));
    if (xQueueSendToBack(g_ctx.vibration.queue, &cmd, 0) == pdTRUE) {
        return true;
    } else {
        free(cmd.dur_ms);
        return false;
    }
}

bool vibration_enqueue_pattern_ms(const int *durations_ms, size_t count) {
    return vib_enqueue_copy(durations_ms, count);
}

static bool parse_ints_from_str(const char *s, int **out_arr, size_t *out_count) {
    if (!s || !out_arr || !out_count) return false;
    // First pass: count numbers
    size_t count = 0;
    const char *p = s;
    while (*p) {
        while (*p && (isspace((unsigned char)*p) || *p == ',' || *p == ';')) p++;
        if (!*p) break;
        char *endp = NULL;
        long v = strtol(p, &endp, 10);
        if (p == endp) {
            // not a number, skip token
            while (*p && !isspace((unsigned char)*p) && *p != ',' && *p != ';') p++;
            continue;
        }
        (void) v;
        count++;
        p = endp;
    }
    if (count == 0) return false;
    int *arr = (int *) malloc(count * sizeof(int));
    if (!arr) return false;
    // Second pass: fill
    p = s;
    size_t idx = 0;
    while (*p && idx < count) {
        while (*p && (isspace((unsigned char)*p) || *p == ',' || *p == ';')) p++;
        if (!*p) break;
        char *endp = NULL;
        long v = strtol(p, &endp, 10);
        if (p == endp) {
            while (*p && !isspace((unsigned char)*p) && *p != ',' && *p != ';') p++;
            continue;
        }
        if (v < 0) v = 0; // clamp negatives to 0
        arr[idx++] = (int) v;
        p = endp;
    }
    if (idx == 0) {
        free(arr);
        return false;
    }
    *out_arr = arr;
    *out_count = idx;
    return true;
}

bool vibration_enqueue_pattern_str(const char *pattern_str) {
    int *arr = NULL;
    size_t n = 0;
    if (!parse_ints_from_str(pattern_str, &arr, &n)) return false;
    bool ok = vib_enqueue_copy(arr, n);
    free(arr);
    return ok;
}

bool vibration_pulse(int on_ms, int off_ms, int repeat) {
    if (on_ms < 0) on_ms = 0;
    if (off_ms < 0) off_ms = 0;
    if (repeat < 1) repeat = 1;
    size_t n = (size_t) (repeat * 2);
    int *arr = (int *) malloc(n * sizeof(int));
    if (!arr) return false;
    for (int i = 0; i < repeat; ++i) {
        arr[i * 2] = on_ms;
        arr[i * 2 + 1] = off_ms;
    }
    bool ok = vib_enqueue_copy(arr, n);
    free(arr);
    return ok;
}
