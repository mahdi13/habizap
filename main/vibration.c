#include "vibration.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "driver/gpio.h"

static const char *TAG = "VIB";

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

// Internal vibration context definition (opaque to users)
struct vibration_ctx_t {
    QueueHandle_t queue;          // queue of commands (each command is a pattern)
    TaskHandle_t task;            // worker task processing the queue
    int gpio;                     // GPIO controlling the motor/driver
    bool active_high;             // true if HIGH = ON
    int default_on_ms;            // defaults for convenience helpers
    int default_off_ms;
    size_t max_items;             // queue length
};

static inline void vib_gpio_set(vibration_ctx_t *ctx, bool on) {
    bool level = ctx->active_high ? on : !on;
    gpio_set_level(ctx->gpio, level);
}

static void vib_process_pattern(vibration_ctx_t *ctx, const vib_cmd_t *cmd) {
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
        vib_gpio_set(ctx, on);
        vTaskDelay(pdMS_TO_TICKS(d));
        on = !on;
    }
    // Ensure motor is OFF at end
    vib_gpio_set(ctx, false);
}

static void vib_task(void *arg) {
    vibration_ctx_t *ctx = (vibration_ctx_t *)arg;
    vib_cmd_t cmd;
    while (1) {
        if (xQueueReceive(ctx->queue, &cmd, portMAX_DELAY) == pdTRUE) {
            vib_process_pattern(ctx, &cmd);
            // free resources
            if (cmd.dur_ms) free(cmd.dur_ms);
        }
    }
}

static bool vibration_gpio_init(vibration_ctx_t *ctx) {
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
    ctx->gpio = gpio_num;
    ctx->active_high = CONFIG_HABIZAP_VIBRATION_ACTIVE_HIGH;
    vib_gpio_set(ctx, false);
    return true;
}

vibration_ctx_t *vibration_init(void) {
    vibration_ctx_t *ctx = (vibration_ctx_t *)calloc(1, sizeof(vibration_ctx_t));
    if (!ctx) return NULL;

    if (!vibration_gpio_init(ctx)) {
        free(ctx);
        return NULL;
    }

    size_t qlen = CONFIG_HABIZAP_VIBRATION_QUEUE_LEN;
    ctx->queue = xQueueCreate(qlen, sizeof(vib_cmd_t));
    if (!ctx->queue) {
        ESP_LOGE(TAG, "Failed to create vibration queue");
        free(ctx);
        return NULL;
    }
    ctx->max_items = qlen;
    ctx->default_on_ms = 100;
    ctx->default_off_ms = 100;

    BaseType_t ok = xTaskCreate(vib_task, "vib_task", 2048, ctx, tskIDLE_PRIORITY + 2, &ctx->task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create vibration task");
        vQueueDelete(ctx->queue);
        free(ctx);
        return NULL;
    }
    ESP_LOGI(TAG, "Vibration subsystem initialized on GPIO %d (active_%s), queue=%u", ctx->gpio,
             ctx->active_high ? "high" : "low", (unsigned) qlen);
    return ctx;
}

static bool vib_enqueue_copy(vibration_ctx_t *ctx, const int *dur_ms, size_t count) {
    if (!ctx || !ctx->queue || !dur_ms || count == 0) return false;
    vib_cmd_t cmd = {0};
    cmd.count = count;
    cmd.dur_ms = (int *) malloc(count * sizeof(int));
    if (!cmd.dur_ms) return false;
    memcpy(cmd.dur_ms, dur_ms, count * sizeof(int));
    if (xQueueSendToBack(ctx->queue, &cmd, 0) == pdTRUE) {
        return true;
    } else {
        free(cmd.dur_ms);
        return false;
    }
}

bool vibration_enqueue_pattern_ms(vibration_ctx_t *ctx, const int *durations_ms, size_t count) {
    return vib_enqueue_copy(ctx, durations_ms, count);
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

bool vibration_enqueue_pattern_str(vibration_ctx_t *ctx, const char *pattern_str) {
    int *arr = NULL;
    size_t n = 0;
    if (!parse_ints_from_str(pattern_str, &arr, &n)) return false;
    bool ok = vib_enqueue_copy(ctx, arr, n);
    free(arr);
    return ok;
}

bool vibration_pulse(vibration_ctx_t *ctx, int on_ms, int off_ms, int repeat) {
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
    bool ok = vib_enqueue_copy(ctx, arr, n);
    free(arr);
    return ok;
}
