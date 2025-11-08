#include "inference.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

// Edge Impulse SDK (use component include paths)
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy_types.h"
#include "model-parameters/model_metadata.h"
#include "model-parameters/model_variables.h"

#define TAG "inference"

#ifndef CONFIG_HABIZAP_ML_QUEUE_LEN
#define CONFIG_HABIZAP_ML_QUEUE_LEN 4
#endif
#ifndef CONFIG_HABIZAP_ML_USE_CONTINUOUS
#define CONFIG_HABIZAP_ML_USE_CONTINUOUS 1
#endif
#ifndef CONFIG_HABIZAP_ML_POSITIVE_THRESHOLD
#define CONFIG_HABIZAP_ML_POSITIVE_THRESHOLD 60
#endif

// Derive float threshold in [0..1] from config which is percent (1..100) or legacy float [0..1]
#if CONFIG_HABIZAP_ML_POSITIVE_THRESHOLD > 1
#define HABIZAP_POS_THRESH_FLOAT ( (CONFIG_HABIZAP_ML_POSITIVE_THRESHOLD) / 100.0f )
#else
#define HABIZAP_POS_THRESH_FLOAT ( (float)(CONFIG_HABIZAP_ML_POSITIVE_THRESHOLD) )
#endif
#ifndef CONFIG_HABIZAP_ML_TASK_STACK
#define CONFIG_HABIZAP_ML_TASK_STACK (6*1024)
#endif
#ifndef CONFIG_HABIZAP_ML_TASK_PRIORITY
#define CONFIG_HABIZAP_ML_TASK_PRIORITY 5
#endif
#ifndef CONFIG_HABIZAP_ML_POSITIVE_LABEL
#define CONFIG_HABIZAP_ML_POSITIVE_LABEL "positive"
#endif

// Internal message types
typedef enum {
    MSG_FRAME,
    MSG_SAMPLES,
    MSG_EXIT
} msg_type_t;

typedef struct {
    msg_type_t type;
    size_t len;           // number of floats in payload
    float *payload;       // heap-allocated copy for queue safety
} infer_msg_t;

struct inference_instance_t {
    // Configuration
    inference_config_t cfg;

    // RTOS primitives
    QueueHandle_t q;
    TaskHandle_t task;
    SemaphoreHandle_t result_mtx;

    // Latest result
    bool have_result;
    inference_result_t last;

    // For continuous mode: ring buffer holding last frame worth of samples
    float *ring;
    size_t ring_size;         // EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
    size_t ring_write_idx;    // next write position
    size_t ring_count;        // how many valid entries currently in ring
};

static bool is_label_positive(const char *label, const char *positive_label)
{
    if (!label || !positive_label) return false;
    return strcmp(label, positive_label) == 0;
}

static void fill_result_flags(inference_result_t *dst)
{
    // Determine positive by matching label string and threshold
    const char *label = NULL;
    if (dst->top_index >= 0 && dst->top_index < EI_CLASSIFIER_LABEL_COUNT) {
        label = ei_classifier_inferencing_categories[dst->top_index];
    }
    bool pos = label && is_label_positive(label, CONFIG_HABIZAP_ML_POSITIVE_LABEL) && dst->top_score >= HABIZAP_POS_THRESH_FLOAT;
    dst->is_positive = pos;
}

static void run_inference_on_buffer(struct inference_instance_t *h, const float *buffer)
{
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;

    // capture buffer pointer in lambda matching SDK's std::function signature
    signal.get_data = [buffer](size_t offset, size_t length, float *out_ptr) -> int {
        memcpy(out_ptr, buffer + offset, length * sizeof(float));
        return 0; // 0 == OK per EI convention
    };

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, /*debug*/ false);
    if (err != EI_IMPULSE_OK) {
        ESP_LOGE(TAG, "run_classifier failed: %d", (int)err);
        return;
    }

    // Find top score
    int top_idx = -1;
    float top_score = -1.0f;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; ++i) {
        if (result.classification[i].value > top_score) {
            top_score = result.classification[i].value;
            top_idx = (int)i;
        }
    }

    inference_result_t out = {
        .top_index = top_idx,
        .top_score = top_score,
        .is_positive = false,
        .timestamp_ms = (uint64_t)(esp_timer_get_time() / 1000ULL)
    };
    fill_result_flags(&out);

    xSemaphoreTake(h->result_mtx, portMAX_DELAY);
    h->last = out;
    h->have_result = true;
    xSemaphoreGive(h->result_mtx);
}

static void run_inference_continuous(struct inference_instance_t *h)
{
    // Build a signal that reads from the current sliding window (ring buffer)
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;

    // capture pointer to h and implement ring-reading logic inside lambda
    signal.get_data = [h](size_t offset, size_t length, float *out_ptr) -> int {
        // Provide data starting at 'offset' from the logical ring buffer end (most-recent window)
        if (h->ring_count < h->ring_size) {
            // Not enough data yet; pad with zeros at the beginning
            size_t missing = h->ring_size - h->ring_count;
            size_t end = offset + length;
            for (size_t i = offset; i < end; ++i) {
                if (i < missing) {
                    out_ptr[i - offset] = 0.0f;
                } else {
                    size_t j = (h->ring_write_idx + i - missing) % h->ring_size;
                    out_ptr[i - offset] = h->ring[j];
                }
            }
            return 0;
        }
        // Enough data: window is exactly the last ring_size samples ending at write_idx
        size_t end = offset + length;
        for (size_t i = offset; i < end; ++i) {
            size_t j = (h->ring_write_idx + i) % h->ring_size;
            out_ptr[i - offset] = h->ring[j];
        }
        return 0;
    };

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier_continuous(&signal, &result, /*debug*/ false);
    if (err != EI_IMPULSE_OK) {
        ESP_LOGE(TAG, "run_classifier_continuous failed: %d", (int)err);
        return;
    }

    int top_idx = -1;
    float top_score = -1.0f;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; ++i) {
        if (result.classification[i].value > top_score) {
            top_score = result.classification[i].value;
            top_idx = (int)i;
        }
    }

    inference_result_t out = {
        .top_index = top_idx,
        .top_score = top_score,
        .is_positive = false,
        .timestamp_ms = (uint64_t)(esp_timer_get_time() / 1000ULL)
    };
    fill_result_flags(&out);

    xSemaphoreTake(h->result_mtx, portMAX_DELAY);
    h->last = out;
    h->have_result = true;
    xSemaphoreGive(h->result_mtx);
}

static void inference_task(void *arg)
{
    struct inference_instance_t *h = (struct inference_instance_t*)arg;

    ESP_LOGI(TAG, "inference task started, continuous=%d", (int)h->cfg.use_continuous);
    if (h->cfg.use_continuous) {
        ESP_LOGI(TAG, "initializing continuous classifier...");
        run_classifier_init();
        ESP_LOGI(TAG, "initialized continuous classifier");
    }

    infer_msg_t msg;
    while (1) {
        if (xQueueReceive(h->q, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (msg.type == MSG_EXIT) {
            if (msg.payload) free(msg.payload);
            break;
        }
        if (msg.type == MSG_FRAME) {
            if (msg.len != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                ESP_LOGW(TAG, "Frame size %u mismatch expected %u", (unsigned)msg.len, (unsigned)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
            } else {
                if (h->cfg.use_continuous) {
                    // Update ring with the whole frame and run continuous inference once
                    for (size_t i = 0; i < msg.len; ++i) {
                        h->ring[h->ring_write_idx] = msg.payload[i];
                        h->ring_write_idx = (h->ring_write_idx + 1) % h->ring_size;
                        if (h->ring_count < h->ring_size) h->ring_count++;
                    }
                    run_inference_continuous(h);
                } else {
                    run_inference_on_buffer(h, msg.payload);
                }
            }
        } else if (msg.type == MSG_SAMPLES) {
            if (h->cfg.use_continuous) {
                // Append to ring
                for (size_t i = 0; i < msg.len; ++i) {
                    h->ring[h->ring_write_idx] = msg.payload[i];
                    h->ring_write_idx = (h->ring_write_idx + 1) % h->ring_size;
                    if (h->ring_count < h->ring_size) h->ring_count++;
                }
                // Trigger continuous inference each time we get more data
                run_inference_continuous(h);
            } else {
                // Accumulate until we have a full frame, then run
                static float *accum = NULL;
                static size_t filled = 0;
                if (!accum) {
                    accum = (float*)heap_caps_malloc(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * sizeof(float), MALLOC_CAP_DEFAULT);
                    if (!accum) {
                        ESP_LOGE(TAG, "alloc failed for accum buffer");
                        free(msg.payload);
                        continue;
                    }
                    filled = 0;
                }
                size_t to_copy = msg.len;
                const float *src = msg.payload;
                while (to_copy > 0) {
                    size_t space = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - filled;
                    size_t chunk = (to_copy < space) ? to_copy : space;
                    memcpy(accum + filled, src, chunk * sizeof(float));
                    filled += chunk;
                    src += chunk;
                    to_copy -= chunk;
                    if (filled == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                        run_inference_on_buffer(h, accum);
                        filled = 0; // start next frame
                    }
                }
            }
        }
        if (msg.payload) free(msg.payload);
    }

    vTaskDelete(NULL);
}

static bool enqueue_msg(struct inference_instance_t *h, msg_type_t type, const float *data, size_t len)
{
    infer_msg_t m = { .type = type, .len = len, .payload = NULL };
    if (data && len > 0) {
        size_t bytes = len * sizeof(float);
        float *copy = (float*)heap_caps_malloc(bytes, MALLOC_CAP_DEFAULT);
        if (!copy) {
            ESP_LOGE(TAG, "alloc failed for message payload (%u floats)", (unsigned)len);
            return false;
        }
        memcpy(copy, data, bytes);
        m.payload = copy;
    }
    if (xQueueSend(h->q, &m, 0) != pdTRUE) {
        if (m.payload) free(m.payload);
        return false;
    }
    return true;
}

extern "C" bool inference_create(const inference_config_t *cfg, inference_handle_t **out)
{
    if (!out) return false;
    *out = NULL;

    struct inference_instance_t *h = (struct inference_instance_t*)calloc(1, sizeof(*h));
    if (!h) return false;

    // Defaults from Kconfig
    h->cfg.queue_len = CONFIG_HABIZAP_ML_QUEUE_LEN;
    h->cfg.use_continuous = CONFIG_HABIZAP_ML_USE_CONTINUOUS;
    h->cfg.positive_threshold = HABIZAP_POS_THRESH_FLOAT;
    h->cfg.positive_label = CONFIG_HABIZAP_ML_POSITIVE_LABEL;
    h->cfg.task_stack = CONFIG_HABIZAP_ML_TASK_STACK;
    h->cfg.task_priority = CONFIG_HABIZAP_ML_TASK_PRIORITY;

    // Override with user cfg if provided
    if (cfg) {
        if (cfg->queue_len) h->cfg.queue_len = cfg->queue_len;
        h->cfg.use_continuous = cfg->use_continuous;
        if (cfg->positive_threshold > 0) h->cfg.positive_threshold = cfg->positive_threshold;
        if (cfg->positive_label) h->cfg.positive_label = cfg->positive_label;
        if (cfg->task_stack) h->cfg.task_stack = cfg->task_stack;
        if (cfg->task_priority) h->cfg.task_priority = cfg->task_priority;
    }

    // If the impulse's DSP blocks are not slice-based (MFCC/MFE/Spectrogram),
    // continuous inference (run_classifier_continuous) is not supported and will
    // print "Unknown extract function" and fail. Detect that case here and
    // silently fall back to non-continuous mode while warning the user.
    bool continuous_supported = true;
    for (size_t i = 0; i < ei_dsp_blocks_size; ++i) {
        // Only these extractors support per-slice processing required by
        // run_classifier_continuous()
        if (ei_dsp_blocks[i].extract_fn != extract_mfcc_features &&
            ei_dsp_blocks[i].extract_fn != extract_mfe_features &&
            ei_dsp_blocks[i].extract_fn != extract_spectrogram_features) {
            continuous_supported = false;
            break;
        }
    }

    if (!continuous_supported && h->cfg.use_continuous) {
        ESP_LOGW(TAG, "Model DSP blocks are not slice-based (raw features); disabling continuous inference to avoid DSP errors");
        h->cfg.use_continuous = false;
    }

    // Create queue and synchronization primitives
    h->q = xQueueCreate(h->cfg.queue_len, sizeof(infer_msg_t));
    if (!h->q) { free(h); return false; }
    h->result_mtx = xSemaphoreCreateMutex();
    if (!h->result_mtx) { vQueueDelete(h->q); free(h); return false; }

    // Allocate ring buffer for continuous mode
    h->ring_size = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    h->ring = (float*)heap_caps_calloc(h->ring_size, sizeof(float), MALLOC_CAP_DEFAULT);
    if (!h->ring) {
        vSemaphoreDelete(h->result_mtx);
        vQueueDelete(h->q);
        free(h);
        return false;
    }

    // Start task
    BaseType_t ok = xTaskCreatePinnedToCore(
        inference_task, "infer", h->cfg.task_stack, h, h->cfg.task_priority, &h->task,
        tskNO_AFFINITY
    );
    if (ok != pdPASS) {
        free(h->ring);
        vSemaphoreDelete(h->result_mtx);
        vQueueDelete(h->q);
        free(h);
        return false;
    }

    *out = h;
    ESP_LOGI(TAG, "Inference engine created (queue_len=%u, continuous=%d, frame=%u)",
             (unsigned)h->cfg.queue_len, (int)h->cfg.use_continuous, (unsigned)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    return true;
}

extern "C" void inference_destroy(inference_handle_t *h)
{
    if (!h) return;
    // Ask task to exit
    (void) enqueue_msg(h, MSG_EXIT, NULL, 0);
    // Best-effort delay to let task exit
    vTaskDelay(pdMS_TO_TICKS(10));

    if (h->ring) free(h->ring);
    if (h->result_mtx) vSemaphoreDelete(h->result_mtx);
    if (h->q) vQueueDelete(h->q);
    free(h);
}

extern "C" size_t inference_feed_samples(inference_handle_t *h, const float *samples, size_t count)
{
    if (!h || !samples || count == 0) return 0;
    if (!enqueue_msg(h, MSG_SAMPLES, samples, count)) return 0;
    return count;
}

extern "C" bool inference_get_latest(inference_handle_t *h, inference_result_t *out)
{
    if (!h || !out) return false;
    bool ok = false;
    xSemaphoreTake(h->result_mtx, portMAX_DELAY);
    if (h->have_result) {
        *out = h->last;
        ok = true;
    }
    xSemaphoreGive(h->result_mtx);
    return ok;
}