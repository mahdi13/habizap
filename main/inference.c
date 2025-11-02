#include "inference.h"
#include "gconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>

// TensorFlow Lite Micro
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

// Edge Impulse exported model as a C array
// Provide these from a separate C file generated from your .tflite (see README)
extern const unsigned char g_model_data[];
extern const unsigned int g_model_data_len;

#ifndef EI_CLASSIFIER_LABEL_COUNT
// If you have generated labels, you can define this. Used for basic checks/logs only.
#define EI_CLASSIFIER_LABEL_COUNT 2
#endif

struct inference_ctx_t {
    // TFLM
    tflite::MicroErrorReporter micro_error_reporter;
    tflite::AllOpsResolver resolver;
    const tflite::Model* model;
    tflite::MicroInterpreter* interpreter;
    TfLiteTensor* input;
    TfLiteTensor* output;
    uint8_t* tensor_arena;
    size_t tensor_arena_size;

    // Filtering
    int window_size;
    int positives_to_trigger;
    float score_threshold;
    int debounce_ms;
    int pos_index;
    int* window;     // ring buffer of 0/1
    int win_head;    // next write index
    int win_count;   // number of valid entries (<= window_size)
    int win_sum;     // running sum of positives in window
    int64_t last_fire_us;

    // Feedback
    vibration_ctx_t* vib;
    int buzz_on_ms;
    int buzz_off_ms;
    int buzz_repeat;
};

static const char* TAG = CONFIG_HABIZAP_INFER_LOG_TAG;

void inference_default_config(inference_config_t *cfg) {
    if (!cfg) return;
    cfg->window_size = CONFIG_HABIZAP_INFER_WINDOW_SIZE;
    cfg->positives_to_trigger = CONFIG_HABIZAP_INFER_POSITIVES_TO_TRIGGER;
    cfg->score_threshold = CONFIG_HABIZAP_INFER_SCORE_THRESHOLD;
    cfg->debounce_ms = CONFIG_HABIZAP_INFER_DEBOUNCE_MS;
    cfg->positive_class_index = CONFIG_HABIZAP_INFER_POSITIVE_CLASS_INDEX;
    cfg->vib = NULL;
    cfg->buzz_on_ms = CONFIG_HABIZAP_INFER_BUZZ_ON_MS;
    cfg->buzz_off_ms = CONFIG_HABIZAP_INFER_BUZZ_OFF_MS;
    cfg->buzz_repeat = CONFIG_HABIZAP_INFER_BUZZ_REPEAT;
}

static bool allocate_window(inference_ctx_t* ctx) {
    ctx->window = (int*)calloc((size_t)ctx->window_size, sizeof(int));
    return ctx->window != NULL;
}

static void free_window(inference_ctx_t* ctx) {
    if (ctx->window) free(ctx->window);
    ctx->window = NULL;
}

static void window_push(inference_ctx_t* ctx, int val01) {
    if (ctx->win_count < ctx->window_size) {
        ctx->window[ctx->win_head] = val01;
        ctx->win_head = (ctx->win_head + 1) % ctx->window_size;
        ctx->win_count++;
        ctx->win_sum += val01;
        return;
    }
    int old = ctx->window[ctx->win_head];
    ctx->window[ctx->win_head] = val01;
    ctx->win_head = (ctx->win_head + 1) % ctx->window_size;
    ctx->win_sum += (val01 - old);
}

static bool window_ready_and_positive(const inference_ctx_t* ctx) {
    return (ctx->win_count >= ctx->positives_to_trigger) && (ctx->win_sum >= ctx->positives_to_trigger);
}

static void maybe_buzz(inference_ctx_t* ctx) {
    if (!ctx->vib) return;
    (void) vibration_pulse(ctx->vib, ctx->buzz_on_ms, ctx->buzz_off_ms, ctx->buzz_repeat);
}

inference_ctx_t *inference_init(const inference_config_t *user_cfg) {
    inference_config_t cfg;
    if (user_cfg) cfg = *user_cfg; else inference_default_config(&cfg);

    inference_ctx_t* ctx = (inference_ctx_t*)calloc(1, sizeof(inference_ctx_t));
    if (!ctx) return NULL;

    ctx->window_size = (cfg.window_size > 0 ? cfg.window_size : 1);
    ctx->positives_to_trigger = (cfg.positives_to_trigger > 0 ? cfg.positives_to_trigger : 1);
    if (ctx->positives_to_trigger > ctx->window_size) ctx->positives_to_trigger = ctx->window_size;
    ctx->score_threshold = cfg.score_threshold;
    ctx->debounce_ms = (cfg.debounce_ms >= 0 ? cfg.debounce_ms : 0);
    ctx->pos_index = (cfg.positive_class_index >= 0 ? cfg.positive_class_index : 0);
    ctx->vib = cfg.vib;
    ctx->buzz_on_ms = cfg.buzz_on_ms;
    ctx->buzz_off_ms = cfg.buzz_off_ms;
    ctx->buzz_repeat = cfg.buzz_repeat;
    ctx->last_fire_us = 0;

    if (!allocate_window(ctx)) {
        free(ctx);
        return NULL;
    }

    // Map the model
    if (g_model_data_len < 16) {
        ESP_LOGE(TAG, "No model linked (g_model_data_len=%u). See README to add your Edge Impulse TFLite model.", (unsigned)g_model_data_len);
        inference_deinit(ctx);
        return NULL;
    }
    ctx->model = tflite::GetModel(g_model_data);
    if (!ctx->model) {
        ESP_LOGE(TAG, "GetModel returned NULL");
        inference_deinit(ctx);
        return NULL;
    }
    if (ctx->model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema %d not equal to supported %d", (int)ctx->model->version(), (int)TFLITE_SCHEMA_VERSION);
        inference_deinit(ctx);
        return NULL;
    }

    // Allocate tensor arena
    ctx->tensor_arena_size = CONFIG_HABIZAP_INFER_TENSOR_ARENA_SIZE;
    ctx->tensor_arena = (uint8_t*)heap_caps_malloc(ctx->tensor_arena_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!ctx->tensor_arena) {
        // fallback to default heap
        ctx->tensor_arena = (uint8_t*)malloc(ctx->tensor_arena_size);
    }
    if (!ctx->tensor_arena) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena (%u bytes)", (unsigned)ctx->tensor_arena_size);
        inference_deinit(ctx);
        return NULL;
    }

    // Create interpreter
    ctx->interpreter = new tflite::MicroInterpreter(ctx->model, ctx->resolver, ctx->tensor_arena, ctx->tensor_arena_size, &ctx->micro_error_reporter);
    if (!ctx->interpreter) {
        ESP_LOGE(TAG, "Failed to create MicroInterpreter");
        inference_deinit(ctx);
        return NULL;
    }

    // Allocate tensors
    TfLiteStatus alloc_status = ctx->interpreter->AllocateTensors();
    if (alloc_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors failed");
        inference_deinit(ctx);
        return NULL;
    }

    ctx->input = ctx->interpreter->input(0);
    ctx->output = ctx->interpreter->output(0);

    ESP_LOGI(TAG, "Inference ready: input type=%d dims=%d, output type=%d dims=%d, arena=%u bytes", (int)ctx->input->type, (int)ctx->input->dims->size, (int)ctx->output->type, (int)ctx->output->dims->size, (unsigned)ctx->tensor_arena_size);
    return ctx;
}

void inference_deinit(inference_ctx_t *ctx) {
    if (!ctx) return;
    if (ctx->interpreter) {
        delete ctx->interpreter;
        ctx->interpreter = NULL;
    }
    if (ctx->tensor_arena) {
        free(ctx->tensor_arena);
        ctx->tensor_arena = NULL;
    }
    free_window(ctx);
    free(ctx);
}

static void fill_input(TfLiteTensor* input, const float* features, size_t n) {
    if (input->type == kTfLiteFloat32) {
        float* data = input->data.f;
        for (size_t i = 0; i < n; ++i) data[i] = features[i];
    } else if (input->type == kTfLiteInt8) {
        // Assume features are normalized -1..1 or 0..1; here we use scale/zero_point
        const float scale = input->params.scale;
        const int zp = input->params.zero_point;
        int8_t* data = input->data.int8;
        for (size_t i = 0; i < n; ++i) {
            int32_t q = (int32_t) (features[i] / scale) + zp;
            if (q < -128) q = -128; if (q > 127) q = 127;
            data[i] = (int8_t) q;
        }
    } else {
        // Add other types if needed
    }
}

static int argmax_float(const float* arr, size_t n, float* max_val) {
    int idx = 0; float mv = arr[0];
    for (size_t i = 1; i < n; ++i) if (arr[i] > mv) { mv = arr[i]; idx = (int)i; }
    if (max_val) *max_val = mv;
    return idx;
}

static int argmax_int8(const int8_t* arr, size_t n, int8_t* max_val) {
    int idx = 0; int8_t mv = arr[0];
    for (size_t i = 1; i < n; ++i) if (arr[i] > mv) { mv = arr[i]; idx = (int)i; }
    if (max_val) *max_val = mv;
    return idx;
}

bool inference_feed(inference_ctx_t *ctx, const float *features, size_t feature_count, inference_result_t *out_result) {
    if (!ctx || !features) return false;

    // Basic shape check (flat vectors only)
    size_t input_elems = 1;
    for (int i = 0; i < ctx->input->dims->size; ++i) input_elems *= ctx->input->dims->data[i];
    if (feature_count != input_elems) {
        ESP_LOGW(TAG, "Feature length %u != model input elems %u", (unsigned)feature_count, (unsigned)input_elems);
        // still try to min(copy)
        size_t n = (feature_count < input_elems) ? feature_count : input_elems;
        fill_input(ctx->input, features, n);
        // zero the rest if any
        if (n < input_elems) {
            if (ctx->input->type == kTfLiteFloat32) memset(ctx->input->data.f + n, 0, (input_elems - n) * sizeof(float));
            else if (ctx->input->type == kTfLiteInt8) memset(ctx->input->data.int8 + n, 0, (input_elems - n) * sizeof(int8_t));
        }
    } else {
        fill_input(ctx->input, features, feature_count);
    }

    if (ctx->interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke failed");
        return false;
    }

    // Read output
    int winner = -1;
    float pos_score = 0.0f;
    float neg_score = 0.0f;
    bool is_positive = false;

    size_t out_elems = 1; for (int i = 0; i < ctx->output->dims->size; ++i) out_elems *= ctx->output->dims->data[i];
    if (ctx->output->type == kTfLiteFloat32) {
        winner = argmax_float(ctx->output->data.f, out_elems, NULL);
        pos_score = (ctx->pos_index < (int)out_elems) ? ctx->output->data.f[ctx->pos_index] : 0.0f;
        if (out_elems >= 2) {
            int neg_idx = (ctx->pos_index == 0 ? 1 : 0);
            neg_score = ctx->output->data.f[neg_idx];
        } else {
            neg_score = 1.0f - pos_score;
        }
    } else if (ctx->output->type == kTfLiteInt8) {
        int8_t maxv;
        winner = argmax_int8(ctx->output->data.int8, out_elems, &maxv);
        // Dequantize positive score
        float scale = ctx->output->params.scale;
        int zp = ctx->output->params.zero_point;
        int8_t q = (ctx->pos_index < (int)out_elems) ? ctx->output->data.int8[ctx->pos_index] : (int8_t)zp;
        pos_score = (q - zp) * scale;
        if (out_elems >= 2) {
            int neg_idx = (ctx->pos_index == 0 ? 1 : 0);
            int8_t qn = ctx->output->data.int8[neg_idx];
            neg_score = (qn - zp) * scale;
        } else {
            neg_score = 1.0f - pos_score;
        }
    } else {
        ESP_LOGW(TAG, "Unsupported output type %d", (int)ctx->output->type);
    }

    is_positive = pos_score >= ctx->score_threshold;

    // Update filter window
    window_push(ctx, is_positive ? 1 : 0);

    bool fired = false;
    if (window_ready_and_positive(ctx)) {
        int64_t now_us = esp_timer_get_time();
        if ((now_us - ctx->last_fire_us) / 1000 >= ctx->debounce_ms) {
            ctx->last_fire_us = now_us;
            fired = true;
            ESP_LOGI(TAG, "Triggered: sum=%d/%d thr=%.2f winner=%d pos=%.3f neg=%.3f", ctx->win_sum, ctx->window_size, ctx->score_threshold, winner, pos_score, neg_score);
            maybe_buzz(ctx);
        }
    }

    if (out_result) {
        out_result->is_positive = is_positive;
        out_result->positive_score = pos_score;
        out_result->negative_score = neg_score;
        out_result->winner_index = winner;
    }

    return fired;
}