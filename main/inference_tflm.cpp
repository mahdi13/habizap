#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "edge-impulse-sdk/tensorflow/lite/c/common.h"
#include "edge-impulse-sdk/tensorflow/lite/micro/micro_interpreter.h"
#include "edge-impulse-sdk/tensorflow/lite/micro/micro_profiler.h"
#include "edge-impulse-sdk/tensorflow/lite/micro/recording_micro_allocator.h"
#include "edge-impulse-sdk/tensorflow/lite/schema/schema_generated.h"
#include "edge-impulse-sdk/tensorflow/lite/version.h"

#include "tflite-model/tflite-resolver.h"
#include "tflite-model/tflite_learn_815924_6.h"

extern "C" {
// C-callable thin shim used by inference.c

typedef struct {
    const tflite::Model *model;
    tflite::MicroInterpreter *interpreter;
    tflite::MicroProfiler *profiler;
    TfLiteTensor *input;
    TfLiteTensor *output;
    uint8_t *arena;
    size_t arena_size;
    int input_len;
    int num_classes;
} tflm_ctx_t;

static const char *TAG = "INFER_TFLM";

// Create and initialize TFLM structures. Returns NULL on failure.
void *tflm_create(int arena_size_override) {
    // Map model buffer
    const unsigned char *model_data = tflite_learn_815924_6;
    unsigned int model_len = tflite_learn_815924_6_len;

    if (!model_data || model_len == 0) {
        ESP_LOGE(TAG, "Model buffer is empty");
        return nullptr;
    }

    auto *ctx = new (std::nothrow) tflm_ctx_t();
    if (!ctx) return nullptr;
    memset(ctx, 0, sizeof(*ctx));

    ctx->model = tflite::GetModel(model_data);
    if (ctx->model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema %d not equal to supported %d", ctx->model->version(), TFLITE_SCHEMA_VERSION);
        delete ctx;
        return nullptr;
    }

    // Arena
    size_t arena_size = arena_size_override > 0 ? (size_t)arena_size_override : (size_t)tflite_learn_815924_6_arena_size;
    ctx->arena = (uint8_t *)heap_caps_malloc(arena_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!ctx->arena) {
        // fallback to regular malloc
        ctx->arena = (uint8_t *)malloc(arena_size);
    }
    if (!ctx->arena) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena (%u bytes)", (unsigned)arena_size);
        delete ctx;
        return nullptr;
    }
    ctx->arena_size = arena_size;

    // Resolver from EI file
    EI_TFLITE_RESOLVER

    // Profiler is optional
    ctx->profiler = new (std::nothrow) tflite::MicroProfiler();

    // Create interpreter
    ctx->interpreter = new (std::nothrow) tflite::MicroInterpreter(ctx->model, resolver, ctx->arena, ctx->arena_size, nullptr, ctx->profiler);
    if (!ctx->interpreter) {
        ESP_LOGE(TAG, "Failed to create MicroInterpreter");
        free(ctx->arena);
        delete ctx->profiler;
        delete ctx;
        return nullptr;
    }

    TfLiteStatus a = ctx->interpreter->AllocateTensors();
    if (a != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors failed");
        delete ctx->interpreter;
        free(ctx->arena);
        delete ctx->profiler;
        delete ctx;
        return nullptr;
    }

    ctx->input = ctx->interpreter->input(0);
    ctx->output = ctx->interpreter->output(0);

    // Determine input length (expect 1D float)
    ctx->input_len = 1;
    for (int i = 0; i < ctx->input->dims->size; ++i) ctx->input_len *= ctx->input->dims->data[i];
    if (ctx->input->type != kTfLiteFloat32) {
        ESP_LOGW(TAG, "Model input type is %d, expected float32", (int)ctx->input->type);
    }

    // Output classes
    ctx->num_classes = 1;
    if (ctx->output && ctx->output->dims) {
        ctx->num_classes = 1;
        for (int i = 0; i < ctx->output->dims->size; ++i) ctx->num_classes *= ctx->output->dims->data[i];
    }

    ESP_LOGI(TAG, "TFLM ready: input_len=%d, num_classes=%d, arena=%u bytes", ctx->input_len, ctx->num_classes, (unsigned)ctx->arena_size);

    return ctx;
}

void tflm_destroy(void *p) {
    if (!p) return;
    auto *ctx = (tflm_ctx_t *)p;
    delete ctx->interpreter;
    delete ctx->profiler;
    if (ctx->arena) free(ctx->arena);
    delete ctx;
}

int tflm_input_len(void *p) {
    if (!p) return -1;
    return ((tflm_ctx_t *)p)->input_len;
}

int tflm_num_classes(void *p) {
    if (!p) return -1;
    return ((tflm_ctx_t *)p)->num_classes;
}

// Copy floats into input tensor and invoke. Returns 0 on success; -1 on error.
int tflm_invoke(void *p, const float *input, size_t input_len, float *out_scores, size_t out_len) {
    if (!p) return -1;
    auto *ctx = (tflm_ctx_t *)p;
    if ((int)input_len != ctx->input_len) {
        ESP_LOGE(TAG, "Bad input length %u (expected %d)", (unsigned)input_len, ctx->input_len);
        return -1;
    }
    if ((int)out_len < ctx->num_classes) {
        ESP_LOGE(TAG, "Output buffer too small %u (need %d)", (unsigned)out_len, ctx->num_classes);
        return -1;
    }

    float *in = ctx->input->data.f;
    memcpy(in, input, sizeof(float) * input_len);

    if (ctx->interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke failed");
        return -1;
    }

    const float *scores = ctx->output->data.f;
    memcpy(out_scores, scores, sizeof(float) * ctx->num_classes);

    return 0;
}

} // extern "C"
