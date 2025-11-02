#include "inference.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "esp_log.h"

#include "gconfig.h"
#include "vibration.h"

// C-callable TFLM shim implemented in inference_tflm.cpp
#ifdef __cplusplus
extern "C" {
#endif
void *tflm_create(int arena_size_override);
void tflm_destroy(void *p);
int tflm_input_len(void *p);
int tflm_num_classes(void *p);
int tflm_invoke(void *p, const float *input, size_t input_len, float *out_scores, size_t out_len);
#ifdef __cplusplus
}
#endif

static const char *TAG = "INFER";

struct inference_ctx_t {
    void *tflm;                 // opaque TFLM interpreter context
    vibration_ctx_t *vib;       // not owned
    int input_len;              // model input length
    int num_classes;            // model output length

    // Sliding window filter state (ring buffer of bool positives)
    uint8_t *win_buf;           // length = window_size
    int window_size;
    int positives_required;
    int write_idx;              // next write position in ring
    int count;                  // how many entries filled (<= window_size)

    // Last scores buffer to expose via inference_result_t.scores
    float *scores;
};

static inline int top1(const float *scores, int n, float *out_score) {
    int best = 0;
    float bs = scores[0];
    for (int i = 1; i < n; ++i) {
        if (scores[i] > bs) { best = i; bs = scores[i]; }
    }
    if (out_score) *out_score = bs;
    return best;
}

inference_ctx_t *inference_init(vibration_ctx_t *vib) {
    if (!gcfg_infer_enable()) {
        ESP_LOGW(TAG, "Inference disabled in config");
        return NULL;
    }

    inference_ctx_t *ctx = (inference_ctx_t *)calloc(1, sizeof(inference_ctx_t));
    if (!ctx) return NULL;

    int arena_override = gcfg_infer_arena_size();
    ctx->tflm = tflm_create(arena_override);
    if (!ctx->tflm) {
        ESP_LOGE(TAG, "Failed to create TFLM context");
        free(ctx);
        return NULL;
    }

    ctx->input_len = tflm_input_len(ctx->tflm);
    ctx->num_classes = tflm_num_classes(ctx->tflm);
    ctx->vib = vib;

    // Allocate buffers
    ctx->scores = (float *)malloc(sizeof(float) * (ctx->num_classes > 0 ? ctx->num_classes : 1));

    ctx->window_size = gcfg_infer_window_size();
    if (ctx->window_size < 1) ctx->window_size = 1;
    ctx->win_buf = (uint8_t *)calloc((size_t)ctx->window_size, 1);
    ctx->positives_required = gcfg_infer_positives_required();
    if (ctx->positives_required < 1) ctx->positives_required = 1;

    ctx->write_idx = 0;
    ctx->count = 0;

    ESP_LOGI(TAG, "Inference init: input_len=%d, classes=%d, window=%d, pos_req=%d, thr=%d%%, pos_class=%d",
             ctx->input_len, ctx->num_classes, ctx->window_size, ctx->positives_required,
             gcfg_infer_threshold_percent(), gcfg_infer_positive_class_index());

    return ctx;
}

void inference_deinit(inference_ctx_t *ctx) {
    if (!ctx) return;
    if (ctx->tflm) tflm_destroy(ctx->tflm);
    free(ctx->scores);
    free(ctx->win_buf);
    free(ctx);
}

int inference_input_len(inference_ctx_t *ctx) {
    return ctx ? ctx->input_len : -1;
}

int inference_num_classes(inference_ctx_t *ctx) {
    return ctx ? ctx->num_classes : -1;
}

static bool filter_push_and_check(inference_ctx_t *ctx, bool positive) {
    // update ring buffer
    int idx = ctx->write_idx;
    uint8_t old = ctx->win_buf[idx];
    ctx->win_buf[idx] = positive ? 1 : 0;
    ctx->write_idx = (idx + 1) % ctx->window_size;
    if (ctx->count < ctx->window_size) ctx->count++;

    // Recompute sum for small N (simple and robust)
    int s = 0;
    for (int i = 0; i < ctx->count; ++i) s += ctx->win_buf[i];
    return s >= ctx->positives_required;
}

bool inference_run(inference_ctx_t *ctx, const float *input, size_t input_len, inference_result_t *out) {
    if (!ctx || !input || input_len == 0 || !out) return false;
    if ((int)input_len != ctx->input_len) {
        ESP_LOGE(TAG, "inference_run: input_len=%u != expected %d", (unsigned)input_len, ctx->input_len);
        return false;
    }

    if (tflm_invoke(ctx->tflm, input, input_len, ctx->scores, ctx->num_classes) != 0) {
        ESP_LOGE(TAG, "TFLM invoke error");
        return false;
    }

    // Determine top-1 and whether positive
    float best_score = 0.0f;
    int best_idx = top1(ctx->scores, ctx->num_classes, &best_score);

    int thr_pct = gcfg_infer_threshold_percent();
    float thr = (float)thr_pct / 100.0f;

    int pos_cls = gcfg_infer_positive_class_index();
    float pos_score = (pos_cls >= 0 && pos_cls < ctx->num_classes) ? ctx->scores[pos_cls] : best_score;
    bool positive = pos_score >= thr;

    bool triggered = filter_push_and_check(ctx, positive);

    if (gcfg_infer_verbose()) {
        printf("[infer] scores:");
        for (int i = 0; i < ctx->num_classes; ++i) {
            printf(" %0.3f", (double)ctx->scores[i]);
        }
        printf(" | top=%d(%0.2f) pos=%d thr=%.2f trig=%d\n", best_idx, (double)best_score, positive, (double)thr, triggered);
    }

    if (triggered) {
        // Buzz and print
        ESP_LOGI(TAG, "TRIGGER: class=%d score=%.2f (pos_cls=%d score=%.2f)", best_idx, (double)best_score, pos_cls, (double)pos_score);
        if (gcfg_infer_buzzer_enable() && ctx->vib) {
            (void) vibration_pulse(ctx->vib, gcfg_infer_buzz_on_ms(), gcfg_infer_buzz_off_ms(), gcfg_infer_buzz_repeat());
        }
    }

    out->top_class = best_idx;
    out->top_score = best_score;
    out->scores = ctx->scores;
    out->num_scores = ctx->num_classes;
    out->positive = positive;
    out->triggered = triggered;

    return true;
}
