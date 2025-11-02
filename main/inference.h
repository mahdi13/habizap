#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include "vibration.h"

// Opaque context for inference
typedef struct inference_ctx_t inference_ctx_t;

// Configuration for inference filtering and thresholds
typedef struct {
    int window_size;              // N: number of recent predictions to keep
    int positives_to_trigger;     // K: trigger when >= K positives in window
    float score_threshold;        // Score threshold for positive class [0..1]
    int debounce_ms;              // Minimum time between triggers
    int positive_class_index;     // Index of the positive class in model output
    // Optional vibration feedback; if NULL, no buzzing
    vibration_ctx_t *vib;
    int buzz_on_ms;               // per pulse ON
    int buzz_off_ms;              // per pulse OFF
    int buzz_repeat;              // number of pulses
} inference_config_t;

// An optional summary of the last inference
typedef struct {
    bool is_positive;
    float positive_score;
    float negative_score; // if available, else 1 - positive
    int winner_index;     // argmax
} inference_result_t;

// Fill config with sensible defaults from gconfig.h
void inference_default_config(inference_config_t *cfg);

// Create and initialize an inference context (TensorFlow Lite Micro interpreter etc.)
// Returns NULL on failure. The context is owned by the caller and must be destroyed via inference_deinit.
inference_ctx_t *inference_init(const inference_config_t *cfg);

// Destroy and free the inference context
void inference_deinit(inference_ctx_t *ctx);

// Run inference on a feature vector (matching the model input shape).
// Returns true if the internal filter triggered a positive event (debounce respected).
// Optionally fills out_result if non-NULL.
bool inference_feed(inference_ctx_t *ctx, const float *features, size_t feature_count, inference_result_t *out_result);

#ifdef __cplusplus
}
#endif
