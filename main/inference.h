#pragma once

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declare vibration context so app can pass it in (owned by app)
typedef struct vibration_ctx_t vibration_ctx_t;

// Opaque inference context type (owned by app)
typedef struct inference_ctx_t inference_ctx_t;

// Minimal result info
typedef struct {
    int top_class;          // index of the highest-scoring class
    float top_score;        // probability of the top class (0..1)
    const float *scores;    // pointer to output scores buffer (length = num_classes)
    int num_scores;         // length of the scores array
    bool positive;          // whether this frame is considered positive (after thresholding only)
    bool triggered;         // whether the sliding window triggered this call
} inference_result_t;

// Create and initialize an inference context. App owns and stores the returned pointer.
//  - vib: optional vibration context; can be NULL if you don't want buzzing
// Returns NULL on failure.
inference_ctx_t *inference_init(vibration_ctx_t *vib);

// Destroy and free the inference context.
void inference_deinit(inference_ctx_t *ctx);

// Return expected input length (number of float features) for the model, or -1 on error.
int inference_input_len(inference_ctx_t *ctx);

// Return number of output classes, or -1 on error.
int inference_num_classes(inference_ctx_t *ctx);

// Run one inference. The input must be a contiguous float array of length = inference_input_len().
// On success returns true and fills out 'out'. The 'scores' pointer inside 'out' is valid until
// the next call to inference_run or until the context is destroyed.
bool inference_run(inference_ctx_t *ctx, const float *input, size_t input_len, inference_result_t *out);

#ifdef __cplusplus
}
#endif
