#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to an inference instance
typedef struct inference_instance_t inference_handle_t;

// Inference result structure
typedef struct {
    int top_index; // index in ei_classifier_inferencing_categories[]
    float top_score; // top class probability [0..1]
    bool is_positive; // convenience flag using configured label+threshold
    uint64_t timestamp_ms; // when the decision was produced
} inference_result_t;

// Configuration for the inference engine
typedef struct {
    uint32_t queue_len; // pending messages in the queue (default from Kconfig)
    bool use_continuous; // use run_classifier_continuous() (default from Kconfig)
    float positive_threshold; // threshold for is_positive (default from Kconfig)
    const char *positive_label; // label name to treat as "positive" (default from Kconfig)
    uint32_t task_stack; // FreeRTOS task stack size (bytes)
    uint32_t task_priority; // FreeRTOS task priority
} inference_config_t;

// Create an inference engine and start its worker task
bool inference_create(const inference_config_t *cfg, inference_handle_t **out);

// Destroy an inference engine and stop its task (safe to call with NULL)
void inference_destroy(inference_handle_t *h);

// Feed a contiguous chunk of samples to be accumulated/streamed
size_t inference_feed_samples(inference_handle_t *h, const float *samples, size_t count);

// Get the latest inference result (non-blocking). Returns false if no result yet
bool inference_get_latest(inference_handle_t *h, inference_result_t *out);

#ifdef __cplusplus
}
#endif
