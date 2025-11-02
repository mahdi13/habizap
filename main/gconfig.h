#pragma once

// Thin config accessors over sdkconfig for inference subsystem
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

// Inference feature toggle
#ifndef CONFIG_HABIZAP_INFER_ENABLE
#define CONFIG_HABIZAP_INFER_ENABLE 1
#endif
static inline int gcfg_infer_enable(void) { return CONFIG_HABIZAP_INFER_ENABLE; }

// Verbose logging for inference
#ifndef CONFIG_HABIZAP_INFER_VERBOSE
#define CONFIG_HABIZAP_INFER_VERBOSE 1
#endif
static inline int gcfg_infer_verbose(void) { return CONFIG_HABIZAP_INFER_VERBOSE; }

// Sliding window length (N)
#ifndef CONFIG_HABIZAP_INFER_WINDOW_SIZE
#define CONFIG_HABIZAP_INFER_WINDOW_SIZE 5
#endif
static inline int gcfg_infer_window_size(void) { return CONFIG_HABIZAP_INFER_WINDOW_SIZE; }

// Positives required in window to trigger
#ifndef CONFIG_HABIZAP_INFER_POSITIVES_REQUIRED
#define CONFIG_HABIZAP_INFER_POSITIVES_REQUIRED 3
#endif
static inline int gcfg_infer_positives_required(void) { return CONFIG_HABIZAP_INFER_POSITIVES_REQUIRED; }

// Target class index to consider as "positive" (0-based). -1 means use top-1 regardless of class.
#ifndef CONFIG_HABIZAP_INFER_POSITIVE_CLASS_INDEX
#define CONFIG_HABIZAP_INFER_POSITIVE_CLASS_INDEX 0
#endif
static inline int gcfg_infer_positive_class_index(void) { return CONFIG_HABIZAP_INFER_POSITIVE_CLASS_INDEX; }

// Confidence threshold in percent (0..100)
#ifndef CONFIG_HABIZAP_INFER_THRESHOLD_PERCENT
#define CONFIG_HABIZAP_INFER_THRESHOLD_PERCENT 80
#endif
static inline int gcfg_infer_threshold_percent(void) { return CONFIG_HABIZAP_INFER_THRESHOLD_PERCENT; }

// Optional arena size override (0 = use model default)
#ifndef CONFIG_HABIZAP_INFER_ARENA_SIZE
#define CONFIG_HABIZAP_INFER_ARENA_SIZE 0
#endif
static inline int gcfg_infer_arena_size(void) { return CONFIG_HABIZAP_INFER_ARENA_SIZE; }

// Buzzer on trigger
#ifndef CONFIG_HABIZAP_INFER_BUZZER_ENABLE
#define CONFIG_HABIZAP_INFER_BUZZER_ENABLE 1
#endif
static inline int gcfg_infer_buzzer_enable(void) { return CONFIG_HABIZAP_INFER_BUZZER_ENABLE; }

// Buzzer pulse pattern (ms)
#ifndef CONFIG_HABIZAP_INFER_BUZZ_ON_MS
#define CONFIG_HABIZAP_INFER_BUZZ_ON_MS 200
#endif
static inline int gcfg_infer_buzz_on_ms(void) { return CONFIG_HABIZAP_INFER_BUZZ_ON_MS; }

#ifndef CONFIG_HABIZAP_INFER_BUZZ_OFF_MS
#define CONFIG_HABIZAP_INFER_BUZZ_OFF_MS 0
#endif
static inline int gcfg_infer_buzz_off_ms(void) { return CONFIG_HABIZAP_INFER_BUZZ_OFF_MS; }

#ifndef CONFIG_HABIZAP_INFER_BUZZ_REPEAT
#define CONFIG_HABIZAP_INFER_BUZZ_REPEAT 1
#endif
static inline int gcfg_infer_buzz_repeat(void) { return CONFIG_HABIZAP_INFER_BUZZ_REPEAT; }

#ifdef __cplusplus
}
#endif
