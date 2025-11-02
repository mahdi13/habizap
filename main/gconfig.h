#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Global configurable defaults for inference and alerts.
// Override via sdkconfig (add Kconfig later) or by defining these macros before including this header.

#ifndef CONFIG_HABIZAP_INFER_WINDOW_SIZE
#define CONFIG_HABIZAP_INFER_WINDOW_SIZE 8
#endif

#ifndef CONFIG_HABIZAP_INFER_POSITIVES_TO_TRIGGER
#define CONFIG_HABIZAP_INFER_POSITIVES_TO_TRIGGER 3
#endif

#ifndef CONFIG_HABIZAP_INFER_SCORE_THRESHOLD
#define CONFIG_HABIZAP_INFER_SCORE_THRESHOLD 0.80f
#endif

#ifndef CONFIG_HABIZAP_INFER_DEBOUNCE_MS
#define CONFIG_HABIZAP_INFER_DEBOUNCE_MS 1500
#endif

#ifndef CONFIG_HABIZAP_INFER_POSITIVE_CLASS_INDEX
#define CONFIG_HABIZAP_INFER_POSITIVE_CLASS_INDEX 1
#endif

#ifndef CONFIG_HABIZAP_INFER_TENSOR_ARENA_SIZE
// Adjust based on your model. Many tiny models fit in 40-80 KB; larger may need 150+ KB.
#define CONFIG_HABIZAP_INFER_TENSOR_ARENA_SIZE (80 * 1024)
#endif

#ifndef CONFIG_HABIZAP_INFER_LOG_TAG
#define CONFIG_HABIZAP_INFER_LOG_TAG "INFER"
#endif

// Buzzer/vibration feedback defaults
#ifndef CONFIG_HABIZAP_INFER_BUZZ_ON_MS
#define CONFIG_HABIZAP_INFER_BUZZ_ON_MS 180
#endif
#ifndef CONFIG_HABIZAP_INFER_BUZZ_OFF_MS
#define CONFIG_HABIZAP_INFER_BUZZ_OFF_MS 120
#endif
#ifndef CONFIG_HABIZAP_INFER_BUZZ_REPEAT
#define CONFIG_HABIZAP_INFER_BUZZ_REPEAT 2
#endif

#ifdef __cplusplus
}
#endif
