#pragma once

#ifdef __cplusplus
extern "C" {

#endif

#include <stddef.h>
#include <stdbool.h>

// Opaque context for the vibration subsystem
typedef struct vibration_ctx_t vibration_ctx_t;

// Vibration subsystem public API
// Patterns are arrays of durations in milliseconds: [on, off, on, off, ...]
// The first entry is ON duration. The last entry may be ON or OFF; the motor will be turned OFF at the end.
// String patterns are comma/space-separated integers, e.g. "200,100,200 400".

// Initialize the vibration subsystem (GPIO + worker task + queue). Returns context pointer on success, NULL on failure.
vibration_ctx_t *vibration_init(void);

// Stop the motor, free resources, and destroy the context.
void vibration_deinit(vibration_ctx_t *ctx);

// Enqueue a pattern given an array of durations (ms). Returns true if queued.
bool vibration_enqueue_pattern_ms(vibration_ctx_t *ctx, const int *durations_ms, size_t count);

// Enqueue a pattern given a string of numbers (comma/space separated). Returns true if queued.
bool vibration_enqueue_pattern_str(vibration_ctx_t *ctx, const char *pattern_str);

// Convenience: enqueue a simple pulse (on_ms, off_ms, repeat)
bool vibration_pulse(vibration_ctx_t *ctx, int on_ms, int off_ms, int repeat);

#ifdef __cplusplus
}
#endif
