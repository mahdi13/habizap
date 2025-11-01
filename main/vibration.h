#pragma once

#ifdef __cplusplus
extern "C" {

#endif

#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Vibration subsystem public API and context
// Patterns are arrays of durations in milliseconds: [on, off, on, off, ...]
// The first entry is ON duration. The last entry may be ON or OFF; the motor will be turned OFF at the end.
// String patterns are comma/space-separated integers, e.g. "200,100,200 400".

typedef struct {
    QueueHandle_t queue; // queue of commands (each command is a pattern)
    TaskHandle_t task; // worker task processing the queue
    int gpio; // GPIO controlling the motor/driver
    bool active_high; // true if HIGH = ON
    int default_on_ms; // default on duration if needed by named patterns
    int default_off_ms; // default off duration if needed by named patterns
    size_t max_items; // queue length (for informational purposes)
} vibration_ctx_t;


// Vibration API
bool vibration_init(void);

// Enqueue a pattern given an array of durations (ms). Returns true if queued.
bool vibration_enqueue_pattern_ms(const int *durations_ms, size_t count);

// Enqueue a pattern given a string of numbers (comma/space separated). Returns true if queued.
bool vibration_enqueue_pattern_str(const char *pattern_str);

// Convenience: enqueue a simple pulse (on_ms, off_ms, repeat)
bool vibration_pulse(int on_ms, int off_ms, int repeat);


#ifdef __cplusplus
}
#endif
