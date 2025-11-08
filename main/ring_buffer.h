#pragma once

#include "freertos/semphr.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {

#endif

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float temp_c;
} motion_sample_t;

typedef struct {
    motion_sample_t *data;
    int capacity;
    int write_idx;
    int read_idx;
    SemaphoreHandle_t mutex;
} motion_buffer_t;

void motion_buffer_init(motion_buffer_t *buf, motion_sample_t *storage, int capacity);

bool motion_buffer_push(motion_buffer_t *buf, const motion_sample_t *sample);

bool motion_buffer_pop(motion_buffer_t *buf, motion_sample_t *out);

int motion_buffer_count(motion_buffer_t *buf);

#ifdef __cplusplus
}
#endif
