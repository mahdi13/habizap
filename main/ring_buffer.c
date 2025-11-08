#include "ring_buffer.h"

#include <string.h>
#include "freertos/semphr.h"

void motion_buffer_init(motion_buffer_t *buf, motion_sample_t *storage, int capacity) {
    buf->data = storage;
    buf->capacity = capacity;
    buf->write_idx = 0;
    buf->read_idx = 0;
    buf->mutex = xSemaphoreCreateMutex();
}

bool motion_buffer_push(motion_buffer_t *buf, const motion_sample_t *sample) {
    xSemaphoreTake(buf->mutex, portMAX_DELAY);
    int next = (buf->write_idx + 1) % buf->capacity;

    if (next == buf->read_idx) {
        // Buffer full — overwrite oldest (drop one)
        buf->read_idx = (buf->read_idx + 1) % buf->capacity;
    }

    buf->data[buf->write_idx] = *sample;
    buf->write_idx = next;
    xSemaphoreGive(buf->mutex);
    return true;
}

bool motion_buffer_pop(motion_buffer_t *buf, motion_sample_t *out) {
    xSemaphoreTake(buf->mutex, portMAX_DELAY);
    if (buf->read_idx == buf->write_idx) {
        xSemaphoreGive(buf->mutex);
        return false; // empty
    }
    *out = buf->data[buf->read_idx];
    buf->read_idx = (buf->read_idx + 1) % buf->capacity;
    xSemaphoreGive(buf->mutex);
    return true;
}

int motion_buffer_count(motion_buffer_t *buf) {
    xSemaphoreTake(buf->mutex, portMAX_DELAY);
    int count = buf->write_idx >= buf->read_idx
                    ? buf->write_idx - buf->read_idx
                    : buf->capacity - buf->read_idx + buf->write_idx;
    xSemaphoreGive(buf->mutex);
    return count;
}
