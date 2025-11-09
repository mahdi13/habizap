#include "ring_buffer.h"
#include <string.h>

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

int motion_buffer_pop_many(motion_buffer_t *buf, motion_sample_t *out, int count) {
    xSemaphoreTake(buf->mutex, portMAX_DELAY);

    // Calculate available samples
    int available = buf->write_idx >= buf->read_idx
                        ? buf->write_idx - buf->read_idx
                        : buf->capacity - buf->read_idx + buf->write_idx;

    if (count > available) {
        count = available; // Only pop what’s available
    }

    for (int i = 0; i < count; i++) {
        out[i] = buf->data[buf->read_idx];
        buf->read_idx = (buf->read_idx + 1) % buf->capacity;
    }

    xSemaphoreGive(buf->mutex);
    return count;
}

int motion_buffer_peek_latest(motion_buffer_t *buf, motion_sample_t *out, int count) {
    if (!buf || !out || count <= 0) return 0;

    xSemaphoreTake(buf->mutex, portMAX_DELAY);

    // Compute available directly (avoid deadlock)
    int available = buf->write_idx >= buf->read_idx
                        ? buf->write_idx - buf->read_idx
                        : buf->capacity - buf->read_idx + buf->write_idx;

    if (available == 0) {
        xSemaphoreGive(buf->mutex);
        return 0;
    }

    if (count > available) count = available;

    int start = buf->write_idx - count;
    if (start < 0) start += buf->capacity;

    for (int i = 0; i < count; i++) {
        int idx = (start + i) % buf->capacity;
        out[i] = buf->data[idx];
    }

    xSemaphoreGive(buf->mutex);
    return count;
}
