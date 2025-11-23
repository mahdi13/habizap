#pragma once

#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

// Configuration for the inference engine
void inference_task_start(motion_buffer_t *buf);

#ifdef __cplusplus
}
#endif
