#pragma once

#ifdef __cplusplus
extern "C" {

#endif

#include <stdbool.h>

typedef struct {
    bool include_jerk;
    bool include_magnitudes;
} training_config_t;

void start_data_collection(const training_config_t *config);

#ifdef __cplusplus
}
#endif
