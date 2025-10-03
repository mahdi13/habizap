#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "mpu6050.h"


#ifdef __cplusplus
extern "C" {



#endif

typedef struct {
    mpu6050_handle_t mpu;
} app_context_t;

app_context_t *app_ctx(void);

void app_run(void);

#ifdef __cplusplus
}
#endif
