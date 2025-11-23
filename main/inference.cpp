#include "inference.h"

#include "ring_buffer.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "esp_log.h"

static const char *TAG = "INFERENCE";

// Your model parameters
#define AXES 6
#define SLICE_SIZE EI_CLASSIFIER_RAW_SAMPLE_COUNT
#define SAMPLE_FREQUENCY EI_CLASSIFIER_FREQUENCY
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_FREQUENCY)

static motion_buffer_t *shared_buffer = nullptr;

static int get_data_callback(size_t offset, size_t length, float *out_ptr) {
    ESP_LOGI(TAG, "get_data_callback: offset=%d, length=%d", (int)offset, (int)length);
    if (!shared_buffer) {
        ESP_LOGE(TAG, "shared_buffer is null");
        return -1;
    }

    ESP_LOGI(TAG, "get_data_callback: checking available samples...");
    int available = motion_buffer_count(shared_buffer);
    ESP_LOGI(TAG, "get_data_callback: available samples = %d", available);
    if (available < (int) SLICE_SIZE) {
        ESP_LOGW(TAG, "Not enough samples yet (%d / %d)", available, (int)SLICE_SIZE);
        return -1;
    }

    // Read the latest SLICE_SIZE samples (FIFO order)
    motion_sample_t temp[SLICE_SIZE];
    ESP_LOGI(TAG, "get_data_callback: peeking latest samples...");
    motion_buffer_peek_latest(shared_buffer, temp, SLICE_SIZE); // non-destructive peek helper preferred
    ESP_LOGI(TAG, "get_data_callback: copying samples to output buffer...");

    int out_idx = 0;
    for (int i = 0; i < SLICE_SIZE; i++) {
        out_ptr[out_idx++] = temp[i].ax;
        out_ptr[out_idx++] = temp[i].ay;
        out_ptr[out_idx++] = temp[i].az;
        out_ptr[out_idx++] = temp[i].gx;
        out_ptr[out_idx++] = temp[i].gy;
        out_ptr[out_idx++] = temp[i].gz;
    }

    ESP_LOGI(TAG, "get_data_callback: provided %d samples", (int)(SLICE_SIZE * AXES));

    return EIDSP_OK;
}

extern "C" void inference_task(void *arg) {
    shared_buffer = (motion_buffer_t *) arg;

    ESP_LOGI(TAG, "Initializing classifier (frequency: %d Hz)...", SAMPLE_FREQUENCY);
    run_classifier_init();

    ei_impulse_result_t result;
    EI_IMPULSE_ERROR ei_err;

    signal_t signal;
    signal.total_length = SLICE_SIZE * AXES;
    signal.get_data = &get_data_callback;

    ESP_LOGI(TAG, "Inference task started, waiting for enough samples...");

    while (true) {
        // Check if enough samples collected for one window
        if (motion_buffer_count(shared_buffer) < SLICE_SIZE) {
            ESP_LOGD(TAG, "Waiting for more samples (so far %d / %d)", motion_buffer_count(shared_buffer),
                     (int)SLICE_SIZE);
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        // Run inference
        ESP_LOGI(TAG, "Running classifier (frequency: %d Hz)...", SAMPLE_FREQUENCY);
        ei_err = run_classifier(&signal, &result, false);

        if (ei_err == EI_IMPULSE_OK) {
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                float v = result.classification[ix].value;
                if (v > 0.6f) {
                    ESP_LOGI(TAG, "Detected: %s (%.2f)", result.classification[ix].label, v);
                }
            }
        } else {
            ESP_LOGW(TAG, "Inference error: %d", ei_err);
        }

        // Wait for next window:
        vTaskDelay(pdMS_TO_TICKS((SLICE_SIZE * 1000) / SAMPLE_FREQUENCY));
    }
}

void inference_task_start(motion_buffer_t *buf) {
    xTaskCreatePinnedToCore(inference_task, "inference_task", 8192, buf, 4, NULL, 1);
}
