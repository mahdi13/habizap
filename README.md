# habizap

Habizap is a hobby project that explores gesture detection using an ESP32 board, an MPU6050 IMU (accelerometer +
gyroscope), and a lightweight machine learning model.

⚠️ Disclaimer:
This is an experimental, in-development project for personal and educational purposes only. It is provided as-is. Use,
modification, or extension of this code is entirely your own responsibility.

---

✨ Features
• Collects accelerometer and gyroscope data from an MPU6050 (training.c).
• Uses Edge Impulse + TensorFlow Lite Micro for real-time gesture classification (inference.c + inference_tflm.cpp).
• Runs on ESP32 with FreeRTOS.
• Can optionally trigger external feedback devices (e.g., vibration motor).

---

Getting started with inference (Edge Impulse + TFLM)

1) Place your model
   - Export your Edge Impulse C++ Library for ESP32 (or generic C++). Copy the following into the repo:
     • `habizap-cpp-mcu-v1/tflite-model/tflite_learn_XXXXXX_Y.h` (already present as example: `tflite_learn_815924_6.h`)
     • The corresponding `.tflite` file should be next to it: `habizap-cpp-mcu-v1/tflite-model/tflite_learn_XXXXXX_Y.tflite`.
       The build links it via INCBIN from the header. If you update the model, just overwrite the `.tflite` file and header.
     • `habizap-cpp-mcu-v1/tflite-model/tflite-resolver.h` must list the ops used by your model. Update it if your model uses other ops.

   - Note: The provided EI headers use INCBIN, which embeds the `.tflite` file directly into the firmware. No filesystem needed.

2) Configure via menuconfig
   - Run: `idf.py menuconfig` → HabiZap Configuration → Inference
     • Enable Inference (on by default)
     • Verbose Inference Logs (optional)
     • Prediction Window Size (N): how many recent frames to keep
     • Positives Required in Window: how many of those N must be positive to trigger
     • Positive Class Index: set to the class index you care about, or -1 to use the top-1 class as "positive"
     • Confidence Threshold (%): minimum probability to treat a frame as positive
     • TFLM Tensor Arena Override: leave 0 to use model default; increase only if AllocateTensors fails
     • Buzz on Trigger + pattern (on/off/repeat)

3) Use the API
   - The application owns the inference context. Minimal example:

```c
#include "inference.h"
#include "vibration.h"

static inference_ctx_t *g_inf = NULL;
static vibration_ctx_t *g_vib = NULL;

void app_start(void) {
    g_vib = vibration_init();
    g_inf = inference_init(g_vib); // pass NULL if you don't want buzzing
}

void app_loop_once(const float *features, size_t len) {
    if (!g_inf) return;
    inference_result_t r;
    if (inference_run(g_inf, features, len, &r)) {
        // Optional: react to r.triggered or inspect r.scores
    }
}

void app_stop(void) {
    inference_deinit(g_inf);
    vibration_deinit(g_vib);
}
```

4) Partition table
   - The default ESP-IDF partition table is sufficient for typical small TFLite Micro models (tens of KB).
   - If your model grows and you hit flash/OTA size limits, consider switching to a larger factory app partition or custom partition table.
   - As shipped, no custom partition table is required.

5) Updating your model
   - Replace `habizap-cpp-mcu-v1/tflite-model/tflite_learn_XXXXXX_Y.tflite` and its header if Edge Impulse generated a new name.
   - Update `tflite-resolver.h` if new ops are required by the updated model.
   - Rebuild: `idf.py build flash monitor`.

