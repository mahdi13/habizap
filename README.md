# habizap

Habizap is a hobby project that explores gesture detection using an ESP32 board, an MPU6050 IMU (accelerometer +
 gyroscope), and a lightweight machine learning model.

⚠️ Disclaimer:
This is an experimental, in-development project for personal and educational purposes only. It is provided as-is. Use,
modification, or extension of this code is entirely your own responsibility.

---

✨ Features
• Collects accelerometer and gyroscope data from an MPU6050 (training.c).
• Uses Edge Impulse and TensorFlow Lite Micro (TFLM) for real-time classification (inference.c).
• Runs on ESP32 with FreeRTOS.
• Optional feedback via vibration motor (vibration.c).

---

Getting started with inference (Edge Impulse + TFLite Micro)

1) Build dependency: add TensorFlow Lite Micro component

This project expects the ESP-IDF TFLM component to be available. The easiest way is via ESP-IDF Component Manager.

- If your project uses component manager (idf.py >= 5): create or edit `idf_component.yml` in the project root with:

```
dependencies:
  espressif/tflite-micro: "^1.0.0"
```

Then run `idf.py reconfigure` or build once to fetch the component.

Alternatively, you can vendor TFLM manually and add to `EXTRA_COMPONENT_DIRS` (not covered here).

~~2) Export your Edge Impulse model as TensorFlow Lite (.tflite)

- In Edge Impulse, go to Deployment -> `TensorFlow Lite (float32)` or `TensorFlow Lite (int8)` (recommended quantized),
  and download the `.tflite` file.

3) Convert the .tflite to a C array and replace `model_data.c`~~

This repository contains `main/model_data.c` with weak default symbols so the app compiles even without a real model.
Replace it with a generated C array from your `.tflite` file. Two simple options:

Option A: Use `xxd -i` (available on macOS/Linux)

```
xxd -i your_model.tflite > main/model_data.c
// Ensure the array name is g_model_data and length is g_model_data_len, e.g.:
// const unsigned char g_model_data[] = { ... };
// const unsigned int  g_model_data_len = sizeof(g_model_data);
```

Option B: Use Python

```
python3 - << 'PY'
import sys
path = 'your_model.tflite'
arr = open(path,'rb').read()
print('const unsigned char g_model_data[] = {')
for i,b in enumerate(arr):
    end = ',' if i+1 < len(arr) else ''
    print(f'0x{b:02x}{end}', end='')
    if (i+1)%12==0: print()
print('\n};')
print('const unsigned int g_model_data_len = sizeof(g_model_data);')
PY
> main/model_data.c
```

4) Configure inference parameters (optional)

Edit `main/gconfig.h` or override via compiler defines / Kconfig later:
- `CONFIG_HABIZAP_INFER_TENSOR_ARENA_SIZE` – memory arena for TFLM (bytes). Adjust to fit your model (80–150 KB typical).
- `CONFIG_HABIZAP_INFER_WINDOW_SIZE` – N for N-of-M smoothing queue.
- `CONFIG_HABIZAP_INFER_POSITIVES_TO_TRIGGER` – K positives required to trigger.
- `CONFIG_HABIZAP_INFER_SCORE_THRESHOLD` – minimum score for a single frame to count as positive.
- `CONFIG_HABIZAP_INFER_DEBOUNCE_MS` – cooldown between triggers.
- `CONFIG_HABIZAP_INFER_POSITIVE_CLASS_INDEX` – index of the "positive" class in model outputs.
- Buzzer/vibration pattern: `CONFIG_HABIZAP_INFER_BUZZ_*`.

5) Use the API from your app

```
#include "inference.h"
#include "vibration.h"

static inference_ctx_t *g_infer = NULL;
static vibration_ctx_t *g_vib = NULL;

void my_init() {
    g_vib = vibration_init();

    inference_config_t cfg;
    inference_default_config(&cfg);
    cfg.vib = g_vib; // optional, to buzz on trigger
    cfg.score_threshold = 0.75f; // example override
    g_infer = inference_init(&cfg);
}

void my_deinit() {
    inference_deinit(g_infer);
    vibration_deinit(g_vib);
}

// When you have a feature vector ready (same layout as used in Edge Impulse)
void my_on_features(const float *features, size_t count) {
    inference_result_t res;
    bool triggered = inference_feed(g_infer, features, count, &res);
    if (triggered) {
        // Already buzzed if cfg.vib != NULL. You can also log more here.
    }
}
```

Notes
- Input shape: `inference_feed` expects a flat float array of the same size as the model input tensor (channel-first or last exactly as trained/exported). For int8 models, values will be quantized internally using the input tensor scale/zero-point.
- Output: the module computes argmax and uses `positive_class_index` and `score_threshold` to decide per-frame positivity.
- Filtering: an N-of-M queue (ring buffer) tallies positives; when the sum reaches K and the debounce period passed, a trigger fires, vibration pattern enqueued, and a log line is printed.

Custom partition table?
- Not required when the model is compiled in as a C array (`model_data.c`). The model bytes are stored in flash as part of the firmware.
- If you decide to store/load the model from external flash (SPIFFS/LittleFS) at runtime, you may need a custom partition table to carve out a filesystem partition. This project does not currently implement runtime loading.

Troubleshooting
- If you see "No model linked" at boot, ensure you've replaced `main/model_data.c` with the generated C array and that the symbols are named `g_model_data` and `g_model_data_len`.
- If you see `AllocateTensors failed`, increase `CONFIG_HABIZAP_INFER_TENSOR_ARENA_SIZE`.
- If you run out of internal RAM, the code falls back to normal heap for the arena; performance may be slightly lower.

