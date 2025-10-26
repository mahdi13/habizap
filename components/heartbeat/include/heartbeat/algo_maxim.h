#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maxim reference algorithm port (25 Hz, 4-second window expected)
// Computes heart rate (bpm) and SpO2 (%) from IR and RED buffers.
// Returns results via out parameters and validity flags (1 = valid, 0 = invalid).
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,
                                            int32_t n_ir_buffer_length,
                                            uint32_t *pun_red_buffer,
                                            int32_t *pn_spo2,
                                            int8_t *pch_spo2_valid,
                                            int32_t *pn_heart_rate,
                                            int8_t *pch_hr_valid);

#ifdef __cplusplus
}
#endif
