// Current Monitor: ADC→current scaling/filtering API (header stub)
#ifndef CURRENT_MONITOR_H
#define CURRENT_MONITOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float vref_volts;            // e.g., 3.3
    uint16_t adc_max_code;       // e.g., 4095 for 12-bit
    float divider_ratio;         // Vadc = divider_ratio * Vsense (e.g., 0.6)
    float left_zero_volts;       // ≈ Vcc/2 after divider at 0 A
    float right_zero_volts;      // same for right
    float left_mV_per_A;         // sensor sensitivity (pre-divider), mV/A
    float right_mV_per_A;        // sensor sensitivity (pre-divider), mV/A
    float lpf_alpha;             // 0..1 EMA factor for filtering (optional)
} CurrentMonConfig;

void  CurrentMon_Init(const CurrentMonConfig* cfg);
void  CurrentMon_OnAdc(uint16_t adc_ch8_raw, uint16_t adc_ch11_raw);

float CurrentMon_LeftA(void);
float CurrentMon_RightA(void);

// Health/stuck detection (non-zero when readings are plausible)
uint8_t CurrentMon_Healthy(void);

#ifdef __cplusplus
}
#endif

#endif // CURRENT_MONITOR_H

