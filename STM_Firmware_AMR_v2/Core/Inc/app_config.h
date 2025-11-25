// Application-wide configuration and tuning constants
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// Direction polarity: set to 1 for forward, 0 to invert that wheel
#define LEFT_DIR_POLARITY    1
#define RIGHT_DIR_POLARITY   0

// Encoder/rate reporting configuration
#define ENCODER_COUNTS_PER_REV  2400U   // 600 PPR A-4 quadrature
#define SAMPLE_INTERVAL_MS      100U    // 100 ms sample period for UART reporting

// Current sensor scaling (ACS758 50B @5 V; divider 10k top / 20k bottom)
#define ADC_VREF_VOLTS        3.3f
#define ADC_MAX_COUNTS        4095.0f
#define ADC_MID_COUNTS        2048U
#define CURR_DIVIDER_RATIO    0.667f   // Vadc = 0.667 * Vsense (5 V -> ~3.3 V)
#define CURR_ZERO_VOLTS       2.5f     // sensor Vout at 0 A (before divider)
#define CURR_SENS_VOLTS_PER_A 0.040f   // 40 mV/A @ 5 V supply
#define CURR_ZERO_SAMPLES     64U      // samples to average for zero offset
#define CURR_AVG_SAMPLES      8U       // oversample to reduce noise without analog RC
#define LEFT_CURR_POLARITY    1        // set -1 to flip left current sign
#define RIGHT_CURR_POLARITY   -1       // set -1 to flip right current sign
#define CURR_LPF_ALPHA        0.2f     // low-pass filter alpha (0..1), higher = less smoothing

#endif // APP_CONFIG_H
