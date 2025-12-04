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
#define CURR_AVG_SAMPLES      16U      // oversample to reduce noise without analog RC
#define CURR_ZERO_VALID_WINDOW_COUNTS 700U  // reject zero-cal values too far from mid-scale
#define CURR_ZERO_TRACK_ALPHA 0.02f    // very slow IIR to track drift in zero offset
#define CURR_ZERO_TRACK_MAX_DELTA_COUNTS 20U   // track only when delta is tiny (~<0.6 A)
#define CURR_ZERO_TRACK_CURRENT_MA 200        // only track when measured current is near zero
#define LEFT_CURR_POLARITY    1        // set -1 to flip left current sign
#define RIGHT_CURR_POLARITY   1        // set -1 to flip right current sign
#define CURR_LPF_ALPHA        0.1f     // low-pass filter alpha (0..1), higher = less smoothing

#endif // APP_CONFIG_H
