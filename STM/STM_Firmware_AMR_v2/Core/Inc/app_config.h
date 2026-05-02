// Application-wide configuration and tuning constants
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// Direction polarity: set to 1 for forward, 0 to invert that wheel
#define LEFT_DIR_POLARITY    0
#define RIGHT_DIR_POLARITY   1

// E-stop input configuration
#define ESTOP_ACTIVE_LOW 1
#define ESTOP_DEBOUNCE_MS 10U

// Encoder/rate reporting configuration
#define ENCODER_COUNTS_PER_REV  2400U   // TI12 quadrature decode target; verify with bench rotation test
#define CONTROL_LOOP_HZ         100U    // fixed-rate control loop (Hz)
#define CONTROL_LOOP_DT_MS      (1000U / CONTROL_LOOP_HZ)
#define CONTROL_LOOP_DT_S       (1.0f / (float)CONTROL_LOOP_HZ)
#define TELEMETRY_DECIMATION    10U     // send telemetry every N control ticks (100 Hz / 10 = 10 Hz)
#define SAMPLE_INTERVAL_MS      100U    // legacy/telemetry period reference (ms)
#define ROS_CRITICAL_PUB_PERIOD_MS 100U // wheel_state/fault/duty/safety publish period (ms)
#define ROS_DIAG_PUB_PERIOD_MS     500U // current/ADC diagnostic publish period (ms)
#define ROS_EXEC_DELAY_MS       10U     // executor loop delay to service subscriptions (ms)
#define SERIAL_TELEMETRY_ENABLE 0       // set to 1 to disable ROS pub task and stream UART telemetry
#define SERIAL_TELEMETRY_PERIOD_MS SAMPLE_INTERVAL_MS // UART telemetry period (ms)
#define SERIAL_TELEMETRY_HEADER_PERIOD_MS 1000U       // resend header so plotter can sync (ms)
#define TRACK_WIDTH_M           0.381f  // calibrated effective track width from 360-degree spin test
#define WHEEL_RADIUS_M          0.0615f // wheel radius (meters)

// Current sensor scaling (ACS758 50B @5 V; divider 10k top / 20k bottom)
#define CURRENT_SENSE_ENABLE 1        // set to 0 to bypass current sensing
#define CURRENT_SENSE_DECIMATE 10U     // sample current every N control ticks (100 Hz / 5 = 20 Hz)
#define CURR_ADC_POLL_TIMEOUT_MS 1U   // ADC poll timeout per conversion (ms)
#define ADC_VREF_VOLTS        3.3f
#define ADC_MAX_COUNTS        4095.0f
#define ADC_MID_COUNTS        2048U
#define CURR_DIVIDER_RATIO    0.667f   // Vadc = 0.667 * Vsense (5 V -> ~3.3 V)
#define CURR_ZERO_VOLTS       2.5f     // sensor Vout at 0 A (before divider)
#define CURR_SENS_VOLTS_PER_A 0.040f   // 40 mV/A @ 5 V supply
#define CURR_ZERO_SAMPLES     64U      // samples to average for zero offset
#define CURR_AVG_SAMPLES      8U      // oversample to reduce noise without analog RC
#define CURR_ZERO_VALID_WINDOW_COUNTS 2000U  // accept wider zero range; reject only near rails
#define CURR_ZERO_TRACK_ALPHA 0.0f     // temporarily disable zero tracking during current-sensor calibration
#define CURR_ZERO_TRACK_MAX_DELTA_COUNTS 20U   // track only when delta is tiny (~<0.6 A)
#define CURR_ZERO_TRACK_CURRENT_MA 200        // only track when measured current is near zero
#define LEFT_CURR_POLARITY    1        // set -1 to flip left current sign
#define RIGHT_CURR_POLARITY  -1        // flipped for current-sensor bring-up after rewiring/sensor swap
#define CURR_LPF_ALPHA        0.1f     // low-pass filter alpha (0..1), higher = less smoothing

// Duty/rpm ramping (units are 0..1 duty fraction per second)
#define DUTY_RAMP_RATE_PER_SEC 1.0f    // smoother duty slew to reduce jerk

// Encoder polarity (set to -1 to flip RPM sign for that wheel)
#define LEFT_ENCODER_POLARITY  1
#define RIGHT_ENCODER_POLARITY -1

// Speed PID gains (per wheel). Output is duty 0..1 (clamped to 0..0.3)
#define SPEED_PID_KP_L     0.015f
#define SPEED_PID_KI_L     0.005f
#define SPEED_PID_KD_L     0.0f
#define SPEED_PID_KP_R     0.015f
#define SPEED_PID_KI_R     0.005f
#define SPEED_PID_KD_R     0.0f
#define SPEED_PID_OUT_MIN -0.90f
#define SPEED_PID_OUT_MAX  0.90f
#define SPEED_PID_I_MIN   -2.0f
#define SPEED_PID_I_MAX    2.0f
#define SPEED_PID_DEADBAND_RPM 0.05f   // suppress low-speed dithering

// Speed feed-forward (duty bias) to reduce steady-state error without high Kp
#define SPEED_FF_ENABLE          1
#define SPEED_FF_K_DUTY_PER_RPM  0.003f  // reduced feed-forward for smoother starts
#define SPEED_FF_STATIC_DUTY     0.015f  // reduced stiction kick to avoid snap
#define SPEED_FF_STATIC_RPM      2.0f    // static bias starts blending in above this RPM
#define SPEED_FF_STATIC_RPM_FULL 8.0f    // static bias reaches full strength at this RPM

// Test setpoints (RPM) and toggle interval
#define SPEED_TEST_RPM_LOW   2.0f
#define SPEED_TEST_RPM_HIGH  4.0f
#define SPEED_TEST_TOGGLE_MS 5000U

// PID tuning sweep (percent of PID_TUNING_RPM_MAX)
#define PID_TUNING_ENABLE    0
#define PID_TUNING_RPM_MAX   50.0f
#define PID_TUNING_LOW_FRAC  0.20f
#define PID_TUNING_HIGH_FRAC 0.50f
#define PID_TUNING_TOGGLE_MS 3000U

// Separate command and duty slew limits. Keep duty ramping to soften wheel
// actuation, but disable v/w command ramping because diff_drive_controller
// already applies acceleration limits and double-smoothing makes reversals lag.
#define CMD_RAMP_ENABLE       0        // set to 1 to slew v/w commands in firmware
#define DUTY_RAMP_ENABLE      1        // set to 0 to apply PID output directly
#define V_CMD_RAMP_RATE_MPS   0.8f    // linear ramp for firmware v command if enabled
#define W_CMD_RAMP_RATE_RAD   0.8f    // angular ramp for firmware w command if enabled
#define MOTOR_DUTY_MAX        0.70f    // absolute duty limit for scaling/clamp

// Open-loop motor test (bypass PID/control loop; fixed duty regardless of wheel_cmd)
#define MOTOR_OPEN_LOOP_TEST        0
#define MOTOR_OPEN_LOOP_DUTY_LEFT   0.0f
#define MOTOR_OPEN_LOOP_DUTY_RIGHT  0.20f

// Differential drive test cases (v, omega) toggled every 5s
#define DIFF_TEST_V1_MPS   0.20f
#define DIFF_TEST_W1_RPS   0.0f
#define DIFF_TEST_V2_MPS   0.15f
#define DIFF_TEST_W2_RPS   0.30f
#define DIFF_TEST_V3_MPS   0.0f
#define DIFF_TEST_W3_RPS   0.50f

// RPM filtering before PID (simple exponential filter)
#define RPM_LPF_ALPHA 0.05f   // more smoothing to reduce velocity jitter
#define RPM_SPIKE_LIMIT_RPM 500.0f  // reject raw RPM magnitudes beyond this (likely noise/wrap)
#define CMD_STOP_EPS_MPS 0.01f        // treat linear cmd below this as zero (m/s)
#define CMD_STOP_EPS_RPS 0.01f        // treat angular cmd below this as zero (rad/s)
#define WHEEL_CMD_TIMEOUT_MS 500U    // zero wheel commands if no wheel_cmd within this timeout
#define WHEEL_CMD_L_POLARITY 1.0f    // flip left wheel command sign if needed
#define WHEEL_CMD_R_POLARITY 1.0f    // flip right wheel command sign if needed

// Launch traction guard: soften initial command to avoid wheel slip on startup.
#define LAUNCH_GUARD_ENABLE 0
#define LAUNCH_GUARD_MS 1200U
#define LAUNCH_MIN_SCALE 0.85f
#define LAUNCH_MAX_V_MPS 0.06f
#define LAUNCH_MAX_W_RPS 0.25f

// Fault thresholds
#define FAULT_OC_THRESH_MA        1500   // restored overcurrent threshold after current-sensor calibration
#define FAULT_OC_DWELL_MS         50     // overcurrent dwell to trip (ms)
#define FAULT_STALL_DUTY_MIN      0.08f  // duty >= 8% considered driving
#define FAULT_STALL_RPM_MAX       0.5f   // RPM below this while driving counts as stall
#define FAULT_STALL_DWELL_MS      500    // stall dwell (ms)
#define FAULT_ENC_TIMEOUT_RPM_MIN 0.5f   // commanded RPM must exceed this to check timeout
#define FAULT_ENC_TIMEOUT_MS      1000    // encoder timeout dwell (ms)
#define FAULT_ADC_STUCK_ENABLED   1      // set to 1 to re-enable ADC stuck/rail fault (disabled for testing)
#define FAULT_ADC_STUCK_SAMPLES   30     // consecutive identical/rail samples to declare ADC stuck
#define FAULT_ADC_RAIL_THRESH     5      // counts from rail to consider as rail (0 or max)
#define FAULT_ADC_STUCK_MIN_DUTY  2.0f   // only check ADC stuck when |duty| >= this percent on either wheel

#include "tuning_profiles.h"

// Apply profile overrides (A/B testing).
#if (TUNING_PROFILE == TUNING_PROFILE_NO_GUARD)
#undef LAUNCH_GUARD_ENABLE
#define LAUNCH_GUARD_ENABLE 0

#elif (TUNING_PROFILE == TUNING_PROFILE_NO_STATIC_FF)
#undef SPEED_FF_STATIC_DUTY
#define SPEED_FF_STATIC_DUTY 0.0f

#elif (TUNING_PROFILE == TUNING_PROFILE_NO_FF)
#undef SPEED_FF_ENABLE
#define SPEED_FF_ENABLE 0
#undef SPEED_FF_STATIC_DUTY
#define SPEED_FF_STATIC_DUTY 0.0f
#undef SPEED_FF_K_DUTY_PER_RPM
#define SPEED_FF_K_DUTY_PER_RPM 0.0f
#endif

#endif // APP_CONFIG_H
