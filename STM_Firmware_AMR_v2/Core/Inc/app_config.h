// Application-wide configuration and tuning constants
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// Direction polarity: set to 1 for forward, 0 to invert that wheel
#define LEFT_DIR_POLARITY    1
#define RIGHT_DIR_POLARITY   0

// Encoder/rate reporting configuration
#define ENCODER_COUNTS_PER_REV  2400U   // 600 PPR A-4 quadrature
#define CONTROL_LOOP_HZ         100U    // fixed-rate control loop (Hz)
#define CONTROL_LOOP_DT_MS      (1000U / CONTROL_LOOP_HZ)
#define CONTROL_LOOP_DT_S       (1.0f / (float)CONTROL_LOOP_HZ)
#define TELEMETRY_DECIMATION    10U     // send telemetry every N control ticks (100 Hz / 10 = 10 Hz)
#define SAMPLE_INTERVAL_MS      100U    // legacy/telemetry period reference (ms)
#define ROS_PUB_PERIOD_MS       200U    // publish period for ROS topics (ms)
#define ROS_EXEC_DELAY_MS       10U     // executor loop delay to service subscriptions (ms)
#define ROS_PUB_PID_DIAG        0       // set to 1 to publish PID P/I/D/error topics over ROS
#define TRACK_WIDTH_M           0.386f  // wheel-to-wheel track width (meters)
#define WHEEL_RADIUS_M          0.0615f // wheel radius (meters)

// Current sensor scaling (ACS758 50B @5 V; divider 10k top / 20k bottom)
#define CURRENT_SENSE_ENABLE 0        // set to 0 to bypass current sensing
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

// Duty/rpm ramping (units are 0..1 duty fraction per second)
#define DUTY_RAMP_RATE_PER_SEC 2.0f    // slew limit for motor duty (e.g., 20% per second)

// Encoder polarity (set to -1 to flip RPM sign for that wheel)
#define LEFT_ENCODER_POLARITY  1
#define RIGHT_ENCODER_POLARITY -1

// Speed PID gains (per wheel). Output is duty 0..1 (clamped to 0..0.3)
#define SPEED_PID_KP_L     0.5f
#define SPEED_PID_KI_L     0.002f
#define SPEED_PID_KD_L     0.0f
#define SPEED_PID_KP_R     0.5f
#define SPEED_PID_KI_R     0.002f
#define SPEED_PID_KD_R     0.0f
#define SPEED_PID_OUT_MIN -0.90f
#define SPEED_PID_OUT_MAX  0.90f
#define SPEED_PID_I_MIN   -2.0f
#define SPEED_PID_I_MAX    2.0f
#define SPEED_PID_DEADBAND_RPM 0.20f   // do not integrate when error magnitude is below this

// Test setpoints (RPM) and toggle interval
#define SPEED_TEST_RPM_LOW   2.0f
#define SPEED_TEST_RPM_HIGH  4.0f
#define SPEED_TEST_TOGGLE_MS 5000U

// Ramp rates for command slewing (applied to v, w) and duty limit
#define RAMPING_ENABLE        1        // set to 0 to disable v/w and duty ramping
#define V_CMD_RAMP_RATE_MPS   2.0f    // max change in linear velocity per second
#define W_CMD_RAMP_RATE_RAD   2.0f    // max change in angular velocity per second
#define MOTOR_DUTY_MAX        0.70f    // absolute duty limit for scaling/clamp

// Open-loop motor test (bypass PID/control loop; fixed duty regardless of cmd_vel)
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
#define RPM_LPF_ALPHA 0.2f   // higher = less smoothing (faster response)
#define RPM_SPIKE_LIMIT_RPM 500.0f  // reject raw RPM magnitudes beyond this (likely noise/wrap)
#define CMD_TIMEOUT_MS 500U          // zero commands if no cmd_vel within this timeout
#define CMD_STOP_EPS_MPS 0.01f        // treat linear cmd below this as zero (m/s)
#define CMD_STOP_EPS_RPS 0.01f        // treat angular cmd below this as zero (rad/s)
#define CMD_V_POLARITY 1.0f          // flip linear velocity sign if forward/back is inverted
#define CMD_W_POLARITY 1.0f          // flip angular velocity sign if turning direction is inverted

// Fault thresholds
#define FAULT_OC_THRESH_MA        1500   // overcurrent threshold (mA)
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

#endif // APP_CONFIG_H
