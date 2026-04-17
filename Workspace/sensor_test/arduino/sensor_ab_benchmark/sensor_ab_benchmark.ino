/*
  Sensor A/B benchmark for Arduino UNO + L298N + DC motor.

  Purpose:
  - Run repeatable PWM duty steps.
  - Sample ACS758 and ACS70331 outputs.
  - Stream CSV for side-by-side analysis with PicoScope captures.

  Serial: 230400 baud
*/

#include <Arduino.h>

// Some toolchains/cores may not define A0/A1 macros at parse time.
// Fallback values match Arduino UNO analog pin mapping.
#ifndef A0
#define A0 14
#endif
#ifndef A1
#define A1 15
#endif

// ------------------------- Pin mapping -------------------------
static const uint8_t PIN_PWM = 5;        // L298N ENA
static const uint8_t PIN_IN1 = 8;        // L298N IN1
static const uint8_t PIN_IN2 = 9;        // L298N IN2
static const uint8_t PIN_TRIG = 7;       // Scope trigger marker

static const uint8_t PIN_ACS758 = A0;    // ACS758 analog output
static const uint8_t PIN_ACS70331 = A1;  // ACS70331 analog output

// ------------------------- ADC conversion ----------------------
static const float ADC_REF_VOLTS = 5.0f;     // UNO default analog reference
static const float ADC_MAX_COUNTS = 1023.0f; // 10-bit ADC

// ------------------------- Sensor scaling ----------------------
// Adjust as needed for your exact modules/variants.
static const float ACS758_SENS_V_PER_A = 0.040f;   // Typical ACS758-050B
static const float ACS70331_SENS_V_PER_A = 0.400f; // Example: 400 mV/A variant
static const float ACS758_POLARITY = 1.0f;         // flip sign if needed
static const float ACS70331_POLARITY = 1.0f;       // flip sign if needed

// ------------------------- Timing ------------------------------
static const uint16_t ADC_AVG_SAMPLES = 8;
static const uint16_t ZERO_CAL_SAMPLES = 400;
static const uint16_t SAMPLE_PERIOD_MS = 5;
static const uint16_t PRINT_PERIOD_MS = 10;

struct Phase {
  const char *name;
  uint8_t duty_pct;
  bool reverse;
  uint16_t duration_ms;
};

// Repeatable profile for A/B sensor comparison.
static const Phase kPhases[] = {
  {"idle_start", 0, false, 2500},
  {"fwd_20", 20, false, 2500},
  {"fwd_40", 40, false, 2500},
  {"fwd_60", 60, false, 2500},
  {"fwd_80", 80, false, 2500},
  {"fwd_60_d", 60, false, 2500},
  {"fwd_40_d", 40, false, 2500},
  {"fwd_20_d", 20, false, 2500},
  {"idle_mid", 0, false, 2000},
  {"rev_20", 20, true, 2500},
  {"rev_40", 40, true, 2500},
  {"rev_60", 60, true, 2500},
  {"rev_80", 80, true, 2500},
  {"rev_60_d", 60, true, 2500},
  {"rev_40_d", 40, true, 2500},
  {"rev_20_d", 20, true, 2500},
  {"idle_end", 0, false, 3500},
};
static const uint8_t kPhaseCount = sizeof(kPhases) / sizeof(kPhases[0]);

static uint8_t g_phase_idx = 0;
static uint32_t g_phase_start_ms = 0;
static uint32_t g_next_sample_ms = 0;
static uint32_t g_next_print_ms = 0;

static uint16_t g_zero_acs758 = 0;
static uint16_t g_zero_acs70331 = 0;

static uint16_t g_adc_acs758 = 0;
static uint16_t g_adc_acs70331 = 0;
static float g_v_acs758 = 0.0f;
static float g_v_acs70331 = 0.0f;
static float g_i_acs758 = 0.0f;
static float g_i_acs70331 = 0.0f;

static uint8_t g_trig_state = LOW;

static uint16_t readAveragedADC(uint8_t pin, uint16_t samples) {
  uint32_t acc = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    acc += (uint16_t)analogRead(pin);
  }
  return (uint16_t)(acc / samples);
}

static float adcCountsToVolts(uint16_t counts) {
  return ((float)counts * ADC_REF_VOLTS) / ADC_MAX_COUNTS;
}

static void setMotor(uint8_t duty_pct, bool reverse) {
  if (duty_pct > 100U) duty_pct = 100U;

  if (duty_pct == 0U) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, 0);
    return;
  }

  if (reverse) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }

  uint8_t pwm = (uint8_t)((duty_pct * 255U) / 100U);
  analogWrite(PIN_PWM, pwm);
}

static void applyPhase(uint8_t idx) {
  const Phase &p = kPhases[idx];
  setMotor(p.duty_pct, p.reverse);

  // Toggle marker pin on each phase transition for scope alignment.
  g_trig_state = (g_trig_state == LOW) ? HIGH : LOW;
  digitalWrite(PIN_TRIG, g_trig_state);
}

static void calibrateZeroOffsets(void) {
  setMotor(0, false);
  delay(500);

  uint32_t acc_758 = 0;
  uint32_t acc_70331 = 0;

  for (uint16_t i = 0; i < ZERO_CAL_SAMPLES; ++i) {
    acc_758 += analogRead(PIN_ACS758);
    acc_70331 += analogRead(PIN_ACS70331);
    delay(2);
  }

  g_zero_acs758 = (uint16_t)(acc_758 / ZERO_CAL_SAMPLES);
  g_zero_acs70331 = (uint16_t)(acc_70331 / ZERO_CAL_SAMPLES);
}

static void sampleSensors(void) {
  g_adc_acs758 = readAveragedADC(PIN_ACS758, ADC_AVG_SAMPLES);
  g_adc_acs70331 = readAveragedADC(PIN_ACS70331, ADC_AVG_SAMPLES);

  g_v_acs758 = adcCountsToVolts(g_adc_acs758);
  g_v_acs70331 = adcCountsToVolts(g_adc_acs70331);

  const float zero_v_758 = adcCountsToVolts(g_zero_acs758);
  const float zero_v_70331 = adcCountsToVolts(g_zero_acs70331);

  g_i_acs758 = ((g_v_acs758 - zero_v_758) / ACS758_SENS_V_PER_A) * ACS758_POLARITY;
  g_i_acs70331 = ((g_v_acs70331 - zero_v_70331) / ACS70331_SENS_V_PER_A) * ACS70331_POLARITY;
}

static void printHeader(void) {
  Serial.println(F("# Sensor benchmark start"));
  Serial.print(F("#INFO: zero_acs758="));
  Serial.print(g_zero_acs758);
  Serial.print(F(",zero_acs70331="));
  Serial.println(g_zero_acs70331);
  Serial.println(F("#HEADER: t_ms,phase_idx,phase,duty_pct,direction,trig,adc_acs758,adc_acs70331,v_acs758,v_acs70331,i_acs758_A,i_acs70331_A,zero_acs758,zero_acs70331"));
}

static void printSample(uint32_t now_ms) {
  const Phase &p = kPhases[g_phase_idx];

  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(g_phase_idx);
  Serial.print(',');
  Serial.print(p.name);
  Serial.print(',');
  Serial.print(p.duty_pct);
  Serial.print(',');

  if (p.duty_pct == 0) {
    Serial.print(F("STOP"));
  } else {
    Serial.print(p.reverse ? F("REV") : F("FWD"));
  }

  Serial.print(',');
  Serial.print(g_trig_state == HIGH ? 1 : 0);
  Serial.print(',');
  Serial.print(g_adc_acs758);
  Serial.print(',');
  Serial.print(g_adc_acs70331);
  Serial.print(',');
  Serial.print(g_v_acs758, 5);
  Serial.print(',');
  Serial.print(g_v_acs70331, 5);
  Serial.print(',');
  Serial.print(g_i_acs758, 5);
  Serial.print(',');
  Serial.print(g_i_acs70331, 5);
  Serial.print(',');
  Serial.print(g_zero_acs758);
  Serial.print(',');
  Serial.println(g_zero_acs70331);
}

void setup() {
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);

  digitalWrite(PIN_TRIG, LOW);
  setMotor(0, false);

  Serial.begin(230400);
  delay(800);

  calibrateZeroOffsets();
  printHeader();

  g_phase_idx = 0;
  g_phase_start_ms = millis();
  g_next_sample_ms = g_phase_start_ms;
  g_next_print_ms = g_phase_start_ms;
  applyPhase(g_phase_idx);
}

void loop() {
  uint32_t now = millis();

  // Advance phase when its duration completes.
  if ((uint32_t)(now - g_phase_start_ms) >= kPhases[g_phase_idx].duration_ms) {
    g_phase_idx++;
    if (g_phase_idx >= kPhaseCount) {
      g_phase_idx = 0;
    }
    g_phase_start_ms = now;
    applyPhase(g_phase_idx);
  }

  // Fixed-rate sampling.
  if ((int32_t)(now - g_next_sample_ms) >= 0) {
    g_next_sample_ms += SAMPLE_PERIOD_MS;
    sampleSensors();
  }

  // Decimated printing.
  if ((int32_t)(now - g_next_print_ms) >= 0) {
    g_next_print_ms += PRINT_PERIOD_MS;
    printSample(now);
  }
}
