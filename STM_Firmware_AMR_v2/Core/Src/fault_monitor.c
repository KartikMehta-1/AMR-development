#include "fault_monitor.h"
#include "app_config.h"
#include <math.h>

void FaultMonitor_Init(FaultMonitor *fm)
{
  fm->oc_l_accum_ms = fm->oc_r_accum_ms = 0;
  fm->stall_l_accum_ms = fm->stall_r_accum_ms = 0;
  fm->enc_l_idle_ms = fm->enc_r_idle_ms = 0;
  fm->adc_l_last = fm->adc_r_last = 0;
  fm->adc_stuck_count = 0;
}

uint32_t FaultMonitor_Update(FaultMonitor *fm,
                             const SensingData *sense,
                             float rpm_target,
                             float duty_l_pct,
                             float duty_r_pct,
                             uint32_t dt_ms)
{
  uint32_t fault_bits = 0U;

  // Overcurrent dwell
  fm->oc_l_accum_ms = (sense->curr_l_mA > FAULT_OC_THRESH_MA) ? (fm->oc_l_accum_ms + dt_ms) : 0U;
  fm->oc_r_accum_ms = (sense->curr_r_mA > FAULT_OC_THRESH_MA) ? (fm->oc_r_accum_ms + dt_ms) : 0U;
  if (fm->oc_l_accum_ms >= FAULT_OC_DWELL_MS) fault_bits |= CTRL_FAULT_OC_LEFT;
  if (fm->oc_r_accum_ms >= FAULT_OC_DWELL_MS) fault_bits |= CTRL_FAULT_OC_RIGHT;

  // Stall detection: duty high, RPM low
  bool driving_l = (duty_l_pct / 100.0f) >= FAULT_STALL_DUTY_MIN;
  bool driving_r = (duty_r_pct / 100.0f) >= FAULT_STALL_DUTY_MIN;
  fm->stall_l_accum_ms = (driving_l && (fabsf(sense->rpm_l) <= FAULT_STALL_RPM_MAX)) ? (fm->stall_l_accum_ms + dt_ms) : 0U;
  fm->stall_r_accum_ms = (driving_r && (fabsf(sense->rpm_r) <= FAULT_STALL_RPM_MAX)) ? (fm->stall_r_accum_ms + dt_ms) : 0U;
  if (fm->stall_l_accum_ms >= FAULT_STALL_DWELL_MS) fault_bits |= CTRL_FAULT_STALL_LEFT;
  if (fm->stall_r_accum_ms >= FAULT_STALL_DWELL_MS) fault_bits |= CTRL_FAULT_STALL_RIGHT;

  // Encoder timeout: commanded rpm above threshold but RPM ~0 for dwell
  float tgt_rpm_abs = fabsf(rpm_target);
  bool cmd_active = tgt_rpm_abs >= FAULT_ENC_TIMEOUT_RPM_MIN;
  fm->enc_l_idle_ms = (cmd_active && fabsf(sense->rpm_l) <= FAULT_ENC_TIMEOUT_RPM_MIN) ? (fm->enc_l_idle_ms + dt_ms) : 0U;
  fm->enc_r_idle_ms = (cmd_active && fabsf(sense->rpm_r) <= FAULT_ENC_TIMEOUT_RPM_MIN) ? (fm->enc_r_idle_ms + dt_ms) : 0U;
  if (fm->enc_l_idle_ms >= FAULT_ENC_TIMEOUT_MS) fault_bits |= CTRL_FAULT_ENC_TIMEOUT_LEFT;
  if (fm->enc_r_idle_ms >= FAULT_ENC_TIMEOUT_MS) fault_bits |= CTRL_FAULT_ENC_TIMEOUT_RIGHT;

  // ADC stuck/rail detection
  bool rail_l = (sense->adc_l_counts <= FAULT_ADC_RAIL_THRESH) || (sense->adc_l_counts >= (uint16_t)(ADC_MAX_COUNTS - FAULT_ADC_RAIL_THRESH));
  bool rail_r = (sense->adc_r_counts <= FAULT_ADC_RAIL_THRESH) || (sense->adc_r_counts >= (uint16_t)(ADC_MAX_COUNTS - FAULT_ADC_RAIL_THRESH));
  bool same_l = (sense->adc_l_counts == fm->adc_l_last);
  bool same_r = (sense->adc_r_counts == fm->adc_r_last);
  if ((rail_l && rail_r) || (same_l && same_r)) {
    if (fm->adc_stuck_count < 0xFFFFFFFFU) {
      fm->adc_stuck_count++;
    }
  } else {
    fm->adc_stuck_count = 0U;
  }
  fm->adc_l_last = sense->adc_l_counts;
  fm->adc_r_last = sense->adc_r_counts;
  if (fm->adc_stuck_count >= FAULT_ADC_STUCK_SAMPLES) {
    fault_bits |= CTRL_FAULT_ADC_STUCK;
  }

  return fault_bits;
}
