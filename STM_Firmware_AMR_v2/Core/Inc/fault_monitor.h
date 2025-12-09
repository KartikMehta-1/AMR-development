// Fault detection helpers
#ifndef FAULT_MONITOR_H
#define FAULT_MONITOR_H

#include <stdint.h>
#include "sensing.h"
#include "control_state.h"

typedef struct {
  uint32_t oc_l_accum_ms;
  uint32_t oc_r_accum_ms;
  uint32_t stall_l_accum_ms;
  uint32_t stall_r_accum_ms;
  uint32_t enc_l_idle_ms;
  uint32_t enc_r_idle_ms;
  uint16_t adc_l_last;
  uint16_t adc_r_last;
  uint32_t adc_stuck_count;
} FaultMonitor;

void FaultMonitor_Init(FaultMonitor *fm);
uint32_t FaultMonitor_Update(FaultMonitor *fm,
                             const SensingData *sense,
                             float rpm_target_l,
                             float rpm_target_r,
                             float duty_l_pct,
                             float duty_r_pct,
                             uint32_t dt_ms);

#endif // FAULT_MONITOR_H
