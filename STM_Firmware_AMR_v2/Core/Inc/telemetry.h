// Lightweight telemetry formatter/sender
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "main.h"
#include <stdint.h>

typedef struct {
  uint32_t t_ms;
  uint32_t cnt_l;
  uint32_t cnt_r;
  int32_t rpm_l_x10;
  int32_t rpm_r_x10;
  int32_t rpm_l_tgt_x10;
  int32_t rpm_r_tgt_x10;
  int32_t duty_l_pct;
  int32_t duty_r_pct;
  uint16_t adc_l_counts;
  uint16_t adc_r_counts;
  uint16_t zero_l_counts;
  uint16_t zero_r_counts;
  int32_t curr_l_mA;
  int32_t curr_r_mA;
  uint32_t state;
  uint32_t fault_mask;
} TelemetryFrame;

void Telemetry_SendHeader(UART_HandleTypeDef *huart);
void Telemetry_SendFrame(UART_HandleTypeDef *huart, const TelemetryFrame *f);

#endif // TELEMETRY_H
