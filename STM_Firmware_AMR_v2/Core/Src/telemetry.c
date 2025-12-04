#include "telemetry.h"

#include <stdio.h>
#include <string.h>

void Telemetry_SendHeader(UART_HandleTypeDef *huart)
{
  const char *hdr = "#HEADER: t_ms,l_cnt,r_cnt,l_rpm_x10,r_rpm_x10,l_duty_pct,r_duty_pct,l_adc,r_adc,l_zero,r_zero,l_mA,r_mA\r\n";
  HAL_UART_Transmit(huart, (uint8_t*)hdr, (uint16_t)strlen(hdr), HAL_MAX_DELAY);
}

void Telemetry_SendFrame(UART_HandleTypeDef *huart, const TelemetryFrame *f)
{
  char buf[160];
  int len = snprintf(buf, sizeof(buf),
                     "%lu,%lu,%lu,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%ld,%ld\r\n",
                     (unsigned long)f->t_ms,
                     (unsigned long)f->cnt_l,
                     (unsigned long)f->cnt_r,
                     (long)f->rpm_l_x10,
                     (long)f->rpm_r_x10,
                     (long)f->duty_l_pct,
                     (long)f->duty_r_pct,
                     (unsigned)f->adc_l_counts,
                     (unsigned)f->adc_r_counts,
                     (unsigned)f->zero_l_counts,
                     (unsigned)f->zero_r_counts,
                     (long)f->curr_l_mA,
                     (long)f->curr_r_mA);
  if (len > 0) {
    HAL_UART_Transmit(huart, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
  }
}
