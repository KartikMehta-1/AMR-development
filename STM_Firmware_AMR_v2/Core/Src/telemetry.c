#include "telemetry.h"

#include <stdio.h>
#include <string.h>

void Telemetry_SendHeader(UART_HandleTypeDef *huart)
{
  const char *hdr = "#HEADER: t_ms,l_cnt,r_cnt,l_rpm_x10,r_rpm_x10,l_rpm_tgt_x10,r_rpm_tgt_x10,l_duty_pct,r_duty_pct,l_pid_p_pct,r_pid_p_pct,l_pid_i_pct,r_pid_i_pct,l_pid_d_pct,r_pid_d_pct,l_pid_err_x10,r_pid_err_x10,l_adc,r_adc,l_zero,r_zero,l_mA,r_mA,state,fault_mask\r\n";
  HAL_UART_Transmit(huart, (uint8_t*)hdr, (uint16_t)strlen(hdr), HAL_MAX_DELAY);
}

void Telemetry_SendFrame(UART_HandleTypeDef *huart, const TelemetryFrame *f)
{
  char buf[320];
  int len = snprintf(buf, sizeof(buf),
                     "%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%ld,%ld,%lu,%lu\r\n",
                     (unsigned long)f->t_ms,
                     (unsigned long)f->cnt_l,
                     (unsigned long)f->cnt_r,
                     (long)f->rpm_l_x10,
                     (long)f->rpm_r_x10,
                     (long)f->rpm_l_tgt_x10,
                     (long)f->rpm_r_tgt_x10,
                     (long)f->duty_l_pct,
                     (long)f->duty_r_pct,
                     (long)f->pid_p_l_pct,
                     (long)f->pid_p_r_pct,
                     (long)f->pid_i_l_pct,
                     (long)f->pid_i_r_pct,
                     (long)f->pid_d_l_pct,
                     (long)f->pid_d_r_pct,
                     (long)f->pid_err_l_x10,
                     (long)f->pid_err_r_x10,
                     (unsigned)f->adc_l_counts,
                     (unsigned)f->adc_r_counts,
                     (unsigned)f->zero_l_counts,
                     (unsigned)f->zero_r_counts,
                     (long)f->curr_l_mA,
                     (long)f->curr_r_mA,
                     (unsigned long)f->state,
                     (unsigned long)f->fault_mask);
  if (len > 0) {
    HAL_UART_Transmit(huart, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
  }
}
