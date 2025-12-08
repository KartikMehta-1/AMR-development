// Sensing aggregation: encoder RPM (with LPF) and current readings
#ifndef SENSING_H
#define SENSING_H

#include <stdint.h>
#include "encoder.h"
#include "current_sense.h"

typedef struct {
  uint32_t cnt_l;
  uint32_t cnt_r;
  float rpm_l;
  float rpm_r;
  int32_t curr_l_mA;
  int32_t curr_r_mA;
  uint16_t adc_l_counts;
  uint16_t adc_r_counts;
  uint16_t zero_l_counts;
  uint16_t zero_r_counts;
} SensingData;

typedef struct {
  EncoderChannel *enc_l;
  EncoderChannel *enc_r;
  CurrentSense   *curr;
  float rpm_l_filt;
  float rpm_r_filt;
  float rpm_lpf_alpha;
  uint8_t rpm_filt_init;
} Sensing;

void Sensing_Init(Sensing *s, EncoderChannel *enc_l, EncoderChannel *enc_r, CurrentSense *curr, float rpm_lpf_alpha);
void Sensing_Step(Sensing *s, float dt_s, SensingData *out);

#endif // SENSING_H
