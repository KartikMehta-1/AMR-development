#include "sensing.h"
#include <math.h>

void Sensing_Init(Sensing *s, EncoderChannel *enc_l, EncoderChannel *enc_r, CurrentSense *curr, float rpm_lpf_alpha)
{
  s->enc_l = enc_l;
  s->enc_r = enc_r;
  s->curr = curr;
  s->rpm_l_filt = 0.0f;
  s->rpm_r_filt = 0.0f;
  s->rpm_lpf_alpha = rpm_lpf_alpha;
  s->rpm_filt_init = 0;
  s->curr_decim_count = 0U;
  s->curr_l_mA = 0;
  s->curr_r_mA = 0;
  s->adc_l_counts = 0U;
  s->adc_r_counts = 0U;
  s->zero_l_counts = 0U;
  s->zero_r_counts = 0U;
  s->curr_valid = 0U;
}

void Sensing_Step(Sensing *s, float dt_s, SensingData *out)
{
  if (dt_s <= 0.0f) {
    return;
  }

  Encoder_Update(s->enc_l, dt_s);
  Encoder_Update(s->enc_r, dt_s);

  out->cnt_l = Encoder_GetRawCount(s->enc_l);
  out->cnt_r = Encoder_GetRawCount(s->enc_r);

  float rpm_l_raw = Encoder_GetRPM(s->enc_l);
  float rpm_r_raw = Encoder_GetRPM(s->enc_r);

  // Reject implausible raw RPM spikes (wraps or scheduling stalls)
  if (fabsf(rpm_l_raw) > RPM_SPIKE_LIMIT_RPM) {
    rpm_l_raw = s->rpm_filt_init ? s->rpm_l_filt : 0.0f;
  }
  if (fabsf(rpm_r_raw) > RPM_SPIKE_LIMIT_RPM) {
    rpm_r_raw = s->rpm_filt_init ? s->rpm_r_filt : 0.0f;
  }
  out->rpm_l_raw = rpm_l_raw;
  out->rpm_r_raw = rpm_r_raw;

  if (!s->rpm_filt_init) {
    s->rpm_l_filt = rpm_l_raw;
    s->rpm_r_filt = rpm_r_raw;
    s->rpm_filt_init = 1;
  } else {
    s->rpm_l_filt += s->rpm_lpf_alpha * (rpm_l_raw - s->rpm_l_filt);
    s->rpm_r_filt += s->rpm_lpf_alpha * (rpm_r_raw - s->rpm_r_filt);
  }

  out->rpm_l = s->rpm_l_filt;
  out->rpm_r = s->rpm_r_filt;

#if CURRENT_SENSE_ENABLE
  if (s->curr_decim_count == 0U) {
    s->curr_valid = CurrentSense_ReadFiltered(s->curr, &s->curr_l_mA, &s->curr_r_mA,
                                              &s->adc_l_counts, &s->adc_r_counts);
    if (!s->curr_valid) {
      s->curr_l_mA = 0;
      s->curr_r_mA = 0;
      s->adc_l_counts = 0U;
      s->adc_r_counts = 0U;
    }
    s->zero_l_counts = s->curr->zero_left;
    s->zero_r_counts = s->curr->zero_right;
  }
  s->curr_decim_count++;
  if (s->curr_decim_count >= CURRENT_SENSE_DECIMATE) {
    s->curr_decim_count = 0U;
  }
  out->curr_l_mA = s->curr_l_mA;
  out->curr_r_mA = s->curr_r_mA;
  out->adc_l_counts = s->adc_l_counts;
  out->adc_r_counts = s->adc_r_counts;
  out->zero_l_counts = s->zero_l_counts;
  out->zero_r_counts = s->zero_r_counts;
  out->curr_valid = s->curr_valid;
#else
  out->curr_l_mA = 0;
  out->curr_r_mA = 0;
  out->adc_l_counts = 0;
  out->adc_r_counts = 0;
  out->zero_l_counts = 0;
  out->zero_r_counts = 0;
  out->curr_valid = 0;
#endif
}
