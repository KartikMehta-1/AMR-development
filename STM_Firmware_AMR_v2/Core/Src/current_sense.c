#include "current_sense.h"

#include <stdint.h>

// Convert raw ADC counts to current in amperes for ACS758 with 10k/20k divider
static float Current_FromRaw(uint16_t adc_counts, uint16_t zero_counts)
{
  float vadc = ((float)adc_counts * ADC_VREF_VOLTS) / ADC_MAX_COUNTS;
  float vadc_zero = ((float)zero_counts * ADC_VREF_VOLTS) / ADC_MAX_COUNTS;
  float vsense = (vadc - vadc_zero) / CURR_DIVIDER_RATIO;
  return vsense / CURR_SENS_VOLTS_PER_A;
}

// Read both current channels (rank1: PB0 left, rank2: PC1 right); returns 1 on success
static uint8_t ReadAveragedCounts(CurrentSense *cs, uint16_t* left_counts, uint16_t* right_counts)
{
  uint32_t acc_l = 0, acc_r = 0, good = 0;

  for (uint32_t i = 0; i < CURR_AVG_SAMPLES; ++i) {
    if (HAL_ADC_Start(cs->hadc) != HAL_OK) {
      HAL_ADC_Stop(cs->hadc);
      continue;
    }

    if (HAL_ADC_PollForConversion(cs->hadc, 5) != HAL_OK) {
      HAL_ADC_Stop(cs->hadc);
      continue;
    }
    uint16_t l = (uint16_t)HAL_ADC_GetValue(cs->hadc);

    if (HAL_ADC_PollForConversion(cs->hadc, 5) != HAL_OK) {
      HAL_ADC_Stop(cs->hadc);
      continue;
    }
    uint16_t r = (uint16_t)HAL_ADC_GetValue(cs->hadc);

    HAL_ADC_Stop(cs->hadc);
    acc_l += l;
    acc_r += r;
    good++;
  }

  if (good == 0) {
    return 0;
  }

  *left_counts = (uint16_t)(acc_l / good);
  *right_counts = (uint16_t)(acc_r / good);
  return 1;
}

void CurrentSense_Init(CurrentSense *cs, ADC_HandleTypeDef *hadc)
{
  cs->hadc = hadc;
  cs->zero_left = ADC_MID_COUNTS;
  cs->zero_right = ADC_MID_COUNTS;
  cs->filt_left_mA = 0.0f;
  cs->filt_right_mA = 0.0f;
  cs->filt_init = 0;
}

void CurrentSense_Calibrate(CurrentSense *cs)
{
  uint32_t acc_l = 0, acc_r = 0, good = 0;
  for (uint32_t i = 0; i < CURR_ZERO_SAMPLES; ++i) {
    uint16_t l = 0, r = 0;
    if (ReadAveragedCounts(cs, &l, &r)) {
      acc_l += l;
      acc_r += r;
      good++;
    }
    HAL_Delay(1);
  }

  if (good == 0) {
    cs->zero_left = ADC_MID_COUNTS;
    cs->zero_right = ADC_MID_COUNTS;
    return;
  }

  cs->zero_left = (uint16_t)(acc_l / good);
  cs->zero_right = (uint16_t)(acc_r / good);
}

uint8_t CurrentSense_ReadFiltered(CurrentSense *cs, int32_t *left_mA, int32_t *right_mA)
{
  uint16_t adc_left = 0, adc_right = 0;
  if (!ReadAveragedCounts(cs, &adc_left, &adc_right)) {
    return 0;
  }

  int32_t curr_l = (int32_t)(Current_FromRaw(adc_left, cs->zero_left) * 1000.0f) * LEFT_CURR_POLARITY;
  int32_t curr_r = (int32_t)(Current_FromRaw(adc_right, cs->zero_right) * 1000.0f) * RIGHT_CURR_POLARITY;

  if (!cs->filt_init) {
    cs->filt_left_mA = (float)curr_l;
    cs->filt_right_mA = (float)curr_r;
    cs->filt_init = 1;
  } else {
    cs->filt_left_mA += CURR_LPF_ALPHA * ((float)curr_l - cs->filt_left_mA);
    cs->filt_right_mA += CURR_LPF_ALPHA * ((float)curr_r - cs->filt_right_mA);
  }

  *left_mA = (int32_t)cs->filt_left_mA;
  *right_mA = (int32_t)cs->filt_right_mA;
  return 1;
}
