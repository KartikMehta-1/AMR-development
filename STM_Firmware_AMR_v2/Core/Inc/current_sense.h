// Current sensing helper: zero-calibration, averaging, polarity, and smoothing
#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "main.h"
#include "app_config.h"

typedef struct {
  ADC_HandleTypeDef *hadc;
  uint16_t zero_left;
  uint16_t zero_right;
  float filt_left_mA;
  float filt_right_mA;
  uint8_t filt_init;
} CurrentSense;

void CurrentSense_Init(CurrentSense *cs, ADC_HandleTypeDef *hadc);
void CurrentSense_Calibrate(CurrentSense *cs);
uint8_t CurrentSense_ReadFiltered(CurrentSense *cs,
                                  int32_t *left_mA, int32_t *right_mA,
                                  uint16_t *left_counts, uint16_t *right_counts);

#endif // CURRENT_SENSE_H
