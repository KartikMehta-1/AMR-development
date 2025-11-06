// Safety Manager: fault mask and gating API (header stub)
#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAFETY_OK = 0,
    SAFETY_FAULT = 1,
} SafetyStatus;

// Fault mask bit assignments (aligns with docs/firmware_motor_control.md)
#define SAFETY_FAULT_ESTOP_ACTIVE         (1u << 0)
#define SAFETY_FAULT_OVERCURRENT_LEFT     (1u << 1)
#define SAFETY_FAULT_OVERCURRENT_RIGHT    (1u << 2)
#define SAFETY_FAULT_ENCODER_TIMEOUT_LEFT (1u << 3)
#define SAFETY_FAULT_ENCODER_TIMEOUT_RIGHT (1u << 4)
#define SAFETY_FAULT_ADC_RANGE            (1u << 5)
#define SAFETY_FAULT_SUPPLY_UNDERVOLT     (1u << 6)
#define SAFETY_FAULT_SUPPLY_OVERVOLT      (1u << 7)
#define SAFETY_FAULT_DRIVER               (1u << 8)

void Safety_Init(void);
void Safety_Update(uint32_t now_ms);

void Safety_SetEStop(uint8_t active_low_true);
void Safety_ReportOvercurrent(uint8_t left_active, uint8_t right_active);
void Safety_ReportEncoderActivity(uint8_t left_active, uint8_t right_active);
void Safety_ReportAdcHealthy(uint8_t healthy);

void Safety_LatchFault(uint32_t mask);
void Safety_RequestClear(void);

uint32_t    Safety_GetFaultMask(void);
SafetyStatus Safety_GetStatus(void);
uint8_t     Safety_IsSafe(void);

#ifdef __cplusplus
}
#endif

#endif // SAFETY_H

