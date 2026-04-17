// Control state machine (INIT/IDLE/ENABLED/FAULT) with fault mask/latching
#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  CTRL_STATE_INIT = 0,
  CTRL_STATE_IDLE,
  CTRL_STATE_ENABLED,
  CTRL_STATE_FAULT
} ControlState;

// Fault bits
#define CTRL_FAULT_ESTOP            (1U << 0)
#define CTRL_FAULT_OC_LEFT          (1U << 1)
#define CTRL_FAULT_OC_RIGHT         (1U << 2)
#define CTRL_FAULT_STALL_LEFT       (1U << 3)
#define CTRL_FAULT_STALL_RIGHT      (1U << 4)
#define CTRL_FAULT_ENC_TIMEOUT_LEFT (1U << 5)
#define CTRL_FAULT_ENC_TIMEOUT_RIGHT (1U << 6)
#define CTRL_FAULT_ADC_STUCK        (1U << 7)
#define CTRL_FAULT_GENERIC          (1U << 15)

typedef struct {
  bool estop;          // hardware or software estop active
  bool enable_cmd;     // request to enable
  bool disable_cmd;    // request to disable
  bool clear_cmd;      // request to clear latched faults
  uint32_t fault_bits; // specific fault bits to latch
} ControlInputs;

typedef struct {
  ControlState state;
  uint32_t fault_mask;
} ControlStateMgr;

void ControlState_Init(ControlStateMgr *m);
void ControlState_Update(ControlStateMgr *m, const ControlInputs *in, uint32_t now_ms);
static inline bool ControlState_IsEnabled(const ControlStateMgr *m) { return m->state == CTRL_STATE_ENABLED; }
static inline ControlState ControlState_GetState(const ControlStateMgr *m) { return m->state; }
static inline uint32_t ControlState_GetFaultMask(const ControlStateMgr *m) { return m->fault_mask; }

#endif // CONTROL_STATE_H
