#include "control_state.h"

void ControlState_Init(ControlStateMgr *m)
{
  m->state = CTRL_STATE_IDLE;
  m->fault_mask = 0U;
}

void ControlState_Update(ControlStateMgr *m, const ControlInputs *in, uint32_t now_ms)
{
  (void)now_ms; // reserved for future dwell timers

  uint32_t bits = in->fault_bits;
  if (in->estop) {
    bits |= CTRL_FAULT_ESTOP;
  }

  if (bits != 0U) {
    m->fault_mask |= bits;
    m->state = CTRL_STATE_FAULT;
    return;
  }

  if (in->clear_cmd && (m->fault_mask != 0U)) {
    m->fault_mask = 0U;
    m->state = CTRL_STATE_IDLE;
  }

  if (in->disable_cmd) {
    m->state = CTRL_STATE_IDLE;
    return;
  }

  if (in->enable_cmd && m->fault_mask == 0U) {
    m->state = CTRL_STATE_ENABLED;
  } else if (m->state == CTRL_STATE_INIT) {
    m->state = CTRL_STATE_IDLE;
  }
}
