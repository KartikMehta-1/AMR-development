# STM32 Firmware - Motor Control Diagrams

This document explains the intended motor control model and how it maps to the STM32 firmware setup. It reflects the **single-loop speed PI** approach (no cascaded current loop). Current sensing is used for telemetry/protection only.

```mermaid
graph TD
  %% High level control architecture
  subgraph Inputs
    CMD[Speed Setpoint]
    ESTOP[Estop GPIO]
  end

  subgraph Sensing
    ENC_L[Encoder Left TIM3]
    ENC_R[Encoder Right TIM2]
    ADC_L[ADC1 CH8 Left Current]
    ADC_R[ADC1 CH11 Right Current]
    VEL[Velocity Estimator]
    CUR_MON[Current Monitor]
  end

  subgraph Control
    MODE[Mode Manager]
    SAFE[Safety Manager]
    W_L[Speed PI/PID Left]
    W_R[Speed PI/PID Right]
    POS[Optional Position PID]
  end

  subgraph Actuation
    PWM1[TIM1 CH1 PWM Left]
    PWM2[TIM1 CH2 PWM Right]
    DIRL[DIR Left GPIO PB4]
    DIRR[DIR Right GPIO PB5]
    MDD[Motor Driver Cytron]
  end

  subgraph Telemetry
    UART[USART2 Telemetry]
  end

  %% Data flow
  CMD --> MODE
  ESTOP --> SAFE
  SAFE --> MODE

  ENC_L --> VEL
  ENC_R --> VEL
  VEL --> W_L
  VEL --> W_R

  ADC_L --> CUR_MON
  ADC_R --> CUR_MON
  CUR_MON --> SAFE

  MODE --> W_L
  MODE --> W_R
  POS --> W_L
  POS --> W_R
  W_L --> PWM1
  W_R --> PWM2
  MODE --> DIRL
  MODE --> DIRR

  PWM1 --> MDD
  PWM2 --> MDD
  DIRL --> MDD
  DIRR --> MDD

  VEL --> UART
  CUR_MON --> UART
  MODE --> UART
```

- Setpoint enters Mode Manager, which gates control based on Safety Manager.
- Encoders feed the velocity estimator; current sensors feed the current monitor (telemetry/protection only).
- Single speed loop per wheel produces duty commands to TIM1 PWM; direction is set via GPIO.
- Telemetry publishes selected signals over USART2.

```mermaid
graph LR
  %% Control tick sequence (single-loop speed)
  TICK_W[Outer Tick 100-200 Hz]
  READ_W[Read encoder counts]
  EST_W[Compute wheel velocity LPF]
  CTRL_W[Speed PI/PID -> duty target]
  APPLY[Update PWM duty with ramp/cap]
  TEL[Telemetry at 50-100 Hz]

  TICK_W --> READ_W
  READ_W --> EST_W
  EST_W --> CTRL_W
  CTRL_W --> APPLY
  APPLY --> TEL
```

- Rates: outer speed loop target 100-200 Hz (currently slower); telemetry decimated to ~50-100 Hz.
- ADC samples left/right motor currents for protection/telemetry; encoder deltas compute velocity for speed loop.
- Speed loop updates duty (with ramp and duty cap at 30%).

```mermaid
stateDiagram-v2
  [*] --> INIT
  INIT --> IDLE: peripherals ready
  IDLE --> ENABLED: enable command and safe
  ENABLED --> IDLE: disable command
  ENABLED --> FAULT: estop or overcurrent or sensor fault
  FAULT --> IDLE: fault cleared and safe
```

- INIT: clocks and peripherals configured.
- IDLE: outputs disabled, waiting for enable.
- ENABLED: control loop active, PWM driving motors.
- FAULT: outputs latched off until manual clear and safe.

Implementation mapping (current + target)
- PWM generation: `STM_Firmware_AMR_v2/Core/Src/main.c:242` initializes TIM1 with CH1 and CH2. `STM_Firmware_AMR_v2/Core/Src/motor.c` applies duty via compare registers.
- Direction GPIO: `STM_Firmware_AMR_v2/Core/Src/main.c:471` configures PB4 and PB5 as outputs; `STM_Firmware_AMR_v2/Core/Src/motor.c` sets direction pins.
- Encoders: `STM_Firmware_AMR_v2/Core/Src/main.c:311` TIM2 and `STM_Firmware_AMR_v2/Core/Src/main.c:360` TIM3 configured in encoder mode (filters enabled per `.ioc`).
- Current sensing: `STM_Firmware_AMR_v2/Core/Src/main.c:181` ADC1 configured with channels 8 and 11 (left and right current). Target: sample at inner loop rate, filter, and feed current PI.
- Telemetry serial: `STM_Firmware_AMR_v2/Core/Src/main.c:409` USART2 at 460800 bps.

Telemetry v2 (proposed additions)
- Inner loop fields: `i_cmd`, `i_meas`, `i_err`, `i_p`, `i_i`, `i_out` and final `duty`.
- Outer loop fields: `w_cmd`, `w_meas`, `w_err`, `w_p`, `w_i`, `w_d`, `i_ref` (alias of `i_cmd`).

Notes and next steps
- Current firmware (v2) integrates PWM, encoders, and dual-wheel speed PI with duty ramp and RPM filtering; current sensing is calibrated and used for telemetry/protection (not in the control loop).
- Next steps: add base timer tick for speed loop; keep single-loop speed PI; extend telemetry with control terms if needed; add safety/fault handling (overcurrent/stall/estop) and ROS `/cmd_vel` intake with staleness timeout.

## Faults and Clear Criteria (Tentative)

Faults are detected in the Safety Manager and drive transitions to FAULT. Faults are latched until clear criteria are met. The list below is a starting point and can be refined as hardware is finalized.

Fault table (proposed)
- ESTOP_ACTIVE
  - Condition: E-stop input indicates stop asserted (active level after debounce)
  - Action: Immediate FAULT; force PWM outputs to 0; ignore enable
  - Clear: E-stop released (inactive for >= 50 ms) and manual clear requested
- OVERCURRENT_LEFT / OVERCURRENT_RIGHT
  - Condition: Filtered current exceeds Ithresh_L/R for > T_over_ms (e.g., 20-50 ms)
  - Action: FAULT; force PWM 0; record which side tripped
  - Clear: Current below Ithresh_clear for >= 200 ms and manual clear requested
- ENCODER_TIMEOUT_LEFT / ENCODER_TIMEOUT_RIGHT
  - Condition: Commanded speed magnitude > v_min_cmd and no encoder edges for > T_enc_ms
  - Action: FAULT (mechanical/electrical issue suspected)
  - Clear: Encoder activity resumes for that wheel and manual clear requested
- ADC_RANGE_FAULT
  - Condition: ADC reading stuck at 0 or max code for > N samples (channel-specific)
  - Action: FAULT; treat sensors unreliable
  - Clear: ADC readings return to nominal range for >= 200 ms and manual clear requested
- SUPPLY_UNDERVOLT / SUPPLY_OVERVOLT (from BMS or ADC if monitored)
  - Condition: Pack voltage below/above thresholds with hysteresis for > T_uv/ov
  - Action: FAULT; optionally pre-fault warn and ramp down
  - Clear: Voltage back within safe band for >= 1 s and manual clear requested
- DRIVER_FAULT (if driver has a fault pin/telemetry)
  - Condition: Motor driver reports fault state
  - Action: FAULT
  - Clear: Driver clears fault and manual clear requested

Recommended thresholds (to be tuned)
- Overcurrent Ithresh: based on motor/driver ratings; start with 1.2x nominal and time filter 25 ms
- Encoder timeout T_enc_ms: 200-300 ms when |v_cmd| > small threshold
- ADC stuck detection: 64 consecutive identical max/min codes or near-DC with zero variance
- Under/Over-voltage: per battery chemistry; include hysteresis bands

Clear policy
- Non-sticky conditions (e.g., brief noise) do not set FAULT; use time filters.
- Sticky FAULT requires both:
  - All fault conditions inactive and stable for the specified dwell time.
  - Explicit clear request (`clear_fault_cmd`) from host/CLI or user input.
- On clear, transition to IDLE; require `enable_cmd` to enter ENABLED.

Suggested fault mask (bit assignment)
- Bit 0: ESTOP_ACTIVE
- Bit 1: OVERCURRENT_LEFT
- Bit 2: OVERCURRENT_RIGHT
- Bit 3: ENCODER_TIMEOUT_LEFT
- Bit 4: ENCODER_TIMEOUT_RIGHT
- Bit 5: ADC_RANGE_FAULT
- Bit 6: SUPPLY_UNDERVOLT
- Bit 7: SUPPLY_OVERVOLT
- Bit 8: DRIVER_FAULT
- Bits 9-15: reserved

Telemetry
- Include `state`, `fault_mask`, and debounced inputs in the 50 Hz telemetry frame for observability.
