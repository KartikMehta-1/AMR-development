# STM32 Architecture (Firmware + micro-ROS)

Owner: Kartik Mehta  
Status: In progress - dual-motor speed PI with ramp; current used for protection; micro-ROS client active on USART2 with wheel-command + enable/estop/clear subscribers and wheel-state + diagnostic publishers.  
Last Updated: 2026-05-09

## Current Implementation Snapshot
- Control loop: TIM4 at 100 Hz; speed PI per wheel; duty ramp; differential-drive mapping from left/right wheel command topics; command staleness timeout (500 ms).
- Sensing: encoders TIM3 (left, 16-bit) / TIM2 (right, 32-bit) with RPM LPF; ADC1 CH8/11 current sense (ACS758) with scaling and LPF.
- Faults: overcurrent, stall, encoder timeout, ADC stuck detection; fault mask latched in ControlState and cleared via `/amr_stm/clear_fault` when faults/estop are inactive.
- micro-ROS: USART2 custom transport over the STM32 ST-LINK virtual COM path; subscribers are `/amr_stm/wheel_cmd_left`, `/amr_stm/wheel_cmd_right`, `/amr_stm/enable`, `/amr_stm/estop`, `/amr_stm/clear_fault`; publishers are `/amr_stm/wheel_state`, `/amr_stm/fault_mask`, `/amr_stm/safety_state`, `/amr_stm/duty_cmd_left`, `/amr_stm/duty_cmd_right`, `/amr_stm/current_left_ma`, `/amr_stm/current_right_ma`, `/amr_stm/current_left_adc`, `/amr_stm/current_right_adc`, `/amr_stm/current_left_zero`, `/amr_stm/current_right_zero`, and `/amr_stm/ros_diag`. Critical topics publish every 100 ms; current/ADC diagnostics publish every 500 ms.
- PWM/Dir: TIM1 CH1/CH2 at 20 kHz; DIR PB4/PB5; duty capped at 70%.
- Legacy UART telemetry: disabled to avoid contention with micro-ROS on USART2.
- Diagnostics policy: all current, duty, fault, and safety topics are intentionally still published. Some are mainly used by bench/monitor tools, but none are being pruned from firmware yet.

## Goals
- Deterministic control on STM32: dual-wheel speed PI (single-loop) with smooth ramps.
- Current sensing used for protection/diagnostics (not in the control loop).
- micro-ROS interface for per-wheel commands, enable/estop/clear, and telemetry.
- Safety gating and fault latching.

## Peripherals and IO
- PWM: TIM1 CH1/CH2 at 20 kHz (PA8/PA9).
- DIR GPIO: PB4/PB5.
- Encoders: TIM3 (left PA6/PA7), TIM2 (right PA0/PA1).
- ADC: ADC1 CH8 (PB0), CH11 (PC1) for ACS758 current.
- UART: USART2 460800 bps for micro-ROS custom transport.
- E-stop sense: PB10 (active low, pull-up).

## Parameters (current values in app_config.h)
- Geometry: TRACK_WIDTH_M=0.381, WHEEL_RADIUS_M=0.0615.
- Control loop: CONTROL_LOOP_HZ=100; CMD_TIMEOUT_MS=500.
- Duty limits: MOTOR_DUTY_MAX=0.70; DUTY_RAMP_RATE_PER_SEC=1.0.
- Command ramping: CMD_RAMP_ENABLE=0; V_CMD_RAMP_RATE_MPS=0.8 and W_CMD_RAMP_RATE_RAD=0.8 are configured for firmware command ramping if it is re-enabled.
- RPM filtering: RPM_LPF_ALPHA=0.05; RPM_SPIKE_LIMIT_RPM=500.
- Speed PI: KP/KI per wheel; output clamped to +/-0.90; I clamped to +/-2.0.
- Current sense: divider ratio 0.667; LPF alpha 0.1; zero tracking disabled with CURR_ZERO_TRACK_ALPHA=0.0 during current-sensor calibration.
- Fault thresholds:
  - OC: 1500 mA with 50 ms dwell.
  - Stall: duty >= 8% and |RPM| <= 0.5 for 500 ms.
  - Encoder timeout: cmd RPM >= 0.5 and measured ~0 for 1000 ms.
  - ADC stuck: 30 samples at rail or identical values (when |duty| >= 2%).

## Loop Rates and Ownership
- Inner/current tick: reserved (1-5 kHz) tied to PWM/ADC (deferred until higher-accuracy sensor).
- Outer/speed loop: 100 Hz via TIM4. Computes RPM, runs speed PI, updates duty targets.
- micro-ROS publish: critical STM topics every 100 ms; current/ADC diagnostic topics and `/amr_stm/ros_diag` every 500 ms.
- ISRs/ticks own control math; RTOS tasks move messages/buffers.

## Data Flow (single-loop)
1) Receive `/amr_stm/wheel_cmd_left` and `/amr_stm/wheel_cmd_right` via micro-ROS; store latest with timestamps.
2) Outer tick: read left/right wheel commands, clamp, apply ramp, run speed PI -> duty targets.
3) Apply duty via TIM1 CH1/CH2 (DIR set per polarity).
4) Sense: encoder deltas -> RPM (LPF); currents -> filtered for protection.
5) Fault monitor computes fault_bits; ControlState latches faults.
6) micro-ROS publishes wheel_state plus fault, safety, duty, and current diagnostics.

## Motor Control Diagrams

```mermaid
graph TD
  subgraph Inputs
    CMD[Speed Setpoint]
    ESTOP[Estop GPIO PB10]
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
  end

  subgraph Actuation
    PWM1[TIM1 CH1 PWM Left]
    PWM2[TIM1 CH2 PWM Right]
    DIRL[DIR Left GPIO PB4]
    DIRR[DIR Right GPIO PB5]
    MDD[Motor Driver Cytron]
  end

  subgraph Telemetry
    UROS[micro-ROS over USART2]
  end

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
  W_L --> PWM1
  W_R --> PWM2
  MODE --> DIRL
  MODE --> DIRR

  PWM1 --> MDD
  PWM2 --> MDD
  DIRL --> MDD
  DIRR --> MDD

  VEL --> UROS
  CUR_MON --> UROS
  MODE --> UROS
```

```mermaid
graph LR
  TICK_W[Outer Tick 100 Hz]
  READ_W[Read encoder counts]
  EST_W[Compute wheel velocity LPF]
  CTRL_W[Speed PI -> duty target]
  APPLY[Update PWM duty with ramp/cap]
  TEL[micro-ROS publish 100 ms critical / 500 ms diagnostics]

  TICK_W --> READ_W
  READ_W --> EST_W
  EST_W --> CTRL_W
  CTRL_W --> APPLY
  APPLY --> TEL
```

```mermaid
stateDiagram-v2
  [*] --> INIT
  INIT --> IDLE: peripherals ready
  IDLE --> ENABLED: enable command and safe
  ENABLED --> IDLE: disable command
  ENABLED --> FAULT: estop or sensor fault
  FAULT --> IDLE: fault cleared and safe
```

## micro-ROS Integration Architecture

```mermaid
flowchart TD
  subgraph ISRS[ISRs / High-Rate Ticks]
    ADCISR[ADC DMA Half / Full ISR\nInner Tick 1-5 kHz]
    TIMWISR[TIM Base ISR\nOuter Tick 100 Hz]
  end

  subgraph CTRL[Control + IO]
    CURMON[Current Monitor\noffset, scale, LPF]
    W_L[Outer Speed PI Left]
    W_R[Outer Speed PI Right]
    VEL[Velocity Estimator\nenc deltas to RPM]
    SAFE[Fault Monitor + State]    
    MODE[Mode Manager]    
    PWM1[PWM OUT TIM1 CH1]    
    WM2[PWM OUT TIM1 CH2]    
    ENC[Encoders TIM2/TIM3]    
    ADC[ADC1 CH8/11]
  end

  subgraph TASKS[FreeRTOS Tasks]
    RPUB[ros_pub_task\nexecutor + publish loop]
  end

  ADC --> ADCISR
  ADCISR --> CURMON
  ENC --> TIMWISR
  TIMWISR --> VEL

  VEL --> W_L
  VEL --> W_R
  CURMON --> SAFE
  SAFE --> MODE
  MODE --> W_L
  MODE --> W_R

  W_L --> PWM1
  W_R --> PWM2

  RPUB -->|/amr_stm/wheel_cmd_left right| MODE
  VEL --> RPUB
  SAFE --> RPUB
```

Current topics
- Sub: `/amr_stm/wheel_cmd_left`, `/amr_stm/wheel_cmd_right` (std_msgs/Float32, wheel angular velocity command in rad/s as currently consumed by the firmware).
- Sub: `/amr_stm/enable` (std_msgs/Bool), `/amr_stm/estop` (std_msgs/Bool), `/amr_stm/clear_fault` (std_msgs/Empty).
- Pub: `/amr_stm/duty_cmd_left`, `/amr_stm/duty_cmd_right` (std_msgs/Float32, duty percent).
- Pub: `/amr_stm/fault_mask` (std_msgs/Int32).
- Pub: `/amr_stm/wheel_state` (sensor_msgs/JointState).
- Pub: `/amr_stm/safety_state` (std_msgs/UInt32).
- Pub: `/amr_stm/current_left_ma`, `/amr_stm/current_right_ma` (std_msgs/Int32, filtered current estimate in mA).
- Pub: `/amr_stm/current_left_adc`, `/amr_stm/current_right_adc` (std_msgs/UInt32, raw ADC sample).
- Pub: `/amr_stm/current_left_zero`, `/amr_stm/current_right_zero` (std_msgs/UInt32, zero-offset estimate).
- Pub: `/amr_stm/ros_diag` (std_msgs/UInt32MultiArray, micro-ROS diagnostics).

## Fault Mask (current bits)
- Bit 0: CTRL_FAULT_ESTOP
- Bit 1: CTRL_FAULT_OC_LEFT
- Bit 2: CTRL_FAULT_OC_RIGHT
- Bit 3: CTRL_FAULT_STALL_LEFT
- Bit 4: CTRL_FAULT_STALL_RIGHT
- Bit 5: CTRL_FAULT_ENC_TIMEOUT_LEFT
- Bit 6: CTRL_FAULT_ENC_TIMEOUT_RIGHT
- Bit 7: CTRL_FAULT_ADC_STUCK
- Bit 15: CTRL_FAULT_GENERIC

## Implementation Mapping
- Control loop + tasks: `STM/STM_Firmware_AMR_v2/Core/Src/main.c` (control_task at 100 Hz, ros_pub_task executor + publish).
- Control law + ramps: `STM/STM_Firmware_AMR_v2/Core/Src/control_loop.c`.
- Fault detection: `STM/STM_Firmware_AMR_v2/Core/Src/fault_monitor.c`.
- State machine and fault latch: `STM/STM_Firmware_AMR_v2/Core/Src/control_state.c`.
- Current sensing: `STM/STM_Firmware_AMR_v2/Core/Src/current_sense.c`.

## Known Gaps
- voltage faults pending ADC/BMS integration.

## Next Steps
- Add voltage faults when available.
- Optional: per-wheel feedforward to reduce duty skew; finalize PI gains and log overshoot with `Workspace/python_scripts/check_overshoot.py`.
- Defer cascaded current control unless needed; keep current for protection/logging.
