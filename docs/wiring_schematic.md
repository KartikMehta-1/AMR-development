# AMR Wiring and Power Schematic (Text Overview)

This document captures the practical wiring plan for the AMR project: power distribution, E‑stop, and signal interconnects between the STM32 controller, motor driver, encoders, Jetson Nano, and sensors.

---

## 1) Power Distribution

- Battery (TBD V, TBD Ah)
  - Main Fuse (TBD A) as close to battery as possible
  - Main Switch / E‑Stop contactor (see E‑stop section)
  - Branches:
    - Motor Power Bus → Motor Driver VM (Cytron MDD20A)
    - DC‑DC Buck → 5 V Rail (Jetson Nano)
    - DC‑DC Buck → 5 V/12 V Rails for sensors (LiDAR, depth cam, proximity)

Notes
- Use star ground: join motor return, STM32 GND, Jetson GND, and sensor grounds at a solid common point.
- Size DC‑DC modules with margin (Jetson Nano can draw 3–4 A peak on 5 V; depth cameras and LiDAR add significant load).
- Keep motor currents off the logic 5 V/3.3 V rails; separate power domains that meet at ground.

---

## 2) Emergency Stop (E‑stop)

- Primary action: Hardware cut of Motor Power Bus feeding the motor driver VM input.
  - Option A: Latching E‑stop switch in series with motor supply (simplest).
  - Option B: E‑stop switch drives a relay/contactor or high‑side switch that disconnects VM.
- Secondary action: STM32 reads E‑stop state on a GPIO input to report and latch a software fault (PWM forced to 0 until manual clear).
- Do not rely solely on software for E‑stop.

Wiring summary
- E‑stop switch in series with motor supply to Cytron MDD20A VM.
- E‑stop sense line → STM32 GPIO (with pull‑up/down as appropriate). Debounce in hardware and/or firmware.

---

## 3) STM32 ↔ Motor Driver (Single Channel shown)

Two common configurations are shown. Choose one based on your current stage.

### A) Current bench (L298N)
- PWM: `PA8 (TIM1_CH1)` → L298N `ENA`
- Direction: `PB4` → `IN1`, `PB5` → `IN2`
- Motor power: Battery → L298N Vmotor (through fuse and E‑stop). GND common with STM32.

### B) Final driver (Cytron MDD20A)
- PWM: `PA8 (TIM1_CH1)` → `M1 PWM`
- Direction: `PB4` → `M1 DIR`
- (Optional) Brake/Coast: tie as per driver manual or implement in software via PWM/DIR logic
- Motor power: Battery → MDD20A `VM` (through fuse and E‑stop). GND common with STM32.
- Logic supply: MDD20A uses the same ground; logic inputs accept 3.3 V/5 V TTL (verify VIH in datasheet).

PWM frequency target
- Set ~20 kHz for quiet operation with MDD20A (TIM1 example prescaler/ARR to be configured accordingly).

---

## 4) Encoder ↔ STM32 (HN3806‑AB‑600N, open‑collector)

- Signals: `A`, `B` (quadrature). Index `Z` if used (optional, not required for speed control).
- Voltage: 5–24 V supply supported by encoder; outputs are NPN open‑collector.
- Interface to MCU:
  - Provide external pull‑ups to 3.3 V on `A` and `B` (e.g., 4.7–10 kΩ).
  - Connect `A` → `PA6 (TIM3_CH1)`, `B` → `PA7 (TIM3_CH2)`.
  - Common ground between encoder and STM32.
- Filtering: Enable a small digital filter on TIM3 inputs to reject noise.
- Mounting: Post‑gearbox (wheel/output shaft) — confirmed.
- COUNTS_PER_REV: 600 PPR × 4 = 2400 counts per wheel revolution.
  - If remounted pre‑gearbox (motor shaft) with 30:1 ratio: 2400 × 30 = 72,000 counts per wheel rev (update firmware accordingly).

---

## 5) STM32 ↔ Jetson Nano

Data link options
- UART (3.3 V TTL): STM32 `USART2 TX (PA2)` → Jetson `UART RX` (J41 pin 10), STM32 `USART2 RX (PA3)` ← Jetson `UART TX` (J41 pin 8). GND common.
- USB: Use ST‑Link USB serial or dedicated USB‑UART adapter to Jetson USB.

Power (Jetson)
- Provide dedicated 5 V rail with sufficient current (≥ 4 A recommended). Power Jetson via 5 V header or barrel jack per NVIDIA guidance.

---

## 6) Sensors ↔ Jetson (typical)

- LiDAR: USB or UART depending on model (TBD). Power from 5 V rail (TBD current).
- Depth Camera: USB 3.0 to Jetson; power from 5 V rail (TBD current).
- Proximity Sensors: Choose interface (GPIO/I2C/UART). Initial plan: connect to Jetson for ROS integration; alternatively, to STM32 if real‑time is needed.

Notes
- Use powered USB hub if multiple high‑draw USB devices are attached.
- Keep sensor grounds tied to the logic ground.

---

## 7) Optional: Current Sensing (future)

- Sensor: ACS758 (model TBD) on motor supply line.
- Output: Analog voltage proportional to current.
- Interface: To STM32 ADC input (requires 0–3.3 V range; scale/offset as needed).
- Since ADC is currently skipped, plan the PCB/wiring now; enable in firmware later.

---

## 8) Quick Connector Summary

- STM32 (Nucleo‑F401RE pinout references):
  - `PA8` → PWM to driver (L298N ENA or MDD20A PWM)
  - `PB4` → DIR (and `PB5` for L298N IN2 if used)
  - `PA6/PA7` → Encoder A/B (TIM3)
  - `PA2/PA3` → UART2 TX/RX to Jetson (optional)
  - `PA5` → Status LED
- Motor Driver (choose one):
  - L298N: ENA, IN1, IN2, VM, GND, Motor outputs
  - Cytron MDD20A: M1 PWM, M1 DIR, VM, GND, Motor outputs
- Encoder: A, B, V+, GND (open‑collector outputs with 3.3 V pull‑ups)
- Jetson Nano: 5 V, GND, USB ports, J41 UART if used
- LiDAR/Depth/Proximity: USB/UART/I2C as per model (TBD)

---

## 9) Layout and EMI Tips

- Keep motor and driver wiring away from encoder and logic wiring; cross at 90° when needed.
- Twist encoder A/B with ground return; shield if available.
- Use proper ferrules and strain relief; avoid loose connectors near moving parts.
- Verify polarity before powering; bring‑up with current‑limited supply when possible.
