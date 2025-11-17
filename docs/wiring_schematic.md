# AMR Wiring and Power Schematic (Text Overview)

This document captures the practical wiring plan for the AMR project: power distribution, E‑stop, and signal interconnects between the STM32 controller, motor driver, encoders, Jetson Nano, and sensors.

---

## 1) Power Distribution

- Battery: 12.8 V LiFePO4 4S, 18 Ah (pack has basic internal BMS)
  - Main Fuse: size for expected peak (e.g., 30–40 A slow‑blow for current drivetrain; revisit after measurements)
  - Main Switch / E‑Stop contactor (see E‑stop section)
  - Branches:
    - Motor Power Bus → Motor Driver VM (Cytron MDD20A)
    - DC‑DC Buck → 5 V Rail (Jetson Nano, USB hub)
    - DC‑DC Buck → 5 V/12 V Rails for sensors (LiDAR, depth cam, proximity)

Notes
- Use star ground: join motor return, STM32 GND, Jetson GND, and sensor grounds at a solid common point.
- Size DC‑DC modules with margin (Jetson Nano can draw 3–4 A peak on 5 V; depth cameras and LiDAR add significant load).
- Keep motor currents off the logic 5 V/3.3 V rails; separate power domains that meet at ground.

### 1a) Power Distribution Details
- Battery Pack: 12.8 V LiFePO4 4S 18 Ah with internal basic BMS. Outputs raw pack voltage; no regulated 5 V/12 V and limited telemetry.
- DC-DC 5 V Jetson: dedicated 5 V buck with ≥ 6–8 A continuous (e.g., Mean Well RSD‑60G‑5 wide‑input or equivalent). Powers Jetson Nano and powered USB hub.
- DC-DC 5 V Logic: 5 V supply for STM32 board and proximity sensors (1–2 A typical). Can share with Jetson rail if capacity and noise allow; otherwise isolate.
- DC-DC 12 V Sensors (optional): only if any sensor requires 12 V; otherwise omit.

Voltage monitoring (optional)
- Add a resistive divider from pack P+/P- to an ADC input (on STM32 or a small monitor) to estimate state of charge and low‑voltage cutoff warnings. Choose values to keep ADC input under 3.3 V at 14.6 V full charge.

Wiring intent
- Battery/BMS → Main Fuse → E-Stop → Motor Power Bus → Motor driver VM.
- Battery/BMS → DC-DC 5 V Jetson → Jetson Nano, USB hub.
- Battery/BMS → DC-DC 5 V Logic → STM32, proximity sensors.
- Battery/BMS → DC-DC 12 V Sensors → 12 V sensors (if used).

---

## 1b) Current Sensor Wiring (ACS758LCB-050B)

- Placement: One module per motor supply line; wire IP+ and IP- in series with each wheel feed.
- Power: Vcc = 5 V; GND common with STM32. Add 0.1 µF decoupling at the sensor.
- Sense output: Vout into a 10 kΩ top resistor and 15 kΩ bottom to GND; tap the midpoint to ADC (PB0 left, PC1 right) to keep ADC under 3.3 V at 5 V Vout.
- Filter: Add 1 kΩ series plus 100 nF to GND after the divider to reduce PWM ripple.
- Layout: Keep the IP+/IP- loop short and away from signal wiring; route Vout away from motor leads.

---

## 1c) Wiring Diagram (text/mermaid overview)

```mermaid
graph TD
  BATT[12.8 V LiFePO4<br/>B+/B-] --> FUSE[Main Fuse]
  FUSE --> ESTOP[E-stop / Switch]
  ESTOP --> VM[Motor Power Bus]
  VM --> CS_L[ACS758L Left<br/>IP+ → IP-]
  VM --> CS_R[ACS758R Right<br/>IP+ → IP-]
  CS_L --> MDD_L[MDD20A M1 VM]
  CS_R --> MDD_R[MDD20A M2 VM]

  subgraph MDD[Cytron MDD20A]
    MDD_L -- PWM PA8 --> PWM1[PA8 TIM1_CH1]
    MDD_R -- PWM PA9 --> PWM2[PA9 TIM1_CH2]
    DIRL[PB4 DIR] --> MDD_L
    DIRR[PB5 DIR] --> MDD_R
    GNDMDD[GND] -.-> PWM1
    GNDMDD -.-> PWM2
  end

  subgraph Encoders
    ENC_LA[PA6] --- ENCL[Enc L A]
    ENC_LB[PA7] --- ENCLB[Enc L B]
    ENC_RA[PA0] --- ENCR[Enc R A]
    ENC_RB[PA1] --- ENCRB[Enc R B]
    ENC_GND[GND] --- ENCG[Enc GND]
  end

  subgraph Currents to ADC
    CS_L_V[ACS758L Vout] -->|10k/15k divider + 1k/100nF| ADC_L[PB0 ADC1_IN8]
    CS_R_V[ACS758R Vout] -->|10k/15k divider + 1k/100nF| ADC_R[PC1 ADC1_IN11]
  end

  VM --> BUCK5V[5 V Buck Jetson >=6–8 A]
  BUCK5V --> JET[Jetson + Powered Hub]
  VM --> BUCK5V_LOGIC[5 V Buck Logic]
  BUCK5V_LOGIC --> MCU[STM32F401RE]
  GNDALL[GND star] -.-> MDD
  GNDALL -.-> MCU
  GNDALL -.-> JET
  GNDALL -.-> Encoders
  GNDALL -.-> Currents to ADC
```

Notes
- Insert one ACS758 per wheel supply. Keep IP+/IP- loops short. Route Vout away from motor leads.
- Jetson/hub on dedicated high-current 5 V buck. Logic on separate 5 V buck or shared if capacity/noise is acceptable.
- All grounds meet at a solid star point near the power entry.

---

## 1d) Wire Gauge Guidance (initial sizing)

- Battery → Fuse → E-stop → Motor driver VM/GND: AWG 12–14, keep short and well-crimped.
- Motor driver → Motors (each channel): AWG 14–16 depending on run length and expected current; shorter runs can use 16.
- DC-DC 5 V Jetson rail: AWG 16–18 from buck to Jetson/hub to minimize drop; use quality connectors.
- DC-DC 5 V Logic rail: AWG 20–22 (STM32 and light sensors).
- Encoder A/B and logic signals: AWG 24–26 twisted pair with ground return; optionally shield if noisy.
- Sensor USB cables: use powered hub for LiDAR/RealSense; keep USB leads short and rated for current.
- Grounds: implement star point with the same gauge as the largest branch it serves (typically match the supply feed gauge).

Notes
- If measured peaks exceed assumptions, upsize the affected runs one gauge thicker.
- Keep high-current runs separated from encoder and ADC wiring; cross at right angles when needed.

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

## 3) STM32 ↔ Motor Driver (Cytron MDD20A)

Cytron MDD20A dual channel driver is the active configuration. Connections are dead simple PWM plus DIR lines per channel. Keep motor supply routed through fuse and E stop before entering the VM pin.

### Signal wiring
- PWM: `PA8 TIM1_CH1` → `M1 PWM` left wheel. `PA9 TIM1_CH2` → `M2 PWM` right wheel.
- Direction: `PB4` → `M1 DIR` left. `PB5` → `M2 DIR` right.
- Ground: Tie STM32 ground to driver logic ground close to the driver header.
- Optional brake or coast pins can stay tied per driver manual until firmware exposes those modes.

### Power wiring
- Motor power: Battery or pack output → Fuse → E stop → Cytron MDD20A `VM`.
- Logic power: Driver logic shares ground with STM32 and is driven by the PWM and DIR inputs (no extra 5 V logic supply pin).
- Ensure motor returns and logic returns join at a solid point to avoid injecting noise into ADC grounds.

### Timing targets
- Set TIM1 prescaler plus ARR for roughly 20 kHz PWM. Higher frequencies reduce audible whine but watch driver thermal behavior.
- Keep duty updates synchronized with the inner current loop so left and right channels stay matched.

---

## 4) Encoders ↔ STM32 (HN3806‑AB‑600N, open‑collector)

- Signals: `A`, `B` (quadrature). Index `Z` if used (optional, not required for speed control).
- Voltage: 5–24 V supply supported by encoder; outputs are NPN open‑collector.
- Interface to MCU:
  - Provide external pull‑ups to 3.3 V on `A` and `B` (e.g., 4.7–10 kΩ).
  - Left wheel: `A` → `PA6 (TIM3_CH1)`, `B` → `PA7 (TIM3_CH2)` (TIM3 encoder mode).
  - Right wheel: `A` → `PA0 (TIM2_CH1/ETR)`, `B` → `PA1 (TIM2_CH2)` (TIM2 encoder mode).
  - Common ground between encoders and STM32.
- Filtering: Enable digital filters on TIM2/TIM3 inputs (CubeMX IC filter ≈ 10) to reject noise.
- Mounting: Post‑gearbox (wheel/output shaft) — confirmed.
- COUNTS_PER_REV: 600 PPR × 4 = 2400 counts per wheel revolution.
  - If remounted pre‑gearbox (motor shaft) with 30:1 ratio: 2400 × 30 = 72,000 counts per wheel rev (update firmware accordingly).

---

## 5) STM32 ↔ Jetson Nano

Data link options
- UART (3.3 V TTL): STM32 `USART2 TX (PA2)` → Jetson `UART RX` (J41 pin 10), STM32 `USART2 RX (PA3)` ← Jetson `UART TX` (J41 pin 8). GND common.
- USB: Use ST‑Link USB serial or dedicated USB‑UART adapter to Jetson USB.

Power (Jetson)
- Provide dedicated 5 V rail with sufficient current (target ≥ 6–8 A to cover Jetson plus USB peripherals). Power Jetson via 5 V header or barrel jack per NVIDIA guidance; keep harness short and low resistance.

---

## 6) Sensors ↔ Jetson (typical)

- LiDAR (YDLidar G4): USB (USB‑to‑UART) to Jetson via powered USB hub; 5 V power from sensor/USB rail (budget ~0.5 A nominal; confirm peaks). Keep cable short; ensure stable 5 V.
- Depth Camera (Intel RealSense D455): USB 3.x (Type‑C cable) to Jetson (prefer powered hub if multiple devices). Power from USB 5 V; ensure USB 3 bandwidth.
- Proximity Sensors: Choose interface (GPIO/I2C/UART). Initial plan: connect to STM32 for real-time safety and integration; can later move to Jetson for ROS-level processing if needed.

Notes
- Use powered USB hub if multiple high‑draw USB devices are attached.
- Keep sensor grounds tied to the logic ground.

---

## 7) Proximity Sensors ↔ STM32 (x8 planned)

Goal: Obstruction detection around the AMR perimeter using 8 proximity sensors mounted near corners/edges.

Interfaces (choose per sensor model; to be finalized):
- Digital GPIO (thresholded distance or presence):
  - Each sensor → 1x STM32 GPIO input
  - Add pull-up/down as recommended; consider RC debounce (~1–5 ms)
  - Pros: simple; Cons: less range resolution (binary)
- Analog (voltage proportional to distance):
  - Each sensor → 1x STM32 ADC channel (0–3.3 V). Use resistor divider if sensor outputs 5 V
  - Sample via round-robin scheduler; apply low-pass filtering
  - Pros: simple wiring; Cons: uses many ADC channels
- I2C (addressable rangefinders):
  - Shared I2C bus (3.3 V level). If identical addresses, add I2C mux or per-sensor enable
  - Power decoupling near each sensor; twisted pair for SCL/SDA to reduce noise
- UART (less common for 8x):
  - Requires multiplexing or shared UART with addressing; generally avoid if many sensors

Power
- Provide clean 5 V or 3.3 V rail as required by sensors (TBD current). Decouple locally (0.1 µF + 10 µF)
- Route sensor returns to logic ground; avoid sharing high-current motor returns

Firmware notes
- Driver will support 8 channels with sampling, debounce/filtering, timeout faulting, and obstacle event reporting
- micro-ROS topic plan: `/amr/obstacles` (e.g., sensor_msgs/Range[] or custom)

Schematic placeholders (to be finalized on sensor selection)
- `S1..S8`: V+, GND, SIGNAL → STM32 (GPIO/ADC/I2C)
- Pin assignments: TBD in `docs/pin_map.yaml` once sensor model/interface is chosen

---

## 8) Current Sensing (ACS758, both motors)

Goal: Measure per‑motor current for protection, logging, and control.

Hardware
- Sensor: ACS758 (variant TBD per current range) installed in series with each motor power line (left/right).
- Supply: 5.0 V recommended; output is ratiometric (≈ Vcc/2 at 0 A).
- Output conditioning to STM32 ADC:
  - Resistor divider: 10 kΩ (top) + 15 kΩ (bottom) → scales 0–5 V to ~0–3.0 V (see `docs/pin_map.yaml`).
  - RC filter: 1 kΩ series + 100 nF to ground after divider (fc ≈ 1.6 kHz) to reduce PWM ripple/EMI.
  - ADC pins: `PB0 / ADC1_IN8` (Left current), `PC1 / ADC1_IN11` (Right current).
  - Sampling: ADC1 with DMA in circular mode for periodic current reads.
- Decoupling: 0.1 µF ceramic close to ACS758 Vcc; follow datasheet layout guidance.
- Grounding: Star‑point ground; keep sensor Vout return clean and away from high di/dt loops.

Calibration and math
- Zero offset (at 0 A): ≈ Vcc/2 at sensor output; after divider ≈ 0.6 × (Vcc/2).
- Sensitivity (mV/A): depends on variant (e.g., ~40 mV/A for ±50 A). Confirm actual part.
- Formula (at sensor output): I[A] = (Vout − Vcc/2) / Sensitivity.
- With divider (ratio ≈ 0.6): I[A] = ((Vadc/0.6) − Vcc/2) / Sensitivity.
- Firmware should estimate Vcc (5 V) or measure the ADC reference to compensate ratiometric behavior.

Safety and layout
- Size conductors for expected peak current; ensure secure mechanical mounting and insulation.
- Route high‑current paths short and tight; keep analog lines separated from PWM motor traces.
- Verify polarity/orientation per ACS758 datasheet so that positive current matches expected sign.

---

## 9) Quick Connector Summary

- STM32 (Nucleo‑F401RE pinout references):
  - `PA8 / TIM1_CH1` → PWM Left (M1)
  - `PA9 / TIM1_CH2` → PWM Right (M2)
  - `PB4` → DIR Left (M1), `PB5` → DIR Right (M2)
  - `PA6/PA7 / TIM3` → Left encoder A/B
  - `PA0/PA1 / TIM2` → Right encoder A/B
  - `PB0 / ADC1_IN8` → Left motor current (ACS758)
  - `PC1 / ADC1_IN11` → Right motor current (ACS758)
  - `PA2/PA3` → UART2 TX/RX to Jetson (optional)
  - `PA5` → Status LED
- Motor Driver:
  - Cytron MDD20A terminals: M1 PWM, M1 DIR, M2 PWM, M2 DIR, VM, GND, Motor outputs
- Encoder: A, B, V+, GND (open‑collector outputs with 3.3 V pull‑ups)
- Proximity Sensors (x8): V+, GND, SIGNAL to STM32 (GPIO/ADC/I2C) — exact interface TBD
- Jetson Nano: 5 V, GND, USB ports, J41 UART if used
- YDLidar G4 / RealSense D455: USB to Jetson via powered hub; 5 V from sensor/USB rail
- Proximity: USB/UART/I2C/GPIO as per model

---

## 10) Layout and EMI Tips

- Keep motor and driver wiring away from encoder and logic wiring; cross at 90° when needed.
- Twist encoder A/B with ground return; shield if available.
- Use proper ferrules and strain relief; avoid loose connectors near moving parts.
- Verify polarity before powering; bring‑up with current‑limited supply when possible.


