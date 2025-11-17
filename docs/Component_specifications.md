# AMR Component Specifications

## 1. Drive Motor
**Model:** Z2D30-12GN  
**Gearbox:** 2GN 30K (30:1)

| Parameter | Value |
|------------|--------|
| Rated Power | 30 W |
| Rated Voltage | 12 V |
| Load Speed | 2800 rpm |
| No-load Speed | 3200 rpm |
| Motor Torque @ Load | ≈ 0.102 N·m |
| Output Speed @ Load | ≈ 93 rpm |
| Output Torque (Ideal) | ≈ 3.07 N·m |
| Output Torque (70–80 % η) | 2.15 – 2.46 N·m |
| Est. Current | ≈ 2.5 A |

---

## 2. Motor Driver
**Model:** Cytron MDD20A (Dual Channel)

| Parameter | Value |
|------------|--------|
| Motor Supply | 6 – 30 V DC |
| Continuous Current / Channel | 20 A |
| Peak Current (Short-term) | 60 A |
| PWM Frequency | ~20 kHz |
| Logic Inputs | 0 – 5 V (DIR, PWM) |
| Channels | 2 |
| Notes | At high PWM frequencies, continuous current may reduce. |

---

## 3. Encoder
**Model:** HN3806-AB-600N (Incremental Optical)

| Parameter | Value |
|------------|--------|
| Pulses per Revolution | 600 PPR |
| Quadrature Output | A/B (up to 2400 counts/rev) |
| Supply Voltage | 5 – 24 V DC |
| Output Type | NPN Open-Collector |
| Shaft Diameter | 6 mm |
| Body Diameter | 38 mm |
| Max Speed | 5000 – 6300 rpm |
| Output Frequency | ~30 kHz |

Mounting and effective resolution
- Mount Location: Post-gearbox (wheel/output shaft) — confirmed
- Effective Counts/Rev (quadrature): 600 PPR × 4 = 2400 counts/rev

---

## 4. Jetson Nano (Host Computer)
**Model:** NVIDIA Jetson Nano Developer Kit

| Parameter | Value |
|------------|--------|
| GPU | NVIDIA Maxwell, 128 CUDA cores |
| CPU | Quad‑Core ARM Cortex‑A57 |
| Memory | 4 GB LPDDR4 |
| Storage | microSD (OS and data) |
| Networking | 1× Gigabit Ethernet (RJ45) |
| USB | USB 3.0 + USB 2.0 (via dev kit carrier) |
| Camera | 2× MIPI CSI‑2 (15‑pin) connectors |
| Display | HDMI (per dev kit carrier) |
| GPIO Header | 40‑pin (3.3 V logic: I2C, SPI, UART, GPIO) |
| Power Input | 5 V DC (barrel jack recommended) |
| Recommended Supply | ≥ 5 V, 4–6 A (headroom for peripherals) |
| Typical Current Draw | 2–4 A depending on load/peripherals |
| Notes | Prefer powered USB hub for high‑draw USB sensors (LiDAR/depth cam). |

---

## 5. LiDAR
**Model:** YDLidar G4 (2D 360°)

| Parameter | Value |
|------------|--------|
| Measurement Type | 2D planar, 360° scanning |
| Range | up to ~16 m (confirm per datasheet) |
| Scan Rate | ~5–12 Hz (adjustable) |
| Angular Resolution | ≈ 0.5–1.0° (mode‑dependent) |
| Interface | USB (USB‑to‑UART) or TTL UART |
| Power | 5 V DC, nominal ~0.5 A (peaks higher; confirm) |
| Mounting | Level, unobstructed 360° field of view |
| Notes | Use powered USB hub; route 5 V from sensor rail. Validate exact range and current from vendor docs. |

---

## 6. Depth Camera
**Model:** Intel RealSense D455 (Stereo Depth + RGB)

| Parameter | Value |
|------------|--------|
| Interface | USB 3.x (Type‑C cable), UVC/UAC compliant |
| Power | 5 V via USB (use powered hub if unstable) |
| Baseline | ~95 mm (depth sensors; confirm) |
| FOV | Wide FOV; verify per D455 datasheet for exact degrees |
| Depth Resolution / FPS | e.g., 848×480 up to high fps; 1280×720 typical (confirm) |
| RGB | Integrated color sensor |
| Mounting | Rigid, vibration‑damped; clear view ahead |
| Notes | Requires USB 3 bandwidth for full‑rate depth. Confirm exact specs from Intel datasheet. |

---

## 7. Proximity Sensors
**Model(s):** TBD (x8 units planned)

| Parameter | Value |
|------------|--------|
| Quantity | 8 |
| Power | TBD (3.3 V or 5 V) |
| Interface | TBD (GPIO digital / ADC analog / I2C / UART) |
| Range | TBD |
| Update Rate | TBD |
| Mounting | Corners/edges of chassis for obstruction detection |
| Notes | Final model and interface will determine wiring and firmware driver; debounce/filtering required |

---

## 8. Current Sensors
**Model:** Allegro ACS758LCB-050B (±50 A bidirectional)

| Parameter | Value |
|------------|--------|
| Supply Voltage | 5.0 V recommended (ratiometric) |
| Output Type | Analog, ratiometric to Vcc (≈ Vcc/2 at 0 A) |
| Measurement Range | ±50 A bidirectional |
| Sensitivity | ~40 mV/A at 5 V (module variant ACS758LCB-050B) |
| Bandwidth | Typ. up to hundreds of kHz (datasheet dependent) |
| Isolation | Hall‑effect, galvanically isolated conductor |
| Interface to MCU | STM32 ADC via resistor divider (see pin map) |
| Quantity | 2 (Left and Right motor lines) |

Notes
- Choose the variant so nominal operating current is within 20–70% of full scale for good resolution; ±50 A covers motor peaks with margin.
- Sensor output is centered at Vcc/2; firmware must subtract offset and apply sensitivity.
- With a 10k/15k divider, the ADC sees ~0–3.0 V for a 0–5 V sensor range; ensures 3.3 V ADC compliance. Sensitivity at ADC is ~24 mV/A after the divider.
- Add RC filtering post-divider to reduce PWM ripple; see wiring doc for recommended values.

---

## 9. Microcontroller
**Model:** STM32F401RE (Nucleo-F401RE board)

| Parameter | Value |
|------------|--------|
| Core | ARM Cortex-M4 @ 84 MHz with single-precision FPU |
| Flash / RAM | 512 KB Flash, 96 KB SRAM |
| Timers in use | TIM1 PWM (CH1/CH2), TIM2 encoder (right, 32-bit), TIM3 encoder (left, 16-bit) |
| ADC | ADC1 12-bit with DMA; channels PB0 (IN8) left current, PC1 (IN11) right current |
| UART | USART2 at 460800 bps for telemetry |
| GPIO map | PA8/PA9 PWM, PB4/PB5 DIR, PA6/PA7 TIM3 enc, PA0/PA1 TIM2 enc, PA2/PA3 UART2, PA5 LED, PC13 button |
| Debug | ST-LINK/V2-1 onboard |
| Notes | Hosts motor-control firmware and planned micro-ROS client |

---

## 10. Battery Pack
**Model/Chemistry:** 12.8 V LiFePO4, 4S, 18 Ah (18,000 mAh)

| Parameter | Value |
|------------|--------|
| Nominal Voltage | 12.8 V (LiFePO4 4S) |
| Charge Voltage (full) | ~14.6 V |
| Discharge Cutoff | ~10.0–10.5 V (depends on BMS) |
| Capacity | 18 Ah |
| Max Continuous Discharge | Depends on cell pack; size for ≥ 30–40 A to match drivetrain and headroom |
| Peak Discharge (short) | Depends on cells/BMS; referenced to pack’s internal BMS capability |
| Connector | Nylon T plug (male on charger; ensure matching female or adapter on pack) |
| Notes | LiFePO4 provides flatter discharge curve and better cycle life; pack includes internal BMS with limited telemetry |

---

## 11. Battery Management System (BMS)
**Model:** Built-in pack BMS (basic protection, limited telemetry)

| Parameter | Value |
|------------|--------|
| Series Cells Supported | 4S LiFePO4 (internal) |
| Continuous/Peak Current | Per pack design; not externally specified |
| Protections | OVP, UVP, OCP, SCP (basic) |
| Balance Method | Passive (typical for pack-integrated BMS) |
| Notes | Pack exposes only P+/P-; no cell-level telemetry. Add pack-voltage sensing if host needs charge status. |

## 12. Charger
**Model:** Pro Range 4S LiFePO4 14.6 V 7 A with nylon T male

| Parameter | Value |
|------------|--------|
| Chemistry | LiFePO4, 4S |
| Charge Voltage | 14.6 V CC/CV |
| Charge Current | 7 A |
| Connector | Nylon T male output (match to pack/BMS pigtail) |
| Notes | 7 A provides ~3 h charge time for 18 Ah pack; ensure BMS and wiring support ≥7 A charge current |

---

## 13. DC-DC Converters
Three supplies recommended; exact models TBD.

### 11.1 Jetson 5 V Supply
| Parameter | Value |
|------------|--------|
| Input | Battery pack (pre- or post-E-Stop per design) |
| Output | 5.0 V |
| Max Current | TBD (>= 6 A recommended) |
| Ripple/Noise | TBD (low ripple preferred) |
| Notes | Powers Jetson Nano and powered USB hub |

### 11.2 Logic 5 V Supply
| Parameter | Value |
|------------|--------|
| Input | Battery pack |
| Output | 5.0 V |
| Max Current | TBD (1-2 A typical) |
| Notes | Powers STM32 board and light sensors |

### 11.3 Sensors 12 V Supply (Optional)
| Parameter | Value |
|------------|--------|
| Input | Battery pack |
| Output | 12.0 V |
| Max Current | TBD |
| Notes | Only needed if any sensor requires 12 V |
