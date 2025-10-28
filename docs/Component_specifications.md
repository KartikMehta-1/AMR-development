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
**Model:** TBD

| Parameter | Value |
|------------|--------|
| Power Input | TBD |
| Typical Current Draw | TBD |
| Data Interfaces | TBD |
| Notes | TBD |

---

## 5. LiDAR
**Model:** TBD

| Parameter | Value |
|------------|--------|
| Power | TBD |
| Interface | TBD (USB/UART/Ethernet) |
| Range / FOV | TBD |
| Notes | TBD |

---

## 6. Depth Camera
**Model:** TBD

| Parameter | Value |
|------------|--------|
| Power | TBD |
| Interface | TBD (USB 3.0) |
| Resolution / FPS | TBD |
| Notes | TBD |

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
