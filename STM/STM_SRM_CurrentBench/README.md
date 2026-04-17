# STM_SRM_CurrentBench

Standalone STM32 bench firmware to validate ACS758 current sensor noise while driving the AMR motors with existing wiring.

## Purpose
- Keep AMR main firmware unchanged.
- Reuse current AMR wiring:
  - PWM: `PA8` (left), `PA9` (right)
  - DIR: `PB4` (left), `PB5` (right)
  - Current sense: `PB0` (`ADC1_IN8`, left), `PC1` (`ADC1_IN11`, right)
  - UART2: `PA2/PA3` @ `460800`
- Stream raw and filtered current to UART as CSV.

## Runtime sequence
1. Startup with motors off.
2. Auto zero-calibration (`CAL_SAMPLES`).
3. Repeating phases:
   - `idle` -> both motors off
   - `left` -> left motor at fixed duty (`TEST_DUTY_PCT`)
   - `coast` -> both motors off
   - `right` -> right motor at fixed duty

## UART output
Header:
`#HEADER: t_ms,phase,duty_l_pct,duty_r_pct,adc_l,adc_r,zero_l,zero_r,raw_l_mA,raw_r_mA,filt_l_mA,filt_r_mA`

Data rows:
- `adc_*`: averaged ADC counts
- `zero_*`: startup zero offsets
- `raw_*_mA`: converted current from counts
- `filt_*_mA`: low-pass filtered current

## Tuning knobs in `Src/main.c`
- `TEST_DUTY_PCT`
- `SAMPLE_PERIOD_MS`, `PRINT_PERIOD_MS`
- `ADC_AVG_SAMPLES`
- `CURR_LPF_ALPHA`
- `LEFT_CURR_POLARITY`, `RIGHT_CURR_POLARITY`
- phase durations (`PHASE_*_MS`)

## Build (CubeIDE generated makefiles)
From project root:
```bash
cd Debug
make all
```

## Suggested bench checks
1. Motor OFF noise floor (`phase=idle`): look at `adc_l`, `adc_r`, `raw_*_mA`.
2. Left-only drive (`phase=left`): confirm left current rises and right stays near zero.
3. Right-only drive (`phase=right`): mirror behavior.
4. If current sign is inverted, flip `LEFT_CURR_POLARITY` / `RIGHT_CURR_POLARITY`.
