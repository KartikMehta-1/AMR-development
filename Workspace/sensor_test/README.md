# Sensor Bench Test (ACS758 vs ACS70331)

This folder contains a complete bench-test package to compare current sensor performance before changing AMR wiring.

## Contents
- `test_plan.md` - test objectives, procedure, and analysis checklist.
- `wiring_schematic.md` - wiring map and scope channel assignments.
- `arduino/sensor_ab_benchmark/sensor_ab_benchmark.ino` - Arduino UNO sketch for repeatable PWM step tests and CSV logging.
- `tools/picoscope_linux_setup.md` - Linux PicoScope 7 + PicoSDK setup notes.
- `tools/setup_picoscope_linux.sh` - helper script to configure Pico repository and install packages.

## Quick start
1. Wire hardware using `wiring_schematic.md`.
2. Flash `sensor_ab_benchmark.ino` to Arduino UNO.
3. Start serial capture:
   - `arduino-cli monitor -p /dev/ttyACM0 -c baudrate=230400 | tee sensor_test_run.csv`
4. Capture synchronized waveforms in PicoScope:
   - CH-A: `ACS758_VOUT`
   - CH-B: `ACS70331_VOUT`
   - CH-C: trigger pin from Arduino (`D7`)
   - CH-D (optional): shunt/amplifier output for reference current
5. Follow pass/fail checks from `test_plan.md`.
6. Create a plot from Arduino CSV:
   - `python3 Workspace/sensor_test/tools/plot_sensor_benchmark.py sensor_test_run.csv`
   - Output image defaults to `sensor_test_run.png` in the same directory.
7. Create a merged Arduino + PicoScope plot:
   - `python3 Workspace/sensor_test/tools/plot_sensor_benchmark.py sensor_test_run.csv --pico-csv pico_export.csv --pico-col-acs758 "Channel A" --pico-col-acs70331 "Channel B" --pico-col-trig "Channel C" --out merged_plot.png`
   - If Pico channel names differ, pass exact names from your CSV header.

## D7 usage
- `D7` is a phase marker output from the Arduino script.
- The pin toggles HIGH/LOW on every phase transition (idle, 20%, 40%, etc.), so each duty block is clearly marked in scope captures.
- Connect `D7` to PicoScope `CH-C` and GND to common bench ground.
- In PicoScope, configure trigger on `CH-C` rising edge around `2V` threshold (for 5V logic).
- This gives repeatable capture alignment across runs and helps line up CSV rows with waveform segments.
- The plot script uses `D7` for time alignment by default when Pico trigger data is available (`--pico-align trigger`).

## Notes
- If you do not add a shunt reference, you can still compare noise, dynamic response, and PWM ripple behavior between both sensors.
- For absolute gain/linearity benchmarking, include a current reference path (low-side shunt + differential measurement).
