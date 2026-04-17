"""
Live telemetry plotter for STM UART output.

Assumes firmware prints lines like:
#HEADER: t_ms,l_cnt,r_cnt,l_rpm_x10,r_rpm_x10,l_mA,r_mA
<data rows...>

Left/right RPM, duty (if present), and current are plotted in real time.
"""

from collections import deque
import math
import time

import matplotlib.pyplot as plt
import serial

# =========================
# USER CONFIG
# =========================
PORT = "COM4"            # <-- change to your STM32's COM port
BAUD = 460800            # must match firmware UART2
MAX_POINTS = 400         # number of samples shown
REFRESH_INTERVAL = 0.05  # seconds between plot refresh


# Helper: safe float conversion (non-numeric -> nan)
def safe_float(val):
  try:
    return float(val)
  except Exception:
    try:
      return float(str(val).strip())
    except Exception:
      return float("nan")


def normalize_time(buffers):
  """Return time axis in seconds, zeroed to first sample."""
  if "t_ms" in buffers and len(buffers["t_ms"]) > 0:
    t0 = buffers["t_ms"][0]
    return [(v - t0) / 1000.0 for v in buffers["t_ms"]]
  if "t" in buffers and len(buffers["t"]) > 0:
    t0 = buffers["t"][0]
    return [v - t0 for v in buffers["t"]]
  n = len(next(iter(buffers.values())))
  return list(range(n))


def duty_from_buffer(buffers, keys):
  """Extract duty (%) from any of the provided keys; handles 0-1 or 0-100."""
  for k in keys:
    if k not in buffers:
      continue
    arr = list(buffers[k])
    if any((not math.isnan(v)) and v > 1.5 for v in arr):
      return arr  # already in percent
    return [v * 100.0 for v in arr]  # assume 0.0-1.0 -> percent
  return []


# =========================
# INITIALIZE SERIAL
# =========================
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
print("Waiting for header...")

# =========================
# WAIT FOR HEADER LINE
# =========================
headers = []
meta = {}

while True:
  line = ser.readline().decode(errors="ignore").strip()
  if not line:
    continue
  if line.startswith("#HEADER:"):
    parts = [p.strip() for p in line.replace("#HEADER:", "").split(",")]
    numeric_headers = []
    for p in parts:
      if "=" in p:
        k, v = p.split("=", 1)
        meta[k.strip().lower()] = v.strip()
      elif p:
        numeric_headers.append(p)
    headers = numeric_headers
    print(f"Received header: {headers} meta={meta}")
    break

if not headers:
  ser.close()
  raise SystemExit("No headers received. Check your STM telemetry '#HEADER:' output.")

# Initialize data buffers
buckets = {h: deque(maxlen=MAX_POINTS) for h in headers}

# =========================
# SETUP PLOTS (RPM, Duty, Current)
# =========================
plt.ion()
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.suptitle(f"Live Telemetry (baud={BAUD}) test={meta.get('test','')}", fontsize=14, fontweight="bold")

ax_rpm, ax_duty, ax_curr = axes

line_rpm_l, = ax_rpm.plot([], [], "r-", label="L RPM")
line_rpm_r, = ax_rpm.plot([], [], "b-", label="R RPM")
line_rpm_l_tgt, = ax_rpm.plot([], [], "r--", label="L RPM tgt")
line_rpm_r_tgt, = ax_rpm.plot([], [], "b--", label="R RPM tgt")
ax_rpm.set_ylabel("RPM")
ax_rpm.legend(loc="upper right")
ax_rpm.grid(True)

line_duty_l, = ax_duty.plot([], [], "r-", label="L Duty (%)")
line_duty_r, = ax_duty.plot([], [], "b-", label="R Duty (%)")
ax_duty.set_ylabel("Duty (%)")
ax_duty.legend(loc="upper right")
ax_duty.grid(True)

line_curr_l, = ax_curr.plot([], [], "r-", label="L Current (A)")
line_curr_r, = ax_curr.plot([], [], "b-", label="R Current (A)")
ax_curr.set_ylabel("Current (A)")
ax_curr.set_xlabel("Time (s)")
ax_curr.legend(loc="upper right")
ax_curr.grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.95])
print("Streaming data... Press Ctrl+C to stop.\n")

# =========================
# STREAM + PLOT LOOP
# =========================
try:
  while True:
    line = ser.readline().decode(errors="ignore").strip()
    if not line:
      continue

    if line.startswith("#HEADER:") or line.startswith("#"):
      continue  # ignore comments/redundant headers

    values = [v.strip() for v in line.split(",")]
    if len(values) != len(headers):
      continue  # skip malformed lines

    parsed = {h: safe_float(v) for h, v in zip(headers, values)}
    for h in headers:
      buckets[h].append(parsed[h])

    if len(next(iter(buckets.values()))) < 2:
      continue

    t = normalize_time(buckets)

    # RPM (x10 fields -> RPM)
    l_rpm = [v / 10.0 for v in buckets.get("l_rpm_x10", [])] if "l_rpm_x10" in buckets else []
    r_rpm = [v / 10.0 for v in buckets.get("r_rpm_x10", [])] if "r_rpm_x10" in buckets else []
    l_rpm_tgt = [v / 10.0 for v in buckets.get("l_rpm_tgt_x10", [])] if "l_rpm_tgt_x10" in buckets else []
    r_rpm_tgt = [v / 10.0 for v in buckets.get("r_rpm_tgt_x10", [])] if "r_rpm_tgt_x10" in buckets else []

    # Duty percent (if available)
    l_duty = duty_from_buffer(buckets, ["l_duty_pct", "l_duty", "pwm_left", "pwm_l"])
    r_duty = duty_from_buffer(buckets, ["r_duty_pct", "r_duty", "pwm_right", "pwm_r"])

    # Currents (mA -> A)
    l_cur = [v / 1000.0 for v in buckets.get("l_mA", [])] if "l_mA" in buckets else []
    r_cur = [v / 1000.0 for v in buckets.get("r_mA", [])] if "r_mA" in buckets else []

    line_rpm_l.set_data(t[:len(l_rpm)], l_rpm)
    line_rpm_r.set_data(t[:len(r_rpm)], r_rpm)
    line_rpm_l_tgt.set_data(t[:len(l_rpm_tgt)], l_rpm_tgt)
    line_rpm_r_tgt.set_data(t[:len(r_rpm_tgt)], r_rpm_tgt)

    line_duty_l.set_data(t[:len(l_duty)], l_duty)
    line_duty_r.set_data(t[:len(r_duty)], r_duty)

    line_curr_l.set_data(t[:len(l_cur)], l_cur)
    line_curr_r.set_data(t[:len(r_cur)], r_cur)

    for ax in axes:
      ax.relim()
      ax.autoscale_view()

    plt.pause(REFRESH_INTERVAL)

except KeyboardInterrupt:
  print("\nStopped by user.")
finally:
  ser.close()
  plt.ioff()
  plt.show()
