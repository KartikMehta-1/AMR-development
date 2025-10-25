import serial
import matplotlib.pyplot as plt
from collections import deque
import time
import math

# =========================
# USER CONFIG
# =========================
PORT = "COM4"           # <-- change to your STM32's COM port
BAUD = 460800           # updated baud rate
MAX_POINTS = 400        # number of samples shown
REFRESH_INTERVAL = 0.1  # seconds between plot updates

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
test_name = None

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        continue
    if line.startswith("#HEADER:"):
        # remove prefix and split on commas
        parts = [p.strip() for p in line.replace("#HEADER:", "").split(",")]
        numeric_headers = []
        # extract any key=value pairs (e.g., test=Step) and remove them from numeric header list
        for p in parts:
            if "=" in p:
                k, v = p.split("=", 1)
                k = k.strip()
                v = v.strip()
                if k.lower() == "test":
                    test_name = v
                # ignore any key=value for numeric headers
            else:
                if p: numeric_headers.append(p)
        headers = numeric_headers
        print(f"Received header (test={test_name}): {headers}")
        break

if not headers:
    raise SystemExit("No headers received. Check your STM telemetry '#HEADER:' output.")

# Initialize data buffers
buffers = {h: deque(maxlen=MAX_POINTS) for h in headers}

# =========================
# SETUP 2-COLUMN PLOTS
# =========================
plt.ion()
fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
fig.suptitle(f"Live PID Telemetry (baud={BAUD})    test={test_name}", fontsize=14, fontweight="bold")

# Flatten axes for easier access
(ax_rpm, ax_p,
 ax_pwm_pid, ax_i,
 ax_err, ax_d) = axes.flatten()

# ---------- LEFT COLUMN ----------

# 1️⃣ Target vs Measured RPM
line_cmd, = ax_rpm.plot([], [], 'b--', label="Target RPM")
line_meas, = ax_rpm.plot([], [], 'r-', label="Measured RPM")
ax_rpm.set_ylabel("RPM")
ax_rpm.legend(loc="upper right")
ax_rpm.grid(True)

# 2️⃣ PWM vs P+I+D
line_pwm, = ax_pwm_pid.plot([], [], 'g-', label="PWM Output")
line_pid_sum, = ax_pwm_pid.plot([], [], 'k--', label="P+I+D Sum")
ax_pwm_pid.set_ylabel("PWM / PID Sum")
ax_pwm_pid.legend(loc="upper right")
ax_pwm_pid.grid(True)

# 3️⃣ Error
line_err, = ax_err.plot([], [], 'm-', label="Error (RPM)")
ax_err.set_ylabel("Error")
ax_err.set_xlabel("Time (s)")
ax_err.legend(loc="upper right")
ax_err.grid(True)

# ---------- RIGHT COLUMN ----------

# 4️⃣ P Term
line_p, = ax_p.plot([], [], 'c-', label="P Term")
ax_p.set_ylabel("P")
ax_p.legend(loc="upper right")
ax_p.grid(True)

# 5️⃣ I Term
line_i, = ax_i.plot([], [], 'y-', label="I Term")
ax_i.set_ylabel("I")
ax_i.legend(loc="upper right")
ax_i.grid(True)

# 6️⃣ D Term
line_d, = ax_d.plot([], [], 'orange', label="D Term")
ax_d.set_ylabel("D")
ax_d.set_xlabel("Time (s)")
ax_d.legend(loc="upper right")
ax_d.grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.95])
print("Streaming data... Press Ctrl+C to stop.\n")

# Helper: safe float conversion (non-numeric -> nan)
def safe_float(s):
    try:
        return float(s)
    except Exception:
        # handle empty or non-numeric fields gracefully
        try:
            # try to handle integers with stray chars
            return float(s.strip())
        except Exception:
            return float("nan")

# Quick check that required fields exist (not strictly necessary but nice)
required = {"t", "cmd", "meas", "err", "p", "i", "d", "pwm"}
missing = required - set(headers)
if missing:
    print("Warning: some expected fields are missing from header:", missing)
    # we will still try to plot what we can

# =========================
# STREAM + PLOT LOOP
# =========================
try:
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue

        # ignore header echo lines; handle updated headers dynamically if needed
        if line.startswith("#HEADER:"):
            # optional: handle header re-send mid-stream - simple approach: ignore or rebuild
            # For simplicity we ignore repeated headers (you can add logic to rebuild buffers here)
            continue

        # skip comments
        if line.startswith("#"):
            continue

        values = [v.strip() for v in line.split(",")]
        if len(values) != len(headers):
            # mismatch — ignore this row (could be spurious)
            # optionally, print for debugging:
            # print("Skipping line (len mismatch):", line)
            continue

        # parse floats into dict
        parsed = {}
        for h, v in zip(headers, values):
            parsed[h] = safe_float(v)

        # append into buffers (only fields present in header)
        for h in headers:
            buffers[h].append(parsed[h])

        # need at least two samples to plot
        if "t" not in buffers or len(buffers["t"]) < 2:
            continue

        # Extract time axis and lists (use get with default zeros if missing)
        t = list(buffers.get("t", []))
        cmd = list(buffers.get("cmd", [float("nan")] * len(t)))
        meas = list(buffers.get("meas", [float("nan")] * len(t)))
        err = list(buffers.get("err", [float("nan")] * len(t)))
        pwm = list(buffers.get("pwm", [float("nan")] * len(t)))
        p = list(buffers.get("p", [float("nan")] * len(t)))
        i = list(buffers.get("i", [float("nan")] * len(t)))
        d = list(buffers.get("d", [float("nan")] * len(t)))

        # Compute pid sum safely (handle NaNs)
        pid_sum = []
        for j in range(len(t)):
            pj = p[j] if j < len(p) else float("nan")
            ij = i[j] if j < len(i) else float("nan")
            dj = d[j] if j < len(d) else float("nan")
            s = 0.0
            s += 0.0 if math.isnan(pj) else pj
            s += 0.0 if math.isnan(ij) else ij
            s += 0.0 if math.isnan(dj) else dj
            pid_sum.append(s)

        # --- Update plots ---
        line_cmd.set_data(t, cmd)
        line_meas.set_data(t, meas)
        line_pwm.set_data(t, pwm)
        line_pid_sum.set_data(t, pid_sum)
        line_err.set_data(t, err)
        line_p.set_data(t, p)
        line_i.set_data(t, i)
        line_d.set_data(t, d)

        # Auto-scale each axis
        for ax in axes.flatten():
            ax.relim()
            ax.autoscale_view()

        plt.pause(REFRESH_INTERVAL)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    ser.close()
    plt.ioff()
    plt.show()
