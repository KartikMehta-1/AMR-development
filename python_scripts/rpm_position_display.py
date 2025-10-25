import serial
import matplotlib.pyplot as plt
from collections import deque
import time

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
while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line.startswith("#HEADER:"):
        headers = [h.strip() for h in line.replace("#HEADER:", "").split(",")]
        print(f"Received header: {headers}")
        break

# Initialize data buffers
buffers = {h: deque(maxlen=MAX_POINTS) for h in headers}

# =========================
# SETUP 2-COLUMN PLOTS
# =========================
plt.ion()
fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
fig.suptitle("Live PID Telemetry (460800 baud)", fontsize=14, fontweight="bold")

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

plt.tight_layout(rect=[0, 0, 1, 0.97])
print("Streaming data... Press Ctrl+C to stop.\n")

# =========================
# STREAM + PLOT LOOP
# =========================
try:
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line or line.startswith("#"):
            continue

        values = line.split(",")
        if len(values) != len(headers):
            continue

        # Parse values into float dictionary
        data = dict(zip(headers, map(float, values)))
        for h in headers:
            buffers[h].append(data[h])

        # Extract time axis
        t = list(buffers["t"])
        if len(t) < 2:
            continue

        # Extract key data lists
        cmd = list(buffers["cmd"])
        meas = list(buffers["meas"])
        err = list(buffers["err"])
        pwm = list(buffers["pwm"])
        p = list(buffers["p"])
        i = list(buffers["i"])
        d = list(buffers["d"])
        pid_sum = [p[j] + i[j] + d[j] for j in range(len(p))]

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
