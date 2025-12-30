"""
Live PID tuning plotter for STM UART telemetry.

Expected fields in #HEADER (subset ok):
  t_ms,l_rpm_x10,r_rpm_x10,l_rpm_tgt_x10,r_rpm_tgt_x10,
  l_duty_pct,r_duty_pct,
  l_pid_p_pct,r_pid_p_pct,l_pid_i_pct,r_pid_i_pct,l_pid_d_pct,r_pid_d_pct,
  l_pid_err_x10,r_pid_err_x10,l_mA,r_mA
"""

from collections import deque
import time

import matplotlib.pyplot as plt
import serial

# =========================
# USER CONFIG
# =========================
PORT = "/dev/ttyACM0"    # change to your STM32 serial port
BAUD = 460800            # must match firmware UART2
MAX_POINTS = 400         # number of samples shown (>= window * sample_rate)
REFRESH_INTERVAL = 0.05  # seconds between plot refresh
WINDOW_SEC = 20.0        # show last N seconds


def safe_float(val):
    try:
        return float(val)
    except Exception:
        try:
            return float(str(val).strip())
        except Exception:
            return float("nan")


def normalize_time(buffers):
    if "t_ms" in buffers and len(buffers["t_ms"]) > 0:
        t0 = buffers["t_ms"][0]
        return [(v - t0) / 1000.0 for v in buffers["t_ms"]]
    if "t" in buffers and len(buffers["t"]) > 0:
        t0 = buffers["t"][0]
        return [v - t0 for v in buffers["t"]]
    n = len(next(iter(buffers.values())))
    return list(range(n))


def get_scaled(buffers, key, scale):
    if key not in buffers:
        return []
    return [v / scale for v in buffers[key]]


def duty_from_buffer(buffers, keys):
    for k in keys:
        if k not in buffers:
            continue
        arr = list(buffers[k])
        if any((v is not None) and (v > 1.5) for v in arr):
            return arr  # already in percent
        return [v * 100.0 for v in arr]  # 0.0-1.0 -> percent
    return []

def window_slice(arr, start_idx):
    if start_idx <= 0:
        return arr
    if start_idx >= len(arr):
        return []
    return arr[start_idx:]

def clamp_abs(arr, limit):
    out = []
    for v in arr:
        if v != v:
            out.append(float("nan"))
        elif abs(v) > limit:
            out.append(float("nan"))
        else:
            out.append(v)
    return out

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
print("Waiting for header...")

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

buckets = {h: deque(maxlen=MAX_POINTS) for h in headers}

plt.ion()
fig, axes = plt.subplots(5, 1, figsize=(11, 12), sharex=True)
fig.suptitle(f"PID Tuning Telemetry (baud={BAUD})", fontsize=14, fontweight="bold")

ax_rpm, ax_duty, ax_err, ax_pid, ax_curr = axes

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

line_err_l, = ax_err.plot([], [], "r-", label="L Err (RPM)")
line_err_r, = ax_err.plot([], [], "b-", label="R Err (RPM)")
ax_err.set_ylabel("Error (RPM)")
ax_err.legend(loc="upper right")
ax_err.grid(True)

line_p_l, = ax_pid.plot([], [], "r-", label="P L (%)")
line_p_r, = ax_pid.plot([], [], "r--", label="P R (%)")
line_i_l, = ax_pid.plot([], [], "g-", label="I L (%)")
line_i_r, = ax_pid.plot([], [], "g--", label="I R (%)")
line_d_l, = ax_pid.plot([], [], "k-", label="D L (%)")
line_d_r, = ax_pid.plot([], [], "k--", label="D R (%)")
ax_pid.set_ylabel("PID Terms (%)")
ax_pid.set_xlabel("Time (s)")
ax_pid.legend(loc="upper right", ncol=3)
ax_pid.grid(True)

line_curr_l, = ax_curr.plot([], [], "r-", label="L Current (A)")
line_curr_r, = ax_curr.plot([], [], "b-", label="R Current (A)")
ax_curr.set_ylabel("Current (A)")
ax_curr.set_xlabel("Time (s)")
ax_curr.legend(loc="upper right")
ax_curr.grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show(block=False)
fig.canvas.draw()
fig.canvas.flush_events()
plt.pause(0.1)
print("Streaming data... Press Ctrl+C to stop.\n")

good_samples = 0
last_title_update = time.time()
last_t_ms = None

try:
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            plt.pause(REFRESH_INTERVAL)
            continue
        if line.startswith("#HEADER:") or line.startswith("#"):
            plt.pause(REFRESH_INTERVAL)
            continue

        values = [v.strip() for v in line.split(",")]
        if len(values) < len(headers):
            values += ["nan"] * (len(headers) - len(values))
        elif len(values) > len(headers):
            values = values[:len(headers)]
        parsed = {h: safe_float(v) for h, v in zip(headers, values)}
        t_ms = parsed.get("t_ms")
        if t_ms is None or t_ms != t_ms:
            plt.pause(REFRESH_INTERVAL)
            continue
        if last_t_ms is not None and t_ms <= last_t_ms:
            plt.pause(REFRESH_INTERVAL)
            continue
        last_t_ms = t_ms
        for h in headers:
            buckets[h].append(parsed[h])
        good_samples += 1

        if len(next(iter(buckets.values()))) < 2:
            plt.pause(REFRESH_INTERVAL)
            continue

        t_full = normalize_time(buckets)
        if not t_full:
            plt.pause(REFRESH_INTERVAL)
            continue
        t_end = t_full[-1]
        if t_end != t_end:  # NaN guard
            plt.pause(REFRESH_INTERVAL)
            continue
        t_start = t_end - WINDOW_SEC
        if t_start < 0.0:
            t_start = 0.0
        start_idx = 0
        for i, tv in enumerate(t_full):
            if tv >= t_start:
                start_idx = i
                break
        t = t_full[start_idx:]

        l_rpm = window_slice(get_scaled(buckets, "l_rpm_x10", 10.0), start_idx)
        r_rpm = window_slice(get_scaled(buckets, "r_rpm_x10", 10.0), start_idx)
        l_rpm_tgt = window_slice(get_scaled(buckets, "l_rpm_tgt_x10", 10.0), start_idx)
        r_rpm_tgt = window_slice(get_scaled(buckets, "r_rpm_tgt_x10", 10.0), start_idx)

        l_duty = window_slice(duty_from_buffer(buckets, ["l_duty_pct", "l_duty"]), start_idx)
        r_duty = window_slice(duty_from_buffer(buckets, ["r_duty_pct", "r_duty"]), start_idx)

        l_err = clamp_abs(window_slice(get_scaled(buckets, "l_pid_err_x10", 10.0), start_idx), 500.0)
        r_err = clamp_abs(window_slice(get_scaled(buckets, "r_pid_err_x10", 10.0), start_idx), 500.0)

        l_p = window_slice(get_scaled(buckets, "l_pid_p_pct", 1.0), start_idx)
        r_p = window_slice(get_scaled(buckets, "r_pid_p_pct", 1.0), start_idx)
        l_i = window_slice(get_scaled(buckets, "l_pid_i_pct", 1.0), start_idx)
        r_i = window_slice(get_scaled(buckets, "r_pid_i_pct", 1.0), start_idx)
        l_d = window_slice(get_scaled(buckets, "l_pid_d_pct", 1.0), start_idx)
        r_d = window_slice(get_scaled(buckets, "r_pid_d_pct", 1.0), start_idx)
        l_cur = window_slice(get_scaled(buckets, "l_mA", 1000.0), start_idx)
        r_cur = window_slice(get_scaled(buckets, "r_mA", 1000.0), start_idx)

        line_rpm_l.set_data(t[:len(l_rpm)], l_rpm)
        line_rpm_r.set_data(t[:len(r_rpm)], r_rpm)
        line_rpm_l_tgt.set_data(t[:len(l_rpm_tgt)], l_rpm_tgt)
        line_rpm_r_tgt.set_data(t[:len(r_rpm_tgt)], r_rpm_tgt)

        line_duty_l.set_data(t[:len(l_duty)], l_duty)
        line_duty_r.set_data(t[:len(r_duty)], r_duty)

        line_err_l.set_data(t[:len(l_err)], l_err)
        line_err_r.set_data(t[:len(r_err)], r_err)

        line_p_l.set_data(t[:len(l_p)], l_p)
        line_p_r.set_data(t[:len(r_p)], r_p)
        line_i_l.set_data(t[:len(l_i)], l_i)
        line_i_r.set_data(t[:len(r_i)], r_i)
        line_d_l.set_data(t[:len(l_d)], l_d)
        line_d_r.set_data(t[:len(r_d)], r_d)

        line_curr_l.set_data(t[:len(l_cur)], l_cur)
        line_curr_r.set_data(t[:len(r_cur)], r_cur)

        for ax in axes:
            ax.relim()
            ax.autoscale_view(scalex=False)
            if t_end > WINDOW_SEC:
                ax.set_xlim(t_end - WINDOW_SEC, t_end)
            else:
                ax.set_xlim(0.0, WINDOW_SEC)

        now = time.time()
        if (now - last_title_update) >= 1.0:
            fig.suptitle(f"PID Tuning Telemetry (baud={BAUD}) samples={good_samples}", fontsize=14, fontweight="bold")
            last_title_update = now

        plt.pause(REFRESH_INTERVAL)

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    ser.close()
    plt.ioff()
    plt.show()
