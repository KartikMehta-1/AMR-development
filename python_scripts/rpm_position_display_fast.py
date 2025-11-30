"""
Live telemetry plotter using fastplotlib.
Parses UART lines like:
#HEADER: t_ms,l_cnt,r_cnt,l_rpm_x10,r_rpm_x10,l_duty_pct,r_duty_pct,l_mA,r_mA
"""

import asyncio
from collections import deque
import math
import time

import fastplotlib as fpl
import numpy as np
import serial

PORT = "COM4"            # change to your STM32 COM port
BAUD = 460800
MAX_POINTS = 400
POLL_SLEEP = 0.001


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
        return np.array([(v - t0) / 1000.0 for v in buffers["t_ms"]], dtype=np.float32)
    if "t" in buffers and len(buffers["t"]) > 0:
        t0 = buffers["t"][0]
        return np.array([v - t0 for v in buffers["t"]], dtype=np.float32)
    n = len(next(iter(buffers.values())))
    return np.arange(n, dtype=np.float32)


def duty_from_buffer(buffers, keys):
    for k in keys:
        if k not in buffers:
            continue
        arr = np.array(buffers[k], dtype=np.float32)
        if np.any(~np.isnan(arr) & (arr > 1.5)):
            return arr  # percent already
        return arr * 100.0  # 0-1 -> percent
    return np.array([], dtype=np.float32)


async def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.5)
    print("Waiting for header...")

    headers = []
    meta = {}
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            await asyncio.sleep(POLL_SLEEP)
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
        raise SystemExit("No headers received.")

    buckets = {h: deque(maxlen=MAX_POINTS) for h in headers}

    fig = fpl.Figure(shape=(3, 1), size=(900, 700))
    fig.title = f"Live Telemetry (baud={BAUD}) test={meta.get('test','')}"
    fig[0, 0].axes.y_label = "RPM"
    fig[1, 0].axes.y_label = "Duty (%)"
    fig[2, 0].axes.y_label = "Current (A)"
    fig[2, 0].axes.x_label = "Time (s)"

    # Invisible anchors to give camera bounds
    for cell in [(0, 0), (1, 0), (2, 0)]:
        fig[cell].add_scatter(
            data=np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
            sizes=0.0,
            colors="#00000000",
        )

    def make_line(cell, color):
        # Fixed-size buffer; we'll update in-place without changing shape
        init = np.full((MAX_POINTS, 3), np.nan, dtype=np.float32)
        return fig[cell].add_line(data=init, colors=color)

    line_rpm_l = make_line((0, 0), "#d62728")
    line_rpm_r = make_line((0, 0), "#1f77b4")
    line_duty_l = make_line((1, 0), "#d62728")
    line_duty_r = make_line((1, 0), "#1f77b4")
    line_curr_l = make_line((2, 0), "#d62728")
    line_curr_r = make_line((2, 0), "#1f77b4")

    fig.show()
    print("Streaming data... Press Ctrl+C to stop.\n")

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                await asyncio.sleep(POLL_SLEEP)
                continue
            if line.startswith("#HEADER:") or line.startswith("#"):
                continue

            values = [v.strip() for v in line.split(",")]
            if len(values) != len(headers):
                continue

            parsed = {h: safe_float(v) for h, v in zip(headers, values)}
            for h in headers:
                buckets[h].append(parsed[h])

            if len(next(iter(buckets.values()))) < 2:
                continue

            t = normalize_time(buckets)
            l_rpm = np.array([v / 10.0 for v in buckets.get("l_rpm_x10", [])], dtype=np.float32) if "l_rpm_x10" in buckets else np.array([], dtype=np.float32)
            r_rpm = np.array([v / 10.0 for v in buckets.get("r_rpm_x10", [])], dtype=np.float32) if "r_rpm_x10" in buckets else np.array([], dtype=np.float32)
            l_duty = duty_from_buffer(buckets, ["l_duty_pct", "l_duty", "pwm_left", "pwm_l"])
            r_duty = duty_from_buffer(buckets, ["r_duty_pct", "r_duty", "pwm_right", "pwm_r"])
            l_cur = np.array([v / 1000.0 for v in buckets.get("l_mA", [])], dtype=np.float32) if "l_mA" in buckets else np.array([], dtype=np.float32)
            r_cur = np.array([v / 1000.0 for v in buckets.get("r_mA", [])], dtype=np.float32) if "r_mA" in buckets else np.array([], dtype=np.float32)

            def update_line(line, xvals, yvals):
                n = min(xvals.size, yvals.size, MAX_POINTS)
                if n == 0:
                    return
                buf = line.data  # feature buffer (MAX_POINTS, 3)
                buf[:] = np.nan
                buf[:n, 0] = xvals[:n]
                buf[:n, 1] = yvals[:n]
                buf[:n, 2] = 0.0
                line.data = buf  # push updated positions into the graphic

            update_line(line_rpm_l, t, l_rpm)
            update_line(line_rpm_r, t, r_rpm)
            update_line(line_duty_l, t, l_duty)
            update_line(line_duty_r, t, r_duty)
            update_line(line_curr_l, t, l_cur)
            update_line(line_curr_r, t, r_cur)

            # Slide x-axis to recent data window
            if t.size > 1:
                x0 = float(t[0])
                x1 = float(t[-1])
                fig[0, 0].axes.x_limits = (x0, x1)
                fig[1, 0].axes.x_limits = (x0, x1)
                fig[2, 0].axes.x_limits = (x0, x1)

            await asyncio.sleep(0)  # yield to renderer

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()
        fig.close()


if __name__ == "__main__":
    asyncio.run(main())
