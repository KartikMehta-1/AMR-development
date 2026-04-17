"""
Quick checker for RPM step response overshoot.

Usage:
  python check_overshoot.py --log path/to/log.csv [--save plot.png] [--kp 0.03 --ki 0.05 --kd 0]

Input log is the UART CSV with header lines like:
#HEADER: t_ms,l_cnt,r_cnt,l_rpm_x10,r_rpm_x10,l_rpm_tgt_x10,r_rpm_tgt_x10,...

Outputs:
- Prints max overshoot (%) per wheel and pass/fail vs 10% criterion.
- Optionally saves a plot with targets vs actual RPM and duty.
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def parse_log(path):
    headers = []
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#HEADER:"):
                hdr = line.replace("#HEADER:", "").strip()
                headers = [h.strip() for h in hdr.split(",") if h.strip()]
                continue
            if line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if headers and len(parts) != len(headers):
                continue
            if not headers:
                # fallback: skip if we don't know the schema
                continue
            rows.append({k: p for k, p in zip(headers, parts)})
    return headers, rows


def as_float(rows, key, scale=1.0):
    out = []
    for r in rows:
        try:
            out.append(float(r[key]) * scale)
        except Exception:
            out.append(math.nan)
    return out


def find_steps(targets, tol=0.05):
    """Return list of (start_idx, end_idx, target_value)."""
    steps = []
    if not targets:
        return steps
    current = targets[0]
    start = 0
    for i, v in enumerate(targets[1:], start=1):
        if abs(v - current) > tol:
            steps.append((start, i - 1, current))
            start = i
            current = v
    steps.append((start, len(targets) - 1, current))
    return steps


def compute_overshoot(targets, meas):
    steps = find_steps(targets)
    overs = []
    for s, e, tgt in steps:
        if tgt <= 0.1:
            continue
        peak = max(meas[s:e+1]) if e >= s else meas[s]
        os_pct = ((peak - tgt) / tgt) * 100.0
        overs.append(os_pct)
    return overs, steps


def plot_data(t, l_rpm, r_rpm, l_tgt, r_tgt, l_duty, r_duty, save_path=None, kp=None, ki=None, kd=None):
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax_rpm, ax_duty = axes
    ax_rpm.plot(t, l_rpm, "r-", label="L RPM")
    ax_rpm.plot(t, r_rpm, "b-", label="R RPM")
    ax_rpm.plot(t, l_tgt, "r--", label="L RPM tgt")
    ax_rpm.plot(t, r_tgt, "b--", label="R RPM tgt")
    if kp is not None:
        ax_rpm.set_title(f"Speed PI: kp={kp}, ki={ki}, kd={kd}")
    ax_rpm.set_ylabel("RPM")
    ax_rpm.legend(loc="upper right")
    ax_rpm.grid(True)

    ax_duty.plot(t, l_duty, "r-", label="L Duty (%)")
    ax_duty.plot(t, r_duty, "b-", label="R Duty (%)")
    ax_duty.set_ylabel("Duty (%)")
    ax_duty.set_xlabel("Time (s)")
    ax_duty.legend(loc="upper right")
    ax_duty.grid(True)

    fig.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
    else:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description="Check RPM overshoot from telemetry log.")
    ap.add_argument("--log", required=True, help="Path to UART CSV log")
    ap.add_argument("--save", help="Optional path to save plot (png)")
    ap.add_argument("--kp", type=float, help="Gain metadata for plot title")
    ap.add_argument("--ki", type=float, help="Gain metadata for plot title")
    ap.add_argument("--kd", type=float, help="Gain metadata for plot title")
    args = ap.parse_args()

    headers, rows = parse_log(Path(args.log))
    if not rows:
        print("No data rows parsed.")
        return

    t_ms = as_float(rows, "t_ms")
    t = [(v - t_ms[0]) / 1000.0 for v in t_ms]
    l_rpm = as_float(rows, "l_rpm_x10", scale=0.1)
    r_rpm = as_float(rows, "r_rpm_x10", scale=0.1)
    l_tgt = as_float(rows, "l_rpm_tgt_x10", scale=0.1)
    r_tgt = as_float(rows, "r_rpm_tgt_x10", scale=0.1)
    l_duty = as_float(rows, "l_duty_pct")
    r_duty = as_float(rows, "r_duty_pct")

    l_over, _ = compute_overshoot(l_tgt, l_rpm)
    r_over, _ = compute_overshoot(r_tgt, r_rpm)
    l_max = max(l_over) if l_over else 0.0
    r_max = max(r_over) if r_over else 0.0

    def verdict(val):
        return "PASS" if val <= 10.0 else "FAIL"

    print(f"Left overshoot max: {l_max:.1f}% -> {verdict(l_max)}")
    print(f"Right overshoot max: {r_max:.1f}% -> {verdict(r_max)}")

    plot_data(t, l_rpm, r_rpm, l_tgt, r_tgt, l_duty, r_duty,
              save_path=args.save, kp=args.kp, ki=args.ki, kd=args.kd)


if __name__ == "__main__":
    main()
