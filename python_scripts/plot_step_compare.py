import csv
import math
import argparse
import matplotlib.pyplot as plt


def read_csv(path):
    headers = []
    rows = []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        for raw in reader:
            if not raw:
                continue
            if raw[0].startswith("#HEADER:"):
                # parse header after #HEADER:
                line = ",".join(raw)[len("#HEADER:"):]
                parts = [p.strip() for p in line.split(",")]
                numeric = [p for p in parts if "=" not in p and p]
                headers = numeric
            elif raw[0].startswith("#"):
                continue
            else:
                if not headers:
                    # assume first non-comment row is header
                    headers = [h.strip() for h in raw]
                    continue
                if len(raw) != len(headers):
                    continue
                row = {h: raw[i].strip() for i, h in enumerate(headers)}
                rows.append(row)
    return headers, rows


def get_float(row, key, default=float("nan")):
    try:
        return float(row.get(key, default))
    except Exception:
        try:
            return float(str(row.get(key, default)).strip())
        except Exception:
            return default


def extract_series(headers, rows):
    # tolerant field naming
    t_key = next((k for k in ("t", "time") if k in headers), None)
    cmd_key = next((k for k in ("w_cmd", "cmd", "setpoint") if k in headers), None)
    meas_key = next((k for k in ("w_meas", "meas") if k in headers), None)

    if not (t_key and cmd_key and meas_key):
        raise RuntimeError("Missing required fields: need time (t), cmd/w_cmd, meas/w_meas")

    t = [get_float(r, t_key) for r in rows]
    cmd = [get_float(r, cmd_key) for r in rows]
    meas = [get_float(r, meas_key) for r in rows]
    return t, cmd, meas


def metrics(t, cmd, meas, tol=0.02):
    # assume step from initial cmd[0] to final cmd_end
    if not t:
        return {}
    t0 = t[0]
    y0 = meas[0] if meas else 0.0
    u0 = cmd[0] if cmd else 0.0
    uf = cmd[-1] if cmd else 0.0
    step = uf - u0
    if abs(step) < 1e-6:
        # try using mean of last 20% as reference
        n = max(1, len(cmd)//5)
        uf = sum(cmd[-n:]) / n
        step = uf - u0

    # rise time 10-90%
    y10 = y0 + 0.1 * step
    y90 = y0 + 0.9 * step
    t10 = next((t[i] for i, y in enumerate(meas) if (y - y0) * math.copysign(1, step) >= (y10 - y0) * math.copysign(1, step)), float("nan"))
    t90 = next((t[i] for i, y in enumerate(meas) if (y - y0) * math.copysign(1, step) >= (y90 - y0) * math.copysign(1, step)), float("nan"))

    # overshoot
    peak = max(meas) if step >= 0 else min(meas)
    overshoot = (peak - uf) / step * 100.0 if step != 0 else float("nan")

    # steady-state error (mean of last 10%)
    n = max(1, len(meas)//10)
    yss = sum(meas[-n:]) / n
    sse = (uf - yss)

    # settling time (within tol of uf)
    band = abs(step) * tol
    ts = float("nan")
    for i in range(len(meas)-1, -1, -1):
        if abs(meas[i] - uf) > band:
            if i+1 < len(t):
                ts = t[i+1]
            break
    return {
        "rise_time": (t90 - t10) if (not math.isnan(t10) and not math.isnan(t90)) else float("nan"),
        "t10": t10,
        "t90": t90,
        "overshoot_percent": overshoot,
        "settling_time": ts,
        "sse": sse,
        "uf": uf,
    }


def main():
    ap = argparse.ArgumentParser(description="Compare step responses of single-loop vs cascaded controllers from CSV logs")
    ap.add_argument("csv_single", help="CSV log: single-loop PID")
    ap.add_argument("csv_cascade", help="CSV log: cascaded current+speed")
    ap.add_argument("--title", default="PID vs Cascaded Comparison")
    args = ap.parse_args()

    h1, r1 = read_csv(args.csv_single)
    h2, r2 = read_csv(args.csv_cascade)

    t1, c1, y1 = extract_series(h1, r1)
    t2, c2, y2 = extract_series(h2, r2)

    m1 = metrics(t1, c1, y1)
    m2 = metrics(t2, c2, y2)

    fig, ax = plt.subplots(1, 1, figsize=(10, 5))
    ax.set_title(args.title)
    ax.plot(t1, c1, 'k--', alpha=0.5, label='Setpoint (single)')
    ax.plot(t1, y1, 'r-', label='Measured (single)')
    ax.plot(t2, y2, 'b-', label='Measured (cascaded)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (RPM)')
    ax.grid(True)
    ax.legend(loc='best')

    txt = (
        f"Single: rise={m1['rise_time']:.3f}s, overshoot={m1['overshoot_percent']:.1f}%, set={m1['settling_time']:.3f}s, SSE={m1['sse']:.2f}\n"
        f"Cascade: rise={m2['rise_time']:.3f}s, overshoot={m2['overshoot_percent']:.1f}%, set={m2['settling_time']:.3f}s, SSE={m2['sse']:.2f}"
    )
    fig.text(0.02, 0.02, txt, fontsize=9, family='monospace')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

