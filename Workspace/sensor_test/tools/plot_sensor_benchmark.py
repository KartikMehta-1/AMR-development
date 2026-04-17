#!/usr/bin/env python3
"""
Plot Arduino sensor benchmark CSV and optionally merge PicoScope CSV.

Usage:
  python3 Workspace/sensor_test/tools/plot_sensor_benchmark.py sensor_test_run.csv
  python3 Workspace/sensor_test/tools/plot_sensor_benchmark.py sensor_test_run.csv \
      --pico-csv pico_export.csv \
      --pico-col-acs758 "Channel A" \
      --pico-col-acs70331 "Channel B" \
      --pico-col-trig "Channel C" \
      --out merged.png --show
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import re
import statistics
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt


def _getf(d: Dict[str, str], key: str) -> float:
    try:
        return float(d.get(key, "nan"))
    except Exception:
        return math.nan


def _clean_cell(s: str) -> str:
    # Remove embedded NULs and trim whitespace. Keep commas handled by csv module.
    return s.replace("\x00", "").strip()


def parse_arduino_csv(path: pathlib.Path) -> List[Dict[str, str]]:
    headers: List[str] = []
    rows: List[Dict[str, str]] = []

    # Serial captures may contain non-UTF bytes; replace undecodable bytes safely.
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        for raw in reader:
            if not raw:
                continue
            raw = [_clean_cell(x) for x in raw]
            first = raw[0]
            if first.startswith("#HEADER:"):
                line = ",".join(raw)
                headers = [h.strip() for h in line[len("#HEADER:") :].split(",")]
                continue
            if first.startswith("#"):
                continue

            if not headers:
                headers = [h.strip() for h in raw]
                continue

            if len(raw) < len(headers):
                raw = raw + ([""] * (len(headers) - len(raw)))
            elif len(raw) > len(headers):
                raw = raw[: len(headers)]

            row = {headers[i]: raw[i].strip() for i in range(len(headers))}
            rows.append(row)

    if not rows:
        raise RuntimeError(f"No data rows parsed from Arduino CSV: {path}")

    return rows


def _normalize_header(h: str) -> str:
    h = h.strip().lower()
    h = re.sub(r"\s+", " ", h)
    return h


def _unit_scale_from_header(header: str, kind: str) -> float:
    """
    Infer scaling to SI base units from header text.
    kind='time' -> seconds, kind='volt' -> volts.
    """
    h = _normalize_header(header)

    # Common unit patterns from Pico CSV exports.
    # Support "(mV)", "[mV]", " mV" style.
    if kind == "time":
        if re.search(r"[\(\[]\s*ns\s*[\)\]]|\bns\b", h):
            return 1e-9
        if re.search(r"[\(\[]\s*us\s*[\)\]]|\bus\b", h):
            return 1e-6
        if re.search(r"[\(\[]\s*ms\s*[\)\]]|\bms\b", h):
            return 1e-3
        # default assume seconds
        return 1.0

    if kind == "volt":
        if re.search(r"[\(\[]\s*uv\s*[\)\]]|\buv\b", h):
            return 1e-6
        if re.search(r"[\(\[]\s*mv\s*[\)\]]|\bmv\b", h):
            return 1e-3
        if re.search(r"[\(\[]\s*kv\s*[\)\]]|\bkv\b", h):
            return 1e3
        # default assume volts
        return 1.0

    return 1.0


def _find_header_index(rows: List[List[str]]) -> int:
    """Find first likely header row in Pico CSV.

    Heuristic: row has >=2 cols and one contains 'time'.
    """
    for i, r in enumerate(rows):
        if len(r) < 2:
            continue
        norm = [_normalize_header(c) for c in r]
        if any("time" in c for c in norm):
            return i
    raise RuntimeError("Could not find Pico CSV header row containing a time column.")


def parse_pico_csv(path: pathlib.Path) -> Tuple[List[str], List[Dict[str, float]]]:
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        raw_rows = list(csv.reader(f))

    if not raw_rows:
        raise RuntimeError(f"Empty Pico CSV: {path}")

    hdr_idx = _find_header_index(raw_rows)
    headers = [_clean_cell(c) for c in raw_rows[hdr_idx]]
    data_rows = raw_rows[hdr_idx + 1 :]

    # Pico exports often store units in a dedicated row right after header:
    # e.g. (s),(V),(mV),(V). Fold this into header names so downstream can infer scale.
    if data_rows:
        maybe_units = [_clean_cell(c) for c in data_rows[0]]
        unit_like = len(maybe_units) == len(headers) and all(
            (u == "") or bool(re.match(r"^\(.*\)$", u)) for u in maybe_units
        )
        if unit_like and any(u != "" for u in maybe_units):
            headers = [
                f"{headers[i]} {maybe_units[i]}".strip()
                for i in range(len(headers))
            ]
            data_rows = data_rows[1:]

    parsed: List[Dict[str, float]] = []
    for r in data_rows:
        if len(r) < len(headers):
            r = r + ([""] * (len(headers) - len(r)))
        elif len(r) > len(headers):
            r = r[: len(headers)]
        r = [_clean_cell(x) for x in r]
        row: Dict[str, float] = {}
        ok = False
        for i, h in enumerate(headers):
            txt = r[i].strip()
            if txt == "":
                row[h] = math.nan
                continue
            # keep first numeric-only rows; skip unit rows etc.
            try:
                v = float(txt)
                row[h] = v
                ok = True
            except ValueError:
                row[h] = math.nan
        if ok:
            parsed.append(row)

    if not parsed:
        raise RuntimeError(f"No numeric data rows parsed from Pico CSV: {path}")

    return headers, parsed


def _resolve_col(headers: Sequence[str], wanted: Optional[str], fallback_patterns: Sequence[str]) -> str:
    norm_map = {_normalize_header(h): h for h in headers}

    if wanted:
        w = _normalize_header(wanted)
        if w in norm_map:
            return norm_map[w]
        # substring fallback
        for nh, oh in norm_map.items():
            if w in nh:
                return oh
        raise RuntimeError(f"Could not resolve Pico column '{wanted}'. Available: {list(headers)}")

    # Auto by patterns.
    for p in fallback_patterns:
        p = _normalize_header(p)
        for nh, oh in norm_map.items():
            if p in nh:
                return oh

    raise RuntimeError(
        "Could not auto-detect required Pico column. "
        f"Patterns={list(fallback_patterns)}, available={list(headers)}"
    )


def _first_rising_edge_time(t: List[float], y: List[float], thresh: float = 0.5) -> Optional[float]:
    for i in range(1, len(y)):
        if y[i - 1] < thresh <= y[i]:
            return t[i]
    return None


def _mean(vs: List[float]) -> float:
    if not vs:
        return math.nan
    return sum(vs) / len(vs)


def _rms(vs: List[float], mean_v: float) -> float:
    if not vs:
        return math.nan
    return math.sqrt(sum((x - mean_v) ** 2 for x in vs) / len(vs))


def _moving_average(values: List[float], window: int) -> List[float]:
    """
    NaN-aware centered moving average in O(n).
    Keeps output length equal to input.
    """
    n = len(values)
    if n == 0:
        return []
    if window <= 1:
        return list(values)

    w = int(window)
    if w < 1:
        w = 1
    if (w % 2) == 0:
        w += 1
    half = w // 2

    # Prefix sums for values and validity mask.
    ps = [0.0] * (n + 1)
    pm = [0] * (n + 1)
    for i, v in enumerate(values):
        if math.isnan(v):
            ps[i + 1] = ps[i]
            pm[i + 1] = pm[i]
        else:
            ps[i + 1] = ps[i] + v
            pm[i + 1] = pm[i] + 1

    out: List[float] = [math.nan] * n
    for i in range(n):
        s = max(0, i - half)
        e = min(n, i + half + 1)
        cnt = pm[e] - pm[s]
        if cnt > 0:
            out[i] = (ps[e] - ps[s]) / cnt
    return out


def _rebuild_time_seconds_from_ms(t_ms: List[float]) -> Tuple[List[float], str]:
    """
    Build a robust seconds vector even if some/many t_ms samples are NaN/corrupt.
    Returns (t_seconds, mode_description).
    """
    n = len(t_ms)
    valid_idx = [i for i, v in enumerate(t_ms) if not math.isnan(v)]
    if len(valid_idx) >= 2:
        # Estimate dt from valid time deltas.
        diffs_ms = []
        for i in range(1, len(valid_idx)):
            d = t_ms[valid_idx[i]] - t_ms[valid_idx[i - 1]]
            if d > 0:
                diffs_ms.append(d)
        dt_s = (statistics.median(diffs_ms) / 1000.0) if diffs_ms else 0.01
        if dt_s <= 0:
            dt_s = 0.01

        t: List[float] = [0.0] * n
        first = valid_idx[0]
        t0 = t_ms[first]
        # Fill known points.
        for i in valid_idx:
            t[i] = (t_ms[i] - t0) / 1000.0
        # Fill missing points by forward/backward interpolation with dt estimate.
        # Forward fill
        last = first
        for i in range(first + 1, n):
            if math.isnan(t_ms[i]):
                t[i] = t[last] + dt_s
            else:
                t[i] = (t_ms[i] - t0) / 1000.0
                last = i
        # Backward fill
        for i in range(first - 1, -1, -1):
            if math.isnan(t_ms[i]):
                t[i] = t[i + 1] - dt_s
            else:
                t[i] = (t_ms[i] - t0) / 1000.0
        return t, f"mixed(valid={len(valid_idx)}/{n}, dt~{dt_s:.4f}s)"

    # Fallback: no usable timestamp, use sample index timebase.
    dt_s = 0.01
    t = [i * dt_s for i in range(n)]
    return t, f"index_only(valid={len(valid_idx)}/{n}, dt={dt_s:.4f}s)"


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot Arduino benchmark CSV and optional Pico CSV overlay")
    ap.add_argument("csv_file", type=pathlib.Path, help="Arduino CSV captured from serial output")
    ap.add_argument("--out", type=pathlib.Path, default=None, help="Output PNG path")
    ap.add_argument("--show", action="store_true", help="Show interactive window")
    ap.add_argument("--no-smoothing", action="store_true", help="Disable smoothing of noisy traces")
    ap.add_argument("--smooth-window-arduino", type=int, default=11, help="Moving-average window for Arduino traces")
    ap.add_argument("--smooth-window-pico", type=int, default=1001, help="Moving-average window for Pico traces")
    ap.add_argument("--plot-raw", action="store_true", help="Overlay raw traces (faint) when smoothing is enabled")

    # Optional Pico merge
    ap.add_argument("--pico-csv", type=pathlib.Path, default=None, help="PicoScope CSV export")
    ap.add_argument("--pico-col-time", default=None, help="Pico time column (auto if omitted)")
    ap.add_argument("--pico-col-acs758", default=None, help="Pico column for ACS758 voltage")
    ap.add_argument("--pico-col-acs70331", default=None, help="Pico column for ACS70331 voltage")
    ap.add_argument("--pico-col-trig", default=None, help="Pico column for D7 trigger (optional)")
    ap.add_argument("--pico-align", choices=["start", "trigger"], default="trigger", help="Alignment method")
    ap.add_argument("--pico-time-scale", type=float, default=None, help="Override time scale to seconds (e.g., 1e-3 for ms)")
    ap.add_argument("--pico-volt-scale", type=float, default=None, help="Override voltage scale to volts (e.g., 1e-3 for mV)")
    ap.add_argument("--save-pico-zoom", type=pathlib.Path, default=None, help="Optional extra PNG: Pico-only zoomed delta plot")

    # Pico V->A conversion
    ap.add_argument("--pico-acs758-zero-v", type=float, default=None, help="ACS758 zero voltage for Pico conversion")
    ap.add_argument("--pico-acs70331-zero-v", type=float, default=None, help="ACS70331 zero voltage for Pico conversion")
    ap.add_argument("--pico-zero-window-s", type=float, default=0.5, help="Window to estimate zero if not provided")
    ap.add_argument("--pico-acs758-sens-v-per-a", type=float, default=0.040)
    ap.add_argument("--pico-acs70331-sens-v-per-a", type=float, default=0.400)
    ap.add_argument("--pico-acs758-polarity", type=float, default=1.0)
    ap.add_argument("--pico-acs70331-polarity", type=float, default=1.0)

    args = ap.parse_args()

    if not args.csv_file.exists():
        raise FileNotFoundError(args.csv_file)

    out_png = args.out if args.out else args.csv_file.with_suffix(".png")
    smooth_enabled = not args.no_smoothing
    w_ardu = max(1, int(args.smooth_window_arduino))
    w_pico = max(1, int(args.smooth_window_pico))

    # ---------------- Arduino data ----------------
    rows = parse_arduino_csv(args.csv_file)
    t_ms = [_getf(r, "t_ms") for r in rows]
    t, t_mode = _rebuild_time_seconds_from_ms(t_ms)

    duty = [_getf(r, "duty_pct") for r in rows]
    trig = [_getf(r, "trig") for r in rows]

    adc_758 = [_getf(r, "adc_acs758") for r in rows]
    adc_70331 = [_getf(r, "adc_acs70331") for r in rows]

    v_758 = [_getf(r, "v_acs758") for r in rows]
    v_70331 = [_getf(r, "v_acs70331") for r in rows]

    i_758 = [_getf(r, "i_acs758_A") for r in rows]
    i_70331 = [_getf(r, "i_acs70331_A") for r in rows]

    adc_758_plot = _moving_average(adc_758, w_ardu) if smooth_enabled else adc_758
    adc_70331_plot = _moving_average(adc_70331, w_ardu) if smooth_enabled else adc_70331
    v_758_plot = _moving_average(v_758, w_ardu) if smooth_enabled else v_758
    v_70331_plot = _moving_average(v_70331, w_ardu) if smooth_enabled else v_70331
    i_758_plot = _moving_average(i_758, w_ardu) if smooth_enabled else i_758
    i_70331_plot = _moving_average(i_70331, w_ardu) if smooth_enabled else i_70331

    phase_idx: List[int] = []
    last_phase = 0
    for r in rows:
        pv = _getf(r, "phase_idx")
        if math.isnan(pv):
            phase_idx.append(last_phase)
        else:
            iv = int(pv)
            phase_idx.append(iv)
            last_phase = iv
    phase_change_t: List[float] = []
    for i in range(1, len(phase_idx)):
        if phase_idx[i] != phase_idx[i - 1]:
            phase_change_t.append(t[i])

    # ---------------- Pico data (optional) ----------------
    pico_t: Optional[List[float]] = None
    pico_v_758: Optional[List[float]] = None
    pico_v_70331: Optional[List[float]] = None
    pico_i_758: Optional[List[float]] = None
    pico_i_70331: Optional[List[float]] = None
    pico_trig: Optional[List[float]] = None
    pico_v_758_plot: Optional[List[float]] = None
    pico_v_70331_plot: Optional[List[float]] = None
    pico_i_758_plot: Optional[List[float]] = None
    pico_i_70331_plot: Optional[List[float]] = None

    if args.pico_csv:
        if not args.pico_csv.exists():
            raise FileNotFoundError(args.pico_csv)

        pico_headers, pico_rows = parse_pico_csv(args.pico_csv)

        col_time = _resolve_col(pico_headers, args.pico_col_time, ["time"])
        col_758 = _resolve_col(pico_headers, args.pico_col_acs758, ["channel a", "ch a", "a"])
        col_70331 = _resolve_col(pico_headers, args.pico_col_acs70331, ["channel b", "ch b", "b"])

        col_trig: Optional[str] = None
        if args.pico_col_trig:
            col_trig = _resolve_col(pico_headers, args.pico_col_trig, [])
        else:
            # optional auto
            try:
                col_trig = _resolve_col(pico_headers, None, ["channel c", "ch c", "c", "trigger"])
            except Exception:
                col_trig = None

        t_scale = args.pico_time_scale if args.pico_time_scale is not None else _unit_scale_from_header(col_time, "time")
        v_scale_758 = args.pico_volt_scale if args.pico_volt_scale is not None else _unit_scale_from_header(col_758, "volt")
        v_scale_70331 = args.pico_volt_scale if args.pico_volt_scale is not None else _unit_scale_from_header(col_70331, "volt")

        raw_t = [float(r[col_time]) * t_scale for r in pico_rows]
        t_start = raw_t[0]
        pico_t = [x - t_start for x in raw_t]

        pico_v_758 = [float(r[col_758]) * v_scale_758 for r in pico_rows]
        pico_v_70331 = [float(r[col_70331]) * v_scale_70331 for r in pico_rows]

        if col_trig is not None:
            pico_trig = [float(r[col_trig]) for r in pico_rows]

        # Alignment to Arduino time base
        if args.pico_align == "trigger" and pico_trig is not None:
            ar_rise = _first_rising_edge_time(t, trig)
            pico_min = min(pico_trig)
            pico_max = max(pico_trig)
            pico_thr = pico_min + 0.5 * (pico_max - pico_min)
            pc_rise = _first_rising_edge_time(pico_t, pico_trig, thresh=pico_thr)
            if ar_rise is not None and pc_rise is not None:
                shift = ar_rise - pc_rise
                pico_t = [x + shift for x in pico_t]

        # Pico V->A conversion
        def est_zero(v: List[float], window_s: float) -> float:
            if not pico_t:
                return math.nan
            sel = [v[i] for i, tt in enumerate(pico_t) if tt <= window_s]
            return _mean(sel if sel else v[: max(1, min(2000, len(v)))])

        z758 = args.pico_acs758_zero_v if args.pico_acs758_zero_v is not None else est_zero(pico_v_758, args.pico_zero_window_s)
        z70331 = (
            args.pico_acs70331_zero_v
            if args.pico_acs70331_zero_v is not None
            else est_zero(pico_v_70331, args.pico_zero_window_s)
        )

        pico_i_758 = [((vv - z758) / args.pico_acs758_sens_v_per_a) * args.pico_acs758_polarity for vv in pico_v_758]
        pico_i_70331 = [((vv - z70331) / args.pico_acs70331_sens_v_per_a) * args.pico_acs70331_polarity for vv in pico_v_70331]
        pico_v_758_plot = _moving_average(pico_v_758, w_pico) if smooth_enabled else pico_v_758
        pico_v_70331_plot = _moving_average(pico_v_70331, w_pico) if smooth_enabled else pico_v_70331
        pico_i_758_plot = _moving_average(pico_i_758, w_pico) if smooth_enabled else pico_i_758
        pico_i_70331_plot = _moving_average(pico_i_70331, w_pico) if smooth_enabled else pico_i_70331

        print(f"Pico columns: time='{col_time}', acs758='{col_758}', acs70331='{col_70331}', trig='{col_trig}'")
        print(f"Pico scales applied: time={t_scale:g} s/unit, acs758={v_scale_758:g} V/unit, acs70331={v_scale_70331:g} V/unit")
        print(f"Pico zero estimates: ACS758={z758:.6f} V, ACS70331={z70331:.6f} V")

        # Optional Pico-only zoom figure (delta mV and estimated current)
        pico_zoom_path = args.save_pico_zoom
        if pico_zoom_path is None:
            if args.out is not None:
                pico_zoom_path = args.out.with_name(args.out.stem + "_pico_zoom.png")
            else:
                pico_zoom_path = args.csv_file.with_name(args.csv_file.stem + "_pico_zoom.png")

        dv758_mv = [(v - z758) * 1000.0 for v in pico_v_758]
        dv70331_mv = [(v - z70331) * 1000.0 for v in pico_v_70331]
        dv758_mv_plot = _moving_average(dv758_mv, w_pico) if smooth_enabled else dv758_mv
        dv70331_mv_plot = _moving_average(dv70331_mv, w_pico) if smooth_enabled else dv70331_mv

        f2, ax2 = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
        if smooth_enabled and args.plot_raw:
            ax2[0].plot(pico_t, dv758_mv, color="tab:red", linewidth=0.4, alpha=0.2)
            ax2[0].plot(pico_t, dv70331_mv, color="tab:green", linewidth=0.4, alpha=0.2)
        ax2[0].plot(pico_t, dv758_mv_plot, label="ACS758 delta (mV)", color="tab:red", linewidth=1.0)
        ax2[0].plot(pico_t, dv70331_mv_plot, label="ACS70331 delta (mV)", color="tab:green", linewidth=1.0)
        ax2[0].set_ylabel("Delta V (mV)")
        ax2[0].grid(True, alpha=0.3)
        ax2[0].legend(loc="upper right")

        if smooth_enabled and args.plot_raw:
            ax2[1].plot(pico_t, pico_i_758, color="tab:red", linewidth=0.4, alpha=0.2)
            ax2[1].plot(pico_t, pico_i_70331, color="tab:green", linewidth=0.4, alpha=0.2)
        ax2[1].plot(pico_t, pico_i_758_plot, label="ACS758 current est (A)", color="tab:red", linewidth=1.0)
        ax2[1].plot(pico_t, pico_i_70331_plot, label="ACS70331 current est (A)", color="tab:green", linewidth=1.0)
        ax2[1].set_ylabel("Current (A)")
        ax2[1].set_xlabel("Time (s)")
        ax2[1].grid(True, alpha=0.3)
        ax2[1].legend(loc="upper right")
        f2.suptitle("Pico-only Sensor Zoom: delta-V and estimated current", fontsize=12)
        f2.tight_layout(rect=(0, 0, 1, 0.97))
        f2.savefig(pico_zoom_path, dpi=150)
        print(f"Saved Pico zoom plot: {pico_zoom_path}")

    print(f"Arduino rows parsed: {len(rows)} | time mode: {t_mode}")

    # ---------------- Plot ----------------
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

    # 1) Duty + trig
    axes[0].plot(t, duty, label="Duty (%)", color="tab:blue", linewidth=1.3)
    axes[0].plot(t, [x * 100.0 for x in trig], label="D7 trig x100 (Arduino)", color="tab:orange", alpha=0.8)
    if pico_t is not None and pico_trig is not None:
        pmin, pmax = min(pico_trig), max(pico_trig)
        den = (pmax - pmin) if (pmax - pmin) != 0 else 1.0
        pico_trig_norm = [100.0 * (x - pmin) / den for x in pico_trig]
        axes[0].plot(pico_t, pico_trig_norm, label="D7 trig norm (Pico)", color="tab:purple", linewidth=0.9, alpha=0.8)
    axes[0].set_ylabel("Duty / trig")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    # 2) Arduino ADC
    if smooth_enabled and args.plot_raw:
        axes[1].plot(t, adc_758, color="tab:red", linewidth=0.4, alpha=0.2)
        axes[1].plot(t, adc_70331, color="tab:green", linewidth=0.4, alpha=0.2)
    axes[1].plot(t, adc_758_plot, label="ACS758 ADC (Arduino)", color="tab:red")
    axes[1].plot(t, adc_70331_plot, label="ACS70331 ADC (Arduino)", color="tab:green")
    axes[1].set_ylabel("ADC counts")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    # 3) Voltage traces
    if smooth_enabled and args.plot_raw:
        axes[2].plot(t, v_758, color="tab:red", alpha=0.2, linewidth=0.4)
        axes[2].plot(t, v_70331, color="tab:green", alpha=0.2, linewidth=0.4)
    axes[2].plot(t, v_758_plot, label="ACS758 Vout (Arduino)", color="tab:red", alpha=0.8)
    axes[2].plot(t, v_70331_plot, label="ACS70331 Vout (Arduino)", color="tab:green", alpha=0.8)
    if pico_t is not None and pico_v_758 is not None and pico_v_70331 is not None:
        if smooth_enabled and args.plot_raw:
            axes[2].plot(pico_t, pico_v_758, "--", color="tab:red", linewidth=0.4, alpha=0.2)
            axes[2].plot(pico_t, pico_v_70331, "--", color="tab:green", linewidth=0.4, alpha=0.2)
        axes[2].plot(pico_t, pico_v_758_plot, "--", label="ACS758 Vout (Pico)", color="tab:red", linewidth=1.0)
        axes[2].plot(pico_t, pico_v_70331_plot, "--", label="ACS70331 Vout (Pico)", color="tab:green", linewidth=1.0)
    axes[2].set_ylabel("Volts")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right")

    # 4) Current traces
    if smooth_enabled and args.plot_raw:
        axes[3].plot(t, i_758, color="tab:red", alpha=0.2, linewidth=0.4)
        axes[3].plot(t, i_70331, color="tab:green", alpha=0.2, linewidth=0.4)
    axes[3].plot(t, i_758_plot, label="ACS758 current (Arduino)", color="tab:red", alpha=0.85)
    axes[3].plot(t, i_70331_plot, label="ACS70331 current (Arduino)", color="tab:green", alpha=0.85)
    if pico_t is not None and pico_i_758 is not None and pico_i_70331 is not None:
        if smooth_enabled and args.plot_raw:
            axes[3].plot(pico_t, pico_i_758, "--", color="tab:red", linewidth=0.4, alpha=0.2)
            axes[3].plot(pico_t, pico_i_70331, "--", color="tab:green", linewidth=0.4, alpha=0.2)
        axes[3].plot(pico_t, pico_i_758_plot, "--", label="ACS758 current (Pico est)", color="tab:red", linewidth=1.0)
        axes[3].plot(pico_t, pico_i_70331_plot, "--", label="ACS70331 current (Pico est)", color="tab:green", linewidth=1.0)
    axes[3].set_ylabel("Current (A)")
    axes[3].set_xlabel("Time (s)")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="upper right")

    for ax in axes:
        for x in phase_change_t:
            ax.axvline(x, color="0.7", linewidth=0.6, alpha=0.45)

    idle_i_758 = [i_758[i] for i, d in enumerate(duty) if d == 0.0 and not math.isnan(i_758[i])]
    idle_i_70331 = [i_70331[i] for i, d in enumerate(duty) if d == 0.0 and not math.isnan(i_70331[i])]
    txt = ""
    if idle_i_758 and idle_i_70331:
        m1 = _mean(idle_i_758)
        m2 = _mean(idle_i_70331)
        r1 = _rms(idle_i_758, m1)
        r2 = _rms(idle_i_70331, m2)
        txt = (
            f"Idle current (Arduino, duty=0): ACS758 mean={m1:.4f} A, RMS={r1:.4f} A | "
            f"ACS70331 mean={m2:.4f} A, RMS={r2:.4f} A"
        )
        fig.text(0.01, 0.01, txt, fontsize=9, family="monospace")

    title = "Sensor A/B Benchmark: ACS758 vs ACS70331"
    if args.pico_csv:
        title += " (Arduino + Pico overlay)"
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=(0, 0.03, 1, 0.98))
    fig.savefig(out_png, dpi=150)
    print(f"Saved plot: {out_png}")
    if smooth_enabled:
        print(f"Smoothing enabled: Arduino window={w_ardu}, Pico window={w_pico}")
    if txt:
        print(txt)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
