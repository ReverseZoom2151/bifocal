#!/usr/bin/env python3
"""Focused investigation of why the MIDDLE sensor (index 2) is the noisiest
per-sensor channel under the variance-based SWITCHING strategy.

Run from the repo root:  python analysis/sensor_noise.py

Reads the per-sensor variance logs, the on-line black/white readings, and the
calibration variance logs. Computes a per-sensor mean/median variance table for
each method and track, quantifies how far sensor 2 stands out under switching,
and weighs three hypotheses (line-straddle, mixed-calibration, calibration
scale) against the numbers. Saves a grouped bar chart to gallery/sensor_noise.png.

Parsing mirrors analysis/make_gallery.py: drop PuTTY "=~" header lines and any
line containing letters (PuTTY headers, "AnalogVariance:"/"DigitalVariance:"
label rows, DN1..DN5 column headers).
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "data", "results")
CALIB = os.path.join(ROOT, "data", "calibration")
OUT = os.path.join(ROOT, "gallery")
os.makedirs(OUT, exist_ok=True)

C = {"analog": "#4C72B0", "digital": "#DD8452", "switching": "#55A868"}
plt.rcParams.update({
    "figure.dpi": 130, "font.size": 11, "axes.grid": True,
    "grid.alpha": 0.25, "axes.spines.top": False, "axes.spines.right": False,
})


def floats(line):
    out = []
    for tok in line.replace(";", ",").split(","):
        tok = tok.strip()
        try:
            out.append(float(tok))
        except ValueError:
            pass
    return out


def load_rows(path, drop_alpha=True):
    """Return an (N,5) array of numeric rows. Skip PuTTY "=~" headers, and
    (when drop_alpha) any line containing letters: label rows such as
    'AnalogVariance:'/'DigitalVariance:' and 'DN1..DN5' column headers."""
    rows = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            if line.startswith("=~"):
                continue
            if drop_alpha and any(c.isalpha() for c in line):
                continue
            v = floats(line)
            if len(v) >= 5:
                rows.append(v[:5])
    return np.array(rows) if rows else np.empty((0, 5))


def load_switching_split(path):
    """The switching log tags each data row with the mode that produced it:
    an 'AnalogVariance:' or 'DigitalVariance:' label line precedes each row.
    Return (analog_rows, digital_rows) so we can test whether the extra
    middle-sensor variance is concentrated in one mode (hypothesis b)."""
    ana, dig = [], []
    bucket = None
    with open(path, "r", errors="ignore") as f:
        for line in f:
            if line.startswith("=~"):
                continue
            low = line.lower()
            if "analogvariance" in low:
                bucket = ana
                continue
            if "digitalvariance" in low:
                bucket = dig
                continue
            if any(c.isalpha() for c in line):
                continue
            v = floats(line)
            if len(v) >= 5 and bucket is not None:
                bucket.append(v[:5])
    return (np.array(ana) if ana else np.empty((0, 5)),
            np.array(dig) if dig else np.empty((0, 5)))


def per_sensor(arr):
    if arr.size == 0:
        return np.full(5, np.nan), np.full(5, np.nan)
    return arr.mean(axis=0), np.median(arr, axis=0)


METHODS = ["analog", "digital", "switching"]
TRACKS = ["straight_line", "curve"]


def collect():
    table = {}
    for m in METHODS:
        for t in TRACKS:
            arr = load_rows(os.path.join(RESULTS, f"{m}_variances_{t}.csv"))
            mean, med = per_sensor(arr)
            table[(m, t)] = {"n": len(arr), "mean": mean, "median": med}
    return table


def print_table(table):
    print("=" * 74)
    print("PER-SENSOR VARIANCE  (columns = sensor index 0..4, 2 = middle)")
    print("=" * 74)
    for t in TRACKS:
        print(f"\nTrack: {t}")
        print(f"  {'method':<10} {'n':>4}  {'stat':<6} " +
              " ".join(f"s{i:>8}" for i in range(5)))
        for m in METHODS:
            e = table[(m, t)]
            for stat in ("mean", "median"):
                vals = e[stat]
                print(f"  {m:<10} {e['n']:>4}  {stat:<6} " +
                      " ".join(f"{v:9.6f}" for v in vals))

    print("\n" + "-" * 74)
    print("MIDDLE-SENSOR STANDOUT  (sensor 2 vs the mean of the four edge/side")
    print("sensors 0,1,3,4), using per-sensor MEDIAN variance")
    print("-" * 74)
    for m in METHODS:
        for t in TRACKS:
            med = table[(m, t)]["median"]
            mid = med[2]
            others = np.mean([med[0], med[1], med[3], med[4]])
            ratio = mid / others if others else np.nan
            print(f"  {m:<10} {t:<14} mid={mid:9.6f}  "
                  f"others_avg={others:9.6f}  ratio={ratio:6.2f}x")


def analyse_hypotheses(table):
    print("\n" + "=" * 74)
    print("HYPOTHESIS EVIDENCE")
    print("=" * 74)

    # (a) middle sensor sits over the line -> largest signal / boundary straddle
    print("\n(a) Line-straddle: on-line black/white reading per sensor")
    for m in ("analog", "digital"):
        arr = load_rows(os.path.join(RESULTS, f"{m}_BW_on_line.csv"))
        mean, _ = per_sensor(arr)
        print(f"    {m:<8} on-line mean reading: " +
              " ".join(f"{v:.3f}" for v in mean) +
              f"   (argmax = sensor {int(np.nanargmax(mean))})")

    # (b) switching mixes analog + digital calibrations of the SAME sensor
    print("\n(b) Mixed-calibration: split switching rows by the mode label that")
    print("    produced them (AnalogVariance vs DigitalVariance).")
    for t in TRACKS:
        ana, dig = load_switching_split(
            os.path.join(RESULTS, f"switching_variances_{t}.csv"))
        _, ana_med = per_sensor(ana)
        _, dig_med = per_sensor(dig)
        print(f"    {t}:")
        print(f"      analog-mode rows  n={len(ana):>4} median: " +
              " ".join(f"{v:.6f}" for v in ana_med))
        print(f"      digital-mode rows n={len(dig):>4} median: " +
              " ".join(f"{v:.6f}" for v in dig_med))
        # per-sensor gap between the two calibration populations
        gap = np.abs(dig_med - ana_med)
        print(f"      |digital-analog| median gap:      " +
              " ".join(f"{v:.6f}" for v in gap) +
              f"   (max gap at sensor {int(np.nanargmax(gap))})")

    # (c) calibration scale for sensor 2 amplifies noise
    print("\n(c) Calibration scale: static-target variance per sensor and the")
    print("    black-vs-white contrast (span) per sensor.")
    for m in ("analog", "digital"):
        vb = load_rows(os.path.join(CALIB, f"{m}_Variance_Blackline.csv"))
        # white file name casing differs between analog/digital
        wp = os.path.join(CALIB, f"{m}_Variance_white.csv")
        if not os.path.exists(wp):
            wp = os.path.join(CALIB, f"{m}_Variance_White.csv")
        vw = load_rows(wp)
        _, mb = per_sensor(vb)
        _, mw = per_sensor(vw)
        rb = load_rows(os.path.join(CALIB, f"{m}_calibratedReadings_blackline.csv"))
        rwp = os.path.join(CALIB, f"{m}_calibratedReadings_white.csv")
        rw = load_rows(rwp)
        black_mean, _ = per_sensor(rb)
        white_mean, _ = per_sensor(rw)
        span = black_mean - white_mean
        print(f"    {m}:")
        print(f"      calib var (black) median: " +
              " ".join(f"{v:.6f}" for v in mb))
        print(f"      calib var (white) median: " +
              " ".join(f"{v:.6f}" for v in mw))
        print(f"      black-white contrast span:" +
              " ".join(f"{v:6.3f}" for v in span) +
              f"   (max span at sensor {int(np.nanargmax(span))})")


def make_figure(table):
    idx = np.arange(5)
    w = 0.26
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), sharey=True)
    for ax, t in zip(axes, TRACKS):
        for k, m in enumerate(METHODS):
            med = table[(m, t)]["median"]
            bars = ax.bar(idx + (k - 1) * w, med, w, label=m.capitalize(),
                          color=C[m], edgecolor="black", linewidth=0.5)
            if m == "switching":
                # highlight the middle sensor bar under switching
                bars[2].set_edgecolor("#B22222")
                bars[2].set_linewidth(2.2)
                ax.annotate("middle\nsensor",
                            xy=(2 + (k - 1) * w, med[2]),
                            xytext=(2 + (k - 1) * w, med[2] * 3),
                            ha="center", fontsize=8.5, color="#B22222",
                            arrowprops=dict(arrowstyle="->", color="#B22222"))
        ax.set_yscale("log")
        ax.set_xticks(idx)
        ax.set_xticklabels([f"{i}" for i in idx])
        ax.set_xlabel("Sensor index (0=L .. 2=mid .. 4=R)")
        ax.set_title(t.replace("_", " ").capitalize())
    axes[0].set_ylabel("Median per-sensor variance (log)")
    axes[0].legend(frameon=False, fontsize=9)
    fig.suptitle("Per-sensor reading variance by method: the middle sensor "
                 "spikes only under switching", y=1.02)
    fig.tight_layout()
    out = os.path.join(OUT, "sensor_noise.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"\nwrote: {out}")


if __name__ == "__main__":
    table = collect()
    print_table(table)
    analyse_hypotheses(table)
    make_figure(table)
