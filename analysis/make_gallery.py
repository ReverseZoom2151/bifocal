#!/usr/bin/env python3
"""Generate the README gallery plots from the raw experimental CSVs.

Run from the repo root:  python analysis/make_gallery.py
Outputs PNGs into gallery/. Parsing is defensive: several CSVs carry PuTTY log
headers (=~=~...), interleaved label rows ("AnalogVariance:"), and the theta
logs reprint their whole growing history each line (so we take the longest line).
"""
import os
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "data", "results")
CALIB = os.path.join(ROOT, "data", "calibration")
OUT = os.path.join(ROOT, "gallery")
os.makedirs(OUT, exist_ok=True)

# Colour-blind-friendly palette, consistent across all figures.
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


def load_variance_rows(path):
    """Return an (N,5) array of per-sensor variance rows, skipping junk lines."""
    rows = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            if line.startswith("=~") or "Variance" in line:
                continue
            v = floats(line)
            if len(v) >= 5:
                rows.append(v[:5])
    return np.array(rows) if rows else np.empty((0, 5))


def load_theta_longest(path):
    """Theta logs print fixed-point values with NO delimiters (e.g.
    "-8.28-8.16-8.10") and reprint a growing history, so we take the longest
    line and regex out each signed decimal. Drop the -1.0 sentinel and any
    PuTTY date-leak artifacts (|x| > 100). The result is a dispersion sample of
    the visited headings, not a clean trajectory (see README caveats)."""
    best = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            if line.startswith("=~"):
                continue
            v = [float(x) for x in re.findall(r"-?\d+\.\d+", line)]
            v = [x for x in v if abs(x) < 100 and x != -1.0]
            if len(v) > len(best):
                best = v
    return np.array(best)


def load_calibration_means(path):
    rows = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            if line.startswith("=~") or any(c.isalpha() for c in line):
                continue
            v = floats(line)
            if len(v) >= 5:
                rows.append(v[:5])
    a = np.array(rows)
    return a.mean(axis=0) if len(a) else np.zeros(5)


# --- Figure 1: variance comparison (median per-sensor variance) --------------
def fig_variance():
    methods = ["analog", "digital", "switching"]
    tracks = ["straight_line", "curve"]
    med = {t: [] for t in tracks}
    for t in tracks:
        for m in methods:
            arr = load_variance_rows(os.path.join(RESULTS, f"{m}_variances_{t}.csv"))
            med[t].append(np.median(arr) if arr.size else np.nan)

    x = np.arange(len(methods))
    w = 0.36
    fig, ax = plt.subplots(figsize=(6.2, 3.8))
    ax.bar(x - w/2, med["straight_line"], w, label="Straight",
           color=[C[m] for m in methods], alpha=0.55, edgecolor="black", linewidth=0.5)
    ax.bar(x + w/2, med["curve"], w, label="Curve",
           color=[C[m] for m in methods], edgecolor="black", linewidth=0.5)
    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels([m.capitalize() for m in methods])
    ax.set_ylabel("Median per-sensor variance (log)")
    ax.set_title("Sensor-reading variance by method and track\n(lower is steadier)")
    # legend: solid = curve, faded = straight
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(facecolor="grey", alpha=0.55, label="Straight"),
                       Patch(facecolor="grey", label="Curve")], frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "variance_comparison.png"), bbox_inches="tight")
    plt.close(fig)


# --- Figure 2: heading spread on a straight line (roughness proxy) -----------
def fig_theta():
    methods = ["analog", "digital", "switching"]
    stds, ns = [], []
    for m in methods:
        th = load_theta_longest(os.path.join(RESULTS, f"{m}_straight_line_theta.csv"))
        stds.append(th.std() if th.size else np.nan)
        ns.append(th.size)

    x = np.arange(len(methods))
    fig, ax = plt.subplots(figsize=(6.2, 3.8))
    bars = ax.bar(x, stds, 0.6, color=[C[m] for m in methods],
                  edgecolor="black", linewidth=0.5)
    for b, s, n in zip(bars, stds, ns):
        ax.text(b.get_x() + b.get_width()/2, s, f"{s:.3f}\n(n≈{n})",
                ha="center", va="bottom", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels([m.capitalize() for m in methods])
    ax.set_ylabel("Heading spread — std of θ (rad)")
    ax.set_ylim(0, max(stds) * 1.25)
    ax.set_title("Heading stability on a straight line\n"
                 "(lower is smoother; preliminary — thin, uneven runs)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "theta_stability.png"), bbox_inches="tight")
    plt.close(fig)


# --- Figure 3: calibration contrast (black vs white per sensor) -------------
def fig_calibration():
    idx = np.arange(5)
    fig, axes = plt.subplots(1, 2, figsize=(7.6, 3.6), sharey=True)
    for ax, arr in zip(axes, ["analog", "digital"]):
        black = load_calibration_means(os.path.join(CALIB, f"{arr}_calibratedReadings_blackline.csv"))
        white = load_calibration_means(os.path.join(CALIB, f"{arr}_calibratedReadings_white.csv"))
        ax.plot(idx, black, "o-", color="#222222", label="Black line")
        ax.plot(idx, white, "o--", color="#999999", label="White surface")
        ax.fill_between(idx, white, black, color=C[arr], alpha=0.18)
        ax.set_title(f"{arr.capitalize()} array")
        ax.set_xlabel("Sensor index (0=L … 4=R)")
        ax.set_xticks(idx)
    axes[0].set_ylabel("Calibrated reading (0–1)")
    axes[0].legend(frameon=False, fontsize=9)
    fig.suptitle("Calibration contrast: black line vs white surface", y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "calibration_contrast.png"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    fig_variance()
    fig_theta()
    fig_calibration()
    print("wrote:", ", ".join(sorted(os.listdir(OUT))))
