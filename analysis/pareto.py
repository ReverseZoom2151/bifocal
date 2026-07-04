#!/usr/bin/env python3
"""Plot the Bifocal parameter sweep: a Pareto frontier and Monte Carlo CIs.

Reads the sweep CSV produced by sim/sweepsim (sim/sweep_results.csv) and writes
two figures into gallery/:

  pareto.png      tracking accuracy (IAE) vs steering smoothness (steerRMS)
                  scatter, with the Pareto frontier highlighted and the
                  frontier points labelled by strategy / (margin, dwell).
  montecarlo.png  per-strategy steerRMS and IAE with Monte Carlo 95 percent
                  confidence intervals as error bars, aggregated over the fixed
                  seed list used by the sweep.

Run from the repo root:  python analysis/pareto.py

The sweep aggregates a fixed, deterministic seed list, so these figures are
reproducible. Style and palette match analysis/make_gallery.py.
"""
import os
import csv
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSV_PATH = os.path.join(ROOT, "sim", "sweep_results.csv")
OUT = os.path.join(ROOT, "gallery")
os.makedirs(OUT, exist_ok=True)

# Colour-blind-friendly palette, consistent with make_gallery.py. Fusion is a
# fourth strategy not present there, so it gets an added colour from the same
# qualitative family.
C = {"analog-only": "#4C72B0", "digital-only": "#DD8452",
     "switching": "#55A868", "fusion": "#8172B3"}
plt.rcParams.update({
    "figure.dpi": 130, "font.size": 11, "axes.grid": True,
    "grid.alpha": 0.25, "axes.spines.top": False, "axes.spines.right": False,
})

# The Pareto plot uses a single Kp slice so it reads as a clean 2D trade-off.
PRIMARY_KP = 1.0
# Default switching parameters (mirrors the firmware) used for the per-strategy
# Monte Carlo comparison.
DEFAULT_MARGIN = 0.0005
DEFAULT_DWELL = 3


def load_rows(path):
    """Read the sweep CSV into a list of dicts with typed fields."""
    rows = []
    with open(path, "r", newline="") as f:
        for r in csv.DictReader(f):
            rows.append({
                "strategy": r["strategy"],
                "margin": float(r["switch_margin"]),
                "dwell": int(r["switch_dwell"]),
                "Kp": float(r["Kp"]),
                "n": int(r["seeds"]),
                "steerRMS_mean": float(r["steerRMS_mean"]),
                "steerRMS_std": float(r["steerRMS_std"]),
                "IAE_mean": float(r["IAE_mean"]),
                "IAE_std": float(r["IAE_std"]),
                "sw_mean": float(r["switch_per100_mean"]),
                "sw_std": float(r["switch_per100_std"]),
            })
    return rows


def ci95(std, n):
    """95 percent confidence interval half-width for the mean of n samples."""
    if n < 2:
        return 0.0
    return 1.96 * std / math.sqrt(n)


def pareto_front(points):
    """Return the indices of the non-dominated points (minimise both x and y).

    points is a list of (x, y). Point i is on the frontier if no other point is
    less-or-equal in both coordinates and strictly less in at least one.
    """
    idx = []
    for i, (xi, yi) in enumerate(points):
        dominated = False
        for j, (xj, yj) in enumerate(points):
            if j == i:
                continue
            if xj <= xi and yj <= yi and (xj < xi or yj < yi):
                dominated = True
                break
        if not dominated:
            idx.append(i)
    return idx


# --- Figure 1: Pareto frontier ----------------------------------------------
def fig_pareto(rows):
    slice_rows = [r for r in rows if abs(r["Kp"] - PRIMARY_KP) < 1e-9]

    sw = [r for r in slice_rows if r["strategy"] == "switching"]
    baselines = [r for r in slice_rows if r["strategy"] != "switching"]

    # All candidate points: every swept switching cell plus the three baselines.
    all_rows = sw + baselines
    pts = [(r["steerRMS_mean"], r["IAE_mean"]) for r in all_rows]
    front = set(pareto_front(pts))

    fig, ax = plt.subplots(figsize=(7.2, 5.2))

    # Switching grid points (faint green), non-frontier.
    for i, r in enumerate(sw):
        if i in front:
            continue
        ax.scatter(r["steerRMS_mean"], r["IAE_mean"], s=28,
                   color=C["switching"], alpha=0.35, edgecolor="none", zorder=2)

    # Baseline strategies as distinct large markers.
    for i, r in enumerate(all_rows):
        if r["strategy"] == "switching":
            continue
        ax.scatter(r["steerRMS_mean"], r["IAE_mean"], s=150, marker="D",
                   color=C[r["strategy"]], edgecolor="black", linewidth=0.6,
                   zorder=4, label=r["strategy"])
        ax.annotate(r["strategy"],
                    (r["steerRMS_mean"], r["IAE_mean"]),
                    textcoords="offset points", xytext=(8, 6),
                    fontsize=9, color=C[r["strategy"]])

    # Pareto frontier: sort by steerRMS and draw a connecting step line.
    front_rows = [all_rows[i] for i in sorted(front,
                  key=lambda i: all_rows[i]["steerRMS_mean"])]
    fx = [r["steerRMS_mean"] for r in front_rows]
    fy = [r["IAE_mean"] for r in front_rows]
    ax.plot(fx, fy, "-", color="#C44E52", linewidth=1.6, alpha=0.9,
            zorder=3, label="Pareto frontier")

    # Highlight the frontier points and label the switching ones by params.
    for r in front_rows:
        if r["strategy"] == "switching":
            ax.scatter(r["steerRMS_mean"], r["IAE_mean"], s=70,
                       color=C["switching"], edgecolor="black", linewidth=0.7,
                       zorder=5)
            ax.annotate("m=%.4f\nd=%d" % (r["margin"], r["dwell"]),
                        (r["steerRMS_mean"], r["IAE_mean"]),
                        textcoords="offset points", xytext=(6, -14),
                        fontsize=7.5, color="#2f6b48")

    ax.set_xlabel("steerRMS  (steering-command jerkiness; lower is smoother)")
    ax.set_ylabel("IAE  (integral abs tracking error; lower tracks better)")
    ax.set_title("Accuracy vs smoothness trade-off (Kp=%.1f)\n"
                 "switching swept over (margin, dwell); "
                 "lower-left is better" % PRIMARY_KP)

    # De-duplicate legend labels.
    handles, labels = ax.get_legend_handles_labels()
    seen = {}
    for h, l in zip(handles, labels):
        if l not in seen:
            seen[l] = h
    ax.legend(seen.values(), seen.keys(), frameon=False, fontsize=9,
              loc="upper right")

    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "pareto.png"), bbox_inches="tight")
    plt.close(fig)


# --- Figure 2: Monte Carlo per-strategy metrics with 95% CIs ----------------
def fig_montecarlo(rows):
    kp_rows = [r for r in rows if abs(r["Kp"] - PRIMARY_KP) < 1e-9]

    def pick(strategy):
        if strategy == "switching":
            for r in kp_rows:
                if (r["strategy"] == "switching"
                        and abs(r["margin"] - DEFAULT_MARGIN) < 1e-9
                        and r["dwell"] == DEFAULT_DWELL):
                    return r
        else:
            for r in kp_rows:
                if r["strategy"] == strategy:
                    return r
        return None

    order = ["analog-only", "digital-only", "switching", "fusion"]
    picked = [pick(s) for s in order]

    x = np.arange(len(order))
    colors = [C[s] for s in order]

    fig, axes = plt.subplots(1, 2, figsize=(9.2, 4.2))

    # Left: steerRMS with 95% CI error bars.
    rms = [r["steerRMS_mean"] for r in picked]
    rms_ci = [ci95(r["steerRMS_std"], r["n"]) for r in picked]
    axes[0].bar(x, rms, 0.6, color=colors, edgecolor="black", linewidth=0.5,
                yerr=rms_ci, capsize=5, error_kw={"elinewidth": 1.2})
    axes[0].set_ylabel("steerRMS (lower is smoother)")
    axes[0].set_title("Steering smoothness")

    # Right: IAE with 95% CI error bars.
    iae = [r["IAE_mean"] for r in picked]
    iae_ci = [ci95(r["IAE_std"], r["n"]) for r in picked]
    axes[1].bar(x, iae, 0.6, color=colors, edgecolor="black", linewidth=0.5,
                yerr=iae_ci, capsize=5, error_kw={"elinewidth": 1.2})
    axes[1].set_ylabel("IAE (lower tracks better)")
    axes[1].set_title("Tracking accuracy")

    n = picked[0]["n"]
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels([s.replace("-only", "") for s in order],
                           rotation=15)

    fig.suptitle("Per-strategy metrics with Monte Carlo 95%% CIs "
                 "(n=%d seeds, Kp=%.1f, switching m=%.4f d=%d)"
                 % (n, PRIMARY_KP, DEFAULT_MARGIN, DEFAULT_DWELL), y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "montecarlo.png"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    rows = load_rows(CSV_PATH)
    fig_pareto(rows)
    fig_montecarlo(rows)
    print("wrote: gallery/pareto.png, gallery/montecarlo.png")
