#!/usr/bin/env python3
"""Rigorous statistical analysis of the Bifocal trial logs.

Run from the repo root:  python analysis/statistics.py

The original paper (and the README) claim that variance-based switching
improves sensor reliability but reduces heading smoothness, and quote point
estimates with no confidence intervals or significance tests. This script
quantifies both claims properly:

  1. Variance stability: the per-method distribution of the average per-sensor
     variance (mean across the 5 sensors within each logged row), for the
     straight and curve tracks. Reports median, mean and IQR.
  2. Heading smoothness: from the theta runs, a roughness metric given by the
     std of theta and the mean absolute step |dtheta|. The runs are short and
     uneven, so n is reported for each.
  3. Uncertainty: a 95 percent bootstrap confidence interval (10000 resamples,
     fixed seed np.random.default_rng(0)) for every metric and method, by
     resampling the underlying rows with replacement.
  4. Pairwise comparison: switching vs analog and switching vs digital on the
     average variance (straight and curve). Reports the difference in medians
     with a bootstrap CI and a Mann-Whitney U test (scipy if present, otherwise
     a permutation p-value implemented by hand). States significance at
     alpha = 0.05.

Parsing is deliberately identical to analysis/make_gallery.py and
analysis/sensor_noise.py: PuTTY "=~" header lines are skipped, lines carrying
the "Variance" label are skipped, each kept row is 5 comma-separated per-sensor
variances, and the theta dumps (delimiter-free, reprinting a growing history)
are reduced to the single longest run via a regex over signed decimals, with
the -1.0 sentinel and |x| > 100 PuTTY artifacts dropped.

Outputs gallery/statistics.png and prints every number it computes. Written
values are duplicated in docs/statistics.md.
"""
import os
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from scipy.stats import mannwhitneyu
    HAVE_SCIPY = True
except Exception:  # pragma: no cover - scipy is expected but optional
    HAVE_SCIPY = False

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "data", "results")
OUT = os.path.join(ROOT, "gallery")
os.makedirs(OUT, exist_ok=True)

# Same colour-blind-friendly palette and neutral style as make_gallery.py.
C = {"analog": "#4C72B0", "digital": "#DD8452", "switching": "#55A868"}
plt.rcParams.update({
    "figure.dpi": 130, "font.size": 11, "axes.grid": True,
    "grid.alpha": 0.25, "axes.spines.top": False, "axes.spines.right": False,
})

METHODS = ["analog", "digital", "switching"]
TRACKS = ["straight_line", "curve"]
N_BOOT = 10000
SEED = 0
ALPHA = 0.05

RNG = np.random.default_rng(SEED)


# --- parsing (mirrors make_gallery.py / sensor_noise.py) ---------------------
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
    """Take the single longest delimiter-free theta run; drop the -1.0 sentinel
    and |x| > 100 PuTTY date-leak artifacts. This is a dispersion sample of the
    visited headings, not a clean trajectory (see README caveats)."""
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


# --- metrics -----------------------------------------------------------------
def avg_variance_sample(arr):
    """Average per-sensor variance per logged row: mean across the 5 sensors.
    This is exactly the scalar the firmware compares when it decides to switch.
    Returns a 1-D sample of length N (one value per row)."""
    if arr.size == 0:
        return np.empty(0)
    return arr.mean(axis=1)


def iqr(x):
    q1, q3 = np.percentile(x, [25, 75])
    return q1, q3, q3 - q1


# --- bootstrap ---------------------------------------------------------------
def bootstrap_ci(sample, stat, n_boot=N_BOOT, rng=RNG, ci=95):
    """Percentile bootstrap CI for a 1-argument statistic over a 1-D sample.
    Resamples the underlying rows (values) with replacement."""
    sample = np.asarray(sample, dtype=float)
    n = sample.size
    if n == 0:
        return np.nan, np.nan
    idx = rng.integers(0, n, size=(n_boot, n))
    boot = stat(sample[idx], axis=1)
    lo = np.percentile(boot, (100 - ci) / 2)
    hi = np.percentile(boot, 100 - (100 - ci) / 2)
    return lo, hi


def bootstrap_median_diff_ci(a, b, n_boot=N_BOOT, rng=RNG, ci=95):
    """Bootstrap CI for median(a) - median(b), resampling each group
    independently with replacement."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    na, nb = a.size, b.size
    ia = rng.integers(0, na, size=(n_boot, na))
    ib = rng.integers(0, nb, size=(n_boot, nb))
    diff = np.median(a[ia], axis=1) - np.median(b[ib], axis=1)
    lo = np.percentile(diff, (100 - ci) / 2)
    hi = np.percentile(diff, 100 - (100 - ci) / 2)
    return float(np.median(a) - np.median(b)), lo, hi


def permutation_pvalue(a, b, n_perm=N_BOOT, rng=RNG):
    """Two-sided permutation p-value for a difference in medians, used only if
    scipy is unavailable. Shuffles the pooled labels."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    obs = abs(np.median(a) - np.median(b))
    pooled = np.concatenate([a, b])
    na = a.size
    count = 0
    for _ in range(n_perm):
        rng.shuffle(pooled)
        if abs(np.median(pooled[:na]) - np.median(pooled[na:])) >= obs:
            count += 1
    return (count + 1) / (n_perm + 1)


def significance_test(a, b):
    """Return (label, statistic_or_none, p_value)."""
    if HAVE_SCIPY:
        u, p = mannwhitneyu(a, b, alternative="two-sided")
        return "Mann-Whitney U", float(u), float(p)
    p = permutation_pvalue(a, b)
    return "permutation (median diff)", None, float(p)


# --- collect -----------------------------------------------------------------
def collect_variance():
    """samples[(method, track)] -> 1-D array of average per-sensor variance."""
    samples = {}
    for m in METHODS:
        for t in TRACKS:
            arr = load_variance_rows(os.path.join(RESULTS, f"{m}_variances_{t}.csv"))
            samples[(m, t)] = avg_variance_sample(arr)
    return samples


def collect_theta():
    thetas = {}
    for m in METHODS:
        thetas[m] = load_theta_longest(
            os.path.join(RESULTS, f"{m}_straight_line_theta.csv"))
    return thetas


# --- reporting ---------------------------------------------------------------
def report_variance(samples):
    print("=" * 78)
    print("1. VARIANCE STABILITY  (average per-sensor variance per logged row)")
    print("   metric = mean across the 5 sensors within each row; lower is steadier")
    print("=" * 78)
    stats = {}
    for t in TRACKS:
        print(f"\nTrack: {t}")
        print(f"  {'method':<10} {'n':>4} {'median':>11} {'mean':>11} "
              f"{'IQR':>11}   {'95% CI of median':>26}")
        for m in METHODS:
            s = samples[(m, t)]
            med = float(np.median(s))
            mean = float(np.mean(s))
            q1, q3, iq = iqr(s)
            lo, hi = bootstrap_ci(s, np.median)
            stats[(m, t)] = dict(n=s.size, median=med, mean=mean, q1=q1, q3=q3,
                                 iqr=iq, ci_lo=lo, ci_hi=hi)
            print(f"  {m:<10} {s.size:>4} {med:>11.6f} {mean:>11.6f} "
                  f"{iq:>11.6f}   [{lo:.6f}, {hi:.6f}]")
    return stats


def report_theta(thetas):
    print("\n" + "=" * 78)
    print("2. HEADING SMOOTHNESS  (straight line; roughness = std of theta and")
    print("   mean |dtheta| between consecutive samples; lower is smoother)")
    print("   NOTE: runs are short and uneven; switching has far fewer samples.")
    print("=" * 78)
    stats = {}
    print(f"\n  {'method':<10} {'n':>5} {'std_theta':>11} "
          f"{'95% CI std':>26}   {'mean|dtheta|':>12} {'95% CI':>26}")
    for m in METHODS:
        th = thetas[m]
        n = th.size
        std = float(np.std(th))
        std_lo, std_hi = bootstrap_ci(th, np.std)
        steps = np.abs(np.diff(th)) if n > 1 else np.empty(0)
        mstep = float(np.mean(steps)) if steps.size else float("nan")
        st_lo, st_hi = bootstrap_ci(steps, np.mean) if steps.size else (np.nan, np.nan)
        stats[m] = dict(n=n, std=std, std_lo=std_lo, std_hi=std_hi,
                        mstep=mstep, mstep_lo=st_lo, mstep_hi=st_hi,
                        n_steps=steps.size)
        print(f"  {m:<10} {n:>5} {std:>11.4f} "
              f"[{std_lo:.4f}, {std_hi:.4f}]   {mstep:>12.4f} "
              f"[{st_lo:.4f}, {st_hi:.4f}]")
    return stats


def report_pairwise(samples):
    print("\n" + "=" * 78)
    print("3. PAIRWISE COMPARISON  (switching vs baseline, average variance)")
    print("   difference in medians = median(switching) - median(baseline)")
    print("   negative => switching is steadier (lower variance)")
    print("=" * 78)
    results = []
    for t in TRACKS:
        sw = samples[("switching", t)]
        for base in ("analog", "digital"):
            bs = samples[(base, t)]
            dmed, lo, hi = bootstrap_median_diff_ci(sw, bs)
            label, stat, p = significance_test(sw, bs)
            sig = p < ALPHA
            results.append(dict(track=t, baseline=base, dmed=dmed, lo=lo, hi=hi,
                                test=label, stat=stat, p=p, sig=sig,
                                n_sw=sw.size, n_base=bs.size))
            print(f"\n  {t}:  switching vs {base}")
            print(f"    median(switching)={np.median(sw):.6f} (n={sw.size})  "
                  f"median({base})={np.median(bs):.6f} (n={bs.size})")
            print(f"    diff in medians = {dmed:+.6f}   "
                  f"95% CI [{lo:+.6f}, {hi:+.6f}]")
            stat_str = f"U={stat:.1f}, " if stat is not None else ""
            print(f"    {label}: {stat_str}p={p:.3e}  -> "
                  f"{'SIGNIFICANT' if sig else 'not significant'} at alpha={ALPHA}")
    return results


# --- figure ------------------------------------------------------------------
def make_figure(vstats, tstats, pairs):
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.4))

    # Panel A: median average variance with 95% bootstrap CIs, per method/track.
    axA = axes[0]
    x = np.arange(len(METHODS))
    w = 0.36
    for j, t in enumerate(TRACKS):
        meds = np.array([vstats[(m, t)]["median"] for m in METHODS])
        los = np.array([vstats[(m, t)]["ci_lo"] for m in METHODS])
        his = np.array([vstats[(m, t)]["ci_hi"] for m in METHODS])
        err = np.vstack([meds - los, his - meds])
        off = (j - 0.5) * w
        alpha = 0.55 if t == "straight_line" else 1.0
        axA.bar(x + off, meds, w, color=[C[m] for m in METHODS], alpha=alpha,
                edgecolor="black", linewidth=0.5)
        axA.errorbar(x + off, meds, yerr=err, fmt="none", ecolor="black",
                     elinewidth=1.1, capsize=3)
    axA.set_yscale("log")
    axA.set_xticks(x)
    axA.set_xticklabels([m.capitalize() for m in METHODS])
    axA.set_ylabel("Median average per-sensor variance (log)")
    axA.set_title("Variance stability with 95% bootstrap CIs\n(lower is steadier)")
    from matplotlib.patches import Patch
    axA.legend(handles=[Patch(facecolor="grey", alpha=0.55, label="Straight"),
                        Patch(facecolor="grey", label="Curve")],
               frameon=False, fontsize=9)

    # Panel B: forest plot of switching-vs-baseline median differences with CIs.
    axB = axes[1]
    labels = [f"{p['track'].replace('_',' ')}\nsw vs {p['baseline']}" for p in pairs]
    y = np.arange(len(pairs))[::-1]
    dmeds = [p["dmed"] for p in pairs]
    lo = [p["dmed"] - p["lo"] for p in pairs]
    hi = [p["hi"] - p["dmed"] for p in pairs]
    colors = [C["switching"] if p["sig"] else "#999999" for p in pairs]
    axB.axvline(0.0, color="black", linewidth=1.0, linestyle="--", alpha=0.7)
    for yi, d, l, h, col, p in zip(y, dmeds, lo, hi, colors, pairs):
        axB.errorbar(d, yi, xerr=[[l], [h]], fmt="o", color=col, ecolor=col,
                     capsize=4, markersize=7, elinewidth=1.6)
        star = " *" if p["sig"] else ""
        axB.annotate(f"p={p['p']:.1e}{star}", xy=(d, yi), xytext=(0, 9),
                     textcoords="offset points", ha="center", fontsize=8)
    axB.set_yticks(y)
    axB.set_yticklabels(labels, fontsize=9)
    axB.set_xlabel("Difference in median average variance\n"
                   "(switching minus baseline; <0 favours switching)")
    axB.set_title("Switching vs baseline\n(filled = significant at alpha=0.05)")
    axB.grid(axis="y", alpha=0.0)

    fig.suptitle("Bifocal: variance stability and switching-vs-baseline "
                 "comparison", y=1.03, fontsize=12)
    fig.tight_layout()
    out = os.path.join(OUT, "statistics.png")
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"\nwrote: {out}")


if __name__ == "__main__":
    print(f"bootstrap resamples = {N_BOOT}, seed = {SEED}, "
          f"scipy = {'yes' if HAVE_SCIPY else 'no (using permutation test)'}")
    samples = collect_variance()
    thetas = collect_theta()
    vstats = report_variance(samples)
    tstats = report_theta(thetas)
    pairs = report_pairwise(samples)
    make_figure(vstats, tstats, pairs)
