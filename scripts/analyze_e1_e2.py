#!/usr/bin/env python3
"""
E1: safety layer ON vs OFF  (wall at x = 1.50 m, DLS ON in both runs)
E2: singularity robustness, DLS vs plain pseudo-inverse (safety OFF in both runs)

usage: python3 analyze_e1_e2.py [E1_ON E1_OFF E2_DLS E2_PINV]
default: the four CSV files in results/data/
outputs: results/e1_safety.png, results/e2_singularity.png, results/e1_e2_results.md
"""
import os
import sys
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.normpath(os.path.join(HERE, "..", "results"))
DATA = os.path.join(RES, "data")
WALL, D_INF, D_SAFE = 1.50, 0.10, 0.02   # must match shared_control.cpp


def load(path):
    with open(path) as f:
        header = next(csv.reader(f))
    c = {name: i for i, name in enumerate(header)}
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    a = a[a[:, c["mode"]] == 1]                      # teleoperation rows only
    qd = np.abs(a[:, [c[f"qd{i}"] for i in range(1, 7)]]).max(axis=1)
    return {
        "t": a[:, c["t"]] - a[0, c["t"]],
        "x": a[:, c["x"]],
        "d": a[:, c["dmin"]],
        "qd": qd,
        "sigma": a[:, c["sigma_min"]],
        "lam": a[:, c["lambda"]],
        "damper": a[:, c["damper"]],
        "q3": a[:, c["q3"]],
        "q5": a[:, c["q5"]],
    }


def row(name, r):
    i = int(np.argmin(r["sigma"]))
    return [name,
            f"{r['x'].max():.4f}",
            f"{1000 * max(0.0, r['x'].max() - WALL):.1f}",
            f"{1000 * (WALL - r['x'].max()):+.1f}",
            f"{r['qd'].max():.3f}",
            f"{r['sigma'].min():.5f} (t={r['t'][i]:.1f}s, q3={r['q3'][i]:+.3f}, q5={r['q5'][i]:+.3f})",
            f"{r['lam'].max():.3f}",
            f"{100 * r['damper'].mean():.0f}"]


files = sys.argv[1:5] if len(sys.argv) == 5 else [
    os.path.join(DATA, n) for n in
    ("e1_safety_on.csv", "e1_safety_off.csv", "e2_dls.csv", "e2_pinv.csv")]
on, off, dls, pinv = (load(f) for f in files)

# ---------------- results table ----------------
head = ["Run", "max x [m]", "penetration [mm]", "closest to wall [mm]",
        "max |qd| [rad/s]", "min sigma (where)", "max lambda", "damper [%]"]
rows = [row("E1 safety ON (DLS ON)", on), row("E1 safety OFF (DLS ON)", off),
        row("E2 DLS (safety OFF)", dls), row("E2 pseudo-inverse (safety OFF)", pinv)]
md = "| " + " | ".join(head) + " |\n|" + "---|" * len(head) + "\n"
md += "".join("| " + " | ".join(r) + " |\n" for r in rows)
md += (f"\nE2 joint-velocity reduction: {pinv['qd'].max() / dls['qd'].max():,.0f}x  "
       f"(reach cost: {1000 * (pinv['x'].max() - dls['x'].max()):.1f} mm)\n")
with open(os.path.join(RES, "e1_e2_results.md"), "w") as f:
    f.write(md)
print(md)

# ---------------- E1 figure ----------------
fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
for r, lab, col in [(on, "Safety layer ON", "tab:blue"), (off, "Safety layer OFF", "tab:red")]:
    ax[0].plot(r["t"], r["x"], color=col, label=lab)
    ax[1].plot(r["t"], 1000 * (WALL - r["x"]), color=col, label=lab)
ax[0].axhline(WALL, color="k", ls="--", label="Keep-out wall (x = 1.50 m)")
ax[0].axhspan(WALL - D_INF, WALL, color="orange", alpha=0.15, label="Damper zone (10 cm)")
ax[0].set_ylabel("Tool x [m]")
ax[0].set_title("E1 - Safety layer during teleoperation towards a keep-out wall")
ax[0].legend(loc="lower right", fontsize=8)
ax[1].axhline(0, color="k", ls="--", label="Wall")
ax[1].axhline(1000 * D_SAFE, color="g", ls=":", label="d_s = 20 mm")
ax[1].set_ylabel("Distance to wall [mm]")
ax[1].set_xlabel("Time [s]")
ax[1].legend(loc="lower left", fontsize=8)
for a in ax:
    a.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(RES, "e1_safety.png"), dpi=150)

# ---------------- E2 figure ----------------
fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
for r, lab, col in [(dls, "Damped least squares", "tab:blue"), (pinv, "Pseudo-inverse", "tab:red")]:
    ax[0].semilogy(r["t"], np.clip(r["qd"], 1e-4, None), color=col, label=lab)
    ax[1].semilogy(r["t"], np.clip(r["sigma"], 1e-6, None), color=col, label=lab)
ax[0].set_ylabel("max |joint velocity| [rad/s]")
ax[0].set_title("E2 - Singularity robustness while stretching the arm (KR16)")
ax[0].legend(fontsize=8)
ax[1].axhline(0.05, color="k", ls=":", label="DLS activation (sigma_0 = 0.05)")
ax[1].set_ylabel("Smallest singular value of J")
ax[1].set_xlabel("Time [s]")
ax[1].legend(fontsize=8)
for a in ax:
    a.grid(alpha=0.3, which="both")
fig.tight_layout()
fig.savefig(os.path.join(RES, "e2_singularity.png"), dpi=150)
print("Saved:", os.path.join(RES, "e1_safety.png"), "and", os.path.join(RES, "e2_singularity.png"))
