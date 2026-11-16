#!/usr/bin/env python3
"""E1 - safety layer ON vs OFF.
Usage:  python3 analyze_e1.py <log_safety_ON.csv> <log_safety_OFF.csv>
"""
import csv
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WALL_X = 1.75   # keep-out plane x <= 1.75 m (as in shared_control.cpp)


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    data = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    # keep only active teleoperation: Manual Cartesian control with a non-zero operator command
    vop = np.abs(data["vop_x"]) + np.abs(data["vop_y"]) + np.abs(data["vop_z"])
    keep = (data["mode"] == 1) & (vop > 1e-6)
    out = {k: v[keep] for k, v in data.items()}
    out["t"] = out["t"] - out["t"][0]
    return out


def stats(d, name):
    qd = np.vstack([d[f"qd{i}"] for i in range(1, 7)])
    s = {
        "max x [m]": d["x"].max(),
        "min distance to planes [m]": d["dmin"].min(),
        "max penetration [mm]": max(0.0, -d["dmin"].min()) * 1000,
        "max |joint velocity| [rad/s]": np.abs(qd).max(),
        "damper active [% of time]": 100 * d["damper"].mean(),
        "stops": int(d["stop"].sum()),
        "duration [s]": d["t"][-1],
    }
    print(f"\n{name}")
    for k, v in s.items():
        print(f"  {k:32s} {v:10.4f}" if isinstance(v, float) else f"  {k:32s} {v:10d}")
    return s


on, off = load(sys.argv[1]), load(sys.argv[2])
s_on, s_off = stats(on, "Safety layer ON"), stats(off, "Safety layer OFF")

fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
ax[0].plot(on["t"], on["x"], label="safety ON")
ax[0].plot(off["t"], off["x"], label="safety OFF")
ax[0].axhline(WALL_X, color="k", ls="--", lw=1, label="keep-out plane x = 1.75 m")
ax[0].set_ylabel("tool x [m]")
ax[0].legend(loc="lower right")
ax[0].set_title("E1 - operator pushes towards the keep-out plane")
ax[1].plot(on["t"], on["dmin"] * 1000, label="safety ON")
ax[1].plot(off["t"], off["dmin"] * 1000, label="safety OFF")
ax[1].axhline(0, color="k", ls="--", lw=1)
ax[1].axhline(20, color="gray", ls=":", lw=1, label="safety distance 20 mm")
ax[1].set_ylabel("distance to plane [mm]")
ax[1].set_xlabel("time since operator command started [s]")
ax[1].legend(loc="lower right")
fig.tight_layout()
fig.savefig("e1_safety_on_off.png", dpi=150)
print("\nSaved e1_safety_on_off.png")
