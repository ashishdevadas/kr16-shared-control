#!/usr/bin/env python3
"""
Mode 1 - autonomous inspection scan analysis

usage: python3 analyze_scan.py [run.csv] [inspection.json]
default: the latest ~/cern_logs/run_*.csv that contains Mode 1 rows, and ~/cern_logs/inspection_latest.json
outputs (in the package results/ folder): scan_coverage.png, scan_tracking.png, scan_results.md
"""
import os
import sys
import csv
import glob
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.normpath(os.path.join(HERE, "..", "results"))
LOGS = os.path.expanduser("~/cern_logs")
FOV_HALF = np.radians(20.)   # must match shared_control.cpp


def load(path):
    with open(path) as f:
        header = next(csv.reader(f))
    c = {name: i for i, name in enumerate(header)}
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return c, a


def rotation(tu):
    """theta-u vector -> rotation matrix (Rodrigues)"""
    th = np.linalg.norm(tu)
    if th < 1e-12:
        return np.eye(3)
    u = tu / th
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def latest_scan_csv():
    for f in sorted(glob.glob(os.path.join(LOGS, "run_*.csv")), reverse=True):
        c, a = load(f)
        if "phase" in c and np.any(a[:, c["mode"]] == 5):
            return f
    sys.exit("No log with Mode 1 rows found in ~/cern_logs")


csv_path = sys.argv[1] if len(sys.argv) > 1 else latest_scan_csv()
json_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(LOGS, "inspection_latest.json")
c, a = load(csv_path)
a = a[a[:, c["mode"]] == 5]
rep = json.load(open(json_path))
xf = rep["component"]["plane_x"]
print("CSV :", csv_path)
print("JSON:", json_path)

scan = a[a[:, c["phase"]] == 2]
appr = a[a[:, c["phase"]] == 1]
t0 = a[0, c["t"]]

# optical-axis footprint on the face for every scan row
foot = []
for r in scan:
    p = r[[c["x"], c["y"], c["z"]]]
    zax = rotation(r[[c["tux"], c["tuy"], c["tuz"]]])[:, 2]
    s = (xf - p[0]) / zax[0]
    foot.append([p[1] + s * zax[1], p[2] + s * zax[2], s * np.tan(FOV_HALF)])
foot = np.array(foot)

# ---------------- results table ----------------
qd = np.abs(scan[:, [c[f"qd{i}"] for i in range(1, 7)]]).max(axis=1)
lines = ["| Quantity | Value |", "|---|---|",
         f"| Approach duration | {appr[-1, c['t']] - appr[0, c['t']]:.1f} s |" if len(appr) else "| Approach | - |",
         f"| Scan duration | {rep['scan']['duration_s']:.1f} s |",
         f"| Photos | {rep['scan']['photos']} |",
         f"| Rows / standoff / tilt | {rep['scan']['rows']} / {rep['scan']['standoff_m']:.2f} m / {rep['scan']['tilt_deg']:.0f} deg |",
         f"| Tracking error during scan (mean / max) | {1000 * scan[:, c['e_pos']].mean():.2f} / {1000 * scan[:, c['e_pos']].max():.2f} mm |",
         f"| min sigma during scan | {scan[:, c['sigma_min']].min():.3f} |",
         f"| max lambda during scan | {scan[:, c['lambda']].max():.3f} |",
         f"| max joint velocity during scan | {qd.max():.3f} rad/s |",
         f"| closest distance to keep-out planes | {1000 * scan[:, c['dmin']].min():.0f} mm |",
         f"| damper active during scan | {100 * scan[:, c['damper']].mean():.0f} % |"]
for d in rep["defects"]:
    lines.append(f"| Defect {d['id']}: estimate (y, z) | ({d['position'][1]:+.4f}, {d['position'][2]:.4f}) m, "
                 f"{d['detections']} detections, std {1000 * d['std_m'][0]:.2f}/{1000 * d['std_m'][1]:.2f} mm |")
    lines.append(f"| Defect {d['id']}: error vs simulated truth | {1000 * d['sim_error_m']:.2f} mm |")
md = "\n".join(lines) + "\n"
os.makedirs(RES, exist_ok=True)
with open(os.path.join(RES, "scan_results.md"), "w") as f:
    f.write(md)
print(md)

# ---------------- coverage figure (view of the face) ----------------
fig, ax = plt.subplots(figsize=(8, 5.5))
ry, rz = rep["scan"]["region_y"], rep["scan"]["region_z"]
ax.add_patch(Rectangle((ry[0], rz[0]), ry[1] - ry[0], rz[1] - rz[0], fill=False, ls="--",
                       color="gray", label="Planned scan region"))
photo_idx = np.where(np.diff(scan[:, c["photos"]]) > 0)[0] + 1
for k, i in enumerate(photo_idx):
    ax.add_patch(plt.Circle((foot[i, 0], foot[i, 1]), foot[i, 2], color="tab:blue", alpha=0.06,
                            label="Camera footprint (each photo)" if k == 0 else None))
ax.plot(foot[:, 0], foot[:, 1], color="tab:blue", lw=1, label="Optical-axis trace")
for d in rep["defects"]:
    ax.plot(d["sim_truth"][1], d["sim_truth"][2], "kx", ms=10, mew=2)
    ax.plot(d["position"][1], d["position"][2], "o", color="tab:red", mfc="none", ms=12, mew=2)
    ax.annotate(f"{d['id']}  err {1000 * d['sim_error_m']:.1f} mm", (d["position"][1], d["position"][2]),
                xytext=(10, 10), textcoords="offset points", color="tab:red", fontsize=9)
ax.plot([], [], "kx", ms=10, mew=2, label="Defect (simulated truth)")
ax.plot([], [], "o", color="tab:red", mfc="none", ms=12, mew=2, label="Defect (estimated)")
ax.set_xlabel("y on the face [m]")
ax.set_ylabel("z on the face [m]")
ax.set_title(f"Mode 1 - Autonomous inspection of the equipment face (x = {xf:.2f} m)")
ax.set_aspect("equal")
ax.invert_xaxis()   # seen from the robot, looking towards +x
ax.grid(alpha=0.3)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=3, fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(RES, "scan_coverage.png"), dpi=150, bbox_inches="tight")

# ---------------- tracking figure ----------------
fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
tt = a[:, c["t"]] - t0
ph = a[:, c["phase"]]
for k, (val, lab, col) in enumerate([("e_pos", "Position error [mm]", "tab:blue"),
                                     ("sigma_min", "Smallest singular value of J", "tab:green"),
                                     ("dmin", "Distance to keep-out planes [mm]", "tab:purple")]):
    y = a[:, c[val]] * (1000 if val in ("e_pos", "dmin") else 1)
    ax[k].plot(tt, y, color=col)
    ax[k].set_ylabel(lab, fontsize=9)
    ax[k].grid(alpha=0.3)
    for p_val, name, shade in [(1, "approach", "orange"), (2, "scan", "green")]:
        m = ph == p_val
        if m.any():
            ax[k].axvspan(tt[m][0], tt[m][-1], color=shade, alpha=0.08, label=name if k == 0 else None)
ax[0].set_yscale("symlog", linthresh=1)
ax[0].legend(fontsize=8)
ax[1].axhline(0.05, color="k", ls=":", lw=1)
ax[2].axhline(100, color="orange", ls=":", lw=1)
ax[2].axhline(20, color="g", ls=":", lw=1)
ax[2].set_xlabel("Time [s]")
ax[0].set_title("Mode 1 - Approach and raster scan under the common safety layer")
fig.tight_layout()
fig.savefig(os.path.join(RES, "scan_tracking.png"), dpi=150, bbox_inches="tight")
print("Saved:", os.path.join(RES, "scan_coverage.png"), "and", os.path.join(RES, "scan_tracking.png"))
