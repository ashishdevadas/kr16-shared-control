#!/usr/bin/env python3
"""
E3 - teleoperation with no assistance vs static keep-out planes vs virtual fixtures generated from the inspection

usage: python3 analyze_e3.py [e3_none.csv e3_static.csv e3_fixtures.csv]
default: the latest ~/cern_logs/e3_<assist>_*.csv for each condition
outputs (package results/ folder): e3_metrics.png, e3_trajectories.png, e3_results.md
"""
import os
import sys
import csv
import glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.normpath(os.path.join(HERE, "..", "results"))
LOGS = os.path.expanduser("~/cern_logs")
COND = ["none", "static", "fixtures"]
LABEL = {"none": "No assistance", "static": "Static keep-out planes", "fixtures": "Generated fixtures"}
COLOR = {"none": "tab:red", "static": "tab:orange", "fixtures": "tab:green"}
D_WORK, S_NEAR, R_TIP, R_ENTRY = 0.03, 0.15, 0.010, 0.060   # must match shared_control.cpp


def load(path):
    with open(path) as f:
        header = next(csv.reader(f))
    c = {name: i for i, name in enumerate(header)}
    rows = list(csv.reader(open(path)))[1:]
    return c, rows


if len(sys.argv) == 4:
    files = dict(zip(COND, sys.argv[1:4]))
else:
    files = {}
    for k in COND:
        found = sorted(glob.glob(os.path.join(LOGS, f"e3_{k}_*.csv")))
        if not found:
            sys.exit(f"No e3_{k}_*.csv in ~/cern_logs: run ASSIST={k} OPERATOR=sim first")
        files[k] = found[-1]

data = {}
for k in COND:
    c, rows = load(files[k])
    num = lambda name: np.array([float(r[c[name]]) for r in rows])
    data[k] = {"time": num("time_s"), "e_pos": num("e_pos_mm"), "e_lat": num("e_lat_mm"),
               "e_depth": num("e_depth_mm"), "e_rot": num("e_rot_deg"), "intrusion": num("intrusion_mm"),
               "face": num("face_min_mm"), "success": num("success")}
    print(f"{k:9s}: {files[k]}  ({len(rows)} trials)")
    if len(rows) == 0:
        sys.exit(f"No finished trial in {files[k]}: run ASSIST={k} OPERATOR=sim again and wait for the summary")

# ---------------- results table ----------------
def ms(x):
    return f"{x.mean():.2f} ± {x.std(ddof=1) if len(x) > 1 else 0:.2f}"

head = ["Condition", "Trials", "Position error [mm]", "Lateral [mm]", "Abs. depth [mm]", "Orientation [deg]",
        "Keep-out intrusion [mm]", "Trials with intrusion", "Closest to face [mm]", "Time [s]", "Success"]
md = "| " + " | ".join(head) + " |\n|" + "---|" * len(head) + "\n"
for k in COND:
    d = data[k]
    n = len(d["time"])
    md += (f"| {LABEL[k]} | {n} | {ms(d['e_pos'])} | {ms(d['e_lat'])} | {ms(np.abs(d['e_depth']))} | "
           f"{ms(d['e_rot'])} | {ms(d['intrusion'])} | {int((d['intrusion'] > 1).sum())}/{n} | "
           f"{d['face'].min():.1f} (min) | {ms(d['time'])} | {int(d['success'].sum())}/{n} |\n")
md += ("\nSuccess = position error < 5 mm, orientation error < 3 deg, no keep-out intrusion (> 1 mm), "
       "no contact with the face.\nErrors are measured against the simulated true defect; the fixtures are "
       "built from the inspection estimate, so the fixture errors include the inspection error.\n")
os.makedirs(RES, exist_ok=True)
with open(os.path.join(RES, "e3_results.md"), "w") as f:
    f.write(md)
print("\n" + md)

# ---------------- metrics figure ----------------
fig, ax = plt.subplots(1, 4, figsize=(13, 3.8))
for a, (key, title) in zip(ax, [("e_pos", "Position error [mm]"), ("e_rot", "Orientation error [deg]"),
                                ("intrusion", "Keep-out intrusion [mm]"), ("time", "Task time [s]")]):
    vals = [data[k][key] for k in COND]
    bp = a.boxplot(vals, patch_artist=True, widths=0.55)
    for patch, k in zip(bp["boxes"], COND):
        patch.set_facecolor(COLOR[k])
        patch.set_alpha(0.35)
    for i, (v, k) in enumerate(zip(vals, COND)):
        a.scatter(np.full(len(v), i + 1) + np.random.default_rng(0).uniform(-0.12, 0.12, len(v)), v,
                  s=12, color=COLOR[k], zorder=3)
    a.set_xticks([1, 2, 3])
    a.set_xticklabels(["None", "Static", "Fixtures"])
    a.set_title(title, fontsize=10)
    a.grid(alpha=0.3, axis="y")
fig.suptitle("E3 - Assisted teleoperation towards the inspected defect (simulated operator, paired trials)",
             fontsize=11)
fig.tight_layout()
fig.savefig(os.path.join(RES, "e3_metrics.png"), dpi=150, bbox_inches="tight")

# ---------------- trajectory figure (side view of the approach) ----------------
fig, ax = plt.subplots(1, 3, figsize=(13, 4), sharey=True)
s_f = np.linspace(D_WORK, S_NEAR, 50)
r_f = R_TIP + (s_f - D_WORK) * (R_ENTRY - R_TIP) / (S_NEAR - D_WORK)
for a, k in zip(ax, COND):
    stamp = os.path.basename(files[k])[len(f"e3_{k}_"):-4]
    run = os.path.join(LOGS, f"run_{stamp}.csv")
    a.fill_between(s_f * 1000, r_f * 1000, 100, color="gray", alpha=0.15, label="Keep-out zone (generated)")
    a.plot(s_f * 1000, r_f * 1000, color="gray", lw=1)
    a.axvline(0, color="k", lw=2, label="Equipment face")
    a.axvline(D_WORK * 1000, color="g", ls=":", label="Target depth")
    if os.path.exists(run):
        c, rows = load(run)
        a_run = np.array([[float(r[c["trial"]]), float(r[c["op_stage"]]), float(r[c["s_axis"]]),
                           float(r[c["rho"]])] for r in rows if r[c["op_stage"]] in ("1", "2")]).reshape(-1, 4)
        for tr in np.unique(a_run[:, 0]):
            m = (a_run[:, 0] == tr) & (a_run[:, 1] >= 1) & (a_run[:, 1] <= 2) & (a_run[:, 2] < 0.3)
            a.plot(a_run[m, 2] * 1000, a_run[m, 3] * 1000, color=COLOR[k], lw=0.9, alpha=0.8)
    a.set_title(LABEL[k], fontsize=10)
    a.set_xlim(320, -5)
    a.set_ylim(0, 100)
    a.set_xlabel("Distance to the face along the normal [mm]")
    a.grid(alpha=0.3)
ax[0].set_ylabel("Distance to the approach axis [mm]")
ax[0].legend(fontsize=7, loc="upper left")
fig.suptitle("E3 - Tool approach paths (all trials): the generated funnel keeps the tool out of the keep-out zone",
             fontsize=11)
fig.tight_layout()
fig.savefig(os.path.join(RES, "e3_trajectories.png"), dpi=150, bbox_inches="tight")
print("Saved:", os.path.join(RES, "e3_metrics.png"), "and", os.path.join(RES, "e3_trajectories.png"))
