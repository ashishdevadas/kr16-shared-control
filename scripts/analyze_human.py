#!/usr/bin/env python3
"""
Human trials (cern_shared_control v0.7): no assistance vs static keep-out planes vs generated fixtures,
within-subject design (every participant does the 3 conditions).

usage: python3 analyze_human.py [folder]        (default ~/cern_logs/human)
outputs (package results/ folder): human_results.md, human_metrics.png, human_tlx.png

Per participant and condition, the measured trials (practice excluded) are averaged; the conditions are
compared with the Friedman test, then pairwise Wilcoxon signed-rank tests (exact, Holm correction) with
the matched-pairs rank-biserial correlation as effect size. numpy + matplotlib only (no scipy needed).
"""
import csv
import glob
import itertools
import json
import os
import re
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.normpath(os.path.join(HERE, "..", "results"))
FOLDER = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/cern_logs/human")
COND = ["none", "static", "fixtures"]
LABEL = {"none": "No assistance", "static": "Static planes", "fixtures": "Generated fixtures"}
COLOR = {"none": "tab:red", "static": "tab:orange", "fixtures": "tab:green"}
METRICS = [("e_pos", "Position error [mm]"), ("e_rot", "Orientation error [deg]"),
           ("intrusion", "Keep-out intrusion [mm]"), ("time", "Task time [s]"),
           ("success", "Success rate [%]"), ("raw_tlx", "Workload, raw NASA-TLX")]
TLX_KEYS = ["mental", "physical", "temporal", "performance", "effort", "frustration"]


# ---------------- statistics (numpy only) ----------------
def ranks(x):
    """average ranks (1-based), ties get the mean rank"""
    x = np.asarray(x, float)
    order = np.argsort(x, kind="mergesort")
    r = np.empty(len(x))
    r[order] = np.arange(1, len(x) + 1)
    for v in np.unique(x):
        m = x == v
        r[m] = r[m].mean()
    return r


def friedman(data):
    """data: n participants x k conditions -> (chi2, p); chi-square with k-1 dof, tie corrected"""
    n, k = data.shape
    R = np.array([ranks(row) for row in data])
    chi2 = 12. / (n * k * (k + 1)) * np.sum(R.sum(axis=0) ** 2) - 3. * n * (k + 1)
    ties = sum(np.sum(np.unique(row, return_counts=True)[1] ** 3 - np.unique(row, return_counts=True)[1])
               for row in data)
    denom = 1. - ties / (n * k * (k * k - 1))
    if denom <= 0:
        return 0., 1.
    chi2 /= denom
    # survival function of chi-square with 2 dof (k = 3): exp(-x/2)
    p = float(np.exp(-chi2 / 2.)) if k == 3 else float("nan")
    return float(chi2), p


def wilcoxon_exact(a, b):
    """paired a vs b -> (p two-sided, rank-biserial r); zero differences dropped, exact over sign flips"""
    d = np.asarray(a, float) - np.asarray(b, float)
    d = d[np.abs(d) > 1e-12]
    n = len(d)
    if n == 0:
        return 1., 0.
    r = ranks(np.abs(d))
    w_plus = r[d > 0].sum()
    total = r.sum()
    signs = np.array(list(itertools.product([0, 1], repeat=n)))   # n <= ~20
    w_all = signs @ r
    p = min(1., 2. * min(np.mean(w_all <= w_plus + 1e-9), np.mean(w_all >= w_plus - 1e-9)))
    return float(p), float((2. * w_plus - total) / total)


def holm(ps):
    order = np.argsort(ps)
    adj = np.empty(len(ps))
    running = 0.
    for i, j in enumerate(order):
        running = max(running, (len(ps) - i) * ps[j])
        adj[j] = min(1., running)
    return adj


# ---------------- load ----------------
def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


trials = {}   # (participant, condition) -> latest trials file
for path in sorted(glob.glob(os.path.join(FOLDER, "P*_*_*.csv"))):
    name = os.path.basename(path)
    mm = re.fullmatch(r"(P\d+)_(none|static|fixtures)_(\d{8}_\d{6})\.csv", name)
    if mm:
        trials[(mm.group(1), mm.group(2))] = path     # sorted by stamp: the last one wins (repeated condition)

participants = sorted({p for p, _ in trials})
if not participants:
    sys.exit(f"No participant files P*_<condition>_<stamp>.csv in {FOLDER}")

data = {}     # participant -> condition -> metric -> value
for (pid, cond), path in trials.items():
    rows = [r for r in read_csv(path) if r.get("practice", "0") == "0"]
    if not rows:
        continue
    num = lambda k: np.array([float(r[k]) for r in rows])
    data.setdefault(pid, {})[cond] = {
        "e_pos": num("e_pos_mm").mean(), "e_rot": num("e_rot_deg").mean(),
        "intrusion": num("intrusion_mm").mean(), "time": num("time_s").mean(),
        "success": 100. * num("success").mean(), "n": len(rows),
        "intrusion_trials": int((num("intrusion_mm") > 1.).sum()),
        "contact_trials": int((num("face_min_mm") <= 0.).sum())}

for pid in participants:
    tlx_path = os.path.join(FOLDER, f"{pid}_tlx.csv")
    if os.path.exists(tlx_path):
        for r in read_csv(tlx_path):
            if r["condition"] in data.get(pid, {}):
                data[pid][r["condition"]]["raw_tlx"] = float(r["raw_tlx"])
                for k in TLX_KEYS:
                    data[pid][r["condition"]][k] = float(r[k])

complete = [p for p in participants if all(c in data.get(p, {}) for c in COND)]
print(f"Participants: {len(participants)} found, {len(complete)} with all 3 conditions: {', '.join(complete)}")
if len(complete) < 2:
    sys.exit("Need at least 2 participants with all 3 conditions for the comparison")

prefs = []
for pid in complete:
    sp = os.path.join(FOLDER, f"{pid}_session.json")
    if os.path.exists(sp):
        s = json.load(open(sp))
        if "preference" in s:
            prefs.append(s["preference"])


def matrix(key):
    return np.array([[data[p][c].get(key, np.nan) for c in COND] for p in complete])


# ---------------- table ----------------
def med_iqr(x):
    q1, q2, q3 = np.percentile(x, [25, 50, 75])
    return f"{q2:.1f} [{q1:.1f}-{q3:.1f}]"


pairs = [("none", "fixtures"), ("static", "fixtures"), ("none", "static")]
head = ["Metric"] + [LABEL[c] for c in COND] + ["Friedman p"] + [f"{LABEL[a]} vs {LABEL[b]} (p Holm, r)"
                                                               for a, b in pairs]
md = (f"# Human trials: {len(complete)} participants, within-subject\n\n"
      "Median [IQR] over participants of the per-participant mean of the measured trials.\n\n"
      "| " + " | ".join(head) + " |\n|" + "---|" * len(head) + "\n")
for key, title in METRICS:
    X = matrix(key)
    if np.isnan(X).any():
        continue
    chi2, p = friedman(X)
    raw = [wilcoxon_exact(X[:, COND.index(a)], X[:, COND.index(b)]) for a, b in pairs]
    adj = holm(np.array([r[0] for r in raw]))
    md += (f"| {title} | " + " | ".join(med_iqr(X[:, i]) for i in range(3)) + f" | {p:.3f} | "
           + " | ".join(f"{pa:.3f}, {r[1]:+.2f}" for pa, r in zip(adj, raw)) + " |\n")

tot = {c: sum(data[p][c]["n"] for p in complete) for c in COND}
intr = {c: sum(data[p][c]["intrusion_trials"] for p in complete) for c in COND}
cont = {c: sum(data[p][c]["contact_trials"] for p in complete) for c in COND}
md += ("\n| Condition | Measured trials | Trials with intrusion > 1 mm | Trials touching the face |\n"
       "|---|---|---|---|\n")
for c in COND:
    md += f"| {LABEL[c]} | {tot[c]} | {intr[c]} | {cont[c]} |\n"
if prefs:
    md += "\n| Preference | " + " | ".join(LABEL[c] for c in COND) + " |\n|---|---|---|---|\n"
    for rank, name in enumerate(["Most preferred", "Middle", "Least preferred"]):
        md += f"| {name} | " + " | ".join(str(sum(p[rank] == c for p in prefs)) for c in COND) + " |\n"
md += ("\nSuccess = position error < 5 mm, orientation error < 3 deg, no intrusion > 1 mm, no contact.\n"
       "r = matched-pairs rank-biserial correlation (positive: the first condition has the larger values).\n"
       f"With {len(complete)} participants this is a pilot study: report effect sizes, not only p-values.\n")
os.makedirs(RES, exist_ok=True)
with open(os.path.join(RES, "human_results.md"), "w") as f:
    f.write(md)
print("\n" + md)

# ---------------- metrics figure: box plots + one line per participant ----------------
fig, ax = plt.subplots(1, len(METRICS), figsize=(3.1 * len(METRICS), 3.9))
rng = np.random.default_rng(0)
for a, (key, title) in zip(ax, METRICS):
    X = matrix(key)
    if np.isnan(X).any():
        a.set_visible(False)
        continue
    bp = a.boxplot([X[:, i] for i in range(3)], patch_artist=True, widths=0.55, showfliers=False)
    for patch, c in zip(bp["boxes"], COND):
        patch.set_facecolor(COLOR[c])
        patch.set_alpha(0.3)
    jitter = rng.uniform(-0.06, 0.06, X.shape)
    for i in range(X.shape[0]):
        a.plot(np.arange(1, 4) + jitter[i], X[i], color="gray", lw=0.7, alpha=0.6, zorder=2)
    for j, c in enumerate(COND):
        a.scatter(j + 1 + jitter[:, j], X[:, j], s=14, color=COLOR[c], zorder=3)
    a.set_xticks([1, 2, 3])
    a.set_xticklabels(["None", "Static", "Fixtures"])
    a.set_title(title, fontsize=10)
    a.grid(alpha=0.3, axis="y")
fig.suptitle(f"Human trials - {len(complete)} participants (lines connect the same participant)", fontsize=11)
fig.tight_layout()
fig.savefig(os.path.join(RES, "human_metrics.png"), dpi=150, bbox_inches="tight")

# ---------------- NASA-TLX subscales ----------------
if not np.isnan(matrix("raw_tlx")).any():
    fig, a = plt.subplots(figsize=(8, 3.6))
    w = 0.26
    for j, c in enumerate(COND):
        vals = [np.median([data[p][c][k] for p in complete]) for k in TLX_KEYS]
        a.bar(np.arange(len(TLX_KEYS)) + (j - 1) * w, vals, w, color=COLOR[c], alpha=0.8, label=LABEL[c])
    a.set_xticks(np.arange(len(TLX_KEYS)))
    a.set_xticklabels([k.capitalize() for k in TLX_KEYS])
    a.set_ylabel("Median score (0-100)")
    a.set_ylim(0, 100)
    a.grid(alpha=0.3, axis="y")
    a.legend(fontsize=8)
    a.set_title("NASA-TLX subscales (lower = less workload; performance: lower = better)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "human_tlx.png"), dpi=150, bbox_inches="tight")
print("Saved:", ", ".join(os.path.join(RES, f) for f in ("human_results.md", "human_metrics.png", "human_tlx.png")))
