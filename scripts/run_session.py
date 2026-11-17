#!/usr/bin/env python3
"""
Human-trial session runner (cern_shared_control v0.7): one participant, the 3 assistance conditions in a
counterbalanced order, a NASA-TLX questionnaire after each condition, a preference ranking at the end.

Before the first participant (once):
  1. ros2 launch ecn_manip simulation_launch.py robot:=kr16      (simulator + RViz + GUI)
  2. run the inspection scan once (Mode 1) so ~/cern_logs/inspection_latest.json exists,
     then keep that report for every participant (same fixtures for everyone)
  3. set the RViz view behind the robot looking at the equipment face and save the RViz config

For each participant:
  python3 run_session.py P01            (P01, P02, ... the number picks the condition order)
  options: --trials 5 --practice 1 --timeout 90 --no-window

Output in ~/cern_logs/human/: P01_session.json, P01_tlx.csv, and the controller files
P01_<condition>_<stamp>.csv (trials) and ..._run.csv (full log) for each condition.
"""
import argparse
import csv
import json
import os
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime

LOGS = os.path.expanduser("~/cern_logs")
HUMAN = os.path.join(LOGS, "human")
CONDITIONS = ["none", "static", "fixtures"]
# all 6 orders of the 3 conditions: participant k uses ORDERS[(k - 1) % 6]
ORDERS = [("none", "static", "fixtures"), ("none", "fixtures", "static"), ("static", "none", "fixtures"),
          ("static", "fixtures", "none"), ("fixtures", "none", "static"), ("fixtures", "static", "none")]
BRIEFING = {
    "none": "No assistance: the robot does exactly what you command. Nothing stops you from hitting the "
            "equipment.",
    "static": "Safety planes: the robot slows down and stops before the floor zone and the equipment face. "
              "There is no guidance towards the target.",
    "fixtures": "Guidance: near the defect, the robot keeps the tool inside the green funnel, turns it "
                "towards the face and stops at the working depth. You still drive.",
}
TLX = [
    ("mental", "Mental demand", "How mentally demanding was the task?", "very low", "very high"),
    ("physical", "Physical demand", "How physically demanding was the task?", "very low", "very high"),
    ("temporal", "Temporal demand", "How hurried or rushed was the pace of the task?", "very low", "very high"),
    # asked as 0 = failure ... 100 = perfect (intuitive), stored with the TLX convention 0 = perfect
    ("performance", "Performance", "How successful were you in accomplishing what you were asked to do?",
     "failure", "perfect"),
    ("effort", "Effort", "How hard did you have to work to accomplish your level of performance?",
     "very low", "very high"),
    ("frustration", "Frustration", "How insecure, discouraged, irritated, stressed or annoyed were you?",
     "very low", "very high"),
]


def ask(prompt, valid=None, cast=str):
    while True:
        ans = input(prompt).strip()
        try:
            val = cast(ans)
        except ValueError:
            print("  please type a valid answer")
            continue
        if valid is None or val in valid:
            return val
        print(f"  please type one of: {', '.join(str(v) for v in valid)}")


def ask_scale(prompt, lo, hi):
    return ask(f"{prompt} [{lo}-{hi}]: ", valid=range(lo, hi + 1), cast=int)


def banner(text):
    print("\n" + "=" * 72 + f"\n  {text}\n" + "=" * 72)


def nasa_tlx(participant, condition, position, label):
    banner(f"QUESTIONNAIRE - {label}  (participant types the answers)")
    print("For each question, type a number from 0 to 100 (steps of 5 are fine).\n")
    row = {"participant": participant, "condition": condition, "position": position}
    for key, name, question, lo, hi in TLX:
        print(f"{name}: {question}")
        row[key] = ask(f"   0 = {lo} ... 100 = {hi}: ", valid=range(0, 101), cast=int)
        if key == "performance":
            row[key] = 100 - row[key]     # NASA-TLX convention: 0 = perfect, 100 = failure (higher = more load)
    row["raw_tlx"] = round(sum(row[k] for k, *_ in TLX) / len(TLX), 2)
    path = os.path.join(HUMAN, f"{participant}_tlx.csv")
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)
    print(f"\nWorkload score (raw TLX): {row['raw_tlx']}  - saved to {path}")
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("participant", help="participant ID, e.g. P01")
    ap.add_argument("--trials", type=int, default=5, help="measured trials per condition (default 5)")
    ap.add_argument("--practice", type=int, default=1, help="practice trials per condition (default 1)")
    ap.add_argument("--timeout", type=float, default=90., help="time limit per trial in s (default 90)")
    ap.add_argument("--no-window", action="store_true", help="do not start the keyboard window")
    args = ap.parse_args()

    pid = args.participant.upper()
    m = re.fullmatch(r"P(\d+)", pid)
    if not m:
        sys.exit("Participant ID must look like P01, P02, ...")
    order = ORDERS[(int(m.group(1)) - 1) % len(ORDERS)]
    report = os.path.join(LOGS, "inspection_latest.json")
    if not os.path.exists(report):
        sys.exit("No ~/cern_logs/inspection_latest.json: run the inspection scan (Mode 1) once first")
    os.makedirs(HUMAN, exist_ok=True)
    session_path = os.path.join(HUMAN, f"{pid}_session.json")
    if os.path.exists(session_path):
        if ask(f"{session_path} exists. Overwrite this participant's session? [y/n]: ", ["y", "n"]) == "n":
            sys.exit(0)
        for f in (session_path, os.path.join(HUMAN, f"{pid}_tlx.csv")):
            if os.path.exists(f):
                os.remove(f)
    shutil.copy(report, os.path.join(HUMAN, f"{pid}_inspection.json"))

    banner(f"SESSION {pid}   order: {' -> '.join(order)}")
    print("Experimenter checklist:\n"
          "  - simulator, RViz and the GUI are running; the GUI is on 'Manual Cartesian control'\n"
          "  - the consent form is signed\n"
          "  - the participant sits in front of RViz and the keyboard window\n")
    session = {"participant": pid, "order": list(order), "trials": args.trials, "practice": args.practice,
               "timeout_s": args.timeout, "started": datetime.now().isoformat(timespec="seconds")}

    print("Short background questions (participant answers):")
    age = ask("  Age (years): ", valid=range(10, 100), cast=int)
    if age < 18:
        sys.exit("Participants must be 18 or older (adult volunteers only).")
    session["age_range"] = "18-24" if age < 25 else "25-34" if age < 35 else "35+"   # only the range is saved
    session["robot_experience"] = ask_scale("  Experience with robots, 1 = none ... 5 = expert", 1, 5)
    session["gaming_experience"] = ask_scale("  Video games with keyboard, 1 = never ... 5 = every day", 1, 5)
    session["teleop_experience"] = ask("  Have you teleoperated a robot before? [y/n]: ", ["y", "n"])

    window = None
    if not args.no_window:
        window = subprocess.Popen(["ros2", "run", "cern_shared_control", "cern_teleop.py"])
        print("\nKeyboard window started. Explain the keys (legend in the window) and let the participant\n"
              "try them for 2-3 minutes during the first practice trial.")

    session["conditions"] = []
    try:
        for i, cond in enumerate(order, start=1):
            label = f"Condition {i} of 3"
            banner(f"{label}: {cond.upper()}")
            print("Read to the participant:\n  \"" + BRIEFING[cond] + "\"\n"
                  "  \"Bring the tool tip onto the green target and turn it to match the target frame, without\n"
                  "   touching the equipment. The readout shows depth, side offset and angle: the target is\n"
                  "   reached when all three are green. Position AND angle count. The target changes between\n"
                  "   trials (D1 or D2, shown in the window). Press SPACE to start each trial and ENTER when\n"
                  "   you are done.\"\n"
                  f"  ({args.practice} practice trial(s), then {args.trials} measured trials)\n")
            while True:
                input("Press ENTER here to start the controller for this condition...")
                env = dict(os.environ, ASSIST=cond, OPERATOR="human", PARTICIPANT=pid, TRIALS=str(args.trials),
                           PRACTICE=str(args.practice), TIMEOUT=str(args.timeout), LABEL=label)
                t0 = time.time()
                code = subprocess.call(["ros2", "run", "cern_shared_control", "shared_control"], env=env)
                if code == 0:
                    break
                print(f"\nThe controller stopped with code {code} before the end of the condition.")
                if ask("Repeat this condition from the start? [y/n]: ", ["y", "n"]) == "n":
                    break
            entry = {"condition": cond, "position": i, "duration_s": round(time.time() - t0, 1),
                     "completed": code == 0}
            entry["tlx"] = nasa_tlx(pid, cond, i, label)
            session["conditions"].append(entry)
            with open(session_path, "w") as f:
                json.dump(session, f, indent=2)
    finally:
        if window is not None:
            window.terminate()

    banner("FINAL QUESTIONS (participant answers)")
    labels = {str(i): c for i, c in enumerate(order, start=1)}
    print("Conditions: " + ", ".join(f"{k} = condition {k} ({BRIEFING[c].split(':')[0]})" for k, c in labels.items()))
    first = ask("  Which condition did you prefer most? [1/2/3]: ", list(labels))
    last = ask("  Which condition did you prefer least? [1/2/3]: ",
               [k for k in labels if k != first])
    middle = next(k for k in labels if k not in (first, last))
    session["preference"] = [labels[first], labels[middle], labels[last]]
    session["comment"] = input("  Any comment about the task or the assistance? (ENTER to skip): ").strip()
    session["finished"] = datetime.now().isoformat(timespec="seconds")
    with open(session_path, "w") as f:
        json.dump(session, f, indent=2)
    banner(f"Session {pid} complete - saved to {HUMAN}. Thank the participant!")


if __name__ == "__main__":
    main()
