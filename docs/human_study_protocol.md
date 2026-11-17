# Human study protocol: assisted teleoperation towards an inspected defect

## Question

Do virtual fixtures generated automatically from the inspection pass help a human operator reach a defect more accurately and more safely, with less workload, than no assistance or static keep-out planes?

## Design

- **Within-subject:** every participant does the 3 conditions, `none`, `static` and `fixtures`.
- **Counterbalanced order:** participant k uses order (k − 1) mod 6 of the 6 possible orders (`run_session.py`). Use a multiple of 6 participants if possible (6 or 12).
- **Per condition:** 1 practice trial (not analysed), then 5 measured trials.
  - The target changes every trial: the defects of the inspection report in turn (D1, D2, D1, …). The window and the green sphere show the active one.
  - The 6 start poses (each one relative to its trial's target) are always used in the same sequence, in every condition, so the conditions are paired and nobody can learn one fixed motion.
  - Time limit: 90 s per trial.
- **Task:** bring the tool tip onto the green target (30 mm in front of the defect), with the tool aligned with the face normal, without entering the keep-out zone or touching the equipment.
  - Press SPACE to start a trial and ENTER when done.
- **Same visual information in every condition:**
  - RViz scene, true defect (red), inspection estimate (orange), green target sphere and target frame.
  - The funnel is drawn only in the `fixtures` condition, because it is part of the assistance.
  - The keyboard window shows a readout of the tool relative to the green target: depth, side offset and angle. Each turns green inside the success tolerance. It acts like a range sensor on the tool. The pilot showed that depth cannot be judged from a single RViz view, and that operators ignore the orientation without feedback. Use `HUD=0` to hide it, but keep the same choice for every participant.
- **Same inspection report for every participant:** run the scan once before the study and keep `inspection_latest.json`. The runner copies it into each session folder.

## Participants

8–12 adult volunteers (students). This is a pilot study: report effect sizes and medians, not only p-values. Background questions recorded: age range, robot experience, keyboard-gaming experience, teleoperation experience.

## Measures

**Objective (controller, per trial):**
- position error, split into lateral and depth;
- orientation error;
- maximum keep-out intrusion;
- closest distance to the face;
- task time;
- path length;
- success: error < 5 mm, orientation < 3°, intrusion ≤ 1 mm, no contact.

**Subjective:**
- raw NASA-TLX after each condition (6 subscales, 0–100, unweighted mean). Performance is asked as 0 = failure … 100 = perfect, because the pilot participant read the original scale the wrong way. It is stored with the TLX convention (0 = perfect), so for every subscale higher = more workload;
- preference ranking and a free comment at the end.

## Analysis

Run `scripts/analyze_human.py`:
1. Compute each participant's mean over the measured trials of each condition.
2. Friedman test across the 3 conditions.
3. Pairwise exact Wilcoxon signed-rank tests with Holm correction.
4. Effect size: matched-pairs rank-biserial correlation.

**Expected result (hypotheses):** H1, fixtures give a lower position and orientation error than none and static. H2, fixtures and static give less intrusion than none. H3, fixtures give a lower workload than none and static.

## Session (about 45 min)

1. Welcome. The participant reads and signs the consent form (`docs/consent_form.md`). Assign the next ID: P01, P02, …
2. `python3 scripts/run_session.py P0k`: background questions, then the keyboard window opens.
3. Explain the keys. Free practice for 2–3 minutes during the first practice trial.
4. For each condition: read the briefing shown by the runner, run 1 practice + 5 trials, then the participant fills in the NASA-TLX.
5. Preference ranking and comment. Thank the participant.

## Setup checklist (before each session)

- [ ] `ros2 launch ecn_manip simulation_launch.py robot:=kr16` is running, and the GUI is on **Manual Cartesian control**
- [ ] The same `~/cern_logs/inspection_latest.json` as for the previous participants
- [ ] The RViz view is saved (behind the robot, looking at the equipment face) and the same for everyone
- [ ] The laptop is plugged in and other programs are closed (the simulation needs the CPU)
- [ ] Consent form printed, participant ID chosen

## Data protection

- Files contain only the participant ID (P01, …). There is no name, e-mail or recording.
- Signed consent forms are stored separately from the data.
- A participant can withdraw at any time. Their files (`~/cern_logs/human/P0k_*`) are then deleted.
- Before publication, ask the supervisor whether the school's ethics process applies to this study.
