# Part D: Control (project page section)

## Summary
The path tracker is state-feedback LQR on the cross-track error model `x = [e_y, e_theta]`, `w = -K x`, `K = [6.94, 2.54]` from Bryson-rule weights, computed from the EKF pose only. The reference path is a straightened A* route held in the EKF frame (`ReferencePath`) and replaced only when blocked. When `|e_theta| > 0.4` rad the robot rotates in place on heading error alone, because the error model loses controllability at zero speed.

All results come from one final set of 165 simulation trials with **five seeds per condition** (same source snapshot in every trial). Five paired seeds cannot give a p-value below 0.0625, so results are reported as means ± sd, Wilson intervals and paired seed counts, and described as directions, not as significant differences.

**LQR against the baseline bearing-law tracker** (4 conditions × 5 seeds × 2 controllers = 40 trials). The LQR keeps the robot closer to the latched reference in every condition: cross-track RMS 7.8 against 15.9 cm nominal (lower in 5/5 paired seeds), 9.0 against 15.1 cm with camera noise and dropout, 7.7 against 13.4 cm on the slippery floor. Driving effort is 3–7× lower. In the shifted-obstacle condition it completed 5/5 trials against 2/5 for the baseline (Fisher p = 0.167, McNemar p = 0.25: a clear direction, not a demonstrated difference at five seeds). The price is time: 21–36% of its tracking time is spent rotating in place and docking takes 6–10 s longer. Most of its remaining error comes from the estimator (70–76% of the true cross-track error), not from the controller. An opt-in no-path recovery fixed all three stalled baseline seeds (5/5 against 2/5). It works by nudging the tilted LiDAR map, not by correcting heading: no marker update was accepted during any recovery episode and the heading error changed by at most 0.5°.

**Actuator compensation.** The simulated wheels have a friction dead zone of about 2 rad/s and deliver 0.85 of the nominal ground speed, so the uncompensated robot drives at 0.094 m/s for a 0.18 m/s command and rotates in place for 27–30% of its tracking time. A feedforward wheel-command compensator (`actuator_compensation.py`, identified from open-loop bench runs) was evaluated over 60 trials (3 controllers × 4 conditions × 5 seeds, all arrived). With it, rotate-in-place time falls from 9.4–11.5 s to 2.1–3.2 s per run, total control effort falls by 69–76%, and time to dock by 29–34%; both fall in all 20 paired (condition, seed) pairs. Cross-track error is not resolvably changed (paired medians −0.6 to +1.0 cm, smaller than the seed-to-seed spread). Compensating the docking phase as well (`compall`) is another 11–14 s faster but docks 0.9–1.4 cm less accurately with about 30% fewer camera updates, so it is reported as an ablation only.

**Gain sweep.** Plain LQR works for R×0.25 to R×1 (5/5 in both conditions) but fails at R×2 (4/5 nominal, marginal at 100 s of a 110 s limit; 0/5 with camera noise) and R×4 (0/5). All 11 failed traces show the same cause: near the 0.25 rad exit threshold the turn command (0.50 rad/s at R×2, 0.39 at R×4) falls inside the actuator dead band, the robot stops turning at about 0.27–0.32 rad of heading error and cannot finish rotating. With the compensator all 40 compensated sweep trials arrived, at R×0.25 to R×4. At a matched R of 1 or below the compensator cuts total effort by about 70% and ties on cross-track error at R×1 (7.6 against 7.7 cm nominal) but is 1–3 cm worse at R×0.25 and R×0.5.

**Open items.** The rotate-in-place threshold study is not run. The causes of the slowest plain (shifted-obstacle seed 2, 61.6 s) and compensated (shifted-obstacle seed 0, XTE 16.7 cm) seeds were not investigated.

Results, plots and raw files: [`evidence/part-d/`](robotis_tb3/evidence/part-d/). Code: [`robotis_tb3/lqr/`](robotis_tb3/lqr/).

![Cross-track error per trial](robotis_tb3/evidence/part-d/eval_cmp/tracking_boxplot.png)

## Parameters
Control period 0.1 s; `V_NOM = 0.18` m/s; yaw saturation ±0.8 rad/s; `Q = diag(100, 8.2)`, `R = 1.56`; rotate-in-place enter 0.40 rad, exit 0.25 rad; segment switch 0.10 m before the vertex; reference replaced after 3 blocked frames, when the goal moves, or when the robot is 0.4 m off it. Actuator compensator (`--compensate path`): ground speed per wheel rate 0.0281 m/rad, yaw per wheel-rate difference 0.075, friction offset 2.0 rad/s, lag 0.11 s, command limit ±7.88 rad/s, not used while docking.

## Limits, stated plainly
Five seeds per cell, so every comparison is a direction and a count, and success rates have wide intervals (5/5 is 0.57–1.00). The comparison against the baseline (40 trials) was run on the uncompensated robot; the compensator was evaluated separately on the LQR only (60 trials including the plain LQR), and there is no compensated baseline. In the shifted-obstacle condition the baseline's tracking metrics rest on its two surviving trials. The estimator, not the controller, limits cross-track error (est−truth gap about three quarters of the true error), and the EKF is overconfident (Part C). Seeds vary noise, not the scene layout. Cross-track error is measured against a latched reference that the baseline does not track, and each reference replacement resets it. Docking is the shared relay law. Simulation only; no hardware result is claimed.

## Run it
```sh
python -m unittest discover -s robotis_tb3 -p "test_track_a*.py" -v
python robotis_tb3/track_a_project.py --headless --controller lqr --output out_lqr
python robotis_tb3/lqr/evaluate_controllers.py --controllers heuristic lqr --trials 5 --output eval_cmp
python robotis_tb3/lqr/analyse_controllers.py eval_cmp/summary.json --out eval_cmp
python robotis_tb3/lqr/plot_tracking.py out_lqr/trace.json --occupancy out_lqr/occupancy.npy --out tracking.png
python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr+comp lqr+compall --trials 5 --output eval_comp
python robotis_tb3/lqr/sweep_compare.py eval_sweep/summary.json eval_cmp/summary.json eval_comp/summary.json eval_sweep_comp/summary.json --out figures
python robotis_tb3/lqr/diagnose_failures.py .
```
The full command list, including the sweeps and recovery runs, is under *Reproduce* in the write-up. The `eval_*` folders are about 1 MB per trial and are not committed; each `summary.json` and the generated tables are.

## Generative AI declaration (Part D) 

> Claude (Anthropic, via claude.ai) was used for Part D to: review the path-error model and LQR design; write and test `lqr/track_a_control.py` (the latched reference path and the LQR tracker), `lqr/evaluate_controllers.py`, `lqr/analyse_controllers.py`, `lqr/part_d_summary.py`, `lqr/plot_tracking.py` and their tests; `lqr/plot_trace.py` and `lqr/sweep_compare.py` (diagnostic plots and the compensated R-sweep comparison); `lqr/diagnose_failures.py`; the analysis of the compensator and sweep results; add the opt-in no-path recovery and the rotate-threshold option; diagnose the baseline stall from the saved traces and occupancy grids; and draft and revise sections of `PART_D_CONTROL.md` and this page, including re-analysing the final five-seed data set. **Ethan** ran all simulations personally, parsed through, expanded and corrected generated code base.