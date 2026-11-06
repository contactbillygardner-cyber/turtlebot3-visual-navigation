# Track A: Docking a TurtleBot3 on the estimated state

**41014 Sensors and Control in Mechatronics Systems, Assessment 4: Integrated Project**

**Group 11**: Ethan Wong - 13608993, Ibrahym Abdillah -25782362, Ho Youn Garwood (), William Gardener ()
**Repository:** [[GitHub repository URL](https://github.com/Ewongated/Turtlebot3-Sim-Sensors.git)]
If access to Repository is restricted contact ethanqcw@gmail.com as the group member with ownership rights over the repo has left the team.
**Demo video** included inside robotis_tb3
**Platform:** simulation only (MuJoCo), no hardware.

A simulated TurtleBot3 Waffle Pi crosses a room with obstacles by LiDAR path planning, tracks the path with an LQR state-feedback law, and docks at an ArUco-marked charging station. Everything the controller sees comes from an EKF that fuses wheel odometry with the camera. Simulator ground truth is used only to score runs.

| Result (5 seeds, LQR + actuator compensation) | Value |
|---|---|
| Docked, all four test conditions | 20/20 trials |
| EKF position RMSE, nominal | 0.067 m (odometry alone: 0.32 m) |
| RMS cross-track error, nominal | 0.076 m (baseline tracker: 0.159 m) |
| Mean time to dock, nominal | 35 s (baseline: 46 s) |

## System overview

```
encoders ──► EKF [x, y, θ] ──► path projection ──► LQR  w = −K[e_y, e_θ] ──► (v, w)→wheel rates ──► MuJoCo
camera (ArUco range, bearing, heading) ──► EKF correction
LiDAR 360 rays ──► rolling occupancy grid ──► inflated grid path to dock
```

The loop runs at 10 Hz. The EKF, planner and tracker take only sensor readings and the EKF pose. MuJoCo state enters only through `evaluation_pose()` and `evaluation_collision()` in `track_a_sensors.py`, which feed the metrics and never the control path.

## Model and frames

Differential drive. Wheel radius *r* and track width *b* are read from the MuJoCo model, and the encoder wheel-angle increments (Δl, Δr, rad) drive an exact circular-arc prediction:

```
Δs = r (Δl + Δr)/2        Δθ = r (Δr − Δl)/b
x+ = x + (Δs/Δθ)(sin(θ+Δθ) − sin θ)
y+ = y + (Δs/Δθ)(cos θ − cos(θ+Δθ))     (straight-line form when Δθ ≈ 0)
θ+ = θ + Δθ
```

**Frames.** World W: x toward the marker, y left, θ anticlockwise from +x, origin at the start pose. The base frame B is the pose the filter estimates. The camera C is mounted 0.09 m ahead of B and tilted 10°. The ArUco marker (0.22 m side) is a fixed landmark at (3.0, 0) in W. The camera gives marker range and bearing, converted to the base pose with the 0.09 m offset inside the measurement function. The LiDAR feeds the occupancy grid through the EKF pose.

**Observability.** Odometry alone integrates error without bound, so the pose is not observable from it. With the marker in view, the camera range, bearing and heading give a 3×3 measurement Jacobian of rank **3** at a representative pose, so the full pose is observable. With no marker sighting the filter dead-reckons and its covariance grows until the next accepted update.

**Controllability.** For path tracking the error state is x = [e_y, e_θ] with e_y' = v sin e_θ ≈ v e_θ and e_θ' = w. The controllability matrix [B, AB] has rank 2 exactly when v ≠ 0. At v = 0 lateral error cannot be fixed by turning, so the tracker rotates in place on heading error alone and only drives once |e_θ| falls below a hysteresis threshold.

## Estimation

State: pose [x, y, θ]. The correction uses a three-component observation (range, bearing, heading) and the Joseph-form covariance update. A correction is rejected if its normalised innovation squared exceeds 11.345, the 99% χ² limit for 3 degrees of freedom.

| Parameter | Value |
|---|---|---|
| Wheel increment σ | 0.002 + 4% of \|Δ\| (rad) |
| Process floor (x, y, θ) | 0.003 m, 0.003 m, 0.02 rad |
| R camera (range, bearing, heading) | 0.03 m, 1.5°, 8° |
| P₀ | diag(0.06², 0.06², 5°²) |
| Injected encoder bias | 0.002 nominal, 0.004 stressed |

Values are from `track_a_estimation.py`.

![trajectory](assets/trajectory.png)

*Top-down path, nominal LQR + compensation run (seed 0, 34.8 s). Raw odometry drifts; the EKF estimate stays on the true path.*

![estimation error](assets/estimation_error.png)

*Position error over time, same run. EKF RMSE 0.063 m against 0.306 m for odometry. The error is inside the filter's own 3σ bound for 78% of samples.*

**What this does not show.** Mean NEES over nominal runs is about 6, against an expected 3 for a consistent 3-state filter, and the error is within 3σ only 78% of the time. The filter is overconfident. A likely contributor is the pose-only state, which has no encoder-bias or scale state while the simulated wheels deviate from nominal kinematics. Inflating Q or R would bring NEES nearer 3, but we have not tuned for consistency, so we report it as a limitation.

## Control

Discrete LQR on the error model above, discretised exactly at 10 Hz and designed at v = 0.18 m/s. Weights follow Bryson's rule: Q = diag(1/0.10², 1/0.35²) = diag(100, 8.2) from a 0.10 m lateral and 0.35 rad heading limit, and R = 1/0.8² = 1.56 from the 0.8 rad/s yaw limit.

```
K = [6.94  2.54]      closed-loop poles at v = 0.18: s = −0.67, −1.87 (real, no overshoot)
w = −K [e_y, e_θ]ᵀ, clipped to ±0.8 rad/s
```

The controller reads the EKF pose only, projecting it onto the current path segment to form e_y and e_θ. The comparison baseline is a pure-pursuit-style heuristic, w = 2.8 × bearing to a point 0.20 m ahead, equivalent to a fixed gain of [14, 2.8].

**Actuator compensation.** Bench tests on the MuJoCo wheels showed a dead zone of about 2 rad/s, about 0.85 of nominal ground speed, 0.5 to 0.75 of nominal yaw rate and a 0.11 s lag. Without correction the baseline reaches only half its commanded speed (0.089 against 0.178 m/s in the reference run). A feed-forward map from (v, w) to wheel rates restores it. A PI trim on top made turning worse, so it is off.

## Results

Each row is the mean of five seeds (0 to 4). Columns: successful docks, EKF and odometry position RMSE, mean NEES, RMS cross-track error against ground truth (XTE), control effort ∫w², time to dock.

**Baseline heuristic tracker**

| Condition | Success | EKF RMSE [m] | Odo RMSE [m] | NEES | XTE rms [m] | Effort | Time [s] |
|---|--:|--:|--:|--:|--:|--:|--:|
| Nominal | 5/5 | 0.097 | 0.476 | 8.7 | 0.159 | 4.6 | 46 |
| Slippery floor (μ 0.6) | 5/5 | 0.097 | 0.473 | 8.7 | 0.134 | 4.4 | 45 |
| Obstacle shifted 0.15 m | 2/5 | 0.078 | 0.329 | 4.7 | 0.069 | 3.5 | 86 |
| Camera noise + 30% dropout | 5/5 | 0.097 | 0.497 | 7.7 | 0.151 | 4.5 | 51 |

**LQR tracker**

| Condition | Success | EKF RMSE [m] | Odo RMSE [m] | NEES | XTE rms [m] | Effort | Time [s] |
|---|--:|--:|--:|--:|--:|--:|--:|
| Nominal | 5/5 | 0.073 | 0.414 | 6.7 | 0.078 | 6.7 | 52 |
| Slippery floor (μ 0.6) | 5/5 | 0.072 | 0.341 | 6.4 | 0.077 | 7.7 | 54 |
| Obstacle shifted 0.15 m | 5/5 | 0.099 | 0.421 | 8.3 | 0.127 | 8.4 | 57 |
| Camera noise + 30% dropout | 5/5 | 0.076 | 0.370 | 6.0 | 0.090 | 6.6 | 58 |

**LQR tracker with actuator compensation (final configuration)**

| Condition | Success | EKF RMSE [m] | Odo RMSE [m] | NEES | XTE rms [m] | Effort | Time [s] |
|---|--:|--:|--:|--:|--:|--:|--:|
| Nominal | 5/5 | 0.067 | 0.321 | 5.9 | 0.076 | 1.7 | 35 |
| Slippery floor (μ 0.6) | 5/5 | 0.071 | 0.356 | 6.0 | 0.082 | 1.9 | 35 |
| Obstacle shifted 0.15 m | 5/5 | 0.109 | 0.425 | 8.0 | 0.133 | 2.6 | 38 |
| Camera noise + 30% dropout | 5/5 | 0.071 | 0.300 | 5.3 | 0.094 | 1.9 | 41 |

**What the numbers say**

- **LQR tracks better.** Nominal cross-track error falls from 0.159 m to 0.076 m. Part of the heuristic's error is the actuator problem: it commands 0.178 m/s but moves at about half that.
- **Compensation removes the effort and time penalty.** Plain LQR docks in about 52 s with ∫w² near 6.7, spending roughly 10 s rotating in place because the uncompensated wheels cannot turn at low yaw demand. With compensation it docks in 35 s with ∫w² of 1.7, and rotation drops to about 2 s.
- **A failure traced to its cause.** The baseline docks in only 2/5 shifted-obstacle trials. The 0.15 m shift leaves the planner with no route for a stretch and the robot stalls. A recovery behaviour (rotate to search for the marker after 1 s without a path) lifts this to 5/5. The LQR variants handle the same shift in 5/5 trials without it.
- **Noise and dropout cost little.** With 5 px pixel noise and 30% camera dropout, EKF RMSE for the compensated LQR moves by under 0.01 m, consistent with odometry bridging the gaps while the NIS gate rejects bad frames.

**LQR weight sweep** (nominal condition)

| R scale | K (e_y, e_theta) | No comp: success | XTE rms | Effort | Comp: success | XTE rms | Effort |
|---|--:|--:|--:|--:|--:|--:|--:|
| 0.25 | 12.38, 4.12 | 5/5 | 0.075 | 8.3 | 5/5 | 0.089 | 2.5 |
| 0.5 | 9.36, 3.24 | 5/5 | 0.056 | 6.1 | 5/5 | 0.083 | 2.0 |
| 1 (design) | 6.94, 2.54 | 5/5 | 0.078 | 6.7 | 5/5 | 0.076 | 1.7 |
| 2 | 5.08, 1.98 | 4/5 | 0.079 | 20.6 | 5/5 | 0.081 | 1.6 |
| 4 | 3.68, 1.56 | 0/5 | 0.048 | 23.8 | 5/5 | 0.093 | 1.5 |

Without compensation, large R breaks docking (4/5 at R = 2, 0/5 at R = 4): the robot spends most of the run rotating in place (about 60 s at R = 2, about 98 s at R = 4) and times out, consistent with the gentler yaw commands falling inside the actuator dead zone. With compensation every scale docks in all trials and the choice becomes a trade between tracking error and effort. Here a controller limit came from the plant, not the design equations.

## Limitations

Results are simulation only, with five seeds per condition, so small differences (for example between R scales) are within trial scatter. The EKF is overconfident (NEES ≈ 6). There is a single marker, so pose is only observable while it is in view. We did not run on hardware.

## Install and run

```bash
git clone [REPO URL] && cd [REPO]
python -m venv .venv && source .venv/bin/activate
pip install mujoco numpy scipy opencv-contrib-python   # ArUco needs the contrib build

# final configuration, headless, seed 0
python track_a_project.py --headless --controller lqr --compensate path --seed 0 --output out

# stressed trial
python track_a_project.py --headless --controller lqr --compensate path --seed 1 \
    --friction 0.6 --dropout 0.3 --pixel-noise 5 --output out-stress
```

Each run writes `result.json`, a full `trace.json` (estimate, truth, covariance, NIS, NEES, commands) and a source snapshot with hashes in `source-manifest.json`. Other flags: `--r-scale`, `--q-scale`, `--lqr-r-scale`, `--obstacle-shift`, `--encoder-bias`, `--recovery`.

> Confirm the install line and flags on a fresh environment before submitting.

## Individual contributions

| Member | Contribution |
|---|---|
| [William] | [e.g. model, observability and controllability analysis] |
| [Ibrahym] | [e.g. sensing, camera and ArUco pipeline, frames] |
| [Ho Youn] | [e.g. EKF design and tuning] |
| [Ethan] | [e.g. LQR design, actuator compensation, evaluation] |

## Generative AI declaration

Claude (Anthropic) was used to generate and review baseline code, which was then reviewed by group members. Claude was also used to draft this project page from the group's result files.

## Acknowledgements

Built from the weekly tutorial code, MuJoCo, and the ROBOTIS TurtleBot3 model (Apache 2.0, see `LICENSE`).
