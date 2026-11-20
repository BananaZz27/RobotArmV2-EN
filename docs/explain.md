# How the maths works — RobotArmV2 (4-joint arm: base, shoulder, elbow, wrist; PC drives the servos directly)

Charts are generated from the real code by `docs/make_charts.py` (edit a formula and re-run) · the FK/IK arm figures are adapted from RobotControl `v2/docs` to the 4-row DH table

## 1. Overview: data flow from the window to the servos

![Data flow](figures/system_flow.svg)

| File | Job |
|---|---|
| `qt_app.py` | the window (Joint · Jog · IK · Status · 3D) |
| `motion.py` | **pose streamer** — computes the next pose step by step (sections 4–5) |
| `driver.py` | thread that sends poses to the arm + safety + correction (sections 6–7) |
| `link.py` / `scs.py` | q ↔ raw conversion, then SCS packets over COM11 |
| `kinematics.py` / `singularity.py` | FK, IK (sections 2–3) · singularity check (section 8) |

## 2. Forward Kinematics: joint angles → tip position

![Joints](figures/dof_overview.svg)
![Side view](figures/kinematic_side.svg)
![Top view](figures/kinematic_top.svg)

- 4-row DH table (head servo J5 removed → wrist to tip is one straight link, **a4 = 50 mm**):

| Link | a (mm) | α | d (mm) | θ |
|---|---|---|---|---|
| 1 base | 0 | 90° | 96.7 | θ₁ |
| 2 shoulder | 137.0 | 0 | 0 | θ₂ |
| 3 elbow | 98.3 | 0 | 0 | θ₃ |
| 4 wrist | 50.0 | 0 | 0 | θ₄ |
- The servo angle q (0 = straight up) is not the DH angle θ → convert before the equations (only in `q_to_theta`):

| Joint | θ (DH) | Why |
|---|---|---|
| J1 base | θ₁ = q₁ | base positive = counter-clockwise = positive θ about z₀ |
| J2 shoulder | θ₂ = 90° − q₂ | θ₂ is measured from horizontal · upright = 90° |
| J3 elbow | θ₃ = −q₃ | elbow leaning forward (positive q) = clockwise in the arm plane |
| J4 wrist | θ₄ = −q₄ | wrist leaning forward = clockwise, like the elbow |

- `T⁰₄ = A₁A₂A₃A₄` → 4th column = position (x, y, z) · **φ = θ2 + θ3 + θ4** (tip angle vs horizontal — the shoulder, elbow and wrist axes are parallel, so the angles simply add)
- The 4-row table gives exactly the same tip as the old 5-row table (α₄ = 90°, d5 = 50, θ₅ = 0) in every pose — a test checks 300 poses

## 3. Inverse Kinematics: position → joint angles

![IK steps](figures/ik_steps.svg)

1. θ1 = atan2(y, x) · 2. step back from the tip to the wrist along φ · 3. law of cosines gives θ3 · 4. θ2 · 5. θ4 = φ − θ2 − θ3
- **Near the base axis** (r < 30 mm) the slides' second solution θ1 + 180° with negative r is used when it is closer to the current base angle — otherwise a tip 0.2 mm behind the axis made atan2 jump to 180° (base out of limit)
- **φ not reachable** (position reachable, angle not) → the window finds the nearest reachable φ (1° steps) and says so in the Log

## 4. Smooth motion: the min-jerk profile (`Mover.goto_q`)

![min-jerk](figures/min_jerk.svg)

- **s(τ) = 10τ³ − 15τ⁴ + 6τ⁵**, τ = t / T → starts and ends with velocity = acceleration = 0 (no jerk at start/stop)
- Every joint uses the same s(τ): q(t) = q₀ + (q₁ − q₀)·s(t/T) → **all joints arrive together**
- Duration **T = 1.875 · Δ_max / v** (the peak velocity of s is 1.875 at τ = 0.5) · v = 150 °/s · going to Standby uses v = **60 °/s** (e.g. 90° takes 2.8 s)
- Straight line to X Y Z (`goto_xyz`): split the line every 1 mm, IK at every point, same s(τ) · T = 1.875 × max(length / 300 mm/s, time the fastest joint needs at 150 °/s)
- **Dragging a slider** (`follow`): the goal keeps changing, so no s(τ) — chase it with acceleration a = **600 °/s²** and ceiling **v ≤ min(150, √(2a·|error|))** → it slows down and stops exactly at the goal

## 5. Jog: hold / release / brake before a limit

![jog](figures/jog_ramp.svg)

- Velocity changes by at most **a·dt** per step (a = 90 °/s², jog 30 °/s, tip 40 mm/s) · on release: decelerate at **0.5·a** (full a made the arm bounce), then correct sag at the stop pose
- Near a limit: ceiling **v ≤ √(a · distance to the limit)** → arrives at the limit slowly (the old √(2a·distance) let the ramp fall behind and hit at ~14 °/s)
- Tip jog: tip += v·dt then IK, trying the current φ first then the nearest φ within ±30° (position matters more than angle) · if a joint would have to spin faster than the jog speed → shrink the step rather than stop

## 6. Sending the pose 80 ms ahead (`Mover.peek`) — fixing jerky jog

![Measured](figures/lookahead_measured.svg)

- Measured: a thread set to 40 ms on Windows actually ticks every **27–52 ms**
- Before: send "the current pose" + a fixed 60 ms → each step's distance varied with the tick → servo speed swung ±10.8 around a mean of 22.4 °/s (**±48%**)
- Now: send **the pose 80 ms ahead** + 80 ms → servo speed = (goal − now) / 80 ms ≈ v (measured mean 25.9 °/s for a set 30 — the servo lags slightly)
- Measured on the elbow jogging at 30 °/s: swing ±10.8 → **±5.8 °/s**, lag 4.4° → **2.5°**

## 7. Correcting sag after arriving (`Driver._correction`)

![Correction](figures/correction_loop.svg)

wait 0.4 s for the arm to settle → error = q_target − q_actual → joints with |error| > 0.5°: **offset ← offset + 0.8·error**, then send **q_target + offset** (smoothly), ≤ 3 rounds, total offset ≤ ±5° · a first error > 5° = never got there, no correction

- The offset **accumulates** each round (= integral control, one round at a time) → constant sag from gravity still reaches the goal · sending a fresh q_target + 0.8·error each round would never reach the goal (some error always remains)

## 8. Singularity: poses where the tip loses a direction (`singularity.py`)

![Two kinds](figures/singularity_cases.svg)
![Evidence](figures/singularity_det.svg)

- **|det J| = r · a2 · a3 · |sin θ3|** → 0 when the tip is on the base axis (r = 0) or the arm is fully stretched/folded (θ3 = 0°/180°)
- Evidence: compute J numerically (nudge each joint by 0.001°) and take det → **matches the formula in every pose** (left chart, tests on 300 random poses)
- In the window: the box right of Forward Kinematics shows det both ways + σ_min · red = near a singularity · tip jog stops before entering one

## 9. Servo angle q ↔ raw (`config.py`)

```
raw = center + dir × q × 4.092        (1023 ticks / 250° = 4.092 ticks/°)
q   = dir × (raw − center) / 4.092
```
center = raw when the link points straight up · dir = +1/−1 so positive q = lean forward / base turns left · recalibrate: `Calibrate.bat`
