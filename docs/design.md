# Design: RobotArmV2 — smooth (jerk-free) control GUI for the 4-joint arm

Date 2026-11-19 · approved in chat · builds on `D:\PIM\RobotControl\v2` (code copied over, no cross-repo imports)

## Why
The v2 GUI sent the final angles in one go → the servo starts/stops instantly at constant speed, each joint covers a different distance, and the correction step jerks again → the arm moves jerkily.
The user wants every kind of control in one window: **jog (hold a button), target (angles / XYZ), live slider** — and the arm must move smoothly.

## Goals / non-goals
- ✅ ramp up/down on every move · all joints start and arrive together · XYZ target makes the tip travel a straight line · immediate stop
- ✅ usable without an arm (simulation) · built-in calibration (`calib_app.py`)
- ❌ no ESP32 support (COM11 adapter only) · no teach/playback · no Jacobian velocity panel
- Fails if: the arm lunges or exceeds the set speed, STOP does not stop, commands are sent below 6 V

## Options
| Option | | |
|---|---|---|
| **A. PC streams small pose steps every 40 ms with speed + acceleration limits** | one mechanism serves jog / target / slider | **chosen** |
| B. send the final target once and let the servo time it (as in v2) | jerky, no jog | not chosen |
| C. set the speed inside the servo | the SCS15 has no acceleration setting, still stops abruptly | not chosen |

## Design
| File | Job |
|---|---|
| `config.py` | arm values + calibration (copied from v2 — **the only copy in this repo**) + speed values |
| `kinematics.py`, `scs.py` | copied from v2 |
| `link.py` | `DirectLink` (adapter) + `SimLink` (simulation) — `send_q(q, time_ms)`, `stop()`, `q_actual`, `volt`, `online()`, `torque(on)` |
| `motion.py` | **the pose streamer `Mover`** (no GUI/hardware dependency — testable) |
| `driver.py` | 40 ms thread: `Mover.step()` → `link.send_q(...)` · voltage/online checks · correction after arriving |
| `qt_app.py`, `viewer3d.py` | the window (see "Changes since the original design") |
| `calib_app.py`, `servo_calibrate.py` | copied from v2 (write only `config.py`) |

**Mover** (degrees, seconds; setpoint `q` = the pose being commanded):
- `goto_q(q_goal)` — joint space: every joint shares the min-jerk profile `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵` → arrive together, T = 1.875·Δmax / v_max
- `goto_xyz(x, y, z, φ)` — straight line in space: IK every ~1 mm along the line, min-jerk timing with T long enough that the tip ≤ v_cart and every joint ≤ v_max · a point without IK → go in joint space to the end point's IK (end point without IK = rejected)
- `jog_joint(i, ±1)` / `jog_xyz(axis, ±1)` / `release()` — velocity ramps up at a_max, release eases to a stop · xyz jog: tip += v·dt then IK — no IK / limit → stop with a reason
- `follow(q_goal)` — slider: chase at v ≤ v_max, easing in before arrival (v ≤ √(2·a·|e|))
- `stop()` — v = 0 immediately, target dropped · `reset(q)` — setpoint = actual angles
- every value clamped to `Q_MIN..Q_MAX` · J5 (removed) locked at 0

**Sending to the servos:** every tick send the pose with a servo move time so the servo interpolates between points — DirectLink's streaming mode does not enforce 300 ms but **rejects commands where a joint jumps too far in < 300 ms** (bug guard)

**Safety:** voltage < 6 V / offline → `stop()` + hold at the actual angles + a message · STOP = `stop()` + `link.stop()` (goal = actual position) + `reset(actual)` · opening the program / connecting = hold in place, no motion

**Correction (can be turned off, on by default):** after a move finishes wait 0.4 s → joints with |error| > 0.5° and < 5° → `goto_q(setpoint + 0.8·error)` smoothly, ≤ 3 rounds, total ≤ ±5°

## Changes since the original design (all asked for by the user)
- GUI: tkinter → **PyQt6 + PyVista 3D in the ws-control style** (Joint / Jog / IK / Status tabs, FK box with T⁰₅, log)
- **No speed limit in the GUI**: v = 150 °/s, a = 600 °/s² for sliders/targets; sliders drive the arm directly (no Live checkbox)
- Jog has its own slower values (30 °/s, 90 °/s², tip 40 mm/s) and eases out at 0.5·a; correction also runs after a jog stops
- **Look-ahead streaming**: send the pose 80 ms ahead with an 80 ms servo time (Windows ticks jitter 27–52 ms); jump guard 20°
- **Singularity check** `singularity.py`: |det J| = r·a2·a3·|sin θ3|, proven against a numerical Jacobian, shown in the GUI; tip jog stops before entering one
- IK near the base axis uses the slides' second solution (θ1 + 180°, negative r) to avoid a false "base out of limit 180°"

## Left to the builder
Helper function names, GUI look, IK spacing along the line (≈1 mm)
