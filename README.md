# RobotArmV2 (English version) — smooth control for a 4-joint arm

English edition of [`BananaZz27/RobotArmV2`](https://github.com/BananaZz27/RobotArmV2) (the Thai original).

Arm: SCS15 × 4 (base ID 4, shoulder 9, elbow 7, wrist 2) via a Waveshare Bus Servo Adapter (A), jumper on B → **COM11** · servo supply **7.4 V only**
Built on top of RobotControl `v2` · design: `docs/design.md`

Looks like the earlier ws-control GUI (PyQt6 + PyVista 3D view) but **needs no ESP32** (the PC drives the servos directly) and **every move is smooth**.

```
[Port COM11][Connect] ● Connected 7.8 V                     [Torque OFF] [■ STOP (Space)]
┌───────── 3D (orange = Target, translucent blue = actual) ─────────┐ ┌ Joint | Jog | Inverse Kinematics | Status ┐
└───────────────────────────────────────────────────────────────────┘ └───────────────────────────────────────────┘
Forward Kinematics (Target pose): X Y Z φ Yaw + T⁰₅ + singularity check          Log
```

- **Start:** `RobotArm.bat` → Simulation mode → **Connect** → the arm moves to the **Standby pose** by itself, slowly (20 °/s, after checking ≥ 6 V and all 4 servos; STOP/Space stops it)
- **Standby pose:** put the arm in the pose you want → Joint tab → **📌 Set this pose as Standby** (saved to `src/config.py` `STANDBY_Q`) · the **⌂ Standby** button in the top bar goes there any time
- **Joint:** drag a slider / type a number → the arm follows immediately (no speed limit to set — full servo speed with a short ramp up/down) · Zero = back to straight up
- **Jog:** hold = keep moving, release = ease slowly to a stop (0.5·a to avoid bounce), then sag is corrected · per joint (J1–J4 ±) or tip in a straight line (X Y Z ±) — tip jog stops by itself before a singularity
- **Singularity (box right of FK):** red = the Target pose is near a singularity · proven with |det J| computed two ways, which must match
  (numerical 4×4 Jacobian vs the formula **r·a2·a3·|sin θ3|** → 0 when the tip is on the base axis or the arm is fully stretched/folded) + σ_min · tests check 300 random poses (`singularity.py`)
- **Inverse Kinematics:** X Y Z φ → Preview (elbow up/down solutions) / Go · if φ is unreachable the nearest reachable φ is used
- **Status:** per joint: online, raw, actual angle, error, temperature, load · supply voltage · sag-correction checkbox
- **STOP / Space:** the arm holds at its actual position immediately · **Torque OFF:** the arm goes limp and can be moved by hand (Target follows the real arm)
- Voltage < 6 V / servos not answering → nothing is sent / motion stops · **Calibration:** `Calibrate.bat` → `src/config.py`

How the maths works, with figures (FK, IK, min-jerk, jog, look-ahead streaming, correction, singularity): **`docs/explain.md`**

Code: `qt_app.py` (window) · `viewer3d.py` (3D) · `motion.py` (pose streamer) → `driver.py` (40 ms) → `link.py` (adapter / simulation)

Install: `py -3.11 -m venv .venv && .venv\Scripts\python.exe -m pip install -r requirements.txt`
Tests: `cd src && ..\.venv\Scripts\python.exe -m pytest -q`
Charts in `docs/figures` (optional): `pip install matplotlib`, then `cd src && ..\.venv\Scripts\python.exe ..\docs\make_charts.py`
