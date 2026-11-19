# RobotArmV2 Implementation Plan (original plan, kept for history — see design.md "Changes since the original design")

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans (Native — done in one chat). Steps use `- [ ]`.

**Goal:** a smooth control GUI (jog / target / slider) for the 4-joint arm via the COM11 adapter
**Architecture:** `Mover` (motion.py) computes a setpoint every 40 ms → `Driver` (driver.py) sends it through `DirectLink` (link.py) → the GUI
**Tech:** Python 3.11, numpy, pyserial, pytest (GUI later moved to PyQt6 + PyVista) · **Spec:** `docs/design.md`

## Global Constraints
- Calibration only in `src/config.py` · no sending below 6 V · J5 locked at 0 · nothing moves on its own when opening/connecting
- Tests: `cd src && ../.venv/Scripts/python.exe -m pytest -q`

## Review Focus
1. jog hits a limit → stops smoothly at the limit, never past it — test `test_jog_stops_at_limit`
2. xyz jog into an unreachable point → stops + reason, no jump — `test_jog_xyz_unreachable_stops`
3. a new target mid-move → continues from the setpoint, no jump — `test_new_goal_mid_move_no_jump`
4. voltage drop / offline mid-move → stop — `test_driver_stops_on_low_volt`
5. no step of any joint jumps more than v_max·dt·1.05 — checked in every Mover test by the `_run` helper

### Task 1: repo skeleton + config + link
Files: `src/config.py` (drop the ESP32/old correction values, add `PERIOD_S`, `STREAM_MS`, speed/acceleration values, `JUMP_GUARD_DEG`), `src/link.py` (`DirectLink`, `SimLink`, + `torque(on)`), tests `test_link.py`
- [x] tests: streamed `send_q` sends the short time · too-large jump → not sent · nan not sent · connect = hold + torque · never sends/reads J5 · SimLink follows instantly
- [x] implement → pass → commit

### Task 2: motion.Mover
Interfaces: `Mover(q0, v, a, v_cart)`; `.q`, `.busy`, `.message`; `goto_q(q)->str`, `goto_xyz(x,y,z,phi)->str` (""=ok), `jog_joint(i,d)`, `jog_xyz(axis,d)`, `release()`, `follow(q)`, `stop()`, `reset(q)`, `step(dt)->list`, `peek(dt)->list`
- [x] tests: goto_q reaches the goal within T+dt, all joints together, speed ≤ v, start/end speed ~0 · goto_xyz stays < 1 mm off the line and arrives · unreachable end → message, no motion · jog ramp/decel · jog limit · jog xyz unreachable · follow catches up without overshoot · stop · new goal mid-move
- [x] implement → pass → commit

### Task 3: driver.Driver
Interfaces: `Driver(link, mover, period=0.04)`; `.start()`, `.close()`, `.correct=True`, `.message`; thread: step → send_q · voltage/offline → stop · correction after a move ends
- [x] tests (SimLink + a fake sagging arm): continuous streaming, stop on low voltage/offline, correction brings error ≤ 0.5°, ≤ ±5°
- [x] implement → pass → commit

### Task 4: GUI + launchers + README/CLAUDE.md
- [x] tests (hidden window): jog press/release → mover.jog/release · target → goto · slider follow · stop · no arm = simulation
- [x] implement → pass → commit · README (ASCII window sketch + how to use), CLAUDE.md (safety rules)

### Task 5: real-arm tests (ask the user every time) — done 2026-11-19, see CLAUDE.md for the measurements
