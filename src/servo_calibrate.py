# Recalibrate the arm joint by joint via the USB adapter (COM11) — find center / direction / limits and write config.py
# Never writes the servos' permanent memory (EEPROM) — only torque and goal
#
#   read            read raw / q / torque / voltage of every connected joint (read-only)
#   free  j2|all    torque off  ⚠ that joint goes limp immediately — support the arm first
#   hold  j2|all    hold at the current position, then torque on (the arm does not move)
#   sweep j2 [sec]  record the min/max raw while the user moves that joint by hand to both ends (free it first)
#
# Procedure (one joint at a time, support the arm throughout):
#   1) free → hold that link straight up → read  = center
#   2) move it the positive way (lean forward / base turns left / head tilts right) → read : raw up = dir +1, down = dir −1
#   3) sweep → move to both ends as far as nothing collides → limits (shrunk inward by 10 ticks)
#   4) hold
import re
import sys
import time
from pathlib import Path

import serial

import config
from scs import ScsBus

LIMIT_MARGIN = 10      # shrink the limits inward from the positions actually reached, as a safety margin


def pick_joints(name):
    """'j1'..'j5' or 'all' (connected joints only) → list of joint dicts"""
    used = [j for j in config.JOINTS if j["used"]]
    if name == "all":
        return used
    for i, j in enumerate(config.JOINTS):
        if name == f"j{i + 1}" and j["used"]:
            return [j]
    sys.exit(f"Unknown joint '{name}' — use j1..j{len(config.JOINTS)} (connected ones) or all")


def read_all(bus, joints):
    rows = []
    for j in joints:
        got = bus.read_pos_volt(j["id"])
        t = bus.read(j["id"], 40, 1)
        rows.append({
            "name": j["name"], "id": j["id"],
            "raw": got[0] if got else None,
            "q": round(config.raw_to_q(j, got[0]), 1) if got else None,   # using the center/dir in config right now
            "volt": got[1] if got else None,
            "torque": bool(t[0]) if t else None,
        })
    return rows


def free(bus, joints):
    for j in joints:
        bus.torque(j["id"], False)


def hold(bus, joints):
    for j in joints:
        got = bus.read_pos_volt(j["id"])
        if got:
            bus.write_pos(j["id"], got[0], config.MIN_MOVE_MS)   # goal = where it is now → no motion
        bus.torque(j["id"], True)


def sweep(bus, joint, seconds=15.0, period_s=0.05):
    """read raw repeatedly until time runs out → (min, max) seen"""
    lo, hi = None, None
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        got = bus.read_pos_volt(joint["id"])
        if got:
            lo = got[0] if lo is None else min(lo, got[0])
            hi = got[0] if hi is None else max(hi, got[0])
        time.sleep(period_s)
    return lo, hi


def suggest_limits(lo, hi):
    return lo + LIMIT_MARGIN, hi - LIMIT_MARGIN


# ---------- write calibration values into config.py ----------
CONFIG_PY = Path(__file__).resolve().parent / "config.py"


def update_config_py(text, values):
    """values = {id: (dir, center, raw_min, raw_max)} → rewrite that id's JOINTS row in config.py"""
    for sid, (d, c, lo, hi) in values.items():
        pat = rf'"id": {sid}, "dir": [+-]1, "center": \d+, "raw_min": \d+, "raw_max": \d+'
        new = f'"id": {sid}, "dir": {d:+d}, "center": {c}, "raw_min": {lo}, "raw_max": {hi}'
        text, n = re.subn(pat, new, text)
        if n != 1:
            raise ValueError(f"Row for ID {sid} not found in config.py")
    return text


def save_calibration(values, py_path=CONFIG_PY):
    """validate first, then write config.py (bad values → ValueError, nothing written)"""
    for sid, (d, c, lo, hi) in values.items():
        if d not in (1, -1) or not 0 <= lo < c < hi <= config.RAW_MAX:
            raise ValueError(f"ID {sid}: need dir ±1 and 0 ≤ min < center < max ≤ {config.RAW_MAX}")
    py = update_config_py(Path(py_path).read_text(encoding="utf-8"), values)
    Path(py_path).write_text(py, encoding="utf-8")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__ or "usage: servo_calibrate.py read | free j2 | hold j2 | sweep j2 [seconds]")
    cmd = sys.argv[1]
    target = sys.argv[2] if len(sys.argv) > 2 else "all"
    ser = serial.Serial(config.DIRECT_PORT, config.SERVO_BAUD, timeout=0.002)
    bus = ScsBus(ser)
    try:
        bus.detect_echo()
        if cmd == "read":
            for r in read_all(bus, pick_joints(target)):
                print(f"{r['name']:12} ID {r['id']:2}  raw {r['raw']!s:5} q {r['q']!s:6}  "
                      f"torque {r['torque']!s:5}  volt {r['volt'] and round(r['volt'], 1)} V")
        elif cmd == "free":
            free(bus, pick_joints(target))
            print(f"torque off: {target}")
        elif cmd == "hold":
            hold(bus, pick_joints(target))
            print(f"holding in place + torque on: {target}")
        elif cmd == "sweep":
            seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
            joint = pick_joints(target)[0]
            print(f"{joint['name']}: move it by hand to both ends within {seconds:.0f} s ...")
            lo, hi = sweep(bus, joint, seconds)
            if lo is None:
                print("nothing could be read")
            else:
                print(f"saw raw {lo}–{hi} → suggested limits {suggest_limits(lo, hi)}")
        else:
            sys.exit(f"unknown command {cmd}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
