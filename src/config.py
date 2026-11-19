# Every number for the arm lives in this one file (copied from RobotControl/v2 on 2026-11-19)
# Recalibrate: Calibrate.bat → Save → writes into this file

# ---------- Link lengths (mm) — names follow the DH table in the lecture slides ----------
D1 = 96.7    # base → shoulder axis (d1)
A2 = 137.0   # shoulder → elbow (a2)
A3 = 98.3    # elbow → wrist (a3)
D5 = 50.0    # wrist → arm tip (d5) — head servo removed, re-measured 2026-11-19 (was 79.7 with the head)

# ---------- SCS15 servo ----------
RAW_MAX = 1023
DEG_RANGE = 250.0          # 0..1023 ticks ≈ 250° (measured by eye ±15° — re-measure in step 6)
TICK_PER_DEG = RAW_MAX / DEG_RANGE

# used: False = joint not connected (J5 head removed 2026-11-19) → locked at q = 0, never sent/read
#       the equations stay 5-joint because J5 rolls about the last link and does not move the tip
# center: raw value when that link points straight up = q 0° (calibrate with calib_app.py)
# dir: +1/−1 so that positive q = lean forward / base turns left (CCW seen from above) / head tilts right
JOINTS = [
    {"name": "J1 Base", "id": 4, "dir": -1, "center": 484, "raw_min": 10, "raw_max": 1012, "used": True},
    {"name": "J2 Shoulder", "id": 9, "dir": -1, "center": 506, "raw_min": 107, "raw_max": 883, "used": True},
    {"name": "J3 Elbow", "id": 7, "dir": +1, "center": 527, "raw_min": 10, "raw_max": 1012, "used": True},
    {"name": "J4 Wrist", "id": 2, "dir": -1, "center": 522, "raw_min": 202, "raw_max": 821, "used": True},
    {"name": "J5 Head", "id": 1, "dir": +1, "center": 511, "raw_min": 20, "raw_max": 1003, "used": False},
]


USED = [j["used"] for j in JOINTS]       # which joints are actually connected


def raw_to_q(joint, raw):
    """q = dir × (raw − center) / tick_per_deg  (degrees)"""
    return joint["dir"] * (raw - joint["center"]) / TICK_PER_DEG


def q_to_raw(joint, q):
    """raw = center + dir × q × tick_per_deg, then clamped to the raw limits"""
    raw = round(joint["center"] + joint["dir"] * q * TICK_PER_DEG)
    return max(joint["raw_min"], min(joint["raw_max"], raw))


# Commandable q range (degrees), derived from the raw limits
Q_MIN = [min(raw_to_q(j, j["raw_min"]), raw_to_q(j, j["raw_max"])) for j in JOINTS]
Q_MAX = [max(raw_to_q(j, j["raw_min"]), raw_to_q(j, j["raw_max"])) for j in JOINTS]

# ---------- Connection ----------
DIRECT_PORT = "COM11"      # Waveshare Bus Servo Adapter (A), jumper on B (USB-SERVO)
SERVO_BAUD = 1_000_000     # SCS15 servo bus (1 Mbps default)
SERVO_TIMEOUT_S = 0.02     # max wait for a servo reply (allows for USB latency)
MIN_MOVE_MS = 300          # minimum move time for "hold where you are" commands
VOLT_MIN = 6.0             # below this servo voltage, no command is sent

# ---------- Smooth pose streaming (motion.py / driver.py) ----------
PERIOD_S = 0.04            # stream a new pose every 40 ms (25 per second)
STREAM_MS = 80             # send the pose 80 ms ahead + tell the servo to take 80 ms (measured: loop jitters 27–52 ms)
JUMP_GUARD_DEG = 20.0      # bug guard: a joint jumping more than this in a command shorter than MIN_MOVE_MS → not sent
V_JOINT = 150.0            # ≈ top speed of the SCS15 carrying the arm — the user asked for no speed limit (2026-11-19)
A_JOINT = 600.0            # acceleration (°/s²) — full speed in 0.25 s, still ramps up/down to avoid jerks
V_CART = 300.0             # tip speed on straight-line moves (mm/s)
JOG_V = 30.0               # joint jog (°/s) — slower than the sliders so it stays controllable
JOG_A = 90.0               # jog acceleration (°/s²) — full speed in ~0.3 s
JOG_V_CART = 40.0          # tip jog (mm/s)
JOG_STOP = 0.5             # releasing a jog button: decelerate at 0.5 × accel (measured: full decel made the arm bounce)
IK_STEP_MM = 1.0           # step size when splitting a straight line into IK points

# ---------- Correction when the arm stops short of the target (driver.py) ----------
CORR_TOL = 0.5             # error at or below this counts as arrived (degrees)
CORR_GAIN = 0.8            # correct by 0.8 × error per round
CORR_ROUNDS = 3            # at most this many rounds
CORR_MAX = 5.0             # total correction at most ±5° per joint
CORR_GIVE_UP = 5.0         # error above this = never got there (blocked / power sag) → do not correct
CORR_WAIT_S = 0.4          # wait for the arm to settle after arriving before measuring the error
