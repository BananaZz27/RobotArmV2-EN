# Talks to the arm: DirectLink = PC drives the servos directly via a Waveshare Bus Servo Adapter (A), jumper B (COM11)
#                   SimLink    = simulated arm (nothing connected) with the same methods
# Methods: send_q(q, time_ms) → bool, stop(), torque(on), online(), close() · values: q_actual, volt, message, simulated
import math
import threading
import time

import serial

import config
from scs import ScsBus

ONLINE_TIMEOUT_S = 0.5                                  # no servo answered for longer than this = offline


def jump_too_big(q_from, q_to, time_ms):
    """bug guard: a command shorter than MIN_MOVE_MS where some joint jumps more than JUMP_GUARD_DEG → True (do not send)"""
    if time_ms >= config.MIN_MOVE_MS or q_from is None:
        return False
    return any(abs(b - a) > config.JUMP_GUARD_DEG for a, b in zip(q_from, q_to) if not math.isnan(a))


class SimLink:
    """simulated arm: reaches the commanded angles instantly, no sag, no voltage"""
    simulated = True

    def __init__(self):
        self.q_actual = [0.0] * len(config.JOINTS)
        self.volt = None
        self.message = ""
        self.torque_on = True
        self.servo = {}

    def send_q(self, q, time_ms):
        if not all(math.isfinite(v) for v in q) or jump_too_big(self.q_actual, q, time_ms):
            self.message = "Not sent: invalid angle or a jump that is too fast"
            return False
        self.q_actual = [float(v) for v in q]
        return True

    def stop(self):
        pass

    def torque(self, on):
        self.torque_on = on

    def online(self):
        return True

    def close(self):
        pass


class DirectLink:
    """a thread reads actual angles + voltage every period_s · the bus is locked for every send/receive (thread and GUI share one wire)
    connecting never moves the arm: read the actual position → set it as the goal → enable torque"""
    simulated = False

    def __init__(self, port=config.DIRECT_PORT, serial_obj=None, period_s=0.1):
        if serial_obj is None:
            serial_obj = serial.Serial(port, config.SERVO_BAUD, timeout=0.002)
        self.ser = serial_obj
        self.bus = ScsBus(serial_obj)
        self.lock = threading.Lock()
        self.period_s = period_s
        self.q_actual = None
        self.volt = None
        self.message = ""
        self.last_rx = -math.inf
        self.q_sent = None                        # last pose sent (for the jump guard)
        self.servo = {}                           # id → {"raw", "load" %, "temp" °C} for joints that answered last round
        try:
            with self.lock:
                self.bus.detect_echo()
                self._hold_here()
                for j in config.JOINTS:
                    self.bus.torque(j["id"], True)
        except OSError:
            self.ser.close()
            raise
        self._running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ---------- internals (caller holds the lock) ----------
    def _hold_here(self):
        """every joint: read the actual position and set it as the goal (= hold where it is)"""
        for j in config.JOINTS:
            got = self.bus.read_pos_volt(j["id"])
            if got:
                self.bus.write_pos(j["id"], got[0], config.MIN_MOVE_MS)
        self.q_sent = None

    def _read_all(self):
        """read addr 56..63 of every joint: position(2) speed(2) load(2) voltage(1) temperature(1)"""
        q, volt, any_ok, servo = [], None, False, {}
        with self.lock:
            for j in config.JOINTS:
                d = self.bus.read(j["id"], 56, 8)
                if d is None:
                    q.append(math.nan)
                    continue
                raw, load = (d[0] << 8) | d[1], (d[4] << 8) | d[5]
                # load: bit 10 = direction, low 10 bits = magnitude (0..1000 = 0..100 %)
                servo[j["id"]] = {"raw": raw, "load": (-1 if load & 0x400 else 1) * (load & 0x3FF) / 10,
                                  "temp": d[7]}
                q.append(config.raw_to_q(j, raw))
                volt, any_ok = d[6] / 10, True
        self.servo = servo
        if any_ok:
            self.q_actual, self.volt = q, volt
            self.last_rx = time.monotonic()

    def _loop(self):
        while self._running:
            try:
                self._read_all()
            except OSError:
                self.last_rx = -math.inf
            time.sleep(self.period_s)

    # ---------- public ----------
    def send_q(self, q, time_ms):
        """send a pose to every joint (clamped to limits) — False if not sent (nan / jump too fast / cable unplugged)"""
        if not all(math.isfinite(v) for v in q):
            self.message = "Not sent: an angle is nan/inf"
            return False
        t = max(20, min(10000, int(time_ms)))
        if jump_too_big(self.q_sent or self.q_actual, q, t):
            self.message = f"Not sent: a joint would jump more than {config.JUMP_GUARD_DEG}° in {t} ms"
            return False
        try:
            with self.lock:
                for j, qi in zip(config.JOINTS, q):
                    self.bus.write_pos(j["id"], config.q_to_raw(j, qi), t)
                self.q_sent = list(q)
        except OSError:
            self.last_rx = -math.inf
            return False
        return True

    def stop(self):
        """stop = every joint holds at its actual position now"""
        try:
            with self.lock:
                self._hold_here()
        except OSError:
            self.last_rx = -math.inf

    def torque(self, on):
        """on = hold in place then enable torque · off = every joint goes limp immediately"""
        try:
            with self.lock:
                if on:
                    self._hold_here()
                for j in config.JOINTS:
                    self.bus.torque(j["id"], on)
                self.q_sent = None
        except OSError:
            self.last_rx = -math.inf

    def online(self):
        return time.monotonic() - self.last_rx < ONLINE_TIMEOUT_S

    def close(self):
        self._running = False
        self.thread.join(timeout=1.0)
        try:
            self.ser.close()
        except OSError:
            pass
