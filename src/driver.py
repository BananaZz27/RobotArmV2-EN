# Driver: every PERIOD_S, advance the Mover one step, then send the pose STREAM_MS ahead to the arm (steady servo speed)
#   voltage < 6 V / offline / arm refuses the command → stop + hold at the actual angles
#   correction: after a goto finishes and settles → joints with error > 0.5° get +0.8 × error, smoothly, ≤ 3 rounds, total ≤ ±5°
# The GUI issues commands via do(lambda m: m.goto_q(...)) so they never collide with the thread (lock)
import math
import threading
import time

import config


class Driver:
    def __init__(self, link, mover, period=config.PERIOD_S):
        self.link, self.mover, self.period = link, mover, period
        self.correct = True
        self.message = ""
        self.lock = threading.Lock()
        self._corr = None          # {"target", "offset", "rounds", "check_at", "moving"}
        self._thread = None
        self._running = False
        self._last = None          # time of the previous tick — the Mover advances by real time, not by tick count

    # ---------- used by the GUI ----------
    def do(self, fn):
        """command the Mover safely (a new command cancels any pending correction) → returns what fn returns"""
        with self.lock:
            self._corr = None
            result = fn(self.mover)
            if self.mover.message:
                self.message = self.mover.message
            return result

    def emergency_stop(self):
        """stop now: drop the motion → servos hold at their actual position → commanded pose = actual angles"""
        with self.lock:
            self._halt("Stopped")

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    # ---------- internals ----------
    def _loop(self):
        next_t = time.monotonic()
        while self._running:
            self.tick()
            next_t += self.period
            time.sleep(max(0.0, next_t - time.monotonic()))
            if time.monotonic() - next_t > 0.5:        # far behind (machine stalled) → resync, do not race to catch up
                next_t = time.monotonic()

    def _safe(self):
        if not self.link.online():
            return "arm offline (servos not answering)"
        if not self.link.simulated and (self.link.volt is None or self.link.volt < config.VOLT_MIN):
            return f"servo voltage low ({self.link.volt} V < {config.VOLT_MIN} V)"
        return ""

    def _halt(self, why):
        self.mover.stop()
        self._corr = None
        self.link.stop()
        if self.link.q_actual is not None:
            self.mover.reset(self.link.q_actual)
        self.message = why

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        # Windows sleep is coarse (~16 ms) → real ticks run longer than 40 ms; use real time (≤ 0.1 s to avoid jumps after a stall)
        dt = self.period if self._last is None else min(max(now - self._last, 0.0), 0.1)
        self._last = now
        with self.lock:
            if self.mover.busy:
                why = self._safe()
                if why:
                    self._halt("Stopped: " + why)
                    return
                self.mover.step(dt)
                # send "the pose STREAM_MS ahead" + time STREAM_MS → the servo always runs at the set speed
                # (Windows ticks jitter 27–52 ms: sending the current pose with a fixed time made servo speed swing ±30% = jerky)
                q = self.mover.peek(config.STREAM_MS / 1000)
                if not self.link.send_q(q, config.STREAM_MS):
                    self._halt("Stopped: " + (self.link.message or "arm refused the command"))
                    return
                if self.mover.message:
                    self.message = self.mover.message
            self._correction(now)

    def _correction(self, now):
        goal = self.mover.pop_finished()
        if goal is not None:
            if self._corr and self._corr["moving"]:               # a correction round finished → settle, then measure again
                self._corr["moving"], self._corr["check_at"] = False, now + config.CORR_WAIT_S
            elif self.correct and not self.link.simulated:         # the user's move finished → start correcting
                self._corr = {"target": goal, "offset": [0.0] * len(goal), "rounds": 0,
                              "check_at": now + config.CORR_WAIT_S, "moving": False}
            else:
                self.message = "Target reached"
        c = self._corr
        if c is None or c["moving"] or self.mover.busy or now < c["check_at"]:
            return
        actual = self.link.q_actual
        if actual is None or self._safe():
            self._corr, self.message = None, "Not correcting: cannot read angles/voltage"
            return
        err = [0.0 if math.isnan(a) else t - a for t, a in zip(c["target"], actual)]
        worst = max(abs(e) for e in err)
        if worst <= config.CORR_TOL:
            self._corr = None
            self.message = f"Target reached (corrected in {c['rounds']} rounds, max error {worst:.2f}°)"
            return
        if c["rounds"] == 0 and worst > config.CORR_GIVE_UP:
            self._corr = None
            self.message = f"Not correcting: error {worst:.1f}° is abnormally large (blocked / power sag / not enough torque?)"
            return
        if c["rounds"] >= config.CORR_ROUNDS:
            self._corr = None
            self.message = f"Corrected {config.CORR_ROUNDS} rounds, still {worst:.2f}° off"
            return
        c["offset"] = [max(-config.CORR_MAX, min(config.CORR_MAX, o + config.CORR_GAIN * e if abs(e) > config.CORR_TOL else o))
                       for o, e in zip(c["offset"], err)]
        c["rounds"] += 1
        c["moving"] = True
        self.mover.goto_q([t + o for t, o in zip(c["target"], c["offset"])])
        self.message = f"Correcting, round {c['rounds']} (max error {worst:.2f}°)"
