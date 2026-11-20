# Automatic A → B → C … → A loop, like a factory machine (no GUI → testable)
#   the window calls update() every poll with whether the arm is still moving (mover.busy) and the commanded pose (mover.q)
#   update() returns "the next point to command" (or None = wait) — commanding + safety checks happen in the window (_ready)
#
#   start → go to the first point (A) → arrive, wait dwell → next point → … → back at A = 1 cycle
#   the arm stopped before a point (voltage drop / servo silent / STOP) → the loop stops, never resumes by itself
import json
from dataclasses import dataclass, field
from pathlib import Path

ARRIVE_TOL = 0.5        # commanded pose within 0.5° of the point on every joint = arrived (more = stopped on the way)
POINTS_FILE = Path(__file__).resolve().parent.parent / "auto_points.json"   # saved points (not in git)
MAX_POINTS = 8          # most points in a work sequence (A … H)


@dataclass
class AutoCycle:
    points: list                         # [[q1..q4], ...] — points A, B, C … (in running order)
    speed: float = 60.0                  # max joint speed (°/s)
    dwell: float = 0.5                   # wait at each point (s) — e.g. time to pick/place a part
    cycles: int = 0                      # number of cycles (0 = unlimited, until Stop)
    running: bool = False
    phase: str = "idle"                  # idle | move | dwell | done | stopped
    index: int = 0                       # point being approached / waited at
    done: int = 0                        # completed cycles
    last_cycle_s: float | None = None    # duration of the last cycle (s)
    reason: str = ""                     # why it stopped
    _dwell_end: float = 0.0
    _cycle_start: float | None = None
    _first_arrival: bool = field(default=True, repr=False)

    def start(self, now):
        """start the loop → returns the first point to command"""
        if len(self.points) < 2 or any(p is None for p in self.points):
            raise ValueError("save every point first")
        self.running, self.phase, self.index, self.done = True, "move", 0, 0
        self.last_cycle_s, self.reason, self._cycle_start, self._first_arrival = None, "", None, True
        return self.points[0]

    @property
    def approaching(self):
        """first move to the first point (from any pose — the window always uses a joint move, never a line through odd poses)"""
        return self.running and self._first_arrival

    def stop(self, reason="stopped by the user"):
        if self.running:
            self.running, self.phase, self.reason = False, "stopped", reason

    def update(self, now, busy, q_now):
        """call every poll → returns the next point to command or None"""
        if not self.running:
            return None
        if self.phase == "move":
            if busy:
                return None
            goal = self.points[self.index]
            if max(abs(a - b) for a, b in zip(q_now, goal)) > ARRIVE_TOL:
                self.stop("the arm stopped before reaching point " + self.name(self.index))
                return None
            if self.index == 0:                          # at A: one cycle done (except the first arrival, which only sets up)
                if self._first_arrival:
                    self._first_arrival = False
                else:
                    self.done += 1
                    self.last_cycle_s = now - self._cycle_start
                    if self.cycles and self.done >= self.cycles:
                        self.running, self.phase = False, "done"
                        return None
                self._cycle_start = now
            self.phase, self._dwell_end = "dwell", now + self.dwell
            return None
        if self.phase == "dwell" and now >= self._dwell_end:
            self.index = (self.index + 1) % len(self.points)
            self.phase = "move"
            return self.points[self.index]
        return None

    @staticmethod
    def name(i):
        return chr(ord("A") + i)

    def status(self, now):
        """short status text for the window"""
        if self.phase == "move":
            return f"▶ moving to point {self.name(self.index)}"
        if self.phase == "dwell":
            return f"⏸ waiting at point {self.name(self.index)}  {max(0.0, self._dwell_end - now):.1f} s"
        if self.phase == "done":
            return f"✅ {self.done} cycles done"
        if self.phase == "stopped":
            return f"■ stopped — {self.reason}"
        return "ready (not started)"


def load_points(path=POINTS_FILE):
    """points saved last time → [q, q, ...] (no/unreadable file = [None, None] — empty A, B)"""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        pts = data["points"] if "points" in data else [data.get("A"), data.get("B")]   # old format had only A, B
        pts = [[float(v) for v in p] if p else None for p in pts][:MAX_POINTS]
        return pts + [None] * (2 - len(pts))
    except (OSError, ValueError, TypeError, AttributeError, KeyError):
        return [None, None]


def save_points(points, path=POINTS_FILE):
    Path(path).write_text(json.dumps({"points": points}, indent=1), encoding="utf-8")
