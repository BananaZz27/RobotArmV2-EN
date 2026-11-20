# Smooth pose streamer (design: docs/design.md) — computes the next pose every dt seconds, never touches hardware (testable)
#   goto_q   go to target angles: every joint shares one min-jerk profile → slow start, ease in, all arrive together
#   goto_xyz the tip travels a straight line to a point (IK every ~1 mm)
#   jog_*    hold = accelerate up to the set speed · release() = ease out to a stop
#   follow   chase a target that can change at any time (slider) with limited speed/acceleration
import copy
import math

import numpy as np

import config
from kinematics import forward_kinematics, inverse_kinematics
from singularity import getting_worse

N = len(config.JOINTS)


def min_jerk(tau):
    """s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ : 0→1, zero velocity and acceleration at both ends · peak velocity = 1.875 at τ = 0.5"""
    tau = min(max(tau, 0.0), 1.0)
    return tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)


def clamp_q(q):
    """clamp every joint to its limit"""
    return [min(max(v, config.Q_MIN[i]), config.Q_MAX[i]) for i, v in enumerate(q)]


class Mover:
    def __init__(self, q0, v=config.V_JOINT, a=config.A_JOINT, v_cart=config.V_CART):
        self.q = clamp_q([float(x) for x in q0])     # pose being commanded (setpoint)
        self.v, self.a, self.v_cart = v, a, v_cart
        self.vel = [0.0] * N                         # current joint velocity (jog / follow)
        self.mode = None                             # None | "path" | "jog_joint" | "jog_xyz" | "follow"
        self.message = ""
        self._finished = None

    @property
    def busy(self):
        return self.mode is not None

    # ---------- commands ----------
    def goto_q(self, q_goal, v=None):
        """go to target angles in joint space (returns "" = accepted) · v = max speed for this move only (e.g. slow Standby)"""
        goal = clamp_q(q_goal)
        start = list(self.q)
        dist = max(abs(g - s) for g, s in zip(goal, start))
        self._set_path([start, goal], 1.875 * dist / (v or self.v), goal)
        return ""

    def goto_xyz(self, x, y, z, phi):
        """tip travels a straight line to (x, y, z) mm, φ degrees · if the end point has no IK = no move, returns the reason"""
        elbow_up = self.q[2] >= 0
        end = inverse_kinematics(x, y, z, phi, elbow_up, theta1_prev=self.q[0])
        if not end.ok:
            self.message = end.reason
            return end.reason
        f0 = forward_kinematics(self.q)
        p0, p1 = np.array([f0.x, f0.y, f0.z]), np.array([x, y, z])
        length = float(np.linalg.norm(p1 - p0))
        n = max(1, math.ceil(length / config.IK_STEP_MM))
        points, q_prev = [list(self.q)], self.q
        for k in range(1, n + 1):                    # IK point by point along the line (φ also interpolated linearly)
            s = k / n
            p = p0 + s * (p1 - p0)
            ik = inverse_kinematics(*p, f0.phi + s * (phi - f0.phi), elbow_up, theta1_prev=q_prev[0])
            if not ik.ok:                            # a point on the way fails → go in joint space instead
                self.goto_q(end.q)
                self.message = "Straight line passes an unreachable point → moving joint-by-joint instead"
                return ""
            points.append(clamp_q(ik.q))
            q_prev = ik.q
        # timing: tip ≤ v_cart and every joint ≤ v (peak speed of min-jerk = 1.875 × distance / T)
        joint_rate = max(max(abs(b - a) for a, b in zip(p, q)) for p, q in zip(points, points[1:])) * n
        t_cart, t_joint = length / self.v_cart, joint_rate / self.v
        if t_joint > 3 * t_cart + 1.0:               # line passes near full stretch, joints must spin very fast → too slow to use
            self.goto_q(end.q)
            self.message = "Near full stretch → moving joint-by-joint instead of a straight line"
            return ""
        self._set_path(points, 1.875 * max(t_cart, t_joint), points[-1])
        return ""

    def jog_joint(self, i, direction):
        self.mode, self._jog = "jog_joint", (i, direction)
        self.message = ""

    def jog_xyz(self, axis, direction):
        """axis 0/1/2 = X/Y/Z · φ starts at the pose when the button was pressed"""
        f = forward_kinematics(self.q)
        self.mode, self._jog = "jog_xyz", (axis, direction)
        self._tip, self._phi, self._vc = np.array([f.x, f.y, f.z]), f.phi, 0.0
        self.vel = [0.0] * N
        self.message = ""

    def release(self):
        """jog button released → ease out to a stop"""
        if self.mode in ("jog_joint", "jog_xyz"):
            self._jog = (self._jog[0], 0)

    def follow(self, q_goal):
        if self.mode != "follow":
            self.mode = "follow"
            self.message = ""
        self._goal = clamp_q(q_goal)

    def stop(self):
        self.mode, self.vel = None, [0.0] * N

    def reset(self, q):
        """set the commanded pose = actual angles (joints that could not be read keep their value)"""
        self.q = clamp_q([old if q_i is None or math.isnan(q_i) else q_i for old, q_i in zip(self.q, q)])
        self.stop()

    def pop_finished(self):
        """target of the move that just finished (once) — used by the driver for correction"""
        done, self._finished = self._finished, None
        return done

    # ---------- one step ----------
    def peek(self, dt):
        """pose dt seconds ahead without changing the real state (shallow copy + the lists step mutates)"""
        ghost = copy.copy(self)
        ghost.q, ghost.vel = list(self.q), list(self.vel)
        return ghost.step(dt)

    def step(self, dt):
        if dt <= 0:                                  # no time passed → no move (avoids division by zero)
            return list(self.q)
        if self.mode == "path":
            self._step_path(dt)
        elif self.mode == "jog_joint":
            self._step_jog_joint(dt)
        elif self.mode == "jog_xyz":
            self._step_jog_xyz(dt)
        elif self.mode == "follow":
            self._step_follow(dt)
        return list(self.q)

    def _set_path(self, points, T, goal):
        self.mode, self.vel, self.message = "path", [0.0] * N, ""
        self._points, self._T, self._t, self._goal = points, max(T, 1e-6), 0.0, goal

    def _step_path(self, dt):
        self._t += dt
        s = min_jerk(self._t / self._T) * (len(self._points) - 1)
        k = min(int(s), len(self._points) - 2)
        a, b = self._points[k], self._points[k + 1]
        self.q = clamp_q([qa + (s - k) * (qb - qa) for qa, qb in zip(a, b)])
        if self._t >= self._T:
            self.q = list(self._goal)
            self._finished = list(self._goal)
            self.mode = None

    def _ramp(self, vel, v_target, dt, a=None):
        """move the velocity toward the target by at most a·dt per step"""
        dv = (a or self.a) * dt
        return vel + max(-dv, min(dv, v_target - vel))

    def _jog_speed(self):
        """jog is slower than the sliders: full 150 °/s + 0.25 s ramp made the arm lunge/jerk on press"""
        return min(self.v, config.JOG_V), min(self.a, config.JOG_A), min(self.v_cart, config.JOG_V_CART)

    def _step_jog_joint(self, dt):
        i, d = self._jog
        v, a, _ = self._jog_speed()
        lo, hi = config.Q_MIN[i], config.Q_MAX[i]
        room = (hi - self.q[i]) if self.vel[i] >= 0 else (self.q[i] - lo)
        # braking ceiling v ≤ √(a·distance) = brake with half of a — with full a (√(2a·distance)) the ramp
        # cannot keep up and the joint hits the limit at ~14 °/s
        brake = math.sqrt(a * max(room, 0.0))
        v_target = d * min(v, brake)
        self.vel[i] = self._ramp(self.vel[i], v_target, dt, a if d else a * config.JOG_STOP)
        self.q[i] = min(max(self.q[i] + self.vel[i] * dt, lo), hi)
        if self.q[i] in (lo, hi):
            self.vel[i] = 0.0
            if d:
                self.message = f"{config.JOINTS[i]['name']} reached its limit"
        if d == 0 and self.vel[i] == 0.0:
            self.mode = None
            self._finished = list(self.q)                      # let the driver correct sag at the stop pose

    def _step_jog_xyz(self, dt):
        axis, d = self._jog
        v, a, v_cart = self._jog_speed()
        dv = a / v * v_cart * dt * (1 if d else config.JOG_STOP)   # released: ease out slower than ramp-up, avoids bounce
        self._vc += max(-dv, min(dv, d * v_cart - self._vc))
        if d == 0 and abs(self._vc) < 1e-9:
            self._vc, self.mode = 0.0, None
            self._finished = list(self.q)                      # let the driver correct sag at the stop pose
            return
        step = self._vc * dt
        for _ in range(12):                                    # a joint would exceed jog v → shrink the step and retry
            tip = self._tip.copy()
            tip[axis] += step
            ik, phi = self._ik_keep_position(tip)
            if not ik.ok:
                # edge of reachable space = arm fully stretched (θ3 = 0) = singularity — at speed one step can skip the warning zone
                edge = "Out of reach" in ik.reason
                self.message = ("Stopped: near singularity — arm fully stretched (edge of reachable space)" if edge
                                else "Stopped: " + ik.reason)
                self.stop()
                return
            q = clamp_q(ik.q)
            ratio = max(abs(b - a) for a, b in zip(self.q, q)) / (v * dt)
            if ratio <= 1.0:
                break
            step /= max(ratio, 2.0)                            # near singularity joint motion ∝ √distance → shrink hard
        else:                                                  # still too fast after 12 shrinks → cannot continue this way
            self.message = "Stopped: near singularity — arm fully stretched, cannot continue this way (try another direction)"
            self.stop()
            return
        self._vc = step / dt
        worse = getting_worse(self.q, q)                       # moving toward a singularity (moving away is allowed)
        if worse:
            self.message = "Stopped: near singularity — " + worse
            self.stop()
            return
        if q is not self.q:
            self._tip, self.q, self._phi = tip, q, phi

    def _ik_keep_position(self, tip):
        """IK at tip: try the current φ first, otherwise the nearest φ within ±30° (position matters more than angle —
        a fully stretched arm must be able to go Z− by changing φ) · pick the elbow solution closest to the current pose"""
        first = None
        for dphi in [0] + [s * k for k in range(1, 31) for s in (1, -1)]:
            phi = self._phi + dphi
            best = None
            for up in (True, False):
                ik = inverse_kinematics(*tip, phi, up, theta1_prev=self.q[0])
                first = first or ik
                if ik.ok:
                    gap = max(abs(a - b) for a, b in zip(ik.q, self.q))
                    if best is None or gap < best[0]:
                        best = (gap, ik)
            if best:
                return best[1], phi
        return first, self._phi

    def _step_follow(self, dt):
        done = True
        for i in range(N):
            e = self._goal[i] - self.q[i]
            v_target = math.copysign(min(self.v, math.sqrt(2 * self.a * abs(e))), e)
            self.vel[i] = self._ramp(self.vel[i], v_target, dt)
            step = self.vel[i] * dt
            if abs(step) >= abs(e) and (e == 0 or step * e >= 0):   # this step reaches/passes the target → stop on it
                self.q[i], self.vel[i] = self._goal[i], 0.0
            else:
                self.q[i] += step
                done = False
        self.q = clamp_q(self.q)
        if done:
            self.mode = None
