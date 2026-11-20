# ws-control style Qt window tests (offscreen, fake 3D view) — the driver is stepped with tick()
import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import config  # noqa: E402
import pytest  # noqa: E402
import serial  # noqa: E402
from PyQt6.QtWidgets import QApplication, QWidget  # noqa: E402
from link import SimLink  # noqa: E402
from qt_app import MainWindow  # noqa: E402

DT = config.PERIOD_S
APP = QApplication.instance() or QApplication([])


class FakeViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.target = self.actual = self.marker = None

    def show_target(self, q):
        self.target = list(q)

    def show_actual(self, q):
        self.actual = q

    def show_ik_marker(self, xyz):
        self.marker = xyz


@pytest.fixture
def win(tmp_path):
    w = MainWindow(viewer=FakeViewer(), auto_file=tmp_path / "auto_points.json")   # never touches the real point file
    w.timer.stop()
    w.driver.close()                       # the test calls tick() itself
    yield w
    w.close()


def _run(w, seconds):
    t = time.monotonic()
    for _ in range(int(seconds / DT)):
        t += DT
        w.driver.tick(now=t)


def test_starts_in_simulation(win):
    assert win.link.simulated and "Simulation" in win.conn_label.text()


def test_slider_drives_arm(win):
    win.rows[1][0].setValue(300)           # slider ×10 → 30°
    assert win.q_target[1] == 30 and win.viewer.target[1] == 30
    assert win.rows[1][1].value() == 30    # the spin box follows
    _run(win, 0.1)
    assert 0 < win.link.q_actual[1] < 30   # ramps up, no jump
    _run(win, 1.0)
    assert win.link.q_actual[1] == pytest.approx(30)


def test_send_moves_smoothly(win):
    win.set_target([0, 60, 0, 0])
    win.send()
    _run(win, 0.2)
    assert 0 < win.link.q_actual[1] < 60
    _run(win, 2.0)
    assert win.link.q_actual[1] == pytest.approx(60)


def test_spin_drives_arm(win):
    win.rows[2][1].setValue(6)
    _run(win, 1.0)
    assert win.link.q_actual[2] == pytest.approx(6)


def test_fk_display(win):
    win.set_target([30, 20, 40, 30])
    assert win.fk_values["X"].text().strip() == "157.6" and win.fk_values["Z"].text().strip() == "274.6"


def test_ik_preview_and_go(win):
    win.ik_solve(send=False)
    assert "★" in win.ik_result.toPlainText() and win.viewer.marker == (110, 10, 350)
    _run(win, 0.5)
    assert win.link.q_actual[2] == 0
    win.ik_solve(send=True)
    _run(win, 6.0)
    assert win.link.q_actual[:4] == pytest.approx(win.q_target[:4], abs=1e-6)


def test_ik_unreachable(win):
    win.ik_x.setValue(400)
    win.ik_z.setValue(0)
    win.ik_solve(send=True)
    assert "Cannot reach this point" in win.ik_result.toPlainText() and not win.driver.mover.busy


def test_low_volt_blocks(win):
    arm = SimLink()
    arm.simulated, arm.volt = False, 5.0
    win._use_link(arm)
    win.driver.close()
    win.set_target([0, 10, 0, 0])
    win.send()
    assert not win.driver.mover.busy and "Servo voltage low" in win.log_view.toPlainText()


def test_torque_off_blocks_and_target_follows_actual(win):
    win.set_torque(False)
    win.set_target([0, 10, 0, 0])
    win.send()
    assert not win.driver.mover.busy
    win.link.q_actual = [0, 4, 5, 6]
    win.poll()
    assert win.q_target[:4] == [0, 4, 5, 6]


def test_stop(win):
    win.set_target([0, 20, 0, 0])
    win.send()
    _run(win, 0.5)
    win.stop()
    assert not win.driver.mover.busy and win.q_target[1] == pytest.approx(win.driver.mover.q[1])


def test_connect_failure(win):
    def boom():
        raise serial.SerialException("busy")
    win.connect_fn = boom
    win.toggle_connect()
    assert win.link.simulated and "Cannot connect" in win.conn_label.text()


def _fake_arm(q=(5.0, 6.0, 7.0, 8.0), volt=7.6):
    arm = SimLink()
    arm.simulated, arm.volt, arm.q_actual = False, volt, list(q)
    return arm


def test_connect_goes_to_standby_slowly(win):
    arm = _fake_arm()
    win.connect_fn = lambda: arm
    win.toggle_connect()
    win.driver.close()
    assert win.connect_btn.text() == "Disconnect" and "Standby" in win.log_view.toPlainText()
    assert win.q_target == pytest.approx(config.STANDBY_Q) and win.driver.mover.busy
    prev, t = list(arm.q_actual), time.monotonic()
    for _ in range(int(12 / DT)):                      # farthest ~83° at 20 °/s → ~7.8 s
        t += DT
        win.driver.tick(now=t)
        assert max(abs(a - b) for a, b in zip(arm.q_actual, prev)) <= config.STANDBY_V * 0.12 + 1e-6   # slow (≤ 20 °/s, 80 ms look-ahead)
        prev = list(arm.q_actual)
    assert arm.q_actual[:4] == pytest.approx(config.STANDBY_Q[:4], abs=1e-6)
    win.poll()
    assert "7.6" in win.conn_label.text()


def test_connect_low_volt_does_not_move(win):
    arm = _fake_arm(volt=5.2)
    win.connect_fn = lambda: arm
    win.toggle_connect()
    win.driver.close()
    assert not win.driver.mover.busy and "Servo voltage low" in win.log_view.toPlainText()


def test_connect_missing_servo_does_not_move(win):
    arm = _fake_arm(q=(5.0, float("nan"), 7.0, 8.0))
    win.connect_fn = lambda: arm
    win.toggle_connect()
    win.driver.close()
    assert not win.driver.mover.busy and "not every servo" in win.log_view.toPlainText()


def test_standby_button_in_sim(win):
    assert win.go_standby()
    _run(win, 10.0)
    assert win.link.q_actual[:4] == pytest.approx(config.STANDBY_Q[:4], abs=1e-6)


def test_set_standby_here(win, monkeypatch):
    from PyQt6.QtWidgets import QMessageBox
    saved, old = [], list(config.STANDBY_Q)
    win.save_standby = saved.append
    monkeypatch.setattr(QMessageBox, "question", lambda *a, **k: QMessageBox.StandardButton.Yes)
    try:
        win.set_target([10, 20, 30, 40])
        win.set_standby_here()
        assert saved == [[10.0, 20.0, 30.0, 40.0]] and config.STANDBY_Q == [10.0, 20.0, 30.0, 40.0]
    finally:
        config.STANDBY_Q = old


def test_no_speed_cap_default_fast(win):
    assert win.driver.mover.v >= 100


def test_jog_joint_hold_and_release(win):
    win.jog_buttons[(1, +1)].pressed.emit()
    _run(win, 0.5)
    win.poll()
    assert win.driver.mover.q[1] > 5 and win.q_target[1] == pytest.approx(win.driver.mover.q[1])
    win.jog_buttons[(1, +1)].released.emit()
    _run(win, 1.0)                         # easing at 0.5·a takes ~0.7 s
    assert not win.driver.mover.busy


def test_jog_xyz_stops_at_singularity_and_logs(win):
    win.set_target([0, 30, 20, 20])
    win.send()
    _run(win, 2.0)
    win.jog_buttons[("X", +1)].pressed.emit()
    _run(win, 5.0)
    win.poll()
    assert not win.driver.mover.busy and "singularity" in win.log_view.toPlainText()


def test_singularity_label_warns(win):
    win.set_target([0, 0, 0, 0])                 # pointing up = both kinds of singularity
    assert "SINGULARITY" in win.sing_label.text() and "e63946" in win.sing_label.styleSheet()
    win.set_target([10, 30, 50, 20])
    assert "Not near" in win.sing_label.text()


def test_ik_shows_proof(win):
    win.ik_solve(send=False)
    assert "det J" in win.ik_result.toPlainText()


def test_ik_falls_back_to_nearest_phi(win):
    from kinematics import forward_kinematics, inverse_kinematics
    f = forward_kinematics([0, 30, 60, 20])
    assert not inverse_kinematics(f.x, f.y, f.z, -90).ok          # φ −90 cannot reach this point
    win.ik_x.setValue(f.x)
    win.ik_y.setValue(f.y)
    win.ik_z.setValue(f.z)
    win.ik_phi.setValue(-90)
    win.ik_solve(send=True)
    _run(win, 3.0)
    tip = forward_kinematics(win.link.q_actual)
    assert "using φ" in win.ik_result.toPlainText()
    assert abs(tip.x - f.x) < 0.5 and abs(tip.z - f.z) < 0.5


def test_startup_connects_and_goes_standby(win):
    arm = _fake_arm()
    win.connect_fn = lambda: arm
    win.startup()
    win.driver.close()
    assert not win.link.simulated and win.driver.mover.busy
    assert win.q_target == pytest.approx(config.STANDBY_Q)


def test_startup_without_arm_shows_sim_at_standby(win):
    def boom():
        raise serial.SerialException("no adapter")
    win.connect_fn = boom
    win.startup()
    _run(win, 10.0)
    assert win.link.simulated and win.link.q_actual[:4] == pytest.approx(config.STANDBY_Q[:4], abs=1e-6)
    assert "simulated arm" in win.log_view.toPlainText()


def test_ik_unreachable_shows_reason_and_sketch(win):
    win.ik_x.setValue(400)
    win.ik_y.setValue(0)
    win.ik_z.setValue(100)
    win.ik_phi.setValue(0)
    win.ik_solve(send=True)
    text = win.ik_result.toPlainText()
    assert "Cannot reach this point" in text and "Too far" in text and "Try" in text
    assert win.ik_sketch.d is not None and win.ik_sketch.d.q is None
    assert not win.driver.mover.busy
    win.ik_sketch.grab()                              # draws without error


# ---------- Auto tab: A ⇄ B loop ----------
def _run_auto(w, seconds):
    """run the driver + the Auto loop together on simulated time"""
    t = time.monotonic()
    for _ in range(int(seconds / DT)):
        t += DT
        w.driver.tick(now=t)
        w._auto_tick(t)


def _teach(w, qa, qb):
    w.set_target(qa)
    w.auto_record(0)
    w.set_target(qb)
    w.auto_record(1)


def test_auto_loop_runs_cycles_and_stops_at_a(win):
    qa, qb = [0.0, 20.0, 60.0, 10.0], [30.0, 0.0, 80.0, -10.0]
    _teach(win, qa, qb)
    win.auto_speed.setValue(150)
    win.auto_dwell.setValue(0.1)
    win.auto_cycles.setValue(2)
    win.auto_start()
    assert win.auto.running
    _run_auto(win, 8)
    assert win.auto.phase == "done" and win.auto.done == 2
    assert win.link.q_actual == pytest.approx(qa, abs=0.1)        # ends at A
    assert win.auto.last_cycle_s > 0 and "finished 2 cycles" in win.log_view.toPlainText()
    assert win.auto_start_btn.isEnabled()


def test_auto_points_saved_and_loaded(win, tmp_path):
    _teach(win, [0.0, 20.0, 60.0, 10.0], [30.0, 0.0, 80.0, -10.0])
    w2 = MainWindow(viewer=FakeViewer(), auto_file=win.auto_file)
    try:
        assert w2.auto.points == win.auto.points and "q" in w2.auto_labels[1].text()
    finally:
        w2.timer.stop()
        w2.close()


def test_auto_needs_points(win):
    win.auto_start()
    assert not win.auto.running and "Save points A and B" in win.log_view.toPlainText()


@pytest.mark.parametrize("action", ["stop", "jog", "torque"])
def test_auto_stops_when_user_takes_over(win, action):
    _teach(win, [0.0, 20.0, 60.0, 10.0], [30.0, 0.0, 80.0, -10.0])
    win.auto_start()
    _run_auto(win, 0.3)
    {"stop": win.stop, "jog": lambda: win.jog(lambda m: m.jog_joint(0, 1)),
     "torque": lambda: win.set_torque(False)}[action]()
    assert not win.auto.running and win.auto.phase == "stopped"
    _run_auto(win, 3)
    assert not win.auto.running                                     # does not restart by itself


def test_auto_stops_when_arm_goes_offline(win):
    _teach(win, [0.0, 20.0, 60.0, 10.0], [30.0, 0.0, 80.0, -10.0])
    win.auto_start()
    _run_auto(win, 0.3)
    win.link.online = lambda: False                                # servos go silent mid-way → the driver halts the arm
    _run_auto(win, 1)
    assert win.auto.phase == "stopped" and "before reaching point" in win.auto.reason


def test_auto_finish_cycle_button(win):
    _teach(win, [0.0, 20.0, 60.0, 10.0], [30.0, 0.0, 80.0, -10.0])
    win.auto_dwell.setValue(0)
    win.auto_start()
    _run_auto(win, 0.2)
    win.auto_finish_cycle()                                        # unlimited → finish this cycle, then stop
    _run_auto(win, 8)
    assert win.auto.phase == "done" and win.auto.done == 1
