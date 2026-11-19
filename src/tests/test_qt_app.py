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
def win():
    w = MainWindow(viewer=FakeViewer())
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
    win.set_target([0, 60, 0, 0, 0])
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
    win.set_target([30, 20, 40, 30, 0])
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
    win.set_target([0, 10, 0, 0, 0])
    win.send()
    assert not win.driver.mover.busy and "Servo voltage low" in win.log_view.toPlainText()


def test_torque_off_blocks_and_target_follows_actual(win):
    win.set_torque(False)
    win.set_target([0, 10, 0, 0, 0])
    win.send()
    assert not win.driver.mover.busy
    win.link.q_actual = [0, 4, 5, 6, 0]
    win.poll()
    assert win.q_target[:4] == [0, 4, 5, 6]


def test_stop(win):
    win.set_target([0, 20, 0, 0, 0])
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


def test_connect_success_target_is_actual(win):
    arm = SimLink()
    arm.simulated, arm.volt, arm.q_actual = False, 7.6, [5.0, 6.0, 7.0, 8.0, 0.0]
    win.connect_fn = lambda: arm
    win.toggle_connect()
    win.driver.close()
    assert win.q_target == [5.0, 6.0, 7.0, 8.0, 0.0] and win.connect_btn.text() == "Disconnect"
    win.poll()
    assert "7.6" in win.conn_label.text()


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
    win.set_target([0, 30, 20, 20, 0])
    win.send()
    _run(win, 2.0)
    win.jog_buttons[("X", +1)].pressed.emit()
    _run(win, 5.0)
    win.poll()
    assert not win.driver.mover.busy and "singularity" in win.log_view.toPlainText()


def test_singularity_label_warns(win):
    win.set_target([0, 0, 0, 0, 0])                 # pointing up = both kinds of singularity
    assert "SINGULARITY" in win.sing_label.text() and "e63946" in win.sing_label.styleSheet()
    win.set_target([10, 30, 50, 20, 0])
    assert "Not near" in win.sing_label.text()


def test_ik_shows_proof(win):
    win.ik_solve(send=False)
    assert "det J" in win.ik_result.toPlainText()


def test_ik_falls_back_to_nearest_phi(win):
    from kinematics import forward_kinematics, inverse_kinematics
    f = forward_kinematics([0, 30, 60, 20, 0])
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
