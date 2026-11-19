# Arm control window in the style of ws-control (PyQt6 + 3D PyVista) — but driving the servos directly via the adapter
#   (no ESP32), and every move is smooth (Mover/Driver) · RobotArm.bat  or  cd src && ../.venv/Scripts/python.exe qt_app.py
#
#  ┌ [Connect COM11] ● status ──────────────────────── [Torque OFF] [■ STOP (Space)] ┐
#  │ ┌──────── 3D ────────┐ ┌ tabs: Joint | Jog | Inverse Kinematics | Status ┐     │
#  │ └────────────────────┘ └──────────────────────────────────────────────────┘     │
#  │ Forward Kinematics (Target pose): X Y Z φ Yaw + T⁰₅ + singularity check          │
#  │ Log                                                                              │
#  └──────────────────────────────────────────────────────────────────────────────────┘
import math
import sys
import time
from datetime import datetime

import serial
from PyQt6.QtCore import QLocale, Qt, QTimer
from PyQt6.QtGui import QFont, QKeySequence, QShortcut
from PyQt6.QtWidgets import (QApplication, QCheckBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QSlider, QSplitter,
                             QTableWidget, QTableWidgetItem, QTabWidget, QTextEdit, QVBoxLayout, QWidget)

import config
from driver import Driver
from kinematics import check_limits, forward_kinematics, inverse_kinematics
from link import DirectLink, SimLink
from motion import Mover, clamp_q
from singularity import analyze, report
from servo_calibrate import save_standby

USED = [i for i, j in enumerate(config.JOINTS) if j["used"]]
SLIDER_SCALE = 10      # sliders are int → ×10 for 0.1° resolution
MONO = QFont("Consolas")


def _spin(lo, hi, step, suffix, value=0.0, decimals=1):
    s = QDoubleSpinBox(decimals=decimals, minimum=lo, maximum=hi, singleStep=step, suffix=suffix)
    s.setValue(value)
    return s


def _set_quiet(widget, value):
    """set a value without emitting signals (so it does not loop back into commanding the arm)"""
    widget.blockSignals(True)
    widget.setValue(value)
    widget.blockSignals(False)


class MainWindow(QMainWindow):
    def __init__(self, link=None, connect_fn=None, viewer=None, save_standby_fn=save_standby):
        super().__init__()
        self.connect_fn = connect_fn or (lambda: DirectLink(config.DIRECT_PORT))
        self.save_standby = save_standby_fn          # tests pass a fake so config.py is not written
        self.link, self.driver = None, None
        self.q_target = [0.0] * 5
        self.torque_on = True
        self.setWindowTitle("RobotArm 4DOF — Control GUI (USB adapter)")
        self.resize(1400, 900)

        if viewer is None:
            from viewer3d import Viewer3D            # slow import (VTK) — tests pass a fake viewer instead
            viewer = Viewer3D()
        self.viewer = viewer
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_joint_tab(), "Joint")
        self.tabs.addTab(self._build_jog_tab(), "Jog")
        self.tabs.addTab(self._build_ik_tab(), "Inverse Kinematics")
        self.tabs.addTab(self._build_status_tab(), "Status")
        self.log_view = QPlainTextEdit(readOnly=True, maximumBlockCount=500)
        self.log_view.setMaximumHeight(130)
        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(self.viewer)
        split.addWidget(self.tabs)
        split.setSizes([800, 600])

        central = QWidget()
        lay = QVBoxLayout(central)
        lay.addLayout(self._build_toolbar())
        lay.addWidget(split, stretch=1)
        lay.addWidget(self._build_fk_box())
        lay.addWidget(self.log_view)
        self.setCentralWidget(central)
        QShortcut(QKeySequence("Space"), self, activated=self.stop)

        self._use_link(link or SimLink())
        self.set_target(self.q_target)
        self.log("Ready (Simulation mode) — press Connect to drive the arm via COM11, Space = STOP")
        self.timer = QTimer(self, interval=50)
        self.timer.timeout.connect(self.poll)
        self.timer.start()

    # ================= build the window =================
    def _build_toolbar(self):
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connect)
        self.conn_label = QLabel()
        self.torque_btn = QPushButton("Torque OFF")
        self.torque_btn.setToolTip("Turn torque off to move the arm by hand — the GUI pose follows the real arm")
        self.torque_btn.clicked.connect(lambda: self.set_torque(not self.torque_on))
        stop = QPushButton("■ STOP  (Space)")
        stop.setStyleSheet("background:#e63946; color:white; font-weight:bold; padding:6px 18px")
        stop.clicked.connect(self.stop)
        bar = QHBoxLayout()
        for w in (QLabel(f"Port {config.DIRECT_PORT}"), self.connect_btn, self.conn_label):
            bar.addWidget(w)
        bar.addStretch(1)
        standby = QPushButton("⌂ Standby")
        standby.setToolTip(f"Go to the Standby pose slowly ({config.STANDBY_V:.0f} °/s) — set the pose with 📌 in the Joint tab")
        standby.clicked.connect(self.go_standby)
        bar.addWidget(standby)
        bar.addWidget(self.torque_btn)
        bar.addWidget(stop)
        return bar

    def _build_joint_tab(self):
        grid = QGridLayout()
        for col, text in enumerate(("Joint", "", "Target", "Actual")):
            grid.addWidget(QLabel(f"<b>{text}</b>"), 0, col)
        self.rows = {}
        for r, i in enumerate(USED, start=1):
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(config.Q_MIN[i] * SLIDER_SCALE), int(config.Q_MAX[i] * SLIDER_SCALE))
            spin = _spin(config.Q_MIN[i], config.Q_MAX[i], 1.0, " °")
            spin.setKeyboardTracking(False)             # apply on Enter, not on every keystroke
            actual = QLabel("—")
            actual.setMinimumWidth(60)
            actual.setStyleSheet("color:#3a86ff")
            for col, w in enumerate((QLabel(config.JOINTS[i]["name"]), slider, spin, actual)):
                grid.addWidget(w, r, col)
            slider.valueChanged.connect(lambda v, i=i: self.on_joint(i, v / SLIDER_SCALE))
            spin.valueChanged.connect(lambda v, i=i: self.on_joint(i, v))
            self.rows[i] = (slider, spin, actual)
        grid.setColumnStretch(1, 1)

        zero = QPushButton("Zero (straight up)")
        zero.clicked.connect(lambda: (self.set_target([0.0] * 5), self.send()))
        pin = QPushButton("📌 Set this pose as Standby")
        pin.setToolTip("Save the current Target pose to config.py — the next Connect moves the arm here")
        pin.clicked.connect(self.set_standby_here)
        bar = QHBoxLayout()
        bar.addWidget(pin)
        bar.addStretch(1)
        bar.addWidget(zero)
        hint = QLabel("Drag a slider / type a number → the arm follows immediately · orange in 3D = Target, "
                      "translucent blue = actual pose\nJ5 head removed")
        hint.setStyleSheet("color:#666")
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addLayout(grid)
        lay.addLayout(bar)
        lay.addWidget(hint)
        lay.addStretch(1)
        return w

    def _build_jog_tab(self):
        grid = QGridLayout()
        grid.addWidget(QLabel("<b>Joints</b> (+ = lean forward / base turns left)"), 0, 0, 1, 3)
        self.jog_buttons = {}

        def hold(text, row, col, fn):
            b = QPushButton(text)
            b.setAutoRepeat(False)
            b.pressed.connect(lambda: self.jog(fn))          # hold = accelerate
            b.released.connect(self.jog_release)             # release = ease to a stop
            grid.addWidget(b, row, col)
            return b

        for r, i in enumerate(USED, start=1):
            grid.addWidget(QLabel(config.JOINTS[i]["name"]), r, 0)
            for col, d in ((1, -1), (2, +1)):
                self.jog_buttons[(i, d)] = hold("−" if d < 0 else "+", r, col, lambda m, i=i, d=d: m.jog_joint(i, d))
        base = len(USED) + 1
        grid.addWidget(QLabel("<b>Tip</b> (straight line, φ kept)"), base, 0, 1, 3)
        for k, axis in enumerate("XYZ"):
            grid.addWidget(QLabel(axis), base + 1 + k, 0)
            for col, d in ((1, -1), (2, +1)):
                self.jog_buttons[(axis, d)] = hold(f"{axis}{'−' if d < 0 else '+'}", base + 1 + k, col,
                                                   lambda m, k=k, d=d: m.jog_xyz(k, d))
        hint = QLabel("Hold = the arm keeps moving · release = smooth stop\n"
                      "Tip jog stops by itself before a singularity (tip above the base axis / arm fully stretched)\n"
                      "— the |det J| evidence is in the box right of Forward Kinematics · moving away from a singularity is always allowed")
        hint.setStyleSheet("color:#666")
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addLayout(grid)
        lay.addWidget(hint)
        lay.addStretch(1)
        return w

    def jog(self, fn):
        if self._ready(self.driver.mover.q):
            self.driver.do(fn)

    def jog_release(self):
        self.driver.do(lambda m: m.release())

    def _build_ik_tab(self):
        reach = config.D1 + config.A2 + config.A3 + config.D5
        self.ik_x = _spin(-reach, reach, 5, " mm", 110)
        self.ik_y = _spin(-reach, reach, 5, " mm", 10)
        self.ik_z = _spin(-reach / 2, reach, 5, " mm", 350)
        self.ik_phi = _spin(-180, 180, 5, " °", 40)
        form = QFormLayout()
        for name, w in (("X", self.ik_x), ("Y", self.ik_y), ("Z", self.ik_z), ("φ (tip angle)", self.ik_phi)):
            form.addRow(name, w)
        btns = QHBoxLayout()
        for text, fn in (("⟲ Copy current pose", self.ik_copy), ("👁 Preview (no send)", lambda: self.ik_solve(False)),
                         ("▶ Go (send to arm)", lambda: self.ik_solve(True))):
            b = QPushButton(text)
            b.clicked.connect(fn)
            btns.addWidget(b)
        self.ik_result = QTextEdit(readOnly=True)
        self.ik_result.setFont(MONO)
        help_ = QLabel("φ: 0° = tip points straight ahead, −90° = points down, +90° = points up\n"
                       "A 4-joint arm can only control φ (yaw is set by the X, Y position itself)")
        help_.setStyleSheet("color:#666")
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addLayout(form)
        lay.addLayout(btns)
        lay.addWidget(help_)
        lay.addWidget(QLabel("<b>IK result</b>"))
        lay.addWidget(self.ik_result)
        return w

    def _build_status_tab(self):
        cols = ["Joint", "Online", "Raw", "Actual °", "Error °", "Temp °C", "Load %"]
        self.status_table = QTableWidget(len(USED), len(cols))
        self.status_table.setHorizontalHeaderLabels(cols)
        self.status_table.verticalHeader().setVisible(False)
        self.status_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        for r, i in enumerate(USED):
            self.status_table.setItem(r, 0, QTableWidgetItem(config.JOINTS[i]["name"]))
        self.summary = QLabel()
        note = QLabel("Error = Target − actual pose · after arriving the program corrects sag automatically (≤ ±5°)")
        note.setStyleSheet("color:#666")
        self.correct_box = QCheckBox("Correct sag after arriving")
        self.correct_box.setChecked(True)
        self.correct_box.toggled.connect(lambda on: setattr(self.driver, "correct", on))
        box = QGroupBox("Servo")
        bl = QVBoxLayout(box)
        for x in (self.summary, self.status_table, note, self.correct_box):
            bl.addWidget(x)
        w = QWidget()
        QVBoxLayout(w).addWidget(box)
        return w

    def _build_fk_box(self):
        box = QGroupBox("Forward Kinematics (Target pose)")
        grid = QGridLayout()
        self.fk_values = {}
        for k, (name, unit) in enumerate([("X", "mm"), ("Y", "mm"), ("Z", "mm"), ("φ", "°"), ("Yaw", "°")]):
            lbl = QLabel("0.0")
            lbl.setFont(MONO)
            lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            lbl.setMinimumWidth(70)
            grid.addWidget(QLabel(name), k % 3, (k // 3) * 3)
            grid.addWidget(lbl, k % 3, (k // 3) * 3 + 1)
            grid.addWidget(QLabel(unit), k % 3, (k // 3) * 3 + 2)
            self.fk_values[name] = lbl
        self.fk_table = QTableWidget(4, 4)
        self.fk_table.setFont(MONO)
        self.fk_table.horizontalHeader().setVisible(False)
        self.fk_table.verticalHeader().setVisible(False)
        self.fk_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.fk_table.setFixedSize(4 * 72 + 4, 4 * 24 + 4)
        for c in range(4):
            self.fk_table.setColumnWidth(c, 72)
            self.fk_table.setRowHeight(c, 24)
        lay = QHBoxLayout(box)
        lay.addLayout(grid)
        lay.addSpacing(12)
        lay.addWidget(QLabel("T⁰₅ ="))
        lay.addWidget(self.fk_table)
        lay.addSpacing(12)
        self.sing_label = QLabel()                    # singularity check of the Target pose + evidence (singularity.py)
        self.sing_label.setFont(MONO)
        self.sing_label.setToolTip("|det J| = r·a2·a3·|sin θ3| — computed two ways (numerical/formula), they must match\n"
                                   "= 0 when the tip is on the base axis (r = 0) or the arm is fully stretched/folded (θ3 = 0°/180°)")
        lay.addWidget(self.sing_label)
        lay.addStretch(1)
        return box

    # ================= Target pose =================
    def set_target(self, q):
        """set the pose to command → update sliders/spin boxes, FK, 3D view (does not send)"""
        # joints that cannot be read (nan — servo not answering) keep their value: a nan slider value used to crash Connect
        self.q_target = clamp_q([old if v is None or math.isnan(v) else float(v) for old, v in zip(self.q_target, q)])
        for i, (slider, spin, _a) in self.rows.items():
            _set_quiet(slider, int(round(self.q_target[i] * SLIDER_SCALE)))
            _set_quiet(spin, self.q_target[i])
        fk = forward_kinematics(self.q_target)
        yaw = math.degrees(math.atan2(fk.y, fk.x)) if math.hypot(fk.x, fk.y) > 1e-6 else self.q_target[0]
        for name, v in (("X", fk.x), ("Y", fk.y), ("Z", fk.z), ("φ", fk.phi), ("Yaw", yaw)):
            self.fk_values[name].setText(f"{v:8.1f}")
        for r in range(4):
            for c in range(4):
                item = QTableWidgetItem(f"{fk.T[r, c]:.1f}" if c == 3 else f"{fk.T[r, c]:.3f}")
                item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.fk_table.setItem(r, c, item)
        warn = bool(analyze(self.q_target)["warnings"])
        self.sing_label.setText(report(self.q_target))
        self.sing_label.setStyleSheet("color:#e63946; font-weight:bold" if warn else "color:#2a9d8f")
        self.viewer.show_target(self.q_target)

    def on_joint(self, i, deg):
        q = list(self.q_target)
        q[i] = deg
        self.set_target(q)
        if self._ready():                              # sliders drive the arm immediately (no Send needed)
            target = list(self.q_target)
            self.driver.do(lambda m: m.follow(target))

    # ================= command the arm =================
    def _ready(self, q=None):
        """pre-command check — True if it may be sent (otherwise the reason goes to the log)"""
        why = check_limits(q or self.q_target)
        if not why and not self.torque_on:
            why = "Torque is off — press Torque ON first"
        if not why and not self.link.online():
            why = "Arm offline: servos not answering"
        if not why and not self.link.simulated and not (self.link.volt or 0) >= config.VOLT_MIN:
            why = f"Servo voltage low ({self.link.volt} V < {config.VOLT_MIN} V) — not sent"
        if why:
            self.log("❌ " + why)
        return not why

    def send(self):
        if self._ready():
            target = list(self.q_target)
            self.driver.do(lambda m: m.goto_q(target))
            self.log("▶ Send: " + "  ".join(f"{target[i]:.1f}" for i in USED))

    def stop(self):
        self.driver.emergency_stop()
        self.set_target(self.driver.mover.q)
        self.log("■ STOP — the arm holds at its actual position")

    def set_torque(self, on):
        self.driver.emergency_stop()
        self.link.torque(on)
        self.torque_on = on
        self.torque_btn.setText("Torque OFF" if on else "Torque ON")
        if on and self.link.q_actual is not None:
            self.driver.do(lambda m: m.reset(self.link.q_actual))
            self.set_target(self.link.q_actual)
        self.log("Torque ON — the arm holds this pose" if on
                 else "Torque OFF — the arm is limp, support it (the GUI pose follows the real arm)")

    # ================= IK =================
    def ik_copy(self):
        fk = forward_kinematics(self.q_target)
        for w, v in ((self.ik_x, fk.x), (self.ik_y, fk.y), (self.ik_z, fk.z), (self.ik_phi, fk.phi)):
            w.setValue(float(v))

    def ik_solve(self, send):
        x, y, z, phi = self.ik_x.value(), self.ik_y.value(), self.ik_z.value(), self.ik_phi.value()
        self.viewer.show_ik_marker((x, y, z))
        lines, best = [], None
        for elbow_up, name in ((True, "elbow up"), (False, "elbow down")):
            ik = inverse_kinematics(x, y, z, phi, elbow_up, theta1_prev=self.q_target[0])
            if ik.ok:
                lines.append(f"{'★' if best is None else '✓'} {name:<10} [" +
                             "  ".join(f"{ik.q[i]:7.1f}" for i in USED) + "]")
                best = best or ik.q
            else:
                lines.append(f"✗ {name:<10} {ik.reason}")
        head = "✅ Solution found (★ = used)" if best else "❌ Cannot reach this point"
        if best is None:                     # the position may be reachable, just not with this φ → nearest φ that works
            for p in sorted(range(-180, 181), key=lambda p: abs(p - phi)):
                ik = inverse_kinematics(x, y, z, p, True, theta1_prev=self.q_target[0])
                if not ik.ok:
                    ik = inverse_kinematics(x, y, z, p, False, theta1_prev=self.q_target[0])
                if ik.ok:
                    best = ik.q
                    head = f"⚠ φ = {phi:.0f}° is not reachable → using φ = {p}° instead (nearest reachable)"
                    lines.append(f"★ φ {p:<8} [" + "  ".join(f"{ik.q[i]:7.1f}" for i in USED) + "]")
                    self.log(head)
                    break
        proof = "\n\n" + report(best) if best else ""
        self.ik_result.setPlainText(head + "\n\n" + "\n".join(lines) + proof)
        if best:
            if analyze(best)["warnings"]:
                self.log(f"⚠ Point ({x:.0f}, {y:.0f}, {z:.0f}) is near a singularity — see the evidence in the IK tab")
            self.set_target(best)
            if send:
                self.send()

    # ================= connection =================
    def _use_link(self, link):
        if self.driver:
            self.driver.close()
        if self.link:
            self.link.close()
        self.link = link
        q0 = link.q_actual if link.q_actual is not None else [0.0] * 5
        self.driver = Driver(link, Mover(q0))
        self.driver.correct = self.correct_box.isChecked()
        self.driver.start()
        self.torque_on = True
        self.torque_btn.setText("Torque OFF")
        self.connect_btn.setText("Connect" if link.simulated else "Disconnect")
        self._label("● Simulation" if link.simulated else "● Connected", "#888" if link.simulated else "#2a9d8f")

    def toggle_connect(self):
        if not self.link.simulated:
            self._use_link(SimLink())
            self.log("Disconnected — back to Simulation mode (the arm holds its last pose)")
            return
        try:
            link = self.connect_fn()
        except (serial.SerialException, OSError) as e:
            self._label("● Cannot connect", "#e63946", str(e))
            self.log(f"❌ Cannot open {config.DIRECT_PORT} (adapter plugged in? another program using the port?): {e}")
            return
        end = time.monotonic() + 2.0
        while link.q_actual is None and time.monotonic() < end:       # wait for the first reading from the servos
            QApplication.processEvents()
            time.sleep(0.02)
        if link.q_actual is None:
            link.close()
            self._label("● Servos not answering", "#e63946")
            self.log("❌ The port opened but no servo answers — servo power on at 7.4 V? jumper on B?")
            return
        self._use_link(link)
        self.set_target(link.q_actual)
        self.log(f"Connected to {config.DIRECT_PORT} — the arm holds in place, Target = actual pose")
        self.go_standby()                            # every start → go to Standby by itself (slowly, after voltage/servo checks)

    # ================= Standby pose =================
    def go_standby(self):
        """move to the Standby pose at STANDBY_V (slow) — not if voltage is low / a servo is missing / torque is off"""
        q = config.STANDBY_Q
        act = self.link.q_actual
        if act is None or any(math.isnan(act[i]) for i in USED):
            self.log("❌ Not going to Standby: not every servo is answering")
            return False
        if not self._ready(q):
            return False
        self.set_target(q)
        self.driver.do(lambda m: m.goto_q(q, v=config.STANDBY_V))
        self.log(f"⌂ Going to Standby ({config.STANDBY_V:.0f} °/s): "
                 + "  ".join(f"{q[i]:.1f}" for i in USED) + " — press STOP / Space to stop")
        return True

    def set_standby_here(self):
        """save the current Target pose as the Standby pose (writes config.py + takes effect now)"""
        q = list(self.q_target)
        text = "  ".join(f"{config.JOINTS[i]['name']} {q[i]:.1f}°" for i in USED)
        if QMessageBox.question(self, "Set Standby pose", f"{text}\n\nUse this as the Standby pose?") \
                != QMessageBox.StandardButton.Yes:
            return
        try:
            self.save_standby(q)
        except ValueError as e:
            self.log(f"❌ Standby pose not saved: {e}")
            return
        config.STANDBY_Q = [round(v, 1) for v in q]
        self.log("📌 Standby pose saved: " + text)

    def _label(self, text, color, tip=""):
        self.conn_label.setText(text)
        self.conn_label.setToolTip(tip)
        self.conn_label.setStyleSheet(f"color:{color}; font-weight:bold")

    # ================= refresh every 50 ms =================
    def poll(self):
        link, m = self.link, self.driver.mover
        if self.driver.message:
            self.log(self.driver.message)
            self.driver.message = ""
        if m.mode in ("jog_joint", "jog_xyz"):                      # jog: Target/sliders follow the moving pose
            self.set_target(m.q)
        q_act = link.q_actual if not link.simulated else m.q        # simulation: show the moving pose
        if not self.torque_on and link.q_actual is not None:          # torque off: Target follows the real arm
            self.set_target(link.q_actual)
        self.viewer.show_actual(q_act)
        for r, i in enumerate(USED):
            a = None if q_act is None or math.isnan(q_act[i]) else q_act[i]
            self.rows[i][2].setText("—" if a is None else f"{a:6.1f}°")
            s = getattr(link, "servo", {}).get(config.JOINTS[i]["id"])
            cells = ["✅" if (a is not None and (link.simulated or s)) else "❌",
                     str(s["raw"]) if s else "-",
                     "-" if a is None else f"{a:.1f}",
                     "-" if a is None or m.busy else f"{self.q_target[i] - a:+.2f}",
                     str(s["temp"]) if s else "-",
                     f"{s['load']:+.1f}" if s else "-"]
            for c, text in enumerate(cells, start=1):
                self.status_table.setItem(r, c, QTableWidgetItem(text))
        if link.simulated:
            self.summary.setText("Simulation mode (no arm connected)")
        else:
            v = link.volt
            low = v is None or v < config.VOLT_MIN
            self.summary.setText(f"Supply {v if v is None else round(v, 1)} V   •   Torque: "
                                 f"{'ON' if self.torque_on else 'OFF (can be moved by hand)'}   •   "
                                 f"{'offline' if not link.online() else 'moving' if m.busy else 'still'}")
            if not link.online():
                self._label("● Offline (servos not answering)", "#e63946")
            elif low:
                self._label(f"● Connected — servo voltage low {v} V (needs ≥ {config.VOLT_MIN:.0f} V)", "#f4a261")
            else:
                self._label(f"● Connected  {v:.1f} V", "#2a9d8f")

    def log(self, text):
        self.log_view.appendPlainText(f"[{datetime.now():%H:%M:%S}] {text}")

    def closeEvent(self, event):
        self.timer.stop()
        self.driver.close()
        self.link.close()
        self.viewer.close()
        super().closeEvent(event)


def main():
    # On a Windows machine with a Thai region format, Qt shows Thai digits in spin boxes → force Western digits
    QLocale.setDefault(QLocale(QLocale.Language.English, QLocale.Country.UnitedStates))
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
