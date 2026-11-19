# Arm calibration window (via the USB adapter on COM11) — find center / direction / limits joint by joint by hand,
# then save to config.py
#   Calibrate.bat  or  cd src && ../.venv/Scripts/python.exe calib_app.py
# Never writes the servos' permanent memory (EEPROM) — only torque on/off and goal = current position (when locking)
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial

import config
from scs import ScsBus
from servo_calibrate import save_calibration, suggest_limits

DIR_MIN_TICK = 20          # must move at least this much (≈5°) to tell the direction
STEPS = ("Steps (support the arm at all times — a joint with torque released goes limp immediately):\n"
         "1) Release torque → hold the link straight up (base: facing front) → press \"Use this\" = center\n"
         "2) Move it the positive way ≥ 5° (shoulder/elbow/wrist lean forward · base turns left) → press \"Find dir\"\n"
         "3) Press \"Record\" → move to both ends as far as is safe → \"Stop\" (limits shrink inward by 10 ticks)\n"
         "4) Lock torque → Save to config.py → close this window and reopen RobotArm")


class BusReader:
    """a thread reads raw values of every connected joint + issues torque/lock on the same bus (bus lock)"""

    def __init__(self, bus, joints, period_s=0.05):
        self.bus, self.joints, self.period_s = bus, joints, period_s
        self.lock = threading.Lock()
        self.raw = {j["id"]: None for j in joints}
        self.torque = {j["id"]: None for j in joints}
        self.volt = None
        self.error = ""
        self._running = True
        with self.lock:
            bus.detect_echo()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self._running:
            try:
                with self.lock:
                    for j in self.joints:
                        got = self.bus.read_pos_volt(j["id"])
                        self.raw[j["id"]] = got[0] if got else None
                        if got:
                            self.volt = got[1]
                        t = self.bus.read(j["id"], 40, 1)
                        self.torque[j["id"]] = bool(t[0]) if t else None
                self.error = ""
            except OSError as e:
                self.error = f"Cannot read (cable unplugged?): {e}"
            time.sleep(self.period_s)

    def set_torque(self, sid, on):
        try:
            with self.lock:
                if on:                                    # lock: goal = where it is now → no motion
                    got = self.bus.read_pos_volt(sid)
                    if got:
                        self.bus.write_pos(sid, got[0], config.MIN_MOVE_MS)
                self.bus.torque(sid, on)
        except OSError as e:
            self.error = f"Command failed: {e}"

    def close(self):
        self._running = False
        self.thread.join(timeout=1.0)


class CalibApp:
    def __init__(self, root, reader, save=save_calibration):
        self.root, self.reader, self.save = root, reader, save
        self.joints = reader.joints
        self.rec = {}                                    # id → [min, max] while recording limits
        root.title("Arm calibration (USB adapter)")
        frame = ttk.Frame(root, padding=10)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text=STEPS, justify="left").grid(row=0, column=0, columnspan=12, sticky="w", pady=(0, 8))
        heads = ["Joint", "live raw", "torque", "", "", "center", "", "dir", "", "limit min", "max", ""]
        for c, h in enumerate(heads):
            ttk.Label(frame, text=h, font=("", 9, "bold")).grid(row=1, column=c, padx=3)
        self.rows = {}
        for r, j in enumerate(self.joints, start=2):
            sid = j["id"]
            w = {
                "raw": tk.StringVar(value="—"), "torque": tk.StringVar(value="—"),
                "center": tk.StringVar(value=str(j["center"])), "dir": tk.StringVar(value=f"{j['dir']:+d}"),
                "min": tk.StringVar(value=str(j["raw_min"])), "max": tk.StringVar(value=str(j["raw_max"])),
            }
            ttk.Label(frame, text=f"{j['name']} (ID {sid})").grid(row=r, column=0, sticky="w")
            ttk.Label(frame, textvariable=w["raw"], width=6).grid(row=r, column=1)
            w["torque_label"] = tk.Label(frame, textvariable=w["torque"], width=6)
            w["torque_label"].grid(row=r, column=2)
            ttk.Button(frame, text="Release", width=7, command=lambda s=sid: self.on_torque(s, False)).grid(row=r, column=3)
            ttk.Button(frame, text="Lock", width=5, command=lambda s=sid: self.on_torque(s, True)).grid(row=r, column=4)
            ttk.Entry(frame, textvariable=w["center"], width=6).grid(row=r, column=5)
            ttk.Button(frame, text="Use this", command=lambda s=sid: self.on_center(s)).grid(row=r, column=6)
            ttk.Combobox(frame, textvariable=w["dir"], values=["+1", "-1"], width=3, state="readonly").grid(row=r, column=7)
            ttk.Button(frame, text="Find dir", command=lambda s=sid: self.on_dir(s)).grid(row=r, column=8)
            ttk.Entry(frame, textvariable=w["min"], width=6).grid(row=r, column=9)
            ttk.Entry(frame, textvariable=w["max"], width=6).grid(row=r, column=10)
            w["rec_btn"] = ttk.Button(frame, text="Record", width=8, command=lambda s=sid: self.on_record(s))
            w["rec_btn"].grid(row=r, column=11)
            self.rows[sid] = w
        bottom = ttk.Frame(frame)
        bottom.grid(row=len(self.joints) + 2, column=0, columnspan=12, sticky="we", pady=(10, 0))
        self.volt_text = tk.StringVar(value="Volt —")
        self.volt_label = tk.Label(bottom, textvariable=self.volt_text, width=12)
        self.volt_label.pack(side="left")
        ttk.Button(bottom, text="Release all", command=lambda: self.on_all(False)).pack(side="left", padx=4)
        ttk.Button(bottom, text="Lock all", command=lambda: self.on_all(True)).pack(side="left", padx=4)
        ttk.Button(bottom, text="Save to config", command=self.on_save).pack(side="right")
        self.status = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.status, foreground="#185FA5").grid(
            row=len(self.joints) + 3, column=0, columnspan=12, sticky="w", pady=(6, 0))
        self.tick()

    # ---------- buttons ----------
    def on_torque(self, sid, on):
        self.reader.set_torque(sid, on)
        self.status.set(f"ID {sid}: " + ("locked (holding in place)" if on else "torque released — support it"))

    def on_all(self, on):
        for j in self.joints:
            self.reader.set_torque(j["id"], on)
        self.status.set("All joints locked (holding in place)" if on else "All joints released — the arm is limp, support it")

    def on_center(self, sid):
        raw = self.reader.raw[sid]
        if raw is None:
            self.status.set(f"ID {sid} cannot be read")
            return
        self.rows[sid]["center"].set(str(raw))
        self.status.set(f"ID {sid}: center = {raw}")

    def on_dir(self, sid):
        raw = self.reader.raw[sid]
        try:
            center = int(self.rows[sid]["center"].get())
        except ValueError:
            self.status.set("Set the center first")
            return
        if raw is None or abs(raw - center) < DIR_MIN_TICK:
            self.status.set(f"ID {sid}: move more than ~5° the positive way from center, then press")
            return
        d = 1 if raw > center else -1
        self.rows[sid]["dir"].set(f"{d:+d}")
        self.status.set(f"ID {sid}: raw {center} → {raw} ⇒ dir {d:+d}")

    def on_record(self, sid):
        w = self.rows[sid]
        if sid in self.rec:                              # stop recording
            lo, hi = self.rec.pop(sid)
            w["rec_btn"].config(text="Record")
            if lo is None:
                self.status.set(f"ID {sid}: no values recorded")
                return
            lo, hi = suggest_limits(lo, hi)
            w["min"].set(str(lo))
            w["max"].set(str(hi))
            self.status.set(f"ID {sid}: limit {lo}–{hi} (shrunk inward by 10 ticks)")
        else:
            self.rec[sid] = [None, None]
            w["rec_btn"].config(text="Stop")
            self.status.set(f"ID {sid}: recording — move slowly to both ends, then press \"Stop\"")

    def values(self):
        """values to save {id: (dir, center, min, max)} (ValueError if an entry is not a number)"""
        out = {}
        for sid, w in self.rows.items():
            out[sid] = (int(w["dir"].get()), int(w["center"].get()), int(w["min"].get()), int(w["max"].get()))
        return out

    def on_save(self):
        try:
            vals = self.values()
            text = "\n".join(f"ID {s}: dir {d:+d}, center {c}, limit {lo}–{hi}" for s, (d, c, lo, hi) in vals.items())
            if not messagebox.askyesno("Save to config", text + "\n\nWrite to config.py?", parent=self.root):
                return
            self.save(vals)
        except ValueError as e:
            self.status.set(f"Not saved: {e}")
            return
        self.status.set("Saved — close this window and reopen RobotArm")

    # ---------- periodic refresh ----------
    def tick(self):
        for sid, w in self.rows.items():
            raw = self.reader.raw[sid]
            w["raw"].set("—" if raw is None else str(raw))
            t = self.reader.torque[sid]
            w["torque"].set({True: "locked", False: "free", None: "—"}[t])
            w["torque_label"].config(foreground={True: "green", False: "#A32D2D", None: "gray"}[t])
            if sid in self.rec and raw is not None:
                lo, hi = self.rec[sid]
                self.rec[sid] = [raw if lo is None else min(lo, raw), raw if hi is None else max(hi, raw)]
                w["min"].set(str(self.rec[sid][0]))
                w["max"].set(str(self.rec[sid][1]))
        v = self.reader.volt
        self.volt_text.set("Volt —" if v is None else f"Volt {v:.1f} V")
        self.volt_label.config(foreground="green" if v is not None and v >= config.VOLT_MIN else "red")
        if self.reader.error:
            self.status.set(self.reader.error)
        self.root.after(100, self.tick)


def main():
    try:
        ser = serial.Serial(config.DIRECT_PORT, config.SERVO_BAUD, timeout=0.002)
    except serial.SerialException as e:
        messagebox.showerror("Cannot open the port", f"{config.DIRECT_PORT}: {e}\nIs another program using the port?")
        return
    reader = BusReader(ScsBus(ser), [j for j in config.JOINTS if j["used"]])
    root = tk.Tk()
    CalibApp(root, reader)

    def close():
        reader.close()
        ser.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    root.mainloop()


if __name__ == "__main__":
    main()
