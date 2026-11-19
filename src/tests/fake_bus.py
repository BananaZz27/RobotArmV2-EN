# fake serial port that behaves like several SCS15 servos on one bus (tests scs.py / DirectLink without an arm)
# can be made to misbehave: bad checksum, wrong reply ID, no reply, leading garbage, echo, unplugged cable
import threading

import serial


def _chk(body):
    return (~sum(body)) & 0xFF


class FakeBus:
    def __init__(self, positions, volt=7.6, echo=False, follow=True):
        self.positions = dict(positions)      # id → current raw
        self.volt = volt
        self.echo = echo
        self.follow = follow                  # True = a goal is reached instantly
        self.goals = {}                       # id → latest (raw, time_ms)
        self.torque = {}
        self.packets = []                     # packets received (valid)
        self.bad_packets = 0                  # packets with a bad checksum (e.g. interleaved sends)
        self.bad_checksum_ids = set()
        self.wrong_id_map = {}
        self.silent_ids = set()
        self.garbage_prefix = b""
        self.fail = False
        self.timeout = 0
        self._out = bytearray()
        self._lock = threading.Lock()

    # ---------- pyserial-like interface ----------
    @property
    def in_waiting(self):
        self._check()
        return len(self._out)

    def reset_input_buffer(self):
        self._check()
        with self._lock:
            self._out.clear()

    def read(self, n=1):
        self._check()
        with self._lock:
            data = bytes(self._out[:n])
            del self._out[:n]
        return data

    def write(self, data):
        self._check()
        with self._lock:
            if self.echo:
                self._out += data
            self._handle(bytes(data))
        return len(data)

    def flush(self):
        pass

    def close(self):
        pass

    def _check(self):
        if self.fail:
            raise serial.SerialException("cable unplugged (simulated)")

    # ---------- servo ----------
    def memory(self, sid):
        """70-byte servo memory: ID at 5, limits 9/11, position 56, voltage 62, temperature 63"""
        mem = bytearray(70)
        raw = self.positions[sid]
        mem[5] = sid
        mem[9:11] = (20).to_bytes(2, "big")
        mem[11:13] = (1003).to_bytes(2, "big")
        mem[40] = int(self.torque.get(sid, False))
        mem[56:58] = raw.to_bytes(2, "big")
        mem[62] = round(self.volt * 10)
        mem[63] = 31
        return mem

    def _handle(self, pkt):
        if len(pkt) < 6 or pkt[0] != 0xFF or pkt[1] != 0xFF or pkt[3] != len(pkt) - 4 \
                or _chk(pkt[2:-1]) != pkt[-1]:
            self.bad_packets += 1
            return
        sid, inst, params = pkt[2], pkt[4], pkt[5:-1]
        self.packets.append((sid, inst, params))
        if sid not in self.positions or sid in self.silent_ids:
            return
        data = b""
        if inst == 0x02:                                 # READ: slice of the simulated memory
            mem = self.memory(sid)
            data = bytes(mem[params[0]: params[0] + params[1]])
        elif inst == 0x03 and params[0] == 42:          # WRITE goal
            raw, t = (params[1] << 8) | params[2], (params[3] << 8) | params[4]
            self.goals[sid] = (raw, t)
            if self.follow:
                self.positions[sid] = raw
        elif inst == 0x03 and params[0] == 40:          # WRITE torque
            self.torque[sid] = bool(params[1])
        rid = self.wrong_id_map.get(sid, sid)
        body = bytes([rid, len(data) + 2, 0]) + data
        chk = _chk(body) ^ (0x01 if sid in self.bad_checksum_ids else 0)
        self._out += self.garbage_prefix + b"\xff\xff" + body + bytes([chk])
