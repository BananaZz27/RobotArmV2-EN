# Talks to Feetech SCS15 servos on a half-duplex bus directly from the PC (ported from v2/firmware/arm_v2/scs.cpp)
# Used with a Waveshare Bus Servo Adapter (A), jumper on B (USB-SERVO) — design 6.9
#
# Packet the PC sends (instruction):
#   FF FF  ID  LEN  INST  PARAM...  CHK
#   │  │   │   │    │     │         └ checksum = ~(ID + LEN + INST + every PARAM), low 8 bits only
#   │  │   │   │    │     └ data: first byte = address in servo memory, the rest = values
#   │  │   │   │    └ instruction: 01 ping, 02 read, 03 write
#   │  │   │   └ LEN = number of PARAMs + 2
#   │  │   └ servo ID
#   └──┴ two-byte packet header
# Packet the servo replies with (status):  FF FF  ID  LEN  ERR  DATA...  CHK   (LEN = number of DATA + 2)
# 2-byte values (position, time) are sent high byte first (big-endian)
import time

import config

HEADER = 0xFF
INST_PING, INST_READ, INST_WRITE = 0x01, 0x02, 0x03
ADDR_TORQUE = 40        # 1 byte: 1 = torque on
ADDR_GOAL = 42          # 6 bytes: goal position(2) time ms(2) speed(2)
ADDR_PRESENT = 56       # 7 bytes: position(2) speed(2) load(2) voltage(1) — voltage is at addr 62
PROBE_ID = 253          # an ID no servo uses, for detecting echo


def checksum(body):
    """body = ID, LEN, INST/ERR, params → ~sum, 8 bits"""
    return (~sum(body)) & 0xFF


def make_packet(sid, inst, params=b""):
    body = bytes([sid, len(params) + 2, inst]) + bytes(params)   # ID, LEN, INST, PARAM...
    return bytes([HEADER, HEADER]) + body + bytes([checksum(body)])


class ScsBus:
    """send/receive one packet at a time (not thread-safe — the caller must lock)"""

    def __init__(self, ser, timeout_s=config.SERVO_TIMEOUT_S):
        self.ser = ser
        self.timeout_s = timeout_s
        self.echo = False       # some circuits hear their own transmission looped back
        self._buf = bytearray()

    # ---------- byte level ----------
    def _byte(self, deadline):
        """read 1 byte before the deadline (None on timeout)"""
        while not self._buf:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                self._buf += chunk
            elif time.monotonic() > deadline:
                return None
        return self._buf.pop(0)

    def _send(self, sid, inst, params=b""):
        pkt = make_packet(sid, inst, params)
        self.ser.reset_input_buffer()           # drop stale bytes before sending
        self._buf.clear()
        self.ser.write(pkt)
        self.ser.flush()                        # wait until fully sent before listening for the reply
        if self.echo:                           # drop our own echo, one packet's length
            deadline = time.monotonic() + self.timeout_s
            for _ in pkt:
                if self._byte(deadline) is None:
                    break
        return pkt

    def _receive(self, sid, n):
        """find FF FF then check ID / length / checksum — on mismatch discard and keep looking until timeout
        returns n DATA bytes or None (lesson from the old project: not checking the ID gave "ghost IDs")"""
        deadline = time.monotonic() + self.timeout_s
        while True:
            # 1) find the FF FF header
            prev = None
            while True:
                b = self._byte(deadline)
                if b is None:
                    return None
                if prev == HEADER and b == HEADER:
                    break
                prev = b
            # 2) skip any extra FF → the next byte is the ID
            while b == HEADER:
                b = self._byte(deadline)
                if b is None:
                    return None
            rid = b
            # 3) LEN, ERR
            length, err = self._byte(deadline), self._byte(deadline)
            if length is None or err is None or length < 2:
                return None
            # 4) DATA (LEN − 2 bytes)
            data = bytearray()
            for _ in range(length - 2):
                v = self._byte(deadline)
                if v is None:
                    return None
                data.append(v)
            # 5) CHK
            chk = self._byte(deadline)
            if chk is None:
                return None
            # 6) must be the ID we asked, the length we asked, and a valid checksum — otherwise look for the next packet
            if rid == sid and len(data) == n and checksum(bytes([rid, length, err]) + data) == chk:
                return bytes(data)

    def _write(self, sid, addr, data):
        self._send(sid, INST_WRITE, bytes([addr]) + bytes(data))
        return self._receive(sid, 0) is not None

    # ---------- commands ----------
    def detect_echo(self):
        """ping an unused ID; if every byte comes back = the circuit echoes"""
        self.echo = False
        pkt = self._send(PROBE_ID, INST_PING)
        deadline = time.monotonic() + self.timeout_s
        got = 0
        while got < len(pkt) and self._byte(deadline) is not None:
            got += 1
        self.echo = got == len(pkt)
        return self.echo

    def ping(self, sid):
        self._send(sid, INST_PING)
        return self._receive(sid, 0) is not None

    def torque(self, sid, on):
        return self._write(sid, ADDR_TORQUE, [1 if on else 0])

    def write_pos(self, sid, raw, time_ms):
        # position(2) + time(2) + speed(2) = 0 → the servo is time-controlled (same as the firmware)
        return self._write(sid, ADDR_GOAL, [raw >> 8, raw & 0xFF, time_ms >> 8, time_ms & 0xFF, 0, 0])

    def read(self, sid, addr, n):
        """read n bytes of servo memory starting at addr → bytes or None"""
        self._send(sid, INST_READ, bytes([addr, n]))             # PARAM = start address, byte count
        return self._receive(sid, n)

    def read_pos_volt(self, sid):
        """read addr 56..62 → (raw, volts) or None"""
        d = self.read(sid, ADDR_PRESENT, 7)
        if d is None:
            return None
        return (d[0] << 8) | d[1], d[6] * 0.1                     # big-endian position, voltage in 0.1 V units
