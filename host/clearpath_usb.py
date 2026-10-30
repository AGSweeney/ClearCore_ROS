"""Read-only monitor for a ClearPath MC/SD on its own USB port (the MSP port).

Talks the Teknic node protocol that ClearPath-MSP uses, at 9600 baud. Only
set-address and get-parameter packets are sent; nothing is written to the
motor. Close MSP first: the COM port allows one owner.

Framing, parameter numbers, and register layouts come from Teknic's public
sFoundation SDK (pubMnNetDef.h, SerialEx.cpp, and the ClearPath-SC tables in
cpmAPI.cpp and pubCpmRegs.h). They were confirmed on a ClearPath MC/SD by
reading them back over its USB port. Status and alert bit names follow the
ClearPath-SC layout; register sizes match on MC/SD, but treat the names as
provisional.

Usage:
  clearpath_usb.py [--port COM12] info
  clearpath_usb.py [--port COM12] monitor [--seconds 30] [--csv out.csv]
"""

from __future__ import annotations

import argparse
import csv
import sys
import time

import serial
from serial.tools import list_ports

TEKNIC_VID, CLEARPATH_PID = 0x2890, 0x0201
BAUD = 9600
GET_CMD = {0: 0, 1: 2, 2: 11, 3: 13}  # MN_CMD_GET_PARAM0..3 by bank
PKT_CMD, PKT_SET_ADDR = 0, -3

# name: (param number, bytes, signed, divisor, unit), per the public
# sFoundation ClearPath-SC table. Units marked "?" are not yet checked.
PARAMS = {
    "device_id": (0x000, 2, False, 1, ""),
    "fw_version": (0x001, 2, False, 1, ""),
    "hw_version": (0x002, 2, False, 1, ""),
    "serial_number": (0x004, 4, False, 1, ""),
    "alert": (0x009, 12, False, 1, "bits"),
    "status_accum": (0x00D, 6, False, 1, "bits"),     # clears on read
    "status_rt": (0x010, 6, False, 1, "bits"),
    "warning": (0x01A, 12, False, 1, "bits"),         # clears on read
    "torque_cmd": (0x050, 2, True, 32768, "frac?"),
    "torque_meas": (0x051, 2, True, 32768, "frac?"),
    "pos_meas": (0x053, 3, True, 1, "counts"),
    "pos_cmd": (0x054, 3, True, 1, "counts"),
    "pos_error": (0x05A, 4, True, 1, "counts"),
}

STATUS_FIELDS = (
    (0, 1, 'Warning'), (1, 1, 'UserAlert'), (2, 1, 'NotReady'), (3, 1, 'MoveBufAvail'),
    (4, 1, 'Ready'), (5, 1, 'PowerEvent'), (6, 1, 'AlertPresent'), (7, 1, 'Attn0'),
    (8, 1, 'InPosLimit'), (9, 1, 'InNegLimit'), (10, 1, 'MotionBlocked'), (11, 1, 'WasHomed'),
    (12, 1, 'Homing'), (13, 1, 'GoingDisabled'), (14, 1, 'StatusEvent'), (15, 1, 'Enabled'),
    (16, 1, 'MoveCanceled'), (17, 1, 'MoveDone'), (18, 1, 'OutOfRange'), (19, 1, 'BFromEnd'),
    (20, 1, 'AbovePosn'), (21, 1, 'AtTargetVel'), (22, 1, 'InA'), (23, 1, 'InB'),
    (24, 1, 'InvInA'), (25, 1, 'InvInB'), (26, 1, 'AtVelLatched'), (27, 1, 'MoveDoneLatched'),
    (28, 1, 'AFromStart'), (29, 1, 'MoveCmdNeg'), (30, 1, 'Disabled'), (31, 1, 'TimerExpired'),
    (32, 2, 'InMotion'), (34, 1, 'InDisableStop'), (35, 1, 'InCtrlStop'), (36, 1, 'FanOn'),
    (37, 1, 'VectorSearch'), (38, 1, 'MoveCmdComplete'), (39, 1, 'InHardStop'),
    (40, 2, 'ShutdownState'), (42, 1, 'HwFailure'), (43, 1, 'TriggerArmed'),
    (44, 1, 'StepsActive'), (45, 1, 'IndexMtr'), (46, 1, 'SoftwareInputs'),
    (47, 1, 'RefiningVector'),
)
ALERT_FIELDS = (
    (0, 1, 'FwSelfTest'), (1, 1, 'FwNetBufferOverrun'), (2, 1, 'FwError0'), (3, 1, 'FwError1'),
    (4, 1, 'FwStackOverrun'), (5, 1, 'FwWatchdogRestarted'), (6, 1, 'FwInvalidConfiguration'),
    (11, 1, 'HwPowerProblem'), (12, 1, 'HwClockProblem'),
    (13, 1, 'HwEEPROMdead'), (14, 1, 'HwFlashCorrupt'), (15, 1, 'HwFlashChanged'),
    (16, 1, 'HwRAM'), (17, 1, 'HwADC'), (18, 1, 'HwADCsat'),
    (24, 1, 'NetVoltageLow'), (25, 1, 'NetWatchdog'), (28, 1, 'EStopped'),
    (29, 1, 'ConfigOutOfDate'), (30, 1, 'RunTimeErr'),
    (36, 1, 'JrkLimRequestBad'), (37, 1, 'MoveBufUnderrun'), (38, 1, 'JrkLimVelRequestBad'),
    (39, 1, 'MoveSpecAltered'), (40, 1, 'PhaseSensorFailed'), (41, 1, 'LimSwitchActivated'),
    (42, 1, 'SoftLimitExceeded'), (45, 1, 'Mv2SwMaxIndexWarn'), (52, 1, 'ACPowerLost'),
    (53, 1, 'ACPhaseLost'), (55, 1, 'MtrTempLow'), (57, 1, 'MLError'), (59, 1, 'MtrVectorBad'),
    (61, 1, 'MtrEncGlitch'), (62, 1, 'MtrEncOverspeed'), (65, 1, 'MtrPhaseOverload'),
    (67, 1, 'MtrBadSetupParams'), (68, 1, 'HardStopBrokeFree'), (70, 1, 'TrackingShutdown'),
    (71, 1, 'RMSOverload'), (72, 1, 'RMSOverloadShutdown'), (75, 1, 'BusVoltSat'),
    (76, 1, 'TrqSat'), (77, 1, 'NoCommSweepFailed'), (78, 1, 'NoCommSweepReversed'),
    (79, 1, 'NoCommFailed'), (80, 1, 'IndexCountZeroWarn'), (81, 1, 'DriveTempHigh'),
    (82, 1, 'StatorTempHigh'), (83, 1, 'BusOverCurrent'), (84, 1, 'BusOverVoltage'),
    (85, 1, 'BusVoltageLow'), (86, 1, 'BusRMSOverload'), (88, 1, 'MtrEncIndexMissing'),
    (89, 1, 'BusVoltageUnderOperatingV'), (94, 1, 'MtrEncIndexMisplaced'),
    (95, 1, 'StepsDuringPosnRecovery'),
)
BIT_FIELDS = {"alert": ALERT_FIELDS, "warning": ALERT_FIELDS,
              "status_rt": STATUS_FIELDS, "status_accum": STATUS_FIELDS}


def pack7(data: bytes) -> bytes:
    """8-bit payload to 7-bit link characters (SerialEx convert8to7)."""
    out = []
    for i in range(0, len(data), 7):
        chunk = data[i:i + 7]
        s = list(chunk) + [0] * (7 - len(chunk))
        d = [s[0] & 0x7F]
        for k in range(1, 7):
            d.append(((s[k] << k) | (s[k - 1] >> (8 - k))) & 0x7F)
        d.append((s[6] >> 1) & 0x7F)
        out.extend(d[:len(chunk) + 1])
    return bytes(out)


def frame(pkt_type: int, addr: int, payload: bytes = b"") -> bytes:
    body = pack7(payload) if payload else b""
    pkt = bytes([0x80 | ((pkt_type & 7) << 4) | (addr & 0xF), len(body) & 0x1F]) + body
    return pkt + bytes([(-sum(pkt)) & 0x7F])


def groups_to_int(chars: bytes) -> int:
    """7-bit link characters are little-endian 7-bit groups."""
    return sum((c & 0x7F) << (7 * i) for i, c in enumerate(chars))


def find_port() -> str:
    for p in list_ports.comports():
        if p.vid == TEKNIC_VID and p.pid == CLEARPATH_PID:
            return p.device
    raise SystemExit("no ClearPath USB port found (Teknic 2890:0201); is it plugged in?")


class ClearPathUsb:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, BAUD, bytesize=8, parity="N", stopbits=1, timeout=0.02)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.write(frame(PKT_SET_ADDR, 0))
        reply = self._read_packet(1.0)
        if reply is None:
            raise ConnectionError("no reply to set-address; is MSP holding the port?")
        self.node_count = reply[0] & 0xF

    def close(self) -> None:
        self.ser.close()

    def _read_packet(self, timeout: float = 0.3) -> bytes | None:
        deadline = time.time() + timeout
        buf = bytearray()
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] & 0x80:
                buf = bytearray(b)
            elif buf:
                buf += b
            if len(buf) >= 2 and len(buf) >= 3 + (buf[1] & 0x1F):
                pkt = bytes(buf[:3 + (buf[1] & 0x1F)])
                return pkt if sum(pkt) & 0x7F == 0 else None
        return None

    def raw(self, pnum: int) -> bytes:
        """Payload characters (7-bit) of a get-parameter reply."""
        bank, idx = pnum >> 8, pnum & 0xFF
        self.ser.reset_input_buffer()
        self.ser.write(frame(PKT_CMD, 0, bytes([GET_CMD[bank], idx])))
        pkt = self._read_packet()
        if pkt is None:
            raise TimeoutError("no valid reply for parameter 0x%03x" % pnum)
        return pkt[2:-1]

    def read(self, name: str) -> int | float:
        pnum, size, signed, divisor, _unit = PARAMS[name]
        chars = self.raw(pnum)
        bits = min(8 * size, 7 * len(chars))
        value = groups_to_int(chars) & ((1 << bits) - 1)
        if signed and bits and value >> (bits - 1):
            value -= 1 << bits
        return value / divisor if divisor != 1 else value


def bit_names(name: str, value: int) -> list[str]:
    out = []
    for bit, width, field in BIT_FIELDS[name]:
        v = (value >> bit) & ((1 << width) - 1)
        if v:
            out.append(field if width == 1 else "%s=%d" % (field, v))
    return out


def fmt(name: str, value) -> str:
    if name in BIT_FIELDS:
        return "0x%x [%s]" % (value, " ".join(bit_names(name, value)) or "-")
    if isinstance(value, float):
        return "%.4f" % value
    return str(value)


def cmd_info(cp: ClearPathUsb, _args) -> None:
    dev = cp.read("device_id")
    print("nodes on link: %d" % cp.node_count)
    print("device id 0x%04x (type %d%s)" % (dev, dev >> 8, ", ClearPath MC/SD" if dev >> 8 == 11 else ""))
    for name in PARAMS:
        if name == "device_id":
            continue
        print("%-15s %s" % (name, fmt(name, cp.read(name))))


def cmd_monitor(cp: ClearPathUsb, args) -> None:
    names = ("alert", "status_accum", "status_rt", "pos_error", "torque_meas", "pos_meas")
    writer = None
    if args.csv:
        fh = open(args.csv, "w", newline="")
        writer = csv.writer(fh)
        writer.writerow(("t_s",) + names)
    last_bits = {}
    t0 = time.time()
    peak_err = 0
    try:
        while time.time() - t0 < args.seconds:
            row = {n: cp.read(n) for n in names}
            t = time.time() - t0
            peak_err = max(peak_err, abs(row["pos_error"]))
            if writer:
                writer.writerow(["%.3f" % t] + [row[n] for n in names])
            for n in ("alert", "status_accum"):
                # status_accum clears on read, so any set bit is news.
                bits = set(bit_names(n, row[n]))
                new = bits - last_bits.get(n, set()) if n == "alert" else bits
                if new:
                    print("%7.3fs %-12s +%s" % (t, n, " +".join(sorted(new))), flush=True)
                if n == "alert":
                    gone = last_bits.get(n, set()) - bits
                    if gone:
                        print("%7.3fs %-12s -%s" % (t, n, " -".join(sorted(gone))), flush=True)
                last_bits[n] = bits
    finally:
        print("monitor done: %.1f s, peak |pos_error| %d counts" % (time.time() - t0, peak_err))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="COM port (default: auto-detect Teknic 2890:0201)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info").set_defaults(func=cmd_info)
    mon = sub.add_parser("monitor")
    mon.add_argument("--seconds", type=float, default=30.0)
    mon.add_argument("--csv")
    mon.set_defaults(func=cmd_monitor)
    args = ap.parse_args()
    cp = ClearPathUsb(args.port or find_port())
    try:
        args.func(cp, args)
    finally:
        cp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
