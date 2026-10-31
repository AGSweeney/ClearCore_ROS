# MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
"""Monitor a ClearPath MC/SD on its own USB port (the MSP port).

Talks the Teknic node protocol that ClearPath-MSP uses. Close MSP first: the
COM port allows one owner.

info and monitor send only set-address and get-parameter packets. scope
also raises the link rate, changes the working (volatile) setting of monitor
channel 1, and starts the motor's sample stream. On exit it stops the
stream, restores channel 1's original setting, and drops the link back to
9600 baud. It never sets the non-volatile save flag, so the motor's stored
monitor setting is not touched.

Framing, parameter numbers, and register layouts come from Teknic's public
sFoundation SDK (pubMnNetDef.h, SerialEx.cpp, and the ClearPath-SC tables in
cpmAPI.cpp and pubCpmRegs.h). They were confirmed on a ClearPath MC/SD by
reading them back over its USB port. Status and alert bit names follow the
ClearPath-SC layout; register sizes match on MC/SD, but treat the names as
provisional. Parameters marked "capture" were identified from USB captures of
ClearPath-MSP; the evidence is in docs/CLEARPATH_USB.md.

Usage:
  clearpath_usb.py [--port COM12] info
  clearpath_usb.py [--port COM12] monitor [--seconds 30] [--csv out.csv]
  clearpath_usb.py [--port COM12] scope [--var position_error] [--seconds 10] --csv out.csv
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
PKT_CMD, PKT_RESP, PKT_EXT_LOW, PKT_SET_ADDR, PKT_EXT_HIGH = 0, 1, 3, -3, -1

# Link rate change (public SDK infcChangeLinkSpeed): extended-high packet
# [MN_CTL_EXT_BAUD_RATE, rate / MN_NET_RATE_DIVIDER]. Do not use a serial break
# to reset the rate: on our MC/SD a break latches the EStopped alert.
EXT_BAUD_RATE, RATE_DIVIDER, STREAM_BAUD = 9, 9600, 230400
NODE_STOP_CMD, STOP_TYPE_CLR_ESTOP = 4, 0x18    # public SDK netNodeStop

# Monitor/scope (docs/CLEARPATH_USB.md "Scope stream"). Commands are the SDK's
# MN_CMD_GET_SET_MON0/1; the payload is the SDK's iscMonNodeState.
MON_CMD = {1: 0x16, 2: 0x1E}
SCOPE_CMD, KEEPALIVE_CMD = 0x19, 0x05
MON_SAVE_NV = 0x4000                  # never set by this tool
MON_SCALE = 1 << 16                   # ISC_MON_SCALE
# name: (monTestPoints id, gain ClearPath-MSP writes for its default scale)
MON_VARS = {
    "position_error": (27, 0x00800000),
    "velocity_cmd": (3, 0x00206C3C),
    "velocity_meas": (2, 0x00206C3C),
    "torque_meas_peak": (35, 0x00033F36),
    "torque_error_peak": (34, 0x00040F04),
    "torque_cmd": (8, 0x00008000),
}

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
    "velocity_a": (0x055, 4, True, 16555.0, "RPM"),    # capture; measured/commanded not yet told apart
    "velocity_b": (0x056, 4, True, 16555.0, "RPM"),    # capture
    "tracking_limit": (0x110, 4, False, 1, "counts"),  # capture
    "input_resolution": (0x140, 4, False, 1, "steps/rev"),  # capture
    "drive_temp": (0x123, 2, True, 1, "degC"),          # capture
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


def is_stream(pkt: bytes) -> bool:
    """Node-originated extended-low packet: a scope sample."""
    return (pkt[0] >> 4) & 7 == PKT_EXT_LOW and pkt[1] & 0x40 != 0


class ClearPathUsb:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, BAUD, bytesize=8, parity="N", stopbits=1, timeout=0.02)
        self._rx = bytearray()
        self._pkts = []          # (time, packet) of command replies
        self.stream = []         # (time, packet) of scope samples
        self.bad = 0
        time.sleep(0.2)
        reply = None
        for baud in (BAUD, STREAM_BAUD):   # a previous session may have left the node fast
            self.ser.baudrate = baud
            self.ser.reset_input_buffer()
            # Link reset as ClearPath-MSP sends at connect: extended-low and
            # extended-high packets with payload 00. They clear a half-parsed packet.
            for pkt_type in (PKT_EXT_LOW, PKT_EXT_HIGH):
                self.ser.write(frame(pkt_type, 0, b"\x00"))
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self._pkts.clear()
            self.ser.write(frame(PKT_SET_ADDR, 0))
            reply = self._read_packet(1.0)
            if reply is not None:
                break
        if reply is None:
            raise ConnectionError("no reply to set-address; is MSP holding the port?")
        self.node_count = reply[0] & 0xF
        if self.ser.baudrate != BAUD:
            self.reset_rate()

    def close(self) -> None:
        self.ser.close()

    def _pump(self) -> None:
        """Move received bytes into checksum-valid packets."""
        data = self.ser.read(max(1, self.ser.in_waiting))
        for b in data:
            if b & 0x80:
                self._rx = bytearray([b])
            elif self._rx:
                self._rx.append(b)
            else:
                continue
            if len(self._rx) >= 2 and len(self._rx) == 3 + (self._rx[1] & 0x1F):
                pkt = bytes(self._rx)
                self._rx = bytearray()
                if sum(pkt) & 0x7F:
                    self.bad += 1
                elif is_stream(pkt):
                    self.stream.append((time.time(), pkt))
                else:
                    self._pkts.append((time.time(), pkt))

    def _read_packet(self, timeout: float = 0.3) -> bytes | None:
        """Next command reply; scope samples received meanwhile go to self.stream."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._pkts:
                return self._pkts.pop(0)[1]
            self._pump()
        return None

    def command(self, payload: bytes, pkt_type: int = PKT_CMD, timeout: float = 0.5) -> bytes:
        """Send one command and return its reply's payload characters."""
        self._pkts.clear()
        self.ser.write(frame(pkt_type, 0, payload))
        pkt = self._read_packet(timeout)
        if pkt is None:
            raise TimeoutError("no reply to command %s" % payload.hex())
        return pkt[2:-1]

    def set_rate(self, baud: int) -> None:
        """Ask the node for a new link rate, then follow it (public SDK infcChangeLinkSpeed)."""
        if baud % RATE_DIVIDER:
            raise ValueError("rate must be a multiple of %d" % RATE_DIVIDER)
        self.command(bytes([EXT_BAUD_RATE, baud // RATE_DIVIDER]), PKT_EXT_HIGH)
        self.ser.baudrate = baud
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self._rx = bytearray()
        if self.read("device_id") >> 8 != 11:
            raise ConnectionError("no ClearPath reply at %d baud" % baud)

    def reset_rate(self) -> None:
        """Back to 9600 baud with the same rate-change command."""
        if self.ser.baudrate != BAUD:
            self.set_rate(BAUD)

    def clear_estop(self) -> None:
        """Clear the node's EStopped alert (public SDK STOP_TYPE_CLR_ESTOP)."""
        self.command(bytes([NODE_STOP_CMD, STOP_TYPE_CLR_ESTOP]))

    def monitor_get(self, ch: int) -> tuple[int, int, int, int]:
        """(variable, gain, filter, tune sync) of monitor channel ch (1 or 2)."""
        raw = groups_to_int(self.command(bytes([MON_CMD[ch]]))).to_bytes(10, "little")
        return (int.from_bytes(raw[0:2], "little"), int.from_bytes(raw[2:6], "little", signed=True),
                int.from_bytes(raw[6:8], "little", signed=True), int.from_bytes(raw[8:10], "little"))

    def monitor_set(self, ch: int, var: int, gain: int, filt: int, sync: int) -> None:
        """Write the channel's working setting. MON_SAVE_NV is refused."""
        if var & MON_SAVE_NV:
            raise ValueError("refusing to save the monitor setting to the motor")
        # Command byte plus 9 bytes: the SDK's PktLen of 10 cuts tuneSync to its low byte.
        payload = (bytes([MON_CMD[ch]]) + var.to_bytes(2, "little") + gain.to_bytes(4, "little", signed=True)
                   + filt.to_bytes(2, "little", signed=True) + bytes([sync & 0xFF]))
        reply = self.command(payload)
        if reply:
            raise ConnectionError("unexpected reply to monitor write: %s" % reply.hex())

    def scope(self, mode: int) -> None:
        """mode 3 streams channel 1, 4 both channels, 0 stops."""
        self.command(bytes([SCOPE_CMD, mode]))

    def raw(self, pnum: int) -> bytes:
        """Payload characters (7-bit) of a get-parameter reply."""
        bank, idx = pnum >> 8, pnum & 0xFF
        return self.command(bytes([GET_CMD[bank], idx]))

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


ENCODER_COUNTS_PER_REV = 6400   # measured: 2 revolutions read as 12,800 counts
VEL_RAW_PER_RPM = 8.136         # measured at gain 0x00206C3C (MSP's 500 RPM/div)


class ScopeSession:
    """Stream one monitor variable on channel 1, then put everything back.

    Samples are (host_time, counter, raw, flags4, flags5, lanes6). Use as a
    context manager, and call poll() often so samples are collected.
    """

    def __init__(self, cp: ClearPathUsb, var_name: str):
        self.cp = cp
        self.var, self.gain = MON_VARS[var_name]
        self.name = var_name
        self.samples = []
        self.original = None
        self._last_keepalive = 0.0
        steps_per_rev = cp.read("input_resolution")
        self.res_scale = ENCODER_COUNTS_PER_REV / steps_per_rev

    def __enter__(self):
        self.cp.set_rate(STREAM_BAUD)
        self.original = self.cp.monitor_get(1)
        orig_var = self.original[0]
        self.cp.monitor_set(1, self.var, self.gain, 0, self.original[3])
        if self.cp.monitor_get(1)[0] != self.var:
            raise ConnectionError("monitor channel 1 did not take variable %d" % self.var)
        self.cp.stream.clear()
        self.cp.scope(3)
        print("scope: streaming %s (id %d); channel 1 was id %d" % (self.name, self.var, orig_var & 0x0FFF),
              flush=True)
        return self

    def poll(self) -> None:
        self.cp._pump()
        if time.time() - self._last_keepalive > 0.3:   # MSP sends cmd 5 about 3 times a second
            self.cp.ser.write(frame(PKT_CMD, 0, bytes([KEEPALIVE_CMD])))
            self._last_keepalive = time.time()
        self.cp._pkts.clear()
        for t, pkt in self.cp.stream:
            b = groups_to_int(pkt[2:-1]).to_bytes(7, "little")
            self.samples.append((t, b[1] & 3, int.from_bytes(b[2:4], "little", signed=True), b[4], b[5], b[6]))
        self.cp.stream.clear()

    def scaled(self, raw: int):
        """Physical value where the scale is known, else None."""
        if self.var == 27:     # public SDK convertMonGain, MON_POSN_DIR_TRK; returns encoder counts
            full_scale_steps = 32768 / (self.gain / MON_SCALE * self.res_scale)
            return raw / 65536 * full_scale_steps * self.res_scale
        if self.var in (2, 3):
            return raw / (VEL_RAW_PER_RPM * self.gain / 0x00206C3C)
        return None

    def __exit__(self, *exc):
        errors = []
        for step in (lambda: self.cp.scope(0), lambda: self.cp.scope(0),
                     lambda: self.cp.monitor_set(1, self.original[0] & ~MON_SAVE_NV, *self.original[1:]),
                     lambda: self.cp.monitor_get(1) == self.original or errors.append("restore mismatch"),
                     self.cp.reset_rate):
            try:
                step()
            except Exception as e:  # keep restoring the rest
                errors.append(repr(e))
        print("scope: stopped, channel 1 restored, link back at %d baud%s"
              % (BAUD, ("; problems: " + "; ".join(errors)) if errors else ""), flush=True)
        return False


def cmd_scope(cp: ClearPathUsb, args) -> None:
    with ScopeSession(cp, args.var) as sc:
        t_end = time.time() + args.seconds
        while time.time() < t_end:
            sc.poll()
    t0 = sc.samples[0][0] if sc.samples else 0.0
    with open(args.csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(("t_s", "counter", "raw", "scaled", "flags4", "flags5", "lanes6"))
        for t, ctr, raw, f4, f5, l6 in sc.samples:
            v = sc.scaled(raw)
            w.writerow(("%.4f" % (t - t0), ctr, raw, "" if v is None else "%.4f" % v, f4, f5, l6))
    span = (sc.samples[-1][0] - t0) if sc.samples else 0.0
    raws = [s[2] for s in sc.samples]
    print("%d samples in %.2f s (%.0f/s), %d bad packets; raw min %s max %s -> %s"
          % (len(sc.samples), span, len(sc.samples) / span if span else 0, cp.bad,
             min(raws) if raws else "-", max(raws) if raws else "-", args.csv))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="COM port (default: auto-detect Teknic 2890:0201)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info").set_defaults(func=cmd_info)
    mon = sub.add_parser("monitor")
    mon.add_argument("--seconds", type=float, default=30.0)
    mon.add_argument("--csv")
    mon.set_defaults(func=cmd_monitor)
    sc = sub.add_parser("scope", help="stream one monitor variable to CSV")
    sc.add_argument("--var", choices=sorted(MON_VARS), default="position_error")
    sc.add_argument("--seconds", type=float, default=10.0)
    sc.add_argument("--csv", required=True)
    sc.set_defaults(func=cmd_scope)
    args = ap.parse_args()
    cp = ClearPathUsb(args.port or find_port())
    try:
        args.func(cp, args)
    finally:
        cp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
