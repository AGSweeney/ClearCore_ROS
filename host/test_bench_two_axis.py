"""Two-axis bench run on M0 (X) and M1 (Y). Moves real motors.

Bare motors on a bench only. Travel is up to 0.12 m of joint motion (about 24
revolutions at 5 mm pitch). Phases:

  enable cycles, point moves, simultaneous move, coordinated square and
  diagonal, feed rate not carried into set_joints, arc, streamed track,
  soft-limit reject, stop mid-move,
  estop mid-move and recovery, watchdog trip and recovery, stream loss,
  multi-target endurance.

The saved config is captured first and written back at the end, with motors
parked at 0 and disabled. Test mode is forced off so enable checks HLFB.
Usage: test_bench_two_axis.py HOST [--cycles N]
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "ros2_ws" / "src" / "clearcore_bridge"))
sys.path.insert(0, str(HERE))

from clearcore_bridge.wire import SessionClient, StreamClient  # noqa: E402
from test_m0_live import restore_mismatches, restore_params, same_number  # noqa: E402

TOL_M = 0.0005
SETTLE_S = 10.0
results: list[tuple[str, bool, str]] = []
# Per-motor anomalies seen in any status poll, tagged by phase.
motor_events: list[str] = []
current_phase = "setup"
_t0 = time.time()
_last_axis_state: dict[int, str] = {}


def axis_state(st: dict, a: int) -> str:
    ax = st["axes"][a]
    bits = []
    if ax.get("motor_in_fault"):
        bits.append("motor_in_fault")
    if ax.get("alerts_present"):
        bits.append("alerts_present")
    if ax.get("alert_reg"):
        bits.append("alert_reg=0x%x" % ax["alert_reg"])
    if st.get("enabled") and ax.get("hlfb") == "deasserted":
        bits.append("hlfb deasserted while enabled")
    return ",".join(bits)


def status(session) -> dict:
    """get_status, logging each change in M0/M1 fault state."""
    st = session.call("get_status")
    for a in (0, 1):
        now = axis_state(st, a)
        if now != _last_axis_state.get(a, ""):
            if now:
                motor_events.append(
                    "%7.2fs  %-28s M%d %s  (alerts=%s pos %s)"
                    % (time.time() - _t0, current_phase, a, now, st["alerts"], pos(st))
                )
            _last_axis_state[a] = now
    return st


def record(name: str, ok: bool, detail: str = "") -> bool:
    results.append((name, ok, detail))
    print("%s  %-34s %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)
    return ok


def bad_flags(st: dict) -> str:
    out = [k for k in ("fault", "estop", "watchdog") if st.get(k)]
    if not st.get("enabled"):
        out.append("disabled")
    return ",".join(out)


def wait_at(session, x: float, y: float, timeout: float = SETTLE_S) -> dict:
    deadline = time.time() + timeout
    st = status(session)
    while time.time() < deadline:
        st = status(session)
        if bad_flags(st):
            return st
        px, py = st["position"][0], st["position"][1]
        if abs(px - x) <= TOL_M and abs(py - y) <= TOL_M and not st["moving"]:
            return st
        time.sleep(0.05)
    return st


def at(st: dict, x: float, y: float) -> bool:
    return (
        not bad_flags(st)
        and abs(st["position"][0] - x) <= TOL_M
        and abs(st["position"][1] - y) <= TOL_M
        and not st["moving"]
    )


def pos(st: dict) -> str:
    return "x=%+.5f y=%+.5f" % (st["position"][0], st["position"][1])


def recover(session) -> dict:
    """clear_alerts, enable, park at 0. Used after deliberate trips."""
    status(session)  # log motor faults before clear_alerts wipes them
    session.call("clear_alerts")
    st = status(session)
    if not st["enabled"]:
        session.call("enable")
    session.call("set_joints", {"x": 0.0, "y": 0.0})
    return wait_at(session, 0.0, 0.0)


def preflight(session) -> dict:
    cfg = session.call("get_config")
    st = status(session)
    problems = []
    for i, axis in enumerate(("x", "y")):
        if int(cfg["rotary"][i]) != 0:
            problems.append("rotary_%s" % axis)
        if int(cfg["direction"][i]) != 1:
            problems.append("direction_%s" % axis)
        if not same_number(cfg["gear"][i], 1.0):
            problems.append("gear_%s" % axis)
        if not same_number(cfg["offset"][i], 0.0):
            problems.append("offset_%s" % axis)
    if st["enabled"]:
        problems.append("motors already enabled")
    if st["fault"] or st["estop"]:
        problems.append("fault/estop present: %s" % st["alerts"])
    if problems:
        raise SystemExit("preflight: " + ", ".join(problems))
    return cfg


def phase_enable_cycles(session) -> None:
    ok = True
    detail = ""
    for i in range(5):
        session.call("enable")
        st = status(session)
        hlfb = [a["hlfb"] for a in st["axes"][:2]]
        if not st["enabled"] or any(h == "deasserted" for h in hlfb):
            ok = False
            detail = "cycle %d enabled=%s hlfb=%s" % (i + 1, st["enabled"], hlfb)
            break
        session.call("disable")
        if status(session)["enabled"]:
            ok = False
            detail = "cycle %d did not disable" % (i + 1)
            break
    record("enable/disable x5 (HLFB checked)", ok, detail or "hlfb asserted each cycle")


def phase_point_moves(session) -> None:
    for name, target in (("X 10 mm out/back", {"x": 0.010}), ("Y 10 mm out/back", {"y": 0.010})):
        session.call("set_joints", target)
        x = target.get("x", 0.0)
        y = target.get("y", 0.0)
        out = wait_at(session, x, y)
        session.call("set_joints", {"x": 0.0, "y": 0.0})
        back = wait_at(session, 0.0, 0.0)
        record(name, at(out, x, y) and at(back, 0.0, 0.0), "out %s, back %s" % (pos(out), pos(back)))
    session.call("set_joints", {"x": 0.03, "y": 0.03})
    out = wait_at(session, 0.03, 0.03)
    session.call("set_joints", {"x": 0.0, "y": 0.0})
    back = wait_at(session, 0.0, 0.0)
    record("XY 30 mm together", at(out, 0.03, 0.03) and at(back, 0.0, 0.0), "out %s, back %s" % (pos(out), pos(back)))


def sample_move(session, timeout: float, fn) -> tuple[dict, float]:
    """Poll get_status until motion is idle and return (last, peak fn(position))."""
    peak = 0.0
    deadline = time.time() + timeout
    time.sleep(0.03)
    st = status(session)
    while time.time() < deadline:
        st = status(session)
        peak = max(peak, fn(st["position"]))
        if bad_flags(st) or not st["moving"]:
            break
    time.sleep(0.05)
    return status(session), peak


def phase_coordinated(session) -> None:
    corners = ((0.02, 0.0), (0.02, 0.02), (0.0, 0.02), (0.0, 0.0))
    worst = 0.0
    ok = True
    for x, y in corners:
        reply = session.call("move_linear", {"x": x, "y": y, "feed_mps": 0.03})
        session.call("wait_idle", {"timeout_ms": 10000}, timeout=12.0)
        st = status(session)
        err = max(abs(st["position"][0] - x), abs(st["position"][1] - y))
        worst = max(worst, err)
        ok = ok and reply.get("coordinated", False) and err <= TOL_M and not bad_flags(st)
    record("coordinated square 20 mm", ok, "worst corner error %.5f m" % worst)

    session.call("move_linear", {"x": 0.03, "y": 0.03, "feed_mps": 0.03})
    st, peak = sample_move(session, 10.0, lambda p: abs(p[1] - p[0]))
    end_ok = at(st, 0.03, 0.03)
    session.call("move_linear", {"x": 0.0, "y": 0.0, "feed_mps": 0.03})
    st2, peak2 = sample_move(session, 10.0, lambda p: abs(p[1] - p[0]))
    peak = max(peak, peak2)
    record(
        "coordinated diagonal 30 mm",
        end_ok and at(st2, 0.0, 0.0) and peak <= 0.0005,
        "peak |Y-X| %.4f mm, end %s" % (peak * 1000.0, pos(st2)),
    )


def phase_feed_does_not_stick(session, cfg: dict) -> None:
    """A slow move_linear feed must not slow the next set_joints move."""
    session.call("move_linear", {"x": 0.01, "y": 0.0, "feed_mps": 0.01})
    session.call("wait_idle", {"timeout_ms": 10000}, timeout=12.0)
    session.call("set_joints", {"x": 0.0, "y": 0.0})
    wait_at(session, 0.0, 0.0)
    steps_per_m = cfg["steps_per_rev"][0] / (cfg["pitch_mm"][0] / 1000.0)
    expect = 0.05 * steps_per_m / cfg["vel_steps"] + cfg["vel_steps"] / cfg["accel_steps"]
    t0 = time.time()
    session.call("set_joints", {"x": 0.05})
    st = wait_at(session, 0.05, 0.0)
    took = time.time() - t0
    session.call("set_joints", {"x": 0.0})
    back = wait_at(session, 0.0, 0.0)
    record(
        "set_joints speed after feed move",
        at(st, 0.05, 0.0) and at(back, 0.0, 0.0) and took < 2.0 * expect,
        "50 mm took %.2f s, vel_steps predicts %.2f s" % (took, expect),
    )


def phase_arc(session) -> None:
    # Half circle from (0,0) to (0.02,0) about (0.01,0), then back the other way.
    radius = 0.01

    def rad_err(p):
        return abs(math.hypot(p[0] - 0.01, p[1]) - radius)

    session.call("move_arc", {"x": 0.02, "y": 0.0, "i": 0.01, "j": 0.0, "cw": True, "feed_mps": 0.02})
    st, peak = sample_move(session, 10.0, rad_err)
    out_ok = at(st, 0.02, 0.0)
    session.call("move_arc", {"x": 0.0, "y": 0.0, "i": -0.01, "j": 0.0, "cw": True, "feed_mps": 0.02})
    st2, peak2 = sample_move(session, 10.0, rad_err)
    peak = max(peak, peak2)
    record(
        "arc 10 mm radius, full circle",
        out_ok and at(st2, 0.0, 0.0) and peak <= 0.0005,
        "peak radius error %.4f mm, end %s" % (peak * 1000.0, pos(st2)),
    )


def phase_track(session, host: str) -> None:
    """Stream track frames on X and Y for two periods, then hold at 0."""
    stream = StreamClient(host, 9201, 3.0)
    amp, period, dur = 0.02, 3.0, 6.0
    w = 2 * math.pi / period
    peak = [0.0, 0.0]
    frames = 0
    try:
        t0 = time.time()
        next_t = t0
        while True:
            t = time.time() - t0
            if t > dur:
                break
            x, vx = amp * math.sin(w * t), amp * w * math.cos(w * t)
            y, vy = amp * (1 - math.cos(w * t)), amp * w * math.sin(w * t)
            stream.send_track(0x3, (x, y, 0.0, 0.0), (vx, vy, 0.0, 0.0))
            for fr in stream.read(0.0):
                if fr.get("type") != "state":
                    continue
                frames += 1
                for a in (0, 1):
                    if fr["track_mask"] & (1 << a) and fr["target_latch_ms"][a]:
                        dt = (fr["time_ms"] - fr["target_latch_ms"][a]) / 1000.0
                        ref = fr["target_position"][a] + fr["target_velocity"][a] * dt
                        if t > 0.3:
                            peak[a] = max(peak[a], abs(fr["position"][a] - ref))
            next_t += 0.02
            time.sleep(max(0.0, next_t - time.time()))
        stream.send_position(0x3, (0.0, 0.0, 0.0, 0.0))
        for _ in range(30):
            stream.send_heartbeat()
            time.sleep(0.05)
        st = wait_at(session, 0.0, 0.0)
    finally:
        stream.close()
    record(
        "streamed track, sine on X and Y",
        at(st, 0.0, 0.0) and max(peak) <= 0.0005 and frames > 100,
        "peak local residual X %.4f mm Y %.4f mm, %d state frames, end %s"
        % (peak[0] * 1000.0, peak[1] * 1000.0, frames, pos(st)),
    )


def phase_soft_limit(session) -> None:
    session.call("configure", {"max_x": 0.015})
    rejected = ""
    try:
        session.call("set_joints", {"x": 0.02})
    except RuntimeError as exc:
        rejected = str(exc)
    time.sleep(0.2)
    st = status(session)
    session.call("configure", {"clear_max_x": True})
    record(
        "soft limit rejects past max_x",
        "above max" in rejected and abs(st["position"][0]) <= TOL_M,
        "error '%s', x stayed %+.5f" % (rejected, st["position"][0]),
    )


def phase_stop(session) -> None:
    session.call("set_joints", {"x": 0.10})
    watch(session, 0.25)
    session.call("stop")
    st = watch(session, 0.6)
    x = st["position"][0]
    ok = not st["moving"] and st["enabled"] and 0.001 < x < 0.099 and not bad_flags(st)
    back = recover(session)
    record("stop mid-move, stays enabled", ok and at(back, 0.0, 0.0), "stopped at x=%+.5f, back %s" % (x, pos(back)))


def watch(session, seconds: float) -> dict:
    """Poll status for a while so a transient motor fault is logged."""
    end = time.time() + seconds
    st = status(session)
    while time.time() < end:
        st = status(session)
    return st


def phase_estop(session) -> None:
    session.call("set_joints", {"x": 0.10, "y": 0.10})
    watch(session, 0.25)
    session.call("estop")
    st = watch(session, 1.0)
    stopped = not st["enabled"] and not st["moving"]
    back = recover(session)
    record(
        "estop mid-move, clear_alerts, re-enable",
        stopped and at(back, 0.0, 0.0),
        "after estop enabled=%s moving=%s alerts=%s; back %s" % (st["enabled"], st["moving"], st["alerts"], pos(back)),
    )


def phase_watchdog(session, host: str, watchdog_ms: int) -> None:
    if watchdog_ms == 0:
        record("watchdog trip on silent host", False, "watchdog_ms is 0 on this board")
        return
    stream = StreamClient(host, 9201, 3.0)
    try:
        stream.send_velocity(0x1, (0.01, 0.0, 0.0, 0.0))
        # Silence on both sockets: any session call would refresh the host timer.
        time.sleep(watchdog_ms / 1000.0 + 0.7)
        st = status(session)
        tripped = st["watchdog"] and not st["moving"]
        try:
            session.call("set_joints", {"x": 0.0})
        except RuntimeError:
            pass
        time.sleep(0.3)
        ignored = status(session)
    finally:
        stream.close()
    x_after = ignored["position"][0]
    held = abs(x_after - st["position"][0]) <= TOL_M
    back = recover(session)
    record(
        "watchdog trip, commands ignored, recover",
        tripped and held and at(back, 0.0, 0.0),
        "tripped=%s at x=%+.5f, command ignored=%s, back %s" % (st["watchdog"], st["position"][0], held, pos(back)),
    )


def phase_stream_loss(session, host: str) -> None:
    stream = StreamClient(host, 9201, 3.0)
    stream.send_velocity(0x1, (0.02, 0.0, 0.0, 0.0))
    for _ in range(4):
        stream.send_heartbeat()
        time.sleep(0.05)
    stream.close()
    t_close = time.time()
    st = status(session)
    while st["moving"] and time.time() - t_close < 2.0:
        st = status(session)
    stop_ms = (time.time() - t_close) * 1000.0
    ok = st["watchdog"] and not st["moving"]
    back = recover(session)
    record(
        "stream disconnect mid-move stops",
        ok and stop_ms < 300 and at(back, 0.0, 0.0),
        "watchdog=%s, still after %.0f ms, back %s" % (st["watchdog"], stop_ms, pos(back)),
    )


def phase_endurance(session, cycles: int) -> None:
    pattern = ((0.05, -0.03), (0.12, 0.08), (0.02, 0.10), (-0.04, -0.06), (0.08, 0.0), (0.0, 0.0))
    worst = 0.0
    moves = 0
    t0 = time.time()
    ok = True
    detail = ""
    for c in range(cycles):
        for x, y in pattern:
            session.call("set_joints", {"x": x, "y": y})
            st = wait_at(session, x, y)
            moves += 1
            err = max(abs(st["position"][0] - x), abs(st["position"][1] - y))
            worst = max(worst, err)
            if not at(st, x, y):
                ok = False
                detail = "cycle %d target (%.3f,%.3f) got %s flags=%s" % (c + 1, x, y, pos(st), bad_flags(st))
                break
        if not ok:
            break
    record(
        "endurance %d cycles x %d targets" % (cycles, len(pattern)),
        ok,
        detail or "%d moves in %.1f s, worst settle error %.5f m" % (moves, time.time() - t0, worst),
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("host", nargs="?", default=os.environ.get("CCROS_HOST", ""))
    ap.add_argument("--cycles", type=int, default=5)
    ap.add_argument("--phases", nargs="*", help="run only these phases (default: all)")
    args = ap.parse_args()
    if not args.host:
        raise SystemExit("pass the board address or set CCROS_HOST")

    session = SessionClient(args.host, 9200, 3.0)
    saved = preflight(session)
    print("board %s  saved axis_mask=%s test_mode=%s watchdog_ms=%s" % (
        args.host, saved["axis_mask"], saved["test_mode"], saved["watchdog_ms"]))
    restore_error = None
    try:
        session.call("set_test_mode", {"on": False})
        session.call("configure", {"axis_mask": 3})
        session.call("clear_alerts")
        phase_enable_cycles(session)
        session.call("enable")
        st = status(session)
        record("start at zero", at(st, 0.0, 0.0), pos(st))
        phases = (
            ("points", lambda: phase_point_moves(session)),
            ("coordinated", lambda: phase_coordinated(session)),
            ("feed", lambda: phase_feed_does_not_stick(session, saved)),
            ("arc", lambda: phase_arc(session)),
            ("track", lambda: phase_track(session, args.host)),
            ("softlimit", lambda: phase_soft_limit(session)),
            ("stop", lambda: phase_stop(session)),
            ("estop", lambda: phase_estop(session)),
            ("watchdog", lambda: phase_watchdog(session, args.host, int(saved["watchdog_ms"]))),
            ("streamloss", lambda: phase_stream_loss(session, args.host)),
            ("endurance", lambda: phase_endurance(session, args.cycles)),
        )
        global current_phase
        for name, run in phases:
            if args.phases and name not in args.phases:
                continue
            current_phase = name
            try:
                run()
            except Exception as exc:  # keep going; park before the next phase
                record("phase error", False, repr(exc))
                recover(session)
            current_phase = name + " (after)"
            status(session)
    finally:
        current_phase = "cleanup"
        try:
            st = recover(session)
            print("parked %s" % pos(st))
            session.call("disable")
            session.call("set_test_mode", {"on": bool(saved["test_mode"])})
            session.call("configure", restore_params(saved))
            bad = restore_mismatches(saved, session.call("get_config"), status(session))
            if bad:
                restore_error = "restore mismatch: " + ", ".join(bad)
        except Exception as exc:
            restore_error = repr(exc)
        session.close()
    record("config restored, motors disabled", restore_error is None, restore_error or "")
    record("no motor faults on M0/M1", not motor_events, "%d event(s)" % len(motor_events))
    for line in motor_events:
        print("  " + line)
    failed = [r for r in results if not r[1]]
    print("\n%d/%d passed" % (len(results) - len(failed), len(results)))
    print("BENCH_TWO_AXIS_OK" if not failed else "BENCH_TWO_AXIS_FAIL")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
