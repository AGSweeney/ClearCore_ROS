"""Reconcile the ros2_control URDF against the board's saved configuration.

The URDF (clearcore.urdf.xacro) carries the joint names, the ros2_control
`<param>` mechanics the plugin writes on activation, and `<limit>` values a
planner reads. The board keeps its own copy in NVM. They can drift: a param the
plugin never sends (gear, offset, direction), an edited URDF limit, or a board
configured out of band. This compares the two and reports every mismatch so the
two descriptions cannot silently disagree.

Usage:
    python check_urdf_config.py <board-ip> [path/to/clearcore.urdf.xacro]

Exit code is non-zero if any ERROR-level mismatch is found. WARN items (e.g. a
URDF limit the firmware does not enforce) do not fail the check.
"""

from __future__ import annotations

import os
import re
import sys
import xml.etree.ElementTree as ET

AXES = ["x", "y", "z", "a"]
DEFAULT_URDF = os.path.join(
    os.path.dirname(__file__), "..", "ros2_ws", "src",
    "clearcore_hardware", "urdf", "clearcore.urdf.xacro",
)


def parse_urdf(path: str) -> tuple[dict, list[dict]]:
    """Return (ros2_control params, joints). `$(arg x)` resolves to its
    `<xacro:arg default>`; this scaffold has no includes or macros to expand."""
    text = open(path, encoding="utf-8").read()
    args = dict(re.findall(r'<xacro:arg\s+name="([^"]+)"\s+default="([^"]*)"', text))
    text = re.sub(r"\$\(arg ([^)]+)\)", lambda m: args.get(m.group(1), m.group(0)), text)
    root = ET.fromstring(text)

    params: dict[str, str] = {}
    rc = root.find("ros2_control")
    if rc is not None:
        hw = rc.find("hardware")
        for p in (hw.findall("param") if hw is not None else []):
            params[p.get("name")] = (p.text or "").strip()

    joints = []
    for j in root.findall("joint"):
        lim = j.find("limit")
        if lim is None:
            continue  # fixed joints etc. have no limit
        joints.append({
            "name": j.get("name"),
            "lower": float(lim.get("lower")) if lim.get("lower") is not None else None,
            "upper": float(lim.get("upper")) if lim.get("upper") is not None else None,
            "velocity": float(lim.get("velocity")) if lim.get("velocity") is not None else None,
        })
    return params, joints


def compare(cfg: dict, params: dict, joints: list[dict]) -> list[tuple[str, str]]:
    """Return (level, message) findings. level is 'ERROR', 'WARN', or 'OK'."""
    out: list[tuple[str, str]] = []
    mask = cfg.get("axis_mask", 0)
    active = [i for i in range(4) if mask & (1 << i)]

    # axis_mask
    if "axis_mask" in params:
        want = int(params["axis_mask"])
        lvl = "OK" if want == mask else "ERROR"
        out.append((lvl, f"axis_mask: urdf={want} board={mask}"))

    # joint names, in axis order, for the active axes
    names = cfg.get("names", [])
    for idx, j in enumerate(joints):
        if idx >= len(active):
            out.append(("ERROR", f"joint '{j['name']}': urdf declares more joints than axis_mask has active axes"))
            continue
        a = active[idx]
        fw = names[a] if a < len(names) else "?"
        lvl = "OK" if fw == j["name"] else "ERROR"
        out.append((lvl, f"joint[{AXES[a]}] name: urdf='{j['name']}' board='{fw}'"))

    # scalar mechanics the plugin writes, compared per active axis / globally
    def scalar(name, key, cast):
        if name not in params:
            return
        want = cast(params[name])
        got = cfg.get(key)
        lvl = "OK" if got == want else "ERROR"
        out.append((lvl, f"{name}: urdf={want} board={got}"))

    scalar("vel_steps", "vel_steps", int)
    scalar("accel_steps", "accel_steps", int)
    scalar("decel_steps", "decel_steps", int)
    scalar("watchdog_ms", "watchdog_ms", int)

    # steps_per_rev / pitch_mm are a single URDF param applied to every axis
    for key, pname, cast in (("steps_per_rev", "steps_per_rev", int),
                             ("pitch_mm", "pitch_mm", float)):
        if pname not in params:
            continue
        want = cast(params[pname])
        arr = cfg.get(key, [])
        for a in active:
            got = arr[a] if a < len(arr) else None
            same = got == want if cast is int else abs((got or 0) - want) < 1e-6
            out.append(("OK" if same else "ERROR", f"{key}[{AXES[a]}]: urdf={want} board={got}"))

    # URDF <limit> vs firmware soft limits. The URDF comment calls these
    # informational; firmware enforces only when its soft-limit flag is set.
    flags = cfg.get("limit_flags", 0)
    lmin = cfg.get("limits_min", [0, 0, 0, 0])
    lmax = cfg.get("limits_max", [0, 0, 0, 0])
    for idx, j in enumerate(joints):
        if idx >= len(active):
            break
        a = active[idx]
        min_en = bool(flags & (1 << (a * 2)))
        max_en = bool(flags & (1 << (a * 2 + 1)))
        for side, en, fw_val, urdf_val in (
            ("lower", min_en, lmin[a] if a < len(lmin) else None, j["lower"]),
            ("upper", max_en, lmax[a] if a < len(lmax) else None, j["upper"]),
        ):
            if urdf_val is None:
                continue
            if not en:
                out.append(("WARN", f"limit {AXES[a]}.{side}: urdf={urdf_val} but firmware soft limit is OFF (not enforced)"))
            else:
                same = abs(fw_val - urdf_val) < 1e-6
                out.append(("OK" if same else "ERROR", f"limit {AXES[a]}.{side}: urdf={urdf_val} board={fw_val}"))
    return out


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(2)
    ip = sys.argv[1]
    urdf = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_URDF
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "clearcore_bridge"))
    from clearcore_bridge.wire import SessionClient

    params, joints = parse_urdf(urdf)
    s = SessionClient(ip, 9200, 3.0)
    try:
        cfg = s.call("get_config")
    finally:
        s.close()

    findings = compare(cfg, params, joints)
    errors = sum(1 for lvl, _ in findings if lvl == "ERROR")
    warns = sum(1 for lvl, _ in findings if lvl == "WARN")
    for lvl, msg in findings:
        print(f"[{lvl:5}] {msg}")
    print(f"\n{errors} mismatch(es), {warns} warning(s)")
    if errors:
        raise SystemExit(1)
    print("URDF and board configuration agree.")


if __name__ == "__main__":
    main()
