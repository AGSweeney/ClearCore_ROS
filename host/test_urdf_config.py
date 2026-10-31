# MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
"""Unit tests for the URDF/board reconciliation checker (no hardware)."""

from __future__ import annotations

import os

from check_urdf_config import DEFAULT_URDF, compare, parse_urdf


def _board(**over):
    """A board get_config that matches the repo URDF by default."""
    cfg = {
        "axis_mask": 3,
        "names": ["joint_x", "joint_y", "joint_z", "joint_a"],
        "steps_per_rev": [800, 800, 800, 800],
        "pitch_mm": [5.0, 5.0, 5.0, 5.0],
        "vel_steps": 27000, "accel_steps": 250000, "decel_steps": 250000,
        "watchdog_ms": 500,
        "limit_flags": 0,
        "limits_min": [0.0, 0.0, 0.0, 0.0],
        "limits_max": [0.0, 0.0, 0.0, 0.0],
    }
    cfg.update(over)
    return cfg


def _params():
    return {
        "axis_mask": "3", "steps_per_rev": "800", "pitch_mm": "5.0",
        "vel_steps": "27000", "accel_steps": "250000", "decel_steps": "250000",
        "watchdog_ms": "500",
    }


def _joints():
    return [
        {"name": "joint_x", "lower": -1.0, "upper": 1.0, "velocity": 0.17},
        {"name": "joint_y", "lower": -1.0, "upper": 1.0, "velocity": 0.17},
    ]


def _errors(findings):
    return [m for lvl, m in findings if lvl == "ERROR"]


def test_repo_urdf_parses():
    assert os.path.exists(DEFAULT_URDF)
    params, joints = parse_urdf(DEFAULT_URDF)
    assert params["steps_per_rev"] == "800"
    assert params["axis_mask"] == "3"  # resolved from $(arg axis_mask) default
    assert [j["name"] for j in joints] == ["joint_x", "joint_y"]
    assert joints[0]["upper"] == 1.0


def test_matching_config_has_no_errors():
    findings = compare(_board(), _params(), _joints())
    assert _errors(findings) == []


def test_scalar_mismatch_is_error():
    findings = compare(_board(vel_steps=20000), _params(), _joints())
    assert any("vel_steps" in m for m in _errors(findings))


def test_joint_name_mismatch_is_error():
    findings = compare(_board(names=["jx", "joint_y", "", ""]), _params(), _joints())
    assert any("joint[x] name" in m for m in _errors(findings))


def test_per_axis_scaling_mismatch_is_error():
    findings = compare(_board(pitch_mm=[5.0, 2.0, 5.0, 5.0]), _params(), _joints())
    assert any("pitch_mm[y]" in m for m in _errors(findings))


def test_disabled_soft_limit_is_warn_not_error():
    findings = compare(_board(), _params(), _joints())  # limit_flags 0 = off
    assert _errors(findings) == []
    assert any(lvl == "WARN" and "not enforced" in m for lvl, m in findings)


def test_enabled_soft_limit_mismatch_is_error():
    # X min+max enabled (bits 0,1) but board limits differ from URDF +/-1.0
    findings = compare(
        _board(limit_flags=0b11, limits_min=[-0.5, 0, 0, 0], limits_max=[0.5, 0, 0, 0]),
        _params(), _joints())
    assert any("limit x.lower" in m for m in _errors(findings))
    assert any("limit x.upper" in m for m in _errors(findings))


if __name__ == "__main__":
    test_repo_urdf_parses()
    test_matching_config_has_no_errors()
    test_scalar_mismatch_is_error()
    test_joint_name_mismatch_is_error()
    test_per_axis_scaling_mismatch_is_error()
    test_disabled_soft_limit_is_warn_not_error()
    test_enabled_soft_limit_mismatch_is_error()
    print("urdf_config ok")
