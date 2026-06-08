"""Batch-2 safety-filter completeness (from the conformance audit): attributable
clip reasons, opt-in acceleration cap, the STALE_STATE watchdog, and bounded-hold
escalation. The opt-in features default OFF, so default behaviour is unchanged."""

from __future__ import annotations

import time

import numpy as np

from flexiv_control import Robot, RobotConfig, SafetyStatus
from flexiv_control.safety import SafetyFilter, SafetyProfile
from flexiv_control.server.control_loop import ReactiveServoLoop
from flexiv_control.types import RobotState, StopReason


def _state(pose):
    return RobotState(tcp_pose=np.asarray(pose, float), q=np.zeros(7), wrench=np.zeros(6))


def test_clip_reasons_are_attributed():
    f = SafetyFilter(SafetyProfile(), control_dt=0.01)  # tabletop_safe defaults
    f.reset(pose=np.array([0.45, 0.0, 0.30, 1, 0, 0, 0]))

    # Outside the workspace box (x=0.9 > 0.75) -> WORKSPACE is among the reasons.
    r = f.filter_cartesian(np.array([0.9, 0.0, 0.30, 1, 0, 0, 0]), _state([0.45, 0, 0.30, 1, 0, 0, 0]))
    assert r.clipped and StopReason.WORKSPACE in r.clip_reasons
    assert r.primary_clip_reason == StopReason.WORKSPACE

    # In-workspace but a big jump (0.45 -> 0.50 = 0.05 > 0.03) -> POSE_JUMP, not WORKSPACE.
    f.reset(pose=np.array([0.45, 0.0, 0.30, 1, 0, 0, 0]))
    r2 = f.filter_cartesian(np.array([0.50, 0.0, 0.30, 1, 0, 0, 0]), _state([0.45, 0, 0.30, 1, 0, 0, 0]))
    assert StopReason.POSE_JUMP in r2.clip_reasons
    assert StopReason.WORKSPACE not in r2.clip_reasons

    # A clean in-spec command clips nothing.
    f.reset(pose=np.array([0.45, 0.0, 0.30, 1, 0, 0, 0]))
    r3 = f.filter_cartesian(np.array([0.451, 0.0, 0.30, 1, 0, 0, 0]), _state([0.45, 0, 0.30, 1, 0, 0, 0]))
    assert not r3.clipped and r3.clip_reasons == []


def test_acceleration_cap_bounds_velocity_change():
    dt = 0.01
    # Disable jump/speed caps so ONLY the accel cap acts; cap dv to 1.0*dt = 0.01 m/s.
    p = SafetyProfile(max_linear_accel=1.0, max_linear_speed=1e3, max_pose_jump_linear=1e3)
    f = SafetyFilter(p, control_dt=dt)
    f.reset(pose=np.array([0.45, 0.0, 0.30, 1, 0, 0, 0]))  # v_prev = 0

    tgt = np.array([0.55, 0.0, 0.30, 1, 0, 0, 0])  # in-workspace, far
    r1 = f.filter_cartesian(tgt, _state([0.45, 0, 0.30, 1, 0, 0, 0]))
    v1 = (r1.pose[:3] - np.array([0.45, 0, 0.30])) / dt
    assert np.linalg.norm(v1) <= 1.0 * dt + 1e-9  # from rest, |v| <= a_max*dt
    assert r1.clipped and StopReason.TCP_SPEED in r1.clip_reasons

    r2 = f.filter_cartesian(tgt, _state(r1.pose))
    v2 = (r2.pose[:3] - r1.pose[:3]) / dt
    assert np.linalg.norm(v2) > np.linalg.norm(v1)            # velocity ramps up
    assert np.linalg.norm(v2 - v1) <= 1.0 * dt + 1e-9         # but dv stays C1-bounded


def test_state_age_watchdog_opt_in():
    on = SafetyFilter(SafetyProfile(stop_on_state_stale=True, state_timeout_ms=20.0), 0.01)
    assert on.check_state_age(5.0).ok
    bad = on.check_state_age(50.0)
    assert not bad.ok and bad.reason == StopReason.STALE_STATE
    # Default off: never trips, even for an absurd age.
    assert SafetyFilter(SafetyProfile(), 0.01).check_state_age(1e9).ok


def test_loop_escalates_stale_hold_to_protective_stop():
    r = Robot(RobotConfig(backend="fake"))
    r.connect()
    r.start_cartesian_impedance()
    r.filter.p.command_timeout_ms = 50.0
    r.filter.p.hold_timeout_ms = 250.0  # escalate hold -> stop after 250 ms stale
    loop = ReactiveServoLoop(r, control_hz=200.0)
    loop.start()
    try:
        loop.set_cartesian_target(loop.get_state().tcp_pose.copy())
        time.sleep(0.12)  # 50 ms < age < 250 ms -> HOLDING
        assert loop.get_state().safety_status == SafetyStatus.HOLDING
        time.sleep(0.30)  # age > 250 ms -> escalated protective stop
        st = loop.get_state()
        assert st.safety_status == SafetyStatus.STOPPED
        assert st.stop_reason == StopReason.STALE_COMMAND
    finally:
        loop.stop()
