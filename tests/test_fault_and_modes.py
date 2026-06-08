"""Batch-1 hardware-correctness regressions: the per-tick robot-fault gate and
the env reset() control-mode ordering (both found by the conformance audit)."""

from __future__ import annotations

import time

from flexiv_control import Robot, RobotConfig, SafetyStatus
from flexiv_control.envs import FlexivRealEnv
from flexiv_control.server.control_loop import ReactiveServoLoop
from flexiv_control.types import ControlMode, StopReason


def test_control_loop_halts_on_backend_fault():
    """A robot-side fault is invisible to the host safety filter; the loop must
    poll backend.in_fault() every tick and stop streaming when it trips."""
    r = Robot(RobotConfig(backend="fake"))
    r.connect()
    r.start_cartesian_impedance()
    loop = ReactiveServoLoop(r, control_hz=200.0)
    loop.start()
    try:
        s0 = loop.get_state()
        tgt = s0.tcp_pose.copy()
        tgt[0] += 0.05
        loop.set_cartesian_target(tgt)
        time.sleep(0.1)
        assert len(r.backend.cartesian_log) > 0  # streaming while healthy

        # Inject a robot-side fault.
        r.backend._fault = True
        time.sleep(0.1)
        st = loop.get_state()
        assert st.safety_status == SafetyStatus.FAULT
        assert st.stop_reason == StopReason.BACKEND_FAULT

        # While faulted, the loop must NOT issue new setpoints (it calls stop()).
        n = len(r.backend.cartesian_log)
        time.sleep(0.1)
        assert len(r.backend.cartesian_log) == n  # frozen -- no streaming into a faulted arm

        # Clearing the fault lets streaming resume.
        r.backend._fault = False
        loop.set_cartesian_target(tgt)
        time.sleep(0.1)
        assert len(r.backend.cartesian_log) > n
        assert loop.get_state().safety_status != SafetyStatus.FAULT
    finally:
        loop.stop()


def test_gym_reset_ends_in_cartesian_motion_mode():
    """reset() homes via the NRT 'Home' primitive (which leaves the backend in
    NRT_PRIMITIVE), so it must re-enter Cartesian mode before returning -- else
    the first step()'s Cartesian command would be rejected on hardware."""
    env = FlexivRealEnv(config=RobotConfig(backend="fake"), control_hz=50.0)
    try:
        env.reset()
        # home() alone would leave NRT_PRIMITIVE; the fix re-enters impedance.
        assert env.robot.backend._mode == ControlMode.NRT_CARTESIAN_MOTION_FORCE
        # and a step() right after reset works (mode is correct).
        obs, reward, terminated, truncated, info = env.step([0.0] * 7)
        assert info["execution_success"]
    finally:
        env.close()


def test_fake_home_mirrors_real_primitive_mode():
    """The fake backend's home() now leaves NRT_PRIMITIVE like the real RDK
    backend, so offline tests can catch mode-sequencing bugs."""
    r = Robot(RobotConfig(backend="fake"))
    r.connect()
    r.start_cartesian_impedance()
    assert r.backend._mode == ControlMode.NRT_CARTESIAN_MOTION_FORCE
    r.home()
    assert r.backend._mode == ControlMode.NRT_PRIMITIVE
