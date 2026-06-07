"""An always-on, single-writer reactive control loop (Python tier).

This is the Python embodiment of the 1 kHz C++ daemon: one background thread is
the *only* writer to the backend. High-level code (a SpaceMouse bridge, an MPC
or RL step) just updates the latest setpoint; the loop streams it at a fixed
rate and -- crucially -- **holds position if commands go stale** (watchdog),
instead of replaying an old motion. That hold-on-silence behaviour is exactly
what you want for teleoperation and for a policy that occasionally stalls.

For the lowest latency / jitter, run the C++ daemon (``cpp/``) instead and point
a :class:`~flexiv_control.client.RemoteRobot` at it; the contract is identical.
This Python loop is the no-root, no-build tier that already covers 100-500 Hz
use on Flexiv hardware (the robot runs the hard real-time loop internally).

The loop is the single writer; everyone else mutates a small, lock-guarded
command buffer -- the realtime-buffer pattern used by ros2_control / Polymetis.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..robot import Robot
from ..types import GripperCommand, RobotState, SafetyStatus, StopReason


@dataclass
class LoopStats:
    period_ms: float = 0.0
    jitter_us: float = 0.0
    missed_cycles: int = 0
    command_age_ms: float = 0.0
    last_clipped: bool = False
    last_status: SafetyStatus = SafetyStatus.OK
    last_stop_reason: StopReason = StopReason.NONE


class ReactiveServoLoop:
    def __init__(self, robot: Robot, control_hz: Optional[float] = None):
        self.robot = robot
        self.backend = robot.backend
        self.filter = robot.filter
        self.hz = float(control_hz or robot.control_hz)
        self.dt = 1.0 / self.hz

        self._lock = threading.Lock()
        self._cartesian = True  # else joint
        self._target_pose: Optional[np.ndarray] = None
        self._target_q: Optional[np.ndarray] = None
        self._target_gripper: Optional[GripperCommand] = None
        self._cmd_stamp = 0.0

        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._latest_state: Optional[RobotState] = None
        self._stats = LoopStats()

    # -- target updates (called by teleop / MPC / RL at their own rate) ------
    def set_cartesian_target(
        self, pose: np.ndarray, gripper: Optional[GripperCommand] = None
    ) -> None:
        with self._lock:
            self._cartesian = True
            self._target_pose = np.asarray(pose, float).reshape(7).copy()
            if gripper is not None:
                self._target_gripper = gripper
            self._cmd_stamp = time.time()

    def set_joint_target(self, q: np.ndarray) -> None:
        with self._lock:
            self._cartesian = False
            self._target_q = np.asarray(q, float).copy()
            self._cmd_stamp = time.time()

    # -- introspection -------------------------------------------------------
    def get_state(self) -> RobotState:
        s = self._latest_state or self.backend.read_state()
        st = self._stats
        s.loop_period_ms = st.period_ms
        s.loop_jitter_us = st.jitter_us
        s.missed_cycles = st.missed_cycles
        s.command_latency_ms = st.command_age_ms
        s.safety_status = st.last_status
        s.stop_reason = st.last_stop_reason
        return s

    @property
    def stats(self) -> LoopStats:
        return self._stats

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        if self._running:
            return
        # Seed the target with the current pose so we hold on startup.
        s = self.backend.read_state()
        self.filter.reset(s)
        with self._lock:
            self._target_pose = s.tcp_pose.copy()
            self._target_q = s.q.copy()
            self._cmd_stamp = time.time()
        self._running = True
        self._thread = threading.Thread(target=self._run, name="flexiv-servo-loop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.backend.stop()

    def __enter__(self) -> "ReactiveServoLoop":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    # -- the loop ------------------------------------------------------------
    def _run(self) -> None:
        next_tick = time.perf_counter()
        prev_tick = next_tick
        timeout_s = self.filter.p.command_timeout_ms / 1000.0
        while self._running:
            t0 = time.perf_counter()
            state = self.backend.read_state()
            self._latest_state = state

            with self._lock:
                cartesian = self._cartesian
                target_pose = None if self._target_pose is None else self._target_pose.copy()
                target_q = None if self._target_q is None else self._target_q.copy()
                gripper = self._target_gripper
                age = time.time() - self._cmd_stamp

            self._stats.command_age_ms = age * 1000.0

            stale = age > timeout_s
            if stale and self.filter.p.stop_on_stale_command:
                # Hold position: re-issue the *current measured* pose, don't
                # keep tracking an old setpoint. Re-anchor the filter so the
                # next live command is referenced to where we actually are.
                self._stats.last_status = SafetyStatus.HOLDING
                self.filter.reset(state)
                if cartesian:
                    self.backend.stream_cartesian(state.tcp_pose)
                else:
                    self.backend.stream_joint(state.q)
            else:
                if cartesian and target_pose is not None:
                    sr = self.filter.filter_cartesian(target_pose, state)
                    if sr.ok:
                        self.backend.stream_cartesian(sr.pose)
                        if gripper is not None:
                            self.backend.move_gripper(gripper)
                            with self._lock:
                                self._target_gripper = None
                        self._stats.last_clipped = sr.clipped
                        self._stats.last_status = (
                            SafetyStatus.CLIPPED if sr.clipped else SafetyStatus.OK
                        )
                    else:
                        self.backend.stop()
                        self._stats.last_status = SafetyStatus.STOPPED
                        self._stats.last_stop_reason = sr.reason
                elif not cartesian and target_q is not None:
                    sr = self.filter.filter_joint(target_q, state)
                    if sr.ok:
                        self.backend.stream_joint(sr.q)
                        self._stats.last_clipped = sr.clipped
                        self._stats.last_status = (
                            SafetyStatus.CLIPPED if sr.clipped else SafetyStatus.OK
                        )
                    else:
                        self.backend.stop()
                        self._stats.last_status = SafetyStatus.STOPPED
                        self._stats.last_stop_reason = sr.reason

            # diagnostics
            now = time.perf_counter()
            self._stats.period_ms = (now - prev_tick) * 1000.0
            jitter = abs((now - prev_tick) - self.dt)
            self._stats.jitter_us = jitter * 1e6
            prev_tick = now

            next_tick += self.dt
            sleep = next_tick - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # we overran the budget
                self._stats.missed_cycles += 1
                next_tick = time.perf_counter()
