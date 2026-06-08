"""The :class:`Robot` facade -- the one API everything uses.

A receding-horizon planner, an MPC loop, an RL policy, and a SpaceMouse bridge all talk to this
same object. They never touch a backend, ROS topic, or RDK struct directly.

Design notes
------------
* ``execute_cartesian_chunk`` expands the chunk to setpoints, runs the fixed-rate
  loop in Python (the "NRT / modest-rate" tier), applies the safety filter every
  tick, and returns an :class:`ExecutionResult` -- which is what turns a planner's
  "execution" failure category into real numbers.
* For the lowest-latency path, point this at the C++ RT daemon via the network
  client (``flexiv_control.client.RemoteRobot``) instead; the API is identical.
* Lease + stop are here so a single process is well-behaved; the *server*
  enforces the lease across multiple processes (RL + MPC + teleop can't fight
  over the arm).
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from .action_chunk import (
    CartesianChunk,
    CartesianDelta,
    CartesianWaypoint,
    ExecutionResult,
    JointChunk,
    JointWaypoint,
)
from .backends import RobotBackend, get_backend
from .config import RobotConfig, load_safety_profile
from .interpolation import (
    CartesianChunkInterpolator,
    JointChunkInterpolator,
    delta_to_target_pose,
)
from .safety import SafetyFilter, SafetyProfile
from .types import (
    ControlMode,
    ForceControlParams,
    GripperCommand,
    ImpedanceParams,
    JointImpedanceParams,
    RobotState,
    StopReason,
)


class LeaseError(RuntimeError):
    pass


class Robot:
    def __init__(
        self,
        config: Optional[RobotConfig] = None,
        backend: Optional[RobotBackend] = None,
        control_hz: Optional[float] = None,
        safety_profile: Optional[str] = None,
    ):
        self.cfg = config or RobotConfig()
        self.control_hz = float(control_hz or self.cfg.control_hz)
        self.dt = 1.0 / self.control_hz
        self.backend = backend or get_backend(self.cfg.backend, **self._backend_kwargs())
        self._owner: Optional[str] = None
        self.profile: SafetyProfile = load_safety_profile(
            safety_profile or self.cfg.default_safety_profile
        )
        self.filter = SafetyFilter(self.profile, self.dt)

    def _backend_kwargs(self) -> dict:
        """Per-backend construction kwargs drawn from the config."""
        b = self.cfg.backend.lower()
        if b in ("flexiv_rdk", "rdk", "flexiv"):
            return dict(robot_sn=self.cfg.robot_sn, gripper_name=self.cfg.gripper_name)
        if b in ("mujoco", "mjx"):
            return dict(
                model_path=self.cfg.model_path,
                n_joints=self.cfg.n_joints,
                control_dt=self.cfg.control_dt,
                tcp_site=self.cfg.mujoco_tcp_site,
                gripper_actuator=self.cfg.mujoco_gripper_actuator,
                gripper_width_scale=self.cfg.mujoco_gripper_width_scale,
                gripper_width_offset=self.cfg.mujoco_gripper_width_offset,
            )
        return {}

    # -- construction helpers ------------------------------------------------
    @classmethod
    def from_config(cls, path_or_name: str, **overrides) -> "Robot":
        cfg = RobotConfig.load(path_or_name)
        return cls(config=cfg, **overrides)

    # -- lifecycle -----------------------------------------------------------
    def connect(self) -> None:
        self.backend.connect()

    def disconnect(self) -> None:
        self.backend.disconnect()

    def __enter__(self) -> "Robot":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        try:
            self.stop()
        finally:
            self.disconnect()

    # -- lease (single-process; the server enforces cross-process) ----------
    def acquire_lease(self, owner: str) -> None:
        if self._owner is not None and self._owner != owner:
            raise LeaseError(f"robot already leased by {self._owner!r}")
        self._owner = owner

    def release_lease(self) -> None:
        self._owner = None

    def _check_lease(self) -> None:
        if self._owner is None:
            # Single-process convenience: auto-lease to "default".
            self._owner = "default"

    # -- safety profile ------------------------------------------------------
    def set_safety_profile(self, name_or_path: str) -> None:
        self.profile = load_safety_profile(name_or_path)
        # Update in place so a running control loop / server keeps its reference.
        self.filter.set_profile(self.profile)

    # -- state ---------------------------------------------------------------
    def get_state(self) -> RobotState:
        return self.backend.read_state()

    # -- mode start helpers --------------------------------------------------
    def start_cartesian_impedance(
        self,
        impedance: Optional[ImpedanceParams] = None,
        *,
        realtime: bool = False,
        force_control: Optional[ForceControlParams] = None,
        nullspace_q: Optional[np.ndarray] = None,
    ) -> None:
        mode = (
            ControlMode.RT_CARTESIAN_MOTION_FORCE
            if realtime
            else ControlMode.NRT_CARTESIAN_MOTION_FORCE
        )
        self.backend.set_mode(
            mode,
            impedance=impedance or ImpedanceParams(),
            force_control=force_control,
            nullspace_q=nullspace_q if nullspace_q is not None else self.cfg.q_home,
            max_contact_wrench=self.profile.max_contact_wrench,
        )

    def start_joint_impedance(
        self,
        joint_impedance: Optional[JointImpedanceParams] = None,
        *,
        realtime: bool = False,
    ) -> None:
        mode = ControlMode.RT_JOINT_IMPEDANCE if realtime else ControlMode.NRT_JOINT_IMPEDANCE
        self.backend.set_mode(mode, joint_impedance=joint_impedance or JointImpedanceParams())

    # -- the RL / MPC / teleop workhorse ------------------------------------
    def servo_cartesian_delta(
        self,
        delta,
        *,
        duration: Optional[float] = None,
        frame: str = "base",
        gripper: Optional[GripperCommand] = None,
    ) -> ExecutionResult:
        """Apply a relative ``[dx,dy,dz,drx,dry,drz]`` move over ``duration``."""
        self._check_lease()
        if not isinstance(delta, CartesianDelta):
            delta = CartesianDelta(
                delta=delta, duration=duration or self.dt, frame=frame, gripper=gripper
            )
        state = self.get_state()
        target = delta_to_target_pose(delta, state.tcp_pose)
        wp = CartesianWaypoint(
            position=target[:3], quaternion=target[3:7],
            gripper=delta.gripper, duration=delta.duration, frame=delta.frame,
        )
        chunk = CartesianChunk(waypoints=[wp], frame=delta.frame,
                               safety_profile=self.profile.name)
        return self.execute_cartesian_chunk(chunk, blocking=True)

    def servo_cartesian_pose(
        self, pose: np.ndarray, *, duration: float = 0.2,
        gripper: Optional[GripperCommand] = None,
    ) -> ExecutionResult:
        pose = np.asarray(pose, float).reshape(7)
        wp = CartesianWaypoint(position=pose[:3], quaternion=pose[3:7],
                               gripper=gripper, duration=duration)
        return self.execute_cartesian_chunk(
            CartesianChunk(waypoints=[wp], safety_profile=self.profile.name), blocking=True
        )

    # -- planner chunk / MPC-horizon / scripted manipulation ---------------
    def execute_cartesian_chunk(
        self, chunk: CartesianChunk, *, blocking: bool = True
    ) -> ExecutionResult:
        """Execute a Cartesian chunk at the control rate with per-tick safety.

        Returns an :class:`ExecutionResult` with tracking error, clipping, stop
        reason, and peak quantities -- the observable signal a planner can log
        under its "execution" failure category.
        """
        self._check_lease()
        start = self.get_state()
        self.filter.reset(start)
        # Resolve relative-to-start poses against the live start pose and slice to
        # the execution horizon (receding horizon): only the first H_exec waypoints
        # run here; the rest are re-predicted by the planner next cycle.
        chunk = chunk.for_execution(start.tcp_pose)
        interp = CartesianChunkInterpolator(
            chunk,
            start.tcp_pose,
            self.control_hz,
            max_linear_speed=self.profile.max_linear_speed,
            max_angular_speed=self.profile.max_angular_speed,
        )

        result = ExecutionResult(success=True, stop_reason=StopReason.NONE.value)
        max_err = 0.0
        max_speed = 0.0
        max_wrench = 0.0
        prev_pos = start.tcp_position.copy()
        prev_cmd_pos = None  # commanded TCP position from the previous tick
        t_loop = time.perf_counter()

        for pose, grip in interp:
            state = self.get_state()
            sr = self.filter.filter_cartesian(pose, state)
            if not sr.ok:
                self.backend.stop()
                result.success = False
                result.stop_reason = sr.reason.value
                break
            if sr.clipped:
                result.clipped = True
            self.backend.stream_cartesian(sr.pose, wrench=_chunk_wrench(chunk))
            if grip is not None:
                self.backend.move_gripper(grip)

            # bookkeeping. path_tracking_error is the lag between the PREVIOUS
            # tick's command and the CURRENT measurement (a true residual), not
            # the size of this tick's commanded step (skipped on the first tick).
            if prev_cmd_pos is not None:
                err = float(np.linalg.norm(prev_cmd_pos - state.tcp_position))
                max_err = max(max_err, err)
            spd = float(np.linalg.norm(state.tcp_position - prev_pos)) / self.dt
            max_speed = max(max_speed, spd)
            max_wrench = max(max_wrench, float(np.max(np.abs(state.wrench))))
            prev_pos = state.tcp_position.copy()
            prev_cmd_pos = sr.pose[:3].copy()

            # maintain control rate
            t_loop += self.dt
            sleep = t_loop - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)

        end = self.get_state()
        result.executed_duration = end.stamp - start.stamp
        result.path_tracking_error = max_err
        result.max_tcp_speed = max_speed
        result.max_wrench = max_wrench
        result.gripper_width_final = end.gripper_width
        result.final_state = end
        return result

    # -- joint space (reset / home / MoveIt-plan execution) ----------------
    def move_joint(
        self, q_target: np.ndarray, *, duration: float = 3.0, realtime: bool = False
    ) -> ExecutionResult:
        self._check_lease()
        self.start_joint_impedance(realtime=realtime)
        chunk = JointChunk(
            waypoints=[
                JointWaypoint(positions=np.asarray(q_target, float), duration=duration)
            ],
            safety_profile=self.profile.name,
        )
        return self.execute_joint_chunk(chunk)

    def execute_joint_chunk(self, chunk: JointChunk) -> ExecutionResult:
        self._check_lease()
        start = self.get_state()
        self.filter.reset(start)
        interp = JointChunkInterpolator(
            chunk,
            start.q,
            self.control_hz,
            max_joint_speed=2.0 * self.profile.max_joint_speed_scale,
        )
        result = ExecutionResult(success=True)
        t_loop = time.perf_counter()
        for q in interp:
            state = self.get_state()
            sr = self.filter.filter_joint(q, state)
            if not sr.ok:
                self.backend.stop()
                result.success = False
                result.stop_reason = sr.reason.value
                break
            if sr.clipped:
                result.clipped = True
            self.backend.stream_joint(sr.q)
            t_loop += self.dt
            sleep = t_loop - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
        result.final_state = self.get_state()
        return result

    # -- gripper / home / stop ----------------------------------------------
    def command_gripper(self, cmd: GripperCommand) -> None:
        self._check_lease()
        self.backend.move_gripper(cmd)

    def home(self) -> None:
        self._check_lease()
        try:
            self.backend.home(self.cfg.q_home)
        except NotImplementedError:
            self.move_joint(self.cfg.q_home, duration=4.0)

    def stop(self) -> None:
        self.backend.stop()


def _chunk_wrench(chunk: CartesianChunk):
    if chunk.force_control is not None and np.any(chunk.force_control.enabled_axes):
        return chunk.force_control.target_wrench
    return None
