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

import threading
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


class ChunkStoppedError(RuntimeError):
    """Raised by ``execute_*_chunk(..., raise_on_stop=True)`` when the safety
    filter (or a cancel request) aborted the chunk. Carries the full
    :class:`ExecutionResult` so the caller can inspect what happened."""

    def __init__(self, result: ExecutionResult):
        super().__init__(f"chunk stopped: {result.summary()}")
        self.result = result


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
        # Cooperative cancel: another thread (e.g. the server's stop handler)
        # sets this and the executing chunk loop aborts at its next tick.
        self._cancel = threading.Event()
        # Last state read from the backend -- a cheap, lock-free snapshot the
        # server can serve while a blocking chunk owns the backend.
        self._last_state: Optional[RobotState] = None

    def _backend_kwargs(self) -> dict:
        """Per-backend construction kwargs drawn from the config."""
        b = self.cfg.backend.lower()
        if b in ("flexiv_rdk", "rdk", "flexiv"):
            return dict(robot_sn=self.cfg.robot_sn, gripper_name=self.cfg.gripper_name)
        if b in ("mujoco", "mjx"):
            return dict(
                model_path=self.cfg.model_path,
                n_joints=self.cfg.n_joints,
                # Default the sim substep to the control period so a chunk plays
                # back at the same speed it is streamed; honour an explicit override.
                control_dt=self.cfg.control_dt if self.cfg.control_dt is not None else self.dt,
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
        s = self.backend.read_state()
        self._last_state = s
        return s

    def peek_state(self) -> Optional[RobotState]:
        """Latest state already read from the backend, without touching it.

        During a blocking chunk the execute loop refreshes this every tick, so
        the server can answer ``get_state`` mid-chunk (at most one tick stale)
        instead of blocking on the backend lock for the chunk's whole duration.
        """
        return self._last_state

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
    def _verify_chunk_profile(self, requested: str, result: ExecutionResult) -> None:
        """Enforce the reproducibility contract on ``chunk.safety_profile``.

        Empty = "run under whatever is active". A non-empty name must match the
        active profile, else we raise: silently executing under a different
        envelope than the one the chunk was planned for is exactly the failure
        the field exists to prevent. Requested/active are always logged.
        """
        result.log["requested_profile"] = requested
        result.log["active_profile"] = self.profile.name
        if requested and requested != self.profile.name:
            raise ValueError(
                f"chunk requests safety profile {requested!r} but the active "
                f"profile is {self.profile.name!r}; call set_safety_profile"
                f"({requested!r}) first or fix the chunk"
            )

    def execute_cartesian_chunk(
        self,
        chunk: CartesianChunk,
        *,
        blocking: bool = True,
        raise_on_stop: bool = False,
        record: bool = False,
    ) -> ExecutionResult:
        """Execute a Cartesian chunk at the control rate with per-tick safety.

        Returns an :class:`ExecutionResult` with tracking error, clipping, stop
        reason, and peak quantities -- the observable signal a planner can log
        under its "execution" failure category.

        * If the backend is not already in a Cartesian motion mode, the NRT
          Cartesian impedance mode is started automatically with the chunk's
          ``impedance`` (the documented examples call
          ``start_cartesian_impedance()`` explicitly; forgetting it must not be
          a hardware-only failure).
        * The chunk's kinematic/contact envelope tightens the active profile:
          the interpolator runs at ``min(chunk cap, profile cap)`` and the
          contact check at the elementwise minimum wrench.
        * A non-empty ``chunk.safety_profile`` must match the active profile.
        * ``blocking`` is currently always True (kept for future async parity);
          a concurrent ``stop()``/``request_stop()`` cancels mid-chunk. A cancel
          that is already pending at entry aborts THIS chunk immediately (a stop
          issued between chunks must not be silently erased by the next one).
        * ``raise_on_stop=True`` raises :class:`ChunkStoppedError` instead of
          returning a failed result, so a protective stop cannot be ignored.
        * ``record=True`` fills ``result.log["trajectory"]`` with per-tick rows
          ``[t, *pose_cmd, *pose_meas, *wrench]`` and sets
          ``result.log["stopped_at_waypoint"]`` when the run aborts -- the
          measured-vs-commanded series for sim-vs-real attribution that the
          loop otherwise measures every tick and throws away.
        """
        self._check_lease()
        result = ExecutionResult(success=True, stop_reason=StopReason.NONE.value)
        if self._cancel.is_set():
            self._cancel.clear()
            result.success = False
            result.stop_reason = StopReason.USER.value
            result.log["aborted_at_entry"] = True
            result.final_state = self.get_state()
            if raise_on_stop:
                raise ChunkStoppedError(result)
            return result
        self._verify_chunk_profile(chunk.safety_profile, result)

        start = self.get_state()
        if not start.control_mode.is_cartesian:
            self.start_cartesian_impedance(impedance=chunk.impedance)
            result.log["mode_autostarted"] = True
            start = self.get_state()
        self.filter.reset(start)
        # Resolve relative-to-start poses against the live start pose and slice to
        # the execution horizon (receding horizon): only the first H_exec waypoints
        # run here; the rest are re-predicted by the planner next cycle.
        chunk = chunk.for_execution(start.tcp_pose)
        # Tightening-only envelope: a chunk may slow itself below the profile's
        # caps but can never relax them.
        lin_cap = self.profile.max_linear_speed
        if chunk.max_tcp_linear_speed and chunk.max_tcp_linear_speed > 0:
            lin_cap = min(lin_cap, float(chunk.max_tcp_linear_speed))
        ang_cap = self.profile.max_angular_speed
        if chunk.max_tcp_angular_speed and chunk.max_tcp_angular_speed > 0:
            ang_cap = min(ang_cap, float(chunk.max_tcp_angular_speed))
        result.log["linear_speed_cap"] = lin_cap
        result.log["angular_speed_cap"] = ang_cap
        wrench_cap = self.profile.max_contact_wrench
        if chunk.max_contact_wrench is not None:
            wrench_cap = np.minimum(wrench_cap, chunk.max_contact_wrench)
        interp = CartesianChunkInterpolator(
            chunk,
            start.tcp_pose,
            self.control_hz,
            max_linear_speed=lin_cap,
            max_angular_speed=ang_cap,
        )

        max_err = 0.0
        max_speed = 0.0
        max_wrench = 0.0
        prev_pos = start.tcp_position.copy()
        prev_cmd_pos = None  # commanded TCP position from the previous tick
        trajectory: list = [] if record else None
        t0 = time.perf_counter()
        t_loop = time.perf_counter()

        for pose, grip in interp:
            if self._cancel.is_set():
                self._cancel.clear()
                self.backend.stop()
                result.success = False
                result.stop_reason = StopReason.USER.value
                break
            state = self.get_state()
            # Robot-side fault gate: a collision reflex / protective stop /
            # E-stop on the robot is invisible to the host-side geometry filter.
            if self.backend.in_fault():
                self.backend.stop()
                result.success = False
                result.stop_reason = StopReason.BACKEND_FAULT.value
                break
            # Per-chunk contact tightening (the filter enforces the profile cap;
            # this catches a chunk-requested lower cap).
            if np.any(np.abs(state.wrench) > wrench_cap):
                self.backend.stop()
                result.success = False
                result.stop_reason = StopReason.CONTACT_WRENCH.value
                break
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
            if trajectory is not None:
                trajectory.append(
                    [time.perf_counter() - t0, *sr.pose.tolist(),
                     *state.tcp_pose.tolist(), *state.wrench.tolist()]
                )

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
        if trajectory is not None:
            result.log["trajectory"] = trajectory
        if not result.success:
            result.log["stopped_at_waypoint"] = interp.current_segment
        if raise_on_stop and not result.success:
            raise ChunkStoppedError(result)
        return result

    # -- joint space (reset / home / MoveIt-plan execution) ----------------
    @staticmethod
    def _joint_move_duration(
        q_now: np.ndarray,
        q_target: np.ndarray,
        *,
        duration: Optional[float],
        max_joint_speed: Optional[float],
        default: float = 3.0,
        floor: float = 1.0,
    ) -> float:
        """Resolve a joint move's duration from either a fixed time or a speed cap.

        A recovery move is naturally specified as "go there slowly" (a rad/s
        cap on the largest joint excursion); a fixed duration is dangerously
        fast for a large displacement and pointlessly slow for a small one.
        """
        if duration is not None:
            return float(duration)
        if max_joint_speed is not None and max_joint_speed > 0:
            dq = float(np.max(np.abs(np.asarray(q_target, float) - np.asarray(q_now, float))))
            return max(floor, dq / float(max_joint_speed))
        return default

    def move_joint(
        self,
        q_target: np.ndarray,
        *,
        duration: Optional[float] = None,
        max_joint_speed: Optional[float] = None,
        realtime: bool = False,
    ) -> ExecutionResult:
        """Interpolated joint move. Give either a fixed ``duration`` (seconds)
        or a ``max_joint_speed`` (rad/s) cap on the largest joint excursion;
        with neither, a 3 s default applies."""
        self._check_lease()
        dur = self._joint_move_duration(
            self.get_state().q, q_target, duration=duration, max_joint_speed=max_joint_speed
        )
        self.start_joint_impedance(realtime=realtime)
        chunk = JointChunk(
            waypoints=[JointWaypoint(positions=np.asarray(q_target, float), duration=dur)],
            safety_profile=self.profile.name,
        )
        return self.execute_joint_chunk(chunk)

    def execute_joint_chunk(
        self, chunk: JointChunk, *, raise_on_stop: bool = False
    ) -> ExecutionResult:
        self._check_lease()
        result = ExecutionResult(success=True)
        if self._cancel.is_set():
            # A pending stop aborts THIS chunk rather than being silently
            # erased (same consume-on-abort semantics as the Cartesian path).
            self._cancel.clear()
            result.success = False
            result.stop_reason = StopReason.USER.value
            result.log["aborted_at_entry"] = True
            result.final_state = self.get_state()
            if raise_on_stop:
                raise ChunkStoppedError(result)
            return result
        self._verify_chunk_profile(chunk.safety_profile, result)
        start = self.get_state()
        if start.control_mode.is_cartesian or start.control_mode == ControlMode.IDLE:
            self.start_joint_impedance()
            result.log["mode_autostarted"] = True
            start = self.get_state()
        self.filter.reset(start)
        interp = JointChunkInterpolator(
            chunk,
            start.q,
            self.control_hz,
            max_joint_speed=2.0 * self.profile.max_joint_speed_scale,
        )
        t_loop = time.perf_counter()
        for q in interp:
            if self._cancel.is_set():
                self._cancel.clear()
                self.backend.stop()
                result.success = False
                result.stop_reason = StopReason.USER.value
                break
            state = self.get_state()
            # The recovery/home path is the one most likely to run from an
            # abnormal pose: give joint moves the same robot-fault and
            # contact-wrench gates the Cartesian path has.
            if self.backend.in_fault():
                self.backend.stop()
                result.success = False
                result.stop_reason = StopReason.BACKEND_FAULT.value
                break
            if np.any(np.abs(state.wrench) > self.profile.max_contact_wrench):
                self.backend.stop()
                result.success = False
                result.stop_reason = StopReason.CONTACT_WRENCH.value
                break
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
        if raise_on_stop and not result.success:
            raise ChunkStoppedError(result)
        return result

    # -- gripper / home / stop ----------------------------------------------
    def command_gripper(
        self, cmd: GripperCommand, *, wait: bool = False, timeout: float = 5.0
    ) -> Optional[float]:
        """Send a gripper command; with ``wait=True``, block until the fingers
        settle and return the final width in metres (``None`` without ``wait``,
        matching ``RemoteRobot.command_gripper``). Without ``wait`` this is
        fire-and-forget (RDK ``Gripper.Move``/``Grasp`` return immediately),
        which is why every consumer that needs "open, then proceed" used to
        fabricate a do-nothing motion chunk just to ride its blocking executor.

        Settle detection: reaching the commanded width (non-grasp), or width
        unchanged while not moving -- the latter only counts after motion has
        been OBSERVED or a 0.5 s dwell has passed, because real hardware has an
        actuation-latency window after the command in which the unchanged OLD
        width would otherwise read as "settled"."""
        self._check_lease()
        initial = self.get_state().gripper_width
        self.backend.move_gripper(cmd)
        if not wait:
            return None
        t0 = time.time()
        prev_width = None
        moved = False
        state = self.get_state()
        while time.time() - t0 < timeout:
            if self._cancel.is_set():
                break  # a stop request also ends a gripper wait
            state = self.get_state()
            if state.gripper_is_moving or abs(state.gripper_width - initial) > 1e-3:
                moved = True
            settled_target = (not cmd.grasp) and abs(state.gripper_width - cmd.width) < 2e-3
            settled_still = (
                prev_width is not None
                and abs(state.gripper_width - prev_width) < 5e-4
                and not state.gripper_is_moving
                and (moved or time.time() - t0 >= 0.5)
            )
            if settled_target or settled_still:
                break
            prev_width = state.gripper_width
            time.sleep(0.05)
        return state.gripper_width

    def home(
        self,
        q: Optional[np.ndarray] = None,
        *,
        max_joint_speed: Optional[float] = None,
        duration: Optional[float] = None,
    ) -> None:
        """Move to the canonical posture: ``q`` if given, else the config's
        ``q_home``; then command ``cfg.gripper_home_width`` if configured, so
        "home" means the full recorded posture (joints AND gripper), not just
        joints."""
        self._check_lease()
        q_home = np.asarray(q if q is not None else self.cfg.q_home, float)
        try:
            if q is not None or duration is not None or max_joint_speed is not None:
                # An explicit target/timing always uses the interpolated move so
                # the caller's posture (not the vendor primitive's factory home)
                # is what the arm reaches.
                raise NotImplementedError
            self.backend.home(q_home)
        except NotImplementedError:
            self.move_joint(
                q_home,
                duration=duration,
                max_joint_speed=max_joint_speed if max_joint_speed is not None else 0.3,
            )
        if self.cfg.gripper_home_width is not None:
            self.command_gripper(
                GripperCommand(width=float(self.cfg.gripper_home_width)), wait=True
            )

    def go_home_safe(
        self,
        *,
        q_home: Optional[np.ndarray] = None,
        lift_m: float = 0.10,
        open_gripper_width: Optional[float] = None,
        max_tcp_speed: float = 0.10,
        max_joint_speed: float = 0.3,
    ) -> ExecutionResult:
        """The standard end-of-session exit ritual as one resilient call:
        lift straight up -> open the gripper (and wait) -> joint-move home.

        Every robot-facing app needs exactly this block in a ``finally`` clause;
        hand-rolling it is where restore bugs live. Each stage tolerates the
        previous one failing (a recovery path must not give up halfway), and the
        returned result is the home move's, with the lift/gripper outcomes in
        ``result.log``."""
        self._check_lease()
        log: dict = {}
        contact_abort = False
        # 1) lift straight up, orientation held, capped slow.
        try:
            s = self.get_state()
            target = s.tcp_position.copy()
            target[2] = min(target[2] + float(lift_m), self.profile.ws_z[1])
            lift_chunk = CartesianChunk(
                waypoints=[
                    CartesianWaypoint(
                        position=target,
                        quaternion=None,
                        duration=max(1.5, float(lift_m) / max(max_tcp_speed, 1e-3)),
                    )
                ],
                max_tcp_linear_speed=max_tcp_speed,
            )
            lift_result = self.execute_cartesian_chunk(lift_chunk)
            log["lift"] = lift_result.summary()
            contact_abort = lift_result.stop_reason in (
                StopReason.CONTACT_WRENCH.value,
                StopReason.BACKEND_FAULT.value,
            )
        except Exception as e:  # recovery continues even if the lift fails
            log["lift"] = f"FAILED: {type(e).__name__}: {e}"
        # 2) open the gripper and wait for it.
        width = open_gripper_width
        if width is None:
            width = self.cfg.gripper_home_width
        if width is not None:
            try:
                self.command_gripper(GripperCommand(width=float(width)), wait=True)
                log["gripper"] = f"opened to {float(width):.4f} m"
            except Exception as e:
                log["gripper"] = f"FAILED: {type(e).__name__}: {e}"
        # A stop request or a contact/fault during the lift makes the blind
        # joint-space sweep toward home the WRONG move: the arm is plausibly
        # snagged on the scene, and a joint home from there can drag the TCP
        # through the table. Hand control back to the operator instead.
        if contact_abort or self._cancel.is_set():
            result = ExecutionResult(
                success=False,
                stop_reason=(
                    StopReason.USER.value if self._cancel.is_set()
                    else StopReason.CONTACT_WRENCH.value
                ),
            )
            log["home"] = "skipped: lift ended in contact/fault or a stop was requested"
            result.final_state = self.get_state()
            result.log.update(log)
            return result
        # 3) joint-move to the canonical posture, speed-capped.
        q_target = np.asarray(q_home if q_home is not None else self.cfg.q_home, float)
        result = self.move_joint(q_target, max_joint_speed=max_joint_speed)
        result.log.update(log)
        return result

    def request_stop(self) -> None:
        """Cooperative cancel: the executing chunk loop aborts at its next tick
        (StopReason ``user``). Safe to call from another thread; does not touch
        the backend itself, so it cannot race the executing thread's stream."""
        self._cancel.set()

    def stop(self) -> None:
        self._cancel.set()
        self.backend.stop()


def _chunk_wrench(chunk: CartesianChunk):
    if chunk.force_control is not None and np.any(chunk.force_control.enabled_axes):
        return chunk.force_control.target_wrench
    return None
